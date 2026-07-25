// Copyright 2026 xingchen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cmath>

#include "gtest/gtest.h"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_route_polyline_planner/route_clearance_planner_core.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace
{

geometry_msgs::msg::PoseStamped makePose(
  double x,
  double y,
  double yaw = 0.0,
  const std::string & frame_id = "map")
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
  return pose;
}

nav2_costmap_2d::Costmap2D makeCostmap(
  unsigned int size_x = 30,
  unsigned int size_y = 20,
  double resolution = 0.1)
{
  return nav2_costmap_2d::Costmap2D(
    size_x, size_y, resolution, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & orientation)
{
  const double siny_cosp =
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cosy_cosp =
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double meanInteriorY(const nav_msgs::msg::Path & path)
{
  if (path.poses.size() <= 2U) {
    return path.poses.empty() ? 0.0 : path.poses.front().pose.position.y;
  }

  double sum = 0.0;
  size_t count = 0;
  for (size_t index = 1; index + 1 < path.poses.size(); ++index) {
    sum += path.poses[index].pose.position.y;
    ++count;
  }
  return sum / static_cast<double>(count);
}

double meanInteriorYInXRange(
  const nav_msgs::msg::Path & path,
  double min_x,
  double max_x)
{
  double sum = 0.0;
  size_t count = 0;
  for (size_t index = 1; index + 1 < path.poses.size(); ++index) {
    const auto & position = path.poses[index].pose.position;
    if (position.x < min_x || position.x > max_x) {
      continue;
    }
    sum += position.y;
    ++count;
  }
  return count == 0U ? meanInteriorY(path) : sum / static_cast<double>(count);
}

double maxTurnDegreesInTail(const nav_msgs::msg::Path & path, size_t tail_count)
{
  if (path.poses.size() < 3U) {
    return 0.0;
  }

  const size_t begin = path.poses.size() > tail_count ? path.poses.size() - tail_count : 1U;
  double max_turn = 0.0;
  for (size_t index = std::max<size_t>(1U, begin); index + 1 < path.poses.size(); ++index) {
    const auto & previous = path.poses[index - 1].pose.position;
    const auto & current = path.poses[index].pose.position;
    const auto & next = path.poses[index + 1].pose.position;
    const double ax = current.x - previous.x;
    const double ay = current.y - previous.y;
    const double bx = next.x - current.x;
    const double by = next.y - current.y;
    const double a_length = std::hypot(ax, ay);
    const double b_length = std::hypot(bx, by);
    if (a_length <= 1e-9 || b_length <= 1e-9) {
      continue;
    }
    const double dot = std::max(
      -1.0,
      std::min(1.0, (ax * bx + ay * by) / (a_length * b_length)));
    max_turn = std::max(max_turn, std::acos(dot) * 180.0 / M_PI);
  }
  return max_turn;
}

double maxTurnDegrees(const nav_msgs::msg::Path & path)
{
  return maxTurnDegreesInTail(path, path.poses.size());
}

double maxTurnDegreesInHead(const nav_msgs::msg::Path & path, size_t head_count)
{
  if (path.poses.size() < 3U) {
    return 0.0;
  }

  const size_t end = std::min(path.poses.size() - 1U, head_count);
  double max_turn = 0.0;
  for (size_t index = 1; index < end; ++index) {
    const auto & previous = path.poses[index - 1].pose.position;
    const auto & current = path.poses[index].pose.position;
    const auto & next = path.poses[index + 1].pose.position;
    const double ax = current.x - previous.x;
    const double ay = current.y - previous.y;
    const double bx = next.x - current.x;
    const double by = next.y - current.y;
    const double a_length = std::hypot(ax, ay);
    const double b_length = std::hypot(bx, by);
    if (a_length <= 1e-9 || b_length <= 1e-9) {
      continue;
    }
    const double dot = std::max(
      -1.0,
      std::min(1.0, (ax * bx + ay * by) / (a_length * b_length)));
    max_turn = std::max(max_turn, std::acos(dot) * 180.0 / M_PI);
  }
  return max_turn;
}

void addVerticalWall(
  nav2_costmap_2d::Costmap2D & costmap,
  unsigned int mx,
  unsigned int gap_min_y,
  unsigned int gap_max_y)
{
  for (unsigned int my = 0; my < costmap.getSizeInCellsY(); ++my) {
    if (my < gap_min_y || my > gap_max_y) {
      costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

void addHorizontalWall(
  nav2_costmap_2d::Costmap2D & costmap,
  unsigned int my)
{
  for (unsigned int mx = 0; mx < costmap.getSizeInCellsX(); ++mx) {
    costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
}

}  // namespace

TEST(RouteClearancePlannerCoreTest, FreeSpaceReturnsNearlyStraightPath)
{
  auto costmap = makeCostmap();
  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.path_interpolation_resolution = 0.2;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto result = planner.createPlan(
    costmap, makePose(0.25, 0.25), makePose(2.45, 0.25, 1.57), "map");

  EXPECT_FALSE(result.adjusted_goal);
  ASSERT_GT(result.path.poses.size(), 2U);
  EXPECT_NEAR(result.path.poses.front().pose.position.x, 0.25, 0.11);
  EXPECT_NEAR(result.path.poses.front().pose.position.y, 0.25, 0.11);
  EXPECT_NEAR(result.path.poses.back().pose.position.x, 2.45, 0.11);
  EXPECT_NEAR(result.path.poses.back().pose.position.y, 0.25, 0.11);
  EXPECT_NEAR(
    result.path.poses.back().pose.orientation.z, makePose(
      0.0, 0.0,
      1.57).pose.orientation.z, 1e-6);
  for (const auto & pose : result.path.poses) {
    EXPECT_NEAR(pose.pose.position.y, 0.25, 0.16);
  }
}

TEST(RouteClearancePlannerCoreTest, ObstacleForcesDetourRespectingHardClearance)
{
  auto costmap = makeCostmap(40, 30, 0.1);
  addVerticalWall(costmap, 20, 22, 26);

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.20;
  config.soft_target_clearance = 0.40;
  config.path_interpolation_resolution = 0.1;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto result = planner.createPlan(
    costmap, makePose(0.35, 1.45), makePose(3.55, 1.45), "map");

  ASSERT_FALSE(result.path.poses.empty());
  EXPECT_GT(result.path.poses.back().pose.position.x, 3.4);

  bool found_detour = false;
  for (const auto & pose : result.path.poses) {
    if (pose.pose.position.y > 2.0) {
      found_detour = true;
    }
    EXPECT_GE(
      planner.clearanceAt(costmap, pose.pose.position.x, pose.pose.position.y),
      0.20 - 1e-6);
  }
  EXPECT_TRUE(found_detour);
}

TEST(RouteClearancePlannerCoreTest, OpenSpacePrefersHigherClearanceLane)
{
  auto costmap = makeCostmap(60, 30, 0.1);

  for (unsigned int mx = 0; mx < costmap.getSizeInCellsX(); ++mx) {
    costmap.setCost(mx, 3, nav2_costmap_2d::LETHAL_OBSTACLE);
    costmap.setCost(mx, 26, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  for (unsigned int my = 4; my <= 24; ++my) {
    costmap.setCost(18, my, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  }

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.20;
  config.soft_target_clearance = 0.50;
  config.path_interpolation_resolution = 0.1;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto result = planner.createPlan(
    costmap, makePose(0.45, 1.55), makePose(5.55, 1.55), "map");

  ASSERT_FALSE(result.path.poses.empty());
  double min_y = std::numeric_limits<double>::infinity();
  for (const auto & pose : result.path.poses) {
    min_y = std::min(min_y, pose.pose.position.y);
  }
  EXPECT_GT(min_y, 1.0);
}

TEST(RouteClearancePlannerCoreTest, PositiveCenterlineWeightStaysNearShortestPath)
{
  auto costmap = makeCostmap(60, 30, 0.1);

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.20;
  config.soft_target_clearance = 0.50;
  config.centerline_weight = 2.0;
  config.path_interpolation_resolution = 0.1;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto start = makePose(0.35, 1.45);
  const auto goal = makePose(5.55, 1.45);
  const auto result = planner.createPlan(costmap, start, goal, "map");

  ASSERT_GT(result.path.poses.size(), 2U);
  double length = 0.0;
  for (size_t index = 1; index < result.path.poses.size(); ++index) {
    const auto & previous = result.path.poses[index - 1].pose.position;
    const auto & current = result.path.poses[index].pose.position;
    length += std::hypot(current.x - previous.x, current.y - previous.y);
  }

  const double straight_distance = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y);
  EXPECT_LT(length, straight_distance * 1.2);
}

TEST(RouteClearancePlannerCoreTest, LongShortcutPathIsSmoothedAfterShortcutting)
{
  auto costmap = makeCostmap(520, 160, 0.1);
  for (unsigned int my = 0; my < costmap.getSizeInCellsY(); ++my) {
    if (my >= 110 && my <= 125) {
      continue;
    }
    costmap.setCost(250, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.20;
  config.soft_target_clearance = 0.50;
  config.centerline_weight = 2.0;
  config.reference_corridor_half_width = 8.0;
  config.path_interpolation_resolution = 0.1;
  config.right_side_bias = true;
  config.right_side_weight = 0.0;
  config.right_side_max_offset = 0.0;
  config.lateral_smoothing_passes = 6;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto result = planner.createPlan(
    costmap, makePose(0.55, 4.05), makePose(50.55, 4.05), "map");

  ASSERT_GT(result.path.poses.size(), 20U);
  EXPECT_LT(maxTurnDegrees(result.path), 25.0);
  for (const auto & pose : result.path.poses) {
    EXPECT_GE(
      planner.clearanceAt(costmap, pose.pose.position.x, pose.pose.position.y),
      config.hard_min_clearance - 1e-6);
  }
}

TEST(RouteClearancePlannerCoreTest, RightSideBiasSeparatesForwardAndReturnRoutes)
{
  auto costmap = makeCostmap(70, 30, 0.1);
  addHorizontalWall(costmap, 3);
  addHorizontalWall(costmap, 26);

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.20;
  config.soft_target_clearance = 0.40;
  config.reference_corridor_half_width = 2.0;
  config.path_interpolation_resolution = 0.1;
  config.right_side_bias = true;
  config.right_side_weight = 8.0;
  config.right_side_target_clearance = 0.55;
  config.right_side_probe_distance = 1.5;
  config.right_side_max_offset = 0.7;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto forward = planner.createPlan(
    costmap, makePose(0.55, 1.45, 0.0), makePose(6.25, 1.45), "map");
  const auto backward = planner.createPlan(
    costmap, makePose(6.25, 1.45, M_PI), makePose(0.55, 1.45), "map");

  ASSERT_GT(forward.path.poses.size(), 4U);
  ASSERT_GT(backward.path.poses.size(), 4U);
  EXPECT_LT(meanInteriorY(forward.path), 1.20);
  EXPECT_GT(meanInteriorY(backward.path), 1.70);

  for (const auto & path : {forward.path, backward.path}) {
    for (const auto & pose : path.poses) {
      EXPECT_GE(
        planner.clearanceAt(costmap, pose.pose.position.x, pose.pose.position.y),
        0.20 - 1e-6);
    }
  }
}

TEST(RouteClearancePlannerCoreTest, RightSideBiasUsesReferenceDirectionNotStartYaw)
{
  auto costmap = makeCostmap(70, 30, 0.1);
  addHorizontalWall(costmap, 3);
  addHorizontalWall(costmap, 26);

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.20;
  config.soft_target_clearance = 0.40;
  config.reference_corridor_half_width = 2.0;
  config.path_interpolation_resolution = 0.1;
  config.right_side_bias = true;
  config.right_side_weight = 8.0;
  config.right_side_target_clearance = 0.55;
  config.right_side_probe_distance = 1.5;
  config.right_side_max_offset = 0.7;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto facing_forward = planner.createPlan(
    costmap, makePose(0.55, 1.45, 0.0), makePose(6.25, 1.45), "map");
  const auto facing_backward = planner.createPlan(
    costmap, makePose(0.55, 1.45, M_PI), makePose(6.25, 1.45), "map");

  ASSERT_GT(facing_forward.path.poses.size(), 4U);
  ASSERT_GT(facing_backward.path.poses.size(), 4U);
  EXPECT_LT(meanInteriorY(facing_forward.path), 1.20);
  EXPECT_LT(meanInteriorY(facing_backward.path), 1.20);
}

TEST(RouteClearancePlannerCoreTest, RightSideBiasUsesReferenceDirectionOnVerticalRoute)
{
  auto costmap = makeCostmap(30, 70, 0.1);
  addVerticalWall(costmap, 3, 0, 0);
  addVerticalWall(costmap, 26, 0, 0);

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.20;
  config.soft_target_clearance = 0.40;
  config.reference_corridor_half_width = 2.0;
  config.path_interpolation_resolution = 0.1;
  config.right_side_bias = true;
  config.right_side_weight = 8.0;
  config.right_side_target_clearance = 0.55;
  config.right_side_probe_distance = 1.5;
  config.right_side_max_offset = 0.7;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto result = planner.createPlan(
    costmap, makePose(1.45, 0.55, M_PI), makePose(1.45, 6.25), "map");

  ASSERT_GT(result.path.poses.size(), 4U);
  double mean_x = 0.0;
  for (size_t index = 1; index + 1 < result.path.poses.size(); ++index) {
    mean_x += result.path.poses[index].pose.position.x;
  }
  mean_x /= static_cast<double>(result.path.poses.size() - 2U);
  EXPECT_GT(mean_x, 1.70);
}

TEST(RouteClearancePlannerCoreTest, RightSideBiasStaysNearReferenceWhenLeftSideIsWider)
{
  auto costmap = makeCostmap(70, 30, 0.1);
  addHorizontalWall(costmap, 3);
  addHorizontalWall(costmap, 26);

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.20;
  config.soft_target_clearance = 0.40;
  config.reference_corridor_half_width = 2.0;
  config.path_interpolation_resolution = 0.1;
  config.right_side_bias = true;
  config.right_side_weight = 8.0;
  config.right_side_target_clearance = 0.55;
  config.right_side_probe_distance = 1.5;
  config.right_side_max_offset = 0.7;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto result = planner.createPlan(
    costmap, makePose(0.55, 0.95, 0.0), makePose(6.25, 0.95), "map");

  ASSERT_GT(result.path.poses.size(), 4U);
  EXPECT_GT(meanInteriorY(result.path), 0.85);
}

TEST(RouteClearancePlannerCoreTest, RightSideBiasMovesLocallyWhereRightSideOpens)
{
  auto costmap = makeCostmap(100, 32, 0.1);
  addHorizontalWall(costmap, 3);
  addHorizontalWall(costmap, 21);
  for (unsigned int mx = 0; mx < costmap.getSizeInCellsX(); ++mx) {
    if (mx >= 35 && mx <= 65) {
      continue;
    }
    costmap.setCost(mx, 10, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.20;
  config.soft_target_clearance = 0.40;
  config.reference_corridor_half_width = 2.0;
  config.path_interpolation_resolution = 0.05;
  config.right_side_bias = true;
  config.right_side_weight = 8.0;
  config.right_side_target_clearance = 0.55;
  config.right_side_probe_distance = 1.5;
  config.right_side_max_offset = 0.7;
  config.lateral_smoothing_passes = 4;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto result = planner.createPlan(
    costmap, makePose(0.55, 1.45, 0.0), makePose(9.25, 1.45), "map");

  ASSERT_GT(result.path.poses.size(), 20U);
  EXPECT_LT(meanInteriorYInXRange(result.path, 4.0, 6.0), 1.25);
  EXPECT_GT(meanInteriorYInXRange(result.path, 0.8, 2.5), 1.30);
  EXPECT_GT(meanInteriorYInXRange(result.path, 7.0, 9.0), 1.30);
  EXPECT_LT(maxTurnDegrees(result.path), 18.0);
}

TEST(RouteClearancePlannerCoreTest, LateralChangeWeightKeepsRightBiasPathContinuous)
{
  auto costmap = makeCostmap(70, 30, 0.1);
  addHorizontalWall(costmap, 3);
  addHorizontalWall(costmap, 26);

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.20;
  config.soft_target_clearance = 0.40;
  config.reference_corridor_half_width = 2.0;
  config.path_interpolation_resolution = 0.1;
  config.right_side_bias = true;
  config.right_side_weight = 8.0;
  config.right_side_target_clearance = 0.55;
  config.right_side_probe_distance = 1.5;
  config.right_side_max_offset = 0.7;
  config.lateral_change_weight = 4.0;
  config.lateral_smoothing_passes = 2;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto result = planner.createPlan(
    costmap, makePose(0.55, 1.45), makePose(6.25, 1.45), "map");

  ASSERT_GT(result.path.poses.size(), 4U);
  EXPECT_LT(meanInteriorY(result.path), 1.20);
  for (const auto & pose : result.path.poses) {
    EXPECT_GE(
      planner.clearanceAt(costmap, pose.pose.position.x, pose.pose.position.y),
      0.20 - 1e-6);
  }
}

TEST(RouteClearancePlannerCoreTest, RightSideBiasDoesNotHugWallWhenClearanceIsLimited)
{
  auto costmap = makeCostmap(70, 30, 0.1);
  addHorizontalWall(costmap, 3);
  addHorizontalWall(costmap, 26);

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.15;
  config.soft_target_clearance = 0.60;
  config.reference_corridor_half_width = 2.0;
  config.path_interpolation_resolution = 0.1;
  config.right_side_bias = true;
  config.right_side_weight = 0.1;
  config.right_side_target_clearance = 0.60;
  config.right_side_probe_distance = 1.5;
  config.right_side_max_offset = 0.6;
  config.lateral_smoothing_passes = 2;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto result = planner.createPlan(
    costmap, makePose(0.55, 0.95), makePose(6.25, 0.95), "map");

  ASSERT_GT(result.path.poses.size(), 4U);
  for (size_t index = 1; index + 1 < result.path.poses.size(); ++index) {
    EXPECT_GE(
      planner.clearanceAt(
        costmap,
        result.path.poses[index].pose.position.x,
        result.path.poses[index].pose.position.y),
      0.60 - 1e-6);
  }
}

TEST(RouteClearancePlannerCoreTest, RightSideBiasAvoidsInflatedBandWhenFreeSpaceAvailable)
{
  auto costmap = makeCostmap(70, 36, 0.1);
  addHorizontalWall(costmap, 3);
  addHorizontalWall(costmap, 32);
  for (unsigned int mx = 0; mx < costmap.getSizeInCellsX(); ++mx) {
    for (unsigned int my = 4; my <= 11; ++my) {
      costmap.setCost(mx, my, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    }
  }

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.20;
  config.soft_target_clearance = 0.60;
  config.reference_corridor_half_width = 2.0;
  config.path_interpolation_resolution = 0.1;
  config.right_side_bias = true;
  config.right_side_weight = 0.1;
  config.right_side_target_clearance = 0.60;
  config.right_side_probe_distance = 1.5;
  config.right_side_max_offset = 0.7;
  config.lateral_smoothing_passes = 2;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto result = planner.createPlan(
    costmap, makePose(0.55, 2.05), makePose(6.25, 2.05), "map");

  ASSERT_GT(result.path.poses.size(), 4U);
  for (size_t index = 1; index + 1 < result.path.poses.size(); ++index) {
    unsigned int mx = 0;
    unsigned int my = 0;
    ASSERT_TRUE(
      costmap.worldToMap(
        result.path.poses[index].pose.position.x,
        result.path.poses[index].pose.position.y,
        mx,
        my));
    EXPECT_LT(costmap.getCost(mx, my), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    EXPECT_GE(result.path.poses[index].pose.position.y, 1.70);
  }
}

TEST(RouteClearancePlannerCoreTest, RightSideBiasTapersSmoothlyNearShortPathGoal)
{
  auto costmap = makeCostmap(40, 30, 0.1);
  addHorizontalWall(costmap, 3);
  addHorizontalWall(costmap, 26);

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.20;
  config.soft_target_clearance = 0.60;
  config.reference_corridor_half_width = 2.0;
  config.path_interpolation_resolution = 0.05;
  config.right_side_bias = true;
  config.right_side_weight = 0.1;
  config.right_side_target_clearance = 0.60;
  config.right_side_probe_distance = 1.5;
  config.right_side_max_offset = 0.7;
  config.lateral_smoothing_passes = 2;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto result = planner.createPlan(
    costmap, makePose(0.55, 1.45), makePose(2.05, 1.45), "map");

  ASSERT_GT(result.path.poses.size(), 8U);
  EXPECT_LT(maxTurnDegreesInHead(result.path, 12U), 18.0);
  EXPECT_LT(maxTurnDegreesInTail(result.path, 12U), 18.0);
}

TEST(RouteClearancePlannerCoreTest, RightSideBiasRampsDownSmoothlyNearLimitedClearance)
{
  auto costmap = makeCostmap(120, 40, 0.1);
  for (unsigned int mx = 70; mx < costmap.getSizeInCellsX(); ++mx) {
    costmap.setCost(mx, 14, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.20;
  config.soft_target_clearance = 0.60;
  config.centerline_weight = 0.0;
  config.cost_weight = 0.5;
  config.path_interpolation_resolution = 0.1;
  config.right_side_bias = true;
  config.right_side_weight = 8.0;
  config.right_side_target_clearance = 0.55;
  config.right_side_probe_distance = 1.5;
  config.right_side_max_offset = 0.7;
  config.lateral_smoothing_passes = 4;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto result = planner.createPlan(
    costmap, makePose(0.5, 2.5, 0.0), makePose(10.5, 2.5, 0.0), "map");

  ASSERT_FALSE(result.path.poses.empty());
  EXPECT_LT(maxTurnDegrees(result.path), 10.0);
}

TEST(RouteClearancePlannerCoreTest, RightSideBiasIgnoresShortOffsetPulses)
{
  auto costmap = makeCostmap(50, 90, 0.1);
  addVerticalWall(costmap, 3, 0, 0);
  addVerticalWall(costmap, 46, 0, 0);

  for (unsigned int my = 8; my < 82; my += 14) {
    for (unsigned int mx = 16; mx <= 20; ++mx) {
      for (unsigned int band_y = my; band_y < my + 4; ++band_y) {
        costmap.setCost(mx, band_y, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
      }
    }
  }

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.20;
  config.soft_target_clearance = 0.60;
  config.reference_corridor_half_width = 2.0;
  config.path_interpolation_resolution = 0.05;
  config.right_side_bias = true;
  config.right_side_weight = 0.1;
  config.right_side_target_clearance = 0.60;
  config.right_side_probe_distance = 1.5;
  config.right_side_max_offset = 0.7;
  config.lateral_smoothing_passes = 2;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto result = planner.createPlan(
    costmap, makePose(2.55, 8.15, -M_PI_2), makePose(2.55, 0.75, -M_PI_2), "map");

  ASSERT_GT(result.path.poses.size(), 20U);
  const double max_turn = maxTurnDegrees(result.path);
  EXPECT_LT(max_turn, 12.0);
}

TEST(RouteClearancePlannerCoreTest, RightSideBiasSeparatesForwardAndReturnAroundBlockedCenterline)
{
  auto costmap = makeCostmap(70, 30, 0.1);
  addHorizontalWall(costmap, 3);
  addHorizontalWall(costmap, 26);
  for (unsigned int my = 8; my <= 21; ++my) {
    costmap.setCost(35, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.20;
  config.soft_target_clearance = 0.40;
  config.reference_corridor_half_width = 2.0;
  config.path_interpolation_resolution = 0.1;
  config.right_side_bias = true;
  config.right_side_weight = 8.0;
  config.right_side_target_clearance = 0.55;
  config.right_side_probe_distance = 1.5;
  config.right_side_max_offset = 0.9;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto forward = planner.createPlan(
    costmap, makePose(0.55, 1.45, 0.0), makePose(6.25, 1.45), "map");
  const auto backward = planner.createPlan(
    costmap, makePose(6.25, 1.45, M_PI), makePose(0.55, 1.45), "map");

  ASSERT_GT(forward.path.poses.size(), 4U);
  ASSERT_GT(backward.path.poses.size(), 4U);
  const double forward_mean_y = meanInteriorY(forward.path);
  const double backward_mean_y = meanInteriorY(backward.path);
  EXPECT_LT(forward_mean_y, 1.20);
  EXPECT_GT(backward_mean_y, 1.45);
  EXPECT_GT(backward_mean_y - forward_mean_y, 0.45);
}

TEST(RouteClearancePlannerCoreTest, ReferenceCorridorAvoidsFarObstacleField)
{
  auto costmap = makeCostmap(120, 80, 0.1);
  for (unsigned int mx = 70; mx < costmap.getSizeInCellsX(); ++mx) {
    for (unsigned int my = 0; my < 25; ++my) {
      costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.20;
  config.soft_target_clearance = 0.50;
  config.path_interpolation_resolution = 0.1;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto result = planner.createPlan(
    costmap, makePose(0.45, 4.55), makePose(5.55, 4.55), "map");

  ASSERT_FALSE(result.path.poses.empty());
  EXPECT_NEAR(result.path.poses.front().pose.position.y, 4.55, 0.2);
  EXPECT_NEAR(result.path.poses.back().pose.position.y, 4.55, 0.2);
  EXPECT_LT(result.path.poses.size(), 120U);
}

TEST(RouteClearancePlannerCoreTest, ReferenceCorridorCanExpandWhenStraightTubeFails)
{
  auto costmap = makeCostmap(120, 80, 0.1);
  for (unsigned int my = 0; my < costmap.getSizeInCellsY(); ++my) {
    if (my < 45 || my > 60) {
      costmap.setCost(55, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.20;
  config.soft_target_clearance = 0.50;
  config.reference_corridor_half_width = 0.8;
  config.path_interpolation_resolution = 0.1;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto result = planner.createPlan(
    costmap, makePose(0.45, 4.55), makePose(8.55, 4.55), "map");

  ASSERT_FALSE(result.path.poses.empty());
  EXPECT_GT(result.path.poses.back().pose.position.x, 8.2);
}

TEST(RouteClearancePlannerCoreTest, InflatedInscribedCellsAreSoftCostNotHardObstacles)
{
  auto costmap = makeCostmap(60, 15, 0.1);
  for (unsigned int my = 0; my < costmap.getSizeInCellsY(); ++my) {
    costmap.setCost(30, my, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  }

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.25;
  config.soft_target_clearance = 0.50;
  config.path_interpolation_resolution = 0.1;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto result = planner.createPlan(
    costmap, makePose(0.35, 0.75), makePose(5.55, 0.75), "map");

  ASSERT_FALSE(result.path.poses.empty());
  bool crossed_high_cost_band = false;
  for (const auto & pose : result.path.poses) {
    if (std::abs(pose.pose.position.x - 3.05) < 0.08) {
      crossed_high_cost_band = true;
    }
  }
  EXPECT_TRUE(crossed_high_cost_band);
}

TEST(RouteClearancePlannerCoreTest, NarrowGapCanUseCellsBelowSoftClearance)
{
  auto costmap = makeCostmap(40, 30, 0.1);
  addVerticalWall(costmap, 20, 13, 16);

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.15;
  config.soft_target_clearance = 0.50;
  config.clearance_weight = 5.0;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto result = planner.createPlan(
    costmap, makePose(0.35, 1.45), makePose(3.55, 1.45), "map");

  ASSERT_FALSE(result.path.poses.empty());
  bool passed_narrow_soft_zone = false;
  for (const auto & pose : result.path.poses) {
    const double clearance = planner.clearanceAt(
      costmap, pose.pose.position.x, pose.pose.position.y);
    EXPECT_GE(clearance, 0.15 - 1e-6);
    if (pose.pose.position.x > 1.8 && pose.pose.position.x < 2.2 && clearance < 0.50) {
      passed_narrow_soft_zone = true;
    }
  }
  EXPECT_TRUE(passed_narrow_soft_zone);
}

TEST(RouteClearancePlannerCoreTest, StartGoalKeepoutCanBeLowerThanHardClearance)
{
  auto costmap = makeCostmap(70, 30, 0.1);
  addVerticalWall(costmap, 0, 0, 0);

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.55;
  config.soft_target_clearance = 0.60;
  config.start_goal_keepout_radius = 0.35;
  config.path_interpolation_resolution = 0.1;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto result = planner.createPlan(
    costmap, makePose(0.45, 1.45), makePose(5.55, 1.45), "map");

  ASSERT_FALSE(result.path.poses.empty());
  EXPECT_LT(planner.clearanceAt(costmap, 0.45, 1.45), config.hard_min_clearance);
  EXPECT_GE(planner.clearanceAt(costmap, 0.45, 1.45), config.start_goal_keepout_radius);
  for (size_t index = 1; index + 1 < result.path.poses.size(); ++index) {
    const auto & pose = result.path.poses[index];
    if (std::hypot(pose.pose.position.x - 0.45, pose.pose.position.y - 1.45) <=
      config.start_goal_keepout_radius)
    {
      continue;
    }
    EXPECT_GE(
      planner.clearanceAt(costmap, pose.pose.position.x, pose.pose.position.y),
      config.hard_min_clearance - 1e-6);
  }
}

TEST(RouteClearancePlannerCoreTest, BlockedGoalIsMovedToSafeNearbyPose)
{
  auto costmap = makeCostmap(40, 30, 0.1);
  costmap.setCost(30, 15, nav2_costmap_2d::LETHAL_OBSTACLE);

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.20;
  config.goal_search_radius = 0.8;
  config.max_goal_candidates = 120;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto goal = makePose(3.05, 1.55, -1.2);
  const auto result = planner.createPlan(costmap, makePose(0.35, 1.55), goal, "map");

  EXPECT_TRUE(result.adjusted_goal);
  ASSERT_FALSE(result.path.poses.empty());
  const auto & end_pose = result.path.poses.back();
  EXPECT_GT(
    std::hypot(
      end_pose.pose.position.x - goal.pose.position.x,
      end_pose.pose.position.y - goal.pose.position.y), 0.05);
  EXPECT_LE(
    std::hypot(
      end_pose.pose.position.x - goal.pose.position.x,
      end_pose.pose.position.y - goal.pose.position.y), 0.8);
  EXPECT_GE(
    planner.clearanceAt(
      costmap, end_pose.pose.position.x,
      end_pose.pose.position.y), 0.20 - 1e-6);
  EXPECT_NEAR(end_pose.pose.orientation.z, goal.pose.orientation.z, 1e-6);
  EXPECT_NEAR(end_pose.pose.orientation.w, goal.pose.orientation.w, 1e-6);
}

TEST(RouteClearancePlannerCoreTest, FullyBlockedMapThrows)
{
  auto costmap = makeCostmap(20, 20, 0.1);
  for (unsigned int my = 0; my < costmap.getSizeInCellsY(); ++my) {
    costmap.setCost(10, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.1;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  EXPECT_THROW(
    planner.createPlan(costmap, makePose(0.25, 1.0), makePose(1.75, 1.0), "map"),
    nav2_core::PlannerException);
}

TEST(RouteClearancePlannerCoreTest, IntermediateOrientationsFollowPathTangent)
{
  auto costmap = makeCostmap(40, 30, 0.1);
  addVerticalWall(costmap, 20, 22, 26);

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.20;
  config.path_interpolation_resolution = 0.1;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto result = planner.createPlan(
    costmap, makePose(0.35, 1.45, 0.7), makePose(3.55, 1.45, -0.7), "map");

  ASSERT_GT(result.path.poses.size(), 4U);
  EXPECT_NEAR(
    result.path.poses.front().pose.orientation.z, makePose(
      0.0, 0.0,
      0.7).pose.orientation.z, 1e-6);
  EXPECT_NEAR(
    result.path.poses.back().pose.orientation.z, makePose(
      0.0, 0.0,
      -0.7).pose.orientation.z, 1e-6);

  for (size_t index = 1; index + 1 < result.path.poses.size(); ++index) {
    const auto & current = result.path.poses[index].pose.position;
    const auto & next = result.path.poses[index + 1].pose.position;
    const double expected_yaw = std::atan2(next.y - current.y, next.x - current.x);
    const double actual_yaw = yawFromQuaternion(result.path.poses[index].pose.orientation);
    EXPECT_NEAR(normalizeAngle(actual_yaw - expected_yaw), 0.0, 1.0e-5);
  }
}

TEST(RouteClearancePlannerCoreTest, DefersGlobalClearanceMapUntilGoalAdjustment)
{
  auto costmap = makeCostmap(60, 30, 0.1);

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.20;
  config.soft_target_clearance = 0.50;
  config.path_interpolation_resolution = 0.2;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  planner.createPlan(costmap, makePose(0.35, 1.45), makePose(2.55, 1.45), "map");
  const auto first_stats = planner.clearanceCacheStats();
  EXPECT_EQ(first_stats.builds, 0U);
  EXPECT_EQ(first_stats.hits, 0U);

  costmap.setCost(25, 14, nav2_costmap_2d::LETHAL_OBSTACLE);
  const auto adjusted = planner.createPlan(
    costmap, makePose(0.35, 1.45), makePose(2.55, 1.45), "map");
  const auto second_stats = planner.clearanceCacheStats();
  EXPECT_EQ(second_stats.builds, 1U);
  EXPECT_EQ(second_stats.hits, 0U);
  EXPECT_TRUE(adjusted.adjusted_goal);

  planner.createPlan(costmap, makePose(0.35, 1.45), makePose(2.55, 1.45), "map");
  const auto cached_stats = planner.clearanceCacheStats();
  EXPECT_EQ(cached_stats.builds, 1U);
  EXPECT_EQ(cached_stats.hits, 1U);

  costmap.setCost(50, 25, nav2_costmap_2d::LETHAL_OBSTACLE);
  planner.createPlan(costmap, makePose(0.35, 1.45), makePose(2.55, 1.45), "map");
  const auto changed_stats = planner.clearanceCacheStats();
  EXPECT_EQ(changed_stats.builds, 2U);
  EXPECT_EQ(changed_stats.hits, 1U);
}

TEST(RouteClearancePlannerCoreTest, PoseDirectedCropAvoidsFullMapReferenceSearch)
{
  auto costmap = makeCostmap(1600, 1000, 0.05);
  for (unsigned int my = 0; my < costmap.getSizeInCellsY(); ++my) {
    if (my < 430 || my > 450) {
      costmap.setCost(800, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.4;
  config.soft_target_clearance = 1.0;
  config.clearance_weight = 8.0;
  config.cost_weight = 2.5;
  config.turn_weight = 0.45;
  config.reference_corridor_half_width = 2.5;
  config.pose_directed_crop_enabled = true;
  config.goal_approach_length = 1.0;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto start_time = std::chrono::steady_clock::now();
  const auto result = planner.createPlan(
    costmap, makePose(2.0, 20.0), makePose(78.0, 20.0, 0.0), "map");
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start_time);

  ASSERT_FALSE(result.path.poses.empty());
  EXPECT_LT(elapsed.count(), 450);
}

TEST(RouteClearancePlannerCoreTest, PoseDirectedCropCanExpandBeforeGlobalFallback)
{
  auto costmap = makeCostmap(1200, 800, 0.05);
  for (unsigned int my = 0; my < costmap.getSizeInCellsY(); ++my) {
    if (my < 480 || my > 500) {
      costmap.setCost(600, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.35;
  config.soft_target_clearance = 0.8;
  config.reference_corridor_half_width = 2.0;
  config.pose_directed_crop_enabled = true;
  config.pose_directed_max_corridor_half_width = 5.0;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto start_time = std::chrono::steady_clock::now();
  const auto result = planner.createPlan(
    costmap, makePose(2.0, 20.0), makePose(55.0, 20.0, 0.0), "map");
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start_time);

  ASSERT_FALSE(result.path.poses.empty());
  EXPECT_LT(elapsed.count(), 650);
}

TEST(RouteClearancePlannerCoreTest, LongReferenceSearchAvoidsFullMapWork)
{
  auto costmap = makeCostmap(1400, 3600, 0.05);
  for (unsigned int mx = 0; mx < costmap.getSizeInCellsX(); ++mx) {
    if (mx >= 820 && mx <= 900) {
      continue;
    }
    costmap.setCost(mx, 1800, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.hard_min_clearance = 0.25;
  config.soft_target_clearance = 0.60;
  config.reference_corridor_half_width = 5.0;
  config.pose_directed_crop_enabled = false;
  config.path_interpolation_resolution = 0.2;
  config.output_path_resolution = 0.5;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto start_time = std::chrono::steady_clock::now();
  const auto result = planner.createPlan(
    costmap, makePose(5.0, 10.0), makePose(5.0, 160.0, 0.0), "map");
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start_time);

  ASSERT_FALSE(result.path.poses.empty());
  EXPECT_LT(elapsed.count(), 2500);
}

TEST(RouteClearancePlannerCoreTest, OutputPathResolutionReducesReturnedPoseCount)
{
  auto costmap = makeCostmap(300, 40, 0.05);

  nav2_route_polyline_planner::RouteClearancePlannerConfig dense_config;
  dense_config.hard_min_clearance = 0.20;
  dense_config.path_interpolation_resolution = 0.05;
  nav2_route_polyline_planner::RouteClearancePlannerCore dense_planner(dense_config);

  nav2_route_polyline_planner::RouteClearancePlannerConfig sparse_config = dense_config;
  sparse_config.output_path_resolution = 0.50;
  nav2_route_polyline_planner::RouteClearancePlannerCore sparse_planner(sparse_config);

  const auto start = makePose(0.5, 1.0, 0.3);
  const auto goal = makePose(14.5, 1.0, -0.6);
  const auto dense_result = dense_planner.createPlan(costmap, start, goal, "map");
  const auto sparse_result = sparse_planner.createPlan(costmap, start, goal, "map");

  ASSERT_FALSE(dense_result.path.poses.empty());
  ASSERT_FALSE(sparse_result.path.poses.empty());
  EXPECT_LT(sparse_result.path.poses.size(), dense_result.path.poses.size() / 4U);
  EXPECT_NEAR(sparse_result.path.poses.front().pose.position.x, start.pose.position.x, 1e-6);
  EXPECT_NEAR(sparse_result.path.poses.front().pose.position.y, start.pose.position.y, 1e-6);
  EXPECT_NEAR(sparse_result.path.poses.back().pose.position.x, goal.pose.position.x, 1e-6);
  EXPECT_NEAR(sparse_result.path.poses.back().pose.position.y, goal.pose.position.y, 1e-6);
  EXPECT_NEAR(
    sparse_result.path.poses.back().pose.orientation.z,
    goal.pose.orientation.z,
    1e-6);
}
