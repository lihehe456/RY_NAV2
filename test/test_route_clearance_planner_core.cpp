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

}  // namespace

TEST(RouteClearancePlannerCoreTest, FreeSpaceReturnsNearlyStraightPath)
{
  auto costmap = makeCostmap();
  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  EXPECT_FALSE(config.debug_timing);
  config.debug_timing = true;
  config.path_interpolation_resolution = 0.2;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  const auto result = planner.createPlan(
    costmap, makePose(0.25, 0.25), makePose(2.45, 0.25, 1.57), "map");

  EXPECT_GT(result.timing.total_ms, 0.0);
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
  config.reference_search_margin = 0.3;
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

TEST(RouteClearancePlannerCoreTest, ReusesGlobalClearanceMapUntilCostmapChanges)
{
  auto costmap = makeCostmap(60, 30, 0.1);

  nav2_route_polyline_planner::RouteClearancePlannerConfig config;
  config.debug_timing = true;
  config.hard_min_clearance = 0.20;
  config.soft_target_clearance = 0.50;
  config.path_interpolation_resolution = 0.2;
  nav2_route_polyline_planner::RouteClearancePlannerCore planner(config);

  planner.createPlan(costmap, makePose(0.35, 1.45), makePose(2.55, 1.45), "map");
  const auto first_stats = planner.clearanceCacheStats();
  EXPECT_EQ(first_stats.builds, 1U);
  EXPECT_EQ(first_stats.hits, 0U);

  planner.createPlan(costmap, makePose(2.55, 1.45), makePose(5.35, 1.45), "map");
  const auto second_stats = planner.clearanceCacheStats();
  EXPECT_EQ(second_stats.builds, 1U);
  EXPECT_EQ(second_stats.hits, 1U);

  costmap.setCost(30, 15, nav2_costmap_2d::LETHAL_OBSTACLE);
  planner.createPlan(costmap, makePose(0.35, 0.55), makePose(2.55, 0.55), "map");
  const auto changed_stats = planner.clearanceCacheStats();
  EXPECT_EQ(changed_stats.builds, 2U);
  EXPECT_EQ(changed_stats.hits, 1U);
}
