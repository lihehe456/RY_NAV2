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
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_route_polyline_planner/route_narrow_planner_core.hpp"
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
  unsigned int size_x = 80,
  unsigned int size_y = 50,
  double resolution = 0.1)
{
  return nav2_costmap_2d::Costmap2D(
    size_x, size_y, resolution, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
}

void addHorizontalWall(nav2_costmap_2d::Costmap2D & costmap, unsigned int my)
{
  for (unsigned int mx = 0; mx < costmap.getSizeInCellsX(); ++mx) {
    costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
}

double meanInteriorY(
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
  return count == 0U ? 0.0 : sum / static_cast<double>(count);
}

double maxTurnDegrees(const nav_msgs::msg::Path & path)
{
  double max_turn = 0.0;
  for (size_t index = 1; index + 1 < path.poses.size(); ++index) {
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

}  // namespace

TEST(RouteNarrowPlannerCoreTest, ReturnsToCorridorCenterWhenEndpointsAreNearOneWall)
{
  auto costmap = makeCostmap();
  addHorizontalWall(costmap, 10);
  addHorizontalWall(costmap, 30);

  nav2_route_polyline_planner::RouteNarrowPlannerConfig config;
  config.hard_min_clearance = 0.25;
  config.soft_target_clearance = 0.90;
  config.path_interpolation_resolution = 0.1;
  config.output_path_resolution = 0.1;
  nav2_route_polyline_planner::RouteNarrowPlannerCore planner(config);

  const auto path = planner.createPlan(
    costmap, makePose(0.5, 1.35), makePose(7.5, 1.35, 0.0), "map");

  ASSERT_GT(path.poses.size(), 4U);
  EXPECT_NEAR(path.poses.front().pose.position.y, 1.35, 0.11);
  EXPECT_NEAR(path.poses.back().pose.position.y, 1.35, 0.11);
  EXPECT_NEAR(meanInteriorY(path, 2.0, 6.0), 2.0, 0.18);
  EXPECT_LT(maxTurnDegrees(path), 35.0);
  for (const auto & pose : path.poses) {
    EXPECT_GE(
      planner.clearanceAt(costmap, pose.pose.position.x, pose.pose.position.y),
      config.hard_min_clearance - 1e-6);
  }
}
