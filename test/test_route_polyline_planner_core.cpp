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

#include "gtest/gtest.h"

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_route_polyline_planner/route_polyline_planner_core.hpp"
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
  unsigned int size_x = 10,
  unsigned int size_y = 10,
  double resolution = 1.0)
{
  return nav2_costmap_2d::Costmap2D(
    size_x, size_y, resolution, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
}

}  // namespace

TEST(RoutePolylinePlannerCoreTest, StraightSegmentIsCollisionFreeInFreeSpace)
{
  auto costmap = makeCostmap();
  nav2_route_polyline_planner::RoutePolylinePlannerConfig config;
  config.collision_check_resolution = 0.5;
  nav2_route_polyline_planner::RoutePolylinePlannerCore planner(config);

  EXPECT_TRUE(
    planner.isSegmentCollisionFree(
      costmap, makePose(0.5, 0.5), makePose(5.5, 0.5)));
}

TEST(RoutePolylinePlannerCoreTest, StraightSegmentDetectsObstacle)
{
  auto costmap = makeCostmap();
  costmap.setCost(3, 0, nav2_costmap_2d::LETHAL_OBSTACLE);

  nav2_route_polyline_planner::RoutePolylinePlannerConfig config;
  config.collision_check_resolution = 0.5;
  nav2_route_polyline_planner::RoutePolylinePlannerCore planner(config);

  EXPECT_FALSE(
    planner.isSegmentCollisionFree(
      costmap, makePose(0.5, 0.5), makePose(5.5, 0.5)));
}

TEST(RoutePolylinePlannerCoreTest, CorridorMaskMarksCellsOutsideCorridorAsLethal)
{
  auto costmap = makeCostmap();
  nav2_route_polyline_planner::RoutePolylinePlannerConfig config;
  config.fallback_corridor_half_width = 0.2;
  config.start_goal_keepout_radius = 0.0;
  nav2_route_polyline_planner::RoutePolylinePlannerCore planner(config);

  auto masked = planner.buildMaskedCostmapData(
    costmap, makePose(1.5, 1.5), makePose(8.5, 1.5));

  EXPECT_EQ(masked[costmap.getIndex(5, 1)], nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(masked[costmap.getIndex(5, 2)], nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(RoutePolylinePlannerCoreTest, KeepoutRadiusPreservesCellsNearEndpoints)
{
  auto costmap = makeCostmap();
  costmap.setCost(1, 2, 80);

  nav2_route_polyline_planner::RoutePolylinePlannerConfig config;
  config.fallback_corridor_half_width = 0.2;
  config.start_goal_keepout_radius = 1.1;
  nav2_route_polyline_planner::RoutePolylinePlannerCore planner(config);

  auto masked = planner.buildMaskedCostmapData(
    costmap, makePose(1.5, 1.5), makePose(8.5, 1.5));

  EXPECT_EQ(masked[costmap.getIndex(1, 2)], 80);
}

TEST(RoutePolylinePlannerCoreTest, StraightPathUsesGoalOrientationOnFinalPose)
{
  auto costmap = makeCostmap();
  nav2_route_polyline_planner::RoutePolylinePlannerConfig config;
  config.path_interpolation_resolution = 1.0;
  nav2_route_polyline_planner::RoutePolylinePlannerCore planner(config);

  auto path = planner.buildStraightPath(
    makePose(0.5, 0.5), makePose(3.5, 0.5, 1.57), "map");

  ASSERT_GE(path.poses.size(), 2U);
  EXPECT_EQ(path.header.frame_id, "map");
  EXPECT_NEAR(path.poses.front().pose.orientation.z, 0.0, 1e-6);
  EXPECT_NEAR(path.poses.front().pose.orientation.w, 1.0, 1e-6);
  EXPECT_NEAR(
    path.poses.back().pose.orientation.z,
    makePose(0.0, 0.0, 1.57).pose.orientation.z,
    1e-6);
  EXPECT_NEAR(
    path.poses.back().pose.orientation.w,
    makePose(0.0, 0.0, 1.57).pose.orientation.w,
    1e-6);
}

TEST(RoutePolylinePlannerCoreTest, BlockedSegmentFallsBackToCorridorNavFn)
{
  auto costmap = makeCostmap();
  costmap.setCost(4, 1, nav2_costmap_2d::LETHAL_OBSTACLE);

  nav2_route_polyline_planner::RoutePolylinePlannerConfig config;
  config.path_interpolation_resolution = 0.5;
  config.collision_check_resolution = 0.5;
  config.fallback_corridor_half_width = 1.5;
  nav2_route_polyline_planner::RoutePolylinePlannerCore planner(config);

  auto result = planner.planSegment(
    costmap, makePose(1.5, 1.5), makePose(8.5, 1.5), "map");

  EXPECT_TRUE(result.used_fallback);
  ASSERT_FALSE(result.path.poses.empty());
  EXPECT_GT(result.path.poses.back().pose.position.x, 8.0);

  bool found_detour = false;
  for (const auto & pose : result.path.poses) {
    if (pose.pose.position.y > 1.6) {
      found_detour = true;
      break;
    }
  }
  EXPECT_TRUE(found_detour);
}

TEST(RoutePolylinePlannerCoreTest, FullyBlockedCorridorThrows)
{
  auto costmap = makeCostmap();
  for (unsigned int y = 0; y <= 3; ++y) {
    costmap.setCost(4, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  nav2_route_polyline_planner::RoutePolylinePlannerConfig config;
  config.path_interpolation_resolution = 0.5;
  config.collision_check_resolution = 0.5;
  config.fallback_corridor_half_width = 1.5;
  nav2_route_polyline_planner::RoutePolylinePlannerCore planner(config);

  EXPECT_THROW(
    planner.planSegment(costmap, makePose(1.5, 1.5), makePose(8.5, 1.5), "map"),
    nav2_core::PlannerException);
}
