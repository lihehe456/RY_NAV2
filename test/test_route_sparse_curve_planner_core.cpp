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
#include "nav2_route_polyline_planner/route_sparse_curve_planner_core.hpp"
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
  unsigned int size_x = 12,
  unsigned int size_y = 12,
  double resolution = 1.0)
{
  return nav2_costmap_2d::Costmap2D(
    size_x, size_y, resolution, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
}

double degToRad(double degrees)
{
  return degrees * M_PI / 180.0;
}

}  // namespace

TEST(RouteSparseCurvePlannerCoreTest, StraightModeReturnsStraightLineWhenYawMatchesChord)
{
  auto costmap = makeCostmap();
  nav2_route_polyline_planner::RouteSparseCurvePlannerConfig config;
  config.path_interpolation_resolution = 0.5;
  config.straight_angle_threshold_deg = 12.0;
  nav2_route_polyline_planner::RouteSparseCurvePlannerCore planner(config);

  auto result = planner.planSegment(
    costmap,
    makePose(1.5, 1.5, 0.0),
    makePose(8.5, 1.5, 0.0),
    "map");

  EXPECT_FALSE(result.used_curve);
  EXPECT_FALSE(result.used_fallback);
  ASSERT_FALSE(result.path.poses.empty());
  for (const auto & pose : result.path.poses) {
    EXPECT_NEAR(pose.pose.position.y, 1.5, 1e-6);
  }
  EXPECT_NEAR(
    result.path.poses.back().pose.orientation.z,
    makePose(0.0, 0.0, 0.0).pose.orientation.z,
    1e-6);
}

TEST(RouteSparseCurvePlannerCoreTest, CurveModeReturnsHermiteCurveWhenEndpointYawDiffersFromChord)
{
  auto costmap = makeCostmap();
  nav2_route_polyline_planner::RouteSparseCurvePlannerConfig config;
  config.curve_sample_resolution = 0.25;
  config.tangent_scale = 0.45;
  config.max_tangent_length = 2.0;
  config.straight_angle_threshold_deg = 12.0;
  nav2_route_polyline_planner::RouteSparseCurvePlannerCore planner(config);

  const auto start = makePose(1.5, 1.5, degToRad(45.0));
  const auto goal = makePose(8.5, 1.5, degToRad(-45.0));
  auto result = planner.planSegment(costmap, start, goal, "map");

  EXPECT_TRUE(result.used_curve);
  EXPECT_FALSE(result.used_fallback);
  ASSERT_GT(result.path.poses.size(), 2U);
  EXPECT_NEAR(result.path.poses.front().pose.position.x, start.pose.position.x, 1e-6);
  EXPECT_NEAR(result.path.poses.front().pose.position.y, start.pose.position.y, 1e-6);
  EXPECT_NEAR(result.path.poses.back().pose.position.x, goal.pose.position.x, 1e-6);
  EXPECT_NEAR(result.path.poses.back().pose.position.y, goal.pose.position.y, 1e-6);
  EXPECT_NEAR(result.path.poses.front().pose.orientation.z, start.pose.orientation.z, 1e-6);
  EXPECT_NEAR(result.path.poses.front().pose.orientation.w, start.pose.orientation.w, 1e-6);
  EXPECT_NEAR(result.path.poses.back().pose.orientation.z, goal.pose.orientation.z, 1e-6);
  EXPECT_NEAR(result.path.poses.back().pose.orientation.w, goal.pose.orientation.w, 1e-6);

  bool found_curvature = false;
  for (const auto & pose : result.path.poses) {
    if (std::abs(pose.pose.position.y - 1.5) > 0.1) {
      found_curvature = true;
      break;
    }
  }
  EXPECT_TRUE(found_curvature);
}

TEST(RouteSparseCurvePlannerCoreTest, StraightModeThresholdUsesConfiguredAngle)
{
  auto costmap = makeCostmap();
  nav2_route_polyline_planner::RouteSparseCurvePlannerConfig config;
  config.straight_angle_threshold_deg = 12.0;
  nav2_route_polyline_planner::RouteSparseCurvePlannerCore planner(config);

  EXPECT_TRUE(
    planner.shouldUseStraightMode(
      makePose(1.5, 1.5, degToRad(10.0)),
      makePose(8.5, 1.5, degToRad(-10.0))));
  EXPECT_FALSE(
    planner.shouldUseStraightMode(
      makePose(1.5, 1.5, degToRad(15.0)),
      makePose(8.5, 1.5, 0.0)));
  (void) costmap;
}

TEST(RouteSparseCurvePlannerCoreTest, CurveCollisionTriggersFallback)
{
  auto costmap = makeCostmap(24, 24, 0.5);
  nav2_route_polyline_planner::RouteSparseCurvePlannerConfig config;
  config.curve_sample_resolution = 0.25;
  config.fallback_corridor_half_width = 1.4;
  config.straight_angle_threshold_deg = 12.0;
  nav2_route_polyline_planner::RouteSparseCurvePlannerCore planner(config);

  const auto start = makePose(1.0, 1.0, degToRad(45.0));
  const auto goal = makePose(5.0, 1.0, degToRad(-45.0));
  const auto curve_points = planner.buildHermiteCurvePoints(costmap, start, goal);
  const auto midpoint = curve_points[curve_points.size() / 2];

  unsigned int mx = 0;
  unsigned int my = 0;
  ASSERT_TRUE(costmap.worldToMap(midpoint.first, midpoint.second, mx, my));
  costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);

  auto result = planner.planSegment(costmap, start, goal, "map");
  EXPECT_TRUE(result.used_curve);
  EXPECT_TRUE(result.used_fallback);
  EXPECT_FALSE(result.path.poses.empty());
}

TEST(RouteSparseCurvePlannerCoreTest, HighCurveCostTriggersFallback)
{
  auto costmap = makeCostmap();
  nav2_route_polyline_planner::RouteSparseCurvePlannerConfig config;
  config.curve_sample_resolution = 0.25;
  config.max_curve_cell_cost = 150;
  config.fallback_corridor_half_width = 1.2;
  config.straight_angle_threshold_deg = 12.0;
  nav2_route_polyline_planner::RouteSparseCurvePlannerCore planner(config);

  const auto start = makePose(1.5, 1.5, 0.0);
  const auto goal = makePose(8.5, 1.5, degToRad(90.0));
  const auto curve_points = planner.buildHermiteCurvePoints(costmap, start, goal);
  const auto midpoint = curve_points[curve_points.size() / 2];

  unsigned int mx = 0;
  unsigned int my = 0;
  ASSERT_TRUE(costmap.worldToMap(midpoint.first, midpoint.second, mx, my));
  costmap.setCost(mx, my, 200);

  auto result = planner.planSegment(costmap, start, goal, "map");
  EXPECT_TRUE(result.used_fallback);
  EXPECT_FALSE(result.path.poses.empty());
}

TEST(RouteSparseCurvePlannerCoreTest, CurveCorridorMaskFollowsReferencePolyline)
{
  auto costmap = makeCostmap();
  nav2_route_polyline_planner::RouteSparseCurvePlannerConfig config;
  config.fallback_corridor_half_width = 0.8;
  config.start_goal_keepout_radius = 0.0;
  nav2_route_polyline_planner::RouteSparseCurvePlannerCore planner(config);

  const std::vector<std::pair<double, double>> polyline{
    {1.5, 1.5},
    {4.5, 4.5},
    {8.5, 1.5},
  };
  auto masked = planner.buildMaskedCostmapDataForPolyline(
    costmap,
    polyline,
    makePose(1.5, 1.5),
    makePose(8.5, 1.5));

  EXPECT_EQ(masked[costmap.getIndex(4, 4)], nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(masked[costmap.getIndex(4, 1)], nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(RouteSparseCurvePlannerCoreTest, FullyBlockedCurveCorridorThrows)
{
  auto costmap = makeCostmap();
  nav2_route_polyline_planner::RouteSparseCurvePlannerConfig config;
  config.curve_sample_resolution = 0.25;
  config.fallback_corridor_half_width = 0.9;
  config.straight_angle_threshold_deg = 12.0;
  nav2_route_polyline_planner::RouteSparseCurvePlannerCore planner(config);

  const auto start = makePose(1.5, 1.5, 0.0);
  const auto goal = makePose(8.5, 1.5, degToRad(90.0));
  const auto curve_points = planner.buildHermiteCurvePoints(costmap, start, goal);

  for (const auto & point : curve_points) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (costmap.worldToMap(point.first, point.second, mx, my)) {
      costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  EXPECT_THROW(planner.planSegment(costmap, start, goal, "map"), nav2_core::PlannerException);
}

TEST(RouteSparseCurvePlannerCoreTest, TwoCurveSegmentsMeetAtWaypointOrientation)
{
  auto costmap = makeCostmap(20, 20, 1.0);
  nav2_route_polyline_planner::RouteSparseCurvePlannerConfig config;
  config.curve_sample_resolution = 0.25;
  config.straight_angle_threshold_deg = 12.0;
  nav2_route_polyline_planner::RouteSparseCurvePlannerCore planner(config);

  const auto shared = makePose(6.5, 6.5, degToRad(45.0));
  auto first = planner.planSegment(
    costmap,
    makePose(1.5, 4.5, 0.0),
    shared,
    "map");
  auto second = planner.planSegment(
    costmap,
    shared,
    makePose(10.5, 10.5, degToRad(90.0)),
    "map");

  EXPECT_TRUE(first.used_curve);
  EXPECT_TRUE(second.used_curve);
  ASSERT_FALSE(first.path.poses.empty());
  ASSERT_FALSE(second.path.poses.empty());
  EXPECT_NEAR(first.path.poses.back().pose.orientation.z, shared.pose.orientation.z, 1e-6);
  EXPECT_NEAR(first.path.poses.back().pose.orientation.w, shared.pose.orientation.w, 1e-6);
  EXPECT_NEAR(second.path.poses.front().pose.orientation.z, shared.pose.orientation.z, 1e-6);
  EXPECT_NEAR(second.path.poses.front().pose.orientation.w, shared.pose.orientation.w, 1e-6);
}
