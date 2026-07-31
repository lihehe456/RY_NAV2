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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_route_polyline_planner/local_costmap_overlay.hpp"

namespace
{

nav2_costmap_2d::Costmap2D makeGlobalCostmap()
{
  return nav2_costmap_2d::Costmap2D(
    80, 80, 0.1, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
}

nav2_msgs::msg::Costmap makeLocalCostmap(
  unsigned int size_x = 4,
  unsigned int size_y = 4,
  double resolution = 0.1)
{
  nav2_msgs::msg::Costmap costmap;
  costmap.header.frame_id = "odom";
  costmap.metadata.size_x = size_x;
  costmap.metadata.size_y = size_y;
  costmap.metadata.resolution = static_cast<float>(resolution);
  costmap.metadata.origin.orientation.w = 1.0;
  costmap.data.assign(
    static_cast<size_t>(size_x) * static_cast<size_t>(size_y),
    nav2_costmap_2d::FREE_SPACE);
  return costmap;
}

geometry_msgs::msg::TransformStamped makeTransform(double x, double y)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "odom";
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.rotation.w = 1.0;
  return transform;
}

geometry_msgs::msg::PoseStamped makePose(double x, double y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;
  return pose;
}

}  // namespace

TEST(LocalCostmapOverlayTest, OverlaysTransformedLocalCostsNearStart)
{
  auto global_costmap = makeGlobalCostmap();
  auto local_costmap = makeLocalCostmap();
  local_costmap.data[static_cast<size_t>(1U * local_costmap.metadata.size_x + 1U)] = 120;
  local_costmap.data[static_cast<size_t>(3U * local_costmap.metadata.size_x + 3U)] = 160;

  nav2_route_polyline_planner::LocalCostmapOverlayOptions options;
  options.influence_distance = 0.20;
  options.min_overlay_cost = 1;
  nav2_route_polyline_planner::LocalCostmapOverlayStats stats;

  const bool applied = nav2_route_polyline_planner::overlayLocalCostmap(
    local_costmap, makeTransform(1.0, 2.0), makePose(1.15, 2.15), options, global_costmap, &stats);

  EXPECT_TRUE(applied);
  EXPECT_EQ(global_costmap.getCost(11, 21), 120);
  EXPECT_EQ(global_costmap.getCost(13, 23), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(stats.overlaid_cells, 1U);
}

TEST(LocalCostmapOverlayTest, NeverLowersExistingGlobalCost)
{
  auto global_costmap = makeGlobalCostmap();
  global_costmap.setCost(11, 21, 220);
  auto local_costmap = makeLocalCostmap();
  local_costmap.data[static_cast<size_t>(1U * local_costmap.metadata.size_x + 1U)] = 100;

  nav2_route_polyline_planner::LocalCostmapOverlayOptions options;
  options.influence_distance = 1.0;

  const bool applied = nav2_route_polyline_planner::overlayLocalCostmap(
    local_costmap, makeTransform(1.0, 2.0), makePose(1.15, 2.15), options, global_costmap);

  EXPECT_TRUE(applied);
  EXPECT_EQ(global_costmap.getCost(11, 21), 220);
}

TEST(LocalCostmapOverlayTest, ReportsRestoreCellsForRaisedCosts)
{
  auto global_costmap = makeGlobalCostmap();
  global_costmap.setCost(11, 21, 20);
  auto local_costmap = makeLocalCostmap();
  local_costmap.data[static_cast<size_t>(1U * local_costmap.metadata.size_x + 1U)] = 100;

  nav2_route_polyline_planner::LocalCostmapOverlayOptions options;
  options.influence_distance = 1.0;
  std::vector<nav2_route_polyline_planner::LocalCostmapOverlayRestoreCell> restore_cells;

  const bool applied = nav2_route_polyline_planner::overlayLocalCostmap(
    local_costmap,
    makeTransform(1.0, 2.0),
    makePose(1.15, 2.15),
    options,
    global_costmap,
    nullptr,
    &restore_cells);

  ASSERT_TRUE(applied);
  ASSERT_EQ(restore_cells.size(), 1U);
  EXPECT_EQ(restore_cells.front().mx, 11U);
  EXPECT_EQ(restore_cells.front().my, 21U);
  EXPECT_EQ(restore_cells.front().old_cost, 20);
  EXPECT_EQ(global_costmap.getCost(11, 21), 100);
  global_costmap.setCost(
    restore_cells.front().mx,
    restore_cells.front().my,
    restore_cells.front().old_cost);
  EXPECT_EQ(global_costmap.getCost(11, 21), 20);
}

TEST(LocalCostmapOverlayTest, RejectsMalformedLocalCostmapWithoutChangingGlobalMap)
{
  auto global_costmap = makeGlobalCostmap();
  auto local_costmap = makeLocalCostmap();
  local_costmap.data.pop_back();

  nav2_route_polyline_planner::LocalCostmapOverlayOptions options;

  const bool applied = nav2_route_polyline_planner::overlayLocalCostmap(
    local_costmap, makeTransform(1.0, 2.0), makePose(1.15, 2.15), options, global_costmap);

  EXPECT_FALSE(applied);
  EXPECT_EQ(global_costmap.getCost(11, 21), nav2_costmap_2d::FREE_SPACE);
}
