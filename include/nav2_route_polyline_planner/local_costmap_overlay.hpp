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

#ifndef NAV2_ROUTE_POLYLINE_PLANNER__LOCAL_COSTMAP_OVERLAY_HPP_
#define NAV2_ROUTE_POLYLINE_PLANNER__LOCAL_COSTMAP_OVERLAY_HPP_

#include <cstddef>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/msg/costmap.hpp"

namespace nav2_route_polyline_planner
{

struct LocalCostmapOverlayOptions
{
  double influence_distance{3.0};
  unsigned char min_overlay_cost{1};
  unsigned char lethal_threshold{nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE};
};

struct LocalCostmapOverlayStats
{
  size_t considered_cells{0};
  size_t overlaid_cells{0};
  size_t skipped_cells{0};
};

struct LocalCostmapOverlayRestoreCell
{
  unsigned int mx{0};
  unsigned int my{0};
  unsigned char old_cost{nav2_costmap_2d::FREE_SPACE};
};

bool overlayLocalCostmap(
  const nav2_msgs::msg::Costmap & local_costmap,
  const geometry_msgs::msg::TransformStamped & local_to_global,
  const geometry_msgs::msg::PoseStamped & start,
  const LocalCostmapOverlayOptions & options,
  nav2_costmap_2d::Costmap2D & global_costmap,
  LocalCostmapOverlayStats * stats = nullptr,
  std::vector<LocalCostmapOverlayRestoreCell> * restore_cells = nullptr);

}  // namespace nav2_route_polyline_planner

#endif  // NAV2_ROUTE_POLYLINE_PLANNER__LOCAL_COSTMAP_OVERLAY_HPP_
