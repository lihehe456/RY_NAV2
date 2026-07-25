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

#ifndef NAV2_ROUTE_POLYLINE_PLANNER__ROUTE_NARROW_PLANNER_CORE_HPP_
#define NAV2_ROUTE_POLYLINE_PLANNER__ROUTE_NARROW_PLANNER_CORE_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_route_polyline_planner
{

struct RouteNarrowPlannerConfig
{
  bool allow_unknown{false};
  double hard_min_clearance{0.40};
  double soft_target_clearance{0.65};
  double clearance_weight{12.0};
  double reference_weight{0.10};
  double cost_weight{2.0};
  double turn_weight{2.0};
  double lateral_change_weight{8.0};
  int smoothing_passes{20};
  double corridor_half_width{2.0};
  double start_goal_keepout_radius{0.35};
  double path_interpolation_resolution{0.05};
  double output_path_resolution{0.05};
  bool use_final_goal_orientation{true};
};

class RouteNarrowPlannerCore
{
public:
  explicit RouteNarrowPlannerCore(
    RouteNarrowPlannerConfig config = RouteNarrowPlannerConfig());

  nav_msgs::msg::Path createPlan(
    const nav2_costmap_2d::Costmap2D & costmap,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & frame_id) const;

  double clearanceAt(
    const nav2_costmap_2d::Costmap2D & costmap,
    double wx,
    double wy) const;

private:
  RouteNarrowPlannerConfig config_;
};

}  // namespace nav2_route_polyline_planner

#endif  // NAV2_ROUTE_POLYLINE_PLANNER__ROUTE_NARROW_PLANNER_CORE_HPP_
