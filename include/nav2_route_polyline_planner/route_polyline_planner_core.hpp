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

#ifndef NAV2_ROUTE_POLYLINE_PLANNER__ROUTE_POLYLINE_PLANNER_CORE_HPP_
#define NAV2_ROUTE_POLYLINE_PLANNER__ROUTE_POLYLINE_PLANNER_CORE_HPP_

#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_route_polyline_planner
{

struct RoutePolylinePlannerConfig
{
  bool allow_unknown{false};
  double path_interpolation_resolution{0.0};
  double collision_check_resolution{0.0};
  double fallback_corridor_half_width{1.5};
  bool fallback_use_astar{true};
  bool fallback_allow_unknown{false};
  double start_goal_keepout_radius{0.35};
};

struct SegmentPlanResult
{
  nav_msgs::msg::Path path;
  bool used_fallback{false};
};

class RoutePolylinePlannerCore
{
public:
  explicit RoutePolylinePlannerCore(
    RoutePolylinePlannerConfig config = RoutePolylinePlannerConfig());

  bool isSegmentCollisionFree(
    const nav2_costmap_2d::Costmap2D & costmap,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  std::vector<unsigned char> buildMaskedCostmapData(
    const nav2_costmap_2d::Costmap2D & costmap,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  nav_msgs::msg::Path buildStraightPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & frame_id) const;

  SegmentPlanResult planSegment(
    const nav2_costmap_2d::Costmap2D & costmap,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & frame_id) const;

private:
  bool isPoseTraversable(
    const nav2_costmap_2d::Costmap2D & costmap,
    const geometry_msgs::msg::PoseStamped & pose,
    bool allow_unknown) const;

  bool isCostTraversable(unsigned char cost, bool allow_unknown) const;

  double resolvePathInterpolationResolution(
    const nav2_costmap_2d::Costmap2D & costmap) const;

  double resolveCollisionCheckResolution(
    const nav2_costmap_2d::Costmap2D & costmap) const;

  std::vector<std::pair<double, double>> buildStraightLinePoints(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double interpolation_resolution) const;

  std::vector<std::pair<double, double>> buildFallbackPathPoints(
    const nav2_costmap_2d::Costmap2D & costmap,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  nav_msgs::msg::Path buildPathFromPoints(
    const std::vector<std::pair<double, double>> & points,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & frame_id) const;

  void appendUniquePoint(
    std::vector<std::pair<double, double>> & points,
    const std::pair<double, double> & point,
    double min_distance) const;

  double pointToSegmentDistance(
    const std::pair<double, double> & point,
    const std::pair<double, double> & segment_start,
    const std::pair<double, double> & segment_end) const;

  RoutePolylinePlannerConfig config_;
};

}  // namespace nav2_route_polyline_planner

#endif  // NAV2_ROUTE_POLYLINE_PLANNER__ROUTE_POLYLINE_PLANNER_CORE_HPP_
