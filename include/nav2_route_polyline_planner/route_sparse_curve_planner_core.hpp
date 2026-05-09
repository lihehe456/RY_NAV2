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

#ifndef NAV2_ROUTE_POLYLINE_PLANNER__ROUTE_SPARSE_CURVE_PLANNER_CORE_HPP_
#define NAV2_ROUTE_POLYLINE_PLANNER__ROUTE_SPARSE_CURVE_PLANNER_CORE_HPP_

#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_route_polyline_planner
{

struct RouteSparseCurvePlannerConfig
{
  double path_interpolation_resolution{0.0};
  double collision_check_resolution{0.0};
  double straight_angle_threshold_deg{12.0};
  double curve_sample_resolution{0.0};
  double tangent_scale{0.45};
  double max_tangent_length{2.0};
  unsigned int max_curve_cell_cost{180U};
  double fallback_corridor_half_width{1.0};
  bool fallback_use_astar{true};
  bool fallback_allow_unknown{false};
  double start_goal_keepout_radius{0.35};
};

struct RouteSparseCurveSegmentPlanResult
{
  nav_msgs::msg::Path path;
  bool used_curve{false};
  bool used_fallback{false};
};

class RouteSparseCurvePlannerCore
{
public:
  explicit RouteSparseCurvePlannerCore(
    RouteSparseCurvePlannerConfig config = RouteSparseCurvePlannerConfig());

  bool shouldUseStraightMode(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  std::vector<std::pair<double, double>> buildHermiteCurvePoints(
    const nav2_costmap_2d::Costmap2D & costmap,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  std::vector<unsigned char> buildMaskedCostmapDataForPolyline(
    const nav2_costmap_2d::Costmap2D & costmap,
    const std::vector<std::pair<double, double>> & polyline,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  RouteSparseCurveSegmentPlanResult planSegment(
    const nav2_costmap_2d::Costmap2D & costmap,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & frame_id) const;

private:
  bool isCostTraversable(unsigned char cost) const;

  double resolvePathInterpolationResolution(
    const nav2_costmap_2d::Costmap2D & costmap) const;

  double resolveCollisionCheckResolution(
    const nav2_costmap_2d::Costmap2D & costmap) const;

  double resolveCurveSampleResolution(
    const nav2_costmap_2d::Costmap2D & costmap) const;

  std::vector<std::pair<double, double>> buildStraightLinePoints(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double interpolation_resolution) const;

  nav_msgs::msg::Path buildPathFromPoints(
    const std::vector<std::pair<double, double>> & points,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & frame_id) const;

  bool arePointsTraversable(
    const nav2_costmap_2d::Costmap2D & costmap,
    const std::vector<std::pair<double, double>> & points,
    bool enforce_cost_limit) const;

  std::vector<std::pair<double, double>> buildFallbackPathPoints(
    const nav2_costmap_2d::Costmap2D & costmap,
    const std::vector<std::pair<double, double>> & reference_points,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  void appendUniquePoint(
    std::vector<std::pair<double, double>> & points,
    const std::pair<double, double> & point,
    double min_distance) const;

  double pointToSegmentDistance(
    const std::pair<double, double> & point,
    const std::pair<double, double> & segment_start,
    const std::pair<double, double> & segment_end) const;

  double pointToPolylineDistance(
    const std::pair<double, double> & point,
    const std::vector<std::pair<double, double>> & polyline) const;

  RouteSparseCurvePlannerConfig config_;
};

}  // namespace nav2_route_polyline_planner

#endif  // NAV2_ROUTE_POLYLINE_PLANNER__ROUTE_SPARSE_CURVE_PLANNER_CORE_HPP_
