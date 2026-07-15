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

#ifndef NAV2_ROUTE_POLYLINE_PLANNER__ROUTE_CLEARANCE_PLANNER_CORE_HPP_
#define NAV2_ROUTE_POLYLINE_PLANNER__ROUTE_CLEARANCE_PLANNER_CORE_HPP_

#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_navfn_planner/navfn.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_route_polyline_planner
{

struct RouteClearancePlannerConfig
{
  bool allow_unknown{false};
  double hard_min_clearance{0.25};
  double soft_target_clearance{0.50};
  double clearance_weight{3.0};
  double centerline_weight{0.0};
  double cost_weight{1.0};
  double turn_weight{0.15};
  double lateral_change_weight{1.5};
  int lateral_smoothing_passes{0};
  bool right_side_bias{false};
  double right_side_weight{0.0};
  double right_side_target_clearance{0.70};
  double right_side_probe_distance{2.0};
  double right_side_max_offset{0.8};
  bool pose_directed_crop_enabled{false};
  double goal_approach_length{1.0};
  double pose_directed_max_corridor_half_width{0.0};
  double goal_search_radius{1.0};
  double reference_corridor_half_width{2.0};
  bool reference_use_astar{true};
  bool reference_allow_unknown{false};
  double start_goal_keepout_radius{0.35};
  int max_goal_candidates{80};
  double path_interpolation_resolution{0.0};
  double output_path_resolution{0.0};
  bool use_final_goal_orientation{true};
};

struct RouteClearancePlanResult
{
  nav_msgs::msg::Path path;
  bool adjusted_goal{false};
  geometry_msgs::msg::PoseStamped effective_goal;
};

struct ClearanceCacheStats
{
  size_t builds{0};
  size_t hits{0};
  double last_build_ms{0.0};
};

class RouteClearancePlannerCore
{
public:
  explicit RouteClearancePlannerCore(
    RouteClearancePlannerConfig config = RouteClearancePlannerConfig());

  RouteClearancePlanResult createPlan(
    const nav2_costmap_2d::Costmap2D & costmap,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & frame_id) const;

  double clearanceAt(
    const nav2_costmap_2d::Costmap2D & costmap,
    double wx,
    double wy) const;

  ClearanceCacheStats clearanceCacheStats() const;

private:
  struct GridCell
  {
    unsigned int mx{0};
    unsigned int my{0};
  };

  struct SearchResult
  {
    bool found{false};
    std::vector<GridCell> cells;
  };

  struct PlanningContext
  {
    nav2_costmap_2d::Costmap2D costmap;
    GridCell start;
    GridCell goal;
    std::vector<std::pair<double, double>> reference_points;
  };

  struct ClearanceCacheKey
  {
    unsigned int size_x{0};
    unsigned int size_y{0};
    double resolution{0.0};
    double origin_x{0.0};
    double origin_y{0.0};
    bool allow_unknown{false};
    double hard_min_clearance{0.0};
    double soft_target_clearance{0.0};
    size_t hash{0};
  };

  struct ClearanceCache
  {
    bool valid{false};
    ClearanceCacheKey key;
    std::vector<double> map;
  };

  bool isCostTraversable(unsigned char cost) const;

  bool isCellTraversable(
    const nav2_costmap_2d::Costmap2D & costmap,
    const std::vector<double> & clearance_map,
    unsigned int mx,
    unsigned int my) const;

  bool isCellTraversableWithMinClearance(
    const nav2_costmap_2d::Costmap2D & costmap,
    const std::vector<double> & clearance_map,
    unsigned int mx,
    unsigned int my,
    double min_clearance) const;

  std::vector<double> buildClearanceMap(
    const nav2_costmap_2d::Costmap2D & costmap) const;

  std::vector<double> buildInflationBoundaryClearanceMap(
    const nav2_costmap_2d::Costmap2D & costmap) const;

  const std::vector<double> & getGlobalClearanceMap(
    const nav2_costmap_2d::Costmap2D & costmap,
    double & lookup_ms) const;

  ClearanceCacheKey makeClearanceCacheKey(
    const nav2_costmap_2d::Costmap2D & costmap) const;

  bool sameClearanceCacheKey(
    const ClearanceCacheKey & lhs,
    const ClearanceCacheKey & rhs) const;

  std::vector<GridCell> buildGoalCandidates(
    const nav2_costmap_2d::Costmap2D & costmap,
    const std::vector<double> & clearance_map,
    unsigned int goal_mx,
    unsigned int goal_my) const;

  SearchResult searchPath(
    const nav2_costmap_2d::Costmap2D & costmap,
    const std::vector<double> & clearance_map,
    const GridCell & start,
    const GridCell & goal) const;

  bool hasLineOfSight(
    const nav2_costmap_2d::Costmap2D & costmap,
    const std::vector<double> & clearance_map,
    const GridCell & start,
    const GridCell & goal) const;

  double lineTraversalCost(
    const nav2_costmap_2d::Costmap2D & costmap,
    const std::vector<double> & clearance_map,
    const GridCell & start,
    const GridCell & goal) const;

  double cellTraversalPenalty(
    const nav2_costmap_2d::Costmap2D & costmap,
    const std::vector<double> & clearance_map,
    const GridCell & cell) const;

  nav_msgs::msg::Path biasPathToRightIfSafe(
    const nav_msgs::msg::Path & path,
    const nav2_costmap_2d::Costmap2D & costmap,
    const std::vector<double> & clearance_map) const;

  double turnPenalty(
    const std::vector<int> & parents,
    const GridCell & from,
    const GridCell & to,
    int from_index,
    unsigned int size_x) const;

  nav_msgs::msg::Path buildPath(
    const nav2_costmap_2d::Costmap2D & costmap,
    const std::vector<double> & clearance_map,
    const std::vector<GridCell> & cells,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & effective_goal,
    const std::string & frame_id) const;

  std::vector<std::pair<double, double>> interpolatePoints(
    const std::vector<std::pair<double, double>> & points,
    double resolution) const;

  nav_msgs::msg::Path downsampleOutputPath(
    const nav_msgs::msg::Path & dense_path,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & effective_goal) const;

  bool worldToMapChecked(
    const nav2_costmap_2d::Costmap2D & costmap,
    const geometry_msgs::msg::PoseStamped & pose,
    unsigned int & mx,
    unsigned int & my,
    const char * label) const;

  double resolveInterpolationResolution(
    const nav2_costmap_2d::Costmap2D & costmap) const;

  bool isSegmentClearanceSafe(
    const nav2_costmap_2d::Costmap2D & costmap,
    const std::vector<double> & clearance_map,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  double pointToSegmentDistance(
    const std::pair<double, double> & point,
    const std::pair<double, double> & segment_start,
    const std::pair<double, double> & segment_end) const;

  double pointToPolylineDistance(
    const std::pair<double, double> & point,
    const std::vector<std::pair<double, double>> & polyline) const;

  void appendUniquePoint(
    std::vector<std::pair<double, double>> & points,
    const std::pair<double, double> & point,
    double min_distance) const;

  std::vector<std::pair<double, double>> buildPoseDirectedReferencePoints(
    const nav2_costmap_2d::Costmap2D & costmap,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  std::vector<std::pair<double, double>> buildReferencePathPoints(
    const nav2_costmap_2d::Costmap2D & costmap,
    const std::vector<double> & clearance_map,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  PlanningContext buildPlanningContext(
    const nav2_costmap_2d::Costmap2D & costmap,
    const std::vector<std::pair<double, double>> & reference_points,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double corridor_half_width_override = -1.0) const;

  nav_msgs::msg::Path buildOptimizedPathFromReference(
    const PlanningContext & planning_context,
    const std::vector<double> & clearance_map,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & effective_goal,
    const std::string & frame_id) const;

  RouteClearancePlannerConfig config_;
  mutable ClearanceCache clearance_cache_;
  mutable ClearanceCacheStats clearance_cache_stats_;
};

}  // namespace nav2_route_polyline_planner

#endif  // NAV2_ROUTE_POLYLINE_PLANNER__ROUTE_CLEARANCE_PLANNER_CORE_HPP_
