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
  double goal_search_radius{1.0};
  double reference_corridor_half_width{2.0};
  bool reference_use_astar{true};
  bool reference_allow_unknown{false};
  double start_goal_keepout_radius{0.35};
  int max_goal_candidates{80};
  double path_interpolation_resolution{0.0};
  bool use_final_goal_orientation{true};
};

struct RouteClearancePlanResult
{
  nav_msgs::msg::Path path;
  bool adjusted_goal{false};
  geometry_msgs::msg::PoseStamped effective_goal;
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

  bool isCostTraversable(unsigned char cost) const;

  bool isCellTraversable(
    const nav2_costmap_2d::Costmap2D & costmap,
    const std::vector<double> & clearance_map,
    unsigned int mx,
    unsigned int my) const;

  std::vector<double> buildClearanceMap(
    const nav2_costmap_2d::Costmap2D & costmap) const;

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

  double turnPenalty(
    const std::vector<int> & parents,
    const GridCell & from,
    const GridCell & to,
    int from_index,
    unsigned int size_x) const;

  nav_msgs::msg::Path buildPath(
    const nav2_costmap_2d::Costmap2D & costmap,
    const std::vector<GridCell> & cells,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & effective_goal,
    const std::string & frame_id) const;

  std::vector<std::pair<double, double>> interpolatePoints(
    const std::vector<std::pair<double, double>> & points,
    double resolution) const;

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

  std::vector<std::pair<double, double>> buildReferencePathPoints(
    const nav2_costmap_2d::Costmap2D & costmap,
    const std::vector<double> & clearance_map,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  PlanningContext buildPlanningContext(
    const nav2_costmap_2d::Costmap2D & costmap,
    const std::vector<std::pair<double, double>> & reference_points,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  nav_msgs::msg::Path buildOptimizedPathFromReference(
    const PlanningContext & planning_context,
    const std::vector<double> & clearance_map,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & effective_goal,
    const std::string & frame_id) const;

  RouteClearancePlannerConfig config_;
};

}  // namespace nav2_route_polyline_planner

#endif  // NAV2_ROUTE_POLYLINE_PLANNER__ROUTE_CLEARANCE_PLANNER_CORE_HPP_
