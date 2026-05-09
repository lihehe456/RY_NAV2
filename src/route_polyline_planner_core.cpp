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

#include "nav2_route_polyline_planner/route_polyline_planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_navfn_planner/navfn.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace nav2_route_polyline_planner
{

namespace
{

constexpr double kEpsilon = 1.0e-6;

double euclideanDistance(
  const std::pair<double, double> & point_a,
  const std::pair<double, double> & point_b)
{
  const double dx = point_b.first - point_a.first;
  const double dy = point_b.second - point_a.second;
  return std::hypot(dx, dy);
}

}  // namespace

RoutePolylinePlannerCore::RoutePolylinePlannerCore(RoutePolylinePlannerConfig config)
: config_(std::move(config))
{
}

bool RoutePolylinePlannerCore::isCostTraversable(
  unsigned char cost,
  bool allow_unknown) const
{
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown;
  }
  return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

bool RoutePolylinePlannerCore::isPoseTraversable(
  const nav2_costmap_2d::Costmap2D & costmap,
  const geometry_msgs::msg::PoseStamped & pose,
  bool allow_unknown) const
{
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap.worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
    return false;
  }
  return isCostTraversable(costmap.getCost(mx, my), allow_unknown);
}

double RoutePolylinePlannerCore::resolvePathInterpolationResolution(
  const nav2_costmap_2d::Costmap2D & costmap) const
{
  return config_.path_interpolation_resolution > 0.0 ?
         config_.path_interpolation_resolution : costmap.getResolution();
}

double RoutePolylinePlannerCore::resolveCollisionCheckResolution(
  const nav2_costmap_2d::Costmap2D & costmap) const
{
  return config_.collision_check_resolution > 0.0 ?
         config_.collision_check_resolution : costmap.getResolution();
}

double RoutePolylinePlannerCore::pointToSegmentDistance(
  const std::pair<double, double> & point,
  const std::pair<double, double> & segment_start,
  const std::pair<double, double> & segment_end) const
{
  const double seg_x = segment_end.first - segment_start.first;
  const double seg_y = segment_end.second - segment_start.second;
  const double seg_len_sq = seg_x * seg_x + seg_y * seg_y;
  if (seg_len_sq <= kEpsilon) {
    return euclideanDistance(point, segment_start);
  }

  const double rel_x = point.first - segment_start.first;
  const double rel_y = point.second - segment_start.second;
  const double projection = std::max(
    0.0,
    std::min(1.0, (rel_x * seg_x + rel_y * seg_y) / seg_len_sq));
  const std::pair<double, double> projected_point{
    segment_start.first + projection * seg_x,
    segment_start.second + projection * seg_y};
  return euclideanDistance(point, projected_point);
}

void RoutePolylinePlannerCore::appendUniquePoint(
  std::vector<std::pair<double, double>> & points,
  const std::pair<double, double> & point,
  double min_distance) const
{
  if (points.empty() || euclideanDistance(points.back(), point) > min_distance) {
    points.push_back(point);
  }
}

std::vector<std::pair<double, double>> RoutePolylinePlannerCore::buildStraightLinePoints(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  double interpolation_resolution) const
{
  const std::pair<double, double> start_point{
    start.pose.position.x, start.pose.position.y};
  const std::pair<double, double> goal_point{
    goal.pose.position.x, goal.pose.position.y};
  const double distance = euclideanDistance(start_point, goal_point);
  if (distance <= kEpsilon) {
    return {start_point};
  }

  const double safe_resolution = interpolation_resolution > 0.0 ?
    interpolation_resolution : distance;
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / safe_resolution)));

  std::vector<std::pair<double, double>> points;
  points.reserve(static_cast<size_t>(steps) + 1U);
  for (int index = 0; index <= steps; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(steps);
    points.push_back(
      {
        start_point.first + (goal_point.first - start_point.first) * ratio,
        start_point.second + (goal_point.second - start_point.second) * ratio,
      });
  }
  return points;
}

nav_msgs::msg::Path RoutePolylinePlannerCore::buildPathFromPoints(
  const std::vector<std::pair<double, double>> & points,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & frame_id) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;

  for (const auto & point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  if (path.poses.empty()) {
    return path;
  }

  if (path.poses.size() == 1U) {
    path.poses.front().pose.orientation = goal.pose.orientation;
    return path;
  }

  geometry_msgs::msg::Quaternion carry_orientation = goal.pose.orientation;
  for (int index = static_cast<int>(path.poses.size()) - 2; index >= 0; --index) {
    int next_index = index + 1;
    while (
      next_index < static_cast<int>(path.poses.size()) &&
      euclideanDistance(
        points[static_cast<size_t>(index)],
        points[static_cast<size_t>(next_index)]) <= kEpsilon)
    {
      ++next_index;
    }

    if (next_index < static_cast<int>(path.poses.size())) {
      const double dx =
        points[static_cast<size_t>(next_index)].first -
        points[static_cast<size_t>(index)].first;
      const double dy =
        points[static_cast<size_t>(next_index)].second -
        points[static_cast<size_t>(index)].second;
      carry_orientation = nav2_util::geometry_utils::orientationAroundZAxis(std::atan2(dy, dx));
    }
    path.poses[static_cast<size_t>(index)].pose.orientation = carry_orientation;
  }
  path.poses.back().pose.orientation = goal.pose.orientation;

  return path;
}

bool RoutePolylinePlannerCore::isSegmentCollisionFree(
  const nav2_costmap_2d::Costmap2D & costmap,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  if (
    !isPoseTraversable(costmap, start, config_.allow_unknown) ||
    !isPoseTraversable(costmap, goal, config_.allow_unknown))
  {
    return false;
  }

  const double resolution = resolveCollisionCheckResolution(costmap);
  const auto points = buildStraightLinePoints(start, goal, resolution);
  for (const auto & point : points) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap.worldToMap(point.first, point.second, mx, my)) {
      return false;
    }
    if (!isCostTraversable(costmap.getCost(mx, my), config_.allow_unknown)) {
      return false;
    }
  }

  return true;
}

std::vector<unsigned char> RoutePolylinePlannerCore::buildMaskedCostmapData(
  const nav2_costmap_2d::Costmap2D & costmap,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();
  const unsigned char * source_map = costmap.getCharMap();
  std::vector<unsigned char> masked_map(
    source_map, source_map + static_cast<size_t>(size_x) * static_cast<size_t>(size_y));

  const std::pair<double, double> start_point{
    start.pose.position.x, start.pose.position.y};
  const std::pair<double, double> goal_point{
    goal.pose.position.x, goal.pose.position.y};
  const double keepout_radius = std::max(0.0, config_.start_goal_keepout_radius);
  const double corridor_half_width = std::max(0.0, config_.fallback_corridor_half_width);

  for (unsigned int my = 0; my < size_y; ++my) {
    for (unsigned int mx = 0; mx < size_x; ++mx) {
      double wx = 0.0;
      double wy = 0.0;
      costmap.mapToWorld(mx, my, wx, wy);
      const std::pair<double, double> cell_point{wx, wy};
      const bool within_keepout =
        euclideanDistance(cell_point, start_point) <= keepout_radius ||
        euclideanDistance(cell_point, goal_point) <= keepout_radius;
      const bool within_corridor =
        pointToSegmentDistance(cell_point, start_point, goal_point) <= corridor_half_width;

      if (!within_keepout && !within_corridor) {
        masked_map[costmap.getIndex(mx, my)] = nav2_costmap_2d::LETHAL_OBSTACLE;
      }
    }
  }

  unsigned int start_mx = 0;
  unsigned int start_my = 0;
  if (costmap.worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my)) {
    masked_map[costmap.getIndex(start_mx, start_my)] = nav2_costmap_2d::FREE_SPACE;
  }

  unsigned int goal_mx = 0;
  unsigned int goal_my = 0;
  if (costmap.worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my)) {
    masked_map[costmap.getIndex(goal_mx, goal_my)] = nav2_costmap_2d::FREE_SPACE;
  }

  return masked_map;
}

nav_msgs::msg::Path RoutePolylinePlannerCore::buildStraightPath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & frame_id) const
{
  const double interpolation_resolution =
    config_.path_interpolation_resolution > 0.0 ?
    config_.path_interpolation_resolution : 1.0;
  return buildPathFromPoints(
    buildStraightLinePoints(start, goal, interpolation_resolution),
    goal,
    frame_id);
}

std::vector<std::pair<double, double>> RoutePolylinePlannerCore::buildFallbackPathPoints(
  const nav2_costmap_2d::Costmap2D & costmap,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  unsigned int start_mx = 0;
  unsigned int start_my = 0;
  if (!costmap.worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my)) {
    throw nav2_core::PlannerException("route polyline planner start pose is outside costmap");
  }

  unsigned int goal_mx = 0;
  unsigned int goal_my = 0;
  if (!costmap.worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my)) {
    throw nav2_core::PlannerException("route polyline planner goal pose is outside costmap");
  }

  auto masked_map = buildMaskedCostmapData(costmap, start, goal);
  nav2_navfn_planner::NavFn planner(
    static_cast<int>(costmap.getSizeInCellsX()),
    static_cast<int>(costmap.getSizeInCellsY()));
  planner.setCostmap(masked_map.data(), true, config_.fallback_allow_unknown);

  int map_start[2] = {static_cast<int>(start_mx), static_cast<int>(start_my)};
  int map_goal[2] = {static_cast<int>(goal_mx), static_cast<int>(goal_my)};

  planner.setStart(map_goal);
  planner.setGoal(map_start);
  if (config_.fallback_use_astar) {
    planner.calcNavFnAstar();
  } else {
    planner.calcNavFnDijkstra(true);
  }

  const unsigned int goal_index = goal_my * static_cast<unsigned int>(planner.nx) + goal_mx;
  if (planner.potarr[goal_index] >= POT_HIGH) {
    throw nav2_core::PlannerException(
            "route polyline planner fallback could not find a corridor-bounded path");
  }

  planner.setStart(map_goal);
  const int max_cycles =
    static_cast<int>(std::max(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()) * 4U);
  if (planner.calcPath(max_cycles) == 0 || planner.getPathLen() == 0) {
    throw nav2_core::PlannerException(
            "route polyline planner fallback path extraction failed");
  }

  float * path_x = planner.getPathX();
  float * path_y = planner.getPathY();
  const int path_length = planner.getPathLen();
  const double min_distance = std::max(costmap.getResolution() * 0.25, kEpsilon);

  std::vector<std::pair<double, double>> points;
  appendUniquePoint(
    points,
    {start.pose.position.x, start.pose.position.y},
    min_distance);

  for (int index = path_length - 1; index >= 0; --index) {
    const double wx = costmap.getOriginX() + static_cast<double>(path_x[index]) *
      costmap.getResolution();
    const double wy = costmap.getOriginY() + static_cast<double>(path_y[index]) *
      costmap.getResolution();
    appendUniquePoint(points, {wx, wy}, min_distance);
  }

  const std::pair<double, double> goal_point{
    goal.pose.position.x, goal.pose.position.y};
  if (!points.empty() && euclideanDistance(points.back(), goal_point) <= min_distance) {
    points.back() = goal_point;
  } else {
    points.push_back(goal_point);
  }
  return points;
}

SegmentPlanResult RoutePolylinePlannerCore::planSegment(
  const nav2_costmap_2d::Costmap2D & costmap,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & frame_id) const
{
  SegmentPlanResult result;

  const std::pair<double, double> start_point{
    start.pose.position.x, start.pose.position.y};
  const std::pair<double, double> goal_point{
    goal.pose.position.x, goal.pose.position.y};
  if (euclideanDistance(start_point, goal_point) <= kEpsilon) {
    if (!isPoseTraversable(costmap, start, config_.allow_unknown)) {
      throw nav2_core::PlannerException("route polyline planner start pose is obstructed");
    }
    result.path = buildPathFromPoints({start_point}, goal, frame_id);
    return result;
  }

  if (isSegmentCollisionFree(costmap, start, goal)) {
    result.path = buildPathFromPoints(
      buildStraightLinePoints(start, goal, resolvePathInterpolationResolution(costmap)),
      goal,
      frame_id);
    return result;
  }

  result.used_fallback = true;
  result.path = buildPathFromPoints(buildFallbackPathPoints(costmap, start, goal), goal, frame_id);
  return result;
}

}  // namespace nav2_route_polyline_planner
