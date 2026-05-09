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

#include "nav2_route_polyline_planner/route_sparse_curve_planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & orientation)
{
  const double siny_cosp =
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cosy_cosp =
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

RouteSparseCurvePlannerCore::RouteSparseCurvePlannerCore(RouteSparseCurvePlannerConfig config)
: config_(std::move(config))
{
}

bool RouteSparseCurvePlannerCore::isCostTraversable(unsigned char cost) const
{
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return false;
  }
  return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

double RouteSparseCurvePlannerCore::resolvePathInterpolationResolution(
  const nav2_costmap_2d::Costmap2D & costmap) const
{
  return config_.path_interpolation_resolution > 0.0 ?
         config_.path_interpolation_resolution : costmap.getResolution();
}

double RouteSparseCurvePlannerCore::resolveCollisionCheckResolution(
  const nav2_costmap_2d::Costmap2D & costmap) const
{
  return config_.collision_check_resolution > 0.0 ?
         config_.collision_check_resolution : costmap.getResolution();
}

double RouteSparseCurvePlannerCore::resolveCurveSampleResolution(
  const nav2_costmap_2d::Costmap2D & costmap) const
{
  return config_.curve_sample_resolution > 0.0 ?
         config_.curve_sample_resolution : costmap.getResolution();
}

void RouteSparseCurvePlannerCore::appendUniquePoint(
  std::vector<std::pair<double, double>> & points,
  const std::pair<double, double> & point,
  double min_distance) const
{
  if (points.empty() || euclideanDistance(points.back(), point) > min_distance) {
    points.push_back(point);
  }
}

double RouteSparseCurvePlannerCore::pointToSegmentDistance(
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

double RouteSparseCurvePlannerCore::pointToPolylineDistance(
  const std::pair<double, double> & point,
  const std::vector<std::pair<double, double>> & polyline) const
{
  if (polyline.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  if (polyline.size() == 1U) {
    return euclideanDistance(point, polyline.front());
  }

  double min_distance = std::numeric_limits<double>::infinity();
  for (size_t index = 1; index < polyline.size(); ++index) {
    min_distance = std::min(
      min_distance,
      pointToSegmentDistance(point, polyline[index - 1], polyline[index]));
  }
  return min_distance;
}

bool RouteSparseCurvePlannerCore::shouldUseStraightMode(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  const std::pair<double, double> start_point{
    start.pose.position.x, start.pose.position.y};
  const std::pair<double, double> goal_point{
    goal.pose.position.x, goal.pose.position.y};
  if (euclideanDistance(start_point, goal_point) <= kEpsilon) {
    return true;
  }

  const double chord_yaw = std::atan2(
    goal.pose.position.y - start.pose.position.y,
    goal.pose.position.x - start.pose.position.x);
  const double threshold = config_.straight_angle_threshold_deg * M_PI / 180.0;
  const double start_delta =
    std::abs(normalizeAngle(yawFromQuaternion(start.pose.orientation) - chord_yaw));
  const double goal_delta =
    std::abs(normalizeAngle(yawFromQuaternion(goal.pose.orientation) - chord_yaw));
  return start_delta <= threshold && goal_delta <= threshold;
}

std::vector<std::pair<double, double>> RouteSparseCurvePlannerCore::buildStraightLinePoints(
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

std::vector<std::pair<double, double>> RouteSparseCurvePlannerCore::buildHermiteCurvePoints(
  const nav2_costmap_2d::Costmap2D & costmap,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  const std::pair<double, double> start_point{
    start.pose.position.x, start.pose.position.y};
  const std::pair<double, double> goal_point{
    goal.pose.position.x, goal.pose.position.y};
  const double chord_length = euclideanDistance(start_point, goal_point);
  if (chord_length <= kEpsilon) {
    return {start_point};
  }

  const double sample_resolution = resolveCurveSampleResolution(costmap);
  const int steps = std::max(2, static_cast<int>(std::ceil(chord_length / sample_resolution)));
  const double tangent_length = std::max(
    0.0,
    std::min(chord_length * config_.tangent_scale, config_.max_tangent_length));
  const double start_yaw = yawFromQuaternion(start.pose.orientation);
  const double goal_yaw = yawFromQuaternion(goal.pose.orientation);
  const std::pair<double, double> start_tangent{
    tangent_length * std::cos(start_yaw),
    tangent_length * std::sin(start_yaw)};
  const std::pair<double, double> goal_tangent{
    tangent_length * std::cos(goal_yaw),
    tangent_length * std::sin(goal_yaw)};

  std::vector<std::pair<double, double>> points;
  points.reserve(static_cast<size_t>(steps) + 1U);
  for (int index = 0; index <= steps; ++index) {
    const double t = static_cast<double>(index) / static_cast<double>(steps);
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    const double h10 = t3 - 2.0 * t2 + t;
    const double h01 = -2.0 * t3 + 3.0 * t2;
    const double h11 = t3 - t2;
    points.push_back(
      {
        h00 * start_point.first + h10 * start_tangent.first +
        h01 * goal_point.first + h11 * goal_tangent.first,
        h00 * start_point.second + h10 * start_tangent.second +
        h01 * goal_point.second + h11 * goal_tangent.second,
      });
  }

  return points;
}

nav_msgs::msg::Path RouteSparseCurvePlannerCore::buildPathFromPoints(
  const std::vector<std::pair<double, double>> & points,
  const geometry_msgs::msg::PoseStamped & start,
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

  for (size_t index = 0; index + 1 < path.poses.size(); ++index) {
    size_t next_index = index + 1;
    while (
      next_index < path.poses.size() &&
      euclideanDistance(points[index], points[next_index]) <= kEpsilon)
    {
      ++next_index;
    }

    if (next_index >= path.poses.size()) {
      break;
    }

    const double dx = points[next_index].first - points[index].first;
    const double dy = points[next_index].second - points[index].second;
    path.poses[index].pose.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(std::atan2(dy, dx));
  }

  path.poses.front().pose.orientation = start.pose.orientation;
  path.poses.back().pose.orientation = goal.pose.orientation;
  return path;
}

bool RouteSparseCurvePlannerCore::arePointsTraversable(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<std::pair<double, double>> & points,
  bool enforce_cost_limit) const
{
  for (const auto & point : points) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap.worldToMap(point.first, point.second, mx, my)) {
      return false;
    }

    const unsigned char cost = costmap.getCost(mx, my);
    if (!isCostTraversable(cost)) {
      return false;
    }
    if (enforce_cost_limit && cost > config_.max_curve_cell_cost) {
      return false;
    }
  }

  return true;
}

std::vector<unsigned char> RouteSparseCurvePlannerCore::buildMaskedCostmapDataForPolyline(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<std::pair<double, double>> & polyline,
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
        pointToPolylineDistance(cell_point, polyline) <= corridor_half_width;

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

std::vector<std::pair<double, double>> RouteSparseCurvePlannerCore::buildFallbackPathPoints(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<std::pair<double, double>> & reference_points,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  unsigned int start_mx = 0;
  unsigned int start_my = 0;
  if (!costmap.worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my)) {
    throw nav2_core::PlannerException("route sparse curve planner start pose is outside costmap");
  }

  unsigned int goal_mx = 0;
  unsigned int goal_my = 0;
  if (!costmap.worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my)) {
    throw nav2_core::PlannerException("route sparse curve planner goal pose is outside costmap");
  }

  auto masked_map = buildMaskedCostmapDataForPolyline(costmap, reference_points, start, goal);
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
            "route sparse curve planner fallback could not find a curve-bounded path");
  }

  planner.setStart(map_goal);
  const int max_cycles =
    static_cast<int>(std::max(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()) * 4U);
  if (planner.calcPath(max_cycles) == 0 || planner.getPathLen() == 0) {
    throw nav2_core::PlannerException(
            "route sparse curve planner fallback path extraction failed");
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

RouteSparseCurveSegmentPlanResult RouteSparseCurvePlannerCore::planSegment(
  const nav2_costmap_2d::Costmap2D & costmap,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & frame_id) const
{
  RouteSparseCurveSegmentPlanResult result;

  const std::pair<double, double> start_point{
    start.pose.position.x, start.pose.position.y};
  const std::pair<double, double> goal_point{
    goal.pose.position.x, goal.pose.position.y};
  if (euclideanDistance(start_point, goal_point) <= kEpsilon) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (
      !costmap.worldToMap(start.pose.position.x, start.pose.position.y, mx, my) ||
      !isCostTraversable(costmap.getCost(mx, my)))
    {
      throw nav2_core::PlannerException("route sparse curve planner start pose is obstructed");
    }
    result.path = buildPathFromPoints({start_point}, start, goal, frame_id);
    return result;
  }

  const bool use_straight_mode = shouldUseStraightMode(start, goal);
  result.used_curve = !use_straight_mode;

  const auto output_points = use_straight_mode ?
    buildStraightLinePoints(start, goal, resolvePathInterpolationResolution(costmap)) :
    buildHermiteCurvePoints(costmap, start, goal);
  const auto reference_points = use_straight_mode ?
    buildStraightLinePoints(start, goal, resolveCollisionCheckResolution(costmap)) :
    output_points;
  const bool reference_feasible = arePointsTraversable(
    costmap,
    reference_points,
    result.used_curve);

  if (reference_feasible) {
    result.path = buildPathFromPoints(output_points, start, goal, frame_id);
    return result;
  }

  result.used_fallback = true;
  result.path = buildPathFromPoints(
    buildFallbackPathPoints(costmap, reference_points, start, goal),
    start,
    goal,
    frame_id);
  return result;
}

}  // namespace nav2_route_polyline_planner
