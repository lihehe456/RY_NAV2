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

#include "nav2_route_polyline_planner/route_clearance_planner_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <string>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
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
  return std::hypot(point_b.first - point_a.first, point_b.second - point_a.second);
}

double gridDistance(
  unsigned int ax,
  unsigned int ay,
  unsigned int bx,
  unsigned int by)
{
  return std::hypot(
    static_cast<double>(bx) - static_cast<double>(ax),
    static_cast<double>(by) - static_cast<double>(ay));
}

int cellIndex(unsigned int mx, unsigned int my, unsigned int size_x)
{
  return static_cast<int>(my * size_x + mx);
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & orientation)
{
  const double siny_cosp =
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cosy_cosp =
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::pair<double, double> rightVectorFromTangent(double tangent_x, double tangent_y)
{
  const double length = std::hypot(tangent_x, tangent_y);
  if (length <= kEpsilon) {
    return {0.0, 0.0};
  }

  return {tangent_y / length, -tangent_x / length};
}

struct QueueEntry
{
  int index{0};
  double f{0.0};
};

struct QueueCompare
{
  bool operator()(const QueueEntry & a, const QueueEntry & b) const
  {
    return a.f > b.f;
  }
};

}  // namespace

RouteClearancePlannerCore::RouteClearancePlannerCore(RouteClearancePlannerConfig config)
: config_(std::move(config))
{
}

bool RouteClearancePlannerCore::isCostTraversable(unsigned char cost) const
{
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return config_.allow_unknown;
  }
  return cost < nav2_costmap_2d::LETHAL_OBSTACLE;
}

ClearanceCacheStats RouteClearancePlannerCore::clearanceCacheStats() const
{
  return clearance_cache_stats_;
}

double RouteClearancePlannerCore::resolveInterpolationResolution(
  const nav2_costmap_2d::Costmap2D & costmap) const
{
  return config_.path_interpolation_resolution > 0.0 ?
         config_.path_interpolation_resolution : costmap.getResolution();
}

bool RouteClearancePlannerCore::worldToMapChecked(
  const nav2_costmap_2d::Costmap2D & costmap,
  const geometry_msgs::msg::PoseStamped & pose,
  unsigned int & mx,
  unsigned int & my,
  const char * label) const
{
  if (costmap.worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
    return true;
  }
  throw nav2_core::PlannerException(
          std::string("route clearance planner ") + label + " pose is outside costmap");
}

std::vector<double> RouteClearancePlannerCore::buildClearanceMap(
  const nav2_costmap_2d::Costmap2D & costmap) const
{
  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();
  const size_t size = static_cast<size_t>(size_x) * static_cast<size_t>(size_y);
  const double inf = std::numeric_limits<double>::infinity();
  const double max_clearance_for_cost =
    std::max(0.0, std::max(config_.hard_min_clearance, config_.soft_target_clearance));
  const double max_distance_cells =
    max_clearance_for_cost > kEpsilon ?
    std::ceil(max_clearance_for_cost / costmap.getResolution()) + std::sqrt(2.0) :
    0.0;
  std::vector<double> distance_cells(size, inf);
  std::queue<GridCell> queue;
  const unsigned char * char_map = costmap.getCharMap();

  for (unsigned int my = 0; my < size_y; ++my) {
    for (unsigned int mx = 0; mx < size_x; ++mx) {
      const unsigned char cost =
        char_map[static_cast<size_t>(cellIndex(mx, my, size_x))];
      const bool obstacle =
        cost == nav2_costmap_2d::NO_INFORMATION ? !config_.allow_unknown :
        cost >= nav2_costmap_2d::LETHAL_OBSTACLE;
      if (obstacle) {
        distance_cells[static_cast<size_t>(cellIndex(mx, my, size_x))] = 0.0;
        queue.push({mx, my});
      }
    }
  }

  if (queue.empty()) {
    std::fill(distance_cells.begin(), distance_cells.end(), inf);
    return distance_cells;
  }

  const int neighbor_dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int neighbor_dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  while (!queue.empty()) {
    const GridCell current = queue.front();
    queue.pop();
    const double current_distance =
      distance_cells[static_cast<size_t>(cellIndex(current.mx, current.my, size_x))];
    for (int neighbor = 0; neighbor < 8; ++neighbor) {
      const int nx = static_cast<int>(current.mx) + neighbor_dx[neighbor];
      const int ny = static_cast<int>(current.my) + neighbor_dy[neighbor];
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(size_x) || ny >= static_cast<int>(size_y)) {
        continue;
      }
      const double step = neighbor < 4 ? 1.0 : std::sqrt(2.0);
      const double next_distance = current_distance + step;
      if (max_distance_cells > kEpsilon && next_distance > max_distance_cells) {
        continue;
      }
      const int next_index = cellIndex(
        static_cast<unsigned int>(nx),
        static_cast<unsigned int>(ny),
        size_x);
      if (next_distance + kEpsilon < distance_cells[static_cast<size_t>(next_index)]) {
        distance_cells[static_cast<size_t>(next_index)] = next_distance;
        queue.push({static_cast<unsigned int>(nx), static_cast<unsigned int>(ny)});
      }
    }
  }

  for (auto & distance : distance_cells) {
    if (std::isfinite(distance)) {
      distance *= costmap.getResolution();
    }
  }
  return distance_cells;
}

std::vector<double> RouteClearancePlannerCore::buildInflationBoundaryClearanceMap(
  const nav2_costmap_2d::Costmap2D & costmap) const
{
  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();
  const size_t size = static_cast<size_t>(size_x) * static_cast<size_t>(size_y);
  const double inf = std::numeric_limits<double>::infinity();
  const double max_clearance_for_cost = std::max(
    0.0,
    std::max(config_.right_side_target_clearance, config_.soft_target_clearance));
  const double max_distance_cells =
    max_clearance_for_cost > kEpsilon ?
    std::ceil(max_clearance_for_cost / costmap.getResolution()) + std::sqrt(2.0) :
    0.0;
  std::vector<double> distance_cells(size, inf);
  std::queue<GridCell> queue;
  const unsigned char * char_map = costmap.getCharMap();

  for (unsigned int my = 0; my < size_y; ++my) {
    for (unsigned int mx = 0; mx < size_x; ++mx) {
      const unsigned char cost = char_map[static_cast<size_t>(cellIndex(mx, my, size_x))];
      const bool boundary = cost == nav2_costmap_2d::NO_INFORMATION ?
        !config_.allow_unknown : cost != nav2_costmap_2d::FREE_SPACE;
      if (boundary) {
        distance_cells[static_cast<size_t>(cellIndex(mx, my, size_x))] = 0.0;
        queue.push({mx, my});
      }
    }
  }

  if (queue.empty()) {
    return distance_cells;
  }

  const int neighbor_dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int neighbor_dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  while (!queue.empty()) {
    const GridCell current = queue.front();
    queue.pop();
    const double current_distance =
      distance_cells[static_cast<size_t>(cellIndex(current.mx, current.my, size_x))];
    for (int neighbor = 0; neighbor < 8; ++neighbor) {
      const int nx = static_cast<int>(current.mx) + neighbor_dx[neighbor];
      const int ny = static_cast<int>(current.my) + neighbor_dy[neighbor];
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(size_x) || ny >= static_cast<int>(size_y)) {
        continue;
      }
      const double step = neighbor < 4 ? 1.0 : std::sqrt(2.0);
      const double next_distance = current_distance + step;
      if (max_distance_cells > kEpsilon && next_distance > max_distance_cells) {
        continue;
      }
      const int next_index = cellIndex(
        static_cast<unsigned int>(nx),
        static_cast<unsigned int>(ny),
        size_x);
      if (next_distance + kEpsilon < distance_cells[static_cast<size_t>(next_index)]) {
        distance_cells[static_cast<size_t>(next_index)] = next_distance;
        queue.push({static_cast<unsigned int>(nx), static_cast<unsigned int>(ny)});
      }
    }
  }

  for (auto & distance : distance_cells) {
    if (std::isfinite(distance)) {
      distance *= costmap.getResolution();
    }
  }
  return distance_cells;
}

RouteClearancePlannerCore::ClearanceCacheKey
RouteClearancePlannerCore::makeClearanceCacheKey(
  const nav2_costmap_2d::Costmap2D & costmap) const
{
  ClearanceCacheKey key;
  key.size_x = costmap.getSizeInCellsX();
  key.size_y = costmap.getSizeInCellsY();
  key.resolution = costmap.getResolution();
  key.origin_x = costmap.getOriginX();
  key.origin_y = costmap.getOriginY();
  key.allow_unknown = config_.allow_unknown;
  key.hard_min_clearance = config_.hard_min_clearance;
  key.soft_target_clearance = config_.soft_target_clearance;

  const size_t size =
    static_cast<size_t>(key.size_x) * static_cast<size_t>(key.size_y);
  const unsigned char * char_map = costmap.getCharMap();
  size_t hash = 1469598103934665603ULL;
  for (size_t index = 0; index < size; ++index) {
    hash ^= static_cast<size_t>(char_map[index]);
    hash *= 1099511628211ULL;
  }
  key.hash = hash;
  return key;
}

bool RouteClearancePlannerCore::sameClearanceCacheKey(
  const ClearanceCacheKey & lhs,
  const ClearanceCacheKey & rhs) const
{
  return lhs.size_x == rhs.size_x &&
         lhs.size_y == rhs.size_y &&
         std::abs(lhs.resolution - rhs.resolution) <= kEpsilon &&
         std::abs(lhs.origin_x - rhs.origin_x) <= kEpsilon &&
         std::abs(lhs.origin_y - rhs.origin_y) <= kEpsilon &&
         lhs.allow_unknown == rhs.allow_unknown &&
         std::abs(lhs.hard_min_clearance - rhs.hard_min_clearance) <= kEpsilon &&
         std::abs(lhs.soft_target_clearance - rhs.soft_target_clearance) <= kEpsilon &&
         lhs.hash == rhs.hash;
}

const std::vector<double> & RouteClearancePlannerCore::getGlobalClearanceMap(
  const nav2_costmap_2d::Costmap2D & costmap,
  double & lookup_ms) const
{
  using SteadyClock = std::chrono::steady_clock;
  const auto lookup_start = SteadyClock::now();
  auto elapsed_ms = [](const SteadyClock::time_point & begin) -> double {
      return std::chrono::duration<double, std::milli>(SteadyClock::now() - begin).count();
    };

  const auto key = makeClearanceCacheKey(costmap);
  if (clearance_cache_.valid && sameClearanceCacheKey(clearance_cache_.key, key)) {
    ++clearance_cache_stats_.hits;
    lookup_ms = elapsed_ms(lookup_start);
    return clearance_cache_.map;
  }

  const auto build_start = SteadyClock::now();
  clearance_cache_.map = buildClearanceMap(costmap);
  clearance_cache_.key = key;
  clearance_cache_.valid = true;
  ++clearance_cache_stats_.builds;
  clearance_cache_stats_.last_build_ms = elapsed_ms(build_start);
  lookup_ms = elapsed_ms(lookup_start);
  return clearance_cache_.map;
}

double RouteClearancePlannerCore::clearanceAt(
  const nav2_costmap_2d::Costmap2D & costmap,
  double wx,
  double wy) const
{
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap.worldToMap(wx, wy, mx, my)) {
    return 0.0;
  }
  double global_clearance_lookup_ms = 0.0;
  const auto & clearance_map = getGlobalClearanceMap(costmap, global_clearance_lookup_ms);
  (void) global_clearance_lookup_ms;
  return clearance_map[static_cast<size_t>(cellIndex(mx, my, costmap.getSizeInCellsX()))];
}

bool RouteClearancePlannerCore::isCellTraversable(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<double> & clearance_map,
  unsigned int mx,
  unsigned int my) const
{
  return isCellTraversableWithMinClearance(
    costmap,
    clearance_map,
    mx,
    my,
    config_.hard_min_clearance);
}

bool RouteClearancePlannerCore::isCellTraversableWithMinClearance(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<double> & clearance_map,
  unsigned int mx,
  unsigned int my,
  double min_clearance) const
{
  if (!isCostTraversable(costmap.getCost(mx, my))) {
    return false;
  }
  const double clearance =
    clearance_map[static_cast<size_t>(cellIndex(mx, my, costmap.getSizeInCellsX()))];
  return clearance + kEpsilon >= std::max(0.0, min_clearance);
}

double RouteClearancePlannerCore::pointToSegmentDistance(
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

double RouteClearancePlannerCore::pointToPolylineDistance(
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

void RouteClearancePlannerCore::appendUniquePoint(
  std::vector<std::pair<double, double>> & points,
  const std::pair<double, double> & point,
  double min_distance) const
{
  if (points.empty() || euclideanDistance(points.back(), point) > min_distance) {
    points.push_back(point);
  }
}

std::vector<std::pair<double, double>> RouteClearancePlannerCore::buildPoseDirectedReferencePoints(
  const nav2_costmap_2d::Costmap2D & costmap,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  const std::pair<double, double> start_point{
    start.pose.position.x, start.pose.position.y};
  const std::pair<double, double> goal_point{
    goal.pose.position.x, goal.pose.position.y};
  const double distance = euclideanDistance(start_point, goal_point);
  const double min_distance = std::max(costmap.getResolution() * 2.0, 0.10);

  std::vector<std::pair<double, double>> points;
  points.reserve(3U);
  appendUniquePoint(points, start_point, min_distance);

  if (distance > min_distance) {
    double approach_length = std::max(0.0, config_.goal_approach_length);
    if (approach_length <= kEpsilon) {
      approach_length = std::min(1.0, distance * 0.25);
    }
    approach_length = std::min(approach_length, std::max(0.0, distance - min_distance));

    if (approach_length > min_distance) {
      const double goal_yaw = yawFromQuaternion(goal.pose.orientation);
      const std::pair<double, double> pre_goal{
        goal_point.first - std::cos(goal_yaw) * approach_length,
        goal_point.second - std::sin(goal_yaw) * approach_length};
      appendUniquePoint(points, pre_goal, min_distance);
    }
  }

  appendUniquePoint(points, goal_point, min_distance);
  if (points.back() != goal_point) {
    points.push_back(goal_point);
  }
  return points;
}

bool RouteClearancePlannerCore::isSegmentClearanceSafe(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<double> & clearance_map,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  const double resolution = resolveInterpolationResolution(costmap);
  const std::pair<double, double> start_point{
    start.pose.position.x, start.pose.position.y};
  const std::pair<double, double> goal_point{
    goal.pose.position.x, goal.pose.position.y};
  const double distance = euclideanDistance(start_point, goal_point);
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / resolution)));

  for (int index = 0; index <= steps; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(steps);
    const double wx = start_point.first + (goal_point.first - start_point.first) * ratio;
    const double wy = start_point.second + (goal_point.second - start_point.second) * ratio;
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap.worldToMap(wx, wy, mx, my)) {
      return false;
    }
    if (!isCellTraversable(costmap, clearance_map, mx, my)) {
      return false;
    }
  }
  return true;
}

std::vector<std::pair<double, double>> RouteClearancePlannerCore::buildReferencePathPoints(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<double> & clearance_map,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  (void)clearance_map;
  const std::pair<double, double> start_point{
    start.pose.position.x, start.pose.position.y};
  const std::pair<double, double> goal_point{
    goal.pose.position.x, goal.pose.position.y};
  auto coarsen_costmap =
    [&](const nav2_costmap_2d::Costmap2D & source) -> nav2_costmap_2d::Costmap2D {
      const int scale = std::max(
        1,
        static_cast<int>(std::ceil(0.15 / std::max(source.getResolution(), kEpsilon))));
      if (
        scale <= 1 ||
        static_cast<size_t>(source.getSizeInCellsX()) *
        static_cast<size_t>(source.getSizeInCellsY()) < 1000000U)
      {
        return source;
      }

      const unsigned int coarse_size_x =
        (source.getSizeInCellsX() + static_cast<unsigned int>(scale) - 1U) /
        static_cast<unsigned int>(scale);
      const unsigned int coarse_size_y =
        (source.getSizeInCellsY() + static_cast<unsigned int>(scale) - 1U) /
        static_cast<unsigned int>(scale);
      nav2_costmap_2d::Costmap2D coarse_costmap(
        coarse_size_x,
        coarse_size_y,
        source.getResolution() * static_cast<double>(scale),
        source.getOriginX(),
        source.getOriginY(),
        nav2_costmap_2d::FREE_SPACE);

      for (unsigned int cy = 0; cy < coarse_size_y; ++cy) {
        for (unsigned int cx = 0; cx < coarse_size_x; ++cx) {
          unsigned char max_cost = nav2_costmap_2d::FREE_SPACE;
          const unsigned int begin_x = cx * static_cast<unsigned int>(scale);
          const unsigned int begin_y = cy * static_cast<unsigned int>(scale);
          const unsigned int end_x = std::min(
            source.getSizeInCellsX(),
            begin_x + static_cast<unsigned int>(scale));
          const unsigned int end_y = std::min(
            source.getSizeInCellsY(),
            begin_y + static_cast<unsigned int>(scale));
          for (unsigned int sy = begin_y; sy < end_y; ++sy) {
            for (unsigned int sx = begin_x; sx < end_x; ++sx) {
              max_cost = std::max(max_cost, source.getCost(sx, sy));
            }
          }
          coarse_costmap.setCost(cx, cy, max_cost);
        }
      }
      return coarse_costmap;
    };

  auto build_cropped_costmap = [&](double margin) -> nav2_costmap_2d::Costmap2D {
      const double min_wx =
        std::min(start.pose.position.x, goal.pose.position.x) - margin;
      const double min_wy =
        std::min(start.pose.position.y, goal.pose.position.y) - margin;
      const double max_wx =
        std::max(start.pose.position.x, goal.pose.position.x) + margin;
      const double max_wy =
        std::max(start.pose.position.y, goal.pose.position.y) + margin;

      auto clamp_to_index = [](int value, unsigned int limit) -> unsigned int {
          if (value < 0) {
            return 0U;
          }
          return std::min(limit - 1U, static_cast<unsigned int>(value));
        };

      const unsigned int min_mx = clamp_to_index(
        static_cast<int>(std::floor((min_wx - costmap.getOriginX()) / costmap.getResolution())),
        costmap.getSizeInCellsX());
      const unsigned int min_my = clamp_to_index(
        static_cast<int>(std::floor((min_wy - costmap.getOriginY()) / costmap.getResolution())),
        costmap.getSizeInCellsY());
      const unsigned int max_mx = clamp_to_index(
        static_cast<int>(std::ceil((max_wx - costmap.getOriginX()) / costmap.getResolution())),
        costmap.getSizeInCellsX());
      const unsigned int max_my = clamp_to_index(
        static_cast<int>(std::ceil((max_wy - costmap.getOriginY()) / costmap.getResolution())),
        costmap.getSizeInCellsY());
      const unsigned int width = max_mx - min_mx + 1U;
      const unsigned int height = max_my - min_my + 1U;

      nav2_costmap_2d::Costmap2D cropped_costmap(
        width,
        height,
        costmap.getResolution(),
        costmap.getOriginX() + static_cast<double>(min_mx) * costmap.getResolution(),
        costmap.getOriginY() + static_cast<double>(min_my) * costmap.getResolution(),
        nav2_costmap_2d::FREE_SPACE);
      for (unsigned int my = 0; my < height; ++my) {
        for (unsigned int mx = 0; mx < width; ++mx) {
          cropped_costmap.setCost(mx, my, costmap.getCost(min_mx + mx, min_my + my));
        }
      }
      return cropped_costmap;
    };

  auto build_from_costmap =
    [&](const nav2_costmap_2d::Costmap2D & reference_costmap)
    -> std::vector<std::pair<double, double>>
    {
      unsigned int start_mx = 0;
      unsigned int start_my = 0;
      if (!reference_costmap.worldToMap(
          start.pose.position.x, start.pose.position.y, start_mx, start_my))
      {
        throw nav2_core::PlannerException("route clearance planner start pose is outside costmap");
      }

      unsigned int goal_mx = 0;
      unsigned int goal_my = 0;
      if (!reference_costmap.worldToMap(
          goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my))
      {
        throw nav2_core::PlannerException("route clearance planner goal pose is outside costmap");
      }

      nav2_navfn_planner::NavFn planner(
        static_cast<int>(reference_costmap.getSizeInCellsX()),
        static_cast<int>(reference_costmap.getSizeInCellsY()));
      std::vector<unsigned char> reference_costmap_data(
        reference_costmap.getCharMap(),
        reference_costmap.getCharMap() +
        static_cast<size_t>(reference_costmap.getSizeInCellsX()) *
        static_cast<size_t>(reference_costmap.getSizeInCellsY()));
      const auto local_clearance_map = buildClearanceMap(reference_costmap);
      const double keepout_radius = std::max(0.0, config_.start_goal_keepout_radius);
      const double endpoint_min_clearance =
        std::min(config_.hard_min_clearance, config_.start_goal_keepout_radius);
      for (unsigned int my = 0; my < reference_costmap.getSizeInCellsY(); ++my) {
        for (unsigned int mx = 0; mx < reference_costmap.getSizeInCellsX(); ++mx) {
          double wx = 0.0;
          double wy = 0.0;
          reference_costmap.mapToWorld(mx, my, wx, wy);
          const std::pair<double, double> cell_point{wx, wy};
          const bool within_endpoint_keepout =
            euclideanDistance(cell_point, start_point) <= keepout_radius ||
            euclideanDistance(cell_point, goal_point) <= keepout_radius;
          const bool traversable = within_endpoint_keepout ?
            isCellTraversableWithMinClearance(
            reference_costmap,
            local_clearance_map,
            mx,
            my,
            endpoint_min_clearance) :
            isCellTraversable(reference_costmap, local_clearance_map, mx, my);
          if (!traversable) {
            reference_costmap_data[reference_costmap.getIndex(mx, my)] =
              nav2_costmap_2d::LETHAL_OBSTACLE;
          }
        }
      }

      if (config_.right_side_bias && config_.right_side_weight > kEpsilon) {
        const double route_dx = goal.pose.position.x - start.pose.position.x;
        const double route_dy = goal.pose.position.y - start.pose.position.y;
        const double route_length = std::hypot(route_dx, route_dy);
        const auto route_right = rightVectorFromTangent(route_dx, route_dy);
        if (
          route_length > kEpsilon &&
          std::hypot(route_right.first, route_right.second) > kEpsilon)
        {
          const double tangent_x = route_dx / route_length;
          const double tangent_y = route_dy / route_length;
          const double lateral_bias_band = std::max(
            config_.right_side_max_offset,
            config_.right_side_target_clearance);
          const double right_reference_clearance = std::max(
            reference_costmap.getResolution(),
            config_.right_side_target_clearance);
          for (unsigned int my = 0; my < reference_costmap.getSizeInCellsY(); ++my) {
            for (unsigned int mx = 0; mx < reference_costmap.getSizeInCellsX(); ++mx) {
              const size_t index = reference_costmap.getIndex(mx, my);
              if (reference_costmap_data[index] >= nav2_costmap_2d::LETHAL_OBSTACLE) {
                continue;
              }

              double wx = 0.0;
              double wy = 0.0;
              reference_costmap.mapToWorld(mx, my, wx, wy);
              const double relative_x = wx - start.pose.position.x;
              const double relative_y = wy - start.pose.position.y;
              const double along_route = relative_x * tangent_x + relative_y * tangent_y;
              if (along_route < -reference_costmap.getResolution() ||
                along_route > route_length + reference_costmap.getResolution())
              {
                continue;
              }

              const double signed_right =
                relative_x * route_right.first + relative_y * route_right.second;
              if (signed_right >= -reference_costmap.getResolution() ||
                std::abs(signed_right) > lateral_bias_band)
              {
                continue;
              }

              const double mirrored_wx = wx - route_right.first * signed_right * 2.0;
              const double mirrored_wy = wy - route_right.second * signed_right * 2.0;
              unsigned int mirrored_mx = 0;
              unsigned int mirrored_my = 0;
              if (!reference_costmap.worldToMap(
                  mirrored_wx, mirrored_wy, mirrored_mx,
                  mirrored_my))
              {
                continue;
              }
              if (!isCellTraversableWithMinClearance(
                  reference_costmap,
                  local_clearance_map,
                  mirrored_mx,
                  mirrored_my,
                  right_reference_clearance))
              {
                continue;
              }
              reference_costmap_data[index] = std::max<unsigned char>(
                reference_costmap_data[index],
                180U);
            }
          }
        }
      }
      reference_costmap_data[reference_costmap.getIndex(start_mx, start_my)] =
        nav2_costmap_2d::FREE_SPACE;
      reference_costmap_data[reference_costmap.getIndex(goal_mx, goal_my)] =
        nav2_costmap_2d::FREE_SPACE;

      planner.setCostmap(
        reference_costmap_data.data(),
        true,
        config_.reference_allow_unknown);

      int map_start[2] = {static_cast<int>(start_mx), static_cast<int>(start_my)};
      int map_goal[2] = {static_cast<int>(goal_mx), static_cast<int>(goal_my)};
      planner.setStart(map_goal);
      planner.setGoal(map_start);
      if (config_.reference_use_astar) {
        planner.calcNavFnAstar();
      } else {
        planner.calcNavFnDijkstra(true);
      }

      const unsigned int goal_index = goal_my * static_cast<unsigned int>(planner.nx) + goal_mx;
      if (planner.potarr[goal_index] >= POT_HIGH) {
        throw nav2_core::PlannerException(
                "route clearance planner could not find a coarse reference path");
      }

      planner.setStart(map_goal);
      const int max_cycles =
        static_cast<int>(
        std::max(reference_costmap.getSizeInCellsX(), reference_costmap.getSizeInCellsY()) * 4U);
      if (planner.calcPath(max_cycles) == 0 || planner.getPathLen() == 0) {
        throw nav2_core::PlannerException(
                "route clearance planner coarse reference path extraction failed");
      }

      float * path_x = planner.getPathX();
      float * path_y = planner.getPathY();
      const int path_length = planner.getPathLen();
      const double min_distance = std::max(reference_costmap.getResolution() * 2.0, 0.25);

      std::vector<std::pair<double, double>> points;
      appendUniquePoint(
        points,
        {start.pose.position.x, start.pose.position.y},
        min_distance);

      for (int index = path_length - 1; index >= 0; --index) {
        const double wx = reference_costmap.getOriginX() + static_cast<double>(path_x[index]) *
          reference_costmap.getResolution();
        const double wy = reference_costmap.getOriginY() + static_cast<double>(path_y[index]) *
          reference_costmap.getResolution();
        appendUniquePoint(points, {wx, wy}, min_distance);
      }

      if (!points.empty() && euclideanDistance(points.back(), goal_point) <= min_distance) {
        points.back() = goal_point;
      } else {
        points.push_back(goal_point);
      }
      const double shortcut_min_clearance = std::max(
        config_.hard_min_clearance,
        std::min(
          config_.soft_target_clearance,
          std::max(config_.right_side_target_clearance, config_.hard_min_clearance + 0.25)));
      auto segment_has_clearance =
        [&](const std::pair<double, double> & from,
          const std::pair<double, double> & to) -> bool
        {
          const double distance = euclideanDistance(from, to);
          const double sample_resolution = std::max(reference_costmap.getResolution(), 0.10);
          const int steps = std::max(1, static_cast<int>(std::ceil(distance / sample_resolution)));
          for (int step = 0; step <= steps; ++step) {
            const double ratio = static_cast<double>(step) / static_cast<double>(steps);
            const double wx = from.first + (to.first - from.first) * ratio;
            const double wy = from.second + (to.second - from.second) * ratio;
            unsigned int mx = 0;
            unsigned int my = 0;
            if (!reference_costmap.worldToMap(wx, wy, mx, my)) {
              return false;
            }
            const std::pair<double, double> point{wx, wy};
            const bool within_endpoint_keepout =
              euclideanDistance(point, start_point) <= keepout_radius ||
              euclideanDistance(point, goal_point) <= keepout_radius;
            const double min_clearance = within_endpoint_keepout ?
              endpoint_min_clearance : shortcut_min_clearance;
            if (!isCellTraversableWithMinClearance(
                reference_costmap,
                local_clearance_map,
                mx,
                my,
                min_clearance))
            {
              return false;
            }
          }
          return true;
        };
      const auto turn_angle_degrees =
        [](const std::pair<double, double> & previous,
          const std::pair<double, double> & current,
          const std::pair<double, double> & next) -> double
        {
          const double ax = current.first - previous.first;
          const double ay = current.second - previous.second;
          const double bx = next.first - current.first;
          const double by = next.second - current.second;
          const double a_length = std::hypot(ax, ay);
          const double b_length = std::hypot(bx, by);
          if (a_length <= kEpsilon || b_length <= kEpsilon) {
            return 0.0;
          }
          const double dot = std::max(
            -1.0,
            std::min(1.0, (ax * bx + ay * by) / (a_length * b_length)));
          return std::acos(dot) * 180.0 / M_PI;
        };

      if (points.size() > 2U) {
        constexpr double max_shortcut_turn_degrees = 25.0;
        const double max_shortcut_distance = std::max(
          60.0,
          config_.reference_corridor_half_width * 10.0);
        std::vector<std::pair<double, double>> shortcut_points;
        shortcut_points.reserve(points.size());
        size_t index = 0;
        shortcut_points.push_back(points.front());
        while (index + 1U < points.size()) {
          size_t best_index = index + 1U;
          for (size_t candidate = index + 2U; candidate < points.size(); ++candidate) {
            if (euclideanDistance(points[index], points[candidate]) > max_shortcut_distance) {
              break;
            }
            if (shortcut_points.size() >= 2U &&
              turn_angle_degrees(
                shortcut_points[shortcut_points.size() - 2U],
                points[index],
                points[candidate]) > max_shortcut_turn_degrees)
            {
              continue;
            }
            if (segment_has_clearance(points[index], points[candidate])) {
              best_index = candidate;
            }
          }
          appendUniquePoint(shortcut_points, points[best_index], min_distance);
          index = best_index;
        }
        if (!shortcut_points.empty() && shortcut_points.back() != goal_point) {
          shortcut_points.push_back(goal_point);
        }
        points = std::move(shortcut_points);
      }
      return points;
    };

  const double base_margin = std::max(
    20.0,
    std::max(
      config_.reference_corridor_half_width * 4.0,
      std::max(config_.pose_directed_max_corridor_half_width * 2.0, config_.goal_search_radius)));
  const double margins[] = {base_margin, std::max(base_margin * 1.8, 45.0)};
  for (const double margin : margins) {
    try {
      return build_from_costmap(coarsen_costmap(build_cropped_costmap(margin)));
    } catch (const nav2_core::PlannerException &) {
    }
  }

  return build_from_costmap(coarsen_costmap(costmap));
}

RouteClearancePlannerCore::PlanningContext RouteClearancePlannerCore::buildPlanningContext(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<std::pair<double, double>> & reference_points,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  double corridor_half_width_override) const
{
  unsigned int start_mx = 0;
  unsigned int start_my = 0;
  unsigned int goal_mx = 0;
  unsigned int goal_my = 0;
  worldToMapChecked(costmap, start, start_mx, start_my, "start");
  worldToMapChecked(costmap, goal, goal_mx, goal_my, "goal");

  const double corridor_half_width =
    corridor_half_width_override >= 0.0 ?
    corridor_half_width_override : config_.reference_corridor_half_width;
  const double margin = std::max(
    corridor_half_width + config_.start_goal_keepout_radius,
    costmap.getResolution() * 2.0);

  double min_wx = std::min(start.pose.position.x, goal.pose.position.x);
  double min_wy = std::min(start.pose.position.y, goal.pose.position.y);
  double max_wx = std::max(start.pose.position.x, goal.pose.position.x);
  double max_wy = std::max(start.pose.position.y, goal.pose.position.y);

  std::vector<std::pair<double, double>> sampled_reference;
  const double sample_min_distance = std::max(
    costmap.getResolution() * 2.0,
    std::max(0.2, corridor_half_width * 0.25));
  for (const auto & point : reference_points) {
    appendUniquePoint(sampled_reference, point, sample_min_distance);
    min_wx = std::min(min_wx, point.first);
    min_wy = std::min(min_wy, point.second);
    max_wx = std::max(max_wx, point.first);
    max_wy = std::max(max_wy, point.second);
  }
  if (sampled_reference.empty()) {
    sampled_reference = reference_points;
  }

  min_wx -= margin;
  min_wy -= margin;
  max_wx += margin;
  max_wy += margin;

  auto clamp_to_index = [](int value, unsigned int limit) -> unsigned int {
      if (value < 0) {
        return 0U;
      }
      const unsigned int cast = static_cast<unsigned int>(value);
      return std::min(limit - 1U, cast);
    };

  const double origin_x = costmap.getOriginX();
  const double origin_y = costmap.getOriginY();
  const double resolution = costmap.getResolution();
  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();

  const unsigned int min_mx = clamp_to_index(
    static_cast<int>(std::floor((min_wx - origin_x) / resolution)),
    size_x);
  const unsigned int min_my = clamp_to_index(
    static_cast<int>(std::floor((min_wy - origin_y) / resolution)),
    size_y);
  const unsigned int max_mx = clamp_to_index(
    static_cast<int>(std::ceil((max_wx - origin_x) / resolution)),
    size_x);
  const unsigned int max_my = clamp_to_index(
    static_cast<int>(std::ceil((max_wy - origin_y) / resolution)),
    size_y);

  const unsigned int width = max_mx - min_mx + 1U;
  const unsigned int height = max_my - min_my + 1U;
  nav2_costmap_2d::Costmap2D cropped_costmap(
    width,
    height,
    costmap.getResolution(),
    costmap.getOriginX() + static_cast<double>(min_mx) * costmap.getResolution(),
    costmap.getOriginY() + static_cast<double>(min_my) * costmap.getResolution(),
    nav2_costmap_2d::LETHAL_OBSTACLE);

  const std::pair<double, double> start_point{
    start.pose.position.x, start.pose.position.y};
  const std::pair<double, double> goal_point{
    goal.pose.position.x, goal.pose.position.y};
  const double keepout_radius = std::max(0.0, config_.start_goal_keepout_radius);
  const double effective_corridor_half_width = std::max(0.0, corridor_half_width);

  auto copy_source_cell =
    [&](unsigned int mx, unsigned int my) {
      const unsigned int source_mx = min_mx + mx;
      const unsigned int source_my = min_my + my;
      cropped_costmap.setCost(mx, my, costmap.getCost(source_mx, source_my));
    };

  auto clamp_local_index = [](int value, unsigned int limit) -> unsigned int {
      if (value < 0) {
        return 0U;
      }
      return std::min(limit - 1U, static_cast<unsigned int>(value));
    };

  auto open_disc =
    [&](const std::pair<double, double> & center, double radius) {
      const double radius_for_bounds = std::max(radius, resolution * 0.5);
      const unsigned int local_min_mx = clamp_local_index(
        static_cast<int>(
          std::floor(
            (center.first - radius_for_bounds - cropped_costmap.getOriginX()) / resolution)),
        width);
      const unsigned int local_min_my = clamp_local_index(
        static_cast<int>(
          std::floor(
            (center.second - radius_for_bounds - cropped_costmap.getOriginY()) / resolution)),
        height);
      const unsigned int local_max_mx = clamp_local_index(
        static_cast<int>(
          std::ceil(
            (center.first + radius_for_bounds - cropped_costmap.getOriginX()) / resolution)),
        width);
      const unsigned int local_max_my = clamp_local_index(
        static_cast<int>(
          std::ceil(
            (center.second + radius_for_bounds - cropped_costmap.getOriginY()) / resolution)),
        height);

      for (unsigned int my = local_min_my; my <= local_max_my; ++my) {
        for (unsigned int mx = local_min_mx; mx <= local_max_mx; ++mx) {
          double wx = 0.0;
          double wy = 0.0;
          cropped_costmap.mapToWorld(mx, my, wx, wy);
          if (euclideanDistance({wx, wy}, center) <= radius_for_bounds) {
            copy_source_cell(mx, my);
          }
        }
      }
    };

  auto open_segment =
    [&](const std::pair<double, double> & segment_start,
      const std::pair<double, double> & segment_end)
    {
      const double corridor_for_bounds =
        effective_corridor_half_width + resolution * std::sqrt(2.0);
      const unsigned int local_min_mx = clamp_local_index(
        static_cast<int>(
          std::floor(
            (std::min(segment_start.first, segment_end.first) - corridor_for_bounds -
            cropped_costmap.getOriginX()) / resolution)),
        width);
      const unsigned int local_min_my = clamp_local_index(
        static_cast<int>(
          std::floor(
            (std::min(segment_start.second, segment_end.second) - corridor_for_bounds -
            cropped_costmap.getOriginY()) / resolution)),
        height);
      const unsigned int local_max_mx = clamp_local_index(
        static_cast<int>(
          std::ceil(
            (std::max(segment_start.first, segment_end.first) + corridor_for_bounds -
            cropped_costmap.getOriginX()) / resolution)),
        width);
      const unsigned int local_max_my = clamp_local_index(
        static_cast<int>(
          std::ceil(
            (std::max(segment_start.second, segment_end.second) + corridor_for_bounds -
            cropped_costmap.getOriginY()) / resolution)),
        height);

      for (unsigned int my = local_min_my; my <= local_max_my; ++my) {
        for (unsigned int mx = local_min_mx; mx <= local_max_mx; ++mx) {
          double wx = 0.0;
          double wy = 0.0;
          cropped_costmap.mapToWorld(mx, my, wx, wy);
          if (pointToSegmentDistance(
              {wx, wy},
              segment_start,
              segment_end) <= effective_corridor_half_width)
          {
            copy_source_cell(mx, my);
          }
        }
      }
    };

  if (sampled_reference.size() == 1U) {
    open_disc(sampled_reference.front(), effective_corridor_half_width);
  } else {
    for (size_t index = 1; index < sampled_reference.size(); ++index) {
      open_segment(sampled_reference[index - 1], sampled_reference[index]);
    }
  }
  open_disc(start_point, keepout_radius);
  open_disc(goal_point, keepout_radius);
  copy_source_cell(start_mx - min_mx, start_my - min_my);
  copy_source_cell(goal_mx - min_mx, goal_my - min_my);

  unsigned int cropped_start_mx = 0;
  unsigned int cropped_start_my = 0;
  unsigned int cropped_goal_mx = 0;
  unsigned int cropped_goal_my = 0;
  worldToMapChecked(cropped_costmap, start, cropped_start_mx, cropped_start_my, "start");
  worldToMapChecked(cropped_costmap, goal, cropped_goal_mx, cropped_goal_my, "goal");

  PlanningContext context{
    std::move(cropped_costmap),
    {cropped_start_mx, cropped_start_my},
    {cropped_goal_mx, cropped_goal_my},
    sampled_reference};
  return context;
}

nav_msgs::msg::Path RouteClearancePlannerCore::buildOptimizedPathFromReference(
  const PlanningContext & planning_context,
  const std::vector<double> & clearance_map,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & effective_goal,
  const std::string & frame_id) const
{
  auto build_path_from_points =
    [&](const std::vector<std::pair<double, double>> & raw_points) -> nav_msgs::msg::Path {
      nav_msgs::msg::Path path;
      path.header.frame_id = frame_id;
      const auto points = interpolatePoints(
        raw_points,
        resolveInterpolationResolution(planning_context.costmap));
      path.poses.reserve(points.size());
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
        path.poses.front().pose.orientation = effective_goal.pose.orientation;
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
      if (config_.use_final_goal_orientation) {
        path.poses.back().pose.orientation = effective_goal.pose.orientation;
      } else if (path.poses.size() > 1U) {
        path.poses.back().pose.orientation =
          path.poses[path.poses.size() - 2U].pose.orientation;
      }
      return path;
    };

  auto path_respects_hard_clearance =
    [&](const nav_msgs::msg::Path & path) -> bool {
      const double endpoint_min_clearance =
        std::min(config_.hard_min_clearance, config_.start_goal_keepout_radius);
      const double keepout_radius = std::max(0.0, config_.start_goal_keepout_radius);
      const auto min_clearance_for_point = [&](double wx, double wy) -> double {
          const std::pair<double, double> point{wx, wy};
          const std::pair<double, double> start_point{
            start.pose.position.x, start.pose.position.y};
          const std::pair<double, double> goal_point{
            effective_goal.pose.position.x, effective_goal.pose.position.y};
          if (
            euclideanDistance(point, start_point) <= keepout_radius ||
            euclideanDistance(point, goal_point) <= keepout_radius)
          {
            return endpoint_min_clearance;
          }
          return config_.hard_min_clearance;
        };
      for (size_t index = 0; index < path.poses.size(); ++index) {
        const auto & pose = path.poses[index];
        unsigned int mx = 0;
        unsigned int my = 0;
        if (!planning_context.costmap.worldToMap(
            pose.pose.position.x,
            pose.pose.position.y,
            mx,
            my))
        {
          return false;
        }
        if (!isCellTraversableWithMinClearance(
            planning_context.costmap,
            clearance_map,
            mx,
            my,
            min_clearance_for_point(pose.pose.position.x, pose.pose.position.y)))
        {
          return false;
        }
      }
      return true;
    };

  std::vector<std::pair<double, double>> optimized_points;
  const double sample_resolution = std::max(
    planning_context.costmap.getResolution(),
    resolveInterpolationResolution(planning_context.costmap));
  const auto inflation_boundary_clearance_map =
    buildInflationBoundaryClearanceMap(planning_context.costmap);
  const double right_bias_min_clearance = std::max(
    config_.hard_min_clearance,
    std::min(
      config_.soft_target_clearance,
      std::max(config_.right_side_target_clearance, config_.hard_min_clearance)));
  const double lateral_limit = std::max(
    planning_context.costmap.getResolution(),
    config_.reference_corridor_half_width * 0.5);
  const double lateral_step = std::max(
    planning_context.costmap.getResolution(),
    config_.reference_corridor_half_width * 0.1);
  const double endpoint_min_clearance =
    std::min(config_.hard_min_clearance, config_.start_goal_keepout_radius);
  const double keepout_radius = std::max(0.0, config_.start_goal_keepout_radius);
  const auto min_clearance_for_point = [&](double wx, double wy) -> double {
      const std::pair<double, double> point{wx, wy};
      const std::pair<double, double> start_point{
        start.pose.position.x, start.pose.position.y};
      const std::pair<double, double> goal_point{
        effective_goal.pose.position.x, effective_goal.pose.position.y};
      if (
        euclideanDistance(point, start_point) <= keepout_radius ||
        euclideanDistance(point, goal_point) <= keepout_radius)
      {
        return endpoint_min_clearance;
      }
      return config_.hard_min_clearance;
    };
  const double shortcut_min_clearance = std::max(
    config_.hard_min_clearance,
    std::min(
      config_.soft_target_clearance,
      std::max(config_.right_side_target_clearance, config_.hard_min_clearance + 0.25)));
  const auto min_shortcut_clearance_for_point = [&](double wx, double wy) -> double {
      const std::pair<double, double> point{wx, wy};
      const std::pair<double, double> start_point{
        start.pose.position.x, start.pose.position.y};
      const std::pair<double, double> goal_point{
        effective_goal.pose.position.x, effective_goal.pose.position.y};
      if (
        euclideanDistance(point, start_point) <= keepout_radius ||
        euclideanDistance(point, goal_point) <= keepout_radius)
      {
        return endpoint_min_clearance;
      }
      return shortcut_min_clearance;
    };
  const auto segment_is_valid =
    [&](const std::pair<double, double> & from, const std::pair<double, double> & to) -> bool {
      const double distance = euclideanDistance(from, to);
      const int steps = std::max(1, static_cast<int>(std::ceil(distance / sample_resolution)));
      for (int step = 1; step <= steps; ++step) {
        const double ratio = static_cast<double>(step) / static_cast<double>(steps);
        const double wx = from.first + (to.first - from.first) * ratio;
        const double wy = from.second + (to.second - from.second) * ratio;
        unsigned int mx = 0;
        unsigned int my = 0;
        if (!planning_context.costmap.worldToMap(wx, wy, mx, my)) {
          return false;
        }
        if (!isCellTraversableWithMinClearance(
            planning_context.costmap,
            clearance_map,
            mx,
            my,
            min_clearance_for_point(wx, wy)))
        {
          return false;
        }
      }
      return true;
    };
  const auto shortcut_segment_is_valid =
    [&](const std::pair<double, double> & from, const std::pair<double, double> & to) -> bool {
      const double distance = euclideanDistance(from, to);
      const int steps = std::max(1, static_cast<int>(std::ceil(distance / sample_resolution)));
      for (int step = 1; step <= steps; ++step) {
        const double ratio = static_cast<double>(step) / static_cast<double>(steps);
        const double wx = from.first + (to.first - from.first) * ratio;
        const double wy = from.second + (to.second - from.second) * ratio;
        unsigned int mx = 0;
        unsigned int my = 0;
        if (!planning_context.costmap.worldToMap(wx, wy, mx, my)) {
          return false;
        }
        if (!isCellTraversableWithMinClearance(
            planning_context.costmap,
            clearance_map,
            mx,
            my,
            min_shortcut_clearance_for_point(wx, wy)))
        {
          return false;
        }
      }
      return true;
    };

  auto smooth_points_if_safe =
    [&](std::vector<std::pair<double, double>> points) -> std::vector<std::pair<double, double>> {
      const int smoothing_passes = config_.right_side_bias ?
        std::clamp(config_.lateral_smoothing_passes, 0, 8) : 0;
      if (smoothing_passes == 0 || points.size() < 4U) {
        return points;
      }

      const double smoothing_alpha = 0.35;
      for (int pass = 0; pass < smoothing_passes; ++pass) {
        for (size_t index = 1; index + 1 < points.size(); ++index) {
          const auto & previous = points[index - 1];
          const auto & current = points[index];
          const auto & next = points[index + 1];
          const std::pair<double, double> midpoint{
            (previous.first + next.first) * 0.5,
            (previous.second + next.second) * 0.5};
          const std::pair<double, double> candidate{
            current.first * (1.0 - smoothing_alpha) + midpoint.first * smoothing_alpha,
            current.second * (1.0 - smoothing_alpha) + midpoint.second * smoothing_alpha};

          unsigned int mx = 0;
          unsigned int my = 0;
          if (!planning_context.costmap.worldToMap(candidate.first, candidate.second, mx, my)) {
            continue;
          }
          if (!isCellTraversable(planning_context.costmap, clearance_map, mx, my)) {
            continue;
          }
          if (!segment_is_valid(previous, candidate) || !segment_is_valid(candidate, next)) {
            continue;
          }
          points[index] = candidate;
        }
      }
      return points;
    };
  auto shortcut_path_if_safe =
    [&](const nav_msgs::msg::Path & path) -> nav_msgs::msg::Path {
      if (path.poses.size() < 4U) {
        return path;
      }
      std::vector<std::pair<double, double>> points;
      points.reserve(path.poses.size());
      for (const auto & pose : path.poses) {
        points.push_back({pose.pose.position.x, pose.pose.position.y});
      }
      double path_length = 0.0;
      for (size_t index = 1; index < points.size(); ++index) {
        path_length += euclideanDistance(points[index - 1], points[index]);
      }
      if (path_length < 30.0) {
        return path;
      }

      const auto turn_angle_degrees =
        [](const std::pair<double, double> & previous,
          const std::pair<double, double> & current,
          const std::pair<double, double> & next) -> double
        {
          const double ax = current.first - previous.first;
          const double ay = current.second - previous.second;
          const double bx = next.first - current.first;
          const double by = next.second - current.second;
          const double a_length = std::hypot(ax, ay);
          const double b_length = std::hypot(bx, by);
          if (a_length <= kEpsilon || b_length <= kEpsilon) {
            return 0.0;
          }
          const double dot = std::max(
            -1.0,
            std::min(1.0, (ax * bx + ay * by) / (a_length * b_length)));
          return std::acos(dot) * 180.0 / M_PI;
        };
      constexpr double max_shortcut_turn_degrees = 25.0;
      const double max_shortcut_distance = std::max(
        60.0,
        config_.reference_corridor_half_width * 10.0);
      std::vector<std::pair<double, double>> shortcut_points;
      shortcut_points.reserve(points.size());
      size_t index = 0;
      shortcut_points.push_back(points.front());
      while (index + 1U < points.size()) {
        size_t best_index = index + 1U;
        for (size_t candidate = index + 2U; candidate < points.size(); ++candidate) {
          if (euclideanDistance(points[index], points[candidate]) > max_shortcut_distance) {
            break;
          }
          if (shortcut_points.size() >= 2U &&
            turn_angle_degrees(
              shortcut_points[shortcut_points.size() - 2U],
              points[index],
              points[candidate]) > max_shortcut_turn_degrees)
          {
            continue;
          }
          if (shortcut_segment_is_valid(points[index], points[candidate])) {
            best_index = candidate;
          }
        }
        appendUniquePoint(shortcut_points, points[best_index], sample_resolution * 0.5);
        index = best_index;
      }
      if (shortcut_points.size() >= points.size()) {
        return path;
      }
      return build_path_from_points(shortcut_points);
    };

  auto right_side_score =
    [&](const std::pair<double, double> & point,
      double right_x,
      double right_y,
      double right_offset,
      double lateral_limit_value) -> double
    {
      if (!config_.right_side_bias || config_.right_side_weight <= kEpsilon) {
        return 0.0;
      }

      const double max_right_offset = std::max(
        planning_context.costmap.getResolution(),
        std::min(
          lateral_limit_value,
          config_.right_side_max_offset > kEpsilon ?
          config_.right_side_max_offset : lateral_limit_value));
      const double target_clearance = std::max(
        planning_context.costmap.getResolution(),
        config_.right_side_target_clearance);
      const double probe_distance = std::max(
        target_clearance,
        config_.right_side_probe_distance);
      const double probe_step = std::max(
        planning_context.costmap.getResolution(),
        target_clearance * 0.1);

      const auto measure_boundary_space =
        [&](double direction_x, double direction_y) -> double {
          for (double distance = probe_step; distance <= probe_distance + kEpsilon;
            distance += probe_step)
          {
            unsigned int mx = 0;
            unsigned int my = 0;
            const double wx = point.first + direction_x * distance;
            const double wy = point.second + direction_y * distance;
            if (!planning_context.costmap.worldToMap(wx, wy, mx, my)) {
              return distance;
            }
            const unsigned char cost = planning_context.costmap.getCost(mx, my);
            const bool boundary = cost == nav2_costmap_2d::NO_INFORMATION ?
              !config_.allow_unknown : cost != nav2_costmap_2d::FREE_SPACE;
            if (boundary) {
              return distance;
            }
          }
          return probe_distance;
        };

      const double right_space = measure_boundary_space(right_x, right_y);
      const double left_space = measure_boundary_space(-right_x, -right_y);
      const double wider_margin = std::max(
        planning_context.costmap.getResolution(),
        target_clearance * 0.20);
      const bool right_is_wide_enough = right_space + kEpsilon >= target_clearance;
      const bool right_is_not_narrower =
        right_space + wider_margin >= left_space ||
        (right_space >= probe_distance - kEpsilon && left_space >= probe_distance - kEpsilon);

      double score = 0.0;
      if (!right_is_wide_enough || !right_is_not_narrower) {
        if (right_offset > 0.0) {
          score += right_offset / max_right_offset * 6.0;
        }
        return score * config_.right_side_weight;
      }

      const double normalized_offset_error =
        std::max(0.0, max_right_offset - right_offset) / max_right_offset;
      if (right_space < probe_distance - kEpsilon) {
        const double normalized_error =
          std::abs(right_space - target_clearance) / target_clearance;
        if (right_space < target_clearance) {
          score += normalized_error * normalized_error * 16.0;
        } else {
          score += normalized_error * normalized_error * 0.5;
          score += normalized_offset_error * normalized_offset_error;
        }
      } else {
        score += normalized_offset_error * normalized_offset_error;
      }

      if (right_offset < 0.0) {
        score += std::abs(right_offset) / max_right_offset * 2.0;
      }
      if (right_offset > max_right_offset) {
        const double normalized_excess = (right_offset - max_right_offset) / max_right_offset;
        score += normalized_excess * normalized_excess * 4.0;
      }
      return score * config_.right_side_weight;
    };

  optimized_points.reserve(planning_context.reference_points.size() + 2U);
  optimized_points.push_back({start.pose.position.x, start.pose.position.y});

  std::vector<std::pair<double, double>> sampled_reference;
  const double optimize_stride = std::max(sample_resolution * 4.0, 0.25);
  for (const auto & point : planning_context.reference_points) {
    appendUniquePoint(sampled_reference, point, optimize_stride);
  }
  if (sampled_reference.size() < 3U && planning_context.reference_points.size() >= 2U) {
    sampled_reference = interpolatePoints(planning_context.reference_points, optimize_stride);
  }
  if (sampled_reference.size() < 3U) {
    sampled_reference = planning_context.reference_points;
  }

  double previous_offset = 0.0;
  for (size_t index = 1; index + 1 < sampled_reference.size(); ++index) {
    const auto & previous = sampled_reference[index - 1];
    const auto & current = sampled_reference[index];
    const auto & next = sampled_reference[index + 1];
    const std::pair<double, double> next_target =
      index + 2 < sampled_reference.size() ?
      sampled_reference[index + 1] :
      std::pair<double, double>{
      effective_goal.pose.position.x,
      effective_goal.pose.position.y};

    const double tangent_x = next.first - previous.first;
    const double tangent_y = next.second - previous.second;
    const double tangent_length = std::hypot(tangent_x, tangent_y);
    if (tangent_length <= kEpsilon) {
      appendUniquePoint(optimized_points, current, sample_resolution * 0.5);
      continue;
    }

    const double normal_x = -tangent_y / tangent_length;
    const double normal_y = tangent_x / tangent_length;
    const auto right_vector = rightVectorFromTangent(tangent_x, tangent_y);
    std::pair<double, double> best_point = current;
    double best_score = std::numeric_limits<double>::infinity();
    double best_offset = 0.0;
    for (
      double offset = -lateral_limit; offset <= lateral_limit + kEpsilon;
      offset += lateral_step)
    {
      const double wx = current.first + normal_x * offset;
      const double wy = current.second + normal_y * offset;
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!planning_context.costmap.worldToMap(wx, wy, mx, my)) {
        continue;
      }
      if (!isCellTraversable(planning_context.costmap, clearance_map, mx, my)) {
        continue;
      }
      if (!segment_is_valid(optimized_points.back(), {wx, wy})) {
        continue;
      }
      if (!segment_is_valid({wx, wy}, next_target)) {
        continue;
      }

      const double clearance = clearance_map[static_cast<size_t>(cellIndex(
            mx,
            my,
            planning_context.costmap.getSizeInCellsX()))];
      const double inflation_boundary_clearance =
        inflation_boundary_clearance_map[static_cast<size_t>(cellIndex(
            mx,
            my,
            planning_context.costmap.getSizeInCellsX()))];
      const double clearance_deficit = std::max(0.0, config_.soft_target_clearance - clearance);
      const double inflation_boundary_deficit = std::max(
        0.0,
        right_bias_min_clearance - inflation_boundary_clearance);
      const double lateral_penalty = std::abs(offset);
      const double centerline_penalty =
        lateral_penalty * std::max(0.0, config_.centerline_weight);
      const double lateral_change_penalty = std::abs(offset - previous_offset);
      const double right_offset = (normal_x * offset) * right_vector.first +
        (normal_y * offset) * right_vector.second;
      const double raw_cost = static_cast<double>(planning_context.costmap.getCost(mx, my));
      const double normalized_cost = raw_cost /
        static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
      const auto & previous_point = optimized_points.back();
      const double incoming_distance = euclideanDistance(previous_point, {wx, wy});
      const double outgoing_distance = euclideanDistance({wx, wy}, next_target);
      const double direct_distance = euclideanDistance(previous_point, next_target);
      const double detour_penalty =
        std::max(0.0, incoming_distance + outgoing_distance - direct_distance);
      double turn_angle = 0.0;
      if (incoming_distance > kEpsilon && outgoing_distance > kEpsilon) {
        const double incoming_x = wx - previous_point.first;
        const double incoming_y = wy - previous_point.second;
        const double outgoing_x = next_target.first - wx;
        const double outgoing_y = next_target.second - wy;
        const double dot = std::max(
          -1.0,
          std::min(
            1.0,
            (incoming_x * outgoing_x + incoming_y * outgoing_y) /
            (incoming_distance * outgoing_distance)));
        turn_angle = std::acos(dot);
      }
      const double score =
        clearance_deficit * config_.clearance_weight * 10.0 +
        lateral_penalty * 0.5 +
        centerline_penalty * 2.0 +
        lateral_change_penalty * std::max(0.0, config_.lateral_change_weight) +
        right_side_score(
        {wx, wy},
        right_vector.first,
        right_vector.second,
        right_offset,
        lateral_limit) +
        inflation_boundary_deficit * inflation_boundary_deficit *
        std::max(config_.clearance_weight, config_.right_side_weight) * 12.0 +
        detour_penalty * 4.0 +
        turn_angle * config_.turn_weight * 15.0 +
        normalized_cost * normalized_cost * config_.cost_weight * 4.0;
      if (score < best_score) {
        best_score = score;
        best_point = {wx, wy};
        best_offset = offset;
      }
    }

    appendUniquePoint(optimized_points, best_point, sample_resolution * 0.5);
    previous_offset = best_offset;
  }

  optimized_points.push_back(
    {
      effective_goal.pose.position.x,
      effective_goal.pose.position.y});

  optimized_points = smooth_points_if_safe(optimized_points);

  auto optimized_path = biasPathToRightIfSafe(
    build_path_from_points(optimized_points),
    planning_context.costmap,
    clearance_map);
  optimized_path = shortcut_path_if_safe(optimized_path);
  if (path_respects_hard_clearance(optimized_path)) {
    return downsampleOutputPath(optimized_path, start, effective_goal);
  }

  if (
    isCellTraversable(
      planning_context.costmap,
      clearance_map,
      planning_context.start.mx,
      planning_context.start.my) &&
    isCellTraversable(
      planning_context.costmap,
      clearance_map,
      planning_context.goal.mx,
      planning_context.goal.my))
  {
    const auto searched_path = searchPath(
      planning_context.costmap,
      clearance_map,
      planning_context.start,
      planning_context.goal);
    if (searched_path.found) {
      const auto safe_path = buildPath(
        planning_context.costmap,
        clearance_map,
        searched_path.cells,
        start,
        effective_goal,
        frame_id);
      const auto biased_safe_path = biasPathToRightIfSafe(
        safe_path,
        planning_context.costmap,
        clearance_map);
      if (path_respects_hard_clearance(biased_safe_path)) {
        return downsampleOutputPath(biased_safe_path, start, effective_goal);
      }
    }
  }

  std::vector<std::pair<double, double>> fallback_points = planning_context.reference_points;
  if (fallback_points.empty()) {
    fallback_points = optimized_points;
  }
  if (
    fallback_points.front().first != start.pose.position.x ||
    fallback_points.front().second != start.pose.position.y)
  {
    fallback_points.insert(
      fallback_points.begin(),
      {start.pose.position.x, start.pose.position.y});
  }
  if (
    fallback_points.back().first != effective_goal.pose.position.x ||
    fallback_points.back().second != effective_goal.pose.position.y)
  {
    fallback_points.push_back(
      {effective_goal.pose.position.x, effective_goal.pose.position.y});
  }
  const auto fallback_path = biasPathToRightIfSafe(
    build_path_from_points(fallback_points),
    planning_context.costmap,
    clearance_map);
  if (path_respects_hard_clearance(fallback_path)) {
    return downsampleOutputPath(fallback_path, start, effective_goal);
  }
  nav_msgs::msg::Path empty_path;
  empty_path.header.frame_id = frame_id;
  return empty_path;
}

std::vector<RouteClearancePlannerCore::GridCell>
RouteClearancePlannerCore::buildGoalCandidates(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<double> & clearance_map,
  unsigned int goal_mx,
  unsigned int goal_my) const
{
  struct Candidate
  {
    GridCell cell;
    double score{0.0};
  };

  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();
  const int max_radius_cells = std::max(
    0,
    static_cast<int>(std::ceil(
      std::max(
        0.0,
        config_.goal_search_radius) / costmap.getResolution())));

  std::vector<Candidate> candidates;
  candidates.reserve(static_cast<size_t>(std::max(1, config_.max_goal_candidates)));
  for (int dy = -max_radius_cells; dy <= max_radius_cells; ++dy) {
    for (int dx = -max_radius_cells; dx <= max_radius_cells; ++dx) {
      const int nx = static_cast<int>(goal_mx) + dx;
      const int ny = static_cast<int>(goal_my) + dy;
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(size_x) || ny >= static_cast<int>(size_y)) {
        continue;
      }
      const double metric_distance = std::hypot(
        static_cast<double>(dx),
        static_cast<double>(dy)) * costmap.getResolution();
      if (metric_distance > config_.goal_search_radius + kEpsilon) {
        continue;
      }
      const GridCell cell{static_cast<unsigned int>(nx), static_cast<unsigned int>(ny)};
      if (!isCellTraversable(costmap, clearance_map, cell.mx, cell.my)) {
        continue;
      }
      const double clearance =
        clearance_map[static_cast<size_t>(cellIndex(cell.mx, cell.my, size_x))];
      const double soft_deficit =
        std::max(0.0, std::max(0.0, config_.soft_target_clearance) - clearance);
      candidates.push_back({cell, metric_distance + soft_deficit * 0.25});
    }
  }

  std::sort(
    candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) {
      if (std::abs(a.score - b.score) > kEpsilon) {
        return a.score < b.score;
      }
      if (a.cell.my != b.cell.my) {
        return a.cell.my < b.cell.my;
      }
      return a.cell.mx < b.cell.mx;
    });

  if (config_.max_goal_candidates > 0 &&
    candidates.size() > static_cast<size_t>(config_.max_goal_candidates))
  {
    candidates.resize(static_cast<size_t>(config_.max_goal_candidates));
  }

  std::vector<GridCell> cells;
  cells.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    cells.push_back(candidate.cell);
  }
  return cells;
}

double RouteClearancePlannerCore::cellTraversalPenalty(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<double> & clearance_map,
  const GridCell & cell) const
{
  const double clearance =
    clearance_map[static_cast<size_t>(cellIndex(cell.mx, cell.my, costmap.getSizeInCellsX()))];
  const double soft_target = std::max(0.0, config_.soft_target_clearance);
  const double clearance_deficit = std::max(0.0, soft_target - clearance);
  const double normalized_clearance_deficit =
    soft_target > kEpsilon ? clearance_deficit / soft_target : 0.0;
  const double finite_clearance = std::isfinite(clearance) ? clearance : soft_target;
  const double normalized_clearance_bonus =
    soft_target > kEpsilon ? finite_clearance / soft_target : finite_clearance;

  const unsigned char raw_cost = costmap.getCost(cell.mx, cell.my);
  const double normalized_cost =
    raw_cost == nav2_costmap_2d::NO_INFORMATION ? 1.0 :
    static_cast<double>(raw_cost) /
    static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  double centerline_penalty = 0.0;
  if (config_.centerline_weight > 0.0) {
    const double normalized_bonus = std::max(0.0, normalized_clearance_bonus);
    centerline_penalty =
      config_.centerline_weight / (1.0 + std::log1p(normalized_bonus));
  }

  return config_.clearance_weight * normalized_clearance_deficit * normalized_clearance_deficit +
         config_.cost_weight * normalized_cost * normalized_cost -
         0.0 + centerline_penalty;
}

double RouteClearancePlannerCore::turnPenalty(
  const std::vector<int> & parents,
  const GridCell & from,
  const GridCell & to,
  int from_index,
  unsigned int size_x) const
{
  if (config_.turn_weight <= 0.0 || from_index < 0) {
    return 0.0;
  }
  const int parent_index = parents[static_cast<size_t>(from_index)];
  if (parent_index < 0 || parent_index == from_index) {
    return 0.0;
  }
  const GridCell parent{
    static_cast<unsigned int>(parent_index % static_cast<int>(size_x)),
    static_cast<unsigned int>(parent_index / static_cast<int>(size_x))};
  const double v1x = static_cast<double>(from.mx) - static_cast<double>(parent.mx);
  const double v1y = static_cast<double>(from.my) - static_cast<double>(parent.my);
  const double v2x = static_cast<double>(to.mx) - static_cast<double>(from.mx);
  const double v2y = static_cast<double>(to.my) - static_cast<double>(from.my);
  const double len1 = std::hypot(v1x, v1y);
  const double len2 = std::hypot(v2x, v2y);
  if (len1 <= kEpsilon || len2 <= kEpsilon) {
    return 0.0;
  }
  const double dot = std::max(-1.0, std::min(1.0, (v1x * v2x + v1y * v2y) / (len1 * len2)));
  const double angle = std::acos(dot);
  return config_.turn_weight * angle / M_PI;
}

bool RouteClearancePlannerCore::hasLineOfSight(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<double> & clearance_map,
  const GridCell & start,
  const GridCell & goal) const
{
  int x0 = static_cast<int>(start.mx);
  int y0 = static_cast<int>(start.my);
  const int x1 = static_cast<int>(goal.mx);
  const int y1 = static_cast<int>(goal.my);
  const int dx = std::abs(x1 - x0);
  const int dy = std::abs(y1 - y0);
  const int sx = x0 < x1 ? 1 : -1;
  const int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;

  while (true) {
    if (!isCellTraversable(costmap, clearance_map, x0, y0)) {
      return false;
    }
    if (x0 == x1 && y0 == y1) {
      return true;
    }
    const int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }
}

double RouteClearancePlannerCore::lineTraversalCost(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<double> & clearance_map,
  const GridCell & start,
  const GridCell & goal) const
{
  int x0 = static_cast<int>(start.mx);
  int y0 = static_cast<int>(start.my);
  const int x1 = static_cast<int>(goal.mx);
  const int y1 = static_cast<int>(goal.my);
  const int dx = std::abs(x1 - x0);
  const int dy = std::abs(y1 - y0);
  const int sx = x0 < x1 ? 1 : -1;
  const int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;
  double total = 0.0;
  int count = 0;

  while (true) {
    total += cellTraversalPenalty(
      costmap,
      clearance_map,
      {static_cast<unsigned int>(x0), static_cast<unsigned int>(y0)});
    ++count;
    if (x0 == x1 && y0 == y1) {
      break;
    }
    const int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }

  const double distance = gridDistance(start.mx, start.my, goal.mx, goal.my);
  const double average_penalty = count > 0 ? total / static_cast<double>(count) : 0.0;
  return distance * (1.0 + average_penalty);
}

RouteClearancePlannerCore::SearchResult RouteClearancePlannerCore::searchPath(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<double> & clearance_map,
  const GridCell & start,
  const GridCell & goal) const
{
  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();
  const size_t size = static_cast<size_t>(size_x) * static_cast<size_t>(size_y);
  const int start_index = cellIndex(start.mx, start.my, size_x);
  const int goal_index = cellIndex(goal.mx, goal.my, size_x);

  const double endpoint_min_clearance =
    std::min(config_.hard_min_clearance, config_.start_goal_keepout_radius);
  if (!isCellTraversableWithMinClearance(
      costmap,
      clearance_map,
      start.mx,
      start.my,
      endpoint_min_clearance) ||
    !isCellTraversableWithMinClearance(
      costmap,
      clearance_map,
      goal.mx,
      goal.my,
      endpoint_min_clearance))
  {
    return {};
  }

  std::vector<double> g_cost(size, std::numeric_limits<double>::infinity());
  std::vector<int> parents(size, -1);
  std::vector<bool> closed(size, false);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> open;

  g_cost[static_cast<size_t>(start_index)] = 0.0;
  parents[static_cast<size_t>(start_index)] = start_index;
  open.push({start_index, gridDistance(start.mx, start.my, goal.mx, goal.my)});

  const int neighbor_dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int neighbor_dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  while (!open.empty()) {
    const QueueEntry entry = open.top();
    open.pop();
    if (closed[static_cast<size_t>(entry.index)]) {
      continue;
    }
    closed[static_cast<size_t>(entry.index)] = true;
    if (entry.index == goal_index) {
      break;
    }

    const GridCell current{
      static_cast<unsigned int>(entry.index % static_cast<int>(size_x)),
      static_cast<unsigned int>(entry.index / static_cast<int>(size_x))};
    const int parent_index = parents[static_cast<size_t>(entry.index)];
    const GridCell parent{
      static_cast<unsigned int>(parent_index % static_cast<int>(size_x)),
      static_cast<unsigned int>(parent_index / static_cast<int>(size_x))};

    for (int neighbor = 0; neighbor < 8; ++neighbor) {
      const int nx = static_cast<int>(current.mx) + neighbor_dx[neighbor];
      const int ny = static_cast<int>(current.my) + neighbor_dy[neighbor];
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(size_x) || ny >= static_cast<int>(size_y)) {
        continue;
      }
      const GridCell next{static_cast<unsigned int>(nx), static_cast<unsigned int>(ny)};
      const int next_index = cellIndex(next.mx, next.my, size_x);
      if (closed[static_cast<size_t>(next_index)]) {
        continue;
      }
      const bool next_is_goal = next_index == goal_index;
      const bool next_traversable = next_is_goal ?
        isCellTraversableWithMinClearance(
        costmap,
        clearance_map,
        next.mx,
        next.my,
        endpoint_min_clearance) :
        isCellTraversable(costmap, clearance_map, next.mx, next.my);
      if (!next_traversable) {
        continue;
      }

      int candidate_parent_index = entry.index;
      double candidate_g =
        g_cost[static_cast<size_t>(entry.index)] +
        gridDistance(current.mx, current.my, next.mx, next.my) *
        (1.0 + cellTraversalPenalty(costmap, clearance_map, next)) +
        turnPenalty(parents, current, next, entry.index, size_x);

      if (
        parent_index >= 0 &&
        parent_index != entry.index &&
        gridDistance(parent.mx, parent.my, next.mx, next.my) <=
        std::max(4.0, config_.reference_corridor_half_width / costmap.getResolution()) &&
        hasLineOfSight(costmap, clearance_map, parent, next))
      {
        const double theta_g =
          g_cost[static_cast<size_t>(parent_index)] +
          lineTraversalCost(costmap, clearance_map, parent, next) +
          turnPenalty(parents, parent, next, parent_index, size_x);
        if (theta_g < candidate_g) {
          candidate_g = theta_g;
          candidate_parent_index = parent_index;
        }
      }

      if (candidate_g + kEpsilon < g_cost[static_cast<size_t>(next_index)]) {
        g_cost[static_cast<size_t>(next_index)] = candidate_g;
        parents[static_cast<size_t>(next_index)] = candidate_parent_index;
        open.push(
          {
            next_index,
            candidate_g + gridDistance(next.mx, next.my, goal.mx, goal.my),
          });
      }
    }
  }

  if (!std::isfinite(g_cost[static_cast<size_t>(goal_index)])) {
    return {};
  }

  std::vector<GridCell> cells;
  int current_index = goal_index;
  const int max_backtrace = static_cast<int>(size);
  for (int guard = 0; guard < max_backtrace; ++guard) {
    cells.push_back(
      {
        static_cast<unsigned int>(current_index % static_cast<int>(size_x)),
        static_cast<unsigned int>(current_index / static_cast<int>(size_x)),
      });
    if (current_index == start_index) {
      break;
    }
    current_index = parents[static_cast<size_t>(current_index)];
    if (current_index < 0) {
      return {};
    }
  }

  if (cells.empty() || cells.back().mx != start.mx || cells.back().my != start.my) {
    return {};
  }

  std::reverse(cells.begin(), cells.end());
  return {true, cells};
}

std::vector<std::pair<double, double>> RouteClearancePlannerCore::interpolatePoints(
  const std::vector<std::pair<double, double>> & points,
  double resolution) const
{
  if (points.size() < 2U || resolution <= 0.0) {
    return points;
  }

  std::vector<std::pair<double, double>> output;
  output.push_back(points.front());
  for (size_t index = 1; index < points.size(); ++index) {
    const auto & previous = points[index - 1];
    const auto & current = points[index];
    const double distance = euclideanDistance(previous, current);
    const int steps = std::max(1, static_cast<int>(std::ceil(distance / resolution)));
    for (int step = 1; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(steps);
      output.push_back(
        {
          previous.first + (current.first - previous.first) * ratio,
          previous.second + (current.second - previous.second) * ratio,
        });
    }
  }
  return output;
}

nav_msgs::msg::Path RouteClearancePlannerCore::biasPathToRightIfSafe(
  const nav_msgs::msg::Path & path,
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<double> & clearance_map) const
{
  if (
    !config_.right_side_bias ||
    config_.right_side_weight <= kEpsilon ||
    path.poses.size() < 3U)
  {
    return path;
  }

  const double max_offset = std::max(0.0, config_.right_side_max_offset);
  if (max_offset <= kEpsilon) {
    return path;
  }

  const double resolution = std::max(
    costmap.getResolution(),
    resolveInterpolationResolution(costmap));
  const double offset_step = std::max(costmap.getResolution(), max_offset * 0.15);
  const double max_offset_delta = std::max(costmap.getResolution() * 0.25, max_offset * 0.025);
  const double target_clearance = std::max(
    costmap.getResolution(),
    config_.right_side_target_clearance);
  const double probe_distance = std::max(
    target_clearance,
    config_.right_side_probe_distance);
  const double probe_step = std::max(
    costmap.getResolution(),
    target_clearance * 0.1);
  const double right_bias_min_clearance = std::max(
    config_.hard_min_clearance,
    std::min(
      config_.soft_target_clearance,
      std::max(config_.right_side_target_clearance, config_.hard_min_clearance)));
  const auto inflation_boundary_clearance_map = buildInflationBoundaryClearanceMap(costmap);
  const auto clearance_at_cell = [&](unsigned int mx, unsigned int my) -> double {
      const size_t index = static_cast<size_t>(cellIndex(mx, my, costmap.getSizeInCellsX()));
      if (index >= clearance_map.size()) {
        return 0.0;
      }
      return clearance_map[index];
    };
  const auto inflation_boundary_clearance_at_cell =
    [&](unsigned int mx, unsigned int my) -> double {
      const size_t index = static_cast<size_t>(cellIndex(mx, my, costmap.getSizeInCellsX()));
      if (index >= inflation_boundary_clearance_map.size()) {
        return 0.0;
      }
      return inflation_boundary_clearance_map[index];
    };
  const auto has_right_bias_clearance = [&](double wx, double wy) -> bool {
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap.worldToMap(wx, wy, mx, my)) {
        return false;
      }
      if (!isCellTraversable(costmap, clearance_map, mx, my)) {
        return false;
      }
      if (costmap.getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return false;
      }
      return clearance_at_cell(mx, my) + kEpsilon >= right_bias_min_clearance &&
             inflation_boundary_clearance_at_cell(mx, my) + kEpsilon >=
             right_bias_min_clearance;
    };
  const auto measure_boundary_space = [&](double wx, double wy, double direction_x,
      double direction_y) -> double {
      for (double distance = probe_step; distance <= probe_distance + kEpsilon;
        distance += probe_step)
      {
        unsigned int mx = 0;
        unsigned int my = 0;
        const double probe_x = wx + direction_x * distance;
        const double probe_y = wy + direction_y * distance;
        if (!costmap.worldToMap(probe_x, probe_y, mx, my)) {
          return distance;
        }
        const unsigned char cost = costmap.getCost(mx, my);
        const bool boundary = cost == nav2_costmap_2d::NO_INFORMATION ?
          !config_.allow_unknown : cost != nav2_costmap_2d::FREE_SPACE;
        if (boundary) {
          return distance;
        }
      }
      return probe_distance;
    };
  const auto right_side_has_local_space =
    [&](const geometry_msgs::msg::Point & point, const std::pair<double, double> & right) -> bool {
      const double right_space =
        measure_boundary_space(point.x, point.y, right.first, right.second);
      const double left_space =
        measure_boundary_space(point.x, point.y, -right.first, -right.second);
      const double wider_margin = std::max(costmap.getResolution(), target_clearance * 0.20);
      return right_space + kEpsilon >= target_clearance &&
             (right_space + wider_margin >= left_space ||
             (right_space >= probe_distance - kEpsilon &&
             left_space >= probe_distance - kEpsilon));
    };
  std::vector<std::pair<double, double>> right_vectors(path.poses.size(), {0.0, 0.0});
  std::vector<double> safe_offsets(path.poses.size(), 0.0);
  for (size_t index = 1; index + 1 < path.poses.size(); ++index) {
    const auto & previous = path.poses[index - 1].pose.position;
    const auto & current = path.poses[index].pose.position;
    const auto & next = path.poses[index + 1].pose.position;
    const auto right_vector = rightVectorFromTangent(next.x - previous.x, next.y - previous.y);
    if (std::hypot(right_vector.first, right_vector.second) <= kEpsilon) {
      continue;
    }
    right_vectors[index] = right_vector;
    if (!right_side_has_local_space(current, right_vector)) {
      continue;
    }
    for (double offset = max_offset; offset >= resolution - kEpsilon; offset -= offset_step) {
      const double wx = current.x + right_vector.first * offset;
      const double wy = current.y + right_vector.second * offset;
      if (!has_right_bias_clearance(wx, wy)) {
        continue;
      }
      safe_offsets[index] = offset;
      break;
    }
  }

  std::vector<double> offsets = safe_offsets;
  for (size_t index = 1; index + 1 < offsets.size(); ++index) {
    offsets[index] = std::min(offsets[index], offsets[index - 1] + max_offset_delta);
  }
  for (size_t index = offsets.size() - 2; index > 0; --index) {
    offsets[index] = std::min(offsets[index], offsets[index + 1] + max_offset_delta);
  }

  const int smoothing_passes = std::max(1, std::clamp(config_.lateral_smoothing_passes, 0, 8));
  for (int pass = 0; pass < smoothing_passes; ++pass) {
    std::vector<double> smoothed = offsets;
    for (size_t index = 1; index + 1 < offsets.size(); ++index) {
      smoothed[index] = std::min(
        safe_offsets[index],
        offsets[index] * 0.5 + (offsets[index - 1] + offsets[index + 1]) * 0.25);
    }
    offsets = std::move(smoothed);
  }

  std::vector<double> arc_lengths(path.poses.size(), 0.0);
  for (size_t index = 1; index < path.poses.size(); ++index) {
    const auto & previous = path.poses[index - 1].pose.position;
    const auto & current = path.poses[index].pose.position;
    arc_lengths[index] = arc_lengths[index - 1] +
      std::hypot(current.x - previous.x, current.y - previous.y);
  }
  const double total_length = arc_lengths.back();
  const double endpoint_taper_length = std::max(max_offset * 2.0, resolution * 8.0);
  if (total_length > kEpsilon && endpoint_taper_length > kEpsilon) {
    for (size_t index = 1; index + 1 < offsets.size(); ++index) {
      const double endpoint_distance = std::min(
        arc_lengths[index],
        total_length - arc_lengths[index]);
      double factor = std::min(1.0, std::max(0.0, endpoint_distance / endpoint_taper_length));
      factor = factor * factor * (3.0 - 2.0 * factor);
      offsets[index] *= factor;
    }
  }

  nav_msgs::msg::Path shifted = path;
  for (size_t index = 1; index + 1 < shifted.poses.size(); ++index) {
    const auto & current = path.poses[index].pose.position;
    const auto & right = right_vectors[index];
    for (double offset = offsets[index]; offset >= 0.0; offset -= offset_step) {
      const double wx = current.x + right.first * offset;
      const double wy = current.y + right.second * offset;
      if (!has_right_bias_clearance(wx, wy)) {
        continue;
      }
      shifted.poses[index].pose.position.x = wx;
      shifted.poses[index].pose.position.y = wy;
      break;
    }
  }

  const int geometry_smoothing_passes =
    std::max(4, std::clamp(config_.lateral_smoothing_passes + 6, 4, 14));
  constexpr double geometry_smoothing_alpha = 0.45;
  for (int pass = 0; pass < geometry_smoothing_passes; ++pass) {
    nav_msgs::msg::Path smoothed = shifted;
    for (size_t index = 1; index + 1 < shifted.poses.size(); ++index) {
      const auto & previous = shifted.poses[index - 1].pose.position;
      const auto & current = shifted.poses[index].pose.position;
      const auto & next = shifted.poses[index + 1].pose.position;
      const double midpoint_x = (previous.x + next.x) * 0.5;
      const double midpoint_y = (previous.y + next.y) * 0.5;
      const double wx =
        current.x * (1.0 - geometry_smoothing_alpha) +
        midpoint_x * geometry_smoothing_alpha;
      const double wy =
        current.y * (1.0 - geometry_smoothing_alpha) +
        midpoint_y * geometry_smoothing_alpha;
      if (!has_right_bias_clearance(wx, wy)) {
        continue;
      }
      smoothed.poses[index].pose.position.x = wx;
      smoothed.poses[index].pose.position.y = wy;
    }
    shifted = std::move(smoothed);
  }

  const auto has_hard_clearance = [&](double wx, double wy) -> bool {
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap.worldToMap(wx, wy, mx, my)) {
        return false;
      }
      return isCellTraversable(costmap, clearance_map, mx, my);
    };
  const auto keeps_smoothing_clearance = [&](
    const geometry_msgs::msg::Point & current,
    double wx,
    double wy) -> bool
    {
      if (!has_hard_clearance(wx, wy)) {
        return false;
      }
      (void)current;
      return true;
    };
  const auto segment_has_hard_clearance = [&, resolution](
    const geometry_msgs::msg::Point & from,
    double to_x,
    double to_y) -> bool
    {
      const double distance = std::hypot(to_x - from.x, to_y - from.y);
      const int steps = std::max(1, static_cast<int>(std::ceil(distance / resolution)));
      for (int step = 1; step <= steps; ++step) {
        const double ratio = static_cast<double>(step) / static_cast<double>(steps);
        const double wx = from.x + (to_x - from.x) * ratio;
        const double wy = from.y + (to_y - from.y) * ratio;
        if (!has_hard_clearance(wx, wy)) {
          return false;
        }
      }
      return true;
    };
  constexpr double zigzag_smoothing_alpha = 0.45;
  for (int pass = 0; pass < 4; ++pass) {
    nav_msgs::msg::Path smoothed = shifted;
    for (size_t index = 1; index + 1 < shifted.poses.size(); ++index) {
      const auto & previous = shifted.poses[index - 1].pose.position;
      const auto & current = shifted.poses[index].pose.position;
      const auto & next = shifted.poses[index + 1].pose.position;
      const double midpoint_x = (previous.x + next.x) * 0.5;
      const double midpoint_y = (previous.y + next.y) * 0.5;
      const double wx =
        current.x * (1.0 - zigzag_smoothing_alpha) +
        midpoint_x * zigzag_smoothing_alpha;
      const double wy =
        current.y * (1.0 - zigzag_smoothing_alpha) +
        midpoint_y * zigzag_smoothing_alpha;
      if (!keeps_smoothing_clearance(current, wx, wy)) {
        continue;
      }
      if (!segment_has_hard_clearance(previous, wx, wy) ||
        !segment_has_hard_clearance(next, wx, wy))
      {
        continue;
      }
      smoothed.poses[index].pose.position.x = wx;
      smoothed.poses[index].pose.position.y = wy;
    }
    shifted = std::move(smoothed);
  }
  const auto turn_angle_degrees = [](
    const geometry_msgs::msg::Point & previous,
    const geometry_msgs::msg::Point & current,
    const geometry_msgs::msg::Point & next) -> double
    {
      const double ax = current.x - previous.x;
      const double ay = current.y - previous.y;
      const double bx = next.x - current.x;
      const double by = next.y - current.y;
      const double a_length = std::hypot(ax, ay);
      const double b_length = std::hypot(bx, by);
      if (a_length <= kEpsilon || b_length <= kEpsilon) {
        return 0.0;
      }
      const double dot = std::max(
        -1.0,
        std::min(1.0, (ax * bx + ay * by) / (a_length * b_length)));
      return std::acos(dot) * 180.0 / M_PI;
    };
  constexpr double sharp_turn_smoothing_alpha = 0.80;
  constexpr double sharp_turn_threshold_degrees = 2.5;
  for (int pass = 0; pass < 8; ++pass) {
    nav_msgs::msg::Path smoothed = shifted;
    for (size_t index = 1; index + 1 < shifted.poses.size(); ++index) {
      const auto & previous = shifted.poses[index - 1].pose.position;
      const auto & current = shifted.poses[index].pose.position;
      const auto & next = shifted.poses[index + 1].pose.position;
      if (turn_angle_degrees(previous, current, next) < sharp_turn_threshold_degrees) {
        continue;
      }
      const double midpoint_x = (previous.x + next.x) * 0.5;
      const double midpoint_y = (previous.y + next.y) * 0.5;
      const double wx =
        current.x * (1.0 - sharp_turn_smoothing_alpha) +
        midpoint_x * sharp_turn_smoothing_alpha;
      const double wy =
        current.y * (1.0 - sharp_turn_smoothing_alpha) +
        midpoint_y * sharp_turn_smoothing_alpha;
      if (!keeps_smoothing_clearance(current, wx, wy)) {
        continue;
      }
      if (!segment_has_hard_clearance(previous, wx, wy) ||
        !segment_has_hard_clearance(next, wx, wy))
      {
        continue;
      }
      smoothed.poses[index].pose.position.x = wx;
      smoothed.poses[index].pose.position.y = wy;
    }
    shifted = std::move(smoothed);
  }
  constexpr double endpoint_smoothing_alpha = 0.70;
  for (int pass = 0; pass < 6; ++pass) {
    nav_msgs::msg::Path smoothed = shifted;
    for (size_t index = 1; index + 1 < shifted.poses.size(); ++index) {
      const double endpoint_distance = std::min(
        arc_lengths[index],
        total_length - arc_lengths[index]);
      if (endpoint_distance > endpoint_taper_length) {
        continue;
      }
      const auto & previous = shifted.poses[index - 1].pose.position;
      const auto & current = shifted.poses[index].pose.position;
      const auto & next = shifted.poses[index + 1].pose.position;
      const double midpoint_x = (previous.x + next.x) * 0.5;
      const double midpoint_y = (previous.y + next.y) * 0.5;
      const double wx =
        current.x * (1.0 - endpoint_smoothing_alpha) +
        midpoint_x * endpoint_smoothing_alpha;
      const double wy =
        current.y * (1.0 - endpoint_smoothing_alpha) +
        midpoint_y * endpoint_smoothing_alpha;
      if (!keeps_smoothing_clearance(current, wx, wy)) {
        continue;
      }
      smoothed.poses[index].pose.position.x = wx;
      smoothed.poses[index].pose.position.y = wy;
    }
    shifted = std::move(smoothed);
  }

  for (size_t index = 1; index + 1 < shifted.poses.size(); ++index) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap.worldToMap(
        shifted.poses[index].pose.position.x,
        shifted.poses[index].pose.position.y,
        mx,
        my) ||
      !isCellTraversable(costmap, clearance_map, mx, my))
    {
      return path;
    }
  }

  for (size_t index = 1; index + 1 < shifted.poses.size(); ++index) {
    const auto & current = shifted.poses[index].pose.position;
    const auto & next = shifted.poses[index + 1].pose.position;
    const double dx = next.x - current.x;
    const double dy = next.y - current.y;
    if (std::hypot(dx, dy) > kEpsilon) {
      shifted.poses[index].pose.orientation =
        nav2_util::geometry_utils::orientationAroundZAxis(std::atan2(dy, dx));
    }
  }
  return shifted;
}

nav_msgs::msg::Path RouteClearancePlannerCore::downsampleOutputPath(
  const nav_msgs::msg::Path & dense_path,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & effective_goal) const
{
  const double output_resolution = std::max(0.0, config_.output_path_resolution);
  if (output_resolution <= kEpsilon || dense_path.poses.size() < 3U) {
    return dense_path;
  }

  nav_msgs::msg::Path path;
  path.header = dense_path.header;
  path.poses.reserve(dense_path.poses.size());
  path.poses.push_back(dense_path.poses.front());

  std::pair<double, double> last_kept{
    dense_path.poses.front().pose.position.x,
    dense_path.poses.front().pose.position.y};
  for (size_t index = 1; index + 1 < dense_path.poses.size(); ++index) {
    const std::pair<double, double> current{
      dense_path.poses[index].pose.position.x,
      dense_path.poses[index].pose.position.y};
    if (euclideanDistance(last_kept, current) + kEpsilon >= output_resolution) {
      path.poses.push_back(dense_path.poses[index]);
      last_kept = current;
    }
  }
  path.poses.push_back(dense_path.poses.back());

  if (path.poses.size() == 1U) {
    path.poses.front().pose.orientation = effective_goal.pose.orientation;
    return path;
  }

  for (size_t index = 0; index + 1 < path.poses.size(); ++index) {
    size_t next_index = index + 1;
    while (
      next_index < path.poses.size() &&
      std::hypot(
        path.poses[next_index].pose.position.x - path.poses[index].pose.position.x,
        path.poses[next_index].pose.position.y - path.poses[index].pose.position.y) <= kEpsilon)
    {
      ++next_index;
    }
    if (next_index >= path.poses.size()) {
      break;
    }
    const double dx =
      path.poses[next_index].pose.position.x - path.poses[index].pose.position.x;
    const double dy =
      path.poses[next_index].pose.position.y - path.poses[index].pose.position.y;
    path.poses[index].pose.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(std::atan2(dy, dx));
  }

  path.poses.front().pose.orientation = start.pose.orientation;
  if (config_.use_final_goal_orientation) {
    path.poses.back().pose.orientation = effective_goal.pose.orientation;
  } else if (path.poses.size() > 1U) {
    path.poses.back().pose.orientation =
      path.poses[path.poses.size() - 2U].pose.orientation;
  }
  return path;
}

nav_msgs::msg::Path RouteClearancePlannerCore::buildPath(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<double> & clearance_map,
  const std::vector<GridCell> & cells,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & effective_goal,
  const std::string & frame_id) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  if (cells.empty()) {
    return path;
  }

  std::vector<GridCell> simplified_cells;
  simplified_cells.reserve(cells.size());
  size_t anchor_index = 0;
  simplified_cells.push_back(cells.front());
  while (anchor_index + 1 < cells.size()) {
    size_t next_index = cells.size() - 1;
    while (next_index > anchor_index + 1 &&
      !hasLineOfSight(costmap, clearance_map, cells[anchor_index], cells[next_index]))
    {
      --next_index;
    }
    simplified_cells.push_back(cells[next_index]);
    anchor_index = next_index;
  }

  std::vector<std::pair<double, double>> raw_points;
  raw_points.reserve(simplified_cells.size());
  raw_points.push_back({start.pose.position.x, start.pose.position.y});
  for (size_t index = 1; index + 1 < simplified_cells.size(); ++index) {
    double wx = 0.0;
    double wy = 0.0;
    costmap.mapToWorld(simplified_cells[index].mx, simplified_cells[index].my, wx, wy);
    raw_points.push_back({wx, wy});
  }
  raw_points.push_back({effective_goal.pose.position.x, effective_goal.pose.position.y});

  const auto points = interpolatePoints(raw_points, resolveInterpolationResolution(costmap));
  path.poses.reserve(points.size());
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
    path.poses.front().pose.orientation = effective_goal.pose.orientation;
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
  if (config_.use_final_goal_orientation) {
    path.poses.back().pose.orientation = effective_goal.pose.orientation;
  } else if (path.poses.size() > 1U) {
    path.poses.back().pose.orientation =
      path.poses[path.poses.size() - 2U].pose.orientation;
  }
  return path;
}

RouteClearancePlanResult RouteClearancePlannerCore::createPlan(
  const nav2_costmap_2d::Costmap2D & costmap,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & frame_id) const
{
  unsigned int start_mx = 0;
  unsigned int start_my = 0;
  unsigned int goal_mx = 0;
  unsigned int goal_my = 0;
  worldToMapChecked(costmap, start, start_mx, start_my, "start");
  worldToMapChecked(costmap, goal, goal_mx, goal_my, "goal");

  RouteClearancePlanResult result;
  result.effective_goal = goal;
  auto try_reference_path =
    [&](const std::vector<std::pair<double, double>> & reference_points,
      const geometry_msgs::msg::PoseStamped & effective_goal,
      double corridor_half_width_override = -1.0) -> nav_msgs::msg::Path
    {
      auto planning_context = buildPlanningContext(
        costmap,
        reference_points,
        start,
        effective_goal,
        corridor_half_width_override);
      const auto local_clearance_map = buildClearanceMap(planning_context.costmap);
      return buildOptimizedPathFromReference(
        planning_context,
        local_clearance_map,
        start,
        effective_goal,
        frame_id);
    };
  const double endpoint_min_clearance =
    std::min(config_.hard_min_clearance, config_.start_goal_keepout_radius);
  auto is_cell_traversable_locally =
    [&](unsigned int source_mx, unsigned int source_my, double min_clearance) -> bool {
      const unsigned int size_x = costmap.getSizeInCellsX();
      const unsigned int size_y = costmap.getSizeInCellsY();
      const int radius_cells = std::max(
        1,
        static_cast<int>(
          std::ceil(
            (std::max(0.0, min_clearance) + costmap.getResolution() * 2.0) /
            costmap.getResolution())));
      const unsigned int min_mx = source_mx > static_cast<unsigned int>(radius_cells) ?
        source_mx - static_cast<unsigned int>(radius_cells) : 0U;
      const unsigned int min_my = source_my > static_cast<unsigned int>(radius_cells) ?
        source_my - static_cast<unsigned int>(radius_cells) : 0U;
      const unsigned int max_mx = std::min(
        size_x - 1U,
        source_mx + static_cast<unsigned int>(radius_cells));
      const unsigned int max_my = std::min(
        size_y - 1U,
        source_my + static_cast<unsigned int>(radius_cells));
      const unsigned int width = max_mx - min_mx + 1U;
      const unsigned int height = max_my - min_my + 1U;

      nav2_costmap_2d::Costmap2D local_costmap(
        width,
        height,
        costmap.getResolution(),
        costmap.getOriginX() + static_cast<double>(min_mx) * costmap.getResolution(),
        costmap.getOriginY() + static_cast<double>(min_my) * costmap.getResolution(),
        nav2_costmap_2d::FREE_SPACE);
      for (unsigned int my = 0; my < height; ++my) {
        for (unsigned int mx = 0; mx < width; ++mx) {
          local_costmap.setCost(mx, my, costmap.getCost(min_mx + mx, min_my + my));
        }
      }
      const auto local_clearance_map = buildClearanceMap(local_costmap);
      return isCellTraversableWithMinClearance(
        local_costmap,
        local_clearance_map,
        source_mx - min_mx,
        source_my - min_my,
        min_clearance);
    };
  auto is_segment_clearance_safe_locally =
    [&](const geometry_msgs::msg::PoseStamped & segment_start,
      const geometry_msgs::msg::PoseStamped & segment_goal) -> bool
    {
      const double local_margin = std::max(
        config_.hard_min_clearance + costmap.getResolution() * 2.0,
        costmap.getResolution() * 2.0);
      const double min_wx =
        std::min(segment_start.pose.position.x, segment_goal.pose.position.x) - local_margin;
      const double min_wy =
        std::min(segment_start.pose.position.y, segment_goal.pose.position.y) - local_margin;
      const double max_wx =
        std::max(segment_start.pose.position.x, segment_goal.pose.position.x) + local_margin;
      const double max_wy =
        std::max(segment_start.pose.position.y, segment_goal.pose.position.y) + local_margin;
      auto clamp_to_index = [](int value, unsigned int limit) -> unsigned int {
          if (value < 0) {
            return 0U;
          }
          return std::min(limit - 1U, static_cast<unsigned int>(value));
        };
      const unsigned int min_mx = clamp_to_index(
        static_cast<int>(std::floor((min_wx - costmap.getOriginX()) / costmap.getResolution())),
        costmap.getSizeInCellsX());
      const unsigned int min_my = clamp_to_index(
        static_cast<int>(std::floor((min_wy - costmap.getOriginY()) / costmap.getResolution())),
        costmap.getSizeInCellsY());
      const unsigned int max_mx = clamp_to_index(
        static_cast<int>(std::ceil((max_wx - costmap.getOriginX()) / costmap.getResolution())),
        costmap.getSizeInCellsX());
      const unsigned int max_my = clamp_to_index(
        static_cast<int>(std::ceil((max_wy - costmap.getOriginY()) / costmap.getResolution())),
        costmap.getSizeInCellsY());
      const unsigned int width = max_mx - min_mx + 1U;
      const unsigned int height = max_my - min_my + 1U;
      if (static_cast<size_t>(width) * static_cast<size_t>(height) > 500000U) {
        return false;
      }

      nav2_costmap_2d::Costmap2D local_costmap(
        width,
        height,
        costmap.getResolution(),
        costmap.getOriginX() + static_cast<double>(min_mx) * costmap.getResolution(),
        costmap.getOriginY() + static_cast<double>(min_my) * costmap.getResolution(),
        nav2_costmap_2d::FREE_SPACE);
      for (unsigned int my = 0; my < height; ++my) {
        for (unsigned int mx = 0; mx < width; ++mx) {
          local_costmap.setCost(mx, my, costmap.getCost(min_mx + mx, min_my + my));
        }
      }

      const auto local_clearance_map = buildClearanceMap(local_costmap);
      const double resolution = resolveInterpolationResolution(costmap);
      const std::pair<double, double> start_point{
        segment_start.pose.position.x, segment_start.pose.position.y};
      const std::pair<double, double> goal_point{
        segment_goal.pose.position.x, segment_goal.pose.position.y};
      const double distance = euclideanDistance(start_point, goal_point);
      const int steps = std::max(1, static_cast<int>(std::ceil(distance / resolution)));
      for (int index = 0; index <= steps; ++index) {
        const double ratio = static_cast<double>(index) / static_cast<double>(steps);
        const double wx = start_point.first + (goal_point.first - start_point.first) * ratio;
        const double wy = start_point.second + (goal_point.second - start_point.second) * ratio;
        unsigned int local_mx = 0;
        unsigned int local_my = 0;
        if (!local_costmap.worldToMap(wx, wy, local_mx, local_my)) {
          return false;
        }
        if (!isCellTraversable(local_costmap, local_clearance_map, local_mx, local_my)) {
          return false;
        }
      }
      return true;
    };
  auto estimate_segment_bbox_cells =
    [&](double margin) -> size_t {
      const double width_m =
        std::abs(goal.pose.position.x - start.pose.position.x) + margin * 2.0;
      const double height_m =
        std::abs(goal.pose.position.y - start.pose.position.y) + margin * 2.0;
      const auto width_cells = static_cast<size_t>(
        std::ceil(width_m / std::max(costmap.getResolution(), kEpsilon)));
      const auto height_cells = static_cast<size_t>(
        std::ceil(height_m / std::max(costmap.getResolution(), kEpsilon)));
      return width_cells * height_cells;
    };

  if (config_.pose_directed_crop_enabled) {
    const auto pose_reference_points = buildPoseDirectedReferencePoints(costmap, start, goal);
    std::vector<double> corridor_widths{
      std::max(0.0, config_.reference_corridor_half_width)};
    const double max_corridor_half_width =
      std::max(0.0, config_.pose_directed_max_corridor_half_width);
    if (max_corridor_half_width > corridor_widths.front() + kEpsilon) {
      corridor_widths.push_back(max_corridor_half_width);
    }

    try {
      for (const double corridor_width : corridor_widths) {
        if (estimate_segment_bbox_cells(corridor_width + config_.start_goal_keepout_radius) >
          1000000U)
        {
          continue;
        }
        result.path = try_reference_path(
          pose_reference_points,
          result.effective_goal,
          corridor_width);
        if (!result.path.poses.empty()) {
          return result;
        }
      }
    } catch (const nav2_core::PlannerException &) {
    }
  }

  if (!is_cell_traversable_locally(start_mx, start_my, endpoint_min_clearance)) {
    throw nav2_core::PlannerException("route clearance planner start pose is not traversable");
  }

  bool exact_goal_attempted = false;
  if (is_cell_traversable_locally(goal_mx, goal_my, endpoint_min_clearance)) {
    exact_goal_attempted = true;
    try {
      const std::vector<double> no_global_clearance_map;
      const auto reference_points = is_segment_clearance_safe_locally(start, goal) ?
        std::vector<std::pair<double, double>>{
        {start.pose.position.x, start.pose.position.y},
        {goal.pose.position.x, goal.pose.position.y}} :
      buildReferencePathPoints(
        costmap,
        no_global_clearance_map,
        start,
        goal);
      result.path = try_reference_path(reference_points, result.effective_goal);
      if (!result.path.poses.empty()) {
        return result;
      }
    } catch (const nav2_core::PlannerException &) {
    }
  }

  double global_clearance_lookup_ms = 0.0;
  const auto & clearance_map = getGlobalClearanceMap(costmap, global_clearance_lookup_ms);
  (void) global_clearance_lookup_ms;
  if (!isCellTraversableWithMinClearance(
      costmap,
      clearance_map,
      start_mx,
      start_my,
      endpoint_min_clearance))
  {
    throw nav2_core::PlannerException("route clearance planner start pose is not traversable");
  }

  PlanningContext planning_context;
  const GridCell requested_goal_world{goal_mx, goal_my};
  if (!exact_goal_attempted && isCellTraversableWithMinClearance(
      costmap,
      clearance_map,
      goal_mx,
      goal_my,
      endpoint_min_clearance))
  {
    std::vector<std::pair<double, double>> reference_points;
    if (isSegmentClearanceSafe(costmap, clearance_map, start, goal)) {
      reference_points = {
        {start.pose.position.x, start.pose.position.y},
        {goal.pose.position.x, goal.pose.position.y}};
    } else {
      reference_points = buildReferencePathPoints(costmap, clearance_map, start, goal);
    }
    planning_context = buildPlanningContext(costmap, reference_points, start, goal);
    const auto local_clearance_map = buildClearanceMap(planning_context.costmap);
    result.path = buildOptimizedPathFromReference(
      planning_context,
      local_clearance_map,
      start,
      result.effective_goal,
      frame_id);
    if (!result.path.poses.empty()) {
      return result;
    }
  }

  std::vector<GridCell> goal_candidates;
  const auto nearby_candidates = buildGoalCandidates(
    costmap, clearance_map, goal_mx, goal_my);
  for (const auto & candidate : nearby_candidates) {
    if (candidate.mx == goal_mx && candidate.my == goal_my) {
      continue;
    }
    goal_candidates.push_back(candidate);
  }

  if (goal_candidates.empty()) {
    throw nav2_core::PlannerException(
            "route clearance planner could not find a safe goal candidate");
  }

  SearchResult best_result;
  GridCell best_goal = goal_candidates.front();
  double best_score = std::numeric_limits<double>::infinity();
  PlanningContext best_context;
  std::vector<double> best_clearance_map;
  for (const auto & candidate : goal_candidates) {
    geometry_msgs::msg::PoseStamped candidate_goal = goal;
    costmap.mapToWorld(
      candidate.mx,
      candidate.my,
      candidate_goal.pose.position.x,
      candidate_goal.pose.position.y);

    std::vector<std::pair<double, double>> reference_points;
    if (isSegmentClearanceSafe(costmap, clearance_map, start, candidate_goal)) {
      reference_points = {
        {start.pose.position.x, start.pose.position.y},
        {candidate_goal.pose.position.x, candidate_goal.pose.position.y}};
    } else {
      reference_points = buildReferencePathPoints(
        costmap,
        clearance_map,
        start,
        candidate_goal);
    }
    auto candidate_context = buildPlanningContext(
      costmap, reference_points, start, candidate_goal);
    const auto candidate_clearance_map = buildClearanceMap(candidate_context.costmap);
    const double goal_distance = gridDistance(
      candidate.mx,
      candidate.my,
      requested_goal_world.mx,
      requested_goal_world.my) * costmap.getResolution();
    const double score =
      goal_distance * 100.0;
    if (score < best_score) {
      best_score = score;
      best_goal = candidate;
      best_context = std::move(candidate_context);
      best_clearance_map = candidate_clearance_map;
    }
  }

  if (best_clearance_map.empty()) {
    throw nav2_core::PlannerException("route clearance planner could not find a path");
  }

  result.adjusted_goal = true;
  costmap.mapToWorld(
    best_goal.mx,
    best_goal.my,
    result.effective_goal.pose.position.x,
    result.effective_goal.pose.position.y);
  result.effective_goal.pose.position.z = goal.pose.position.z;
  result.effective_goal.pose.orientation = goal.pose.orientation;
  result.path = buildOptimizedPathFromReference(
    best_context,
    best_clearance_map,
    start,
    result.effective_goal,
    frame_id);
  return result;
}

}  // namespace nav2_route_polyline_planner
