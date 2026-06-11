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
  const auto clearance_map = buildClearanceMap(costmap);
  return clearance_map[static_cast<size_t>(cellIndex(mx, my, costmap.getSizeInCellsX()))];
}

bool RouteClearancePlannerCore::isCellTraversable(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<double> & clearance_map,
  unsigned int mx,
  unsigned int my) const
{
  if (!isCostTraversable(costmap.getCost(mx, my))) {
    return false;
  }
  const double clearance =
    clearance_map[static_cast<size_t>(cellIndex(mx, my, costmap.getSizeInCellsX()))];
  return clearance + kEpsilon >= std::max(0.0, config_.hard_min_clearance);
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
  unsigned int start_mx = 0;
  unsigned int start_my = 0;
  if (!costmap.worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my)) {
    throw nav2_core::PlannerException("route clearance planner start pose is outside costmap");
  }

  unsigned int goal_mx = 0;
  unsigned int goal_my = 0;
  if (!costmap.worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my)) {
    throw nav2_core::PlannerException("route clearance planner goal pose is outside costmap");
  }

  auto clamp_to_index = [](int value, unsigned int limit) -> unsigned int {
      if (value < 0) {
        return 0U;
      }
      const unsigned int cast = static_cast<unsigned int>(value);
      return std::min(limit - 1U, cast);
    };

  auto run_navfn =
    [&](unsigned int min_mx,
      unsigned int min_my,
      unsigned int max_mx,
      unsigned int max_my,
      std::vector<std::pair<double, double>> & points) -> bool
    {
      const unsigned int width = max_mx - min_mx + 1U;
      const unsigned int height = max_my - min_my + 1U;
      nav2_navfn_planner::NavFn planner(
        static_cast<int>(width),
        static_cast<int>(height));
      std::vector<unsigned char> reference_costmap_data(
        static_cast<size_t>(width) * static_cast<size_t>(height),
        nav2_costmap_2d::FREE_SPACE);
      for (unsigned int local_my = 0; local_my < height; ++local_my) {
        for (unsigned int local_mx = 0; local_mx < width; ++local_mx) {
          const unsigned int source_mx = min_mx + local_mx;
          const unsigned int source_my = min_my + local_my;
          const size_t local_index =
            static_cast<size_t>(local_my) * static_cast<size_t>(width) + local_mx;
          reference_costmap_data[local_index] = costmap.getCharMap()[
            static_cast<size_t>(costmap.getIndex(source_mx, source_my))];
          if (!isCellTraversable(costmap, clearance_map, source_mx, source_my)) {
            reference_costmap_data[local_index] = nav2_costmap_2d::LETHAL_OBSTACLE;
          }
        }
      }
      const unsigned int local_start_mx = start_mx - min_mx;
      const unsigned int local_start_my = start_my - min_my;
      const unsigned int local_goal_mx = goal_mx - min_mx;
      const unsigned int local_goal_my = goal_my - min_my;
      reference_costmap_data[
        static_cast<size_t>(local_start_my) * static_cast<size_t>(width) +
        local_start_mx] = nav2_costmap_2d::FREE_SPACE;
      reference_costmap_data[
        static_cast<size_t>(local_goal_my) * static_cast<size_t>(width) +
        local_goal_mx] = nav2_costmap_2d::FREE_SPACE;

      planner.setCostmap(
        reference_costmap_data.data(),
        true,
        config_.reference_allow_unknown);

      int map_start[2] = {
        static_cast<int>(local_start_mx),
        static_cast<int>(local_start_my)};
      int map_goal[2] = {
        static_cast<int>(local_goal_mx),
        static_cast<int>(local_goal_my)};
      planner.setStart(map_goal);
      planner.setGoal(map_start);
      if (config_.reference_use_astar) {
        planner.calcNavFnAstar();
      } else {
        planner.calcNavFnDijkstra(true);
      }

      const unsigned int goal_index =
        local_goal_my * static_cast<unsigned int>(planner.nx) + local_goal_mx;
      if (planner.potarr[goal_index] >= POT_HIGH) {
        return false;
      }

      planner.setStart(map_goal);
      const int max_cycles =
        static_cast<int>(std::max(width, height) * 4U);
      if (planner.calcPath(max_cycles) == 0 || planner.getPathLen() == 0) {
        return false;
      }

      float * path_x = planner.getPathX();
      float * path_y = planner.getPathY();
      const int path_length = planner.getPathLen();
      const double min_distance = std::max(costmap.getResolution() * 2.0, 0.25);

      points.clear();
      appendUniquePoint(
        points,
        {start.pose.position.x, start.pose.position.y},
        min_distance);

      for (int index = path_length - 1; index >= 0; --index) {
        const double wx =
          costmap.getOriginX() +
          static_cast<double>(min_mx + path_x[index]) * costmap.getResolution();
        const double wy =
          costmap.getOriginY() +
          static_cast<double>(min_my + path_y[index]) * costmap.getResolution();
        appendUniquePoint(points, {wx, wy}, min_distance);
      }

      const std::pair<double, double> goal_point{
        goal.pose.position.x, goal.pose.position.y};
      if (!points.empty() && euclideanDistance(points.back(), goal_point) <= min_distance) {
        points.back() = goal_point;
      } else {
        points.push_back(goal_point);
      }
      return true;
    };

  std::vector<std::pair<double, double>> points;
  if (config_.reference_search_margin > kEpsilon) {
    const double search_margin =
      std::max(config_.reference_search_margin, config_.hard_min_clearance);
    const int margin_cells = static_cast<int>(std::ceil(search_margin / costmap.getResolution()));
    const unsigned int local_min_mx = clamp_to_index(
      static_cast<int>(std::min(start_mx, goal_mx)) - margin_cells,
      costmap.getSizeInCellsX());
    const unsigned int local_min_my = clamp_to_index(
      static_cast<int>(std::min(start_my, goal_my)) - margin_cells,
      costmap.getSizeInCellsY());
    const unsigned int local_max_mx = clamp_to_index(
      static_cast<int>(std::max(start_mx, goal_mx)) + margin_cells,
      costmap.getSizeInCellsX());
    const unsigned int local_max_my = clamp_to_index(
      static_cast<int>(std::max(start_my, goal_my)) + margin_cells,
      costmap.getSizeInCellsY());
    const bool local_contains_endpoints =
      local_min_mx <= start_mx && start_mx <= local_max_mx &&
      local_min_my <= start_my && start_my <= local_max_my &&
      local_min_mx <= goal_mx && goal_mx <= local_max_mx &&
      local_min_my <= goal_my && goal_my <= local_max_my;
    if (
      local_contains_endpoints &&
      run_navfn(local_min_mx, local_min_my, local_max_mx, local_max_my, points))
    {
      return points;
    }
  }

  if (
    !run_navfn(
      0U,
      0U,
      costmap.getSizeInCellsX() - 1U,
      costmap.getSizeInCellsY() - 1U,
      points))
  {
    throw nav2_core::PlannerException(
            "route clearance planner could not find a coarse reference path");
  }
  return points;
}

RouteClearancePlannerCore::PlanningContext RouteClearancePlannerCore::buildPlanningContext(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<std::pair<double, double>> & reference_points,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  unsigned int start_mx = 0;
  unsigned int start_my = 0;
  unsigned int goal_mx = 0;
  unsigned int goal_my = 0;
  worldToMapChecked(costmap, start, start_mx, start_my, "start");
  worldToMapChecked(costmap, goal, goal_mx, goal_my, "goal");

  const double margin = std::max(
    config_.reference_corridor_half_width + config_.start_goal_keepout_radius,
    costmap.getResolution() * 2.0);

  double min_wx = std::min(start.pose.position.x, goal.pose.position.x);
  double min_wy = std::min(start.pose.position.y, goal.pose.position.y);
  double max_wx = std::max(start.pose.position.x, goal.pose.position.x);
  double max_wy = std::max(start.pose.position.y, goal.pose.position.y);

  std::vector<std::pair<double, double>> sampled_reference;
  const double sample_min_distance = std::max(
    costmap.getResolution() * 2.0,
    std::max(0.2, config_.reference_corridor_half_width * 0.25));
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
  const double corridor_half_width = std::max(0.0, config_.reference_corridor_half_width);

  auto copy_source_cost = [&](unsigned int source_mx, unsigned int source_my) {
      cropped_costmap.setCost(
        source_mx - min_mx,
        source_my - min_my,
        costmap.getCharMap()[costmap.getIndex(source_mx, source_my)]);
    };

  auto mark_disc = [&](const std::pair<double, double> & center, double radius) {
      if (radius <= kEpsilon) {
        unsigned int center_mx = 0;
        unsigned int center_my = 0;
        if (costmap.worldToMap(center.first, center.second, center_mx, center_my)) {
          center_mx = std::min(max_mx, std::max(min_mx, center_mx));
          center_my = std::min(max_my, std::max(min_my, center_my));
          copy_source_cost(center_mx, center_my);
        }
        return;
      }

      const unsigned int disc_min_mx = clamp_to_index(
        static_cast<int>(std::floor((center.first - radius - origin_x) / resolution)),
        size_x);
      const unsigned int disc_min_my = clamp_to_index(
        static_cast<int>(std::floor((center.second - radius - origin_y) / resolution)),
        size_y);
      const unsigned int disc_max_mx = clamp_to_index(
        static_cast<int>(std::ceil((center.first + radius - origin_x) / resolution)),
        size_x);
      const unsigned int disc_max_my = clamp_to_index(
        static_cast<int>(std::ceil((center.second + radius - origin_y) / resolution)),
        size_y);
      for (unsigned int source_my = std::max(min_my, disc_min_my);
        source_my <= std::min(max_my, disc_max_my);
        ++source_my)
      {
        for (unsigned int source_mx = std::max(min_mx, disc_min_mx);
          source_mx <= std::min(max_mx, disc_max_mx);
          ++source_mx)
        {
          double wx = 0.0;
          double wy = 0.0;
          costmap.mapToWorld(source_mx, source_my, wx, wy);
          if (euclideanDistance({wx, wy}, center) <= radius + kEpsilon) {
            copy_source_cost(source_mx, source_my);
          }
        }
      }
    };

  auto mark_segment_corridor =
    [&](const std::pair<double, double> & segment_start,
      const std::pair<double, double> & segment_end)
    {
      const double min_segment_wx =
        std::min(segment_start.first, segment_end.first) - corridor_half_width;
      const double min_segment_wy =
        std::min(segment_start.second, segment_end.second) - corridor_half_width;
      const double max_segment_wx =
        std::max(segment_start.first, segment_end.first) + corridor_half_width;
      const double max_segment_wy =
        std::max(segment_start.second, segment_end.second) + corridor_half_width;
      const unsigned int segment_min_mx = clamp_to_index(
        static_cast<int>(std::floor((min_segment_wx - origin_x) / resolution)),
        size_x);
      const unsigned int segment_min_my = clamp_to_index(
        static_cast<int>(std::floor((min_segment_wy - origin_y) / resolution)),
        size_y);
      const unsigned int segment_max_mx = clamp_to_index(
        static_cast<int>(std::ceil((max_segment_wx - origin_x) / resolution)),
        size_x);
      const unsigned int segment_max_my = clamp_to_index(
        static_cast<int>(std::ceil((max_segment_wy - origin_y) / resolution)),
        size_y);

      for (unsigned int source_my = std::max(min_my, segment_min_my);
        source_my <= std::min(max_my, segment_max_my);
        ++source_my)
      {
        for (unsigned int source_mx = std::max(min_mx, segment_min_mx);
          source_mx <= std::min(max_mx, segment_max_mx);
          ++source_mx)
        {
          double wx = 0.0;
          double wy = 0.0;
          costmap.mapToWorld(source_mx, source_my, wx, wy);
          if (
            pointToSegmentDistance({wx, wy}, segment_start, segment_end) <=
            corridor_half_width + kEpsilon)
          {
            copy_source_cost(source_mx, source_my);
          }
        }
      }
    };

  mark_disc(start_point, keepout_radius);
  mark_disc(goal_point, keepout_radius);
  if (sampled_reference.size() == 1U) {
    mark_disc(sampled_reference.front(), corridor_half_width);
  } else {
    for (size_t index = 1; index < sampled_reference.size(); ++index) {
      mark_segment_corridor(sampled_reference[index - 1], sampled_reference[index]);
    }
  }

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
      for (const auto & pose : path.poses) {
        unsigned int mx = 0;
        unsigned int my = 0;
        if (
          !planning_context.costmap.worldToMap(
            pose.pose.position.x,
            pose.pose.position.y,
            mx,
            my) ||
          !isCellTraversable(planning_context.costmap, clearance_map, mx, my))
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
  const double lateral_limit = std::max(
    planning_context.costmap.getResolution(),
    config_.reference_corridor_half_width * 0.5);
  const double lateral_step = std::max(
    planning_context.costmap.getResolution(),
    config_.reference_corridor_half_width * 0.1);
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
        if (!isCellTraversable(planning_context.costmap, clearance_map, mx, my)) {
          return false;
        }
      }
      return true;
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
    const std::pair<double, double> next_reference_target =
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
      if (!segment_is_valid({wx, wy}, next_reference_target)) {
        continue;
      }

      const double clearance = clearance_map[static_cast<size_t>(cellIndex(
            mx,
            my,
            planning_context.costmap.getSizeInCellsX()))];
      const double clearance_deficit = std::max(0.0, config_.soft_target_clearance - clearance);
      const double lateral_penalty = std::abs(offset);
      const double lateral_change_penalty = std::abs(offset - previous_offset);
      const double raw_cost = static_cast<double>(planning_context.costmap.getCost(mx, my));
      const auto & previous_point = optimized_points.back();
      const double incoming_distance = euclideanDistance(previous_point, {wx, wy});
      const double outgoing_distance = euclideanDistance({wx, wy}, next_reference_target);
      const double direct_distance = euclideanDistance(previous_point, next_reference_target);
      const double detour_penalty =
        std::max(0.0, incoming_distance + outgoing_distance - direct_distance);
      double turn_angle = 0.0;
      if (incoming_distance > kEpsilon && outgoing_distance > kEpsilon) {
        const double incoming_x = wx - previous_point.first;
        const double incoming_y = wy - previous_point.second;
        const double outgoing_x = next_reference_target.first - wx;
        const double outgoing_y = next_reference_target.second - wy;
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
        lateral_change_penalty * 1.5 +
        detour_penalty * 4.0 +
        turn_angle * config_.turn_weight * 15.0 +
        raw_cost * config_.cost_weight / 253.0;
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

  auto optimized_path = build_path_from_points(optimized_points);
  if (path_respects_hard_clearance(optimized_path)) {
    return optimized_path;
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
        searched_path.cells,
        start,
        effective_goal,
        frame_id);
      if (path_respects_hard_clearance(safe_path)) {
        return safe_path;
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
  const auto fallback_path = build_path_from_points(fallback_points);
  if (path_respects_hard_clearance(fallback_path)) {
    return fallback_path;
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

  if (!isCellTraversable(costmap, clearance_map, start.mx, start.my) ||
    !isCellTraversable(costmap, clearance_map, goal.mx, goal.my))
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
      if (closed[static_cast<size_t>(next_index)] ||
        !isCellTraversable(costmap, clearance_map, next.mx, next.my))
      {
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

nav_msgs::msg::Path RouteClearancePlannerCore::buildPath(
  const nav2_costmap_2d::Costmap2D & costmap,
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

  std::vector<std::pair<double, double>> raw_points;
  raw_points.reserve(cells.size());
  raw_points.push_back({start.pose.position.x, start.pose.position.y});
  for (size_t index = 1; index + 1 < cells.size(); ++index) {
    double wx = 0.0;
    double wy = 0.0;
    costmap.mapToWorld(cells[index].mx, cells[index].my, wx, wy);
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
  using SteadyClock = std::chrono::steady_clock;
  const auto total_start = SteadyClock::now();
  auto elapsed_ms = [](const SteadyClock::time_point & begin) -> double {
      return std::chrono::duration<double, std::milli>(SteadyClock::now() - begin).count();
    };
  RouteClearanceTiming timing;

  unsigned int start_mx = 0;
  unsigned int start_my = 0;
  unsigned int goal_mx = 0;
  unsigned int goal_my = 0;
  worldToMapChecked(costmap, start, start_mx, start_my, "start");
  worldToMapChecked(costmap, goal, goal_mx, goal_my, "goal");

  double global_clearance_lookup_ms = 0.0;
  const auto & clearance_map = getGlobalClearanceMap(costmap, global_clearance_lookup_ms);
  timing.global_clearance_ms += global_clearance_lookup_ms;
  auto stage_start = SteadyClock::now();
  if (!isCellTraversable(costmap, clearance_map, start_mx, start_my)) {
    throw nav2_core::PlannerException("route clearance planner start pose is not traversable");
  }

  RouteClearancePlanResult result;
  result.effective_goal = goal;
  PlanningContext planning_context;
  const GridCell requested_goal_world{goal_mx, goal_my};
  if (isCellTraversable(costmap, clearance_map, goal_mx, goal_my)) {
    std::vector<std::pair<double, double>> reference_points;
    stage_start = SteadyClock::now();
    const bool direct_segment_safe = isSegmentClearanceSafe(costmap, clearance_map, start, goal);
    timing.direct_segment_check_ms += elapsed_ms(stage_start);
    if (direct_segment_safe) {
      reference_points = {
        {start.pose.position.x, start.pose.position.y},
        {goal.pose.position.x, goal.pose.position.y}};
    } else {
      stage_start = SteadyClock::now();
      reference_points = buildReferencePathPoints(costmap, clearance_map, start, goal);
      timing.reference_path_ms += elapsed_ms(stage_start);
    }
    stage_start = SteadyClock::now();
    planning_context = buildPlanningContext(costmap, reference_points, start, goal);
    timing.planning_context_ms += elapsed_ms(stage_start);
    stage_start = SteadyClock::now();
    const auto local_clearance_map = buildClearanceMap(planning_context.costmap);
    timing.local_clearance_ms += elapsed_ms(stage_start);
    stage_start = SteadyClock::now();
    result.path = buildOptimizedPathFromReference(
      planning_context,
      local_clearance_map,
      start,
      result.effective_goal,
      frame_id);
    timing.optimize_path_ms += elapsed_ms(stage_start);
    if (!result.path.poses.empty()) {
      timing.total_ms = elapsed_ms(total_start);
      if (config_.debug_timing) {
        result.timing = timing;
      }
      return result;
    }
  }

  std::vector<GridCell> goal_candidates;
  stage_start = SteadyClock::now();
  const auto nearby_candidates = buildGoalCandidates(
    costmap, clearance_map, goal_mx, goal_my);
  timing.goal_candidates_ms += elapsed_ms(stage_start);
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
  stage_start = SteadyClock::now();
  for (const auto & candidate : goal_candidates) {
    geometry_msgs::msg::PoseStamped candidate_goal = goal;
    costmap.mapToWorld(
      candidate.mx,
      candidate.my,
      candidate_goal.pose.position.x,
      candidate_goal.pose.position.y);

    std::vector<std::pair<double, double>> reference_points;
    const auto direct_check_start = SteadyClock::now();
    const bool direct_segment_safe =
      isSegmentClearanceSafe(costmap, clearance_map, start, candidate_goal);
    timing.direct_segment_check_ms += elapsed_ms(direct_check_start);
    if (direct_segment_safe) {
      reference_points = {
        {start.pose.position.x, start.pose.position.y},
        {candidate_goal.pose.position.x, candidate_goal.pose.position.y}};
    } else {
      const auto reference_start = SteadyClock::now();
      reference_points = buildReferencePathPoints(
        costmap,
        clearance_map,
        start,
        candidate_goal);
      timing.reference_path_ms += elapsed_ms(reference_start);
    }
    const auto context_start = SteadyClock::now();
    auto candidate_context = buildPlanningContext(
      costmap, reference_points, start, candidate_goal);
    timing.planning_context_ms += elapsed_ms(context_start);
    const auto clearance_start = SteadyClock::now();
    const auto candidate_clearance_map = buildClearanceMap(candidate_context.costmap);
    timing.local_clearance_ms += elapsed_ms(clearance_start);
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
  timing.adjusted_goal_scan_ms += elapsed_ms(stage_start);

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
  stage_start = SteadyClock::now();
  result.path = buildOptimizedPathFromReference(
    best_context,
    best_clearance_map,
    start,
    result.effective_goal,
    frame_id);
  timing.optimize_path_ms += elapsed_ms(stage_start);
  timing.total_ms = elapsed_ms(total_start);
  if (config_.debug_timing) {
    result.timing = timing;
  }
  return result;
}

}  // namespace nav2_route_polyline_planner
