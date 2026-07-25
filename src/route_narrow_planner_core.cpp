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

#include "nav2_route_polyline_planner/route_narrow_planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace nav2_route_polyline_planner
{

namespace
{

constexpr double kEpsilon = 1.0e-6;
constexpr double kInf = std::numeric_limits<double>::infinity();

struct Point
{
  double x{0.0};
  double y{0.0};
};

struct GridCell
{
  unsigned int mx{0};
  unsigned int my{0};
};

struct LocalGrid
{
  unsigned int min_mx{0};
  unsigned int min_my{0};
  unsigned int width{0};
  unsigned int height{0};
};

struct QueueItem
{
  double cost{0.0};
  int index{0};

  bool operator>(const QueueItem & other) const
  {
    return cost > other.cost;
  }
};

double distance(const Point & a, const Point & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

double pointToSegmentDistance(const Point & point, const Point & start, const Point & goal)
{
  const double sx = goal.x - start.x;
  const double sy = goal.y - start.y;
  const double len_sq = sx * sx + sy * sy;
  if (len_sq <= kEpsilon) {
    return distance(point, start);
  }
  const double ratio = std::max(
    0.0,
    std::min(1.0, ((point.x - start.x) * sx + (point.y - start.y) * sy) / len_sq));
  return distance(point, {start.x + sx * ratio, start.y + sy * ratio});
}

void appendUnique(std::vector<Point> & points, const Point & point, double min_distance)
{
  if (points.empty() || distance(points.back(), point) > min_distance) {
    points.push_back(point);
  }
}

bool isObstacleCost(unsigned char cost, bool allow_unknown)
{
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return !allow_unknown;
  }
  return cost >= nav2_costmap_2d::LETHAL_OBSTACLE;
}

bool isInflationTraversable(unsigned char cost, bool allow_unknown)
{
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown;
  }
  return cost < nav2_costmap_2d::LETHAL_OBSTACLE;
}

int localIndex(const LocalGrid & grid, unsigned int mx, unsigned int my)
{
  return static_cast<int>((my - grid.min_my) * grid.width + (mx - grid.min_mx));
}

bool contains(const LocalGrid & grid, unsigned int mx, unsigned int my)
{
  return mx >= grid.min_mx && my >= grid.min_my &&
         mx < grid.min_mx + grid.width && my < grid.min_my + grid.height;
}

double normalizeCost(unsigned char cost)
{
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return 1.0;
  }
  if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
    return 1.0;
  }
  return static_cast<double>(cost) /
         static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}

double yawBetween(const Point & from, const Point & to)
{
  return std::atan2(to.y - from.y, to.x - from.x);
}

std::vector<Point> interpolatePolyline(const std::vector<Point> & points, double resolution)
{
  if (points.size() <= 1U) {
    return points;
  }

  const double step = std::max(resolution, kEpsilon);
  std::vector<Point> result;
  result.push_back(points.front());
  for (size_t index = 1; index < points.size(); ++index) {
    const Point & start = points[index - 1];
    const Point & goal = points[index];
    const double length = distance(start, goal);
    const int count = std::max(1, static_cast<int>(std::ceil(length / step)));
    for (int sample = 1; sample <= count; ++sample) {
      const double ratio = static_cast<double>(sample) / static_cast<double>(count);
      appendUnique(
        result,
        {start.x + (goal.x - start.x) * ratio, start.y + (goal.y - start.y) * ratio},
        step * 0.25);
    }
  }
  return result;
}

}  // namespace

RouteNarrowPlannerCore::RouteNarrowPlannerCore(RouteNarrowPlannerConfig config)
: config_(std::move(config))
{
}

double RouteNarrowPlannerCore::clearanceAt(
  const nav2_costmap_2d::Costmap2D & costmap,
  double wx,
  double wy) const
{
  unsigned int query_mx = 0;
  unsigned int query_my = 0;
  if (!costmap.worldToMap(wx, wy, query_mx, query_my)) {
    return 0.0;
  }

  double best = kInf;
  for (unsigned int my = 0; my < costmap.getSizeInCellsY(); ++my) {
    for (unsigned int mx = 0; mx < costmap.getSizeInCellsX(); ++mx) {
      if (!isObstacleCost(costmap.getCost(mx, my), config_.allow_unknown)) {
        continue;
      }
      double obstacle_wx = 0.0;
      double obstacle_wy = 0.0;
      costmap.mapToWorld(mx, my, obstacle_wx, obstacle_wy);
      best = std::min(best, std::hypot(obstacle_wx - wx, obstacle_wy - wy));
    }
  }
  if (!std::isfinite(best)) {
    return std::max(costmap.getSizeInMetersX(), costmap.getSizeInMetersY());
  }
  return best;
}

namespace
{

LocalGrid makeLocalGrid(
  const nav2_costmap_2d::Costmap2D & costmap,
  const Point & start,
  const Point & goal,
  double margin)
{
  const double min_x = std::min(start.x, goal.x) - margin;
  const double max_x = std::max(start.x, goal.x) + margin;
  const double min_y = std::min(start.y, goal.y) - margin;
  const double max_y = std::max(start.y, goal.y) + margin;
  const double resolution = costmap.getResolution();
  const double origin_x = costmap.getOriginX();
  const double origin_y = costmap.getOriginY();

  const auto to_cell_floor = [&](double value, double origin, unsigned int max_size) {
      const int raw = static_cast<int>(std::floor((value - origin) / resolution));
      return static_cast<unsigned int>(
        std::max(0, std::min(raw, static_cast<int>(max_size) - 1)));
    };
  const auto to_cell_ceil = [&](double value, double origin, unsigned int max_size) {
      const int raw = static_cast<int>(std::ceil((value - origin) / resolution));
      return static_cast<unsigned int>(
        std::max(0, std::min(raw, static_cast<int>(max_size) - 1)));
    };

  LocalGrid grid;
  grid.min_mx = to_cell_floor(min_x, origin_x, costmap.getSizeInCellsX());
  grid.min_my = to_cell_floor(min_y, origin_y, costmap.getSizeInCellsY());
  const unsigned int max_mx = to_cell_ceil(max_x, origin_x, costmap.getSizeInCellsX());
  const unsigned int max_my = to_cell_ceil(max_y, origin_y, costmap.getSizeInCellsY());
  grid.width = max_mx - grid.min_mx + 1U;
  grid.height = max_my - grid.min_my + 1U;
  return grid;
}

std::vector<double> buildLocalClearanceMap(
  const nav2_costmap_2d::Costmap2D & costmap,
  const LocalGrid & grid,
  bool allow_unknown)
{
  const size_t count = static_cast<size_t>(grid.width) * static_cast<size_t>(grid.height);
  std::vector<double> clearance(count, kInf);
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;

  for (unsigned int local_y = 0; local_y < grid.height; ++local_y) {
    for (unsigned int local_x = 0; local_x < grid.width; ++local_x) {
      const unsigned int mx = grid.min_mx + local_x;
      const unsigned int my = grid.min_my + local_y;
      if (isObstacleCost(costmap.getCost(mx, my), allow_unknown)) {
        const int index = static_cast<int>(local_y * grid.width + local_x);
        clearance[static_cast<size_t>(index)] = 0.0;
        queue.push({0.0, index});
      }
    }
  }

  const int offsets[8][3] = {
    {-1, 0, 10}, {1, 0, 10}, {0, -1, 10}, {0, 1, 10},
    {-1, -1, 14}, {-1, 1, 14}, {1, -1, 14}, {1, 1, 14}};
  while (!queue.empty()) {
    const auto item = queue.top();
    queue.pop();
    if (item.cost > clearance[static_cast<size_t>(item.index)] + kEpsilon) {
      continue;
    }
    const int local_x = item.index % static_cast<int>(grid.width);
    const int local_y = item.index / static_cast<int>(grid.width);
    for (const auto & offset : offsets) {
      const int nx = local_x + offset[0];
      const int ny = local_y + offset[1];
      if (nx < 0 || ny < 0 ||
        nx >= static_cast<int>(grid.width) || ny >= static_cast<int>(grid.height))
      {
        continue;
      }
      const int next_index = ny * static_cast<int>(grid.width) + nx;
      const double next_cost =
        item.cost + static_cast<double>(offset[2]) * costmap.getResolution() * 0.1;
      if (next_cost + kEpsilon < clearance[static_cast<size_t>(next_index)]) {
        clearance[static_cast<size_t>(next_index)] = next_cost;
        queue.push({next_cost, next_index});
      }
    }
  }

  const double fallback_clearance =
    std::hypot(grid.width, grid.height) * costmap.getResolution();
  for (auto & value : clearance) {
    if (!std::isfinite(value)) {
      value = fallback_clearance;
    }
  }
  return clearance;
}

bool worldToMapCell(
  const nav2_costmap_2d::Costmap2D & costmap,
  const Point & point,
  GridCell & cell)
{
  return costmap.worldToMap(point.x, point.y, cell.mx, cell.my);
}

bool isCellSafe(
  const nav2_costmap_2d::Costmap2D & costmap,
  const LocalGrid & grid,
  const std::vector<double> & clearance_map,
  const GridCell & cell,
  const Point & point,
  const Point & start,
  const Point & goal,
  const RouteNarrowPlannerConfig & config)
{
  if (!contains(grid, cell.mx, cell.my)) {
    return false;
  }
  if (!isInflationTraversable(costmap.getCost(cell.mx, cell.my), config.allow_unknown)) {
    return false;
  }
  const double endpoint_min_clearance =
    std::min(config.hard_min_clearance, config.start_goal_keepout_radius);
  const bool near_endpoint =
    distance(point, start) <= config.start_goal_keepout_radius ||
    distance(point, goal) <= config.start_goal_keepout_radius;
  const double required_clearance = near_endpoint ? endpoint_min_clearance :
    config.hard_min_clearance;
  const double clearance = clearance_map[
    static_cast<size_t>(localIndex(grid, cell.mx, cell.my))];
  return clearance + kEpsilon >= std::max(0.0, required_clearance);
}

bool segmentSafe(
  const nav2_costmap_2d::Costmap2D & costmap,
  const LocalGrid & grid,
  const std::vector<double> & clearance_map,
  const Point & start,
  const Point & goal,
  const Point & route_start,
  const Point & route_goal,
  const RouteNarrowPlannerConfig & config)
{
  const double length = distance(start, goal);
  const int steps = std::max(
    1,
    static_cast<int>(std::ceil(length / std::max(costmap.getResolution() * 0.5, kEpsilon))));
  for (int index = 0; index <= steps; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(steps);
    const Point point{
      start.x + (goal.x - start.x) * ratio,
      start.y + (goal.y - start.y) * ratio};
    GridCell cell;
    if (!worldToMapCell(costmap, point, cell) ||
      !isCellSafe(costmap, grid, clearance_map, cell, point, route_start, route_goal, config))
    {
      return false;
    }
  }
  return true;
}

double traversalPenalty(
  const nav2_costmap_2d::Costmap2D & costmap,
  const LocalGrid & grid,
  const std::vector<double> & clearance_map,
  const GridCell & cell,
  const Point & point,
  const Point & reference_start,
  const Point & reference_goal,
  const RouteNarrowPlannerConfig & config)
{
  const double clearance = clearance_map[
    static_cast<size_t>(localIndex(grid, cell.mx, cell.my))];
  const double soft = std::max(config.soft_target_clearance, kEpsilon);
  const double deficit = std::max(0.0, soft - clearance) / soft;
  const double reference_distance = pointToSegmentDistance(point, reference_start, reference_goal);
  const double normalized_reference =
    reference_distance / std::max(config.corridor_half_width, costmap.getResolution());
  return config.clearance_weight * deficit * deficit +
         config.cost_weight * normalizeCost(costmap.getCost(cell.mx, cell.my)) +
         config.reference_weight * normalized_reference * normalized_reference;
}

std::vector<Point> searchCoarsePath(
  const nav2_costmap_2d::Costmap2D & costmap,
  const LocalGrid & grid,
  const std::vector<double> & clearance_map,
  const Point & start,
  const Point & goal,
  const RouteNarrowPlannerConfig & config)
{
  GridCell start_cell;
  GridCell goal_cell;
  if (!worldToMapCell(costmap, start, start_cell) || !contains(grid, start_cell.mx, start_cell.my)) {
    throw nav2_core::PlannerException("route narrow planner start pose is outside local costmap");
  }
  if (!worldToMapCell(costmap, goal, goal_cell) || !contains(grid, goal_cell.mx, goal_cell.my)) {
    throw nav2_core::PlannerException("route narrow planner goal pose is outside local costmap");
  }
  if (!isCellSafe(costmap, grid, clearance_map, start_cell, start, start, goal, config)) {
    throw nav2_core::PlannerException("route narrow planner start pose is not traversable");
  }
  if (!isCellSafe(costmap, grid, clearance_map, goal_cell, goal, start, goal, config)) {
    throw nav2_core::PlannerException("route narrow planner goal pose is not traversable");
  }

  const int start_index = localIndex(grid, start_cell.mx, start_cell.my);
  const int goal_index = localIndex(grid, goal_cell.mx, goal_cell.my);
  const size_t cell_count = static_cast<size_t>(grid.width) * static_cast<size_t>(grid.height);
  std::vector<double> g_score(cell_count, kInf);
  std::vector<int> parent(cell_count, -1);
  std::vector<unsigned char> closed(cell_count, 0U);
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;

  const auto cell_point = [&](unsigned int mx, unsigned int my) {
      double wx = 0.0;
      double wy = 0.0;
      costmap.mapToWorld(mx, my, wx, wy);
      return Point{wx, wy};
    };
  const auto heuristic = [&](const Point & point) {
      return distance(point, goal);
    };

  g_score[static_cast<size_t>(start_index)] = 0.0;
  open.push({heuristic(start), start_index});

  const int offsets[8][2] = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1},
    {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
  while (!open.empty()) {
    const QueueItem item = open.top();
    open.pop();
    if (closed[static_cast<size_t>(item.index)] != 0U) {
      continue;
    }
    closed[static_cast<size_t>(item.index)] = 1U;
    if (item.index == goal_index) {
      break;
    }

    const unsigned int local_x = static_cast<unsigned int>(
      item.index % static_cast<int>(grid.width));
    const unsigned int local_y = static_cast<unsigned int>(
      item.index / static_cast<int>(grid.width));
    const unsigned int mx = grid.min_mx + local_x;
    const unsigned int my = grid.min_my + local_y;
    const Point current_point = cell_point(mx, my);

    for (const auto & offset : offsets) {
      const int next_mx_i = static_cast<int>(mx) + offset[0];
      const int next_my_i = static_cast<int>(my) + offset[1];
      if (next_mx_i < 0 || next_my_i < 0) {
        continue;
      }
      const unsigned int next_mx = static_cast<unsigned int>(next_mx_i);
      const unsigned int next_my = static_cast<unsigned int>(next_my_i);
      if (!contains(grid, next_mx, next_my)) {
        continue;
      }
      const int next_index = localIndex(grid, next_mx, next_my);
      if (closed[static_cast<size_t>(next_index)] != 0U) {
        continue;
      }
      const Point next_point = cell_point(next_mx, next_my);
      GridCell next_cell{next_mx, next_my};
      if (!isCellSafe(costmap, grid, clearance_map, next_cell, next_point, start, goal, config)) {
        continue;
      }
      const double step_distance = distance(current_point, next_point);
      double turn_penalty = 0.0;
      const int parent_index = parent[static_cast<size_t>(item.index)];
      if (parent_index >= 0) {
        const unsigned int parent_x = grid.min_mx + static_cast<unsigned int>(
          parent_index % static_cast<int>(grid.width));
        const unsigned int parent_y = grid.min_my + static_cast<unsigned int>(
          parent_index / static_cast<int>(grid.width));
        const Point parent_point = cell_point(parent_x, parent_y);
        const double ax = current_point.x - parent_point.x;
        const double ay = current_point.y - parent_point.y;
        const double bx = next_point.x - current_point.x;
        const double by = next_point.y - current_point.y;
        const double denom = std::hypot(ax, ay) * std::hypot(bx, by);
        if (denom > kEpsilon) {
          const double dot = std::max(-1.0, std::min(1.0, (ax * bx + ay * by) / denom));
          turn_penalty = config.turn_weight * (1.0 - dot);
        }
      }
      const double tentative =
        g_score[static_cast<size_t>(item.index)] +
        step_distance *
        (1.0 + traversalPenalty(
          costmap, grid, clearance_map, next_cell, next_point, start, goal, config)) +
        turn_penalty;
      if (tentative + kEpsilon < g_score[static_cast<size_t>(next_index)]) {
        g_score[static_cast<size_t>(next_index)] = tentative;
        parent[static_cast<size_t>(next_index)] = item.index;
        open.push({tentative + heuristic(next_point), next_index});
      }
    }
  }

  if (parent[static_cast<size_t>(goal_index)] < 0 && start_index != goal_index) {
    throw nav2_core::PlannerException("route narrow planner could not find a coarse path");
  }

  std::vector<GridCell> cells;
  for (int index = goal_index; index >= 0; index = parent[static_cast<size_t>(index)]) {
    const unsigned int local_x = static_cast<unsigned int>(
      index % static_cast<int>(grid.width));
    const unsigned int local_y = static_cast<unsigned int>(
      index / static_cast<int>(grid.width));
    cells.push_back({grid.min_mx + local_x, grid.min_my + local_y});
    if (index == start_index) {
      break;
    }
  }
  std::reverse(cells.begin(), cells.end());

  std::vector<Point> points;
  points.reserve(cells.size() + 2U);
  appendUnique(points, start, costmap.getResolution() * 0.25);
  for (const auto & cell : cells) {
    appendUnique(points, cell_point(cell.mx, cell.my), costmap.getResolution() * 0.25);
  }
  if (!points.empty() && distance(points.back(), goal) <= costmap.getResolution() * 1.5) {
    points.back() = goal;
  } else {
    appendUnique(points, goal, costmap.getResolution() * 0.25);
  }
  return points;
}

std::vector<Point> optimizeLaterally(
  const nav2_costmap_2d::Costmap2D & costmap,
  const LocalGrid & grid,
  const std::vector<double> & clearance_map,
  const std::vector<Point> & coarse_points,
  const Point & start,
  const Point & goal,
  const RouteNarrowPlannerConfig & config)
{
  const double resolution = std::max(
    config.path_interpolation_resolution > 0.0 ?
    config.path_interpolation_resolution : costmap.getResolution(),
    costmap.getResolution());
  const auto stations = interpolatePolyline(coarse_points, resolution);
  if (stations.size() <= 2U) {
    return coarse_points;
  }

  const double offset_step = std::max(costmap.getResolution(), 0.05);
  const int offset_count = std::max(
    1,
    static_cast<int>(std::floor(config.corridor_half_width / offset_step)));
  std::vector<double> offsets;
  offsets.reserve(static_cast<size_t>(offset_count) * 2U + 1U);
  for (int offset_index = -offset_count; offset_index <= offset_count; ++offset_index) {
    offsets.push_back(static_cast<double>(offset_index) * offset_step);
  }
  const size_t states = offsets.size();

  std::vector<std::vector<double>> dp(stations.size(), std::vector<double>(states, kInf));
  std::vector<std::vector<int>> parent(stations.size(), std::vector<int>(states, -1));
  std::vector<std::vector<Point>> candidates(stations.size(), std::vector<Point>(states));
  std::vector<std::vector<unsigned char>> valid(stations.size(), std::vector<unsigned char>(states, 0U));

  const auto tangent_at = [&](size_t index) {
      const Point & prev = stations[index == 0U ? 0U : index - 1U];
      const Point & next = stations[std::min(index + 1U, stations.size() - 1U)];
      double tx = next.x - prev.x;
      double ty = next.y - prev.y;
      const double length = std::hypot(tx, ty);
      if (length <= kEpsilon) {
        tx = goal.x - start.x;
        ty = goal.y - start.y;
      }
      const double fallback_length = std::hypot(tx, ty);
      if (fallback_length <= kEpsilon) {
        return Point{1.0, 0.0};
      }
      return Point{tx / fallback_length, ty / fallback_length};
    };

  for (size_t station_index = 0; station_index < stations.size(); ++station_index) {
    const Point tangent = tangent_at(station_index);
    const Point normal{-tangent.y, tangent.x};
    for (size_t state = 0; state < states; ++state) {
      const bool endpoint = station_index == 0U || station_index + 1U == stations.size();
      const bool center_state = state == states / 2U;
      if (endpoint && !center_state) {
        continue;
      }
      Point point{
        stations[station_index].x + normal.x * offsets[state],
        stations[station_index].y + normal.y * offsets[state]};
      if (station_index == 0U) {
        point = start;
      } else if (station_index + 1U == stations.size()) {
        point = goal;
      }
      candidates[station_index][state] = point;
      GridCell cell;
      if (!worldToMapCell(costmap, point, cell)) {
        continue;
      }
      valid[station_index][state] = isCellSafe(
        costmap, grid, clearance_map, cell, point, start, goal, config) ? 1U : 0U;
    }
  }

  for (size_t state = 0; state < states; ++state) {
    if (valid.front()[state] != 0U) {
      dp.front()[state] = 0.0;
    }
  }

  const auto point_cost = [&](const Point & point) {
      GridCell cell;
      if (!worldToMapCell(costmap, point, cell) || !contains(grid, cell.mx, cell.my)) {
        return kInf;
      }
      return traversalPenalty(costmap, grid, clearance_map, cell, point, start, goal, config);
    };

  for (size_t station_index = 1; station_index < stations.size(); ++station_index) {
    for (size_t state = 0; state < states; ++state) {
      if (valid[station_index][state] == 0U) {
        continue;
      }
      const Point & point = candidates[station_index][state];
      const double local_cost = point_cost(point);
      if (!std::isfinite(local_cost)) {
        continue;
      }
      for (size_t previous_state = 0; previous_state < states; ++previous_state) {
        if (!std::isfinite(dp[station_index - 1U][previous_state])) {
          continue;
        }
        const Point & previous = candidates[station_index - 1U][previous_state];
        if (!segmentSafe(costmap, grid, clearance_map, previous, point, start, goal, config)) {
          continue;
        }
        const double lateral_delta = offsets[state] - offsets[previous_state];
        const double smooth_cost =
          config.lateral_change_weight * lateral_delta * lateral_delta;
        const double candidate_cost =
          dp[station_index - 1U][previous_state] + distance(previous, point) *
          (1.0 + local_cost) + smooth_cost;
        if (candidate_cost + kEpsilon < dp[station_index][state]) {
          dp[station_index][state] = candidate_cost;
          parent[station_index][state] = static_cast<int>(previous_state);
        }
      }
    }
  }

  size_t best_state = 0U;
  double best_cost = kInf;
  for (size_t state = 0; state < states; ++state) {
    if (dp.back()[state] < best_cost) {
      best_cost = dp.back()[state];
      best_state = state;
    }
  }
  if (!std::isfinite(best_cost)) {
    return coarse_points;
  }

  std::vector<Point> result(stations.size());
  int state = static_cast<int>(best_state);
  for (int station_index = static_cast<int>(stations.size()) - 1; station_index >= 0; --station_index) {
    result[static_cast<size_t>(station_index)] =
      candidates[static_cast<size_t>(station_index)][static_cast<size_t>(state)];
    state = parent[static_cast<size_t>(station_index)][static_cast<size_t>(state)];
    if (station_index == 0) {
      break;
    }
    if (state < 0) {
      return coarse_points;
    }
  }
  return result;
}

std::vector<Point> smoothSafely(
  const nav2_costmap_2d::Costmap2D & costmap,
  const LocalGrid & grid,
  const std::vector<double> & clearance_map,
  const std::vector<Point> & points,
  const Point & start,
  const Point & goal,
  const RouteNarrowPlannerConfig & config)
{
  if (points.size() <= 3U) {
    return points;
  }

  auto smoothed = points;
  const int passes = std::max(0, config.smoothing_passes);
  for (int pass = 0; pass < passes; ++pass) {
    auto next_points = smoothed;
    for (size_t index = 1; index + 1U < smoothed.size(); ++index) {
      const Point candidate{
        smoothed[index].x * 0.5 + (smoothed[index - 1U].x + smoothed[index + 1U].x) * 0.25,
        smoothed[index].y * 0.5 + (smoothed[index - 1U].y + smoothed[index + 1U].y) * 0.25};
      GridCell cell;
      if (!worldToMapCell(costmap, candidate, cell) ||
        !isCellSafe(costmap, grid, clearance_map, cell, candidate, start, goal, config))
      {
        continue;
      }
      if (!segmentSafe(
          costmap, grid, clearance_map, smoothed[index - 1U], candidate, start, goal, config) ||
        !segmentSafe(
          costmap, grid, clearance_map, candidate, smoothed[index + 1U], start, goal, config))
      {
        continue;
      }
      next_points[index] = candidate;
    }
    smoothed = next_points;
  }
  smoothed.front() = start;
  smoothed.back() = goal;
  return smoothed;
}

nav_msgs::msg::Path buildPath(
  const std::vector<Point> & points,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const std::string & frame_id,
  bool use_final_goal_orientation,
  double output_resolution)
{
  const auto output_points = output_resolution > 0.0 ?
    interpolatePolyline(points, output_resolution) : points;
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.poses.reserve(output_points.size());
  for (const auto & point : output_points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  if (path.poses.empty()) {
    return path;
  }
  if (path.poses.size() == 1U) {
    path.poses.front().pose.orientation = use_final_goal_orientation ?
      goal_pose.pose.orientation : path.poses.front().pose.orientation;
    return path;
  }

  for (size_t index = 0; index + 1U < path.poses.size(); ++index) {
    const Point current{
      path.poses[index].pose.position.x,
      path.poses[index].pose.position.y};
    const Point next{
      path.poses[index + 1U].pose.position.x,
      path.poses[index + 1U].pose.position.y};
    path.poses[index].pose.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(yawBetween(current, next));
  }
  if (use_final_goal_orientation) {
    path.poses.back().pose.orientation = goal_pose.pose.orientation;
  } else {
    path.poses.back().pose.orientation = path.poses[path.poses.size() - 2U].pose.orientation;
  }
  return path;
}

}  // namespace

nav_msgs::msg::Path RouteNarrowPlannerCore::createPlan(
  const nav2_costmap_2d::Costmap2D & costmap,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & frame_id) const
{
  const Point start_point{start.pose.position.x, start.pose.position.y};
  const Point goal_point{goal.pose.position.x, goal.pose.position.y};
  if (distance(start_point, goal_point) <= kEpsilon) {
    return buildPath(
      {start_point}, goal, frame_id, config_.use_final_goal_orientation,
      config_.output_path_resolution);
  }

  const double margin = std::max(
    config_.corridor_half_width,
    std::max(config_.soft_target_clearance, config_.hard_min_clearance) + 0.5);
  const LocalGrid grid = makeLocalGrid(costmap, start_point, goal_point, margin);
  const auto clearance_map = buildLocalClearanceMap(costmap, grid, config_.allow_unknown);
  auto coarse_points = searchCoarsePath(
    costmap, grid, clearance_map, start_point, goal_point, config_);
  auto centered_points = optimizeLaterally(
    costmap, grid, clearance_map, coarse_points, start_point, goal_point, config_);
  centered_points = smoothSafely(
    costmap, grid, clearance_map, centered_points, start_point, goal_point, config_);
  return buildPath(
    centered_points, goal, frame_id, config_.use_final_goal_orientation,
    config_.output_path_resolution);
}

}  // namespace nav2_route_polyline_planner
