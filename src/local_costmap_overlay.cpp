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

#include "nav2_route_polyline_planner/local_costmap_overlay.hpp"

#include <algorithm>
#include <cmath>

namespace nav2_route_polyline_planner
{
namespace
{

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & orientation)
{
  const double siny_cosp =
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cosy_cosp =
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::pair<double, double> rotateAndTranslate(
  double x,
  double y,
  double yaw,
  double tx,
  double ty)
{
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  return {
    tx + cos_yaw * x - sin_yaw * y,
    ty + sin_yaw * x + cos_yaw * y};
}

}  // namespace

bool overlayLocalCostmap(
  const nav2_msgs::msg::Costmap & local_costmap,
  const geometry_msgs::msg::TransformStamped & local_to_global,
  const geometry_msgs::msg::PoseStamped & start,
  const LocalCostmapOverlayOptions & options,
  nav2_costmap_2d::Costmap2D & global_costmap,
  LocalCostmapOverlayStats * stats,
  std::vector<LocalCostmapOverlayRestoreCell> * restore_cells)
{
  LocalCostmapOverlayStats local_stats;
  auto & output_stats = stats == nullptr ? local_stats : *stats;
  output_stats = LocalCostmapOverlayStats{};
  if (restore_cells != nullptr) {
    restore_cells->clear();
  }

  const unsigned int size_x = local_costmap.metadata.size_x;
  const unsigned int size_y = local_costmap.metadata.size_y;
  const double resolution = static_cast<double>(local_costmap.metadata.resolution);
  const size_t expected_size = static_cast<size_t>(size_x) * static_cast<size_t>(size_y);
  if (size_x == 0U || size_y == 0U || resolution <= 0.0 ||
    local_costmap.data.size() != expected_size)
  {
    return false;
  }

  const double origin_yaw = yawFromQuaternion(local_costmap.metadata.origin.orientation);
  const double transform_yaw = yawFromQuaternion(local_to_global.transform.rotation);
  const double influence_distance = std::max(0.0, options.influence_distance);
  const double influence_distance_sq = influence_distance * influence_distance;
  const bool limit_by_distance = influence_distance > 0.0;

  for (unsigned int my = 0; my < size_y; ++my) {
    for (unsigned int mx = 0; mx < size_x; ++mx) {
      const size_t index = static_cast<size_t>(my) * static_cast<size_t>(size_x) +
        static_cast<size_t>(mx);
      const unsigned char source_cost = local_costmap.data[index];
      if (source_cost == nav2_costmap_2d::NO_INFORMATION ||
        source_cost < options.min_overlay_cost)
      {
        continue;
      }
      ++output_stats.considered_cells;

      const double cell_x = (static_cast<double>(mx) + 0.5) * resolution;
      const double cell_y = (static_cast<double>(my) + 0.5) * resolution;
      const auto local_point = rotateAndTranslate(
        cell_x,
        cell_y,
        origin_yaw,
        local_costmap.metadata.origin.position.x,
        local_costmap.metadata.origin.position.y);
      const auto global_point = rotateAndTranslate(
        local_point.first,
        local_point.second,
        transform_yaw,
        local_to_global.transform.translation.x,
        local_to_global.transform.translation.y);

      const double dx = global_point.first - start.pose.position.x;
      const double dy = global_point.second - start.pose.position.y;
      if (limit_by_distance && dx * dx + dy * dy > influence_distance_sq) {
        ++output_stats.skipped_cells;
        continue;
      }

      unsigned int global_mx = 0;
      unsigned int global_my = 0;
      if (!global_costmap.worldToMap(
          global_point.first, global_point.second, global_mx, global_my))
      {
        ++output_stats.skipped_cells;
        continue;
      }

      const unsigned char overlay_cost =
        source_cost >= options.lethal_threshold ?
        nav2_costmap_2d::LETHAL_OBSTACLE : source_cost;
      const unsigned char old_cost = global_costmap.getCost(global_mx, global_my);
      if (overlay_cost > old_cost) {
        if (restore_cells != nullptr) {
          restore_cells->push_back({global_mx, global_my, old_cost});
        }
        global_costmap.setCost(global_mx, global_my, overlay_cost);
        ++output_stats.overlaid_cells;
      }
    }
  }

  return true;
}

}  // namespace nav2_route_polyline_planner
