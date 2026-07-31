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

#include "nav2_route_polyline_planner/route_clearance_planner.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace nav2_route_polyline_planner
{
namespace
{

class ScopedCostmapOverlayRestore
{
public:
  ScopedCostmapOverlayRestore(
    nav2_costmap_2d::Costmap2D * costmap,
    std::vector<LocalCostmapOverlayRestoreCell> * restore_cells)
  : costmap_(costmap),
    restore_cells_(restore_cells)
  {
  }

  ~ScopedCostmapOverlayRestore()
  {
    if (costmap_ == nullptr || restore_cells_ == nullptr) {
      return;
    }
    for (const auto & cell : *restore_cells_) {
      costmap_->setCost(cell.mx, cell.my, cell.old_cost);
    }
  }

  ScopedCostmapOverlayRestore(const ScopedCostmapOverlayRestore &) = delete;
  ScopedCostmapOverlayRestore & operator=(const ScopedCostmapOverlayRestore &) = delete;

private:
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::vector<LocalCostmapOverlayRestoreCell> * restore_cells_{nullptr};
};

}  // namespace

using nav2_util::declare_parameter_if_not_declared;

RouteClearancePlanner::RouteClearancePlanner()
= default;

RouteClearancePlanner::~RouteClearancePlanner()
{
  RCLCPP_INFO(
    logger_,
    "Destroying plugin %s of type RouteClearancePlanner",
    name_.c_str());
}

void RouteClearancePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  name_ = std::move(name);

  auto node = node_.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, name_ + ".hard_min_clearance", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(
    node, name_ + ".soft_target_clearance", rclcpp::ParameterValue(0.50));
  declare_parameter_if_not_declared(
    node, name_ + ".clearance_weight", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, name_ + ".centerline_weight", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    node, name_ + ".cost_weight", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, name_ + ".turn_weight", rclcpp::ParameterValue(0.15));
  declare_parameter_if_not_declared(
    node, name_ + ".lateral_change_weight", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(
    node, name_ + ".lateral_smoothing_passes", rclcpp::ParameterValue(0));
  declare_parameter_if_not_declared(
    node, name_ + ".right_side_bias", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, name_ + ".right_side_weight", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    node, name_ + ".right_side_target_clearance", rclcpp::ParameterValue(0.70));
  declare_parameter_if_not_declared(
    node, name_ + ".right_side_probe_distance", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, name_ + ".right_side_max_offset", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(
    node, name_ + ".lateral_preference", rclcpp::ParameterValue(""));
  declare_parameter_if_not_declared(
    node, name_ + ".lateral_preference_offset", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    node, name_ + ".lateral_preference_weight", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    node, name_ + ".pose_directed_crop_enabled", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, name_ + ".goal_approach_length", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, name_ + ".pose_directed_max_corridor_half_width", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    node, name_ + ".goal_search_radius", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, name_ + ".reference_corridor_half_width", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, name_ + ".reference_use_astar", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, name_ + ".reference_allow_unknown", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, name_ + ".start_goal_keepout_radius", rclcpp::ParameterValue(0.35));
  declare_parameter_if_not_declared(
    node, name_ + ".max_goal_candidates", rclcpp::ParameterValue(80));
  declare_parameter_if_not_declared(
    node, name_ + ".path_interpolation_resolution", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    node, name_ + ".output_path_resolution", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    node, name_ + ".use_final_goal_orientation", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, name_ + ".use_local_costmap_overlay", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, name_ + ".local_costmap_topic",
    rclcpp::ParameterValue("/local_costmap/costmap_raw"));
  declare_parameter_if_not_declared(
    node, name_ + ".local_costmap_max_age", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(
    node, name_ + ".local_costmap_influence_distance", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, name_ + ".local_costmap_min_overlay_cost", rclcpp::ParameterValue(1));
  declare_parameter_if_not_declared(
    node, name_ + ".local_costmap_lethal_threshold", rclcpp::ParameterValue(253));
  declare_parameter_if_not_declared(
    node, name_ + ".local_costmap_tf_timeout", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, name_ + ".local_costmap_stale_policy", rclcpp::ParameterValue("ignore"));

  RouteClearancePlannerConfig config;
  node->get_parameter(name_ + ".allow_unknown", config.allow_unknown);
  node->get_parameter(name_ + ".hard_min_clearance", config.hard_min_clearance);
  node->get_parameter(name_ + ".soft_target_clearance", config.soft_target_clearance);
  node->get_parameter(name_ + ".clearance_weight", config.clearance_weight);
  node->get_parameter(name_ + ".centerline_weight", config.centerline_weight);
  node->get_parameter(name_ + ".cost_weight", config.cost_weight);
  node->get_parameter(name_ + ".turn_weight", config.turn_weight);
  node->get_parameter(name_ + ".lateral_change_weight", config.lateral_change_weight);
  node->get_parameter(name_ + ".lateral_smoothing_passes", config.lateral_smoothing_passes);
  node->get_parameter(name_ + ".right_side_bias", config.right_side_bias);
  node->get_parameter(name_ + ".right_side_weight", config.right_side_weight);
  node->get_parameter(
    name_ + ".right_side_target_clearance",
    config.right_side_target_clearance);
  node->get_parameter(
    name_ + ".right_side_probe_distance",
    config.right_side_probe_distance);
  node->get_parameter(name_ + ".right_side_max_offset", config.right_side_max_offset);
  std::string lateral_preference;
  double lateral_preference_offset = 0.0;
  double lateral_preference_weight = 0.0;
  node->get_parameter(name_ + ".lateral_preference", lateral_preference);
  node->get_parameter(name_ + ".lateral_preference_offset", lateral_preference_offset);
  node->get_parameter(name_ + ".lateral_preference_weight", lateral_preference_weight);
  if (lateral_preference == "right") {
    config.right_side_bias = true;
    if (lateral_preference_weight > 0.0 && config.right_side_weight <= 0.0) {
      config.right_side_weight = lateral_preference_weight;
    }
    if (lateral_preference_offset > 0.0) {
      config.right_side_max_offset = lateral_preference_offset;
    }
  }
  node->get_parameter(
    name_ + ".pose_directed_crop_enabled",
    config.pose_directed_crop_enabled);
  node->get_parameter(
    name_ + ".goal_approach_length",
    config.goal_approach_length);
  node->get_parameter(
    name_ + ".pose_directed_max_corridor_half_width",
    config.pose_directed_max_corridor_half_width);
  node->get_parameter(name_ + ".goal_search_radius", config.goal_search_radius);
  node->get_parameter(
    name_ + ".reference_corridor_half_width",
    config.reference_corridor_half_width);
  node->get_parameter(name_ + ".reference_use_astar", config.reference_use_astar);
  node->get_parameter(
    name_ + ".reference_allow_unknown",
    config.reference_allow_unknown);
  node->get_parameter(
    name_ + ".start_goal_keepout_radius",
    config.start_goal_keepout_radius);
  node->get_parameter(name_ + ".max_goal_candidates", config.max_goal_candidates);
  node->get_parameter(
    name_ + ".path_interpolation_resolution",
    config.path_interpolation_resolution);
  node->get_parameter(
    name_ + ".output_path_resolution",
    config.output_path_resolution);
  node->get_parameter(
    name_ + ".use_final_goal_orientation",
    config.use_final_goal_orientation);
  node->get_parameter(name_ + ".use_local_costmap_overlay", use_local_costmap_overlay_);
  node->get_parameter(name_ + ".local_costmap_topic", local_costmap_topic_);
  node->get_parameter(name_ + ".local_costmap_max_age", local_costmap_max_age_sec_);
  node->get_parameter(
    name_ + ".local_costmap_influence_distance",
    local_costmap_overlay_options_.influence_distance);
  int min_overlay_cost = static_cast<int>(local_costmap_overlay_options_.min_overlay_cost);
  int lethal_threshold = static_cast<int>(local_costmap_overlay_options_.lethal_threshold);
  node->get_parameter(name_ + ".local_costmap_min_overlay_cost", min_overlay_cost);
  node->get_parameter(name_ + ".local_costmap_lethal_threshold", lethal_threshold);
  node->get_parameter(name_ + ".local_costmap_tf_timeout", local_costmap_tf_timeout_sec_);
  node->get_parameter(name_ + ".local_costmap_stale_policy", local_costmap_stale_policy_);
  local_costmap_max_age_sec_ = std::max(0.0, local_costmap_max_age_sec_);
  local_costmap_tf_timeout_sec_ = std::max(0.0, local_costmap_tf_timeout_sec_);
  local_costmap_overlay_options_.influence_distance =
    std::max(0.0, local_costmap_overlay_options_.influence_distance);
  local_costmap_overlay_options_.min_overlay_cost =
    static_cast<unsigned char>(std::clamp(min_overlay_cost, 0, 255));
  local_costmap_overlay_options_.lethal_threshold =
    static_cast<unsigned char>(std::clamp(lethal_threshold, 0, 255));

  if (use_local_costmap_overlay_) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    local_costmap_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
      local_costmap_topic_,
      qos,
      std::bind(&RouteClearancePlanner::localCostmapCallback, this, std::placeholders::_1));
    RCLCPP_INFO(
      logger_,
      "RouteClearance local costmap overlay enabled on topic '%s'",
      local_costmap_topic_.c_str());
  }

  core_ = std::make_unique<RouteClearancePlannerCore>(config);
  RCLCPP_INFO(
    logger_,
    "Configuring plugin %s of type RouteClearancePlanner",
    name_.c_str());
}

void RouteClearancePlanner::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up plugin %s of type RouteClearancePlanner",
    name_.c_str());
  core_.reset();
  local_costmap_sub_.reset();
  {
    std::lock_guard<std::mutex> lock(local_costmap_mutex_);
    latest_local_costmap_.reset();
  }
  costmap_ = nullptr;
  costmap_ros_.reset();
  tf_.reset();
}

void RouteClearancePlanner::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating plugin %s of type RouteClearancePlanner",
    name_.c_str());
}

void RouteClearancePlanner::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating plugin %s of type RouteClearancePlanner",
    name_.c_str());
}

void RouteClearancePlanner::localCostmapCallback(nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(local_costmap_mutex_);
  latest_local_costmap_ = std::move(msg);
  latest_local_costmap_receive_time_ = clock_->now();
}

nav_msgs::msg::Path RouteClearancePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!core_ || !costmap_) {
    throw nav2_core::PlannerException("route clearance planner is not configured");
  }
  if (!start.header.frame_id.empty() && start.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException("route clearance planner start pose has wrong frame_id");
  }
  if (!goal.header.frame_id.empty() && goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException("route clearance planner goal pose has wrong frame_id");
  }

  nav2_msgs::msg::Costmap::SharedPtr local_costmap;
  rclcpp::Time local_costmap_receive_time;
  if (use_local_costmap_overlay_) {
    std::lock_guard<std::mutex> lock(local_costmap_mutex_);
    local_costmap = latest_local_costmap_;
    local_costmap_receive_time = latest_local_costmap_receive_time_;
  }

  bool use_local_overlay = false;
  geometry_msgs::msg::TransformStamped local_to_global;
  const bool fail_on_stale = local_costmap_stale_policy_ == "fail";
  if (use_local_costmap_overlay_) {
    if (!local_costmap) {
      if (fail_on_stale) {
        throw nav2_core::PlannerException("route clearance local costmap overlay has no data");
      }
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "RouteClearance local costmap overlay has no data; using global costmap only");
    } else if (
      (clock_->now() - local_costmap_receive_time).seconds() > local_costmap_max_age_sec_)
    {
      if (fail_on_stale) {
        throw nav2_core::PlannerException("route clearance local costmap overlay data is stale");
      }
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "RouteClearance local costmap overlay data is stale; using global costmap only");
    } else if (local_costmap->header.frame_id.empty()) {
      if (fail_on_stale) {
        throw nav2_core::PlannerException("route clearance local costmap frame_id is empty");
      }
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "RouteClearance local costmap frame_id is empty; using global costmap only");
    } else {
      try {
        if (local_costmap->header.frame_id == global_frame_) {
          local_to_global.header.frame_id = global_frame_;
          local_to_global.child_frame_id = global_frame_;
          local_to_global.transform.rotation.w = 1.0;
        } else {
          local_to_global = tf_->lookupTransform(
            global_frame_,
            local_costmap->header.frame_id,
            tf2::TimePointZero,
            tf2::durationFromSec(local_costmap_tf_timeout_sec_));
        }
        use_local_overlay = true;
      } catch (const tf2::TransformException & ex) {
        if (fail_on_stale) {
          throw nav2_core::PlannerException(
                  std::string("route clearance local costmap transform failed: ") + ex.what());
        }
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 5000,
          "RouteClearance local costmap transform failed: %s; using global costmap only",
          ex.what());
      }
    }
  }

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  const nav2_costmap_2d::Costmap2D * planning_costmap = costmap_;
  std::vector<LocalCostmapOverlayRestoreCell> restore_cells;
  ScopedCostmapOverlayRestore overlay_restore(costmap_, &restore_cells);
  if (use_local_overlay && local_costmap) {
    LocalCostmapOverlayStats stats;
    const bool applied = overlayLocalCostmap(
      *local_costmap,
      local_to_global,
      start,
      local_costmap_overlay_options_,
      *costmap_,
      &stats,
      &restore_cells);
    if (!applied) {
      if (fail_on_stale) {
        throw nav2_core::PlannerException("route clearance local costmap overlay is malformed");
      }
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "RouteClearance local costmap overlay is malformed; using global costmap only");
    } else if (stats.overlaid_cells > 0U) {
      planning_costmap = costmap_;
      RCLCPP_DEBUG(
        logger_,
        "RouteClearance overlaid %zu/%zu local costmap cells near the start",
        stats.overlaid_cells,
        stats.considered_cells);
    }
  }

  auto result = core_->createPlan(*planning_costmap, start, goal, global_frame_);

  result.path.header.stamp = clock_->now();
  result.path.header.frame_id = global_frame_;
  for (auto & pose : result.path.poses) {
    pose.header = result.path.header;
  }
  return result.path;
}

}  // namespace nav2_route_polyline_planner

PLUGINLIB_EXPORT_CLASS(
  nav2_route_polyline_planner::RouteClearancePlanner,
  nav2_core::GlobalPlanner)
