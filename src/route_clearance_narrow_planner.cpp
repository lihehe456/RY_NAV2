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

#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_route_polyline_planner/route_narrow_planner_core.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_route_polyline_planner
{

using nav2_util::declare_parameter_if_not_declared;

class RouteClearanceNarrowPlanner : public nav2_core::GlobalPlanner
{
public:
  RouteClearanceNarrowPlanner() = default;

  ~RouteClearanceNarrowPlanner() override
  {
    RCLCPP_INFO(
      logger_,
      "Destroying plugin %s of type RouteClearanceNarrowPlanner",
      name_.c_str());
  }

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
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
      node, name_ + ".hard_min_clearance", rclcpp::ParameterValue(0.40));
    declare_parameter_if_not_declared(
      node, name_ + ".soft_target_clearance", rclcpp::ParameterValue(0.70));
    declare_parameter_if_not_declared(
      node, name_ + ".clearance_weight", rclcpp::ParameterValue(18.0));
    declare_parameter_if_not_declared(
      node, name_ + ".centerline_weight", rclcpp::ParameterValue(0.08));
    declare_parameter_if_not_declared(
      node, name_ + ".cost_weight", rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(
      node, name_ + ".turn_weight", rclcpp::ParameterValue(2.0));
    declare_parameter_if_not_declared(
      node, name_ + ".lateral_change_weight", rclcpp::ParameterValue(10.0));
    declare_parameter_if_not_declared(
      node, name_ + ".lateral_smoothing_passes", rclcpp::ParameterValue(24));
    declare_parameter_if_not_declared(
      node, name_ + ".right_side_bias", rclcpp::ParameterValue(false));
    declare_parameter_if_not_declared(
      node, name_ + ".right_side_weight", rclcpp::ParameterValue(0.0));
    declare_parameter_if_not_declared(
      node, name_ + ".right_side_target_clearance", rclcpp::ParameterValue(0.55));
    declare_parameter_if_not_declared(
      node, name_ + ".right_side_probe_distance", rclcpp::ParameterValue(0.8));
    declare_parameter_if_not_declared(
      node, name_ + ".right_side_max_offset", rclcpp::ParameterValue(0.0));
    declare_parameter_if_not_declared(
      node, name_ + ".pose_directed_crop_enabled", rclcpp::ParameterValue(true));
    declare_parameter_if_not_declared(
      node, name_ + ".goal_approach_length", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(
      node, name_ + ".pose_directed_max_corridor_half_width", rclcpp::ParameterValue(2.0));
    declare_parameter_if_not_declared(
      node, name_ + ".goal_search_radius", rclcpp::ParameterValue(1.2));
    declare_parameter_if_not_declared(
      node, name_ + ".reference_corridor_half_width", rclcpp::ParameterValue(2.0));
    declare_parameter_if_not_declared(
      node, name_ + ".reference_use_astar", rclcpp::ParameterValue(false));
    declare_parameter_if_not_declared(
      node, name_ + ".reference_allow_unknown", rclcpp::ParameterValue(false));
    declare_parameter_if_not_declared(
      node, name_ + ".start_goal_keepout_radius", rclcpp::ParameterValue(0.35));
    declare_parameter_if_not_declared(
      node, name_ + ".max_goal_candidates", rclcpp::ParameterValue(90));
    declare_parameter_if_not_declared(
      node, name_ + ".path_interpolation_resolution", rclcpp::ParameterValue(0.10));
    declare_parameter_if_not_declared(
      node, name_ + ".output_path_resolution", rclcpp::ParameterValue(0.05));
    declare_parameter_if_not_declared(
      node, name_ + ".use_final_goal_orientation", rclcpp::ParameterValue(true));

    RouteNarrowPlannerConfig config;
    double centerline_weight = 0.0;
    node->get_parameter(name_ + ".allow_unknown", config.allow_unknown);
    node->get_parameter(name_ + ".hard_min_clearance", config.hard_min_clearance);
    node->get_parameter(name_ + ".soft_target_clearance", config.soft_target_clearance);
    node->get_parameter(name_ + ".clearance_weight", config.clearance_weight);
    node->get_parameter(name_ + ".centerline_weight", centerline_weight);
    config.reference_weight = centerline_weight;
    node->get_parameter(name_ + ".cost_weight", config.cost_weight);
    node->get_parameter(name_ + ".turn_weight", config.turn_weight);
    node->get_parameter(name_ + ".lateral_change_weight", config.lateral_change_weight);
    node->get_parameter(name_ + ".lateral_smoothing_passes", config.smoothing_passes);
    node->get_parameter(
      name_ + ".reference_corridor_half_width",
      config.corridor_half_width);
    node->get_parameter(name_ + ".start_goal_keepout_radius", config.start_goal_keepout_radius);
    node->get_parameter(
      name_ + ".path_interpolation_resolution",
      config.path_interpolation_resolution);
    node->get_parameter(name_ + ".output_path_resolution", config.output_path_resolution);
    node->get_parameter(name_ + ".use_final_goal_orientation", config.use_final_goal_orientation);

    core_ = std::make_unique<RouteNarrowPlannerCore>(config);
    RCLCPP_INFO(
      logger_,
      "Configuring plugin %s of type RouteClearanceNarrowPlanner",
      name_.c_str());
  }

  void cleanup() override
  {
    RCLCPP_INFO(
      logger_,
      "Cleaning up plugin %s of type RouteClearanceNarrowPlanner",
      name_.c_str());
    core_.reset();
    costmap_ = nullptr;
    costmap_ros_.reset();
    tf_.reset();
  }

  void activate() override
  {
    RCLCPP_INFO(
      logger_,
      "Activating plugin %s of type RouteClearanceNarrowPlanner",
      name_.c_str());
  }

  void deactivate() override
  {
    RCLCPP_INFO(
      logger_,
      "Deactivating plugin %s of type RouteClearanceNarrowPlanner",
      name_.c_str());
  }

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    if (!core_ || !costmap_) {
      throw nav2_core::PlannerException("route clearance narrow planner is not configured");
    }
    if (!start.header.frame_id.empty() && start.header.frame_id != global_frame_) {
      throw nav2_core::PlannerException(
              "route clearance narrow planner start pose has wrong frame_id");
    }
    if (!goal.header.frame_id.empty() && goal.header.frame_id != global_frame_) {
      throw nav2_core::PlannerException(
              "route clearance narrow planner goal pose has wrong frame_id");
    }

    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
    auto path = core_->createPlan(*costmap_, start, goal, global_frame_);
    lock.unlock();

    path.header.stamp = clock_->now();
    path.header.frame_id = global_frame_;
    for (auto & pose : path.poses) {
      pose.header = path.header;
    }
    return path;
  }

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("RouteClearanceNarrowPlanner")};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string global_frame_;
  std::string name_;
  std::unique_ptr<RouteNarrowPlannerCore> core_;
};

}  // namespace nav2_route_polyline_planner

PLUGINLIB_EXPORT_CLASS(
  nav2_route_polyline_planner::RouteClearanceNarrowPlanner,
  nav2_core::GlobalPlanner)
