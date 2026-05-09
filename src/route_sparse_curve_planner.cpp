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

#include "nav2_route_polyline_planner/route_sparse_curve_planner.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_route_polyline_planner
{

using nav2_util::declare_parameter_if_not_declared;

RouteSparseCurvePlanner::RouteSparseCurvePlanner()
= default;

RouteSparseCurvePlanner::~RouteSparseCurvePlanner()
{
  RCLCPP_INFO(
    logger_,
    "Destroying plugin %s of type RouteSparseCurvePlanner",
    name_.c_str());
}

void RouteSparseCurvePlanner::configure(
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
    node, name_ + ".path_interpolation_resolution", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    node, name_ + ".collision_check_resolution", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    node, name_ + ".straight_angle_threshold_deg", rclcpp::ParameterValue(12.0));
  declare_parameter_if_not_declared(
    node, name_ + ".curve_sample_resolution", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    node, name_ + ".tangent_scale", rclcpp::ParameterValue(0.45));
  declare_parameter_if_not_declared(
    node, name_ + ".max_tangent_length", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, name_ + ".max_curve_cell_cost", rclcpp::ParameterValue(180));
  declare_parameter_if_not_declared(
    node, name_ + ".fallback_corridor_half_width", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, name_ + ".fallback_use_astar", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, name_ + ".fallback_allow_unknown", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, name_ + ".start_goal_keepout_radius", rclcpp::ParameterValue(0.35));

  RouteSparseCurvePlannerConfig config;
  node->get_parameter(
    name_ + ".path_interpolation_resolution",
    config.path_interpolation_resolution);
  node->get_parameter(
    name_ + ".collision_check_resolution",
    config.collision_check_resolution);
  node->get_parameter(
    name_ + ".straight_angle_threshold_deg",
    config.straight_angle_threshold_deg);
  node->get_parameter(
    name_ + ".curve_sample_resolution",
    config.curve_sample_resolution);
  node->get_parameter(
    name_ + ".tangent_scale",
    config.tangent_scale);
  node->get_parameter(
    name_ + ".max_tangent_length",
    config.max_tangent_length);
  int max_curve_cell_cost = static_cast<int>(config.max_curve_cell_cost);
  node->get_parameter(
    name_ + ".max_curve_cell_cost",
    max_curve_cell_cost);
  config.max_curve_cell_cost = static_cast<unsigned int>(std::max(0, max_curve_cell_cost));
  node->get_parameter(
    name_ + ".fallback_corridor_half_width",
    config.fallback_corridor_half_width);
  node->get_parameter(
    name_ + ".fallback_use_astar",
    config.fallback_use_astar);
  node->get_parameter(
    name_ + ".fallback_allow_unknown",
    config.fallback_allow_unknown);
  node->get_parameter(
    name_ + ".start_goal_keepout_radius",
    config.start_goal_keepout_radius);

  core_ = std::make_unique<RouteSparseCurvePlannerCore>(config);
  RCLCPP_INFO(
    logger_,
    "Configuring plugin %s of type RouteSparseCurvePlanner",
    name_.c_str());
}

void RouteSparseCurvePlanner::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up plugin %s of type RouteSparseCurvePlanner",
    name_.c_str());
  core_.reset();
  costmap_ = nullptr;
  costmap_ros_.reset();
  tf_.reset();
}

void RouteSparseCurvePlanner::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating plugin %s of type RouteSparseCurvePlanner",
    name_.c_str());
}

void RouteSparseCurvePlanner::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating plugin %s of type RouteSparseCurvePlanner",
    name_.c_str());
}

nav_msgs::msg::Path RouteSparseCurvePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!core_ || !costmap_) {
    throw nav2_core::PlannerException("route sparse curve planner is not configured");
  }
  if (!start.header.frame_id.empty() && start.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException("route sparse curve planner start pose has wrong frame_id");
  }
  if (!goal.header.frame_id.empty() && goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException("route sparse curve planner goal pose has wrong frame_id");
  }

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  auto result = core_->planSegment(*costmap_, start, goal, global_frame_);
  lock.unlock();

  result.path.header.stamp = clock_->now();
  result.path.header.frame_id = global_frame_;
  for (auto & pose : result.path.poses) {
    pose.header = result.path.header;
  }
  return result.path;
}

}  // namespace nav2_route_polyline_planner

PLUGINLIB_EXPORT_CLASS(
  nav2_route_polyline_planner::RouteSparseCurvePlanner,
  nav2_core::GlobalPlanner)
