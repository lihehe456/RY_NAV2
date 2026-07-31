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

#ifndef NAV2_ROUTE_POLYLINE_PLANNER__ROUTE_CLEARANCE_PLANNER_HPP_
#define NAV2_ROUTE_POLYLINE_PLANNER__ROUTE_CLEARANCE_PLANNER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "nav2_core/global_planner.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_route_polyline_planner/local_costmap_overlay.hpp"
#include "nav2_route_polyline_planner/route_clearance_planner_core.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_route_polyline_planner
{

class RouteClearancePlanner : public nav2_core::GlobalPlanner
{
public:
  RouteClearancePlanner();
  ~RouteClearancePlanner() override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  void localCostmapCallback(nav2_msgs::msg::Costmap::SharedPtr msg);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("RouteClearancePlanner")};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string global_frame_;
  std::string name_;
  std::unique_ptr<RouteClearancePlannerCore> core_;

  bool use_local_costmap_overlay_{false};
  std::string local_costmap_topic_{"/local_costmap/costmap_raw"};
  double local_costmap_max_age_sec_{0.8};
  double local_costmap_tf_timeout_sec_{0.05};
  std::string local_costmap_stale_policy_{"ignore"};
  LocalCostmapOverlayOptions local_costmap_overlay_options_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr local_costmap_sub_;
  std::mutex local_costmap_mutex_;
  nav2_msgs::msg::Costmap::SharedPtr latest_local_costmap_;
  rclcpp::Time latest_local_costmap_receive_time_;
};

}  // namespace nav2_route_polyline_planner

#endif  // NAV2_ROUTE_POLYLINE_PLANNER__ROUTE_CLEARANCE_PLANNER_HPP_
