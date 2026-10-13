// SPDX-License-Identifier: MIT
#pragma once
#include <memory>
#include <vector>
#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
namespace sawyer_bridge {
struct ActionBridge { virtual ~ActionBridge() = default; };
std::vector<std::shared_ptr<ActionBridge>> create_action_bridges(
  ros::NodeHandle & ros1, rclcpp::Node::SharedPtr ros2,
  const std::string & parameter = "/sawyer_bridge/actions");
}
