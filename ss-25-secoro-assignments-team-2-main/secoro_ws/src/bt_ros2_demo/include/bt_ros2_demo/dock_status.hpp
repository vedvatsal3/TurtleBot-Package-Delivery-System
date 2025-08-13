#ifndef BT_ROS2_DEMO__DOCK_STATUS_HPP_
#define BT_ROS2_DEMO__DOCK_STATUS_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include "irobot_create_msgs/msg/dock_status.hpp"
#include <rclcpp/qos.hpp>
#include <optional>

using namespace BT;

class CheckDockStatus : public ConditionNode
{
public:
  CheckDockStatus(const std::string& name,
                  const NodeConfiguration& config,
                  const rclcpp::Node::SharedPtr& ros_node);

  // Ports for the CheckDockStatus node are defined here
  static PortsList providedPorts();

  // tick is triggered when the node is executed. It should return appropriate status
  NodeStatus tick() override;

private:
  // Callback function for the subscription to /dock_status topic
  void dockCallback(const irobot_create_msgs::msg::DockStatus::SharedPtr msg);

  // Other member variables
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<irobot_create_msgs::msg::DockStatus>::SharedPtr dock_sub_;
  std::optional<bool> dock_status_ = std::nullopt;
};

#endif  // BT_ROS2_DEMO__DOCK_STATUS_HPP_
