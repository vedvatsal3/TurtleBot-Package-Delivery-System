#include "bt_ros2_demo/undock_action.hpp"

UndockAction::UndockAction(const std::string& name,
                       const NodeConfig& conf,
                       const RosNodeParams& params)
  : RosActionNode<Undock>(name, conf, params)
{}

PortsList UndockAction::providedPorts()
{
  return providedBasicPorts({});
}

bool UndockAction::setGoal(RosActionNode::Goal& goal)
{
  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("UndockAction"), "Undock server is not available.");
    return false;
  }
  return true; 
}

NodeStatus UndockAction::onResultReceived(const WrappedResult& wr)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("UndockAction: Failed to lock ROS node.");
  }

  if (wr.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO(node->get_logger(), "Undocking succeeded.");
    return NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_WARN(node->get_logger(), "Undocking failed.");
    return NodeStatus::FAILURE;
  }
}

NodeStatus UndockAction::onFailure(ActionNodeErrorCode error)
{
  auto node = node_.lock();
  if (node)
  {
    RCLCPP_ERROR(node->get_logger(), "Undock action failed with error code: %d", error);
  }
  return NodeStatus::FAILURE;
}

NodeStatus UndockAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  auto node = node_.lock();
  if (node)
  {
    RCLCPP_INFO(node->get_logger(), "Undocking in progress...");
  }
  return NodeStatus::RUNNING;
}
