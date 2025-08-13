#include "bt_ros2_demo/dock_action.hpp"

DockAction::DockAction(const std::string& name,
                       const NodeConfig& conf,
                       const RosNodeParams& params)
  : RosActionNode<Dock>(name, conf, params)
{}

PortsList DockAction::providedPorts()
{
  return providedBasicPorts({});
}

bool DockAction::setGoal(RosActionNode::Goal& goal)
{
  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("DockAction"), "Dock server is not available.");
    return false;
  }
  return true; 
}

NodeStatus DockAction::onResultReceived(const WrappedResult& wr)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("DockAction: Failed to lock ROS node.");
  }

  if (wr.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO(node->get_logger(), "Docking succeeded.");
    return NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_WARN(node->get_logger(), "Docking failed.");
    return NodeStatus::FAILURE;
  }
}

NodeStatus DockAction::onFailure(ActionNodeErrorCode error)
{
  auto node = node_.lock();
  if (node)
  {
    RCLCPP_ERROR(node->get_logger(), "Dock action failed with error code: %d", error);
  }
  return NodeStatus::FAILURE;
}

NodeStatus DockAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  auto node = node_.lock();
  if (node)
  {
    RCLCPP_INFO(node->get_logger(), "Docking in progress...");
  }
  return NodeStatus::RUNNING;
}
