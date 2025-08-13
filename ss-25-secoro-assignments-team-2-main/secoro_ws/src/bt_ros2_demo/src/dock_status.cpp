#include "bt_ros2_demo/dock_status.hpp"

CheckDockStatus::CheckDockStatus(const std::string& name,
                                 const NodeConfiguration& config,
                                 const rclcpp::Node::SharedPtr& ros_node)
  : ConditionNode(name, config), node_(ros_node)
{
  dock_sub_ = node_->create_subscription<irobot_create_msgs::msg::DockStatus>(
      "/dock_status", 
      rclcpp::QoS(10).best_effort(),
      std::bind(&CheckDockStatus::dockCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "Subscription created to /dock_status");

}

PortsList CheckDockStatus::providedPorts()
{
  return {};
}

NodeStatus CheckDockStatus::tick()
{
  if (!dock_status_.has_value())
  {
    RCLCPP_WARN(node_->get_logger(), "Dock status not available yet.");
    return NodeStatus::RUNNING;
  }
  else if(dock_status_ == true)
  {
    RCLCPP_INFO(node_->get_logger(), "Docking status: True");
    return NodeStatus::SUCCESS;
  }
  else if(dock_status_ == false)
  {
    RCLCPP_INFO(node_->get_logger(), "Docking status: False");
    return NodeStatus::FAILURE;
  }
}

void CheckDockStatus::dockCallback(const irobot_create_msgs::msg::DockStatus::SharedPtr msg)
{
  dock_status_ = msg->is_docked;
}
