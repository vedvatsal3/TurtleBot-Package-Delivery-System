#ifndef BT_ROS2_DEMO__DOCK_ACTION_HPP_
#define BT_ROS2_DEMO__DOCK_ACTION_HPP_

#include <behaviortree_ros2/bt_action_node.hpp>
#include <irobot_create_msgs/action/dock.hpp>
#include <rclcpp/rclcpp.hpp>

using Dock = irobot_create_msgs::action::Dock;
using namespace BT;

class DockAction : public RosActionNode<Dock>
{
public:
  DockAction(const std::string& name,
             const NodeConfig& conf,
             const RosNodeParams& params);

  // Ports for the Dock action node are defined here
  static PortsList providedPorts();

  // setGoal is triggered when tick is passed. It should send the goal to the action server
  bool setGoal(RosActionNode::Goal& goal) override;

  // onResultReceived is triggered when the result is received from the action server. It should return appropriate status
  NodeStatus onResultReceived(const WrappedResult& wr) override;

  // onFailure is triggered when the action fails. It should return appropriate status
  NodeStatus onFailure(ActionNodeErrorCode error) override;

  // onFeedback is triggered when feedback is received from the action server. It should return appropriate status
  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
};

#endif  // BT_ROS2_DEMO__DOCK_ACTION_HPP_