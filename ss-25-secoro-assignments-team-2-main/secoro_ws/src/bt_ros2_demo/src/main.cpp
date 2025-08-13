#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/bt_action_node.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <rclcpp/rclcpp.hpp>
#include <fstream>

// Include your custom BT nodes
#include "bt_ros2_demo/dock_action.hpp"
#include "bt_ros2_demo/undock_action.hpp"
#include "bt_ros2_demo/dock_status.hpp"
// ...

using namespace BT;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_task_planner");

  BehaviorTreeFactory factory;

  // RosNodeParams: struct to hold parameters for the action nodes (ref: https://github.com/BehaviorTree/BehaviorTree.ROS2/blob/humble/behaviortree_ros2/include/behaviortree_ros2/ros_node_params.hpp)
  RosNodeParams params;
  params.nh = node;

  // Register all your BT nodes
  RosNodeParams undock_params = params;
  // set the action name for the action server
  undock_params.default_port_value = "/dock";
  factory.registerNodeType<DockAction>("Dock", undock_params);

  RosNodeParams dock_params = params;
  // set the action name for the action server
  dock_params.default_port_value = "/undock"; 
  factory.registerNodeType<UndockAction>("Undock", dock_params);

  factory.registerNodeType<CheckDockStatus>("CheckDockStatus", node);
  
  // ...

  // Load the XML file that defines the Behavior Tree structure
  std::string bt_ros2_demo_dir = ament_index_cpp::get_package_share_directory("bt_ros2_demo");
  std::string xml_path = bt_ros2_demo_dir + "/config/tb4_demo.xml";
  auto tree = factory.createTreeFromFile(xml_path);

  // Tick the tree at a fixed rate
  rclcpp::Rate loop_rate(10);

  // Note: Manual ticking with tickOnce() is used inside a loop combined with rclcpp::spin_some()
  // to ensure ROS callbacks are handled while the BT runs.
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    BT::NodeStatus status = tree.tickOnce();
    if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE)
    {
      break;
    }
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}