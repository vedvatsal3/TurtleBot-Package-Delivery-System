# BehaviorTree.CPP + ROS2 Integration for TurtleBot4

This repository provides a minimal working example to integrate ROS 2 functionalities with BehaviorTree.CPP, using the [BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2) wrapper. It is used for the TurtleBot4 platform and its simulation environment for SECORO course.

## Prerequisites

- Install BehaviorTree.CPP. If using ROS Jazzy:
  ```bash
  sudo apt install ros-jazzy-behaviortree-cpp

- Clone the following repositories to your workspace:

  ```bash
  git clone https://github.com/secorolab/create3_sim.git
  git clone https://github.com/secorolab/turtlebot4.git
  git clone https://github.com/secorolab/turtlebot4_simulator.git
  git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git -b humble
  ```
- Build and source setup.bash from your workspace

## Running the Demo
- Start the TurtleBot4 simulation (eg: using the office_area world):

  ```bash
  ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py world:=office_area
  ```

- In a separate terminal, run the behavior tree task planner node:

  ```bash
  ros2 run bt_ros2_demo bt_task_planner
  ```
