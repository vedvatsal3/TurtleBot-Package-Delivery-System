# Robot Package Delivery Project 

## Overview
This project was implemented as part of a university subject and is **still under construction**.  
The primary goal is to develop and simulate a robot delivery system using TurtleBot4 and ROS 2, integrating navigation, behavior trees and a cool user interface where a server node autonomously controls the environment while controlling which robot to choose for delivery depending on battery status and similar destination.

## Repository Structure
- **`ss-25-secoro-assignments-team-2/secoro_ws`** –  
  This is the **main workspace** containing everything required to run the simulation.  
- **Assignments** –  
  Each assignment folder contains:
  1. **`task.md`** – Description of all tasks for that assignment.
  2. **`submission_files/`** – Corresponding files submitted for that assignment.  
  These are included so that viewers can explore the step-by-step progress and development process.
- **`worksheets/`** – A collection of worksheets covering the entire implementation and assignments, serving as a reference and study material.
    *`installation_instructions/`* - Guide to all the necessary installation setup

## Resources Used
- **TurtleBot4 Manual**: [https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_common.html](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_common.html)  
- **Behavior Trees**: [https://github.com/BehaviorTree/BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2)

## Installation & Setup
All guides for initial installation, dependencies, and running the simulation are provided in the **`installation_instructions`** folder. The task specific workflow can be found in the worksheets.
Follow these step-by-step to set up your environment and run the project successfully.

## Current Status
- Core simulation environment inclusive of spawnning of 2 turtlebot4 & docking stations, navigation, localization and user interface set up in ROS 2 Jazzy with TurtleBot4.
- We have 2 approaches:
  1. A basic single terminal delivery : Completely working *location*- [ss-25-secoro-assignments-team-2/secoro_ws/src/delivery_interface/delivery_interface/delivery_commander.py]
     *command to run the interface* (ros2 run delivery_interface delivery_commander)
  2. Different terminal for server and clients (sender and receiver), security through ids and PINs, efficient command line interface between server and client displaying warnings and updates, battery constraints, optimized failure handling and more. 
  

## Notes
While **`secoro_ws`** is the main workspace to get the project running, the assignments and worksheets are highly recommended for those who wish to understand the development journey, task requirements, and problem-solving methods applied throughout the course.
