## Overview of Behavior Trees

A Behavior Tree is a structured and modular way to organize and execute actions under the right conditions and at the right time to meet objective of the task. It is widely used in game development and robotics. It support component-based software engineering principles, making them especially suitable for building flexible and maintainable systems. Their hierarchical nature allows developers to compose simple actions into more complex behaviors.

On the contrary, finite state machines (FSM) are suitable for simple systems and when the environment considered for the tasks are predictable. But for complex systems which requires reactivity, modularity, scalability, and better readability, Behavior Tree is more suitable. For detailed comparison between them, refer [1].

Two popular libraries supporting behavior trees in the ROS ecosystem are **py_trees** and **BehaviorTree.CPP**. py_trees is Python-based, beginner-friendly, and well-suited for prototyping, though its support in ROS2 is limited. BehaviorTree.CPP, on the other hand, is a robust C++ library with strong ROS2 integration, where it is used within the ROS2 navigation stack, making it the recommended choice for complex and performance-critical robotics projects.

## Structure and Execution

Behavior Tree is composed of **nodes** arranged hierarchically to represent tasks or decisions. During execution, the tree is evaluated in a regular cycle known as a **tick**, where each node is visited in a defined order. Nodes return a **status**, typically Success, Failure, or Running, which determines how the tree progresses. This architecture supports **asynchronous, non-blocking operations**, allowing tasks to run concurrently without blocking each other.

Unlike traditional state machines that may block while waiting for a task to complete, behavior tree nodes can continue running in the background, allowing other parts of the system to remain active and responsive. This makes them particularly well-suited for robotics applications that demand **concurrent task management** and **real-time responsiveness**. For a deeper conceptual understanding, it is recommended to read Chapters 1 to 3 in the reference book [2].

In addition to nodes, **blackboards** and **ports** facilitate information exchange between nodes, enabling coordination and data sharing across the tree. For a detailed understanding of these concepts, refer to the [official BehaviorTree.CPP tutorials](https://www.behaviortree.dev/docs/category/tutorials-basic) and this [guide](https://www.behaviortree.dev/docs/guides/ports_vs_blackboard) over their usage.
An overview on the structure of nodes and relevant references can be found below. 
### Nodes
Nodes are classified as follows,
- [**Control Node**](https://www.behaviortree.dev/docs/learn-the-basics/BT_basics): These nodes manage the execution flow of their child nodes. Two common types are the [Sequence Node](https://www.behaviortree.dev/docs/nodes-library/sequencenode/) and the [Fallback Node](https://www.behaviortree.dev/docs/nodes-library/FallbackNode/). Control nodes tick their children from left to right, determining the next step based on its own or each child’s return status
	- _Example:_ A **Sequence node** can be used to execute a series of tasks, such as checking preconditions before navigating to a goal. A **Fallback node** can be used to initiate docking behavior when the robot is not docked yet
- [**Decorator Node**](https://www.behaviortree.dev/docs/nodes-library/decoratornode/): These nodes wrap a single child node and are often used to implement retries, timeouts, or logical inversions
	- _Example:_ A decorator can be used to **retry** getting status of docking up to a certain number of attempts if the first attempt fails
- **Condition Node**: These are leaf nodes that perform checks and return either SUCCESS or FAILURE. Condition nodes must never return RUNNING
	- _Example:_ Checking if the battery level is above a minimum threshold before starting a delivery task
- **Action Node**: These are also leaf nodes and they trigger and monitor the execution of specific tasks
	- _Example:_ Sending a **goal pose** to the robot and monitoring whether it has successfully reached the destination

For more detailed explanation on nodes, please refer to the [official BehaviorTree.CPP documentation](https://www.behaviortree.dev/docs/learn-the-basics/BT_basics).

**Note**: The control nodes used in ROS2 Nav2 Behavior Trees follow a slightly different ticking logic compared to standard implementations. Nav2 also provides a set of custom **action nodes**, **condition nodes**, and **decorator nodes** tailored specifically for navigation tasks. For a detailed explanation on the differences, refer to this page on official [Nav2 documentation](https://docs.nav2.org/behavior_trees/overview/nav2_specific_nodes.html#action-nodes), which is also explained in this [video tutorial](https://www.youtube.com/watch?v=sVUKeHMBtpQ).

## Implementation
In the context of this project, it is important to understand the following core concepts:
1. [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP/tree/master): A lightweight C++ library for behavior trees
2. [BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2/tree/humble): A wrapper to integrate BehaviorTree.CPP with ROS2
3. [Nav2 Behavior Trees](https://docs.nav2.org/behavior_trees/index.html): Behavior Trees used within the Navigation2 stack
4. ROS 2 action servers, topics, parameters, and services relevant to robot control and communication 

Minimal examples of a tutorial on BehaviorTree.CPP can be found [here](https://github.com/secorolab/bt_cpp_demo) and its integration with ROS2 concepts can be found [here](https://github.com/secorolab/bt_ros2_demo/tree/master). These can be used as templates or extended to suit your project requirements.

### Installation
- To install **BehaviorTree.CPP** compatible with ROS 2 Jazzy, install it using apt package manager
	```
	sudo apt install ros-jazzy-behaviortree-cpp
	```
- Clone the wrapper repository into the `src` folder of your ROS2 workspace and build from the root of the workspace
	```
	git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git
	```
- If the TurtleBot4 simulation and navigation packages are not yet cloned and built, refer to the second worksheet for setup instructions

### Steps to Develop Your Behavior Tree

1. **Identifying Required Components**: determining missing behavior tree nodes (ROS-dependent or independent) and their implementation
2. **Implement Leaf Nodes**: writing custom **action** and **condition** nodes. Use the [BehaviorTree.CPP demo](https://github.com/secorolab/bt_cpp_demo) as a reference, and see the [ROS 2 integration demo](https://github.com/secorolab/bt_ros2_demo/tree/master) for ROS-specific examples
3. **Define the Tree Using XML**: compose the behavior tree using control nodes in an XML file. For further details, refer to [Official BehaviorTree.CPP Tutorials](https://www.behaviortree.dev/docs/category/tutorials-basic) and [XML Format Guide](https://www.behaviortree.dev/docs/learn-the-basics/xml_format)
### Tips
- **Long-Running ROS Actions**: for action nodes which are long-running and needs to be cancellable, it is recommended to use the ROS2 action clients, as shown in this [tutorial](https://www.behaviortree.dev/docs/ros2_integration). An example for this can be found in the action node defined in [dock_action.hpp](https://github.com/secorolab/bt_ros2_demo/blob/master/include/bt_ros2_demo/dock_action.hpp). It is suitable for tasks such as reaching a particular goal, executing a manipulation motion, and (un)docking
- **Accessing ROS Topics and Parameters**: To read from or write to ROS topics or parameters, you can use action or condition nodes from BehaviorTree.CPP, passing a pointer to the desired node to enable interaction with them. An example implementation to this can be found in the action node defined in [dock_status.hpp](https://github.com/secorolab/bt_ros2_demo/blob/master/include/bt_ros2_demo/dock_status.hpp)
- **Using the BehaviorTreeFactory**: For details about the functionality available in BehaviorTreeFactory, refer to its [header file](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/793a6bdd75386a557382697dd2d7940351a6357d/include/behaviortree_cpp/bt_factory.h)
- **Different options of implementing action nodes and condition nodes:**
	- **SimpleActionNode**: To trigger a function of a class as a pointer to a function (functor)
	- **SyncActionNode**: This can be used for nodes of synchronous nature (return either SUCCESS or FAILURE but not RUNNING). It requires implementing your own tick method and optionally define the ports
	- **StatefulActionNode**: This can be used for nodes with asynchronous nature, which can also return RUNNING while it has not completed the intended behavior. It allows to define the following methods ([ref](https://www.behaviortree.dev/docs/tutorial-basics/tutorial_04_sequence)),
		- onStart()
		- onRunning()
		- onHalted()
	- **SimpleConditionNode**: The condition nodes can be implemented as a functor (similar to SimpleActionNode)

## Tasks
A brief understanding of [concepts](https://docs.nav2.org/concepts/index.html#concepts) involved in ROS2 Nav2 will help in meeting these tasks and the objective of the project. For details on implementing certain navigation functionalities, refer to the tutorials listed in [General Tutorials page](https://docs.nav2.org/tutorials/index.html).
- Use the [keepout filter](https://docs.nav2.org/tutorials/docs/navigation2_with_keepout_filter.html) to prevent the robot from entering restricted areas and optionally to define designated lanes for navigation. When following the launch file from the tutorial, ensure that the `use_composition` launch argument is set to `False` from the command line. Also, provide the full path to the filter's configuration file via the `params_file`, and to the masked map image via the `mask` command line arguments. This doesn't require any implementation of behavior trees
- Optionally, navigate with speed limits at designated regions, for example around doors ([speed_filter](https://docs.nav2.org/tutorials/docs/navigation2_with_speed_filter.html))
- Refer or reuse the  [demo integration of BT with ROS2 for (un)docking](https://github.com/secorolab/bt_ros2_demo/tree/master) to navigate to a particular location
- Develop rest of the solution for your prototype as per the third assignment

## Resources
1. [Comparison between Behavior Trees and Finite State Machines](https://arxiv.org/abs/2405.16137)
2. [Behavior Trees in Robotics and AI: An Introduction](https://arxiv.org/abs/1709.00084)
3. [BT.CPP Tutorials](https://www.behaviortree.dev/docs/category/tutorials-basic/) and [documentation](https://www.behaviortree.dev/docs/learn-the-basics/bt_basics/)
4. [Ports vs Blackboard Guide](https://www.behaviortree.dev/docs/guides/ports_vs_blackboard/)
5. [BT.CPP - Git](https://github.com/BehaviorTree/BehaviorTree.CPP/tree/master)
6. [BT.ROS2 - Git](https://github.com/BehaviorTree/BehaviorTree.ROS2/tree/humble)
7. [bt-cpp-demo - Git](https://github.com/secorolab/bt_cpp_demo)
8. [bt-ros2-demo - Git](https://github.com/secorolab/bt_cpp_demo)
9. [Nav2 Concepts](https://docs.nav2.org/concepts/index.html#concepts)
10. [Nav2 Tutorials](https://docs.nav2.org/tutorials/index.html)
11. [Nav2 Behavior Trees](https://docs.nav2.org/behavior_trees/index.html)
12. [Nav2 BT Nodes](https://docs.nav2.org/configuration/packages/configuring-bt-xml.html)
13. [BT.CPP - YouTube](https://www.youtube.com/watch?v=kRp3eA09JkM&t=23s)
14. [Nav2 in ROS2 - YouTube](https://www.youtube.com/watch?v=sVUKeHMBtpQ)






