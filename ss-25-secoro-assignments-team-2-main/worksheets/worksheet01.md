## Overview

In this worksheet, we will understand the tools and libraries involved in ROS2 framework. For an overview to ROS2, please refer to these [slides](https://github.com/a2s-institute/foundations_course/blob/master/content/ros2/introduction/Introduction_to_ROS2.pdf)


Before heading to the tasks, if you are new to Ubuntu, please explore the following command line commands: [link](https://phoenixnap.com/kb/wp-content/uploads/2023/11/linux-commands-cheat-sheet-pdf.pdf)
- Out of these, Directory navigation, Packages (Debian/Ubuntu) SSH login, Files, and Variables will be particularly helpful throughput the course. 
- For more information on individual commands, execute a particular command with --help at the end to view its functionalities.

## Task

- Start with multiple terminals where the same instance of the docker is running.  We will follow the official [ros2 tutorials](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html)
- Execute `xhost +local:root` from any terminal only once before starting the wsl or docker instance
- In every terminal execute the following command to run the bash script for ros jazzy: `source /opt/ros/jazzy/setup.bash`. To avoid sourcing it in every terminal, please add it to the `.bashrc` file located in the home directory in the docker container.
- Follow the [tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html) on using turtlesim, ros2, and rqt until te subsection `3 Use turtlesim`.
- Once you are able to move the turtle around, you should be able to answer the following information from running relevant commands from the terminal. Please explore the following commands and find relevant argument to use: `ros2 node`, `ros2 topic`
	- How many nodes are running? What are their names?
	- How many topics are available?
	- What is the message type that each topic is configured to transfer?
  - What is the pose of the turtle in the environment?
  - What is the frequency of communication of this topic?
  - What are the Publishers, Subscribers, Service Servers, Service Clients, Action Servers, Action Clients setup in the node `turtlesim`?
- Explore the further concepts of nodes, topics, services, parameters, and actions from the [ros2 tutorials](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html) CLI tools. We will further discuss about these and navigation packages in the upcoming sessions.