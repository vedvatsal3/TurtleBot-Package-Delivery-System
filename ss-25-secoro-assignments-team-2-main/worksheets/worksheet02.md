## System setup
- From within your wsl environment, create a directory for your workspace (e.g., named `ros2_ws`) where you'll clone or develop all your ROS packages. Inside this workspace, create a folder named `src/`. Then, from within the `src/` directory, clone the following repositories,
```
git clone https://github.com/secorolab/create3_sim.git
git clone https://github.com/secorolab/turtlebot4.git
git clone https://github.com/secorolab/turtlebot4_simulator.git
```
- Install the following packages
```
sudo apt update 
sudo apt install -y build-essential ros-jazzy-control-msgs ros-jazzy-irobot-create-nodes ros-jazzy-turtlebot4-desktop ros-jazzy-controller-manager ros-jazzy-gz-ros2-control ros-jazzy-ros-gz-interfaces ros-jazzy-nav2-bringup
```
- From the root of your workspace (i.e., ros2_ws/), build the workspace by executing `colcon build --allow-overriding $(colcon list --names-only)`
- **Note:** Generally `colcon build` is sufficient to build the packages, but as some of these are also included in the *underlay* workspace, they should be explicitly built with the flag *--allow-overriding*
- Every time you build the workspace, you need to **source** it so that your terminal session can recognize the linked libraries. To do this, execute `source install/setup.bash` from the root of your workspace
- The rest of the tutorial on mapping and navigation is based on the detailed explanation provided in (1)

## Simulation
- **Note**: if you have Nvidia-graphics card, then under nvidia-settings, select **NVIDIA(Performance Mode)**
- Execute `xhost +local:root` from any terminal only once before starting the wsl or docker instance
- Run TurtleBot4 in simulation
```
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py world:=office_area
```
- Note: For first instance it will take time for the robot and the environment to load. Even after~15 minutes if you do not see the robot in the environment, then rerun the above launch script
- To move the robot around, ether use the teleoperation interface from the simulator or execute the following command from the terminal.
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

## Mapping
- We will use Simultaneous Localization and Mapping (SLAM) for robust mapping. Before executing SLAM, check if laser scanner data is being published by echoing to the /scan topic: `ros2 topic echo /scan --once`. If it is being published, then execute the SLAM node
```
ros2 launch turtlebot4_navigation slam.launch.py use_sim_time:=true
```
- To visualize the mapping process in rviz, execute the following command
```
ros2 launch turtlebot4_viz view_navigation.launch.py use_sim_time:=true
```
- Now move the robot around in the gazebo world to map the indoor environment completely
- Once the mapping is completed, from the path `turtlebot4_simulator/turtlebot4_gz_bringup/maps` execute the following command to save the map. Change <map_name> to desired name of the map
```
ros2 run nav2_map_server map_saver_cli -f <map_name> --occ 0.65 --free 0.15 --ros-args -p save_map_timeout:=20.0
```

## Localization
- Localization can be achieved by two ways, one via SLAM when the map is partially known, and other Localization algorithms when a map already exists. We will look into Adaptive Monte-Carlo Localization (AMCL) algorithm for localization
- After saving the map, you can stop mapping node. In a new terminal run localization node
```
ros2 launch turtlebot4_navigation localization.launch.py map:=<complete_path_to_map_yaml_config_file> use_sim_time:=true
```
- In rviz, after selecting the map topic, under its properties, set the `Durability` to `Transient Local` for it to be visible
- Before setting the pose estimate or to set a goal pose, make sure that the fixed frame in rviz is set to `map`
- Now use the `2D Pose Estimate` from rviz and drag the arrow in the approximate location where the robot is placed. Move the robot around and observe the robot localizing in the environment

## Navigation
- Once the robot is localized, to provide different goals for the robot, execute the navigation node
```
ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true
```
- Use the `Nav2 Goal` from rviz and drag the arrow to specify the goal in the indoor environment
- **Note**: If the map is already available, and you want to load the simulation along with the localization and navigation functionalities, then you can execute
```
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py nav2:=true slam:=false localization:=true rviz:=true world:=office_area map:=<complete_path_to_map_yaml_config_file>
```

## Task
- Simulate multiple robots in the same Gazebo environment. Please refer to (2) for hints

## Reference
1. https://turtlebot.github.io/turtlebot4-user-manual/tutorials/
2. https://turtlebot.github.io/turtlebot4-user-manual/tutorials/multiple_robots.html
