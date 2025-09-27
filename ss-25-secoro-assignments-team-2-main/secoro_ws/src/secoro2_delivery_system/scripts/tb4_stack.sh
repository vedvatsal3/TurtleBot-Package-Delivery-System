#!/usr/bin/env bash
set -u 

gnome-terminal \
  --tab --title="George+World" \
    -e "bash -lc 'ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py world:=secoro_team_2 use_sim_time:=true namespace:=george x:=-2.25 y:=-2.4 z:=0.0 yaw:=0.0 gz_resource_path:=/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2/gazebo/models:/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2/gazebo/worlds:/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2; exec bash'" \
  --tab --title="Geo-Localization" \
    -e "bash -lc 'sleep 2; ros2 launch turtlebot4_navigation localization.launch.py namespace:=george use_sim_time:=true map:=/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2/maps/secoro_team_2.yaml; exec bash'" \
  --tab --title="Geo-RViz" \
    -e "bash -lc 'ros2 launch turtlebot4_viz view_navigation.launch.py use_sim_time:=true namespace:=george; exec bash'" \
  --tab --title="Geo-Nav2" \
    -e "bash -lc 'ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true namespace:=george; exec bash'" \
  --tab --title="Geo-InitPose" \
    -e "bash -lc 'sleep 4; ros2 topic pub --once george/initialpose geometry_msgs/PoseWithCovarianceStamped \"{header: {frame_id: \\\"map\\\"}, pose: {pose: {position: {x: -2.25, y: -2.4, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.068]}}\"; exec bash'"
