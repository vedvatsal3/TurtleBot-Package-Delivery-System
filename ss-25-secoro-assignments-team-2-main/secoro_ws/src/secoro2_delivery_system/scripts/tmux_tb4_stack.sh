#!/usr/bin/env bash
#sudo apt-get update
#sudo apt-get install -y tmux
#chmod +x src/secoro_bringup/scripts/tmux_tb4_stack.sh
#./src/secoro_bringup/scripts/tmux_tb4_stack.sh

set -euo pipefail

SESSION="tb4"

# neue tmux-Session (detached)
tmux new-session -d -s "$SESSION" -n "George+World"

# Tab 1: George + Welt
tmux send-keys -t "$SESSION:George+World" \
 "ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py \
  world:=secoro_team_2 use_sim_time:=true namespace:=george \
  x:=-2.25 y:=-2.4 z:=0.0 yaw:=0.0 \
  gz_resource_path:=/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2/gazebo/models:/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2/gazebo/worlds:/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2" C-m

# Tab X: George Controller-Reset (Stop -> (optional Unload) -> Load -> Configure -> Activate)
tmux new-window -t "$SESSION" -n "Geo-CTRL-Reset"
tmux send-keys -t "$SESSION:Geo-CTRL-Reset" \
"set -euo pipefail
echo '>>> Warte auf /george/controller_manager ...'
ros2 service list | grep -q '/george/controller_manager' || sleep 2

echo '>>> Stoppe diffdrive_controller (falls aktiv)...'
ros2 service call /george/controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
\"{start_controllers: [], stop_controllers: ['diffdrive_controller'], strictness: 2}\"

echo '>>> (Optional) Unload...'
ros2 service call /george/controller_manager/unload_controller controller_manager_msgs/srv/UnloadController \
\"{name: 'diffdrive_controller'}\" || true

echo '>>> Load...'
ros2 service call /george/controller_manager/load_controller controller_manager_msgs/srv/LoadController \
\"{name: 'diffdrive_controller'}\"

echo '>>> Configure...'
ros2 service call /george/controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController \
\"{name: 'diffdrive_controller'}\"

echo '>>> Activate...'
ros2 service call /george/controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
\"{start_controllers: ['diffdrive_controller'], stop_controllers: [], strictness: 2}\"

echo '>>> Status:'
ros2 service call /george/controller_manager/list_controllers controller_manager_msgs/srv/ListControllers {}
" C-m


# # Tab 2: Fred Spawn
# tmux new-window -t "$SESSION" -n "Fred"
# tmux send-keys -t "$SESSION:Fred" \
#  "sleep 2; \
#   ros2 launch turtlebot4_gz_bringup turtlebot4_spawn.launch.py \
#   namespace:=fred x:=-2.55 y:=2.2 z:=0.0 yaw:=0.0 \
#   world:=secoro_team_2 \
#   gz_resource_path:=/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2/gazebo/models:/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2/gazebo/worlds:/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2" C-m

# Tab 3: George Localization
tmux new-window -t "$SESSION" -n "Geo-Localization"
tmux send-keys -t "$SESSION:Geo-Localization" \
 "sleep 2; \
  ros2 launch turtlebot4_navigation localization.launch.py \
  namespace:=george use_sim_time:=true \
  map:=/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2/maps/secoro_team_2.yaml" C-m


# Tab 4: George RViz
tmux new-window -t "$SESSION" -n "Geo-RViz"
tmux send-keys -t "$SESSION:Geo-RViz" \
 "ros2 launch turtlebot4_viz view_navigation.launch.py use_sim_time:=true namespace:=george" C-m


# Tab 5: George InitialPose
tmux new-window -t "$SESSION" -n "Geo-InitPose"
tmux send-keys -t "$SESSION:Geo-InitPose" \
 "sleep 5; \
  ros2 topic pub --once /george/initialpose geometry_msgs/PoseWithCovarianceStamped \
  '{header: {frame_id: 'map'}, pose: {pose: {position: {x: -2.25, y: -2.4, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.068]}}' " C-m

# Tab 6: George Nav2
tmux new-window -t "$SESSION" -n "Geo-Nav2"
tmux send-keys -t "$SESSION:Geo-Nav2" \
 "ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true namespace:=george" C-m



# Session im aktuellen Terminal anzeigen
tmux attach -t "$SESSION"
