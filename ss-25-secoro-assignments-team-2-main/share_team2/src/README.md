# Assignment 3 – Team 2


## 1. spawn 2 robots with their docking stations and robot1 with navigation and localization enabled

1. spawn robot1 in room Tabl_251 and the world model (TAB office)  in gazebo 

```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py     nav2:=true     slam:=false     localization:=true     rviz:=false     world:=secoro_team_2  namespace:=robot1 x:=-2.55 y:=2.2     gz_resource_path:=/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2/gazebo/models:/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2/gazebo/worlds:/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2 map:=/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2/maps/secoro_team_2.yaml
```
2. spawn robot2 in the same world in room 256
```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_spawn.launch.py namespace:=robot2 x:=-2.25 y:=-2.4 world:=secoro_team_2 gz_resource_path:=/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2/gazebo/models:/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2/gazebo/worlds:/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2 
```

3. Estimate the initila postition of robot1 (paasing initial pose)
```bash
ros2 topic pub /robot1/initialpose geometry_msgs/PoseWithCovarianceStamped "
header:
  frame_id: 'map'
pose:
  pose:
    position:
      x: -2.55
      y: 2.2
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.068]"
```

## 2. Run the delivery_interface_commander and interact with robot1 
enter (pickup room, delivery room) confirm(pickup, delivery)
Run the delivery_interface_commander: 
```bash
ros2 run delivery_interface delivery_commander
```


## Alternative: 

## 1. spawn 2 robots with their docking stations and

1. spawn robot1 in room Tabl_251 and the world model (TAB office)  in gazebo 
```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py world:=secoro_team_2  namespace:=robot1 x:=-2.55 y:=2.2 gz_resource_path:=/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2/gazebo/models:/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2/gazebo/worlds:/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2 

```
2. spawn robot2 in the same world in room 256
```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_spawn.launch.py namespace:=robot2 x:=-2.25 y:=-2.4 world:=secoro_team_2 gz_resource_path:=/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2/gazebo/models:/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2/gazebo/worlds:/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2 
```

## 2. launch localization and navigation for robot1

1. Launch localization for robot1


```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2/maps/secoro_team_2.yaml namespace:=/robot1
```

2. Estimate the initila postition of robot1 (paasing initial pose)
```bash
ros2 topic pub /robot1/initialpose geometry_msgs/PoseWithCovarianceStamped "
header:
  frame_id: 'map'
pose:
  pose:
    position:
      x: -2.55
      y: 2.2
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.068]"
```

3. Excute the navigation Node 
```bash
ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true namespace:=/robot1
```
## 2. Run the delivery_interface_commander and interact with robot1 
enter (pickup room, delivery room) confirm(pickup, delivery)
Run the delivery_interface_commander: 
```bash
ros2 run delivery_interface delivery_commander
```


# Assignment 2 – Team 2

## 📦 Deliverables

---

### ✅ Task 1

- Submitted via **Stud.IP** on **07.05.**

---

### ✅ Task 2

- See model definition: [`secoro_team_2.fpm2`](./fpm2_models/secoro_team_2.fpm2)

After writing the `secoro_team_2.fpm2`, we generated the models and world files using Docker and textX:

#### 🔧 Step 1 – Run the Docker container:

```bash
docker run -v ./jsonld_models:/usr/src/app/output \
           -v ./fpm2_models:/usr/src/app/models \
           -u $(id -u):$(id -g) \
           -it floorplan:latest bash
```

#### 🔧 Step 2 – Generate JSON-LD model:

```bash
textx generate models/secoro_team_2.fpm2 --target json-ld -o output/
```

#### 🔧 Step 3 – Generate Gazebo world with scenery builder:

```bash
docker run -v ./world_models_secoro_team_2:/usr/src/app/output \
           -v ./jsonld_models:/usr/src/app/models \
           ghcr.io/secorolab/scenery_builder:devel
```

---

### ✅ Task 3

- Result images: [`deliverables_images`](./deliverables_images)
- World model output: [`turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2`](./turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2)
- Or backup: `Copy of world_models_secoro_team_2`

We did **not edit** any launch file. To start the simulation, we ran:

```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py     nav2:=true     slam:=false     localization:=true     rviz:=true     world:=secoro_team_2     gz_resource_path:=/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2/gazebo/models:/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2/gazebo/worlds:/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2 map:=/home/sireen/secoro/secoro_ws/src/turtlebot4_simulator/turtlebot4_gz_bringup/world_models_secoro_team_2/maps/secoro_team_2.yaml
```

In another terminal tab, we ran:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

---

### 🧭 In RViz

Use the **2D Pose Estimate** tool to initialize the TurtleBot's pose.  
After doing this:
- The TurtleBot will appear in RViz
- TF errors in the **RobotModel** display will disappear

---

### 🖼️ Result

You can now see the TurtleBot in **Gazebo** and **RViz**, moving inside our `secoro_team_2` office environment using `teleop_twist_keyboard`.

✅ See the result: [`deliverables_images/secoro_2_tb_env_gz_rvz.png`](./deliverables_images/secoro_2_tb_env_gz_rvz.png)

---


### ✅ Task 4

We assumed there is **no presentation required** for each assignment, only:
- An oral exam after each assignment
- A final presentation at the end of the semester
---
### ⚠️ YAML Parse Error & Fix

We encountered an error after generating the map with the scenery builder and by passing the map to rviz:

```bash
docker run -v ./world_models_secoro_team_2:/usr/src/app/output \
           -v ./jsonld_models:/usr/src/app/models \
           ghcr.io/secorolab/scenery_builder:devel
```

The map server failed to process the generated `secoro_team_2.yaml` file with the following error:

```
[map_server-25] [INFO] [1747135733.703406014] [map_server]: Configuring
[map_server-25] [INFO] [1747135733.703486091] [map_io]: Loading yaml file: /home/sireen/FloorPlan-DSL/world_models_secoro_team_2/maps/secoro_team_2.yaml
[map_server-25] [ERROR] [1747135733.703958113] [map_io]: Failed processing YAML file /home/sireen/FloorPlan-DSL/world_models_secoro_team_2/maps/secoro_team_2.yaml at position (3:8) for reason: yaml-cpp: error at line 4, column 9: Failed to parse YAML tag 'negate' for reason: bad conversion
[map_server-25] [ERROR] [1747135733.704042225] [map_server]: Caught exception in callback for transition 10
[map_server-25] [ERROR] [1747135733.704069003] [map_server]: Original error: Failed to load map yaml file: /home/sireen/FloorPlan-DSL/world_models_secoro_team_2/maps/secoro_team_2.yaml
[map_server-25] [WARN] [1747135733.704109410] [map_server]: Error occurred while doing error handling.
[map_server-25] [FATAL] [1747135733.704126847] [map_server]: Lifecycle node map_server does not have error state implemented
[lifecycle_manager-27] [ERROR] [1747135733.722802466] [lifecycle_manager_localization]: Failed to change state for node: map_server
```

#### ✅ Cause:
The generated file had:
```yaml
negate: 0.0
```

But `map_server` expects an **integer**, not a float.

#### ✅ Fix:
We manually edited the YAML:

```yaml
negate: 0
```

After this, the map loaded successfully in RViz.

🖼️ Old image (`mapping_attemp.png`) shows mapping mode we used **before** solving the YAML error — not needed anymore since the corrected map loads successfully.


Furniture and obstacles will be added to the map and env later.
