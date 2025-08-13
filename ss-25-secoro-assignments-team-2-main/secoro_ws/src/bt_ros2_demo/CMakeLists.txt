cmake_minimum_required(VERSION 3.8)
project(bt_ros2_demo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(bt_task_planner
  src/main.cpp
  src/dock_action.cpp
  src/undock_action.cpp
  src/dock_status.cpp
)

ament_target_dependencies(bt_task_planner
  rclcpp
  behaviortree_cpp
  behaviortree_ros2
  nav2_msgs
  irobot_create_msgs
  std_msgs
  ament_index_cpp
)

target_include_directories(bt_task_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS bt_task_planner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY include/
  DESTINATION include)

ament_package()