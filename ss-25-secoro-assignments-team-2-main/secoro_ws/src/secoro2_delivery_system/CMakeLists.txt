cmake_minimum_required(VERSION 3.8)
project(secoro2_delivery_system)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Signin.srv"
  "srv/RequestDelivery.srv"
  "srv/DeliveryCommand.srv"
  "srv/DockRequest.srv"
  "srv/ReceiverConfirm.srv"
  "srv/ReceiverSelectRoom.srv"
  "srv/SenderConfirm.srv"
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

add_executable(delivery_bt_server src/servers/delivery_bt_server.cpp)
ament_target_dependencies(delivery_bt_server
  rclcpp rclcpp_action std_msgs std_srvs geometry_msgs sensor_msgs nav2_msgs nav_msgs tf2_ros tf2_geometry_msgs behaviortree_cpp_v3 irobot_create_msgs
)

add_executable(request_delivery src/clients/request_delivery_cli.cpp)
ament_target_dependencies(request_delivery rclcpp std_msgs std_srvs)

add_executable(sender_client src/clients/sender_client_cli.cpp)
ament_target_dependencies(sender_client rclcpp std_msgs std_srvs)

add_executable(receiver_client src/clients/receiver_client_cli.cpp)
ament_target_dependencies(receiver_client rclcpp std_msgs std_srvs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(delivery_bt_server ${cpp_typesupport_target})
target_link_libraries(request_delivery ${cpp_typesupport_target})
target_link_libraries(sender_client ${cpp_typesupport_target})
target_link_libraries(receiver_client ${cpp_typesupport_target})



install(PROGRAMS
  scripts/tb4_stack.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/tmux_tb4_stack.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  delivery_bt_server
  request_delivery
  sender_client
  receiver_client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY bt_trees launch srv config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_cli_input test/test_cli_input.py)
  ament_add_pytest_test(test_bt_doorsim test/test_bt_doorsim.py)
  ament_add_pytest_test(test_inaccessible_return test/test_inaccessible_return.py)
  ament_add_pytest_test(test_end_to_end_sim test/test_end_to_end_sim.py)
endif()

ament_package()
