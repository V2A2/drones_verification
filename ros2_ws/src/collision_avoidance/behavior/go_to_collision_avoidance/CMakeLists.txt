cmake_minimum_required(VERSION 3.8)
project(collision_avoidance_behavior)

find_package(ament_cmake REQUIRED)
find_package(as2_behavior REQUIRED)
find_package(as2_behaviors_motion REQUIRED)
find_package(as2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp REQUIRED)
find_package(as2_core REQUIRED)
find_package(as2_motion_reference_handlers REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# GoTo behavior with collision avoidance (reuses the GoTo plugins of as2_behaviors_motion)
add_executable(go_to_collision_avoidance_behavior_node
  src/go_to_collision_avoidance_behavior.cpp
  src/go_to_collision_avoidance_behavior_node.cpp)
target_compile_features(go_to_collision_avoidance_behavior_node PUBLIC cxx_std_20)
ament_target_dependencies(go_to_collision_avoidance_behavior_node
  rclcpp rclcpp_action pluginlib std_msgs geometry_msgs as2_core as2_msgs as2_behavior
  as2_behaviors_motion as2_motion_reference_handlers)
install(TARGETS go_to_collision_avoidance_behavior_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
