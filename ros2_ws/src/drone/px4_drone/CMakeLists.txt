cmake_minimum_required(VERSION 3.8)
project(px4_drone)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# State estimate in the shared earth frame (state/pose, state/twist), for the verifiers
add_executable(state_estimate src/state_estimate.cpp)
# Random walk mission, a PX4 external mode
add_executable(random_walk src/random_walk.cpp)

foreach(target state_estimate random_walk)
  target_include_directories(${target} PRIVATE include)
  target_link_libraries(${target}
    px4_ros2_cpp::px4_ros2_cpp rclcpp::rclcpp Eigen3::Eigen
    ${px4_msgs_TARGETS} ${geometry_msgs_TARGETS} ${std_msgs_TARGETS})
endforeach()

install(TARGETS state_estimate random_walk DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
