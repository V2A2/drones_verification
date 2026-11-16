cmake_minimum_required(VERSION 3.8)
project(fl_reach)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Interval.msg
  msg/HyperRectangle.msg
  DEPENDENCIES std_msgs builtin_interfaces
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

include_directories(include)

foreach(node verifier collision_checker)
  add_executable(${node} src/${node}.cpp)
  target_compile_features(${node} PUBLIC cxx_std_20)
  ament_target_dependencies(${node} rclcpp std_msgs geometry_msgs)
  target_link_libraries(${node} ${cpp_typesupport_target})
endforeach()

install(TARGETS verifier collision_checker
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/reach_viz
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# rqt reachable set viewer: visualization/ goes into the python package rosidl installs (fl_reach),
# the base class comes from the shared visualization package
install(DIRECTORY visualization DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  PATTERN __pycache__ EXCLUDE)
install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})

ament_package()
