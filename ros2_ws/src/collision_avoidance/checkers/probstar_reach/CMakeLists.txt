cmake_minimum_required(VERSION 3.8)
project(probstar_reach)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ProbStar.msg
  DEPENDENCIES std_msgs builtin_interfaces
)

# rosidl already installs the python package probstar_reach (for probstar_reach.msg) with
# ament_python_install_package, a second call with the same name fails, so add our modules to it
install(FILES probstar_reach/utils.py probstar_reach/verifier.py
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME})

install(PROGRAMS probstar_reach/verifier.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME verifier)
install(PROGRAMS scripts/reach_viz
  DESTINATION lib/${PROJECT_NAME})

# rqt reachable set viewer: visualization/ goes into the python package probstar_reach (probstar_reach.visualization),
# the base class comes from the shared visualization package
install(DIRECTORY visualization DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  PATTERN __pycache__ EXCLUDE)
install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
