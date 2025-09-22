cmake_minimum_required(VERSION 3.0.2)
project(tam_sampling_planner)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  dynamic_reconfigure
  f110_msgs
)

## Generate dynamic reconfigure options
generate_dynamic_reconfigure_options(
  cfg/dyn_tam_sampling_tuner.cfg
)

## Catkin package configuration
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    dynamic_reconfigure
    f110_msgs
)

## Build configuration
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  src/tam_sampling_node.py
  src/dynamic_tam_sampling_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.yml"
)
