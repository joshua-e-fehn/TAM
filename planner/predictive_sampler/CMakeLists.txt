cmake_minimum_required(VERSION 3.0.2)
project(predictive_sampler)

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

## Install Python scripts (if any custom scripts are added later)
# catkin_install_python(PROGRAMS
#   scripts/predictive_sampler_node.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)
