cmake_minimum_required(VERSION 3.0.2)
project(multi_car_interaction)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Mark executable scripts (Python scripts)
catkin_install_python(PROGRAMS
  src/multi_car_obstacle_publisher.py
  src/car_collision_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files  
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
