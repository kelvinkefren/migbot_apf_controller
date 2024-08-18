cmake_minimum_required(VERSION 3.0.2)
project(migbot_apf_controller)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf  # Add tf since you're using it for transformations
  dynamic_obstacle_avoidance  # Add this since you're using RobotState from this package
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy std_msgs tf dynamic_obstacle_avoidance
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
install(PROGRAMS
  scripts/wrench_calculator.py
  scripts/gazebo_scenario.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install the configuration files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
