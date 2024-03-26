################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(rover_slam)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs  
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
