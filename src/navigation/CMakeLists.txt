cmake_minimum_required(VERSION 3.0.2)
project(navigation)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES navigation
  #  CATKIN_DEPENDS rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/vlad_bag_processor_v3.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Optionally, install launch files (if you have any)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
