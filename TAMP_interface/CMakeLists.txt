cmake_minimum_required(VERSION 2.8.3)
project(TAMP_interface)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
  smach_ros
  task_assembly
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cookie_bot
#  CATKIN_DEPENDS rospy smach smach_ros time
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
