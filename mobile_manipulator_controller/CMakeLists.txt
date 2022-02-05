cmake_minimum_required(VERSION 2.8.3)
project(mobile_manipulator_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

# set(PCL_DIR /home/min/repos/pcl/build)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  message_generation
  task_assembly

  ## image processing
  cv_bridge
  image_transport
  pcl_ros
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.9 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Obstacle2D.msg
  Obstacle3D.msg
  PoseConstraint.msg
  MobileTrajectory.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  plan_arm_motion.srv
  plan_mobile_motion.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES mobile_manipulator_controller
 CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs trajectory_msgs
 DEPENDS system_lib
)

link_directories(${Boost_LIBRARY_DIR})

set(SRCS
  src/Controller.cpp
  src/RosVrepBridge.cpp
  src/Utils.cpp
)

set(HEADS
  include/Utils.h
  include/Controller.h
  include/RosVrepBridge.h
)
add_library(Controller 
  ${SRCS} 
  ${HEADS}
)
target_link_libraries(Controller ${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} rbdl rbdl_urdfreader yaml-cpp trajectory)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions (-Ddof=7)

#ros_tutorial_srv_client
# add_executable(RequestRightArmPlanner src/RequestRightArmPlanner.cpp)
# target_link_libraries(RequestRightArmPlanner ${catkin_LIBRARIES})
# add_dependencies(RequestRightArmPlanner mobile_maipulator_controller_gencpp)

# add_executable(RequestLeftArmPlanner src/RequestLeftArmPlanner.cpp)
# target_link_libraries(RequestLeftArmPlanner ${catkin_LIBRARIES})
# add_dependencies(RequestLeftArmPlanner mobile_maipulator_controller_gencpp)

# add_executable(RequestMobilePlanner src/RequestMobilePlanner.cpp)
# target_link_libraries(RequestMobilePlanner ${catkin_LIBRARIES} )
# add_dependencies(RequestMobilePlanner mobile_maipulator_controller_gencpp)

add_executable(mobile_manipulator_controller src/main.cpp)
target_link_libraries(mobile_manipulator_controller ${PCL_LIBRARY_DIRS} ${catkin_LIBRARIES} Controller)
add_dependencies(mobile_manipulator_controller mobile_maipulator_controller_gencpp)
