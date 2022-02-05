cmake_minimum_required(VERSION 2.8.3)
project(mobile_motion_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  task_assembly
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Obstacle2D.msg
  MobileTrajectory.msg

)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  plan_mobile_motion.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES mobile_motion_planner
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs task_assembly
#  DEPENDS system_lib
)

link_directories(${Boost_LIBRARY_DIR})

set(SRCS
    src/MobilePlanner.cpp
    src/NHRRTFunction.cpp
    src/Utils.cpp
)

set(HEADS
    include/MobilePlanner.h
    include/Fwd.h
    include/NHRRTFunction.h
    include/Utils.h
)
add_library(NHRRT 
  ${SRCS} 
  ${HEADS}
)
target_link_libraries(NHRRT ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp trajectory)

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
)


#ros_tutorial_srv_server
add_executable(MobilePlannerServer src/MobilePlannerServer.cpp)
target_link_libraries(MobilePlannerServer NHRRT ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(MobilePlannerServer mobile_motion_planner_gencpp)

#ros_tutorial_srv_client
add_executable(MobilePlannerClient src/MobilePlannerClient.cpp)
target_link_libraries(MobilePlannerClient NHRRT ${catkin_LIBRARIES})
add_dependencies(MobilePlannerClient mobile_motion_planner_gencpp)

