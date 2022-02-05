cmake_minimum_required(VERSION 2.8.3)
project(arm_motion_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  message_generation
  kdl_parser  # KDL IK
  trac_ik_lib # track-IK
  urdf
  task_assembly
)

LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../rbdl" )
LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMake" )
INCLUDE(CMake/FindRBDL.cmake)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(RBDL REQUIRED)
#it results in the creation of several CMake environment variables that give information about the found package. 
#These environment variables can be utilized later in the CMake script. 



## Generate messages in the 'msg' folder
add_message_files(
  FILES
  AttachBoxInfo.msg
  Obstacle3D.msg
  PoseConstraint.msg
  GoalStateInfo.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  plan_arm_motion.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages( # Message file + Service File
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES arm_motion_planner
 CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs trajectory_msgs kdl_parser urdf 
 DEPENDS system_lib
)
# This is required to specify catkin-specific information to the build system 
# which in turn is used to generate pkg-config and CMake files.
# This function must be called before declaring any targets with add_library() or add_executable().


link_directories(${Boost_LIBRARY_DIR})

set(SRCS
    src/ArmPlanner.cpp
    src/RRTFunction.cpp
    src/Utils.cpp
)

set(HEADS
    include/ArmPlanner.h
    include/Fwd.h
    include/NodeTree.h
    include/RRTFunction.h
    include/RRTNode.h
    include/Utils.h
)
add_library(CBiRRT 
  ${SRCS} 
)

target_link_libraries(CBiRRT ${catkin_LIBRARIES} ${Boost_LIBRARIES} rbdl rbdl_urdfreader yaml-cpp trajectory)

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
)

# Server 
# add_executable(RightArmPlannerServer src/RightArmPlannerServer.cpp)
# target_link_libraries(RightArmPlannerServer CBiRRT ${catkin_LIBRARIES} yaml-cpp)
# add_dependencies(RightArmPlannerServer arm_motion_planner_generate_messages_cpp)

add_executable(ArmPlannerServer src/Arm_server.cpp)
target_link_libraries(ArmPlannerServer CBiRRT ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(ArmPlannerServer arm_motion_planner_generate_messages_cpp)

# add_executable(FrankaArmPlannerServer src/FrankaArmPlannerServer.cpp)
# target_link_libraries(FrankaArmPlannerServer CBiRRT ${catkin_LIBRARIES} yaml-cpp)
# add_dependencies(FrankaArmPlannerServer arm_motion_planner_generate_messages_cpp)


# Client
# add_executable(RightArmPlannerClient src/RightArmPlannerClient.cpp)
# target_link_libraries(RightArmPlannerClient CBiRRT ${catkin_LIBRARIES})
# add_dependencies(RightArmPlannerClient arm_motion_planner_generate_messages_cpp)

add_executable(ArmPlannerClient src/Arm_client.cpp)
target_link_libraries(ArmPlannerClient CBiRRT ${catkin_LIBRARIES})
add_dependencies(ArmPlannerClient arm_motion_planner_generate_messages_cpp)

# add_executable(FrankaArmPlannerClient src/FrankaArmPlannerClient.cpp)
# target_link_libraries(FrankaArmPlannerClient CBiRRT ${catkin_LIBRARIES})
# add_dependencies(FrankaArmPlannerClient arm_motion_planner_generate_messages_cpp)