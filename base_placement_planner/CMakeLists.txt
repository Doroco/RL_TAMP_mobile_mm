cmake_minimum_required(VERSION 2.8.3)
project(base_placement_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  visualization_msgs
  message_generation
  eigen_conversions
  kdl_parser  # KDL IK
  trac_ik_lib # track-IK
  urdf
  task_assembly

  tf2
  pcl_ros
  octomap_ros

  ## image processing
  cv_bridge
  image_transport
)

LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../rbdl" )
LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMake" )
INCLUDE(CMake/FindRBDL.cmake)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(RBDL REQUIRED)
find_package(OpenCV REQUIRED)
#it results in the creation of several CMake environment variables that give information about the found package. 
#These environment variables can be utilized later in the CMake script. 



## Generate messages in the 'msg' folder

## Generate added messages and services with any dependencies listed here
generate_messages( # Message file + Service File
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  visualization_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES arm_motion_planner
 CATKIN_DEPENDS roscpp std_msgs eigen_conversions sensor_msgs geometry_msgs trajectory_msgs kdl_parser urdf 
 DEPENDS system_lib
)
# This is required to specify catkin-specific information to the build system 
# which in turn is used to generate pkg-config and CMake files.
# This function must be called before declaring any targets with add_library() or add_executable().


link_directories(${Boost_LIBRARY_DIR})

set(SRCS
    # robot workspace
    src/RobotWorkSpace/PoseQualityMeasurement.cpp
    src/RobotWorkSpace/PoseQualityManipulability.cpp
    src/RobotWorkSpace/PoseQualityExtendedManipulability.cpp
    
    src/RobotWorkSpace/Exception.cpp
    src/RobotWorkSpace/CompressionBZip2.cpp
    src/RobotWorkSpace/CompressionRLE.cpp

    # src/RobotWorkSpace/Grasp.cpp
    # src/RobotWorkSpace/WorkspaceData.cpp 이거 선언하면 컴퍼일러가 문제일으킨다 (불안정한 선언이라고 인식)
    src/RobotWorkSpace/WorkspaceDataArray.cpp
    src/RobotWorkSpace/WorkspaceRepresentation.cpp
    src/RobotWorkSpace/Reachability.cpp
    src/RobotWorkSpace/WorkspaceGrid.cpp
    # Arm motion planners
    src/ArmPlanner/RRTFunction.cpp

    # Utils ... ect...
    src/Utils.cpp

    src/FileIO.cpp
    src/MathTools.cpp
    src/Random.cpp
    # interface 이거 선언하는 순간! 멀티 메인오류뜹니다. 
    # src/interface_client.cpp
    # src/interface_server.cpp
    # src/planningInterface.cpp
)

set(HEADS
    # robot workspace
    include/RobotWorkSpace/YamlConfig.h
    include/RobotWorkSpace/PoseQualityMeasurement.h
    include/RobotWorkSpace/PoseQualityManipulability.h
    include/RobotWorkSpace/PoseQualityExtendedManipulability.h
    include/RobotWorkSpace/RobotWorkSpace.h
    
    include/RobotWorkSpace/Exception.h
    include/RobotWorkSpace/CompressionBZip2.h
    include/RobotWorkSpace/CompressionRLE.h

    include/RobotWorkSpace/WorkspaceData.h
    include/RobotWorkSpace/WorkspaceDataArray.h
    include/RobotWorkSpace/Reachability.h
    # include/RobotWorkSpace/Grasp.h

    include/RobotWorkSpace/WorkspaceRepresentation.h
    include/RobotWorkSpace/WorkspaceGrid.h

    # Arm motion planners
    include/ArmPlanner/NodeTree.h
    include/ArmPlanner/RRTFunction.h
    include/ArmPlanner/RRTNode.h

    # Utils ... ect...
    include/Fwd.h
    include/Utils.h
    include/pcl_headers
    include/FileIO.h
    include/MathTools.h
    include/Random.h
    
    # include/planningInterface.h
)
add_library(MMIRD 
  ${SRCS} 
  ${HEADS} 
)

target_link_libraries(MMIRD ${catkin_LIBRARIES} ${Boost_LIBRARIES} rbdl rbdl_urdfreader yaml-cpp trajectory)

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
)

# Test for Debug 
add_executable(testIRM src/test.cpp)
target_link_libraries(testIRM MMIRD ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(testIRM base_placement_planner_generate_messages_cpp)

# interfaceServer 
add_executable(interfaceServer src/interface_server.cpp)
target_link_libraries(interfaceServer MMIRD ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(interfaceServer base_placement_planner_generate_messages_cpp)

# interfaceServer 
add_executable(interfaceClient src/interface_client.cpp)
target_link_libraries(interfaceClient MMIRD ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(interfaceClient base_placement_planner_generate_messages_cpp)


# Server 
# add_executable(RightArmPlannerServer src/RightArmPlannerServer.cpp)
# target_link_libraries(RightArmPlannerServer CBiRRT ${catkin_LIBRARIES} yaml-cpp)
# add_dependencies(RightArmPlannerServer arm_motion_planner_generate_messages_cpp)

# add_executable(ArmPlannerServer src/Arm_server.cpp)
# target_link_libraries(ArmPlannerServer CBiRRT ${catkin_LIBRARIES} yaml-cpp)
# add_dependencies(ArmPlannerServer arm_motion_planner_generate_messages_cpp)

# add_executable(FrankaArmPlannerServer src/FrankaArmPlannerServer.cpp)
# target_link_libraries(FrankaArmPlannerServer CBiRRT ${catkin_LIBRARIES} yaml-cpp)
# add_dependencies(FrankaArmPlannerServer arm_motion_planner_generate_messages_cpp)


# Client
# add_executable(RightArmPlannerClient src/RightArmPlannerClient.cpp)
# target_link_libraries(RightArmPlannerClient CBiRRT ${catkin_LIBRARIES})
# add_dependencies(RightArmPlannerClient arm_motion_planner_generate_messages_cpp)

# add_executable(ArmPlannerClient src/Arm_client.cpp)
# target_link_libraries(ArmPlannerClient CBiRRT ${catkin_LIBRARIES})
# add_dependencies(ArmPlannerClient arm_motion_planner_generate_messages_cpp)

# add_executable(FrankaArmPlannerClient src/FrankaArmPlannerClient.cpp)
# target_link_libraries(FrankaArmPlannerClient CBiRRT ${catkin_LIBRARIES})
# add_dependencies(FrankaArmPlannerClient arm_motion_planner_generate_messages_cpp)