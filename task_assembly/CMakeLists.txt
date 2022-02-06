cmake_minimum_required(VERSION 2.8.3)
project(task_assembly)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
)

add_message_files(
 FILES
 BoundingBoxes3d.msg
 BoundingBox3d.msg
 target.msg
 MobileTrajectory.msg
 Obstacle2D.msg
 ObstacleBox2D.msg
 AttachBoxInfo.msg
 GoalStateInfo.msg
 Obstacle3D.msg
 PoseConstraint.msg
 CloudIndexed.msg
 CloudSamples.msg
 CloudSources.msg
 GraspConfig.msg
 GraspConfigList.msg
 SamplesMsg.msg
)

add_service_files(
  FILES
  detect_grasps.srv
  door_open_planner.srv
  plan_arm_motion.srv
  plan_mobile_motion.srv
  base_placement.srv
  sim_request.srv
)

generate_messages(
 DEPENDENCIES
 std_msgs
 geometry_msgs
 sensor_msgs
 trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
)
