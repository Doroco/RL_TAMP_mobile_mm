#include "ros/ros.h"
#include "task_assembly/plan_arm_motion.h"
#include "task_assembly/Obstacle3D.h"
#include "task_assembly/PoseConstraint.h"
#include "task_assembly/GoalStateInfo.h"

#include "trajectory_msgs/JointTrajectory.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" // for Quaternion 

#include "Utils.h"

#include <cstdlib>

#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

const std::string Joint_name[7] =
    {
        "panda_joint1", "panda_joint2", "panda_joint3",
        "panda_joint4", "panda_joint5", "panda_joint6",
        "panda_joint7"};

int main(int argc, char **argv)
{
   ros::init(argc, argv, "ArmPlannerClient");
   ros::NodeHandle nh("~");

   ros::ServiceClient arm_planner_client = nh.serviceClient<task_assembly::plan_arm_motion>("/plan_arm_motion");
   task_assembly::plan_arm_motion arm_srv;

   ros::Publisher right_joint_trajectory_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/panda/joint_trajectory", 1);
   trajectory_msgs::JointTrajectory right_joint_trajectory_;

   // Joint State
   sensor_msgs::JointState q_service;
   geometry_msgs::Pose pose_service;
   geometry_msgs::Pose2D mobile_service;
   // task_assembly::GoalStateInfo goal_state;



   q_service.name.resize(7);
   q_service.position.resize(7);
   q_service.velocity.resize(7);
   q_service.effort.resize(7);

   for (int i = 0; i < 7; i++)
   {
      q_service.name[i] = Joint_name[i];
      q_service.position[i] = 0.1;
      q_service.velocity[i] = 0.0;
      q_service.effort[i] = 0.0;
   }
   // pose 1
   q_service.position[0] = 0.0;
   q_service.position[1] = 0.0;
   q_service.position[2] = 0.0;
   q_service.position[3] = -1.5708;
   q_service.position[4] = 0.0;
   q_service.position[5] = 1.5708;
   q_service.position[6] = 0.0;
   //  -0.384498     2.70636    0.947735    0.563895    0.939746   -0.814296 -0.00618101

   // q_service.position[0] = M_PI/6.0;
   // q_service.position[1] = M_PI/6.0;
   // q_service.position[2] = M_PI/6.0;
   // q_service.position[3] = M_PI/6.0;
   // q_service.position[4] = M_PI/6.0;
   // q_service.position[5] = M_PI/6.0;
   // q_service.position[6] = M_PI/6.0;



   // Target Pose
   // pose_service.position.x = 0.375;
   // pose_service.position.y = 0.275;
   // pose_service.position.z = 0.825;

   // pose_service.position.x = 0.1;
   // pose_service.position.y = -0.25;
   // pose_service.position.z = 0.8209;

   pose_service.position.x = 2.0;
   pose_service.position.y = 1.95;
   pose_service.position.z = 0.834;

   //Matrix3d rot_d = Rotate_with_X(M_PI/2.0)*Rotate_with_Z(M_PI/2.0);d
   tf2::Quaternion q_rot;
   q_rot.setRPY(0.0, M_PI/2.0, -M_PI/2.0); // 0.0, M_PI/2.0, -M_PI/2.0 : when the object in X-direction
   //q_rot.setRPY(-M_PI/2.0, M_PI, 0.0);
   


   pose_service.orientation.x = q_rot.getX();
   pose_service.orientation.y = q_rot.getY();
   pose_service.orientation.z = q_rot.getZ();
   pose_service.orientation.w = q_rot.getW();
   // std::cout << q_rot.getX()<< "\t" << q_rot.getY() << "\t" << q_rot.getZ() << "\t" << q_rot.getW() << std::endl;
   // Mobile pose
   // mobile_service.x = 4.9;
   // mobile_service.y = -2.3;
   // mobile_service.theta = -M_PI/2.0;

   mobile_service.x = 0.0;
   mobile_service.y = 0.0;
   mobile_service.theta = 0.0;

   // Obstacle
   task_assembly::Obstacle3D Obstacle1; // Table 
   Obstacle1.Box_dimension.x = 0.35;
   Obstacle1.Box_dimension.y = 0.56;
   Obstacle1.Box_dimension.z = 0.345;
   Obstacle1.Box_pose.position.x = 5.575;
   Obstacle1.Box_pose.position.y = 5.0;
   Obstacle1.Box_pose.position.z = 115.365;
   Obstacle1.Box_pose.orientation.x = 0.0;
   Obstacle1.Box_pose.orientation.y = 0.0;
   Obstacle1.Box_pose.orientation.z = 0.0;
   Obstacle1.Box_pose.orientation.w = 1.0;

 

   task_assembly::Obstacle3D Obstacle2; // Refrigerator
   Obstacle2.Box_dimension.x = 0.25;
   Obstacle2.Box_dimension.y = 0.25;
   Obstacle2.Box_dimension.z = 0.68;
   Obstacle2.Box_pose.position.x = 5.35;
   Obstacle2.Box_pose.position.y = 5.75;
   Obstacle2.Box_pose.position.z = 115.69;
   Obstacle2.Box_pose.orientation.x = 0.0;
   Obstacle2.Box_pose.orientation.y = 0.0;
   Obstacle2.Box_pose.orientation.z = 0.0;
   Obstacle2.Box_pose.orientation.w = 1.0;

   task_assembly::PoseConstraint pose_const;
   pose_const.constrain_pose.data = false;

   // 4.6387 -2.5016 0.8325
   // 4.75 -2.78 0.8114
   pose_const.position_bound_lower.x = -0.35;
   pose_const.position_bound_lower.y = -0.15;
   pose_const.position_bound_lower.z = -0.15;

   pose_const.position_bound_upper.x = 0.35;
   pose_const.position_bound_upper.y = 0.15;
   pose_const.position_bound_upper.z = 0.15;

   pose_const.orientation_bound_lower.x = -0.05;
   pose_const.orientation_bound_lower.y = -0.05;
   pose_const.orientation_bound_lower.z = -0.05;

   pose_const.orientation_bound_upper.x = 0.05;
   pose_const.orientation_bound_upper.y = 0.05;
   pose_const.orientation_bound_upper.z = 0.05;


   geometry_msgs::Pose graspHandleResult;

   // Test Set  (T_BO)
   graspHandleResult.position.x = 0.8327;// trans.x();  0.662;
   graspHandleResult.position.y = -0.1273;//trans.y();  -1.71e-12;
   graspHandleResult.position.z = 0.779045;//trans.z();

   graspHandleResult.orientation.x =  -0.00110387;//quat.x();
   graspHandleResult.orientation.y =  -0.706509;//quat.y();
   graspHandleResult.orientation.z =  0.00201453;//quat.z();
   graspHandleResult.orientation.w =  0.707237;//quat.w();

   // graspHandleResult.orientation.x =  -0.498605;//quat.x();
   // graspHandleResult.orientation.y =  -0.496106;//quat.y();
   // graspHandleResult.orientation.z =  0.504848;//quat.z();
   // graspHandleResult.orientation.w =  0.500401;//quat.w();

   // goal information
   // goal_state.Pose_info.data = true;
   // goal_state.Goal_pose = graspHandleResult;


   // Get joint state & target pose
   arm_srv.request.current_joint_state = q_service;
   arm_srv.request.target_ee_pose = graspHandleResult;
   arm_srv.request.Pose_bound = pose_const;  
   arm_srv.request.Obstacles3D.push_back(Obstacle1);
   arm_srv.request.Obstacles3D.push_back(Obstacle2);
   arm_srv.request.current_mobile_state = mobile_service;
   arm_srv.request.interpolate_path.data = true;


   while(right_joint_trajectory_pub_.getNumSubscribers() == 0)
   {
     
   }

   if (arm_planner_client.call(arm_srv)) // call the service and return the response value
   {
      right_joint_trajectory_.joint_names = arm_srv.response.joint_trajectory.joint_names;
      right_joint_trajectory_.points = arm_srv.response.joint_trajectory.points;
      right_joint_trajectory_pub_.publish(right_joint_trajectory_);
   }
   else
   {
      ROS_ERROR("Failed to call service ros_tutorial_srv");
      return 1;
   }

   ros::Rate r(10);
   int shutdown_cnt = 5;
   while(true)
   {
      r.sleep();
      ros::spinOnce();
      if(--shutdown_cnt == 0) break;
   }


   return 0;
}