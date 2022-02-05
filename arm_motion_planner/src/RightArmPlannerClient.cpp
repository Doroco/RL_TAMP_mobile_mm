#include "ros/ros.h"
#include "arm_motion_planner/plan_arm_motion.h"
#include "arm_motion_planner/Obstacle3D.h"
#include "arm_motion_planner/PoseConstraint.h"

#include "Utils.h"

#include <cstdlib>

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
   ros::init(argc, argv, "RightArmPlannerClient");
   ros::NodeHandle nh("~");

   ros::ServiceClient ros_right_arm_planner_client = nh.serviceClient<arm_motion_planner::plan_arm_motion>("/plan_right_arm_motion");
   arm_motion_planner::plan_arm_motion right_arm_srv;


   if(ros::ok()) 
    ros::spinOnce();


   // Joint State
   sensor_msgs::JointState q_service;
   geometry_msgs::Pose pose_service;
   geometry_msgs::Pose2D mobile_service;



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

   // q_service.position[0] = 0.1708;
   // q_service.position[1] = 0.6098;
   // q_service.position[2] = 1.222;
   // q_service.position[3] = 2.689;
   // q_service.position[4] = -2.240;
   // q_service.position[5] = -0.5522;
   // q_service.position[6] = -0.1092;
   q_service.position[0] = 0.0;
   q_service.position[1] = 0.0;
   q_service.position[2] = 0.0;
   q_service.position[3] = 0.0;
   q_service.position[4] = 0.0;
   q_service.position[5] = 0.0;
   q_service.position[6] = 0.0;


   // Target Pose
   // pose_service.position.x = 4.65;
   // pose_service.position.y = -2.704;
   // pose_service.position.z = 0.8114;

   pose_service.position.x = 0.4250;
   pose_service.position.y = 0.0;
   pose_service.position.z = 0.83;


   // pose_service.position.x = 0.1;
   // pose_service.position.y = -0.25;
   // pose_service.position.z = 0.8209;

   //Matrix3d rot_d = Rotate_with_X(M_PI/2.0)*Rotate_with_Z(M_PI/2.0);
   tf2::Quaternion q_rot;
   //q_rot.setRPY(0.0, M_PI/2.0, M_PI/2.0); // when the object in X-direction
   q_rot.setRPY(0.0, M_PI/2.0, M_PI/2.0); // M_PI/2.0, M_PI/2.0
   pose_service.orientation.x = q_rot.getX();
   pose_service.orientation.y = q_rot.getY();
   pose_service.orientation.z = q_rot.getZ();
   pose_service.orientation.w = q_rot.getW();

   // Mobile pose
   // mobile_service.x = 4.9;
   // mobile_service.y = -2.3;
   // mobile_service.theta = -M_PI/2.0;

   mobile_service.x = 0.0;
   mobile_service.y = -0.0;
   mobile_service.theta = 0.0;

   // Obstacle
   arm_motion_planner::Obstacle3D Obstacle1;  // Table 
   Obstacle1.Box_dimension.x = 0.35;
   Obstacle1.Box_dimension.y = 0.56;
   Obstacle1.Box_dimension.z = 0.345;
   Obstacle1.Box_pose.position.x = 0.575;
   Obstacle1.Box_pose.position.y = 0.0;
   Obstacle1.Box_pose.position.z = 0.365;
   Obstacle1.Box_pose.orientation.x = 0.0;
   Obstacle1.Box_pose.orientation.y = 0.0;
   Obstacle1.Box_pose.orientation.z = 0.0;
   Obstacle1.Box_pose.orientation.w = 1.0;

   arm_motion_planner::Obstacle3D Obstacle2; // Refrigerator
   Obstacle2.Box_dimension.x = 0.25;
   Obstacle2.Box_dimension.y = 0.25;
   Obstacle2.Box_dimension.z = 0.68;
   Obstacle2.Box_pose.position.x = 2.35;
   Obstacle2.Box_pose.position.y = 1.75;
   Obstacle2.Box_pose.position.z = 0.69;
   Obstacle2.Box_pose.orientation.x = 0.0;
   Obstacle2.Box_pose.orientation.y = 0.0;
   Obstacle2.Box_pose.orientation.z = 0.0;
   Obstacle2.Box_pose.orientation.w = 1.0;

   arm_motion_planner::PoseConstraint pose_const;
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



   // Get joint state & target pose
   right_arm_srv.request.current_arm_joint_state = q_service;
   //right_arm_srv.request.target_ee_pose = pose_service;
   right_arm_srv.request.Pose_bound = pose_const;
   right_arm_srv.request.Obstacles3D.push_back(Obstacle1);
   right_arm_srv.request.Obstacles3D.push_back(Obstacle2);
   right_arm_srv.request.current_mobile_state = mobile_service;
   right_arm_srv.request.interpolate_path.data = true;

   if (ros_right_arm_planner_client.call(right_arm_srv)) // call the service and return the response value
   {
      //ROS_INFO("send srv, srv.Request.a and b : %1d, %1d", (long int)srv.request.a, (long int)srv.request.b);
      //ROS_INFO("recieve srv, srv.Response.result : %1d", (long int)srv.response.result);
   }
   else
   {
      ROS_ERROR("Failed to call service ros_tutorial_srv");
      return 1;
   }

   return 0;
}


