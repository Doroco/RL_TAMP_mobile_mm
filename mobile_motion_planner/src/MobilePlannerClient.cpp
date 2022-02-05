#include "ros/ros.h"
#include "task_assembly/plan_mobile_motion.h"
#include "task_assembly/Obstacle2D.h"

#include "Utils.h"
#include <cstdlib>

#include "geometry_msgs/Pose2D.h"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "MobilePlannerClient");
   ros::NodeHandle nh("~");

   ros::ServiceClient ros_mobile_planner_client = nh.serviceClient<task_assembly::plan_mobile_motion>("/plan_mobile_motion");
   task_assembly::plan_mobile_motion mobile_srv;

   

   geometry_msgs::Pose2D target_pose_service;
   geometry_msgs::Pose2D current_pose_service;

   // Current pose
   current_pose_service.x = 0.0;
   current_pose_service.y = 0.0;
   current_pose_service.theta = 0.0;

   // Target Pose
   target_pose_service.x = 4.9;
   target_pose_service.y = -2.3;
   target_pose_service.theta = -M_PI/2.0;

   // Obstacle
   task_assembly::Obstacle2D Obstacle1;
   Obstacle1.x.data = 2.7251;
   Obstacle1.y.data = -0.5276;
   Obstacle1.radius.data = 0.6;

   task_assembly::Obstacle2D Obstacle2;
   Obstacle2.x.data = 4.5502;
   Obstacle2.y.data = 2.0;
   Obstacle2.radius.data = 0.6;

   task_assembly::Obstacle2D Obstacle3;
   Obstacle3.x.data = 4.5;
   Obstacle3.y.data = -2.825;
   Obstacle3.radius.data = 0.1;

   
   // Get current state & target pose
   mobile_srv.request.current_mobile_state = current_pose_service;
   mobile_srv.request.target_mobile_pose = target_pose_service;
   mobile_srv.request.Obstacles2D.push_back(Obstacle1);
   mobile_srv.request.Obstacles2D.push_back(Obstacle2);
   mobile_srv.request.Obstacles2D.push_back(Obstacle3);


   if (ros_mobile_planner_client.call(mobile_srv)) // call the service and return the response value
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