#include "ros/ros.h"

#include "../include/RobotWorkSpace/RobotWorkSpace.h"
#include "../include/RobotWorkSpace/PoseQualityExtendedManipulability.h"
#include "../include/RobotWorkSpace/WorkspaceRepresentation.h"
#include "task_assembly/base_placement.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "interfaceClient ");
    ros::NodeHandle nh("~");

    ros::ServiceClient bpClient = nh.serviceClient<task_assembly::base_placement>("/plan_base_sampling");
    task_assembly::base_placement bpSrv;
    bpSrv.request.minEntry.data = 20;
    bpSrv.request.eGridySearch.data = false;

   if (bpClient.call(bpSrv)) // call the service and return the response value
   {

   }
   else
   {
      ROS_ERROR("Failed to call service ros_tutorial_srv");
      return 1;
   }

    return 0;
}