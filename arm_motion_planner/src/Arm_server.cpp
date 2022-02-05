#include "ros/ros.h"
#include "task_assembly/plan_arm_motion.h"
#include "task_assembly/base_placement.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"

#include "ArmPlanner.h"
#include "YamlConfig.h"

// Input : current q / current base pose / target pose / obstacle / pose constraint
// Output : trajectory_msgs/JointTrajectory/points/positions


class RosPlanningInterface
{
public:
    RosPlanningInterface(ros::NodeHandle & nh, YAMLConfig &config)
    : arm_planner_(config)    
    {
        nh.param("/urdf_param", arm_planner_.urdf_param_, std::string("/robot_description"));
        //cout << arm_planner_.urdf_param_ << endl;
        arm_planner_.initModel();
        planning_server_ =  nh.advertiseService("/plan_arm_motion", &RosPlanningInterface::calculation, this);
        // Workspace_server = nh.advertiseService("/plan_basePlacement", &RosPlanningInterface::baseplacement, this);
    }
private:
    bool calculation(task_assembly::plan_arm_motion::Request &req, task_assembly::plan_arm_motion::Response &res)
    {
        ROS_INFO("Generating trajectory.... ");
        arm_planner_.initializeData(req);

        std::string output_message ;
        arm_planner_.compute(res.joint_trajectory);
        if (res.joint_trajectory.points.empty())
        {
            output_message = "Planning failed";
        }
        else
        {
            output_message = "Planning success";
        }
        ROS_INFO("sending back response: [%s]", output_message.c_str() ); // res.q_trajectory.joint_names[0].c_str()
        return true;
    }

    bool baseplacement(task_assembly::base_placement::Request & req, task_assembly::base_placement::Response &res)
    {
        return true;
    }

    ros::ServiceServer planning_server_;
    // ros::ServiceServer Workspace_server;
    ArmPlanner arm_planner_;
};

int main(int argc, char **argv)
{

    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    srand(spec.tv_nsec);

    ros::init(argc, argv, "ArmPlannerServer");
    ros::NodeHandle nh("~");
    
    std::string yaml_path;
    nh.getParam("yaml_path", yaml_path);

    YAMLConfig config;
    config.loadConfig(yaml_path);

    RosPlanningInterface arm_planning_interface(nh, config);

    ROS_INFO("ready arm planner server!");

    ros::spin();

    return 0;
}