#include "ros/ros.h"
#include "arm_motion_planner/plan_arm_motion.h"
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
        arm_planner_.initModel();
        planning_server_ =  nh.advertiseService("/plan_franka_arm_motion", &RosPlanningInterface::calculation, this);
    }
private:
    bool calculation(arm_motion_planner::plan_arm_motion::Request &req, arm_motion_planner::plan_arm_motion::Response &res)
    {
        arm_planner_.initializeData(req);

        std::string output_message ;
        res.joint_trajectory = arm_planner_.compute();
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

    ros::ServiceServer planning_server_;
    ArmPlanner arm_planner_;
};

int main(int argc, char **argv)
{

    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    srand(spec.tv_nsec);

    ros::init(argc, argv, "FrankaArmPlannerServer");
    ros::NodeHandle nh("~");
    
    std::string yaml_path;
    nh.getParam("yaml_path", yaml_path);

    YAMLConfig config;
    config.loadConfig(yaml_path);

    RosPlanningInterface franka_arm_planning_interface(nh, config);

    ROS_INFO("ready franka arm planner server!");

    ros::spin();

    return 0;
}