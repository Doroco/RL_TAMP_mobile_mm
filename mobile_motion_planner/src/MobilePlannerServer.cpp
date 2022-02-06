#include "ros/ros.h"
#include "task_assembly/plan_mobile_motion.h"

#include "MobilePlanner.h"
#include "YamlConfig.h"

// Input : current q / current base pose / target pose / obstacle / pose constraint
// Output : trajectory_msgs/JointTrajectory/points/positions


class RosPlanningInterface
{
public:
    RosPlanningInterface(ros::NodeHandle & nh, YAMLConfig &config)
    : mobile_planner_(config)    
    {
        //nh.param("urdf_param", mobile_planner_.urdf_param_, std::string("/robot_description"));
        //mobile_planner_.initModel();
        planning_server_ =  nh.advertiseService("/plan_mobile_motion", &RosPlanningInterface::calculation, this);
    }
private:
    bool calculation(task_assembly::plan_mobile_motion::Request &req, task_assembly::plan_mobile_motion::Response &res)
    {
        mobile_planner_.initializeData(req);

        std::string output_message ;
        res.mobile_trajectory = mobile_planner_.compute();
        if (res.mobile_trajectory.points.empty())
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
    MobilePlanner mobile_planner_;
};

int main(int argc, char **argv)
{

    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    srand(spec.tv_nsec);

    ros::init(argc, argv, "MobilePlannerClient");
    ros::NodeHandle nh("~");
    
    std::string yaml_path;
    nh.getParam("yaml_path", yaml_path);

    YAMLConfig config;
    config.loadConfig(yaml_path);

    RosPlanningInterface planning_interface(nh, config);

    ROS_INFO("ready srv server!");

    ros::spin();

    return 0;
}