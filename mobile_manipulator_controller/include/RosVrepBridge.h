
#ifndef SIMULATION_INTERFACE_H
#define SIMULATION_INTERFACE_H

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>

#include <sensor_msgs/JointState.h>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <task_assembly/MobileTrajectory.h>

#include <Eigen/Dense>

//image transforamtion
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// pcl Headers
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include "task_assembly/BoundingBoxes3d.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h> 
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>

// tf2 transformation
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#define TOTAL_DOF 9 // 9 dof robot(7 + 2 gripper)
#define SIM_DT 0.01 // 10ms
//#define PI 3.14159265359
//#define deg2rad(deg) ((deg)*PI / 180.0)
//#define rad2deg(rad) ((rad)*180.0 / PI)

const std::string R_JOINT_NAME[dof] = {"panda_joint1","panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7"};
const std::string R_GRIPPER_NAME[2] = {"panda_finger_joint1","panda_finger_joint2"};

const std::string L_JOINT_NAME[dof] = {"Waist_Pitch", "LShoulder_Pitch", "LShoulder_Roll", "LElbow_Pitch", "LElbow_Yaw", "LWrist_Pitch", "LWrist_Roll"};
const std::string L_GRIPPER_NAME[3] = {"LFinger_1", "LFinger_2", "LFinger_3"};

const std::string BASE_JOINT_NAME[4] = {"FL_joint","FR_joint","RR_joint","RL_joint"};

class RosVrepBridge
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
public:
    RosVrepBridge(ros::NodeHandle nh_, double hz_);
    ~RosVrepBridge();

    void leftArmJointCallBack(const sensor_msgs::JointStateConstPtr &msg);
    void rightArmJointCallBack(const sensor_msgs::JointStateConstPtr &msg);
    void leftArmTrajectoryCallBack(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
    void rightArmTrajectoryCallBack(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
    void mobileTrajectoryCallBack(const task_assembly::MobileTrajectory::ConstPtr &msg);
    void basePoseCallBack(const geometry_msgs::Pose2DConstPtr &msg);
    void baseTwistCallBack(const geometry_msgs::TwistConstPtr &msg);

    // ros Point Cloud Generation
    void sim_create_points(const std_msgs::Float32MultiArrayConstPtr& depth_img);
    void sim_image_cb(const sensor_msgs::ImageConstPtr& img);
    void reigon_cb(const task_assembly::BoundingBoxes3dConstPtr &boxes);

    void vrepStatusCallBack(const std_msgs::Int32ConstPtr &msg);
    void vrepTimeCallBack(const std_msgs::Float32ConstPtr &msg);
    void vrepStepDoneCallBack(const std_msgs::BoolConstPtr &msg);

    void vrepStepTrigger();
    void vrepEnableSyncMode();

    // void setJointPosition(std::vector<float> dq);
    // void setExecutionTime(float t);

    void vrepStart();
    void vrepStop();
    void vrepRead();
    void vrepWrite();
    void wait();

    Eigen::VectorXd current_ql_, current_qr_;
    Eigen::VectorXd current_ql_dot_, current_qr_dot_;
    Eigen::VectorXd desired_ql_, desired_qr_;
    Eigen::VectorXd target_x_, target_xr_;
    Eigen::Vector3d desired_grasping_l, desired_grasping_r;

    Eigen::Vector3d euler_;
    Eigen::VectorXd desired_base_vel_;
    Eigen::Vector3d current_base_pose_;
    Eigen::Vector3d current_base_twist_;

    //Eigen::MatrixXd desired_ql_traj, desired_qr_traj;

    trajectory_msgs::JointTrajectory desired_qr_traj_;
    trajectory_msgs::JointTrajectory desired_ql_traj_;
    task_assembly::MobileTrajectory desired_mob_traj_;

    bool get_left_arm_trajectory;
    bool get_right_arm_trajectory;
    bool get_mobile_trajectory;
    bool waist_pitch_priority; // true : right arm, false : left arm 


private:
    ros::Publisher vrep_ljoint_set_pub_;
    ros::Publisher vrep_lgripper_set_pub_;

    ros::Publisher vrep_rjoint_set_pub_;
    ros::Publisher vrep_rgripper_set_pub_;

    ros::Publisher vrep_base_set_pub_;
    ros::Publisher vrep_sim_start_pub_;
    ros::Publisher vrep_sim_stop_pub_;
    ros::Publisher vrep_sim_step_trigger_pub_;
    ros::Publisher vrep_sim_enable_syncmode_pub_;
    ros::Publisher vrep_gripper_set_pub;

    ros::Publisher vrep_sim_pointcloud_pub;
    ros::Publisher vrep_pointcloud_roi;

    /**
    * @brief controller to vrep simulator subscriber message lists
    * @param vrep_joint_state_sub_(sensor_msgs::JointState) : get vrep simulator robot current joint value
    * @param vrep_sim_step_done_sub_(std_msgs::Bool) : get simulation trigger step done signal
    * @param vrep_sim_time_sub_(std_msgs::Float32) : get vrep simulator time tick
    * @param vrep_sim_status_sub_(std_msgs::Int32) : get vrep simulation status(0:sim not running, 1: sim paused, 2:sim stopped, 3:sim running)
    */
    ros::Subscriber vrep_ljoint_state_sub_;
    ros::Subscriber vrep_rjoint_state_sub_;

    ros::Subscriber ljoint_trajectory_sub_;
    ros::Subscriber rjoint_trajectory_sub_;
    ros::Subscriber mobile_trajectory_sub_;

    ros::Subscriber vrep_base_joint_state_sub_;
    ros::Subscriber vrep_base_pose_sub_;
    ros::Subscriber vrep_base_twist_sub_;

    ros::Subscriber vrep_sim_step_done_sub_;
    ros::Subscriber vrep_sim_time_sub_;
    ros::Subscriber vrep_sim_status_sub_;

    ros::Subscriber vrep_rgb_sub;
    ros::Subscriber vrep_depth_sub;

    ros::Subscriber yolo_detection_sub_;
    /**
    * @brief Other parameters for using ros interface and control    
    */
    geometry_msgs::TransformStamped world_base;
    geometry_msgs::TransformStamped mobile_base;
    geometry_msgs::TransformStamped panda_base;
    geometry_msgs::TransformStamped camera_base;
    geometry_msgs::TransformStamped rgb_optical_frame;

    bool sim_step_done_;
    float sim_time_; // from v-rep simulation time
    int tick;
    ros::Rate rate_;

    sensor_msgs::JointState ljoint_cmd_;
    sensor_msgs::JointState lgripper_cmd_;

    sensor_msgs::JointState rjoint_cmd_;
    sensor_msgs::JointState rgripper_cmd_;

    sensor_msgs::JointState base_cmd_;
    sensor_msgs::JointState gripper_cmd_;

    bool is_first_run;
    double start_time;
    double current_time;
    double final_time;
    int vrep_sim_status;
    float exec_time_;
    
    // sim data
    cv::Mat rgb_img_;
    sensor_msgs::PointCloud2 g_cloud;
};

#endif
