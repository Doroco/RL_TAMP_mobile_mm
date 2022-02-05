#include "RosVrepBridge.h"

RosVrepBridge::RosVrepBridge(ros::NodeHandle nh_, double hz_) : rate_(hz_)
{
    is_first_run = true;
    tick = 0;
    sim_step_done_ = false;
    sim_time_ = 0.0f;

    get_left_arm_trajectory = false;
    get_right_arm_trajectory = false;
    get_mobile_trajectory = false;
    waist_pitch_priority = false;

    current_ql_.resize(dof);
    current_ql_dot_.resize(dof);
    desired_ql_.resize(dof);
    desired_ql_.setZero();
    target_x_.resize(3);

    current_qr_.resize(dof);
    current_qr_dot_.resize(dof);
    desired_qr_.resize(dof);
    desired_qr_.setZero();
    target_xr_.resize(3);

    desired_base_vel_.resize(4);
    desired_base_vel_.setZero();

    // current_base_.setZero();

    ljoint_cmd_.name.resize(dof);
    ljoint_cmd_.position.resize(dof);

    lgripper_cmd_.name.resize(3);
    lgripper_cmd_.position.resize(3);

    rjoint_cmd_.name.resize(dof);
    rjoint_cmd_.position.resize(dof);

    rgripper_cmd_.name.resize(3);
    rgripper_cmd_.position.resize(3);

    base_cmd_.name.resize(4);
    base_cmd_.velocity.resize(4);

    desired_grasping_l(0) = -40.0 * M_PI / 180.0;
    desired_grasping_l(1) = -40.0 * M_PI / 180.0;
    desired_grasping_l(2) = 40.0 * M_PI / 180.0;

    desired_grasping_r(0) = -40.0 * M_PI / 180.0;
    desired_grasping_r(1) = -40.0 * M_PI / 180.0;
    desired_grasping_r(2) = 40.0 * M_PI / 180.0;

    for (size_t i = 0; i < dof; i++)
    {
        ljoint_cmd_.name[i] = L_JOINT_NAME[i];
        rjoint_cmd_.name[i] = R_JOINT_NAME[i];
    }

    for (size_t i = 0; i < 3; i++)
    {
        lgripper_cmd_.name[i] = L_GRIPPER_NAME[i];
        rgripper_cmd_.name[i] = R_GRIPPER_NAME[i];
    }
    for (size_t i = 0; i < 4; i++)
    {
        base_cmd_.name[i] = BASE_JOINT_NAME[i];
    }

    vrep_sim_start_pub_ = nh_.advertise<std_msgs::Bool>("/startSimulation", 5);
    vrep_sim_stop_pub_ = nh_.advertise<std_msgs::Bool>("/stopSimulation", 5);
    vrep_sim_step_trigger_pub_ = nh_.advertise<std_msgs::Bool>("/triggerNextStep", 100);
    vrep_sim_enable_syncmode_pub_ = nh_.advertise<std_msgs::Bool>("/enableSyncMode", 5);

    // point cloud pub
    vrep_sim_pointcloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("/k4a/depth_registered/points",1);
    vrep_pointcloud_roi = nh_.advertise<sensor_msgs::PointCloud2>("/k4a/roi/points",1);

    // Queue_size = 발행하는 메세지를 몇 개까지 저장해 둘 것인지
    vrep_ljoint_set_pub_ = nh_.advertise<sensor_msgs::JointState>("/robocare/left_joint_set", 1);
    vrep_lgripper_set_pub_ = nh_.advertise<sensor_msgs::JointState>("/robocare/left_gripper_joint_set", 1);

    vrep_rjoint_set_pub_ = nh_.advertise<sensor_msgs::JointState>("/panda/joint_set", 1);
    vrep_rgripper_set_pub_ = nh_.advertise<sensor_msgs::JointState>("/panda/gripper_joint_set", 1);

    vrep_base_set_pub_ = nh_.advertise<sensor_msgs::JointState>("/husky/base_joint_set", 1); //

    vrep_ljoint_state_sub_ = nh_.subscribe("/robocare/left_joint_states", 100, &RosVrepBridge::leftArmJointCallBack, this);
    vrep_rjoint_state_sub_ = nh_.subscribe("/panda/joint_states", 100, &RosVrepBridge::rightArmJointCallBack, this);

    vrep_base_pose_sub_ = nh_.subscribe("/husky/base_pose", 100, &RosVrepBridge::basePoseCallBack, this);
    vrep_base_twist_sub_ = nh_.subscribe("/husky/base_twist", 100, &RosVrepBridge::baseTwistCallBack, this); //


    vrep_sim_step_done_sub_ = nh_.subscribe("/simulationStepDone", 100, &RosVrepBridge::vrepStepDoneCallBack, this);
    vrep_sim_time_sub_ = nh_.subscribe("/simulationTime", 100, &RosVrepBridge::vrepTimeCallBack, this);
    vrep_sim_status_sub_ = nh_.subscribe("/simulationState", 100, &RosVrepBridge::vrepStatusCallBack, this);

    ljoint_trajectory_sub_ = nh_.subscribe("/robocare/left_joint_trajectory", 100, &RosVrepBridge::leftArmTrajectoryCallBack, this);
    rjoint_trajectory_sub_ = nh_.subscribe("/panda/joint_trajectory", 100, &RosVrepBridge::rightArmTrajectoryCallBack, this);
    mobile_trajectory_sub_ = nh_.subscribe("/husky/mobile_trajectory", 100, &RosVrepBridge::mobileTrajectoryCallBack, this);

    // image sub from vrep scence
    vrep_rgb_sub = nh_.subscribe("/k4a/rgb/image_rect_color",1,&RosVrepBridge::sim_image_cb,this);
    vrep_depth_sub = nh_.subscribe("/vrep/depth",1,&RosVrepBridge::sim_create_points,this);

    yolo_detection_sub_ = nh_.subscribe("/darknet_ros_3d/bounding_boxes",1, &RosVrepBridge::reigon_cb, this);
}
RosVrepBridge::~RosVrepBridge()
{
}

//   void RosVrepBridge::setExecutionTime(float t)
//   {
//     exec_time_ = t;
//   }

void RosVrepBridge::leftArmJointCallBack(const sensor_msgs::JointStateConstPtr &msg)
{
    for (size_t i = 0; i < msg->name.size(); i++)
    {
        current_ql_[i] = msg->position[i];
        current_ql_dot_[i] = msg->velocity[i];
    }
    //  cout <<"left" << current_ql_.transpose() << endl;
}

void RosVrepBridge::rightArmJointCallBack(const sensor_msgs::JointStateConstPtr &msg)
{
    for (size_t i = 0; i < msg->name.size(); i++)
    {
        current_qr_[i] = msg->position[i];
        current_qr_dot_[i] = msg->velocity[i];
    }

    //         cout <<"right" << current_qr_.transpose() << endl;
}

void RosVrepBridge::leftArmTrajectoryCallBack(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
    std::cout << "callback!!" << std::endl;

    waist_pitch_priority = false;

    desired_ql_traj_.points.clear();
    desired_ql_traj_.joint_names = msg->joint_names;
    desired_ql_traj_.points = msg->points;

    get_left_arm_trajectory = true;

}

void RosVrepBridge::rightArmTrajectoryCallBack(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
    std::cout << "callback!!" << std::endl;
    waist_pitch_priority = true;

    desired_qr_traj_.points.clear();
    desired_qr_traj_.joint_names = msg->joint_names;
    desired_qr_traj_.points = msg->points;
    get_right_arm_trajectory = true;

}

void RosVrepBridge::mobileTrajectoryCallBack(const task_assembly::MobileTrajectory::ConstPtr &msg)
{
    std::cout << "callback!!" << std::endl;
    desired_mob_traj_.points.clear();
    desired_mob_traj_.points = msg->points;
    get_mobile_trajectory = true;
}

void RosVrepBridge::basePoseCallBack(const geometry_msgs::Pose2DConstPtr &msg)
{
    current_base_pose_(0) = msg->x;
    current_base_pose_(1) = msg->y;
    current_base_pose_(2) = msg->theta;
    
    ////////////// BroadCast Tf form Simulation Information//////////////////////////
    tf2::Quaternion q;
    static tf2_ros::TransformBroadcaster br;

    // mobile_base.header.stamp = ros::Time::now();
    // mobile_base.header.frame_id = "mobile_base";
    // mobile_base.child_frame_id = "panda_base";
    // mobile_base.transform.translation.x = msg->x;
    // mobile_base.transform.translation.y = msg->y;
    // mobile_base.transform.translation.z = 0.0;
    // q.setRPY(0, 0, msg->theta);
    // mobile_base.transform.rotation.x = q.x();
    // mobile_base.transform.rotation.y = q.y();
    // mobile_base.transform.rotation.z = q.z();
    // mobile_base.transform.rotation.w = q.w();
    // br.sendTransform(mobile_base);

    // 필요없음 그냥 스태틱으로 해도 무관할듯
    // panda_base.header.stamp = ros::Time::now();
    // panda_base.header.frame_id = "panda_base";
    // panda_base.child_frame_id = "camera_base";
    // panda_base.transform.translation.x = 0.30861;
    // panda_base.transform.translation.y = 0.0;
    // panda_base.transform.translation.z = 0.4405;
    // q.setRPY(0, 0, 0);
    // panda_base.transform.rotation.x = q.x();
    // panda_base.transform.rotation.y = q.y();
    // panda_base.transform.rotation.z = q.z();
    // panda_base.transform.rotation.w = q.w();
    // br.sendTransform(panda_base);

    // camera_base.header.stamp = ros::Time::now();
    // camera_base.header.frame_id = "camera_base";
    // camera_base.child_frame_id = "rgb_optical_frame";
    // camera_base.transform.translation.x = -0.21855;
    // camera_base.transform.translation.y = -0.1159;
    // camera_base.transform.translation.z =  1.5190;
    // q.setRPY(0, 0, 0);
    // camera_base.transform.rotation.x = q.x();
    // camera_base.transform.rotation.y = q.y();
    // camera_base.transform.rotation.z = q.z();
    // camera_base.transform.rotation.w = q.w();
    // br.sendTransform(camera_base);
}

void RosVrepBridge::baseTwistCallBack(const geometry_msgs::TwistConstPtr &msg){
    current_base_twist_(0) = msg->linear.x;
    current_base_twist_(1) = msg->linear.y;
    current_base_twist_(2) = msg->angular.x;

    //std::cout << current_base_twist_.transpose() << std::endl;
}


void RosVrepBridge::vrepStatusCallBack(const std_msgs::Int32ConstPtr &msg)
{
    vrep_sim_status = msg->data;
}

void RosVrepBridge::vrepTimeCallBack(const std_msgs::Float32ConstPtr &msg)
{
    sim_time_ = msg->data;
    tick = (sim_time_ * 100) / (SIM_DT * 100);
}

void RosVrepBridge::vrepStepDoneCallBack(const std_msgs::BoolConstPtr &msg)
{
    sim_step_done_ = msg->data;
}

void RosVrepBridge::vrepStepTrigger()
{
    std_msgs::Bool msg;
    msg.data = true;
    vrep_sim_step_trigger_pub_.publish(msg);
}
void RosVrepBridge::vrepEnableSyncMode()
{
    ROS_INFO("Sync Mode On");
    std_msgs::Bool msg;
    msg.data = true;
    vrep_sim_enable_syncmode_pub_.publish(msg);
}

void RosVrepBridge::vrepStart()
{
    ROS_INFO("Starting V-REP Simulation");
    std_msgs::Bool msg;
    msg.data = true;
    std_msgs::Bool msg2;
    msg2.data = false;
    vrep_sim_start_pub_.publish(msg);
    vrep_sim_enable_syncmode_pub_.publish(msg2);
}
void RosVrepBridge::vrepStop()
{
    ROS_INFO("Stopping V-REP Simulation");
    std_msgs::Bool msg;
    msg.data = true;
    vrep_sim_stop_pub_.publish(msg);
}

void RosVrepBridge::vrepRead()
{
    ros::spinOnce();
}

void RosVrepBridge::vrepWrite()
{
    if(waist_pitch_priority)
        desired_ql_(0) = desired_qr_(0);
    else
        desired_qr_(0) = desired_ql_(0);

    for (size_t i = 0; i < dof; i++)
    {
        ljoint_cmd_.position[i] = desired_ql_(i);
        rjoint_cmd_.position[i] = desired_qr_(i);
    }



    for (size_t i = 0; i < 3; i++)
    {
        lgripper_cmd_.position[i] = desired_grasping_l(i);
        rgripper_cmd_.position[i] = desired_grasping_r(i);
    }


    for (size_t i = 0; i < 4; i++)
    {
        base_cmd_.velocity[i] = desired_base_vel_(i);
    }

    vrep_ljoint_set_pub_.publish(ljoint_cmd_);
    vrep_lgripper_set_pub_.publish(lgripper_cmd_);

    vrep_rjoint_set_pub_.publish(rjoint_cmd_);
    vrep_rgripper_set_pub_.publish(rgripper_cmd_);

    vrep_base_set_pub_.publish(base_cmd_);


	get_right_arm_trajectory = false;
	get_left_arm_trajectory = false;
    get_mobile_trajectory = false;

    vrepStepTrigger();


}

void RosVrepBridge::wait()
{
    while (ros::ok() && !sim_step_done_)
    {
        ros::spinOnce();
    }
    sim_step_done_ = false;
    rate_.sleep();
}

void RosVrepBridge::sim_image_cb(const sensor_msgs::ImageConstPtr& img)
{
  rgb_img_ = cv_bridge::toCvShare(img, "bgr8")->image;
} 

void RosVrepBridge::sim_create_points(const std_msgs::Float32MultiArrayConstPtr& depth_img)
{
    std::vector<float> depth_raw = depth_img->data;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->width = 480; // 480
    cloud->height = 640;
    cloud->header.frame_id = "rgb_optical_frame";
    cloud->is_dense = false;
    cloud->points.resize(cloud->width*cloud->height);
    unsigned int u_res = 480;
    unsigned int v_res = 640;
    unsigned int datalen = u_res * v_res;
    float scale = (3.5 - 0.01) / 1.0;
    std::vector<float> x_scale, y_scale;
    float f = (std::max(u_res, v_res) / 2) / tan(57.0  / 2); 
    
    if( rgb_img_.size().area() != 0)
    {
        for (int j = 0; j < u_res; j++)
        {
            float y = (j - u_res / 2.0);
            for(int i = 0; i < v_res; i++)
            {
                int k = j * v_res + i;
                float x = -(i - v_res / 2.0);
                x_scale.push_back(x / f);
                y_scale.push_back(y / f);

                auto rgb_ints = rgb_img_.at<cv::Vec3b>(j, i);  // this is the RGB image from the sensor (e.g. Kinect/realsense) , but u need to align to depth! 
                
                float depth = 1.0 + scale * depth_raw[k];
                float xyz[3] = {depth * x_scale[k], depth * y_scale[k], depth};
                pcl::PointXYZRGB p;
                p.x = xyz[0]; 
                p.y = xyz[1]; 
                p.z = xyz[2]; 
                p.r = (int)rgb_ints[0];
                p.g = (int)rgb_ints[1];
                p.b = (int)rgb_ints[2];
                cloud->at(i,j) = p;
            }
        }
    }

    //////////////  Vertical Filiing ///////////////////////////////////////// 
    // double rotx = 0.0;
    // double roty = M_PI/2.0;
    // double rotz = -M_PI/1.0;
    // Eigen::Matrix4f rotMatrixX;
    // Eigen::Matrix4f rotMatrixY;
    // Eigen::Matrix4f rotMatrixZ;
    // Eigen::Matrix4f rotMatrix;
    // Eigen::Matrix4f compensateFrame;

    // rotMatrixX <<
    // 1.0, 0.0, 0.0, 0.0,
    // 0.0, cos(rotx), -sin(rotx), 0.0,
    // 0.0, sin(rotx), cos(rotx), 0.0,
    // 0.0, 0.0, 0.0, 1.0;

    // rotMatrixY <<
    // cos(roty), 0.0, sin(roty), 0.0,
    // 0.0, 1.0, 0.0, 0.0,
    // -sin(roty), 0.0, cos(roty), 0.0,
    // 0.0, 0.0, 0.0, 1.0;

    // rotMatrix <<
    // 0.0, 1.0, 0.0, 0.0,
    // 1.0, 0.0, 0.0, 0.0,
    // 0.0, 0.0, -1.0, 0.0,
    // 0.0, 0.0, 0.0, 1.0;

    // rotMatrixZ <<
    // cos(rotz), -sin(rotz), 0.0, 0.0,
    // sin(rotz), cos(rotz), 0.0, 0.0,
    // 0.0, 0.0, 1.0, 0.0,
    // 0.0, 0.0, 0.0, 1.0;

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr Testcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    // Testcloud->header.frame_id = "rgb_optical_frame";
    // pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/min/test_ws/src/base_placement_planner/mapConfig/test.pcd", *Testcloud);

    // *cloud = *cloud +*Testcloud;
    // std::cout<<"transform rgb_optical_\n"<<rotMatrixX * rotMatrixY * rotMatrixZ *rotMatrix<<std::endl;
    //rotMatrixX * rotMatrixY * rotMatrixZ 
    // pcl::transformPointCloud(*cloud, *cloud, rotMatrixX * rotMatrixY * rotMatrixZ *rotMatrix);
    pcl::PCLPointCloud2 cloud_Generated;
    pcl::toPCLPointCloud2(*cloud, cloud_Generated);
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_Generated, output);
    g_cloud = output;
    vrep_sim_pointcloud_pub.publish(output); 

    ///////////////////// 실험용 (욜로 동작한다고 그냥 생각하고 진행) --> 컴터가 욜로를 감당못하네요...
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);       
    pcl::fromROSMsg(g_cloud,*cloud_filtered);

    float offset = 0.08;
    Eigen::Vector4f minPoint;
    minPoint[0] = -0.00444465782493 - offset;
    minPoint[1] =  0.246667996049 - 0.06;
    minPoint[2] =  3.06391882896 - 0.02;
    minPoint[3] = 1.0;
    Eigen::Vector4f maxPoint;
    maxPoint[0] = 0.0200009606779 + offset;
    maxPoint[1] = 0.32668235898 + offset;
    maxPoint[2] = 3.09763979912 + 0.02;
    maxPoint[3] = 1.0;

    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    boxFilter.setMin(minPoint);
    boxFilter.setMax(maxPoint);
    boxFilter.setInputCloud(cloud_filtered);
    // boxFilter.setTransform(T_LK);
    boxFilter.filter(*cloud_filtered);

    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*cloud_filtered,cloud_publish);
    cloud_publish.header = g_cloud.header;
    vrep_pointcloud_roi.publish(cloud_publish);
}

void RosVrepBridge::reigon_cb(const task_assembly::BoundingBoxes3dConstPtr &boxes)
{
    // Eigen::Affine3f T_LK;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    for(unsigned int num = 0; num < boxes->bounding_boxes.size(); num++)
    {
        // T_LK.translation() << -0.0039076 , -0.0320149, -0.000327477;

        // camera base to rgb_optical_frame --> firxed now working frame is rgb_optical_frame
        // T_LK.translation() << -0.890 -0.055 -0.0299;
        // T_LK.linear() = Eigen::Quaternionf(0.498297,-0.49927,0.501692,-0.500734).toRotationMatrix();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);       
        pcl::fromROSMsg(g_cloud,*cloud_filtered);

        float offset = 0.08;
        Eigen::Vector4f minPoint;
        minPoint[0] = boxes->bounding_boxes.at(num).xmin - offset;
        minPoint[1] = boxes->bounding_boxes.at(num).ymin - 0.06;
        minPoint[2] = boxes->bounding_boxes.at(num).zmin - 0.02;
        minPoint[3] = 1.0;
        Eigen::Vector4f maxPoint;
        maxPoint[0] = boxes->bounding_boxes.at(num).xmax + offset;
        maxPoint[1] = boxes->bounding_boxes.at(num).ymax + offset;
        maxPoint[2] = boxes->bounding_boxes.at(num).zmax + 0.02;
        maxPoint[3] = 1.0;
        
        pcl::CropBox<pcl::PointXYZRGB> boxFilter;
        boxFilter.setMin(minPoint);
        boxFilter.setMax(maxPoint);
        boxFilter.setInputCloud(cloud_filtered);
        // boxFilter.setTransform(T_LK);
        boxFilter.filter(*cloud_filtered);
        // pcl::PointXYZRGB addCloud;

        // float gapx = (maxPoint[0] - minPoint[0])/100.;
        // float gapy = (maxPoint[1] - minPoint[1])/100.;
        // float gapz = (maxPoint[2] - minPoint[2])/100.;

        // addCloud.x = (maxPoint[0] + minPoint[0] )/ 2. ;
        // addCloud.y = (maxPoint[0] + minPoint[1] )/ 2.;
        // addCloud.z = (maxPoint[0] + minPoint[2] )/ 2.;
        // addCloud.r = 0;
        // addCloud.g = 255;
        // addCloud.b = 0;
        // cloud_filtered->push_back(addCloud);
        // pcl::transformPointCloud (*cloud_filtered, *cloud_filtered, T_LK);

        // 두 핸들중에 위에 것만 보려고 이거사용했어욤
        if(num == 0)
            *cloud  = *cloud + *cloud_filtered; // sum all cloud in Yolo Bounding Boxes
    }
    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*cloud,cloud_publish);
    cloud_publish.header = g_cloud.header;
    vrep_pointcloud_roi.publish(cloud_publish);
    
    // pcl::io::savePCDFileASCII (file_Path_+"test_pcd.pcd", *cloud);
}