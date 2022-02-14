#include "ros/ros.h"

#include "../include/RobotWorkSpace/RobotWorkSpace.h"
#include "../include/RobotWorkSpace/PoseQualityExtendedManipulability.h"
#include "../include/RobotWorkSpace/Reachability.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "testIRM");
    ros::NodeHandle nh("~");

    std::string ws_path;
    nh.getParam("ws_path",ws_path);

    std::string file_name;
    nh.getParam("file_name",file_name);

    std::string yaml_path;
    nh.getParam("yaml_path", yaml_path);

    RobotWorkSpace::YAMLConfig config;
    config.loadConfig(yaml_path);

    std::string urdf_param;
    nh.param("/urdf_param",urdf_param,std::string("/robot_description"));
    
    std::cout<<"file name : "<<ws_path + file_name<<std::endl;
    //////////////////////////////////////////////////  Step1. Loada IRM (or sampling)///////////////////////////////////////////////////////////////////////////////////////////
    KDL::JntArray nominal(7);
    RobotWorkSpace::ReachabilityPtr testWork(new RobotWorkSpace::Reachability(nh,100,config,nominal,RobotWorkSpace::PoseQualityExtendedManipulability::ManipulabilityIndexType::eMultiplySV));
    testWork->setVerbose(false);
    testWork->initModel(urdf_param);   

    float minB[6] = {-1.0473f ,-1.0473f,-0.5523f};
    float maxB[6] = { 1.0473f , 1.0473f, 1.3113f};

    for(int i = 0; i < 3; ++i)
    {
        minB[3 + i] = (float)0;
        maxB[3 + i] = (float)2*M_PI;
    }
    testWork->initialize(0.08,0.08f,minB,maxB);
    testWork->addRandomTCPPoses(15000000);
    testWork->save(ws_path + file_name);
    // testWork->load("/home/min/ws_ws/src/base_placement_planner/mapConfig/myReachFile.bin");

    Eigen::Matrix4f m;
    m.setIdentity();
    
    m <<0.293884,  0.661803, -0.689673,    0.2183,
        -0.73479,   0.61789,  0.279812, -0.467098,
        0.611323,  0.424532,  0.667875,  0.166587,
                0,         0,         0,         1;
    // float p[6] ={-1.0073,-1.0073,0.744945,0.0482733,-1.52595, 0.0431668};

    std::cout<<(int)(testWork->getEntry(m))<<std::endl;
    ROS_INFO("IRM Load Complete");


    /////////////////////////////////////////////// Step4. Quality Measurement) /////////////////////////////////////////////////////////////////////////////
    return 0;
}
