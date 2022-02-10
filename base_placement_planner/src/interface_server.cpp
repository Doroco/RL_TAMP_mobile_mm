#include "ros/ros.h"

#include "../include/RobotWorkSpace/Reachability.h"
#include "../include/RobotWorkSpace/WorkspaceGrid.h"

#include "task_assembly/base_placement.h"
#include "task_assembly/GraspConfigList.h"
#include "task_assembly/GraspConfig.h"

// tf include Header
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

ros::Publisher marker_pub;

class RosPlanningInterface
{
public:
    RosPlanningInterface(ros::NodeHandle & nh, const RobotWorkSpace::ReachabilityPtr &testWork)
    {
        basePose_planner_ = testWork->clone_ws();
        planning_server_ =  nh.advertiseService("/plan_base_sampling", &RosPlanningInterface::baseplacement, this);
    }
private:
    bool baseplacement(task_assembly::base_placement::Request & req, task_assembly::base_placement::Response &res)
    {

        ////////////////////////////////////////////////////  Step2. Receive target Grasp Pose /////////////////////////////////////////////////////////////////////////////////////////////
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transformStamped;
        geometry_msgs::TransformStamped GlobaltoArm;

        do{
            try{
                // tfListener.waitForTransform("mobile_base", "rgb_optical_frame", ros::Time(0), ros::Duration(1.5)); //panda_base
                transformStamped = tfBuffer.lookupTransform("map","rgb_optical_frame",ros::Time(0));
                GlobaltoArm = tfBuffer.lookupTransform("map","panda_base",ros::Time(0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }while(transformStamped.header.frame_id != "map" || GlobaltoArm.header.frame_id != "map");

        // Global to Cam
        Eigen::Affine3f T_GC;
        T_GC.translation() << transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z;
        T_GC.linear() = Eigen::Quaternionf(transformStamped.transform.rotation.w,
                                        transformStamped.transform.rotation.x,
                                        transformStamped.transform.rotation.y,
                                        transformStamped.transform.rotation.z).toRotationMatrix();

        // Global to Arm
        Eigen::Affine3f T_GA;
        T_GA.translation() << GlobaltoArm.transform.translation.x, GlobaltoArm.transform.translation.y, GlobaltoArm.transform.translation.z;
        T_GA.linear() = Eigen::Quaternionf(GlobaltoArm.transform.rotation.w,
                                        GlobaltoArm.transform.rotation.x,
                                        GlobaltoArm.transform.rotation.y,
                                        GlobaltoArm.transform.rotation.z).toRotationMatrix();

        int GraspNum = req.targetGrasp.grasps.size();
        
        // set Coordinate System
        basePose_planner_->setLocalBasepose(T_GA.matrix());
        basePose_planner_->setVisionBasepose(T_GC.matrix());
        // // /////////// //////////////////////////////////// Step3. Get 2D base pose frome EE_target & IRM map ////////////////////////////////////////////////////////
        Eigen::Vector3f minBB,maxBB;
        //transformto global
        basePose_planner_->getWorkspaceExtends(minBB,maxBB);
        RobotWorkSpace::WorkspaceGridPtr reachGrid(new RobotWorkSpace::WorkspaceGrid(minBB(0),maxBB(0),minBB(1),maxBB(1),basePose_planner_->getDiscretizeParameterTranslation(),basePose_planner_->getDiscretizeParameterRotation()));
    
        reachGrid->setObsnum(req.Obstacles2D.size());
        for(int i = 0; i < req.Obstacles2D.size(); ++i)
            reachGrid->setObs(req.Obstacles2D[i],i);

        for(int i = 0; i < 1 /*GraspNum*/; ++i)
        {
            task_assembly::GraspConfig g = req.targetGrasp.grasps.at(i);
            Eigen::Matrix4f gp = basePose_planner_->getGlobalEEpose(g);
            
            // std::cout<<gp<<std::endl;
            reachGrid->setGridPosition(gp(0,3),gp(1,3)); //- 0.30861
            reachGrid->fillGridData(basePose_planner_,g);
            float x,y,r;
            int entries = 0;
            bool ok = reachGrid->getRandomPos(req.minEntry.data, x, y, r, g,1000,&entries);

            // tmpTrans = (rotMat * T_GA).inverse() * gp;
            
            // Eigen::Vector3f T_tmp;
            // T_tmp << gp(0,3),gp(1,3),gp(2,3);
            
            // T_tmp = ( T_GA *rotMat).inverse().linear()*T_tmp + ( T_GA *rotMat).inverse().translation();
            // tmpTrans(0,3)= T_tmp.x();
            // tmpTrans(1,3)= T_tmp.y();
            // tmpTrans(2,3)= T_tmp.z();
            // tmpTrans.block(0,0,3,3) = gp.block(0,0,3,3) * (T_GA*rotMat).inverse().linear().inverse(); 

            if(ok)
            {
                // Eigen::Affine3f rotMat;
                // rotMat.linear() = rotateZaxis_f(r); //- 0.249999216
                
                // to local
                Eigen::Matrix4f tmpTrans =  T_GA.inverse() * gp; //
                // r = std::fmod(r,6.28319) - M_PI;

                // x  -=  ( 0.30861 / sqrt(2) ) * cosf(r);
                // y  -=  ( 0.30861 / sqrt(2) ) * sinf(r);


                std::cout << x <<" , " << y <<" , "<< r * 180 / M_PI << " , "<<entries <<std::endl; //- 14.3239

                MathTools::Quaternion quat = MathTools::eigen4f2quat(gp);
                res.target_ee_pose.position.x = gp(0,3);
                res.target_ee_pose.position.y = gp(1,3);
                res.target_ee_pose.position.z = tmpTrans(2,3);
                res.target_ee_pose.orientation.w = quat.w;
                res.target_ee_pose.orientation.x = quat.x;
                res.target_ee_pose.orientation.y = quat.y;
                res.target_ee_pose.orientation.z = quat.z;

                std::cout<< "target grasp pose : \n"<<tmpTrans<<std::endl;
                std::cout<< "global grasp pose : \n"<<gp<<std::endl;

                res.target_mobile_pose.x = x;
                res.target_mobile_pose.y = y;
                res.target_mobile_pose.theta = r; //- 0.2499992167
                res.IsSucceeded.data = true;
            }
            else
            {
                res.IsSucceeded.data = false;
                ROS_ERROR("Sampling has Failed!!!");
                return false;
            }
        }
        marker_pub.publish(reachGrid->getIRMVisulization());

        return true;
    }

    ros::ServiceServer planning_server_;
    // ros::ServiceServer Workspace_server;
    RobotWorkSpace::ReachabilityPtr  basePose_planner_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "interfaceServer");
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
    
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/ws/IRM", 1);
    //////////////////////////////////////////////////  Step1. Loada Workspace //////////////////////////////////////////////////////////////////////////////////////////////
    KDL::JntArray nominal(7);
    RobotWorkSpace::ReachabilityPtr testWork(new RobotWorkSpace::Reachability(nh,100,config,nominal,RobotWorkSpace::PoseQualityExtendedManipulability::ManipulabilityIndexType::eMultiplySV));
    testWork->setVerbose(false);
    testWork->initModel(urdf_param);   

    testWork->load(ws_path + file_name);
    ROS_INFO("IRM Load Complete  Server Base Placement Start");

    RosPlanningInterface base_pose_sampler_interface(nh,testWork);
    ROS_INFO("Base placement Terminated");

    ros::spin();
    return 0;
}

/*
import wxversion
if wxversion.checkInstalled("2.8"):
    wxversion.select("2.8")
else:
    print("wxversion 2.8 is not installed, installed versions are {}".format(wxversion.getInstalled()))
    */
