#include "../include/RobotWorkSpace/PoseQualityMeasurement.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <cmath>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;

const std::string Joint_name[7] =
    {
        "panda_joint1", "panda_joint2", "panda_joint3",
        "panda_joint4", "panda_joint5", "panda_joint6",
        "panda_joint7"};

namespace RobotWorkSpace
{
    PoseQualityMeasurement::PoseQualityMeasurement(RobotWorkSpace::YAMLConfig robotConfig, KDL::JntArray nominal)
        : config_(robotConfig), nominal_(nominal)
    {
        name = "PoseQualityMeasurement";
        considerObstacle = false;
        verbose = false;
    }

    PoseQualityMeasurement::~PoseQualityMeasurement()
    = default;

    float PoseQualityMeasurement::getPoseQuality()
    {
        std::cout << "Please use derived classes..." << std::endl;
        return 0.0f;
    }

    float PoseQualityMeasurement::getPoseQuality(const Eigen::VectorXf& /*direction*/)
    {
        std::cout << "Please use derived classes..." << std::endl;
        return 0.0f;
    }

    void PoseQualityMeasurement::setVerbose(bool v)
    {
        verbose = v;
    }

    void PoseQualityMeasurement::initModel(std::string URDFPram)
    {
        urdf_param_ = URDFPram;
        std::string urdf_absolute_path;
        std::string mod_url = config_.urdf_path;
        if (config_.urdf_path.find("package://") == 0)
        {
            mod_url.erase(0, strlen("package://"));
            size_t pos = mod_url.find("/");
            if (pos == std::string::npos)
            {
                cout << "Could not parse package:// format into file:// format" << endl;;
            }
            std::string package = mod_url.substr(0, pos);
            mod_url.erase(0, pos);
            std::string package_path = ros::package::getPath(package);

            if (package_path.empty())
            {
                cout << "Package does not exist" << endl;;
            }

            urdf_absolute_path =  package_path + mod_url;
        }

        RigidBodyDynamics::Addons::URDFReadFromFile(urdf_absolute_path.c_str(), &rbdl_model_, false, false);

        nb_of_joints_= config_.joint_limit_lower.size();
        rrt_.dofSize = nb_of_joints_;

        // rbdl이 상체 모든 걸 다 받아버리면 말단 장치 위치 계산할때 모든 관절의 위치를 알고 있어야함.
        end_effector_id_ = rbdl_model_.GetBodyId((config_.chain_end).c_str());
        arm_base_frame_id_ = rbdl_model_.GetBodyId((config_.chain_start).c_str());

        if (rbdl_model_.IsFixedBodyId(end_effector_id_))
        {
            end_effector_com_ = rbdl_model_.mFixedBodies[end_effector_id_ - rbdl_model_.fixed_body_discriminator].mCenterOfMass;
        }
        else
        {
            end_effector_com_ = rbdl_model_.mBodies[end_effector_id_].mCenterOfMass;
        }

        if (rbdl_model_.IsFixedBodyId(arm_base_frame_id_))
        {
            arm_base_frame_pos_ = rbdl_model_.mFixedBodies[arm_base_frame_id_ - rbdl_model_.fixed_body_discriminator].mCenterOfMass;
        }
        else
        {
            arm_base_frame_pos_ = rbdl_model_.mBodies[arm_base_frame_id_].mCenterOfMass;
        }

        // init Joint States
        joint_limit_.lower_.resize(nb_of_joints_);
        joint_limit_.lower_rad_.resize(nb_of_joints_);
        joint_limit_.upper_.resize(nb_of_joints_);
        joint_limit_.upper_rad_.resize(nb_of_joints_);
        
        joint_state_.qInit_.resize(nb_of_joints_);

        output_arm_trajectory_.joint_names.clear();
            for (int i = 0;i<nb_of_joints_;i++)
                output_arm_trajectory_.joint_names.push_back(Joint_name[i]);

        for (std::vector<int>::size_type i=0;i<config_.joint_limit_lower.size();i++){
            joint_limit_.lower_(i) = config_.joint_limit_lower[i];
            joint_limit_.upper_(i) = config_.joint_limit_upper[i];
        }

        joint_limit_.lower_rad_ = joint_limit_.lower_ / 180.0*M_PI;
	    joint_limit_.upper_rad_ = joint_limit_.upper_ / 180.0*M_PI;

        rrt_.torso_config.clear();
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        rrt_.box_num_obs = 0;
        rrt_.Box_env.clear();
        rrt_.Box_robot.clear();
        rrt_.box_num_link = config_.link_name.size();

        for (int i=0;i<nb_of_joints_;i++)	
		    joint_state_.qInit_(i) = 0;

        ros::NodeHandle node_handle("~");

        std::string xml_string;

        std::string urdf_xml, full_urdf_xml;
        node_handle.param("urdf_xml", urdf_xml, urdf_param_);
        node_handle.searchParam(urdf_xml, full_urdf_xml);

        //ROS_DEBUG_NAMED("IK", "Reading xml file from parameter server");
        if (!node_handle.getParam(full_urdf_xml, xml_string))
        {
            ROS_INFO("Could not load the xml from parameter server");
            std::cout<<urdf_absolute_path<<std::endl;
            std::cout<<full_urdf_xml<<std::endl;
            std::cout<<xml_string<<std::endl;
            return;
        }

        // std::cout<<urdf_absolute_path<<std::endl;
        // std::cout<<full_urdf_xml<<std::endl;
        // std::cout<<xml_string<<std::endl;

        node_handle.param(full_urdf_xml, xml_string, std::string());
        IK_robot_model.initString(xml_string);

        //ROS_DEBUG_STREAM_NAMED("trac_ik", "Reading joints and links from URDF");

        if (!kdl_parser::treeFromUrdfModel(IK_robot_model, IK_tree))
            ROS_INFO("Failed to extract kdl tree from xml robot description");

        if (!IK_tree.getChain(config_.chain_start, config_.chain_end, IK_chain))
            ROS_INFO("Couldn't find chain");

        std::vector<KDL::Segment> chain_segs = IK_chain.segments;

        urdf::JointConstSharedPtr joint;

        IK_lb.resize(IK_chain.getNrOfJoints());
        IK_ub.resize(IK_chain.getNrOfJoints());

        uint joint_num = 0;
        for (unsigned int i = 0; i < chain_segs.size(); ++i)
        {
            joint = IK_robot_model.getJoint(chain_segs[i].getJoint().getName());
            if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
            {
                joint_num++;
                float lower, upper;
                int hasLimits;
                if (joint->type != urdf::Joint::CONTINUOUS)
                {
                    if (joint->safety)
                    {
                        lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
                        upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
                    }
                    else
                    {
                        lower = joint->limits->lower;
                        upper = joint->limits->upper;
                    }
                    hasLimits = 1;
                }
                else
                {
                    hasLimits = 0;
                }
                if (hasLimits)
                {
                    IK_lb(joint_num - 1) = lower;
                    IK_ub(joint_num - 1) = upper;
                }
                else
                {
                    IK_lb(joint_num - 1) = std::numeric_limits<float>::lowest();
                    IK_ub(joint_num - 1) = std::numeric_limits<float>::max();
                }
                //ROS_DEBUG_STREAM_NAMED("trac_ik", "IK Using joint " << joint->name << " " << lb(joint_num - 1) << " " << ub(joint_num - 1));
            }
        }

        body_id_collision_.clear();
        body_com_position_.clear();

        for (std::vector<int>::size_type i = 0; i < config_.link_dimension.size(); i++)
        {
            body_id_collision_.push_back(rbdl_model_.GetBodyId(config_.link_name[i].c_str()));
            body_com_position_.push_back(rbdl_model_.mBodies[rbdl_model_.GetBodyId(config_.link_name[i].c_str())].mCenterOfMass);
        };

        // push back end_effector
        body_id_collision_.push_back(end_effector_id_);
        body_com_position_.push_back(end_effector_com_);

        /////////////////////  setUp Collision Model /////////////////////////////////////////////////////////////
        rrt_.torso_config.clear();
        rrt_.Box_env.clear();    // 외부 충돌감지
        rrt_.Box_robot.clear();  // 현재 충돌감지
        rrt_.box_num_link = config_.link_name.size();

        for (int i=0;i<rrt_.box_num_link;i++){
            ArmPlanner::ST_OBB box2;
            Eigen::Vector3d link_dim;
            for (int j = 0; j < config_.link_dimension[i].size(); j++)
            {
                link_dim(j) = config_.link_dimension[i][j];
                box2.vCenter(j) = config_.link_position[i][j];
            }

            box2.vRot = rotateXaxis(config_.link_orientation[i][0]/180.0*M_PI)*rotateYaxis(config_.link_orientation[i][1]/180.0*M_PI)*rotateZaxis(config_.link_orientation[i][2]/180.0*M_PI);
            box2.fAxis = link_dim;
            box2.id_name = config_.link_name[i];

            rrt_.Box_robot.push_back(box2); 
        }

        rrt_model_.model_ = rbdl_model_;
        rrt_model_.body_id_vec.clear();
        rrt_model_.body_id_vec.assign(body_id_collision_.begin(), body_id_collision_.end());
        rrt_model_.body_com_pos.clear();
        rrt_model_.body_com_pos.assign(body_com_position_.begin(), body_com_position_.end());


        // double eps = 5e-3;
        // double num_samples = 100;
        // double timeout = 0.005;

        // std::string chain_start = config_.chain_start;
        // std::string chain_end= config_.chain_end;

        // KDL::Frame end_effector_pose;
        
        // TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param_, timeout, eps);
	    // bool valid = tracik_solver.getKDLChain(IK_chain);

        // if (!valid)
        // {
        //     std::cout<<"There was no valid KDL chain found"<<std::endl;
        //     // std::cout<<urdf_param_<<std::endl;
        //     // return false;
        // }

        // valid = tracik_solver.getKDLLimits(IK_lb,IK_ub);

        // if (!valid)
        // {
        //     std::cout<<"There were no valid KDL joint limits found"<<std::endl;
        //     // return false;
        // }
        constrain_pose_ = true;
        playTime_ = 0.0;
    }

    std::string PoseQualityMeasurement::getName()
    {
        return name;
    }

    void PoseQualityMeasurement::setRandomConfig()
    {
        // Make Ramdom Seed
        // struct timespec spec;
        // clock_gettime(CLOCK_REALTIME, &spec);
        // srand(spec.tv_nsec);

        //std::cout<<IK_chain.getNrOfJoints()<<std::endl;
        std::vector<double> R;

		for (int i = 0; i < IK_chain.getNrOfJoints(); i++)
		{
			double jointrange = joint_limit_.upper_rad_(i) - joint_limit_.lower_rad_(i); // angle
			// double r = ((double)rand() / (double)RAND_MAX) * jointrange;  //RandomFloat
            double r = RandomFloat(0.0f,1.0f) * jointrange; 
			R.push_back(joint_limit_.lower_rad_(i) + r);
		}

		for (size_t j = 0; j < nominal_.data.size(); j++)
		{
			this->nominal_(j) = R[j];
		}
    }

    Eigen::MatrixXf PoseQualityMeasurement::getJacobian()
    {
        Eigen::MatrixXd J_SB;
        Eigen::MatrixXf jac_result;

        J_SB.resize(6,IK_chain.getNrOfJoints());
        jac_result.resize(6,IK_chain.getNrOfJoints());
        
        // Spatial Jacobian about given body position id
        CalcBodySpatialJacobian(rbdl_model_,nominal_.data,end_effector_id_,J_SB,true);

        if(verbose)
        {
            std::cout<<"jacobian Matrix of given joint set"<<std::endl;
            std::cout<<J_SB<<std::endl;
        }

        for(int y = 0; y < 6; y++){
            for(int x = 0; x < nb_of_joints_; x++){
                jac_result(y,x) = (float)J_SB(y,x);
            }
        }

        return jac_result;
    }

    Eigen::MatrixXf PoseQualityMeasurement::getJacobianGlobal()
    {
        Eigen::MatrixXd J_SB;
        Eigen::MatrixXf jac_result;

        J_SB.resize(6,IK_chain.getNrOfJoints());
        jac_result(6,IK_chain.getNrOfJoints());

        // id 기준의 자코비안을 반환해준다.
        CalcBodySpatialJacobian(rbdl_model_,nominal_.data,end_effector_id_,J_SB,true);

        for(int y = 0; y < 6; y++){
            for(int x = 0; x < nb_of_joints_; x++){
                jac_result(y,x) = (float)J_SB(y,x);
            }
        }

        return jac_result;
    }

    void PoseQualityMeasurement::setObstacleDistanceVector(const Eigen::Vector3f& directionSurfaceToObstance)
    {
        considerObstacle = true;
        obstacleDir = directionSurfaceToObstance;
    }

    void PoseQualityMeasurement::disableObstacleDistance()
    {
        considerObstacle = false;
    }

    RobotWorkSpace::YAMLConfig PoseQualityMeasurement::getRNS()
    {
        return config_;
    }

    bool PoseQualityMeasurement::consideringJointLimits()
    {
        return false;
    }

    //////////////////////////////////////////////////////////////// Kinematics Uitility ///////////////////////////////////////////////////////

    bool PoseQualityMeasurement::solveFK(Eigen::Matrix4f &EE_pose)
    {
        KDL::Frame EE_frame;
        KDL::ChainFkSolverPos_recursive Fksolver = KDL::ChainFkSolverPos_recursive(IK_chain);

        std::vector<double> config;
		config.clear();

        if(Fksolver.JntToCart(nominal_,EE_frame) < 0)
        {
            if(verbose)
                std::cerr<<"Fk about robot Model Failed!!!!"<<std::endl;
            return false;
        }

        ////////// check SelfCollision /////////////////
        for (int i = 0; i < nb_of_joints_; i++)
        {
            config.push_back(nominal_.data(i) * 180 / M_PI);
        }

        if(!rrt_.checkSelfCollision(rrt_model_,config))
        {
            EE_pose = Frame2Eigen(EE_frame);
            return true;
        }
        else
        {
            if(verbose)
                std::cerr<<"self Collision Occured!!!!"<<std::endl;
            return false;
        }
    }

    bool PoseQualityMeasurement::solveIK(RobotWorkSpace::Transform3d _target_ee_pose)
    {
        ///////////////////  track - IK params ///////////////////////////////////////////////////////
        double eps = 5e-3;
        double num_samples = 100;
        double timeout = 0.005;

        std::string chain_start = config_.chain_start;
        std::string chain_end= config_.chain_end;

        KDL::Frame end_effector_pose;
        
        TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param_, timeout, eps);
	    bool valid = tracik_solver.getKDLChain(IK_chain);

        if (!valid)
        {
            std::cout<<"There was no valid KDL chain found"<<std::endl;
            // std::cout<<urdf_param_<<std::endl;
            return false;
        }

        valid = tracik_solver.getKDLLimits(IK_lb,IK_ub);

        if (!valid)
        {
            std::cout<<"There were no valid KDL joint limits found"<<std::endl;
            return false;
        }

        //////// setup target configuration ////////////////////////////
        Eigen::Matrix4d target_global;
        target_global.block(0, 0, 3, 3) = _target_ee_pose.linear();
        target_global.block(0, 3, 3, 1) = _target_ee_pose.translation();

        for (int i = 0; i < 3; i++)
        {
            end_effector_pose.p(i) = target_global(i,3);
        }

        KDL::Rotation rot;
        rot.data[0] = target_global(0, 0);
        rot.data[1] = target_global(0, 1);
        rot.data[2] = target_global(0, 2);
        rot.data[3] = target_global(1, 0);
        rot.data[4] = target_global(1, 1);
        rot.data[5] = target_global(1, 2);
        rot.data[6] = target_global(2, 0);
        rot.data[7] = target_global(2, 1);
        rot.data[8] = target_global(2, 2);
        end_effector_pose.M = rot;

        ////////////////////////////////////    track ik - IK solver ////////////////////////////////////////////////////////////////
        KDL::JntArray tmpnominal(IK_chain.getNrOfJoints());
        KDL::JntArray IK_result_;
        int rc;
        double total_time = 0;
        uint success = 0;
        bool solved = true;

        while(true) 
        {
            if(--num_samples == 0){
                solved = false;
                break;
            }

            std::vector<double> R;
            for (int i = 0; i < IK_chain.getNrOfJoints(); i++)
            {
                double jointrange = joint_limit_.upper_rad_(i) - joint_limit_.lower_rad_(i); // angle
                double r = ((double)rand() / (double)RAND_MAX) * jointrange;
                R.push_back(joint_limit_.lower_rad_(i) + r);
            }

            for (size_t j = 0; j < tmpnominal.data.size(); j++)
            {
                tmpnominal(j) = R[j];
            }

            //cout <<"iteration?" << endl;
            double elapsed = 0;
            rc = tracik_solver.CartToJnt(tmpnominal, end_effector_pose, IK_result_);
            // int CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist& bounds=KDL::Twist::Zero());
            
            std::vector<double> config;	
            config.clear();

            if (rc >= 0)
            {
                //ROS_INFO_STREAM("IK solution is : " << result.data.transpose() * 180 / M_PI);
                for (int i = 0; i < nb_of_joints_; i++)
                {
                    // rad to Deg
                    config.push_back(IK_result_.data(i) * 180 / M_PI);
                }
                
                if (!rrt_.checkSelfCollision(rrt_model_, config) && !rrt_.checkExternalCollision(rrt_model_, config))
                {

                    // ROS_INFO_STREAM("IK solution is : " << IK_result_.data.transpose() * 180 / M_PI);
                    joint_state_.qGoal_ = IK_result_.data;
                    solved = true;
                    nominal_ = tmpnominal;
                    break;
                }
                else
                {
                    continue;
                }

            }
            else
            {
                continue;
            }
        }
        return solved;
    }

    Eigen::Matrix4f PoseQualityMeasurement::Frame2Eigen(KDL::Frame &frame)
    {
        Eigen::Matrix4f resMat;
        resMat.resize(4,4);
        double d[16] = {0,};
        frame.Make4x4(d);

        resMat(0,0) = d[0];
        resMat(0,1) = d[1];
        resMat(0,2) = d[2];
        resMat(0,3) = d[3];
        resMat(1,0) = d[4];
        resMat(1,1) = d[5];
        resMat(1,2) = d[6];
        resMat(1,3) = d[7];
        resMat(2,0) = d[8];
        resMat(2,1) = d[9];
        resMat(2,2) = d[10];
        resMat(2,3) = d[11];
        resMat(3,0) = d[12];
        resMat(3,1) = d[13];
        resMat(3,2) = d[14];
        resMat(3,3) = d[15];

        return resMat;
    }

    /////////////////////////////////////////////////////////////// Planning Utils ////////////////////////////////////////////////////////////////
    bool PoseQualityMeasurement::setupRRT(Eigen::VectorXd q_start, Eigen::VectorXd q_goal, Eigen::MatrixXd& joint_target)
    {
        rrt_.lower_limit = joint_limit_.lower_;
        rrt_.upper_limit = joint_limit_.upper_;

        rrt_.qinit = q_start;
        rrt_.qgoal = q_goal;

        ofstream outFile("home/min/test_ws/src/base_placement_planner/path_result.txt", ios::out); // open file for "writing"

        if (rrt_.solveRRT(rrt_model_, outFile))
        {
            ofstream outFile2("home/min/test_ws/src/base_placement_planner/path_result2.txt", ios::out); // "writing"
            // path_result -> Smooth -> path_result2
            ifstream inFile("home/min/test_ws/src/base_placement_planner/path_result.txt"); // "reading"
            rrt_.smoothPath(outFile2, inFile);

            outFile2.close();
            Eigen::MatrixXd joint_temp(100, nb_of_joints_);

            ifstream inFile2("home/min/test_ws/src/base_placement_planner/path_result2.txt"); // "reading"
            int size = 0;
            std::vector<std::string> parameters;
            char inputString[1000];
            while (!inFile2.eof())
            { // eof : end of file
                inFile2.getline(inputString, 1000);
                boost::split(parameters, inputString, boost::is_any_of(","));
                if (parameters.size() == nb_of_joints_)
                {
                    for (int j = 0; j < parameters.size(); j++)
                        joint_temp(size, j) = atof(parameters[j].c_str()); // c_str() : string -> char* // atof : char* -> float
                    size++;
                }
            }
            //cout << "trajectory size" << size << endl;
            inFile2.close();
            joint_target = joint_temp.topLeftCorner(size, nb_of_joints_);
            return true;
        }
        else
        {
            cout << "Time out!! 1" << endl;
            return false;
        }
    }
    bool PoseQualityMeasurement::setupCRRT(Eigen::VectorXd q_start, Eigen::VectorXd q_goal, Eigen::MatrixXd &joint_target)
    {
        rrt_.lower_limit = joint_limit_.lower_;
        rrt_.upper_limit = joint_limit_.upper_;

        rrt_.qinit = q_start;
        rrt_.qgoal = q_goal;

        ofstream outFile3("home/min/test_ws/src/base_placement_planner/path_result3.txt", ios::out);

        if (rrt_.solveCRRT(rrt_model_, outFile3))
        {
            outFile3.close();

            Eigen::MatrixXd joint_temp(5000, nb_of_joints_);

            ifstream inFile3("home/min/test_ws/src/base_placement_planner/path_result3.txt");
            int size = 0;
            std::vector<std::string> parameters;
            char inputString[50000];
            while (!inFile3.eof())
            {
                inFile3.getline(inputString, 50000);
                boost::split(parameters, inputString, boost::is_any_of(","));
                if (parameters.size() == nb_of_joints_)
                {
                    for (int j = 0; j < parameters.size(); j++)
                        joint_temp(size, j) = atof(parameters[j].c_str());
                    size++;
                }
            }
            inFile3.close();
            //cout << "size" << size << endl;
            joint_target = joint_temp.topLeftCorner(size, nb_of_joints_);
            
            return true;
        }
        else
        {
            cout << "Time out!! 2" << endl;
            return false;
        }

    }



    bool PoseQualityMeasurement::solveRRTwithMultipleGoals(Eigen::VectorXd q_start, std::vector<VectorXd> q_goal_set, Eigen::MatrixXd &joint_target)
    {
        rrt_.lower_limit = joint_limit_.lower_;
        rrt_.upper_limit = joint_limit_.upper_;

        rrt_.qinit = q_start;
        rrt_.qgoals.assign(q_goal_set.begin(), q_goal_set.end());

        ofstream outFile("home/min/test_ws/src/base_placement_planner/path_result.txt", ios::out); // open file for "writing"

        if (rrt_.solveRRT(rrt_model_, outFile))
        {

            // outFile.close();
            // MatrixXd joint_temp(5000, nb_of_joints_);

            // ifstream inFile2("path_result.txt"); // "reading"
            // int size = 0;
            // std::vector<std::string> parameters;
            // char inputString[1000];


            ofstream outFile2("home/min/test_ws/src/base_placement_planner/path_result2.txt", ios::out); // "writing"
            // path_result -> Smooth -> path_result2
            ifstream inFile("home/min/test_ws/src/base_placement_planner/path_result.txt"); // "reading"
            rrt_.smoothPath(outFile2, inFile);

            outFile2.close();
            Eigen::MatrixXd joint_temp(100, nb_of_joints_);

            ifstream inFile2("home/min/test_ws/src/base_placement_planner/path_result2.txt"); // "reading"
            int size = 0;
            std::vector<std::string> parameters;
            char inputString[1000];

            while (!inFile2.eof())
            {
                inFile2.getline(inputString, 50000);
                boost::split(parameters, inputString, boost::is_any_of(","));
                if (parameters.size() == nb_of_joints_)
                {
                    for (int j = 0; j < parameters.size(); j++)
                        joint_temp(size, j) = atof(parameters[j].c_str());
                    size++;
                }
            }
            inFile2.close();
            //cout << "size" << size << endl;
            joint_target = joint_temp.topLeftCorner(size, nb_of_joints_);
            
            return true;
        }
        else
        {
            cout << "Time out!! 3" << endl;
            return false;
        }

    }

    bool PoseQualityMeasurement::solveCRRTwithMultipleGoals(Eigen::VectorXd q_start, std::vector<VectorXd> q_goal_set, Eigen::MatrixXd &joint_target)
    {
        rrt_.lower_limit = joint_limit_.lower_;
        rrt_.upper_limit = joint_limit_.upper_;

        rrt_.qinit = q_start;
        rrt_.qgoals.assign(q_goal_set.begin(), q_goal_set.end());

        ofstream outFile3("home/min/test_ws/src/base_placement_planner/path_result3.txt", ios::out);

        if (rrt_.solveCRRT(rrt_model_, outFile3))
        {
            outFile3.close();

            // cout << "111"<<endl;
            Eigen::MatrixXd joint_temp(5000, nb_of_joints_);

            ifstream inFile3("home/min/test_ws/src/base_placement_planner/path_result3.txt");
            int size = 0;
            std::vector<std::string> parameters;
            char inputString[50000];
            while (!inFile3.eof())
            {
                inFile3.getline(inputString, 50000);
                boost::split(parameters, inputString, boost::is_any_of(","));
                if (parameters.size() == nb_of_joints_)
                {
                    for (int j = 0; j < parameters.size(); j++)
                        joint_temp(size, j) = atof(parameters[j].c_str());
                    size++;
                }
            }
            inFile3.close();
            //cout << "size" << size << endl;
            joint_target = joint_temp.topLeftCorner(size, nb_of_joints_);
            return true;
        }
        else
        {
            cout << "Time out!! 4" << endl;
            return false;
        }
    }

    ////////////////////////////////////////////////// Plane Arm Motion Planning //////////////////////////////////////////////////////////////////////////////
    bool PoseQualityMeasurement::computeArmMtion()
    {
        std::vector<Eigen::VectorXd> q_goal_set;
        q_goal_set.clear();
        int nb_of_sols = 0 ;

        for (int i = 0; i < 10; i++)
        {
            // if (solveIK(target_pose_, rrt_model_))
            // {
            //     q_goal_set.push_back(joint_state_.qGoal_);
            //     nb_of_sols++;
            // }
        }

        if (nb_of_sols == 0)
        {
            ROS_INFO_STREAM("IK solutions does not exist!!");
            // output_arm_trajectory_.points.clear();
            // arm_trajectory = output_arm_trajectory_;
            return false;
        }        
    }
}
