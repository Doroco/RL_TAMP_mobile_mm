#include "ArmPlanner.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
FILE *fp1 = fopen("/home/min/catkin_ws/src/position_data.txt","w");

#define DEGREE M_PI/180.0

using std::ofstream;


ArmPlanner::ArmPlanner(YAMLConfig yaml_config):
config_(yaml_config)
{
	
}
ArmPlanner::~ArmPlanner()
{
}
void ArmPlanner::initModel()
{

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

	//nb_of_joints_ = rbdl_model_.q_size;
	nb_of_joints_ = config_.joint_limit_lower.size(); // dof for using planning
	// Fixed Body : The value of max(unsigned int) is
	// * determined via std::numeric_limits<unsigned int>::max() and the
	// * default value of fixed_body_discriminator is max (unsigned int) / 2.
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

	// end_effector_com_ : 0.0, 0.0, 0.0 for robocare

	//cout << "end_effector_com_" << end_effector_com_.transpose() << endl;
}
void ArmPlanner::initializeData(task_assembly::plan_arm_motion::Request &req) {

	//cout << "nb_of_joints_" << nb_of_joints_ <<endl;
	//cout << "config_.joint_limit_lower.size()" << config_.joint_limit_lower.size()<< endl;

	// joint limit 
	//assert(nb_of_joints_ == config_.joint_limit_lower.size());
	//assert(nb_of_joints_ == config_.joint_limit_upper.size());
	joint_limit_.lower_.resize(nb_of_joints_);
	joint_limit_.lower_rad_.resize(nb_of_joints_);
	joint_limit_.upper_.resize(nb_of_joints_);
	joint_limit_.upper_rad_.resize(nb_of_joints_);

	for (std::vector<int>::size_type i=0;i<config_.joint_limit_lower.size();i++){
		joint_limit_.lower_(i) = config_.joint_limit_lower[i];
		joint_limit_.upper_(i) = config_.joint_limit_upper[i];
	}
	
	joint_limit_.lower_rad_ = joint_limit_.lower_ / 180.0*M_PI;
	joint_limit_.upper_rad_ = joint_limit_.upper_ / 180.0*M_PI;

	torso_dof = 0;
	arm_dof = req.current_joint_state.position.size();
	total_dof = torso_dof + arm_dof;

	nb_of_torso_joints_ = nb_of_joints_ - arm_dof; 

	int torso_residual = torso_dof - nb_of_torso_joints_;
	rrt_.torso_config.clear();

	// if (torso_residual > 0)
	// {
	// 	for (int i = 0; i < torso_residual; i++)
	// 		rrt_.torso_config.push_back(req.current_torso_joint_state.position[i]);
	// }

	// current joint position
	joint_state_.qInit_.resize(total_dof);
	// for (int i =0;i<torso_dof;i++)
	// 	joint_state_.qInit_(i) = req.current_torso_joint_state.position[i];

	for (int i=torso_dof;i<total_dof;i++)	
		joint_state_.qInit_(i) = req.current_joint_state.position[i-torso_dof];

    // output_torso_trajectory_.joint_names.clear();
    // for (int i = 0;i<torso_dof;i++)
    //     output_torso_trajectory_.joint_names.push_back(req.current_torso_joint_state.name[i]);

    output_arm_trajectory_.joint_names.clear();
        for (int i = 0;i<arm_dof;i++)
            output_arm_trajectory_.joint_names.push_back(req.current_joint_state.name[i]);

	// current base position
	mobile_pose_.qInit_.resize(3);
	mobile_pose_.qInit_(0) = req.current_mobile_state.x;
	mobile_pose_.qInit_(1) = req.current_mobile_state.y;
	mobile_pose_.qInit_(2) = req.current_mobile_state.theta;
	mobile_pose_.rotInit_ = rotateZaxis(req.current_mobile_state.theta);

	// initial pose &  target pose in Global frame
	init_pose_.translation().head(2) = mobile_pose_.qInit_.head(2);
	init_pose_.translation()(2) = 0.0;
	//init_pose_.translation() += mobile_pose_.rotInit_ * base_frame_pos_;
	init_pose_.translation() += mobile_pose_.rotInit_ * CalcBodyToBaseCoordinates(rbdl_model_, joint_state_.qInit_, end_effector_id_, end_effector_com_, true);
	init_pose_.linear() = CalcBodyWorldOrientation(rbdl_model_ , joint_state_.qInit_, end_effector_id_, true).transpose();
	// cout << "init_pose_.translation() \n" << init_pose_.translation().transpose() << endl;
	// cout << "init_pose_ \t" <<init_pose_.linear() << endl;

	// if (req.goal_state.Pose_info.data == true)
	// {
		goal_info_from_SE3_ = true;
		target_pose_.translation()(0) = req.target_ee_pose.position.x;
		target_pose_.translation()(1) = req.target_ee_pose.position.y;
		target_pose_.translation()(2) = req.target_ee_pose.position.z;
		Eigen::Quaterniond quat(req.target_ee_pose.orientation.w, req.target_ee_pose.orientation.x, req.target_ee_pose.orientation.y, req.target_ee_pose.orientation.z);		
		target_pose_.linear() = quat.toRotationMatrix();

	// }
	// else
	// {
	// 	goal_info_from_SE3_ = false;
	// 	joint_state_.qGoal_.resize(req.goal_state.Goal_config.position.size());
	// 	for (int i = 0; i < req.goal_state.Goal_config.position.size(); i++)
	// 		joint_state_.qGoal_(i) = req.goal_state.Goal_config.position[i];
	// }
	//cout << "target_pose_.linear() \t" << target_pose_.linear() << endl;
	//cout << "target_pose_.translation()" << target_pose_.translation().transpose() << endl;

	Vector3d from_init_to_goal_in_local = mobile_pose_.rotInit_.transpose()*(init_pose_.translation() - target_pose_.translation());

	// Obstacles 
	rrt_.box_num_obs = req.Obstacles3D.size();
	rrt_.Box_env.clear();
	rrt_.Box_robot.clear();
	
	// for (int i=0;i<rrt_.box_num_obs;i++)
	// {
	// 	Eigen::Quaterniond quat_box(req.Obstacles3D[i].Box_pose.orientation.w, req.Obstacles3D[i].Box_pose.orientation.x, req.Obstacles3D[i].Box_pose.orientation.y, req.Obstacles3D[i].Box_pose.orientation.z);
	// 	Matrix3d Box_rot_temp = quat_box.toRotationMatrix();
	// 	rrt_.Box_obs[i].fAxis = Vector3d(req.Obstacles3D[i].Box_dimension.x, req.Obstacles3D[i].Box_dimension.y, req.Obstacles3D[i].Box_dimension.z);
	// 	rrt_.Box_obs[i].vAxis[0] = mobile_pose_.rotInit_.transpose() * Box_rot_temp.col(0);
	// 	rrt_.Box_obs[i].vAxis[1] = mobile_pose_.rotInit_.transpose() * Box_rot_temp.col(1);
	// 	rrt_.Box_obs[i].vAxis[2] = mobile_pose_.rotInit_.transpose() * Box_rot_temp.col(2);
	// 	rrt_.Box_obs[i].vPos =  mobile_pose_.rotInit_.transpose()* Vector3d(req.Obstacles3D[i].Box_pose.position.x - mobile_pose_.qInit_(0), req.Obstacles3D[i].Box_pose.position.y - mobile_pose_.qInit_(1), req.Obstacles3D[i].Box_pose.position.z); // axis center pos
	// }

	for (int i = 0; i < rrt_.box_num_obs; i++)
	{
		ST_OBB box;
		box.id_name = req.Obstacles3D[i].Box_name.data;
		Eigen::Quaterniond quat_box(req.Obstacles3D[i].Box_pose.orientation.w, req.Obstacles3D[i].Box_pose.orientation.x, req.Obstacles3D[i].Box_pose.orientation.y, req.Obstacles3D[i].Box_pose.orientation.z);
		Matrix3d Box_rot_temp = quat_box.toRotationMatrix();
		box.fAxis = Vector3d(req.Obstacles3D[i].Box_dimension.x, req.Obstacles3D[i].Box_dimension.y, req.Obstacles3D[i].Box_dimension.z);
		box.vAxis[0] = mobile_pose_.rotInit_.transpose() * Box_rot_temp.col(0);
		box.vAxis[1] = mobile_pose_.rotInit_.transpose() * Box_rot_temp.col(1);
		box.vAxis[2] = mobile_pose_.rotInit_.transpose() * Box_rot_temp.col(2);
		box.vPos = mobile_pose_.rotInit_.transpose() * Vector3d(req.Obstacles3D[i].Box_pose.position.x - mobile_pose_.qInit_(0), req.Obstacles3D[i].Box_pose.position.y - mobile_pose_.qInit_(1), req.Obstacles3D[i].Box_pose.position.z); // axis center pos
		rrt_.Box_env.push_back(box);
	}

	// for (int i=0;i<rrt_.box_num_obs;i++)
	// 	cout << rrt_.Box_env[i].vPos << endl;

	// cout << rrt_.Box_env.size() << endl;
	// cout << rrt_.Box_env[0].fAxis << endl;
	// cout << rrt_.Box_env[1].fAxis << endl;

	// link data (link dimension)
	// 킬때마다 새로 실행됨...
	// rrt_.box_num_link = config_.link_name.size();
	// for (int i=0;i<rrt_.box_num_link;i++){
	// 	Vector3d link_dim;
	// 	for (int j=0;j<config_.link_dimension[i].size();j++){
	// 		link_dim(j) =  config_.link_dimension[i][j];
	// 		rrt_.Box_link[i].vCenter(j) = config_.link_position[i][j];
	// 	}
	// 	rrt_.Box_link[i].vRot = rotateXaxis(config_.link_orientation[i][0]/180.0*M_PI)*rotateYaxis(config_.link_orientation[i][1]/180.0*M_PI)*rotateZaxis(config_.link_orientation[i][2]/180.0*M_PI);
	// 	rrt_.Box_link[i].fAxis = link_dim;
	// }
	
	rrt_.box_num_link = config_.link_name.size();
	for (int i=0;i<rrt_.box_num_link;i++){
		ST_OBB box2;
		Vector3d link_dim;
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
	//for (int i=0;i<rrt_.box_num_link;i++)
		//cout << rrt_.Box_robot[i].fAxis << endl;

	//////////////////////////////// Attach Box to Robot ////////////////////////////////
	Matrix3d ee_rot;
	ee_rot.setZero();
	ee_rot(0,0) = -1.0;
	ee_rot(1,2) = 1.0;
	ee_rot(2,1) = 1.0;

	// if (req.AttachBox.Attach_to_robot.data == true)
	// {
	// 	// Remove the information about last 2 links(Finger links)
	// 	rrt_.Box_robot.pop_back();
	// 	rrt_.Box_robot.pop_back();
	// 	config_.link_dimension.pop_back();
	// 	config_.link_dimension.pop_back();
	// 	config_.link_name.pop_back();
	// 	config_.link_name.pop_back();

	// 	// Convert global pose to local
	// 	Matrix3d global_to_local_ee = (init_pose_.linear() * ee_rot.transpose()).transpose() * mobile_pose_.rotInit_.transpose();
	// 	Vector3d local_position_ = global_to_local_ee * Vector3d(req.AttachBox.Box_pose.position.x - init_pose_.translation()(0), req.AttachBox.Box_pose.position.y - init_pose_.translation()(1), req.AttachBox.Box_pose.position.z - init_pose_.translation()(2));
	// 	attached_box_pose_.translation() = local_position_;
	// 	Eigen::Quaterniond quat_attached_box(req.AttachBox.Box_pose.orientation.w, req.AttachBox.Box_pose.orientation.x, req.AttachBox.Box_pose.orientation.y, req.AttachBox.Box_pose.orientation.z);
	// 	attached_box_pose_.linear() = (init_pose_.linear() * ee_rot.transpose()).transpose() * mobile_pose_.rotInit_.transpose() * quat_attached_box.toRotationMatrix();
	// 	std::vector<double> fake_link_dim;
	// 	fake_link_dim.clear();
	// 	fake_link_dim.push_back(0.1);
	// 	fake_link_dim.push_back(0.1);
	// 	fake_link_dim.push_back(0.1);

	// 	config_.link_dimension.push_back(fake_link_dim);
	// 	ST_OBB attached_box;
	// 	attached_box.fAxis(0) = req.AttachBox.Box_dimension.x;
	// 	attached_box.fAxis(1) = req.AttachBox.Box_dimension.y;
	// 	attached_box.fAxis(2) = req.AttachBox.Box_dimension.z;
	// 	attached_box.vRot = attached_box_pose_.linear();

	// 	if (req.AttachBox.Right_arm.data == true)
	// 	{
	// 		std::string link_name = "RWrist_Roll";
	// 		config_.link_name.push_back(link_name);
	// 		attached_box.vCenter(0) = attached_box_pose_.translation()(0); 
	// 		attached_box.vCenter(1) = attached_box_pose_.translation()(1) - 0.075; 
	// 		attached_box.vCenter(2) = attached_box_pose_.translation()(2); 
	// 		rrt_.box_num_link = config_.link_name.size();
	// 	}
	// 	else
	// 	{
	// 		std::string link_name = "LWrist_Roll";
	// 		config_.link_name.push_back(link_name);
	// 		attached_box.vCenter(0) = attached_box_pose_.translation()(0); 
	// 		attached_box.vCenter(1) = attached_box_pose_.translation()(1) + 0.075; 
	// 		attached_box.vCenter(2) = attached_box_pose_.translation()(2);
	// 		rrt_.box_num_link = config_.link_name.size();
	// 	}
	// 	rrt_.Box_robot.push_back(attached_box);
	// }

	////////////////////////////////////////////////////////////////////////////////////////////
	this->initializeIKparam(config_.chain_start, config_.chain_end, urdf_param_);

	//////////////////////////////// Comparison between RBDL and KDL forward kinematics
	// Eigen::Isometry3d pose_from_rbdl;
	// pose_from_rbdl.translation() = CalcBodyToBaseCoordinates(rbdl_model_, joint_state_.qInit_, arm_base_frame_id_, arm_base_frame_pos_, true);
	// pose_from_rbdl.linear() = CalcBodyWorldOrientation(rbdl_model_ , joint_state_.qInit_, arm_base_frame_id_, true).transpose();

	// Eigen::Isometry3d init_pose_ee;
	// init_pose_ee.translation() = CalcBodyToBaseCoordinates(rbdl_model_, joint_state_.qInit_, end_effector_id_, end_effector_com_, true);
	// init_pose_ee.linear() = CalcBodyWorldOrientation(rbdl_model_ , joint_state_.qInit_, end_effector_id_, true).transpose();

	// cout << "pose_from_rbdl.translation() \n" << pose_from_rbdl.translation().transpose() << endl;
	// cout << "pose_from_rbdl \n" <<pose_from_rbdl.linear() << endl;	

	// Eigen::Isometry3d pose_from_base = pose_from_rbdl.inverse()*init_pose_ee;

	// cout << "pose_from_base.translation() \n" << pose_from_base.translation().transpose() << endl;
	// cout << "pose_from_base \n" <<pose_from_base.linear() << endl;	
	// KDL::ChainFkSolverPos_recursive fk_solver(IK_chain);									  // Forward kin. solver

	// KDL::JntArray jarr_q;
	// KDL::Frame frame;
	// jarr_q.resize(IK_chain.getNrOfJoints());
	// jarr_q.data = joint_state_.qInit_.tail(7);
	// fk_solver.JntToCart(jarr_q, frame);

  	// Eigen::Isometry3d transform;
  	// transform.translation() = getEigenVector(frame.p);
  	// transform.linear() = getEigenRotation(frame.M);
	  
	// cout << "trac ik frame" << endl;
	// cout << transform.translation().transpose() << endl;
	// cout << transform.linear() << endl;
	////////////////////////////////////////////////////////////////////////////////////////////

	// Constraint bound
	constrain_pose_ = req.Pose_bound.constrain_pose.data;
	rrt_.C.resize(6,2);
	// for (int i = 0; i < 3; i++)
	// {
	// 	if (from_init_to_goal_in_local(i) > 0)
	// 	{
	// 		rrt_.C(i, 0) = from_init_to_goal_in_local(i) + 0.05;
	// 		rrt_.C(i, 1) = -0.05;
	// 	}
	// 	else
	// 	{
	// 		rrt_.C(i, 0) = 0.05;
	// 		rrt_.C(i, 1) = from_init_to_goal_in_local(i) - 0.05;
	// 	}
	// }

	rrt_.C(0,0) = req.Pose_bound.position_bound_upper.x;
	rrt_.C(1,0) = req.Pose_bound.position_bound_upper.y;
	rrt_.C(2,0) = req.Pose_bound.position_bound_upper.z;

	rrt_.C(0,1) = req.Pose_bound.position_bound_lower.x;
	rrt_.C(1,1) = req.Pose_bound.position_bound_lower.y;
	rrt_.C(2,1) = req.Pose_bound.position_bound_lower.z;

	rrt_.C(3,0) = req.Pose_bound.orientation_bound_upper.x;
	rrt_.C(4,0) = req.Pose_bound.orientation_bound_upper.y;
	rrt_.C(5,0) = req.Pose_bound.orientation_bound_upper.z;

	rrt_.C(3,1) = req.Pose_bound.orientation_bound_lower.x;
	rrt_.C(4,1) = req.Pose_bound.orientation_bound_lower.y;
	rrt_.C(5,1) = req.Pose_bound.orientation_bound_lower.z;


	// Trajectory library 
	interpolate_path_ = req.interpolate_path.data;
	maxVelocity.resize(nb_of_joints_);
	maxVelocity.setZero();
	maxAcceleration.resize(nb_of_joints_);
	maxAcceleration.setZero();

	for (size_t i = 0; i < nb_of_joints_; i++)
	{
		maxAcceleration(i) = 10.0;
		maxVelocity(i) = 10.0;
	}
	wayPoints.clear();
	playTime_ = 0.0;
}


bool ArmPlanner::initializeIKparam(const std::string& base_link, const std::string& tip_link, const std::string& URDF_param){
	ros::NodeHandle node_handle("~");

	std::string xml_string;

	std::string urdf_xml, full_urdf_xml;
	node_handle.param("urdf_xml", urdf_xml, URDF_param);
	node_handle.searchParam(urdf_xml, full_urdf_xml);

	//ROS_DEBUG_NAMED("IK", "Reading xml file from parameter server");
	if (!node_handle.getParam(full_urdf_xml, xml_string))
	{
		ROS_INFO("Could not load the xml from parameter server");
		return false;
	}

	node_handle.param(full_urdf_xml, xml_string, std::string());
	IK_robot_model.initString(xml_string);

	//ROS_DEBUG_STREAM_NAMED("trac_ik", "Reading joints and links from URDF");

	if (!kdl_parser::treeFromUrdfModel(IK_robot_model, IK_tree))
		ROS_INFO("Failed to extract kdl tree from xml robot description");

	if (!IK_tree.getChain(base_link, tip_link, IK_chain))
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

	return true;
}

void ArmPlanner::compute(trajectory_msgs::JointTrajectory& arm_trajectory)
{
	// Get body Id of Link 

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
	// Write the model data
	rrt_model_.model_ = rbdl_model_;
	rrt_model_.body_id_vec.clear();
	rrt_model_.body_id_vec.assign(body_id_collision_.begin(), body_id_collision_.end());
	rrt_model_.body_com_pos.clear();
	rrt_model_.body_com_pos.assign(body_com_position_.begin(), body_com_position_.end());

	// for (int i=0;i<5;i++){
	// cout << CalcBodyToBaseCoordinates(rbdl_model_, joint_state_.qInit_, body_id_collision_[i], body_com_position_[i], true).transpose()<< endl;
	// }
	// Calculate IK solution
	std::vector<VectorXd> q_goal_set;
	q_goal_set.clear();
	int nb_of_sols = 0 ;
	if (goal_info_from_SE3_ == true)
	{
		for (int i = 0; i < 10; i++)
		{
			if (solveIK(target_pose_, rrt_model_))
			{
				q_goal_set.push_back(joint_state_.qGoal_);
				nb_of_sols++;
			}
		}
	}
	else
	{
		q_goal_set.push_back(joint_state_.qGoal_);
		target_pose_.translation().head(2) = mobile_pose_.qInit_.head(2);
		target_pose_.translation()(2) = 0.0;
		target_pose_.translation() += mobile_pose_.rotInit_ * CalcBodyToBaseCoordinates(rbdl_model_, joint_state_.qGoal_, end_effector_id_, end_effector_com_, true);	
		target_pose_.linear() = mobile_pose_.rotInit_ * CalcBodyWorldOrientation(rbdl_model_ , joint_state_.qGoal_, end_effector_id_, true).transpose();		
		nb_of_sols++;
	}

	if (nb_of_sols == 0)
	{
		ROS_INFO_STREAM("IK solutions does not exist!!");
		output_arm_trajectory_.points.clear();
		arm_trajectory = output_arm_trajectory_;
		return;
	}

	// Plan the trajectory
	if(constrain_pose_)
	{
		rrt_.refer_pos = target_pose_.translation(); 
		rrt_.refer_rot = target_pose_.linear();

		rrt_.local_pos.head(2) = mobile_pose_.qInit_.head(2);
		rrt_.local_pos(2) = 0.0;
		rrt_.local_rot = mobile_pose_.rotInit_;

		Vector3d ee_pos;
		//if (CRRT_planning(joint_state_.qInit_, joint_state_.qGoal_, joint_target2_))

		if (solveCRRTwithMultipleGoals(joint_state_.qInit_, q_goal_set, joint_target2_))
		{
			if (interpolate_path_)
			{
				// output_torso_trajectory_.points.clear();
				output_arm_trajectory_.points.clear();
				for (int i = 0; i < joint_target2_.rows(); i++)
				{
					wayPoints.push_back(joint_target2_.row(i));
				}
				trajectory_generator_ = new Trajectory(Path(wayPoints, 0.1), maxVelocity, maxAcceleration);
				//	trajectory_generator_->outputPhasePlaneTrajectory();
				duration_ = trajectory_generator_->getDuration();
				// cout <<"duration" << duration_ << endl;
				while (playTime_ / 10.0 < duration_)
				{
					// torso_trajectory_point_.positions.clear();
					// torso_trajectory_point_.velocities.clear();

					arm_trajectory_point_.positions.clear();
					arm_trajectory_point_.velocities.clear();

					// for (int i = 0; i < nb_of_torso_joints_; i++)
					// {
					// 	torso_trajectory_point_.positions.push_back(trajectory_generator_->getPosition(playTime_ / 10.0)[i]);
					// 	torso_trajectory_point_.velocities.push_back(trajectory_generator_->getVelocity(playTime_ / 10.0)[i]);
					// }
					for (int i = 0; i < nb_of_joints_; i++)
					{
						arm_trajectory_point_.positions.push_back(trajectory_generator_->getPosition(playTime_ / 10.0)[i]);
						arm_trajectory_point_.velocities.push_back(trajectory_generator_->getVelocity(playTime_ / 10.0)[i]);
					}

					// torso_trajectory_point_.time_from_start = ros::Duration(playTime_ / 10.0);
					arm_trajectory_point_.time_from_start = ros::Duration(playTime_ / 10.0);

					// output_torso_trajectory_.points.push_back(torso_trajectory_point_);
					output_arm_trajectory_.points.push_back(arm_trajectory_point_);

					//ee_pos = CalcBodyToBaseCoordinates(rbdl_model_, trajectory_generator_->getPosition(playTime_ / 10.0) / 180.0 * M_PI, end_effector_id_, end_effector_com_, true);
					//cout << ee_pos.transpose() << endl;
					playTime_++;
				}
			}
			else
			{
				// output_torso_trajectory_.points.clear();
				output_arm_trajectory_.points.clear();
				for (int i = 0; i < joint_target2_.rows(); i++)
				{
					//q_tra = joint_target2_.row(i) / 180.0 * M_PI;
					//ee_pos = CalcBodyToBaseCoordinates(rbdl_model_, q_tra, end_effector_id_, end_effector_com_, true);
					//cout << ee_pos.transpose() << endl;
					//cout << joint_target2_.row(i)/180.0*M_PI << endl;
					//cout << "\n"
					//	 << endl;
					//fprintf(fp1, "%f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t \n", joint_target2_(i, 0), joint_target2_(i, 1), joint_target2_(i, 2), joint_target2_(i, 3), joint_target2_(i, 4), joint_target2_(i, 5), joint_target2_(i, 6), ee_pos(0), ee_pos(1), ee_pos(2));
					cout <<"Path\t" << i << "\t" << "rad:" << joint_target2_.row(i)/180.0*M_PI << endl;
					cout << "Path\t" << i << "\t" << "angle:" << joint_target2_.row(i) << endl;					
					arm_trajectory_point_.positions.clear();
					// torso_trajectory_point_.positions.clear();

					// for (int j = 0; j < nb_of_torso_joints_; j++)
					// 	torso_trajectory_point_.positions.push_back(joint_target2_(i, j));

					for (int j = nb_of_torso_joints_; j < nb_of_joints_; j++)
						arm_trajectory_point_.positions.push_back(joint_target2_(i, j));

					// output_torso_trajectory_.points.push_back(torso_trajectory_point_);
					output_arm_trajectory_.points.push_back(arm_trajectory_point_);
				}
			}
		}
		else
		{
			// output_torso_trajectory_.points.clear();
			output_arm_trajectory_.points.clear();
		}
	}
	else
	{
		if (solveRRTwithMultipleGoals(joint_state_.qInit_, q_goal_set, joint_target_))
		{
			if (interpolate_path_)
			{
				// output_torso_trajectory_.points.clear();
				output_arm_trajectory_.points.clear();
				for (int i = 0; i < joint_target_.rows(); i++)
				{
					wayPoints.push_back(joint_target_.row(i));
				}
				trajectory_generator_ = new Trajectory(Path(wayPoints, 0.1), maxVelocity, maxAcceleration);
				//	trajectory_generator_->outputPhasePlaneTrajectory();
				duration_ = trajectory_generator_->getDuration();
				//cout <<"duration" << duration_ << endl;
				while (playTime_ / 10.0 < duration_)
				{
					// torso_trajectory_point_.positions.clear();
					// torso_trajectory_point_.velocities.clear();

					arm_trajectory_point_.positions.clear();
					arm_trajectory_point_.velocities.clear();

					// for (int i = 0; i < nb_of_torso_joints_; i++)
					// {
					// 	torso_trajectory_point_.positions.push_back(trajectory_generator_->getPosition(playTime_ / 10.0)[i]);
					// 	torso_trajectory_point_.velocities.push_back(trajectory_generator_->getVelocity(playTime_ / 10.0)[i]);
					// }
					for (int i = nb_of_torso_joints_; i < nb_of_joints_; i++)
					{
						arm_trajectory_point_.positions.push_back(trajectory_generator_->getPosition(playTime_ / 10.0)[i]);
						arm_trajectory_point_.velocities.push_back(trajectory_generator_->getVelocity(playTime_ / 10.0)[i]);
					}

					// torso_trajectory_point_.time_from_start = ros::Duration(playTime_ / 10.0);
					arm_trajectory_point_.time_from_start = ros::Duration(playTime_ / 10.0);

					// output_torso_trajectory_.points.push_back(torso_trajectory_point_);
					output_arm_trajectory_.points.push_back(arm_trajectory_point_);

					//ee_pos = CalcBodyToBaseCoordinates(rbdl_model_, trajectory_generator_->getPosition(playTime_ / 10.0) / 180.0 * M_PI, end_effector_id_, end_effector_com_, true);
					//cout << ee_pos.transpose() << endl;
					playTime_++;
				}
			}
			else
			{
				// output_torso_trajectory_.points.clear();
				output_arm_trajectory_.points.clear();
				for (int i = 0; i < joint_target_.rows(); i++)
				{
					//q_tra = joint_target_.row(i) / 180.0 * M_PI;
					// ee_pos = CalcBodyToBaseCoordinates(rbdl_model_, q_tra, end_effector_id_, end_effector_com_, true);
					// cout << ee_pos.transpose() << endl;
					cout <<"Path\t" << i << "\t" << "rad:" << joint_target_.row(i)/180.0*M_PI << endl;
					cout << "Path\t" << i << "\t" << "angle:" << joint_target_.row(i) << endl;
					//fprintf(fp1, "%f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t \n", joint_target2_(i, 0), joint_target2_(i, 1), joint_target2_(i, 2), joint_target2_(i, 3), joint_target2_(i, 4), joint_target2_(i, 5), joint_target2_(i, 6), ee_pos(0), ee_pos(1), ee_pos(2));
					arm_trajectory_point_.positions.clear();
					// torso_trajectory_point_.positions.clear();

					// for (int j = 0; j < nb_of_torso_joints_; j++)
					// 	torso_trajectory_point_.positions.push_back(joint_target_(i, j));

					for (int j = nb_of_torso_joints_; j < nb_of_joints_; j++)
						arm_trajectory_point_.positions.push_back(joint_target_(i, j));

					// output_torso_trajectory_.points.push_back(torso_trajectory_point_);
					output_arm_trajectory_.points.push_back(arm_trajectory_point_);
				}
			}

		}
		else
		{
			// output_torso_trajectory_.points.clear();
			output_arm_trajectory_.points.clear();

		}
	}

	for (int i = 0; i < joint_target2_.rows(); i++)
	{
		cout <<"Path\t" << i << "\t" << "rad:" << joint_target2_.row(i)/180.0*M_PI << endl;
		cout << "Path\t" << i << "\t" << "angle:" << joint_target2_.row(i) << endl;		
	}
	// torso_trajectory = output_torso_trajectory_;
	arm_trajectory = output_arm_trajectory_;
	//return output_arm_trajectory_;
}


bool ArmPlanner::solveIK(Transform3d pose, Robotmodel& model) // from panda_arm.xacro
{
	double eps = 5e-3;
	double num_samples = 10;

	// Set up KDL IK
	// KDL::ChainFkSolverPos_recursive fk_solver(IK_chain);									  // Forward kin. solver
	// KDL::ChainIkSolverVel_pinv vik_solver(IK_chain);										  // PseudoInverse vel solver
	// KDL::ChainIkSolverPos_NR_JL kdl_solver(IK_chain, IK_lb, IK_ub, fk_solver, vik_solver, 100, eps); // Joint Limit Solver

	// Set up Track-IK
	std::string chain_start = config_.chain_start;
	std::string chain_end= config_.chain_end;

	TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param_, 100, eps);
	bool valid = tracik_solver.getKDLChain(IK_chain);

	if (!valid)
	{
		std::cout<<"There was no valid KDL chain found"<<std::endl;
		return false;
	}

	valid = tracik_solver.getKDLLimits(IK_lb,IK_ub);

	if (!valid)
	{
		std::cout<<"There were no valid KDL joint limits found"<<std::endl;
		return false;
	}

	// Create Nominal chain configuration midway between all joint limits
	KDL::JntArray nominal(IK_chain.getNrOfJoints());
		
	KDL::JntArray result;
	KDL::Frame end_effector_pose;

	Matrix4d target_from_base;
	Matrix4d target_global;
	Matrix4d base_global;
	base_global.setIdentity();
	target_from_base.setIdentity();
	target_global.setIdentity();

	Transform3d pose_from_rbdl;
	pose_from_rbdl.translation() = CalcBodyToBaseCoordinates(rbdl_model_, joint_state_.qInit_, arm_base_frame_id_, arm_base_frame_pos_, true);
	pose_from_rbdl.linear() = CalcBodyWorldOrientation(rbdl_model_ , joint_state_.qInit_, arm_base_frame_id_, true).transpose();

	base_global.block(0,0,3,3) = rotateZaxis(mobile_pose_.qInit_(2))*pose_from_rbdl.linear();
	base_global(0, 3) = mobile_pose_.qInit_(0) + pose_from_rbdl.translation()(0);
	base_global(1, 3) = mobile_pose_.qInit_(1) + pose_from_rbdl.translation()(1);
	base_global(2, 3) = pose_from_rbdl.translation()(2);

	target_global.block(0, 0, 3, 3) = pose.linear();
	target_global.block(0, 3, 3, 1) = pose.translation();

	target_from_base = base_global.inverse() * target_global;

	//cout << "target_global" << target_global << endl;
	//cout << "base_global" << base_global << endl;
	//cout << "target_from_base \n" << target_from_base << endl;

	for (int i = 0; i < 3; i++)
	{
		end_effector_pose.p(i) = target_from_base(i,3);
	}
	//cout << "waist to desired ee in local : " << target_from_base.block(0, 3, 3, 1).transpose() << endl;

	KDL::Rotation A;
	A.data[0] = target_from_base(0, 0);
	A.data[1] = target_from_base(0, 1);
	A.data[2] = target_from_base(0, 2);
	A.data[3] = target_from_base(1, 0);
	A.data[4] = target_from_base(1, 1);
	A.data[5] = target_from_base(1, 2);
	A.data[6] = target_from_base(2, 0);
	A.data[7] = target_from_base(2, 1);
	A.data[8] = target_from_base(2, 2);
	end_effector_pose.M = A;

	int rc;

	double total_time = 0;
	uint success = 0;
	bool solved = true;
	while(true) //for (uint i = 0; i < num_samples; i++)
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

		for (size_t j = 0; j < nominal.data.size(); j++)
		{
			nominal(j) = R[j];
		}

		//cout <<"iteration?" << endl;
		double elapsed = 0;
		//start_time = boost::posix_time::microsec_clock::local_time();
		rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result);
		// int CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist& bounds=KDL::Twist::Zero());

		std::vector<double> config;
		config.clear();
		if (rc >= 0)
		{
			//ROS_INFO_STREAM("IK solution is : " << result.data.transpose() * 180 / M_PI);
			for (int i = 0; i < nb_of_joints_; i++)
			{
				config.push_back(result.data(i) * 180 / M_PI);
			}

			// joint_state_.qGoal_ = result.data;
			// solved = true;
			// break;
			if (!rrt_.checkExternalCollision(model, config) && !rrt_.checkSelfCollision(model, config) )
			{

				ROS_INFO_STREAM("IK solution is : " << result.data.transpose() * 180 / M_PI);
				joint_state_.qGoal_ = result.data;
				solved = true;
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
bool ArmPlanner::setupRRT(VectorXd q_start, VectorXd q_goal, MatrixXd& joint_target)
{
	rrt_.lower_limit = joint_limit_.lower_;
	rrt_.upper_limit = joint_limit_.upper_;

	rrt_.qinit = q_start;
	rrt_.qgoal = q_goal;

	ofstream outFile("path_result.txt", ios::out); // open file for "writing"

	if (rrt_.solveRRT(rrt_model_, outFile))
	{
		ofstream outFile2("path_result2.txt", ios::out); // "writing"
		// path_result -> Smooth -> path_result2
		ifstream inFile("path_result.txt"); // "reading"
		rrt_.smoothPath(outFile2, inFile);

		outFile2.close();
		MatrixXd joint_temp(100, nb_of_joints_);

		ifstream inFile2("path_result2.txt"); // "reading"
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
bool ArmPlanner::setupCRRT(VectorXd q_start, VectorXd q_goal, MatrixXd &joint_target)
{
	rrt_.lower_limit = joint_limit_.lower_;
	rrt_.upper_limit = joint_limit_.upper_;

	rrt_.qinit = q_start;
	rrt_.qgoal = q_goal;

	ofstream outFile3("path_result3.txt", ios::out);

	if (rrt_.solveCRRT(rrt_model_, outFile3))
	{
		outFile3.close();

		MatrixXd joint_temp(5000, nb_of_joints_);

		ifstream inFile3("path_result3.txt");
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



bool ArmPlanner::solveRRTwithMultipleGoals(VectorXd q_start, std::vector<VectorXd> q_goal_set, MatrixXd &joint_target)
{
	rrt_.lower_limit = joint_limit_.lower_;
	rrt_.upper_limit = joint_limit_.upper_;

	rrt_.qinit = q_start;
	rrt_.qgoals.assign(q_goal_set.begin(), q_goal_set.end());

	ofstream outFile("path_result.txt", ios::out); // open file for "writing"

	if (rrt_.solveRRT(rrt_model_, outFile))
	{

		// outFile.close();
		// MatrixXd joint_temp(5000, nb_of_joints_);

		// ifstream inFile2("path_result.txt"); // "reading"
		// int size = 0;
		// std::vector<std::string> parameters;
		// char inputString[1000];


		ofstream outFile2("path_result2.txt", ios::out); // "writing"
		// path_result -> Smooth -> path_result2
		ifstream inFile("path_result.txt"); // "reading"
		rrt_.smoothPath(outFile2, inFile);

		outFile2.close();
		MatrixXd joint_temp(100, nb_of_joints_);

		ifstream inFile2("path_result2.txt"); // "reading"
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

bool ArmPlanner::solveCRRTwithMultipleGoals(VectorXd q_start, std::vector<VectorXd> q_goal_set, MatrixXd &joint_target)
{
	rrt_.lower_limit = joint_limit_.lower_;
	rrt_.upper_limit = joint_limit_.upper_;

	rrt_.qinit = q_start;
	rrt_.qgoals.assign(q_goal_set.begin(), q_goal_set.end());

	ofstream outFile3("path_result3.txt", ios::out);

	if (rrt_.solveCRRT(rrt_model_, outFile3))
	{
		outFile3.close();

		// cout << "111"<<endl;
		MatrixXd joint_temp(5000, nb_of_joints_);

		ifstream inFile3("path_result3.txt");
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

void ArmPlanner::generalizeRM(int res)
{
	//프랑카 팔의 길이 
	float l_max = 1.19;

	// voxel index;
	int idx = 0;
	int sample_number = 0;

	while (true)
	{
		for(float voxel_x = -l_max; voxel_x < l_max; voxel_x += res)
		{
			for(float voxel_y = l_max; voxel_y < l_max; voxel_y += res)
			{
				for(float voxel_z = l_max; voxel_z < l_max; voxel_z += res)
				{
					
					idx++;
				}
			}
		}
		break;
	}
}