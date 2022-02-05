#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#define Hz 100.0

#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include "RRTFunction.h"
#include "YamlConfig.h"

#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/node_handle.h>
#include <urdf/model.h>

#include <rbdl/addons/urdfreader/urdfreader.h>

// for output joint trajectory
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"

// for input data
#include "task_assembly/plan_arm_motion.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"

// for generating trajectory
// #include "trajectory/Trajectory.h"
// #include "trajectory/Path.h"

// for generating trajectory
#include "/home/min/repos/trajectory_smoothing/include/Trajectory.h"
#include "/home/min/repos/trajectory_smoothing/include/Path.h"

#include <ctime>

  // random with microsec

using namespace RigidBodyDynamics;
using namespace std;
using namespace Eigen;

class ArmPlanner
{

public: 
	ArmPlanner(YAMLConfig yaml_config);
	~ArmPlanner();

public:
	void compute(trajectory_msgs::JointTrajectory& arm_trajectory);
	void initModel();
	void initializeData(task_assembly::plan_arm_motion::Request &req);
	void generalizeRM(int res);
	void computeIRM();
	geometry_msgs::Pose2D calcBasePlacement();
// variable Setting

	// Manipulator joint states
	struct jointState {
		VectorXd qInit_;
		VectorXd qGoal_;
	};
	// Mobile base joint states
	struct mobileState {
		VectorXd qInit_;
		Matrix3d rotInit_;
	};
	struct jointLimit {
		VectorXd lower_;
		VectorXd upper_;
		VectorXd lower_rad_;
		VectorXd upper_rad_;
	};


	//// RBDL ////
	Model rbdl_model_;
	jointState joint_state_;

	RRT rrt_;

	Robotmodel rrt_model_;
	MatrixXd joint_target_, joint_target2_;

	// Trajectory library // 
	bool interpolate_path_;
	Trajectory *trajectory_generator_;
	double duration_;
	VectorXd maxAcceleration;
	VectorXd maxVelocity;
	list<VectorXd> wayPoints;


	double playTime_ ;

	//////////////////// Service Server Parameters ///////////////////////
	YAMLConfig config_;
	unsigned int end_effector_id_;
	unsigned int arm_base_frame_id_;
	Vector3d end_effector_com_;
	Vector3d arm_base_frame_pos_;
	int nb_of_joints_;
	int nb_of_torso_joints_;
	std::vector<int> body_id_collision_;
	std::vector<Vector3d> body_com_position_;
	jointLimit joint_limit_;

    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;


	Transform3d target_pose_, init_pose_;
	Transform3d attached_box_pose_;
	mobileState mobile_pose_;
	bool constrain_pose_;
	trajectory_msgs::JointTrajectory output_arm_trajectory_;
	trajectory_msgs::JointTrajectory output_torso_trajectory_;
	trajectory_msgs::JointTrajectoryPoint arm_trajectory_point_;
	trajectory_msgs::JointTrajectoryPoint torso_trajectory_point_;

	Eigen::Matrix3d getEigenRotation(const KDL::Rotation &r)
	{
		Eigen::Matrix3d matrix;
		for (int i = 0; i < 9; i++)
		{
			matrix(i / 3, i % 3) = r.data[i];
		}
		return matrix;
	}

	Eigen::Vector3d getEigenVector(const KDL::Vector &v)
	{
		Eigen::Vector3d vector;
		for (int i = 0; i < 3; i++)
		{
			vector(i) = v(i);
		}
		return vector;
	}

	Vector3d base_frame_pos_;

	bool goal_info_from_SE3_;

	int torso_dof;
	int arm_dof;
	int total_dof;

	/////////////////////////////////////////////////////////////////////////

	bool setupRRT(VectorXd q_start, VectorXd q_goal, MatrixXd& joint_target);
	bool setupCRRT(VectorXd q_start, VectorXd q_goal, MatrixXd& joint_target);
	bool initializeIKparam(const std::string& base_link, const std::string& tip_link, const std::string& URDF_param);
	KDL::JntArray IK_lb, IK_ub;
	KDL::Chain IK_chain;
	KDL::Tree IK_tree;
  	urdf::Model IK_robot_model;

	bool solveRRTwithMultipleGoals(VectorXd q_start, std::vector<VectorXd> q_goal_set, MatrixXd& joint_target);
	bool solveCRRTwithMultipleGoals(VectorXd q_start, std::vector<VectorXd> q_goal_set, MatrixXd& joint_target);
	bool solveIK(Transform3d pose, Robotmodel &model);
	std::string urdf_param_;
	
	double fRand(double min, double max)
	{
		double f = (double)rand() / RAND_MAX;
		return min + f * (max - min);
	};

	static const Matrix3d Vec_2_Rot(const Vector3d& vec) {
		Matrix3d Rot;
		double angle = vec.norm();

		if (angle == 0.0)
			Rot = Matrix3d::Identity();
		else {
			Vector3d axis = vec / angle;
			Matrix3d V;
			V.setZero();
			V(0, 1) = -axis(2);
			V(0, 2) = axis(1);
			V(1, 0) = axis(2);
			V(1, 2) = -axis(0);
			V(2, 0) = -axis(1);
			V(2, 1) = axis(0);

			Rot = Matrix3d::Identity() + V * sin(angle) + (1 - cos(angle))* V* V;
		}
		return Rot;
	};
};
#endif