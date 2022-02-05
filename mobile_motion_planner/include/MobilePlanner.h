#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#define Hz 100.0

#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include "NHRRTFunction.h"
#include "YamlConfig.h"

#include <ros/ros.h>

// for input data
#include "geometry_msgs/Pose2D.h"
#include "task_assembly/plan_mobile_motion.h"
#include "task_assembly/MobileTrajectory.h"

// for generating trajectory
#include "Trajectory.h"
#include "Path.h"


#include <ctime>

using namespace std;
using namespace Eigen;


class MobilePlanner
{

public: 
	MobilePlanner(YAMLConfig yaml_config);
	~MobilePlanner();

public:
	task_assembly::MobileTrajectory compute();
	void initializeData(task_assembly::plan_mobile_motion::Request &req);

// variable Setting

	// Manipulator joint states
	struct jointState {
		VectorXd qInit_;
		VectorXd qGoal_;
	};
	// Mobile base joint states
	struct mobileState {
		VectorXd qInit_;
		VectorXd qGoal_;
	};
	struct positionLimit {
		VectorXd lower_;
		VectorXd upper_;
	};

	jointState joint_state_;

	NonHolonomicRRT nh_rrt_;

	MatrixXd joint_target_, joint_target2_;

	//////////////////// Service Server Parameters ///////////////////////
	YAMLConfig config_;
	unsigned int end_effector_id_;
	unsigned int arm_base_frame_id_;
	Vector3d end_effector_com_;
	Vector3d arm_base_frame_pos_;
	int cSpaceDimension;
	std::vector<int> body_id_collision_;
	std::vector<Vector3d> body_com_position_;
	positionLimit position_limit_;

	Transform3d target_pose_, init_pose_;
	mobileState mobile_state_;
	bool constrain_pose_;


	task_assembly::MobileTrajectory output_trajectory_;
	
	/////////////////////////////////////////////////////////////////////////
	bool setupAndSolveNHRRRT(VectorXd q_start, VectorXd q_goal, MatrixXd& joint_target);
	std::string urdf_param_;

		// Trajectory library // 
	Trajectory *trajectory_generator_;
	double duration_;
	VectorXd maxAcceleration;
	VectorXd maxVelocity;
	list<VectorXd> wayPoints;
		double playTime_ ;

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
