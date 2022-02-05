#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#define Hz_ 100.0

#include <iostream>
#include <Eigen/Dense>
#include <memory>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

// msgs
#include <trajectory_msgs/JointTrajectory.h>
#include <task_assembly/MobileTrajectory.h>

#include "Utils.h"

using namespace std;
using namespace Eigen;


class Controller
{

public: 
	Controller();
	~Controller();

public:
	void compute();
	bool mobileController_PurePursuit();
	bool mobileController_Kanayama();
	bool jointPIDControl(double duration, bool left);
	bool mobilePIDControl(double duration);

	void readData(VectorXd &ql, VectorXd &qr, VectorXd &qldot, VectorXd &qrdot, Vector3d base_pos, Vector3d base_twist);
	void writeData(VectorXd& ql, VectorXd& qr, VectorXd& base);
    void getTrajectory(trajectory_msgs::JointTrajectory &ql_traj, trajectory_msgs::JointTrajectory &qr_traj, task_assembly::MobileTrajectory &mob_traj);
	double cubicSpline(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double rx_f, double rx_dot_f);
	void checkTrajectory(bool &left, bool &right, bool &mobile);
	double normAngle(double& angle, double min);



    // Time Variable
	double playTime_;
	double cnt_;

	struct jointState {
		VectorXd q_;
		VectorXd qdot_;
		VectorXd qInit_;
		VectorXd qdotInit_;
		VectorXd qdotGoal_;
	};
	struct jointTarget
	{
		double start_time_;
		trajectory_msgs::JointTrajectory q_traj_;
		VectorXd q_;
		VectorXd desired_q_;
		VectorXd cubic_q_;

		int nb_of_points;
		int traj_index;
		bool joint_flag;

		double controlStartTime_;
	};
	struct baseState{
		geometry_msgs::Pose2D pose_; // x, y, theta
		geometry_msgs::Pose2D poseInit_;
		geometry_msgs::Twist twist_;

		Vector3d pose_error_local_;
		Vector3d pose_error_global_;
		Matrix3d Rot_global_;

	};
	struct baseTarget{
		geometry_msgs::Pose2D poseGoal_; // x, y, theta
		geometry_msgs::Pose2D poseCubic_;
		geometry_msgs::Pose2D poseTarget_;

		Vector4d desired_vel_; // wheel velocity
		task_assembly::MobileTrajectory q_traj_;


		int ctrl_mode;
		int nb_of_points;
		int traj_index;

		double controlStartTime_;
		bool mobile_flag;

		double v_r;
		double w_r;

		Vector2d desired_v_w_;
		Vector2d desired_wheel_;

	};
	struct baseParam{
		double b, c, r, d;
		double maxVel, distThre;
		double distBuff, ratio;
		double distToTarget;
		double oriErr;

		double Kx, Ky, Ktheta;

		Matrix2d vel_to_wheel;
	};


	jointState js_l_, js_r_;
	jointTarget jt_l_, jt_r_;
	baseState bs_;
	baseTarget bt_;
	baseParam bp_;

	bool joint_left_exec_, joint_right_exec_, mobile_exec_;
	bool initialize_;

    // Mobile Robot Controller Variable
	MatrixXd mobile_S;
	MatrixXd mobile_v;
	MatrixXd mobile_p_err;

};
#endif