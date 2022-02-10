#include "Controller.h"
#include <iostream>
// FILE *fp1 = fopen("/home/kendrick/catkin_ws/src/mobile_data.txt","w");

#define DEGREE M_PI / 180.0

Controller::Controller()
{
	js_l_.qdot_.resize(dof);
	js_l_.q_.resize(dof);
	js_l_.qInit_.resize(dof);
	js_l_.qdotInit_.resize(dof);
	js_l_.qdotGoal_.resize(dof);

	js_l_.qdot_.setZero();
	js_l_.q_.setZero();
	js_l_.qInit_.setZero();
	js_l_.qdotInit_.setZero();
	js_l_.qdotGoal_.setZero();

	js_r_.qdot_.resize(dof);
	js_r_.q_.resize(dof);
	js_r_.qInit_.resize(dof);
	js_r_.qdotInit_.resize(dof);
	js_r_.qdotGoal_.resize(dof);

	js_r_.qdot_.setZero();
	js_r_.q_.setZero();
	js_r_.qInit_.setZero();
	js_r_.qdotInit_.setZero();
	js_r_.qdotGoal_.setZero();

	jt_l_.desired_q_.resize(dof);
	jt_l_.desired_q_.setZero();

	jt_r_.desired_q_.resize(dof);
	jt_r_.desired_q_.setZero();

	jt_l_.cubic_q_.resize(dof);
	jt_l_.cubic_q_.setZero();

	jt_r_.cubic_q_.resize(dof);
	jt_r_.cubic_q_.setZero();

	jt_l_.q_.resize(dof);
	jt_l_.q_.setZero();

	jt_r_.q_.resize(dof);
	jt_r_.q_.setZero();

	////////////// Mobile ////////////////
	bp_.b = 0.545; // width
	bp_.r = 0.165; // wheel raidus
	bp_.c = bp_.r / (2.0 * bp_.b);
	bp_.d = 0.35; //축과 모바일 중심사이 거리

	bp_.maxVel = 1.2;
	bp_.distThre = 0.05; //0.05

	mobile_S.resize(2, 2);
	mobile_S.setZero();

	mobile_v.resize(2, 1);
	mobile_v.setZero();

	mobile_p_err.resize(2, 1);
	mobile_p_err.setZero();

	bt_.desired_vel_.setZero();

	bp_.vel_to_wheel(0,0) = bp_.r / 2.0;
	bp_.vel_to_wheel(0,1) = bp_.r / 2.0;

	bp_.vel_to_wheel(1,0) = bp_.r / (2.0*bp_.b);
	bp_.vel_to_wheel(1,1) = - bp_.r / (2.0*bp_.b);

	////////////////////////

	playTime_ = 0.0;
	cnt_ = 0.0;

	joint_left_exec_ = false;
	joint_right_exec_ = false;
	mobile_exec_ = false;

	initialize_ = true;

	mobilePlanEnded = false;
}
Controller::~Controller()
{
}

void Controller::compute()
{
	playTime_ = cnt_ / Hz_;

	if (initialize_)
	{
		jt_l_.desired_q_ = js_l_.q_;
		jt_r_.desired_q_ = js_r_.q_;
		initialize_ = false;
	}

	/////////// Data 
	if (joint_left_exec_)
	{
		jt_l_.controlStartTime_ = playTime_;
		//jt_l_.start_time_ = playTime_;
		jt_l_.nb_of_points = jt_l_.q_traj_.points.size();
		jt_l_.traj_index = 0;

		js_l_.qInit_ = js_l_.q_;
		js_l_.qdotInit_ = js_l_.qdot_;
		joint_left_exec_ = false;
	}

	if (joint_right_exec_)
	{
		jt_r_.controlStartTime_ = playTime_;
		//jt_r_.start_time_ = playTime_;
		jt_r_.nb_of_points = jt_r_.q_traj_.points.size();
		jt_r_.traj_index = 0;

		js_r_.qInit_ = js_r_.q_;
		js_r_.qdotInit_ = js_r_.qdot_;
		joint_right_exec_ = false;
	}

	if (mobile_exec_)
	{
		bt_.controlStartTime_ = playTime_;

		bt_.nb_of_points = bt_.q_traj_.points.size();
		bt_.traj_index = 0;
		bt_.ctrl_mode = 1;
		bt_.mobile_flag = true;

		bt_.poseGoal_.x = bt_.q_traj_.points.back().x;
		bt_.poseGoal_.y = bt_.q_traj_.points.back().y;
		bt_.poseGoal_.theta = bt_.q_traj_.points.back().theta;

		bs_.poseInit_ = bs_.pose_;
		mobile_exec_ = false;
	}

	//////////////////////////////////////////////////////////////////////
	if(mobilePlanEnded)
	{
		if (!jt_l_.q_traj_.points.empty())
		{
			jt_l_.joint_flag = false;

			if (jt_l_.nb_of_points != jt_l_.traj_index)
				for (int i = 0; i < dof; i++)
					jt_l_.q_(i) = jt_l_.q_traj_.points[jt_l_.traj_index].positions[i] * DEGREE;

			jt_l_.joint_flag = jointPIDControl(0.01, true); //2.0

			//std::cout << jt_l_.desired_q_.transpose() << std::endl;

			if (jt_l_.joint_flag && jt_l_.traj_index < jt_l_.nb_of_points)
			{
				jt_l_.controlStartTime_ = playTime_;
				jt_l_.traj_index++;

				js_l_.qInit_ = js_l_.q_;
			}
			else
			{
				// jt_l_.q_traj_.points.clear();
			}
		}

		if (!jt_r_.q_traj_.points.empty())
		{
			jt_r_.joint_flag = false;

			if (jt_r_.nb_of_points != jt_r_.traj_index)
				for (int i = 0; i < dof; i++)
					jt_r_.q_(i) = jt_r_.q_traj_.points[jt_r_.traj_index].positions[i] * DEGREE;

			jt_r_.joint_flag = jointPIDControl(0.01, false); //2.0

			//std::cout << jt_r_.desired_q_.transpose() << std::endl;

			if (jt_r_.joint_flag && jt_r_.traj_index < jt_r_.nb_of_points)
			{
				jt_r_.controlStartTime_ = playTime_;
				jt_r_.traj_index++;
				//std::cout << jt_r_.traj_index << std::endl;

				js_r_.qInit_ = js_r_.q_;
				js_r_.qdotInit_ = js_r_.qdot_;
			}
			else
			{
				// jt_r_.q_traj_.points.clear();
			}
		}
	}

	if (!bt_.q_traj_.points.empty())
	{
		mobileController_PurePursuit();
		// mobileController_Kanayama();
	}

	cnt_++;
}
bool Controller::mobileController_PurePursuit()
{
	// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ////// ctrl_mode = 0 : rotate to the next target orientation at current position (during path following)
	// ////// ctrl_mode = 1 : path following by using pure pursuit controller
	// ////// ctrl_mode = 2 : path following to last goal point by using S matrix controller
	// ////// ctrl_mode = 3 : rotate to the goal orientation at goal position
	// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//	std::cout << bt_.ctrl_mode << std::endl;
	//	std::cout << "x \t" << bt_.q_traj_.points[bt_.traj_index].x << "\t" << "y \t" << bt_.q_traj_.points[bt_.traj_index].y << std::endl;
	double errorMeasure = 0.0;
	mobilePlanEnded = false;

	if ((bt_.ctrl_mode == 1))
	{
		
		bp_.distToTarget = sqrt(pow((bt_.q_traj_.points[bt_.traj_index].x - bs_.pose_.x), 2) + pow((bt_.q_traj_.points[bt_.traj_index].y - bs_.pose_.y), 2));

		if ((bt_.nb_of_points - 1) <= bt_.traj_index)
		{
			bp_.distThre = 0.015; //0.01
		}

		if (bp_.distToTarget < bp_.distThre)
		{
			bt_.mobile_flag = true;
			bt_.traj_index++;

			// if ((bt_.nb_of_points - 1) == bt_.traj_index)
			// {
			// 	bt_.ctrl_mode = 2;
			// }
			if ((bt_.nb_of_points) == bt_.traj_index)
			{
				bt_.ctrl_mode = 2;
			}
		}
	}

	switch (bt_.ctrl_mode)
	{

	case 1:
		mobile_S(0, 0) = bp_.c * (bp_.b * cos(bs_.pose_.theta) - bp_.d * sin(bs_.pose_.theta));
		mobile_S(0, 1) = bp_.c * (bp_.b * cos(bs_.pose_.theta) + bp_.d * sin(bs_.pose_.theta));
		mobile_S(1, 0) = bp_.c * (bp_.b * sin(bs_.pose_.theta) + bp_.d * cos(bs_.pose_.theta));
		mobile_S(1, 1) = bp_.c * (bp_.b * sin(bs_.pose_.theta) - bp_.d * cos(bs_.pose_.theta));

		// 인풋 순서 WR WL
		//mobile_v_err(0) = -(bs_.pose_(0) - bs_.pose_pre_(0)) / Hz;
		//mobile_v_err(1) = -(bs_.pose_(1) - bs_.pose_pre_(1)) / Hz;

		if (bt_.mobile_flag)
		{
			bt_.poseTarget_.x = bt_.q_traj_.points[bt_.traj_index].x;
			bt_.poseTarget_.y = bt_.q_traj_.points[bt_.traj_index].y;

			std::cout<<bt_.poseTarget_<<std::endl;

			bt_.controlStartTime_ = playTime_;

			bs_.poseInit_.x = bs_.pose_.x;
			bs_.poseInit_.y = bs_.pose_.y;

			bt_.mobile_flag = false;
		}

		//bt_.mobile_flag = mobilePIDControl(0.5); //cubic trajectory

		std::cout << "x target :  " << bt_.poseTarget_.x  << std::endl;
		std::cout << "y target :  " << bt_.poseTarget_.y  << std::endl;

		// PD Controll
		mobile_p_err(0) = 2.0 * (bt_.poseTarget_.x - bs_.pose_.x) - 0.5 * bs_.twist_.linear.x;
		mobile_p_err(1) = 2.0 * (bt_.poseTarget_.y - bs_.pose_.y) - 0.5 * bs_.twist_.linear.y;

		std::cout << "pos error" << mobile_p_err.transpose() << std::endl;

		//mobile_v = mobile_S.inverse() * (0.7 * mobile_p_err + 0.05 * mobile_v_err);

		// 시뮬레이션이기 때문에 반대로 구해주어야 한다.
		mobile_v = mobile_S.inverse() * (mobile_p_err);

		// WR WL
		// 최대 속도를 설정
		for (int i = 0; i < 2; i++)
		{
			if (abs(mobile_v(i)) > 4.0)
			{
				mobile_v(i) = 4.0 * mobile_v(i) / abs(mobile_v(i)); // max_vel : 2
			}
		}
		//fprintf(fp1, "%f\t %f\t %f\t %f\t %f\t %f\t \n", bt_.poseTarget_.x,  bt_.poseTarget_.y , bs_.pose_.x, bs_.pose_.y, mobile_v(0), mobile_v(1));

		bt_.desired_vel_(0) = mobile_v(1);  //왼 
		bt_.desired_vel_(1) = mobile_v(0);  //오
		bt_.desired_vel_(2) = mobile_v(0);
		bt_.desired_vel_(3) = mobile_v(1);

		break;

	case 2:
		if (bt_.mobile_flag)
		{
			bt_.controlStartTime_ = playTime_;
			bs_.poseInit_.theta = bs_.pose_.theta;
			bt_.mobile_flag = false;
		}

		bt_.poseTarget_.theta = bt_.q_traj_.points.back().theta;
		errorMeasure = bt_.poseTarget_.theta;

		if(bt_.poseTarget_.theta  > M_PI)
		{
			errorMeasure = bt_.poseTarget_.theta - 2 * M_PI;
		}
		
		bt_.poseCubic_.theta = cubicSpline(playTime_, bt_.controlStartTime_, bt_.controlStartTime_ + 5.0, bs_.poseInit_.theta, 0.0, errorMeasure, 0.0); //bt_.poseGoal_.theta,

		bp_.oriErr = bs_.pose_.theta - bt_.poseCubic_.theta;

		// errorMeasure = -fmod(bp_.oriErr, 2.0* M_PI);
		std::cout << "ori target  " << errorMeasure * 180 / M_PI << std::endl;
		std::cout << "ori error  " << fabs(errorMeasure - bs_.pose_.theta) * 180 / M_PI << std::endl;

		if(fabs(errorMeasure - bs_.pose_.theta) < 0.02)
		{
			bt_.ctrl_mode = 3;
			// stop
			bt_.desired_vel_(0) = 0.0;
			bt_.desired_vel_(1) = 0.0;
			bt_.desired_vel_(2) = 0.0;
			bt_.desired_vel_(3) = 0.0;
		}
		else
		{
			bp_.oriErr = 8 * normAngle(bp_.oriErr, -M_PI); // rotation gain: 3

			
			if (abs(bp_.oriErr) > 2.0) //0.5
			{
				bp_.oriErr = 0.5 * bp_.oriErr / abs(bp_.oriErr); // max_vel : 1
			}

			//회전 하기위해 반대방향으로
			bt_.desired_vel_(0) = 1.0 * bp_.oriErr;
			bt_.desired_vel_(1) = -1.0 * bp_.oriErr;
			bt_.desired_vel_(2) = -1.0 * bp_.oriErr;
			bt_.desired_vel_(3) = 1.0 * bp_.oriErr;
		}
		cout << "base ori: " << bs_.pose_.theta * 180 / M_PI << endl;
		break;

	default:
		// if(!bt_.q_traj_.points.empty())
		// {
		// 	bt_.ctrl_mode = 1;
		// 	mobilePlanEnded = false;
		// }	
		// else
		// {
			mobilePlanEnded = true;
		// }
		break;
	}

	// fprintf(fp1, "%f\t %f\t %f\t %f\t %f\t %f\t \n", bt_.poseTarget_.x,  bt_.poseTarget_.y ,bt_.poseGoal_.theta, bs_.pose_.x, bs_.pose_.y,bs_.pose_.theta);

	// bs_.pose_pre_ = bs_.pose_;
	return true;
}

bool Controller::mobileController_Kanayama()
{
	//	std::cout << bt_.ctrl_mode << std::endl;
	//	std::cout << "x \t" << bt_.q_traj_.points[bt_.traj_index].x << "\t" << "y \t" << bt_.q_traj_.points[bt_.traj_index].y << std::endl;

	// Calculate error posture
	bs_.Rot_global_ = rotateZaxis(bs_.pose_.theta);

	// pose_desired - pose_current
	bs_.pose_error_global_(0) = bt_.q_traj_.points[bt_.traj_index].x - bs_.pose_.x;
	bs_.pose_error_global_(1) = bt_.q_traj_.points[bt_.traj_index].y - bs_.pose_.y;
	// TODO : normAngle
	bs_.pose_error_global_(2) = bt_.q_traj_.points[bt_.traj_index].theta - bs_.pose_.theta;
	// double theata_error = bt_.q_traj_.points[bt_.traj_index].theta - bs_.pose_.theta;
	// bs_.pose_error_global_(2) = normAngle(theata_error,- M_PI); 

	// local 시점으로 보기위해서 인버스로 곱해버려
	bs_.pose_error_local_ = bs_.Rot_global_.transpose()*bs_.pose_error_global_;

	std::cout << "del x \t" << bs_.pose_error_local_(0) << "\t" << "del y \t" << bs_.pose_error_local_(1) << "\t" << "del t \t" << bs_.pose_error_local_(2) << std::endl;
	// std::cout << bp_.distToTarget << std::endl;

	bt_.v_r = 0.3;
	bt_.w_r = 0.2;
	// bt_.v_r = sqrt(pow((bt_.q_traj_.points[bt_.traj_index].x - bs_.pose_.x), 2) + pow((bt_.q_traj_.points[bt_.traj_index].y - bs_.pose_.y), 2) + 0.1*pow((bt_.q_traj_.points[bt_.traj_index].theta - bs_.pose_.theta), 2));
	// bt_.w_r = bs_.pose_error_global_(2);

	bp_.Kx = 15.0;
	bp_.Ky = 16.0;
	bp_.Ktheta =4.0;
	
	// bp_.Kx = 8.5;
	// bp_.Ky = 4.0;
	// bp_.Ktheta = 8.0;

	// damping ratio = Ktheta / (2*sqrt(Ky))
	// Ktheta.^2 = 4 Ky (critical damping)
	
	
	bt_.desired_v_w_(0) = bt_.v_r * cos(bs_.pose_error_local_(2)) + bp_.Kx*bs_.pose_error_local_(0);
	bt_.desired_v_w_(1) = bt_.w_r + bt_.v_r * ( bp_.Ky*bs_.pose_error_local_(1) + bp_.Ktheta*sin(bs_.pose_error_local_(2)) );



	if ((bt_.ctrl_mode == 1))
	{

		bp_.distToTarget = sqrt(pow((bt_.q_traj_.points[bt_.traj_index].x - bs_.pose_.x), 2) + pow((bt_.q_traj_.points[bt_.traj_index].y - bs_.pose_.y), 2) + 0.1*pow((bt_.q_traj_.points[bt_.traj_index].theta - bs_.pose_.theta), 2));

		// if ((bt_.nb_of_points - 1) <= bt_.traj_index)
		// {
		// 	// init 0.05r
		// 	bp_.distThre = 0.01;
		// }

		if (bp_.distToTarget < bp_.distThre)
		{
			bt_.mobile_flag = true;
			bt_.traj_index++;

			if ((bt_.nb_of_points) == bt_.traj_index)
			{
				bt_.ctrl_mode = 2;
			}
		}
	}

	switch (bt_.ctrl_mode)
	{
	case 1:


		//bt_.desired_v_w_(0) = bt_.v_r * cos(bs_.pose_error_local_(2)) + bp_.Kx*bs_.pose_error_local_(0);
		//bt_.desired_v_w_(1) = bt_.w_r + bt_.v_r * ( bp_.Ky*bs_.pose_error_local_(1) + bp_.Ktheta*sin(bs_.pose_error_local_(2)) );

		bt_.desired_wheel_ = bp_.vel_to_wheel.inverse()*bt_.desired_v_w_;
		std::cout << "wheel" << bt_.desired_wheel_.transpose() << std::endl;
		std::cout << "position error" << bp_.distToTarget << std::endl;
		// std::cout << "idx" << bt_.traj_index<<" / "<< bt_.nb_of_points <<std::endl;
		bt_.desired_vel_(0) = bt_.desired_wheel_(1);
		bt_.desired_vel_(1) = bt_.desired_wheel_(0);
		bt_.desired_vel_(2) = bt_.desired_wheel_(0);
		bt_.desired_vel_(3) = bt_.desired_wheel_(1);

		// fprintf(fp1, "%f\t %f\t %f\t %f\t \n", bt_.q_traj_.points[bt_.traj_index].x, bt_.q_traj_.points[bt_.traj_index].y, bs_.pose_.x, bs_.pose_.y);


		break;
	case 2:

		if (bt_.mobile_flag)
		{
			bt_.controlStartTime_ = playTime_;
			bs_.poseInit_.theta = bs_.pose_.theta;
			bt_.mobile_flag = false;
		}
		bt_.poseCubic_.theta = cubicSpline(playTime_, bt_.controlStartTime_, bt_.controlStartTime_ + 5.0, bs_.poseInit_.theta, 0.0, bt_.poseGoal_.theta, 0.0);

		bp_.oriErr = bs_.pose_.theta - bt_.poseCubic_.theta;
		
		std::cout << "ori error" << - bp_.oriErr  << std::endl;
		bp_.oriErr = 3 * normAngle(bp_.oriErr, -M_PI); // rotation gain: 3

		
		if (abs(bp_.oriErr) > 0.5)
		{
			bp_.oriErr = 0.5 * bp_.oriErr / abs(bp_.oriErr); // max_vel : 1
		}
		bt_.desired_vel_(0) = 1.0 * bp_.oriErr;
		bt_.desired_vel_(1) = -1.0 * bp_.oriErr;
		bt_.desired_vel_(2) = -1.0 * bp_.oriErr;
		bt_.desired_vel_(3) = 1.0 * bp_.oriErr;

		break;

	default:

		break;
	}
	// bs_.pose_pre_ = bs_.pose_;

	return true;
}

bool Controller::jointPIDControl(double duration, bool left)
{
	bool res = false;
	if (left == true)
	{
		for (int i = 0; i < dof; i++)
			jt_l_.cubic_q_(i) = cubicSpline(playTime_, jt_l_.controlStartTime_, jt_l_.controlStartTime_ + duration, js_l_.qInit_(i), 0.0, jt_l_.q_(i), 0.0);
		jt_l_.desired_q_ = jt_l_.cubic_q_;
	}
	else
	{
		for (int i = 0; i < dof; i++)
			jt_r_.cubic_q_(i) = cubicSpline(playTime_, jt_r_.controlStartTime_, jt_r_.controlStartTime_ + duration, js_r_.qInit_(i), 0.0, jt_r_.q_(i), 0.0);
		jt_r_.desired_q_ = jt_r_.cubic_q_;
	}

	if (left)
	{
		if (jt_l_.controlStartTime_ + duration < playTime_)
			res = true;
	}
	else
	{
		//std::cout << jt_r_.controlStartTime_  << "\t" <<  playTime_<< std::endl;
		if (jt_r_.controlStartTime_ + duration < playTime_)
			res = true;
	}

	return res;
}

bool Controller::mobilePIDControl(double duration)
{
	bool res = false;

	bt_.poseCubic_.x = cubicSpline(playTime_, bt_.controlStartTime_, bt_.controlStartTime_ + duration, bs_.poseInit_.x, 0.0, bt_.poseTarget_.x, 0.0);
	bt_.poseCubic_.y = cubicSpline(playTime_, bt_.controlStartTime_, bt_.controlStartTime_ + duration, bs_.poseInit_.y, 0.0, bt_.poseTarget_.y, 0.0);

	// if (bt_.controlStartTime_ + duration < playTime_)
	// 	res = true;

	return res;
}

void Controller::readData(VectorXd &ql, VectorXd &qr, VectorXd &qldot, VectorXd &qrdot, Vector3d base_pos, Vector3d base_twist)
{
	js_l_.q_ = ql;
	js_l_.qdot_ = qldot;
	js_r_.q_ = qr;
	js_r_.qdot_ = qrdot;

	bs_.pose_.x = base_pos(0);
	bs_.pose_.y = base_pos(1);
	bs_.pose_.theta = base_pos(2);

	bs_.twist_.linear.x = base_twist(0);
	bs_.twist_.linear.y = base_twist(1);
	bs_.twist_.angular.z = base_twist(2);
}
void Controller::writeData(VectorXd &ql, VectorXd &qr, VectorXd &base)
{
	ql = jt_l_.desired_q_;
	qr = jt_r_.desired_q_;
	base = bt_.desired_vel_;
}
void Controller::getTrajectory(trajectory_msgs::JointTrajectory &ql_traj, trajectory_msgs::JointTrajectory &qr_traj, task_assembly::MobileTrajectory &mob_traj)
{
	jt_l_.q_traj_ = ql_traj;
	jt_r_.q_traj_ = qr_traj;
	bt_.q_traj_ = mob_traj;
}

double Controller::cubicSpline(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double rx_f, double rx_dot_f)
{

	double rx_t;
	if (rT < rT_0)
	{
		rx_t = rx_0;
	}
	else if (rT >= rT_0 && rT < rT_f)
	{
		rx_t = rx_0 + rx_dot_0 * (rT - rT_0) + (3 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0)) - 2 * rx_dot_0 / ((rT_f - rT_0) * (rT_f - rT_0)) - rx_dot_f / ((rT_f - rT_0) * (rT_f - rT_0))) * (rT - rT_0) * (rT - rT_0) + (-2 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0) * (rT_f - rT_0)) + (rx_dot_0 + rx_dot_f) / ((rT_f - rT_0) * (rT_f - rT_0) * (rT_f - rT_0))) * (rT - rT_0) * (rT - rT_0) * (rT - rT_0);
	}
	else
	{
		rx_t = rx_f;
	}
	return (rx_t);
}

void Controller::checkTrajectory(bool &left, bool &right, bool &mobile)
{
	joint_left_exec_ = left;
	joint_right_exec_ = right;
	mobile_exec_ = mobile;
}

double Controller::normAngle(double &angle, double min)
{
	double temp_angle = angle;
	while (temp_angle >= min + 2 * M_PI)
	{
		temp_angle -= 2 * M_PI;
	}

	while (temp_angle < min)
	{
		temp_angle += 2 * M_PI;
	}

	double a = temp_angle;
	//cout << a << endl;

	return a;
}
