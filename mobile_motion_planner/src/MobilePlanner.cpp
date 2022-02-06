#include "MobilePlanner.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>

FILE *fp1 = fopen("/home/kendrick/catkin_ws/src/position_data.txt","w");

#define DEGREE M_PI/180.0

using std::ofstream;


MobilePlanner::MobilePlanner(YAMLConfig yaml_config):
config_(yaml_config)
{
	
}
MobilePlanner::~MobilePlanner()
{       
}

void MobilePlanner::initializeData(task_assembly::plan_mobile_motion::Request &req) {

	// joint limit 
	// assert(nb_of_joints_ == config_.joint_limit_lower.size());
	// assert(nb_of_joints_ == config_.joint_limit_upper.size());
	cSpaceDimension = config_.position_limit_lower.size();

	position_limit_.lower_.resize(cSpaceDimension);
	position_limit_.upper_.resize(cSpaceDimension);

	for (std::vector<int>::size_type i=0;i<config_.position_limit_lower.size();i++){
		position_limit_.lower_(i) = config_.position_limit_lower[i];
		position_limit_.upper_(i) = config_.position_limit_upper[i];
	}
	
	nh_rrt_.goal_bias_upper.resize(cSpaceDimension);
	nh_rrt_.goal_bias_lower.resize(cSpaceDimension);

	// current base position
	mobile_state_.qInit_.resize(cSpaceDimension);
	mobile_state_.qInit_(0) = req.current_mobile_state.x;
	mobile_state_.qInit_(1) = req.current_mobile_state.y;
	mobile_state_.qInit_(2) = req.current_mobile_state.theta;

	mobile_state_.qGoal_.resize(cSpaceDimension);
	mobile_state_.qGoal_(0) = req.target_mobile_pose.x;
	mobile_state_.qGoal_(1) = req.target_mobile_pose.y;
	mobile_state_.qGoal_(2) = req.target_mobile_pose.theta;	

	// Obstacles 
	nh_rrt_.obs_num = req.Obstacles2D.size();
	// rrt_.box_num_obs = req.Obstacles.size();
	for (int i=0;i<nh_rrt_.obs_num;i++)
	{
		nh_rrt_.obs_info_[i].pos(0) = req.Obstacles2D[i].x.data;
		nh_rrt_.obs_info_[i].pos(1) = req.Obstacles2D[i].y.data;
		nh_rrt_.obs_info_[i].radius = req.Obstacles2D[i].radius.data;
	}
	nh_rrt_.mobile_length = config_.mobile_length;
	nh_rrt_.mobile_width = config_.mobile_width;


	maxVelocity.resize(3);
	maxVelocity.setZero();
	maxAcceleration.resize(3);
	maxAcceleration.setZero();
	for (size_t i = 0; i < 2; i++)
	{
		maxAcceleration(i) = 10.0;
		maxVelocity(i) = 0.3;
	}
	maxAcceleration(2) = 10.0;
	maxVelocity(2) = 0.5; // 1.85
	wayPoints.clear();
	playTime_ = 0.0;


}

task_assembly::MobileTrajectory MobilePlanner::compute() {

	// Plan the trajectory
	// output_trajectory_.points.reserve(1);
	ROS_INFO("1");
	if (setupAndSolveNHRRRT(mobile_state_.qInit_, mobile_state_.qGoal_, joint_target2_))
	{
		//cout << joint_target2_.size() << endl;
		output_trajectory_.points.clear();
		for (int i = 0; i < joint_target2_.rows(); i++)
		{
			wayPoints.push_back(joint_target2_.row(i));
		}

		trajectory_generator_ = new Trajectory(Path(wayPoints, 0.1), maxVelocity, maxAcceleration);
		duration_ = trajectory_generator_->getDuration();

		geometry_msgs::Pose2D pose1;

		Vector2d tangent_;
		double arc_;

		while (playTime_ / 10.0 < duration_)
		{
			
			pose1.x = trajectory_generator_->getPosition(playTime_ / 10.0)[0];
			pose1.y = trajectory_generator_->getPosition(playTime_ / 10.0)[1];
			pose1.theta = trajectory_generator_->getPosition(playTime_ / 10.0)[2];

			cout << "x \t" << pose1.x << "\t" << "y \t" << pose1.y <<"\t" << "theta \t" << 	pose1.theta<< endl;

			output_trajectory_.points.push_back(pose1);

			playTime_ ++; 
		}
	}
	else
	{
		output_trajectory_.points.clear();
	}

	return output_trajectory_;
}



bool MobilePlanner::setupAndSolveNHRRRT(VectorXd q_start, VectorXd q_goal, MatrixXd& joint_target)
{

	nh_rrt_.pos_lower_limit = position_limit_.lower_;
	nh_rrt_.pos_upper_limit = position_limit_.upper_;

	nh_rrt_.qinit = q_start;
	nh_rrt_.qgoal = q_goal;

	nh_rrt_.goal_bias_upper(0) = q_goal(0) + 0.2;
	nh_rrt_.goal_bias_upper(1) = q_goal(1) + 0.2;
	nh_rrt_.goal_bias_upper(2) = q_goal(2) + 0.05;

	nh_rrt_.goal_bias_lower(0) = q_goal(0) - 0.2;
	nh_rrt_.goal_bias_lower(1) = q_goal(1) - 0.2;
	nh_rrt_.goal_bias_lower(2) = q_goal(2) - 0.05;

	ofstream outFile("path_result.txt", ios::out); // open file for "writing"
	
	if (nh_rrt_.solveNHRRT(outFile))
	{
		outFile.close();
		MatrixXd joint_temp(1000, 3);

		ifstream inFile2("path_result.txt"); // open file for "writing"

		int size = 0;
		std::vector<std::string> parameters;
		char inputString[10000];
		while (!inFile2.eof())
		{ // eof : end of file
			inFile2.getline(inputString, 10000);
			boost::split(parameters, inputString, boost::is_any_of(","));
			if (parameters.size() == 3)
			{
				for (int j = 0; j < parameters.size(); j++)
					joint_temp(size, j) = atof(parameters[j].c_str()); // c_str() : string -> char* // atof : char* -> float
				size++;
			}
		}
		inFile2.close();
		joint_target = joint_temp.topLeftCorner(size, 3);
		return true;
	}
	else
	{
		return false;
	}
}


