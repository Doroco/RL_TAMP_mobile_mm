/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include <string.h>
#include <vector>
#include <memory>
#include <assert.h>

// motion planner Header
#include "../include/ArmPlanner/RRTFunction.h"

// workspace Header
#include "RobotWorkSpace.h"

// Utils
#include "../include/Utils.h"

// IK solver (track-ik)
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>


// ROS includes
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/node_handle.h>

//extern Libary
#include <Eigen/Core>

// Model type
#include <urdf/model.h>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "YamlConfig.h"

// output Msgs type
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"

// Utils
#include "../Fwd.h"


#include "../Random.h"

namespace RobotWorkSpace
{
    /**
    * \class PoseQualityMeasurement
    *
    * An interface definition for quality measurements of poses.
    * PoseQualityMeasurement 인터페이스입니다.
    */
    class PoseQualityMeasurement
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        PoseQualityMeasurement(RobotWorkSpace::YAMLConfig robotConfig, KDL::JntArray nominal);
        ~PoseQualityMeasurement();

        // Manipulator joint states
        struct jointState {
            Eigen::VectorXd qInit_;
            Eigen::VectorXd qGoal_;
        };
        // Mobile base joint states
        struct mobileState {
            Eigen::VectorXd qInit_;
            Eigen::Matrix3d rotInit_;
        };
        struct jointLimit {
            Eigen::VectorXd lower_;
            Eigen::VectorXd upper_;
            Eigen::VectorXd lower_rad_;
            Eigen::VectorXd upper_rad_;
        };
        /*!
            The main method for determining the pose quality.
            The current configuration of the corresponding RNS is analyzed and the quality is returned.
            See derived classes for details.
        */
        virtual float getPoseQuality();

        /*!
            The quality is determined for a given Cartesian direction.
            The current configuration of the corresponding RNS is analyzed and the quality is returned.
            See derived classes for details.
            \param direction A 3d or 6d vector with the Cartesian direction to investigate.
        */
        virtual float getPoseQuality(const Eigen::VectorXf& direction);

        // 디버깅하실래요??
        void setVerbose(bool v);

        // Arm Kinematics Uitility 나중에 코드 리팩토링할때 다른 해더로 분해를 합시다!///////////////////////////////
        void initModel(std::string URDFPram);
        bool solveFK(Eigen::Matrix4f &EE_pose);
        bool solveIK(RobotWorkSpace::Transform3d _target_ee_pose);
        Eigen::Matrix4f Frame2Eigen(KDL::Frame &frame);

        ////////// Planning Utils ////////////////////////////////////////////////////////////////////////
        bool setupRRT(Eigen::VectorXd q_start, Eigen::VectorXd q_goal, Eigen::MatrixXd& joint_target);
	    bool setupCRRT(Eigen::VectorXd q_start, Eigen::VectorXd q_goal, Eigen::MatrixXd& joint_target);
    	bool solveRRTwithMultipleGoals(Eigen::VectorXd q_start, std::vector<VectorXd> q_goal_set, Eigen::MatrixXd& joint_target);
	    bool solveCRRTwithMultipleGoals(Eigen::VectorXd q_start, std::vector<VectorXd> q_goal_set, Eigen::MatrixXd& joint_target);
        bool computeArmMtion();
        /*!
            Returns the RobotNodeSte that is used for computing the manipulability.
        */
        RobotWorkSpace::YAMLConfig getRNS();
        Eigen::MatrixXf getJacobian();
        Eigen::MatrixXf getJacobianGlobal();
        //! A string that identifies the type of pose quality measure.

        std::string getName();

        void setRandomConfig();

        //! Indicates if joint limits are considered.
        virtual bool consideringJointLimits();

        /*!
            Consider obstacles. Here, the shortest distance on the surface of the RNS to an obstacle is set (in TCP coords).
            This obstacle vector may be considered by any further calculations (depending on the implementation).
        */
        virtual void setObstacleDistanceVector(const Eigen::Vector3f& directionSurfaceToObstance);
        virtual void disableObstacleDistance();

        virtual PoseQualityMeasurementPtr clone()
        {
            PoseQualityMeasurementPtr m(new PoseQualityMeasurement(this->config_,this->nominal_));
            return m;
        };

    protected:
        //joint_state_.qGoal_. --> TCP 받을떄 하기

        // Dynamic Model
        std::string name;
        std::string urdf_param_;
        RobotWorkSpace::YAMLConfig config_;
        RigidBodyDynamics::Model rbdl_model_;

        ArmPlanner::Robotmodel rrt_model_;  
        ArmPlanner::RRT rrt_;

        // Joint State
        KDL::JntArray nominal_;
        jointState joint_state_;
        jointLimit joint_limit_;
        int nb_of_joints_;

        //Arm State
        unsigned int end_effector_id_;
	    unsigned int arm_base_frame_id_;
        Eigen::Vector3d end_effector_com_;
	    Eigen::Vector3d arm_base_frame_pos_;
        std::vector<int> body_id_collision_;
	    std::vector<Vector3d> body_com_position_;

        //IK params Model
        KDL::JntArray IK_lb, IK_ub;
        KDL::Chain IK_chain;
        KDL::Tree IK_tree;
        urdf::Model IK_robot_model;

        // TCP pose
        RobotWorkSpace::Transform3d target_pose_;
        Eigen::Matrix4f TCP_pose;

        // consider Object??
        bool considerObstacle;
        Eigen::Vector3f obstacleDir;

        // Trajectory library // 
        trajectory_msgs::JointTrajectory arm_trajectory_point_;
        trajectory_msgs::JointTrajectory output_arm_trajectory_;
        bool interpolate_path_;

        double duration_;
        Eigen::VectorXd maxAcceleration;
        Eigen::VectorXd maxVelocity;
        std::list<VectorXd> wayPoints;


        // Other Utils 
        bool constrain_pose_;
        double playTime_ ;

        bool verbose;
    };

}

