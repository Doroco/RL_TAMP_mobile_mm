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
* @author     Peter Kaiser, Nikolaus Vahrenkamp
* @copyright  2011 Peter Kaiser, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include "RobotWorkSpace.h"
#include "WorkspaceRepresentation.h"

#include "task_assembly/GraspConfig.h"
#include "task_assembly/GraspConfigList.h"


namespace RobotWorkSpace
{

    /*!
            This class represents an approximation of the reachability distribution of a kinematic chain (e.g. an arm).
            Consists of voxels covering the 6D space for position (XYZ) and orientation (Tait�Bryan angles, EulerXYZ, static frame).
            Each voxel holds a counter with the number of successful IK solver calls,
            representing the approximated probability that an IK solver call can be successfully answered.
            The discretized reachability data can be written to and loaded from binary files.

            The reachability is linked to a base coordinate system which is defined by a robot joint.
            This base system is used to align the data when the robot is moving.
            I.E. think of an arm of a humanoid where the reachability is linked to the shoulder.
            When the torso moves, the reachability also changes it's position according to the position of the shoulder.
    */
    class Reachability : public WorkspaceRepresentation, public std::enable_shared_from_this<Reachability>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Reachability(ros::NodeHandle & _nh,double _hz,RobotWorkSpace::YAMLConfig robotConfig, KDL::JntArray nominal, PoseQualityManipulability::ManipulabilityIndexType i = PoseQualityManipulability::eMinMaxRatio);

        /*!
            Returns true, if the corresponding reachability entry is non zero.
        */
        bool isReachable(const Eigen::Matrix4f& globalPose);

        /*!
            Returns all reachable grasps that can be applied at the current position of object.
        */
        task_assembly::GraspConfigListPtr getReachableGrasps(task_assembly::GraspConfigListPtr grasps);


        //! returns a random pose that is covered by the workspace data.
        Eigen::Matrix4f sampleReachablePose();

        /*!
            Creates a deep copy of this data structure. A ReachabilityPtr is returned.
        */
        PoseQualityMeasurementPtr clone() override;

        ReachabilityPtr clone_ws();

    protected:

    };


} // namespace VirtualRobot