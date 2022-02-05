#include "../include/RobotWorkSpace/Reachability.h"
#include "../include/RobotWorkSpace/Exception.h"
#include <fstream>
#include <cmath>
#include <cfloat>
#include <climits>

namespace RobotWorkSpace
{

    Reachability::Reachability(ros::NodeHandle & _nh,double _hz,RobotWorkSpace::YAMLConfig robotConfig, KDL::JntArray nominal, PoseQualityManipulability::ManipulabilityIndexType i)
     : WorkspaceRepresentation(_nh,_hz,robotConfig,nominal,i)
    {
        type = "Reachability";
    }


    bool Reachability::isReachable(const Eigen::Matrix4f& globalPose)
    {
        return isCovered(globalPose);
    }

    task_assembly::GraspConfigListPtr Reachability::getReachableGrasps(task_assembly::GraspConfigListPtr grasps)
    {
        task_assembly::GraspConfigListPtr res;
        res->header = grasps->header;

        for (unsigned int i = 0; i < grasps->grasps.size(); i++)
        {
            Eigen::Matrix4f m = getGlobalEEpose(grasps->grasps.at(i));

            if (isReachable(m))
            {
                res->grasps.push_back(grasps->grasps.at(i));
            }
        }

        return res;
    }

    Eigen::Matrix4f Reachability::sampleReachablePose()
    {
        return sampleCoveredPose();
    }

    RobotWorkSpace::PoseQualityMeasurementPtr Reachability::clone()
    {
        RobotWorkSpace::ReachabilityPtr res(new Reachability(this->nh_,this->hz_,this->config_,this->nominal_, this->manipulabilityType));
        res->setOrientationType(this->orientationType);
        res->versionMajor = this->versionMajor;
        res->versionMinor = this->versionMinor;
        res->type = this->type;

        res->buildUpLoops = this->buildUpLoops;
        res->collisionConfigs = this->collisionConfigs;
        res->discretizeStepTranslation = this->discretizeStepTranslation;
        res->discretizeStepRotation = this->discretizeStepRotation;
        memcpy(res->minBounds, this->minBounds, sizeof(float) * 6);
        memcpy(res->maxBounds, this->maxBounds, sizeof(float) * 6);
        memcpy(res->numVoxels, this->numVoxels, sizeof(float) * 6);
        memcpy(res->achievedMinValues, this->achievedMinValues, sizeof(float) * 6);
        memcpy(res->achievedMaxValues, this->achievedMaxValues, sizeof(float) * 6);
        memcpy(res->spaceSize, this->spaceSize, sizeof(float) * 6);

        res->adjustOnOverflow = this->adjustOnOverflow;
        res->data.reset(this->data->clone());

        return res;
    }

    ReachabilityPtr Reachability::clone_ws()
    {
        RobotWorkSpace::ReachabilityPtr res(new Reachability(this->nh_,this->hz_,this->config_,this->nominal_, this->manipulabilityType));
        res->setOrientationType(this->orientationType);
        res->versionMajor = this->versionMajor;
        res->versionMinor = this->versionMinor;
        res->type = this->type;

        res->buildUpLoops = this->buildUpLoops;
        res->collisionConfigs = this->collisionConfigs;
        res->discretizeStepTranslation = this->discretizeStepTranslation;
        res->discretizeStepRotation = this->discretizeStepRotation;
        memcpy(res->minBounds, this->minBounds, sizeof(float) * 6);
        memcpy(res->maxBounds, this->maxBounds, sizeof(float) * 6);
        memcpy(res->numVoxels, this->numVoxels, sizeof(float) * 6);
        memcpy(res->achievedMinValues, this->achievedMinValues, sizeof(float) * 6);
        memcpy(res->achievedMaxValues, this->achievedMaxValues, sizeof(float) * 6);
        memcpy(res->spaceSize, this->spaceSize, sizeof(float) * 6);

        res->file_format_ = this->file_format_;
        res->file_Path_ = this->file_Path_;
        res->LocalBase = this->LocalBase;
        res->VisionBase = this->VisionBase;

        res->adjustOnOverflow = this->adjustOnOverflow;
        res->data.reset(this->data->clone());

        return res;
    }

} // namespace VirtualRobot
