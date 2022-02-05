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

#include "Exception.h"
#include "RobotWorkSpace.h"
#include "WorkspaceData.h"
#include "WorkspaceDataArray.h"
#include "PoseQualityExtendedManipulability.h"
#include "../MathTools.h"
#include "../FileIO.h"

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "task_assembly/GraspConfig.h"
#include "task_assembly/GraspConfigList.h"

namespace RobotWorkSpace
{
    // 추상함수이다. 구현은 다른애한테 맡길께~
    /*!
            This class represents a voxelized approximation of the workspace that is covered by a kinematic chain of a robot.
            The voxel grid covers the 6d Cartesian space: xyz translations (mm) and Tait-Bryan angles (eulerXYZ, fixed frame, extrinsic) orientations.
            Older versions (<=2.5) used RPY (intrinsic) for storing orientations, but it turned out that this representation is not suitable for discretization.

            Each voxels holds a counter(uchar) that holds information, e.g. about reachability. 복셀은 리처빌리티에 대한 정보를 담고 있다.
            The discretized data can be written to and loaded from binary files.

            The data is linked to a base coordinate system which is defined by a robot joint.
            This base system is used to align the data when the robot is moving.

            I.E. think of an arm of a humanoid where the workspace representation is linked to the shoulder.
            When the torso moves, the data representation also changes it's position according to the position of the shoulder.
            
            *********** 움직일때 투게더 움직인다.
    */

    class WorkspaceRepresentation : public PoseQualityExtendedManipulability, public std::enable_shared_from_this<WorkspaceRepresentation>
    {    
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef int32_t ioIntTypeWrite;
        typedef int32_t ioIntTypeRead;

        enum eOrientationType
        {
            RPY,
            EulerXYZ,           // intrinsic
            EulerXYZExtrinsic,  // fixed frame (standard)
            Hopf                // hopf coordinates
        };

        struct VolumeInfo
        {
            unsigned int voxelCount3D;              // overall number of 3d voxels
            unsigned int filledVoxelCount3D;        // number of filled voxels (nr of 3d position voxels for which at least one 6D cell exists that is filled)
            unsigned int borderVoxelCount3D;        // number of border voxels (3d voxels with at least 2 emoty neighbors in 3d)
            float volumeVoxel3D;                    // volume of one voxel (m^3)
            float volumeFilledVoxels3D;             // accumulated volume of all filled voxels (m^3)
            float volume3D;                         // (filledVoxelCount3D - 0.5*borderVoxelCount3D) * volumeVoxel3D
        };

        WorkspaceRepresentation(ros::NodeHandle & _nh,double _hz,RobotWorkSpace::YAMLConfig robotConfig, KDL::JntArray nominal, PoseQualityManipulability::ManipulabilityIndexType i = PoseQualityManipulability::eMinMaxRatio);
        
        /*!
            Reset all data.
        */
        void reset();

        //! Uncompress the data
        void uncompressData(const unsigned char* source, int size, unsigned char* dest);
        //! Compress the data
        unsigned char* compressData(const unsigned char* source, int size, int& compressedSize);

        /*!
            Load the workspace data from a binary file.
            Exceptions are thrown on case errors are detected.
        */
        void load(const std::string& filename);

        /*!
            Store the workspace data to a binary file.
            Exceptions are thrown on case errors are detected.
        */
        void save(const std::string& filename);

        /*!
            좌표계 변환 유틸들 !!!!!!!!
        */
    
        void setLocalBasepose(Eigen::Matrix4f& p);
        void setVisionBasepose(Eigen::Matrix4f& p);
        void toLocal(Eigen::Matrix4f& p) const;
        void toGlobal(Eigen::Matrix4f& p) const;
        void rotateLocalBase(Eigen::Matrix4f p);
        void toLocalVec(Eigen::Vector3f& positionGlobal) const;
        void toGlobalVec(Eigen::Vector3f& positionLocal) const;

        // Local 기준 to End Effector의 위치를 선정한다.
        Eigen::Matrix4f getTCPpose();
        Eigen::Matrix4f getToLocalTransformation() const;
        Eigen::Matrix4f getToGlobalTransformation() const;

        //! Convert a 4x4 matrix to a pos + ori vector 6열의 벡터형식으로 바꾸어준다.
        void matrix2Vector(const Eigen::Matrix4f& m, float x[6]) const;
        void vector2Matrix(const float x[6], Eigen::Matrix4f& m) const;
        void vector2Matrix(const Eigen::Vector3f& pos, const Eigen::Vector3f& rot, Eigen::Matrix4f& m) const;

        /*!
            Initialize and reset all data.
            \param discretizeStepTranslation The extend of a voxel dimension in translational dimensions (x,y,z) [mm]
            \param discretizeStepRotation The extend of a voxel dimension in rotational dimensions (roll, pitch, yaw) [rad]
            \param minBounds The minimum workspace poses (x,y,z,ro,pi,ya) given in baseNode's coordinate system [mm and rad]
            \param maxBounds The maximum workspace poses (x,y,z,ro,pi,ya) given in baseNode's coordinate system [mm and rad]
            \param adjustOnOverflow If set, the 8bit data is divided by 2 when one voxel entry exceeds 255. Otherwise the entries remain at 255.
        */
        virtual void initialize(float discretizeStepTranslation,
                                float discretizeStepRotation,
                                float minBounds[6],
                                float maxBounds[6],
                                bool adjustOnOverflow = true);
        /*!
            The bounding box in global frame.
            \param achievedValues If not set the bounding box as defined on construction is returned. If set the min/max achieved positions are used.
            \return The object oriented bounding box
        */
        MathTools::OOBB getOOBB(bool achievedValues = false) const;
        float getDiscretizeParameterTranslation();
        float getDiscretizeParameterRotation();
        unsigned char getEntry(const Eigen::Matrix4f& globalPose) const;
        unsigned char getVoxelEntry(unsigned int a, unsigned int b, unsigned int c, unsigned int d, unsigned int e, unsigned int f) const;

        //! Returns the maximum entry of a voxel.
        int getMaxEntry() const;
        /*!
            Searches all angle entries (x3,x4,x5) for maximum entry.
            (x0,x1,x2) is the voxel position.
        */
        int getMaxEntry(int x0, int x1, int x2) const;

        /*!
            Returns the maximum workspace entry that can be achieved by an arbitrary orientation at the given position.
        */
        int getMaxEntry(const Eigen::Vector3f& position_global) const;

        //! returns the extends of a voxel at corresponding dimension. discre사이즈
        float getVoxelSize(int dim) const;

        int getNumVoxels(int dim) const;
        float getMinBound(int dim) const;
        float getMaxBound(int dim) const;

        /*!
            Get the corresponding voxel coordinates.
            If false is returned the position is outside the covered workspace.
        */
        bool getVoxelFromPosition(float x[3], unsigned int v[3]) const;
        bool getVoxelFromPosition(const Eigen::Matrix4f& globalPose, unsigned int v[3]) const;

        /*!
            Sets entry that corresponds to TCP pose to e, if current entry is lower than e.
            Therefore the corresponding voxel of the current TCP pose is determined and its entry is adjusted.
        */
        bool getPoseFromVoxel(unsigned int x[], float v[]) const;
        /*!
            Computes center of corresponding voxel in global coord system.
        */
        Eigen::Matrix4f getPoseFromVoxel(unsigned int v[6], bool transformToGlobalPose = true);
        Eigen::Matrix4f getPoseFromVoxel(float v[6], bool transformToGlobalPose = true);

        bool getVoxelFromPose(const Eigen::Matrix4f& globalPose, unsigned int v[6]) const;
        bool getVoxelFromPose(float x[6], unsigned int v[6]) const;
        WorkspaceDataPtr getData();
        /////////////////////////////////////////////// Write Voxel Data /////////////////////////////////////////////////////////////////////////////
        /*!
            Returns true, if the corresponding voxel entry is not zero.
        */
        bool isCovered(const Eigen::Matrix4f& globalPose);

        /*!
            Returns true, if voxel entry is not zero.
        */
        bool isCovered(unsigned int v[6]);
        void setVoxelEntry(unsigned int v[6], unsigned char e);
        void setEntry(const Eigen::Matrix4f& poseGlobal, unsigned char e);
        void setEntryCheckNeighbors(const Eigen::Matrix4f& poseGlobal, unsigned char e, unsigned int neighborVoxels);
        void setOrientationType(eOrientationType t);
        VolumeInfo computeVolumeInformation();

        //////////////////////////////////////// Check data in Voxel space /////////////////////////////////////////////////////////////////
        /*!
            Sums all angle (x3,x4,x5) entries for the given position.
        */
        int sumAngleReachabilities(int x0, int x1, int x2) const;
        int avgAngleReachabilities(int x0, int x1, int x2) const;
        /*!
            Returns the maximum that can be achieved by calling sumAngleReachabilities()
        */
        int getMaxSummedAngleReachablity();
        bool hasEntry(unsigned int x, unsigned int y, unsigned int z);

        /*!
            Estimate a parameter setup for the given RNS by randomly set configurations and check for achieved workspace extends. The results are slightly scaled.
            \param ndoeSet
            \param steps How many loops should be performed to estimate the result. Chose a value >= 1000.
            \param storeMinBounds Workspace extend from min
            \param storeMaxBounds Workspace extend to max
            \return True on success.
        */
        bool checkForParameters(float steps,
                                        float storeMinBounds[6],
                                        float storeMaxBounds[6]);

        //// //////////////////////////////////////////Configuration Uitls///////////////////////////////////////////////////////////////////////////////
        /*!
            Sets entry that corresponds to TCP pose to e, if current entry is lower than e.
            Therefore the corresponding voxel of the current TCP pose is determined and its entry is adjusted.
        */
        void setCurrentTCPPoseEntryIfLower(unsigned char e);

        /*!
            Sets entry that corresponds to TCP pose to e.
            Therefore the corresponding voxel of the current TCP pose is determined and its entry is set.
        */
        void setCurrentTCPPoseEntry(unsigned char e);
        /*!
            Generate a random configuration for the robot node set. This configuration is within the joint limits of the current robot node set.
            \param checkForSelfCollisions Build a collision-free configuration. If true, random configs are generated until one is collision-free.
        */
        bool setRobotNodesToRandomConfig(bool checkForSelfCollisions = true);

        /*
            Add pose to data.
            This means that the entry of the corresponding WorkspaceData voxel is increased by 1.
        */
        void addPose(const Eigen::Matrix4f& globalPose);
        /*!
            Compute the quality of the current pose and add entry to voxel data
            (if is larger than the existing entry).
        */
        void addCurrentTCPPose();
        /*!
            Append a number of random TCP poses to workspace Data.
            For bigger sampling rates (loops > 50.000) you should consider this method's multithreaded pendant.
            \param loops Number of poses that should be appended
            \param checkForSelfCollisions Build a collision-free configuration. If true, random configs are generated until one is collision-free.
        */
        void addRandomTCPPoses(unsigned int loops, bool checkForSelfCollisions = true);
        /*!
            Appends a number of random TCP poses to workspace Data (multithreaded).
            This method is blocking, i.e. it returns as soon as all threads are done.
            \param loops Number of poses that should be appended
            \param numThreads number of worker threads used behind the scenes to append random TCP poses to workspace data.
            \param checkForSelfCollisions Build a collision-free configuration. If true, random configs are generated until one is collision-free.
        */
        // void addRandomTCPPoses(unsigned int loops, unsigned int numThreads, bool checkForSelfCollisions = true);

        /*!
            Clears all data
        */
        void clear();
        /*!
            Creates a deep copy of this data structure. Derived classes may overwrite this method that provides a generic interface for cloning.
        */
        bool getAdjustOnOverflow()
        {
            return adjustOnOverflow;
        }
        PoseQualityMeasurementPtr clone() override;
        WorkspaceRepresentationPtr clone_ws();
        void binarize();
        //! returns a random pose that is covered by the workspace data
        Eigen::Matrix4f sampleCoveredPose();
        /// Global grasp pose to Local
        Eigen::Matrix4f getGlobalEEpose(task_assembly::GraspConfig grasp);
        //////////////////////////////////////////////// 2D workspace Representation ///////////////////////////////////////////////////////////
        /*!
            2D data that represents a cut through the workspace representation.
            Usually the z component and the orientation of the 6D workspace data is fixed and the x and y components are iterated in order to store the resulting workspace entries.
        */
        struct WorkspaceCut2D
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Eigen::Matrix4f referenceGlobalPose;
            Eigen::MatrixXi entries;
            float minBounds[2]; // in global coord system
            float maxBounds[2]; // in global coord system
        };
        typedef std::shared_ptr<WorkspaceCut2D> WorkspaceCut2DPtr;

        struct WorkspaceCut2DTransformation
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            int value;
            Eigen::Matrix4f transformation;
        };
        typedef std::shared_ptr<WorkspaceCut2DTransformation> WorkspaceCut2DTransformationPtr;

        /*!
            Create a horizontal cut through this workspace data. Therefore, the z component and the orientation of the reference pose (in global coordinate system) is used.
            Then the x and y components are iterated and the corresponding entires are used to fill the 2d grid.
        */
        WorkspaceCut2DPtr createCut(const Eigen::Matrix4f& referencePose, float cellSize, bool sumAngles) const;

        /*!
        * \brief createCut Create a cut at a specific height (assuming z is upwards).
        * \param heightPercent Value in [0,1]
        * \param cellSize The discretization step size of the result
        * \return
        */
        WorkspaceCut2DPtr createCut(float heightPercent, float cellSize, bool sumAngles) const;

        /*!
            Build all transformations from referenceNode to cutXY data.h Only entries>0 are considered.
            If referenceNode is set, the transformations are given in the corresponding coordinate system.
        */
        std::vector<WorkspaceCut2DTransformationPtr> createCutTransformations(WorkspaceCut2DPtr cutXY);

        /*!
            Computes the axis aligned bounding box of this object in global coordinate system.
            Note, that the bbox changes when the robot moves. (로봇의 글로벌 위치를 구합니다)
        */
        bool getWorkspaceExtends(Eigen::Vector3f& storeMinBBox, Eigen::Vector3f& storeMaxBBox) const;

        ////// inline functions
        // ros::NodeHandle getRosHandle(){return nh_;};
        //////////////// 데이터 형식 유틸들 //////////////////////////////////////////////////////////////////////////////////////
    protected:
        // Number of processed random configs
        int buildUpLoops;

        // Number of reported collisions
        int collisionConfigs;

        // Tells how to discretize the workspace data
        float discretizeStepTranslation;
        float discretizeStepRotation;
        float minBounds[6];
        float maxBounds[6];

        // Number of voxels in each dimension(space만큼의 길이를 discrete step으로 진행하여서 each dimension 마다 얼마나 있는지)
        int numVoxels[6];

        // The smallest/greatest tcp workspace pose value reached in each dimension
        float achievedMinValues[6];
        float achievedMaxValues[6];

        // workspace extend in each dimension(선분의 길이 )
        float spaceSize[6];

        WorkspaceDataPtr data;

        bool adjustOnOverflow;

        std::string type;

        int versionMajor;
        int versionMinor;

        //! Specifies how the rotation part (x[3],x[4],x[5]) of an 6D voxel entry is encoded.
        eOrientationType orientationType;

        Eigen::Matrix4f LocalBase;
        Eigen::Matrix4f VisionBase;
        
        std::string file_format_;
        std::string file_Path_;
        ros::NodeHandle nh_;
        double hz_;

        float RealMax;
    };
}