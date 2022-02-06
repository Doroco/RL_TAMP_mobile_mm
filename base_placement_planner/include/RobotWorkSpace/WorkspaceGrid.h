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
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include "RobotWorkSpace.h"
#include <string>
#include <vector>
#include "WorkspaceRepresentation.h"

#include "task_assembly/GraspConfig.h"
#include "task_assembly/GraspConfigList.h"
#include "task_assembly/ObstacleBox2D.h"
// for visualization
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace RobotWorkSpace
{
    /*!
    * A 2D grid which represents a quality distribution (e.g. the reachability) at 2D positions w.r.t. one or multiple grasp(s).
    * Internally the inverse workspace data (@see WorkspaceRepresentation), which encodes the
    * transformation between robot's base and grasping position, is used.
    * This data is useful to quickly sample positions from where the probability that a grasp is reachable is high (see \func getRandomPos).
    */
    class WorkspaceGrid
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
        /*!
            Setup the 2D grid with given extends and discretization parameter.
        */
        WorkspaceGrid(float minX, float maxX, float minY, float maxY, float discretizeSize, float discretizeSizeRotation);
        ~WorkspaceGrid();

        //! returns entry of position in x/y plane (in world coords)

        int getEntry(float x, float y, float r);
        /*!
            Gets the corresponding entry of a world / global 2d position.
            \param x The x coordinate (in global coordinate system)
            \param y The y coordinate (in global coordinate system)
            \param storeEntry The corresponding entry is stored.
            \param storeGrasp A random grasp of corresponding cell is stored.
            \return True if x/y is inside this grid and an entry>0 was found, false otherwise.
        */
        bool getEntry(float x, float y, float r, int& storeEntry, task_assembly::GraspConfig& storeGrasp);
        bool getEntry(float x, float y, float r, int& nStoreEntry, std::vector<task_assembly::GraspConfig>& storeGrasps);

        //! returns entry of discretized square (x/y)
        int getCellEntry(int cellX, int cellY, int cellR);
        bool getCellEntry(int cellX, int cellY, int cellR, int& nStoreEntry, task_assembly::GraspConfig& storeGrasp);
        bool getCellEntry(int cellX, int cellY, int cellR, int& nStoreEntry, std::vector<task_assembly::GraspConfig>& storeGrasps);
        
        /*!
            sets the entry to value, if the current value is lower
        */
        void setEntry(float x, float y, float r, int value, task_assembly::GraspConfig grasp);
        void setCellEntry(int cellX, int cellY, int cellR, int value, task_assembly::GraspConfig pGrasp);


        int getMaxEntry();

        /*!
            This method sets the grid value to nValue and checks if the neighbors have a lower value and in case the value of the neighbors is set to nValue.
        */
        void setEntryCheckNeighbors(float x, float y, float r, int value, task_assembly::GraspConfig grasp);

        //! tries to find a random position with a entry >= minEntry
        bool getRandomPos(int minEntry, float& storeXGlobal, float& storeYGlobal, float& storeRGlobal, task_assembly::GraspConfig& storeGrasp, int maxLoops = 50, int* entries = NULL);
        bool getRandomPos(int minEntry, float& storeXGlobal, float& storeYGlobal, float& storeRGlobal, std::vector<task_assembly::GraspConfig>& storeGrasps, int maxLoops = 50, int* entries = NULL);

        /*!
            Clear all entries.
        */
        void reset();

        /*!
            Fill the grid with inverse reachability data generated from grasp g and object o.
        */
        bool fillGridData(WorkspaceRepresentationPtr ws, task_assembly::GraspConfig g);


        /*!
            Move the grid to (x,y), given in global coordinate system. Sets the new center.
        */
        void setGridPosition(float x, float y);

        /*!
            Get extends in global coord system.
        */
        void getExtends(float& storeMinX, float& storeMaxX, float& storeMinY, float& storeMaxY);
        inline visualization_msgs::MarkerArray getIRMVisulization(){return IRM;};

        /*!
            Number of cells in x and y
        */
        void getCells(int& storeCellsX, int& storeCellsY);


        float getDiscretizeSize() const;
        Eigen::Vector2f getMin() const;
        Eigen::Vector2f getMax() const;

        struct ObstacleInfo
        {
            Eigen::Vector2f minBound;
            Eigen::Vector2f maxBound;
        };
        std::vector<ObstacleInfo> obsBounds;
        int obsnum;

        void setObs(task_assembly::ObstacleBox2D obs, int idx);
        void setObsnum(int num)
        {
            obsnum = num;
            obsBounds.reserve(num);
        };
        /**
         * @brief Creates the intersection between multiple grids into one grid by considering for each x,y position the worst values of all grids.
         * @param reachGrids grids for different grasp poses or object poses
         * @return new, merged grid
         */
        static WorkspaceGridPtr MergeWorkspaceGrids(const std::vector<WorkspaceGridPtr>& reachGrids);
    protected:
        /*!
            Adds data stored in reachability transformations. This data defines transformations from robot base system to grasping pose,
            so when defining a grasping pose, the inverse reachability can be represented by this grid
            Therefor the "world coordinates" of the inverse reachability distributions are computed by T_grasp * ReachTransformation^-1
        */
        void setEntries(std::vector<WorkspaceRepresentation::WorkspaceCut2DTransformationPtr>& wsData, const Eigen::Matrix4f& graspGlobal, task_assembly::GraspConfig grasp,const Eigen::Matrix4f& curBaseRot, float curR);

        inline int getDataPos(int x, int y, int r)
        {
            return (x * gridSizeY * gridSizeRotation + y * gridSizeRotation + r);
        };
        
        float minX, maxX; // in global coord system
        float minY, maxY; // in global coord system
        float minR, maxR;     // in global coord system
        float discretizeSize;
        float discretizeStepRotation;
        int gridSizeX, gridSizeY;
        int gridSizeRotation;

        float gridExtendX, gridExtendY, gridExtendRotation;
        ros::Time MakerDataInstance;
        int MarkerID;
        int* data;                              // stores the quality values
        std::vector<task_assembly::GraspConfig>* graspLink;       // points to list of all reachable grasps
        
        /// for Visualization 
        visualization_msgs::MarkerArray IRM;
    };

}
