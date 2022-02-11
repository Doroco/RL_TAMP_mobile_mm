
#include "../include/RobotWorkSpace/WorkspaceGrid.h"
#include "../include/RobotWorkSpace/Exception.h"
#include "Utils.h"
#include <iostream>
#include <algorithm>
using namespace std;

#define MIN_VALUES_STORE_GRASPS 30

namespace RobotWorkSpace
{

    WorkspaceGrid::WorkspaceGrid(float fMinX, float fMaxX, float fMinY, float fMaxY, float fDiscretizeSize, float fDiscretizeSizeRotation)
    {

        MakerDataInstance = ros::Time::now();
        MarkerID = 0;
        minX = fMinX;
        maxX = fMaxX;
        minY = fMinY;
        maxY = fMaxY;

        minR = 0.0;
        maxR = 2.0 * M_PI;

        // minR = -M_PI;
        // maxR = M_PI;

        discretizeSize = fDiscretizeSize;
        discretizeStepRotation = fDiscretizeSizeRotation;
        data = nullptr;
        obsnum = 0;
        
        if (fMinX >= fMaxX || fMinY >= fMaxY)
        {
            THROW_VR_EXCEPTION("ERROR min >= max");
        }

        gridExtendX = maxX - minX;
        gridExtendY = maxY - minY;
        gridExtendRotation = maxR - minR;
        gridSizeX = (int)((maxX - minX) / discretizeSize) + 1;
        gridSizeY = (int)((maxY - minY) / discretizeSize) + 1;
        gridSizeRotation = (int)((maxR - minR) / discretizeStepRotation) + 1;


        if (gridSizeY <= 0 || gridSizeX <= 0 || gridExtendX <= 0 || gridExtendY <= 0 || gridExtendRotation <= 0 || gridSizeRotation <= 0)
        {
            THROW_VR_EXCEPTION("ERROR negative grid size");
        }

        VR_INFO << ": creating grid with " << gridSizeX << "x" << gridSizeY << "x" << gridSizeRotation << " = " << gridSizeX* gridSizeY*gridSizeRotation << " entries" << std::endl;
        data = new int[gridSizeX * gridSizeY * gridSizeRotation];
        graspLink = new std::vector<task_assembly::GraspConfig>[gridSizeX * gridSizeY * gridSizeRotation];
        memset(data, 0, sizeof(int)*gridSizeX * gridSizeY * gridSizeRotation);
    }

    WorkspaceGrid::~WorkspaceGrid()
    {
        delete []data;
        delete []graspLink;
    }

    void WorkspaceGrid::reset()
    {
        delete []graspLink;
        graspLink = new std::vector<task_assembly::GraspConfig>[gridSizeX * gridSizeY * gridSizeRotation];
        memset(data, 0, sizeof(int)*gridSizeX * gridSizeY * gridSizeRotation);
    }

    int WorkspaceGrid::getEntry(float x, float y, float r)
    {
        if (!data)
        {
            return 0;
        }

        int nPosX = (int)(((x - minX) / gridExtendX) * gridSizeX);
        int nPosY = (int)(((y - minY) / gridExtendY) * gridSizeY);
        int nPosR = (int)(((r - minR) / gridExtendRotation) * gridSizeRotation);

        if (nPosX < 0 || nPosY < 0 || nPosR < 0 || nPosX >= gridSizeX || nPosY >= gridSizeY || nPosR >= gridSizeRotation)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << fX << "," << fY << std::endl;
            return 0;
        }

        return data[getDataPos(nPosX, nPosY, nPosR)];
    }

    bool WorkspaceGrid::getEntry(float x, float y, float r, int& storeEntry, task_assembly::GraspConfig& storeGrasp)
    {
        if (!data)
        {
            return false;
        }

        int nPosX = (int)(((x - minX) / gridExtendX) * gridSizeX);
        int nPosY = (int)(((y - minY) / gridExtendY) * gridSizeY);
        int nPosR = (int)(((r - minR) / gridExtendRotation) * gridSizeRotation);
        
        if (nPosX < 0 || nPosY < 0 || nPosR < 0 || nPosX >= gridSizeX || nPosY >= gridSizeY || nPosR >= gridSizeRotation)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << fX << "," << fY << std::endl;
            return false;
        }

        storeEntry = data[getDataPos(nPosX, nPosY, nPosR)];
        size_t nLinks = graspLink[getDataPos(nPosX, nPosY, nPosR)].size();

        if (nLinks > 0)
        {
            storeGrasp = graspLink[getDataPos(nPosX, nPosY, nPosR)][rand() % nLinks];    // get random grasp
        }
        else
        {
            task_assembly::GraspConfig reset;
            storeGrasp = reset;
        }

        return true;
    }

    bool WorkspaceGrid::getEntry(float x, float y, float r, int& storeEntry, std::vector<task_assembly::GraspConfig>& storeGrasps)
    {
        if (!data)
        {
            return false;
        }

        int nPosX = (int)(((x - minX) / gridExtendX) * gridSizeX);
        int nPosY = (int)(((y - minY) / gridExtendY) * gridSizeY);
        int nPosR = (int)(((r - minR) / gridExtendRotation) * gridSizeRotation);

        if (nPosX < 0 || nPosY < 0 || nPosR < 0 || nPosX >= gridSizeX || nPosY >= gridSizeY || nPosR >= gridSizeRotation)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << fX << "," << fY << std::endl;
            return false;
        }

        storeEntry = data[getDataPos(nPosX, nPosY, nPosR)];
        storeGrasps = graspLink[getDataPos(nPosX, nPosY, nPosR)];
        return true;
    }

    int WorkspaceGrid::getCellEntry(int cellX, int cellY, int cellR)
    {
        if (!data)
        {
            return 0;
        }

        if (cellX < 0 || cellY < 0 ||  cellR < 0 ||cellX >= gridSizeX || cellY >= gridSizeY || cellR >= gridSizeRotation)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << nX << "," << nY << std::endl;
            return 0;
        }

        return data[getDataPos(cellX, cellY, cellR)];
    }

    bool WorkspaceGrid::getCellEntry(int cellX, int cellY, int cellR,int& storeEntry, task_assembly::GraspConfig& storeGrasp)
    {
        if (!data)
        {
            return false;
        }

        if (cellX < 0 || cellY < 0 ||  cellR < 0 ||cellX >= gridSizeX || cellY >= gridSizeY || cellR >= gridSizeRotation)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << nX << "," << nY << std::endl;
            return false;
        }

        storeEntry = data[getDataPos(cellX, cellY, cellR)];
        size_t nLinks = graspLink[getDataPos(cellX, cellY, cellR)].size();

        if (nLinks > 0)
        {
            storeGrasp = graspLink[getDataPos(cellX, cellY, cellR)][rand() % nLinks];
        }
        else
        {
            task_assembly::GraspConfig reset;
            storeGrasp = reset;
        }

        return true;
    }

    bool WorkspaceGrid::getCellEntry(int cellX, int cellY, int cellR, int& storeEntry, std::vector<task_assembly::GraspConfig>& storeGrasps)
    {
        if (!data)
        {
            return false;
        }

        if (cellX < 0 || cellY < 0 ||  cellR < 0 ||cellX >= gridSizeX || cellY >= gridSizeY || cellR >= gridSizeRotation)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << nX << "," << nY << std::endl;
            return false;
        }

        storeEntry = data[getDataPos(cellX, cellY, cellR)];
        storeGrasps = graspLink[getDataPos(cellX, cellY, cellR)];
        return true;
    }

    void WorkspaceGrid::setEntry(float x, float y, float r, int value, task_assembly::GraspConfig grasp)
    {
        if (!data)
        {
            return;
        }

        int nPosX = (int)(((x - minX) / gridExtendX) * gridSizeX);
        int nPosY = (int)(((y - minY) / gridExtendY) * gridSizeY);
        int nPosR = (int)(((r - minR) / gridExtendRotation) * gridSizeRotation);
        setCellEntry(nPosX, nPosY, nPosR, value, grasp);
    }

    void WorkspaceGrid::setCellEntry(int cellX, int cellY, int cellR, int value, task_assembly::GraspConfig grasp)
    {
        if (!data)
        {
            return;
        }

        if(cellX < 0 || cellY < 0 ||  cellR < 0 ||cellX >= gridSizeX || cellY >= gridSizeY || cellR >= gridSizeRotation)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << nPosX << "," << nPosY << std::endl;
            ROS_INFO("응애 여기에러");
            return;
        }

        if (data[getDataPos(cellX, cellY, cellR)] <= value)
        {
            data[getDataPos(cellX, cellY, cellR)] = value;

            if (find(graspLink[getDataPos(cellX, cellY, cellR)].begin(), graspLink[getDataPos(cellX, cellY, cellR)].end(), grasp) == graspLink[getDataPos(cellX, cellY, cellR)].end())
            {
                graspLink[getDataPos(cellX, cellY, cellR)].push_back(grasp);
            }
        }
        else if (value >= MIN_VALUES_STORE_GRASPS)
        {
            if (find(graspLink[getDataPos(cellX, cellY, cellR)].begin(), graspLink[getDataPos(cellX, cellY, cellR)].end(), grasp) == graspLink[getDataPos(cellX, cellY, cellR)].end())
            {
                graspLink[getDataPos(cellX, cellY, cellR)].push_back(grasp);
            }
        }
    }

    void WorkspaceGrid::setEntryCheckNeighbors(float x, float y, float r, int value, task_assembly::GraspConfig grasp)
    {
        if (!data)
        {
            return;
        }

        int nPosX = (int)(((x - minX) / gridExtendX) * gridSizeX);
        int nPosY = (int)(((y - minY) / gridExtendY) * gridSizeY);
        int nPosR = (int)(((r - minR) / gridExtendRotation) * gridSizeRotation);
        if (nPosX < 0 || nPosY < 0 || nPosR < 0 || nPosX >= gridSizeX || nPosY >= gridSizeY || nPosR >= gridSizeRotation)
        {
            // cout << __PRETTY_FUNCTION__ << " internal error: " << fX << "," << fY << std::endl;
            std::cout << " internal error: "<< std::endl;
            return;
        }

        setCellEntry(nPosX, nPosY, nPosR, value, grasp);

        if (nPosX > 0 && nPosX < (gridSizeX - 1) && nPosY > 0 && nPosY < (gridSizeY - 1) && nPosR > 0 && nPosR < (gridSizeRotation - 1))
        {
            setCellEntry(nPosX - 1, nPosY, nPosR, value, grasp);
            setCellEntry(nPosX - 1, nPosY - 1, nPosR, value, grasp);
            setCellEntry(nPosX - 1, nPosY + 1, nPosR, value, grasp);
            setCellEntry(nPosX, nPosY - 1, nPosR, value, grasp);
            setCellEntry(nPosX, nPosY + 1, nPosR, value, grasp);
            setCellEntry(nPosX + 1, nPosY - 1, nPosR, value, grasp);
            setCellEntry(nPosX + 1, nPosY, nPosR, value, grasp);
            setCellEntry(nPosX + 1, nPosY + 1, nPosR, value, grasp);
        }
    }

    // IRM을 만드는 부분
    void WorkspaceGrid::setEntries(std::vector<WorkspaceRepresentation::WorkspaceCut2DTransformationPtr>& wsData, const Eigen::Matrix4f& graspGlobal, task_assembly::GraspConfig grasp,const Eigen::Matrix4f& curBaseRot, float curR)
    {
        if (!data)
        {
            return;
        }

        float x, y;
        for (auto & i : wsData)
        {
            Eigen::Matrix4f tmpPos2 = graspGlobal * i->transformation.inverse();
            // test RM
            // Eigen::Matrix4f tmpPos2 = i->transformation;
            // float z = tmpPos2(2, 3);
            x = tmpPos2(0, 3);
            y = tmpPos2(1, 3);

            // x  -=   0.30861 * cosf(curR);
            // y  -=   0.30861 * sinf(curR);

            bool checkCollision = false;
            for(int i = 0; i < obsnum; ++i)
            {
                if( x >= obsBounds[i].minBound(0) && x <= obsBounds[i].maxBound(0) && y >= obsBounds[i].minBound(1) && y <= obsBounds[i].maxBound(1))
                    checkCollision = true;
            }

            if(checkCollision)
                continue;

            Eigen::Matrix3f mat = curBaseRot.block(0,0,3,3);
            Eigen::Quaternionf quat(mat);
            visualization_msgs::Marker node;
            node.type = visualization_msgs::Marker::ARROW;
            node.header.frame_id = "map";
            node.header.stamp = MakerDataInstance;
            node.id = MarkerID;
            node.action = visualization_msgs::Marker::ADD;
            node.pose.orientation.w = 1.0;
            node.pose.position.x =  x;
            node.pose.position.y =  y;
            node.pose.position.z =  0.0;
            node.pose.orientation.x =  quat.x();
            node.pose.orientation.y =  quat.y();
            node.pose.orientation.z =  quat.z();
            node.pose.orientation.w =  quat.w();
            node.scale.x = 0.035;
            node.scale.y = 0.0175;   
            node.scale.z = 0.0175;  
            
            // Lazy Collision Check
            int type = i->value;
            // if( x >= ObstacleMinX && x <=  ObstacleMaxX && y >= ObstacleMinY && y <=  ObstacleMaxY )
            //     type = 0;
            setEntryCheckNeighbors(x, y, curR, type, grasp);
        
            switch (type/ 35)
            {
            case 0:
                if(type == 0)
                    break;
                node.color.r = 0.0;
                node.color.g = 0.0;
                node.color.b = 1.0;
                break;
            case 1:
                node.color.r = 0.0;
                node.color.g = 1.0;
                node.color.b = 0.0;
                break;
            case 2:
                node.color.r = 1.0;
                node.color.g = 1.0;
                node.color.b = 0.0;
                break;
            default:
                node.color.r = 1.0;
                node.color.g = 0.0;
                node.color.b = 0.0;
                break;
            }
            node.color.a = 1.0;
            // std::cout <<"entri val----------- \n"<<tmpPos2<<" , " << i->value<<std::endl;
            //setEntry(x,y,vData[i].value);
            IRM.markers.push_back(node);
            ++MarkerID;
        }
    }

    int WorkspaceGrid::getMaxEntry()
    {
        if (!data)
        {
            return 0;
        }

        int nMax = 0;

        for (int i = 0; i < gridSizeX; i++)
            for (int j = 0; j < gridSizeY; j++)
                for (int k = 0; k < gridSizeRotation; j++)
                    if (data[getDataPos(i, j, k)] > nMax)
                    {
                        nMax = data[getDataPos(i, j, k)];
                    }

        return nMax;
    }

    bool WorkspaceGrid::getRandomPos(int minEntry, float& storeXGlobal, float& storeYGlobal, float& storeRGlobal, task_assembly::GraspConfig& storeGrasp, int maxLoops /*= 50*/, int* entries)
    {
        if (!data)
        {
            return false;
        }

        int nLoop = 0;
        int nEntry = 0;
        int x, y, r;

        do
        {
            x = rand() % gridSizeX;
            y = rand() % gridSizeY;
            r = rand() % gridSizeRotation;
            getCellEntry(x, y, r, nEntry, storeGrasp);
            if(entries)
                *entries = nEntry;
            if (nEntry >= minEntry)
            {
                storeXGlobal = minX + ((float)x + 0.5f) * discretizeSize;
                storeYGlobal = minY + ((float)y + 0.5f) * discretizeSize;
                storeRGlobal = std::fmod(minR + ((float)r + 0.0f) * discretizeStepRotation, maxR);

                return true;
            }

            nLoop++;
        }
        while (nLoop < maxLoops);

        return false;
    }


    bool WorkspaceGrid::getRandomPos(int minEntry, float& storeXGlobal, float& storeYGlobal, float& storeRGlobal, std::vector<task_assembly::GraspConfig>& storeGrasps, int maxLoops /*= 50*/, int *entries)
    {
        if (!data)
        {
            return false;
        }

        int nLoop = 0;
        int nEntry = 0;
        int x, y, r;

        do
        {
            x = rand() % gridSizeX;
            y = rand() % gridSizeY;
            r = rand() % gridSizeRotation;
            getCellEntry(x, y, r, nEntry, storeGrasps);
            if(entries)
                *entries = nEntry;
            if (nEntry >= minEntry)
            {
                storeXGlobal = minX + ((float)x + 0.5f) * discretizeSize;
                storeYGlobal = minY + ((float)y + 0.5f) * discretizeSize;
                storeRGlobal = std::fmod(minR + ((float)r + 0.0f) * discretizeStepRotation,maxR);

                return true;
            }

            nLoop++;
        }
        while (nLoop < maxLoops);

        return false;
    }

    void WorkspaceGrid::setGridPosition(float x, float y)
    {
        minX = x - gridExtendX / 2.0f;
        maxX = x + gridExtendX / 2.0f;
        minY = y - gridExtendY / 2.0f;
        maxY = y + gridExtendY / 2.0f;
    }

    // 한 평면에 대해서 진행한다  ---> 그럼 discrete하게 여러방향 돌면 해결되겟네 0~ 2PI
    bool WorkspaceGrid::fillGridData(WorkspaceRepresentationPtr ws, task_assembly::GraspConfig g)
    {
        if (!ws)
        {
            return false;
        }

        Eigen::Matrix4f graspGlobal = ws->getGlobalEEpose(g);
        // Eigen::Matrix4f local = ws->getToGlobalTransformation();
        // Eigen::Vector3f minBB, maxBB;
        //  ws->getWorkspaceExtends(minBB, maxBB);  
        // float sizeZ = maxBB[2] - minBB[2];
        // int numVoxelsZ = (int)(sizeZ / discretizeSize);
        // float z =0; 
        // for(int c = 0; c <numVoxelsZ; ++c )
        // {
        //     z  = minBB[2] + c *  discretizeSize;

        for(float RotZ = minR; RotZ < maxR; RotZ += discretizeStepRotation)
        {
            // WorkspaceRepresentationPtr wsTmp = ws->clone_ws();
            // float RotZ = 0.0;
            
            Eigen::Matrix4f rotMat;
            rotMat.setIdentity();
            rotMat.block(0,0,3,3) = rotateZaxis_f(RotZ);
            // rotMat = local * rotMat * local.inverse();
            std::cout<<rotMat<<std::endl;
            ws->rotateLocalBase(rotMat);
            WorkspaceRepresentation::WorkspaceCut2DPtr cutXY = ws->createCut(graspGlobal, discretizeSize, false);
            // RM Visualization Here IRM --> Date in here
            std::cout<<cutXY->entries<<std::endl;
            std::vector<WorkspaceRepresentation::WorkspaceCut2DTransformationPtr> transformations = ws->createCutTransformations(cutXY);
            setEntries(transformations, graspGlobal, g, rotMat, RotZ);
            ws->rotateLocalBase(rotMat.inverse());
        // }
        }
        return true;
    }

    void WorkspaceGrid::getExtends(float& storeMinX, float& storeMaxX, float& storeMinY, float& storeMaxY)
    {
        storeMinX = minX;
        storeMaxX = maxX;
        storeMinY = minY;
        storeMaxY = maxY;
    }

    void WorkspaceGrid::getCells(int& storeCellsX, int& storeCellsY)
    {
        storeCellsX = gridSizeX;
        storeCellsY = gridSizeY;
    }

    float WorkspaceGrid::getDiscretizeSize() const
    {
        return discretizeSize;
    }

    Eigen::Vector2f WorkspaceGrid::getMin() const
    {
        return Eigen::Vector2f(minX, minY);
    }

    Eigen::Vector2f WorkspaceGrid::getMax() const
    {
        return Eigen::Vector2f(maxX, maxY);
    }

    void WorkspaceGrid::setObs(task_assembly::ObstacleBox2D obs, int idx)
    {
        obsBounds[idx].maxBound(0) = obs.maxX.data;
        obsBounds[idx].maxBound(1) = obs.maxY.data;
        obsBounds[idx].minBound(0) = obs.minX.data;
        obsBounds[idx].minBound(1) = obs.minY.data;
    }

    WorkspaceGridPtr WorkspaceGrid::MergeWorkspaceGrids(const std::vector<WorkspaceGridPtr> &grids)
    {
        VR_ASSERT(grids.size() >= 2);
        float totalMaxX = std::numeric_limits<float>::min();
        float totalMinX = std::numeric_limits<float>::max();
        float totalMaxY = std::numeric_limits<float>::min();
        float totalMinY = std::numeric_limits<float>::max();
        for(auto& grid : grids)
        {
//            VR_INFO << "min: " << grid->getMin() << " max: " << grid->getMax() << std::endl;
            totalMinX = std::min(grid->getMin()(0), totalMinX);
            totalMinY = std::min(grid->getMin()(1), totalMinY);
            totalMaxX = std::max(grid->getMax()(0), totalMaxX);
            totalMaxY = std::max(grid->getMax()(1), totalMaxY);
        }
        WorkspaceGridPtr resultGrid(new WorkspaceGrid(totalMinX, totalMaxX, totalMinY, totalMaxY, grids.at(0)->getDiscretizeSize(), grids.at(0)->getDiscretizeSize()));
//        VR_INFO << "minx : " << totalMinX << " maxX: " << totalMaxX << " step size: " << resultGrid->getDiscretizeSize() << std::endl;
//        int sameValueCount =  0;
//        int totalValueCount = 0;


//         for(float x = totalMinX; x < totalMaxX; x += resultGrid->getDiscretizeSize())
//         {
//             for(float y = totalMinY; y < totalMaxY; y += resultGrid->getDiscretizeSize())
//             {
//                 int min = std::numeric_limits<int>::max();
//                 task_assembly::GraspConfig grasp;
//                 for(auto& grid : grids)
//                 {
//                     int score;
//                     task_assembly::GraspConfig tmpGrasp;
//                     if(grid->getEntry(x, y, score, tmpGrasp))
//                     {
//                         grasp = tmpGrasp;
//                     }
//                     min = std::min(score, min);
//                 }
// //                totalValueCount++;
//                 resultGrid->setEntry(x, y, min, grasp);
//             }
//         }


//        VR_INFO << "Same value percentage: " << (100.f * sameValueCount/totalValueCount) << std::endl;
        return resultGrid;

    }

} //  namespace

