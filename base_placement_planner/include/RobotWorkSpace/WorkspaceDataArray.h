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

#include "WorkspaceData.h"
#include "RobotWorkSpace.h"
#include "CompressionBZip2.h"

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

/// 추상함수, 추상클래스 사용이유 --> 함수를 재정의 해 주지 않는다면 코드상에서 오류로 판단합니다. 떄문에 이런 실수들을 방지할 수 있죠.
//때문에 추상 클래스는 저렇게 순수 가상 함수로만 이뤄진 클래스가 좋습니다.!! 그리고 순수 가상 함수로만 이뤄진 추상 클래스를 인터페이스(interface)라고 부릅니다.

// /T 를 구현할 때 std::enable_shared_from_this<T> 를 상속하면 T의 객체가 자신의 shared_ptr 객체를 알 수 있게 해준다.
namespace RobotWorkSpace
{
    /*!
        Stores a 6-dimensional array for the vertex data of a workspace representation.
        Internally unsigned char data types are used (0...255)
    */
    class WorkspaceDataArray : public WorkspaceData, public std::enable_shared_from_this<WorkspaceDataArray>
    {
    public:
        /*!
            Constructor, fills the data with 0
        */
        WorkspaceDataArray(unsigned int size1, unsigned int size2, unsigned int size3,
                           unsigned int size4, unsigned int size5, unsigned int size6, bool adjustOnOverflow);

        //! Clone other data structure
        WorkspaceDataArray(WorkspaceDataArray* other);

        //! create Workspace out of file
        WorkspaceDataArray(std::ofstream& file);

        ~WorkspaceDataArray() override;

        //! Return the amount of data in bytes
        unsigned int getSizeTr() const override;
        unsigned int getSizeRot() const override;

        // setDatum if workspace's voxel is exist
        void setDatum(float x[], unsigned char value, const WorkspaceRepresentation* workspace) override;

        // 6차원 정보를 트랜스 1차원 회전 1차원 인덱스를 구해 data에 접근한 후에 value를 넣어준다.
        // distcrete한 정보이기 때문에 (다담으면 너무 많음) 덮어쓰여지는건 무시한다.(voxel이 0일 때만 value를 넣어준다)
        void setDatum(unsigned int x0, unsigned int x1, unsigned int x2,
                             unsigned int x3, unsigned int x4, unsigned int x5, unsigned char value) override;
        inline void setDatum(unsigned int x[6], unsigned char value) override;

        // 이웃으로 설정한 Voxel Bound크기만큼 돌면서 값을 저장해준다. Density를 파악해준다.
        void setDatumCheckNeighbors(unsigned int x[6], unsigned char value, unsigned int neighborVoxels) override;

        // 복셀에 저장돤 value를 올려준다, 255비트로 저장하고 있기때문에 오버플로나면 bisectData()를 실행해줌
        void increaseDatum(float x[], const WorkspaceRepresentation* workspace) override;
        inline void increaseDatum(unsigned int x0, unsigned int x1, unsigned int x2,
                                  unsigned int x3, unsigned int x4, unsigned int x5);
        inline void increaseDatum(unsigned int x[6]);
        
        // x,y,z 위치에 대해서 회전정보를 넣어줄 수 있답니다!
        void setDataRot(unsigned char* data, unsigned int x, unsigned int y, unsigned int z) override;

        // 배열형태로 회전정보를 빼온다.
        const unsigned char* getDataRot(unsigned int x, unsigned int y, unsigned int z) override;

        // workspace에서 위치에 저장된 뱔류를 가져온다.
        unsigned char get(float x[], const WorkspaceRepresentation* workspace) override;

        // voxel마다 x,y,z위치에서 angle을 비교하여 제일큰를 angle의 섬을 구하여 return
        int getMaxSummedAngleReachablity();

        // data에 접근해서 저장된 value를 가져온다. 없으면 0
        inline unsigned char get(unsigned int x0, unsigned int x1, unsigned int x2,
                                 unsigned int x3, unsigned int x4, unsigned int x5) override;
        inline unsigned char get(unsigned int x[6]) override;

        //voxel에 데이터가 존재하는지 확인한다.
        bool hasEntry(unsigned int x, unsigned int y, unsigned int z) override;

        // Set all entries to 0
        void clear() override;

        // 모든 저장된 배열을 돌면서 가장큰 밸류를 MaxEntry,minValidValue 에 저장한다.
        void binarize() override;

        // 데이터를 2로나누어준다 minValidValue 기준으로(아마 아래로 정렬인듯?)
        void bisectData() override;

        unsigned int getSize(int dim) override
        {
            return sizes[dim];
        }

        unsigned char** getRawData() override
        {
            return data;
        }

        WorkspaceData* clone() override;

        bool save(std::ofstream& file) override;
    protected:
        /// data구조 안에서 회전정보를 넣을 수 있도록 초기화(0,0,0) 시켜준다.
        void ensureData(unsigned int x, unsigned int y, unsigned int z);

        //  x,y,z 위치에 저장되어있는 angle들을 모두 더하어 return
        int sumAngleReachabilities(int x0, int x1, int x2);

        inline void getPos(unsigned int x0, unsigned int x1, unsigned int x2,
                           unsigned int x3, unsigned int x4, unsigned int x5 ,
                           unsigned int& storePosTr, unsigned int& storePosRot) const
        {
            storePosTr  = x0 * sizeTr0  + x1 * sizeTr1  + x2;
            storePosRot = x3 * sizeRot0 + x4 * sizeRot1 + x5;
        }

        // 3차원 정보를 1차원 정보로 만들어준다.
        inline void getPos(unsigned int x[6], unsigned int& storePosTr, unsigned int& storePosRot) const
        {
            storePosTr  = x[0] * sizeTr0  + x[1] * sizeTr1  + x[2];
            storePosRot = x[3] * sizeRot0 + x[4] * sizeRot1 + x[5];
        }

        unsigned int sizes[6];
        unsigned int sizeTr0, sizeTr1;   //  yz z
        unsigned int sizeRot0, sizeRot1; // ptich yaw yaw

        unsigned char** data;   //빠른 데이터 접근을 위해서 바이트형식으로 했을것 이다.
    };



} // namespace VirtualRobot

