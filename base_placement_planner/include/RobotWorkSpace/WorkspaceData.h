/**
* This file is part of Simox.
*
* Simox is free software = 0; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation = 0; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY = 0; without even the implied warranty of
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


#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>


/// 추상함수로만 이루어짐 --->인터페이스
namespace RobotWorkSpace
{
    class WorkspaceRepresentation;
    /*!
        Stores a 6-dimensional array for the vertex data of a workspace representation.
        Internally unsigned char data types are used (0...255)
    */

    class WorkspaceData : public std::enable_shared_from_this<WorkspaceData>
    {
    public:
        virtual ~WorkspaceData() {}

        //! Return the amount of data in bytes
        virtual unsigned int getSizeTr() const = 0;
        virtual unsigned int getSizeRot() const = 0;

        /// transration 설정
        virtual void setDatum(float x[], unsigned char value, const WorkspaceRepresentation* workspace) = 0;

        virtual void setDatum(unsigned int x0, unsigned int x1, unsigned int x2,
                              unsigned int x3, unsigned int x4, unsigned int x5, unsigned char value) = 0;

        virtual void setDatum(unsigned int x[6], unsigned char value) = 0;
        //////////////

        virtual void setDatumCheckNeighbors(unsigned int x[6], unsigned char value, unsigned int neighborVoxels) = 0;

        virtual void increaseDatum(float x[], const WorkspaceRepresentation* workspace) = 0;

        /*!
        virtual void increaseDatum( unsigned int x0, unsigned int x1, unsigned int x2,
                                    unsigned int x3, unsigned int x4, unsigned int x5) = 0;

        virtual void increaseDatum( unsigned int x[6] ) = 0;
        */


        /*!
            Set rotation data for given x,y,z position.
        */
        virtual void setDataRot(unsigned char* data, unsigned int x, unsigned int y, unsigned int z) = 0;
        /*!
            Get rotation data for given x,y,z position.
        */
        virtual const unsigned char* getDataRot(unsigned int x, unsigned int y, unsigned int z) = 0;

        virtual bool hasEntry(unsigned int x, unsigned int y, unsigned int z) = 0;


        //! Simulates a multi-dimensional array access
        virtual unsigned char get(float x[], const WorkspaceRepresentation* workspace) = 0;
        virtual unsigned char get(unsigned int x0, unsigned int x1, unsigned int x2,
                                  unsigned int x3, unsigned int x4, unsigned int x5) = 0;
        virtual unsigned char get(unsigned int x[6]) = 0;

        // clear all entrys
        virtual void clear() = 0;

        virtual void binarize() = 0;

        virtual void bisectData() = 0;

        virtual unsigned int getSize(int dim) = 0;

        virtual unsigned char** getRawData() = 0;

        virtual WorkspaceData* clone() = 0;

        virtual bool save(std::ofstream& file) = 0;

        virtual void setVoxelFilledCount(int c)
        {
            voxelFilledCount = c;
        }

        virtual unsigned char getMaxEntry()
        {
            return maxEntry;
        }

        virtual void setMaxEntry(unsigned char m)
        {
            maxEntry = m;
        }

        virtual unsigned int getVoxelFilledCount() const
        {
            return voxelFilledCount;
        }

        //! Min valid value is 1 by default. In cases some values are needed to indicate special flags (e.g. stability) the minimum valid number can be set here
        virtual void setMinValidValue(unsigned char v)
        {
            minValidValue = v;
        }

    protected:

        unsigned char minValidValue;

        unsigned char maxEntry;
        unsigned int voxelFilledCount;
        bool adjustOnOverflow;
    };

} // namespace VirtualRobot

