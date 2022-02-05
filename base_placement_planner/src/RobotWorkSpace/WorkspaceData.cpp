#include "../include/RobotWorkSpace/WorkspaceData.h"

#include <fstream>
#include <cmath>
#include <float.h>
#include <limits.h>

namespace RobotWorkSpace
{

    WorkspaceData::WorkspaceData(unsigned int size1, unsigned int size2, unsigned int size3,
                                 unsigned int size4, unsigned int size5, unsigned int size6, bool adjustOnOverflow)
    {
        unsigned long long sizeTr = (unsigned long long)size1 * (unsigned long long)size2 * (unsigned long long)size3;
        unsigned long long sizeRot = (unsigned long long)size4 * (unsigned long long)size5 * (unsigned long long)size6;
        sizes[0] = size1;
        sizes[1] = size2;
        sizes[2] = size3;
        sizes[3] = size4;
        sizes[4] = size5;
        sizes[5] = size6;
        sizeTr0 = sizes[1] * sizes[2];
        sizeTr1 = sizes[2];
        sizeRot0 = sizes[4] * sizes[5];
        sizeRot1 = sizes[5];

        //UINT_MAX --> unsinged int의 최대 
        if (sizeRot > UINT_MAX || sizeTr > UINT_MAX)
        {
            std::cout << "Could not assign " << sizeRot << " bytes of memory (>UINT_MAX). Reduce size of reachability space..." << endl;
        }

        try
        {
            data = new unsigned char* [(unsigned int)sizeTr];

            for (unsigned int x = 0; x < size1; x++)
            {
                for (unsigned int y = 0; y < size2; y++)
                {
                    for (unsigned int z = 0; z < size3; z++)
                    {
                        data[x * sizeTr0 + y * sizeTr1 + z] = NULL;
                    }
                }
            }
        }
        catch (const std::exception& e)
        {
            std::cout << "Exception: " << e.what() << endl << "Could not assign " << sizeRot << " bytes of memory. Reduce size of reachability space..." << endl;
            throw;
        }
        catch (...)
        {
            std::cout << "Could not assign " << sizeRot << " bytes of memory. Reduce size of reachability space..." << endl;
            throw;
        }

        minValidValue = 1;
        maxEntry = 0;
        voxelFilledCount = 0;
        this->adjustOnOverflow = adjustOnOverflow;
    }

    WorkspaceData::WorkspaceData(WorkspaceDataPtr other)
    {
        VR_ASSERT(other);

        for (int i = 0; i < 6; i++)
        {
            this->sizes[i] = other->sizes[i];
        }

        unsigned long long sizeTr = (unsigned long long)sizes[0] * (unsigned long long)sizes[1] * (unsigned long long)sizes[2];
        unsigned long long sizeRot = (unsigned long long)sizes[3] * (unsigned long long)sizes[4] * (unsigned long long)sizes[5];
        sizeTr0 = sizes[1] * sizes[2];
        sizeTr1 = sizes[2];
        sizeRot0 = sizes[4] * sizes[5];
        sizeRot1 = sizes[5];

        if (sizeRot > UINT_MAX || sizeTr > UINT_MAX)
        {
            std::cout << "Could not assign " << sizeRot << " bytes of memory (>UINT_MAX). Reduce size of reachability space..." << endl;
        }

        try
        {
            data = new unsigned char* [(unsigned int)sizeTr];

            for (unsigned int x = 0; x < sizes[0]; x++)
            {
                for (unsigned int y = 0; y < sizes[1]; y++)
                {
                    for (unsigned int z = 0; z < sizes[2]; z++)
                    {
                        int pos = x * sizeTr0 + y * sizeTr1 + z;

                        if (other->data[pos] != NULL)
                        {
                            data[pos] = new unsigned char[(unsigned int)sizeRot];
                            memcpy(data[pos], other->data[pos], (unsigned int)sizeRot * sizeof(unsigned char));
                        }
                        else
                        {
                            data[pos] = NULL;
                        }
                    }
                }
            }
        }
        catch (const std::exception& e)
        {
            std::cout << "Exception: " << e.what() << endl << "Could not assign " << sizeRot << " bytes of memory. Reduce size of reachability space..." << endl;
            throw;
        }
        catch (...)
        {
            std::cout << "Could not assign " << sizeRot << " bytes of memory. Reduce size of reachability space..." << endl;
            throw;
        }

        minValidValue = other->minValidValue;
        maxEntry = other->maxEntry;
        voxelFilledCount = other->voxelFilledCount;
        this->adjustOnOverflow = other->adjustOnOverflow;
    }

    WorkspaceData::~WorkspaceData()
    {
        for (unsigned int x = 0; x < sizes[0]; x++)
        {
            for (unsigned int y = 0; y < sizes[1]; y++)
            {
                for (unsigned int z = 0; z < sizes[2]; z++)
                {
                    delete[] data[x * sizeTr0 + y * sizeTr1 + z];
                }
            }
        }

        delete[] data;
    }

    unsigned int WorkspaceData::getSizeTr() const
    {
        return sizes[0] * sizes[1] * sizes[2];
    }

    unsigned int WorkspaceData::getSizeRot() const
    {
        return sizes[3] * sizes[4] * sizes[5];
    }

    void WorkspaceData::ensureData(unsigned int x, unsigned int y, unsigned int z)
    {
        if (data[x * sizeTr0 + y * sizeTr1 + z])
        {
            return;
        }

        unsigned long long sizeRot = (unsigned long long)sizes[3] * (unsigned long long)sizes[4] * (unsigned long long)sizes[5];
        data[x * sizeTr0 + y * sizeTr1 + z] = new unsigned char[(unsigned int)sizeRot];
        memset(data[x * sizeTr0 + y * sizeTr1 + z], 0, (unsigned int)sizeRot * sizeof(unsigned char));
    }

    void WorkspaceData::setDataRot(unsigned char* data, unsigned int x, unsigned int y, unsigned int z)
    {
        ensureData(x, y, z);
        memcpy(this->data[x * sizeTr0 + y * sizeTr1 + z], data, getSizeRot()*sizeof(unsigned char));
    }

    const unsigned char* WorkspaceData::getDataRot(unsigned int x, unsigned int y, unsigned int z)
    {
        ensureData(x, y, z);
        return data[x * sizeTr0 + y * sizeTr1 + z];
    }

    unsigned char WorkspaceData::getMaxEntry() const
    {
        return maxEntry;
    }

    unsigned int WorkspaceData::getVoxelFilledCount() const
    {
        return voxelFilledCount;
    }

    void WorkspaceData::binarize()
    {
        unsigned int posTr = 0, posRot = 0;

        for (unsigned int a = 0; a < sizes[0]; a++)
        {
            for (unsigned int b = 0; b < sizes[1]; b++)
            {
                for (unsigned int c = 0; c < sizes[2]; c++)
                {
                    for (unsigned int d = 0; d < sizes[3]; d++)
                    {
                        for (unsigned int e = 0; e < sizes[4]; e++)
                        {
                            for (unsigned int f = 0; f < sizes[5]; f++)
                            {
                                getPos(a, b, c, d, e, f, posTr, posRot);

                                if (data[posTr])
                                    if (data[posTr][posRot] > minValidValue)
                                    {
                                        data[posTr][posRot] = minValidValue;
                                    }
                            }
                        }
                    }
                }
            }
        }

        maxEntry = minValidValue;
    }

    void WorkspaceData::bisectData()
    {
        unsigned int posTr = 0, posRot = 0;

        for (unsigned int x0 = 0; x0 < sizes[0]; x0++)
            for (unsigned int x1 = 0; x1 < sizes[1]; x1++)
                for (unsigned int x2 = 0; x2 < sizes[2]; x2++)
                    for (unsigned int x3 = 0; x3 < sizes[3]; x3++)
                        for (unsigned int x4 = 0; x4 < sizes[4]; x4++)
                            for (unsigned int x5 = 0; x5 < sizes[5]; x5++)
                            {
                                getPos(x0, x1, x2, x3, x4, x5, posTr, posRot);

                                if (data[posTr])
                                    if (data[posTr][posRot] > minValidValue)
                                    {
                                        data[posTr][posRot] /= 2;

                                        if (data[posTr][posRot] < minValidValue)
                                        {
                                            data[posTr][posRot] = minValidValue;
                                        }
                                    }
                            }

        if (maxEntry > minValidValue)
        {
            maxEntry = maxEntry / 2;

            if (maxEntry < minValidValue)
            {
                maxEntry = minValidValue;
            }
        }
    }

    void WorkspaceData::setDatumCheckNeighbors(unsigned int x[6], unsigned char value, unsigned int neighborVoxels)
    {
        setDatum(x, value);

        if (neighborVoxels == 0)
        {
            return;
        }

        int minX[6];
        int maxX[6];

        for (int i = 0; i < 6; i++)
        {
            minX[i] = x[i] - neighborVoxels;
            maxX[i] = x[i] + neighborVoxels;

            if (minX[i] < 0)
            {
                minX[i] = 0;
            }

            if (maxX[i] >= (int)sizes[i])
            {
                maxX[i] = sizes[i] - 1;
            }
        }

        for (int a = minX[0]; a <= maxX[0]; a++)
            for (int b = minX[1]; b <= maxX[1]; b++)
                for (int c = minX[2]; c <= maxX[2]; c++)
                    for (int d = minX[3]; d <= maxX[3]; d++)
                        for (int e = minX[4]; e <= maxX[4]; e++)
                            for (int f = minX[5]; f <= maxX[5]; f++)
                            {
                                if (get(a, b, c, d, e, f) < value)
                                {
                                    setDatum((unsigned int)a, (unsigned int)b, (unsigned int)c, (unsigned int)d, (unsigned int)e, (unsigned int)f, value);
                                }
                            }
    }

    void WorkspaceData::clear()
    {
        for (unsigned int x = 0; x < sizes[0]; x++)
        {
            for (unsigned int y = 0; y < sizes[1]; y++)
            {
                for (unsigned int z = 0; z < sizes[2]; z++)
                {
                    if (data[x * sizeTr0 + y * sizeTr1 + z])
                    {
                        delete [] data[x * sizeTr0 + y * sizeTr1 + z];
                        data[x * sizeTr0 + y * sizeTr1 + z] = NULL;
                    }
                }
            }
        }

        maxEntry = 0;
        voxelFilledCount = 0;
    }

    bool WorkspaceData::hasEntry(unsigned int x, unsigned int y, unsigned int z)
    {
        if (x < 0 || y < 0 || z < 0 || x >= sizes[0] || y >= sizes[1] || z >= sizes[2])
        {
            return false;
        }

        return (data[x * sizeTr0 + y * sizeTr1 + z] != NULL);
    }

    void WorkspaceData::setMinValidValue(unsigned char v)
    {
        minValidValue = v;
    }

} // namespace VirtualRobot
