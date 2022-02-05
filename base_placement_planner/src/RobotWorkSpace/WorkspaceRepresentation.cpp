#include "../include/RobotWorkSpace/WorkspaceRepresentation.h"
#include "../include/RobotWorkSpace/Exception.h"
#include "../include/RobotWorkSpace/CompressionRLE.h"
#include "../include/RobotWorkSpace/CompressionBZip2.h"
#include "Random.h"
#include <fstream>
#include <cmath>
#include <cfloat>
#include <climits>
#include <thread>

namespace RobotWorkSpace
{
    using std::cout;
    using std::endl;

    WorkspaceRepresentation::WorkspaceRepresentation(ros::NodeHandle & _nh,double _hz,RobotWorkSpace::YAMLConfig robotConfig, KDL::JntArray nominal, PoseQualityManipulability::ManipulabilityIndexType i)
    : PoseQualityExtendedManipulability(robotConfig,nominal,i),
    nh_(_nh),
    hz_(_hz)
    {
        file_format_ = ".bin";
        file_Path_ = "/home/min/ws_ws/src/base_placement_planner/mapConfig";
        type = "WorkspaceRepresentation";
        versionMajor = 2;
        versionMinor = 9;
        orientationType = Hopf; //Hopf 이녀석은 0 ~ 2pi가 범위임으로 주의하세용  (이전버젼)EulerXYZExtrinsic;  Hopf
        LocalBase = Eigen::Matrix4f::Identity();
        VisionBase = Eigen::Matrix4f::Identity();
        reset();
    }

    void WorkspaceRepresentation::reset()
    {
        data.reset();
        buildUpLoops = 0;
        collisionConfigs = 0;
        discretizeStepTranslation = 0;
        discretizeStepRotation = 0;

        for (int i = 0; i < 6; i++)
        {
            minBounds[i] = FLT_MAX;
            maxBounds[i] = -FLT_MAX;
            achievedMinValues[i] = FLT_MAX;
            achievedMaxValues[i] = -FLT_MAX;
            numVoxels[i] = 0;
            spaceSize[i] = 0;
        }
    }
    /////////////////////////////////////////////////////////////// 파일 포멧 유틸들 /////////////////////////////////////////////////////////////////

    void WorkspaceRepresentation::uncompressData(const unsigned char* source, int size, unsigned char* dest)
    {
        unsigned char count;
        unsigned char value;

        for (int i = 0; i < size / 2; i++)
        {
            count = *source;
            source++;
            value = *source;
            source++;
            memset(dest, (int)value, sizeof(unsigned char) * count);
            dest += count;
        }
    }

    unsigned char* WorkspaceRepresentation::compressData(const unsigned char* source, int size, int& compressedSize)
    {
        // on large arrays sometimes an out-of-memory exception is thrown, so in order to reduce the size of the array, we assume we can compress it
        // hence, we have to check if the compressed size does not exceed the original size on every pos increase
        unsigned char* dest;

        try
        {
            dest  = new unsigned char[/*2 * */size];
        }
        catch(const std::exception &e)
        {
            VR_ERROR << "Error:" << e.what() << endl << "Could not assign " << size << " bytes of memory. Reduce size of WorkspaceRepresentation data..." << std::endl;
            throw;
        }
        catch (...)
        {
            VR_ERROR << "Could not assign " << size << " bytes of memory. Reduce size of WorkspaceRepresentation data..." << std::endl;
            throw;
        }

        int pos = 0;

        unsigned char curValue = source[0];
        unsigned char count = 1;

        for (int i = 1; i < size; i++)
        {
            if (source[i] == curValue)
            {
                if (count == 255)
                {
                    dest[pos] = 255;
                    dest[pos + 1] = curValue;
                    pos += 2;
                    THROW_VR_EXCEPTION_IF(pos >= size, "Could not perform run-length compression. Data is too cluttered!!!");

                    count = 1;
                }
                else
                {
                    count++;
                }
            }
            else
            {
                dest[pos] = count;
                dest[pos + 1] = curValue;
                pos += 2;
                THROW_VR_EXCEPTION_IF(pos >= size, "Could not perform run-length compression. Data is too cluttered!!!");

                curValue = source[i];
                count = 1;
            }
        }

        if (count > 0)
        {
            dest[pos] = count;
            dest[pos + 1] = curValue;
            pos += 2;
            THROW_VR_EXCEPTION_IF(pos >= size, "Could not perform run-length compression. Data is too cluttered!!!");
        }

        compressedSize = pos;
        return dest;
    }

    void WorkspaceRepresentation::load(const std::string& filename)
    {
        // filename = file_Path_ + filename + file_format_;
        std::ifstream file(filename.c_str(), std::ios::in | std::ios::binary);
        THROW_VR_EXCEPTION_IF(!file, "File could not be read.");
        reset();

        try
        {
            std::string tmpString;

            std::string tmpStr2 = type;
            tmpStr2 += " Binary File";

            // Check file type
            FileIO::readString(tmpString, file);
            bool fileTypeOK = false;

            if (tmpString == "WorkspaceRepresentation Binary File" ||
                tmpString == "Reachability Binary File" ||
                tmpString == "Reachbaility Binary File" || // typo in old versions
                tmpString == "Manipulability Binary File" ||
                tmpString == "ReachabilitySpace Binary File" ||
                tmpString == tmpStr2)
            {
                fileTypeOK = true;
            }


            THROW_VR_EXCEPTION_IF(!fileTypeOK, "Wrong file format:" << tmpString);

            // Check version
            int version[2];
            version[0] = (int)(FileIO::read<ioIntTypeRead>(file));
            version[1] = (int)(FileIO::read<ioIntTypeRead>(file));

            // first check if the current version is used
            if (version[0] != versionMajor || version[1] != versionMinor)
            {
                std::cout << "File version: " << version[0] << "." << version[1] << std::endl;
                // now check if an older version is used
                THROW_VR_EXCEPTION_IF(
                    (version[0] > 2) ||
                    (version[0] == 2 && !(version[1]>=0 && version[1] <= 9)) ||
                    (version[0] == 1 && !(version[1] == 0 || version[1] == 2 || version[1] == 3)
                    ),  "Wrong file format version");
            }
            if (version[0] > 2 || (version[0] == 2 && version[1] > 7))
            {
                orientationType = Hopf;
            }
            else if (version[0] == 2 && version[1] > 6)
            {
                orientationType = EulerXYZExtrinsic;
            }
            else if (version[0] == 2 && version[1] == 6)
            {
                orientationType = EulerXYZ;
            }
            else
            {
                orientationType = RPY;
            }

            versionMajor = version[0];
            versionMinor = version[1];

            buildUpLoops = (int)(FileIO::read<ioIntTypeRead>(file));

            collisionConfigs = (int)(FileIO::read<ioIntTypeRead>(file));
            discretizeStepTranslation = FileIO::read<float>(file);
            discretizeStepRotation = FileIO::read<float>(file);

            for (int & numVoxel : numVoxels)
            {
                numVoxel = (int)(FileIO::read<ioIntTypeRead>(file));
            }

            //FileIO::readArray<int>(numVoxels, 6, file);
            int voxelFilledCount = (int)(FileIO::read<ioIntTypeRead>(file));
            int maxEntry = (int)(FileIO::read<ioIntTypeRead>(file));

            for (int i = 0; i < 6; i++)
            {
                minBounds[i] = FileIO::read<float>(file);
                maxBounds[i] = FileIO::read<float>(file);
                spaceSize[i] = maxBounds[i] - minBounds[i];
            }

            for (int i = 0; i < 6; i++)
            {
                achievedMinValues[i] = FileIO::read<float>(file);
                achievedMaxValues[i] = FileIO::read<float>(file);
            }

            // Read Data
            FileIO::readString(tmpString, file);
            THROW_VR_EXCEPTION_IF(tmpString != "DATA_START", "Bad file format, expecting DATA_START.");

            long size = numVoxels[0] * numVoxels[1] * numVoxels[2] * numVoxels[3] * numVoxels[4] * numVoxels[5];
            data.reset(new WorkspaceDataArray(numVoxels[0], numVoxels[1], numVoxels[2], numVoxels[3], numVoxels[4], numVoxels[5], true));

            if (version[0] <= 1 || (version[0] == 2 && version[1] <= 3))
            {
                // one data block
                unsigned char* d = new unsigned char[size];

                if (version[0] == 1 && version[1] <= 2)
                {
                    // Data is uncompressed
                    FileIO::readArray<unsigned char>(d, size, file);
                }
                else
                {
                    // Data is compressed
                    int compressedSize = (int)(FileIO::read<ioIntTypeRead>(file));
                    unsigned char* compressedData = new unsigned char[compressedSize];
                    FileIO::readArray<unsigned char>(compressedData, compressedSize, file);

                    if ((version[0] > 2) || (version[0] == 2 && version[1] >= 1))
                    {
                        CompressionRLE::RLE_Uncompress(compressedData, d, compressedSize);
                    }
                    else
                    {
                        uncompressData(compressedData, compressedSize, d);
                    }

                    delete[] compressedData;
                }

                // convert old data format
                unsigned char* dRot;
                unsigned int sizeX0 = numVoxels[1] * numVoxels[2] * numVoxels[3] * numVoxels[4] * numVoxels[5];
                unsigned int sizeX1 = numVoxels[2] * numVoxels[3] * numVoxels[4] * numVoxels[5];
                unsigned int sizeX2 = numVoxels[3] * numVoxels[4] * numVoxels[5];
                unsigned int sizeX3 = numVoxels[4] * numVoxels[5];
                unsigned int sizeX4 = numVoxels[5];
                dRot = new unsigned char[numVoxels[3]*numVoxels[4]*numVoxels[5]];

                for (int x = 0; x < numVoxels[0]; x++)
                    for (int y = 0; y < numVoxels[1]; y++)
                        for (int z = 0; z < numVoxels[2]; z++)
                        {
                            for (int a = 0; a < numVoxels[3]; a++)
                                for (int b = 0; b < numVoxels[4]; b++)
                                    for (int c = 0; c < numVoxels[5]; c++)
                                    {
                                        dRot[a * sizeX3 + b * sizeX4 + c] =
                                            d[x * sizeX0 + y * sizeX1 + z * sizeX2 + a * sizeX3 + b * sizeX4 + c];
                                    }

                            data->setDataRot(dRot, x, y, z);
                        }

                delete [] dRot;
                delete[] d;
            }
            else
            {
                // data is split, only rotations are given in blocks
                // Data is compressed

                bool compressionBZIP2 = false;

                if (version[0] > 2 || (version[0] == 2 && version[1] >= 5))
                {
                    compressionBZIP2 = true;
                }

                if (compressionBZIP2)
                {
                    int dataSize = numVoxels[3] * numVoxels[4] * numVoxels[5];
                    unsigned char* uncompressedData = new unsigned char[dataSize];
                    CompressionBZip2Ptr bzip2(new CompressionBZip2(&file));

                    for (int x = 0; x < numVoxels[0]; x++)
                        for (int y = 0; y < numVoxels[1]; y++)
                            for (int z = 0; z < numVoxels[2]; z++)
                            {
                                int n;
                                bool readOK = bzip2->read((void*)(uncompressedData), dataSize, n);

                                if (!readOK || (n != dataSize && n != 0))
                                {
                                    VR_ERROR << "Invalid number of bytes?!" << std::endl;
                                    bzip2->close();
                                    file.close();
                                    return;
                                }

                                if (n == 0) // no data in block
                                {
                                    continue;
                                }

                                // check if we need to set the data (avoid to allocate memory for empty data blocks)
                                bool empty = true;

                                for (int i = 0; i < dataSize; i++)
                                {
                                    if (uncompressedData[i] != 0)
                                    {
                                        empty = false;
                                        break;
                                    }
                                }

                                if (!empty)
                                {
                                    data->setDataRot(uncompressedData, x, y, z);
                                }
                            }

                    delete[] uncompressedData;
                    bzip2->close();
                }
                else
                {
                    int maxCompressedSize = numVoxels[3] * numVoxels[4] * numVoxels[5] * 3;
                    unsigned char* compressedData = new unsigned char[maxCompressedSize];
                    unsigned char* uncompressedData = new unsigned char[numVoxels[3]*numVoxels[4]*numVoxels[5]];

                    for (int x = 0; x < numVoxels[0]; x++)
                        for (int y = 0; y < numVoxels[1]; y++)
                            for (int z = 0; z < numVoxels[2]; z++)
                            {
                                int compressedSize = (int)(FileIO::read<ioIntTypeRead>(file));

                                FileIO::readArray<unsigned char>(compressedData, compressedSize, file);

                                CompressionRLE::RLE_Uncompress(compressedData, uncompressedData, compressedSize);
                                data->setDataRot(uncompressedData, x, y, z);
                            }

                    delete[] compressedData;
                    delete[] uncompressedData;
                }
            }


            data->setVoxelFilledCount(voxelFilledCount);
            data->setMaxEntry(maxEntry);

            FileIO::readString(tmpString, file);
            THROW_VR_EXCEPTION_IF(tmpString != "DATA_END", "Bad file format, expecting DATA_END");
        }
        catch (VirtualRobotException& e)
        {
            VR_ERROR << e.what() << std::endl;
            file.close();
            throw;
        }

        file.close();
    }

    void WorkspaceRepresentation::save(const std::string& filename)
    {
        // filename = file_Path_ + filename + file_format_;
        std::ofstream file;
        file.open(filename.c_str(), std::ios::out | std::ios::binary);
        THROW_VR_EXCEPTION_IF(!file.is_open(), "Could not open file");

        try
        {
            // File type
            std::string tmpStr = type;
            tmpStr += " Binary File";
            FileIO::writeString(file, tmpStr);

            // Version
            FileIO::write<ioIntTypeWrite>(file, (ioIntTypeWrite)(versionMajor));
            FileIO::write<ioIntTypeWrite>(file, (ioIntTypeWrite)(versionMinor));

            // Build loops
            FileIO::write<ioIntTypeWrite>(file, (ioIntTypeWrite)(buildUpLoops));

            // Collisions
            FileIO::write<ioIntTypeWrite>(file, (ioIntTypeWrite)(collisionConfigs));

            // DiscretizeStep*
            FileIO::write<float>(file, discretizeStepTranslation);
            FileIO::write<float>(file, discretizeStepRotation);

            // Number of voxels
            //FileIO::writeArray<ioIntTypeWrite>(file, numVoxels, 6);
            for (int & numVoxel : numVoxels)
            {
                FileIO::write<ioIntTypeWrite>(file, (ioIntTypeWrite)numVoxel);
            }

            FileIO::write<ioIntTypeWrite>(file, (ioIntTypeWrite)(data->getVoxelFilledCount()));
            FileIO::write<ioIntTypeWrite>(file, (ioIntTypeWrite)(data->getMaxEntry()));

            // Workspace extend
            for (int i = 0; i < 6; i++)
            {
                FileIO::write<float>(file, minBounds[i]);
                FileIO::write<float>(file, maxBounds[i]);
            }

            // Workspace achieved values
            for (int i = 0; i < 6; i++)
            {
                FileIO::write<float>(file, achievedMinValues[i]);
                FileIO::write<float>(file, achievedMaxValues[i]);
            }

            // Data
            FileIO::writeString(file, "DATA_START");

            if (!data->save(file))
            {
                VR_ERROR << "Unable to store data!" << std::endl;
                return;
            }

            FileIO::writeString(file, "DATA_END");
        }
        catch (VirtualRobotException& e)
        {
            std::cout << "exception: " << e.what() << std::endl;
            file.close();
            throw;
        }

        file.close();
    }

    /*!
        좌표계 변환 유틸들 !!!!!!!!////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    */
    void WorkspaceRepresentation::setVisionBasepose(Eigen::Matrix4f& p) 
    {
        VisionBase = p;
    }

    void WorkspaceRepresentation::setLocalBasepose(Eigen::Matrix4f& p) 
    {
        LocalBase = p;
    }

    void WorkspaceRepresentation::rotateLocalBase(Eigen::Matrix4f p) 
    {
        LocalBase = LocalBase * p;
    }

    void WorkspaceRepresentation::toLocal(Eigen::Matrix4f& p) const
    {
        p = getToLocalTransformation() * p;
    }
    void WorkspaceRepresentation::toGlobal(Eigen::Matrix4f& p) const
    {
        p = getToGlobalTransformation() * p;
    }

    void WorkspaceRepresentation::toLocalVec(Eigen::Vector3f& positionGlobal) const
    {
        Eigen::Matrix4f t;
        t.setIdentity();
        t.block(0, 3, 3, 1) = positionGlobal;
        toLocal(t);
        positionGlobal = t.block(0, 3, 3, 1);
    }


    void WorkspaceRepresentation::toGlobalVec(Eigen::Vector3f& positionLocal) const
    {
        Eigen::Matrix4f t;
        t.setIdentity();
        t.block(0, 3, 3, 1) = positionLocal;
        toGlobal(t);
        positionLocal = t.block(0, 3, 3, 1);
    }

    Eigen::Matrix4f WorkspaceRepresentation::getToLocalTransformation() const
    {
        return LocalBase.inverse();
    }

    Eigen::Matrix4f WorkspaceRepresentation::getToGlobalTransformation() const
    {
        return LocalBase;
    }

    Eigen::Matrix4f WorkspaceRepresentation::getTCPpose()
    {
        Eigen::Matrix4f t;
        t.setIdentity();
        if(solveFK(t))
        {
            toGlobal(t);
            return t;
        }
        else
        {
             std::cerr<<"Cannot get TCP pose about given Configuration"<<std::endl;
             return t;
        }
    }

    void WorkspaceRepresentation::matrix2Vector(const Eigen::Matrix4f& m, float x[6]) const
    {
        switch (orientationType)
        {
            case EulerXYZ:
            {
                x[0] = m(0, 3);
                x[1] = m(1, 3);
                x[2] = m(2, 3);

                Eigen::Matrix3f m_3 = m.block(0, 0, 3, 3);
                Eigen::Vector3f rotEulerxyz = m_3.eulerAngles(0, 1, 2);

                // intrinsic rotation (x y z)
                x[3] = rotEulerxyz(0);
                x[4] = rotEulerxyz(1);
                x[5] = rotEulerxyz(2);
            }
            break;

            case EulerXYZExtrinsic:
            {
                x[0] = m(0, 3);
                x[1] = m(1, 3);
                x[2] = m(2, 3);

                Eigen::Matrix3f m_3 = m.block(0, 0, 3, 3);
                Eigen::Vector3f rotEulerxyz = m_3.eulerAngles(0, 1, 2);

                // extrinsic (fixed coord system) rotation (x y z)
                x[5] = rotEulerxyz(0);
                x[4] = rotEulerxyz(1);
                x[3] = rotEulerxyz(2);
            }
            break;

            case RPY:
            {
                MathTools::eigen4f2rpy(m, x);
            }
            break;
            case Hopf:
            {
                MathTools::Quaternion q = MathTools::eigen4f2quat(m);
                Eigen::Vector3f h = MathTools::quat2hopf(q);
                x[0] = m(0,3);
                x[1] = m(1,3);
                x[2] = m(2,3);
                x[3] = h(0);
                x[4] = h(1);
                x[5] = h(2);
            }
            break;

            default:
                THROW_VR_EXCEPTION("mode nyi...");
        }
    }
    /*
    works
    Eigen::Vector3f rotEulerxyz = m_3.eulerAngles(0,1,2);
    x[3] = rotEulerxyz(0);
    x[4] = rotEulerxyz(1);
    x[5] = rotEulerxyz(2);
    m_3 =  Eigen::AngleAxisf(x[3], Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(x[4], Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(x[5], Eigen::Vector3f::UnitZ());
    m.block(0,0,3,3) = m_3;
    == OR ==
    float s1 = sin(x[3]);float s2 = sin(x[4]);float s3 = sin(x[5]);
    float c1 = cos(x[3]);float c2 = cos(x[4]);float c3 = cos(x[5]);
    // Euler XYZ
    m_3(0,0) =  c2*c3;               m_3(0,1) =  -c2*s3;              m_3(0,2) =  s2;
    m_3(1,0) =  c1*s3+c3*s1*s2;      m_3(1,1) =  c1*c3-s1*s2*s3;      m_3(1,2) =  -c2*s1;
    m_3(2,0) =  s1*s3-c1*c3*s2;      m_3(2,1) =  c3*s1+c1*s2*s3;      m_3(2,2) =  c1*c2;
    m.block(0,0,3,3) = m_3;
    */
    void WorkspaceRepresentation::vector2Matrix(const float x[6], Eigen::Matrix4f& m) const
    {
        switch (orientationType)
        {
            case EulerXYZ:
            {
                m.setIdentity();
                m(0, 3) = x[0];
                m(1, 3) = x[1];
                m(2, 3) = x[2];
                Eigen::Matrix3f m_3;
                m_3 = Eigen::AngleAxisf(x[3], Eigen::Vector3f::UnitX())
                      * Eigen::AngleAxisf(x[4], Eigen::Vector3f::UnitY())
                      * Eigen::AngleAxisf(x[5], Eigen::Vector3f::UnitZ());
                /*m_3 = Eigen::AngleAxisf(x[3], Eigen::Vector3f::UnitZ())
                    * Eigen::AngleAxisf(x[4], Eigen::Vector3f::UnitY())
                    * Eigen::AngleAxisf(x[5], Eigen::Vector3f::UnitX());*/
                /*
                float s1 = sin(x[3]);float s2 = sin(x[4]);float s3 = sin(x[5]);
                float c1 = cos(x[3]);float c2 = cos(x[4]);float c3 = cos(x[5]);
                // Euler XYZ
                m_3(0,0) =  c2*c3;               m_3(0,1) =  -c2*s3;              m_3(0,2) =  s2;
                m_3(1,0) =  c1*s3+c3*s1*s2;      m_3(1,1) =  c1*c3-s1*s2*s3;      m_3(1,2) =  -c2*s1;
                m_3(2,0) =  s1*s3-c1*c3*s2;      m_3(2,1) =  c3*s1+c1*s2*s3;      m_3(2,2) =  c1*c2;
                */
                /*
                // Euler ZYX
                m_3(0,0) =  c1*c2;    m_3(0,1) =  c1*s2*s3-c3*s1;   m_3(0,2) =  s1*s3+c1*c3*s2;
                m_3(1,0) =  c2*s1;    m_3(1,1) =  c1*c3+s1*s2*s3;   m_3(1,2) =  c3*s1*s2-c1*s3;
                m_3(2,0) =  -s2;      m_3(2,1) =  c2*s3;            m_3(2,2) =  c2*c3;
                */

                m.block(0, 0, 3, 3) = m_3;
            }
            break;

            case EulerXYZExtrinsic:
            {
                m.setIdentity();
                m(0, 3) = x[0];
                m(1, 3) = x[1];
                m(2, 3) = x[2];
                Eigen::Matrix3f m_3;
                m_3 = Eigen::AngleAxisf(x[5], Eigen::Vector3f::UnitX())
                      * Eigen::AngleAxisf(x[4], Eigen::Vector3f::UnitY())
                      * Eigen::AngleAxisf(x[3], Eigen::Vector3f::UnitZ());
                m.block(0, 0, 3, 3) = m_3;
            }
            break;

            case RPY:
            {
                MathTools::posrpy2eigen4f(x, m);
            }
            break;


            case Hopf:
            {
                Eigen::Vector3f h;
                h << x[3],x[4],x[5];
                MathTools::Quaternion q = MathTools::hopf2quat(h);
                m = MathTools::quat2eigen4f(q);
                m(0, 3) = x[0];
                m(1, 3) = x[1];
                m(2, 3) = x[2];
            }
            break;
            default:
                THROW_VR_EXCEPTION("mode nyi...");
        }
    }

    void WorkspaceRepresentation::vector2Matrix(const Eigen::Vector3f& pos, const Eigen::Vector3f& rot, Eigen::Matrix4f& m) const
    {
        float x[6];
        x[0] = pos[0];
        x[1] = pos[1];
        x[2] = pos[2];
        x[3] = rot[0];
        x[4] = rot[1];
        x[5] = rot[2];
        vector2Matrix(x, m);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void WorkspaceRepresentation::initialize(float discretizeStepTranslation, float discretizeStepRotation,
            float minBounds[6], float maxBounds[6],
            bool adjustOnOverflow)
    {
        reset();
        THROW_VR_EXCEPTION_IF((discretizeStepTranslation <= 0.0f || discretizeStepRotation <= 0.0f), "Need positive discretize steps");

        for (int i = 0; i < 6; i++)
        {
            THROW_VR_EXCEPTION_IF(minBounds[i] >= maxBounds[i], "Min/MaxBound error");
        }

        this->discretizeStepTranslation = discretizeStepTranslation;
        this->discretizeStepRotation = discretizeStepRotation;

        for (int i = 0; i < 6; i++)
        {
            this->minBounds[i] = minBounds[i];
            this->maxBounds[i] = maxBounds[i];
            spaceSize[i] = maxBounds[i] - minBounds[i];

            if (i < 3)
            {
                numVoxels[i] = (int)(spaceSize[i] / discretizeStepTranslation) + 1;
            }
            else
            {
                numVoxels[i] = (int)(spaceSize[i] / discretizeStepRotation) + 1;
            }

            THROW_VR_EXCEPTION_IF((numVoxels[i] <= 0), " numVoxels <= 0 in dimension " << i);
        }

        data.reset(new WorkspaceDataArray(numVoxels[0], numVoxels[1], numVoxels[2], numVoxels[3], numVoxels[4], numVoxels[5], adjustOnOverflow));
    }

    /////////////////////////////////////////////////////// Voxel && real Pose Utils /////////////////////////////////////////////////////
    MathTools::OOBB WorkspaceRepresentation::getOOBB(bool achievedValues) const
    {
        Eigen::Vector3f minBB;
        Eigen::Vector3f maxBB;

        if (achievedValues)
        {
            minBB << achievedMinValues[0], achievedMinValues[1], achievedMinValues[2];
            maxBB << achievedMaxValues[0], achievedMaxValues[1], achievedMaxValues[2];
        }
        else
        {
            minBB << minBounds[0], minBounds[1], minBounds[2];
            maxBB << maxBounds[0], maxBounds[1], maxBounds[2];
        }

        MathTools::OOBB oobb(minBB, maxBB, getToGlobalTransformation());
        return oobb;
    }

    float WorkspaceRepresentation::getDiscretizeParameterTranslation()
    {
        return discretizeStepTranslation;
    }

    float WorkspaceRepresentation::getDiscretizeParameterRotation()
    {
        return discretizeStepRotation;
    }

    unsigned char WorkspaceRepresentation::getEntry(const Eigen::Matrix4f& globalPose) const
    {
        if (!data)
        {
            VR_ERROR << "NULL DATA" << std::endl;
            return 0;
        }

        float x[6];

        Eigen::Matrix4f p = globalPose;
        
        // 로봇 베이스 기준 좌표계로 변환
        toLocal(p);

        // std::cout<< p << std::endl;

        // 6x1 벡터로변환
        matrix2Vector(p, x);

        // return (unsigned char)getMaxEntry(x[0],x[1],x[3]);
        // get entry
        return data->get(x, this);
    }

    unsigned char WorkspaceRepresentation::getVoxelEntry(unsigned int a, unsigned int b, unsigned int c, unsigned int d, unsigned int e, unsigned int f) const
    {
        if (/*a < 0 || b < 0 || c < 0 || d < 0 || e < 0 || f < 0 ||*/
                int(a) >= numVoxels[0] || int(b) >= numVoxels[1] || int(c) >= numVoxels[2] || int(d) >= numVoxels[3] || int(e) >= numVoxels[4] || int(f) >= numVoxels[5])
        {
            return 0;
        }

        return data->get(a, b, c, d, e, f);
    }

    int WorkspaceRepresentation::getMaxEntry() const
    {
        if (!data)
        {
            return 0;
        }

        return data->getMaxEntry();
    }

    int WorkspaceRepresentation::getMaxEntry(const Eigen::Vector3f& position_global) const
    {
        Eigen::Matrix4f gp;
        gp.setIdentity();
        gp.block(0, 3, 3, 1) = position_global;

        // get voxels
        unsigned int v[3];

        if (!getVoxelFromPosition(gp, v))
        {
            return 0;
        }

        return getMaxEntry(v[0], v[1], v[2]);
    }

    int WorkspaceRepresentation::getMaxEntry(int x0, int x1, int x2) const
    {
        int maxValue = 0;

        for (int a = 0; a < getNumVoxels(3); a += 1)
        {
            for (int b = 0; b < getNumVoxels(4); b += 1)
            {
                for (int c = 0; c < getNumVoxels(5); c += 1)
                {
                    int value = data->get(x0, x1, x2, a, b, c);

                    if (value >= maxValue)
                    {
                        maxValue = value;
                    }
                }
            }
        }
        return maxValue;
    }

    float WorkspaceRepresentation::getVoxelSize(int dim) const
    {
        if (dim < 0 || dim > 6)
        {
            return 0.0f;
        }

        if (numVoxels[dim] <= 0)
        {
            return 0.0f;
        }

        return spaceSize[dim] / numVoxels[dim];
    }

    int WorkspaceRepresentation::getNumVoxels(int dim) const
    {
        VR_ASSERT((dim >= 0 && dim < 6));

        return numVoxels[dim];
    }

    float WorkspaceRepresentation::getMinBound(int dim) const
    {
        VR_ASSERT((dim >= 0 && dim < 6));

        return minBounds[dim];
    }

    float WorkspaceRepresentation::getMaxBound(int dim) const
    {
        VR_ASSERT((dim >= 0 && dim < 6));

        return maxBounds[dim];
    }

    bool WorkspaceRepresentation::getVoxelFromPosition(float x[3], unsigned int v[3]) const
    {
        int a;

        for (int i = 0; i < 3; i++)
        {
            a = (int)(((x[i] - minBounds[i]) / spaceSize[i]) * (float)numVoxels[i]);
            if (a < 0)
            {
                return false;    //pos[i] = 0; // if pose is outside of voxel space, ignore it
            }
            else if (a >= numVoxels[i])
            {
                return false;    //pos[i] = m_nVoxels[i]-1; // if pose is outside of voxel space, ignore it
            }

            v[i] = a;
        }

        return true;
    }

    bool WorkspaceRepresentation::getVoxelFromPosition(const Eigen::Matrix4f &globalPose, unsigned int v[3]) const
    {
        float x[6];
        Eigen::Matrix4f p = globalPose;
        toLocal(p);
        matrix2Vector(p, x);
        float x2[3];
        x2[0] = x[0];
        x2[1] = x[1];
        x2[2] = x[2];
        return getVoxelFromPosition(x2, v);
    }


    Eigen::Matrix4f WorkspaceRepresentation::getPoseFromVoxel(unsigned int v[6], bool transformToGlobalPose)
    {
        float x[6];

        for (int j = 0; j < 6; j++)
        {
            x[j] = float(v[j]) + 0.5f;
        }

        return getPoseFromVoxel(x, transformToGlobalPose);
    }

    Eigen::Matrix4f WorkspaceRepresentation::getPoseFromVoxel(float v[6], bool transformToGlobalPose /*= true*/)
    {
        float x[6];

        for (int j = 0; j < 6; j++)
        {
            x[j] = minBounds[j] + v[j] * getVoxelSize(j);
        }

        Eigen::Matrix4f m;
        vector2Matrix(x, m);

        // //MathTools::posrpy2eigen4f(x,m);
        if (transformToGlobalPose)
        {
            toGlobal(m);
        }

        return m;
    }

    bool WorkspaceRepresentation::getPoseFromVoxel(unsigned int x[6], float v[6]) const
    {
        for (int i = 0; i < 6; i++)
        {
            if (/*(x[i] < 0) ||*/ (x[i] >= (unsigned int)(numVoxels[i])))
            {
                return false;
            }

            v[i] = ((((float) x[i]) * spaceSize[i]) / ((float)numVoxels[i]))  + minBounds[i];

            if (i < 3)
            {
                v[i] += discretizeStepTranslation / 2;
            }
            else
            {
                v[i] += discretizeStepRotation / 2;
            }
        }

        return true;
    }

    bool WorkspaceRepresentation::getVoxelFromPose(float x[6], unsigned int v[6]) const
    {
        float pos;
        int a;

        for (int i = 0; i < 6; i++)
        {
            pos = ((x[i] - minBounds[i]) / spaceSize[i]) * (float)numVoxels[i];
            a = (int)(pos);

            if (a < 0)
            {
                // check for rounding errors
                if (a==-1 && (fabs(float(a) - pos)<0.5f))
                    a = 0;
                else
                    return false;    //pos[i] = 0; // if pose is outside of voxel space, ignore it
            }
            else if (a >= numVoxels[i])
            {
                // check for rounding errors
                if (a==numVoxels[i] && (fabs(float(a) - pos)<0.5f))
                    a = numVoxels[i]-1;
                else
                    return false;    //pos[i] = m_nVoxels[i]-1; // if pose is outside of voxel space, ignore it
            }

            v[i] = a;
        }

        return true;
    }

    bool WorkspaceRepresentation::getVoxelFromPose(const Eigen::Matrix4f& globalPose, unsigned int v[6]) const
    {
        float x[6];
        Eigen::Matrix4f p = globalPose;
        toLocal(p);
        matrix2Vector(p, x);
        return getVoxelFromPose(x, v);
    }

    RobotWorkSpace::WorkspaceDataPtr WorkspaceRepresentation::getData()
    {
        return data;
    }

    /////////////////////////////////////////////// Write Voxel Data /////////////////////////////////////////////////////////////////////////////
    bool WorkspaceRepresentation::isCovered(const Eigen::Matrix4f& globalPose)
    {
        return (getEntry(globalPose) > 0);
    }


    bool WorkspaceRepresentation::isCovered(unsigned int v[6])
    {
        return (data->get(v) > 0);
    }

    void WorkspaceRepresentation::setVoxelEntry(unsigned int v[6], unsigned char e)
    {
        data->setDatum(v, e);
        buildUpLoops++;
    }

    void WorkspaceRepresentation::setEntry(const Eigen::Matrix4f& poseGlobal, unsigned char e)
    {
        setEntryCheckNeighbors(poseGlobal, e, 0);
    }

    void WorkspaceRepresentation::setEntryCheckNeighbors(const Eigen::Matrix4f& poseGlobal, unsigned char e, unsigned int neighborVoxels)
    {
        Eigen::Matrix4f p = poseGlobal;
        toLocal(p);

        float x[6];
        matrix2Vector(p, x);
        //MathTools::eigen4f2rpy(p,x);

        // check for achieved values
        for (int i = 0; i < 6; i++)
        {
            if (x[i] < achievedMinValues[i])
            {
                achievedMinValues[i] = x[i];
            }

            if (x[i] > achievedMaxValues[i])
            {
                achievedMaxValues[i] = x[i];
            }
        }

        // get voxels
        unsigned int v[6];

        if (getVoxelFromPose(x, v))
        {

            data->setDatumCheckNeighbors(v, e, neighborVoxels);
        }

        buildUpLoops++;

    }

    void WorkspaceRepresentation::setOrientationType(eOrientationType t)
    {
        orientationType = t;
    }

    WorkspaceRepresentation::VolumeInfo WorkspaceRepresentation::computeVolumeInformation()
    {
        WorkspaceRepresentation::VolumeInfo result;
        result.borderVoxelCount3D = 0;
        result.filledVoxelCount3D = 0;
        result.volume3D = 0;
        result.volumeFilledVoxels3D = 0;
        result.volumeVoxel3D = 0;
        result.voxelCount3D = numVoxels[0]*numVoxels[1]*numVoxels[2];
        for (int a = 0; a < numVoxels[0]; a++)
        {
            for (int b = 0; b < numVoxels[1]; b++)
            {
                for (int c = 0; c < numVoxels[2]; c++)
                {
                    int value = sumAngleReachabilities(a, b, c);

                    if (value > 0)
                    {
                        result.filledVoxelCount3D++;
                        if (a==0 || b==0 || c==0 || a==numVoxels[0]-1 || b==numVoxels[1]-1 || c==numVoxels[2]-1)
                            result.borderVoxelCount3D++;
                        else
                        {
                            int neighborEmptyCount = 0;
                            if (sumAngleReachabilities(a-1, b, c)==0)
                                neighborEmptyCount++;
                            if (sumAngleReachabilities(a+1, b, c)==0)
                                neighborEmptyCount++;
                            if (sumAngleReachabilities(a, b-1, c)==0)
                                neighborEmptyCount++;
                            if (sumAngleReachabilities(a, b+1, c)==0)
                                neighborEmptyCount++;
                            if (sumAngleReachabilities(a, b, c-1)==0)
                                neighborEmptyCount++;
                            if (sumAngleReachabilities(a, b, c+1)==0)
                                neighborEmptyCount++;
                            if (neighborEmptyCount>=1)
                                result.borderVoxelCount3D++;
                        }
                    }
                }
            }
        }
        double discrM = discretizeStepTranslation * 0.001;
        double voxelVolume = discrM * discrM * discrM;
        result.volumeVoxel3D = (float)voxelVolume;
        result.volumeFilledVoxels3D = (float)((double)result.filledVoxelCount3D * voxelVolume);
        result.volume3D = (float)(((double)result.filledVoxelCount3D - 0.5*(double)result.borderVoxelCount3D) * voxelVolume);

        return result;
    }
    //////////////////////////////////////// Check data in Voxel space /////////////////////////////////////////////////////////////////

    int WorkspaceRepresentation::sumAngleReachabilities(int x0, int x1, int x2) const
    {
        int res = 0;

        if (!data->hasEntry(x0, x1, x2))
        {
            return 0;
        }

        for (int d = 0; d < numVoxels[3]; d++)
        {
            for (int e = 0; e < numVoxels[4]; e++)
            {
                for (int f = 0; f < numVoxels[5]; f++)
                {
                    res += data->get(x0, x1, x2, d, e, f);
                }
            }
        }

        return res;
    }

    int WorkspaceRepresentation::avgAngleReachabilities(int x0, int x1, int x2) const
    {
        int res = 0;

        if (!data->hasEntry(x0, x1, x2))
        {
            return 0;
        }

        for (int d = 0; d < numVoxels[3]; d++)
        {
            for (int e = 0; e < numVoxels[4]; e++)
            {
                for (int f = 0; f < numVoxels[5]; f++)
                {
                    res += data->get(x0, x1, x2, d, e, f);
                }
            }
        }

        res /=  numVoxels[3]* numVoxels[4]* numVoxels[5];

        return res;
    }

    int WorkspaceRepresentation::getMaxSummedAngleReachablity()
    {
        int maxValue = 0;

        for (int a = 0; a < getNumVoxels(0); a += 1)
        {
            for (int b = 0; b < getNumVoxels(1); b += 1)
            {
                for (int c = 0; c < getNumVoxels(2); c += 1)
                {
                    int value = sumAngleReachabilities(a, b, c);

                    if (value >= maxValue)
                    {
                        maxValue = value;
                    }
                }
            }
        }

        return maxValue;
    }

    bool WorkspaceRepresentation::hasEntry(unsigned int x, unsigned int y, unsigned int z)
    {
        return data->hasEntry(x, y, z);
    }

    bool WorkspaceRepresentation::checkForParameters(float steps, float storeMinBounds[6], float storeMaxBounds[6])
    {
        for (int i = 0; i < 6; i++)
        {
            storeMinBounds[i] = FLT_MAX;
            storeMaxBounds[i] = -FLT_MAX;
        }

        for (int i = 0; i < steps; i++)
        {
            setRobotNodesToRandomConfig(true);
            Eigen::Matrix4f p = getTCPpose();
            toLocal(p);

            float x[6];
            matrix2Vector(p, x);

            // check for achieved values
            for (int i = 0; i < 6; i++)
            {
                if (x[i] < storeMinBounds[i])
                {
                    storeMinBounds[i] = x[i];
                }

                if (x[i] > storeMaxBounds[i])
                {
                    storeMaxBounds[i] = x[i];
                }
            }
        }

        // assume higher values
        for (int i = 0; i < 6; i++)
        {
            float sizex = storeMaxBounds[i] - storeMinBounds[i];
            float factor = 0.2f;

            if (i > 2)
            {
                factor = 0.05f;    // adjustment for rotation is smaller
            }

            storeMinBounds[i] -= sizex * factor;
            storeMaxBounds[i] += sizex * factor;
        }
        return true;
    }

    /////////////////////////////////////////// Configuration Uitls //////////////////////////////////////////////////////////////////////////
    void WorkspaceRepresentation::setCurrentTCPPoseEntryIfLower(unsigned char e)
    {
        Eigen::Matrix4f p = getTCPpose();
        toLocal(p);

        float x[6];
        matrix2Vector(p, x);
        //MathTools::eigen4f2rpy(p,x);

        // check for achieved values
        for (int i = 0; i < 6; i++)
        {
            if (x[i] < achievedMinValues[i])
            {
                achievedMinValues[i] = x[i];
            }

            if (x[i] > achievedMaxValues[i])
            {
                achievedMaxValues[i] = x[i];
            }
        }

        if (data->get(x, this) < e)
        {
            data->setDatum(x, e, this);
        }


        buildUpLoops++;
    }

    void WorkspaceRepresentation::setCurrentTCPPoseEntry(unsigned char e)
    {
        Eigen::Matrix4f p = getTCPpose(); //tcpNode->getGlobalPose();
        setEntry(p, e);
    }

    // 랜덤으로 조인트를 샘플링해서
    bool WorkspaceRepresentation::setRobotNodesToRandomConfig(bool checkForSelfCollisions /*= true*/)
    {
        float rndValue;
        float minJ, maxJ;
        // Eigen::VectorXf v(this->nominal_.size());
        float maxLoops = 1000;

        int loop = 0;

        do
        {
            setRandomConfig();
            std::vector<double> config;
            config.clear();

            for (int i = 0; i < nb_of_joints_; i++)
            {
                config.push_back(nominal_.data(i) * 180 / M_PI);
            }

            if( !checkForSelfCollisions || !rrt_.checkSelfCollision(rrt_model_,config))
            {
                return true;
            }

            collisionConfigs++;
            loop++;
        }
        while (loop < maxLoops);

        return false;
    }

    void WorkspaceRepresentation::addPose(const Eigen::Matrix4f& globalPose)
    {
        VR_ASSERT(data);
        Eigen::Matrix4f p = globalPose;
        toLocal(p);

        float x[6];
        matrix2Vector(p, x);
        //MathTools::eigen4f2rpy(p,x);
        
        
        // check for achieved values
        for (int i = 0; i < 6; i++)
        {
            if (x[i] < achievedMinValues[i])
            {
                achievedMinValues[i] = x[i];
            }

            if (x[i] > achievedMaxValues[i])
            {
                achievedMaxValues[i] = x[i];
            }
        }

        // // get voxels
        // unsigned int v[6];
        
        // if(getVoxelFromPose(x, v))
        // {
        //     // std::cout<<"sampling ... \n"<<p<<std::endl;

        //     // Eigen::VectorXf direction(6);
        //     // direction.setZero();
        //     // p(0) = 1.0f;
            
        //     float entry = getPoseQuality() / 0.85;

        //     if (entry > 1)
        //     {
        //         if (entry > 1.05)
        //         {
        //             std::cout << "Manipulability is larger than max value. Current Manip:" << getPoseQuality() << ", maxManip:" << 0.702419 << ", percent:" << entry << std::endl;
        //         }

        //         entry = 1.0f;
        //     }

        //     if(RealMax < entry)
        //         RealMax = entry;

        //     if (entry < 0)
        //     {
        //         entry = 0;
        //     }

        //     unsigned char e = (unsigned char)(entry * (float)UCHAR_MAX + 0.5f);
        //     // add at least 1, since the pose is reachable
        //     if (e == 0)
        //     {
        //         e = 1;
        //     }

        //     if (e > data->get(v))
        //     {
        //         data->setDatum(v, e);
        //     }
        //     // std::cout<<"entry ... : "<<(int)e<<std::endl;
        //     // data->increaseDatum(x, this);
        // }
        data->increaseDatum(x, this);
        buildUpLoops++;
    }
    
    void WorkspaceRepresentation::addCurrentTCPPose()
    {
        Eigen::Matrix4f p = getTCPpose();
        addPose(p);
    }

    void WorkspaceRepresentation::addRandomTCPPoses(unsigned int loops, bool checkForSelfCollisions)
    {
        RealMax = 0;
        for (unsigned int i = 0; i < loops; i++)
        {
            if (setRobotNodesToRandomConfig(checkForSelfCollisions))
            {
                addCurrentTCPPose();
            }
            else
            {
                VR_WARNING << "Could not find collision-free configuration...";
            }
            std::cout<<"loop ... : "<<i<<std::endl;
        }

        if(RealMax != 0)
            std::cout<<"update max Manipulability  : "<<RealMax<<std::endl;
    }

    // void WorkspaceRepresentation::addRandomTCPPoses(unsigned int loops, unsigned int numThreads, bool checkForSelfCollisions)
    // {
    //     if (numThreads > loops)
    //     {
    //         VR_ERROR << "Number of threads can not be bigger then number of tcp poses to add.";
    //         return;
    //     }

    //     std::vector<std::thread> threads(numThreads);
    //     unsigned int numPosesPerThread = loops / numThreads; // todo

    //     for (unsigned int i = 0; i < numThreads; i++)
    //     {
    //         threads[i] = std::thread([=, this] ()
    //         {
    //             // now sample some configs and add them to the workspace data
    //             for (unsigned int j = 0; j < numPosesPerThread; j++)
    //             {
    //                 float rndValue;
    //                 float minJ, maxJ;
    //                 // Eigen::VectorXf v(clonedNodeSet->getSize());
    //                 float maxLoops = 1000;

    //                 bool successfullyRandomized = false;

    //                 for (int k = 0; k < maxLoops; k++)
    //                 {
    //                     setRandomConfig();
    //                     std::vector<double> config;
    //                     config.clear();

    //                     for (int i = 0; i < nb_of_joints_; i++)
    //                     {
    //                         config.push_back(nominal_.data(i) * 180 / M_PI);
    //                     }

    //                     if( !checkForSelfCollisions || !rrt_.checkSelfCollision(rrt_model_,config))
    //                     {
    //                         successfullyRandomized = true;
    //                         break;
    //                     }

    //                     this->collisionConfigs++;
    //                 }

    //                 if (successfullyRandomized)
    //                 {
    //                     Eigen::Matrix4f p = getTCPpose(); //tcpNode->getGlobalPose();
    //                     addPose(p);
    //                 }
    //                 else
    //                 {
    //                     VR_WARNING << "Could not find collision-free configuration...";
    //                 }
    //             }
    //         });
    //     }

    //     // wait for all threads to finish
    //     for (unsigned int i = 0; i < numThreads; i++)
    //     {
    //         threads[i].join();
    //     }
    // }

    void WorkspaceRepresentation::clear()
    {
        data->clear();
        buildUpLoops = 0;
        collisionConfigs = 0;

        for (int i = 0; i < 6; i++)
        {
            achievedMinValues[i] = FLT_MAX;
            achievedMaxValues[i] = -FLT_MAX;
        }
    }

    RobotWorkSpace::PoseQualityMeasurementPtr WorkspaceRepresentation::clone()
    {
        RobotWorkSpace::WorkspaceRepresentationPtr res(new WorkspaceRepresentation(this->nh_,this->hz_,this->config_,this->nominal_, this->manipulabilityType));
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

        res->LocalBase = this->LocalBase;
        res->VisionBase = this->VisionBase;
        res->adjustOnOverflow = this->adjustOnOverflow;
        res->file_format_ = this->file_format_;
        res->file_Path_ = this->file_Path_;
        res->data.reset(this->data->clone());

        return res;
    }

    RobotWorkSpace::WorkspaceRepresentationPtr WorkspaceRepresentation::clone_ws()
    {
        RobotWorkSpace::WorkspaceRepresentationPtr res(new WorkspaceRepresentation(this->nh_,this->hz_,this->config_,this->nominal_, this->manipulabilityType));
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

        res->LocalBase = this->LocalBase;
        res->VisionBase = this->VisionBase;
        res->adjustOnOverflow = this->adjustOnOverflow;
        res->file_format_ = this->file_format_;
        res->file_Path_ = this->file_Path_;
        res->data.reset(this->data->clone());

        return res;

    }

    void WorkspaceRepresentation::binarize()
    {
        if (data)
        {
            data->binarize();
        }
    }

    Eigen::Matrix4f WorkspaceRepresentation::sampleCoveredPose()
    {
        int maxLoops = 10000;
        int i = 0;
        Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
        unsigned int nV[6];
        float x[6];

        while (i < maxLoops)
        {
            for (int j = 0; j < 6; j++)
            {
                nV[j] = rand() % numVoxels[j];
            }

            if (isCovered(nV))
            {
                // create pose

                for (int j = 0; j < 6; j++)
                {
                    x[j] = minBounds[j] + ((float)nV[j] + 0.5f) * getVoxelSize(j);
                }

                vector2Matrix(x, m);
                //MathTools::posrpy2eigen4f(x,m);
                toGlobal(m);
                return m;
            }

            i++;
        }

        VR_ERROR << "Could not find a valid pose?!" << std::endl;
        return m;
    }

    Eigen::Matrix4f WorkspaceRepresentation::getGlobalEEpose(task_assembly::GraspConfig grasp)
    {
        Eigen::Affine3f T_local;
        T_local = VisionBase;
        Eigen::Matrix3f m;
        m.setZero(3,3);

        m <<    grasp.approach.x, grasp.binormal.x, grasp.axis.x,
                grasp.approach.y, grasp.binormal.y, grasp.axis.y,
                grasp.approach.z, grasp.binormal.z, grasp.axis.z;

        Eigen::Vector3f T_tmp;
        T_tmp << grasp.position.x,grasp.position.y,grasp.position.z;
        
        T_tmp = T_local.linear()*T_tmp + T_local.translation();

        // // // change bais 기법 (rgb_optical_frame to camera Base)
        Quaternionf q_rot(-0.7071068, 0, 0, 0.7071068);
        m = m * T_local.linear().inverse() * q_rot.toRotationMatrix(); 

        Eigen::Affine3f res;
        res.translation() = T_tmp;
        res.linear() = m;
        return res.matrix();
    }
    
    //////////////////////////////////////////////// 2D workspace Representation ///////////////////////////////////////////////////////////
    WorkspaceRepresentation::WorkspaceCut2DPtr WorkspaceRepresentation::createCut(const Eigen::Matrix4f& referencePose, float cellSize, bool sumAngles) const
    {
        WorkspaceCut2DPtr result(new WorkspaceCut2D());
        result->referenceGlobalPose = referencePose;

        Eigen::Vector3f minBB, maxBB;

        getWorkspaceExtends(minBB, maxBB);      //Sample position at Local Position
        result->minBounds[0] = minBB(0);
        result->maxBounds[0] = maxBB(0);
        result->minBounds[1] = minBB(1);
        result->maxBounds[1] = maxBB(1);
        
        THROW_VR_EXCEPTION_IF(cellSize <= 0.0f, "Invalid parameter");
        // std::cout<<"min Max 2D Boundary ----------------------------\n"<<minBB<<"\n"<<maxBB<<std::endl;

        float sizeX = result->maxBounds[0] - result->minBounds[0];
        int numVoxelsX = (int)(sizeX / cellSize);
        float sizeY = result->maxBounds[1] - result->minBounds[1];
        int numVoxelsY = (int)(sizeY / cellSize);


        Eigen::Matrix4f tmpPose = referencePose;
        Eigen::Matrix4f localPose;
        float x[6];
        unsigned int v[6];

        result->entries.resize(numVoxelsX, numVoxelsY);


        for (int a = 0; a < numVoxelsX; a++)
        {
            tmpPose(0, 3) = result->minBounds[0] + (float)a * cellSize + 0.5f * cellSize;

            for (int b = 0; b < numVoxelsY; b++)
            {
                tmpPose(1, 3) = result->minBounds[1] + (float)b * cellSize + 0.5f * cellSize;
                
                if (sumAngles)
                {
                    localPose = tmpPose;
                    toLocal(localPose);
                    matrix2Vector(localPose,x);

                    if (!getVoxelFromPose(x, v))
                    {
                        result->entries(a, b) = 0;
                    } else
                        result->entries(a, b) = sumAngleReachabilities(v[0],v[1],v[2]);
                } else
                {
                    result->entries(a, b) = getEntry(tmpPose); //position을 옮겼으니 local value도 똑같이
                }
            }
        }

        return result;
    }

    WorkspaceRepresentation::WorkspaceCut2DPtr WorkspaceRepresentation::createCut(float heightPercent, float cellSize, bool sumAngles) const
    {
        THROW_VR_EXCEPTION_IF(cellSize <= 0.0f, "Invalid parameter");
        THROW_VR_EXCEPTION_IF(heightPercent < 0.0f || heightPercent>1.0f, "Invalid parameter");

        WorkspaceCut2DPtr result(new WorkspaceCut2D());

        Eigen::Vector3f minBB, maxBB;

        getWorkspaceExtends(minBB, maxBB);

        result->minBounds[0] = minBB(0);
        result->maxBounds[0] = maxBB(0);
        result->minBounds[1] = minBB(1);
        result->maxBounds[1] = maxBB(1);

        float sizeX = result->maxBounds[0] - result->minBounds[0];
        int numVoxelsX = (int)(sizeX / cellSize);
        float sizeY = result->maxBounds[1] - result->minBounds[1];
        int numVoxelsY = (int)(sizeY / cellSize);

        float sizeZGlobal = maxBB(2) - minBB(2);
        float poseZGlobal = minBB(2) + heightPercent*sizeZGlobal;

        result->entries.resize(numVoxelsX, numVoxelsY);
        result->entries.setZero();

        Eigen::Matrix4f refPose = getToGlobalTransformation();
        refPose(2,3) = poseZGlobal;
        return createCut(refPose, cellSize, sumAngles);
    }

    std::vector<WorkspaceRepresentation::WorkspaceCut2DTransformationPtr> WorkspaceRepresentation::createCutTransformations(WorkspaceRepresentation::WorkspaceCut2DPtr cutXY)
    {
        THROW_VR_EXCEPTION_IF(!cutXY, "NULL data");

        std::vector<WorkspaceRepresentation::WorkspaceCut2DTransformationPtr> result;

        //float x,y,z;
        //z = cutXY->referenceGlobalPose(2,3);

        int nX = cutXY->entries.rows();
        int nY = cutXY->entries.cols();

        float sizeX = (cutXY->maxBounds[0] - cutXY->minBounds[0]) / (float)nX;
        float sizeY = (cutXY->maxBounds[1] - cutXY->minBounds[1]) / (float)nY;

        for (int x = 0; x < nX; x++)
        {
            for (int y = 0; y < nY; y++)
            {
                int v = cutXY->entries(x, y);

                if (v > 0)
                {
                    WorkspaceCut2DTransformationPtr tp(new WorkspaceCut2DTransformation());
                    tp->value = v;
                    float xPos = cutXY->minBounds[0] + (float)x * sizeX + 0.5f * sizeX; // center of voxel
                    float yPos = cutXY->minBounds[1] + (float)y * sizeY + 0.5f * sizeY; // center of voxel
                    tp->transformation = cutXY->referenceGlobalPose;
                    tp->transformation(0, 3) = xPos;
                    tp->transformation(1, 3) = yPos;

                    // if (referenceNode)
                    // {
                    //     tp->transformation = referenceNode->toLocalCoordinateSystem(tp->transformation);
                    // }

                    result.push_back(tp);
                }
            }
        }

        return result;
    }

    bool WorkspaceRepresentation::getWorkspaceExtends(Eigen::Vector3f& storeMinBBox, Eigen::Vector3f& storeMaxBBox) const
    {
        Eigen::Vector3f quadPos[8];
        float x, y, z;

        for (int i = 0; i < 8; i++)
        {
            if (i % 2 == 0)
            {
                x = minBounds[0];
            }
            else
            {
                x = maxBounds[0];
            }

            if ((i >> 1) % 2 == 0)
            {
                y = minBounds[1];
            }
            else
            {
                y = maxBounds[1];
            }

            if ((i >> 2) % 2 == 0)
            {
                z = minBounds[2];
            }
            else
            {
                z = maxBounds[2];
            }

            quadPos[i](0) = x;
            quadPos[i](1) = y;
            quadPos[i](2) = z;
            toGlobalVec(quadPos[i]);
        }

        storeMinBBox = quadPos[0];
        storeMaxBBox = quadPos[0];

        for (auto & quadPo : quadPos)
        {
            for (int i = 0; i < 3; i++)
            {
                if (quadPo(i) < storeMinBBox(i))
                {
                    storeMinBBox(i) = quadPo(i);
                }

                if (quadPo(i) > storeMaxBBox(i))
                {
                    storeMaxBBox(i) = quadPo(i);
                }
            }
        }

        return true;
    }
}