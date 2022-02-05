#include "FileIO.h"

std::vector< Eigen::Vector3f > FileIO::readPts(const std::string& filename, const char separator)
{
    std::ifstream file(filename.c_str());
    if(!file.good())
    {
        std::cout<<"Could not open file" << filename<<std::endl;
        assert(file.good());
    } 
    char tmp;
    float a, b, c;
    std::vector< Eigen::Vector3f > res;
    Eigen::Vector3f v;
    bool needToReadSep = true;

    if (separator == ' ' || separator == '\n')
    {
        needToReadSep = false;
    }

    while (file.good())
    {
        file >> a;

        //THROW_VR_EXCEPTION_IF(!file.good(),"Error in file " << filename << " line " << res.size());
        if (needToReadSep)
        {
            file >> tmp;
            //THROW_VR_EXCEPTION_IF(!file.good(),"Error in file " << filename << " line " << res.size());
        }

        file >> b;

        //THROW_VR_EXCEPTION_IF(!file.good(),"Error in file " << filename << " line " << res.size());
        if (needToReadSep)
        {
            file >> tmp;
            //THROW_VR_EXCEPTION_IF(!file.good(),"Error in file " << filename << " line " << res.size());
        }

        file >> c;
        v << a, b, c;
        res.push_back(v);
    }

    return res;
}

