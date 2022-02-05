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
* @copyright  2013 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include <vector>
#include <fstream>
#include <iostream>
#include <stdint.h>
#include <Eigen/Core>

#include <assert.h>

namespace FileIO
{

    template<typename T> inline T read(std::ifstream& file)
    {
        T t;
        file.read((char*)&t, sizeof(T));
        return t;
    }

    template<typename T> inline void readArray(T* res, int num, std::ifstream& file)
    {
        file.read((char*)res, num * sizeof(T));
    }

    template<typename T> inline void write(std::ofstream& file, T value)
    {
        file.write((char*)&value, sizeof(T));
    }

    template<typename T> inline void writeArray(std::ofstream& file, const T* value, int num)
    {
        file.write((char*)value, num * sizeof(T));
    }

    inline bool readString(std::string& res, std::ifstream& file)
    {
        int length = read<int32_t>(file);

        if (length <= 0)
        {
            std::cout << "Bad string length: " << length << std::endl;
            return false;
        }

        char* data = new char[length + 1];
        file.read(data, length);
        data[length] = '\0';
        res = data;
        delete[] data;
        return true;
    }

    inline bool readMatrix4f(Eigen::Matrix4f& res, std::ifstream& file)
    {
        float m[16];

        try
        {
            readArray<float>(m, 16, file);
        }
        catch (...)
        {
            std::cout << "Error while reading matrix from file" << std::endl;
            return false;
        }

        res << m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11], m[12], m[13], m[14], m[15];
        return true;
    }

    inline void writeMatrix4f(std::ofstream& file, const Eigen::Matrix4f& m)
    {
        float t[16];
        int k = 0;

        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
            {
                t[k] = m(i, j);
                k++;
            }

        writeArray<float>(file, t, 16);
    }

    inline void writeString(std::ofstream& file, const std::string& value)
    {
        size_t len = value.length();
        file.write((char*)&len, sizeof(int32_t));
        file.write(value.c_str(), len);
    }

    /*!
        Read points form ascii file.
        Each row defines one point triple.
        \param filename The absolute filename.
        \param separator Separator character. Standard space, but comma or semicolon could be passed here.
        \return Vector of points.
    */
    std::vector< Eigen::Vector3f > readPts(const std::string& filename, const char separator = ' ');
}



