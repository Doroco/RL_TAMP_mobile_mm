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
* @author     Marcus Geelnard, Nikolaus Vahrenkamp
* @copyright  2011 Marcus Geelnard, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*
* The RLE compression/decompression routines are based on the source code of Marcus Geelnard'S Basic Compression Library
*/
#pragma once

#include "RobotWorkSpace.h"
#include <thread>
#include <mutex>

namespace RobotWorkSpace
{

    class CompressionRLE
    {
    public:

        /*************************************************************************
        * RLE_Compress() - Compress a block of data using an RLE coder.
        *  in     - Input (uncompressed) buffer.
        *  out    - Output (compressed) buffer. This buffer must be 0.4% larger
        *           than the input buffer, plus one byte.
        *  insize - Number of input bytes.
        * The function returns the size of the compressed data.
        *************************************************************************/
        static int RLE_Compress(const unsigned char* in, unsigned char* out, unsigned int insize);

        /*************************************************************************
        * RLE_Uncompress() - Uncompress a block of data using an RLE decoder.
        *  in      - Input (compressed) buffer.
        *  out     - Output (uncompressed) buffer. This buffer must be large
        *            enough to hold the uncompressed data.
        *  insize  - Number of input bytes.
        *************************************************************************/
        static void RLE_Uncompress(const unsigned char* in, unsigned char* out, unsigned int insize);

    protected:
        static void _RLE_WriteRep(unsigned char* out, unsigned int* outpos, unsigned char marker, unsigned char symbol, unsigned int count);
        static void _RLE_WriteNonRep(unsigned char* out, unsigned int* outpos, unsigned char marker, unsigned char symbol);

        static std::mutex mutex;
    };

} // namespace VirtualRobot

