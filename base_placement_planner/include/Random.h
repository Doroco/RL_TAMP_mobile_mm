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
* @author     Raphael Grimm
* @copyright  2018 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include <random>


std::mt19937_64 & PRNG64Bit();

inline typename std::mt19937_64::result_type RandomNumber()
{
    return PRNG64Bit()();
}
inline float RandomFloat()
{
    static thread_local std::uniform_real_distribution<float> d{0.f, std::nextafter(1.f, 2.f)};
    return d(PRNG64Bit());
}
inline float RandomFloat(float min, float max)
{
    return std::uniform_real_distribution<float>{min, max}(PRNG64Bit());
}
