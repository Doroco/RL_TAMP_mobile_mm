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
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include <boost/current_function.hpp>

#include <string>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <cassert>

#ifdef WIN32
#pragma warning(disable:4275)
#endif

namespace RobotWorkSpace
{

    /**
     * This class defines a custom exception which is used in the VirtualRobot library.
     * Use the macros THROW_VR_EXCEPTION and THROW_VR_EXCEPTION_IF to create and
     * throw exceptions of this kind.
     */
    class VirtualRobotException : public std::exception
    {
    public:
        VirtualRobotException(const std::string& what);
        VirtualRobotException(const char* what);

        ~VirtualRobotException() noexcept override;

        const char* what() const noexcept override;
        //virtual const char * what() const;

    protected:
        std::string exception;
    };

} // namespace VirtualRobot

#define VR_INFO std::cout <<__FILE__ << ":" << __LINE__ << ": "
#define VR_WARNING std::cerr <<__FILE__ << ":" << __LINE__ << " -Warning- "
#define VR_ERROR std::cerr <<__FILE__ << ":" << __LINE__ << " - ERROR - "
#define VR_ASSERT( expr )  assert(expr)
#define VR_ASSERT_MESSAGE(expr, msg) assert((expr)&&(msg))
/**
 * This macro throws a VirtualRobot::VirtualRobotException and is the preferred
 * way of creating and throwing these types of exceptions.
 * The reported message is composed of the line number and the file in which
 * the throw occurred and is followed by the \a messageString parameter.
 */
#define THROW_VR_EXCEPTION(messageString) \
    do { \
        std::stringstream s; \
        s << __FILE__ << ":" << __LINE__ << ": " << BOOST_CURRENT_FUNCTION << ": " << messageString; \
        std::string er = s.str(); throw RobotWorkSpace::VirtualRobotException(er); \
    } while(0);

/**
 * This macro checks \a condition and calls THROW_VR_EXCEPTION
 * with the parameter \a messageString if the condition is true.
 */
#define THROW_VR_EXCEPTION_IF(condition, messageString) \
    do { \
        if (condition) \
            THROW_VR_EXCEPTION(messageString); \
    } while(0);

#ifdef WIN32
#pragma warning(default:4275)
#endif

