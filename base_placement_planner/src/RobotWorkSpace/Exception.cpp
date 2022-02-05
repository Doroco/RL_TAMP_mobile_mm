#include "../include/RobotWorkSpace/Exception.h"

namespace RobotWorkSpace
{

    VirtualRobotException::VirtualRobotException(const std::string& what)
    {
        this->exception = "[VirtualRobot] Exception: ";
        this->exception += what;
    }

    VirtualRobotException::VirtualRobotException(const char* what)
    {
        this->exception = "[VirtualRobot] Exception: ";
        this->exception += what;
    }

    VirtualRobotException::~VirtualRobotException() throw()
    = default;

    const char* VirtualRobotException::what() const throw()
    {
        return exception.c_str();
    }

} // namespace VirtualRobot
