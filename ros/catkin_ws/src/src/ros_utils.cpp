// header
#include "ros_utils.h"

namespace ros_utils
{
    void setRosLoggerLevel(int loggerLevel)
    {
        bool res = false;
        switch (loggerLevel)
        {
        case 0:
            res = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
            break;
        case 1:
            res = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
            break;
        case 2:
            res = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);
            break;
        case 3:
            res = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error);
            break;
        case 4:
            res = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
            break;
        default:
            break;
        }

        if (!res)
        {
            throw std::runtime_error("failed to set ros logger level!");
        }
    }
}
