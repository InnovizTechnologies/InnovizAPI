#ifndef ROS_UTILS_H
#define ROS_UTILS_H

// std
#include <string>
#include <sstream>

// ros
#include "ros/ros.h"

namespace ros_utils
{
    // set ros logger level
    void setRosLoggerLevel(int loggerLevel);

    template<typename T>
    T getRosParam(const std::string& name, ros::NodeHandle node = {"~"})
    {
        T value;
        bool success = node.getParam(name, value);
        if(!success)
        {
            throw std::runtime_error("Missing parameter " + name);
        }

        return value;
    }

    // number to string in hex
    template<typename T>
    std::string toStringHex(const T& x)
    {
        std::stringstream ss;
        ss << "0x" << std::hex << x;
        return ss.str();
    }
}

#endif
