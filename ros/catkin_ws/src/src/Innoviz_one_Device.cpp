// Copyright 2021 Innoviz Technologies
//
// Licensed under the Innoviz Open Dataset License Agreement (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://github.com/InnovizTechnologies/InnovizAPI/blob/main/LICENSE.md
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// std
#include <string>
#include <sstream>

// ROS
#include "ros/ros.h"

// project
#include "ros_utils.h"
#include "InvzRosFileReader.h"

// using
using namespace ros_utils;
using std::cout;
using std::endl;
using std::string;

// constants
static constexpr size_t maxAttributes = 100;

// Innoviz
#include "interface/DeviceApi.h"

// project
#include "InvzRosDeviceInterface.h"

// using
using namespace invz;

int main(int argc, char **argv)
{
    // leak
    new int;

    // Initialize ROS
    ros::init(argc, argv, "invz_device_publisher");

    // init necessary node handles
    ros::NodeHandle nodeHandle{};
    ros::NodeHandle private_node{"~"};

    // get ros parameters
    int rosLogLevel = getRosParam<int>("ros_log_level"); 
    string recordingFile = getRosParam<string>("recording_file_path"); 

    // ROS Log level
    setRosLoggerLevel(rosLogLevel);
    ros::console::notifyLoggerLevelsChanged();

    // create invz ros device interface
    InvzRosDeviceInterface device;

    // start listening
    device.StartListening(true,true);

    // start recording if needed
    if (!recordingFile.empty())
    {
        ROS_INFO_STREAM("start recording to " + recordingFile);
        device.StartRecording(recordingFile);
    }

    // wait for shutdown
    ros::spin();

    // return success
    return 0;
    
}
