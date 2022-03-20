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

// ROS
#include "ros/ros.h"

// std
#include <string>
#include <sstream>
#include <thread>
#include <chrono>

// project
#include "ros_utils.h"
#include "InvzRosFileReader.h"

// using
using namespace ros_utils;
using std::cout;
using std::endl;

// constants
static constexpr size_t maxAttributes = 100;

// application entry point
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "invz_filereader_publisher");

    // init necessary node handles
    ros::NodeHandle nodeHandle;
    ros::NodeHandle private_node{"~"};

    // get ros log level
    int rosLogLevel = getRosParam<int>("ros_log_level");

    // ROS Log level
    setRosLoggerLevel(rosLogLevel);
    ros::console::notifyLoggerLevelsChanged();

    // init innoviz ros file reader
    InvzRosFileReader filereader;

    // get number of frames
    auto nofFrames = filereader.getNofFrames();

    // get fps
    auto fps = nofFrames > 1 ? filereader.getFps() : 1;

    // log number of frames
    ROS_INFO_STREAM("Number of frames in file: " << nofFrames);

    // log fps
    ROS_INFO_STREAM("Recording FPS: " << fps);

    // log start reading frames
    ROS_INFO_STREAM("Start reading frames...");

    // fixed frame rate
    ros::Rate loop_rate(fps);

    // read frames while ros ok - restarting at end of recording
    for (size_t frame = 0; ros::ok(); frame = (frame + 1) % nofFrames)
    {
        // grab and handle frames
        filereader.GrabAndPublishFrame(frame, true, true);

        // sleep necessary time to keep frame rate
        loop_rate.sleep();
    }

    return 0;
}
