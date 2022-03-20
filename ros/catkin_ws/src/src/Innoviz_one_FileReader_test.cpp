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
#include <chrono>
#include <thread>
#include <string>
#include <mutex>
#include <condition_variable>

// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"

// invz
#include "common_includes/invz_types.h"

// project
#include "ros_utils.h"

// using
using namespace std::chrono;
using namespace ros_utils;

// constants
static constexpr uint32_t QUEUE_SIZE = 1000;

// static variables
static bool shouldStop = false;
static std::mutex shouldStopMutex;
static std::condition_variable shouldStopCv;
static bool msgReceived = false;

// received messages callback
void messageCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg)
{
    ROS_INFO_STREAM("Received PointCloud message with " << msg->points.size() << " points.");
    std::lock_guard<std::mutex> lck(shouldStopMutex);
    shouldStop = true;
    msgReceived = true;
    shouldStopCv.notify_all();
}

// stops ros when timeout or message
template<typename Time>
void timeoutWork(Time time)
{
    std::unique_lock<std::mutex> lck(shouldStopMutex);
    shouldStopCv.wait_for(lck, time, []{ return shouldStop; });
    ros::shutdown();
}

// application entry point
int main(int argc, char **argv)
{
    // init ros
    ros::init(argc, argv, "innoviz_filereader_test");

    // init necessary node handles
    ros::NodeHandle nodeHandle{};
    ros::NodeHandle private_node{"~"};

    // get parameters
    int subscriptionTimeoutMilliseconds = getRosParam<int>("subscription_timeout_milliseconds");
    int messageTimeoutMilliseconds = getRosParam<int>("message_timeout_milliseconds");

    // try to subscribe for timeout milliseconds
    ros::Subscriber sub;
    auto start = steady_clock::now();
    while (!sub && duration_cast<milliseconds>(steady_clock::now() - start).count() < (int)subscriptionTimeoutMilliseconds)
    {
        sub = nodeHandle.subscribe("invz_reflection_0", QUEUE_SIZE, messageCallback);
        if (!sub)
        {
            std::this_thread::sleep_for(milliseconds(1));
        }
    }

    // check sub
    if (sub)
    {
        ROS_INFO_STREAM("subscribe success!");
    }
    else
    {
        throw std::runtime_error("failed to sub!");
    }

    // create timeout thread
    std::thread timeoutThread([messageTimeoutMilliseconds]{ timeoutWork(std::chrono::milliseconds(messageTimeoutMilliseconds)); });

    // wait for timeout thread to call ros::shutdown - either because a message was received or because of timeout
    ros::spin();

    // join thread
    timeoutThread.join();

    // check msgReceived
    if (msgReceived)
    {
        return 0;
    }
    else
    {
        return -1;
    }
}
