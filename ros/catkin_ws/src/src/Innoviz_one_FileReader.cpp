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
#include "std_msgs/String.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"

// STL
#include <sstream>

// Innoviz
#include "interface/FileReaderApi.h"
#include "common_includes/invz_types.h"
#include "common.h"

using namespace invz;

static constexpr size_t QUEUE_BUFFER = 100;
static size_t EAGLE_PIXELS_PER_FRAME = 192000;
static size_t EAGLE_SUM_PIXELS_PER_FRAME = 48000;
static size_t EAGLE_FPS = 15;
static size_t FALCON_PIXELS_PER_FRAME = 307200;
static size_t FALCON_SUM_PIXELS_PER_FRAME = 76800;
static size_t FALCON_FPS = 10;
size_t buffersCount = 100; // 100 is max size it will be changed to relevant number after GetFrameDataAttributes

// ------------------ FileHandler ----------------------
class FileHandler
{
    IReader *fileReaderItf = nullptr;

    invz::FrameDataUserBuffer reqBuffers[6];
    invz::DeviceMeta meta;
    invz::FrameDataAttributes *attributes = new invz::FrameDataAttributes[buffersCount];

public:
    int pixels_per_frame = 0;
    int sum_pixels_per_frame = 0;
    int fps = 0;
    invz::INVZ2MeasurementXYZType *measurement_reflection0 = nullptr;
    invz::INVZ2MeasurementXYZType *measurement_reflection1 = nullptr;
    invz::INVZ2MeasurementXYZType *measurement_reflection2 = nullptr;
    invz::INVZ2SumMeasurementXYZType *summation_reflection = nullptr;
	invz::INVZ2PixelMetaData *single_pixel_metaData = nullptr;

    FileHandler(std::string path, std::string config_path)
    {
        fileReaderItf = FileReaderInit(path, "", 3, true, false, 1, config_path);
    }

    void GetFrameDataAttributes()
    {
        fileReaderItf->GetFrameDataAttributes(attributes, buffersCount);
        for (size_t i = 0; i < buffersCount; i++)
        {
            if (attributes[i].known_type == GRAB_TYPE_MEASURMENTS_REFLECTION0)
            {
                if (attributes[i].length == EAGLE_PIXELS_PER_FRAME)
                {
                    pixels_per_frame = EAGLE_PIXELS_PER_FRAME;
                    sum_pixels_per_frame = EAGLE_SUM_PIXELS_PER_FRAME;
                    fps = EAGLE_FPS;
                }
                else
                {
                    pixels_per_frame = FALCON_PIXELS_PER_FRAME;
                    sum_pixels_per_frame = FALCON_SUM_PIXELS_PER_FRAME;
                    fps = FALCON_FPS;
                }
                break;
            }
        }
    }

    void AllocateBuffers()
    {
        // Check if API is able to grab this data in the first frame
        // We get this data each frame in the frame meta
        meta.lrf_count = 4;
        for (uint32_t i = 0; i < meta.lrf_count; i++)
        {
            meta.lrf_height[i] = 20;
            meta.lrf_width[i] = 300;
        }

        GetFrameDataAttributes();
        measurement_reflection0 = new INVZ2MeasurementXYZType[pixels_per_frame];
        measurement_reflection1 = new INVZ2MeasurementXYZType[pixels_per_frame];
        measurement_reflection2 = new INVZ2MeasurementXYZType[pixels_per_frame];
        summation_reflection = new INVZ2SumMeasurementXYZType[sum_pixels_per_frame];
		single_pixel_metaData = new INVZ2PixelMetaData[pixels_per_frame];

        // Activating the relevant buffers
        for (size_t i = 0; i < buffersCount; i++)
        {
            if (attributes[i].known_type == GRAB_TYPE_METADATA)
            {
                reqBuffers[0].dataAttrs = attributes[i];
                reqBuffers[0].dataBuffer = new uint8_t[attributes[i].itemSize * attributes[i].length];
            }
            if (attributes[i].known_type == GRAB_TYPE_MEASURMENTS_REFLECTION0)
            {
                reqBuffers[1].dataAttrs = attributes[i];
                reqBuffers[1].dataBuffer = reinterpret_cast<uint8_t *>(measurement_reflection0);
            }
            if (attributes[i].known_type == GRAB_TYPE_MEASURMENTS_REFLECTION1)
            {
                reqBuffers[2].dataAttrs = attributes[i];
                reqBuffers[2].dataBuffer = reinterpret_cast<uint8_t *>(measurement_reflection1);
            }
            if (attributes[i].known_type == GRAB_TYPE_MEASURMENTS_REFLECTION2)
            {
                reqBuffers[3].dataAttrs = attributes[i];
                reqBuffers[3].dataBuffer = reinterpret_cast<uint8_t *>(measurement_reflection2);
            }
            if (attributes[i].known_type == GRAB_TYPE_SUMMATION_REFLECTION0)
            {
                reqBuffers[4].dataAttrs = attributes[i];
                reqBuffers[4].dataBuffer = reinterpret_cast<uint8_t *>(summation_reflection);
            }
			if (attributes[i].known_type == GRAB_TYPE_SINGLE_PIXEL_META_DATA)
			{
				reqBuffers[5].dataAttrs = attributes[i];
				reqBuffers[5].dataBuffer = reinterpret_cast<uint8_t *>(single_pixel_metaData);
			}
        }
    }

    void Close()
    {
        FileReaderClose(fileReaderItf);

        if (reqBuffers != nullptr)
        {
			delete[] reqBuffers[0].dataBuffer;
			delete[] reqBuffers[1].dataBuffer;
			delete[] reqBuffers[2].dataBuffer;
			delete[] reqBuffers[3].dataBuffer;
			delete[] reqBuffers[4].dataBuffer;
			delete[] reqBuffers[5].dataBuffer;
        }
        ROS_INFO_STREAM("Removed the reqBuffers");
    }

    void GetNumOfFrames(size_t &frames)
    {
        fileReaderItf->GetNumOfFrames(frames);
    }

    Result GrabFrame(uint32_t &frameNumber, uint64_t &pc_timestamp, uint32_t run_idx)
    {
        // Grabbing only the measurements buff
        return fileReaderItf->GrabFrame(reqBuffers, 6, frameNumber, pc_timestamp, run_idx);
    }
};

// ---------------------------RVIZ Publisher---------------------------------

void readAndSendFramePCL(ros::Publisher &publisher, INVZ2MeasurementXYZType *measurements, int pixels_per_frame, float fpa, std::string frame_id)
{

    // Set ros pcl message
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_msg(new pcl::PointCloud<pcl::PointXYZI>());
    pcl_msg->header.stamp = pcl_conversions::toPCL(ros::Time::now());
    pcl_msg->header.frame_id = frame_id;
    pcl_msg->points.reserve(pixels_per_frame);

    ROS_DEBUG_STREAM("Preparing msg to publish");

    static constexpr float hundredth = 1.0 / 100.0f;
    // For each pixel - update its details within the message info, in both types
    for (int i = 0; i < pixels_per_frame; i++)
    {
        // PCL Msg
        pcl::PointXYZI current_point;

        //Only process points that meet the fpa requirements
        if (measurements[i].pfa <= fpa && (measurements[i].x != 0 || measurements[i].y != 0 || measurements[i].z != 0))
        {
            current_point.x = measurements[i].x * hundredth;
            current_point.y = measurements[i].y * hundredth;
            current_point.z = measurements[i].z * hundredth;

            current_point.intensity = measurements[i].reflectivity;

            // Add to pointcloud msg
            pcl_msg->points.push_back(current_point);
        }
    }

    // Publish message:
    publisher.publish(pcl_msg);
    ROS_DEBUG_STREAM("Published pcl_msg");
}

void readAndSendFrameSumPCL(ros::Publisher &publisher, INVZ2SumMeasurementXYZType *measurements, int sum_pixels_per_frame, float fpa, std::string frame_id)
{

    // Set ros pcl message
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_msg(new pcl::PointCloud<pcl::PointXYZI>());
    pcl_msg->header.stamp = pcl_conversions::toPCL(ros::Time::now());
    pcl_msg->header.frame_id = frame_id;
    pcl_msg->points.reserve(sum_pixels_per_frame);

    ROS_DEBUG_STREAM("Preparing msg to publish");

    static constexpr float hundredth = 1.0 / 100.0f;
    // For each pixel - update its details within the message info, in both types
    for (int i = 0; i < sum_pixels_per_frame; i++)
    {
        // PCL Msg
        pcl::PointXYZI current_point;

        if (measurements[i].pfa <= fpa && (measurements[i].x != 0 || measurements[i].y != 0 || measurements[i].z != 0))
        {
            current_point.x = measurements[i].x * hundredth;
            current_point.y = measurements[i].y * hundredth;
            current_point.z = measurements[i].z * hundredth;

            current_point.intensity = measurements[i].reflectivity;

            // Add to pointcloud msg
            pcl_msg->points.push_back(current_point);
        }
    }

    // Publish message:
    publisher.publish(pcl_msg);
    ROS_DEBUG_STREAM("Published pcl_msg");
}

int main(int argc, char **argv)
{
    // General parameters
    std::string file_path = "";
    std::string frame_id = "base_link";
    std::string ref0_name = "invz_reflection_0";
    std::string ref1_name = "invz_reflection_1";
    std::string ref2_name = "invz_reflection_2";
    std::string sum_ref_name = "invz_summation_reflection";
    size_t number_of_frames = 0;
    int log_level = 3;
    std::string config_file_path = "";

    // Loop helpers
    uint32_t frameNumber = 0;
    uint64_t pc_timestamp = 0;
    int counter = 0;
    float fpa = 0;

    // Initialize ROSamoount_of_pixels
    ros::init(argc, argv, "invz_filereader_publisher");

    ros::NodeHandle private_node("~");
    private_node.param("reflection_0", ref0_name, std::string("base_link"));
    private_node.param("reflection_1", ref1_name, std::string("base_link"));
    private_node.param("reflection_2", ref2_name, std::string("base_link"));
    private_node.param("summation_reflection", sum_ref_name, std::string("base_link"));
    private_node.param("file_path", file_path, std::string(""));
    private_node.param("frame_id", frame_id, std::string("base_link"));
    private_node.param<float>("false_positive_alarm", fpa, 0.0);
    private_node.param<int>("log_level", log_level, 3);
    private_node.param("config_file_path", config_file_path, std::string(""));

    // ROS Log level
    bool res = false;
    switch (log_level)
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
    if (res)
        ros::console::notifyLoggerLevelsChanged();

    ROS_INFO_STREAM("RVIZ topic 0: " << ref0_name);
    ROS_INFO_STREAM("RVIZ topic 1: " << ref1_name);
    ROS_INFO_STREAM("RVIZ topic 2: " << ref2_name);
    ROS_INFO_STREAM("RVIZ topic 3: " << sum_ref_name);

    ROS_INFO_STREAM("file_path: " << file_path);
    ROS_INFO_STREAM("frame_id: " << frame_id);
    ROS_INFO_STREAM("false_positive_alarm: " << fpa);
    ROS_INFO_STREAM("log_level: " << log_level);

    // Initialize FileHandler Interface
    FileHandler fh = FileHandler(file_path, config_file_path);
    fh.AllocateBuffers();
    ROS_INFO_STREAM("Allocated buffers");

    // Initialize service nodes and get handler
    ros::NodeHandle publisher_node;

    // Initialize publisher for publisher_node
    // Messages are of String type, Queue is limited to size of QUEUE_BUFFER (message buffer)
    ros::Publisher pub0 = publisher_node.advertise<sensor_msgs::PointCloud2>(ref0_name, 1);
    ros::Publisher pub1 = publisher_node.advertise<sensor_msgs::PointCloud2>(ref1_name, 1);
    ros::Publisher pub2 = publisher_node.advertise<sensor_msgs::PointCloud2>(ref2_name, 1);
    ros::Publisher pub3 = publisher_node.advertise<sensor_msgs::PointCloud2>(sum_ref_name, 1);

    // Fix sending rate
    ros::Rate loop_rate(fh.fps);

    // Try opening the file and initialize parameters
    try
    {
        // get number of frames
        fh.GetNumOfFrames(number_of_frames);
        ROS_INFO_STREAM("Number of frames in file: " << number_of_frames);

        // Read File until EOF
        clock_t begin = clock();
        ROS_INFO_STREAM("Start reading frames...");

        // Read each frame and send it as PCL
        while (ros::ok() && counter < number_of_frames - 1)
        {

            invz::Result ret = fh.GrabFrame(frameNumber, pc_timestamp, counter);

            // Validating the grab was successful
            if (ret.error_code != invz::ERROR_CODE_OK)
            {
                ROS_WARN_STREAM("Frame " << counter << " was not grabbed.");
                counter++;
                continue;
            }

            // Publishing the data
            readAndSendFramePCL(pub0, fh.measurement_reflection0, fh.pixels_per_frame, fpa, frame_id);
            readAndSendFramePCL(pub1, fh.measurement_reflection1, fh.pixels_per_frame, fpa, frame_id);
            readAndSendFramePCL(pub2, fh.measurement_reflection2, fh.pixels_per_frame, fpa, frame_id);
            readAndSendFrameSumPCL(pub3, fh.summation_reflection, fh.sum_pixels_per_frame, fpa, frame_id);
            ROS_INFO_STREAM("Sent frame: " << counter << "/" << std::to_string(number_of_frames));

            counter++;
            loop_rate.sleep();

            if (counter == number_of_frames - 1)
            {
                counter = 0;
            }
        }

        ROS_INFO_STREAM("Number of frames read: " << counter);
        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        ROS_INFO_STREAM("Reading successfully finished." << std::to_string(elapsed_secs));
    }

    catch (const std::runtime_error &e)
    {
        // All errors are currently handled as runtime errors
        std::cout << "Error has occured: " << e.what() << std::endl;
    }

    // -----------------------------------------------------------------
    // Close Invz Device API
    // -----------------------------------------------------------------
    // Each API pointer must be Closed, otherwise there might be a memory leak
    fh.Close();

    return 0;
}
