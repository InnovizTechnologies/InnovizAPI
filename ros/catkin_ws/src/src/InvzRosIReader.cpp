///////////////////////////////////////////////////////////
//  InvzRosIReader.cpp
//  Implementation of the Class InvzRosIReader
//  Created on:      06-Mar-2022 12:19:45 PM
//  Original author: tal.levy
///////////////////////////////////////////////////////////

// header
#include "InvzRosIReader.h"

// std
#include <vector>
#include <stdexcept>
#include <algorithm>
#include <thread>

// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"

// project
#include "invz_utils.h"
#include "ros_utils.h"

// using
using std::unique_ptr;
using std::string;
using std::vector;
using std::set;
using std::function;
using namespace std::chrono;
using namespace invz;
using namespace invz_utils;
using namespace ros_utils;

// constants
static constexpr size_t MAX_ATTRIBUTES = 100;
static constexpr uint32_t ADVERTISE_QUEUE_SIZE = 10;
static constexpr float CENTIMETER_METER_RATIO = 0.01;
static constexpr uint64_t SECOND_MICROSECOND_RATIO = 1000000;

// string to PclTimestampOption
PclTimestampOption pclTimestampOptionFromString(const string& s)
{
	if (s == "sampled")
	{
		return PclTimestampOption::sampled;
	} 
	else if (s == "received")
	{
		return PclTimestampOption::received;
	} 
	else if (s == "published")
	{
		return PclTimestampOption::published;
	} 
	else {
		throw std::invalid_argument("cannot convert string " + s + " to PclTimestampOption!");
	} 
}

std::set<GrabType> InvzRosIReader::getRequestedGrabTypes()
{
	return { 
		GRAB_TYPE_METADATA,
		GRAB_TYPE_MEASURMENTS_REFLECTION0,
		GRAB_TYPE_MEASURMENTS_REFLECTION1,
		GRAB_TYPE_MEASURMENTS_REFLECTION2,
		GRAB_TYPE_SUMMATION_REFLECTION0,
		GRAB_TYPE_SINGLE_PIXEL_META_DATA
	};
}

InvzRosIReader::InvzRosIReader(unique_ptr<IReader, function<void(IReader*)>> iReader, unsigned int getAttributesTimeoutMilli, bool activateBuffers) :
	m_iReader{ std::move(iReader) }
{
	// get topic names from parameter server
	std::map<GrabType, string> topicNames;
	topicNames[GRAB_TYPE_MEASURMENTS_REFLECTION0] = getRosParam<string>("reflection_0");
	topicNames[GRAB_TYPE_MEASURMENTS_REFLECTION1] = getRosParam<string>("reflection_1");
	topicNames[GRAB_TYPE_MEASURMENTS_REFLECTION2] = getRosParam<string>("reflection_2");
	topicNames[GRAB_TYPE_SUMMATION_REFLECTION0] = getRosParam<string>("summation_reflection");

	// get other parameters from parameter server
	m_fpa = getRosParam<float>("false_positive_alarm");
	m_frameId = getRosParam<string>("frame_id");
	string pclTimestampOptionString = getRosParam<string>("pcl_timestamp_option");
	m_pclTimestampOption = pclTimestampOptionFromString(pclTimestampOptionString);	

	// get attributes
	vector<FrameDataAttributes> attributes = getAttributesTimeout(*m_iReader, MAX_ATTRIBUTES, getAttributesTimeoutMilli);

	// get requested buffers as set
	auto requestedBuffers = InvzRosIReader::getRequestedGrabTypes();

	// create and map buffers
	m_buffers = createBuffers(attributes, requestedBuffers);
	
	// build map
	for (auto& buffer : m_buffers)
	{
		const auto& grabType = buffer.dataAttrs.known_type;
		m_grabTypeBufferPublisherMap[grabType].first = &buffer;
	}

	// report missing buffers
	for (const auto& grabType : requestedBuffers)
	{
		if (!m_grabTypeBufferPublisherMap.count(grabType))
		{
			ROS_WARN_STREAM("buffer " << toStringHex(grabType) << "is not supplied by the device/recording!");
		}
	}

	// activate buffers if necessary
	if (activateBuffers)
	{
		std::for_each(m_buffers.begin(), m_buffers.end(), [this](FrameDataUserBuffer& buffer) { m_iReader->ActivateBuffer(buffer.dataAttrs, true); });
	}

	// create publishers and insert to map
	for (const auto& grabTypeTopicNamePair : topicNames)
	{
		const auto& grabType = grabTypeTopicNamePair.first;
		const auto& topicName = grabTypeTopicNamePair.second;
		m_grabTypeBufferPublisherMap[grabType].second = m_rosNode.advertise<sensor_msgs::PointCloud2>(topicName, ADVERTISE_QUEUE_SIZE);
	}
}

bool InvzRosIReader::GrabFrame(uint32_t& frameNumber, uint64_t& timestamp, uint32_t frameIndex)
{
	auto res = m_iReader->GrabFrame(m_buffers.data(), m_buffers.size(), frameNumber, timestamp, frameIndex);
	m_lastGrabFrameTimestamp = timestamp;
	ROS_DEBUG_STREAM("m_iReader->GrabFrame result.error_code: " << res.error_code << ", result.error_message: " << res.error_message);
	return res.error_code == ERROR_CODE_OK;
}

template<typename PointType>
void PublishPc(const PointType* pointArray, size_t pointArraySize, ros::Publisher& publisher, float fpa, const std::string& frameId, uint64_t timestamp)
{     
	// Set ros pcl message
	pcl::PointCloud<pcl::PointXYZI>::Ptr pclMsg(new pcl::PointCloud<pcl::PointXYZI>());
	pclMsg->header.stamp = timestamp;
	pclMsg->header.frame_id = frameId;
	pclMsg->points.reserve(pointArraySize);

	// log start creating PCL msg
	ROS_DEBUG_STREAM("Preparing msg to publish");

	// For each pixel - update its details within the message info, in both types
	for (size_t i = 0; i < pointArraySize; i++)
	{
		// get current point in buffer
		const auto& sourcePoint = pointArray[i];

		// create PCL point
		pcl::PointXYZI targetPoint;

		// Only process points that meet the fpa requirements and are not zeroed
		if (sourcePoint.pfa <= fpa && (sourcePoint.x != 0 || sourcePoint.y != 0 || sourcePoint.z != 0))
		{
			// set point x, y, z
			targetPoint.x = sourcePoint.x * CENTIMETER_METER_RATIO;
			targetPoint.y = sourcePoint.y * CENTIMETER_METER_RATIO;
			targetPoint.z = sourcePoint.z * CENTIMETER_METER_RATIO;

			// set point intensity
			targetPoint.intensity = sourcePoint.reflectivity;

			// add to pointcloud msg
			pclMsg->points.push_back(std::move(targetPoint));
		}
	}

	// Publish message:
	publisher.publish(pclMsg);
	ROS_DEBUG_STREAM("Published pcl_msg with timestamp: " << pclMsg->header.stamp);;

}

void InvzRosIReader::PublishBuffer(const FrameDataUserBuffer& buffer, ros::Publisher& publisher, uint64_t timestamp)
{
	switch (buffer.dataAttrs.known_type)
	{
		case GRAB_TYPE_MEASURMENTS_REFLECTION0:
		case GRAB_TYPE_MEASURMENTS_REFLECTION1:
		case GRAB_TYPE_MEASURMENTS_REFLECTION2:
			PublishPc(reinterpret_cast<INVZ2MeasurementXYZType*>(buffer.dataBuffer), buffer.dataAttrs.length, publisher, m_fpa, m_frameId, timestamp);
			break;
		case GRAB_TYPE_SUMMATION_REFLECTION0:
			PublishPc(reinterpret_cast<INVZ2SumMeasurementXYZType*>(buffer.dataBuffer), buffer.dataAttrs.length, publisher, m_fpa, m_frameId, timestamp);
			break;
		default:
			throw std::invalid_argument("PublishPc - unknown buffer type " + std::to_string(buffer.dataAttrs.known_type) + "!");
	}
}

uint64_t InvzRosIReader::getTimestamp()
{
	// init output
	uint64_t output = 0;

	// set output according to pcl timestamp option
	switch (m_pclTimestampOption)
	{
		case PclTimestampOption::sampled:
		{
			// check metadata buffer exists
			if (!m_grabTypeBufferPublisherMap.count(GRAB_TYPE_METADATA))
			{
				ROS_WARN_STREAM("pcl_timestamp_option is sampled but metadata buffer doesn't exist." <<
					" setting timestamp to 0.");
				return 0;
			}

			// get metadata buffer
			const auto* metadataUserBuffer = m_grabTypeBufferPublisherMap.at(GRAB_TYPE_METADATA).first;

			// check metadata buffer full
			if (metadataUserBuffer->status != USER_BUFFER_FULL)
			{
				ROS_WARN_STREAM("pcl_timestamp_option is sampled but metadata buffer is not full." <<
					" setting timestamp to 0.");
				return 0;
			}
			
			// reinterpret as metadata and return timestamp generated by OM at frame sample time
			const auto* metadata = reinterpret_cast<CSampleFrameMeta*>(metadataUserBuffer->dataBuffer);
			return metadata->timestamp_utc_secs * SECOND_MICROSECOND_RATIO + metadata->timestamp_utc_micro;
		}
		case PclTimestampOption::received:
		{
			// return last timestamp returned from IReader::GrabFrame
			return m_lastGrabFrameTimestamp;
		}
		case PclTimestampOption::published:
		{
			// return current time
			return pcl_conversions::toPCL(ros::Time::now());;
		}
		default:
		{
			throw std::logic_error("unknown PclTimestampOption: " + std::to_string(m_pclTimestampOption));
		}
	}

	return output;

}

void InvzRosIReader::PublishFrame(bool checkBufferFull)
{
	// get timestamp by PclTimestampOption
	uint64_t timestamp = getTimestamp();

	// iterate buffers and publish if publisher exists
	for (auto& grabTypeBufferPublisherPair : m_grabTypeBufferPublisherMap)
	{
		// get pair parts
		const auto& grabType = grabTypeBufferPublisherPair.first;
		const auto& buffer = grabTypeBufferPublisherPair.second.first;
		auto& publisher = grabTypeBufferPublisherPair.second.second;

		// check if publisher exists
		if (!publisher)
		{
			continue;
		}

		// warn if buffer not full
		if (buffer->status != USER_BUFFER_FULL)
		{
			ROS_WARN_STREAM("buffer " << toStringHex(grabType) << " is not full.");
		}

		// check full if requested
		if (checkBufferFull && buffer->status != USER_BUFFER_FULL)
		{
		// warn if buffer not full
			ROS_WARN_STREAM("buffer " << toStringHex(grabType) << " is not published because it's not full.");
			continue;
		}

		// handle frame
		PublishBuffer(*buffer, publisher, timestamp);

	}
}
