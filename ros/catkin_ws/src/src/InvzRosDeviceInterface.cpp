///////////////////////////////////////////////////////////
//  InvzRosDeviceInterface.cpp
//  Implementation of the Class InvzRosDeviceInterface
//  Created on:      06-Mar-2022 12:19:43 PM
//  Original author: tal.levy
///////////////////////////////////////////////////////////

// header
#include "InvzRosDeviceInterface.h"

// std
#include <functional>
#include <stdexcept>

// invz
#include "interface/DeviceApi.h"
#include "invz_utils.h"
#include "ros_utils.h"

using std::unique_ptr;
using std::string;
using std::to_string;
using std::function;
using namespace invz;
using namespace invz_utils;
using namespace ros_utils;

// create invz DeviceInterface from parameters in parameter server
IDevice* DeviceInitByParams()
{
    // get necessary parameters
    string invzLogPath = getRosParam<string>("invz_log_path");
    string configFilePath = getRosParam<string>("config_file_path");
    int invzLogLevel = getRosParam<int>("invz_log_level");

	// create invz device interface
	return DeviceInit(configFilePath, invzLogPath, invzLogLevel);
}

/**
 * Creates a DeviceInterface and passes a pointer to base ctor.
 */
InvzRosDeviceInterface::InvzRosDeviceInterface() :
	InvzRosIReader({ DeviceInitByParams(), closeDeviceInterface }, getRosParam<int>("get_attributes_timeout"), true) {

}

InvzRosDeviceInterface::~InvzRosDeviceInterface()
{
	if (m_recording)
	{
		StopRecording();
	}
}

void InvzRosDeviceInterface::StartListening(bool checkSuccess, bool checkBufferFull)
{
	// get ireader as idevice
	auto& device = dynamic_cast<IDevice&>(*m_iReader);

	// create callback
	auto* base = static_cast<InvzRosIReader*>(this);
	std::function<void(uint32_t*)> callback = [base, checkSuccess, checkBufferFull](uint32_t* frameId) {
		uint32_t frameNumber = *frameId;
		uint64_t timestamp = 0;
		uint32_t frameIndex = 0;
		
		// grab frame
		bool res = base->GrabFrame(frameNumber, timestamp, frameIndex);

		// check success if requested
		if (checkSuccess && !res)
		{
			return false;
		}

		// publish 
		base->PublishFrame(checkBufferFull);

		// return success
		return true;

	};

	// register callback
	device.RegisterFrameCallback(callback);
}

void InvzRosDeviceInterface::StopListening()
{
	// get ireader as idevice
	auto& device = dynamic_cast<IDevice&>(*m_iReader);

	// unregister
	device.UnregisterFrameCallback();

}

void InvzRosDeviceInterface::StartRecording(const string& path)
{
	// get ireader as idevice
	auto& device = dynamic_cast<IDevice&>(*m_iReader);

	// start recording
	auto res = device.StartRecording(path);

	// check result
	if (res.error_code != ERROR_CODE_OK)
	{
		throw std::runtime_error("failed to start recording - error code: " + to_string(res.error_code) +
			", error_message: " + res.error_message);
	}

	// set recordings
	m_recording = true;

}

void InvzRosDeviceInterface::StopRecording()
{
	// get ireader as idevice
	auto& device = dynamic_cast<IDevice&>(*m_iReader);

	// start recording
	auto res = device.StopRecording();

	// check result
	if (res.error_code != ERROR_CODE_OK)
	{
		throw std::runtime_error("failed to stop recording - error code: " + to_string(res.error_code) +
			", error_message: " + res.error_message);
	}

	// set recordings
	m_recording = false;
	
}
