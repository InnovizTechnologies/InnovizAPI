///////////////////////////////////////////////////////////
//  InvzRosDeviceInterface.h
//  Implementation of the Class InvzRosDeviceInterface
//  Created on:      06-Mar-2022 12:19:43 PM
//  Original author: tal.levy
///////////////////////////////////////////////////////////

#if !defined(EA_3E2B3508_07D4_4c69_98EE_449A8FBFA825__INCLUDED_)
#define EA_3E2B3508_07D4_4c69_98EE_449A8FBFA825__INCLUDED_

// std
#include <string>

// project
#include "InvzRosIReader.h"

/**
 * Wraps a DeviceInterface for the specific use of Innoviz Ros
 */
class InvzRosDeviceInterface : private InvzRosIReader
{
public:
	// class ctor and dtor
	InvzRosDeviceInterface();
	~InvzRosDeviceInterface();
	
	// start receiving frame from the device and publishing them
	void StartListening(bool checkSuccess, bool checkBufferFull);

	// stop receiving frame from the device and publishing them
	void StopListening();

	// start recording the device's output to file
	void StartRecording(const std::string& path);

	// stop recording the device's output to file
	void StopRecording();

private:
	// whether the device is currently recording or not
	bool m_recording = false;

};

#endif // !defined(EA_3E2B3508_07D4_4c69_98EE_449A8FBFA825__INCLUDED_)
