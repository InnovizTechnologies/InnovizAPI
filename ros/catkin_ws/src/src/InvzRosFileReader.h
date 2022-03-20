///////////////////////////////////////////////////////////
//  InvzRosFileReader.h
//  Implementation of the Class InvzRosFileReader
//  Created on:      06-Mar-2022 12:19:44 PM
//  Original author: tal.levy
///////////////////////////////////////////////////////////

#if !defined(EA_7AC4E9A3_3621_4585_8C1E_A455E3169B71__INCLUDED_)
#define EA_7AC4E9A3_3621_4585_8C1E_A455E3169B71__INCLUDED_

// std
#include <string>
#include <set>

// innoviz
#include "interface/FileReaderApi.h"

// project
#include "InvzRosIReader.h"

/**
 * Wraps a FileReader for the specific use of Innoviz Ros
 */
class InvzRosFileReader : private InvzRosIReader
{
public:
	// class ctor
	InvzRosFileReader();

	// get the number of frames in the recording
	size_t getNofFrames();

	// get recording frame rate: frames / recording_time
	float getFps();

	// grab frame from the recoding and publish it
	bool GrabAndPublishFrame(uint32_t frameIndex, bool checkSuccess, bool checkBufferFull);

private:
	// get the time stamp of a frame
	uint64_t getFrameTimeStamp(uint32_t frameIndex);

};

#endif // !defined(EA_7AC4E9A3_3621_4585_8C1E_A455E3169B71__INCLUDED_)
