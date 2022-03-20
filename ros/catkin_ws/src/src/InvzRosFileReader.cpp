///////////////////////////////////////////////////////////
//  InvzRosFileReader.cpp
//  Implementation of the Class InvzRosFileReader
//  Created on:      06-Mar-2022 12:19:44 PM
//  Original author: tal.levy
///////////////////////////////////////////////////////////

// header
#include "InvzRosFileReader.h"

// std
#include <string>

// project
#include "invz_utils.h"
#include "ros_utils.h"

using std::unique_ptr;
using std::string;
using std::set;
using std::function;

using namespace invz;
using namespace invz_utils;
using namespace ros_utils;

// constants
static constexpr size_t MICROSECONDS_IN_SECOND = 1000000;

IReader* FileReaderInitByParams()
{
    // declare necessary parameters
    string filePath = getRosParam<std::string>("file_path");
    string invzLogPath = getRosParam<std::string>("invz_log_path");
    string configFilePath = getRosParam<std::string>("config_file_path");
    int invzLogLevel = getRosParam<int>("invz_log_level");

    // create invz file reader
    return FileReaderInit(filePath, invzLogPath, invzLogLevel, true, false, 1, configFilePath);
}

/**
 * Creates a FileReader and pass a pointer to base ctor.
 */
InvzRosFileReader::InvzRosFileReader() :
    InvzRosIReader({ FileReaderInitByParams(), closeFileReader }, 0, false)
{

}

size_t InvzRosFileReader::getNofFrames()
{
    size_t nofFrames = 0;
    auto res = m_iReader->GetNumOfFrames(nofFrames);
    if (res.error_code != ERROR_CODE_OK)
    {
        throw std::runtime_error("InvzRosFileReader::getNofFrames - failed to get the number of frames in the recording!");
    }

    return nofFrames;
}

uint64_t InvzRosFileReader::getFrameTimeStamp(uint32_t frameIndex)
{
    // grab frame and return timestamp
    uint32_t frameNumber = 0;
    uint64_t timeStamp = 0;
    GrabFrame(frameNumber, timeStamp, frameIndex);
    return timeStamp;
}

float InvzRosFileReader::getFps()
{
    auto frames = getNofFrames();
    if (frames <= 1)
    {
        throw std::domain_error("InvzRosFileReader::getFps - the recording contains " + std::to_string(frames) + " frames. FPS is undefined!");
    }

    // get timestamp of last and first
    auto firstFrameTimeStamp = getFrameTimeStamp(0);
    auto lastFrameTimeStamp = getFrameTimeStamp(getNofFrames() - 1);

    // assert recording length is positive
    if (lastFrameTimeStamp <= firstFrameTimeStamp)
    {
        throw std::logic_error("last frame timestamp is less or equal to last frame timestamp!");
    }

    // calculate recording length
    auto recordingLengthMicroseconds = (lastFrameTimeStamp - firstFrameTimeStamp);
    auto recordingLengthSeconds = recordingLengthMicroseconds / (float)MICROSECONDS_IN_SECOND;

    // return fps
    return frames / recordingLengthSeconds;

}

bool InvzRosFileReader::GrabAndPublishFrame(uint32_t frameIndex, bool checkSuccess, bool checkBufferFull)
{
    // grab and publish frame
    uint32_t frameNumber = 0;
    uint64_t timestamp = 0;
    
    // grab frame
    bool res = GrabFrame(frameNumber, timestamp, frameIndex);

    // check success if requested
    if (checkSuccess && !res)
    {
        return false;
    }

    // publish 
    PublishFrame(checkBufferFull);

	// return success
	return true;

}
