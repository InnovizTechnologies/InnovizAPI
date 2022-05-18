///////////////////////////////////////////////////////////
//  PY_FileReader.h
//  Implementation of the Class PY_FileReader
//  Created on:      09-Aug-2021 1:35:51 PM
//  Original author: sarah.foox
///////////////////////////////////////////////////////////

#ifndef  __PY_FILE_READER_H__
#define __PY_FILE_READER_H__

#include "../common_files/PY_CommonUtils.h"
#include "interface/FileReaderApi.h"
#include "interface/ConfigApi.h"
#include <set>

class PY_FileReader
{
public:
	size_t NumOfFrames = -1;
	uint32_t FileFormat;
	PY_FileReader(std::string filepath, uint32_t log_severity, bool check_pixel_validity, uint8_t num_of_cores, std::string config_filepath);
	~PY_FileReader();
	PyGrabFrameResult GetFrame(int frame_index, std::vector<FrameDataAttributes> frame_types, int user_max_packets = std::numeric_limits<int>::max());
	py::tuple GetPacket(const std::set<uint32_t>& virtualChannels);
	py::tuple GetDeviceMeta();
	py::list GetFrameDataAttrs();
	void RegisterTapsCallback(std::function<void(PyTapHandler&)> callback);
	void UnregisterTapsCallback();
	void GrabTaps(int frame_index);
	void RegisterLogsCallback(std::function<void(PyLogHandler&)> callback);
	void UnregisterLogsCallback();
	void GrabLogs(int frame_index);
	bool SeekFrame(uint32_t frame_index);

private:
	std::unique_ptr<invz::IReader> m_fr= nullptr;
	invz::byte m_packetBuffer[UINT16_MAX];
	std::vector<invz::FrameDataAttributes> m_frameDataAttrs;
	std::map<GrabType, uint8_t*> m_allBuffers;

	std::function<void(invz::TapEventData*)> tapCallbackCpp;
	std::function<void(PyTapHandler&)> tapCallbackPy;
	std::function<void(invz::RuntimeLogEventData*)> logCallbackCpp;
	std::function<void(PyLogHandler&)> logCallbackPy;
	
	static constexpr uint32_t maxUserBuffers = 100;
};
#endif // !defined __PY_FILE_READER_H__
