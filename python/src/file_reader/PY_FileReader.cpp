///////////////////////////////////////////////////////////
//  PY_FileReader.cpp
//  Implementation of the Class PY_FileReader
//  Created on:      09-Aug-2021 1:35:51 PM
//  Original author: sarah.foox
///////////////////////////////////////////////////////////

#include "PY_FileReader.h"


PY_FileReader::PY_FileReader(std::string filepath, uint32_t log_severity, bool check_pixel_validity, uint8_t num_of_cores, std::string config_filepath)
{
	invz::Result result;
	// initalize reader interface
	m_fr.reset(invz::FileReaderInit(filepath, "", log_severity, true, check_pixel_validity, num_of_cores, config_filepath));

	/* Get number of frames in file */
	result = m_fr->GetNumOfFrames(NumOfFrames);
	CheckResult(result);

	result = m_fr->GetFileFormat(FileFormat);
	CheckResult(result);

	/* Get frame data attributes from file */
	invz::FrameDataAttributes attr[INVZ_CONFIG_GET_FRAME_ATTR_DATA_MAX_SIZE];
	size_t attr_count = INVZ_CONFIG_GET_FRAME_ATTR_DATA_MAX_SIZE;
	result = m_fr->GetFrameDataAttributes(attr, attr_count);
	CheckResult(result);

	/* Get frame data attributes */
	for (int i = 0; i < attr_count; i++)
	{
		m_frameDataAttrs.push_back(attr[i]);
	}

	tapCallbackCpp = [&](invz::TapEventData* th) -> void {
		/* Acquire GIL before calling Python code */
		py::gil_scoped_acquire acquire;
		const invz::DataPoint* dp;
		m_fr->GetDPById(th->parameter_id, dp);
		PyTapHandler py_th(*th, dp);
		tapCallbackPy(py_th);
	};

	logCallbackCpp = [&](invz::RuntimeLogEventData* th) -> void {
		/* Acquire GIL before calling Python code */
		py::gil_scoped_acquire acquire;
		PyLogHandler py_th(*th);
		logCallbackPy(py_th);
	};
}

PY_FileReader::~PY_FileReader()
{
	if (m_fr)
		invz::FileReaderClose(m_fr.release());
}

PyGrabFrameResult PY_FileReader::GetFrame(int frame_index, std::vector<FrameDataAttributes> frame_types, int user_max_packets)
{

	invz::Result result;
	PyGrabFrameResult ret;

	for (auto& dataAttrs : frame_types)
	{
		CheckAttribute(dataAttrs, m_frameDataAttrs);		//check all types are fully defined
	}

	/* Allocate frame data buffers */
	FrameDataUserBuffer userBuffers[maxUserBuffers];
	TypeMeta currentType;
	bool measurment_type = false;
	int count = 0;
	for (auto& dataAttrs : frame_types)
	{
		if (m_allBuffers.find(dataAttrs.known_type) == m_allBuffers.end())
		{
			/* Add user buffer to send to API */
			m_allBuffers[dataAttrs.known_type] = new uint8_t[dataAttrs.nbytes()];
		}
		invz::FrameDataUserBuffer buffer;
		userBuffers[count].dataAttrs = dataAttrs;
		userBuffers[count].dataBuffer = m_allBuffers[dataAttrs.known_type];
		if (dataAttrs.known_type == GrabType::GRAB_TYPE_PC_PLUS || dataAttrs.known_type == GrabType::GRAB_TYPE_TRACKED_OBJECTS_SI
			|| dataAttrs.known_type == GRAB_TYPE_DETECTIONS_SI)
			userBuffers[count].handle_endianess = true;


		count++;

	}

	uint32_t frameNumber;
	uint64_t timestamp;
	bool success;
	result = m_fr->GrabFrame(userBuffers, count, frameNumber, timestamp, frame_index, user_max_packets);
	success = result.error_code == ERROR_CODE_OK;

	for (auto& userBuffer : userBuffers)
	{
		if (userBuffer.dataAttrs.known_type != GRAB_TYPE_UNKOWN)
		{
			if (userBuffer.status == USER_BUFFER_FULL)
			{
				ret.results[py::str(GetBufferName(userBuffer.dataAttrs))] = Get1DArray(userBuffer.dataAttrs.length, GetTypeMeta(FileFormat, userBuffer.dataAttrs.typeMajor, userBuffer.dataAttrs.typeMinor));
				auto ptr = (invz::byte*)(((py::array)ret.results[py::str(GetBufferName(userBuffer.dataAttrs))]).request().ptr);
				std::copy((invz::byte*)userBuffer.dataBuffer, (invz::byte*)(userBuffer.dataBuffer + userBuffer.dataAttrs.nbytes()), ptr);
				if (userBuffer.dataBuffer)
				{
					delete userBuffer.dataBuffer;
					m_allBuffers.erase(userBuffer.dataAttrs.known_type);
				}


			}
			else
				ret.results[py::str(GetBufferName(userBuffer.dataAttrs))] = py::none();
		}

	}
	ret.frame_number = frameNumber;
	ret.success = success;
	ret.timestamp = timestamp;
	return ret;
}

bool PY_FileReader::SeekFrame(uint32_t frame_index)
{
	invz::Result result;

	result = m_fr->seekFrame(frame_index);

	return result.error_code == ErrorCode::ERROR_CODE_OK;

}

py::tuple PY_FileReader::GetPacket(const std::set<uint32_t>& virtualChannels)
{
	invz::Result result;
	size_t packetSize = 0;
	uint64_t timestamp = -1;
	uint16_t rxPort = -1;
	uint32_t rxChannel = -1;

	// get block into getPacketBuffer
	result = m_fr->GetPacket(m_packetBuffer, UINT16_MAX, packetSize, timestamp, rxPort, rxChannel, virtualChannels);
	if (!result.error_code) {
		// validate packet size
		if (packetSize > (size_t)UINT16_MAX)
			throw std::runtime_error("Packet size exceed UINT16_MAX upon successfull get block");

		// allocate nd array according to packet size and return it
		auto ret = Get1DArray(packetSize, GetTypeMeta<invz::byte>());
		auto ptr = (invz::byte*)(ret.request().ptr);
		std::copy(m_packetBuffer, m_packetBuffer + packetSize, ptr);
		return py::make_tuple(true, rxPort, rxChannel, timestamp, ret);
	}
	else {
		py::none none;
		return py::make_tuple(false, none, none, none, none);
	}
}

py::tuple PY_FileReader::GetDeviceMeta()
{

	invz::Result result;
	py::none none;
	py::tuple ret = py::make_tuple(none);
	return ret;
}

py::list PY_FileReader::GetFrameDataAttrs()
{

	py::list ret;

	for (auto& data_attrs : m_frameDataAttrs)
	{
		ret.append(data_attrs);
	}
	return ret;
}

void PY_FileReader::RegisterTapsCallback(std::function<void(PyTapHandler&)> callback)
{
	invz::Result result;
	tapCallbackPy = std::bind(callback, std::placeholders::_1);
	result = m_fr->RegisterTapsCallback(tapCallbackCpp);
	CheckResult(result);
}

void PY_FileReader::UnregisterTapsCallback()
{
	invz::Result result;
	result = m_fr->UnregisterTapsCallback();
	CheckResult(result);
}

void PY_FileReader::GrabTaps(int frame_index)
{
	invz::Result result;
	result = m_fr->GrabTaps(frame_index);
	CheckResult(result);
}

void PY_FileReader::RegisterLogsCallback(std::function<void(PyLogHandler&)> callback)
{
	invz::Result result;
	logCallbackPy = std::bind(callback, std::placeholders::_1);
	result = m_fr->RegisterLogsCallback(logCallbackCpp);
	CheckResult(result);
}

void PY_FileReader::UnregisterLogsCallback()
{
	invz::Result result;
	result = m_fr->UnregisterLogsCallback();
	CheckResult(result);
}

void PY_FileReader::GrabLogs(int frame_index)
{
	invz::Result result;
	result = m_fr->GrabLogs(frame_index);
	CheckResult(result);
}

