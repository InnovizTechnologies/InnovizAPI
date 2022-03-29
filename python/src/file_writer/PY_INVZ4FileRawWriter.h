///////////////////////////////////////////////////////////
//  PY_INVZ4FileRawWriter.h
//  Implementation of the Class PY_INVZ4FileRawWriter
//  Created on:      09-Aug-2021 1:39:16 PM
//  Original author: sarah.foox
///////////////////////////////////////////////////////////

#ifndef __PY_INVZ4_FILE_RAW_WRITER_H__
#define __PY_INVZ4_FILE_RAW_WRITER_H__

#include "../common_files/PY_CommonUtils.h"

namespace invz 
{
	class IRawWriter;
}

class PY_INVZ4FileRawWriter
{
public:
	PY_INVZ4FileRawWriter(std::string file_name, std::string device_ip, py::list virtual_channels_ports);
	~PY_INVZ4FileRawWriter();
	py::bool_ WritePayload(uint64_t timestamp, py::array payload, uint16_t port, int32_t frame_number, uint32_t packet_length);
	py::bool_ Finalize();

private:
	std::unique_ptr<invz::IRawWriter> m_fw;
	IRawWriter* buildWriter();
	std::vector<std::pair<uint32_t, uint16_t>> convertToArray(py::list l);
};
#endif // __PY_INVZ4_FILE_RAW_WRITER_H__
