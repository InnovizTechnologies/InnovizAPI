///////////////////////////////////////////////////////////
//  PY_INVZ4FileRawWriter.cpp
//  Implementation of the Class PY_INVZ4FileRawWriter
//  Created on:      09-Aug-2021 1:39:16 PM
//  Original author: sarah.foox
///////////////////////////////////////////////////////////

#include "PY_INVZ4FileRawWriter.h"
#include "raw_writers/invz4/INVZ4RawWriter.h"


 IRawWriter* PY_INVZ4FileRawWriter::buildWriter()
 {

	 IRawWriter* writer = new INVZ4RawWriter();
	 return writer;
 }

 std::vector<std::pair<uint32_t, uint16_t>> PY_INVZ4FileRawWriter::convertToArray(py::list l)
 {

	 // allocate ret
	 std::vector<std::pair<uint32_t, uint16_t>> ret;
	 for (int i = 0; i < l.size(); i++) {
		 py::list li = (py::list)l[i];
		 if (li.size() != 2)
			 throw std::runtime_error("list[" + std::to_string(i) + " is expected to be a list of size of 2.");

		 // updat ret
		 ret.push_back(std::make_pair(py::cast<std::uint32_t>(li[0]), py::cast<std::uint16_t>(li[1])));
	 }
	 return ret;
 }

 PY_INVZ4FileRawWriter::PY_INVZ4FileRawWriter(std::string file_name, std::string device_ip, py::list virtual_channels_ports)
 {
	 m_fw.reset(buildWriter());
	 m_fw->Initialize(file_name, device_ip, convertToArray(virtual_channels_ports));
 }

 PY_INVZ4FileRawWriter::~PY_INVZ4FileRawWriter()
 {
	 IRawWriter* writer = m_fw.release();
	 delete writer;
 }

 py::bool_ PY_INVZ4FileRawWriter::WritePayload(uint64_t timestamp, py::array payload, uint16_t port, int32_t frame_number, uint32_t payload_size)
 {
	 int i = 0;
	 uint32_t packet_length = (payload_size > 0) ? payload_size : static_cast<uint32_t>(payload.size());
	 Result res = m_fw->WritePayload(timestamp, packet_length, (uint8_t*)(payload.request().ptr), port, frame_number, frame_number > 0);
	 return res.error_code == ERROR_CODE_OK;
 }

 py::bool_ PY_INVZ4FileRawWriter::Finalize()
 {
	 Result res = m_fw->Finalize();
	 return res.error_code == ERROR_CODE_OK;
 }