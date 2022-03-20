// Copyright 2021 Innoviz Technologies
//
// Licensed under the Innoviz Open Dataset License Agreement (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://github.com/InnovizTechnologies/InnovizAPI/blob/master/LICENSE.md
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
*	@file INVZ4RawWriter.h
*	File contains interface used for initializing and wiriting file in INVZ_4_3 format.
*
*/

#ifndef __INVZ_4_RAW_WRITER_H__
#define __INVZ_4_RAW_WRITER_H__


#include <mutex>
#include <fstream>

#include "protocols/ACP.h"
#include "invz_file/InvzRecord.h"
#include "../IRawWriter.h"
#include <unordered_map>

using namespace invz;

class INVZ_API INVZ4RawWriter : public IRawWriter {
public:
	/* Interface*/
	Result Initialize(std::string fileName, std::string device_ip, std::vector<std::pair<uint32_t, uint16_t>> virtual_channels_ports);
	Result WritePayload(uint64_t timestamp, uint32_t packet_length, uint8_t* packet, uint16_t port, uint32_t frame_number, bool index);
	
	Result Finalize();
	
	INVZ4RawWriter();
	~INVZ4RawWriter();

private:

	void InitFileExt(std::ofstream &file, std::string path, uint8_t* file_data, size_t file_data_length);
	void GeneraDataAttrFile(std::vector<FrameDataAttributes> frameDataAttr, std::string filepath);
	std::string m_filename = "";
	uint64_t m_startTime;
	uint64_t m_endTime;

	uint32_t m_frameCount = 0;
	uint32_t m_streamCount = 0;
	
	std::ofstream m_indexFile;
	std::ofstream m_metaFile;

	std::vector<InvzRecordIndexLine> m_indexTable;

	std::unordered_map<uint16_t, std::ofstream> m_portToFileMap;
	std::unordered_map<uint16_t, uint32_t> m_portToStreamIndex;
	std::mutex m_fileAccess;

};


#endif /*__INVZ_4_RAW_WRITER_H__*/