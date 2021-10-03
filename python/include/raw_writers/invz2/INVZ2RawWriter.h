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
*	@file INVZ2RawWriter.h
*	File contains interface used for initializing and wiriting file in INVZ_2 format.
*
*/

#ifndef __INVZ_2_RAW_WRITER_H__
#define __INVZ_2_RAW_WRITER_H__

#include "raw_writers/IRawWriter.h"

//PRO EOL
//using namespace invz;
//
//class INVZ_API INVZ2RawWriter : public IRawWriter {
//public:
//	/* Interface*/
//	Result Initialize(std::string fileName, std::string device_ip, std::vector<std::pair<uint32_t, uint16_t>> virtual_channels_ports);
//	Result WritePayload(uint64_t timestamp, uint32_t packet_length, uint8_t* packet, uint16_t port, uint32_t frame_number, bool index);
//
//	Result Finalize();
//
//	INVZ2RawWriter();
//	~INVZ2RawWriter();
//
//protected:
//};

#endif /*__INVZ_2_RAW_WRITER_H__*/