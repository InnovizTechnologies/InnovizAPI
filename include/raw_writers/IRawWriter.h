// Copyright 2021 Innoviz Technologies
//
// Licensed under the Innoviz Open Dataset License Agreement (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://github.com/InnovizTechnologies/InnovizAPI/blob/main/LICENSE.md
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __IRAW_WRITER_H__
#define __IRAW_WRITER_H__
/**
*	@file IRawWriter.h
*	File contains interface for init and write payload to innoviz files formats.
*
*/

namespace invz {
	
	class INVZ_API IRawWriter
	{
		public:

			/**
			*	@brief Release resources
			*/
			virtual ~IRawWriter() = default;

			/**
			*	@brief Initialize Writer 
			*	@param fileName target file path
			*	@param device_ip deviceIP
			*	@param virtual_channels_ports
			*/
			virtual Result Initialize(std::string fileName, std::string device_ip, std::vector<std::pair<uint32_t, uint16_t>> virtual_channels_ports) = 0;
			/**
			*	@brief Write payload to file
			*	@param packet packet to write
			*/
			virtual Result WritePayload(uint64_t timestamp, uint32_t packet_length, uint8_t* packet, uint16_t port, uint32_t frame_number, bool index) = 0;
			/**
			*	@brief Finalize Writer. Finalize should be performed at the end of writing all packets to file
			*	
			*/
			virtual Result Finalize() = 0;
	};

}

#endif /*__IRAWWRITER_H__*/