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
* @file IWriterFactory.h
* @authors Julia Sher
* @date 02-05-2019
* @brief
*/

#ifndef __IWRITER_FACTORY_H__
#define __IWRITER_FACTORY_H__

#include "IWriter.h"

namespace invz {

	/**
	* @brief Initialize FileWriter Interface
	*
	* @param f_name .invz file path
	*/

	INVZ_API IWriter* FileWriterInit(std::string filename, std::string device_ip, uint32_t file_format,
		std::vector<std::pair<uint32_t, uint16_t>> virtual_channels_ports, std::vector<invz::FrameDataAttributes> frame_data_attr,
		std::string api_log_filename = "", uint32_t api_log_severity_level = 3);

	/**
	* @brief Close FileWriter Interface
	*
	* @param writer_interface Pointer to the FileWriter
	*/
	INVZ_API void FileWriterClose(IWriter* writer_interface);

}
#endif /*__IWRITER_FACTORY_H__*/