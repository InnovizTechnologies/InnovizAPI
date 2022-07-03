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

#ifndef __INVZ_CONSTANTS_H__
#define __INVZ_CONSTANTS_H__

#include <stdint.h>

/**
* @file invz_constants.h
*/

#define INVZ_MARKER 0xCAFEBABE

#define IP_HEADER_SIZE_BYTES 20
#define UDP_HEADER_SIZE_BYTES 8
#define UDP_MAX_PAYLOAD_SIZE_BYTES (UINT16_MAX - UDP_HEADER_SIZE_BYTES - IP_HEADER_SIZE_BYTES)


/* INVZ3 Related constants */
#define INVZ3_RECORD_MAJOR_VERSION 0
#define INVZ3_RECORD_MINOR_VERSION 0
#define INVZ3_META_FILE_NAME	"/RecordMeta"
#define INVZ3_FRAME_DATA_ATTR_FILE_NAME	"/FrameDataAttr.json"
#define INVZ3_INDEX_FILE_NAME	"/RecordIndex"
#define INVZ3_STREAM_FILE_NAME_LEN 12
#define INVZ3_STREAM_FILE_NAME_FORMAT "/%05d_%03d"
#define INVZ3_FILE_MARKER 0xCABABEEF
#define INVZ3_DATA_ATTR_MAJOR 0
#define INVZ3_DATA_ATTR_MINOR 0


/* INVZ4 Related constants */
#define INVZ4_RECORD_MAJOR_VERSION 0
#define INVZ4_RECORD_MINOR_VERSION 2
#define INVZ4_LIDAR_SN_LENGTH 12
#define INVZ4_CSAMPLE_LIDAR_SERIAL_LENGTH 20
#define INVZ4_META_FILE_NAME	"/RecordMeta"
#define INVZ4_FRAME_DATA_ATTR_FILE_NAME	"/FrameDataAttr.json"
#define INVZ4_INDEX_FILE_NAME	"/RecordIndex"
#define INVZ4_STREAM_FILE_NAME_LEN 12
#define INVZ4_STREAM_FILE_NAME_FORMAT "/%05d_%03d"
#define INVZ4_FILE_MARKER 0xBABEBEEF
#define INVZ4_DATA_ATTR_MAJOR 0
#define INVZ4_DATA_ATTR_MINOR 1

#define INVZ4_CSAMPLE_LIDAR_SERIAL 20

#define DEVICE_NUM_OF_LRFS 4
#define LUT_STRUCT_SIZE 300
#define PIXELS_BUFF_SIZE  (300 * 2 * sizeof(MacroPixelFixed) + sizeof(SummationMacroPixelFixed))
#define FIVE_MB (5*1024*1024)
#define META_CALIB_TABLE_SIZE 32

#define FALCON_FOV_SEGMENTS 1600
#define EAGLE_FOV_SEGMENTS 100

namespace invz
{
	// encryption constants
	static constexpr size_t RSA_CYPHER_SIZE_BYTES = 256;
	static constexpr size_t AES_CYPHER_SIZE_BYTES = 32;
	static constexpr size_t AES_SIGNATURE_SIZE = 16;
	static constexpr size_t AES_IV_SIZE = 16;

	// set root key constants
	static constexpr uint16_t MAX_SEGMENT_SIZE = 72;

}

#endif /* __INVZ_CONSTANTS_H__ */
