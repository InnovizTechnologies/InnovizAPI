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

#ifndef __INVZ_RECORD_H__
#define __INVZ_RECORD_H__

/**
 * @file InvzRecord.h
 */

#include "common_includes/invz_constants.h"
#include "common_includes/invz_types.h"

using namespace invz;

#pragma pack(push,1)
struct InvzRecordHeader
{
	uint32_t marker = UINT32_MAX;
	uint32_t major = UINT32_MAX;
	uint32_t minor = UINT32_MAX;
};

struct InvzStreamAttributes
{
#ifdef NEW_INVZ3_VER
	uint16_t port;
	uint16_t channelType;
	uint32_t deviceType;
#else
	uint16_t port;
	uint8_t channelType;
	uint8_t deviceType;
#endif
};
struct Invz3RecordMeta : InvzRecordHeader
{
	uint32_t deviceIp						= 0;
	uint32_t deviceType						= UINT32_MAX;
	uint64_t startTime						= 0;
	uint64_t recordingLength				= 0;
	uint32_t frameCount						= 0;
	uint32_t pixelsPerFrame					= UINT32_MAX;
	uint32_t recordedStreamCount			= 0;
	InvzStreamAttributes* recordedStreams	= nullptr;
	vector3* directions						= nullptr;
	uint32_t* timings						= nullptr;
	uint32_t crc							= 0;
};

struct Invz4RecordMeta
{
	InvzRecordHeader header;
	uint32_t deviceIp = 0;
	uint32_t deviceType = UINT32_MAX;
	uint64_t startTime = 0;
	uint64_t recordingLength = 0;
	uint32_t frameCount = 0;
	uint32_t recordedStreamCount;
	InvzStreamAttributes* recordedStreams = nullptr;
	uint32_t crc;
};


struct InvzRecordIndexLine
{
	uint32_t frameNumber	= UINT32_MAX;
	uint64_t timestamp		= 0;
	uint64_t* offsets		= nullptr;
};

#pragma pack(pop)
#endif /* __INVZ_RECORD_H__ */