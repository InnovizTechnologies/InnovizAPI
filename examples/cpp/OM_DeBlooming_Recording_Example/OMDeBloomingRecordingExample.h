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

///////////////////////////////////////////////////////////
//  OMDeBloomingRecordingExample.h
//  Implementation of the Class OMDeBloomingRecordingExample
//  Created on:      07-Oct-2020 1:13:23 AM
//  Original author: julia.sher
///////////////////////////////////////////////////////////

#ifndef _OM_FILE_DEMO_H
#define _OM_FILE_DEMO_H

#include "interface/FileReaderApi.h"
#include "../common/ExampleTypes.h"
#include <string>

class OMDeBloomingRecordingExample
{

public:
	explicit OMDeBloomingRecordingExample(std::string om_file_path);
	virtual ~OMDeBloomingRecordingExample();

	virtual void GetNumberOfFrames();
	virtual void GetFileMetaData();
	virtual void ReadFrames();

private:
	void initUserBuffers(size_t buffersCount, std::vector<invz::FrameDataUserBuffer>& userBuffers, std::vector<invz::FrameDataAttributes>& attributes);
	uint32_t findTypeIndex(invz::GrabType type);
	
	std::unique_ptr<invz::IReader> m_reader = nullptr;
	std::vector<invz::FrameDataUserBuffer> m_userBuffers;

};
#endif // _OM_FILE_DEMO_H
