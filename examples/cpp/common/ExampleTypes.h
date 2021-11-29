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
//  DemoTypes.h
//  Implementation of common Demo types
//  Created on:      07-Oct-2020 1:13:25 AM
//  Original author: julia.sher
///////////////////////////////////////////////////////////

#ifndef _EXAMPLE_TYPES_H
#define _EXAMPLE_TYPES_H

#include <cstdint>

enum class EOMDataType : uint32_t
{
	Ref0 = 0,
	Ref1 = 1,
	Ref2 = 2,
	Sum0 = 3,
	TotalTypes = 4
};

enum class ECMDataType : uint32_t
{
	Objects = 0,
	Objscts_si = 1 // 
};

struct DemoConfig
{
	static constexpr uint32_t max_num_of_buffers = 100;
	static constexpr uint32_t frames_to_read = 20;
};

#endif // _EXAMPLE_TYPES_H
