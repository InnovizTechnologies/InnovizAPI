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

#ifndef __CONFIG_API_H__
#define __CONFIG_API_H__

/*=========================================================================

Program:   Device Interface
Module:    ConfigApi.h
=========================================================================*/

/**
* @file ConfigApi.h
* @authors Chen Vivnate
* @date 17-03-2019
* @brief File containing File Reader Interface decleration.
*/


/*! \mainpage Home
*
* \section intro_sec Introduction
*
*/
#include <string>

#include "common_includes/invz_status.h"

#define INVZ_CONFIG_GET_FRAME_ATTR_DATA_MAX_SIZE 100

namespace invz {

	namespace config {

		INVZ_API Result GetFrameDataAttrFromConfig(std::string filepath, FrameDataAttributes dst[INVZ_CONFIG_GET_FRAME_ATTR_DATA_MAX_SIZE], size_t &actualSize);

	}

}
#endif /*__CONFIG_API_H__*/