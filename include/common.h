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

#ifndef __COMMON_H__
#define __COMMON_H__

/** @file common.h
*/

// Disable warning for using non-standard C++ extension: zero-sized arrays in a struct.
// For example:
// struct {
//     char data[0];
// };
#if defined(_WIN32)
#pragma warning(disable : 4200)

// Disable warning on not DLL exporting all the STL classes/structs that are being
// used on exported classes/structs.
#pragma warning(disable : 4251)
#endif

#ifdef INVZ_API_EXPORT
#if defined(_WIN32)
#define INVZ_API __declspec(dllexport)
#elif __GNUC__ >= 4
#define INVZ_API __attribute__ ((visibility("default")))
#define __stdcall 
#else
#error Unsupported platform
#endif
#else
#ifdef INVZ_API_IMPORT
#if defined(_WIN32)
#define INVZ_API _declspec(dllimport)
#elif __GNUC__ >= 4
#define INVZ_API 
#define __stdcall 
#else
#error Unsupported platform
#endif
#else
#define INVZ_API
#define __stdcall 
#endif
#endif

#include <cstring>
#include <string.h>
#include <string>

namespace invz {

	/** @brief Defines possible result return codes for Result Object
	*	@enum ErrorCode
	*/
	enum ErrorCode
	{
		ERROR_CODE_OK,
		ERROR_CODE_GENERAL,
		ERROR_CODE_CONNECTION,
		ERROR_CODE_INVALID_DATA_POINT,
		ERROR_CODE_FILE_ERROR,
		ERROR_CODE_INVALID_FRAME,
		ERROR_CODE_INVALID_INPUT,
		ERROR_CODE_DEVICE_ERROR,
		ERROR_CODE_NOT_SUPPORTED,
		ERROR_CODE_BUFFER_TOO_SMALL
	};

	struct INVZ_API Result
	{
		ErrorCode error_code = ERROR_CODE_OK;
		std::string error_message;
	};
}
#endif