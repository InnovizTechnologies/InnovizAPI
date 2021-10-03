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

#ifndef __INVZ_STATUS_H__
#define __INVZ_STATUS_H__

#include <stdint.h>

#include "invz_types.h"

/**
* @file invz_status.h
*/

#define CS_IGNORE

#define RESULT_ERROR(code,message)\
					result.error_code=code;				\
					result.error_message= message;	\
					return result;
#define RESULT_START Result result; \
					 try{
#define RESULT_OBSOLETE Result result;				\
						result.error_code= ERROR_CODE_NOT_SUPPORTED;				\
						result.error_message ="this method is no longer supported";				\
						return result;
#define RESULT_END } \
					 catch(std::exception ex){		\
							RESULT_ERROR(ERROR_CODE_GENERAL,ex.what())	\
						}								\
						return result;

#define ERROR_MESSAGE(code) ("ErrorCode " + std::to_string(code) + ": " + invz::StatusStr[code])




namespace invz {
	
	typedef INVZ_API uint32_t Status;

	enum INVZ_API StatusCode : uint32_t
	{
		STATUS_OK = 0x00000000,
		STATUS_BUFFER_TOO_SMALL,
		STATUS_NULL_POINTER_DETECTED,
		STATUS_CODE_TIMEOUT,  // STATUS_TIMEOUT is used by winnt.h
		STATUS_METHOD_NOT_SUPPORTED,
		STATUS_TCP_CONNECTION_ERROR,

		/* TLV related error */
		STATUS_TLV_SIZE_ZERO,
		STATUS_ILLEGAL_TLV_LENGTH,

		/* Always last */
		STATUS_GENERAL_ERROR,
	};

	 static CS_IGNORE const std::string StatusStr[] =
	{
		"Status OK",
		"Buffer too small",
		"NULL pointer detected",
		"Timout reached",
		"Method is not supported",
		"TCP connection error",

		/* TLV related error */
		"No TLVs detected",
		"Illegal TLV length",

		/* Always last */
		"General error"
	};



	CS_IGNORE constexpr char err_invalid_format_message[] = "File format is invalid";
	CS_IGNORE constexpr char err_invalid_indx_format_message[] = "Index file format is invalid";
}


#endif /* __INVZ_STATUS_H__ */