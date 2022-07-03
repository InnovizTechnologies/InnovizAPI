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

#ifndef __DATAPOINT_H__
#define __DATAPOINT_H__

/**
* @file Datapoint.h
* @authors Niv Sluzki, Mark Kolodkin
* @date 08-05-2018
* @brief File containing innoviz Api.
*/

#include "common_includes/invz_types.h"
#include "ndarray/ndarray.h"

namespace invz {

	typedef uint8_t byte;

#pragma pack(push, 1)
	// A Data Points is a data sent by the target which can be :
	// A parameter, an indicator, a tap, a timer or a telemetry.
	// Note that the data points list, name and id are defined when device is first connected
	
	/**
	* @brief A Data Point is a data sent by the target which can be:
	*
	* - Parameter: regular parameter which can be read using "get" command
	*              or changed using "set" command, assuming the actively
	*              connected user has sufficient permissions
	*
	* - Indicator: A read-only parameter, it can not be changed
	*
	* - Tap: A tap is a part of the debug infrastructure. It allows dumping
	*        an inner data structure, like inner temporary calculations. A tap
	*        can be enabled or disabled. If disabled, nothing is sent out and there
	*        is minimal impact on the data flow. If enabled, the data is copied to a
	*        temporary buffer and sent out every frame in the frame report. Tap data point
	*		 holds the latest value send to it. 
	*		 If tap is disabled and was never transmitted, data is pulled once from device.
	*
	* - Timer: Tap which indicates time measurements. It allows measuring time to
	*          verify real time
	*
	* - Telemetry: a tap which is enabled only on one frame out of a given period. It allows
	*             monitoring some values periodically
	*
	*/
	struct INVZ_API DataPoint
	{
		/**
		* @brief Default contructor
		*/
		DataPoint() : id(-1) { }

		uint32_t id; /*!< Unique ID of the data point. */

		std::string name; /*!< The variable name of the data point */

		std::string dptype; /*!< data point type: tap / param / indicator / timer / telemetry */

		std::string type; /*!< numerical electronic type of the Data Point:
						  int8_t, uint8_t, int16_t, uint16_t, int32_t, uint32_t, char(string), float, double */

		std::string container; /*!< Group name of the parameter */

		size_t size; /*!< Total size in bytes of the parameter data */

		int32_t stride; /*!< Stride size */

		uint32_t lrfs = 1;	/*!< third dimension - number of lrfs */
		
		int32_t rows = 1; /*!< Stride size */

		int32_t columns = 1; /*!< Stride size */		

		std::string desc; /*!< Description of the datapoint */

		std::string rasdescription; /*!< Raw description of the DataPoint received */

		std::vector<byte> value; /*!< Store values for taps */

		ndarray::DType *dtype = nullptr;

		bool is_tapped = false;

		bool is_virtual = false;

		ELoginLevel read_access_level;
		
		ELoginLevel write_access_level;
		/**
		* @brief Get single item size in bytes according to type value
		*/
		size_t GetItemSize() const;

		/**
		* @brief Returns number of elements in data point
		*/
		size_t Length() const;

		/**
		* @brief Get dimenstions of Data Points
		*/
		void GetDims(size_t &rows, size_t &cols);
		
		/**
		* @brief Parse the raw binary format description
		* to fill all the fields of the DataPoint class
		* structure
		*/
		void ParseDescription();

		/**
		* @brief Show its own information
		*/
		void PrintSelf();

	};
#pragma pack(pop)

	INVZ_API ndarray::DType GetDTypeDemo();

	INVZ_API ndarray::NDArray* GetNDArrayDemo();

}
#endif /*__DATAPOINT_H__*/