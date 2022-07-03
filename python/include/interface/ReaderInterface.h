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

#ifndef __READER_INTERFACE_H__
#define __READER_INTERFACE_H__
/*=========================================================================
Program:   Reader Interface
Module:    ReaderInterface.h
=========================================================================*/

/**
* @file ReaderInterface.h
* @authors Chen Vivnate
* @date 17-02-2019
* @brief File containing Reader Interface decleration.
*/


/*! \mainpage Home
*
* \section intro_sec Introduction
*
*/
#include "../common_includes/invz_types.h"
#include "../common_includes/invz_status.h"
#include "../common_includes/invz_constants.h"
#include "data_point/Datapoint.h"
#include <functional>
#include <set>
#include <limits>

namespace invz {

	class INVZ_API IReader{
	public:
		
		/* TODO: Change the using of results */
		/**
		* @brief Getter for number of pixels per frame
		*/
		virtual Result GetFileFormat(uint32_t& file_format) = 0;

		/**
		* @brief Getter for number of pixels per frame
		*/
		/**
		* @brief return device metadata
		*
		* @param device_meta will be copied into
		*/
		virtual Result GetDeviceMetadata(DeviceMeta& device_meta) = 0;
		
		/**
		* @brief Getter for number of frames in current file
		*/
		virtual Result GetNumOfFrames(size_t& number_of_frames) = 0;

		/**
		* @brief return next packet by timestamp
		*
		* @param packet packet pointer to read to
		* @param packet_max_size max packet size.
		* @param packet_len actual packet size.
		* @param timestamp timestamp of current block (relevant for invz3)
		* @param port port of current packet (relevant for invz3)
		*/
		virtual Result GetPacket(byte *packet, size_t packet_max_size, size_t& packet_size, uint64_t& timestamp, uint16_t &port, uint32_t& channel_type, const std::set<uint32_t>& virtualChannels) = 0;

		/**
		* @brief return relevant frame data from device configuration
		*
		* @param frame_data_attrs frame data attributes array pointer allocated by user
		* @param frame_data_attrs_count length of frame_data_attrs array updated by actual length.
		*/
		virtual Result GetFrameDataAttributes(FrameDataAttributes* frame_data_attrs, size_t& frame_data_attrs_count, bool wait_for_device = true) = 0;


		/**
		* @brief set current frame in file
		*
		* @param frame_index the frame_index to set the current frame to
		*/
		virtual Result seekFrame(uint32_t frame_index) = 0;

		/**
		* @brief return frame related data
		*
		* @param frame_data_buffers user buffer frame data will be written to
		* @param frame_data_count number of frame data buffers
		* @param frame_number frame number of collected data
		* @param frame_index in case of reading from file, frame index of frame in file to collect
		*/
		
		virtual Result GrabFrame(FrameDataUserBuffer* frame_data_buffers, uint32_t frame_data_count, uint32_t& frame_number, uint64_t& timestamp, uint32_t frame_index = UINT32_MAX, int user_max_packets = std::numeric_limits<int>::max()) = 0;
		
		/**
		* @brief activate\deactivate buffer from being collected
		*
		* @param attr buffer type
		* @param activate true=activate
		*/
		virtual Result ActivateBuffer(FrameDataAttributes &attr, bool activate) = 0;

		/**
		* @brief Registers a call back to get notified when on TAP event
		*
		* @param callback Function delegate reference
		*/
		virtual Result RegisterTapsCallback(std::function<void(TapEventData*)> callback) = 0;
		virtual Result RegisterTapsCallback(TapEventCallback callback) = 0;

		/**
		* @brief Unregister callback if was registered, else does nothing.
		*/
		virtual Result UnregisterTapsCallback() = 0;

		/**
		* @brief return frame related taps
		*
		* @param frame_index in case of reading from file, frame index of frame in file to collect
		*/
		virtual Result GrabTaps(uint32_t frame_index) = 0;

		/** @brief Get data point from API by mapping index.
		*
		* @return Result OK(0) if response received with no error.
		* @param parameter_index unique parameter index
		* @param data_point DataPoint const object pointer
		*/
		virtual Result GetDPByIndex(uint32_t parameter_index, const DataPoint*& data_point) = 0;

		/** @brief Get data point from API by id.
		*
		* @return Result OK(0) if response received with no error.
		* @param parameter_id unique parameter id
		* @param data_point DataPoint const object pointer
		*/
		virtual Result GetDPById(uint32_t parameter_id, const DataPoint*& data_point) = 0;
		
		/** @brief Get data point count from API.
		*
		* @return Result OK(0) if response received with no error.
		* @param parameter_count number of data points
		*/
		virtual Result GetDPCount(uint32_t& parameter_count) = 0;
		
		/**
		* @brief Registers a call back to get notified when on LOG event
		*
		* @param callback Function delegate reference
		*/
		virtual Result RegisterLogsCallback(std::function<void(RuntimeLogEventData*)> callback) = 0;
		virtual Result RegisterLogsCallback(RuntimeLogEventCallback callback) = 0;
		
		/**
		* @brief Unregister callback if was registered, else does nothing.
		*/
		virtual Result UnregisterLogsCallback() = 0;

		/**
		* @brief return frame related logs
		*
		* @param frame_index in case of reading from file, frame index of frame in file to collect
		*/
		virtual Result GrabLogs(uint32_t frame_index) = 0;

	};
}
#endif /*__READER_INTERFACE_H__*/