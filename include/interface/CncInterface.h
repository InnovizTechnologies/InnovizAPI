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

#ifndef __CNC_INTERFACE_H__
#define __CNC_INTERFACE_H__

/*=========================================================================

Program:   Command & Control Interface
Module:    CncInterface.h
=========================================================================*/

/**
* @file CncInterface.h
* @authors Chen Vivnate
* @date 17-02-2019
* @brief File containing Command & Control Interface decleration.
*/


/*! \mainpage Home
*
* \section intro_sec Introduction
The Innoviz API is the application program interface used for (a) communicating with the software running on an InnovizOne LiDAR sensor system and (b) for viewing the point cloud that the LiDAR produces. Point clouds are saved in Innoviz's proprietary .invz file format.

The LiDAR uses TCP and UDP to transmit the point cloud and allow configuration: UDP is used to send the point cloud data continuously as a broadcast and TCP is used for command and control. All TCP and UDP parameters, including network settings, are configured in the Innoviz Web Server (under development) or sent to the unit via TCP. The LiDAR supports IPv4.

The API is used by third-party visualization utilities to enable them to view Innoviz point clouds.
*
*/
#include "../common_includes/invz_types.h"
#include "../common_includes/invz_status.h"
#include "../common_includes/invz_constants.h"
#include "../data_point/Datapoint.h"
#include <functional>


namespace invz {

	class INVZ_API ICnc {
	public:
		/* TODO: Change the using of results */
		/**
		* @brief Establishing the TCP socket connection
		*/
		virtual Result EstablishConnection() = 0;

		/**
		* @brief Disconnect TCP socket connection
		*/
		virtual Result Disconnect() = 0;

		/**
		* @brief Build ACP packet with TLV and write it to user buffer
		*/
		virtual Result BuildACP(TlvPack request, uint8_t* buffer, size_t& length) = 0;

		/**
		* @brief Send TLV via TCP socket
		*/

		virtual Result SendTLV(TlvPack &request, TlvPack &response, bool returnErrorTlv = false) = 0;

		/** @brief Establish a connection via TCP (configuration channel) with the LiDAR.
		* Collect basic information for internal initialization from the device.
		* Throws exception if connection could not be established
		*
		* @return byte actual access level to LiDAR.
		* @param request_level Login level to access (relevant for advanced users connection)
		* @param password Password to log in with (relevant for advanced users connection)
		*/
		virtual Result Connect(uint8_t &actual_level, uint8_t request_level = E_LOGIN_LEVEL_USER, std::string password = "") = 0;

		/**
		* @brief Perform handshake with the OM.
		*/
		virtual Result CSHandshake() = 0;

		/**
		* @brief Is the cnc connected
		*/
		virtual Result IsConnected(bool &connected) = 0;

		/** @brief Sending PingTlv to device.
		*
		* @return Result OK(0) if response received with no error.
		*/
		virtual Result Ping(uint32_t payload_size = 32) = 0;

		/** @brief Sending get paramater from device by ID.
		*
		* @return Result OK(0) if response received with no error.
		* @param parameter_id unique parameter ID
		* @param parameter_len parameter size in bytes
		* @param parameter_value buffer the parametr value will be written to (in the size of parameter_len)
		* @param policy Auto/Forced Direct
		*/
		virtual Result GetParameterById(uint32_t parameter_id, size_t& parameter_len, uint8_t* parameter_value, uint32_t policy = E_GET_PARAM_POLICY_AUTO) = 0;

		/** @brief Sending get paramater from device by name.
		*
		* @return Result OK(0) if response received with no error.
		* @param parameter_name unique parameter name
		* @param parameter_len parameter size in bytes
		* @param parameter_value buffer the parametr value will be written to (in the size of parameter_len)
		* @param policy Auto/Forced Direct
		*/
		virtual Result GetParameterByName(std::string parameter_name, size_t& parameter_len, uint8_t* parameter_value, uint32_t policy = E_GET_PARAM_POLICY_AUTO) = 0;

		/** @brief Sending get paramater from device by data point.
		*
		* @return Result OK(0) if response received with no error.
		* @param parameter_dp unique parameter data point
		* @param parameter_len parameter size in bytes
		* @param parameter_value buffer the parametr value will be written to (in the size of parameter_len)
		* @param policy Auto/Forced Direct
		*/
		virtual Result GetParameterByDataPoint(const DataPoint* parameter_dp, size_t& parameter_len, uint8_t* parameter_value, uint32_t policy = E_GET_PARAM_POLICY_AUTO) = 0;

		/** @brief Sending get paramater from device by data point (Not Safe - use GetParameterById/Name/DataPoint)
		*
		* @return Result OK(0) if response received with no error.
		* @param parameter_dp unique parameter data point
		* @param parameter_value one item of parameter
		* @param policy Auto/Forced Direct
		*/
		template<typename T>
		Result GetParameterValue(const DataPoint* parameter_datapoint, T& parameter_value, uint32_t policy = E_GET_PARAM_POLICY_AUTO);
		
		/** @brief Sending get paramater from device by ID.
		*
		* @return Result OK(0) if response received with no error.
		* @param parameter_id unique parameter ID
		* @param parameter_value the parametr value by it relevant type
		* @param policy Auto/Forced Direct
		*/
		template<typename T>
		Result GetParameterById(uint32_t parameter_id, T& parameter_value, uint32_t policy = E_GET_PARAM_POLICY_AUTO);

		/** @brief Sending get paramater from device by name.
		*
		* @return Result OK(0) if response received with no error.
		* @param parameter_name unique parameter name
		* @param parameter_value the parametr value by it relevant type
		* @param policy Auto/Forced Direct
		*/
		template<typename T>
		Result GetParameterByName(std::string parameter_name, T& parameter_value, uint32_t policy = E_GET_PARAM_POLICY_AUTO);

		/** @brief Sending get paramater from device by data point.
		*
		* @return Result OK(0) if response received with no error.
		* @param parameter_dp unique parameter data point
		* @param parameter_value the parametr value by it relevant type
		* @param policy Auto/Forced Direct
		*/
		template<typename T>
		Result GetParameterByDataPoint(const DataPoint* parameter_dp, T& parameter_value, uint32_t policy = E_GET_PARAM_POLICY_AUTO);

		/** @brief Sending set paramater from device by ID.
		*
		* @return Result OK(0) if response received with no error.
		* @param parameter_id unique parameter ID
		* @param parameter_len parameter size in bytes
		* @param parameter_value buffer the parametr value 
		* @param burn_to_flash flag to burn the new value to flash
		*/
		virtual Result SetParameterById(uint32_t parameter_id, size_t parameter_len, uint8_t* parameter_value, bool burn_to_flash = false) = 0;

		/** @brief Sending set paramater from device by name.
		*
		* @return Result OK(0) if response received with no error.
		* @param parameter_name unique parameter name
		* @param parameter_len parameter size in bytes
		* @param parameter_value buffer the parametr value 
		* @param burn_to_flash flag to burn the new value to flash
		*/
		virtual Result SetParameterByName(std::string parameter_name, size_t parameter_len, uint8_t* parameter_value, bool burn_to_flash = false) = 0;

		/** @brief Sending set paramater from device by data point.
		*
		* @return Result OK(0) if response received with no error.
		* @param parameter_dp unique parameter data point
		* @param parameter_len parameter size in bytes
		* @param parameter_value buffer the parametr value 
		* @param burn_to_flash flag to burn the new value to flash
		*/
		virtual Result SetParameterByDataPoint(const DataPoint* parameter_dp, size_t parameter_len, uint8_t* parameter_value, bool burn_to_flash = false) = 0;
		
		/** @brief Sending set paramater from device by data point (Not Safe - use SetParameterById/Name/DataPoint)
		* 
		* @return Result OK(0) if response received with no error.
		* @param parameter_dp unique parameter data point
		* @param parameter_value one item of parameter
		* @param burn_to_flash flag to burn the new value to flash
		*/
		template<typename T>
		Result SetParameterValue(const DataPoint* parameter_datapoint, T parameter_value, bool burn_to_flash = false);

		/** @brief Sending set paramater from device by ID.
		*
		* @return Result OK(0) if response received with no error.
		* @param parameter_id unique parameter ID
		* @param parameter_value the parametr value by it relevant type
		* @param burn_to_flash flag to burn the new value to flash
		*/
		template<typename T>
		Result SetParameterById(uint32_t parameter_id, T parameter_value, bool burn_to_flash = false);

		/** @brief Sending set paramater from device by name.
		*
		* @return Result OK(0) if response received with no error.
		* @param parameter_name unique parameter name
		* @param parameter_value the parametr value by it relevant type
		* @param burn_to_flash flag to burn the new value to flash
		*/
		template<typename T>
		Result SetParameterByName(std::string parameter_name, T parameter_value, bool burn_to_flash = false);

		/** @brief Sending set paramater from device by data point.
		*
		* @return Result OK(0) if response received with no error.
		* @param parameter_dp unique parameter data point
		* @param parameter_value the parametr value by it relevant type
		* @param burn_to_flash flag to burn the new value to flash
		*/
		template<typename T>
		Result SetParameterByDataPoint(const DataPoint* parameter_dp, T parameter_value, bool burn_to_flash = false);

		/** @brief set tap activation state by parameter ID
		*
		* @param parameter_id unique parameter ID
		* @param parameter_name unique parameter name
		* @param state set tap state on or off
		*/
		virtual Result SetParameterTapStateById(uint32_t parameter_id, bool state) = 0;

		/** @brief set tap activation state by parameter name
		* 
		* Throws exception if the DP is null pointer.
		* @param parameter_name unique parameter name
		* @param state set tap state on or off
		*/
		virtual Result SetParameterTapStateByName(std::string parameter_name, bool state) = 0;

		/** @brief set tap activation state by data point.
		*
		* Throws exception if the DP is null pointer.
		* @param parameter_dp unique parameter data point
		* @param state set tap state on or off
		*/
		virtual Result SetParameterTapStateByDataPoint(const DataPoint* parameter_dp, bool state) = 0;

		/** @brief burn parameter current value by parameter ID
		*
		* @param parameter_id unique parameter ID
		* @param parameter_name unique parameter name
		*/
		virtual Result BurnParameterCurrentValueById(uint32_t parameter_id) = 0;

		/** @brief burn parameter current value by parameter name
		*
		* Throws exception if the DP is null pointer.
		* @param parameter_name unique parameter name
		*/
		virtual Result BurnParameterCurrentValueByName(std::string parameter_name) = 0;

		/** @brief burn parameter current value by data point .
		*
		* Throws exception if the DP is null pointer.
		* @param parameter_dp unique parameter data point 
		*/
		virtual Result BurnParameterCurrentValueByDataPoint(const DataPoint* parameter_dp) = 0;

		/** @brief Read register from API by address.
		*
		* @return Result OK(0) if response received with no error.
		* @param absolute_address register absolute address
		* @param width register width in bits
		* @param bit_offset register bit offset
		* @param register_value
		*/
		virtual Result ReadRegisterByAddress(uint32_t absolute_address, uint32_t width, uint32_t bit_offset, uint32_t& register_value) = 0;

		/** @brief Read register from API by name.
		*
		* @return Result OK(0) if response received with no error.
		* @param register_name unique register name
		* @param register_value
		*/
		virtual Result ReadRegisterByName(std::string register_name, uint32_t& register_value) = 0;

		/** @brief Read register from API by register struct.
		*
		* @return Result OK(0) if response received with no error.
		* @param register_struct unique register struct
		* @param register_value
		*/
		virtual Result ReadRegister(const Register* register_struct, uint32_t& register_value) = 0;

		/** @brief Write register from API by address.
		*
		* @return Result OK(0) if response received with no error.
		* @param absolute_address register absolute address
		* @param width register width in bits
		* @param bit_offset register bit offset
		* @param register_value
		*/
		virtual Result WriteRegisterByAddress(uint32_t absolute_address, uint32_t width, uint32_t bit_offset, uint32_t register_value) = 0;

		/** @brief Write register from API by register struct.
		*
		* @return Result OK(0) if response received with no error.
		* @param register_name unique register name
		* @param register_value
		*/
		virtual Result WriteRegisterByName(std::string register_name, uint32_t register_value) = 0;

		/** @brief Write register from API by name.
		*
		* @return Result OK(0) if response received with no error.
		* @param register_struct unique register struct
		* @param register_value
		*/
		virtual Result WriteRegister(const Register* register_struct, uint32_t register_value) = 0;

		/** @brief Get NOR partition size in bytes.
		*
		* @return Result OK(0) if response received with no error.
		* @param partition_id NOR patition ID (by enum AcpNorPartition)
		* @param partition_length partition length in bytes
		*/
		virtual Result GetNorPartitionSize(uint32_t partition_id, size_t& partition_length) = 0;

		/** @brief Read NOR partition.
		*
		* @return Result OK(0) if response received with no error.
		* @param partition_id NOR patition ID (by enum AcpNorPartition)
		* @param buffer_length user buffer length (updated by written size)
		* @param buffer user buffer pointer
		*/
		virtual Result ReadNorPartition(uint32_t partition_id, uint32_t& buffer_length, uint8_t* buffer) = 0;

		/** @brief Write NOR partition.
		*
		* @return Result OK(0) if response received with no error.
		* @param partition_id NOR patition ID (by enum AcpNorPartition)
		* @param buffer_length user buffer length
		* @param buffer user buffer pointer
		*/
		virtual Result WriteNorPartition(uint32_t partition_id, uint32_t buffer_length, uint8_t* buffer) = 0;

		/** @brief Erase NOR partition.
		*
		* @return Result OK(0) if response received with no error.
		* @param partition_id NOR patition ID (by enum AcpNorPartition)
		*/
		virtual Result EraseNorPartition(uint32_t partition_id) = 0;

		/** @brief Get data point from API by ID.
		*
		* @return Result OK(0) if response received with no error.
		* @param parameter_id unique parameter ID
		* @param data_point DataPoint const object pointer
		*/
		virtual Result GetDataPointById(uint32_t parameter_id, const DataPoint*& data_point) = 0;

		/** @brief Get data point from API by name.
		*
		* @return Result OK(0) if response received with no error.
		* @param parameter_name unique parameter name
		* @param data_point DataPoint const object pointer
		*/
		virtual Result GetDataPointByName(std::string parameter_name, const  DataPoint*& data_point) = 0;

		/** @brief Get data point from API by mapping index.
		*
		* @return Result OK(0) if response received with no error.
		* @param parameter_index unique parameter index
		* @param data_point DataPoint const object pointer
		*/
		virtual Result GetDataPointByIndex(uint32_t parameter_index, const DataPoint*& data_point) = 0;

		/** @brief Get data point count from API.
		*
		* @return Result OK(0) if response received with no error.
		* @param parameter_count number of data points
		*/
		virtual Result GetDataPointCount(uint32_t& parameter_count) = 0;

		/** @brief Get register from API by name.
		*
		* @return Result OK(0) if response received with no error.
		* @param register_name unique register name
		* @param register_struct Register struct
		*/
		virtual Result GetRegisterByName(std::string register_name, const Register*& register_struct) = 0;

		/** @brief Get register from API by mapping index.
		*
		* @return Result OK(0) if response received with no error.
		* @param register_index unique register index
		* @param register_struct Register struct
		*/
		virtual Result GetRegisterByIndex(uint32_t register_index, const Register*& register_struct) = 0;

		/** @brief Get registers count from API.
		*
		* @return Result OK(0) if response received with no error.
		* @param register_count number of registers
		*/
		virtual Result GetRegisterCount(uint32_t& register_count) = 0;

		/**
		* @brief Record innoviz point cloud into .invz file using UDP.
		* @param file_path directory path where recorded files will be created
		*/
		virtual Result StartRecording(std::string file_path, bool flush_queues = false) = 0;

		/**
		* @brief Stop a previous recording started with StartRecording
		* This method should be used after a StartRecording() method that initializes recording.
		*/
		virtual Result StopRecording() = 0;

		/**
		* @brief Record innoviz point cloud into .invz file using UDP.
		* This method should be used after a Connect() method that initializes internal information.
		* Throws exception if device is was not connected.
		* @param seconds number of seconds to record
		* @param file_path .invz file to save the data into

		*/
		virtual Result Record(double seconds, std::string file_path = "", bool flush_queues = false) = 0;

		/**
		* @brief Registers a call back to get notified when on TAP event
		*
		* @param callback Function delegate reference
		*/
		virtual Result RegisterTapCallback(std::function<void(TapEventData*)> callback) = 0;
		virtual Result RegisterTapCallback(TapEventCallback callback) = 0;

		/**
		* @brief Unregister callback if was registered, else does nothing.
		*/
		virtual Result UnregisterTapCallback() = 0;

		/**
		* @brief Registers a call back to get notified when on TLV event
		*
		* @param callback Function delegate reference
		*/
		virtual Result RegisterTlvCallback(std::function<void(TlvPack*)> callback) = 0;
		virtual Result RegisterTlvCallback(TlvCallback callback) = 0;

		/**
		* @brief Unregister callback if was registered, else does nothing.
		*/
		virtual Result UnregisterTlvCallback() = 0;

		/**
		* @brief Registers a call back to get notified when on Frame event
		*
		* @param callback Function delegate reference
		*/
		virtual Result RegisterFrameCallback(std::function<void(uint32_t*)> callback) = 0;
		virtual Result RegisterFrameCallback(FrameCallback callback) = 0;

		/**
		* @brief Unregister callback if was registered, else does nothing.
		*/
		virtual Result UnregisterFrameCallback() = 0;

	};

	template<typename T>
	Result ICnc::GetParameterValue(const DataPoint* parameter_datapoint, T& parameter_value, uint32_t policy)
	{
		RESULT_START
			/* Get type name */
			std::string typeName(typeid(T).name());

		/* Validate T size corresponds with the DP size */
		if (sizeof(T) != parameter_datapoint->size)
		{
			std::string error_msg{ "Type mismatch asked datapoint type is " + parameter_datapoint->type + " used type is " + typeName };
			result.error_code = ERROR_CODE_GENERAL;
			result.error_message = error_msg;
			throw std::runtime_error(error_msg);
		}
		/* Scalar type non-pointer */
		else
		{
			size_t paramTypeLen = sizeof(T);
			uint8_t* paramByteArray = reinterpret_cast<uint8_t*>(&parameter_value);
			result = GetParameterById(parameter_datapoint->id, paramTypeLen, paramByteArray, policy);
			if (result.error_code != ERROR_CODE_OK)
				return result;
		}
		RESULT_END
	}

	template<> 
	inline
	Result ICnc::GetParameterValue<std::string>(const DataPoint* parameter_datapoint, std::string& parameter_value, uint32_t policy)
	{
		RESULT_START
			size_t strLen = parameter_datapoint->size;
		uint8_t* strParamArray = new uint8_t[strLen + 1]{ 0 };
		result = GetParameterById(parameter_datapoint->id, strLen, strParamArray, policy);
		strParamArray[strLen] = '\0';
		if (result.error_code != ERROR_CODE_OK)
			return result;
		std::string strParam = (char*)strParamArray;
		delete[] strParamArray;
		parameter_value = strParam;
		RESULT_END
	}

	template<> 
	inline
	Result ICnc::GetParameterValue<bool>(const DataPoint* parameter_datapoint, bool& parameter_value, uint32_t policy)
	{
		RESULT_START
			if ((parameter_datapoint->rows * parameter_datapoint->columns) == 1)
			{
				size_t paramTypeLen = parameter_datapoint->size;
				uint8_t* paramByteArray = new uint8_t[paramTypeLen];
				result = GetParameterById(parameter_datapoint->id, paramTypeLen, paramByteArray, policy);
				if (result.error_code != ERROR_CODE_OK)
					return result;
				parameter_value = (bool)(*paramByteArray);
				delete[] paramByteArray;
			}
			else
			{
				std::string error_msg{ "GetParameterValue can return value of one item only" };
				result.error_code = ERROR_CODE_GENERAL;
				result.error_message = error_msg;
				throw std::runtime_error(error_msg);
			}
		RESULT_END
	}

	/** @brief Sending get paramater from device by ID.
*
* @return Result OK(0) if response received with no error.
* @param parameter_id unique parameter ID
* @param parameter_value the parametr value by it relevant type
*/
	template<typename T>
	Result ICnc::GetParameterById(uint32_t parameter_id, T& parameter_value, uint32_t policy)
	{
		RESULT_START
			/* Find data point */
			const DataPoint* dp = nullptr;
		result = GetDataPointById(parameter_id, dp);
		if (result.error_code != ERROR_CODE_OK)
			return result;

		return GetParameterValue(dp, parameter_value, policy);
		RESULT_END
	}

	/** @brief Sending get paramater from device by name.
	*
	* @return Result OK(0) if response received with no error.
	* @param parameter_name unique parameter name
	* @param parameter_value the parametr value by it relevant type
	*/
	template<typename T>
	Result ICnc::GetParameterByName(std::string parameter_name, T& parameter_value, uint32_t policy)
	{
		RESULT_START
			/* Find data point */
			const DataPoint* dp = nullptr;
		result = GetDataPointByName(parameter_name, dp);
		if (result.error_code != ERROR_CODE_OK)
			return result;

		return GetParameterValue(dp, parameter_value, policy);

		RESULT_END
	}

	/** @brief Sending get paramater from device by data point.
	*
	* @return Result OK(0) if response received with no error.
	* @param parameter_dp unique parameter data point
	* @param parameter_value the parametr value by it relevant type
	*/
	template<typename T>
	Result ICnc::GetParameterByDataPoint(const DataPoint* parameter_dp, T& parameter_value, uint32_t policy)
	{
		RESULT_START
			return GetParameterById(parameter_dp->id, parameter_value, policy);
		RESULT_END
	}


	template<typename T>
	Result ICnc::SetParameterValue(const DataPoint* parameter_datapoint, T parameter_value, bool burn_to_flash)
	{
		RESULT_START
			/* Get type name */
			std::string typeName(typeid(T).name());

		if (sizeof(T) != parameter_datapoint->size && typeName.find("*") == std::string::npos)
		{
			std::string error_msg{ "Type mismatch asked datapoint type is " + parameter_datapoint->type + " used type is " + typeName };
			result.error_code = ERROR_CODE_GENERAL;
			result.error_message = error_msg;
			throw std::runtime_error(error_msg);
		}
		/* Scalar type non-pointer */
		else
		{
			size_t paramTypeLen = sizeof(T);
			uint8_t* paramByteArray = reinterpret_cast<uint8_t*>(&parameter_value);
			result = SetParameterById(parameter_datapoint->id, paramTypeLen, paramByteArray, burn_to_flash);
			if (result.error_code != ERROR_CODE_OK)
				return result;
		}
		RESULT_END
	}

	template<> 
	inline
	Result ICnc::SetParameterValue<std::string>(const DataPoint* parameter_datapoint, std::string parameter_value, bool burn_to_flash)
	{
		RESULT_START
			std::string strParam = parameter_value;
		strParam.resize(parameter_datapoint->size);
		size_t paramLen = strParam.size();
		result = SetParameterById(parameter_datapoint->id, paramLen, (uint8_t*)strParam.c_str(), burn_to_flash);
		if (result.error_code != ERROR_CODE_OK)
			return result;
		RESULT_END
	}

	template<> 
	inline
	Result ICnc::SetParameterValue<const char*>(const DataPoint* parameter_datapoint, const char* parameter_value, bool burn_to_flash)
	{
		return SetParameterValue<std::string>(parameter_datapoint, parameter_value, burn_to_flash);
	}

	template<> 
	inline
	Result ICnc::SetParameterValue<char*>(const DataPoint* parameter_datapoint, char* parameter_value, bool burn_to_flash)
	{
		return SetParameterValue<std::string>(parameter_datapoint, parameter_value, burn_to_flash);
	}

	template<> 
	inline
	Result ICnc::SetParameterValue<bool>(const DataPoint* parameter_datapoint, bool parameter_value, bool burn_to_flash)
	{
		RESULT_START
			if ((parameter_datapoint->rows * parameter_datapoint->columns) == 1)
			{
				size_t paramTypeLen = parameter_datapoint->size;
				uint8_t* paramByteArray = new uint8_t[paramTypeLen];
				*paramByteArray = (uint8_t)parameter_value;
				result = SetParameterById(parameter_datapoint->id, paramTypeLen, paramByteArray, burn_to_flash);
				if (result.error_code != ERROR_CODE_OK)
					return result;
				delete[] paramByteArray;
			}
			else
			{
				std::string error_msg{ "SetParameterValue can set value of one item only" };
				result.error_code = ERROR_CODE_GENERAL;
				result.error_message = error_msg;
				throw std::runtime_error(error_msg);
			}
		RESULT_END
	}

	/** @brief Sending set paramater from device by ID.
	*
	* @return Result OK(0) if response received with no error.
	* @param parameter_id unique parameter ID
	* @param parameter_value the parametr value by it relevant type
	* @param burn_to_flash flag to burn the new value to flash
	*/
	template<typename T>
	Result ICnc::SetParameterById(uint32_t parameter_id, T parameter_value, bool burn_to_flash)
	{
		RESULT_START
			/* Find data point */
			const DataPoint* dp = nullptr;
		result = GetDataPointById(parameter_id, dp);
		if (result.error_code != ERROR_CODE_OK)
			return result;

		return SetParameterValue(dp, parameter_value, burn_to_flash);
		RESULT_END
	}

	/** @brief Sending set paramater from device by name.
	*
	* @return Result OK(0) if response received with no error.
	* @param parameter_name unique parameter name
	* @param parameter_value the parametr value by it relevant type
	* @param burn_to_flash flag to burn the new value to flash
	*/
	template<typename T>
	Result ICnc::SetParameterByName(std::string parameter_name, T parameter_value, bool burn_to_flash)
	{
		RESULT_START
			/* Find data point */
			const DataPoint* dp = nullptr;
		result = GetDataPointByName(parameter_name, dp);
		if (result.error_code != ERROR_CODE_OK)
			return result;

		return SetParameterValue(dp, parameter_value, burn_to_flash);
		RESULT_END
	}


	/** @brief Sending set paramater from device by data point.
	*
	* @return Result OK(0) if response received with no error.
	* @param parameter_dp unique parameter data point
	* @param parameter_value the parametr value by it relevant type
	* @param burn_to_flash flag to burn the new value to flash
	*/
	template<typename T>
	Result ICnc::SetParameterByDataPoint(const DataPoint* parameter_dp, T parameter_value, bool burn_to_flash)
	{
		RESULT_START
			return SetParameterById(parameter_dp->id, parameter_value, burn_to_flash);
		RESULT_END
	}
}

#endif /*__CNC_INTERFACE_H__*/
