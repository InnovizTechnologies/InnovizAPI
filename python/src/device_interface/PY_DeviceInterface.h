///////////////////////////////////////////////////////////
//  PY_DeviceInterface.h
//  Implementation of the Class PY_DeviceInterface
//  Created on:      09-Aug-2021 3:25:19 PM
//  Original author: sarah.foox
///////////////////////////////////////////////////////////

#ifndef __PY_DEVICE_INTERFACE_H__
#define __PY_DEVICE_INTERFACE_H__

#include "../common_files/PY_CommonUtils.h"
#include "interface/ConfigApi.h"
#include <mutex>

class PY_DeviceInterface
{
public:
	PY_DeviceInterface(std::string config_file_name, bool is_connect, int login_level, std::string password, uint32_t log_severity, bool require_data_attr);
	~PY_DeviceInterface();

	void DeviceClose();
	IDevice* DI();
	size_t GetNumDataPoints();
	uint8_t Connect(uint8_t request_level, std::string password);
	void Disconnect();
	void Record(double seconds, std::string filepath, bool flush_queues);
	void StartRecording(std::string filepath, bool flush_queues);
	void StopRecording();
	py::bool_ IsConnected();
	void ActivateBuffer(FrameDataAttributes frame_type, bool activate);
	py::dict GetStatistics();
	PyGrabFrameResult GetFrame(std::vector<FrameDataAttributes> frame_types);
	py::object BuildAcp(invz::AcpHeaderTlvPack acp_header, PyTLV request_tlv);
	py::tuple SendTlv(invz::AcpHeaderTlvPack acp_header, PyTLV request_tlv, bool return_error_tlv = false);
	invz::DataPoint GetDpDetails(std::string dp_name);
	invz::DataPoint GetDpDetailsById(uint32_t dp_id);
	py::list GetAllDpDetails();
	py::list GetAllTapDetails();
	py::object GetEmptyDp(std::string dp_name);
	py::object GetZeroDp(std::string dp_name);
	py::object GetDp(std::string dp_name, uint32_t get_dp_policy);
	py::dtype GetDpDtype(std::string dp_name);
	py::dtype GetDpDtypeById(uint32_t dp_id);
	void SetDp(std::string dp_name, py::object obj, bool set_param);
	invz::Register GetRegisterDetails(std::string register_name);
	py::list GetAllRegistersDetails();
	py::tuple GetRegisterByName(std::string register_name);
	py::bool_ SetRegisterByName(std::string register_name, uint32_t regValue);
	py::tuple GetRegisterByAddress(uint32_t regAddress, uint32_t regWidth, uint32_t bitOffset);
	py::bool_ SetRegisterByAddress(uint32_t regAddress, uint32_t regWidth, uint32_t bitOffset, uint32_t regValue);
	void SetTapActivationState(std::string dp_name, bool should_enable);
	void RegisterTapsCallback(std::function<void(PyTapHandler&)> callback);
	void UnregisterTapsCallback();
	void RegisterLogsCallback(std::function<void(PyLogHandler&)> callback);
	void UnregisterLogsCallback();
	void RegisterNewFrameCallback(std::function<void(uint32_t&)> callback);
	void UnregisterNewFrameCallback();
	void RegisterNewTlvCallback(std::function<void(PyTLVPack&)> callback);
	void UnregisterNewTlvCallback();
	void CSHandshake();

	uint8_t ConnectionLevel;

private:
	uint32_t fileFormat = invz::EFileFormat::E_FILE_FORMAT_INVZ5;
	std::vector<invz::FrameDataAttributes> frameDataAttrs;
	std::unique_ptr<invz::IDevice> m_di = nullptr;

	std::function<void(invz::RuntimeLogEventData*)> logCallbackCpp;
	std::function<void(PyLogHandler&)> logCallbackPy;
	std::function<void(invz::TapEventData*)> tapCallbackCpp;
	std::function<void(PyTapHandler&)> tapCallbackPy;
	std::function<void(invz::TlvPack*)> tlvCallbackCpp;
	std::function<void(PyTLVPack&)> tlvCallbackPy;
	std::function<void(uint32_t*)> frameEventCallbackCpp;
	std::function<void(uint32_t&)> frameEventCallbackPy;

	void validateNumpyDType(py::object obj, const invz::DataPoint* dp);
	template<class T> py::object getDpScalar(py::array arr);
	py::object getDpScalar(py::array arr, const invz::DataPoint* dp);
	void setDpPyIntScalar(int64_t val, const invz::DataPoint* dp, bool set_param);
	void setDpPyFloatScalar(double val, const invz::DataPoint* dp, bool set_param);
	size_t getNumRegisters();

	std::mutex m_CnCMutex;
};
#endif // !defined __PY_DEVICE_INTERFACE_H__
