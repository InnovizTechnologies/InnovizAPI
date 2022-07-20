///////////////////////////////////////////////////////////
//  PY_DeviceInterface.cpp
//  Implementation of the Class PY_DeviceInterface
//  Created on:      09-Aug-2021 3:25:19 PM
//  Original author: sarah.foox
///////////////////////////////////////////////////////////


#include "PY_DeviceInterface.h"
#include <thread>
#include <iostream>

 PY_DeviceInterface::PY_DeviceInterface(std::string config_file_name, bool is_connect, int login_level, std::string password, uint32_t log_severity, bool require_data_attr)
 {
	 invz::Result result;
	 ConnectionLevel = 0;

	 m_di.reset(invz::DeviceInit(config_file_name, "", log_severity));
	 size_t attr_count = INVZ_CONFIG_GET_FRAME_ATTR_DATA_MAX_SIZE;
	 invz::FrameDataAttributes attr[INVZ_CONFIG_GET_FRAME_ATTR_DATA_MAX_SIZE];

	 // TODO add deprecation warning
	 if (is_connect)
	 {
		 result = DI()->Connect(ConnectionLevel, login_level, password);

		 //if we failed to connect we need to dispose of di
		 if (result.error_code)
		 {
			 invz::DeviceClose(m_di.release());
		 }

		 CheckResult(result);

		 result = DI()->GetFileFormat(fileFormat);
		 CheckResult(result);
		 /* Get frame data attributes from file */

		 int retries = 10;
		 do
		 {
			 result = DI()->GetFrameDataAttributes(attr, attr_count, require_data_attr);
			 retries--;
			 if (result.error_code != ERROR_CODE_OK)
			 {
				 if (retries == 0)
				 {
					 attr_count = 0;
					 std::cout << "warning: device meta is missing. point cloud grabbing will not be available" << std::endl;
					 break;
				 }
				 std::this_thread::sleep_for(std::chrono::milliseconds(500));
			 }
			 else
				 break;
		 } while (true);

	 }

	 else //allow tcp connection for CM support
		 result = DI()->EstablishConnection();


	 tapCallbackCpp = [&](invz::TapEventData* th) -> void {
		 /* Acquire GIL before calling Python code */
		 py::gil_scoped_acquire acquire;
		 const invz::DataPoint* dp;
		 DI()->GetDataPointById(th->parameter_id, dp);
		 PyTapHandler py_th(*th, dp);
		 tapCallbackPy(py_th);
	 };

	 logCallbackCpp = [&](invz::RuntimeLogEventData* th) -> void {
		 /* Acquire GIL before calling Python code */
		 py::gil_scoped_acquire acquire;
		 PyLogHandler py_th(*th);
		 logCallbackPy(py_th);
	 };

	 tlvCallbackCpp = [&](invz::TlvPack* tlvPack) -> void {
		 /* Acquire GIL before calling Python code */
		 py::gil_scoped_acquire acquire;
		 PyTLVPack py_tlvPack(*tlvPack);
		 tlvCallbackPy(py_tlvPack);
	 };

	 frameEventCallbackCpp = [&](uint32_t* frameNumber) -> void {
		 /* Acquire GIL before calling Python code */
		 py::gil_scoped_acquire acquire;
		 frameEventCallbackPy(*frameNumber);
	 };


	 //try getting attributes without metadata
	 if (attr_count == 0 || !is_connect)
	 {
		 std::this_thread::sleep_for(std::chrono::milliseconds(5000));
		 attr_count = INVZ_CONFIG_GET_FRAME_ATTR_DATA_MAX_SIZE;
		 result = DI()->GetFrameDataAttributes(attr, attr_count, false);
		 if (result.error_code != ERROR_CODE_OK)
			 invz::DeviceClose(m_di.release());
		 CheckResult(result);
	 }

	 /* Get frame data attributes */
	 for (int i = 0; i < attr_count; i++)
	 {
		 frameDataAttrs.push_back(attr[i]);
	 }
 }
 void PY_DeviceInterface::DeviceClose()
 {
	 py::gil_scoped_release release;

	 // close reader if exists
	 if (m_di) {
		 invz::DeviceClose(m_di.release());
	 }
 }
 PY_DeviceInterface::~PY_DeviceInterface() {
	 DeviceClose();
 }

 void PY_DeviceInterface::validateNumpyDType(py::object obj, const invz::DataPoint* dp)
 {
	 std::string typeStr;
	 size_t itemsize;
	 try {
		 typeStr = py::str(obj.attr("dtype"));
		 itemsize = obj.attr("itemsize").cast<size_t>();
	 }
	 catch (const std::exception& e) {
		 (void)e;
		 std::string msg = "Unsupported input class ";
		 msg.append(py::str(obj.attr("__class__")));
		 throw std::runtime_error(msg);
	 }

	 auto tm = GetTypeMeta(dp->type);

	 // validate dtype
	 if (tm.np_dtype != typeStr) {
		 std::stringstream msg;
		 msg << "dp type '" << dp->type << "' doesn't match numpy dtype '" << typeStr << "'.";
		 throw std::runtime_error(msg.str());
	 }

	 // validate size
	 if (tm.itemsize != itemsize) {
		 std::stringstream msg;
		 msg << "dp size, " << dp->size << ", doesn't match numpy " << typeStr << "size, " << itemsize << ".";
		 throw std::runtime_error(msg.str());
	 }
 }

 template<class T> py::object PY_DeviceInterface::getDpScalar(py::array arr)
 {
	 py::object type = py::dtype::of<T>().attr("type");
	 auto buf = arr.request();
	 auto *ptr = (T *)buf.ptr;

	 return type(ptr[0]);
 }

 py::object PY_DeviceInterface::getDpScalar(py::array arr, const invz::DataPoint* dp)
 {
	 py::object ret;

	 // get dp value
	 if (dp->type.find("int8", 0) == 0)
		 ret = getDpScalar<int8_t>(arr);
	 else if (dp->type.find("uint8", 0) == 0)
		 ret = getDpScalar<uint8_t>(arr);
	 else if (dp->type.find("int16", 0) == 0)
		 ret = getDpScalar<int16_t>(arr);
	 else if (dp->type.find("uint16", 0) == 0)
		 ret = getDpScalar<uint16_t>(arr);
	 else if (dp->type.find("int32", 0) == 0)
		 ret = getDpScalar<int32_t>(arr);
	 else if (dp->type.find("uint32", 0) == 0)
		 ret = getDpScalar<uint32_t>(arr);
	 else if (dp->type == "float")
		 ret = getDpScalar<float>(arr);
	 else if (dp->type == "double")
		 ret = getDpScalar<double>(arr);
	 else if (dp->type == "bool")
		 ret = getDpScalar<bool>(arr);
	 else {
		 std::string error{ "Unsupported dp type: " };
		 error.append(dp->type);
		 throw std::runtime_error(error);
	 }

	 return ret;
 }
 void PY_DeviceInterface::setDpPyIntScalar(int64_t val, const invz::DataPoint* dp, bool set_param)
 {
	 Result result;
	 // get dp value
	 if (dp->type.find("int8", 0) == 0)
		 result = DI()->SetParameterValue<int8_t>(dp, static_cast<int8_t>(val), set_param);
	 else if (dp->type.find("uint8", 0) == 0)
		 result = DI()->SetParameterValue<uint8_t>(dp, static_cast<uint8_t>(val), set_param);
	 else if (dp->type.find("int16", 0) == 0)
		 result = DI()->SetParameterValue<int16_t>(dp, static_cast<int16_t>(val), set_param);
	 else if (dp->type.find("uint16", 0) == 0)
		 result = DI()->SetParameterValue<uint16_t>(dp, static_cast<uint16_t>(val), set_param);
	 else if (dp->type.find("int32", 0) == 0)
		 result = DI()->SetParameterValue<int32_t>(dp, static_cast<int32_t>(val), set_param);
	 else if (dp->type.find("uint32", 0) == 0)
		 result = DI()->SetParameterValue<uint32_t>(dp, static_cast<uint32_t>(val), set_param);
	 else {
		 std::string error{ "Invalid set value type. Can't set dp type '" };
		 error.append(dp->type);
		 error.append("' using int value.");
		 throw std::runtime_error(error);
	 }
	 CheckResult(result);
 }
 void PY_DeviceInterface::setDpPyFloatScalar(double val, const invz::DataPoint* dp, bool set_param)
 {
	 Result result;
	 // get dp value
	 if (dp->type == "float")
		 result = DI()->SetParameterValue<float>(dp, static_cast<float>(val), set_param);
	 else if (dp->type == "double")
		 result = DI()->SetParameterValue<double>(dp, val, set_param);
	 else {
		 std::string error{ "Invalid set value type. Can't set dp type '" };
		 error.append(dp->type);
		 error.append("' using float value.");
		 throw std::runtime_error(error);
	 }
	 CheckResult(result);
 }

 IDevice* PY_DeviceInterface::DI()
 {
	 if (m_di) {
		 return m_di.get();
	 }
	 throw std::runtime_error("Invalid call. DeviceInterface not exists.");
 }

 size_t PY_DeviceInterface::GetNumDataPoints()
 {

	 invz::Result result;
	 uint32_t num_data_points;

	 result = DI()->GetDataPointCount(num_data_points);
	 CheckResult(result);

	 return num_data_points;
 }

 uint8_t PY_DeviceInterface::Connect(uint8_t request_level, std::string password)
 {
	 invz::Result result;
	 uint8_t actualConnectionLevel = 0;
	 result = DI()->Connect(actualConnectionLevel, request_level, password);
	 CheckResult(result);
	 return actualConnectionLevel;
 }

 void PY_DeviceInterface::Disconnect()
 {
	 invz::Result result;
	 result = DI()->Disconnect();
	 CheckResult(result);
 }

 void PY_DeviceInterface::Record(double seconds, std::string filepath, bool flush_queues)
 {
	 invz::Result result;
	 result = DI()->Record(seconds, filepath, flush_queues);
	 CheckResult(result);
 }

 void PY_DeviceInterface::StartRecording(std::string filepath, bool flush_queues)
 {
	 invz::Result result;
	 result = DI()->StartRecording(filepath, flush_queues);
	 CheckResult(result);
 }

 void PY_DeviceInterface::StopRecording()
 {
	 invz::Result result;
	 result = DI()->StopRecording();
	 CheckResult(result);
 }

 py::bool_ PY_DeviceInterface::IsConnected()
 {
	 invz::Result result;
	 bool lidar_connected;
	 result = DI()->IsConnected(lidar_connected);
	 CheckResult(result);
	 return py::bool_(lidar_connected);
 }

 void PY_DeviceInterface::ActivateBuffer(FrameDataAttributes frame_type, bool activate) 
 {
	 CheckAttribute(frame_type, frameDataAttrs);
	 DI()->ActivateBuffer(frame_type, activate);
 }

 py::dict PY_DeviceInterface::GetStatistics() 
 {
	 py::dict results;
	 size_t channels_num;
	 ChannelStatistics channels[10];
	 auto result = DI()->GetConnectionStatus(channels, channels_num);

	 if (result.error_code == ERROR_CODE_OK)
	 {
		 for (size_t i = 0; i < channels_num; i++)
		 {
			 results[py::str(std::to_string(channels[i].channel_id))] = channels[i];
		 }
	 }
	 return results;
 }

 PyGrabFrameResult PY_DeviceInterface::GetFrame(std::vector<FrameDataAttributes> frame_types)
 {
	 invz::Result result;

	 PyGrabFrameResult ret;
	 for (auto& dataAttrs : frame_types)
	 {
		 CheckAttribute(dataAttrs, frameDataAttrs);		//check all types are fully defined
	 }
	 std::vector<invz::FrameDataUserBuffer> userBuffers;

	 for (auto& dataAttrs : frame_types)
	 {

		 invz::FrameDataUserBuffer&& dataAttrsBuffer(dataAttrs);

		 /* Add user buffer to send to API */
		 userBuffers.push_back(dataAttrsBuffer);
	 }

	 uint32_t frame_number;
	 uint64_t timestamp;
	 bool success;
	 result = DI()->GrabFrame(userBuffers.data(), static_cast<uint32_t>(userBuffers.size()), frame_number, timestamp);
	 success = result.error_code == ERROR_CODE_OK;

	 for (auto& userBuffer : userBuffers)
	 {
		 if (userBuffer.status == USER_BUFFER_FULL)
		 {
			 auto buff = Get1DArray(userBuffer.dataAttrs.length, GetTypeMeta(fileFormat, userBuffer.dataAttrs.typeMajor, userBuffer.dataAttrs.typeMinor));
			 auto ptr = (invz::byte*)(buff.request().ptr);
			 std::copy((invz::byte*)userBuffer.dataBuffer, (invz::byte*)(userBuffer.dataBuffer + userBuffer.dataAttrs.nbytes()), ptr);
			 ret.results[py::str(GetBufferName(userBuffer.dataAttrs))] = buff;
		 }
		 else
			 ret.results[py::str(GetBufferName(userBuffer.dataAttrs))] = py::none();

	 }
	 ret.frame_number = frame_number;
	 ret.success = success;
	 ret.timestamp = timestamp;
	 //auto res= py::make_tuple(success, frame_number, ret);
	 return ret;
 }

 py::object PY_DeviceInterface::BuildAcp(invz::AcpHeaderTlvPack acp_header, PyTLV request_tlv)
 {
	 py::array arr;
	 invz::Result result;

	 /* Copy request TLV */
	 auto value_buffer = request_tlv.value.request();
	 invz::TlvPack requestTlvPack(request_tlv.type, request_tlv.length, (uint8_t*)value_buffer.ptr);
	 requestTlvPack.acp_header = acp_header;

	 /* Build ACP packet buffer */
	 uint8_t acp_buffer[UINT16_MAX]{ 0 };
	 size_t acp_buffer_length = UINT16_MAX;
	 result = DI()->BuildACP(requestTlvPack, acp_buffer, acp_buffer_length);
	 CheckResult(result);

	 /* Copy buffer to ndarray */
	 arr = Get1DArray(acp_buffer_length, GetTypeMeta<uint8_t>());
	 memcpy(arr.request().ptr, acp_buffer, acp_buffer_length);

	 return arr;
 }

 py::tuple PY_DeviceInterface::SendTlv(invz::AcpHeaderTlvPack acp_header, PyTLV request_tlv, bool return_error_tlv)
 {
	 invz::Result result;

	 /* Copy request TLV */
	 auto value_buffer = request_tlv.value.request();
	 invz::TlvPack requestTlvPack(request_tlv.type, request_tlv.length, (uint8_t*)value_buffer.ptr);
	 requestTlvPack.acp_header = acp_header;

	 /* Get TLV response */
	 invz::TlvPack responseTlvPack;
	 {
		 py::gil_scoped_release release;

		 std::unique_lock<std::mutex> lock(m_CnCMutex);
		 result = DI()->SendTLV(requestTlvPack, responseTlvPack, return_error_tlv);
	 }
	 
	 CheckResult(result);

	 /* Allocate new response */
	 PyTLV pyResponseTlv(responseTlvPack.tlv.type, responseTlvPack.tlv.length, responseTlvPack.tlv.value);

	 py::none none;

	 return py::make_tuple(responseTlvPack.acp_header, pyResponseTlv, responseTlvPack.port, requestTlvPack.acp_header);
 }

 invz::DataPoint PY_DeviceInterface::GetDpDetails(std::string dp_name)
 {

	 invz::Result result;
	 const invz::DataPoint* dp;
	 result = DI()->GetDataPointByName(dp_name, dp);
	 CheckResult(result);

	 return *dp;
 }

 invz::DataPoint PY_DeviceInterface::GetDpDetailsById(uint32_t dp_id)
 {
	 invz::Result result;
	 const invz::DataPoint* dp;
	 result = DI()->GetDataPointById(dp_id, dp);
	 CheckResult(result);
	 return *dp;
 }

 py::list PY_DeviceInterface::GetAllDpDetails()
 {
	 py::list list;

	 auto num_data_points = GetNumDataPoints();

	 for (int i = 0; i < num_data_points; i++) {
		 const invz::DataPoint* data_point;
		 DI()->GetDataPointByIndex(i, data_point);
		 list.append(*(data_point));
	 }

	 return list;
 }

 py::list PY_DeviceInterface::GetAllTapDetails()
 {
	 py::list list;

	 auto num_data_points = GetNumDataPoints();
	 for (int i = 0; i < num_data_points; i++)
	 {
		 const invz::DataPoint* data_point;
		 DI()->GetDataPointByIndex(i, data_point);
		 if (data_point->dptype == "tap")
			 list.append(*(data_point));
	 }

	 return list;
 }

 py::object PY_DeviceInterface::GetEmptyDp(std::string dp_name)
 {

	 py::array arr;
	 invz::Result result;
	 const invz::DataPoint* dp;

	 // get dp
	 result = DI()->GetDataPointByName(dp_name, dp);
	 CheckResult(result);

	 if (dp->type != "struct")
		 arr = GetDpArray(dp, GetTypeMeta(dp->type));
	 else
	 {
		 TypeMeta tm;
		 tm.itemsize = dp->dtype->itemsize();
		 tm.np_dtype = dp->type;
		 tm.np_format = dp->dtype->pybind11_format();
		 arr = GetDpArray(dp, tm);
	 }

	 return arr;
 }

 py::object PY_DeviceInterface::GetZeroDp(std::string dp_name)
 {

	 py::array arr;
	 invz::Result result;
	 const invz::DataPoint* dp;

	 // get dp
	 result = DI()->GetDataPointByName(dp_name, dp);
	 CheckResult(result);

	 if (dp->type != "struct")
		 arr = GetDpArray(dp, GetTypeMeta(dp->type));
	 else
	 {
		 TypeMeta tm;
		 tm.itemsize = dp->dtype->itemsize();
		 tm.np_dtype = dp->type;
		 tm.np_format = dp->dtype->pybind11_format();
		 arr = GetDpArray(dp, tm);
	 }

	 // set val to zero
	 memset(arr.request().ptr, 0, dp->size);

	 return arr;
 }

 py::object PY_DeviceInterface::GetDp(std::string dp_name, uint32_t get_dp_policy)
 {

	 py::array arr;
	 py::object ret;
	 invz::Result result;
	 const invz::DataPoint* dp;

	 // get dp
	 result = DI()->GetDataPointByName(dp_name, dp);
	 CheckResult(result);

	 // handle strings
	 if (dp->type == "char") {
		 std::string str;
		 {
			 py::gil_scoped_release release;
			 std::unique_lock<std::mutex> lock(m_CnCMutex);
			 result = DI()->GetParameterValue(dp, str, get_dp_policy);
		 }
		 CheckResult(result);
		 return py::str(str);
	 }

	 // allocate buffer on form of numpy array
	 if (dp->type != "struct")
		 arr = GetDpArray(dp, GetTypeMeta(dp->type));
	 else
	 {
		 TypeMeta tm;
		 tm.itemsize = dp->dtype->itemsize();
		 tm.np_dtype = dp->type;
		 tm.np_format = dp->dtype->pybind11_format();
		 arr = GetDpArray(dp, tm);
	 }

	 size_t buffLen = dp->size;
	 // copy data to array
	 auto buf = arr.request();
	 {
		 py::gil_scoped_release release;
		 std::unique_lock<std::mutex> lock(m_CnCMutex);
		 result = DI()->GetParameterByDataPoint(dp, buffLen, (uint8_t*)buf.ptr, get_dp_policy);
	 }
	 CheckResult(result);

	 // get first value and validate type
	 auto scalar = getDpScalar(arr, dp);
	 validateNumpyDType(scalar, dp);

	 py::none ret_none;
	 // handle scalars
	 if (dp->Length() == 1)
	 {//scalar
		 if (dp->type == "bool")
			 ret = py::bool_(scalar);
		 else if (dp->type == "float" || dp->type == "double")
			 ret = py::float_(scalar);
		 else
			 ret = py::int_(scalar);
	 }
	 else if (buffLen == 0)
	 {
		 /* In case tap was not initialized */
		 ret = ret_none;
	 }
	 else {
		 ret = arr;
	 }

	 return ret;
 }

 py::dtype PY_DeviceInterface::GetDpDtype(std::string dp_name)
 {

	 invz::Result result;
	 const invz::DataPoint* dp;

	 // get dp
	 result = DI()->GetDataPointByName(dp_name, dp);
	 CheckResult(result);

	 auto ret = py::dtype(GetTypeMeta(dp->type).np_format);

	 return ret;
 }

 py::dtype PY_DeviceInterface::GetDpDtypeById(uint32_t dp_id) {

	 invz::Result result;
	 const invz::DataPoint* dp;

	 // get dp
	 result = DI()->GetDataPointById(dp_id, dp);
	 CheckResult(result);

	 auto ret = py::dtype(GetTypeMeta(dp->type).np_format);

	 return ret;
 }

 void PY_DeviceInterface::SetDp(std::string dp_name, py::object obj, bool set_param)
 {
	
	 invz::Result result;
	 const invz::DataPoint* dp;

	 // get dp
	 result = DI()->GetDataPointByName(dp_name, dp);
	 CheckResult(result);
	 //todo

	 if (py::isinstance<py::str>(obj)) {
		 auto str = obj.cast<std::string>();
		 {
			 py::gil_scoped_release release;
			 std::unique_lock<std::mutex> lock(m_CnCMutex);
			 result = DI()->SetParameterValue(dp, str, set_param);
		 }
		 CheckResult(result);
	 }
	 else if (py::isinstance<py::bool_>(obj))
	 {
		 auto val = obj.cast<bool>();
		 if (dp->type != "bool") {
			 std::string error{ "Invalid set value type. Can't set dp type '" };
			 error.append(dp->type);
			 error.append("' using bool value.");
			 throw std::runtime_error(error);
		 }
		 {
			 py::gil_scoped_release release;
			 std::unique_lock<std::mutex> lock(m_CnCMutex);
			 result = DI()->SetParameterValue<bool>(dp, val, set_param);
		 }
		 CheckResult(result);
	 }
	 else if (py::isinstance<py::int_>(obj))
	 {
		 auto val = obj.cast<int64_t>();
		 {
			 py::gil_scoped_release release;
			 std::unique_lock<std::mutex> lock(m_CnCMutex);
			 setDpPyIntScalar(val, dp, set_param);
		 }
	 }
	 else if (py::isinstance<py::float_>(obj)) {
		 auto val = obj.cast<double>();
		 {
			 py::gil_scoped_release release;
			 std::unique_lock<std::mutex> lock(m_CnCMutex);
			 setDpPyFloatScalar(val, dp, set_param);
		 }
	 }
	 else if (py::isinstance<py::array>(obj))
	 {
		 /* np array */
		 auto arr = py::array(obj);
		 auto buf = arr.request();

		 // validate dp type
		 validateNumpyDType(obj, dp);

		 // validate size
		 if (dp->Length() != buf.size) {
			 std::stringstream msg;
			 msg << "size mismatch: dp length (" << dp->Length() << ") doesn't match input size (" << buf.size << ").";
			 throw std::runtime_error(msg.str());
		 }

		 // set buffer
		 {
			 py::gil_scoped_release release;
			 std::unique_lock<std::mutex> lock(m_CnCMutex);
			 result = DI()->SetParameterByDataPoint(dp, buf.itemsize * buf.size, (uint8_t*)buf.ptr, set_param);
		 }
		 CheckResult(result);
	 }
	 else if (py::dtype(obj, true))
	 {
		 /* scalar */

		 // validate dp type
		 validateNumpyDType(obj, dp);

		 // set buffer
		 auto arr = py::array(obj);
		 auto buf = arr.request();
		 {
			 py::gil_scoped_release release;
			 std::unique_lock<std::mutex> lock(m_CnCMutex);
			 result = DI()->SetParameterByDataPoint(dp, buf.itemsize * buf.size, (uint8_t*)buf.ptr, set_param);
		 }
		 CheckResult(result);
	 }
	 else
	 {
		 std::string msg = "Unsupported input class ";
		 msg.append(py::str(obj));
		 throw std::runtime_error(msg);
	 }
 }

 size_t PY_DeviceInterface::getNumRegisters() 
 {

	 invz::Result result;
	 uint32_t num_registers;

	 result = DI()->GetRegisterCount(num_registers);
	 CheckResult(result);

	 return num_registers;
 }

 invz::Register PY_DeviceInterface::GetRegisterDetails(std::string register_name) 
 {

	 invz::Result result;
	 const invz::Register* reg;
	 result = DI()->GetRegisterByName(register_name, reg);
	 CheckResult(result);

	 return *reg;
 }

 py::list PY_DeviceInterface::GetAllRegistersDetails()
 {

	 invz::Result result;
	 py::list list;

	 auto num_registers = getNumRegisters();

	 for (int i = 0; i < num_registers; i++) {
		 const invz::Register* reg;
		 DI()->GetRegisterByIndex(i, reg);
		 list.append(*(reg));
	 }

	 return list;
 }

 py::tuple PY_DeviceInterface::GetRegisterByName(std::string register_name) {

	 invz::Result result;
	 uint32_t retVal = 0;
	 bool success = true;

	 result = DI()->ReadRegisterByName(register_name, retVal);
	 CheckResult(result);

	 /* Check if succeed */
	 if (result.error_code)
		 success = false;

	 return py::make_tuple(success, retVal);
 }

 py::bool_ PY_DeviceInterface::SetRegisterByName(std::string register_name, uint32_t regValue)
 {

	 invz::Result result;
	 bool success = true;
	 result = DI()->WriteRegisterByName(register_name, regValue);
	 CheckResult(result);

	 /* Check if succeed */
	 if (result.error_code)
		 success = false;

	 return success;
 }

 py::tuple PY_DeviceInterface::GetRegisterByAddress(uint32_t regAddress, uint32_t regWidth, uint32_t bitOffset)
 {

	 invz::Result result;
	 uint32_t retVal = 0;
	 bool success = true;

	 result = DI()->ReadRegisterByAddress(regAddress, regWidth, bitOffset, retVal);
	 CheckResult(result);

	 /* Check if succeed */
	 if (result.error_code)
		 success = false;

	 return py::make_tuple(success, retVal);
 }

 py::bool_ PY_DeviceInterface::SetRegisterByAddress(uint32_t regAddress, uint32_t regWidth, uint32_t bitOffset, uint32_t regValue) 
 {

	 invz::Result result;
	 bool success = true;
	 result = DI()->WriteRegisterByAddress(regAddress, regWidth, bitOffset, regValue);
	 CheckResult(result);

	 /* Check if succeed */
	 if (result.error_code)
		 success = false;

	 return success;
 }

 void PY_DeviceInterface::SetTapActivationState(std::string dp_name, bool should_enable)
 {
	 invz::Result result;
	 const invz::DataPoint* dp;

	 // get dp
	 result = DI()->GetDataPointByName(dp_name, dp);
	 CheckResult(result);

	 result = DI()->SetParameterTapStateByDataPoint(dp, should_enable);
	 CheckResult(result);
 }

 void PY_DeviceInterface::RegisterTapsCallback(std::function<void(PyTapHandler&)> callback)
 {
	 invz::Result result;
	 tapCallbackPy = std::bind(callback, std::placeholders::_1);
	 result = DI()->RegisterTapCallback(tapCallbackCpp);
	 CheckResult(result);
 }

 void PY_DeviceInterface::UnregisterTapsCallback()
 {
	 invz::Result result;
	 result = DI()->UnregisterTapCallback();
	 CheckResult(result);
 }

 void PY_DeviceInterface::RegisterLogsCallback(std::function<void(PyLogHandler&)> callback)
 {
	 invz::Result result;
	 logCallbackPy = std::bind(callback, std::placeholders::_1);
	 result = DI()->RegisterLogsCallback(logCallbackCpp);
	 CheckResult(result);
 }

 void PY_DeviceInterface::UnregisterLogsCallback()
 {
	 invz::Result result;
	 result = DI()->UnregisterLogsCallback();
	 CheckResult(result);
 }

 void PY_DeviceInterface::RegisterNewFrameCallback(std::function<void(uint32_t&)> callback)
 {
	 invz::Result result;
	 frameEventCallbackPy = std::bind(callback, std::placeholders::_1);
	 result = DI()->RegisterFrameCallback(frameEventCallbackCpp);
	 CheckResult(result);
 }

 void PY_DeviceInterface::UnregisterNewFrameCallback()
 {
	 invz::Result result;
	 result = DI()->UnregisterFrameCallback();
	 CheckResult(result);
 }

 void PY_DeviceInterface::RegisterNewTlvCallback(std::function<void(PyTLVPack&)> callback)
 {
	 invz::Result result;
	 tlvCallbackPy = std::bind(callback, std::placeholders::_1);
	 result = DI()->RegisterTlvCallback(tlvCallbackCpp);
	 CheckResult(result);
 }

 void PY_DeviceInterface::UnregisterNewTlvCallback()
 {
	 invz::Result result;
	 result = DI()->UnregisterTlvCallback();
	 CheckResult(result);
 }

 void PY_DeviceInterface::CSHandshake()
 {
	 invz::Result result;
	 result = DI()->CSHandshake();
	 CheckResult(result);
 }
