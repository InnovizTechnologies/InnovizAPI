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

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include <sstream>
#include <limits>
#include <regex>
#include <functional>
#include <chrono>
#include <thread>

#ifndef INVZ4_PIXEL_STREAM
#define INVZ4_PIXEL_STREAM
#endif
#include "interface/ConfigApi.h"
#include "interface/FileReaderApi.h"
#include "interface/DeviceApi.h"
#include "interface/IWriterFactory.h"
#include "protocols/ACP.h"
#include "protocols/BlockTLVs.h"
#include "data_point/Datapoint.h"
#include "common_includes/PointCloudConverters.hpp"
#include "common_includes/VB.hpp"
#include "raw_writers/invz4/INVZ4RawWriter.h"
#include "raw_writers/IRawWriter.h"
#include "common.h"
#include "interface/IUdpReceiver.h"

#define NUM_OF_BLOCKAGE_SEGMENTS 100
#define MAX_USER_BUFFERS 35

#define INNOPY_DEFAULT_RECORD_FILENAME "innopy_record"

using namespace invz;
using namespace pybind11::literals;
namespace py = pybind11;

//namespace iwr = invz::invz_writer;
void checkAttribute(FrameDataAttributes &attr, std::vector<FrameDataAttributes> known)
{
		if (attr.itemSize == 0 || attr.length == 0)
		{
			//if missing info try find by grab type from defined attributes

			for (auto& k : known)
			{
				if (k.known_type == attr.known_type)
				{
					attr = k;
					break;
				}
			}
			//if still not found throw
			if (attr.itemSize == 0 || attr.length == 0)
				throw std::runtime_error("unkown data type missing size or length defintions");
		}
	

}

/* TODO: chenv20181016 - should move to an other file */
void checkResult(invz::Result& result)
{
	/* In case of failure error_code should be different than 0 */
	if (result.error_code)
	{
		/* TODO: change to string or stringstream */
		char error_code[8] = {'\0'};
		snprintf(error_code,8, "%05d: ", result.error_code);
		std::string error_message = "ErrorCode " + std::string(error_code) + result.error_message;
		throw std::runtime_error(error_message);
	}
}

/* TODO: think about change to python types (Templates!!!)*/
struct TypeMeta {
	int itemsize;
	std::string np_dtype;
	std::string np_format; 
};
/*
py::list UserBufferToNPArray(invz::FrameDataUserBuffer buf[], size_t len) {
	py::list ret;

	// iterate throw frame data user buffers and convert to numpy array
	for (int i = 0; i < len; i++) {
		// convert according to type
		if (buf[i].dataAttrs.typeMajor == 0 && buf[i].dataAttrs.typeMinor == 0) {
			auto nparr = Get1DArray(buf[i].dataAttrs.nbytes, GetTypeMeta<uint8_t>()); // in case of invz5 stream
			// memcpy from dataAttrs to nparr
			ret.append(nparr); 
		}
		else {
			throw new std::runtime_error(std::string() + "Un recognized type primary id: " + std::to_string(buf[i].dataAttrs.typeMajor)
				+ ", secodary id: " + std::to_string(buf[i].dataAttrs.typeMinor));
		}
	}

	return ret;
}*/

// for internal usage
template<class T> TypeMeta GetTypeMeta() {
	TypeMeta ret;

	ret.itemsize = sizeof(T);
	ret.np_dtype = py::str(py::dtype::of<T>()).cast<std::string>();
	ret.np_format = py::format_descriptor<T>::format();

	return ret;
}

TypeMeta GetTypeMeta(std::string dp_type) {
	TypeMeta ret;

	if (dp_type.find("int8",0)==0)
		ret = GetTypeMeta<int8_t>();
	else if (dp_type.find("uint8",0)==0)
		ret = GetTypeMeta<uint8_t>();
	else if (dp_type.find("int16",0)==0)
		ret = GetTypeMeta<int16_t>();
	else if (dp_type.find("uint16",0)==0)
		ret = GetTypeMeta<uint16_t>();
	else if (dp_type.find("int32",0)==0)
		ret = GetTypeMeta<int32_t>();
	else if (dp_type.find("uint32",0)==0)
		ret = GetTypeMeta<uint32_t>();
	else if (dp_type.find("char", 0) == 0)
		ret = GetTypeMeta<char>();
	else if (dp_type.find("float", 0) == 0)
		ret = GetTypeMeta<float>();
	else if (dp_type.find("double", 0) == 0)
		ret = GetTypeMeta<double>();
	else if (dp_type.find("bool",0)==0)
		ret = GetTypeMeta<bool>();
	else {
		std::string error{ "Unsupported dp type: " };
		error.append(dp_type);
		throw std::runtime_error(error);
	}

	return ret;
}

py::array Get1DArray(int len, TypeMeta meta) {
	return py::array(py::buffer_info(
		nullptr,            /* Pointer to data (nullptr -> ask NumPy to allocate!) */
		meta.itemsize,			/* Size of one item */
		meta.np_format, /* Buffer format */
		1,          /* How many dimensions? */
		{ len },  /* Number of elements for each dimension */
		{ meta.itemsize }  /* Strides for each dimension */
	));
}

py::array GetDpArray(const invz::DataPoint* dp, TypeMeta tm) {
	size_t rows = dp->rows;
	if (tm.np_dtype != "struct")
		rows = dp->Length() / dp->stride;
		
	size_t cols = dp->stride;

	int ndims = rows > 1 ? 2 : 1;	//note: only 1d and 2d arrays are supported

	std::vector<ssize_t> strides;
	if (ndims == 2) {
		strides.push_back(cols * tm.itemsize);
		strides.push_back(tm.itemsize);
	}
	else {
		strides.push_back(tm.itemsize);
	}
	std::vector<ssize_t> shape;
	if (ndims == 2) {
		shape.push_back(rows);
		shape.push_back(cols);
	}
	else {
		shape.push_back(cols);
	}

	return py::array(py::buffer_info(
		nullptr,            /* Pointer to data (nullptr -> ask NumPy to allocate!) */
		tm.itemsize,     /* Size of one item */
		tm.np_format, /* Buffer format */
		ndims,          /* How many dimensions? */
		shape,  /* Number of elements for each dimension */
		strides  /* Strides for each dimension */
	));
}

py::array GetDpArray(const invz::DataPoint* dp) {
	py::array ret;

	if (dp->type != "struct")
		ret = GetDpArray(dp, GetTypeMeta(dp->type));
	else
	{
		TypeMeta tm;
		tm.itemsize = dp->dtype->itemsize();
		tm.np_dtype = dp->type;
		tm.np_format = dp->dtype->pybind11_format();
		ret = GetDpArray(dp, tm);
		//ret = Get1DArray(1, GetTypeMeta<uint8_t>());
	}

	return ret;
}

template<class T>
void CheckLimitsSigned(long val){
	long min_l;
	long max_l;
	min_l = std::numeric_limits<T>::min();
	max_l = std::numeric_limits<T>::max();
	//auto val = obj.cast<T>();
	if (val<min_l || val>max_l)
	{
		std::string error{ "invalid value" };
		throw std::runtime_error(error);
	}

}

template<class T>
void CheckLimitsUnsigned(long val) {
	unsigned long min_l;
	unsigned long max_l;
	min_l = std::numeric_limits<T>::min();
	max_l = std::numeric_limits<T>::max();
	//auto val = obj.cast<T>();
	if (val<min_l || val>max_l)
	{
		std::string error{ "invalid value" };
		throw std::runtime_error(error);
	}

}

std::string bufferName(FrameDataAttributes attr)
{
	switch (attr.known_type)
	{
	case GRAB_TYPE_METADATA: return "GrabType.GRAB_TYPE_METADATA";
	case GRAB_TYPE_THETA_PHI: return "GrabType.GRAB_TYPE_THETA_PHI";
	//case GRAB_TYPE_GHOST: return "GrabType.GRAB_TYPE_GHOST";
	case GRAB_TYPE_PIXEL_IS_VALID0: return "GrabType.GRAB_TYPE_PIXEL_IS_VALID0";
	case GRAB_TYPE_PIXEL_IS_VALID1: return "GrabType.GRAB_TYPE_PIXEL_IS_VALID1";
	case GRAB_TYPE_PIXEL_IS_VALID2: return "GrabType.GRAB_TYPE_PIXEL_IS_VALID2";
	case GRAB_TYPE_MEASURMENTS_REFLECTION0: return "GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION0";
	case GRAB_TYPE_MEASURMENTS_REFLECTION1: return "GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION1";
	case GRAB_TYPE_MEASURMENTS_REFLECTION2: return "GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION2";
	case GRAB_TYPE_MACRO_PIXEL_META_DATA: return "GrabType.GRAB_TYPE_MACRO_PIXEL_META_DATA";
	case GRAB_TYPE_SINGLE_PIXEL_META_DATA: return "GrabType.GRAB_TYPE_SINGLE_PIXEL_META_DATA";
	case GRAB_TYPE_SUM_PIXEL_META_DATA: return "GrabType.GRAB_TYPE_SUM_PIXEL_META_DATA";
	case GRAB_TYPE_SUMMATION_REFLECTION0: return "GrabType.GRAB_TYPE_SUMMATION_REFLECTION0";
	case GRAB_TYPE_SUMMATION_REFLECTION1: return "GrabType.GRAB_TYPE_SUMMATION_REFLECTION1";
	case GRAB_TYPE_SUMMATION_PIXEL_IS_VALID0: return "GrabType.GRAB_TYPE_SUMMATION_PIXEL_IS_VALID0";
	case GRAB_TYPE_SUMMATION_PIXEL_IS_VALID1: return "GrabType.GRAB_TYPE_SUMMATION_PIXEL_IS_VALID1";
	case GRAB_TYPE_DIRECTIONS: return "GrabType.GRAB_TYPE_DIRECTIONS";
	case GRAB_TYPE_SUMMATION_DIRECTIONS: return "GrabType.GRAB_TYPE_SUMMATION_DIRECTIONS";
	case GRAB_TYPE_PC_PLUS: return "GrabType.GRAB_TYPE_PC_PLUS";
	case GRAB_TYPE_PC_PLUS_SUMMATION: return "GrabType.GRAB_TYPE_PC_PLUS_SUMMATION";
	case GRAB_TYPE_PC_PLUS_METADATA: return "GrabType.GRAB_TYPE_PC_PLUS_METADATA";
	case GRAB_TYPE_PC_PLUS_METADATA_48K: return "GrabType.GRAB_TYPE_PC_PLUS_METADATA_48K";
	case GRAB_TYPE_DETECTIONS: return "GrabType.GRAB_TYPE_DETECTIONS";
	case GRAB_TYPE_DETECTIONS_SI: return "GrabType.GRAB_TYPE_DETECTIONS_SI";
	case GRAB_TYPE_TRACKED_OBJECTS: return "GrabType.GRAB_TYPE_TRACKED_OBJECTS";
	case GRAB_TYPE_TRACKED_OBJECTS_SI: return "GrabType.GRAB_TYPE_TRACKED_OBJECTS_SI";
	case GRAB_TYPE_SENSOR_POSE: return "GrabType.GRAB_TYPE_SENSOR_POSE";
	case GRAB_TYPE_OC_OUTPUT: return "GrabType.GRAB_TYPE_OC_OUTPUT";
	case GRAB_TYPE_DC_OUTPUT: return "GrabType.GRAB_TYPE_DC_OUTPUT";
	case GRAB_TYPE_BLOCKAGE: return "GrabType.GRAB_TYPE_BLOCKAGE";
	case GRAB_TYPE_BLOCKAGE_ENVIRONMENTAL: return "GrabType.GRAB_TYPE_BLOCKAGE_ENVIRONMENTAL";
	case GRAB_TYPE_BLOCKAGE_CLASSIFICATION: return "GrabType.GRAB_TYPE_BLOCKAGE_CLASSIFICATION";
	case GRAB_TYPE_LIDAR_STATUS: return "GrabType.GRAB_TYPE_LIDAR_STATUS";
	case GRAB_TYPE_INS_SIGNALS: return "GrabType.GRAB_TYPE_INS_SIGNALS";
	default:
		std::ostringstream stringStream;
		stringStream << attr.typeMajor<<"_"<<attr.typeMinor;
		return stringStream.str();
		break;
	}
}

struct PyTLV {
	uint32_t type;
	uint16_t length;
	uint16_t reserved;
	py::array value;

	PyTLV()
	{
		type = 0;
		length = 0;
		reserved = 0;
		value = Get1DArray(length, GetTypeMeta<uint8_t>());
	}

	PyTLV(PyTLV const & other) :
		type(other.type),
		length(other.length),
		reserved(other.reserved)
	{
		value = Get1DArray(length, GetTypeMeta<uint8_t>());
		auto pyarrayValue = value.request();
		memcpy(pyarrayValue.ptr, other.value.data(), length);
	}

	PyTLV(invz::eTLV& other) :
		type(other.type),
		length(other.length),
		reserved(other.reserved)
	{
		value = Get1DArray(length, GetTypeMeta<uint8_t>());
		auto pyarrayValue = value.request();
		memcpy(pyarrayValue.ptr, other.value, length);
	}
	
	PyTLV(uint32_t _type, uint16_t _length, py::array _value_array)
	{
		type = _type;
		length = _length;
		reserved = 0;
		value = Get1DArray(length, GetTypeMeta<uint8_t>());
		auto pyarrayValue = value.request();
		memcpy(pyarrayValue.ptr, _value_array.request().ptr, length);
	}

	PyTLV(uint32_t _type, uint16_t _length, uint8_t* _value_buffer)
	{
		type = _type;
		length = _length;
		reserved = 0;
		value = Get1DArray(length, GetTypeMeta<uint8_t>());
		auto pyarrayValue = value.request();
		memcpy(pyarrayValue.ptr, _value_buffer, length);
	}
};

struct PyTapHandler
{
	std::unique_ptr<invz::TapEventData> handler = nullptr;
	uint64_t timestamp = 0;
	uint32_t frame_number = UINT32_MAX;
	uint32_t parameter_id;
	py::object data;

	PyTapHandler()
	{

	}

	PyTapHandler(PyTapHandler const & other)
	{
		handler = std::make_unique<invz::TapEventData>(*(other.handler.get()));
		timestamp = handler->timestamp;
		frame_number = handler->frame_number;
		parameter_id = handler->parameter_id;
		data = other.data;
	}

	PyTapHandler(invz::TapEventData& other, const invz::DataPoint* dp)
	{
		handler = std::make_unique<invz::TapEventData>(other);
		timestamp = handler->timestamp;
		frame_number = handler->frame_number;
		parameter_id = handler->parameter_id;
		auto arr = GetDpArray(dp);
		memcpy(arr.request().ptr, handler->data.get(), handler->length);
		data = arr;
	}
};

struct PyLogHandler
{
	std::unique_ptr<invz::RuntimeLogEventData> handler = nullptr;
	uint64_t timestamp;
	uint32_t frame_number;
	uint32_t core_id;
	uint32_t sequence_number;
	uint32_t flow;
	uint32_t line_number;
	float param0;
	float param1;
	float param2;
	std::string severity;
	std::string package;
	std::string source_file;
	std::string message_name;
	std::string message;

	PyLogHandler(uint64_t _timestamp = UINT64_MAX, uint32_t _frame_number = UINT32_MAX, uint32_t _core_id = 0, uint32_t _sequence_number = 0,
		uint32_t _flow = 0, uint32_t _line_number = 0, float _param0 = 0.0f, float _param1 = 0.0f, float _param2 = 0.0f,
		std::string _severity = "", std::string _package = "", std::string _source_file = "", std::string _message_name = "", std::string _message = "") :
		timestamp{ _timestamp }, frame_number{ _frame_number }, core_id{ _core_id }, sequence_number{ _sequence_number },
		flow{ _flow }, line_number{ _line_number }, param0{ _param0 }, param1{ _param1 }, param2{ _param2 },
		severity{ _severity }, package{ _package }, source_file{ _source_file }, message_name{ _message_name }, message{ _message }
	{

	}

	PyLogHandler(PyLogHandler const& other)
	{
		handler = std::make_unique<invz::RuntimeLogEventData>(*(other.handler.get()));
		timestamp = handler->timestamp;
		frame_number = handler->frame_number;
		core_id = handler->core_id;
		sequence_number = handler->sequence_number;
		flow = handler->flow;
		line_number = handler->line_number;
		param0 = handler->param0;
		param1 = handler->param1;
		param2 = handler->param2;
		severity = handler->severity;
		package = handler->package;
		source_file = handler->source_file;
		message_name = handler->message_name;
		message = handler->message;
	}

	PyLogHandler(invz::RuntimeLogEventData& other)
	{
		handler = std::make_unique<invz::RuntimeLogEventData>(other);
		timestamp = handler->timestamp;
		frame_number = handler->frame_number;
		core_id = handler->core_id;
		sequence_number = handler->sequence_number;
		flow = handler->flow;
		line_number = handler->line_number;
		param0 = handler->param0;
		param1 = handler->param1;
		param2 = handler->param2;
		severity = handler->severity;
		package = handler->package;
		source_file = handler->source_file;
		message_name = handler->message_name;
		message = handler->message;
	}

	~PyLogHandler() = default;
};

struct PyGrabFrameResult
{
	bool success;
	uint32_t frame_number;
	uint64_t timestamp;
	py::dict results;
};

struct PyTLVPack {
	uint32_t virtual_channel = 0;
	uint16_t port = 0;
	invz::AcpHeaderTlvPack acp_header;
	PyTLV tlv;

	PyTLVPack()
	{

	};

	PyTLVPack(PyTLVPack const & other) :
		acp_header(other.acp_header),
		tlv(other.tlv),
		virtual_channel(other.virtual_channel),
		port(other.port)
	{
	}

	PyTLVPack(invz::TlvPack& other) :
		acp_header(other.acp_header),
		tlv(other.tlv),
		virtual_channel(other.virtual_channel),
		port(other.port)
	{
	}

	PyTLVPack(uint32_t _type, uint16_t _length, py::array _data) : tlv(_type, _length, _data)
	{

	};

};

// exposed functions 
py::tuple test() {
	int len = 20;
	auto arr = new invz::MacroPixelFixed[len];

	for (int i = 0; i < len; i++)
	{
		arr[i].header.bits.active_channels = 7;
		arr[i].header.bits.is_blocked = false;
		arr[i].header.bits.pixel_type = 0;
		arr[i].header.bits.short_range_status = 0;
		arr[i].header.bits.summation_type = 0;
		arr[i].header.bits.reserved = 0;
		arr[i].mems_feedback.theta = (float)((float)i / (float)len) * 360.0;
		arr[i].mems_feedback.phi = (float)((float)i / (float)len) * 360.0;
		arr[i].blockage_pw = i * 2;
		for (int j = 0; j < arr[i].header.bits.active_channels + 1; j++)
		{
			arr[i].channels[j].pixel_meta.bits.n_reflections = 3;
			arr[i].channels[j].pixel_meta.bits.short_range = 0;
			arr[i].channels[j].pixel_meta.bits.reflection0_valid = true;
			arr[i].channels[j].pixel_meta.bits.reflection1_valid = true;
			arr[i].channels[j].pixel_meta.bits.reflection2_valid = true;
			arr[i].channels[j].pixel_meta.bits.ghost = 0;
			arr[i].channels[j].noise = rand();
			for (int k = 0; k < arr[i].channels[j].pixel_meta.bits.n_reflections; k++)
			{
				arr[i].channels[j].reflection[k].distance = i;
				arr[i].channels[j].reflection[k].reflectivity = 10;
				arr[i].channels[j].reflection[k].confidence.bits.confidence = 16;
				arr[i].channels[j].reflection[k].confidence.bits.grazing_angle = 0;
			}
			
		}
		
	}

	auto ret = Get1DArray(len, GetTypeMeta<invz::MacroPixelFixed>());

	memcpy(ret.request().ptr, arr, sizeof(arr[0]) * len);

	delete[] arr;

	return py::make_tuple(ret);
}

struct PyDeviceMeta
{
	std::unique_ptr<invz::DeviceMeta> meta = nullptr;
	py::array m_width;
	py::array m_height;
	py::array m_Ri;
	py::array m_di;
	py::array m_vik;

	PyDeviceMeta(py::array lrf_width, py::array lrf_height, py::array Ri, py::array di, py::array vik)
	{
		m_width = lrf_width;
		m_height = lrf_height;
		m_Ri = Ri;
		m_di = di;
		m_vik = vik;
		meta.reset(new invz::DeviceMeta((uint16_t*)lrf_width.request().ptr, (uint8_t*)lrf_height.request().ptr, (invz::ReflectionMatrix*)Ri.request().ptr, (invz::vector3*)di.request().ptr, (invz::ChannelNormal*)vik.request().ptr));
	}
	PyDeviceMeta(const PyDeviceMeta& other)
	{
		m_width = other.m_width;
		m_height = other.m_height;
		m_Ri = other.m_Ri;
		m_di = other.m_di;
		m_vik = other.m_vik;
		meta.reset(new invz::DeviceMeta((uint16_t*)m_width.request().ptr, (uint8_t*)m_height.request().ptr, (invz::ReflectionMatrix*)m_Ri.request().ptr, (invz::vector3*)m_di.request().ptr, (invz::ChannelNormal*)m_vik.request().ptr));
	}

	PyDeviceMeta(const invz::DeviceMeta& otherCpp)
	{
		m_width = Get1DArray(DEVICE_NUM_OF_LRFS, GetTypeMeta<uint16_t>());
		m_height = Get1DArray(DEVICE_NUM_OF_LRFS, GetTypeMeta<uint8_t>());
		m_Ri = Get1DArray(DEVICE_NUM_OF_LRFS, GetTypeMeta<invz::ReflectionMatrix>());
		m_di = Get1DArray(DEVICE_NUM_OF_LRFS, GetTypeMeta<invz::vector3>());
		m_vik = Get1DArray(DEVICE_NUM_OF_LRFS, GetTypeMeta<invz::ChannelNormal>());
		
		memcpy(m_width.request().ptr, otherCpp.lrf_width, sizeof(uint16_t));
		memcpy(m_width.request().ptr, otherCpp.lrf_height, sizeof(uint8_t));
		memcpy(m_Ri.request().ptr, otherCpp.Ri, sizeof(invz::DeviceMeta::Ri));
		memcpy(m_di.request().ptr, otherCpp.di, sizeof(invz::DeviceMeta::di));
		memcpy(m_vik.request().ptr, otherCpp.vik, sizeof(invz::DeviceMeta::vik));

		meta.reset(new invz::DeviceMeta(otherCpp.lrf_width, otherCpp.lrf_height, otherCpp.Ri, otherCpp.di, otherCpp.vik));
	}

	uint32_t get_lrf_count() const
	{
		uint32_t ret = 0;
		if (meta)
			ret = meta->lrf_count;

		return ret;
	}

	~PyDeviceMeta()
	{
		meta.reset();
	}
};

// exposed functions 
struct FrameHelper {

	py::array get_empty_macro_pixels_frame(size_t pixel_count, size_t channel_count, size_t reflection_count) {
		auto arr = new invz::MacroPixelFixed[pixel_count];

		if (channel_count > 8) return py::array();
		if (reflection_count > 3) return py::array();

		for (int i = 0; i < pixel_count; i++)
		{
			arr[i].header.bits.active_channels = channel_count - 1;
			arr[i].header.bits.is_blocked = false;
			arr[i].header.bits.pixel_type = 0;
			arr[i].header.bits.short_range_status = 0;
			arr[i].header.bits.summation_type = 0;
			arr[i].header.bits.reserved = 0;
			arr[i].mems_feedback.theta = 0;
			arr[i].mems_feedback.phi = 0;
			arr[i].blockage_pw = 0;
			for (int j = 0; j < channel_count; j++)
			{
				arr[i].channels[j].pixel_meta.bits.n_reflections = reflection_count;
				arr[i].channels[j].pixel_meta.bits.short_range = 0;
				arr[i].channels[j].pixel_meta.bits.reflection0_valid = reflection_count >= 1 ? true : false;
				arr[i].channels[j].pixel_meta.bits.reflection1_valid = reflection_count >= 2 ? true : false;
				arr[i].channels[j].pixel_meta.bits.reflection2_valid = reflection_count == 3 ? true : false;
				arr[i].channels[j].pixel_meta.bits.ghost = 0;
				arr[i].channels[j].noise = 0;
				for (int k = 0; k < reflection_count; k++)
				{
					arr[i].channels[j].reflection[k].distance = 0;
					arr[i].channels[j].reflection[k].reflectivity = 0;
					arr[i].channels[j].reflection[k].confidence.bits.confidence = 0;
					arr[i].channels[j].reflection[k].confidence.bits.grazing_angle = 0;
				}

			}

		}

		auto ret = Get1DArray(pixel_count, GetTypeMeta<invz::MacroPixelFixed>());

		memcpy(ret.request().ptr, arr, sizeof(arr[0]) * pixel_count);

		delete[] arr;

		return ret;
	}

	py::array get_empty_summation_pixels_frame(size_t pixel_count, size_t channel_count, size_t reflection_count) {
		auto arr = new invz::SummationMacroPixelFixed[pixel_count];

		if (channel_count > 8) return py::array();
		if (reflection_count > 2) return py::array();

		for (int i = 0; i < pixel_count; i++)
		{
			arr[i].header.bits.active_channels = channel_count - 1;
			arr[i].header.bits.pixel_type = 0;
			arr[i].header.bits.summation_type = 0;
			arr[i].header.bits.reserved = 0;
			//arr[i].mems_feedback.theta = 0;
			//arr[i].mems_feedback.phi = 0;
			//arr[i].blockage_pw = 0;
			for (int j = 0; j < channel_count; j++)
			{
				arr[i].channels[j].summation_pixel_meta.bits.n_reflections = reflection_count;
				arr[i].channels[j].summation_pixel_meta.bits.reflection0_valid = reflection_count >= 1 ? true : false;
				arr[i].channels[j].summation_pixel_meta.bits.reflection1_valid = reflection_count >= 2 ? true : false;
				//arr[i].channels[j].summation_pixel_meta.bits.reserved0 = 0;
				arr[i].channels[j].summation_pixel_meta.bits.reserved = 0;
				arr[i].channels[j].noise = 0;
				for (int k = 0; k < reflection_count; k++)
				{
					arr[i].channels[j].reflection[k].distance = 0;
					arr[i].channels[j].reflection[k].reflectivity = 0;
					arr[i].channels[j].reflection[k].confidence.bits.confidence = 0;
					arr[i].channels[j].reflection[k].confidence.bits.grazing_angle = 0;
				}

			}

		}

		auto ret = Get1DArray(pixel_count, GetTypeMeta<invz::SummationMacroPixelFixed>());

		memcpy(ret.request().ptr, arr, sizeof(arr[0]) * pixel_count);

		delete[] arr;

		return ret;
	}

	py::tuple convert_byte_stream_to_macro_pixel_frame(PyDeviceMeta& py_device_meta, py::array byte_stream)
	{
		invz::DeviceMeta device_meta = *(py_device_meta.meta.get());
		size_t macroPixelsPerFrame = 0;
		for (int i = 0; i < device_meta.lrf_count; i++)
		{
			macroPixelsPerFrame += device_meta.lrf_width[i] * device_meta.lrf_height[i];
		}
		auto macroPixelFrame = get_empty_macro_pixels_frame(macroPixelsPerFrame, 8, 3);
		auto summationPixelFrame = get_empty_summation_pixels_frame(macroPixelsPerFrame, 8, 2);
		auto start = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		PointCloudStreamToMacroPixelsFrame(device_meta,
			(invz::INVZ4PixelsStream*)byte_stream.request().ptr,
			(invz::MacroPixelFixed*)macroPixelFrame.request().ptr,
			(invz::SummationMacroPixelFixed*)summationPixelFrame.request().ptr);
		auto end = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		std::cout << "Calc Time: " << (end - start)/1000000.0 << std::endl;
		return py::make_tuple(macroPixelFrame, summationPixelFrame);
	}

	py::array get_direction_by_mems_feedback(PyDeviceMeta deviceMeta, uint8_t lrf, py::array_t<invz::MemsFeedback> memsFeedback)
	{
		auto ret = Get1DArray(INVZ4_MACRO_PIXEL_CHANNEL_COUNT, GetTypeMeta<invz::vector3>());

		invz::DeviceMeta& device_meta = *deviceMeta.meta;
		invz::MemsFeedback* mems_feedback = (invz::MemsFeedback*)(memsFeedback.request().ptr);
		auto result = MemsFeedbackToDirections(device_meta, lrf, *mems_feedback, (invz::vector3*)ret.request().ptr, INVZ4_MACRO_PIXEL_CHANNEL_COUNT);
		checkResult(result);

		return ret;
	}
};

enum GrabFrameType
{
	FRAME_TYPE_FRAME = 1,
	FRAME_TYPE_SUMMATION = 2,
	FRAME_TYPE_BOTH = 3
};

//TODO resotre
std::string api_version() {
	return invz::GetVersion(); 
}

//TODO resotre
void logger_init(std::string filepath, int severity_level) {
	//invz::LoggerInit(filepath, severity_level); 
}

static invz::FrameDataAttributes s_directionsAttrs("directions", DIRECTIONS_TYPE, 0xfffffff0, sizeof(invz::vector3), 192000);
static invz::FrameDataAttributes s_summatioDirectionsAttrs("summation_directions", SUMMATION_DIRECTIONS_TYPE, 0xfffffff0, sizeof(invz::vector3), 48000);

/* Frame Data Types for ndarray allocations */
TypeMeta GetTypeMeta(uint32_t invz_format, uint32_t frame_data_type_major, uint32_t frame_data_type_minor) {
	TypeMeta ret = GetTypeMeta<invz::byte>();
	switch (invz_format)
	{
	case invz::EFileFormat::E_FILE_FORMAT_INVZ4:
	case invz::EFileFormat::E_FILE_FORMAT_INVZ4_4:
	case invz::EFileFormat::E_FILE_FORMAT_INVZ4_5:
		if (frame_data_type_major == invz::POINT_CLOUD_CSAMPLE_METADATA)
		{
			return GetTypeMeta<invz::CSampleFrameMeta>();
		}
		else if (frame_data_type_major == invz::POINT_CLOUD_LIDAR_STATUS)
		{
			return GetTypeMeta<invz::LidarStatus>();
		}
		else if (frame_data_type_major == invz::POINT_CLOUD_END_OF_FRAME)
		{
			return GetTypeMeta < invz::EndOfFrame>();
		}
		else if (frame_data_type_major == invz::POINT_CLOUD_BLOCKAGE_ENVIRONMENTAL)
		{
			return GetTypeMeta < invz::EnvironmentalBlockage>();
		}
		else if (frame_data_type_major == invz::POINT_CLOUD_BLOCKAGE_DETECTION)
		{
			return GetTypeMeta < invz::BlockageDetectionSegment>();
		}
		else if (frame_data_type_major == invz::POINT_CLOUD_BLOCKAGE_CLASSIFICATION)
		{
			return GetTypeMeta < invz::BlockageClassificationSegment>();
		}
		else if (frame_data_type_major == SUMMATION_MEASURMENTS_TYPE)
		{
			return GetTypeMeta < invz::INVZ2SumMeasurementXYZType>();
		}
		else if (frame_data_type_major == POINT_CLOUD_PIXELS_THETA_PHI)
		{
			return GetTypeMeta < invz::MemsFeedback>();
		}
		else if (frame_data_type_major == POINT_CLOUD_PIXELS_IS_VALID || frame_data_type_major == POINT_CLOUD_PIXELS_SUM_IS_VALID)
		{
			return GetTypeMeta <uint8_t>();
		}
		else if (frame_data_type_major == POINT_CLOUD_MACRO_PIXEL_META_DATA)
		{
			return GetTypeMeta < invz::INVZ2MacroMetaData>();
		}
		else if (frame_data_type_major == MEASURMENTS_TYPE)
		{
			return GetTypeMeta < invz::INVZ2MeasurementXYZType>();
		}
		else if (frame_data_type_major == POINT_CLOUD_SINGLE_PIXEL_META_DATA)
		{
			return GetTypeMeta < invz::INVZ2PixelMetaData>();
		}
		else if (frame_data_type_major == POINT_CLOUD_SUM_PIXEL_META_DATA)
		{
			return GetTypeMeta < invz::INVZ2SumPixelMetaData>();
		}
		else if (frame_data_type_major == DIRECTIONS_TYPE || frame_data_type_major == SUMMATION_DIRECTIONS_TYPE)
		{
			return GetTypeMeta < invz::vector3>();
		}
		else if (frame_data_type_major == invz::OBJECT_DETECTION_DEBUG_PORT)
		{
			return GetTypeMeta < invz::ObjectDetection>();
		}
		else if (frame_data_type_major == invz::TRACKED_OBJECT_DEBUG_PORT)
		{
			return GetTypeMeta < invz::TrackedObject>();
		}
		else if (frame_data_type_major == invz::OBJECT_DETECTION || frame_data_type_major == invz::TRACKED_OBJECT)
		{
			return GetTypeMeta < vb_invzbuf::ObjectPodLidar>();
		}
		else if (frame_data_type_major == invz::PC_PLUS_DETECTION)
		{
			return GetTypeMeta < invz::PCPlusDetection>();
		}
		else if (frame_data_type_major == invz::SENSOR_POSE_DEBUG_PORT)
		{
			return GetTypeMeta < invz::Sensor_Pose_Data>();
		}
		else if (frame_data_type_major == invz::OC_OUTPUT_DEBUG_PORT)
		{
			return GetTypeMeta <invz::OCOutput>();
		}
		else if (frame_data_type_major == invz::DC_OUTPUT_DEBUG_PORT)
		{
			return GetTypeMeta <invz::DCOutput>();
		}
		else if (frame_data_type_major == invz::INS_SIGNALS_DEBUG_PORT)
		{
			return GetTypeMeta <invz::INSSignalsStatus>();
		}
		else if (frame_data_type_major == invz::PC_PLUS_METADATA)
		{
			return GetTypeMeta < invz::PCPlusMetaData>();
		}
		else if (frame_data_type_major == invz::PC_PLUS_48K_METADATA)
		{
			return GetTypeMeta < invz::PCPlusMetadata48k>();
		}
		break;
	default:
		break;
	}

	return ret;
}

class PyPCFrameMeta
{
public:
	PyPCFrameMeta(uint32_t _frame_number, uint8_t _scan_mode, uint8_t _system_mode, uint8_t _system_submode, uint32_t _timestamp_internal,
		uint32_t _timestamp_utc_sec, uint32_t _timestamp_utc_micro, uint32_t _fw_version, uint32_t _hw_version, py::array _lidar_serial,
		uint16_t _device_type, uint8_t _active_lrfs, uint8_t _macro_pixel_shape, py::array _rows_in_lrf, py::array _cols_in_lrf, uint32_t _total_number_of_points,
		py::array _R_i, py::array _d_i, py::array _v_i_k) {

		m_frame_meta = std::make_unique<invz::CSampleFrameMeta>(_frame_number, _scan_mode, _system_mode, _system_submode, _timestamp_internal,
			_timestamp_utc_sec, _timestamp_utc_micro, _fw_version, _hw_version,(uint8_t*)_lidar_serial.request().ptr,
			_device_type, _active_lrfs, _macro_pixel_shape, (uint8_t*)_rows_in_lrf.request().ptr, (uint16_t*)_cols_in_lrf.request().ptr, _total_number_of_points,
			(invz::ReflectionMatrix*)_R_i.request().ptr, (invz::vector3*)_d_i.request().ptr, (invz::ChannelNormal*)_v_i_k.request().ptr);

		};
	

	uint32_t frame_number() { return m_frame_meta->frame_number; }
	uint8_t	scan_mode() { return m_frame_meta->scan_mode; }
	uint8_t	system_mode() { return m_frame_meta->system_mode; }
	uint8_t	system_submode() { return m_frame_meta->system_submode; }
	uint32_t timestamp_internal() { return m_frame_meta->timestamp_internal; }
	uint32_t timestamp_utc_secs() { return m_frame_meta->timestamp_utc_secs; }
	uint32_t timestamp_utc_micro() { return m_frame_meta->timestamp_utc_micro; }
	uint32_t fw_version() { return m_frame_meta->fw_version; }
	uint32_t hw_version() { return m_frame_meta->hw_version; }
	py::array lidar_serial_number() { 
		py::array ret = Get1DArray(INVZ4_CSAMPLE_LIDAR_SERIAL, GetTypeMeta<uint8_t>());
		memcpy(ret.request().ptr, m_frame_meta->lidar_serial_number, INVZ4_CSAMPLE_LIDAR_SERIAL);
		return std::move(ret); 
	}
	uint16_t device_type() { return m_frame_meta->device_type; };
	uint8_t active_lrfs() { return m_frame_meta->active_lrfs; };
	uint8_t macro_pixel_shape() { return m_frame_meta->macro_pixel_shape; };
	py::array rows_in_lrf() {
		py::array ret = Get1DArray(DEVICE_NUM_OF_LRFS, GetTypeMeta<uint8_t>());
		memcpy(ret.request().ptr, m_frame_meta->rows_in_lrf, DEVICE_NUM_OF_LRFS * sizeof(uint8_t));
		return std::move(ret);
	};
	py::array cols_in_lrf() {
		py::array ret = Get1DArray(DEVICE_NUM_OF_LRFS, GetTypeMeta<uint16_t>());
		memcpy(ret.request().ptr, m_frame_meta->cols_in_lrf, DEVICE_NUM_OF_LRFS * sizeof(uint16_t));
		return std::move(ret);
	};
	uint32_t total_number_of_points() { return m_frame_meta->total_number_of_points; };
	py::array R_i() {
		py::array ret = Get1DArray(DEVICE_NUM_OF_LRFS, GetTypeMeta<invz::ReflectionMatrix>());
		memcpy(ret.request().ptr, m_frame_meta->R_i, DEVICE_NUM_OF_LRFS * sizeof(invz::ReflectionMatrix));
		return std::move(ret);
	};
	py::array d_i() {
		py::array ret = Get1DArray(DEVICE_NUM_OF_LRFS, GetTypeMeta<invz::vector3>());
		memcpy(ret.request().ptr, m_frame_meta->d_i, DEVICE_NUM_OF_LRFS * sizeof(invz::vector3));
		return std::move(ret);
	};
	py::array v_i_k() {
		py::array ret = Get1DArray(DEVICE_NUM_OF_LRFS, GetTypeMeta<invz::ChannelNormal>());
		memcpy(ret.request().ptr, m_frame_meta->v_i_k, DEVICE_NUM_OF_LRFS * sizeof(invz::ChannelNormal));
		return std::move(ret);
	};

	invz::CSampleFrameMeta getFrameMeta() { return *m_frame_meta; }

private:
	std::unique_ptr<invz::CSampleFrameMeta> m_frame_meta;
};

class FileReader //Python FileReader (wrapping for FileReader:IReader)
{
public:
	std::unique_ptr<invz::IReader> fr = nullptr;
	size_t num_of_frames = -1;
	invz::byte getPacketBuffer[UINT16_MAX];
	std::vector<invz::FrameDataAttributes> frame_data_attrs;
	uint32_t file_format;
	std::map<GrabType, uint8_t*> allBuffers;
	size_t attr_size = 0;

	std::function<void(invz::TapEventData*)> tap_callback_cpp;
	
	std::function<void(PyTapHandler&)> tap_callback_py;

	std::function<void(invz::RuntimeLogEventData*)> log_callback_cpp;

	std::function<void(PyLogHandler&)> log_callback_py;


	FileReader(std::string filepath, uint32_t log_severity, bool check_pixel_validity, uint8_t num_of_cores, std::string config_filepath) {
		invz::Result result;
		// initalize reader interface
		fr.reset(invz::FileReaderInit(filepath,"", log_severity,true, check_pixel_validity, num_of_cores, config_filepath));

		/* Get number of frames in file */
		result = fr->GetNumOfFrames(num_of_frames);
		checkResult(result);

		result = fr->GetFileFormat(file_format);
		checkResult(result);

		/* Get frame data attributes from file */
		invz::FrameDataAttributes attr[INVZ_CONFIG_GET_FRAME_ATTR_DATA_MAX_SIZE];
		size_t attr_count = INVZ_CONFIG_GET_FRAME_ATTR_DATA_MAX_SIZE;
		result = fr->GetFrameDataAttributes(attr, attr_count);

		/* Get frame data attributes */
		for (int i = 0; i < attr_count; i++)
		{
			frame_data_attrs.push_back(attr[i]);
		}

		tap_callback_cpp = [&](invz::TapEventData* th) -> void {
			/* Acquire GIL before calling Python code */
			py::gil_scoped_acquire acquire;
			const invz::DataPoint* dp;
			fr->GetDPById(th->parameter_id, dp);
			PyTapHandler py_th(*th, dp);
			tap_callback_py(py_th);
		};

		log_callback_cpp = [&](invz::RuntimeLogEventData* th) -> void {
			/* Acquire GIL before calling Python code */
			py::gil_scoped_acquire acquire;
			PyLogHandler py_th(*th);
			log_callback_py(py_th);
		};
	}

	~FileReader() {
		// close reader if exists
		if (fr)
			invz::FileReaderClose(fr.release());
	}

	PyGrabFrameResult get_frame(int frame_index, std::vector<FrameDataAttributes> frame_types) {
		invz::Result result;
		PyGrabFrameResult ret;
		
		for (auto& dataAttrs : frame_types)
		{
			checkAttribute(dataAttrs, frame_data_attrs);		//check all types are fully defined
		}

		/* Allocate frame data buffers */
		FrameDataUserBuffer userBuffers[MAX_USER_BUFFERS];
		TypeMeta currentType;
		bool measurment_type = false;
		int count = 0;
		for (auto& dataAttrs : frame_types)
		{
			if (allBuffers.find(dataAttrs.known_type) == allBuffers.end())
			{
				/* Add user buffer to send to API */
				allBuffers[dataAttrs.known_type]=new uint8_t[dataAttrs.nbytes()];
			}
			invz::FrameDataUserBuffer buffer;
			userBuffers[count].dataAttrs = dataAttrs;
			userBuffers[count].dataBuffer = allBuffers[dataAttrs.known_type];
			if (dataAttrs.known_type == GrabType::GRAB_TYPE_PC_PLUS || dataAttrs.known_type == GrabType::GRAB_TYPE_PC_PLUS_SUMMATION || dataAttrs.known_type == GrabType::GRAB_TYPE_TRACKED_OBJECTS_SI)
				userBuffers[count].handle_endianess = true;


			count++;
			
		}

		uint32_t frame_number;
		uint64_t timestamp;
		bool success;
		result = fr->GrabFrame(userBuffers, count, frame_number, timestamp, frame_index);
		success = result.error_code == ERROR_CODE_OK;
		
		for (auto& userBuffer : userBuffers)
		{
			if (userBuffer.dataAttrs.known_type != GRAB_TYPE_UNKOWN)
			{
				if (userBuffer.status == USER_BUFFER_FULL)
				{
					ret.results[py::str(bufferName(userBuffer.dataAttrs))] = Get1DArray(userBuffer.dataAttrs.length, GetTypeMeta(file_format, userBuffer.dataAttrs.typeMajor, userBuffer.dataAttrs.typeMinor));
					auto ptr = (invz::byte*)(((py::array)ret.results[py::str(bufferName(userBuffer.dataAttrs))]).request().ptr);
					std::copy((invz::byte*)userBuffer.dataBuffer, (invz::byte*)(userBuffer.dataBuffer + userBuffer.dataAttrs.nbytes()), ptr);
					if (userBuffer.dataBuffer)
					{
						delete userBuffer.dataBuffer;
						allBuffers.erase(userBuffer.dataAttrs.known_type);
					}
						

				}
				else
					ret.results[py::str(bufferName(userBuffer.dataAttrs))] = py::none();
			}

		}
		ret.frame_number = frame_number;
		ret.success = success;
		ret.timestamp = timestamp;
		return ret;
	}

	py::tuple get_packet()
	{
		invz::Result result;
		size_t packetSize = 0;
		uint64_t timestamp = -1;
		uint16_t rxPort = -1;
		uint32_t rxChannel = -1;

		// get block into getPacketBuffer
		result = fr->GetPacket(getPacketBuffer, UINT16_MAX, packetSize, timestamp, rxPort, rxChannel);
		
		if (!result.error_code) {
			// validate packet size
			if (packetSize > (size_t)UINT16_MAX)
				throw std::runtime_error("Packet size exceed UINT16_MAX upon successfull get block");

			// allocate nd array according to packet size and return it
			auto ret = Get1DArray(packetSize, GetTypeMeta<invz::byte>());
			auto ptr = (invz::byte*)(ret.request().ptr);
			std::copy(getPacketBuffer, getPacketBuffer + packetSize, ptr);

			return py::make_tuple(true, rxPort, rxChannel, timestamp, ret);
		}
		else {
			py::none none;
			return py::make_tuple(false, none, none, none, none);
		}
	}

	py::tuple get_device_meta()
	{
		invz::Result result;
		py::none none;
		py::tuple ret = py::make_tuple(none);	
		return ret;
	}

	py::list get_frame_data_attrs()
	{
		py::list ret;

		for (auto& data_attrs : frame_data_attrs)
		{
			ret.append(data_attrs);
		}
		return ret;
	}

	void register_taps_callback(std::function<void(PyTapHandler&)> callback) {
		invz::Result result;
		tap_callback_py = std::bind(callback, std::placeholders::_1);
		result = fr->RegisterTapsCallback(tap_callback_cpp);
		checkResult(result);
	}

	void unregister_taps_callback() {
		invz::Result result;
		result = fr->UnregisterTapsCallback();
		checkResult(result);
	}

	void grab_taps(int frame_index) {
		invz::Result result;
		result = fr->GrabTaps(frame_index);
		checkResult(result);
	}

	void register_logs_callback(std::function<void(PyLogHandler&)> callback) {
		invz::Result result;
		log_callback_py = std::bind(callback, std::placeholders::_1);
		result = fr->RegisterLogsCallback(log_callback_cpp);
		checkResult(result);
	}

	void unregister_logs_callback() {
		invz::Result result;
		result = fr->UnregisterLogsCallback();
		checkResult(result);
	}

	void grab_logs(int frame_index) {
		invz::Result result;
		result = fr->GrabLogs(frame_index);
		checkResult(result);
	}
};
 
class FileWriter
{
	bool environmentalBlockage = false;
	bool blockageDetection = false;
	bool blockageClassification = false;
public:
	std::unique_ptr<invz::IWriter> fw = nullptr;

	FileWriter(std::string file_name, uint32_t file_format)
	{
		invz::Result result;
		std::string defaultIp("0.0.0.0");
		std::vector<std::pair<uint32_t, uint16_t>> virtualChannelToPort;
		virtualChannelToPort.push_back({ 0,0 });
		virtualChannelToPort.push_back({ 1,1 });
		std::vector<invz::FrameDataAttributes> frame_data_attr = {
			{ "FrameMeta",					invz::POINT_CLOUD_CSAMPLE_METADATA,			UINT32_MAX,					sizeof(invz::CSampleFrameMeta),						1},
			{ "INVZ4Stream",				invz::POINT_CLOUD_INVZ4_PIXELS,				UINT32_MAX,					1,												1024 * 1024 * 5}, // 5 MB for maximal stream of INVZ4 pixels
			{ "EndOfFrame",					invz::POINT_CLOUD_END_OF_FRAME,				UINT32_MAX,					sizeof(invz::EndOfFrame),								1},
			{ "EnvironmentalBlockage",		invz::POINT_CLOUD_BLOCKAGE_ENVIRONMENTAL,		UINT32_MAX,					sizeof(invz::EnvironmentalBlockage),					1},
			{ "BlockageDetection",			invz::POINT_CLOUD_BLOCKAGE_DETECTION,			0,							sizeof(invz::BlockageDetectionSegment),				100},
			{ "BlockageClassification",		invz::POINT_CLOUD_BLOCKAGE_CLASSIFICATION,	0,							sizeof(invz::BlockageClassificationSegment),			100},
		};
		fw.reset(invz::FileWriterInit(file_name, defaultIp, file_format, virtualChannelToPort, frame_data_attr));
	}

	~FileWriter()
	{
		if(fw)
			invz::FileWriterClose(fw.release());
	}

	py::bool_ write_frame(PyPCFrameMeta& frame_meta, py::array macro_pixels_frame, invz::EnvironmentalBlockage environmental_blockage, py::array blockage_detection, py::array blockage_classification)
	{
		invz::BlockageDetectionSegment* blockageDetection = (invz::BlockageDetectionSegment*)(blockage_detection.is_none() ? nullptr : blockage_detection.request().ptr);
		uint8_t* blockageClassification = (uint8_t*)(blockage_classification.is_none() ? nullptr : blockage_classification.request().ptr);
		size_t blockageSegmentCount = 0;
		if (blockageDetection && blockageClassification)
		{
			blockageSegmentCount = (std::min)(blockage_detection.size(), blockage_classification.size());
		}
		else if (blockageDetection)
		{
			blockageSegmentCount = blockage_detection.size();
		}
		else if (blockageClassification)
		{
			blockageSegmentCount = blockage_classification.size();
		}

		invz::Result res = fw->DumpFrameToFile(frame_meta.getFrameMeta(), (invz::MacroPixelFixed*)macro_pixels_frame.request().ptr, macro_pixels_frame.size(), &environmental_blockage,
			blockageDetection, blockageClassification, blockageSegmentCount);

		return res.error_code == invz::ErrorCode::ERROR_CODE_OK;
	}
};

class INVZ4FileRawWriter {
	
private:
	IRawWriter* buildWriter() {

		IRawWriter* writer = new INVZ4RawWriter();
		return writer;
	}
	void closeWriter() {
		
	}


	std::vector<std::pair<uint32_t, uint16_t>> convertToArray(py::list l)
	{
		// allocate ret
		std::vector<std::pair<uint32_t, uint16_t>> ret;
		for (int i = 0; i < l.size(); i++) {
			// todo: remove if from remark and update syntax
			//if (li[i].get_type() != ??list)
			//	throw std::runtime_error("list[" + std::to_string(i) + " is expected to be a list of size of 2.")
			
			py::list li = (py::list)l[i];
			if (li.size() != 2)
				throw std::runtime_error("list[" + std::to_string(i) + " is expected to be a list of size of 2.");

			// updat ret
			ret.push_back(std::make_pair(py::cast<std::uint32_t>(li[0]), py::cast<std::uint16_t>(li[1])));
		}
		return ret;
	}

public: 
	std::unique_ptr<invz::IRawWriter> fw = nullptr;
	INVZ4FileRawWriter(std::string file_name, std::string device_ip, py::list virtual_channels_ports) {
		fw.reset(buildWriter());
		fw->Initialize(file_name, device_ip, convertToArray(virtual_channels_ports));
	}

	~INVZ4FileRawWriter()
	{
		IRawWriter* writer = fw.release();
		delete writer;
	}

	py::bool_ write_payload(uint64_t timestamp, py::array payload , uint16_t port, int32_t frame_number) {
		int i = 0;
		Result res = fw->WritePayload(timestamp, payload.size(), (uint8_t*)(payload.request().ptr), port, frame_number, frame_number > 0);
		return res.error_code == ERROR_CODE_OK;
	}

	py::bool_ finalize()
	{
		Result res = fw->Finalize();
		return res.error_code == ERROR_CODE_OK;
	}
};


typedef void(*PyTapCallback)(PyTapHandler);
typedef void(*PyTlvCallback)(PyTLVPack);

class DeviceInterface
{
public:
	std::unique_ptr<invz::IDevice> m_di = nullptr;
	std::vector<invz::FrameDataAttributes> frame_data_attrs;
	uint8_t connection_level;
	uint32_t file_format = invz::EFileFormat::E_FILE_FORMAT_INVZ4;

	std::function<void(invz::RuntimeLogEventData*)> log_callback_cpp;
	std::function<void(PyLogHandler&)> log_callback_py;

	std::function<void(invz::TapEventData*)> tap_callback_cpp;
	//PyTapCallback tap_callback_py;
	std::function<void(PyTapHandler&)> tap_callback_py;

	std::function<void(invz::TlvPack*)> tlv_callback_cpp;
	//PyTlvCallback tlv_callback_py;
	std::function<void(PyTLVPack&)> tlv_callback_py;

	std::function<void(uint32_t*)> frame_event_callback_cpp;
	std::function<void(uint32_t&)> frame_event_callback_py;

	// TODO restore
	DeviceInterface(std::string config_file_name, bool is_connect, int login_level, std::string password, uint32_t log_severity) {
		// initalize device interface
		invz::Result result;
		connection_level = 0;

		m_di.reset(invz::DeviceInit(config_file_name, "", log_severity));
		size_t attr_count = INVZ_CONFIG_GET_FRAME_ATTR_DATA_MAX_SIZE;
		invz::FrameDataAttributes attr[INVZ_CONFIG_GET_FRAME_ATTR_DATA_MAX_SIZE];

		// TODO add deprecation warning
		if (is_connect)
		{
			result = di()->Connect(connection_level, login_level, password);
			
			//if we failed to connect we need to dispose of di
			if (result.error_code)
			{
				invz::DeviceClose(m_di.release());
			}

			checkResult(result);

			result = di()->GetFileFormat(file_format);
			checkResult(result);
			/* Get frame data attributes from file */
			
			int retries = 10;
			do
			{
				result = di()->GetFrameDataAttributes(attr, attr_count);
				retries--;
				if (result.error_code != ERROR_CODE_OK)
				{
					if (retries == 0)
					{
						attr_count = 0;
						std::cout << "warning: device meta is missing. point cloud grabbing will not be available"<<std::endl;
						break;
					}
					std::this_thread::sleep_for(std::chrono::milliseconds(500));
				}
				else
					break;
			} while (true);

		}
		
		else //allow tcp connection for CM support
			result = di()->EstablishConnection();

			
			tap_callback_cpp = [&](invz::TapEventData* th) -> void {
				/* Acquire GIL before calling Python code */
				py::gil_scoped_acquire acquire;
				const invz::DataPoint* dp;
				di()->GetDataPointById(th->parameter_id, dp);
				PyTapHandler py_th(*th, dp);
				tap_callback_py(py_th);
			};

			log_callback_cpp = [&](invz::RuntimeLogEventData* th) -> void {
				/* Acquire GIL before calling Python code */
				py::gil_scoped_acquire acquire;
				PyLogHandler py_th(*th);
				log_callback_py(py_th);
			};

			tlv_callback_cpp = [&](invz::TlvPack* tlvPack) -> void {
				/* Acquire GIL before calling Python code */
				py::gil_scoped_acquire acquire;
				PyTLVPack py_tlvPack(*tlvPack);
				tlv_callback_py(py_tlvPack);
			};

			frame_event_callback_cpp = [&](uint32_t* frameNumber) -> void {
				/* Acquire GIL before calling Python code */
				py::gil_scoped_acquire acquire;
				frame_event_callback_py(*frameNumber);
			};
		

		//try getting attributes without metadat
		if (attr_count == 0 || !is_connect)
		{
			attr_count = INVZ_CONFIG_GET_FRAME_ATTR_DATA_MAX_SIZE;
			result = di()->GetFrameDataAttributes(attr, attr_count, false);

			if (result.error_code != ERROR_CODE_OK)
				invz::DeviceClose(m_di.release());
			checkResult(result);
		}

		/* Get frame data attributes */
		for (int i = 0; i < attr_count; i++)
		{
			frame_data_attrs.push_back(attr[i]);
		}
	}


	void device_close() {
		py::gil_scoped_release release;
		invz::Result result;

		// close reader if exists
		if (m_di) {
			//result = di->Disconnect();
			checkResult(result);
			invz::DeviceClose(m_di.release());
		}
	}

	~DeviceInterface() {
		device_close();
	}

private:
	void ValidateNumpyDType(py::object obj, const invz::DataPoint* dp) {
		std::string typeStr;
		int itemsize;
		try {
			typeStr = obj.attr("dtype").str().cast<std::string>();
			itemsize = obj.attr("itemsize").cast<int>();
		}
		catch (const std::exception& e) {
			std::string msg = "Unsupported input class ";
			msg.append(obj.attr("__class__").str().cast<std::string>());
			throw std::runtime_error(msg);
		}

		auto tm = GetTypeMeta(dp->type);

		if (tm.np_dtype != typeStr){
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


	// for internal usage
	template<class T> py::object GetDpScalar(py::array arr) {
		py::object type = py::dtype::of<T>().attr("type");
		auto buf = arr.request();
		auto *ptr = (T *)buf.ptr;

		return type(ptr[0]);
	}

	py::object GetDpScalar(py::array arr, const invz::DataPoint* dp) {
		py::object ret;

		// get dp value
		if (dp->type.find("int8", 0) == 0)
			ret = GetDpScalar<int8_t>(arr);
		else if (dp->type.find("uint8",0)==0)
			ret = GetDpScalar<uint8_t>(arr);
		else if (dp->type.find("int16",0)==0)
			ret = GetDpScalar<int16_t>(arr);
		else if (dp->type.find("uint16",0)==0)
			ret = GetDpScalar<uint16_t>(arr);
		else if (dp->type.find("int32",0)==0)
			ret = GetDpScalar<int32_t>(arr);
		else if (dp->type.find("uint32",0)==0)
			ret = GetDpScalar<uint32_t>(arr);
		else if (dp->type == "float")
			ret = GetDpScalar<float>(arr);
		else if (dp->type == "double")
			ret = GetDpScalar<double>(arr);
		else if (dp->type == "bool")
			ret = GetDpScalar<bool>(arr);
		else {
			std::string error{ "Unsupported dp type: " };
			error.append(dp->type);
			throw std::runtime_error(error);
		}

		return ret;
	}


	void SetDpPyIntScalar(int64_t val, const invz::DataPoint* dp, bool set_param) {
		Result result;
		// get dp value
		if (dp->type.find("int8", 0) == 0)
			result = di()->SetParameterValue<int8_t>(dp, val, set_param);
		else if (dp->type.find("uint8",0)==0)
			result = di()->SetParameterValue<uint8_t>(dp, val, set_param);
		else if (dp->type.find("int16",0)==0)
			result = di()->SetParameterValue<int16_t>(dp, val, set_param);
		else if (dp->type.find("uint16",0)==0)
			result = di()->SetParameterValue<uint16_t>(dp, val, set_param);
		else if (dp->type.find("int32",0)==0)
			result = di()->SetParameterValue<int32_t>(dp, val, set_param);
		else if (dp->type.find("uint32",0)==0)
			result = di()->SetParameterValue<uint32_t>(dp, val, set_param);
		else {
			std::string error{ "Invalid set value type. Can't set dp type '" };
			error.append(dp->type);
			error.append("' using int value.");
			throw std::runtime_error(error);
		}
		checkResult(result);
	}

	void SetDpPyFloatScalar(double val, const invz::DataPoint* dp, bool set_param) {
		Result result;
		// get dp value
		if (dp->type == "float")
			result = di()->SetParameterValue<float>(dp, val, set_param);
		else if (dp->type == "double")
			result = di()->SetParameterValue<double>(dp, val, set_param);
		else {
			std::string error{ "Invalid set value type. Can't set dp type '" };
			error.append(dp->type);
			error.append("' using float value.");
			throw std::runtime_error(error);
		}
		checkResult(result);
	}

public:
	IDevice* di() {
		if (m_di) {
			return m_di.get();
		}
		throw std::runtime_error("Invalid call. DeviceInterface not exists.");
	}
	
	size_t get_num_data_points() {
		invz::Result result;
		uint32_t num_data_points;

		result = di()->GetDataPointCount(num_data_points);
		checkResult(result);

		return num_data_points;
	}

	void connect(uint8_t request_level, std::string password) {
		invz::Result result;
		uint8_t actualConnectionLevel = 0;
		result = di()->Connect(actualConnectionLevel, request_level, password);
		checkResult(result);
	}

	void disconnect() {
		invz::Result result;
		result = di()->Disconnect();
		checkResult(result);
	}

	void record(double seconds, std::string filepath, bool flush_queues) {
		invz::Result result;
		result = di()->Record(seconds, filepath, flush_queues);
		checkResult(result);
	}

	void start_recording(std::string filepath, bool flush_queues) {
		invz::Result result;
		result = di()->StartRecording(filepath, flush_queues);
		checkResult(result);
	}

	void stop_recording(double seconds, std::string filepath) {
		invz::Result result;
		result = di()->StopRecording();
		checkResult(result);
	}

	py::bool_ is_connected()
	{
		invz::Result result;
		bool lidar_connected;

		result = di()->IsConnected(lidar_connected);
		checkResult(result);

		return py::bool_(lidar_connected);
	}

	void activate_buffer(FrameDataAttributes frame_type, bool activate)
	{
		checkAttribute(frame_type, frame_data_attrs);
		di()->ActivateBuffer(frame_type, activate);
	}

	py::dict get_statistics()
	{
		py::dict results;
		size_t channels_num;
		ChannelStatistics channels[10];
		auto result = di()->GetConnectionStatus(channels, channels_num);
		if (result.error_code== ERROR_CODE_OK)
		{
			for (size_t i = 0; i < channels_num; i++)
			{
				results[py::str(std::to_string(channels[i].channel_id))] = channels[i];
			}
		}

		return results;
	}
	// TODO restore
	PyGrabFrameResult get_frame(std::vector<FrameDataAttributes> frame_types) {
		invz::Result result;
		
		PyGrabFrameResult ret;
		for (auto& dataAttrs : frame_types)
		{
			checkAttribute(dataAttrs, frame_data_attrs);		//check all types are fully defined
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
		result = di()->GrabFrame(userBuffers.data(), userBuffers.size(), frame_number, timestamp);
		success = result.error_code == ERROR_CODE_OK;

		for (auto& userBuffer : userBuffers)
		{
			if (userBuffer.status == USER_BUFFER_FULL)
			{
				auto buff = Get1DArray(userBuffer.dataAttrs.length, GetTypeMeta(file_format, userBuffer.dataAttrs.typeMajor, userBuffer.dataAttrs.typeMinor));
				auto ptr = (invz::byte*)(buff.request().ptr);
				std::copy((invz::byte*)userBuffer.dataBuffer, (invz::byte*)(userBuffer.dataBuffer + userBuffer.dataAttrs.nbytes()), ptr);
				ret.results[py::str(bufferName(userBuffer.dataAttrs))] = buff;
			}
			else
				ret.results[py::str(bufferName(userBuffer.dataAttrs))] = py::none();

		}
		ret.frame_number = frame_number;
		ret.success = success;
		ret.timestamp = timestamp;
		//auto res= py::make_tuple(success, frame_number, ret);
		return ret;



	}

	py::object build_acp(invz::AcpHeaderTlvPack acp_header, PyTLV request_tlv) {
		py::array arr;
		invz::Result result;

		/* Copy request TLV */
		auto value_buffer = request_tlv.value.request();
		invz::TlvPack requestTlvPack(request_tlv.type, request_tlv.length, (uint8_t*)value_buffer.ptr);
		requestTlvPack.acp_header = acp_header;

		/* Build ACP packet buffer */
		uint8_t acp_buffer[UINT16_MAX]{ 0 };
		size_t acp_buffer_length = UINT16_MAX;
		result = di()->BuildACP(requestTlvPack, acp_buffer, acp_buffer_length);
		checkResult(result);

		/* Copy buffer to ndarray */
		arr = Get1DArray(acp_buffer_length, GetTypeMeta<uint8_t>());
		memcpy(arr.request().ptr, acp_buffer, acp_buffer_length);

		return arr;
	}

	py::tuple send_tlv(invz::AcpHeaderTlvPack acp_header, PyTLV request_tlv, bool return_error_tlv = false)
	{
		invz::Result result;

		/* Copy request TLV */
		auto value_buffer = request_tlv.value.request();
		invz::TlvPack requestTlvPack(request_tlv.type, request_tlv.length, (uint8_t*)value_buffer.ptr);
		requestTlvPack.acp_header = acp_header;
		
		/* Get TLV response */
		invz::TlvPack responseTlvPack;
		result = di()->SendTLV(requestTlvPack, responseTlvPack, return_error_tlv);
		checkResult(result);

		/* Allocate new response */
		PyTLV pyResponseTlv(responseTlvPack.tlv.type, responseTlvPack.tlv.length, responseTlvPack.tlv.value);
		
		py::none none;
		
		return py::make_tuple(responseTlvPack.acp_header, pyResponseTlv, responseTlvPack.port, requestTlvPack.acp_header);
	}

	invz::DataPoint get_dp_details(std::string dp_name) {
		invz::Result result;
		const invz::DataPoint* dp;
		result = di()->GetDataPointByName(dp_name, dp);
		checkResult(result);
		
		return *dp;
	}

	invz::DataPoint get_dp_details_by_id(uint32_t dp_id) {
		invz::Result result;
		const invz::DataPoint* dp;
		result = di()->GetDataPointById(dp_id, dp);
		checkResult(result);

		return *dp;
	}

	py::list get_all_dp_details() {
		invz::Result result;
		py::list list;

		auto num_data_points = get_num_data_points();

		for (int i = 0; i < num_data_points; i++) {
			const invz::DataPoint* data_point;
			di()->GetDataPointByIndex(i, data_point);
			list.append(*(data_point));
		}

		return list;
	}

	py::list get_all_tap_details() {
		invz::Result result;
		py::list list;

		auto num_data_points = get_num_data_points();

		checkResult(result);
		for (int i = 0; i < num_data_points; i++)
		{
			const invz::DataPoint* data_point;
			di()->GetDataPointByIndex(i, data_point);

			if(data_point->dptype == "tap")
				list.append( *(data_point) );
		}
			
		return list;
	}

	py::object get_empty_dp(std::string dp_name) {
		py::array arr;
		invz::Result result;
		const invz::DataPoint* dp;

		// get dp
		result = di()->GetDataPointByName(dp_name, dp);
		checkResult(result);


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

	py::object get_zero_dp(std::string dp_name) {
		py::array arr;
		invz::Result result;
		const invz::DataPoint* dp;

		// get dp
		result = di()->GetDataPointByName(dp_name, dp);
		checkResult(result);
		
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

	py::object get_dp(std::string dp_name, uint32_t get_dp_policy) {
		py::array arr;
		py::object ret;
		invz::Result result;
		const invz::DataPoint* dp;

		// get dp
		result = di()->GetDataPointByName(dp_name, dp);
		checkResult(result);

		// handle strings
		if (dp->type == "char") {
			std::string str;
			result = di()->GetParameterValue(dp, str, get_dp_policy);
			checkResult(result);
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
		result = di()->GetParameterByDataPoint(dp, buffLen, (uint8_t*)buf.ptr, get_dp_policy);
		checkResult(result);

		// get first value and validate type
		auto scalar = GetDpScalar(arr, dp);
		ValidateNumpyDType(scalar, dp);

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
	
	std::vector<int> get_dp_shape(std::string dp_name)
	{
		std::vector<int> ret;
		invz::Result result;
		const invz::DataPoint* dp;

		// get dp
		result = di()->GetDataPointByName(dp_name, dp);
		checkResult(result);

		size_t rows = dp->Length() / dp->stride;
		size_t cols = dp->stride;

		int ndims = rows > 1 ? 2 : 1;	//note: only 1d and 2d arrays are supported

		py::list shape;
		if (ndims == 2) {
			shape.append(rows);
			shape.append(cols);
		}
		else {
			shape.append(cols);
		}
		
		return ret;
	}

	
	py::dtype get_dp_dtype(std::string dp_name) {
		invz::Result result;
		const invz::DataPoint* dp;

		// get dp
		result = di()->GetDataPointByName(dp_name, dp);
		checkResult(result);

		auto ret = py::dtype(GetTypeMeta(dp->type).np_format);

		return ret;
	}

	py::dtype get_dp_dtype_by_id(uint32_t dp_id) {
		invz::Result result;
		const invz::DataPoint* dp;

		// get dp
		result = di()->GetDataPointById(dp_id, dp);
		checkResult(result);

		auto ret = py::dtype(GetTypeMeta(dp->type).np_format);

		return ret;
	}
	
	void set_dp(std::string dp_name, py::object obj, bool set_param) {
		invz::Result result;
		const invz::DataPoint* dp;
		
		// get dp
		result = di()->GetDataPointByName(dp_name, dp);
		checkResult(result);
		//todo


		if (py::str(obj, true).check()) {
			auto str = obj.cast<std::string>();
			result = di()->SetParameterValue(dp, str, set_param);
			checkResult(result);
		}
		else if (py::bool_(obj, true).check())
		{
			auto val = obj.cast<bool>();
			if (dp->type != "bool") {
				std::string error{ "Invalid set value type. Can't set dp type '" };
				error.append(dp->type);
				error.append("' using bool value.");
				throw std::runtime_error(error);
			}
			result = di()->SetParameterValue<bool>(dp, val, set_param);
			checkResult(result);
		}
		else if (py::int_(obj, true).check()) 
		{
			auto val = obj.cast<int64_t>();
			SetDpPyIntScalar(val, dp, set_param);
		}
		else if (py::float_(obj, true).check()) {
			auto val = obj.cast<double>();
			SetDpPyFloatScalar(val, dp, set_param);
		}
		else if (py::array(obj, true).check()) 
		{
			/* np array */
			auto arr = py::array(obj);
			auto buf = arr.request();

			// validate dp type
			ValidateNumpyDType(obj, dp);

			// validate size
			if (dp->Length() != buf.size) {
				std::stringstream msg;
				msg << "size mismatch: dp length (" << dp->Length() << ") doesn't match input size (" << buf.size << ").";
				throw std::runtime_error(msg.str());
			}

			// set buffer
			result = di()->SetParameterByDataPoint(dp, buf.itemsize * buf.size, (uint8_t*)buf.ptr, set_param);
			checkResult(result);
		}
		else if (py::dtype(obj, true))
		{
			/* scalar */

			// validate dp type
			ValidateNumpyDType(obj, dp);

			// set buffer
			auto arr = py::array(obj);
			auto buf = arr.request();
			result = di()->SetParameterByDataPoint(dp, buf.itemsize * buf.size, (uint8_t*)buf.ptr, set_param);
			checkResult(result);
		}
		else
		{
			std::string msg = "Unsupported input class ";
			msg.append(obj.attr("__class__").str().cast<std::string>());
			throw std::runtime_error(msg);
		}
	}

	size_t get_num_registers() {
		invz::Result result;
		uint32_t num_registers;

		result = di()->GetRegisterCount(num_registers);
		checkResult(result);

		return num_registers;
	}

	invz::Register get_register_details(std::string register_name)
	{
		invz::Result result;
		const invz::Register* reg;
		result = di()->GetRegisterByName(register_name, reg);
		checkResult(result);

		return *reg;
	}

	py::list get_all_registers_details()
	{
		invz::Result result;
		py::list list;

		auto num_registers = get_num_registers();

		for (int i = 0; i < num_registers; i++) {
			const invz::Register* reg;
			di()->GetRegisterByIndex(i, reg);
			list.append(*(reg));
		}

		return list;
	}

	py::tuple get_register_by_name(std::string register_name)
	{
		invz::Result result;
		uint32_t retVal = 0;
		bool success = true;

		result = di()->ReadRegisterByName(register_name, retVal);
		checkResult(result);

		/* Check if succeed */
		if (result.error_code)
			success = false;

		return py::make_tuple(success, retVal);
	}

	py::bool_ set_register_by_name(std::string register_name, uint32_t regValue)
	{
		invz::Result result;
		bool success = true;
		result = di()->WriteRegisterByName(register_name, regValue);
		checkResult(result);

		/* Check if succeed */
		if (result.error_code)
			success = false;

		return success;
	}

	py::tuple get_register_by_address(uint32_t regAddress, uint32_t regWidth, uint32_t bitOffset)
	{
		invz::Result result;
		uint32_t retVal = 0;
		bool success = true;

		result = di()->ReadRegisterByAddress(regAddress, regWidth, bitOffset, retVal);
		checkResult(result);

		/* Check if succeed */
		if (result.error_code)
			success = false;

		return py::make_tuple(success, retVal);

	}

	py::bool_ set_register_by_address(uint32_t regAddress, uint32_t regWidth, uint32_t bitOffset, uint32_t regValue)
	{
		invz::Result result;
		bool success = true;
		result = di()->WriteRegisterByAddress(regAddress, regWidth, bitOffset, regValue);
		checkResult(result);

		/* Check if succeed */
		if (result.error_code)
			success = false;

		return success;

	}

	void set_tap_activation_state(std::string dp_name, bool should_enable) {
		invz::Result result;
		const invz::DataPoint* dp;

		// get dp
		result = di()->GetDataPointByName(dp_name, dp);
		checkResult(result);
		
		result = di()->SetParameterTapStateByDataPoint(dp, should_enable);
		checkResult(result);
	}

	void register_taps_callback(std::function<void(PyTapHandler&)> callback) {
		invz::Result result;
		tap_callback_py = std::bind(callback, std::placeholders::_1);
		result = di()->RegisterTapCallback(tap_callback_cpp);
		checkResult(result);
	}

	void unregister_taps_callback() {
		invz::Result result;
		result = di()->UnregisterTapCallback();
		checkResult(result);
	}

	void register_logs_callback(std::function<void(PyLogHandler&)> callback) {
		invz::Result result;
		log_callback_py = std::bind(callback, std::placeholders::_1);
		result = di()->RegisterLogsCallback(log_callback_cpp);
		checkResult(result);
	}

	void unregister_logs_callback() {
		invz::Result result;
		result = di()->UnregisterLogsCallback();
		checkResult(result);
	}

	void register_new_frame_callback(std::function<void(uint32_t&)> callback) {
		invz::Result result;
		frame_event_callback_py = std::bind(callback, std::placeholders::_1);
		result = di()->RegisterFrameCallback(frame_event_callback_cpp);
		checkResult(result);
	}

	void unregister_new_frame_callback() {
		invz::Result result;
		result = di()->UnregisterFrameCallback();
		checkResult(result);
	}


	void register_new_tlv_callback(std::function<void(PyTLVPack&)> callback) {
		invz::Result result;
		tlv_callback_py = std::bind(callback, std::placeholders::_1);
		result = di()->RegisterTlvCallback(tlv_callback_cpp);
		checkResult(result);
	}

	void unregister_new_tlv_callback() {
		invz::Result result;
		result = di()->UnregisterTlvCallback();
		checkResult(result);
	}
};

struct PyPacketContainer
{
	std::unique_ptr<invz::PacketContainer> handler;
	uint64_t timestamp = UINT64_MAX;
	uint32_t length = 0;
	py::array payload;

	PyPacketContainer()
	{

	}

	PyPacketContainer(PyPacketContainer const & other)
	{
		handler = std::make_unique<invz::PacketContainer>(*(other.handler.get()));
		timestamp = other.timestamp;
		length = other.length;
		payload = Get1DArray(handler->length, GetTypeMeta<uint8_t>());
		memcpy(payload.request().ptr, handler->payload.get(), handler->length);
	}

	PyPacketContainer(invz::PacketContainer& other)
	{
		handler = std::make_unique<invz::PacketContainer>(other);
		timestamp = other.timestamp;
		length = other.length;
		payload = Get1DArray(handler->length, GetTypeMeta<uint8_t>());
		memcpy(payload.request().ptr, handler->payload.get(), handler->length);
	}
};

class PyUdpReceiver
{
	std::unique_ptr<invz::IUdpReceiver> udpR = nullptr;
	
	std::function<bool(uint16_t, PacketContainer*)> udp_packet_callback_cpp;
	std::function<void(uint16_t, PyPacketContainer&)> udp_packet_callback_py;

public: 
		PyUdpReceiver(const std::string ipAddress, const int lidarPort, uint32_t marker, uint32_t log_severity) {
			udpR.reset(invz::UdpReceiverInit(marker, ipAddress, lidarPort, log_severity));
			udp_packet_callback_cpp = [&](uint16_t port, invz::PacketContainer* pc) -> bool{
				/* Acquire GIL before calling Python code */
				py::gil_scoped_acquire acquire;
				PyPacketContainer py_container(*pc);
				udp_packet_callback_py(port, py_container);
				return true;
			};
		}

		
		~PyUdpReceiver() {
			if (udpR)
			{
				udpR->CloseConnection();
				udpR.release();
			}
		}

		void registerCallback(std::function<bool(uint16_t, PyPacketContainer&)> callback) {
			
			udp_packet_callback_py = std::bind(callback, std::placeholders::_1, std::placeholders::_2);
			udpR->RegisterCallback(udp_packet_callback_cpp);
		}
		bool startListening() {
			return udpR->StartListening();
		}
		bool stopListening() {
			return udpR->CloseConnection();
		}
};

PYBIND11_MODULE(api, m) {

	m.doc() = R"pbdoc(
        innoviz API python package
        -----------------------

        .. currentmodule:: api

        .. autosummary::
           :toctree: _generate

           api_version
    )pbdoc";

	m.def("api_version", &api_version, R"pbdoc(
        innovizApi version
    )pbdoc");


	m.def("test", &test);
	
	PYBIND11_NUMPY_DTYPE(invz::vector3, x, y, z);

	PYBIND11_NUMPY_DTYPE(invz::INVZ2MeasurementXYZType, distance, confidence, grazing_angle, reflectivity, noise, x, y, z, validity, pfa);
	PYBIND11_NUMPY_DTYPE(invz::INVZ2SumMeasurementXYZType, distance, confidence, reflectivity, noise, x, y, z, validity, pfa);
	PYBIND11_NUMPY_DTYPE(invz::INVZ2MacroMetaData, pixel_type, summation_type,num_active_channels,is_blocked,short_range_detector_status,mems_feedback_x,mems_feedback_y,blockage_pulse_width);
	PYBIND11_NUMPY_DTYPE(invz::INVZ2PixelMetaData, n_reflection, short_range_reflection, ghost,reflection_valid_0, reflection_valid_1, reflection_valid_2);
	PYBIND11_NUMPY_DTYPE(invz::INVZ2SumPixelMetaData, sum_n_reflection,sum_short_range_reflection, sum_ghost, sum_reflection_valid_0, sum_reflection_valid_1);
	PYBIND11_NUMPY_DTYPE(invz::LidarStatus, system_mode, num_ind_pc_info, error_code, timestamp_sec, timestamp_usec, vbat, indications);
	PYBIND11_NUMPY_DTYPE(invz::ChannelStatistics, channel_id, packets, valid_packets, missed_packets, recieved_bytes, data_rate);
	

	
	PYBIND11_NUMPY_DTYPE(invz::EndOfFrame, frame_number, reserved);
	
	
	PYBIND11_NUMPY_DTYPE(invz::Point3D, x, y, z, ValidBitmap);
	PYBIND11_NUMPY_DTYPE(invz::Dimension, width, length, height, ValidBitmap);
	PYBIND11_NUMPY_DTYPE(invz::DimOcclusion, Dim, Occl); /* Occl for "Occlusion" enum */
	PYBIND11_NUMPY_DTYPE(invz::AxisAngle, angle, angle_speed, ValidBitmap);
	PYBIND11_NUMPY_DTYPE(invz::ClassProbability, percentage);
	PYBIND11_NUMPY_DTYPE(invz::CovarianceMatrix, xx, yy, xy);
	PYBIND11_NUMPY_DTYPE(invz::top_view_2d_box, x0, y0, x1, y1);
	PYBIND11_NUMPY_DTYPE(invz::ObjectDetection, time_stamp, unique_id, coord_system, ref_point_type, position, dim_and_occlusion,
		inner_dimensions, axis_angle_params, probability_of_classtype, existance_probability, position_cov_matrix, box_2d);
	PYBIND11_NUMPY_DTYPE(invz::Sensor_Pose_Data, frame_id, timestamps_mili_sec, sp_pitch_deg, sp_roll_deg, sp_z_cm, sp_plane, sp_mat3x3, n_of_inliers, sp_fit_quality_0_to_1, reserved);
	PYBIND11_NUMPY_DTYPE(invz::DisplacementVector, x, y, Z, ValidBitmap);
	PYBIND11_NUMPY_DTYPE(invz::TrackedObject, time_stamp, unique_id, coord_system, ref_point_type, position, dim_and_occlusion,
		inner_dimentions, axis_angle_params, absulute_speed, relative_speed, absolute_acceleration, relative_acceleration,
		measurment_status, movement_status, probability_of_classtype, existance_probability, position_cov_matrix,
		speed_cov_matrix, acceleration_cov_matrix);
	PYBIND11_NUMPY_DTYPE(invz::PCPlusDetection, distance, positive_predictive_value, reflectivity, classification, confidence, angle_azimuth,
		angle_elevation);
	PYBIND11_NUMPY_DTYPE(invz::OCSensorPose, yaw_deg, pitch_deg, roll_deg, z_above_ground_cm);
	PYBIND11_NUMPY_DTYPE(invz::OCOutput, is_oc_data_not_valid, oc_pose_status, oc_pose, oc_state, oc_age_minutes, oc_distance_meters,
		oc_estimated_accuracy, frame_id);
	PYBIND11_NUMPY_DTYPE(invz::DCOutput, frame_id, pitch, roll, z_cm);\
	PYBIND11_NUMPY_DTYPE(invz::ReflectionAttributes, distance, reflectivity, confidence.value);\
	PYBIND11_NUMPY_DTYPE(invz::SinglePixelFixed, pixel_meta.value, noise, reflection);
	PYBIND11_NUMPY_DTYPE(invz::MemsFeedback, theta, phi);
	PYBIND11_NUMPY_DTYPE(invz::MacroPixelFixed, header.value, mems_feedback, blockage_pw, channels);

	PYBIND11_NUMPY_DTYPE(invz::SummationSinglePixelFixed, summation_pixel_meta.value, noise, reflection);
	PYBIND11_NUMPY_DTYPE(invz::SummationMacroPixelFixed, header.value, channels);

	PYBIND11_NUMPY_DTYPE(invz::ReflectionMatrix, matrix);
	PYBIND11_NUMPY_DTYPE(invz::ChannelNormal, channels);

	PYBIND11_NUMPY_DTYPE(invz::EnvironmentalBlockage, frame_number, fov_state, lidar_dq_data_not_valid, reserved, error_reason);
	PYBIND11_NUMPY_DTYPE(invz::BlockageDetectionSegment, blocked, coverage_percentage, gradient, reserved);
	PYBIND11_NUMPY_DTYPE(invz::BlockageClassificationSegment, classification);
	PYBIND11_NUMPY_DTYPE(invz::CSampleFrameMeta, frame_number, scan_mode, reserved1, system_mode, system_submode, timestamp_internal, 
		timestamp_utc_secs, timestamp_utc_micro, fw_version, hw_version, lidar_serial_number, device_type, active_lrfs, macro_pixel_shape, 
		rows_in_lrf, cols_in_lrf, total_number_of_points, reserved2, R_i, d_i, v_i_k);
	PYBIND11_NUMPY_DTYPE(vb_invzbuf::ObjectSummary, id, age, statusMeasurement, statusMovement); \
	PYBIND11_NUMPY_DTYPE(vb_invzbuf::ObjectExistence, invalidFlags, existenceProbability, existencePpv); \
	PYBIND11_NUMPY_DTYPE(vb_invzbuf::SensorStatusFlags, sensorStatusFlags); \
	PYBIND11_NUMPY_DTYPE(vb_invzbuf::EmMeter, value); \
	PYBIND11_NUMPY_DTYPE(vb_invzbuf::EmCentimeter, value); \
	PYBIND11_NUMPY_DTYPE(vb_invzbuf::EmMeterPerSecond, value); \
	PYBIND11_NUMPY_DTYPE(vb_invzbuf::EmMeterPerSecondSquared, value); \
	PYBIND11_NUMPY_DTYPE(vb_invzbuf::EmRadian, value); \
	PYBIND11_NUMPY_DTYPE(vb_invzbuf::EmPercent, value); \
	PYBIND11_NUMPY_DTYPE(vb_invzbuf::ObjectPosition3d, invalidFlags, referencePoint, x, xStdDev, y, yStdDev, z, zStdDev, covarianceXy, orientation, orientationStdDev); \
	PYBIND11_NUMPY_DTYPE(vb_invzbuf::ObjectVelocity, invalidFlags, x, xStdDev, y, yStdDev, covarianceXy); \
	PYBIND11_NUMPY_DTYPE(vb_invzbuf::ObjectAcceleration, invalidFlags, x, xStdDev, y, yStdDev, covarianceXy); \
	PYBIND11_NUMPY_DTYPE(vb_invzbuf::NormalDistributedValue, invalidFlags, mean, stdDev); \
	PYBIND11_NUMPY_DTYPE(vb_invzbuf::ObjectDynamics, velocityAbsolute, velocityRelative, accelerationAbsolute, accelerationRelative, orientationRate); \
	PYBIND11_NUMPY_DTYPE(vb_invzbuf::ShapeEdge, status, edge); \
	PYBIND11_NUMPY_DTYPE(vb_invzbuf::ShapeBoundingBox3d, length, width, height); \
	PYBIND11_NUMPY_DTYPE(vb_invzbuf::ObjectClassification, classCar, classTruck, classMotorcycle, classBicycle, classPedestrian, classAnimal, classHazard, classUnknown, classOverdrivable, classUnderdrivable); \
	PYBIND11_NUMPY_DTYPE(vb_invzbuf::ObjectLights, statusFlashLight, statusBrakeLight, statusHeadLight, statusRearLight, statusReverseLight, statusEmergencyLight); \
	PYBIND11_NUMPY_DTYPE(vb_invzbuf::ObjectPodLidar, summary, existence, statusSensors, position, dynamics, shape3d, classification, lights); \
	PYBIND11_NUMPY_DTYPE(vb_invzbuf::DetectionLidar, distance, positivePredictiveValue, reflectivity, classification, confidence, angleAzimuth, angleElevation);\
		PYBIND11_NUMPY_DTYPE(invz::CoordinateSystemOrigin, invalid_flags, x, x_std_dev, y, y_std_dev, z,
			z_std_dev, roll, roll_std_dev, pitch, pitch_std_dev, yaw, yaw_std_dev);\
    PYBIND11_NUMPY_DTYPE(invz::StndTimestamp, fractional_seconds, seconds, sync_status);\
	PYBIND11_NUMPY_DTYPE(invz::DetectionListHeader, event_data_qualifier, extended_qualifier, time_stamp, origin);\
	PYBIND11_NUMPY_DTYPE(invz::PCPlusMetaData, header, number_of_detections, number_of_detections_low_res_left_origin, number_of_detections_low_res_right_origin, number_of_detections_high_res_left_origin
		, number_of_detections_high_res_right_origin, left_origin_in_sensor_origin, right_origin_in_sensor_origin);\
	PYBIND11_NUMPY_DTYPE(invz::PCPlusMetadata48k, header, number_of_detections, number_of_detections_roi_left_origin, number_of_detections_roi_right_origin, number_of_detections_outer_left_origin
		, number_of_detections_outer_right_origin, left_origin_in_sensor_origin, right_origin_in_sensor_origin, lidarPowerMode, integrityDetectionListLidar);
	PYBIND11_NUMPY_DTYPE(invz::INSSignalsStatus, numOfValidVsInput, frameId);



/*	PYBIND11_NUMPY_DTYPE(invz::PointCloud, x, y, z, I);*/

	py::class_<PyDeviceMeta>(m, "DeviceMeta")
		.def(py::init<py::array, py::array, py::array, py::array, py::array>(),
			"lrf_width"_a,
			"lrf_height"_a,
			"Ri"_a,
			"di"_a,
			"vik"_a)
		.def_property_readonly("lrf_count", &PyDeviceMeta::get_lrf_count)
		.def_readonly("lrf_width", &PyDeviceMeta::m_width)
		.def_readonly("lrf_height", &PyDeviceMeta::m_height)
		.def_readonly("di", &PyDeviceMeta::m_di)
		.def_readonly("Ri", &PyDeviceMeta::m_Ri)
		.def_readonly("vik", &PyDeviceMeta::m_vik);
		
	py::enum_<invz::PixelValidity>(m, "PixelValidity")
		.value("PIXEL_VALIDITY_MISSING", invz::PixelValidity::PIXEL_VALIDITY_MISSING)
		.value("PIXEL_VALIDITY_VALID", invz::PixelValidity::PIXEL_VALIDITY_VALID)
		.value("PIXEL_VALIDITY_INVALID", invz::PixelValidity::PIXEL_VALIDITY_INVALID);

	py::enum_<invz::DeviceType>(m, "DeviceType")
		.value("DEVICE_TYPE_PRO", invz::DeviceType::pro_lidar)
		.value("DEVICE_TYPE_CM", invz::DeviceType::compute_module);
	
	py::enum_<GrabFrameType>(m, "GrabFrameType")
		.value("FRAME_TYPE_FRAME", GrabFrameType::FRAME_TYPE_FRAME)
		.value("FRAME_TYPE_SUMMATION", GrabFrameType::FRAME_TYPE_SUMMATION)
		.value("FRAME_TYPE_BOTH", GrabFrameType::FRAME_TYPE_BOTH);
	py::enum_<GrabType>(m, "GrabType")
		.value("GRAB_TYPE_BLOCKAGE", GrabType::GRAB_TYPE_BLOCKAGE)
		.value("GRAB_TYPE_DETECTIONS", GrabType::GRAB_TYPE_DETECTIONS)
		.value("GRAB_TYPE_DETECTIONS_SI", GrabType::GRAB_TYPE_DETECTIONS_SI)
		.value("GRAB_TYPE_DIRECTIONS", GrabType::GRAB_TYPE_DIRECTIONS)
		.value("GRAB_TYPE_MEASURMENTS_REFLECTION0", GrabType::GRAB_TYPE_MEASURMENTS_REFLECTION0)
		.value("GRAB_TYPE_MEASURMENTS_REFLECTION1", GrabType::GRAB_TYPE_MEASURMENTS_REFLECTION1)
		.value("GRAB_TYPE_MEASURMENTS_REFLECTION2", GrabType::GRAB_TYPE_MEASURMENTS_REFLECTION2)
		.value("GRAB_TYPE_MACRO_PIXEL_META_DATA", GrabType::GRAB_TYPE_MACRO_PIXEL_META_DATA)
		.value("GRAB_TYPE_SINGLE_PIXEL_META_DATA", GrabType::GRAB_TYPE_SINGLE_PIXEL_META_DATA)
		.value("GRAB_TYPE_SUM_PIXEL_META_DATA", GrabType::GRAB_TYPE_SUM_PIXEL_META_DATA)
		//.value("GRAB_TYPE_GHOST", GrabType::GRAB_TYPE_GHOST)
		.value("GRAB_TYPE_PIXEL_IS_VALID0", GrabType::GRAB_TYPE_PIXEL_IS_VALID0)
		.value("GRAB_TYPE_PIXEL_IS_VALID1", GrabType::GRAB_TYPE_PIXEL_IS_VALID1)
		.value("GRAB_TYPE_PIXEL_IS_VALID2", GrabType::GRAB_TYPE_PIXEL_IS_VALID2)
		.value("GRAB_TYPE_METADATA", GrabType::GRAB_TYPE_METADATA)
		.value("GRAB_TYPE_PC_PLUS_SUMMATION", GrabType::GRAB_TYPE_PC_PLUS_SUMMATION)
		.value("GRAB_TYPE_PC_PLUS", GrabType::GRAB_TYPE_PC_PLUS)
		.value("GRAB_TYPE_PC_PLUS_METADATA", GrabType::GRAB_TYPE_PC_PLUS_METADATA)
		.value("GRAB_TYPE_PC_PLUS_METADATA_48K", GrabType::GRAB_TYPE_PC_PLUS_METADATA_48K)
		.value("GRAB_TYPE_SUMMATION_DIRECTIONS", GrabType::GRAB_TYPE_SUMMATION_DIRECTIONS)
		.value("GRAB_TYPE_SUMMATION_REFLECTION0", GrabType::GRAB_TYPE_SUMMATION_REFLECTION0)
		.value("GRAB_TYPE_SUMMATION_REFLECTION1", GrabType::GRAB_TYPE_SUMMATION_REFLECTION1)
		.value("GRAB_TYPE_SUMMATION_PIXEL_IS_VALID0", GrabType::GRAB_TYPE_SUMMATION_PIXEL_IS_VALID0)
		.value("GRAB_TYPE_SUMMATION_PIXEL_IS_VALID1", GrabType::GRAB_TYPE_SUMMATION_PIXEL_IS_VALID1)
		.value("GRAB_TYPE_TRACKED_OBJECTS", GrabType::GRAB_TYPE_TRACKED_OBJECTS)
		.value("GRAB_TYPE_TRACKED_OBJECTS_SI", GrabType::GRAB_TYPE_TRACKED_OBJECTS_SI)
		.value("GRAB_TYPE_SENSOR_POSE", GrabType::GRAB_TYPE_SENSOR_POSE)
		.value("GRAB_TYPE_OC_OUTPUT", GrabType::GRAB_TYPE_OC_OUTPUT)
		.value("GRAB_TYPE_DC_OUTPUT", GrabType::GRAB_TYPE_DC_OUTPUT)
		.value("GRAB_TYPE_LIDAR_STATUS", GrabType::GRAB_TYPE_LIDAR_STATUS)
		.value("GRAB_TYPE_BLOCKAGE_ENVIRONMENTAL", GrabType::GRAB_TYPE_BLOCKAGE_ENVIRONMENTAL)
		.value("GRAB_TYPE_BLOCKAGE_CLASSIFICATION", GrabType::GRAB_TYPE_BLOCKAGE_CLASSIFICATION)
		.value("GRAB_TYPE_THETA_PHI", GrabType::GRAB_TYPE_THETA_PHI)
		.value("GRAB_TYPE_UNKOWN", GrabType::GRAB_TYPE_UNKOWN)
		.value("GRAB_TYPE_INS_SIGNALS", GrabType::GRAB_TYPE_INS_SIGNALS);


	py::enum_<invz::ErrorCode>(m, "ErrorCode")
		.value("ERROR_CODE_OK", invz::ErrorCode::ERROR_CODE_OK)
		.value("ERROR_CODE_GENERAL", invz::ErrorCode::ERROR_CODE_GENERAL)
		.value("ERROR_CODE_CONNECTION", invz::ErrorCode::ERROR_CODE_CONNECTION)
		.value("ERROR_CODE_INVALID_DATA_POINT", invz::ErrorCode::ERROR_CODE_INVALID_DATA_POINT)
		.value("ERROR_CODE_FILE_ERROR", invz::ErrorCode::ERROR_CODE_FILE_ERROR)
		.value("ERROR_CODE_INVALID_FRAME", invz::ErrorCode::ERROR_CODE_INVALID_FRAME)
		.value("ERROR_CODE_INVALID_INPUT", invz::ErrorCode::ERROR_CODE_INVALID_INPUT)
		.value("ERROR_CODE_DEVICE_ERROR", invz::ErrorCode::ERROR_CODE_DEVICE_ERROR)
		.value("ERROR_CODE_NOT_SUPPORTED", invz::ErrorCode::ERROR_CODE_NOT_SUPPORTED);

    py::class_<ndarray::LessThanFilterAttr>(m, "LessThanFilterAttr")
        .def(py::init<std::string, double>(), "selector"_a, "threshold"_a = 0.)
        .def_readwrite("threshold", &ndarray::LessThanFilterAttr::threshold)
        .def_readwrite("selector", &ndarray::LessThanFilterAttr::FilterAttr::selector)
        .def_readonly("filter_type", &ndarray::LessThanFilterAttr::FilterAttr::type);

    py::class_<ndarray::NoiseFilterAttr>(m, "NoiseFilterAttr")
        .def(py::init<const double*, const double*, const uint16_t*>(),
             "data_ranges"_a, "neighbor_threshold"_a, "segment_size_threshold"_a)
        .def_readonly("data_ranges", &ndarray::NoiseFilterAttr::data_ranges)
        .def_readonly("neighbor_threshold", &ndarray::NoiseFilterAttr::neighbor_threshold)
        .def_readonly("segment_size_threshold", &ndarray::NoiseFilterAttr::segment_size_threshold)
        .def_readonly("selector", &ndarray::NoiseFilterAttr::FilterAttr::selector)
        .def_readonly("filter_type", &ndarray::NoiseFilterAttr::FilterAttr::type);

	py::class_<invz::FrameDataAttributes>(m, "FrameDataAttributes")
		.def(py::init<GrabType>(), "grab_type"_a)
		.def(py::init<std::string, uint32_t, uint32_t, uint32_t, uint32_t, bool, GrabType, std::string>(),
             "typeName"_a = "Unknown",  "typeMajor"_a = UINT32_MAX, "typeMinor"_a = UINT32_MAX, "itemSize"_a = 0,
             "length"_a = 0, "optional"_a = false, "grab_type"_a = invz::GRAB_TYPE_UNKOWN, "dtype_format"_a = "")
        .def("add_filter", &invz::FrameDataAttributes::AddFilter, "filter_attr"_a)
        .def("remove_filter_type", &invz::FrameDataAttributes::RemoveFilterType, "filter_type"_a)
        .def("remove_all_filters", &invz::FrameDataAttributes::RemoveAllFilters)
		.def_readwrite("type_name", &invz::FrameDataAttributes::typeName)
		.def_readwrite("item_size", &invz::FrameDataAttributes::itemSize)
		.def_readwrite("length", &invz::FrameDataAttributes::length)
		.def_readwrite("type_major", &invz::FrameDataAttributes::typeMajor)
		.def_readwrite("type_minor", &invz::FrameDataAttributes::typeMinor)
		.def_readwrite("grab_type", &invz::FrameDataAttributes::known_type)
		.def_property_readonly("nbytes", &invz::FrameDataAttributes::nbytes)
        .def_readwrite("filter_attrs", &invz::FrameDataAttributes::filterAttrs)
        .def_readwrite("dtype_format", &invz::FrameDataAttributes::dtype_format);

	py::class_<invz::ChannelStatistics>(m, "ChannelStatistics")
		.def_readwrite("channel_id", &invz::ChannelStatistics::channel_id)
		.def_readwrite("data_rate", &invz::ChannelStatistics::data_rate)
		.def_readwrite("missed_packets", &invz::ChannelStatistics::missed_packets)
		.def_readwrite("packets", &invz::ChannelStatistics::packets)
		.def_readwrite("recieved_bytes", &invz::ChannelStatistics::recieved_bytes)
		.def_readwrite("grab_type", &invz::ChannelStatistics::valid_packets);

	py::class_<invz::DataPoint>(m, "DataPoint")
		.def_readonly("id", &invz::DataPoint::id)
		.def_readonly("name", &invz::DataPoint::name)
		.def_readonly("dptype", &invz::DataPoint::dptype)
		.def_readonly("type", &invz::DataPoint::type)
		.def_readonly("container", &invz::DataPoint::container)
		.def_readonly("size", &invz::DataPoint::size)
		.def_readonly("stride", &invz::DataPoint::stride)
		.def_readonly("desc", &invz::DataPoint::desc)
		.def_readonly("rasdescription", &invz::DataPoint::rasdescription)
		.def_readonly("read_access_level", &invz::DataPoint::read_access_level)
		.def_readonly("write_access_level", &invz::DataPoint::write_access_level)
		.def("display", &invz::DataPoint::PrintSelf);

	py::class_<invz::Register>(m, "Register")
		.def_readonly("container", &invz::Register::container)
		.def_readonly("name", &invz::Register::name)
		.def_readonly("description", &invz::Register::description)
		.def_readonly("base_address", &invz::Register::baseAddress)
		.def_readonly("offset", &invz::Register::offset)
		.def_readonly("width", &invz::Register::width)
		.def_readonly("bit_offset", &invz::Register::bitOffset)
		.def_readonly("access", &invz::Register::access);

	py::enum_<invz::FOVState>(m, "FOVState")
		.value("FOVSTATE_INIT", invz::FOVState::FOVSTATE_INIT)
		.value("FOVSTATE_CLEAR_VIEW", invz::FOVState::FOVSTATE_CLEAR_VIEW)
		.value("FOVSTATE_RESTRICTED_VIEW", invz::FOVState::FOVSTATE_RESTRICTED_VIEW)
		.value("FOVSTATE_ERROR", invz::FOVState::FOVSTATE_ERROR);
	
	py::enum_<invz::ELoginLevel>(m, "ELoginLevel")
		.value("E_LOGIN_LEVEL_USER", invz::ELoginLevel::E_LOGIN_LEVEL_USER)
		.value("E_LOGIN_LEVEL_TECHNICIAN", invz::ELoginLevel::E_LOGIN_LEVEL_TECHNICIAN)
		.value("E_LOGIN_LEVEL_FACTORY", invz::ELoginLevel::E_LOGIN_LEVEL_FACTORY)
		.value("E_LOGIN_LEVEL_DEVELOPER", invz::ELoginLevel::E_LOGIN_LEVEL_DEVELOPER)
		.value("E_LOGIN_LEVEL_ILLEGAL", invz::ELoginLevel::E_LOGIN_LEVEL_ILLEGAL);

	py::enum_<invz::FOVStateError>(m, "FOVStateError")
		.value("FOVSTATEERROR_INTERNAL", invz::FOVStateError::FOVSTATEERROR_INTERNAL)
		.value("FOVSTATEERROR_EXTERNAL", invz::FOVStateError::FOVSTATEERROR_EXTERNAL);

	py::class_<invz::EnvironmentalBlockage>(m, "EnvironmentalBlockage")
		.def(py::init<>())
		.def_readwrite("frame_number", &invz::EnvironmentalBlockage::frame_number)
		.def_readwrite("fov_state", &invz::EnvironmentalBlockage::fov_state)
		.def_readwrite("lidar_dq_data_not_valid", &invz::EnvironmentalBlockage::lidar_dq_data_not_valid)
		.def_readwrite("reserved", &invz::EnvironmentalBlockage::reserved)
		.def_readwrite("error_reason", &invz::EnvironmentalBlockage::error_reason);

	py::class_<invz::FrameMetaData>(m, "FrameMetaData")
		.def(py::init<>())
		.def_readwrite("frame_number", &invz::FrameMetaData::frame_number)
		.def_readwrite("reserved1", &invz::FrameMetaData::reserved1)
		.def_readwrite("frame_event", &invz::FrameMetaData::frame_event)
		.def_readwrite("internal_time", &invz::FrameMetaData::internal_time)
		.def_readwrite("utc_time_seconds", &invz::FrameMetaData::utc_time_seconds)
		.def_readwrite("utc_time_useconds", &invz::FrameMetaData::utc_time_useconds)
		.def_readwrite("measurement_data_type", &invz::FrameMetaData::measurement_data_type)
		.def_readwrite("blockage", &invz::FrameMetaData::blockage);

	py::class_<PyPCFrameMeta>(m, "PointCloudFrameMeta")
		.def(py::init<uint32_t,uint8_t,uint8_t,uint8_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,py::array, 
			uint16_t,uint8_t,uint8_t,py::array,py::array,uint32_t,py::array,py::array,py::array>(),
			"frame_number"_a, 
			"scan_mode"_a,
			"system_mode"_a,
			"system_submode"_a,
			"timestamp_internal"_a,
			"timestamp_utc_sec"_a,
			"timestamp_utc_micro"_a,
			"fw_version"_a,
			"hw_version"_a,
			"lidar_serial"_a,
			"device_type"_a, 
			"active_lrfs"_a, 
			"macro_pixel_shape"_a, 
			"rows_in_lrf"_a, 
			"cols_in_lrf"_a, 
			"total_number_of_points"_a,
			"r_i"_a, 
			"d_i"_a, 
			"v_i_k"_a)
		.def_property_readonly("frame_number", &PyPCFrameMeta::frame_number)
		.def_property_readonly("scan_mode", &PyPCFrameMeta::scan_mode)
		.def_property_readonly("system_mode", &PyPCFrameMeta::system_mode)
		.def_property_readonly("system_submode", &PyPCFrameMeta::system_submode)
		.def_property_readonly("timestamp_internal", &PyPCFrameMeta::timestamp_internal)
		.def_property_readonly("timestamp_utc_secs", &PyPCFrameMeta::timestamp_utc_secs)
		.def_property_readonly("timestamp_utc_micro", &PyPCFrameMeta::timestamp_utc_micro)
		.def_property_readonly("fw_version", &PyPCFrameMeta::fw_version)
		.def_property_readonly("hw_version", &PyPCFrameMeta::hw_version)
		.def_property_readonly("lidar_serial_number", &PyPCFrameMeta::lidar_serial_number)
		.def_property_readonly("device_type", &PyPCFrameMeta::device_type)
		.def_property_readonly("active_lrfs", &PyPCFrameMeta::active_lrfs)
		.def_property_readonly("macro_pixel_shape", &PyPCFrameMeta::macro_pixel_shape)
		.def_property_readonly("rows_in_lrf", &PyPCFrameMeta::rows_in_lrf)
		.def_property_readonly("cols_in_lrf", &PyPCFrameMeta::cols_in_lrf)
		.def_property_readonly("total_number_of_points", &PyPCFrameMeta::total_number_of_points)
		.def_property_readonly("R_i", &PyPCFrameMeta::R_i)
		.def_property_readonly("d_i", &PyPCFrameMeta::d_i)
		.def_property_readonly("v_i_k", &PyPCFrameMeta::v_i_k);

	py::class_<invz::MetaHeader>(m, "MetaHeader")
		.def_readonly("frame_id", &invz::MetaHeader::frameId)
		.def_readonly("version", &invz::MetaHeader::version)
		.def_readonly("first_object", &invz::MetaHeader::firstObject)
		.def_readonly("last_object", &invz::MetaHeader::lastObject)
		.def_readonly("total_objects", &invz::MetaHeader::totalObjects);

	py::enum_<invz::Occlusion>(m, "Occlusion")
		.value("OCCLUSION_VISIBLE", invz::Occlusion::OCCLUSION_VISIBLE)
		.value("OCCLUSION_PARTIAL", invz::Occlusion::OCCLUSION_PARTIAL)
		.value("OCCLUSION_OCCULDED", invz::Occlusion::OCCLUSION_OCCULDED);

	py::enum_<invz::CordinateSystem>(m, "CordinateSystem")
		.value("CAR_CS", invz::CordinateSystem::CAR_CS)
		.value("LIDAR_CS", invz::CordinateSystem::LIDAR_CS);

	py::enum_<invz::RefPoint>(m, "RefPoint")
		.value("REF_POINT_FRONT_LEFT", invz::RefPoint::REF_POINT_FRONT_LEFT)
		.value("REF_POINT_FRONT_MIDDLE", invz::RefPoint::REF_POINT_FRONT_MIDDLE)
		.value("REF_POINT_FRONT_RIGHT", invz::RefPoint::REF_POINT_FRONT_RIGHT)
		.value("REF_POINT_MIDDLE_RIGHT", invz::RefPoint::REF_POINT_MIDDLE_RIGHT)
		.value("REF_POINT_REAR_RIGHT", invz::RefPoint::REF_POINT_REAR_RIGHT)
		.value("REF_POINT_REAR_MIDDLE", invz::RefPoint::REF_POINT_REAR_MIDDLE)
		.value("REF_POINT_REAR_LEFT", invz::RefPoint::REF_POINT_REAR_LEFT)
		.value("REF_POINT_MIDDLE_LEFT", invz::RefPoint::REF_POINT_MIDDLE_LEFT)
		.value("REF_POINT_BUTTOM_CENTER", invz::RefPoint::REF_POINT_BUTTOM_CENTER);

	py::enum_<invz::PoseStatus>(m, "PoseStatus")
		.value("PoseStatus_DefaultValue", invz::PoseStatus::PoseStatus_DefaultValue)
		.value("PoseStatus_Misaligned", invz::PoseStatus::PoseStatus_Misaligned)
		.value("PoseStatus_Outdated", invz::PoseStatus::PoseStatus_Outdated)
		.value("PoseStatus_OutdatedMisaligned", invz::PoseStatus::PoseStatus_OutdatedMisaligned)
		.value("PoseStatus_ConfirmedByQuickCheck", invz::PoseStatus::PoseStatus_ConfirmedByQuickCheck)
		.value("PoseStatus_MisalignedByQuickCheck", invz::PoseStatus::PoseStatus_MisalignedByQuickCheck)
		.value("PoseStatus_Available", invz::PoseStatus::PoseStatus_Available)
		.value("PoseStatus_Error", invz::PoseStatus::PoseStatus_Error)
		.value("PoseStatus_INVALID", invz::PoseStatus::PoseStatus_INVALID);

	py::enum_<invz::CalibrationStatus>(m, "CalibrationStatus")
		.value("CalibStatus_Init", invz::CalibrationStatus::CalibStatus_Init)
		.value("CalibStatus_Error", invz::CalibrationStatus::CalibStatus_Error)
		.value("CalibStatus_PausedVelocity", invz::CalibrationStatus::CalibStatus_PausedVelocity)
		.value("CalibStatus_PausedSAV", invz::CalibrationStatus::CalibStatus_PausedSAV)
		.value("CalibStatus_PausedGround", invz::CalibrationStatus::CalibStatus_PausedGround)
		.value("CalibStatus_PausedCleaning", invz::CalibrationStatus::CalibStatus_PausedCleaning)
		.value("CalibStatus_Available", invz::CalibrationStatus::CalibStatus_Available)
		.value("CalibrationStatusEnum_INVALID", invz::CalibrationStatus::CalibrationStatusEnum_INVALID);

	py::enum_<invz::MeasurmentStatus>(m, "MeasurmentStatus")
		.value("MEASURED", invz::MeasurmentStatus::MEASURED)
		.value("PREDICTED", invz::MeasurmentStatus::PREDICTED)
		.value("NEW_OBJECT", invz::MeasurmentStatus::NEW_OBJECT);
	
	py::enum_<invz::MovementStatus>(m, "MovementStatus")
		.value("STOPPED_OR_IN_MOTION", invz::MovementStatus::STOPPED_OR_IN_MOTION)
		.value("STATIONARY", invz::MovementStatus::STATIONARY);

	py::class_<PyTLV>(m, "TLV")
		.def(py::init<>())
		.def(py::init<uint32_t, uint16_t, py::array>(),"type"_a,"length"_a,"value"_a)
		.def_readwrite("type", &PyTLV::type)
		.def_readwrite("length", &PyTLV::length)
		.def_readwrite("value", &PyTLV::value);

	py::class_<invz::AcpHeaderTlvPack>(m, "AcpHeader")
		.def(py::init<>())
		.def_readwrite("marker", &invz::AcpHeaderTlvPack::marker)
		.def_readwrite("length", &invz::AcpHeaderTlvPack::length)
		.def_readwrite("communication_options", &invz::AcpHeaderTlvPack::communication_options)
		.def_readwrite("master_id", &invz::AcpHeaderTlvPack::master_id)
		.def_readwrite("is_response", &invz::AcpHeaderTlvPack::is_response)
		.def_readwrite("options", &invz::AcpHeaderTlvPack::options)
		.def_readwrite("protocol_version", &invz::AcpHeaderTlvPack::protocol_version)
		.def_readwrite("sequence_number", &invz::AcpHeaderTlvPack::sequence_number)
		.def_readwrite("return_code", &invz::AcpHeaderTlvPack::return_code);

	py::class_<PyTLVPack>(m, "TlvPack")
		.def(py::init<>())
		.def(py::init<uint32_t, uint16_t, py::array>(), "type"_a, "length"_a, "value"_a)
		.def(py::init<PyTLVPack const &>())
		.def_readwrite("virtual_channel", &PyTLVPack::virtual_channel)
		.def_readwrite("port", &PyTLVPack::port)
		.def_readwrite("acp_header", &PyTLVPack::acp_header)
		.def_readwrite("tlv", &PyTLVPack::tlv);



	py::class_<PyTapHandler>(m, "TapHandler")
		.def(py::init<>())
		.def(py::init<PyTapHandler const &>())
		.def_readonly("timestamp", &PyTapHandler::timestamp)
		.def_readonly("frame_number", &PyTapHandler::frame_number)
		.def_readonly("parameter_id", &PyTapHandler::parameter_id)
		.def_readonly("data", &PyTapHandler::data);


	py::class_<PyLogHandler>(m, "LogHandler")
		.def(py::init<>())
		.def(py::init<PyLogHandler const &>())
		.def_readonly("timestamp", &PyLogHandler::timestamp)
		.def_readonly("frame_number", &PyLogHandler::frame_number)
		.def_readonly("core_id", &PyLogHandler::core_id)
		.def_readonly("sequence_number", &PyLogHandler::sequence_number)
		.def_readonly("flow", &PyLogHandler::flow)
		.def_readonly("line_number", &PyLogHandler::line_number)
		.def_readonly("param0", &PyLogHandler::param0)
		.def_readonly("param1", &PyLogHandler::param1)
		.def_readonly("param2", &PyLogHandler::param2)
		.def_readonly("severity", &PyLogHandler::severity)
		.def_readonly("package", &PyLogHandler::package)
		.def_readonly("source_file", &PyLogHandler::source_file)
		.def_readonly("message_name", &PyLogHandler::message_name)
		.def_readonly("message", &PyLogHandler::message);


	py::class_<PyGrabFrameResult>(m, "GrabFrameResult")
		.def(py::init<>())
		.def_readonly("success", &PyGrabFrameResult::success)
		.def_readonly("frame_number", &PyGrabFrameResult::frame_number)
		.def_readonly("timestamp", &PyGrabFrameResult::timestamp)
		.def_readonly("results", &PyGrabFrameResult::results);


	py::enum_<invz::BlockageClassifications>(m, "BlockageClassifications")
		.value("BLOCKAGE_NONE", invz::BlockageClassifications::BLOCKAGE_NONE)
		.value("BLOCKAGE_MUD", invz::BlockageClassifications::BLOCKAGE_MUD)
		.value("BLOCKAGE_RAIN", invz::BlockageClassifications::BLOCKAGE_RAIN)
		.value("BLOCKAGE_SNOW", invz::BlockageClassifications::BLOCKAGE_SNOW)
		.value("BLOCKAGE_OTHER", invz::BlockageClassifications::BLOCKAGE_OTHER);


	py::enum_<invz::Gradient>(m, "Gradient")
		.value("GRADIENT_STABLE", invz::Gradient::GRADIENT_STABLE)
		.value("GRADIENT_DECREASE", invz::Gradient::GRADIENT_DECREASE)
		.value("GRADIENT_INCREASE", invz::Gradient::GRADIENT_INCREASE);


	py::class_<PyPacketContainer>(m, "PacketContainer")
		.def(py::init<>())
		.def(py::init<PyPacketContainer const &>())
		.def_readonly("timestamp", &PyPacketContainer::timestamp)
		.def_readonly("length", &PyPacketContainer::length)
		.def_readonly("payload", &PyPacketContainer::payload);

	m.def("logger_init", &logger_init, "filepath"_a, "severity_level"_a = (int)invz::LogSeverityLevel::info);


	py::enum_<invz::EFileFormat>(m, "InvzFormat")
		.value("INVZ3", invz::EFileFormat::E_FILE_FORMAT_INVZ3)
		.value("INVZ4", invz::EFileFormat::E_FILE_FORMAT_INVZ4)
		.value("INVZ4_4", invz::EFileFormat::E_FILE_FORMAT_INVZ4_4)
		.value("INVZ4_5", invz::EFileFormat::E_FILE_FORMAT_INVZ4_5);


	py::class_<FileReader>(m, "FileReader")
		.def(py::init<const std::string, uint32_t, bool, uint8_t, const std::string>(), "filepath"_a, "log_severity"_a = 3, "check_pixel_validity"_a = false, 
			"num_of_cores"_a = 2, "config_filepath"_a = "")
		.def_readonly("num_of_frames", &FileReader::num_of_frames)
		.def_readonly("file_format", &FileReader::file_format)
		.def("get_device_meta", &FileReader::get_device_meta, pybind11::return_value_policy::copy)
		.def("get_frame", &FileReader::get_frame, "frame_num"_a = -1, "frame_types"_a)
		.def("get_packet", &FileReader::get_packet)
		.def("get_frame_data_attrs", &FileReader::get_frame_data_attrs)
		.def("register_taps_callback", &FileReader::register_taps_callback)
		.def("unregister_taps_callback", &FileReader::unregister_taps_callback)
		.def("grab_taps", &FileReader::grab_taps, "frame_num"_a = -1)
		.def("register_logs_callback", &FileReader::register_logs_callback)
		.def("register_logs_callback", &FileReader::unregister_logs_callback)
		.def("grab_logs", &FileReader::grab_logs, "frame_num"_a = -1);
		
		
	py::class_<INVZ4FileRawWriter>(m, "INVZ4FileRawWriter")
		.def(py::init<const std::string, const std::string, py::list>(), "file_name"_a, "device_ip"_a, "virtual_channels_ports"_a)
		.def("write_payload", &INVZ4FileRawWriter::write_payload, "timestamp"_a, "packet"_a, "port"_a, "frame_number"_a = -1)
		.def("finalize", &INVZ4FileRawWriter::finalize);
		

	py::class_<FileWriter>(m, "FileWriter")
		.def(py::init<const std::string, uint32_t >(), "filepath"_a, "file_format"_a = 3)
		.def("write_frame", &FileWriter::write_frame, "frame_meta"_a, "macro_pixels_frame"_a, "environmental_bloackage"_a, "blockage_detection"_a, "blockage_classification"_a);
		//.def("close_writer", &FileReader::get_packet);
	

	py::class_<PyUdpReceiver>(m, "UdpReceiver")
		.def(py::init<const std::string, const int, uint32_t, uint32_t>(), "ipAddress"_a, "port"_a, "marker"_a = INVZ_MARKER, "log_severity"_a = 3)
		.def("registerCallback", &PyUdpReceiver::registerCallback, "callback"_a)
		.def("startListening", &PyUdpReceiver::startListening)
		.def("stopListening", &PyUdpReceiver::stopListening);
		

	py::class_<DeviceInterface>(m, "DeviceInterface")
		.def_readonly("connection_level", &DeviceInterface::connection_level)
		.def_property_readonly("num_data_points", &DeviceInterface::get_num_data_points)
		.def(py::init<const std::string, bool, int, std::string, uint32_t>(),
			"config_file_name"_a,
			"is_connect"_a = true,
			"login_level"_a = 0,
			"password"_a = "",
			"log_severity"_a = 3)
		.def("connect", &DeviceInterface::connect, "request_level"_a = 0, "password"_a = "")
		.def("disconnect", &DeviceInterface::disconnect)
		.def("device_close", &DeviceInterface::device_close)
		.def("build_acp", &DeviceInterface::build_acp, "acp_header"_a, "tlv"_a)
		.def("get_frame", &DeviceInterface::get_frame, "frame_types"_a)
		.def("get_statistics", &DeviceInterface::get_statistics)
		.def("activate_buffer", &DeviceInterface::activate_buffer, "frame_type"_a,"activate"_a )
		.def("send_tlv", &DeviceInterface::send_tlv,
			"acp_header"_a,
			"tlv"_a,
			"return_error_tlv"_a = false)
		.def("is_connected", &DeviceInterface::is_connected)
		.def("get_dp_details", &DeviceInterface::get_dp_details)
		.def("get_dp_details_by_id", &DeviceInterface::get_dp_details_by_id)
		.def("get_dp_dtype", &DeviceInterface::get_dp_dtype)
		.def("get_dp_dtype_by_id", &DeviceInterface::get_dp_dtype_by_id)
		.def("get_all_dp_details", &DeviceInterface::get_all_dp_details)
		.def("get_all_tap_details", &DeviceInterface::get_all_tap_details)
		.def("get_dp", &DeviceInterface::get_dp, "dp_name"_a,"get_dp_policy"_a = (uint32_t)invz::E_GET_PARAM_POLICY_AUTO)
		.def("get_empty_dp", &DeviceInterface::get_empty_dp)
		.def("get_zero_dp", &DeviceInterface::get_zero_dp)
		.def("set_dp", &DeviceInterface::set_dp, "dp_name"_a, "obj"_a, "set_param"_a = false)
		.def("get_register_details", &DeviceInterface::get_register_details)
		.def("get_all_registers_details", &DeviceInterface::get_all_registers_details)
		.def("get_register_by_name", &DeviceInterface::get_register_by_name)
		.def("set_register_by_name", &DeviceInterface::set_register_by_name)
		.def("get_register_by_address", &DeviceInterface::get_register_by_address)
		.def("set_register_by_address", &DeviceInterface::set_register_by_address)
		.def("set_tap_activation_state", &DeviceInterface::set_tap_activation_state)
		.def("record", &DeviceInterface::record, "seconds"_a, "filepath"_a = INNOPY_DEFAULT_RECORD_FILENAME, "flush_queues"_a = false)
		.def("start_recording", &DeviceInterface::start_recording, "filepath"_a = INNOPY_DEFAULT_RECORD_FILENAME, "flush_queues"_a = false)
		.def("stop_recording", &DeviceInterface::stop_recording)
		.def("register_taps_callback", &DeviceInterface::register_taps_callback)
		.def("unregister_taps_callback", &DeviceInterface::unregister_taps_callback, py::call_guard<py::gil_scoped_release>()) // py::call_guard<py::gil_scoped_release>() called in order to release GIL
		.def("register_logs_callback", &DeviceInterface::register_logs_callback)
		.def("unregister_logs_callback", &DeviceInterface::unregister_logs_callback, py::call_guard<py::gil_scoped_release>())
		.def("register_new_frame_callback", &DeviceInterface::register_new_frame_callback)
		.def("unregister_new_frame_callback", &DeviceInterface::unregister_new_frame_callback, py::call_guard<py::gil_scoped_release>())
		.def("register_new_tlv_callback", &DeviceInterface::register_new_tlv_callback)
		.def("unregister_new_tlv_callback", &DeviceInterface::unregister_new_tlv_callback, py::call_guard<py::gil_scoped_release>());


	py::class_<FrameHelper>(m, "FrameHelper")
		.def(py::init<>())
		.def("get_empty_macro_pixels_frame", &FrameHelper::get_empty_macro_pixels_frame, "pixel_count"_a = 1, "channel_count"_a = 8, "reflection_count"_a = 3)
		.def("get_empty_summation_pixels_frame", &FrameHelper::get_empty_summation_pixels_frame, "pixel_count"_a = 1, "channel_count"_a = 8, "reflection_count"_a = 2)
		.def("convert_byte_stream_to_macro_pixel_frame", &FrameHelper::convert_byte_stream_to_macro_pixel_frame, "py_device_meta"_a, "byte_stream"_a)
		.def("get_direction_by_mems_feedback", &FrameHelper::get_direction_by_mems_feedback, "device_meta"_a, "lrf"_a, "mems_feedback"_a);

}
