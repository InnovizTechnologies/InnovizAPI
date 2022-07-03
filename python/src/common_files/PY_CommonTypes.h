///////////////////////////////////////////////////////////
//  PY_CommonTypes.h
//  Implementation of the Class PY_CommonTypes
//  Created on:      09-Aug-2021 3:20:04 PM
//  Original author: sarah.foox
///////////////////////////////////////////////////////////


#ifndef __PY_COMMON_TYPES_H__
#define __PY_COMMON_TYPES_H__


#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include <sstream>
#include <thread>

#include "data_point/Datapoint.h"
#include "interface/IUdpReceiver.h"

#ifndef INVZ4_PIXEL_STREAM
#define INVZ4_PIXEL_STREAM
#endif


using namespace invz;
using namespace pybind11::literals;
namespace py = pybind11;


struct TypeMeta
{
	size_t itemsize;
	std::string np_dtype;
	std::string np_format;
};
struct PyTapHandler
{
	std::unique_ptr<invz::TapEventData> handler = nullptr;
	uint64_t timestamp = 0;
	uint32_t frame_number = UINT32_MAX;
	uint32_t parameter_id;
	uint32_t ui_cookie = UINT32_MAX;
	py::object data;
	PyTapHandler() = default;

	PyTapHandler(PyTapHandler const & other);

	PyTapHandler(invz::TapEventData& other, const invz::DataPoint* dp);
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

	};

	PyLogHandler(PyLogHandler const& other);

	PyLogHandler(invz::RuntimeLogEventData& other);

	~PyLogHandler() = default;
};
struct PyGrabFrameResult
{
	bool success;
	uint32_t frame_number;
	uint64_t timestamp;
	py::dict results;
};
enum GrabFrameType
{
	FRAME_TYPE_FRAME = 1,
	FRAME_TYPE_SUMMATION = 2,
	FRAME_TYPE_BOTH = 3
};
struct PyTLV
{
	uint32_t type;
	uint16_t length;
	uint16_t reserved;
	py::array value;

	PyTLV();

	PyTLV(PyTLV const & other);

	PyTLV(invz::eTLV& other);

	PyTLV(uint32_t _type, uint16_t _length, py::array _value_array);

	PyTLV(uint32_t _type, uint16_t _length, uint8_t* _value_buffer);
};
struct PyTLVPack
{
	uint32_t virtual_channel = 0;
	uint16_t port = 0;
	invz::AcpHeaderTlvPack acp_header;
	PyTLV tlv;

	PyTLVPack() = default;

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
struct PyPacketContainer
{
	std::unique_ptr<invz::PacketContainer> handler;
	uint64_t timestamp = UINT64_MAX;
	uint32_t length = 0;
	py::array payload;

	PyPacketContainer() = default;

	PyPacketContainer(PyPacketContainer const & other);

	PyPacketContainer(invz::PacketContainer& other);
};
struct PyDeviceMeta

{
	std::unique_ptr<invz::DeviceMeta> meta = nullptr;
	py::array m_width;
	py::array m_height;
	py::array m_Ri;
	py::array m_di;
	py::array m_vik;
	py::array m_alpha_calib;
	py::array m_beta_calib;

	PyDeviceMeta(py::array lrf_width, py::array lrf_height, py::array Ri, py::array di, py::array vik, py::array alpha_calib, py::array beta_calib);
	PyDeviceMeta(const PyDeviceMeta& other);

	PyDeviceMeta(const invz::DeviceMeta& otherCpp);

	uint32_t GetLrfCount() const;

	uint32_t GetFovSegmentsCount() const;

	~PyDeviceMeta();
};
struct FrameHelper
{

	py::array GetEmptyMacroPixelsFrame(size_t pixel_count, size_t channel_count, size_t reflection_count);

	py::array GetEmptySummationPixelsFrame(size_t pixel_count, size_t channel_count, size_t reflection_count);

	py::tuple ConvertByteStreamToMacroPixelFrame(PyDeviceMeta& py_device_meta, py::array byte_stream);

	py::array GetDirectionByMemsFeedback(PyDeviceMeta deviceMeta, uint8_t lrf, py::array_t<invz::MemsFeedback> memsFeedback);
};
#endif //  __PY_COMMON_TYPES_H__
