///////////////////////////////////////////////////////////
//  PY_CommonTypes.cpp
//  Implementation of the Class PY_CommonTypes
//  Created on:      09-Aug-2021 3:20:04 PM
//  Original author: sarah.foox
///////////////////////////////////////////////////////////

#include "PY_CommonTypes.h"

#include "PY_CommonUtils.h"


PyTapHandler::PyTapHandler(PyTapHandler const & other)

{
	handler = std::make_unique<invz::TapEventData>(*(other.handler.get()));
	timestamp = handler->timestamp;
	frame_number = handler->frame_number;
	parameter_id = handler->parameter_id;
	ui_cookie = handler->ui_cookie;
	data = other.data;
}

PyTapHandler::PyTapHandler(invz::TapEventData& other, const invz::DataPoint* dp)

		{
			handler = std::make_unique<invz::TapEventData>(other);
			timestamp = handler->timestamp;
			frame_number = handler->frame_number;
			parameter_id = handler->parameter_id;
			ui_cookie = handler->ui_cookie;
			auto arr = GetDpArray(dp);
			memcpy(arr.request().ptr, handler->data.get(), handler->length);
			data = arr;
		}

PyLogHandler::PyLogHandler(PyLogHandler const& other)

		{
			handler = std::make_unique<invz::RuntimeLogEventData>(*(other.handler.get()));
			timestamp = handler->timestamp;
			frame_number = handler->frame_number;
			core_id = handler->core_id;
			sequence_number = handler->sequence_number;
			flow = handler->flow;
			line_number = handler->line_number;
			param0 = static_cast<float>(handler->param0);
			param1 = static_cast<float>(handler->param1);
			param2 = static_cast<float>(handler->param2);
			severity = handler->severity;
			package = handler->package;
			source_file = handler->source_file;
			message_name = handler->message_name;
			message = handler->message;
		}

PyLogHandler::PyLogHandler(invz::RuntimeLogEventData& other)

		{
			handler = std::make_unique<invz::RuntimeLogEventData>(other);
			timestamp = handler->timestamp;
			frame_number = handler->frame_number;
			core_id = handler->core_id;
			sequence_number = handler->sequence_number;
			flow = handler->flow;
			line_number = handler->line_number;
			param0 = static_cast<float>(handler->param0);
			param1 = static_cast<float>(handler->param1);
			param2 = static_cast<float>(handler->param2);
			severity = handler->severity;
			package = handler->package;
			source_file = handler->source_file;
			message_name = handler->message_name;
			message = handler->message;
		}

PyTLV::PyTLV()

{
	type = 0;
	length = 0;
	reserved = 0;
	value = Get1DArray(length, GetTypeMeta<uint8_t>());
}
PyTLV::PyTLV(PyTLV const & other)
	:
	type(other.type),
	length(other.length),
	reserved(other.reserved)
{
	value = Get1DArray(length, GetTypeMeta<uint8_t>());
	auto pyarrayValue = value.request();
	memcpy(pyarrayValue.ptr, other.value.data(), length);
}
PyTLV::PyTLV(invz::eTLV& other)
	:
	type(other.type),
	length(other.length),
	reserved(other.reserved)
{
	value = Get1DArray(length, GetTypeMeta<uint8_t>());
	auto pyarrayValue = value.request();
	memcpy(pyarrayValue.ptr, other.value, length);
}
PyTLV::PyTLV(uint32_t _type, uint16_t _length, py::array _value_array)

{
	type = _type;
	length = _length;
	reserved = 0;
	value = Get1DArray(length, GetTypeMeta<uint8_t>());
	auto pyarrayValue = value.request();
	memcpy(pyarrayValue.ptr, _value_array.request().ptr, length);
}
PyTLV::PyTLV(uint32_t _type, uint16_t _length, uint8_t* _value_buffer)

{
	type = _type;
	length = _length;
	reserved = 0;
	value = Get1DArray(length, GetTypeMeta<uint8_t>());
	auto pyarrayValue = value.request();
	memcpy(pyarrayValue.ptr, _value_buffer, length);
}

PyPacketContainer::PyPacketContainer(PyPacketContainer const & other)
{
	handler = std::make_unique<invz::PacketContainer>(*(other.handler.get()));
	timestamp = other.timestamp;
	length = other.length;
	payload = Get1DArray(handler->length, GetTypeMeta<uint8_t>());
	memcpy(payload.request().ptr, handler->payload.get(), handler->length);
}

PyPacketContainer::PyPacketContainer(invz::PacketContainer& other)
{
	handler = std::make_unique<invz::PacketContainer>(other);
	timestamp = other.timestamp;
	length = other.length;
	payload = Get1DArray(handler->length, GetTypeMeta<uint8_t>());
	memcpy(payload.request().ptr, handler->payload.get(), handler->length);
}

PyDeviceMeta::PyDeviceMeta(py::array lrf_width, py::array lrf_height, py::array Ri, py::array di, py::array vik, py::array alpha_calib, py::array beta_calib)
{
	m_width = lrf_width;
	m_height = lrf_height;
	m_Ri = Ri;
	m_di = di;
	m_vik = vik;
	m_alpha_calib = alpha_calib;
	m_beta_calib = beta_calib;
	meta.reset(new invz::DeviceMeta((uint16_t*)lrf_width.request().ptr, (uint8_t*)lrf_height.request().ptr, (invz::ReflectionMatrix*)Ri.request().ptr, 
		(invz::vector3*)di.request().ptr, (invz::ChannelNormal*)vik.request().ptr, (float*)alpha_calib.request().ptr, (float*)beta_calib.request().ptr));
}

PyDeviceMeta::PyDeviceMeta(const PyDeviceMeta& other)

{
	m_width = other.m_width;
	m_height = other.m_height;
	m_Ri = other.m_Ri;
	m_di = other.m_di;
	m_vik = other.m_vik;
	m_alpha_calib = other.m_alpha_calib;
	m_beta_calib = other.m_beta_calib;
	meta.reset(new invz::DeviceMeta((uint16_t*)m_width.request().ptr, (uint8_t*)m_height.request().ptr, (invz::ReflectionMatrix*)m_Ri.request().ptr, 
		(invz::vector3*)m_di.request().ptr, (invz::ChannelNormal*)m_vik.request().ptr, (float*)m_alpha_calib.request().ptr, (float*)m_beta_calib.request().ptr));
}

PyDeviceMeta::PyDeviceMeta(const invz::DeviceMeta& otherCpp)

{
	m_width = Get1DArray(DEVICE_NUM_OF_LRFS, GetTypeMeta<uint16_t>());
	m_height = Get1DArray(DEVICE_NUM_OF_LRFS, GetTypeMeta<uint8_t>());
	m_Ri = Get1DArray(DEVICE_NUM_OF_LRFS, GetTypeMeta<invz::ReflectionMatrix>());
	m_di = Get1DArray(DEVICE_NUM_OF_LRFS, GetTypeMeta<invz::vector3>());
	m_vik = Get1DArray(DEVICE_NUM_OF_LRFS, GetTypeMeta<invz::ChannelNormal>());
	m_alpha_calib = Get1DArray(META_CALIB_TABLE_SIZE, GetTypeMeta<float>());
	m_beta_calib = Get1DArray(META_CALIB_TABLE_SIZE, GetTypeMeta<float>());

	memcpy(m_width.request().ptr, otherCpp.lrf_width, sizeof(uint16_t));
	memcpy(m_width.request().ptr, otherCpp.lrf_height, sizeof(uint8_t));
	memcpy(m_Ri.request().ptr, otherCpp.Ri, sizeof(invz::DeviceMeta::Ri));
	memcpy(m_di.request().ptr, otherCpp.di, sizeof(invz::DeviceMeta::di));
	memcpy(m_vik.request().ptr, otherCpp.vik, sizeof(invz::DeviceMeta::vik));
	memcpy(m_alpha_calib.request().ptr, otherCpp.alpha_calib_table, sizeof(float));
	memcpy(m_beta_calib.request().ptr, otherCpp.beta_calib_table, sizeof(float));

	meta.reset(new invz::DeviceMeta(otherCpp.lrf_width, otherCpp.lrf_height, otherCpp.Ri, otherCpp.di, otherCpp.vik, otherCpp.alpha_calib_table, otherCpp.beta_calib_table));
}

uint32_t PyDeviceMeta::GetLrfCount() const

{
	uint32_t ret = 0;
	if (meta)
		ret = meta->lrf_count;

	return ret;
}

uint32_t PyDeviceMeta::GetFovSegmentsCount() const

{
	uint32_t ret = 0;
	if (meta)
		ret = meta->fov_segments_count;

	return ret;
}

PyDeviceMeta::~PyDeviceMeta()

{
	meta.reset();
}

py::array FrameHelper::GetEmptyMacroPixelsFrame(size_t pixel_count, size_t channel_count, size_t reflection_count)
{
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

	auto ret = Get1DArray(static_cast<int>(pixel_count), GetTypeMeta<invz::MacroPixelFixed>());

	memcpy(ret.request().ptr, arr, sizeof(arr[0]) * pixel_count);

	delete[] arr;

	return ret;
}

py::array FrameHelper::GetEmptySummationPixelsFrame(size_t pixel_count, size_t channel_count, size_t reflection_count)
{
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

py::tuple FrameHelper::ConvertByteStreamToMacroPixelFrame(PyDeviceMeta& py_device_meta, py::array byte_stream)
{
	invz::DeviceMeta device_meta = *(py_device_meta.meta.get());
	size_t macroPixelsPerFrame = 0;
	for (int i = 0; i < static_cast<int>(device_meta.lrf_count); i++)
	{
		macroPixelsPerFrame += device_meta.lrf_width[i] * device_meta.lrf_height[i];
	}
	auto macroPixelFrame = GetEmptyMacroPixelsFrame(macroPixelsPerFrame, 8, 3);
	auto summationPixelFrame = GetEmptySummationPixelsFrame(macroPixelsPerFrame, 8, 2);
	auto start = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	PointCloudStreamToMacroPixelsFrame(device_meta,
		(invz::INVZ4PixelsStream*)byte_stream.request().ptr,
		(invz::MacroPixelFixed*)macroPixelFrame.request().ptr,
		(invz::SummationMacroPixelFixed*)summationPixelFrame.request().ptr);
	auto end = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	return py::make_tuple(macroPixelFrame, summationPixelFrame);
}

py::array FrameHelper::GetDirectionByMemsFeedback(PyDeviceMeta deviceMeta, uint8_t lrf, py::array_t<invz::MemsFeedback> memsFeedback)
{
	auto ret = Get1DArray(INVZ4_MACRO_PIXEL_CHANNEL_COUNT, GetTypeMeta<invz::vector3>());

	invz::DeviceMeta& device_meta = *deviceMeta.meta;
	invz::MemsFeedback* mems_feedback = (invz::MemsFeedback*)(memsFeedback.request().ptr);
	auto result = MemsFeedbackToDirections(device_meta, lrf, *mems_feedback, (invz::vector3*)ret.request().ptr, INVZ4_MACRO_PIXEL_CHANNEL_COUNT);
	CheckResult(result);

	return ret;
}