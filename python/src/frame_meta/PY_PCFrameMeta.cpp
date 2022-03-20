///////////////////////////////////////////////////////////
//  PY_PCFrameMeta.cpp
//  Implementation of the Class PY_PCFrameMeta
//  Created on:      15-Aug-2021 10:42:19 AM
//  Original author: sarah.foox
///////////////////////////////////////////////////////////

#include "PY_PCFrameMeta.h"


 PY_PCFrameMeta::PY_PCFrameMeta(uint32_t _frame_number, uint8_t _scan_mode, uint8_t _system_mode, uint8_t _system_submode, uint32_t _timestamp_internal,
	 uint32_t _timestamp_utc_sec, uint32_t _timestamp_utc_micro, uint32_t _fw_version, uint32_t _hw_version, py::array _lidar_serial,
	 uint16_t _device_type, uint8_t _active_lrfs, uint8_t _macro_pixel_shape, py::array _rows_in_lrf, py::array _cols_in_lrf, uint32_t _total_number_of_points,
	 py::array _R_i, py::array _d_i, py::array _v_i_k, py::array _alpha_calib_table, py::array _beta_calib_table)
 {

	 m_FrameMeta = std::make_unique<invz::CSampleFrameMeta>(_frame_number, _scan_mode, _system_mode, _system_submode, _timestamp_internal,
		 _timestamp_utc_sec, _timestamp_utc_micro, _fw_version, _hw_version, (uint8_t*)_lidar_serial.request().ptr,
		 _device_type, _active_lrfs, _macro_pixel_shape, (uint8_t*)_rows_in_lrf.request().ptr, (uint16_t*)_cols_in_lrf.request().ptr, _total_number_of_points,
		 (invz::ReflectionMatrix*)_R_i.request().ptr, (invz::vector3*)_d_i.request().ptr, (invz::ChannelNormal*)_v_i_k.request().ptr,
		 (float*)_alpha_calib_table.request().ptr, (float*)_beta_calib_table.request().ptr);
 }

 uint32_t PY_PCFrameMeta::GetFrameNumber()
 {

	 return m_FrameMeta->frame_number;
 }

 uint8_t PY_PCFrameMeta::ScanMode()
 {

	 return m_FrameMeta->scan_mode;
 }


 uint8_t PY_PCFrameMeta::SystemMode()
 {

	 return m_FrameMeta->system_mode;
 }


 uint8_t PY_PCFrameMeta::SystemSubmode()
 {

	 return m_FrameMeta->system_submode;
 }


 uint32_t PY_PCFrameMeta::TimestampInternal()
 {

	 return m_FrameMeta->timestamp_internal;
 }

 uint32_t PY_PCFrameMeta::TimestampUtcSecs()
 {

	 return m_FrameMeta->timestamp_utc_secs;
 }


 uint32_t PY_PCFrameMeta::TimestampUtcMicro()
 {

	 return m_FrameMeta->timestamp_utc_micro;
 }

 uint32_t PY_PCFrameMeta::FwVersion()
 {

	 return m_FrameMeta->fw_version;
 }

 uint32_t PY_PCFrameMeta::HwVersion()
 {

	 return m_FrameMeta->hw_version;
 }

 py::array PY_PCFrameMeta::LidarSerialNumber()
 {

	 py::array ret = Get1DArray(INVZ4_CSAMPLE_LIDAR_SERIAL, GetTypeMeta<uint8_t>());
	 memcpy(ret.request().ptr, m_FrameMeta->lidar_serial_number, INVZ4_CSAMPLE_LIDAR_SERIAL);
	 return std::move(ret);
 }

 uint16_t PY_PCFrameMeta::DeviceType()
 {

	 return m_FrameMeta->device_type;
 }

 uint8_t PY_PCFrameMeta::ActiveLrfs()
 {

	 return m_FrameMeta->active_lrfs;
 }

 uint8_t PY_PCFrameMeta::MacroPixelShape()
 {

	 return m_FrameMeta->macro_pixel_shape;
 }

 py::array PY_PCFrameMeta::RowsInLrf()
 {

	 py::array ret = Get1DArray(DEVICE_NUM_OF_LRFS, GetTypeMeta<uint8_t>());
	 memcpy(ret.request().ptr, m_FrameMeta->rows_in_lrf, DEVICE_NUM_OF_LRFS * sizeof(uint8_t));
	 return std::move(ret);
 }

 py::array PY_PCFrameMeta::ColsInLrf()
 {

	 py::array ret = Get1DArray(DEVICE_NUM_OF_LRFS, GetTypeMeta<uint16_t>());
	 memcpy(ret.request().ptr, m_FrameMeta->cols_in_lrf, DEVICE_NUM_OF_LRFS * sizeof(uint16_t));
	 return std::move(ret);
 }

 uint32_t PY_PCFrameMeta::TotalNumberOfPoints()
 {

	 return m_FrameMeta->total_number_of_points;
 }

 py::array PY_PCFrameMeta::R_i()
 {

	 py::array ret = Get1DArray(DEVICE_NUM_OF_LRFS, GetTypeMeta<invz::ReflectionMatrix>());
	 memcpy(ret.request().ptr, m_FrameMeta->R_i, DEVICE_NUM_OF_LRFS * sizeof(invz::ReflectionMatrix));
	 return std::move(ret);
 }

 py::array PY_PCFrameMeta::D_i()
 {

	 py::array ret = Get1DArray(DEVICE_NUM_OF_LRFS, GetTypeMeta<invz::vector3>());
	 memcpy(ret.request().ptr, m_FrameMeta->d_i, DEVICE_NUM_OF_LRFS * sizeof(invz::vector3));
	 return std::move(ret);
 }


 py::array PY_PCFrameMeta::V_i_k()
 {

	 py::array ret = Get1DArray(DEVICE_NUM_OF_LRFS, GetTypeMeta<invz::ChannelNormal>());
	 memcpy(ret.request().ptr, m_FrameMeta->v_i_k, DEVICE_NUM_OF_LRFS * sizeof(invz::ChannelNormal));
	 return std::move(ret);
 }


 py::array PY_PCFrameMeta::Alpha_calib_table()
 {

	 py::array ret = Get1DArray(META_CALIB_TABLE_SIZE, GetTypeMeta<float>());
	 memcpy(ret.request().ptr, m_FrameMeta->alpha_calib_table, META_CALIB_TABLE_SIZE * sizeof(float));
	 return std::move(ret);
 }


 py::array PY_PCFrameMeta::Beta_calib_table()
 {

	 py::array ret = Get1DArray(META_CALIB_TABLE_SIZE, GetTypeMeta<float>());
	 memcpy(ret.request().ptr, m_FrameMeta->beta_calib_table, META_CALIB_TABLE_SIZE * sizeof(float));
	 return std::move(ret);
 }

 invz::CSampleFrameMeta PY_PCFrameMeta::GetFrameMeta()
 {
	 return *m_FrameMeta;
 }

