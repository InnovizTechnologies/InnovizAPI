///////////////////////////////////////////////////////////
//  PY_PCFrameMeta.h
//  Implementation of the Class PY_PCFrameMeta
//  Created on:      15-Aug-2021 10:42:19 AM
//  Original author: sarah.foox
///////////////////////////////////////////////////////////

#ifndef __PY_PC_FRAME_META_H__
#define __PY_PC_FRAME_META_H__

#include "../common_files/PY_CommonUtils.h"


class PY_PCFrameMeta
{
public:
	PY_PCFrameMeta(uint32_t _frame_number, uint8_t _scan_mode, uint8_t _system_mode, uint8_t _system_submode, uint32_t _timestamp_internal,
		uint32_t _timestamp_utc_sec, uint32_t _timestamp_utc_micro, uint32_t _fw_version, uint32_t _hw_version, py::array _lidar_serial,
		uint16_t _device_type, uint8_t _active_lrfs, uint8_t _macro_pixel_shape, py::array _rows_in_lrf, py::array _cols_in_lrf, uint32_t _total_number_of_points,
		py::array _R_i, py::array _d_i, py::array _v_i_k, py::array _alpha_calib_table, py::array _beta_calib_table);
	~PY_PCFrameMeta() = default;
	uint32_t GetFrameNumber();
	uint8_t ScanMode();
	uint8_t SystemMode();
	uint8_t SystemSubmode();
	uint32_t TimestampInternal();
	uint32_t TimestampUtcSecs();
	uint32_t TimestampUtcMicro();
	uint32_t FwVersion();
	uint32_t HwVersion();
	py::array LidarSerialNumber();
	uint16_t DeviceType();
	uint8_t ActiveLrfs();
	uint8_t MacroPixelShape();
	py::array RowsInLrf();
	py::array ColsInLrf();
	uint32_t TotalNumberOfPoints();
	py::array R_i();
	py::array D_i();
	py::array V_i_k();
	py::array Alpha_calib_table();
	py::array Beta_calib_table();
	invz::CSampleFrameMeta GetFrameMeta();

private:
	std::unique_ptr<invz::CSampleFrameMeta> m_FrameMeta;
};
#endif // __PY_PC_FRAME_META_H__
