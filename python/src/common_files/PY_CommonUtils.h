///////////////////////////////////////////////////////////
//  PY_CommonUtils.h
//  Implementation of the Class PY_CommonUtils
//  Created on:      09-Aug-2021 3:17:04 PM
//  Original author: sarah.foox
///////////////////////////////////////////////////////////


#ifndef __PY_COMMON_UTILS_H__
#define __PY_COMMON_UTILS_H__

#include "PY_CommonTypes.h"
#include "interface/DeviceApi.h"
#include "protocols/ACP.h"
#include "common.h"
#include "common_includes/PointCloudConverters.hpp"


void CheckAttribute(FrameDataAttributes & attr, std::vector<FrameDataAttributes> known);
void CheckResult(invz::Result& result);

template<class T> TypeMeta GetTypeMeta()
{
	TypeMeta ret;

	ret.itemsize = sizeof(T);
	ret.np_dtype = py::str(py::dtype::of<T>()).cast<std::string>();
	ret.np_format = py::format_descriptor<T>::format();

	return ret;
}

TypeMeta GetTypeMeta(std::string dp_type);
py::array Get1DArray(size_t len, TypeMeta meta);
py::array GetDpArray(const invz::DataPoint* dp, TypeMeta tm);
py::array GetDpArray(const invz::DataPoint* dp);
std::string GetBufferName(FrameDataAttributes attr);
TypeMeta GetTypeMeta(uint32_t invz_format, uint32_t frame_data_type_major, uint32_t frame_data_type_minor);
std::string ApiVersion();

#endif // __PY_COMMON_UTILS_H__
