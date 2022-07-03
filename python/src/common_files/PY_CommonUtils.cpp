///////////////////////////////////////////////////////////
//  PY_CommonUtils.cpp
//  Implementation of the Class PY_CommonUtils
//  Created on:      09-Aug-2021 3:17:04 PM
//  Original author: sarah.foox
///////////////////////////////////////////////////////////


#include "PY_CommonUtils.h"


void CheckAttribute(FrameDataAttributes & attr, std::vector<FrameDataAttributes> known)
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
			throw std::runtime_error("Unkown data type - missing size or length defintion");
	}
}

void CheckResult(invz::Result& result)
{
	/* In case of failure error_code should be different than 0 */
	if (result.error_code)
	{
		/* TODO: change to string or stringstream */
		char error_code[8] = { '\0' };
		snprintf(error_code, 8, "%05d: ", result.error_code);
		std::string error_message = "ErrorCode " + std::string(error_code) + result.error_message;
		throw std::runtime_error(error_message);
	}
}

TypeMeta GetTypeMeta(std::string dp_type)
{
	TypeMeta ret;

	if (dp_type.find("int8", 0) == 0)
		ret = GetTypeMeta<int8_t>();
	else if (dp_type.find("uint8", 0) == 0)
		ret = GetTypeMeta<uint8_t>();
	else if (dp_type.find("int16", 0) == 0)
		ret = GetTypeMeta<int16_t>();
	else if (dp_type.find("uint16", 0) == 0)
		ret = GetTypeMeta<uint16_t>();
	else if (dp_type.find("int32", 0) == 0)
		ret = GetTypeMeta<int32_t>();
	else if (dp_type.find("uint32", 0) == 0)
		ret = GetTypeMeta<uint32_t>();
	else if (dp_type.find("char", 0) == 0)
		ret = GetTypeMeta<char>();
	else if (dp_type.find("float", 0) == 0)
		ret = GetTypeMeta<float>();
	else if (dp_type.find("double", 0) == 0)
		ret = GetTypeMeta<double>();
	else if (dp_type.find("bool", 0) == 0)
		ret = GetTypeMeta<bool>();
	else {
		std::string error{ "Unsupported dp type: " };
		error.append(dp_type);
		throw std::runtime_error(error);
	}

	return ret;
}

py::array Get1DArray(size_t len, TypeMeta meta)
{

	return py::array(py::buffer_info(
		nullptr,            /* Pointer to data (nullptr -> ask NumPy to allocate!) */
		meta.itemsize,			/* Size of one item */
		meta.np_format, /* Buffer format */
		1,          /* How many dimensions? */
		{ len },  /* Number of elements for each dimension */
		{ meta.itemsize }  /* Strides for each dimension */
	));
}

py::array GetDpArray(const invz::DataPoint* dp, TypeMeta tm)
{
	size_t rows = dp->rows;
	//if (tm.np_dtype != "struct")
	//	rows = dp->Length() / dp->stride;

	size_t cols = dp->stride;
	size_t lrfs = dp->lrfs;

	int ndims = 1;
	if (lrfs > 1)
		ndims = 3;
	else if (rows > 1)
		ndims = 2;

	std::vector<long> strides;
	std::vector<long> shape;
	if (ndims == 3)
	{
		strides.push_back(static_cast<long>(rows * cols * tm.itemsize));
		strides.push_back(static_cast<long>(cols * tm.itemsize));
		strides.push_back(static_cast<long>(tm.itemsize));

		shape.push_back(static_cast<long>(lrfs));
		shape.push_back(static_cast<long>(rows));
		shape.push_back(static_cast<long>(cols));
	}
	else if (ndims == 2) {
		strides.push_back(static_cast<long>(cols * tm.itemsize));
		strides.push_back(static_cast<long>(tm.itemsize));

		shape.push_back(static_cast<long>(rows));
		shape.push_back(static_cast<long>(cols));
	}
	else {
		strides.push_back(static_cast<long>(tm.itemsize));

		shape.push_back(static_cast<long>(cols));
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

py::array GetDpArray(const invz::DataPoint* dp)
{
	py::array ret;

	if (dp->type != "struct")
	{
		ret = GetDpArray(dp, GetTypeMeta(dp->type));
	}
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

std::string GetBufferName(FrameDataAttributes attr)
{
	switch (attr.known_type)
	{
	case GRAB_TYPE_METADATA: return "GrabType.GRAB_TYPE_METADATA";
	case GRAB_TYPE_THETA_PHI: return "GrabType.GRAB_TYPE_THETA_PHI";
	case GRAB_TYPE_MEASURMENTS_REFLECTION0: return "GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION0";
	case GRAB_TYPE_MEASURMENTS_REFLECTION1: return "GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION1";
	case GRAB_TYPE_MEASURMENTS_REFLECTION2: return "GrabType.GRAB_TYPE_MEASURMENTS_REFLECTION2";
	case GRAB_TYPE_MACRO_PIXEL_META_DATA: return "GrabType.GRAB_TYPE_MACRO_PIXEL_META_DATA";
	case GRAB_TYPE_SINGLE_PIXEL_META_DATA: return "GrabType.GRAB_TYPE_SINGLE_PIXEL_META_DATA";
	case GRAB_TYPE_SUM_PIXEL_META_DATA: return "GrabType.GRAB_TYPE_SUM_PIXEL_META_DATA";
	case GRAB_TYPE_PIXEL_LANE_MARK_TRAILER: return "GrabType.GRAB_TYPE_PIXEL_LANE_MARK_TRAILER";
	case GRAB_TYPE_SUMMATION_REFLECTION0: return "GrabType.GRAB_TYPE_SUMMATION_REFLECTION0";
	case GRAB_TYPE_SUMMATION_REFLECTION1: return "GrabType.GRAB_TYPE_SUMMATION_REFLECTION1";
	case GRAB_TYPE_DIRECTIONS: return "GrabType.GRAB_TYPE_DIRECTIONS";
	case GRAB_TYPE_SUMMATION_DIRECTIONS: return "GrabType.GRAB_TYPE_SUMMATION_DIRECTIONS";
	case GRAB_TYPE_PC_PLUS: return "GrabType.GRAB_TYPE_PC_PLUS";
	case GRAB_TYPE_PC_PLUS_METADATA: return "GrabType.GRAB_TYPE_PC_PLUS_METADATA";
	case GRAB_TYPE_PC_PLUS_METADATA_48K: return "GrabType.GRAB_TYPE_PC_PLUS_METADATA_48K";
	case GRAB_TYPE_DETECTIONS: return "GrabType.GRAB_TYPE_DETECTIONS";
	case GRAB_TYPE_DETECTIONS_SI: return "GrabType.GRAB_TYPE_DETECTIONS_SI";
	case GRAB_TYPE_GLARE_IN_FOV: return "GrabType.GRAB_TYPE_GLARE_IN_FOV";
	case GRAB_TYPE_TRACKED_OBJECTS: return "GrabType.GRAB_TYPE_TRACKED_OBJECTS";
	case GRAB_TYPE_TRACKED_OBJECTS_SI: return "GrabType.GRAB_TYPE_TRACKED_OBJECTS_SI";
	case GRAB_TYPE_SENSOR_POSE: return "GrabType.GRAB_TYPE_SENSOR_POSE";
	case GRAB_TYPE_OC_OUTPUT: return "GrabType.GRAB_TYPE_OC_OUTPUT";
	case GRAB_TYPE_DC_OUTPUT: return "GrabType.GRAB_TYPE_DC_OUTPUT";
	case GRAB_TYPE_BLOCKAGE: return "GrabType.GRAB_TYPE_BLOCKAGE";
	case GRAB_TYPE_BLOCKAGE_ENVIRONMENTAL: return "GrabType.GRAB_TYPE_BLOCKAGE_ENVIRONMENTAL";
	case GRAB_TYPE_BLOCKAGE_CLASSIFICATION: return "GrabType.GRAB_TYPE_BLOCKAGE_CLASSIFICATION";
	case GRAB_TYPE_LIDAR_STATUS: return "GrabType.GRAB_TYPE_LIDAR_STATUS";
	case GRAB_TYPE_OM_INDICATIONS: return "GrabType.GRAB_TYPE_OM_INDICATIONS";
	case GRAB_TYPE_MEMS_PITCH_STATUS: return "GrabType.GRAB_TYPE_MEMS_PITCH_STATUS";
	case GRAB_TYPE_INS_SIGNALS: return "GrabType.GRAB_TYPE_INS_SIGNALS";
	case GRAB_TYPE_RBD_OUTPUT: return "GrabType.GRAB_TYPE_RBD_OUTPUT";
	case GRAB_TYPE_SIGN_GANTRY_DETECTION: return "GrabType.GRAB_TYPE_SIGN_GANTRY_DETECTION";
	case GRAB_TYPE_OC_OUTPUT_SI: return "GrabType.GRAB_TYPE_OC_OUTPUT_SI";
	case GRAB_TYPE_FOV_OUTPUT_SI: return "GrabType.GRAB_TYPE_FOV_OUTPUT_SI";
	default:
		std::ostringstream stringStream;
		stringStream << attr.typeMajor << "_" << attr.typeMinor;
		return stringStream.str();
		break;
	}
}

TypeMeta GetTypeMeta(uint32_t invz_format, uint32_t frame_data_type_major, uint32_t frame_data_type_minor)
{
	TypeMeta ret = GetTypeMeta<invz::byte>();
	switch (invz_format)
	{
	case invz::EFileFormat::E_FILE_FORMAT_INVZ4:
	case invz::EFileFormat::E_FILE_FORMAT_INVZ4_4:
	case invz::EFileFormat::E_FILE_FORMAT_INVZ4_5:
	case invz::EFileFormat::E_FILE_FORMAT_INVZ4_6:
	case invz::EFileFormat::E_FILE_FORMAT_INVZ4_7:
	case invz::EFileFormat::E_FILE_FORMAT_INVZ5:
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
		else if (frame_data_type_major == invz::POINT_CLOUD_GLARE_IN_FOV_DETECTION)
		{
			return GetTypeMeta < invz::GlareInFovDetectionSegment>();
		}
		else if (frame_data_type_major == SUMMATION_MEASURMENTS_TYPE)
		{
			return GetTypeMeta < invz::INVZ2SumMeasurementXYZType>();
		}
		else if (frame_data_type_major == POINT_CLOUD_PIXELS_THETA_PHI)
		{
			return GetTypeMeta < invz::MemsFeedback>();
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
		else if (frame_data_type_major == POINT_CLOUD_PIXEL_LANE_MARK_TRAILER)
		{
			return GetTypeMeta <invz::INVZ2PixelLaneMarkTrailer>();
		}
		else if (frame_data_type_major == DIRECTIONS_TYPE || frame_data_type_major == SUMMATION_DIRECTIONS_TYPE)
		{
			return GetTypeMeta < invz::vector3>();
		}
		else if (frame_data_type_major == POINT_CLOUD_OM_INDICATIONS)
		{
			return GetTypeMeta < invz::OMIndications>();
		}
		else if (frame_data_type_major == invz::POINT_CLOUD_MEMS_PITCH_STATUS)
		{
			return GetTypeMeta < invz::memsPitchStatus>();
		}
		else if (frame_data_type_major == invz::OBJECT_DETECTION_DEBUG_PORT)
		{
			return GetTypeMeta < invz::ObjectDetection>();
		}
		else if (frame_data_type_major == invz::TRACKED_OBJECT_DEBUG_PORT)
		{
			return GetTypeMeta < invz::TrackedObject>();
		}
		else if (frame_data_type_major == invz::OBJECT_DETECTION_SI || frame_data_type_major == invz::TRACKED_OBJECT_SI)
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
		else if (frame_data_type_major == invz::RBD_OUTPUT_DEBUG_PORT)
		{
			return GetTypeMeta <invz::RBDOutput>();
		}
		else if (frame_data_type_major == invz::INS_SIGNALS_DEBUG_PORT)
		{
			return GetTypeMeta <invz::INSSignalsStatus>();
		}
		else if (frame_data_type_major == invz::SIGN_GANTRY_DEBUG_PORT)
		{
			return GetTypeMeta <invz::SignGantryObject>();
		}
		else if (frame_data_type_major == invz::PC_PLUS_METADATA)
		{
			return GetTypeMeta < invz::PCPlusMetaData>();
		}
		else if (frame_data_type_major == invz::PC_PLUS_48K_METADATA)
		{
			return GetTypeMeta < invz::PCPlusMetadata48k>();
		}
		else if (frame_data_type_major == invz::OC_OUTPUT_SI)
		{
			return GetTypeMeta <invz::OCOutputSI>();
		}
		else if (frame_data_type_major == invz::FOV_OUTPUT_SI)
		{
		return GetTypeMeta <invz::FOVOutputSI>();
		}
		break;
	default:
		break;
	}

	return ret;
}

std::string ApiVersion()
{
	return invz::GetVersion();
}
