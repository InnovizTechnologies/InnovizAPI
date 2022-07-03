///////////////////////////////////////////////////////////
//  innopy_api.cpp
//  Implementation of the Class innopy_api
//  Created on:      15-Aug-2021 3:57:43 PM
//  Original author: sarah.foox
///////////////////////////////////////////////////////////


#include "device_interface/PY_DeviceInterface.h"
#include "file_reader/PY_FileReader.h"
#include "file_writer/PY_FileWriter.h"
#include "file_writer/PY_INVZ4FileRawWriter.h"
#include "udp_receiver/PY_UdpReceiver.h"

#define INNOPY_DEFAULT_RECORD_FILENAME "innopy_record"


PYBIND11_MODULE(api, m) {

	m.doc() = R"pbdoc(
        innoviz API python package
        -----------------------

        .. currentmodule:: api

        .. autosummary::
           :toctree: _generate

           api_version
    )pbdoc";

	m.def("api_version", &ApiVersion, R"pbdoc(
        innovizApi version
    )pbdoc");


	PYBIND11_NUMPY_DTYPE(invz::vector3, x, y, z);\
	PYBIND11_NUMPY_DTYPE(invz::INVZ2MeasurementXYZType, distance, confidence, grazing_angle, reflectivity, noise, x, y, z, validity, pfa);\
	PYBIND11_NUMPY_DTYPE(invz::INVZ2SumMeasurementXYZType, distance, confidence, reflectivity, noise, x, y, z, validity, pfa);\
	PYBIND11_NUMPY_DTYPE(invz::INVZ2MacroMetaData, pixel_type, summation_type, rise_time, num_active_channels, is_blocked, ultra_short_range, artificial_macro_pixel, short_range_detector_status, mems_feedback_x, mems_feedback_y, blockage_pulse_width);\
	PYBIND11_NUMPY_DTYPE(invz::INVZ2PixelMetaData, n_reflection, short_range_reflection, ghost, reflection_valid_0, reflection_valid_1, reflection_valid_2, noise, lane_mark_trailer_available);\
	PYBIND11_NUMPY_DTYPE(invz::INVZ2SumPixelMetaData, sum_n_reflection, sum_short_range_reflection, sum_ghost, sum_reflection_valid_0, sum_reflection_valid_1);\
	PYBIND11_NUMPY_DTYPE(invz::INVZ2PixelLaneMarkTrailer, score, reference, ga_index, pulses_fired); \
	PYBIND11_NUMPY_DTYPE(invz::LidarStatus, system_mode, num_ind_pc_info, error_code, timestamp_sec, timestamp_usec, vbat, indications);\
	PYBIND11_NUMPY_DTYPE(invz::ChannelStatistics, channel_id, packets, valid_packets, missed_packets, recieved_bytes, data_rate);\
	PYBIND11_NUMPY_DTYPE(invz::EndOfFrame, frame_number, reserved);\
	PYBIND11_NUMPY_DTYPE(invz::Point3D, x, y, z, ValidBitmap);\
	PYBIND11_NUMPY_DTYPE(invz::Dimension, width, length, height, ValidBitmap);\
	PYBIND11_NUMPY_DTYPE(invz::DimOcclusion, Dim, Occl); /* Occl for "Occlusion" enum */\
	PYBIND11_NUMPY_DTYPE(invz::AxisAngle, angle, angle_speed, ValidBitmap);\
	PYBIND11_NUMPY_DTYPE(invz::ClassProbability, percentage);\
	PYBIND11_NUMPY_DTYPE(invz::CovarianceMatrix, xx, yy, xy);\
	PYBIND11_NUMPY_DTYPE(invz::top_view_2d_box, x0, y0, x1, y1);\
	
	PYBIND11_NUMPY_DTYPE(invz::ObjectDetection, time_stamp, unique_id, coord_system, ref_point_type, position, dim_and_occlusion,
		inner_dimensions, axis_angle_params, probability_of_classtype, existance_probability, position_cov_matrix, box_2d);\
	PYBIND11_NUMPY_DTYPE(invz::Sensor_Pose_Data, frame_id, timestamps_mili_sec, sp_pitch_deg, sp_roll_deg, sp_z_cm, sp_plane, sp_mat3x3, n_of_inliers, sp_fit_quality_0_to_1, sp_yaw_deg, sp_x_cm, sp_y_cm, reserved);\
	PYBIND11_NUMPY_DTYPE(invz::DisplacementVector, x, y, Z, ValidBitmap);\
	PYBIND11_NUMPY_DTYPE(invz::TrackedObject, time_stamp, unique_id, coord_system, ref_point_type, position, dim_and_occlusion,
		inner_dimentions, axis_angle_params, absulute_speed, relative_speed, absolute_acceleration, relative_acceleration,
		measurment_status, movement_status, probability_of_classtype, existance_probability, position_cov_matrix,
		speed_cov_matrix, acceleration_cov_matrix);\
	PYBIND11_NUMPY_DTYPE(invz::PCPlusDetection, distance, positive_predictive_value, reflectivity, classification, confidence, angle_azimuth,
		angle_elevation);\
	PYBIND11_NUMPY_DTYPE(invz::OCSensorPose, yaw_deg, pitch_deg, roll_deg, z_above_ground_cm);\
	PYBIND11_NUMPY_DTYPE(invz::OCOutput, is_oc_data_not_valid, oc_pose_status, oc_pose, oc_state, oc_age_minutes, oc_distance_meters,
		oc_estimated_accuracy, frame_id);\
	PYBIND11_NUMPY_DTYPE(invz::DCOutput, frame_id, pitch, roll, z_cm); \
		
	PYBIND11_NUMPY_DTYPE(invz::ReflectionAttributes, distance, reflectivity, confidence.value); \
	PYBIND11_NUMPY_DTYPE(invz::SinglePixelFixed, pixel_meta.value, noise, reflection);\
	PYBIND11_NUMPY_DTYPE(invz::MemsFeedback, theta, phi);\
	PYBIND11_NUMPY_DTYPE(invz::MacroPixelFixed, header.value, mems_feedback, blockage_pw, channels);\
	PYBIND11_NUMPY_DTYPE(invz::SummationSinglePixelFixed, summation_pixel_meta.value, noise, reflection);\
	PYBIND11_NUMPY_DTYPE(invz::SummationMacroPixelFixed, header.value, channels);\
	PYBIND11_NUMPY_DTYPE(invz::ReflectionMatrix, matrix);\
	PYBIND11_NUMPY_DTYPE(invz::ChannelNormal, channels);\
	PYBIND11_NUMPY_DTYPE(invz::CSampleFrameMeta, frame_number, scan_mode, pc_protocol_version, system_mode, system_submode, timestamp_internal, \
		timestamp_utc_secs, timestamp_utc_micro, fw_version, hw_version, lidar_serial_number, device_type, active_lrfs, macro_pixel_shape, \
		rows_in_lrf, cols_in_lrf, total_number_of_points, reserved1, R_i, d_i, v_i_k, reserved2, alpha_calib_table, beta_calib_table); \

	PYBIND11_NUMPY_DTYPE(invz::EnvironmentalBlockage, frame_number, fov_state, lidar_dq_data_not_valid, reserved, error_reason);\
	PYBIND11_NUMPY_DTYPE(invz::BlockageDetectionSegment, blocked, coverage_percentage, gradient, reserved);\
	PYBIND11_NUMPY_DTYPE(invz::BlockageClassificationSegment, classification);\
	PYBIND11_NUMPY_DTYPE(invz::GlareInFovDetectionSegment, glared, coverage_percentage, glare_level, reserved);\
	PYBIND11_NUMPY_DTYPE(invz::OMIndications, main_board_temp, detector_temp, mems_ab_temp, mems_cd_temp, laser_board_temp, maui_temp, heater_windows_temp, humidity); \
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
	PYBIND11_NUMPY_DTYPE(vb_invzbuf::DetectionLidar, distance, positivePredictiveValue, reflectivity, classification, confidence, angleAzimuth, angleElevation); \
	PYBIND11_NUMPY_DTYPE(invz::CoordinateSystemOrigin, invalid_flags, x, x_std_dev, y, y_std_dev, z,
		z_std_dev, roll, roll_std_dev, pitch, pitch_std_dev, yaw, yaw_std_dev); \
	PYBIND11_NUMPY_DTYPE(invz::StndTimestamp, fractional_seconds, seconds, sync_status); \
	PYBIND11_NUMPY_DTYPE(invz::DetectionListHeader, event_data_qualifier, extended_qualifier, time_stamp, origin); \
	PYBIND11_NUMPY_DTYPE(invz::PCPlusMetaData, header, number_of_detections, number_of_detections_low_res_left_origin, number_of_detections_low_res_right_origin, number_of_detections_high_res_left_origin
		, number_of_detections_high_res_right_origin, left_origin_in_sensor_origin, right_origin_in_sensor_origin); \
	PYBIND11_NUMPY_DTYPE(invz::PCPlusMetadata48k, header, number_of_detections, number_of_detections_roi_left_origin, number_of_detections_roi_right_origin, number_of_detections_outer_left_origin
			,left_origin_in_sensor_origin, right_origin_in_sensor_origin, lidarPowerMode, integrityDetectionListLidar); 
	PYBIND11_NUMPY_DTYPE(invz::INSSignalsStatus, numOfValidVsInput, frameId); \
	PYBIND11_NUMPY_DTYPE(invz::xyz_t, x, y, z); \
	PYBIND11_NUMPY_DTYPE(invz::RoadsideRegionsDescriptor, region, valid, reserved); \
	PYBIND11_NUMPY_DTYPE(invz::RBDescriptor, p0, p1, p2, valid, reserved); \
	PYBIND11_NUMPY_DTYPE(invz::RBDOutput, roadBoundaries, roadsideRegions, frameId); \
	PYBIND11_NUMPY_DTYPE(invz::dimensions, width, length, height); \
	PYBIND11_NUMPY_DTYPE(invz::orientation, roll, pitch, yaw); \
	PYBIND11_NUMPY_DTYPE(invz::SignGantryObject, dim, angles, center, existence_probability, existence_ppv, age, num_observations, uid, box3d_class); \
	PYBIND11_NUMPY_DTYPE(invz::memsPitchStatus, frame_number, mems_pitch_state, mems_pitch_status, time_left, pitch_current, pitch_target, pitch_max, pitch_min, reserved); \
	PYBIND11_NUMPY_DTYPE(invz::StdTimestamp, nanoseconds, seconds, sync_state); \
	PYBIND11_NUMPY_DTYPE(invz::Pose, poseStatus, poseAge, poseDistance, origin); \
	PYBIND11_NUMPY_DTYPE(invz::OCOutputSI, eventDataQualifier, timestamp, calibrationStatus, pose, lidarPowerMode, integrityOCLidar);\
	PYBIND11_NUMPY_DTYPE(invz::FoVRegionLidar, blockageClassification, detectionQualityIndex); \
	PYBIND11_NUMPY_DTYPE(invz::FOVOutputSI, eventDataQualifier, extendedDataQualifier, timestamp, regions, classificationUpdateFlag, bwdStatus, lidarPowerMode, integrityFoVLidar); \

	py::class_<PyDeviceMeta>(m, "DeviceMeta")
		.def(py::init<py::array, py::array, py::array, py::array, py::array, py::array, py::array>(),
			"lrf_width"_a,
			"lrf_height"_a,
			"Ri"_a,
			"di"_a,
			"vik"_a,
			"alpha_calib"_a,
			"beta_calib"_a)
		.def_property_readonly("lrf_count", &PyDeviceMeta::GetLrfCount)
		.def_property_readonly("fov_segments_count", &PyDeviceMeta::GetFovSegmentsCount)
		.def_readonly("lrf_width", &PyDeviceMeta::m_width)
		.def_readonly("lrf_height", &PyDeviceMeta::m_height)
		.def_readonly("di", &PyDeviceMeta::m_di)
		.def_readonly("Ri", &PyDeviceMeta::m_Ri)
		.def_readonly("vik", &PyDeviceMeta::m_vik)
		.def_readonly("alpha_calib", &PyDeviceMeta::m_alpha_calib)
		.def_readonly("beta_calib", &PyDeviceMeta::m_beta_calib);


	py::enum_<invz::PixelValidity>(m, "PixelValidity")
		.value("PIXEL_VALIDITY_MISSING", invz::PixelValidity::PIXEL_VALIDITY_MISSING)
		.value("PIXEL_VALIDITY_VALID", invz::PixelValidity::PIXEL_VALIDITY_VALID)
		.value("PIXEL_VALIDITY_INVALID", invz::PixelValidity::PIXEL_VALIDITY_INVALID)
		.value("PIXEL_VALIDIDY_BLOOMING", invz::PixelValidity::PIXEL_BLOOMING);


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
		.value("GRAB_TYPE_GLARE_IN_FOV", GrabType::GRAB_TYPE_GLARE_IN_FOV)
		.value("GRAB_TYPE_DETECTIONS_SI", GrabType::GRAB_TYPE_DETECTIONS_SI)
		.value("GRAB_TYPE_DIRECTIONS", GrabType::GRAB_TYPE_DIRECTIONS)
		.value("GRAB_TYPE_MEASURMENTS_REFLECTION0", GrabType::GRAB_TYPE_MEASURMENTS_REFLECTION0)
		.value("GRAB_TYPE_MEASURMENTS_REFLECTION1", GrabType::GRAB_TYPE_MEASURMENTS_REFLECTION1)
		.value("GRAB_TYPE_MEASURMENTS_REFLECTION2", GrabType::GRAB_TYPE_MEASURMENTS_REFLECTION2)
		.value("GRAB_TYPE_MACRO_PIXEL_META_DATA", GrabType::GRAB_TYPE_MACRO_PIXEL_META_DATA)
		.value("GRAB_TYPE_SINGLE_PIXEL_META_DATA", GrabType::GRAB_TYPE_SINGLE_PIXEL_META_DATA)
		.value("GRAB_TYPE_SUM_PIXEL_META_DATA", GrabType::GRAB_TYPE_SUM_PIXEL_META_DATA)
		.value("GRAB_TYPE_PIXEL_LANE_MARK_TRAILER", GrabType::GRAB_TYPE_PIXEL_LANE_MARK_TRAILER)
		.value("GRAB_TYPE_METADATA", GrabType::GRAB_TYPE_METADATA)
		.value("GRAB_TYPE_PC_PLUS", GrabType::GRAB_TYPE_PC_PLUS)
		.value("GRAB_TYPE_PC_PLUS_METADATA", GrabType::GRAB_TYPE_PC_PLUS_METADATA)
		.value("GRAB_TYPE_PC_PLUS_METADATA_48K", GrabType::GRAB_TYPE_PC_PLUS_METADATA_48K)
		.value("GRAB_TYPE_SUMMATION_DIRECTIONS", GrabType::GRAB_TYPE_SUMMATION_DIRECTIONS)
		.value("GRAB_TYPE_SUMMATION_REFLECTION0", GrabType::GRAB_TYPE_SUMMATION_REFLECTION0)
		.value("GRAB_TYPE_SUMMATION_REFLECTION1", GrabType::GRAB_TYPE_SUMMATION_REFLECTION1)
		.value("GRAB_TYPE_TRACKED_OBJECTS", GrabType::GRAB_TYPE_TRACKED_OBJECTS)
		.value("GRAB_TYPE_TRACKED_OBJECTS_SI", GrabType::GRAB_TYPE_TRACKED_OBJECTS_SI)
		.value("GRAB_TYPE_SENSOR_POSE", GrabType::GRAB_TYPE_SENSOR_POSE)
		.value("GRAB_TYPE_OC_OUTPUT", GrabType::GRAB_TYPE_OC_OUTPUT)
		.value("GRAB_TYPE_DC_OUTPUT", GrabType::GRAB_TYPE_DC_OUTPUT)
		.value("GRAB_TYPE_LIDAR_STATUS", GrabType::GRAB_TYPE_LIDAR_STATUS)
		.value("GRAB_TYPE_BLOCKAGE_ENVIRONMENTAL", GrabType::GRAB_TYPE_BLOCKAGE_ENVIRONMENTAL)
		.value("GRAB_TYPE_BLOCKAGE_CLASSIFICATION", GrabType::GRAB_TYPE_BLOCKAGE_CLASSIFICATION)
		.value("GRAB_TYPE_THETA_PHI", GrabType::GRAB_TYPE_THETA_PHI)
		.value("GRAB_TYPE_MEMS_PITCH_STATUS", GrabType::GRAB_TYPE_MEMS_PITCH_STATUS)
		.value("GRAB_TYPE_INS_SIGNALS", GrabType::GRAB_TYPE_INS_SIGNALS)
		.value("GRAB_TYPE_RBD_OUTPUT", GrabType::GRAB_TYPE_RBD_OUTPUT)
		.value("GRAB_TYPE_SIGN_GANTRY_DETECTION", GrabType::GRAB_TYPE_SIGN_GANTRY_DETECTION)
		.value("GRAB_TYPE_OM_INDICATIONS", GrabType::GRAB_TYPE_OM_INDICATIONS)
		.value("GRAB_TYPE_OC_OUTPUT_SI", GrabType::GRAB_TYPE_OC_OUTPUT_SI)
		.value("GRAB_TYPE_FOV_OUTPUT_SI", GrabType::GRAB_TYPE_FOV_OUTPUT_SI);


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
			"typeName"_a = "Unknown", "typeMajor"_a = UINT32_MAX, "typeMinor"_a = UINT32_MAX, "itemSize"_a = 0,
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
		.def_readwrite("valid_packets", &invz::ChannelStatistics::valid_packets);


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


	py::class_<PY_PCFrameMeta>(m, "PointCloudFrameMeta")
		.def(py::init<uint32_t, uint8_t, uint8_t, uint8_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, py::array,
			uint16_t, uint8_t, uint8_t, py::array, py::array, uint32_t, py::array, py::array, py::array, py::array, py::array>(),
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
			"v_i_k"_a,
			"alpha_calib_table"_a,
			"beta_calib_table"_a)
		.def_property_readonly("frame_number", &PY_PCFrameMeta::GetFrameNumber)
		.def_property_readonly("scan_mode", &PY_PCFrameMeta::ScanMode)
		.def_property_readonly("system_mode", &PY_PCFrameMeta::SystemMode)
		.def_property_readonly("system_submode", &PY_PCFrameMeta::SystemSubmode)
		.def_property_readonly("timestamp_internal", &PY_PCFrameMeta::TimestampInternal)
		.def_property_readonly("timestamp_utc_sec", &PY_PCFrameMeta::TimestampUtcSecs)
		.def_property_readonly("timestamp_utc_micro", &PY_PCFrameMeta::TimestampUtcMicro)
		.def_property_readonly("fw_version", &PY_PCFrameMeta::FwVersion)
		.def_property_readonly("hw_version", &PY_PCFrameMeta::HwVersion)
		.def_property_readonly("lidar_serial_number", &PY_PCFrameMeta::LidarSerialNumber)
		.def_property_readonly("device_type", &PY_PCFrameMeta::DeviceType)
		.def_property_readonly("active_lrfs", &PY_PCFrameMeta::ActiveLrfs)
		.def_property_readonly("macro_pixel_shape", &PY_PCFrameMeta::MacroPixelShape)
		.def_property_readonly("rows_in_lrf", &PY_PCFrameMeta::RowsInLrf)
		.def_property_readonly("cols_in_lrf", &PY_PCFrameMeta::ColsInLrf)
		.def_property_readonly("total_number_of_points", &PY_PCFrameMeta::TotalNumberOfPoints)
		.def_property_readonly("R_i", &PY_PCFrameMeta::R_i)
		.def_property_readonly("d_i", &PY_PCFrameMeta::D_i)
		.def_property_readonly("v_i_k", &PY_PCFrameMeta::V_i_k)
		.def_property_readonly("alpha_calib_table", &PY_PCFrameMeta::Alpha_calib_table)
		.def_property_readonly("beta_calib_table", &PY_PCFrameMeta::Beta_calib_table);


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
		.def(py::init<uint32_t, uint16_t, py::array>(), "type"_a, "length"_a, "value"_a)
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
		.def_readonly("ui_cookie", &PyTapHandler::ui_cookie)
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

	py::enum_<invz::EFileFormat>(m, "InvzFormat")
		.value("INVZ3", invz::EFileFormat::E_FILE_FORMAT_INVZ3)
		.value("INVZ4", invz::EFileFormat::E_FILE_FORMAT_INVZ4)
		.value("INVZ4_4", invz::EFileFormat::E_FILE_FORMAT_INVZ4_4)
		.value("INVZ4_5", invz::EFileFormat::E_FILE_FORMAT_INVZ4_5)
		.value("INVZ4_6", invz::EFileFormat::E_FILE_FORMAT_INVZ4_6)
		.value("INVZ4_7", invz::EFileFormat::E_FILE_FORMAT_INVZ4_7)
		.value("INVZ5", invz::EFileFormat::E_FILE_FORMAT_INVZ5);


	py::class_<PY_FileReader>(m, "FileReader")
		.def(py::init<const std::string, uint32_t, bool, uint8_t, const std::string>(), "filepath"_a, "log_severity"_a = 3, "check_pixel_validity"_a = false, 
			"num_of_cores"_a = 2, "config_filepath"_a = "")
		.def_readonly("num_of_frames", &PY_FileReader::NumOfFrames)
		.def_readonly("file_format", &PY_FileReader::FileFormat)
		.def("get_device_meta", &PY_FileReader::GetDeviceMeta, pybind11::return_value_policy::copy)
		.def("get_frame", &PY_FileReader::GetFrame, "frame_num"_a = -1, "frame_types"_a, "user_max_packets"_a = std::numeric_limits<int>::max())
		.def("get_packet", &PY_FileReader::GetPacket, "virtual_channels"_a = pybind11::set())
		.def("seek_frame", &PY_FileReader::SeekFrame, "frame_index"_a)
		.def("get_frame_data_attrs", &PY_FileReader::GetFrameDataAttrs)
		.def("register_taps_callback", &PY_FileReader::RegisterTapsCallback)
		.def("unregister_taps_callback", &PY_FileReader::UnregisterTapsCallback)
		.def("grab_taps", &PY_FileReader::GrabTaps, "frame_num"_a = -1)
		.def("register_logs_callback", &PY_FileReader::RegisterLogsCallback)
		.def("unregister_logs_callback", &PY_FileReader::UnregisterLogsCallback)
		.def("grab_logs", &PY_FileReader::GrabLogs, "frame_num"_a = -1);


	py::class_<PY_INVZ4FileRawWriter>(m, "INVZ4FileRawWriter")
		.def(py::init<const std::string, const std::string, py::list>(), "file_name"_a, "device_ip"_a, "virtual_channels_ports"_a)
		.def("write_payload", &PY_INVZ4FileRawWriter::WritePayload, "timestamp"_a, "packet"_a, "port"_a, "frame_number"_a = -1, "packet_length"_a = 0)
		.def("finalize", &PY_INVZ4FileRawWriter::Finalize);


	py::class_<PY_FileWriter>(m, "FileWriter")
		.def(py::init<const std::string, uint32_t >(), "filepath"_a, "file_format"_a = 3)
		.def("write_frame", &PY_FileWriter::WriteFrame, "frame_meta"_a, "macro_pixels_frame"_a, "environmental_bloackage"_a, "blockage_detection"_a, "blockage_classification"_a);

	py::class_<PY_UdpReceiver>(m, "UdpReceiver")
		.def(py::init<const std::string, const int, uint32_t, uint32_t, const std::string&, const std::string&>(), "ipAddress"_a, "port"_a, 
			"marker"_a = INVZ_MARKER, "log_severity"_a = 3, "multicast_ip"_a = "", "networkAdapterIp"_a = "")
		.def("registerCallback", &PY_UdpReceiver::RegisterCallback, "callback"_a)
		.def("startListening", &PY_UdpReceiver::StartListening)
		.def("stopListening", &PY_UdpReceiver::StopListening);

	py::class_<PY_DeviceInterface>(m, "DeviceInterface")
		.def_readonly("connection_level", &PY_DeviceInterface::ConnectionLevel)
		.def_property_readonly("num_data_points", &PY_DeviceInterface::GetNumDataPoints)
		.def(py::init<const std::string, bool, int, std::string, uint32_t, bool>(),
			"config_file_name"_a,
			"is_connect"_a = true,
			"login_level"_a = 0,
			"password"_a = "",
			"log_severity"_a = 3,
			"require_data_attr"_a = true)
		.def("connect", &PY_DeviceInterface::Connect,
			"Establish TCP connection with the device.\n\n"
			"Keyword arguments:\n"
			"request_level -- level of access requested (default 0)."
			"password -- login password\n",
			"request_level"_a = 0, "password"_a = "")
		.def("disconnect", &PY_DeviceInterface::Disconnect,
			"End any existing connection with a device.")
		.def("device_close", &PY_DeviceInterface::DeviceClose,
			"Finalize this DeviceInterface object")
		.def("build_acp", &PY_DeviceInterface::BuildAcp,
			"Create an ACP packet\n\n"
			"Keyword arguments:\n"
			"acp_header - the header of the created ACP packet\n"
			"tlv - a TLV that the ACP packet will contain\n",
			"acp_header"_a, "tlv"_a)
		.def("get_frame", &PY_DeviceInterface::GetFrame,
			"Get a frame from the device.\n\n"
			"Keyword arguments:\n"
			"frame_types -- the requested frame data attributes\n",
			"frame_types"_a)
		.def("get_statistics", &PY_DeviceInterface::GetStatistics,
			"Get statistics about communication with the device")
		.def("activate_buffer", &PY_DeviceInterface::ActivateBuffer,
			"Set whether a certain grab type will be assembled and available in calls to GrabFrame\n\n"
			"Keyword arguments:\n"
			"frame_type: FrameDataAttributes -- grab type to activate/deactivate\n"
			"activate: bool -- whether to activate (true) or deactivate (false) the grab type\n",
			"frame_type"_a, "activate"_a)
		.def("send_tlv", &PY_DeviceInterface::SendTlv,
			"Send a tlv to a connected device.\n\n"
			"Keyword arguments:"
			"acp_header -- the header of the ACP packet that will be sent\n"
			"tlv -- the TLV to send\n"
			"return_error_tlv -- whether to get the error TLV if an error occurs, or throw an exception\n",
			"acp_header"_a,
			"tlv"_a,
			"return_error_tlv"_a = false)
		.def("is_connected", &PY_DeviceInterface::IsConnected,
			"Return if device is connected.")
		.def("get_dp_details", &PY_DeviceInterface::GetDpDetails)
		.def("get_dp_details_by_id", &PY_DeviceInterface::GetDpDetailsById)
		.def("get_dp_dtype", &PY_DeviceInterface::GetDpDtype)
		.def("get_dp_dtype_by_id", &PY_DeviceInterface::GetDpDtypeById)
		.def("get_all_dp_details", &PY_DeviceInterface::GetAllDpDetails)
		.def("get_all_tap_details", &PY_DeviceInterface::GetAllTapDetails)
		.def("get_dp", &PY_DeviceInterface::GetDp, "dp_name"_a, "get_dp_policy"_a = (uint32_t)invz::E_GET_PARAM_POLICY_AUTO)
		.def("get_empty_dp", &PY_DeviceInterface::GetEmptyDp)
		.def("get_zero_dp", &PY_DeviceInterface::GetZeroDp)
		.def("set_dp", &PY_DeviceInterface::SetDp, "dp_name"_a, "obj"_a, "set_param"_a = false)
		.def("get_register_details", &PY_DeviceInterface::GetRegisterDetails)
		.def("get_all_registers_details", &PY_DeviceInterface::GetAllRegistersDetails)
		.def("get_register_by_name", &PY_DeviceInterface::GetRegisterByName)
		.def("set_register_by_name", &PY_DeviceInterface::SetRegisterByName)
		.def("get_register_by_address", &PY_DeviceInterface::GetRegisterByAddress)
		.def("set_register_by_address", &PY_DeviceInterface::SetRegisterByAddress)
		.def("set_tap_activation_state", &PY_DeviceInterface::SetTapActivationState,
			"Set whether a certain tap will be assembled and be available to a callback registered via register_taps_callback.\n\n"
			"Positional arguments:\n"
			"dp_name: the name of the tap to activate/deactivate\n"
			"should_enable: bool -- whether to activate (true) or deactivate (false) the tap\n")
		.def("record", &PY_DeviceInterface::Record,
			"Start recording (synchronously) the data received from the device to a recording folder.\n\n"
			"Keyword arguments:\n"
			"seconds -- how many seconds to record\n"
			"filepath -- the path of the folder in which the recording files will be saved (default innopy.record)\n"
			"flush_queues -- whether to flush all packets received from the device before starting to record\n",
			"seconds"_a, "filepath"_a = INNOPY_DEFAULT_RECORD_FILENAME, "flush_queues"_a = false)
		.def("start_recording", &PY_DeviceInterface::StartRecording,
			"Same as record but asynchronous and the recording is stopped via stop_recording.\n"
			"filepath"_a = INNOPY_DEFAULT_RECORD_FILENAME, "flush_queues"_a = false)
		.def("stop_recording", &PY_DeviceInterface::StopRecording,
			"Stop a recording started via start_recording.")
		.def("register_taps_callback", &PY_DeviceInterface::RegisterTapsCallback,
			"Register a callback which will be called when a new tap is available.\n\n"
			"Positional arguments:\n"
			"callback: PyTapHandler -> None -- the callback to register\n")
		.def("unregister_taps_callback", &PY_DeviceInterface::UnregisterTapsCallback,
			"Unregister the currently registered taps callback if it exists.",
			py::call_guard<py::gil_scoped_release>()) // py::call_guard<py::gil_scoped_release>() called in order to release GIL
		.def("register_logs_callback", &PY_DeviceInterface::RegisterLogsCallback,
			"Register a callback which will be called when a new log is available\n\n"
			"Positional arguments:\n"
			"callback: PyLogHandler -> None -- the callback to register\n")
		.def("unregister_logs_callback", &PY_DeviceInterface::UnregisterLogsCallback,
			"Unregister the currently registered logs callback if it exists.",
			py::call_guard<py::gil_scoped_release>())
		.def("register_new_frame_callback", &PY_DeviceInterface::RegisterNewFrameCallback,
			"Register a callback which will be called when a new frame is ready.\n\n"
			"Positional arguments:\n"
			"callback: Int -> None -- the callback called when a new frame is ready with the frame number as an argument\n")
		.def("unregister_new_frame_callback", &PY_DeviceInterface::UnregisterNewFrameCallback,
			"Unregister the currently registered new frame callback if it exists.",
			py::call_guard<py::gil_scoped_release>())
		.def("register_new_tlv_callback", &PY_DeviceInterface::RegisterNewTlvCallback,
			"Register a callback which will be called when a tlv is received.\n\n"
			"Positional arguments:\n"
			"callback: PyTLVPack -> None -- the callback called when a new frame is ready with the frame number as an argument\n")
		.def("unregister_new_tlv_callback", &PY_DeviceInterface::UnregisterNewTlvCallback,
			"Unregister the currently registered new frame callback if it exists.",
			py::call_guard<py::gil_scoped_release>())
			.def("cs_handshake", &PY_DeviceInterface::CSHandshake);

	py::class_<FrameHelper>(m, "FrameHelper")
		.def(py::init<>())
		.def("get_empty_macro_pixels_frame", &FrameHelper::GetEmptyMacroPixelsFrame, "pixel_count"_a = 1, "channel_count"_a = 8, "reflection_count"_a = 3)
		.def("get_empty_summation_pixels_frame", &FrameHelper::GetEmptySummationPixelsFrame, "pixel_count"_a = 1, "channel_count"_a = 8, "reflection_count"_a = 2)
		.def("convert_byte_stream_to_macro_pixel_frame", &FrameHelper::ConvertByteStreamToMacroPixelFrame, "py_device_meta"_a, "byte_stream"_a)
		.def("get_direction_by_mems_feedback", &FrameHelper::GetDirectionByMemsFeedback, "device_meta"_a, "lrf"_a, "mems_feedback"_a);


	py::enum_<invz::RBSide>(m, "RBSide")
		.value("RBD_LEFT_SIDE", invz::RBSide::RBD_LEFT_SIDE)
		.value("RBD_RIGHT_SIDE", invz::RBSide::RBD_RIGHT_SIDE)
		.value("RBD_SIDES_MAX", invz::RBSide::RBD_SIDES_MAX);

	py::enum_<invz::box_3d_class>(m, "box_3d_class")
		.value("BOX_3D_TYPE_GENERAL", invz::box_3d_class::BOX_3D_TYPE_GENERAL)
		.value("BOX_3D_TYPE_SIGN_GANTRY", invz::box_3d_class::BOX_3D_TYPE_SIGN_GANTRY);
}
	
