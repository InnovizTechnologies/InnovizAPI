// Copyright 2021 Innoviz Technologies
//
// Licensed under the Innoviz Open Dataset License Agreement (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://github.com/InnovizTechnologies/InnovizAPI/blob/master/LICENSE.md
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __INVZ_TYPES_H__
#define __INVZ_TYPES_H__

/**
* @file invz_types.h
*/

#include "common.h"
#include "invz_constants.h"
#include "ndarray/ndarray.h"

#include "VB.hpp"

#include <string>


#define CS_IGNORE
#define RBD_ROADSIDE_POINTS_NUM 5


namespace invz {

CS_IGNORE	const size_t MAX_DETECTED_OBJS_IN_FRAMES = 100;
CS_IGNORE	const size_t NUMBER_OF_PC_PLUS_DETECTION_POINT = 238301;



	enum CommunicationType {
		TCP,
		UART
	};

	enum FrameType {
		FRAME_TYPE_NONE =					0x000000,
		FRAME_TYPE_NEWFRAME =				0x000001,
		FRAME_TYPE_NEWOBJECT =				0x000002,
		FRAME_TYPE_SENSOR_POSE=				0x000004,
		FRAME_TYPE_TRACKING=				0x000008,
		FRAME_TYPE_PARTIAL_FRAME =			0x000010,
		FRAME_TYPE_BLOCKAGE_CLASSIFICATION= 0x000020,
		FRAME_TYPE_PC_PLUS =				0x000040,
		FRAME_TYPE_LANDMARKS =				0x000080,
		FRAME_TYPE_POLES =					0x000100,
	};

	enum ChannelType : uint16_t
	{
		CHANNEL_TYPE_POINT_CLOUD,
		CHANNEL_TYPE_SLOW_DATA,
		CHANNEL_TYPE_RESERVED,
		CHANNEL_TYPE_DEBUG,
		CHANNEL_TYPE_CNC,
		CHANNEL_TYPE_MAX,
	};

	/**
	* List of frame data attributes to grab.
	*/
	enum GrabType: uint64_t
	{
		GRAB_TYPE_UNKOWN= 0,
		GRAB_TYPE_METADATA = 0XFFFFFFFF00000000 | 0x050017,
		GRAB_TYPE_THETA_PHI = 0XFFFFFFF000000000 | 0xFFFF05,
		GRAB_TYPE_MACRO_PIXEL_META_DATA = 0XFFFFFFF000000000 | 0xFFFF0C,
		GRAB_TYPE_SINGLE_PIXEL_META_DATA = 0XFFFFFFF000000000 | 0xFFFF0D,
		GRAB_TYPE_SUM_PIXEL_META_DATA = 0XFFFFFFF000000000 | 0xFFFF0E,
		GRAB_TYPE_PIXEL_LANE_MARK_TRAILER = 0XFFFFFFF000000000 | 0xFFFF0F,
		GRAB_TYPE_MEASURMENTS_REFLECTION0 = 0XFFFFFFF000000000 | 0xFFFF00,
		GRAB_TYPE_MEASURMENTS_REFLECTION1 = 0XFFFFFFF100000000 | 0xFFFF00,
		GRAB_TYPE_MEASURMENTS_REFLECTION2 = 0XFFFFFFF200000000 | 0xFFFF00,
		GRAB_TYPE_SUMMATION_REFLECTION0 = 0XFFFFFFF000000000 | 0xFFFF02,
		GRAB_TYPE_SUMMATION_REFLECTION1 = 0XFFFFFFF100000000 | 0xFFFF02,
		GRAB_TYPE_DIRECTIONS = 0XFFFFFFF000000000 | 0xFFFF01,
		GRAB_TYPE_SUMMATION_DIRECTIONS = 0XFFFFFFF000000000 | 0xFFFF03,
		GRAB_TYPE_PC_PLUS =0x00101002, /**< PC Plus*/
#ifdef _MSC_VER
		GRAB_TYPE_PC_PLUS_SUMMATION = 0xFFFFFFF100000000 | 0x00101002,
#endif
		GRAB_TYPE_TRACKED_OBJECTS_SI = 0x00101003,/**< Tracked Objects*/
		GRAB_TYPE_DETECTIONS_SI = 0x00101007,/**< Detections Objects*/
		GRAB_TYPE_SENSOR_POSE = 0x100000000 | 0x001000011,
		GRAB_TYPE_PC_PLUS_METADATA = 0x00101001,
		GRAB_TYPE_DETECTIONS = 0x00100001,/**< Detections Objects*/
		GRAB_TYPE_TRACKED_OBJECTS = 0x0010000A,/**< Tracked Objects*/
		GRAB_TYPE_OC_OUTPUT = 0x0010000E, 
		GRAB_TYPE_DC_OUTPUT = 0x00100010,
		GRAB_TYPE_BLOCKAGE =	0x050013,
		GRAB_TYPE_BLOCKAGE_CLASSIFICATION = 0x050014,
		GRAB_TYPE_BLOCKAGE_ENVIRONMENTAL = 0XFFFFFFFF00000000 | 0x050012,
		GRAB_TYPE_GLARE_IN_FOV = 0x050015,
		GRAB_TYPE_OM_INDICATIONS = 0XFFFFFFFF00000000 | 0x050039,
		GRAB_TYPE_LIDAR_STATUS= 0XFFFFFFFF00000000 | 0x050041,
		GRAB_TYPE_PC_PLUS_METADATA_48K = 0x00101010,
		GRAB_TYPE_INS_SIGNALS = 0x00100027,
		GRAB_TYPE_RBD_OUTPUT = 0x00101012,
		GRAB_TYPE_SIGN_GANTRY_DETECTION = 0x00100029,
		GRAB_TYPE_MEMS_PITCH_STATUS = 0XFFFFFFFF00000000 | 0x050042,
		GRAB_TYPE_OC_OUTPUT_SI = 0x00101004,
		GRAB_TYPE_FOV_OUTPUT_SI = 0x00101005
	};

#ifdef __GNUC__
	__attribute__((deprecated)) const GrabType GRAB_TYPE_PC_PLUS_SUMMATION = GrabType(0xFFFFFFF100000000 | 0x00101002);
#elif defined _MSC_VER
	#pragma deprecated(GRAB_TYPE_PC_PLUS_SUMMATION)
#endif


	enum PixelValidity :uint8_t
	{
		PIXEL_VALIDITY_MISSING = 0,
		PIXEL_VALIDITY_VALID = 1,
		PIXEL_VALIDITY_INVALID = 2, 
		PIXEL_BLOOMING = 3,
		PIXEL_NOISE = 4
	};

	struct INVZ_API eTLV
	{
		uint32_t type;
		uint16_t length;
		uint16_t reserved;
		uint8_t  *value;
		eTLV()
		{
			value = nullptr;
			length = 0;
			reserved = 0;
		};

		eTLV(eTLV& other)
		{
			type = other.type;
			length = other.length;
			reserved = other.reserved;
			value = new uint8_t[length];
			memcpy(value, other.value, length);
		};

		eTLV(uint32_t _type, uint16_t _length, uint8_t *_data)
		{
			type = _type;
			length = _length;
			reserved = 0;
			value = new uint8_t[length];
			memcpy(value, _data, length);
		};

		~eTLV()
		{
			if (value)
			{
				delete[] value;
				value = nullptr;
			}
		}

	};

	struct INVZ_API AcpHeaderTlvPack
	{
		uint32_t marker;
		uint16_t length;
		uint8_t communication_options;
		uint8_t master_id;
		uint8_t is_response;
		uint8_t options;
		uint8_t protocol_version;
		uint16_t sequence_number;
		uint16_t return_code;

		AcpHeaderTlvPack() :
		marker(INVZ_MARKER),
		length(0),
		communication_options(0),
		master_id(0),
		is_response(0),
		options(0),
		protocol_version(0),
		sequence_number(UINT16_MAX),
		return_code(0)
		{

		}
	};

	struct INVZ_API TlvPack
	{
		AcpHeaderTlvPack acp_header;
		eTLV tlv;
		uint16_t port = UINT16_MAX;
		uint32_t virtual_channel = UINT32_MAX;

		TlvPack()
		{

		};

		TlvPack(uint32_t _type, uint16_t _length, uint8_t *_data) : tlv(_type, _length, _data)
		{

		};
	};


	struct INVZ_API TapHandler
	{
		uint32_t frame_number;
		uint32_t ids_count;
		uint32_t* ids;


		TapHandler()
		{
			frame_number = UINT32_MAX;
			ids_count = 0;
			ids = nullptr;
		};

		TapHandler(TapHandler& other)
		{
			frame_number = other.frame_number;
			ids_count = other.ids_count;
			ids = new uint32_t[ids_count];
			memcpy(ids, other.ids, ids_count * sizeof(uint32_t));
		};

		TapHandler(uint16_t _length, uint32_t *_data)
		{
			ids_count = _length;
			ids = new uint32_t[_length];
			memcpy(ids, _data, ids_count * sizeof(uint32_t));
		};

		~TapHandler()
		{
			if (ids)
			{
				delete[] ids;
				ids = nullptr;
			}
		}
	};

	struct INVZ_API TapEventData
	{
		uint64_t timestamp = 0;
		uint32_t frame_number;
		uint32_t parameter_id;
		uint32_t ui_cookie = UINT32_MAX;
		size_t length = 0;
		std::unique_ptr<uint8_t[]> data;

		TapEventData()
		{
			frame_number = UINT32_MAX;
		};

		TapEventData(TapEventData& other)
		{
			timestamp = other.timestamp;
			frame_number = other.frame_number;
			parameter_id = other.parameter_id;
			ui_cookie = other.ui_cookie;
			length = other.length;
			data = std::make_unique<uint8_t[]>(length);
			memcpy(data.get(), other.data.get(), length);
		};

		TapEventData(uint16_t _length, uint8_t *_data)
		{
			length = _length;
			data = std::make_unique<uint8_t[]>(length);
			memcpy(data.get(), _data, length);
		};

		~TapEventData()
		{
			data.reset();
		}

	};

	struct INVZ_API RuntimeLogEventData
	{
		uint64_t timestamp;
		uint32_t frame_number;
		uint32_t core_id;
		uint32_t sequence_number;
		uint32_t flow;
		uint32_t line_number;
		double param0;
		double param1;
		double param2;
		std::string severity;
		std::string package;
		std::string source_file;
		std::string message_name;
		std::string message;

		RuntimeLogEventData(uint64_t _timestamp = 0, uint32_t _frame_number = 0, uint32_t _core_id = 0, uint32_t _sequence_number = 0,
			uint32_t _flow = 0, uint32_t _line_number = 0, double _param0 = 0, double _param1 = 0, double _param2 = 0,
			std::string _severity = "", std::string _package = "", std::string _source_file = "", std::string _message_name = "", std::string _message = "") :
			timestamp{ _timestamp }, frame_number{ _frame_number }, core_id{ _core_id }, sequence_number{ _sequence_number },
			flow{ _flow }, line_number{_line_number}, param0{_param0}, param1{_param1}, param2{_param2},
			severity{_severity}, package{_package}, source_file{_source_file}, message_name{_message_name}, message{_message}
		{

		};

		RuntimeLogEventData(const RuntimeLogEventData& other) = default;
		
		~RuntimeLogEventData() = default;

	};

	/* Callback of received TLV */
	typedef void(__stdcall * TlvCallback)(TlvPack*);

	/* Callback of Frame */
	typedef void(__stdcall * FrameCallback)(uint32_t*);

	/* Callback of TAP Event handler */
	typedef void(__stdcall * TapEventCallback)(TapEventData*);

	/* Callback of RuntimeLog Event handler */
	typedef void(__stdcall * RuntimeLogEventCallback)(RuntimeLogEventData*);

	/**
	* @brief A callback function to use reporting progress of misc flows
	*/
	typedef void(__stdcall * ReportProgress)(const uint32_t part, const uint32_t total);


	//! log severity levels
	enum LogSeverityLevel
	{
		trace,
		debug,
		info,
		warning,
		error,
		fatal,
		none
	};


	enum DeviceType
	{
		pro_lidar = 1,
		compute_module
	};

	//firmware upgarde target
	enum UpgradeTarget
	{
		UPGRADE_BOOTLOADER = 0x00000001,
		UPGRADE_APPLICATION = 0x00000002,
		UPGRADE_BOOTLOADER1 = 0x00000003,
		UPGRADE_BOOTLOADER2 = 0x00000004,
		UPGRADE_APPLICATION1 = 0x00000005,
		UPGRADE_APPLICATION2 = 0x00000006
	};
	/**
	* @brief Possible LiDAR's states. some states are not relevant for all tpe of devices
	*/
	enum LidarState
	{
		OPERATIONAL_STATE = 0,
		LOW_POWER_STATE = 1,
		INITIALIZATION_STATE = 2,
		STANDBY_CALIBRATION_ONLY_STATE = 3,
		CALIBRATION_STATE = 4,
		STANDBY_STATE = 5,
		HIGH_POWER_STATE = 6,
		UPGRADE_STATE = 7

	};
	/**
	* @brief Byte variable declaration
	*/
	typedef uint8_t byte;

#pragma pack(push, 1)
	/**
	* @brief Defines a 3d vector.
	*/
	struct INVZ_API vector3 {
		float x = 0.0; /*!< x axis */
		float y = 0.0; /*!< y axis */
		float z = 0.0; /*!< z axis */
	};

	/** @brief Defines a single pixel data content.
	*	@struct Measurement
	*/
	struct INVZ_API INVZ2MeasurementXYZType
	{
		uint16_t distance = 0;		/*!< Distance (in cm) */
		int8_t confidence = 0;
		int8_t grazing_angle = 0;
		uint8_t reflectivity = 0;	/*!< Compressed reflectivity, see "InnovizPro Communication Interface Guide.pdf" for more details */
		uint8_t noise = 0;
		float x = 0.0; /*!< x axis */
		float y = 0.0; /*!< y axis */
		float z = 0.0; /*!< z axis */
		PixelValidity validity = PixelValidity::PIXEL_VALIDITY_VALID;
		float pfa = 0;
	};

	struct INVZ_API INVZ2SumMeasurementXYZType
	{
		uint16_t distance = 0;		/*!< Distance (in cm) */
		int8_t confidence = 0;
		uint8_t reflectivity = 0;	/*!< Compressed reflectivity, see "InnovizPro Communication Interface Guide.pdf" for more details */
		uint8_t noise = 0;
		float x = 0.0; /*!< x axis */
		float y = 0.0; /*!< y axis */
		float z = 0.0; /*!< z axis */
		PixelValidity validity = PixelValidity::PIXEL_VALIDITY_VALID;
		float pfa = 0;
	};

	struct INVZ_API INVZ2MacroMetaData
	{
		uint8_t pixel_type = 0;
		uint8_t summation_type = 0;
		uint8_t rise_time = 0;
		uint8_t num_active_channels = 0;
		uint8_t is_blocked = 0;
		uint8_t ultra_short_range = 0;
		uint8_t artificial_macro_pixel = 0;
		uint8_t short_range_detector_status = 0;
		uint16_t mems_feedback_x = 0;
		uint16_t mems_feedback_y = 0;
		uint16_t blockage_pulse_width = 0;
    };

	struct INVZ_API INVZ2PixelMetaData
	{
		uint8_t n_reflection = 0;
		uint8_t short_range_reflection = 0;
		uint8_t ghost = 0;
		uint8_t reflection_valid_0 = 0;
		uint8_t reflection_valid_1 = 0;
		uint8_t	reflection_valid_2 = 0;
		uint8_t noise = 0;
		uint8_t lane_mark_trailer_available = 0;
	};

	struct INVZ_API INVZ2SumPixelMetaData
	{
		uint8_t sum_n_reflection = 0;
		uint8_t sum_short_range_reflection = 0;
		uint8_t sum_ghost = 0;
		uint8_t sum_reflection_valid_0 = 0;
		uint8_t sum_reflection_valid_1 = 0;

	};

	struct INVZ_API INVZ2PixelLaneMarkTrailer
	{
		uint32_t score = 0;
		uint16_t reference = 0;
		uint8_t ga_index = 0;
		uint8_t pulses_fired = 0;
	};


	union FrameDataType64
	{
		struct MajorMinor
		{
			uint32_t typeMajor;
			uint32_t typeMinor;
		} typeSplit;

		uint64_t typeFull;

		FrameDataType64() : typeFull{ UINT64_MAX }
		{

		}
	};

	struct ReflectionMatrix
	{
		float matrix[9]{ 0 };
	};

	struct ChannelNormal
	{
		vector3 channels[8];
	};

	class INVZ_API DeviceMeta {

	public:
		DeviceMeta(const uint16_t* p_lrf_width = nullptr, const uint8_t* p_lrf_height = nullptr, const ReflectionMatrix* p_Ri = nullptr, 
			const vector3* p_di = nullptr, const ChannelNormal* p_vik = nullptr, const float* p_alpha_calib = nullptr, const float* p_beta_calib = nullptr);

		~DeviceMeta() = default;

		uint32_t lrf_count = DEVICE_NUM_OF_LRFS;
		uint16_t lrf_width[DEVICE_NUM_OF_LRFS];
		uint8_t lrf_height[DEVICE_NUM_OF_LRFS];
		ReflectionMatrix Ri[DEVICE_NUM_OF_LRFS];
		vector3 di[DEVICE_NUM_OF_LRFS];
		ChannelNormal vik[DEVICE_NUM_OF_LRFS];
		float alpha_calib_table[META_CALIB_TABLE_SIZE];
		float beta_calib_table[META_CALIB_TABLE_SIZE];
		uint32_t fov_segments_count = EAGLE_FOV_SEGMENTS;

		bool IsReady();
		void SetReady();
		void SetExternalDi(std::string di_path);
		void SetFovSegmentsCount(uint32_t _fov_segments_count);

	private:
		bool m_ready;

	};

	/* INVZ4 PointCloud */

	struct PCFrameMeta
	{
		uint32_t frame_number;
		uint8_t	scan_mode;
		uint8_t	reserved;
		uint8_t	system_mode;
		uint8_t	system_submode;
		uint32_t timestamp_internal;
		uint32_t timestamp_utc_secs;
		uint32_t timestamp_utc_micro;
		uint32_t fw_version;
		uint32_t hw_version;
		uint8_t lidar_serial_number[INVZ4_LIDAR_SN_LENGTH];

		PCFrameMeta(uint32_t _frame_number, uint8_t _scan_mode, uint8_t _system_mode, uint8_t _system_submode, uint32_t _timestamp_internal,
			uint32_t _timestamp_utc_sec, uint32_t _timestamp_utc_micro, uint32_t _fw_version, uint32_t _hw_version, uint8_t* p_lidar_serial)
		{
			memset(this, 0, sizeof(PCFrameMeta));
			frame_number = _frame_number;
			scan_mode = _scan_mode;
			system_mode = _system_mode;
			system_submode = _system_submode;
			timestamp_internal = _timestamp_internal;
			timestamp_utc_secs = _timestamp_utc_sec;
			timestamp_utc_micro = _timestamp_utc_micro;
			fw_version = _fw_version;
			hw_version = _hw_version;
			memcpy(lidar_serial_number, p_lidar_serial, INVZ4_LIDAR_SN_LENGTH);
		};
	};

	struct INVZ_API CSampleFrameMeta
	{
		uint32_t frame_number;
		uint8_t	scan_mode;
		uint8_t	pc_protocol_version;
		uint8_t	system_mode;
		uint8_t	system_submode;
		uint32_t timestamp_internal;
		uint32_t timestamp_utc_secs;
		uint32_t timestamp_utc_micro;
		uint32_t fw_version;
		uint32_t hw_version;
		uint8_t lidar_serial_number[INVZ4_CSAMPLE_LIDAR_SERIAL_LENGTH];
		uint16_t device_type;
		uint8_t active_lrfs;
		uint8_t macro_pixel_shape; /* (4bits,4bits) */
		uint8_t rows_in_lrf[DEVICE_NUM_OF_LRFS];
		uint16_t cols_in_lrf[DEVICE_NUM_OF_LRFS];
		uint32_t total_number_of_points;
		uint8_t reserved1[8];
		ReflectionMatrix R_i[DEVICE_NUM_OF_LRFS];
		vector3 d_i[DEVICE_NUM_OF_LRFS];
		ChannelNormal v_i_k[DEVICE_NUM_OF_LRFS];
		uint8_t reserved2[4];
		float alpha_calib_table[META_CALIB_TABLE_SIZE];
		float beta_calib_table[META_CALIB_TABLE_SIZE];

		CSampleFrameMeta() {};

		CSampleFrameMeta(uint32_t _frame_number, uint8_t _scan_mode, uint8_t _system_mode, uint8_t _system_submode, uint32_t _timestamp_internal,
			uint32_t _timestamp_utc_sec, uint32_t _timestamp_utc_micro, uint32_t _fw_version, uint32_t _hw_version, uint8_t* _lidar_serial,
			uint16_t _device_type, uint8_t _active_lrfs, uint8_t _macro_pixel_shape, uint8_t* _rows_in_lrf, uint16_t* _cols_in_lrf, uint32_t _total_number_of_points,
			ReflectionMatrix* _R_i, vector3* _d_i, ChannelNormal* _v_i_k, float* _alpha_calib_table, float* _beta_calib_table)
		{
			memset(this, 0, sizeof(CSampleFrameMeta));
			frame_number = _frame_number;
			scan_mode = _scan_mode;
			system_mode = _system_mode;
			system_submode = _system_submode;
			timestamp_internal = _timestamp_internal;
			timestamp_utc_secs = _timestamp_utc_sec;
			timestamp_utc_micro = _timestamp_utc_micro;
			fw_version = _fw_version;
			hw_version = _hw_version;
			memcpy(lidar_serial_number, _lidar_serial, INVZ4_CSAMPLE_LIDAR_SERIAL);
			device_type = _device_type;
			active_lrfs = _active_lrfs;
			macro_pixel_shape = _macro_pixel_shape;
			memcpy(rows_in_lrf, _rows_in_lrf, sizeof(uint8_t)*DEVICE_NUM_OF_LRFS);
			memcpy(cols_in_lrf, _cols_in_lrf, sizeof(uint16_t)*DEVICE_NUM_OF_LRFS);
			total_number_of_points = _total_number_of_points;
			memcpy(R_i, _R_i, sizeof(ReflectionMatrix)*DEVICE_NUM_OF_LRFS);
			memcpy(d_i, _d_i, sizeof(vector3)*DEVICE_NUM_OF_LRFS);
			memcpy(v_i_k, _v_i_k, sizeof(ChannelNormal)*DEVICE_NUM_OF_LRFS);
			memcpy(alpha_calib_table, _alpha_calib_table, META_CALIB_TABLE_SIZE*(sizeof(float)));
			memcpy(beta_calib_table, _beta_calib_table, META_CALIB_TABLE_SIZE*(sizeof(float)));
		};
	};

	struct EndOfFrame
	{
		uint32_t frame_number;
		uint32_t reserved;
	};

	
	/*struct StartOfFrame
	{
		uint32_t frame_number;
		uint32_t reserved;
	};*/

	struct INVZ4PixelsStream {
		uint32_t first_macro_pixel_index;
		uint32_t last_macro_pixel_index;
		uint32_t total_macro_pixels_in_lrf;
		uint32_t first_summation_index; /* Group of macro pixels to complete summation pixel */
		uint32_t last_summation_index;
		uint8_t pixels_buffer[];
	};

	enum SummationTypeEnum {
		E_SUMMATION_TYPE_NONE = 0,
		E_SUMMATION_TYPE_1X1 = 1,
		E_SUMMATION_TYPE_1X2 = 2,
		E_SUMMATION_TYPE_1X3 = 3,
		E_SUMMATION_TYPE_1X4 = 4,
		E_SUMMATION_TYPE_2X1 = 5,
		E_SUMMATION_TYPE_2X2 = 6,
		E_SUMMATION_TYPE_2X3 = 7,
		E_SUMMATION_TYPE_2X4 = 8,
		E_SUMMATION_TYPE_4X1 = 9,
		E_SUMMATION_TYPE_4X2 = 10,
		E_SUMMATION_TYPE_4X3 = 11,
		E_SUMMATION_TYPE_4X4 = 12
	};


	union MacroPixelIndex {
		struct Bits
		{
			uint32_t macro_pixel_number : 16;
			uint32_t reserved : 8;
			uint32_t lrf_number : 8;
		} bits;
		uint32_t macro_pixel_number;
	};

#define INVZ4_SINGLE_PIXEL_REFLECTION_COUNT 3
#define INVZ4_SUMMATION_SINGLE_PIXEL_REFLECTION_COUNT 2
#define INVZ4_MACRO_PIXEL_CHANNEL_COUNT 8
#define INVZ4_SUMMATION_CHANNEL_COUNT 4
	// TODO - Jule - Currenty using summation type 2x2 - update afterwards
#define INVZ4_SUMMATION_MACRO_PIXEL_CHANNEL_COUNT static_cast<int>(INVZ4_MACRO_PIXEL_CHANNEL_COUNT*0.5f)

	union ConfidenceAndAngle8Bits
	{
		struct
		{
			int8_t confidence : 6;
			uint8_t angle_indication : 2;
		} bits_4_3;
		struct
		{
			uint8_t confidence : 6;
			uint8_t grazing_angle : 2;
		} bits;
		int8_t value = 0;
	};

	struct ReflectionAttributes
	{
		uint16_t distance = 0;
		uint8_t reflectivity = 0;
		ConfidenceAndAngle8Bits confidence;
	};

	//macro pixel headers(1+2)
	union MacroPixelMeta16Bits
	{
		struct RegualrBits
		{
			uint16_t pixel_type : 1;
			uint16_t summation_type : 4;
			uint16_t active_channels : 3;
			uint16_t is_blocked : 1;
			uint16_t short_range_status : 2;
			uint16_t reserved : 5;
		} bits;

		struct RegualrBits4_6
		{
			uint16_t pixel_type : 1;
			uint16_t summation_type : 4;
			uint16_t reserved_0 : 3;
			uint16_t is_blocked : 1;
			uint16_t reserved_1 : 1;
			uint16_t rise_time : 5;
			uint16_t artificial_macro_pixel : 1;
		} bits4_6;

		struct RegualrBits4_7
		{
			uint16_t pixel_type : 1;
			uint16_t summation_type : 4;
			uint16_t reserved_0 : 3;
			uint16_t is_blocked : 1;
			uint16_t ultra_short_range : 1;
			uint16_t rise_time : 5;
			uint16_t artificial_macro_pixel : 1;
		} bits4_7;

		uint16_t value = 0;
	};

	union SummationMacroPixelMeta16Bits
	{
		struct SummationBits
		{
			uint16_t pixel_type : 1;
			uint16_t summation_type : 4;
			uint16_t active_channels : 3;
			uint16_t reserved : 8;
		} bits;
		uint16_t value = 0;
	};

	union SinglePixelMeta8Bits {
		struct Bits
		{
			uint8_t n_reflections : 2;
			uint8_t short_range : 2;
			uint8_t ghost : 1;
			uint8_t reflection0_valid : 1;
			uint8_t reflection1_valid : 1;
			uint8_t reflection2_valid : 1;
		} bits;
		uint8_t value = 0;
	};

	union SinglePixelMeta8Bits4_5 {
		struct Bits
		{
			uint8_t n_reflections : 2;
			uint8_t short_range : 2;
			uint8_t ghost_type : 4;
		} bits;
		uint8_t value = 0;
	};

	union SinglePixelMeta8Bits5 {
		struct Bits
		{
			uint8_t n_reflections : 2;
			uint8_t short_range : 1;
			uint8_t lane_mark_trailer_available : 1;
			uint8_t ghost_type : 4;
		} bits;
		uint8_t value = 0;
	};

	union SummationSinglePixelMeta8Bits {
		struct Bits
		{
			uint8_t n_reflections : 2;
			uint8_t short_range : 2;
			uint8_t ghost : 1;
			uint8_t reflection0_valid : 1;
			uint8_t reflection1_valid : 1;
			uint8_t reserved : 1;
		} bits;
		uint8_t value = 0;
	};

	union SummationSinglePixelMeta8Bits4_5 {
		struct Bits
		{
			uint8_t n_reflections : 2;
			uint8_t short_range : 2;
			uint8_t ghost_type : 4;
		} bits;
		uint8_t value = 0;
	};

	struct MemsFeedback {
		int16_t theta = 0; /* tan(theta/2) */
		int16_t phi = 0; 	/* tan(phi/2) 	*/
	};

	struct SinglePixelFixed
	{
		SinglePixelMeta8Bits pixel_meta; 										/* 1 byte  */
		uint8_t noise = 0;															/* 1 byte  */
		ReflectionAttributes reflection[INVZ4_SINGLE_PIXEL_REFLECTION_COUNT];	/* 12 bytes */
	};

	// Jule
	struct SinglePixelOneReflectionFixed
	{
		SinglePixelMeta8Bits pixel_meta; 	/* 1 byte  */
		uint8_t noise = 0;						/* 1 byte  */
		ReflectionAttributes reflection; 	/* 4 bytes */
	};

	struct SummationSinglePixelFixed {
		SummationSinglePixelMeta8Bits summation_pixel_meta; 	/* 1 byte  */
		uint8_t noise = 0;											/* 1 byte  */
		ReflectionAttributes reflection[INVZ4_SUMMATION_SINGLE_PIXEL_REFLECTION_COUNT]; /* 8 bytes */
	};

	// Jule
	struct SummationSinglePixelOneReflectionFixed {
		SummationSinglePixelMeta8Bits summation_pixel_meta; 	/* 1 byte  */
		uint8_t noise = 0;											/* 1 byte  */
		ReflectionAttributes reflection;						/* 4 bytes */
	};

	struct MacroPixelFixed
	{
		MacroPixelMeta16Bits header;	/* 2 byte  */
		MemsFeedback mems_feedback;		/* 4 byte  */
		uint16_t blockage_pw = 0;			/* 2 byte  */
		SinglePixelFixed channels[INVZ4_MACRO_PIXEL_CHANNEL_COUNT]; /* 112 byte  */
	};

	// Jule
	struct MacroPixel8ChannelsOneReflectionFixed
	{
		MacroPixelMeta16Bits header;												/* 2 byte  */
		MemsFeedback mems_feedback;													/* 4 byte  */
		uint16_t blockage_pw = 0;														/* 2 byte  */
		SinglePixelOneReflectionFixed channels[INVZ4_MACRO_PIXEL_CHANNEL_COUNT];	/* 40 byte  */
	};

	struct SummationMacroPixelFixed
	{
		SummationMacroPixelMeta16Bits header;	/* 2 byte  */
		//MemsFeedback mems_feedback;				/* 4 byte  */
		//uint16_t blockage_pw = 0;					/* 2 byte  */
		SummationSinglePixelFixed channels[INVZ4_MACRO_PIXEL_CHANNEL_COUNT]; /* 80 byte  */
	};

	// Jule
	struct SummationMacroPixel4ChannelsOneReflectionFixed
	{
		SummationMacroPixelMeta16Bits header;												/* 2 byte  */
		SummationSinglePixelOneReflectionFixed channels[INVZ4_SUMMATION_MACRO_PIXEL_CHANNEL_COUNT];	/* 48 byte  */
	};

	struct DetailedSinglePixel
	{
		MacroPixelMeta16Bits header;	/* 2 byte  */
		uint16_t blockage_pw;			/* 2 byte  */
		vector3 direction;				/* 12 byte */
		SinglePixelFixed pixel;			/* 14 bytes */
	};



	struct DetailedSummationSinglePixel
	{
		SummationMacroPixelMeta16Bits header;		/* 2 byte  */
		uint16_t blockage_pw;						/* 2 byte  */
		vector3 direction;							/* 12 byte */
		SummationSinglePixelFixed summation_pixel;	/* 10 bytes */
	};

	struct SinglePixelHeader
	{
		union
		{
			SinglePixelMeta8Bits pixel_meta; 	/* 1 byte  */
			SinglePixelMeta8Bits4_5 pixel_meta4_5;
			SinglePixelMeta8Bits5 pixel_meta5;
		};
		uint8_t noise;						/* 1 byte  */
	};
	struct SingleSummationPixelHeader
	{
		union
		{
			SummationSinglePixelMeta8Bits summation_pixel_meta; 	/* 1 byte  */
			SummationSinglePixelMeta8Bits4_5 summation_pixel_meta4_5;
		};
		uint8_t noise;						/* 1 byte  */
	};

	struct SinglePixelDynamic
	{
		SinglePixelMeta8Bits pixel_meta; 	/* 1 byte  */
		uint8_t noise;						/* 1 byte  */
		ReflectionAttributes reflection[1];

		size_t fixed_size() const { return (sizeof(SinglePixelMeta8Bits) + sizeof(uint8_t) + (pixel_meta.bits.n_reflections * sizeof(ReflectionAttributes))); }
	};


	struct SummationSinglePixelDynamic {
		SummationSinglePixelMeta8Bits summation_pixel_meta; 	/* 1 byte  */
		uint8_t noise;											/* 1 byte  */
		ReflectionAttributes reflection[1]; 

		size_t fixed_size() { return (sizeof(SummationSinglePixelMeta8Bits) + sizeof(uint8_t) + (summation_pixel_meta.bits.n_reflections * sizeof(ReflectionAttributes))); }
	};

	struct MacroPixelMetaData
	{
		MacroPixelMeta16Bits header;	/* 2 byte  */
		MemsFeedback mems_feedback;		/* 4 byte  */
		uint16_t blockage_pw;			/* 2 byte  */
	};

	struct MacroPixelDynamic
	{
		MacroPixelMetaData base;
		SinglePixelHeader channels[INVZ4_MACRO_PIXEL_CHANNEL_COUNT];
		ReflectionAttributes reflection[1];

		// INVZ4_6 temporary change
		static constexpr uint16_t active_channels = 8;

		size_t fixed_size() const
			{
			size_t total_size = sizeof(MacroPixelMetaData);
			size_t offset = 0;
			for (int i = 0; i < active_channels; i++)
			{
				offset += sizeof(SinglePixelHeader) + sizeof(ReflectionAttributes)*channels[i].pixel_meta.bits.n_reflections;
				if (channels[i].pixel_meta5.bits.lane_mark_trailer_available)
					offset += sizeof(INVZ2PixelLaneMarkTrailer);
			}
			total_size += offset;
			return total_size;
		}
	};
	struct MacroPixelDynamic4_3
	{
		MacroPixelMetaData base;
		SinglePixelDynamic channels[1];

		size_t fixed_size() const
		{
			size_t total_size = sizeof(MacroPixelMeta16Bits) + sizeof(MemsFeedback) + sizeof(uint16_t);
			size_t offset = 0;
			for (int i = 0; i < base.header.bits.active_channels + 1; i++)
			{
				uint8_t* current_channel_byte_ptr = (uint8_t*)(&channels[0]) + offset;
				SinglePixelDynamic* current_channel_ptr = (SinglePixelDynamic*)current_channel_byte_ptr;
				offset += current_channel_ptr->fixed_size();
			}
			total_size += offset;
			return total_size;
		}
	};

	struct SummationMacroPixelDynamic
	{
		SingleSummationPixelHeader channels[INVZ4_SUMMATION_CHANNEL_COUNT];
		ReflectionAttributes reflection[1];

		size_t fixed_size(size_t channels_num) const
		{
			size_t total_size = sizeof(SummationMacroPixelMeta16Bits) ;
			size_t offset = 0;
			for (size_t i = 0; i <= channels_num; i++)
			{
				uint8_t* current_channel_byte_ptr = (uint8_t*)(&channels[0]) + offset;
				SummationSinglePixelDynamic* current_channel_ptr = (SummationSinglePixelDynamic*)current_channel_byte_ptr;
				offset += current_channel_ptr->fixed_size();
			}
			total_size += offset;
			return total_size;
		}
	};

	struct SummationMacroPixelDynamic4_3
	{
		SummationMacroPixelMeta16Bits header;	/* 2 byte  */
		SummationSinglePixelDynamic channels[1];

		size_t fixed_size() const
		{
			size_t total_size = sizeof(SummationMacroPixelMeta16Bits);
			size_t offset = 0;
			for (int i = 0; i <= header.bits.active_channels; i++)
			{
				uint8_t* current_channel_byte_ptr = (uint8_t*)(&channels[0]) + offset;
				SummationSinglePixelDynamic* current_channel_ptr = (SummationSinglePixelDynamic*)current_channel_byte_ptr;
				offset += current_channel_ptr->fixed_size();
			}
			total_size += offset;
			return total_size;
		}
	};

	enum MeasurementsType
	{
		MEASUREMENTS_TYPE_0 = 0,
		MEASUREMENTS_TYPE_1 = 1,
		MEASUREMENTS_TYPE_OM = 2,
		MEASUREMENTS_TYPE_SUMMATION = 7
	};

	enum EGetParamPolicy : uint32_t
	{
		E_GET_PARAM_POLICY_AUTO = 0,
		E_GET_PARAM_POLICY_DIRECT_FORCED = 1
	};

	enum ECommProtocol : uint32_t
	{
		E_COMM_PROTOCOL_DEVICE_LINK,
		E_COMM_PROTOCOL_ACP,
		E_COMM_PROTOCOL_UNKNOWN = UINT32_MAX,
	};

	enum EFileFormat : uint32_t
	{
		E_FILE_FORMAT_INVZ2,
		E_FILE_FORMAT_INVZ3,
		E_FILE_FORMAT_INVZ4,
		E_FILE_FORMAT_INVZ4_4,
		E_FILE_FORMAT_INVZ4_5,
		E_FILE_FORMAT_INVZ4_6,
		E_FILE_FORMAT_INVZ4_7,
		E_FILE_FORMAT_INVZ5,
		E_FILE_FORMAT_UNKNOWN = UINT32_MAX,
	};

	enum ELoginLevel : uint8_t
	{
		E_LOGIN_LEVEL_USER = 0,
		E_LOGIN_LEVEL_TECHNICIAN = 1,
		E_LOGIN_LEVEL_FACTORY = 2,
		E_LOGIN_LEVEL_DEVELOPER = 3,
		E_LOGIN_LEVEL_ILLEGAL = UINT8_MAX,
	};


	enum BlockageClassifications : uint8_t
	{
		BLOCKAGE_NONE = 0,
		BLOCKAGE_MUD = 1,
		BLOCKAGE_RAIN = 2,
		BLOCKAGE_SNOW = 3,
		BLOCKAGE_OTHER = 4
	};

	enum FOVState : uint8_t
	{
		FOVSTATE_INIT = 0,
		FOVSTATE_CLEAR_VIEW = 1,
		FOVSTATE_RESTRICTED_VIEW = 2,
		FOVSTATE_ERROR = 3
	};

	enum FOVStateError : uint8_t
	{
		FOVSTATEERROR_INTERNAL = 0,
		FOVSTATEERROR_EXTERNAL = 1
	};

	struct EnvironmentalBlockage
	{
		uint32_t frame_number;
		uint8_t fov_state;
		uint8_t lidar_dq_data_not_valid;
		uint8_t reserved;
		uint8_t error_reason;
	};


	struct OMIndications
	{
		float main_board_temp;
		float detector_temp;
		float mems_ab_temp;
		float mems_cd_temp;
		float laser_board_temp;
		float maui_temp;
		float heater_windows_temp;
		float humidity;
	};

	struct memsPitchStatus
	{
		uint32_t frame_number;
		uint8_t mems_pitch_state;
		uint8_t mems_pitch_status;
		uint16_t time_left;
		int16_t pitch_current;
		int16_t pitch_target;
		int16_t pitch_max;
		int16_t pitch_min;
		uint32_t reserved;
	};

	enum CordinateSystem {
		CAR_CS = 0,
		LIDAR_CS
	};

	enum RefPoint {
		REF_POINT_FRONT_LEFT = 0,
		REF_POINT_FRONT_MIDDLE,
		REF_POINT_FRONT_RIGHT,
		REF_POINT_MIDDLE_RIGHT,
		REF_POINT_REAR_RIGHT,
		REF_POINT_REAR_MIDDLE,
		REF_POINT_REAR_LEFT,
		REF_POINT_MIDDLE_LEFT,
		REF_POINT_BUTTOM_CENTER
	};

	enum ClassesType {
		CLASS_TYPE_PASSENGER_CAR = 0,
		CLASS_TYPE_TRUCKS,
		CLASS_TYPE_MOTORCYCLE,
		CLASS_TYPE_PEDESTRIAN,
		CLASS_TYPE_BICYCLE,
		CLASS_TYPE_UNKNOWN_CLASS_TYPE,
		CLASS_TYPE_NUM_OF_CLASS_TYPES
	};

	enum Occlusion {
		OCCLUSION_VISIBLE = 0,
		OCCLUSION_PARTIAL, //partial visible
		OCCLUSION_OCCULDED //hidden
	};

	enum DIM_Names {
		DIM_WIDTH = 0,
		DIM_LENGTH,
		DIM_HEIGHT,
		NUM_OF_DIM_NAMES
	};

	struct top_view_2d_box {
		float x0;
		float y0;
		float x1;
		float y1;
	};

	struct Point3D {
		float x;
		float y;
		float z;
		uint32_t ValidBitmap;
	};

	struct Dimension {	//in CM up to 320M
		float width;
		float length;
		float height;
		uint32_t ValidBitmap;
	};

	struct DimOcclusion {
		Dimension Dim;
		Occlusion Occl[NUM_OF_DIM_NAMES];
	};

	struct AxisAngle {
		float  angle;
		float   angle_speed;
		uint32_t   ValidBitmap;
	};

	struct ClassProbability {
		float percentage[CLASS_TYPE_NUM_OF_CLASS_TYPES]; //in percentage from 0 to 100
	};

	struct CovarianceMatrix {
		float xx;  //TODO: TBD Amir might be uint16
		float yy;
		float xy;
	};

	/** @brief Defines a single object detection data
	*	@struct ObjectDetection
	*/
	struct INVZ_API ObjectDetection {
		uint32_t 				time_stamp;
		uint32_t                unique_id;
		CordinateSystem	        coord_system;
		RefPoint 		        ref_point_type;
		Point3D  				position;
		DimOcclusion 			dim_and_occlusion;
		Dimension				inner_dimensions;
		AxisAngle				axis_angle_params;
		ClassProbability		probability_of_classtype;
		float					existance_probability;
		CovarianceMatrix		position_cov_matrix;
		top_view_2d_box			box_2d;

	};

	struct DisplacementVector {
		float x; //TODO: Amir need to check units and check if int16 and granularity is enough.
		float y;
		float Z;
		uint32_t ValidBitmap;
	};


	/** @brief Defines an object detection header
	*	@struct MetaHeader
	*/
	struct MetaHeader {
		uint32_t	frameId;
		uint32_t	version;
		uint32_t	firstObject;
		uint32_t	lastObject;
		uint32_t	totalObjects;
	};

	struct StdTimestamp {
		uint32_t nanoseconds;
		uint32_t seconds;
		uint8_t sync_state; // enum StdTimestampSyncState
	};

	enum MeasurmentStatus {
		MEASURED = 0, //we see this object
		PREDICTED,  //we predict this object is there
		NEW_OBJECT  //new object identified
	};

	enum MovementStatus {
		STOPPED_OR_IN_MOTION = 0,
		STATIONARY
	};

	struct Sensor_Pose_Data {
		int frame_id;
		uint64_t timestamps_mili_sec;


		//sensor pose
		float sp_pitch_deg;
		float sp_roll_deg;
		float sp_z_cm;
		float sp_plane[3];
		float sp_mat3x3[9];
		int32_t n_of_inliers;
		float sp_fit_quality_0_to_1; //0 - bad, 1 - good
		float sp_yaw_deg;
		int16_t sp_x_cm;
		int16_t sp_y_cm;
		uint8_t reserved[4088];
	};

	/**
	* TrackedObject struct
	*/
	struct TrackedObject {                                //ID_2453
		uint32_t 					time_stamp; // TODO: TBD [32 bits or 64 bits}
		uint32_t                  unique_id;                  //ID_2458
		CordinateSystem	        coord_system; 		        // ID_2451
		RefPoint 		            ref_point_type;
		Point3D  		    position;                   //ID_3161
		DimOcclusion 	    dim_and_occlusion;          //ID_2470
		Dimension			inner_dimentions; 			// [AD]
		AxisAngle           axis_angle_params;

		DisplacementVector absulute_speed;             //TODO: Amir to check units[ID_2453, ID_3170]
		DisplacementVector relative_speed;
		DisplacementVector absolute_acceleration;      //Absulite acceleration
		DisplacementVector relative_acceleration;
		MeasurmentStatus          measurment_status;          //ID_2460, ID_3698, ID_3155, ID_3698, ID_4795
		MovementStatus            movement_status;            //ID_3923,ID_3924,ID_3925
		ClassProbability   probability_of_classtype;   //ID_2472 , sum should be 100
		float                   existance_probability;      //between 0 to 100

															  //add uncertainty: //TODO: open issue for Amir to close.
		CovarianceMatrix   position_cov_matrix;
		CovarianceMatrix   speed_cov_matrix;
		CovarianceMatrix   acceleration_cov_matrix;
	};

	enum PoseStatus {
		PoseStatus_DefaultValue = 0,
		PoseStatus_Misaligned = 1,
		PoseStatus_Outdated = 2,
		PoseStatus_OutdatedMisaligned = 3,
		PoseStatus_ConfirmedByQuickCheck = 4,
		PoseStatus_MisalignedByQuickCheck = 5,
		PoseStatus_Available = 6,
		PoseStatus_Error = 7,
		PoseStatus_INVALID = 255,
	};

	enum CalibrationStatus {
		CalibStatus_Init = 0,
		CalibStatus_Error = 1,
		CalibStatus_PausedVelocity = 2,
		CalibStatus_PausedSAV = 3,
		CalibStatus_PausedGround = 4,
		CalibStatus_PausedCleaning = 5,
		CalibStatus_Available = 6,
		CalibrationStatusEnum_INVALID = 255,
	};

	struct OCSensorPose
	{
		float yaw_deg; // current_pose_yaw
		float pitch_deg; // current_pose_pitch
		float roll_deg; // current_pose_roll
		float z_above_ground_cm; // current_pose_z
	};

	/** @brief Defines an Online Calibration output struct
	*	@struct OCOutput
	*/
	struct OCOutput
	{
		uint32_t is_oc_data_not_valid; // TRUE if oc_pose_status = PoseStatus_DefaultValue/PoseStatus_Outdated/PoseStatus_OutdatedMisaligned, else FALSE
		PoseStatus oc_pose_status;
		OCSensorPose oc_pose;
		CalibrationStatus oc_state;
		float oc_age_minutes; // new
		float oc_distance_meters; // new
		float oc_estimated_accuracy; // new
		uint32_t frame_id;
	};

	/** @brief Defines an Dynamic Calibration output struct
	*	@struct DCOutput
	*/
	struct DCOutput
	{
		int frame_id;
		float pitch;
		float roll;
		float z_cm;
	};

	/**
	* SegmentationLabel in PcPlus
	*/



	enum SegmentationLabel:uint32_t
	{
		Noise = 0, /**< Noise */
		Unrecognized = 1, /**< Unrecognized*/
		Ground = 2,/**< Ground*/
		Road = 3, /**< Road*/
		Obstacle = 4, /**< Obstacle*/
		UnderDrivable = 5, /**< UnderDrivable*/
		SignGantry = 6,/**< SignGanty*/
		NegativeObstacle = 7,/**< Negative Obstacle*/
	};


	struct PCPlus
	{
		uint32_t measurementIndex:19;
		uint32_t segmentation:3;
		uint32_t existenceProbability:10;

	};

	/* PC+ struct params are aligned to Magna structure*/
	/**
	* @brief PCPlusDetection data struct.
	* @struct PCPlusDetection
	*/
	struct PCPlusDetection
	{
		uint16_t	distance;						//Centimeter
		uint8_t		positive_predictive_value;		//EmPercent
		uint8_t		reflectivity;
		uint8_t		classification;					//DetectionClassificationLidar
		uint8_t		confidence;						//EmPercent
		int16_t		angle_azimuth;					//Coding_242308361
		int16_t		angle_elevation;				//Coding_242308378
	};

	struct INSSignalsStatus
	{
		int numOfValidVsInput;
		int frameId;
	};

	enum RBSide
	{
		RBD_LEFT_SIDE = 0,
		RBD_RIGHT_SIDE,
		RBD_SIDES_MAX
	};

	struct xyz_t
	{
		float x;
		float y;
		float z;
	};

	struct RoadsideRegionsDescriptor
	{
		xyz_t region[RBD_ROADSIDE_POINTS_NUM];
		uint8_t valid;
		uint8_t reserved[3];
	};

	struct  RBDescriptor
	{
		xyz_t p0;
		xyz_t p1;
		xyz_t p2;
		uint8_t valid ;
		uint8_t reserved[3];
	};

	//RBD is road boundary detection
	struct RBDOutput
	{
		RBDescriptor roadBoundaries[RBD_SIDES_MAX];
		RoadsideRegionsDescriptor roadsideRegions[RBD_SIDES_MAX];
		uint32_t frameId;
	};


	struct dimensions
	{
		float width;
		float length;
		float height;
	};

	enum box_3d_class {
		BOX_3D_TYPE_GENERAL = 0,
		BOX_3D_TYPE_SIGN_GANTRY = 1,
	};

	struct orientation {
		float roll; //[rad] 
		float pitch; //[rad]
		float yaw; //[rad]
	};


	struct SignGantryObject
	{
		dimensions dim; //[meters] 
		orientation angles; //[rad] 
		xyz_t center; //[meters] 
		float existence_probability; //positive predictive values in [0...1]
		float existence_ppv; //positive predictive values in [0...1]
		uint32_t age;
		uint32_t num_observations;
		uint32_t uid;
		box_3d_class box3d_class;
	};

	struct  StndTimestamp
	{
		uint32_t fractional_seconds;                //EmNanosecond
		uint32_t seconds;                           //EmSecond
		uint8_t sync_status;                        //StdTimestampSyncState
	};

	struct  CoordinateSystemOrigin
	{
		uint16_t invalid_flags;		//CoordinateSystemOrigin_invalid_flags
		float x;					//EmMeter
		float x_std_dev;			//EmMeter
		float y;					//EmMeter
		float y_std_dev;			//EmMeter
		float z;					//EmMeter
		float z_std_dev;			//EmMeter
		float roll;					//EmRadian
		float roll_std_dev;			//EmRadian
		float pitch;				//EmRadian
		float pitch_std_dev;		//EmRadian
		float yaw;					//EmRadian
		float yaw_std_dev;			//EmRadian
	};


	struct Pose
	{
		uint8_t poseStatus;
		uint32_t poseAge;          //EmSecond
		float poseDistance;		   //EmMeter
		CoordinateSystemOrigin origin;
	};


	struct OCOutputSI
	{
		uint8_t eventDataQualifier;
		StdTimestamp timestamp;
		uint8_t calibrationStatus;
		Pose pose;
		uint8_t lidarPowerMode;
		uint8_t integrityOCLidar;
	};

	struct FoVRegionLidar
	{
		uint8_t blockageClassification[8];
		uint8_t detectionQualityIndex;
	};

	struct FOVOutputSI
	{
		uint8_t eventDataQualifier;
		uint8_t extendedDataQualifier;
		StdTimestamp timestamp;
		FoVRegionLidar regions[100];
		uint8_t classificationUpdateFlag;
		uint8_t bwdStatus;
		uint8_t lidarPowerMode;
		uint8_t integrityFoVLidar;
	};

	struct DetectionListHeader
	{
		uint32_t	event_data_qualifier; //EventDataQualifier
		uint8_t		extended_qualifier;                                       //ExtendedQualifier
		StndTimestamp time_stamp;
		CoordinateSystemOrigin origin;
	};

	struct INVZ_API PCPlusMetaData
	{
		DetectionListHeader header;
		uint32_t number_of_detections;
		int32_t number_of_detections_low_res_left_origin;
		int32_t number_of_detections_low_res_right_origin;
		int32_t number_of_detections_high_res_left_origin;
		int32_t number_of_detections_high_res_right_origin;
		CoordinateSystemOrigin left_origin_in_sensor_origin;
		CoordinateSystemOrigin right_origin_in_sensor_origin;
	};


	struct INVZ_API PCPlusMetadata48k
	{
		DetectionListHeader header; 
		uint32_t number_of_detections;
		int32_t number_of_detections_roi_left_origin;
		int32_t number_of_detections_roi_right_origin;
		int32_t number_of_detections_outer_left_origin;
		CoordinateSystemOrigin left_origin_in_sensor_origin;
		CoordinateSystemOrigin right_origin_in_sensor_origin;
		uint8_t lidarPowerMode; 
		uint8_t integrityDetectionListLidar;
	};

	struct PCPlusServiceIf
	{
		DetectionListHeader header;
		uint32_t number_of_detections;
		PCPlusDetection detections[NUMBER_OF_PC_PLUS_DETECTION_POINT];
		int32_t number_of_detections_low_res_left_origin;
		int32_t number_of_detections_low_res_right_origin;
		int32_t number_of_detections_high_res_left_origin;
		int32_t number_of_detections_high_res_right_origin;
		CoordinateSystemOrigin left_origin_in_sensor_origin;
		CoordinateSystemOrigin right_origin_in_sensor_origin;
	};




	enum Gradient : uint8_t
	{
		GRADIENT_STABLE,
		GRADIENT_DECREASE,
		GRADIENT_INCREASE
	};


	struct BlockageDetectionSegment
	{
		uint8_t blocked;
		uint8_t coverage_percentage;
		uint8_t gradient;
		uint8_t reserved;
	};

	struct BlockageClassificationSegment
	{
		uint8_t classification;
	};

	struct GlareInFovDetectionSegment
	{
		uint8_t glared;
		uint8_t coverage_percentage;
		uint8_t glare_level;
		uint8_t reserved;
	};

	struct plane_edges {
		vector3   p0;
		vector3   p1;
		vector3   p2;
		vector3   p3;
	};

	struct plane {
		uint32_t        index;
		uint32_t        point_count;
		double          avg_error;
		double			d;
		vector3			u;
		vector3			ref_point;
		struct plane_edges		edges;
	};

	struct HeaterIndication
	{
		uint32_t internalTimestamp;
		uint32_t utcTimestampSec;
		uint32_t utcTimestampMSec;
		float omTemp;
		float heaterPower;
		uint16_t heaterEvents;
		uint8_t heaterState;
		uint8_t reserved;
	};

	struct LidarStatus {
		uint32_t system_mode; // (8 sys mode, 8 sub mode, 16 category)
		uint32_t num_ind_pc_info; // (8 num indications, 24 point cloud info)
		uint32_t error_code;
		uint32_t timestamp_sec;
		uint32_t timestamp_usec;
		float vbat;
		uint32_t indications[80 * 2]; // 80 indications,
		// each one has 2 u32 values (index and bitmap)
	};
	struct INVZ_API Register
	{
		std::string container; /*!< Group name of the parameter */
		std::string name;
		std::string description;
		uint32_t baseAddress;
		uint32_t offset;
		uint32_t width;
		uint32_t bitOffset;
		std::string access;
	};
	struct INVZ_API ChannelStatistics
	{
		uint8_t	 channel_id;
		uint32_t packets;
		uint32_t valid_packets;
		uint32_t missed_packets;
		uint64_t recieved_bytes;
		float data_rate;
	};

	// todo include invzbuf fw version bitmap (here is a copy past)
	union FWVersionBitmap {
		struct {
			uint32_t build : 16;
			uint32_t minor : 8;
			uint32_t major : 8;
		}bitmap;
		uint32_t value;
	};

#pragma pack(pop)

	struct FWVersion
	{
		uint32_t major;
		uint32_t minor;
		uint32_t build;

		FWVersion(uint32_t _major = 0, uint32_t _minor = 0, uint32_t _build = 0) : major{ _major }, minor{ _minor }, build{ _build }
		{

		}

		uint32_t get_value() const
		{
			uint32_t value = ((major & 0xFF) << 24) | ((minor & 0XFF) << 16) | (build & 0xFFFF);
			return value;
		}

		bool operator>=(const FWVersion& ver)
		{
			return (this->get_value() >= ver.get_value());
		}
	};

	class INVZ_API FilterAttrNode {
	public:
		explicit FilterAttrNode(const ndarray::FilterAttr& data, const FilterAttrNode* _next = nullptr) : data{ data.copy() }, next{ nullptr } {
			if (_next)
				next = new FilterAttrNode{ *_next };
		}

		explicit FilterAttrNode(const FilterAttrNode& other) : FilterAttrNode{ *other.data, other.next } {}

		~FilterAttrNode() {
			if (data)
				delete data;
		}

		const ndarray::FilterAttr *getData() const {
			return data;
		}

		bool hasNext() const {
			return next != nullptr;
		}

		FilterAttrNode *getNext() {
			return next;
		}

		const FilterAttrNode *getNext() const {
			return next;
		}

		void setNext(FilterAttrNode *_next) {
			next = _next;
		}

	private:
		const ndarray::FilterAttr *data = nullptr;
		FilterAttrNode *next = nullptr;
	};

	struct INVZ_API FrameDataAttributes {
		std::string typeName;
		uint32_t typeMajor;	/* TODO: Change to primary and secondary ID */
		uint32_t typeMinor;
		// Jule - for PC attributes define itemSize = 1
		uint32_t itemSize;			/* size of frame item (such as measurement struct, meta struct etc.., in bytes) */
		// Jule - for PC attributes define length as a number of bytes PC measurements require
		uint32_t length;				/* number of items */
		uint32_t optional;
		GrabType known_type;
		bool active;
		std::string dtype_format;
		FilterAttrNode *filterAttrs;

		FrameDataAttributes(std::string _typeName = "Unknown", uint32_t _typeMajor = UINT32_MAX,
			uint32_t _typeMinor = UINT32_MAX, uint32_t _itemSize = 0, uint32_t _length = 0,
			bool _optional = false, GrabType _knownType = GRAB_TYPE_UNKOWN,
			std::string _dtype_format = "") :
			dtype_format{ _dtype_format }, filterAttrs{ nullptr } {
			typeName = _typeName;
			typeMajor = _typeMajor;
			typeMinor = _typeMinor;
			itemSize = _itemSize;
			length = _length;
			optional = _optional;
			known_type = _knownType;
			active = false;
		}

		FrameDataAttributes(GrabType type) : FrameDataAttributes{} {
			known_type = type;
		}

		FrameDataAttributes(const FrameDataAttributes& other) :
			typeName{ other.typeName }, typeMajor{ other.typeMajor }, typeMinor{ other.typeMinor },
			itemSize{ other.itemSize }, length{ other.length }, optional{ other.optional },
			known_type{ other.known_type }, active{ other.active },
			dtype_format{ other.dtype_format }, filterAttrs{ nullptr } {
			if (other.filterAttrs)
				filterAttrs = new FilterAttrNode{ *other.filterAttrs };
		}

		FrameDataAttributes& operator=(const FrameDataAttributes& other) {
			dtype_format = other.dtype_format;
			filterAttrs = nullptr;
			typeName = other.typeName;
			typeMajor = other.typeMajor;
			typeMinor = other.typeMinor;
			itemSize = other.itemSize;
			length = other.length;
			optional = other.optional;
			known_type = other.known_type;
			active = other.active;
			if (other.filterAttrs)
				filterAttrs = new FilterAttrNode{ *other.filterAttrs };
			return *this;
		}

		size_t nbytes() const { return itemSize * length; };

		void AddFilter(const ndarray::FilterAttr& fa) {
			FilterAttrNode *filter_node = new FilterAttrNode{ fa };
			if (!filterAttrs) {
				filterAttrs = filter_node;
				return;
			}

			FilterAttrNode *filter_itr = filterAttrs;
			while (filter_itr->hasNext())
				filter_itr = filter_itr->getNext();
			filter_itr->setNext(filter_node);
		}

		void RemoveFilterType(const ndarray::EFilterType type) {
			FilterAttrNode *itr = filterAttrs;
			FilterAttrNode *previous = nullptr;
			while (itr) {
				FilterAttrNode *next = itr->getNext();
				if (itr->getData()->type == type) {
					delete itr;
					if (previous)
						previous->setNext(next);
					else
						filterAttrs = next;
				}
				else {
					previous = itr;
				}
				itr = next;
			}
		}

		void RemoveAllFilters() {
			FilterAttrNode *itr = filterAttrs;
			while (itr) {
				FilterAttrNode *next = itr->getNext();
				delete itr;
				itr = next;
			}
			filterAttrs = nullptr;
		}

		~FrameDataAttributes() {
			RemoveAllFilters();
		}
	};

	enum UserBufferStatus : uint16_t {
		USER_BUFFER_EMPTY = 0x0000,
		USER_BUFFER_PARTIAL,
		USER_BUFFER_FULL,
		USER_BUFFER_NONE,
		USER_BUFFER_ERROR
	};

	struct INVZ_API FrameDataUserBuffer {
		FrameDataAttributes dataAttrs;
		uint8_t* dataBuffer = nullptr;
		uint16_t status = USER_BUFFER_EMPTY;
		uint8_t handle_endianess;
        bool allocated = false;

        FrameDataUserBuffer() : allocated{false} {}

		FrameDataUserBuffer(const FrameDataUserBuffer& other) {
			dataBuffer = new uint8_t[other.dataAttrs.nbytes()];
            allocated = true;
			std::copy(other.dataBuffer, other.dataBuffer + other.dataAttrs.nbytes(), dataBuffer);
			dataAttrs = other.dataAttrs;
		}

		FrameDataUserBuffer(FrameDataAttributes attr) {
			dataBuffer = new uint8_t[attr.nbytes()];
            allocated = true;
			dataAttrs = attr;
		}

		~FrameDataUserBuffer() {
            if (dataBuffer && allocated)
                delete[] dataBuffer;
		}
	};
}
#endif /* __INVZ_TYPES_H__ */
