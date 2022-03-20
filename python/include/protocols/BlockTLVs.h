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

#ifndef __BLOCK_TLV_H__
#define __BLOCK_TLV_H__

/**
 * @file BlockTLVs.h
 */

#include "common_includes/invz_types.h"
#include "common_includes/invz_status.h"
#include "common_includes/invz_constants.h"
#include "protocols/ACP.h"

using namespace vb_invzbuf;

namespace invz
{
#pragma pack(push, 1)

	struct BlockTLV
	{
		uint32_t type;
		uint16_t length;
		uint16_t reserved;
	};
	struct StandardTLV
	{
		BlockTLV header;
		uint32_t frame_number;
		uint32_t timestamp;
		uint8_t data[];

	};
	struct INVZ4PixelsTLV {
		BlockTLV header;
		uint32_t frame_number;
		uint8_t lrf_number;
		uint8_t summation_type; /* As described in SummationTypeEnum */
		uint8_t reserved0[2];
		uint32_t first_macro_pixel_index;
		uint32_t last_macro_pixel_index;
		uint32_t total_macro_pixels_in_lrf;
		uint32_t reserved; 
		uint32_t macro_pixel_groups_count; 
		uint16_t lut_size; /* BMW project */
		uint8_t reserved1[6];
		uint16_t lut[LUT_STRUCT_SIZE + 1]{ 0 };
		uint8_t pixels[];
	};

	struct CSampleFrameMetaDataTLV
	{
		BlockTLV header;
		CSampleFrameMeta meta;

		CSampleFrameMetaDataTLV(uint32_t _frame_number, uint8_t _scan_mode, uint8_t _system_mode, uint8_t _system_submode, uint32_t _timestamp_internal,
			uint32_t _timestamp_utc_sec, uint32_t _timestamp_utc_micro, uint32_t _fw_version, uint32_t _hw_version, uint8_t* p_lidar_serial,
			uint16_t _device_type, uint8_t _active_lrfs, uint8_t _macro_pixel_shape, uint8_t* _rows_in_lrf, uint16_t* _cols_in_lrf, uint32_t _total_number_of_points,
			ReflectionMatrix* _R_i, vector3* _d_i, ChannelNormal* _v_i_k, float* _alpha_calib_table, float* _beta_calib_table)
		{
			memset(this, 0, sizeof(CSampleFrameMetaDataTLV));
			header.type = POINT_CLOUD_CSAMPLE_METADATA;
			header.length = sizeof(CSampleFrameMetaDataTLV) - sizeof(BlockTLV);
			meta.frame_number = _frame_number;
			meta.scan_mode = _scan_mode;
			meta.system_mode = _system_mode;
			meta.system_submode = _system_submode;
			meta.timestamp_internal = _timestamp_internal;
			meta.timestamp_utc_secs = _timestamp_utc_sec;
			meta.timestamp_utc_micro = _timestamp_utc_micro;
			meta.fw_version = _fw_version;
			meta.hw_version = _hw_version;
			memcpy(meta.lidar_serial_number, p_lidar_serial, INVZ4_CSAMPLE_LIDAR_SERIAL_LENGTH);
			meta.device_type = _device_type;
			meta.active_lrfs = _active_lrfs;
			meta.macro_pixel_shape = _macro_pixel_shape; /* (4bits,4bits) */
			memcpy(meta.rows_in_lrf,_rows_in_lrf, DEVICE_NUM_OF_LRFS * sizeof(uint8_t));
			memcpy(meta.cols_in_lrf, _cols_in_lrf, DEVICE_NUM_OF_LRFS * sizeof(uint16_t));
			meta.total_number_of_points = _total_number_of_points;
			memcpy(meta.R_i, _R_i, DEVICE_NUM_OF_LRFS * sizeof(ReflectionMatrix));
			memcpy(meta.d_i, _d_i, DEVICE_NUM_OF_LRFS * sizeof(vector3));
			memcpy(meta.v_i_k, _v_i_k, DEVICE_NUM_OF_LRFS * sizeof(ChannelNormal));
			memcpy(meta.alpha_calib_table, _alpha_calib_table, META_CALIB_TABLE_SIZE*(sizeof(float)));
			memcpy(meta.beta_calib_table, _beta_calib_table, META_CALIB_TABLE_SIZE*(sizeof(float)));
		};
	};

	struct LidarStatusTLV
	{
		BlockTLV header;
		uint32_t frame_number;
		LidarStatus status;
	};
	
	/* Jule */
	struct EofTLV
	{
		BlockTLV header;
		uint32_t frame_number;
		uint32_t reserved;

		EofTLV(uint32_t _frame_number = 0)
		{
			memset(this, 0, sizeof(EofTLV));
			header.type = POINT_CLOUD_END_OF_FRAME;
			header.length = sizeof(EofTLV) - sizeof(BlockTLV);
			frame_number = _frame_number;
		}
	};



	struct MetadataTLV : BlockTLV
	{
		uint32_t	frameNumber;
		uint8_t		scanMode;
		uint8_t		reserved;
		uint16_t	frameEvent;
		uint32_t	timestamp;
		uint32_t	timestampUTCSecs;
		uint32_t	timestampUTCMicroSecs;
	};

	
	struct EnvironmentalBlockageTLV
	{
		BlockTLV header;
		uint32_t frameNumber;
		uint8_t fov_state;
		uint8_t lidar_dq_data_not_valid;
		uint8_t reserved;
		uint8_t error_reason;
	};
	

	struct BlockageDetectionTLV
	{
		BlockTLV header;
		uint32_t	frameNumber;
		uint16_t	dataType;
		uint16_t	reserved;
		uint32_t	firstBlockageSegmentIndex;
		uint32_t	lastBlockageSegmentIndex;
		uint32_t	totalBlockageSegments;
		BlockageDetectionSegment	blockageDetectionSegments[];
	};


	struct BlockageClassificationTLV
	{
		BlockTLV header;
		uint32_t	frameNumber;
		uint16_t	dataType;
		uint16_t	reserved;
		uint32_t	firstBlockageSegmentIndex;
		uint32_t	lastBlockageSegmentIndex;
		uint32_t	totalBlockageSegments;
		BlockageClassificationSegment	blockageClassificationSegments[];
	};

	
	struct OMIndicationsTLV
	{
		BlockTLV header;
		uint32_t version;
		uint32_t frame_number;
		OMIndications om_indications;
	};

	struct memsPitchStatusTLV
	{
		BlockTLV header;
		memsPitchStatus mems_pitch_status;
	};


	struct GlareInFovDetectionTLV
	{
		BlockTLV	header;
		uint32_t	frameNumber;
		uint16_t	dataType;
		uint16_t	reserved;
		uint32_t	firstGlareSegmentIndex;
		uint32_t	lastGlareSegmentIndex;
		uint32_t	totalGlareSegments;
		GlareInFovDetectionSegment	glareDetectionSegments[];
	};


	struct MemsFeedbackTLV : BlockTLV
	{
		uint32_t	frameNumber;
		uint16_t	feedbackDataType;
		uint16_t	reserved;
		uint32_t	firstFeedbackIndex;
		uint32_t	lastFeedbackIndex;
		uint8_t		feedback[];
	};

	struct EndOfFrameTLV : BlockTLV
	{
		uint32_t frameNumber;
		uint32_t reserved;
	};

	struct FrameStartTLV
	{
		BlockTLV header;
		uint32_t frameNumber = 0;
		uint32_t internalTimestamp = 0;
		uint32_t reseved = 0;
	};

	enum ReflectionType
	{
		REFLECTION_FIRST =  1,
		REFLECTION_SECOND = 2,
		REFLECTION_THIRD =  3,
	};

	struct FileHeader
	{
		uint32_t marker;
		uint8_t majorVersion;
		uint8_t minorVersion;
		uint16_t reserved;
	};

	struct FileHeaderTLV : BlockTLV
	{
		FileHeader hedaer;
	};

	struct Directions
	{
		byte rasterIndex;							/*!< The Index of the raster this block describes */
		byte padding[3];
		uint32_t totalDirections;					/*!< Total number of directions for this raster type (should be the same as the measurements count per frame) */
		uint32_t firstDirection;					/*!< First direction index in this block */
		uint32_t lastDirection;						/*!< Last direction index in this block */
		vector3 directions[];						/*!< The directions data. Each direction is composed of 3 values (each is a 4 bytes float) for x , y , z vector coordinates */


	};

	struct DirectionsTLV : BlockTLV
	{
		Directions directions;
	};

	struct Timings
	{
		byte rasterIndex;							/*!< The Index of the raster this block describes */
		byte padding[3];
		uint32_t totalTimings;					/*!< Total number of directions for this raster type (should be the same as the measurements count per frame) */
		uint32_t firstTiming;					/*!< First direction index in this block */
		uint32_t lastTiming;						/*!< Last direction index in this block */
		uint32_t timings[];						/*!< The directions data. Each direction is composed of 3 values (each is a 4 bytes float) for x , y , z vector coordinates */

	};

	struct TimingsTLV : BlockTLV
	{
		Timings timings;
	};	

	struct ObjectDetectionTLV : BlockTLV
	{
		MetaHeader header;
		ObjectDetection objects[];
	};

	struct TrackedObjectsTLV : BlockTLV
	{
		MetaHeader header;
		TrackedObject objects[];
	};

	struct PCPlusTLV : BlockTLV
	{
		MetaHeader header;
		PCPlus pixels[];
	};
	struct PCPlusSITLV : BlockTLV
	{
		MetaHeader header;
		PCPlusDetection pixels[];
	};
	struct PCPlusSIMetaTLV : BlockTLV
	{
		MetaHeader header;
		PCPlusMetaData metadata;
	};
	struct PCPlusSIMeta48KTLV : BlockTLV
	{
		MetaHeader header;
		PCPlusMetadata48k metadata;
	};
	struct ObjectDetectionSITLV : BlockTLV
	{
		MetaHeader header;
		uint32_t eventDataQualifier; // enum EventDataQualifier
		uint8_t extendedQualifier; // enum ObjectListExtendedQualifier
		StdTimestamp timestamp;
		uint8_t numberOfObjects;
		ObjectPodLidar objects[];
	};

	struct SensorPoseTLV : BlockTLV
	{
		Sensor_Pose_Data data;
	};

	struct LandmarkTLV : BlockTLV
	{
		MetaHeader	header;
		plane planeLensmarks[];
	};

	struct OCOutputTLV
	{
		BlockTLV header;
		OCOutput oc_output;
	};

	struct DCOutputTLV
	{
		BlockTLV header;
		DCOutput dc_output;
	};

	struct INSSignalsTLV
	{
		BlockTLV header;
		INSSignalsStatus INS_signals;
	};

	struct RBDOutputTLV
	{
		BlockTLV header;
		RBDOutput RBD_output;
	};

	struct SignGantryOutputTLV:BlockTLV
	{
		MetaHeader header;
		SignGantryObject gantries[];
	};
		

	struct OCOutputSITLV : BlockTLV
	{
		MetaHeader header;
		OCOutputSI oc_output;
	};

	struct FOVOutputSITLV : BlockTLV
	{
		MetaHeader header;
		FOVOutputSI FOV_output;
	};

	struct RunTimeLogTLV
	{
		static constexpr uint32_t padding_size{ 96 };
		static constexpr uint32_t data_size{ 16256 };

		BlockTLV header;
		uint32_t frameNumber;
		uint32_t logType;
		uint8_t padding[padding_size];
		uint8_t data[data_size];
	};

	struct GetPartial
	{
		uint32_t id =0;
		uint32_t size =0;
		uint32_t offset =0;
		uint8_t data[0];
	};


	struct LogInfo
	{
		uint32_t level;
		uint32_t timestamp;
		uint8_t message[0];
	};
#pragma pack(pop)

	struct TapPartial
	{
		uint32_t id = UINT32_MAX;
		uint32_t offset = 0;
		uint32_t frameNumber = UINT32_MAX;
		uint32_t dataLength = 0;
		uint32_t ui_cookie = UINT32_MAX;
		uint32_t horizontal_dilution = 0;
		uint32_t vertical_dilution = 0;
		uint8_t *data;

		TapPartial(TapPartial& other)
		{
			id = other.id;
			offset = other.offset;
			frameNumber = other.frameNumber;
			dataLength = other.dataLength;
			ui_cookie = other.ui_cookie;
			horizontal_dilution = other.horizontal_dilution;
			vertical_dilution = other.vertical_dilution;
			data = new uint8_t[dataLength];
			memcpy(data, other.data, dataLength);
		};

		TapPartial(TapPartial&& other)
		{
			id = other.id;
			offset = other.offset;
			frameNumber = other.frameNumber;
			dataLength = other.dataLength;
			ui_cookie = other.ui_cookie;
			horizontal_dilution = other.horizontal_dilution;
			vertical_dilution = other.vertical_dilution;
			data = new uint8_t[dataLength];
			memcpy(data, other.data, dataLength);
		};

		TapPartial(uint32_t _length, uint8_t *_data = nullptr)
		{
			dataLength = _length;
			data = new uint8_t[dataLength];
			if(_data)
				memcpy(data, _data, dataLength);
		};

        

		~TapPartial()
		{
			if (data)
			{
				delete[] data;
				data = nullptr;
			}
		}
	};

}

#endif /*__BLOCK_TLV_H__*/