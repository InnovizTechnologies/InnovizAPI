///////////////////////////////////////////////////////////
//  PY_FileWriter.cpp
//  Implementation of the Class FileWriter
//  Created on:      08-Aug-2021 2:27:34 PM
//  Original author: sarah.foox
///////////////////////////////////////////////////////////

#include "PY_FileWriter.h"

PY_FileWriter::PY_FileWriter(std::string file_name, uint32_t file_format)
{
	std::string defaultIp("0.0.0.0");
	std::vector<std::pair<uint32_t, uint16_t>> virtualChannelToPort;
	virtualChannelToPort.push_back({ 0,0 });
	virtualChannelToPort.push_back({ 1,1 });
	std::vector<invz::FrameDataAttributes> frameDataAttr = {
		{ "FrameMeta",					invz::POINT_CLOUD_CSAMPLE_METADATA,			UINT32_MAX,					sizeof(invz::CSampleFrameMeta),						1},
		{ "INVZ4Stream",				invz::POINT_CLOUD_INVZ4_PIXELS,				UINT32_MAX,					1,												1024 * 1024 * 5}, // 5 MB for maximal stream of INVZ4 pixels
		{ "EndOfFrame",					invz::POINT_CLOUD_END_OF_FRAME,				UINT32_MAX,					sizeof(invz::EndOfFrame),								1},
		{ "EnvironmentalBlockage",		invz::POINT_CLOUD_BLOCKAGE_ENVIRONMENTAL,		UINT32_MAX,					sizeof(invz::EnvironmentalBlockage),					1},
		{ "BlockageDetection",			invz::POINT_CLOUD_BLOCKAGE_DETECTION,			0,							sizeof(invz::BlockageDetectionSegment),				100},
		{ "BlockageClassification",		invz::POINT_CLOUD_BLOCKAGE_CLASSIFICATION,	0,							sizeof(invz::BlockageClassificationSegment),			100},
	};
	m_fw.reset(invz::FileWriterInit(file_name, defaultIp, file_format, virtualChannelToPort, frameDataAttr));
}

PY_FileWriter::~PY_FileWriter()
{
	if (m_fw)
		invz::FileWriterClose(m_fw.release());
}

py::bool_ PY_FileWriter::WriteFrame(PY_PCFrameMeta& frame_meta, py::array macro_pixels_frame, invz::EnvironmentalBlockage environmental_blockage, py::array blockage_detection, py::array blockage_classification)
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

	invz::Result res = m_fw->DumpFrameToFile(frame_meta.GetFrameMeta(), (invz::MacroPixelFixed*)macro_pixels_frame.request().ptr, macro_pixels_frame.size(), &environmental_blockage,
		blockageDetection, blockageClassification, blockageSegmentCount);

	return res.error_code == invz::ErrorCode::ERROR_CODE_OK;
}
