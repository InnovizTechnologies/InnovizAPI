///////////////////////////////////////////////////////////
//  PY_FileWriter.h
//  Implementation of the Class FileWriter
//  Created on:      08-Aug-2021 2:27:34 PM
//  Original author: sarah.foox
///////////////////////////////////////////////////////////

#ifndef __PY_FILE_WRITER_H__
#define __PY_FILE_WRITER_H__

#include "../frame_meta/PY_PCFrameMeta.h"
#include "interface/IWriterFactory.h"
#include "protocols/ACP.h"


class PY_FileWriter
{
public:
	PY_FileWriter(std::string file_name, uint32_t file_format);
	~PY_FileWriter();
	py::bool_ WriteFrame(PY_PCFrameMeta& frame_meta, py::array macro_pixels_frame, invz::EnvironmentalBlockage environmental_blockage, py::array blockage_detection, py::array blockage_classification);

private:
	std::unique_ptr<invz::IWriter> m_fw;
};
#endif // !defined __PY_FILE_WRITER_H__
