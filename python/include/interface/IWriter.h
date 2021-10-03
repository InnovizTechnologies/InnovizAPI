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

#ifndef __IWRITER_H__
#define __IWRITER_H__

/**
 * @file IWriter.h
 */

#include "../common_includes/invz_types.h"
#include "../common_includes/invz_status.h"
#include "../common_includes/invz_constants.h"

namespace invz {
	
	class INVZ_API IWriter
	{
	public:
		/* New fame meta */
		virtual Result DumpFrameToFile(	CSampleFrameMeta frame_meta, 
										MacroPixelFixed* macro_pixel_list, 
										size_t num_of_macro_pixels, 
										EnvironmentalBlockage* environmental_blockage = nullptr, 
										BlockageDetectionSegment* blockage_detection = nullptr,
										uint8_t* blockage_classifications =  nullptr, 
										size_t num_of_blockage_segments = 0) = 0;

	};




}

#endif /*__IWRITER_H__*/