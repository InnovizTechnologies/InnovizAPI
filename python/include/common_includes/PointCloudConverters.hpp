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

#ifndef __POINT_CLOUD_CONVERTER_HPP__
#define __POINT_CLOUD_CONVERTER_HPP__
#define CS_IGNORE

#include "invz_status.h"
#include "invz_types.h"
#include <vector>

/**
 *  @file PointCloudConverters.hpp
*/

using namespace invz;
CS_IGNORE const int num_of_ch = 8;

CS_IGNORE const uint32_t MEASURMENTS_TYPE =			0xFFFF00;
CS_IGNORE const uint32_t DIRECTIONS_TYPE =			0xFFFF01;
CS_IGNORE const uint32_t SUMMATION_MEASURMENTS_TYPE = 0xFFFF02;
CS_IGNORE const uint32_t SUMMATION_DIRECTIONS_TYPE =	0xFFFF03;
CS_IGNORE const uint32_t DISTANCE_TAP_TYPE =			0xFFFF04;
CS_IGNORE const uint32_t REFLECTION_TAP_TYPE =		0xFFFF05;
CS_IGNORE const uint32_t CONFIDENCE_TAP_TYPE =		0xFFFF06;
CS_IGNORE const uint32_t NOISE_TAP_TYPE =				0xFFFF07;

struct LRFTaps
{
	bool active = false;
	uint16_t *distance = nullptr;
	uint8_t *reflectivity = nullptr;
	uint8_t *noise = nullptr;
	uint8_t *confidence = nullptr;
};


static Result PointCloudStreamToMacroPixelsFrame(DeviceMeta device_meta, INVZ4PixelsStream* pixel_stream, MacroPixelFixed* macro_pixels_frame, SummationMacroPixelFixed* summation_macro_pixels_frame)
{
	RESULT_OBSOLETE
}

static Result PointCloudStreamToSinglePixelsFrame(DeviceMeta device_meta, INVZ4PixelsStream* pixel_stream, DetailedSinglePixel* pixels_frame, DetailedSummationSinglePixel* summation_pixels_frame)
{
	RESULT_START



	RESULT_END
}

static void computeSinCos(float x, float& sin, float& cos)
{
	float x2 = x * x;
	float tail = (1 - (x2)) * (1 + (x2 * x2));
	sin = (2 * x) * tail;
	cos = (1 - x2) * tail;
}

static int matrix_multiplication_index[][6] = {
	{0, 0, 1, 3, 2, 6},
	{3, 0, 4, 3, 5, 6},
	{6, 0, 7, 3, 8, 6},
	{0, 1, 1, 4, 2, 7},
	{3, 1, 4, 4, 5, 7},
	{6, 1, 7, 4, 8, 7},
	{0, 2, 1, 5, 2, 8},
	{3, 2, 4, 5, 5, 8},
	{6, 2, 7, 5, 8, 8}
};

static int vector_multiplication_index[][2] = {
	{0, 0},
	{0, 1},
	{0, 2},
	{1, 0},
	{1, 1},
	{1, 2},
	{2, 0},
	{2, 1},
	{2, 2}
};

CS_IGNORE static float I[] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };

CS_IGNORE static float pow_2_18 = 0.000003814697265625;

/* Convert macro pixel's MemsFeedback to Direction(x,y,z) */
static Result MemsFeedbackToDirections(DeviceMeta& device_meta, uint8_t lrf, MemsFeedback& mems_feedback, vector3* directions, size_t directions_length)
{
	RESULT_START
#ifdef INVZ4_PIXEL_STREAM

	if (directions_length < 8)
	{
		throw std::runtime_error{ "directions length is too small - should be at least 8" };
	}

	float X = (mems_feedback.theta) * pow_2_18;
	float Y = (mems_feedback.phi) * pow_2_18;
	
	float sin_theta = 0.0;
	float sin_phi = 0.0;
	float cos_theta = 0.0;
	float cos_phi = 0.0;

	/* Compute sin and cos */
	computeSinCos(X, sin_theta, cos_theta);
	computeSinCos(Y, sin_phi, cos_phi);
	
	/* Compute uik */
	float u[3]{ 0 };
	u[0] = cos_theta * cos_phi;
	u[1] = (-sin_theta) * cos_phi;
	u[2] = sin_phi;

	/* Take relevant device */
	float* R = device_meta.Ri[lrf].matrix;
	vector3 d = device_meta.di[lrf];
	ChannelNormal vk = device_meta.vik[lrf];

	float u2[9]{ 0.0 };
	float I_u2[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	
	/* Compute I - 2(u*ut) */
	for (int i = 0; i < 9; i++)
	{
		u2[i] = 2 * u[vector_multiplication_index[i][0]] * u[vector_multiplication_index[i][1]];
		I_u2[i] = I[i] - u2[i];
	}

	/* Compute Q */
	float Q[9]{ 0 };
	for (int i = 0; i < 9; i++)
	{
		Q[i] = 
			(R[matrix_multiplication_index[i][0]] * I_u2[matrix_multiplication_index[i][1]]) + 
			(R[matrix_multiplication_index[i][2]] * I_u2[matrix_multiplication_index[i][3]]) + 
			(R[matrix_multiplication_index[i][4]] * I_u2[matrix_multiplication_index[i][5]]);	
	}

	/* Compute directions */
	float distance = 0;
	for (int i = 0; i < 8; i++)
	{
		directions[i].x = (Q[0] * -vk.channels[i].x) + (Q[3] * -vk.channels[i].y) + (Q[6] * -vk.channels[i].z);
		directions[i].y = (Q[1] * -vk.channels[i].x) + (Q[4] * -vk.channels[i].y) + (Q[7] * -vk.channels[i].z);
		directions[i].z = (Q[2] * -vk.channels[i].x) + (Q[5] * -vk.channels[i].y) + (Q[8] * -vk.channels[i].z);
	}
#endif // INVZ4_PIXEL_STREAM
	RESULT_END
}

static Result PointCloudStreamToMeasurments(DeviceMeta device_meta, uint8_t* pixel_stream, FrameDataUserBuffer* frame_data_buffers, uint32_t frame_data_count, bool verbose = false)
{
	RESULT_OBSOLETE
}



#endif /*__POINT_CLOUD_CONVERTER_HPP__*/
