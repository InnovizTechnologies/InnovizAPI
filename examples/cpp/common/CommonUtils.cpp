// Copyright 2021 Innoviz Technologies
//
// Licensed under the Innoviz Open Dataset License Agreement (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://github.com/InnovizTechnologies/InnovizAPI/blob/main/LICENSE.md
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

///////////////////////////////////////////////////////////
//  CommonUtils.cpp
//  Implementation of class CommonUtils
//  Created on:      07-Oct-2020 1:13:25 AM
//  Original author: julia.sher
///////////////////////////////////////////////////////////

#include "CommonUtils.h"
#include <iostream>


CommonUtils::CommonUtils()
{
	
}



CommonUtils::~CommonUtils() 
{

}



bool CommonUtils::objectExists(const invz::ObjectDetection& object)
{
	bool exists = false;
	for (int i = invz::CLASS_TYPE_PASSENGER_CAR; i < invz::CLASS_TYPE_NUM_OF_CLASS_TYPES; ++i)
	{
		if (object.probability_of_classtype.percentage[i] > 0)
		{
			exists = true;
		}
	}
	return exists;
}


invz::ClassesType CommonUtils::objectClass(invz::ClassProbability probability_of_classtype)
{
	invz::ClassesType object_class = invz::ClassesType::CLASS_TYPE_UNKNOWN_CLASS_TYPE;

	for (int i = invz::CLASS_TYPE_PASSENGER_CAR; i < invz::CLASS_TYPE_NUM_OF_CLASS_TYPES; ++i)
	{
		if (probability_of_classtype.percentage[i] > 0)
		{
			object_class = static_cast<invz::ClassesType>(i);
			break;
		}
	}
	return object_class;
}


std::string CommonUtils::getObjectClassStr(invz::ClassProbability probability_of_classtype)
{
	invz::ClassesType res_class = objectClass(probability_of_classtype);
	std::string res_class_str;

	switch (res_class)
	{
	case invz::ClassesType::CLASS_TYPE_PASSENGER_CAR:
		res_class_str = "Passenger car";
		break;
	case invz::ClassesType::CLASS_TYPE_TRUCKS:
		res_class_str = "Truck";
		break;
	case invz::ClassesType::CLASS_TYPE_MOTORCYCLE:
		res_class_str = "Motorcycle";
		break;
	case invz::ClassesType::CLASS_TYPE_PEDESTRIAN:
		res_class_str = "Pedestrian";
		break;
	case invz::ClassesType::CLASS_TYPE_BICYCLE:
		res_class_str = "Bibycle";
		break;
	case invz::ClassesType::CLASS_TYPE_UNKNOWN_CLASS_TYPE:
	default:
		res_class_str = "Unknown";
		break;
	}
	return res_class_str;
}


void CommonUtils::HandleObjectsData(invz::FrameDataUserBuffer& ref_buffer)
{
	auto detections = reinterpret_cast<invz::ObjectDetection*>(ref_buffer.dataBuffer);
	std::cout << std::endl;
	std::cout << "Reading object detections ..." << std::endl;
	for (uint32_t i = 0; i < ref_buffer.dataAttrs.length; ++i)
	{
		invz::ObjectDetection object = detections[i];
		if (objectExists(object))
		{
			std::cout << "\tObject " << i << " data:" << std::endl;
			std::cout << "\t\tid: " << object.unique_id << std::endl;
			std::cout << "\t\tclassification: " << getObjectClassStr(object.probability_of_classtype) << std::endl;
			std::cout << "\t\texistance probability: " << object.existance_probability << std::endl;
			std::cout << "\t\tdimensions: " << "width " << object.dim_and_occlusion.Dim.width << ", length " << object.dim_and_occlusion.Dim.length
				<< ", height " << object.dim_and_occlusion.Dim.height << std::endl;
			std::cout << "\t\tposition: " << "x " << object.position.x << ", y " << object.position.y << ", z " << object.position.z << std::endl;
		}
	}
}


void CommonUtils::HandleReflection0Data(invz::FrameDataUserBuffer& ref_buffer)
{
	auto measurements = reinterpret_cast<invz::INVZ2MeasurementXYZType*>(ref_buffer.dataBuffer);
	for (uint32_t i = 0; i < ref_buffer.dataAttrs.length; ++i)
	{
		//blooming pixels can be detected by comparin gto invz::PixelValidity::PIXEL_BLOOMING
		if (measurements[i].validity == invz::PixelValidity::PIXEL_VALIDITY_VALID)
		{
			//print pixel data
			int distance = measurements[i].distance;
			int reflectivity = measurements[i].reflectivity;

			std::cout << "\tPixel " << i << " measurement data:" << std::endl;
			std::cout << "\t\t" << "distance: " << distance << std::endl;
			std::cout << "\t\t" << "reflectivity: " << reflectivity << std::endl;
			break;
		}
	}
}
