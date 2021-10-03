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

#pragma once

#ifndef VB_INVZBUF_INVZBUF_0_0_0
#define VB_INVZBUF_INVZBUF_0_0_0

#include <stdint.h>

/**
 *  @file VB.hpp
*/

#ifdef INVZBUF_EXPORT
#if defined(_WIN32)
#define INVZBUF_API __declspec(dllexport)
#elif __GNUC__ >= 4
#define INVZBUF_API __attribute__ ((visibility("default")))
#define __stdcall 
#else
#error Unsupported platform
#endif
#else
#ifdef INVZBUF_IMPORT
#if defined(_WIN32)
#define INVZBUF_API _declspec(dllimport)
#elif __GNUC__ >= 4
#define INVZBUF_API 
#define __stdcall 
#else
#error Unsupported platform
#endif
#else
#define INVZBUF_API
#endif
#endif

#pragma pack(push, 1)

namespace vb_invzbuf
{

	inline void endianess_switch(void* data, size_t len){
		char* ptr = (char*)data;
		
		for (size_t i=0; i<len/2; i++){
			// https://www.geeksforgeeks.org/swap-two-numbers-without-using-temporary-variable/
			// swap using xor without 3rd variable
		    ptr[i] = ptr[i] ^ ptr[len-1-i];        // x = x ^ y, x now becomes 15 (1111) 
			ptr[len-1-i] = ptr[i] ^ ptr[len-1-i];  // y = x ^ y, // y becomes 10 (1010) 
			ptr[i] = ptr[i] ^ ptr[len-1-i]; 	   // x = x ^ y, // x becomes 5 (0101) 
		}
		
	}

// numpy pybind11 decleration macros
#define VB_INVZBUF_PYBIND11_DTYPE_DECLERATIONS \
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
    PYBIND11_NUMPY_DTYPE(vb_invzbuf::DetectionLidar, distance, positivePredictiveValue, reflectivity, classification, confidence, angleAzimuth, angleElevation);

    /*
     * The estimated classification of a detection.
     */
    enum EDetectionClassificationLid{
        E_KDETECTIONCLASSIFICATIONLIDAR_NOCLASSIFICATION,
        E_KDETECTIONCLASSIFICATIONLIDAR_OBSTACLENEGATIVE,
        E_KDETECTIONCLASSIFICATIONLIDAR_CAR,
        E_KDETECTIONCLASSIFICATIONLIDAR_TRUCK,
        E_KDETECTIONCLASSIFICATIONLIDAR_MOTORCYCLE,
        E_KDETECTIONCLASSIFICATIONLIDAR_BICYCLE,
        E_KDETECTIONCLASSIFICATIONLIDAR_PEDESTRIAN,
        E_KDETECTIONCLASSIFICATIONLIDAR_ANIMAL,
        E_KDETECTIONCLASSIFICATIONLIDAR_TRAVERSABLEUNDER,
        E_KDETECTIONCLASSIFICATIONLIDAR_NOISE,
        E_KDETECTIONCLASSIFICATIONLIDAR_GROUND,
        E_KDETECTIONCLASSIFICATIONLIDAR_RAINFOGSMOKESPRAY,
        E_KDETECTIONCLASSIFICATIONLIDAR_SIGNGANTRY,
        E_KDETECTIONCLASSIFICATIONLIDAR_INVALID = 255,
    };

    /*
     * The reference point on the object model which currently defines the position of the object.
     */
    enum EReferencePoint{
        E_CORNER_FRONT_LEFT,
        E_MIDDLE_FRONT,
        E_CORNER_FRONT_RIGHT,
        E_MIDDLE_SIDE_RIGHT,
        E_CORNER_REAR_RIGHT,
        E_MIDDLE_REAR,
        E_CORNER_REAR_LEFT,
        E_MIDDLE_SIDE_LEFT,
        E_SIGNAL_UNFILLED = 255,
    };

    enum EMeasurementStatus{
        E_KMEASUREMENTSTATUS_MEASURED,
        E_KMEASUREMENTSTATUS_NEW	,
        E_KMEASUREMENTSTATUS_PREDICTED,
        E_KMEASUREMENTSTATUS_INVALID = 255,
    };

    enum EMovementStatus{
        E_KMOVEMENTSTATUS_MOVED,
        E_KMOVEMENTSTATUS_STATIONARY,
        E_KMOVEMENTSTATUS_INVALID = 255,
    };

    enum EEdgeVisibilityStatus{
        E_KEDGEVISIBILITYSTATUS_COMPLETELYVISIBLE,
        E_KEDGEVISIBILITYSTATUS_PARTIALLYOCCLUDED,
        E_KEDGEVISIBILITYSTATUS_COMPLETELYOCCLUDED,
        E_KEDGEVISIBILITYSTATUS_INVALID = 255,
    };

    /*
     * Status of the brake light of an object
     */
    enum EBrakeLightStatus{
        E_KBRAKELIGHTSTATUS_UNKNOWN,
        E_KBRAKELIGHTSTATUS_OFF,
        E_KBRAKELIGHTSTATUS_ONREGULAR,
        E_KBRAKELIGHTSTATUS_ONHEAVY,
        E_KBRAKELIGHTSTATUS_INVALID = 255,
    };

    /*
     * Status of the flash light of an object
     */
    enum EFlashLightStatus{
        E_KFLASHLIGHTSTATUS_UNKNOWN,
        E_KFLASHLIGHTSTATUS_NONE,
        E_KFLASHLIGHTSTATUS_LEFT,
        E_KFLASHLIGHTSTATUS_RIGHT,
        E_KFLASHLIGHTSTATUS_BOTHHAZARD,
        E_KFLASHLIGHTSTATUS_INVALID = 255,
    };

    /*
     * Status of the headlight of an object
     */
    enum EHeadLightStatus{
        E_KHEADLIGHTSTATUS_UNKNOWN,
        E_KHEADLIGHTSTATUS_OFF,
        E_KHEADLIGHTSTATUS_ON,
        E_KHEADLIGHTSTATUS_INVALID = 255,
    };

    /*
     * Status of the rear light of an object
     */
    enum ERearLightStatus{
        E_KREARLIGHTSTATUS_UNKNOWN,
        E_KREARLIGHTSTATUS_OFF,
        E_KREARLIGHTSTATUS_ON,
        E_KREARLIGHTSTATUS_INVALID = 255,
    };

    /*
     * Status of the reverse light of an object
     */
    enum EReverseLightStatus{
        E_KREVERSELIGHTSTATUS_UNKNOWN,
        E_KREVERSELIGHTSTATUS_OFF,
        E_KREVERSELIGHTSTATUS_ON,
        E_KREVERSELIGHTSTATUS_INVALID = 255,
    };

    /*
     * Status of the emergency light of an object
     */
    enum EEmergencyLightStatus{
        E_KEMERGENCYLIGHTSTATUS_UNKNOWN,
        E_KEMERGENCYLIGHTSTATUS_OFF,
        E_KEMERGENCYLIGHTSTATUS_ON_COLORUNKNOWN,
        E_KEMERGENCYLIGHTSTATUS_ON_RED,
        E_KEMERGENCYLIGHTSTATUS_ON_BLUE,
        E_KEMERGENCYLIGHTSTATUS_ON_YELLOW,
        E_KEMERGENCYLIGHTSTATUS_INVALID = 255,
    };

    /*
     * Object Summary.
     */
    struct ObjectSummary{
        /* bitmaps */
        /* fields */
        uint32_t id;
        uint16_t age;
        uint8_t statusMeasurement;  // enum field - EMeasurementStatus.
        uint8_t statusMovement;  // enum field - EMovementStatus.

        /* functions */
        void handle_endianess();
    };

    #ifndef INVZBUF_IMPORT
    
    /*
     * @description handle fields endianes, converts all fields to little endian.
     */
	inline void ObjectSummary::handle_endianess(){
    	// handle id endianness
    	endianess_switch(&id, sizeof(ObjectSummary::id));
    
    	// handle age endianness
    	endianess_switch(&age, sizeof(ObjectSummary::age));
    
    }
    
    #endif
    struct ObjectExistence{
        /* bitmaps */
        /* fields */
        uint8_t invalidFlags;  // invalid flags of this data structure.
        float existenceProbability;  // existence probability of the object [0..1].
        float existencePpv;  // [0..1].

        /* functions */
        void handle_endianess();
    };

    #ifndef INVZBUF_IMPORT
    
    /*
     * @description handle fields endianes, converts all fields to little endian.
     */
	inline void ObjectExistence::handle_endianess(){
    	// handle existenceProbability endianness
    	endianess_switch(&existenceProbability, sizeof(ObjectExistence::existenceProbability));
    
    	// handle existencePpv endianness
    	endianess_switch(&existencePpv, sizeof(ObjectExistence::existencePpv));
    
    }
    
    #endif
    struct SensorStatusFlags{
        /* bitmaps */
        /* fields */
        uint16_t sensorStatusFlags;

        /* functions */
        void handle_endianess();
    };

    #ifndef INVZBUF_IMPORT
    
    /*
     * @description handle fields endianes, converts all fields to little endian.
     */
	inline void SensorStatusFlags::handle_endianess(){
    	// handle sensorStatusFlags endianness
    	endianess_switch(&sensorStatusFlags, sizeof(SensorStatusFlags::sensorStatusFlags));
    
    }
    
    #endif
    /*
     * This data type represents a floatingpoint value.
     */
    struct EmMeter{
        /* bitmaps */
        /* fields */
        float value;

        /* functions */
        void handle_endianess();
    };

    #ifndef INVZBUF_IMPORT
    
    /*
     * @description handle fields endianes, converts all fields to little endian.
     */
	inline void EmMeter::handle_endianess(){
    	// handle value endianness
    	endianess_switch(&value, sizeof(EmMeter::value));
    
    }
    
    #endif
    /*
     * This data type represents a floatingpoint value.
     */
    struct EmCentimeter{
        /* bitmaps */
        /* fields */
        uint16_t value;

        /* functions */
        void handle_endianess();
    };

    #ifndef INVZBUF_IMPORT
    
    /*
     * @description handle fields endianes, converts all fields to little endian.
     */
	inline void EmCentimeter::handle_endianess(){
    	// handle value endianness
    	endianess_switch(&value, sizeof(EmCentimeter::value));
    
    }
    
    #endif
    /*
     * This data type represents a floatingpoint value containing meter_per_second.
     */
    struct EmMeterPerSecond{
        /* bitmaps */
        /* fields */
        float value;

        /* functions */
        void handle_endianess();
    };

    #ifndef INVZBUF_IMPORT
    
    /*
     * @description handle fields endianes, converts all fields to little endian.
     */
	inline void EmMeterPerSecond::handle_endianess(){
    	// handle value endianness
    	endianess_switch(&value, sizeof(EmMeterPerSecond::value));
    
    }
    
    #endif
    /*
     * This data type represents a floatingpoint value containing meter_per_second_squared.
     */
    struct EmMeterPerSecondSquared{
        /* bitmaps */
        /* fields */
        float value;

        /* functions */
        void handle_endianess();
    };

    #ifndef INVZBUF_IMPORT
    
    /*
     * @description handle fields endianes, converts all fields to little endian.
     */
	inline void EmMeterPerSecondSquared::handle_endianess(){
    	// handle value endianness
    	endianess_switch(&value, sizeof(EmMeterPerSecondSquared::value));
    
    }
    
    #endif
    /*
     * This data type represents a floatingpoint value.
     */
    struct EmRadian{
        /* bitmaps */
        /* fields */
        float value;

        /* functions */
        void handle_endianess();
    };

    #ifndef INVZBUF_IMPORT
    
    /*
     * @description handle fields endianes, converts all fields to little endian.
     */
	inline void EmRadian::handle_endianess(){
    	// handle value endianness
    	endianess_switch(&value, sizeof(EmRadian::value));
    
    }
    
    #endif
    /*
     * This data type represents a percentage in uint8.
     */
    struct EmPercent{
        /* bitmaps */
        /* fields */
        uint8_t value;

        /* functions */
        void handle_endianess();
    };

    #ifndef INVZBUF_IMPORT
    
    /*
     * @description handle fields endianes, converts all fields to little endian.
     */
	inline void EmPercent::handle_endianess(){
    }
    
    #endif
    struct ObjectPosition3d{
        /* bitmaps */
        /* fields */
        uint16_t invalidFlags;  // invalid flags of this data structure.
        uint8_t referencePoint;  // enum field - EReferencePoint, The reference point on the object model which currently defines the position of the object..
        EmMeter x;  // [m] Position of the reference point along the x-axis in a Cartesian coordinate system..
        EmMeter xStdDev;  // [m] Standard deviation of the position of the reference point along the x-axis in a Cartesian coordinate system..
        EmMeter y;  // [m] Position of the reference point along the y-axis in a Cartesian coordinate system..
        EmMeter yStdDev;  // [m] Standard deviation of the position of the reference point along the y-axis in a Cartesian coordinate system..
        EmMeter z;  //  [m] Position of the reference point along the z-axis in a Cartesian coordinate system..
        EmMeter zStdDev;  // [m] Standard deviation of the position of the reference point along the z-axis in a Cartesian coordinate system..
        float covarianceXy;  // [m^2] Covariance between estimation of the x and y position.
        EmRadian orientation;  // [rad] Orientation of the object bounding box in the ego-vehicle coordinate system..
        EmRadian orientationStdDev;  // [rad] Standard deviation of orientation of the object bounding box in the ego-vehicle coordinate system..

        /* functions */
        void handle_endianess();
    };

    #ifndef INVZBUF_IMPORT
    
    /*
     * @description handle fields endianes, converts all fields to little endian.
     */
	inline void ObjectPosition3d::handle_endianess(){
    	// handle invalidFlags endianness
    	endianess_switch(&invalidFlags, sizeof(ObjectPosition3d::invalidFlags));
    
    	// handle x endianness
    	x.handle_endianess();
    	
    	// handle xStdDev endianness
    	xStdDev.handle_endianess();
    	
    	// handle y endianness
    	y.handle_endianess();
    	
    	// handle yStdDev endianness
    	yStdDev.handle_endianess();
    	
    	// handle z endianness
    	z.handle_endianess();
    	
    	// handle zStdDev endianness
    	zStdDev.handle_endianess();
    	
    	// handle covarianceXy endianness
    	endianess_switch(&covarianceXy, sizeof(ObjectPosition3d::covarianceXy));
    
    	// handle orientation endianness
    	orientation.handle_endianess();
    	
    	// handle orientationStdDev endianness
    	orientationStdDev.handle_endianess();
    	
    }
    
    #endif
    /*
     * Velocity components of an object in the ego-vehicle Cartesian coordinate system..
     */
    struct ObjectVelocity{
        /* bitmaps */
        /* fields */
        uint8_t invalidFlags;  // Bitfield for communicating that float values are invalid.
        EmMeterPerSecond x;  // [m/s] Velocity of the object along the x-axis of the ego-vehicle coordinate system..
        EmMeterPerSecond xStdDev;  // [m/s] Standard Deviation of velocity of the object along the x-axis of the ego-vehicle coordinate system..
        EmMeterPerSecond y;  // [m/s] Velocity of the object along the y-axis of the ego-vehicle coordinate system..
        EmMeterPerSecond yStdDev;  // [m/s] Standard Deviation of velocity of the object along the y-axis of the ego-vehicle coordinate system..
        float covarianceXy;  // [m^2] Covariance between the x and y velocity components.

        /* functions */
        void handle_endianess();
    };

    #ifndef INVZBUF_IMPORT
    
    /*
     * @description handle fields endianes, converts all fields to little endian.
     */
	inline void ObjectVelocity::handle_endianess(){
    	// handle x endianness
    	x.handle_endianess();
    	
    	// handle xStdDev endianness
    	xStdDev.handle_endianess();
    	
    	// handle y endianness
    	y.handle_endianess();
    	
    	// handle yStdDev endianness
    	yStdDev.handle_endianess();
    	
    	// handle covarianceXy endianness
    	endianess_switch(&covarianceXy, sizeof(ObjectVelocity::covarianceXy));
    
    }
    
    #endif
    /*
     * Object acceleration information.
     */
    struct ObjectAcceleration{
        /* bitmaps */
        /* fields */
        uint8_t invalidFlags;  // Bitfield for communicating that float values are invalid.
        EmMeterPerSecondSquared x;  // [m/s^2] Acceleration of the object along the x-axis of the ego-vehicle coordinate system..
        EmMeterPerSecondSquared xStdDev;  // [m/s^2] Standard deviation of acceleration of the object along the x-axis of the ego-vehicle coordinate system..
        EmMeterPerSecondSquared y;  // [m/s^2] Acceleration of the object along the y-axis of the ego-vehicle coordinate system..
        EmMeterPerSecondSquared yStdDev;  // [m/s^2] Standard deviation of acceleration of the object along the y-axis of the ego-vehicle coordinate system..
        float covarianceXy;  // [m^2] Covariance between the x and y acceleration components.

        /* functions */
        void handle_endianess();
    };

    #ifndef INVZBUF_IMPORT
    
    /*
     * @description handle fields endianes, converts all fields to little endian.
     */
	inline void ObjectAcceleration::handle_endianess(){
    	// handle x endianness
    	x.handle_endianess();
    	
    	// handle xStdDev endianness
    	xStdDev.handle_endianess();
    	
    	// handle y endianness
    	y.handle_endianess();
    	
    	// handle yStdDev endianness
    	yStdDev.handle_endianess();
    	
    	// handle covarianceXy endianness
    	endianess_switch(&covarianceXy, sizeof(ObjectAcceleration::covarianceXy));
    
    }
    
    #endif
    /*
     * A value representing a normally distributed random number..
     */
    struct NormalDistributedValue{
        /* bitmaps */
        /* fields */
        uint8_t invalidFlags;  // The status of the normal distribution..
        float mean;  // The mean of the value..
        float stdDev;  // The standard deviation of the value..

        /* functions */
        void handle_endianess();
    };

    #ifndef INVZBUF_IMPORT
    
    /*
     * @description handle fields endianes, converts all fields to little endian.
     */
	inline void NormalDistributedValue::handle_endianess(){
    	// handle mean endianness
    	endianess_switch(&mean, sizeof(NormalDistributedValue::mean));
    
    	// handle stdDev endianness
    	endianess_switch(&stdDev, sizeof(NormalDistributedValue::stdDev));
    
    }
    
    #endif
    /*
     * Dynamic information of an object.
     */
    struct ObjectDynamics{
        /* bitmaps */
        /* fields */
        ObjectVelocity velocityAbsolute;  // Absolute velocity of an object.
        ObjectVelocity velocityRelative;  // Relative velocity of an object.
        ObjectAcceleration accelerationAbsolute;  // Absolute acceleration of an object.
        ObjectAcceleration accelerationRelative;  // Relative acceleration of an object.
        NormalDistributedValue orientationRate;  // [rad/s] Orientation rate of the object..

        /* functions */
        void handle_endianess();
    };

    #ifndef INVZBUF_IMPORT
    
    /*
     * @description handle fields endianes, converts all fields to little endian.
     */
	inline void ObjectDynamics::handle_endianess(){
    	// handle velocityAbsolute endianness
    	velocityAbsolute.handle_endianess();
    	
    	// handle velocityRelative endianness
    	velocityRelative.handle_endianess();
    	
    	// handle accelerationAbsolute endianness
    	accelerationAbsolute.handle_endianess();
    	
    	// handle accelerationRelative endianness
    	accelerationRelative.handle_endianess();
    	
    	// handle orientationRate endianness
    	orientationRate.handle_endianess();
    	
    }
    
    #endif
     struct ShapeEdge{
        /* bitmaps */
        /* fields */
        uint32_t status;  // enum field - EEdgeVisibilityStatus.
        NormalDistributedValue edge;

        /* functions */
        void handle_endianess();
    };

    #ifndef INVZBUF_IMPORT
    
    /*
     * @description handle fields endianes, converts all fields to little endian.
     */
	 inline void ShapeEdge::handle_endianess(){
    	// handle status endianness
    	endianess_switch(&status, sizeof(ShapeEdge::status));
    
    	// handle edge endianness
    	edge.handle_endianess();
    	
    }
    
    #endif
    struct ShapeBoundingBox3d{
        /* bitmaps */
        /* fields */
        ShapeEdge length;
        ShapeEdge width;
        ShapeEdge height;

        /* functions */
        void handle_endianess();
    };

    #ifndef INVZBUF_IMPORT
    
    /*
     * @description handle fields endianes, converts all fields to little endian.
     */
	inline void ShapeBoundingBox3d::handle_endianess(){
    	// handle length endianness
    	length.handle_endianess();
    	
    	// handle width endianness
    	width.handle_endianess();
    	
    	// handle height endianness
    	height.handle_endianess();
    	
    }
    
    #endif
    /*
     * Object type classification.
     */
    struct ObjectClassification{
        /* bitmaps */
        /* fields */
        EmPercent classCar;  // [0,100] Percentage that the object is of class type 'car'..
        EmPercent classTruck;  // [0,100] Percentage that the object is of class type 'truck'..
        EmPercent classMotorcycle;  // [0,100] Percentage that the object is of class type 'motorcycle'..
        EmPercent classBicycle;  // [0,100] Percentage that the object is of class type 'bicycle'..
        EmPercent classPedestrian;  // [0,100] Percentage that the object is of class type 'pedestrian'..
        EmPercent classAnimal;  // [0,100] Percentage that the object is of class type 'animal'..
        EmPercent classHazard;  // [0,100] Percentage that the object is of class type 'Hazard'..
        EmPercent classUnknown;  // [0,100] Percentage that the object is of class type 'unknown'..
        EmPercent classOverdrivable;  // [0,100] Percentage that the object is overdrivable..
        EmPercent classUnderdrivable;  // [0,100] Percentage that the object is underdrivable.

        /* functions */
        void handle_endianess();
    };

    #ifndef INVZBUF_IMPORT
    
    /*
     * @description handle fields endianes, converts all fields to little endian.
     */
	inline void ObjectClassification::handle_endianess(){
    }
    
    #endif
    struct ObjectLights{
        /* bitmaps */
        /* fields */
        uint8_t statusFlashLight;  // enum field - EFlashLightStatus.
        uint8_t statusBrakeLight;  // enum field - EBrakeLightStatus.
        uint8_t statusHeadLight;  // enum field - EHeadLightStatus.
        uint8_t statusRearLight;  // enum field - ERearLightStatus.
        uint8_t statusReverseLight;  // enum field - EReverseLightStatus.
        uint16_t statusEmergencyLight;  // enum field - EEmergencyLightStatus.

        /* functions */
        void handle_endianess();
    };

    #ifndef INVZBUF_IMPORT
    
    /*
     * @description handle fields endianes, converts all fields to little endian.
     */
	inline void ObjectLights::handle_endianess(){
    	// handle statusEmergencyLight endianness
    	endianess_switch(&statusEmergencyLight, sizeof(ObjectLights::statusEmergencyLight));
    
    }
    
    #endif
    /*
     * Representation of a single object in the environment model..
     */
    struct ObjectPodLidar{
        /* bitmaps */
        /* fields */
        ObjectSummary summary;
        ObjectExistence existence;
        SensorStatusFlags statusSensors;
        ObjectPosition3d position;
        ObjectDynamics dynamics;
        ShapeBoundingBox3d shape3d;
        ObjectClassification classification;
        ObjectLights lights;

        /* functions */
        void handle_endianess();
    };

    #ifndef INVZBUF_IMPORT
    
    /*
     * @description handle fields endianes, converts all fields to little endian.
     */
	inline  void ObjectPodLidar::handle_endianess(){
    	// handle summary endianness
    	summary.handle_endianess();
    	
    	// handle existence endianness
    	existence.handle_endianess();
    	
    	// handle statusSensors endianness
    	statusSensors.handle_endianess();
    	
    	// handle position endianness
    	position.handle_endianess();
    	
    	// handle dynamics endianness
    	dynamics.handle_endianess();
    	
    	// handle shape3d endianness
    	shape3d.handle_endianess();
    	
    	// handle lights endianness
    	lights.handle_endianess();
    	
    }
    
    #endif
    /*
     * Representation of a single detection.
     */
    struct DetectionLidar{
        /* bitmaps */
        /* fields */
        EmCentimeter distance;  // The distance in [cm] from origin to detection..
        EmPercent positivePredictiveValue;  // The existence probability in the range [0, 100] of the detection given as positive predictive value..
        uint8_t reflectivity;  // The estimated reflectivity is derived by the amount of the reflected energy normalized with the distance to the target..
        uint8_t classification;  // enum field - EDetectionClassificationLid, The classification of the detection..
        EmPercent confidence;  // The estimated confidence is a measure on how likely there is a detection at the given location..
        int16_t angleAzimuth;  // The azimuth angle in the sensor coordinate system..
        int16_t angleElevation;  // The elevation angle in the sensor coordinate system..

        /* functions */
        void handle_endianess();
    };

    #ifndef INVZBUF_IMPORT
    
    /*
     * @description handle fields endianes, converts all fields to little endian.
     */
	inline void DetectionLidar::handle_endianess(){
    	// handle distance endianness
    	distance.handle_endianess();
    	
    	// handle angleAzimuth endianness
    	endianess_switch(&angleAzimuth, sizeof(DetectionLidar::angleAzimuth));
    
    	// handle angleElevation endianness
    	endianess_switch(&angleElevation, sizeof(DetectionLidar::angleElevation));
    
    }
    
    #endif
}

#pragma pack(pop)

#endif // VB_INVZBUF_INVZBUF_0_0_0
