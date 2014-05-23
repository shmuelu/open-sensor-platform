/* Open Sensor Platform Project
* https://github.com/sensorplatforms/open-sensor-platform
*
* Copyright (C) 2013 Sensor Platforms Inc.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*	  http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/
#if !defined (_OSP_HOSTINTERFACE_H_)
#define   _OSP_HOSTINTERFACE_H_

/*----------------------------------------------------------------------------*\
|	I N C L U D E   F I L E S
\*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*\
|	C O N S T A N T S   &   M A C R O S
\*----------------------------------------------------------------------------*/
#ifdef __KERNEL__
#   define OSP_SH_SUSPEND_DELAY 100		/* msec suspend delay*/
#endif


#define SH_WHO_AM_I				 0x54
#define SH_VERSION0				 0x01
#define SH_VERSION1				 0x22

#define osp_pack __attribute__ ((__packed__))

#define OSP_HOST_MAX_BROADCAST_BUFFER_SIZE 256

#ifndef __KERNEL__
#include <stdint.h>
#endif
#include "osp-types.h"

#ifdef __KERNEL__
#include "linux/osp-sensors.h"
#else
#include "osp-sensors.h"
#endif

struct osp_pack Timestamp40_t {
	uint32_t timeStamp32;
	uint8_t  timeStamp40;
};

struct osp_pack sh_motion_sensor_broadcast_node {
	/*
	* raw time stamp in sensor time capture ticks
	*/
	struct Timestamp40_t timeStamp;
	int16_t Data[3];	/* Raw sensor data */
};

struct osp_pack sh_motion_uncalibrated_sensor_broadcast_node {
	/*
	* raw time stamp in sensor time capture ticks
	*/
	struct Timestamp40_t timeStamp;
	int16_t Data[3];	/* Raw sensor data */
	int16_t Offset[3];	/* Raw sensor offset */
};

struct osp_pack sh_segment_broadcast_node {
	int64_t endTime;	/* in NTTIME  */
	int32_t duration;	/* in NTDELTATIME  */
	uint8_t type;
};


struct osp_pack sh_step_counter_sensor_node {
	/*
	* raw time stamp in sensor time capture ticks
	*/
	struct Timestamp40_t timeStamp;
	uint32_t numSteps;
};

struct osp_pack sh_step_detector_sensor_node {
	/*
	* raw time stamp in sensor time capture ticks
	*/
	struct Timestamp40_t timeStamp;
};

struct osp_pack sh_significant_motion_sensor_node {
	uint8_t sensorId;	/* Holds Sensor type enumeration - MUST be 1st*/
	/*
	* raw time stamp in sensor time capture ticks
	*/
	struct Timestamp40_t timeStamp;
	unsigned char  significantMotionDetected;	/* bool */

};

struct osp_pack sh_quaternion_data {
	/*
	* raw time stamp in sensor time capture ticks
	*/
	struct Timestamp40_t timeStamp;
	int32_t W;					 /* w/x/y/z/e Raw sensor data  in NTPRECISE*/
	int32_t X;
	int32_t Y;
	int32_t Z;
	int32_t E_EST;
} ;

struct osp_pack sh_orientation_broadcast_node {
	/*
	* raw time stamp in sensor time capture ticks
	*/
	struct Timestamp40_t timeStamp;
	int32_t Data[3];	/* Raw sensor data in NTEXTENDED*/
};


struct osp_pack sh_sensor_broadcast_node {
	struct SensorId_t sensorId;	/* enum SensorType_t + sensor sub-type*/
	uint8_t compression;
	union osp_pack {
		struct sh_motion_sensor_broadcast_node			  sensorData;
		struct sh_motion_uncalibrated_sensor_broadcast_node uncalibratedSensordata;
		struct sh_segment_broadcast_node					segmentData;
		struct sh_quaternion_data						   quaternionData;
		struct sh_step_counter_sensor_node				  stepCounterData;
		struct sh_step_detector_sensor_node				 stepDetectorData;
		struct sh_significant_motion_sensor_node			significantMotionData;
		struct sh_orientation_broadcast_node				orientationData;
	} data;
};



struct ShCmdGetHeader_get_8bits_param_t {
	uint8_t param;
};

struct ShCmdGetHeader_get_16bits_param_t {
	uint16_t param;
};

struct osp_pack ShCmdGetEnableHeader_t {
	osp_bool_t enable;	 /* FALSE - to disable, TRUE ( != FALSE) to enable */
};


enum OSP_HOST_HUB_COMMANDS {
	 /* gets 8 bits Device ID */
	OSP_HOST_GET_WHO_AM_I = 0x00,
	/* gets 16 bits version number on following read */
	OSP_HOST_GET_VERSION,
	OSP_HOST_RESET,

	/* there three commands most be gnerated */
	/* atomically, in this sequence */

	/* gets 16 bit of broadcast length */
	OSP_HOST_GET_BROADCAST_LENGTH,
	/* gets as many bytes as broadcast length read */
	OSP_HOST_GET_BROADCAST_DATA,
} ;


struct osp_pack ShHubCmdHeader_t {
	uint8_t command;	/* enum OSP_HOST_HUB_COMMANDS */
};

struct osp_pack ShHubCmdHeader_8bits_param_t {
	uint8_t command;	/* enum OSP_HOST_HUB_COMMANDS */
	uint8_t param;
} ;


enum OSP_HOST_SENSOR_COMMANDS {
	OSP_HOST_SENSOR_SET_ENABLE = 0x20,
	OSP_HOST_SENSOR_GET_ENABLE,

	OSP_HOST_SENSOR_SET_DELAY,
	OSP_HOST_SENSOR_GET_DELAY,

#if defined TRANSMIT_CAL_TO_SH

	OSP_HOST_SENSOR_SET_CALIBRATE,
	OSP_HOST_SENSOR_GET_CALIBRATE,

#endif
} ;

struct osp_pack ShSensorCmdHeader_t {
	uint8_t command;		/* enum OSP_HOST_SENSOR_COMMANDS */
	struct SensorId_t sensorId;		/* enum SensorType_t + sensor sub-type */
};


struct osp_pack ShSensorSetDelayCmdHeader_t {
	uint8_t command;		/* enum OSP_HOST_SENSOR_COMMANDS */
	struct SensorId_t sensorId;		/* enum SensorType_t + sensor sub-type */
	uint16_t delay_milisec;
};

struct osp_pack ShSensorSetEnableCmdHeader_param_t {
	uint8_t command;		/* enum OSP_HOST_SENSOR_COMMANDS */
	struct SensorId_t sensorId;	   /* enum OSP_HOST_SENSOR_ID */
	osp_bool_t enable;		 /* FALSE - to disable, TRUE (!= FALSE) to enable */
};

union osp_pack ShCmdHeaderUnion{
	struct ShSensorCmdHeader_t command;
	struct ShSensorSetEnableCmdHeader_param_t enable;
	struct ShSensorSetDelayCmdHeader_t delay;
	struct ShHubCmdHeader_t hubCmdHeader;
	struct ShHubCmdHeader_8bits_param_t hub_cmd_8bits_param;
} ;


#endif /* _OSP_HOSTINTERFACE_H_ */
/*-------------------------------------------------------------------------------------------------*\
|	E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/

