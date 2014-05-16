/* Open Sensor Platform Project
 * https://github.com/sensorplatforms/open-sensor-platform
 *
 * Copyright (C) 2013 Sensor Platforms Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#if !defined (HOSTINTERFACE_H)
#define   HOSTINTERFACE_H

/*-------------------------------------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/

#define SH_WHO_AM_I                 0x54
#define SH_VERSION0                 0x01
#define SH_VERSION1                 0x22

#define osp_pack __attribute__ ((__packed__))

#define OSP_HOST_MAX_BROADCAST_BUFFER_SIZE 256

#include <stdint.h>

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
	struct Timestamp40_t endTime;	/* in NTTIME  */
	struct Timestamp40_t duration;	/* in NTTIME  */
	uint8_t type;
};

 
struct osp_pack sh_step_counter_sensor_node {
	struct Timestamp40_t timeStamp;	/* in ticks  */
    uint32_t numSteps;
};

struct osp_pack sh_step_detector_sensor_node {
	struct Timestamp40_t timeStamp;	/* in ticks  */
};


struct osp_pack sh_quaternion_data {
    /*
     * raw time stamp in sensor time capture ticks
     */
    struct Timestamp40_t timeStamp;           /* Time in NTTIME */
    int32_t W;	                 /* w/x/y/z Raw sensor data  in NTPRECISE*/
    int32_t X;  
    int32_t Y;	
    int32_t Z;	
} ;



 struct osp_pack sh_sensor_broadcast_node {
	uint8_t sensorId;	/* enum SensorType_t */
    uint8_t compression;
	union osp_pack {
		struct sh_motion_sensor_broadcast_node  sensorData;
        struct sh_motion_uncalibrated_sensor_broadcast_node uncalibratedSensordata;
		struct sh_segment_broadcast_node        segmentData;
        struct sh_quaternion_data               quaternionData;
        struct sh_step_counter_sensor_node      stepCounterData;
        struct sh_step_detector_sensor_node     stepDetectorData;
	} data;
};



struct osp_pack ShCmdGetHeader_get_8bits_param_t {
    uint8_t param;
} ;

struct osp_pack ShCmdGetHeader_get_16bits_param_t {
    uint16_t param;
} ;

struct osp_pack ShCmdGetEnableHeader_t {
    osp_sub_result_mask subResultMask;  /* optional param. Currently used for SENSOR_CONTEXT_DEVICE_MOTION, contains mask based on ContextMotionType_t */
    uint8_t enable;     /* 0 - to disable, !=0 to enable */
};


enum OSP_HOST_HUB_COMMANDS {
    OSP_HOST_GET_WHO_AM_I = 0x00,                 /* gets 8 bits Device ID */
    OSP_HOST_GET_VERSION,                         /* gets 16 bits version number on following read */
    OSP_HOST_RESET,
    
    /* there three commands most be gnerated */
    /* atomicly, in this sequence */
	OSP_HOST_GET_BROADCAST_LENGTH,            /* gets 16 bit of broadcast length */
	OSP_HOST_GET_BROADCAST_DATA,              /* gets as many bytes as broadcast length read */
    
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

#   if defined TRANSMIT_CAL_TO_SH

	OSP_HOST_SENSOR_SET_CALIBRATE,
	OSP_HOST_SENSOR_GET_CALIBRATE,
#   endif

} ;


struct osp_pack ShSensorCmdHeader_t {
    uint8_t command;	/* enum OSP_HOST_SENSOR_COMMANDS */
    uint8_t sensorId;	/* enum OSP_HOST_SENSOR_ID */
} ;


struct osp_pack ShSensorSetDelayCmdHeader_t {
    uint8_t command;	/* enum OSP_HOST_SENSOR_COMMANDS */
    uint8_t sensorId;	/* enum OSP_HOST_SENSOR_ID */
    uint16_t delay_milisec;
};

struct osp_pack ShSensorSetEnableCmdHeader_param_t {
    uint8_t command;	/* enum OSP_HOST_SENSOR_COMMANDS */
    uint8_t sensorId;	/* enum OSP_HOST_SENSOR_ID */
    osp_sub_result_mask subResultMask;  /* optional param. Currently used for SENSOR_CONTEXT_DEVICE_MOTION, contains mask based on ContextMotionType_t */
    uint8_t enable;     /* 0 - to disable, !=0 to enable */
};

union osp_pack ShCmdHeaderUnion{
    struct ShSensorCmdHeader_t command;
    struct ShSensorSetEnableCmdHeader_param_t enable;
    struct ShSensorSetDelayCmdHeader_t delay;
    struct ShHubCmdHeader_t hubCmdHeader;
    struct ShHubCmdHeader_8bits_param_t hub_cmd_8bits_param;
} ;




/*-------------------------------------------------------------------------------------------------*\
 |    T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
/* Register Area for slave device */
typedef struct SH_RegArea_tag
{
    uint8_t whoami;
    uint16_t version;
    struct  ShCmdGetEnableHeader_t enable;
    struct ShCmdGetHeader_get_16bits_param_t delay;
} SH_RegArea_t;







/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/


#endif /* HOSTINTERFACE_H */
/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/