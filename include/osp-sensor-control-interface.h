/* Open Sensor Platform Project
* https://github.com/sensorplatforms/open-sensor-platform
*
* Copyright (C) 2013 Sensor Platforms Inc.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/
#if !defined (_OSP_SENSOR_CONTROL_INTERFACE_H_)
#define   _OSP_SENSOR_CONTROL_INTERFACE_H_

/*----------------------------------------------------------------------------*\
|   I N C L U D E   F I L E S
\*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*\
|   C O N S T A N T S   &   M A C R O S
\*----------------------------------------------------------------------------*/


#define osp_pack __attribute__ ((__packed__))


#ifndef __KERNEL__
# include <stdint.h>
#endif
#include "osp-types.h"

#ifdef __KERNEL__
# include "linux/osp-sensors.h"
#else
# include "osp-sensors.h"
#endif



struct ShCmdGetHeaderGetDelayHeader_t {
    uint16_t delay;
};

struct osp_pack ShCmdGetEnableHeader_t {
    osp_bool_t enable;   /* FALSE - to disable, TRUE ( != FALSE) to enable */
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
};


struct osp_pack ShHubCmdHeader_t {
    uint8_t command;    /* enum OSP_HOST_HUB_COMMANDS */
};



enum OSP_HOST_SENSOR_COMMANDS {
    OSP_HOST_SENSOR_SET_ENABLE = 0x20,
    OSP_HOST_SENSOR_GET_ENABLE,

    OSP_HOST_SENSOR_SET_DELAY,
    OSP_HOST_SENSOR_GET_DELAY,

#if defined TRANSMIT_CAL_TO_SH

    OSP_HOST_SENSOR_SET_CALIBRATE,
    OSP_HOST_SENSOR_GET_CALIBRATE,

#endif
};

struct osp_pack ShSensorCmdHeader_t {
    uint8_t command;        /* enum OSP_HOST_SENSOR_COMMANDS */
    struct SensorId_t sensorId;     /* enum SensorType_t + sensor sub-type */
};


struct osp_pack ShSensorSetDelayCmdHeader_t {
    uint8_t command;        /* enum OSP_HOST_SENSOR_COMMANDS */
    struct SensorId_t sensorId;     /* enum SensorType_t + sensor sub-type */
    uint16_t delay_milisec;
};

struct osp_pack ShSensorSetEnableCmdHeader_param_t {
    uint8_t command;        /* enum OSP_HOST_SENSOR_COMMANDS */
    struct SensorId_t sensorId;    /* enum OSP_HOST_SENSOR_ID */
    osp_bool_t enable;       /* FALSE - to disable, TRUE (!= FALSE) to enable */
};

union osp_pack ShCmdHeaderUnion {
    struct ShSensorCmdHeader_t command;
    struct ShSensorSetEnableCmdHeader_param_t enable;
    struct ShSensorSetDelayCmdHeader_t delay;
    struct ShHubCmdHeader_t hubCmdHeader;
};


#endif /* _OSP_SENSOR_CONTROL_INTERFACE_H_ */
/*-------------------------------------------------------------------------------------------------*\
|   E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
