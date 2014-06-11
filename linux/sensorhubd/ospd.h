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
#ifndef _OSPD_H_
#define _OSPD_H_

/*-------------------------------------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
\*-------------------------------------------------------------------------------------------------*/

#include <stdint.h>
#include "osp-api.h"
#include "virtualsensordevicemanager.h"

/*-------------------------------------------------------------------------------------------------*\
 |    C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

typedef union {
    float f;
    int32_t i;
} OspValue_t;

typedef union {
    double d;
    int64_t ll;
} OspTimeValue_t;

typedef struct {
    OspTimeValue_t timestamp;
    OspValue_t data[3];
} OSPD_CalibratedThreeAxisData_t;

typedef struct {
    OspTimeValue_t timestamp;
    OspValue_t data[6];      // X/Y/Z/offsetX/offsetY/offsetZ/
} OSPD_UncalibratedThreeAxisData_t;

typedef struct {
    OspTimeValue_t timestamp;
} OSPD_StepDetectorData_t;

typedef struct {
    OspTimeValue_t timestamp;
    OspValue_t numSteps;
} OSPD_StepCounterData_t;

typedef struct {
    OspTimeValue_t timestamp;
    OspValue_t significantMotionDetected;
} OSPD_SignificantMotionData_t;







typedef void (*OSPD_ResultDataCallback_t)(const struct SensorId_t *sensorId, void *data);

/****************************************************************************************************
 * @fn      getSensordeviceDriverName
 *          Returns the the string pointer to driver name for the record for the specified sensorId.
 * @param   sensorId - specifies sensorId for result
 * @return  returns a pointer to to driver name if sensorId record is in _ospResultCodes, otherwise returns NULL
 *
 ***************************************************************************************************/
const char *OSPD_getSensordeviceDriverName(const struct SensorId_t *sensorId);

/****************************************************************************************************
 * @fn      OSPD_getResultIndex
 *          Returns an index into _ospResultCodes for the record for the specified sensorId.
 * @param   sensorId - specifies sensorId for result
 * @return  returns an index got sensorId record in _ospResultCodes, otherwise returns -1
 *
 ***************************************************************************************************/
int16_t OSPD_getResultIndex(const struct SensorId_t *sensorId);

/****************************************************************************************************
 * @fn      OSPD_getResultDataReadyCallbackFunction
 *          Returns the data ready call back function pointer for the record for the specified sensorId.
 * @param   sensorId - specifies sensorId for result
 * @return  returns an call back function pointer if sensorId record is in _ospResultCodes, otherwise returns NULL
 *
 ***************************************************************************************************/
OSPD_ResultDataCallback_t OSPD_getResultDataReadyCallbackFunction(const struct SensorId_t *sensorId);

/****************************************************************************************************
 * @fn      getSensorSwapTable
 *          Returns the the address of the SWAP table for the record for the specified sensorId.
 * @param   sensorId - specifies sensorId for result
 * @return  returns a pointer to SWAP table if sensorId record is in _ospResultCodes, otherwise returns NULL
 *
 ***************************************************************************************************/
int *OSPD_getSensorSwapTable(const struct SensorId_t *sensorId);

/****************************************************************************************************
 * @fn      getSensorConversionTable
 *          Returns the the address of the SWAP table for the record for the specified sensorId.
 * @param   sensorId - specifies sensorId for result
 * @return  returns a pointer to SWAP table if sensorId record is in _ospResultCodes, otherwise returns NULL
 *
 ***************************************************************************************************/
osp_float_t *OSPD_getSensorConversionTable(const struct SensorId_t *sensorId);

/****************************************************************************************************
 * @fn      OSPD_Initialize
 *          Initialize remote procedure call for the daemon
 *
 ***************************************************************************************************/
osp_status_t OSPD_Initialize(void);

/****************************************************************************************************
 * @fn      OSPD_Deinitialize
 *          Tear down RPC interface function
 *
 ***************************************************************************************************/
osp_status_t OSPD_Deinitialize(void);

/****************************************************************************************************
 * @fn      initializeSensors
 *          initialized all sensors in _ospResultCodes.
 *
 ***************************************************************************************************/
void OSPD_initializeSensors(VirtualSensorDeviceManager *vsDevMgr);

/****************************************************************************************************
 * @fn      stopAllSensors
 *          Unsubscribe/stop data flow for all sensors in _ospResultCodes
 *
 ***************************************************************************************************/
void OSPD_stopAllSensors(void);

/****************************************************************************************************
 * @fn      parseAndHandleSensorControls
 *          Helper routine for enabling/disabling sensors in the system
 *
 ***************************************************************************************************/
void OSPD_parseAndHandleSensorControls(char *buffer, ssize_t numBytesInBuffer);

/****************************************************************************************************
 * @fn      OSPD_SubscribeResult
 *          Enables subscription for results
 *
 ***************************************************************************************************/
osp_status_t OSPD_SubscribeResult(const struct SensorId_t *sensorId);

/****************************************************************************************************
 * @fn      OSPD_UnsubscribeResult
 *          Unsubscribe from sensor results
 *
 ***************************************************************************************************/
osp_status_t OSPD_UnsubscribeResult(const struct SensorId_t *sensorId);


#endif /* _OSPD_H_ */
