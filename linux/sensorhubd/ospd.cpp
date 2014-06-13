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
/*-------------------------------------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
\*-------------------------------------------------------------------------------------------------*/
#include <climits>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <csignal>
#include <string>

#include <unistd.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <assert.h>

#include "osp_configuration.h"
#include "osp-sensors.h"
#include "osp_debuglogging.h"
#include "osp-sensor-control-interface.h"
#include "osp_relayinterface.h"
#include "ospd.h"


/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/
#define MAX_NUM_FDS(x, y)                ((x) > (y) ? (x) : (y))
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))


/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

typedef struct OspSensorAttributes_t {
    struct SensorId_t sensorId;

    bool subscribed;
    const char          *configFileDeviceName;
    const char          *deviceDriverName;
    OSPD_ResultDataCallback_t dataReadyCallBackFunction;
    int evdevFd;
    std::string sysDelayPath;  //Sysfs path for setting polling interval
    std::string sysEnablePath; //Sysfs path for enable
    char enableValue;   //Value that enables the device (typically 1)*
    char disableValue;  //Value that disables the device (typically 0)
    osp_float_t conversion[3];
    int swap[3];
} OspSensorAttributes_t;


/*-------------------------------------------------------------------------------------------------*\
 |    F O R W A R D   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/
static void _onCalibratedTriAxisSensorResultDataUpdate(const struct SensorId_t *sensorId,
    OSPD_CalibratedThreeAxisData_t *pSensorData);
static void _onUncalibratedTriAxisSensorResultDataUpdate(const struct SensorId_t *sensorId,
    OSPD_UncalibratedThreeAxisData_t *pSensorData);
static void _onStepDetectorSensorResultDataUpdate(const struct SensorId_t *sensorId, OSPD_StepDetectorData_t *pData);
static void _onStepCounterSensorResultDataUpdate(const struct SensorId_t *sensorId,
    OSPD_StepCounterData_t *pSensorData);
static void _onSignificantMotionSensorResultDataUpdate(const struct SensorId_t *sensorId,
    OSPD_SignificantMotionData_t *pSensorData);



/*-------------------------------------------------------------------------------------------------*\
 |    S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
static VirtualSensorDeviceManager *_pVsDevMgr;



static struct OspSensorAttributes_t _ospResultCodes[] = {
    {
        {
            SENSOR_STEP, SENSOR_STEP_COUNTER
        },
        false,
        "osp-step-counter",
        "osp-step-counter",
        (OSPD_ResultDataCallback_t)_onStepCounterSensorResultDataUpdate,
        -1,
        std::string(""),
        std::string(""),
        '1',
        '0',
        {
            0.0f, 0.0f, 0.0f
        },
        {
            0, 1, 2
        }
    },
    {
        {
            SENSOR_STEP, SENSOR_STEP_DETECTOR
        },
        false,
        "osp-step-detector",
        "osp-step-detector",
        (OSPD_ResultDataCallback_t)_onStepDetectorSensorResultDataUpdate,
        -1,
        std::string(""),
        std::string(""),
        '1',
        '0',
        {
            0.0f, 0.0f, 0.0f
        },
        {
            0, 1, 2
        }
    },
    {
        {
            SENSOR_CONTEXT_DEVICE_MOTION, CONTEXT_DEVICE_MOTION_SIGNIFICANT_MOTION
        },
        false,
        "osp-significant-motion",
        "osp-significant-motion",
        (OSPD_ResultDataCallback_t)_onSignificantMotionSensorResultDataUpdate,
        -1,
        std::string(""),
        std::string(""),
        '1',
        '0',
        {
            0.0f, 0.0f, 0.0f
        },
        {
            0, 1, 2
        }
    },
    {
        {
            SENSOR_ACCELEROMETER, SENSOR_ACCELEROMETER_UNCALIBRATED
        },
        false,
        "osp-uncal-accelerometer",
        "osp-uncal-accelerometer",
        (OSPD_ResultDataCallback_t)_onUncalibratedTriAxisSensorResultDataUpdate,
        -1,
        std::string(""),
        std::string(""),
        '1',
        '0',
        {
            0.0f, 0.0f, 0.0f
        },
        {
            0, 1, 2
        }
    },
    {
        {
            SENSOR_ACCELEROMETER, SENSOR_ACCELEROMETER_CALIBRATED
        },
        false,
        "osp-cal-accelerometer",
        "osp-cal-accelerometer",
        (OSPD_ResultDataCallback_t)_onCalibratedTriAxisSensorResultDataUpdate,
        -1,
        std::string(""),
        std::string(""),
        '1',
        '0',
        {
            0.0f, 0.0f, 0.0f
        },
        {
            0, 1, 2
        }
    },
    {
        {
            SENSOR_MAGNETIC_FIELD, SENSOR_MAGNETIC_FIELD_UNCALIBRATED
        },
        false,
        "osp-uncal-magnetometer",
        "osp-uncal-magnetometer",
        (OSPD_ResultDataCallback_t)_onUncalibratedTriAxisSensorResultDataUpdate,
        -1,
        std::string(""),
        std::string(""),
        '1',
        '0',
        {
            0.0f, 0.0f, 0.0f
        },
        {
            0, 1, 2
        }
    },
    {
        {
            SENSOR_MAGNETIC_FIELD, SENSOR_MAGNETIC_FIELD_CALIBRATED
        },
        false,
        "osp-cal-magnetometer",
        "osp-cal-magnetometer",
        (OSPD_ResultDataCallback_t)_onCalibratedTriAxisSensorResultDataUpdate,
        -1,
        std::string(""),
        std::string(""),
        '1',
        '0',
        {
            0.0f, 0.0f, 0.0f
        },
        {
            0, 1, 2
        }
    },
    {
        {
            SENSOR_GYROSCOPE, SENSOR_GYROSCOPE_UNCALIBRATED
        },
        false,
        "osp-uncal-gyroscope",
        "osp-uncal-gyroscope",
        (OSPD_ResultDataCallback_t)_onUncalibratedTriAxisSensorResultDataUpdate,
        -1,
        std::string(""),
        std::string(""),
        '1',
        '0',
        {
            0.0f, 0.0f, 0.0f
        },
        {
            0, 1, 2
        }
    },
    {
        {
            SENSOR_GYROSCOPE, SENSOR_GYROSCOPE_CALIBRATED
        },
        false,
        "osp-cal-gyroscope",
        "osp-cal-gyroscope",
        (OSPD_ResultDataCallback_t)_onCalibratedTriAxisSensorResultDataUpdate,
        -1,
        std::string(""),
        std::string(""),
        '1',
        '0',
        {
            0.0f, 0.0f, 0.0f
        },
        {
            0, 1, 2
        }
    },
};

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/




/****************************************************************************************************
 * @fn      _parseAndHandleEnable
 *          Helper routine for enabling/disabling sensors in the system
 *
 ***************************************************************************************************/
void OSPD_parseAndHandleSensorControls(char *buffer, ssize_t numBytesInBuffer)
{
    int16_t index;

    if (NULL == buffer) {
        LOG_Err("buffer should never be NULL!!!\n");
        return;
    }

    if (numBytesInBuffer < 0) {
        LOG_Err("not going to try parsing empty buffer\n");
        return;
    }
    if (numBytesInBuffer == 0) {
        return;
    }
    union ShCmdHeaderUnion *commandStructure = (union ShCmdHeaderUnion *)buffer;

    switch (commandStructure->command.command) {
    case OSP_HOST_SENSOR_SET_ENABLE:
        LOG_Info("%s: OSP_HOST_SENSOR_SET_ENABLE %d/%d : %s",
            __FUNCTION__,
            commandStructure->enable.sensorId.sensorType,
            commandStructure->enable.sensorId.sensorSubType,
            commandStructure->enable.enable ? "1" : "0");
        index = OSPD_getResultIndex(&commandStructure->enable.sensorId);
        if (index >= 0) {
            if (commandStructure->enable.enable) {
                osp_status_t status = OSPD_SubscribeResult(&commandStructure->enable.sensorId);
                if (status != OSP_STATUS_OK)
                    LOG_Err("error subscribing to result\n");
            } else {
                osp_status_t status = OSPD_UnsubscribeResult(&commandStructure->enable.sensorId);
                if (status != OSP_STATUS_OK)
                    LOG_Err("error unsubscribing from result\n");
            }
        } else {
        }
        break;

    case OSP_HOST_SENSOR_SET_DELAY:
        LOG_Info("%s: OSP_HOST_SENSOR_SET_DELAY %d/%d :  %d milli-seconds",
            __FUNCTION__,
            commandStructure->delay.sensorId.sensorType,
            commandStructure->delay.sensorId.sensorSubType,
            commandStructure->delay.delay_milisec);
        break;

    default:
        break;
    } // switch
}


/****************************************************************************************************
 * @fn      _onCalibratedTriAxisSensorResultDataUpdate
 *          Common callback for sensor data or results received from the hub
 * @param   sensorId - sensor id (type/sub type)
 * @param   data pointer
 *
 ***************************************************************************************************/
static void _onCalibratedTriAxisSensorResultDataUpdate(const struct SensorId_t *sensorId,
    OSPD_CalibratedThreeAxisData_t *pSensorData)
{
    int32_t uinputCompatibleDataFormat[3];
    char ok = 0;

    switch (sensorId->sensorType) {
    case SENSOR_ACCELEROMETER:
        switch (sensorId->sensorSubType) {
        case SENSOR_ACCELEROMETER_CALIBRATED:
            ok = 1;
            break;

        default:
            LOG_Err("%s unexpected result subtype for SENSOR_ACCELEROMETER : %d\n",
                __FUNCTION__, sensorId->sensorSubType);
            break;
        }

        break;

    case SENSOR_MAGNETIC_FIELD:
        switch (sensorId->sensorSubType) {
        case SENSOR_MAGNETIC_FIELD_CALIBRATED:
            ok = 1;
            break;

        default:
            LOG_Err("%s unexpected result subtype for SENSOR_MAGNETIC_FIELD : %d\n",
                __FUNCTION__, sensorId->sensorSubType);
            break;
        }

        break;

    case SENSOR_GYROSCOPE:
        switch (sensorId->sensorSubType) {
        case SENSOR_GYROSCOPE_CALIBRATED:
            ok = 1;
            break;
            LOG_Err("%s unexpected result subtype for SENSOR_GYROSCOPE : %d\n",
                __FUNCTION__, sensorId->sensorSubType);
            break;
        }

        break;

    default:
        LOG_Err("%s unexpected result type %d/%d\n",
            __FUNCTION__,
            sensorId->sensorType,
            sensorId->sensorSubType);
    } // switch

    if (ok) {
        int16_t index = OSPD_getResultIndex(sensorId);
        if (index >= 0) {
            uinputCompatibleDataFormat[0] = pSensorData->data[0].i;
            uinputCompatibleDataFormat[1] = pSensorData->data[1].i;
            uinputCompatibleDataFormat[2] = pSensorData->data[2].i;
#if 0
            LOGS("A %.3f (0x%8x), %.3f (0x%8x), %.3f (0x%8x), %lld\n",
                pSensorData->data[0].f, uinputCompatibleDataFormat[0],
                pSensorData->data[1].f, uinputCompatibleDataFormat[1],
                pSensorData->data[2].f, uinputCompatibleDataFormat[2],
                pSensorData->timestamp.ll);
#endif
            _pVsDevMgr->publish(
                _ospResultCodes[index].evdevFd,
                uinputCompatibleDataFormat,
                pSensorData->timestamp.ll,
                3);
        }
    }
}


static void _onUncalibratedTriAxisSensorResultDataUpdate(const struct SensorId_t *sensorId,
    OSPD_UncalibratedThreeAxisData_t *pSensorData)
{
    int32_t uinputCompatibleDataFormat[6];
    char ok = 0;

    switch (sensorId->sensorType) {
    case SENSOR_ACCELEROMETER:
        switch (sensorId->sensorSubType) {
        case SENSOR_ACCELEROMETER_UNCALIBRATED:
            ok = 1;
            break;

        default:
            LOG_Err("%s unexpected result subtype for SENSOR_ACCELEROMETER : %d\n",
                __FUNCTION__, sensorId->sensorSubType);
            break;
        }

        break;

    case SENSOR_MAGNETIC_FIELD:
        switch (sensorId->sensorSubType) {
        case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
            ok = 1;
            break;

        default:
            LOG_Err("%s unexpected result subtype for SENSOR_MAGNETIC_FIELD : %d\n",
                __FUNCTION__, sensorId->sensorSubType);
            break;
        }

        break;

    case SENSOR_GYROSCOPE:
        switch (sensorId->sensorSubType) {
        case SENSOR_GYROSCOPE_UNCALIBRATED:
            ok = 1;
            break;
            LOG_Err("%s unexpected result subtype for SENSOR_GYROSCOPE : %d\n",
                __FUNCTION__, sensorId->sensorSubType);
            break;
        }

        break;

    default:
        LOG_Err("%s unexpected result type %d/%d\n",
            __FUNCTION__,
            sensorId->sensorType,
            sensorId->sensorSubType);
    } // switch

    if (ok) {
        int16_t index = OSPD_getResultIndex(sensorId);
        if (index >= 0) {
            uinputCompatibleDataFormat[0] = pSensorData->data[0].i;
            uinputCompatibleDataFormat[1] = pSensorData->data[1].i;
            uinputCompatibleDataFormat[2] = pSensorData->data[2].i;
            uinputCompatibleDataFormat[3] = pSensorData->data[3].i;
            uinputCompatibleDataFormat[4] = pSensorData->data[4].i;
            uinputCompatibleDataFormat[5] = pSensorData->data[5].i;
#if 0
            LOGS("RA %.3f (0x%8x), %.3f (0x%8x), %.3f (0x%8x), "
                 "%.3f (0x%8x), %.3f (0x%8x), %.3f (0x%8x), %lld\n",
                pSensorData->data[0].f, uinputCompatibleDataFormat[0],
                pSensorData->data[1].f, uinputCompatibleDataFormat[1],
                pSensorData->data[2].f, uinputCompatibleDataFormat[2],
                pSensorData->data[3].f, uinputCompatibleDataFormat[3],
                pSensorData->data[4].f, uinputCompatibleDataFormat[4],
                pSensorData->data[5].f, uinputCompatibleDataFormat[5],
                pSensorData->timestamp.ll);
#endif
            _pVsDevMgr->publish(
                _ospResultCodes[index].evdevFd,
                uinputCompatibleDataFormat,
                pSensorData->timestamp.ll,
                6);
        }
    }
}


static void _onStepCounterSensorResultDataUpdate(const struct SensorId_t *sensorId, OSPD_StepCounterData_t *pSensorData)
{
    char ok = 0;

    switch (sensorId->sensorType) {
    case SENSOR_STEP:
        switch (sensorId->sensorSubType) {
        case SENSOR_STEP_DETECTOR:
            ok = 1;
            break;

        default:
            LOG_Err("%s unexpected result subtype for SENSOR_STEP : %d\n",
                __FUNCTION__, sensorId->sensorSubType);
            break;
        }

        break;

    default:
        LOG_Err("%s unexpected result type %d/%d\n",
            __FUNCTION__,
            sensorId->sensorType,
            sensorId->sensorSubType);
    } // switch

    if (ok) {
        int16_t index = OSPD_getResultIndex(sensorId);
        if (index >= 0) {
#if 0
            LOGS("SD  %lld\n",
                pSensorData->timestamp.ll);
#endif
            _pVsDevMgr->publish(
                _ospResultCodes[index].evdevFd,
                NULL,
                pSensorData->timestamp.ll,
                0);
        }
    }
}


static void _onStepDetectorSensorResultDataUpdate(const struct SensorId_t *sensorId,
    OSPD_StepDetectorData_t *pSensorData)
{
    char ok = 0;

    switch (sensorId->sensorType) {
    case SENSOR_STEP:
        switch (sensorId->sensorSubType) {
        case SENSOR_STEP_DETECTOR:
            ok = 1;
            break;

        default:
            LOG_Err("%s unexpected result subtype for SENSOR_STEP : %d\n",
                __FUNCTION__, sensorId->sensorSubType);
            break;
        }

        break;

    default:
        LOG_Err("%s unexpected result type %d/%d\n",
            __FUNCTION__,
            sensorId->sensorType,
            sensorId->sensorSubType);
    } // switch

    if (ok) {
        int16_t index = OSPD_getResultIndex(sensorId);
        if (index >= 0) {
#if 0
            LOGS("SD  %lld\n",
                pSensorData->timestamp.ll);
#endif
            _pVsDevMgr->publish(
                _ospResultCodes[index].evdevFd,
                NULL,
                pSensorData->timestamp.ll,
                0);
        }
    }
}


static void _onSignificantMotionSensorResultDataUpdate(const struct SensorId_t *sensorId,
    OSPD_SignificantMotionData_t *pSensorData)
{
    int32_t uinputCompatibleDataFormat;
    char ok = 0;

    switch (sensorId->sensorType) {
    case SENSOR_CONTEXT_DEVICE_MOTION:
        switch (sensorId->sensorSubType) {
        case CONTEXT_DEVICE_MOTION_SIGNIFICANT_MOTION:
            ok = 1;
            break;

        default:
            LOG_Err("%s unexpected result subtype for SENSOR_CONTEXT_DEVICE_MOTION : %d\n",
                __FUNCTION__, sensorId->sensorSubType);
            break;
        }

        break;

    default:
        LOG_Err("%s unexpected result type %d/%d\n",
            __FUNCTION__,
            sensorId->sensorType,
            sensorId->sensorSubType);
    } // switch

    if (ok) {
        int16_t index = OSPD_getResultIndex(sensorId);
        if (index >= 0) {
            uinputCompatibleDataFormat = pSensorData->significantMotionDetected.i;
#if 0
            LOGS("SC %.3f (0x%8x), %lld\n",
                pSensorData->data[0].f, pSensorData->significantMotionDetected.f,
                pSensorData->timestamp.ll);
#endif
            _pVsDevMgr->publish(
                _ospResultCodes[index].evdevFd,
                &uinputCompatibleDataFormat,
                pSensorData->timestamp.ll,
                1);
        }
    }
}


/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/


/****************************************************************************************************
 * @fn      OSPD_getResultIndex
 *          Returns an index into _ospResultCodes for the record for the specified sensorId.
 * @param   sensorId - specifies sensorId for result
 * @return  returns an index if record in _ospResultCodes for specifies sensorId, otherwise returns -1
 *
 ***************************************************************************************************/
int16_t OSPD_getResultIndex(const struct SensorId_t *sensorId)
{
    for (unsigned int index = 0; index < ARRAY_SIZE(_ospResultCodes); index++) {
        if ((_ospResultCodes[index].sensorId.sensorType == sensorId->sensorType) &&
            (_ospResultCodes[index].sensorId.sensorSubType == sensorId->sensorSubType)) {
            return index;
        }
    }
    return -1;
}


/****************************************************************************************************
 * @fn      OSPD_getResultDataReadyCallbackFunction
 *          Returns the data ready call back function pointer for the record for the specified sensorId.
 * @param   sensorId - specifies sensorId for result
 * @return  returns an call back function pointer if sensorId record is in _ospResultCodes, otherwise returns NULL
 *
 ***************************************************************************************************/
OSPD_ResultDataCallback_t OSPD_getResultDataReadyCallbackFunction(const struct SensorId_t *sensorId)
{
    for (unsigned int index = 0; index < ARRAY_SIZE(_ospResultCodes); index++) {
        if ((_ospResultCodes[index].sensorId.sensorType == sensorId->sensorType) &&
            (_ospResultCodes[index].sensorId.sensorSubType == sensorId->sensorSubType)) {
            return _ospResultCodes[index].dataReadyCallBackFunction;
        }
    }
    return NULL;
}


/****************************************************************************************************
 * @fn      OSPD_getSensorSwapTable
 *          Returns the the address of the SWAP table for the record for the specified sensorId.
 * @param   sensorId - specifies sensorId for result
 * @return  returns a pointer to SWAP table if sensorId record is in _ospResultCodes, otherwise returns NULL
 *
 ***************************************************************************************************/
int *OSPD_getSensorSwapTable(const struct SensorId_t *sensorId)
{
    for (unsigned int index = 0; index < ARRAY_SIZE(_ospResultCodes); index++) {
        if ((_ospResultCodes[index].sensorId.sensorType == sensorId->sensorType) &&
            (_ospResultCodes[index].sensorId.sensorSubType == sensorId->sensorSubType)) {
            return _ospResultCodes[index].swap;
        }
    }
    return NULL;
}


/****************************************************************************************************
 * @fn      OSPD_getSensorConversionTable
 *          Returns the the address of the SWAP table for the record for the specified sensorId.
 * @param   sensorId - specifies sensorId for result
 * @return  returns a pointer to SWAP table if sensorId record is in _ospResultCodes, otherwise returns NULL
 *
 ***************************************************************************************************/
osp_float_t *OSPD_getSensorConversionTable(const struct SensorId_t *sensorId)
{
    for (unsigned int index = 0; index < ARRAY_SIZE(_ospResultCodes); index++) {
        if ((_ospResultCodes[index].sensorId.sensorType == sensorId->sensorType) &&
            (_ospResultCodes[index].sensorId.sensorSubType == sensorId->sensorSubType)) {
            return _ospResultCodes[index].conversion;
        }
    }
    return NULL;
}


/****************************************************************************************************
 * @fn      OSPD_getSensordeviceDriverName
 *          Returns the the string pointer to driver name for the record for the specified sensorId.
 * @param   sensorId - specifies sensorId for result
 * @return  returns a pointer to to driver name if sensorId record is in _ospResultCodes, otherwise returns NULL
 *
 ***************************************************************************************************/
const char *OSPD_getSensordeviceDriverName(const struct SensorId_t *sensorId)
{
    for (unsigned int index = 0; index < ARRAY_SIZE(_ospResultCodes); index++) {
        if ((_ospResultCodes[index].sensorId.sensorType == sensorId->sensorType) &&
            (_ospResultCodes[index].sensorId.sensorSubType == sensorId->sensorSubType)) {
            return _ospResultCodes[index].deviceDriverName;
        }
    }
    return NULL;
}


/****************************************************************************************************
 * @fn      OSPD_initializeSensors
 *          initialized all sensors in _ospResultCodes.
 *
 ***************************************************************************************************/
void OSPD_initializeSensors(VirtualSensorDeviceManager *vsDevMgr)
{
    _pVsDevMgr = vsDevMgr;

    LOGT("%s:%d\r\n", __FUNCTION__, __LINE__);

    for (unsigned int i = 0; i < ARRAY_SIZE(_ospResultCodes); i++) {
        _ospResultCodes[i].evdevFd =
            _pVsDevMgr->createSensor(
            _ospResultCodes[i].deviceDriverName, INT_MIN, INT_MAX);
    }
}


/****************************************************************************************************
 * @fn      OSPD_stopAllSensors
 *          Unsubscribe/stop data flow for all sensors in _ospResultCodes
 *
 ***************************************************************************************************/
void OSPD_stopAllSensors(void)
{
    LOGT("%s\r\n", __FUNCTION__);

    for (unsigned int i = 0; i < ARRAY_SIZE(_ospResultCodes); i++) {
        osp_status_t status = OSPD_UnsubscribeResult(&_ospResultCodes[i].sensorId);
        if (status != OSP_STATUS_OK)
            LOG_Err("error unsubscribing to sensor %d/%d",
                _ospResultCodes[i].sensorId.sensorType,
                _ospResultCodes[i].sensorId.sensorSubType);
    }
}


/****************************************************************************************************
 * @fn      _parseAndHandleSensorControls
 *          Helper routine for enabling/disabling sensors in the system
 *
 ***************************************************************************************************/
void parseAndHandleSensorControls(char *buffer, ssize_t numBytesInBuffer)
{
    int16_t index;

    if (NULL == buffer) {
        LOG_Err("buffer should never be NULL!!!\n");
        return;
    }

    if (numBytesInBuffer < 0) {
        LOG_Err("not going to try parsing empty buffer\n");
        return;
    }
    if (numBytesInBuffer == 0) {
        return;
    }
    union ShCmdHeaderUnion *commandStructure = (union ShCmdHeaderUnion *)buffer;

    switch (commandStructure->command.command) {
    case OSP_HOST_SENSOR_SET_ENABLE:
        LOG_Info("%s: OSP_HOST_SENSOR_SET_ENABLE %d/%d : %s",
            __FUNCTION__,
            commandStructure->enable.sensorId.sensorType,
            commandStructure->enable.sensorId.sensorSubType,
            commandStructure->enable.enable ? "1" : "0");
        index = OSPD_getResultIndex(&commandStructure->enable.sensorId);
        if (index >= 0) {
            if (commandStructure->enable.enable) {
                osp_status_t status = OSPD_SubscribeResult(&commandStructure->enable.sensorId);
                if (status != OSP_STATUS_OK)
                    LOG_Err("error subscribing to result\n");
            } else {
                osp_status_t status = OSPD_UnsubscribeResult(&commandStructure->enable.sensorId);
                if (status != OSP_STATUS_OK)
                    LOG_Err("error unsubscribing from result\n");
            }
        } else {
        }
        break;

    case OSP_HOST_SENSOR_SET_DELAY:
        LOG_Info("%s: OSP_HOST_SENSOR_SET_DELAY %d/%d :  %d milli-seconds",
            __FUNCTION__,
            commandStructure->delay.sensorId.sensorType,
            commandStructure->delay.sensorId.sensorSubType,
            commandStructure->delay.delay_milisec);
        break;

    default:
        break;
    } // switch
}


/****************************************************************************************************
 * @fn      InitializeFromConfig
 *          Main Initialization routine. Initializes device configuration
 *
 ***************************************************************************************************/
static int32_t InitializeFromConfig( void )
{
    int32_t result;
    const int *swap;
    unsigned int swaplen;
    unsigned int convlen;
    const osp_float_t *conv;

    for (unsigned int index = 0; index < ARRAY_SIZE(_ospResultCodes); ++index) {
        const char *const drivername = OSPConfig::getNamedConfigItem(
            _ospResultCodes[index].configFileDeviceName,
            OSPConfig::SENSOR_DRIVER_NAME);
        auto sensorlist = OSPConfig::getConfigItemsMultiple("sensor");
        if (drivername && strlen(drivername)) {
            _ospResultCodes[index].deviceDriverName = drivername;
        } else {
            _ospResultCodes[index].deviceDriverName = "";
            for (unsigned short i2 = 0; i2 < sensorlist.size(); ++i2) {
                if (strcmp( sensorlist[i2], _ospResultCodes[index].configFileDeviceName) == 0) {
                    _ospResultCodes[index].deviceDriverName = _ospResultCodes[index].configFileDeviceName;
                    break;
                }
            }
        }


        swap = OSPConfig::getNamedConfigItemInt(
            _ospResultCodes[index].configFileDeviceName, OSPConfig::SENSOR_SWAP, &swaplen);
        if (!swap) {
            for (unsigned int j = 0; j < 3; ++j) {
                _ospResultCodes[index].swap[j] = j;
            }
        } else if (swaplen == 3) {
            for (unsigned int j = 0; j < 3; ++j) {
                _ospResultCodes[index].swap[j] = swap[j];
            }
        } else {
            LOG_Err("Invalid swap indices length of %d fo %s. ABORTING",
                swaplen, _ospResultCodes[index].configFileDeviceName);
            assert(swaplen == 3);
        }

        conv = OSPConfig::getNamedConfigItemFloat(
            _ospResultCodes[index].configFileDeviceName, OSPConfig::SENSOR_CONVERSION, &convlen);
        if (!conv) {
            for (unsigned int j = 0; j < 3; ++j) {
                _ospResultCodes[index].conversion[j] = 1.0f;
            }
        } else if (convlen == 1) {
            for (unsigned int j = 0; j < 3; ++j) {
                _ospResultCodes[index].conversion[j] = conv[0];
            }
        } else if (convlen == 3) {
            for (unsigned int j = 0; j < convlen; ++j) {
                _ospResultCodes[index].conversion[j] = conv[j];
            }
        } else {
            LOG_Err("Invalid conversion value array length of %d fo %s",
                convlen, _ospResultCodes[index].configFileDeviceName);
            assert(convlen == 3);
        }

        if (_ospResultCodes[index].deviceDriverName && strlen(_ospResultCodes[index].deviceDriverName)) {
            osp_char_t *sysfs = NULL;

            if (OSPConfig::getNamedConfigItem(
                _ospResultCodes[index].configFileDeviceName,
                OSPConfig::SENSOR_ENABLE_PATH)) {
                _ospResultCodes[index].enableValue =
                    OSPConfig::getNamedConfigItemIntV(
                    _ospResultCodes[index].deviceDriverName,
                    OSPConfig::SENSOR_ENABLE_VALUE,
                    1);

                _ospResultCodes[index].disableValue =
                    OSPConfig::getNamedConfigItemIntV(
                    _ospResultCodes[index].deviceDriverName,
                    OSPConfig::SENSOR_DISABLE_VALUE,
                    1);

                if (asprintf(
                    &sysfs,
                    "/sys/class/sensor_relay/%s/%s",
                    _ospResultCodes[index].deviceDriverName,
                    OSPConfig::getNamedConfigItem(
                    _ospResultCodes[index].configFileDeviceName,
                    OSPConfig::SENSOR_ENABLE_PATH))< 0) {
                    LOG_Err("asprintf call failed!");
                } else {
                    LOG_Info("Sysfs Enable Path: %s, %d, %d",
                        sysfs,
                        _ospResultCodes[index].enableValue,
                        _ospResultCodes[index].disableValue);
                    _ospResultCodes[index].sysEnablePath.assign(sysfs);
                    free(sysfs);
                }
            }
            if (OSPConfig::getNamedConfigItem(_ospResultCodes[index].configFileDeviceName,
                OSPConfig::SENSOR_DELAY_PATH)) {
                if (asprintf(
                    &sysfs, "/sys/class/sensor_relay/%s/%s",
                    _ospResultCodes[index].deviceDriverName,
                    OSPConfig::getNamedConfigItem(
                    _ospResultCodes[index].deviceDriverName,
                    OSPConfig::SENSOR_DELAY_PATH)) < 0) {
                    LOG_Err("asprintf call failed!");
                } else {
                    LOG_Info("Sysfs Delay Path: %s", sysfs);
                    _ospResultCodes[index].sysDelayPath.assign(sysfs);
                    free(sysfs);
                }
            }
        }
    }
    /* Initialize the micro-second per tick value */
    int32_t relayTickUsec = OSPConfig::getConfigItemIntV(
        OSPConfig::PROTOCOL_RELAY_TICK_USEC,
        1,
        NULL);
    LOG_Info("Relay Ticks per us: %d", relayTickUsec);


    std::string deviceRelayInputName = std::string(OSPConfig::getConfigItem(OSPConfig::PROTOCOL_RELAY_DRIVER));

    if (!deviceRelayInputName.empty()) {
        result = InitializeRelayInput(deviceRelayInputName, relayTickUsec);
        if (result != OSP_STATUS_OK) {
            LOG_Err("InitializeRelayInput failed (%d)", result);
            assert(result != OSP_STATUS_OK);
        }
    } else {
        LOG_Err("InitializeRelayInput failed - relay device name not provided");
        assert(deviceRelayInputName.empty());
    }


    return result;
}


/****************************************************************************************************
 * @fn      OSPD_Initialize
 *          Initialize remote procedure call for the daemon
 *
 ***************************************************************************************************/
osp_status_t OSPD_Initialize(void)
{
    osp_status_t result = OSP_STATUS_OK;

    //int tick_us = 24;
    LOGT("%s\r\n", __FUNCTION__);

    /* Dump config for debug */
    OSPConfig::dump("/data/tmp/config-dump.txt");

    result = InitializeFromConfig();
    if (result != OSP_STATUS_OK) {
        LOG_Err("Initialize failed (%d)", result);
    }
    return result;
}


/****************************************************************************************************
 * @fn      OSPD_GetVersion
 *          Helper routine for getting daemon version information
 *
 ***************************************************************************************************/
osp_status_t OSPD_GetVersion(char *versionString, int bufSize)
{
    osp_status_t result = OSP_STATUS_OK;

    LOGT("%s\r\n", __FUNCTION__);

    return result;
}


/****************************************************************************************************
 * @fn      OSPD_SubscribeResult
 *          Enables subscription for results
 *
 ***************************************************************************************************/
osp_status_t OSPD_SubscribeResult(const struct SensorId_t *sensorId)
{
    LOGT("%s\r\n", __FUNCTION__);

    uint16_t index = OSPD_getResultIndex(sensorId);
    if (index >= 0) {
        _ospResultCodes[index].subscribed = true;
        return OSP_STATUS_OK;
    }

    return OSP_STATUS_ERROR;
}


/****************************************************************************************************
 * @fn      OSPD_UnsubscribeResult
 *          Unsubscribe from sensor results
 *
 ***************************************************************************************************/
osp_status_t OSPD_UnsubscribeResult(const struct SensorId_t *sensorId)
{
    LOGT("%s\r\n", __FUNCTION__);

    uint16_t index = OSPD_getResultIndex(sensorId);
    if (index >= 0) {
        _ospResultCodes[index].subscribed = false;
        return OSP_STATUS_OK;
    }
    return OSP_STATUS_ERROR;
}


/****************************************************************************************************
 * @fn      OSPD_Deinitialize
 *          Tear down RPC interface function
 *
 ***************************************************************************************************/
osp_status_t OSPD_Deinitialize(void)
{
    osp_status_t result = OSP_STATUS_OK;

    LOGT("%s\r\n", __FUNCTION__);
    terminateRelayThread();
    return result;
}


/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
