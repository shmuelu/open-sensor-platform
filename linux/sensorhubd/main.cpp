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

#include <unistd.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

#include "osp-sensors.h"
#include "osp_debuglogging.h"
#include "virtualsensordevicemanager.h"
#include "osp_remoteprocedurecalls.h"

/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/
#define MAX_NUM_FDS(x,y)                ((x) > (y) ? (x) : (y))
#define ACCEL_UINPUT_NAME               "osp-accelerometer"
#define GYRO_UINPUT_NAME                "osp-gyroscope"
#define MAG_UINPUT_NAME                 "osp-magnetometer"

#define TWENTY_MS_IN_US                 (20000)
#define CONTROL_PIPE_NAME               "/data/misc/osp-control"

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/


/*-------------------------------------------------------------------------------------------------*\
 |    S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
static void _logErrorIf(bool condition, const char* msg);
static void _fatalErrorIf(bool condition, int code, const char* msg);
static void _handleQuitSignals(int signum);
static void _initialize();
static void _deinitialize();
static void _parseAndHandleEnable(int sensorIndex, char* buffer, ssize_t numBytesInBuffer);
static VirtualSensorDeviceManager* _pVsDevMgr;
static int _evdevFds[SENSORHUBD_RESULT_INDEX_COUNT] ={-1};

static int _controlPipeFd = -1;


static struct SensorId_t _ospResultCodes[]= {
    { SENSOR_STEP, SENSOR_STEP_COUNTER },
    { SENSOR_STEP, SENSOR_STEP_DETECTOR },
    { SENSOR_CONTEXT_DEVICE_MOTION,	CONTEXT_DEVICE_MOTION_SIGNIFICANT_MOTION },
    { SENSOR_ACCELEROMETER,	SENSOR_ACCELEROMETER_UNCALIBRATED },
    { SENSOR_ACCELEROMETER,	SENSOR_ACCELEROMETER_CALIBRATED },
    { SENSOR_MAGNETIC_FIELD, SENSOR_MAGNETIC_FIELD_UNCALIBRATED },
    { SENSOR_MAGNETIC_FIELD, SENSOR_MAGNETIC_FIELD_CALIBRATED },
    { SENSOR_GYROSCOPE, SENSOR_GYROSCOPE_UNCALIBRATED },
    { SENSOR_GYROSCOPE, SENSOR_GYROSCOPE_CALIBRATED } 
};

/*-------------------------------------------------------------------------------------------------*\
 |    F O R W A R D   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/
static void _onTriAxisSensorResultDataUpdate(SensorType_t sensorType, void* pData);

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/****************************************************************************************************
 * @fn      _logErrorIf
 *          Helper routine to log error on condition
 *
 ***************************************************************************************************/
static void _logErrorIf(bool condition, const char* msg)
{
    if (condition) {
        LOG_Err("%s\n", msg);
    }
}


/****************************************************************************************************
 * @fn      _fatalErrorIf
 *          Helper routine to log error on condition & exit the application
 *
 ***************************************************************************************************/
static void _fatalErrorIf(bool condition, int code, const char* msg)
{
    if (condition) {
        LOG_Err("%s\n", msg);
        _deinitialize();
        _exit(code);
    }
}


/****************************************************************************************************
 * @fn      _parseAndHandleEnable
 *          Helper routine for enabling/disabling sensors in the system
 *
 ***************************************************************************************************/
static void _parseAndHandleEnable(int sensorIndex, char* buffer, ssize_t numBytesInBuffer)
{

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
    LOGT("%s: sensorIndex %d\r\n", __FUNCTION__, sensorIndex);

    if ('0' == buffer[0]) {
        LOG_Info("Unsubscribe from sensor index %d\n", (int)sensorIndex);
        osp_status_t status= OSPD_UnsubscribeResult(_ospResultCodes[sensorIndex]);
        _logErrorIf(status != OSP_STATUS_OK, "error unsubscribing from result\n");

    } else if ('1' == buffer[0]) {
        LOG_Info("Subscribe to sensor index %d\n", (int)sensorIndex);
        osp_status_t status= OSPD_SubscribeResult(_ospResultCodes[sensorIndex], _onTriAxisSensorResultDataUpdate);
        _logErrorIf(status != OSP_STATUS_OK, "error subscribing to result\n");

    } else {
        //LOG_Err("unexpected data in enable buffer: %s", buffer);
    }


}


/****************************************************************************************************
 * @fn      _initializeNamedPipe
 *          Initializes named pipe that receive requests for sensors' control
 *
 ***************************************************************************************************/
static void _initializeNamedPipe()
{
    LOGT("%s:%d\r\n", __FUNCTION__, __LINE__);

    unlink(CONTROL_PIPE_NAME);

    _fatalErrorIf(mkfifo(CONTROL_PIPE_NAME, 0666) != 0, -1, "could not create named pipe");

    _controlPipeFd = open(CONTROL_PIPE_NAME, O_RDONLY|O_NONBLOCK);
    _fatalErrorIf(_controlPipeFd < 0, -1, "could not open named pipe for reading");
}


/****************************************************************************************************
 * @fn      _onTriAxisSensorResultDataUpdate
 *          Common callback for sensor data or results received from the hub
 *
 ***************************************************************************************************/
static void _onTriAxisSensorResultDataUpdate(SensorType_t sensorType, void* pData)
{
    OSPD_ThreeAxisData_t* pSensorData= (OSPD_ThreeAxisData_t*)pData;
    int32_t uinputCompatibleDataFormat[3];


    switch(sensorType)  {

    case SENSOR_ACCELEROMETER_UNCALIBRATED:
        uinputCompatibleDataFormat[0] = pSensorData->data[0].i;
        uinputCompatibleDataFormat[1] = pSensorData->data[1].i;
        uinputCompatibleDataFormat[2] = pSensorData->data[2].i;
#if 0
        LOGS("RA %.3f (0x%8x), %.3f (0x%8x), %.3f (0x%8x), %lld\n",
             pSensorData->data[0].f, uinputCompatibleDataFormat[0],
             pSensorData->data[1].f, uinputCompatibleDataFormat[1],
             pSensorData->data[2].f, uinputCompatibleDataFormat[2],
             pSensorData->timestamp.ll);
#endif
        _pVsDevMgr->publish(
                    _evdevFds[SENSORHUBD_ACCELEROMETER_INDEX],
                    uinputCompatibleDataFormat,
                    pSensorData->timestamp.ll);
        break;


    case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
        uinputCompatibleDataFormat[0] = pSensorData->data[0].i;
        uinputCompatibleDataFormat[1] = pSensorData->data[1].i;
        uinputCompatibleDataFormat[2] = pSensorData->data[2].i;

        _pVsDevMgr->publish(
                    _evdevFds[SENSORHUBD_MAGNETOMETER_INDEX],
                    uinputCompatibleDataFormat,
                    pSensorData->timestamp.ll);
        break;

    case SENSOR_GYROSCOPE_UNCALIBRATED:
        uinputCompatibleDataFormat[0] = pSensorData->data[0].i;
        uinputCompatibleDataFormat[1] = pSensorData->data[1].i;
        uinputCompatibleDataFormat[2] = pSensorData->data[2].i;

        _pVsDevMgr->publish(
                    _evdevFds[SENSORHUBD_GYROSCOPE_INDEX],
                    uinputCompatibleDataFormat,
                    pSensorData->timestamp.ll);
        break;
#if 0
    case SENSOR_CONTEXT_DEVICE_MOTION: 
        //LOGS("SIGM %.3f (0x%8x), %.3f (0x%8x), %.3f (0x%8x)\n", pSensorData->data[0]);

        uinputCompatibleDataFormat[0]= (int)(pSensorData->data[0]);
        timeInNano= (int64_t)(NSEC_PER_SEC * pSensorData->timestamp);

        _pVsDevMgr->publish(_evdevFds[SENSORHUBD_SIG_MOTION_INDEX], uinputCompatibleDataFormat, timeInNano, 1);
        break;
#endif
    default:
        LOG_Err("%s unexpected result type %d\n", __FUNCTION__, sensorType);
        break;
    }

}


/****************************************************************************************************
 * @fn      _subscribeToAllResults
 *          Subscribes to all available results from sensor hub
 *
 ***************************************************************************************************/
static void _subscribeToAllResults()
{
    osp_status_t status;
    LOGT("%s:%d\r\n", __FUNCTION__, __LINE__);

    status = OSPD_SubscribeResult(SENSOR_ACCELEROMETER_UNCALIBRATED, _onTriAxisSensorResultDataUpdate);
    _logErrorIf(status != OSP_STATUS_OK, "error subscribing to SENSOR_ACCELEROMETER");

    status = OSPD_SubscribeResult(SENSOR_MAGNETIC_FIELD_UNCALIBRATED, _onTriAxisSensorResultDataUpdate);
    _logErrorIf(status != OSP_STATUS_OK, "error subscribing to SENSOR_MAGNETIC_FIELD");

    status = OSPD_SubscribeResult(SENSOR_GYROSCOPE_UNCALIBRATED, _onTriAxisSensorResultDataUpdate);
    _logErrorIf(status != OSP_STATUS_OK, "error subscribing to SENSOR_GYROSCOPE");

    //    status = OSPD_SubscribeResult(SENSOR_CONTEXT_DEVICE_MOTION, _onTriAxisSensorResultDataUpdate);
    //    _logErrorIf(status != OSP_STATUS_OK, "error subscribing to SENSOR_CONTEXT_DEVICE_MOTION");

    //    status = OSPD_SubscribeResult(SENSOR_STEP_COUNTER, _onTriAxisSensorResultDataUpdate);
    //    _logErrorIf(status != OSP_STATUS_OK, "error subscribing to SENSOR_STEP_COUNTER");

}


/****************************************************************************************************
 * @fn      _initialize
 *          Application initializer
 *
 ***************************************************************************************************/
static void _initialize()
{
    osp_status_t status;

    LOGT("%s:%d\r\n", __FUNCTION__, __LINE__);

    //create our raw input virtual sensors
    _evdevFds[SENSORHUBD_ACCELEROMETER_INDEX] = _pVsDevMgr->createSensor(ACCEL_UINPUT_NAME, "acc0",  INT_MIN, INT_MAX);
    _evdevFds[SENSORHUBD_MAGNETOMETER_INDEX] = _pVsDevMgr->createSensor(MAG_UINPUT_NAME, "mag0",  INT_MIN, INT_MAX);
    _evdevFds[SENSORHUBD_GYROSCOPE_INDEX] = _pVsDevMgr->createSensor(GYRO_UINPUT_NAME, "gyr0",  INT_MIN, INT_MAX);
    //_evdevFds[SENSORHUBD_SIG_MOTION_INDEX]= _pVsDevMgr->createSensor("osp-significant-motion", "sigm0",  INT_MIN, INT_MAX);
    //_evdevFds[SENSORHUBD_STEP_COUNTER_INDEX]= _pVsDevMgr->createSensor("osp-step-counter", "stc0",  INT_MIN, INT_MAX);

    //Initialize OSP Daemon
    LOGT("%s:%d\r\n", __FUNCTION__, __LINE__);
    status= OSPD_Initialize();

    LOGT("%s:%d\r\n", __FUNCTION__, __LINE__);
    _fatalErrorIf(status!= OSP_STATUS_OK, status, "Failed on OSP Daemon Initialization!");

    //print out the daemon version
    //char versionString[255];
    //OSPD_GetVersion(versionString, 255);
    //LOG_Info("OSP Daemon version %s\n", versionString);

    _initializeNamedPipes();

    //!!! Debug only
    _subscribeToAllResults();

}


/****************************************************************************************************
 * @fn      _stopAllResults
 *          Unsubscribe/stop data flow for sensor data or results
 *
 ***************************************************************************************************/
static void _stopAllResults()
{
    LOGT("%s\r\n", __FUNCTION__);

    osp_status_t status= OSPD_UnsubscribeResult(SENSOR_ACCELEROMETER_UNCALIBRATED);
    _logErrorIf(status != OSP_STATUS_OK, "error unsubscribing to SENSOR_ACCELEROMETER_UNCALIBRATED");

    status= OSPD_UnsubscribeResult(SENSOR_MAGNETIC_FIELD_UNCALIBRATED);
    _logErrorIf(status != OSP_STATUS_OK, "error unsubscribing to SENSOR_MAGNETIC_FIELD_UNCALIBRATED");

    status= OSPD_UnsubscribeResult(SENSOR_GYROSCOPE_UNCALIBRATED);
    _logErrorIf(status != OSP_STATUS_OK, "error unsubscribing to SENSOR_GYROSCOPE_UNCALIBRATED");

    //    status= OSPD_UnsubscribeResult(SENSOR_CONTEXT_DEVICE_MOTION);
    //    _logErrorIf(status != OSP_STATUS_OK, "error unsubscribing to SENSOR_CONTEXT_DEVICE_MOTION");

    status= OSPD_UnsubscribeResult(SENSOR_STEP_COUNTER);
    _logErrorIf(status != OSP_STATUS_OK, "error unsubscribing to SENSOR_STEP_COUNTER");
}

/****************************************************************************************************
 * @fn      _deinitialize
 *          Cleanup & teardown on application exit
 *
 ***************************************************************************************************/
static void _deinitialize()
{
    LOGT("%s\r\n", __FUNCTION__);

    _stopAllResults();

    OSPD_Deinitialize();
}


/****************************************************************************************************
 * @fn      _handleQuitSignals
 *          Handler for linux exit/terminate signal for the application
 *
 ***************************************************************************************************/
static void _handleQuitSignals(int signum)
{
    LOGT("%s\r\n", __FUNCTION__);

    _deinitialize();
    exit(0);
}


/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/****************************************************************************************************
 * @fn      main
 *          Application entry point
 *
 ***************************************************************************************************/
int main(int argc, char** argv)
{
#define MS_TO_US 1000
    int result =0;
    fd_set readFdSet;
    fd_set errFdSet;
    int32_t maxNumFds = 0;
    int selectResult;

    //
    signal(SIGINT, _handleQuitSignals);

    //create this on the stack so we know it always gets cleaned up properly
    VirtualSensorDeviceManager vsDevMgr;
    _pVsDevMgr= &vsDevMgr;

    //After initialize, all the magic happens in the callbacks such as _onAccelerometerResultDataUpdate
    _initialize();

    /* This loop handles sensor enable/disable requests */
    // FROM WHERE??
    while (1) {

        // setup the select to read on all pipes
        FD_ZERO(&readFdSet);
        FD_ZERO(&errFdSet);

        FD_SET( _controlPipeFd, &readFdSet);
        FD_SET( _controlPipeFd, &errFdSet);
        maxNumFds = MAX_NUM_FDS( maxNumFds, _controlPipeFd);

        //Wait for data on one of the pipes
        selectResult = select(maxNumFds+1, &readFdSet, NULL, &errFdSet, NULL);
        _logErrorIf(selectResult<0, "error on select() of named pipes");

        if ( selectResult > 0 ) {
            
            _logErrorIf(FD_ISSET(_controlPipeFd, &errFdSet), "error on FD!\n");
            
            if (FD_ISSET(_controlPipeFd, &readFdSet) ) {
                char readBuf[255];
                ssize_t bytesRead = read(_controlPipeFd, readBuf, 255);
                _logErrorIf(bytesRead < 0, "failed on read of enable pipe for");

                FD_CLR(_controlPipeFd, &readFdSet );
                if (0 == bytesRead) {
                    char pipename[255];

                    //close and reopen the pipe or we'll spike CPU usage b/c select will always
                    //return available and we'll always get 0 bytes from read
                    close(_controlPipeFd);
                    int _controlPipeFd = open(CONTROL_PIPE_NAME, O_RDONLY|O_NONBLOCK);
                    _fatalErrorIf(_controlPipeFd < 0, -1, "could not open named pipe for reading");

                    continue;
                }
                _parseAndHandleEnable(readBuf, bytesRead);
            }
        }
    }
    return result;
}


/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
