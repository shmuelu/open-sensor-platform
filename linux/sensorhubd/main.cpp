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
#include "osp-sensor-control-interface.h"
#include "ospd.h"

/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/
#define MAX_NUM_FDS(x, y)                ((x) > (y) ? (x) : (y))
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define CONTROL_PIPE_NAME               "/data/misc/osp-control"

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    F O R W A R D   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/



/*-------------------------------------------------------------------------------------------------*\
 |    S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
static void _logErrorIf(bool condition, const char *msg);
static void _fatalErrorIf(bool condition, int code, const char *msg);
static void _handleQuitSignals(int signum);
static void _initialize(VirtualSensorDeviceManager *vsDevMgr);
static void _deinitialize();
static int _controlPipeFd = -1;


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
static void _logErrorIf(bool condition, const char *msg)
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
static void _fatalErrorIf(bool condition, int code, const char *msg)
{
    if (condition) {
        LOG_Err("%s\n", msg);
        _deinitialize();
        _exit(code);
    }
}


/****************************************************************************************************
* @fn _initializeNamedPipe
* Initializes named pipes that receive requests for sensor control
*
***************************************************************************************************/
static void _initializeNamedPipe()
{
    LOGT("%s:%d\r\n", __FUNCTION__, __LINE__);

    //Try and remove the pipe if it's already there, but don't complain if it's not
    unlink(CONTROL_PIPE_NAME);

    _fatalErrorIf(mkfifo(CONTROL_PIPE_NAME, 0666) != 0, -1, "could not create named pipe");

    _controlPipeFd = open(CONTROL_PIPE_NAME, O_RDONLY | O_NONBLOCK);
    _fatalErrorIf(_controlPipeFd < 0, -1, "could not open named pipe for reading");
}


/****************************************************************************************************
 * @fn      _initialize
 *          Application initializer
 *
 ***************************************************************************************************/
static void _initialize(VirtualSensorDeviceManager *vsDevMgr)
{
    osp_status_t status;

    LOGT("%s:%d\r\n", __FUNCTION__, __LINE__);

    OSPD_initializeSensors(vsDevMgr);


    //Initialize OSP Daemon
    LOGT("%s:%d\r\n", __FUNCTION__, __LINE__);
    status = OSPD_Initialize();

    LOGT("%s:%d\r\n", __FUNCTION__, __LINE__);
    _fatalErrorIf(status!= OSP_STATUS_OK, status, "Failed on OSP Daemon Initialization!");

    //print out the daemon version
    //char versionString[255];
    //OSPD_GetVersion(versionString, 255);
    //LOG_Info("OSP Daemon version %s\n", versionString);

    _initializeNamedPipe();
}


/****************************************************************************************************
 * @fn      _deinitialize
 *          Cleanup & teardown on application exit
 *
 ***************************************************************************************************/
static void _deinitialize()
{
    LOGT("%s\r\n", __FUNCTION__);

    OSPD_stopAllSensors();

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
int main(int argc, char * *argv)
{
#define MS_TO_US 1000
    int result = 0;
    fd_set readFdSet;
    fd_set errFdSet;
    int32_t maxNumFds = 0;
    int selectResult;

    //
    signal(SIGINT, _handleQuitSignals);

    //create this on the stack so we know it always gets cleaned up properly
    VirtualSensorDeviceManager vsDevMgr;

    //After initialize, all the magic happens in the callbacks such as _onAccelerometerResultDataUpdate
    _initialize(&vsDevMgr);

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
        selectResult = select(maxNumFds + 1, &readFdSet, NULL, &errFdSet, NULL);
        _logErrorIf(selectResult<0, "error on select() of named pipes");

        if (selectResult > 0) {
            _logErrorIf(FD_ISSET(_controlPipeFd, &errFdSet), "error on FD!\n");

            if (FD_ISSET(_controlPipeFd, &readFdSet) ) {
                char readBuf[255];
                ssize_t bytesRead = read(_controlPipeFd, readBuf, 255);
                _logErrorIf(bytesRead < 0, "failed on read of enable pipe for");

                FD_CLR(_controlPipeFd, &readFdSet );
                if (0 == bytesRead) {
                    //close and reopen the pipe or we'll spike CPU usage b/c select will always
                    //return available and we'll always get 0 bytes from read
                    close(_controlPipeFd);
                    int _controlPipeFd = open(CONTROL_PIPE_NAME, O_RDONLY | O_NONBLOCK);
                    _fatalErrorIf(_controlPipeFd < 0, -1, "could not open named pipe for reading");

                    continue;
                }
                OSPD_parseAndHandleSensorControls(readBuf, bytesRead);
            }
        }
    }
    return result;
}


/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
