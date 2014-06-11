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
#include <fcntl.h>
#include <errno.h>
#include <cstring>
#include <cstdlib>
#include <sys/mman.h>
#include <sys/select.h>
#include <linux/input.h>
#include <assert.h>
#include "osp_debuglogging.h"
#include "osp_configuration.h"

#include "sensor_relay.h"
#include "ospd.h"

#include "osp_relayinterface.h"

extern "C" {
#include "uinpututils.h"
}

/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/
#define PROCESS_INPUT_EVT_THRES         1
#define MAX_NUM_FDS(x, y) ((x) > (y) ? (x) : (y))

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/* per-cpu buffer info */
typedef struct {
    size_t produced;
    size_t consumed;
    size_t max_backlog; /* max # sub-buffers ready at one time */
} RelayBufStatus_t;


/*-------------------------------------------------------------------------------------------------*\
 |    S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
static int32_t _relay_fd;
static int32_t _relayTickUsec;
static std::string _deviceRelayInputName;
static std::vector<RelayBufStatus_t> _relayStatus;
static std::vector<int> _relay_file;
static std::vector<unsigned char *> __relay_buffer;
/* control files */
static std::vector<int> _produced_file;
static std::vector<int> _consumed_file;


static pthread_t _relayThread;
static volatile bool _relayThreadActive = false;


/*-------------------------------------------------------------------------------------------------*\
 |    F O R W A R D   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/****************************************************************************************************
 * @fn      _sensorDataPublish
 *          Parse the sensor data and invoke result callbacks
 *
 ***************************************************************************************************/
static void _sensorDataPublish(const struct SensorId_t *sensorId, void *pSensData)
{
    OSPD_ResultDataCallback_t callbackFunction = OSPD_getResultDataReadyCallbackFunction(sensorId);

    if (callbackFunction) {
        callbackFunction(sensorId, pSensData);
    }
}


/****************************************************************************************************
 * @fn      _doAxisSwapAndConvertInt16
 *          Helper function that does the axis swap and unit conversion as specified in configuration
 * @param   intVal   points to raw data array
 * @param   floatVal points for floating point result storage
 * @param   firstEventCode - first input event to be used for this data array
 * @param   lastEventCode  - last input event to be used for this data array
 * @param   swapTable - pointer to axis swap table, NULL if not swap needed
 * @param   conversionTable - pointer to axis unit conversion table
 *
 ***************************************************************************************************/
static void _doAxisSwapAndConvertInt16(
    int16_t *intVal,
    OspValue_t *floatVal,
    int32_t firstEventCode,
    int32_t lastEventCode,
    const int *swapTable,
    const osp_float_t *conversionTable)
{
    assert(conversionTable != NULL);

    for (int eventCode = firstEventCode; eventCode <= lastEventCode; eventCode++) {
        int32_t axisIndex = eventCode - firstEventCode;

        if (swapTable) {
            assert( axisIndex >= 0 );
            assert( axisIndex < 3 );
            axisIndex = swapTable[axisIndex];
        }
        floatVal->f = intVal[axisIndex] * conversionTable[axisIndex];
    }
}


/****************************************************************************************************
 * @fn      _doAxisSwapAndConvertUncalibratedInt16
 *          Helper function that does the axis swap and unit conversion as specified in configuration
 * @param   intVal   points to raw data array
 * @param   floatVal points for floating point result storage
 * @param   firstEventCode - first input event to be used for this data array
 * @param   lastEventCode  - last input event to be used for this data array
 * @param   swapTable - pointer to axis swap table, NULL if not swap needed
 * @param   conversionTable - pointer to axis unit conversion table
 *
 ***************************************************************************************************/
static void _doAxisSwapAndConvertUncalibratedInt16(
    int16_t *intVal,
    OspValue_t *floatVal,
    int32_t firstEventCode,
    int32_t lastEventCode,
    const int *swapTable,
    const osp_float_t *conversionTable)
{
    _doAxisSwapAndConvertInt16(
        intVal,
        &floatVal[0],
        ABS_X,
        ABS_Z,
        swapTable,
        conversionTable);

    _doAxisSwapAndConvertInt16(
        &intVal[3],
        &floatVal[3],
        ABS_X + 3,
        ABS_Z + 3,
        swapTable,
        conversionTable);
}


/****************************************************************************************************
 * @fn      ProcessInputEventsRelay
 *          Helper routine for processing sensor data coming via RelayFS
 *
 ***************************************************************************************************/
static void ProcessInputEventsRelay(void)
{
    size_t size;

    for (unsigned int cpu = 0; cpu < _produced_file.size(); cpu++) {
        lseek(_produced_file[cpu], 0, SEEK_SET);
        if (read(_produced_file[cpu], &size,
                sizeof(size)) < 0) {
            LOG_Info("Couldn't read from consumed file for cpu %d, exiting: errcode = %d: %s\n",
                cpu,
                errno,
                strerror(errno));
            break;
        }
        _relayStatus[cpu].produced = size;

#if 0
        LOG_Info("wakeup  CPU %d produced %d consumed %d  \n",
            cpu,
            _relayStatus[cpu].produced,
            _relayStatus[cpu].consumed);
#endif

        size_t bufidx, start_subbuf, subbuf_idx;

        size_t subbufs_consumed = 0;

        unsigned char *subbuf_ptr;

        size_t subbufs_ready = _relayStatus[cpu].produced - _relayStatus[cpu].consumed + 1;

        start_subbuf = _relayStatus[cpu].consumed % SENSOR_RELAY_NUM_RELAY_BUFFERS;

        if ((_relayStatus[cpu].produced == 0) && (_relayStatus[cpu].consumed == 0))
            subbufs_ready = 0;
#if 0
        LOG_Info("produced  %d   consumed %d\n",
            _relayStatus[cpu].produced,
            _relayStatus[cpu].consumed);
#endif

        for (bufidx = start_subbuf; subbufs_ready-- > 0; bufidx++) {
            subbuf_idx = bufidx % SENSOR_RELAY_NUM_RELAY_BUFFERS;
            subbuf_ptr = __relay_buffer[cpu] + (subbuf_idx * sizeof(union sensor_relay_broadcast_node));
            union  sensor_relay_broadcast_node *sensorNode = (union sensor_relay_broadcast_node *)subbuf_ptr;

#if 0

            LOG_Info("relay_buffer[%d] index %d of %d\n"
                     "%2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x\n"
                     "%2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x\n",
                cpu,
                bufidx,
                subbufs_ready,
                subbuf_ptr[0], subbuf_ptr[1], subbuf_ptr[2],
                subbuf_ptr[3], subbuf_ptr[4], subbuf_ptr[5],
                subbuf_ptr[6], subbuf_ptr[7], subbuf_ptr[8],
                subbuf_ptr[9], subbuf_ptr[10], subbuf_ptr[11],
                subbuf_ptr[12], subbuf_ptr[13], subbuf_ptr[14],
                subbuf_ptr[15], subbuf_ptr[16], subbuf_ptr[17],
                subbuf_ptr[18], subbuf_ptr[19], subbuf_ptr[20],
                subbuf_ptr[21], subbuf_ptr[22], subbuf_ptr[23]);
#endif // if 0

            std::string driverName = std::string(OSPD_getSensordeviceDriverName(&sensorNode->sensorData.sensorId));
            if (driverName.empty()) {
                subbufs_consumed++;
                continue;
            }

            /* Axis unit conversions & result callbacks */
            uint64_t timeTicks = sensorNode->sensorData.timeStamp;
            int64_t timeNsec = (int64_t)(_relayTickUsec * timeTicks * 1000);

            int *swapTable = OSPD_getSensorSwapTable(&sensorNode->sensorData.sensorId);
            osp_float_t *conversionTable = OSPD_getSensorConversionTable(&sensorNode->sensorData.sensorId);

            switch (sensorNode->sensorData.sensorId.sensorType) {
            case SENSOR_ACCELEROMETER:
                switch (sensorNode->sensorData.sensorId.sensorSubType) {
                case SENSOR_ACCELEROMETER_UNCALIBRATED:
                {
                    OSPD_UncalibratedThreeAxisData_t floatVal;
                    floatVal.timestamp.ll = timeNsec;

                    _doAxisSwapAndConvertUncalibratedInt16(
                            &sensorNode->sensorData.Data[0],
                            &floatVal.data[0],
                            ABS_X,
                            ABS_Z,
                            swapTable,
                            conversionTable);
                    _sensorDataPublish(&sensorNode->sensorData.sensorId, &floatVal);
                }
                break;

                case SENSOR_ACCELEROMETER_CALIBRATED:
                {
                    OSPD_CalibratedThreeAxisData_t floatVal;
                    floatVal.timestamp.ll = timeNsec;
                    _doAxisSwapAndConvertInt16(
                            &sensorNode->sensorData.Data[0],
                            &floatVal.data[0],
                            ABS_X,
                            ABS_Z,
                            swapTable,
                            conversionTable);
                    _sensorDataPublish(&sensorNode->sensorData.sensorId, &floatVal);
                }
                break;
                } // switch

                break;

            case SENSOR_MAGNETIC_FIELD:
                switch (sensorNode->sensorData.sensorId.sensorSubType) {
                case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
                {
                    OSPD_UncalibratedThreeAxisData_t floatVal;
                    floatVal.timestamp.ll = timeNsec;
                    _doAxisSwapAndConvertUncalibratedInt16(
                            &sensorNode->sensorData.Data[0],
                            &floatVal.data[0],
                            ABS_X,
                            ABS_Z,
                            swapTable,
                            conversionTable);
                    _sensorDataPublish(&sensorNode->sensorData.sensorId, &floatVal);
                }
                break;

                case SENSOR_MAGNETIC_FIELD_CALIBRATED:
                {
                    OSPD_CalibratedThreeAxisData_t floatVal;
                    floatVal.timestamp.ll = timeNsec;
                    _doAxisSwapAndConvertInt16(
                            &sensorNode->sensorData.Data[0],
                            &floatVal.data[0],
                            ABS_X,
                            ABS_Z,
                            swapTable,
                            conversionTable);
                    _sensorDataPublish(&sensorNode->sensorData.sensorId, &floatVal);
                }
                break;
                } // switch

                break;

            case SENSOR_GYROSCOPE:
                switch (sensorNode->sensorData.sensorId.sensorSubType) {
                case SENSOR_GYROSCOPE_UNCALIBRATED:
                {
                    OSPD_UncalibratedThreeAxisData_t floatVal;
                    floatVal.timestamp.ll = timeNsec;
                    _doAxisSwapAndConvertUncalibratedInt16(
                            &sensorNode->sensorData.Data[0],
                            &floatVal.data[0],
                            ABS_X,
                            ABS_Z,
                            swapTable,
                            conversionTable);
                    _sensorDataPublish(&sensorNode->sensorData.sensorId, &floatVal);
                }
                break;

                case SENSOR_GYROSCOPE_CALIBRATED:
                {
                    OSPD_CalibratedThreeAxisData_t floatVal;
                    floatVal.timestamp.ll = timeNsec;
                    _doAxisSwapAndConvertInt16(
                            &sensorNode->sensorData.Data[0],
                            &floatVal.data[0],
                            ABS_X,
                            ABS_Z,
                            swapTable,
                            conversionTable);
                    _sensorDataPublish(&sensorNode->sensorData.sensorId, &floatVal);
                }
                break;
                } // switch

                break;

            default:
                //TODO Error Handling??
                LOG_Err("Bad Sensor ID %d/%d!!",
                    sensorNode->sensorData.sensorId.sensorType,
                    sensorNode->sensorData.sensorId.sensorSubType);
                return;
            } // switch

#if 0
# ifdef ANDROID
            switch (sensorIndex) {
            case ACCEL_INDEX:
            case MAG_INDEX:
            case GYRO_INDEX:
                LOG_Info(
                    "bufidx %-3.3d Sensor %-2.2x  TimeStamp 0x%-8.8llx x 0x%-8.8x  y 0x%-8.8x  z 0x%-8.8x tv_usec %-20.6f   \n",
                    bufidx,
                    sensorNode->sensorData.sensorId,
                    sensorNode->sensorData.TimeStamp,
                    sensorNode->sensorData.Data[0],
                    sensorNode->sensorData.Data[1],
                    sensorNode->sensorData.Data[2],
                    timeUsec / 1000000.0);

                LOG_Info("{!%s, %20.6f , %10.6f , %10.6f , %10.6f, 0 ,!}\n",
                    label,
                    (float)timeUsec / (float)1000000.0,
                    (float)_sfloatSensorData[sensorIndex][0],
                    (float)_sfloatSensorData[sensorIndex][1],
                    (float)_sfloatSensorData[sensorIndex][2]);
                break;

            default:
                LOG_Info("bufidx %-3.3d Sensor %-2.2x  TimeStamp 0x%-8.8llx value 0x%-8.8x  tv_usec %-20.6f   \n",
                    bufidx,
                    sensorNode->sensorData.sensorId,
                    sensorNode->sensorData.TimeStamp,
                    sensorNode->sensorData.Data[0],
                    timeUsec / 1000000.0);

                LOG_Info("{!%s, %20.6f , %10.6f , 0 ,!}\n",
                    label,
                    (float)timeUsec / (float)1000000.0,
                    (float)_sfloatSensorData[sensorIndex][0]);
                break;
            } // switch

# endif // ifdef ANDROID
#endif // if 0


#if 0
            SensorIndexToInputProducerMap::iterator iIndexToProducer;
            iIndexToProducer = _sensorIndexToInputProducerMap.find(sensorIndex);

            if (iIndexToProducer != _sensorIndexToInputProducerMap.end() ) {
                InputProducerInterface *pProducer = iIndexToProducer->second;

                if (pProducer) {
                    switch (pProducer->getType()) {
                    case PRODUCER_IS_THREEAXIS:
                    {
                        dynamic_cast<ThreeAxisSensorProducer *>(pProducer)->SetDataByFloat
                            (eventTimeDbl,
                                _sfloatSensorData[sensorIndex],
                                3);
                    }
                    break;

                    case PRODUCER_IS_STEP_DATA:
                    {
                        NTTIME eventTime = TOFIX_TIME(eventTimeDbl);
                        StepData_t data;

                        data.startTime = eventTime;
                        data.stopTime = eventTime;
                        data.stepLength = 0;
                        data.stepFrequency = 0;
                        data.numStepsTotal = _numStepsTotal;
                        data.numStepsSinceWalking = 0;
                        data.numStepsUp = 0;
                        data.numStepsDown = 0;
                        data.numStepsLevel = 0;
                        dynamic_cast<StepDataProducer *>(pProducer)->SetData(data);
                    }
                    break;

                    case PRODUCER_IS_QUATERNION:
                    {
                        NTTIME eventTime = TOFIX_TIME(eventTimeDbl);
                        Quat quat;
                        quat <<
                        (float)_sfloatSensorData[sensorIndex][0],
                        (float)_sfloatSensorData[sensorIndex][1],
                        (float)_sfloatSensorData[sensorIndex][2],
                        (float)_sfloatSensorData[sensorIndex][3];

                        AttitudeData data(eventTime, quat, 0.0f, 0.0f);

                        dynamic_cast<AttitudeProducer *>(pProducer)->SetData(data);
                    }
                    break;

                    default:
                        LOG_Err(
                            "No mapping found for producer at sensor index %d as PRODUCER_IS_THREEAXIS nor PRODUCER_IS_STEP_DATA",
                            sensorIndex );
                        break;
                    } // switch
                } else {
                    LOG_Err("No mapping found for producer at sensor index %d",
                        sensorIndex );
                }
            } else {
                LOG_Err("No mapping found for producer at sensor index %d",
                    sensorIndex );
            }
#endif // if 0
            subbufs_consumed++;
        }

        if (subbufs_consumed) {
            if (subbufs_consumed == SENSOR_RELAY_NUM_RELAY_BUFFERS)
                LOG_Err("cpu %d buffer full.  Consider using a larger buffer size", cpu);
            if (subbufs_consumed > _relayStatus[cpu].max_backlog)
                _relayStatus[cpu].max_backlog = subbufs_consumed;

            _relayStatus[cpu].consumed += subbufs_consumed;
#if 0
# ifdef ANDROID
            LOG_Info("cpu %d consumed %d\n", cpu, subbufs_consumed);
# endif
#endif
            if (write(_consumed_file[cpu], &subbufs_consumed, sizeof(subbufs_consumed)) < 0) {
                LOG_Err("Couldn't write to consumed file for cpu %d, exiting: errcode = %d: %s",
                    cpu, errno, strerror(errno));
                exit(1);
            }
            subbufs_consumed = 0;
        }
    }
}


/****************************************************************************************************
 * @fn      _relayReadAndProcessSensorData
 *          Helper routine for reading and handling sensor data coming via RelayFS
 *
 ***************************************************************************************************/
static void _relayReadAndProcessSensorData(int fd)
{
    fd_set readFdSet;
    fd_set excFdSet;
    int32_t bytesRead, nfds = 0;
    int32_t selectResult;
    struct input_event inputEvents[PROCESS_INPUT_EVT_THRES];

    //Since select() modifies its fdset, if the call is being used in a loop, then
    //the set must be re-initialized before each call
    FD_ZERO(&readFdSet);
    FD_ZERO(&excFdSet);

    FD_SET(_relay_fd, &readFdSet);
    FD_SET(_relay_fd, &excFdSet);
    nfds = MAX_NUM_FDS( nfds, _relay_fd);

    /* Wait to recieve data on the relay pipe */
    selectResult = select(nfds + 1, &readFdSet, NULL, &excFdSet, NULL);
    if (!_relayThreadActive) {
        return;
    }

    if (selectResult > 0) {
        //read and publish the events
        if (FD_ISSET(_relay_fd, &excFdSet) ) {
            LOG_Err("ERROR: exception on select of socket for relay");
        } else {
            //make sure this FD has data before trying to read
            if (FD_ISSET(_relay_fd, &readFdSet) ) {
                bytesRead = read(_relay_fd, &inputEvents[0], sizeof(struct input_event));
                if (bytesRead < 0) {
                    LOG_Err("I/O read error on relay input device %d", _relay_fd);
                } else if (bytesRead == 0) {
                    FD_CLR( _relay_fd, &readFdSet );
                    FD_CLR( _relay_fd, &excFdSet );
                } else {
                    if ((bytesRead % sizeof(struct input_event)) != 0) {
                        LOG_Err("partial event struct read. Would lose some samples!");
                    }
                    for (int i = 0; i < (bytesRead / sizeof(struct input_event)); i++) {
                        if (inputEvents[i].code == ABS_VOLUME) {
                            ProcessInputEventsRelay();
                        }
                    }
                }
            }
        }
    }
}


/****************************************************************************************************
 * @fn      _processRelayInput
 *          Thread entry function for relay input processing
 *
 ***************************************************************************************************/
static void *_processRelayInput(void *pData)
{
    LOG_Info("%s", __FUNCTION__);

    if (_relay_fd > 0) {
        _relayThreadActive = true;
    }

    //Main processing loop
    while (_relayThreadActive) {
        _relayReadAndProcessSensorData(_relay_fd);
    }

    LOG_Info("Relay thread exiting...");

    return 0;
}


/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/****************************************************************************************************
 * @fn      InitializeRelayInput
 *          Initializes the relay-fs interface and associated data structures.
 *
 ***************************************************************************************************/
int32_t InitializeRelayInput( std::string deviceRelayInputName, int32_t relayTickUsec )
{
    osp_char_t devname[256];
    unsigned int cpu;

    _deviceRelayInputName = deviceRelayInputName;
    _relayTickUsec = relayTickUsec;

    memset(devname, 0, sizeof(devname));

    _relay_fd = -1;

    //open up the real sensor drivers
    _relay_fd = openInputEventDeviceExt(_deviceRelayInputName.c_str(), devname);
    if (_relay_fd < 0) {
        LOG_Err("Unable to open relay input device with name %s",
            _deviceRelayInputName.c_str() );
        return OSP_STATUS_UNKNOWN_INPUT;
    }
    LOG_Info("Open relay input device with name %s",
        _deviceRelayInputName.c_str() );



    for (cpu = 0; (cpu < 16); ) {
        const RelayBufStatus_t dummyBufStatus = {0, 0, 0};

        sprintf(devname, "/sys/kernel/debug/sensor_relay_kernel%d", cpu);
        const int fileHandle = open(devname, O_RDONLY | O_NONBLOCK);

        if (fileHandle < 0) {
            if (cpu == 0) {
                LOG_Err("Couldn't open relay file %s: errcode = %s\n",
                    devname, strerror(errno));
            }
            break;
        }
        LOG_Info("cpu %d file", cpu);

        unsigned char *bufferP = (unsigned char *)mmap(
            NULL,
            sizeof(union sensor_relay_broadcast_node) *
            SENSOR_RELAY_NUM_RELAY_BUFFERS, PROT_READ,
            MAP_PRIVATE | MAP_POPULATE, fileHandle,
            0);

        if (bufferP == MAP_FAILED) {
            LOG_Err("Couldn't mmap relay file cpu %d, subbuf_size (%d) * n_subbufs(%d), error = %s \n",
                cpu,
                (int)sizeof(union sensor_relay_broadcast_node),
                SENSOR_RELAY_NUM_RELAY_BUFFERS,
                strerror(errno));
            close(fileHandle);
            break;
        }

        LOG_Info("cpu %d mmap", cpu);

        sprintf(devname, "/sys/kernel/debug/sensor_relay_kernel%d.produced", cpu);
        const int producerFile = open(devname, O_RDONLY);
        if (producerFile < 0) {
            LOG_Err("Couldn't open control file %s\n", devname);
            break;
        }

        LOG_Info("cpu %d producer", cpu);

        sprintf(devname, "/sys/kernel/debug/sensor_relay_kernel%d.consumed", cpu);
        const int consumedFile = open(devname, O_RDWR);
        if (consumedFile < 0) {
            LOG_Err("Couldn't open control file %s\n", devname);
            break;
        }
        LOG_Info("cpu %d consumed", cpu);

        _relay_file.push_back(fileHandle);
        __relay_buffer.push_back(bufferP);
        _produced_file.push_back(producerFile);
        _consumed_file.push_back(consumedFile);
        _relayStatus.push_back(dummyBufStatus);

        _relayStatus[cpu].produced = 0;
        _relayStatus[cpu].consumed = 0;
        cpu++;
    }

    if (cpu == 0) {
        LOG_Err("No CPU relay files found");
        return OSP_STATUS_UNKNOWN_INPUT;
    }
    if (pthread_create(&_relayThread, NULL, _processRelayInput, NULL )!= 0) {
        LOG_Err("Unable to create relay input processing thread\n");
        return OSP_STATUS_ERROR;
    }
    return OSP_STATUS_OK;
}


/****************************************************************************************************
 * @fn      terminateRelayThread
 *          terminate the relay thread.
 *
 ***************************************************************************************************/
void terminateRelayThread(void)
{
    int threadStatus;

    pthread_join(_relayThread, (void * *)&threadStatus);
    _relayThreadActive = false;
}


/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
