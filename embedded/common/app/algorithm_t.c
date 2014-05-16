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
#include "common.h"
#include "osp-api.h"
#include "hostinterface.h"
#include "hostFunctions.h"

/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/
#define M_SI_EARTH_GRAVITY              (9.805f)

#define ACCEL_SENSITIVITY_4G            0.002f      //2mg/LSB
#define ACCEL_SCALING_FACTOR            TOFIX_PRECISE(ACCEL_SENSITIVITY_4G * M_SI_EARTH_GRAVITY)
#define ACCEL_UNSCALING_FACTOR          CONST_PRECISE(1.0f / (ACCEL_SENSITIVITY_4G * M_SI_EARTH_GRAVITY))  // LSB / earth grav
#define ACCEL_RANGE_MAX                 TOFIX_EXTENDED(4.0f * M_SI_EARTH_GRAVITY)
#define ACCEL_NOISE                     TOFIX_PRECISE(1.0f/M_SI_EARTH_GRAVITY)
#define ACCEL_INPUT_RATES               TOFIX_EXTENDED(12.5f),TOFIX_EXTENDED(25.0f),TOFIX_EXTENDED(50.0f),TOFIX_EXTENDED(100.0f)
#define ACCEL_OUTPUT_RATES              TOFIX_EXTENDED(50.0f),TOFIX_EXTENDED(0.0f),TOFIX_EXTENDED(0.0f),TOFIX_EXTENDED(0.0f)

#define MAG_SENSITIVITY_2_5G_XY         1.4925f     //mg/LSB
#define MAG_SCALING_XY                  TOFIX_EXTENDED(MAG_SENSITIVITY_2_5G_XY * 0.1f)  //uT/LSB
#define MAG_UNSCALING_XY                CONST_EXTENDED(1.0f / (MAG_SENSITIVITY_2_5G_XY * 0.1f))  //LSB / ut
#define MAG_SENSITIVITY_2_5G_Z          1.6666f     //mg/LSB
#define MAG_SCALING_Z                   TOFIX_EXTENDED(MAG_SENSITIVITY_2_5G_Z * 0.1f)  //uT/LSB
#define MAG_UNSCALING_Z                 CONST_EXTENDED(1.0f / (MAG_SENSITIVITY_2_5G_Z * 0.1f))  //LSB / ut
#define MAG_RANGE_MAX                   (2.5f * 100.0f) //µT
#define MAG_NOISE                       TOFIX_PRECISE(0.2f)
#define MAG_INPUT_RATES                 TOFIX_EXTENDED(12.5f),TOFIX_EXTENDED(25.0f),TOFIX_EXTENDED(50.0f),TOFIX_EXTENDED(100.0f)
#define MAG_OUTPUT_RATES                TOFIX_EXTENDED(50.0f),TOFIX_EXTENDED(0.0f),TOFIX_EXTENDED(0.0f),TOFIX_EXTENDED(0.0f)

#define GYRO_SENSITIVITY                0.07f       //70mdps/digit
#define GYRO_SCALING_FACTOR             TOFIX_PRECISE(GYRO_SENSITIVITY * 0.0175f)  //rad/sec/lsb
#define GYRO_UNSCALING_1_FACTOR         CONST_PRECISE(1.0f / 0.0175f)  //  lsb / rad
#define GYRO_UNSCALING_2_FACTOR         CONST_PRECISE(1.0f / GYRO_SENSITIVITY)  //  lsb / deg

#define GYRO_RANGE_MAX                  (2000.0f * 0.017453f) //in rad/sec
#define GYRO_NOISE                      TOFIX_PRECISE(0.006f)
#define GYRO_INPUT_RATES                TOFIX_EXTENDED(12.5f),TOFIX_EXTENDED(25.0f),TOFIX_EXTENDED(50.0f),TOFIX_EXTENDED(100.0f)
#define GYRO_OUTPUT_RATES               TOFIX_EXTENDED(50.0f),TOFIX_EXTENDED(0.0f),TOFIX_EXTENDED(0.0f),TOFIX_EXTENDED(0.0f)

#define XYZ_FROM_ORIGIN                 TOFIX_PRECISE(0.0f)

#define STEP_COUNT_OUTPUT_RATE          TOFIX_EXTENDED(0.0f),TOFIX_EXTENDED(0.0f),TOFIX_EXTENDED(0.0f),TOFIX_EXTENDED(0.0f)
#define SIG_MOTION_OUTPUT_RATE          TOFIX_EXTENDED(0.0f),TOFIX_EXTENDED(0.0f),TOFIX_EXTENDED(0.0f),TOFIX_EXTENDED(0.0f)

/*-------------------------------------------------------------------------------------------------*\
 |    F O R W A R D   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/

static void EnterCriticalSection(void);
static void ExitCriticalSection(void);

static void stepCounterOutputCallback(OutputSensorHandle_t outputHandle,
    Android_StepCounterOutputData_t* pOutput);

static void stepDetectorOutputCallback(OutputSensorHandle_t outputHandle,
    Android_StepCounterOutputData_t* pOutput);

static void sigMotionOutputCallback(OutputSensorHandle_t outputHandle,
    Android_SignificantMotionOutputData_t* pOutput);

static void UnCalAccelDataResultCallback(OutputSensorHandle_t outputHandle,
    Android_UncalibratedAccelOutputData_t* pOutput);

static void CalAccelDataResultCallback(OutputSensorHandle_t outputHandle,
    Android_CalibratedAccelOutputData_t* pOutput);

static void UnCalMagDataResultCallback(OutputSensorHandle_t outputHandle,
    Android_UncalibratedMagOutputData_t* pOutput);

static void CalMagDataResultCallback(OutputSensorHandle_t outputHandle,
    Android_CalibratedMagOutputData_t* pOutput);

static void UnCalGyroDataResultCallback(OutputSensorHandle_t outputHandle,
    Android_UncalibratedGyroOutputData_t* pOutput);

static void CalGyroDataResultCallback(OutputSensorHandle_t outputHandle,
    Android_CalibratedGyroOutputData_t* pOutput);


/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/* Common bridge between the different data types for base sensors (Accel/Mag/Gyro) */
/* NOTE: the index into this array is NOT necessarily based on Axis index - it is implementation dependant */
typedef struct  {
    uint8_t accuracy;
    union {
        NTEXTENDED  ntExtendedUnscaleFactor[3];    // unscaling factor from 32 bit presentation into 16 bit for NTEXTENDED
        NTPRECISE   ntPreciseUnscaleFactor[3];     // unscaling factor from 32 bit presentation into 16 bit for NTPRECISE
    } unscale;
} ResultOptionalData_t;


/*-------------------------------------------------------------------------------------------------*\
 |    S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/


static OutputSensorHandle_t _unCalAccelHandle = NULL;
static OutputSensorHandle_t _calAccelHandle = NULL;
static OutputSensorHandle_t _unCalMagHandle = NULL;
static OutputSensorHandle_t _calMagHandle = NULL;
static OutputSensorHandle_t _unCalGyroHandle = NULL;
static OutputSensorHandle_t _calGyroHandle = NULL;

static InputSensorHandle_t _AccHandle = NULL;
static InputSensorHandle_t _MagHandle = NULL;
static InputSensorHandle_t _GyroHandle = NULL;

static OutputSensorHandle_t _stepCounterHandle = NULL;
static OutputSensorHandle_t _stepDetectorHandle = NULL;
static OutputSensorHandle_t _sigMotionHandle = NULL;

static const OSP_Library_Version_t* version;
static OS_MUT mutexCritSection;

static SystemDescriptor_t gSystemDesc =
{
    TOFIX_TIMECOEFFICIENT(US_PER_RTC_TICK * 0.000001f),        // timestamp conversion factor = 1us / count
    (OSP_CriticalSectionCallback_t) EnterCriticalSection,
    (OSP_CriticalSectionCallback_t) ExitCriticalSection
};


/***********************************************************/
/* Input Sensors descriptor for registering */
/***********************************************************/



static InputSensorSpecificData_t _AccInputSensor =
{
    0xffffffff,                             // 32 bits of raw data are significant
    AXIS_MAP_POSITIVE_X,AXIS_MAP_POSITIVE_Y,AXIS_MAP_POSITIVE_Z, // X,Y,Z  sensor orientation
    0,0,0,                                  // raw data offset (where is zero?)
    ACCEL_SCALING_FACTOR,ACCEL_SCALING_FACTOR,ACCEL_SCALING_FACTOR, //scale factor (raw to dimensional units)
    ACCEL_RANGE_MAX,                        // max value that is valid (e.g. +/- 4G sensor in M/Sec*Sec)
    -ACCEL_RANGE_MAX,                       // min value that is valid
    ACCEL_NOISE,ACCEL_NOISE,ACCEL_NOISE,    // noise
    (void *) NULL,                          // calibration data structure
    "Acc LSM303DLHC",                       // Sensor name
    0,                                      // Sensor Vendor ID
    0,                                      // Sensor Product ID (defined by vendor)
    0,                                      // Sensor version (defined by vendor)
    0,                                      // Platform id  (defined by vendor)
    XYZ_FROM_ORIGIN,XYZ_FROM_ORIGIN,XYZ_FROM_ORIGIN, // X,Y,Z position from origin (in meters)
};

static SensorDescriptor_t _AccSensDesc =
{
    SENSOR_ACCELEROMETER_UNCALIBRATED,
    DATA_CONVENTION_RAW,
    OSP_NO_OUTPUT_READY_CALLBACK,
    OSP_NO_NVM_WRITE_CALLBACK,
    OSP_NO_SENSOR_CONTROL_CALLBACK,
    ACCEL_INPUT_RATES,
    OSP_FLAGS_INPUT_SENSOR,
    &_AccInputSensor
};

static InputSensorSpecificData_t _MagInputSensor =
{
    0xffffffff,                             // 32 bits of raw data are significant
    AXIS_MAP_POSITIVE_X,AXIS_MAP_POSITIVE_Y,AXIS_MAP_POSITIVE_Z, // X,Y,Z  sensor orientation
    0,0,0,                                  // raw data offset (where is zero?)
    MAG_SCALING_XY,MAG_SCALING_XY,MAG_SCALING_Z, //scale factor (raw to dimensional units)
    MAG_RANGE_MAX,                          // max value that is valid (e.g. +/- 4G sensor in M/Sec*Sec)
    -MAG_RANGE_MAX,                         // min value that is valid
    MAG_NOISE,MAG_NOISE,MAG_NOISE,          // noise
    (void *) NULL,                          // calibration data structure
    "Mag LSM303DLHC",                       // Sensor name
    0,                                      // Sensor Vendor ID
    0,                                      // Sensor Product ID (defined by vendor)
    0,                                      // Sensor version (defined by vendor)
    0,                                      // Platform id  (defined by vendor)
    XYZ_FROM_ORIGIN,XYZ_FROM_ORIGIN,XYZ_FROM_ORIGIN, // X,Y,Z position from origin (in meters)
};

static SensorDescriptor_t _MagSensDesc =
{
    SENSOR_MAGNETIC_FIELD_UNCALIBRATED,
    DATA_CONVENTION_RAW,
    OSP_NO_OUTPUT_READY_CALLBACK,
    OSP_NO_NVM_WRITE_CALLBACK,
    OSP_NO_SENSOR_CONTROL_CALLBACK,
    MAG_INPUT_RATES,
    OSP_FLAGS_INPUT_SENSOR,
    &_MagInputSensor
};

static InputSensorSpecificData_t _GyroInputSensor =
{
    0xffffffff,                             // 32 bits of raw data are significant
    AXIS_MAP_POSITIVE_X,AXIS_MAP_POSITIVE_Y,AXIS_MAP_POSITIVE_Z, // X,Y,Z  sensor orientation
    0,0,0,                                  // raw data offset (where is zero?)
    GYRO_SCALING_FACTOR,GYRO_SCALING_FACTOR,GYRO_SCALING_FACTOR, //scale factor (raw to dimensional units)
    GYRO_RANGE_MAX,                         // max value that is valid (e.g. +/- 4G sensor in M/Sec*Sec)
    -GYRO_RANGE_MAX,                        // min value that is valid
    GYRO_NOISE,GYRO_NOISE,GYRO_NOISE,       // noise
    (void *) NULL,                          // calibration data structure
    "Gyro L3GD20",                          // Sensor name
    0,                                      // Sensor Vendor ID
    0,                                      // Sensor Product ID (defined by vendor)
    0,                                      // Sensor version (defined by vendor)
    0,                                      // Platform id  (defined by vendor)
    XYZ_FROM_ORIGIN,XYZ_FROM_ORIGIN,XYZ_FROM_ORIGIN, // X,Y,Z position from origin (in meters)
};

static SensorDescriptor_t _GyroSensDesc =
{
    SENSOR_GYROSCOPE_UNCALIBRATED,
    DATA_CONVENTION_RAW,
    OSP_NO_OUTPUT_READY_CALLBACK,
    OSP_NO_NVM_WRITE_CALLBACK,
    OSP_NO_SENSOR_CONTROL_CALLBACK,
    GYRO_INPUT_RATES,
    OSP_FLAGS_INPUT_SENSOR,
    &_GyroInputSensor
};


/***********************************************************/
/* Output result descriptor for subscribing */
/***********************************************************/


static SensorDescriptor_t  stepCounterRequest = {
    SENSOR_STEP_COUNTER,
    DATA_CONVENTION_ANDROID,
    (OSP_OutputReadyCallback_t)stepCounterOutputCallback,
    OSP_NO_NVM_WRITE_CALLBACK,
    OSP_NO_SENSOR_CONTROL_CALLBACK,
    STEP_COUNT_OUTPUT_RATE,
    OSP_NO_FLAGS,
    OSP_NO_OPTIONAL_DATA
};

static SensorDescriptor_t  stepDetectorRequest = {
    SENSOR_STEP_DETECTOR,
    DATA_CONVENTION_ANDROID,
    (OSP_OutputReadyCallback_t)stepDetectorOutputCallback,
    OSP_NO_NVM_WRITE_CALLBACK,
    OSP_NO_SENSOR_CONTROL_CALLBACK,
    STEP_COUNT_OUTPUT_RATE,
    OSP_NO_FLAGS,
    OSP_NO_OPTIONAL_DATA
};

/* Output result descriptor for subscribing to significant motion */
static SensorDescriptor_t  sigMotionRequest = {
    SENSOR_CONTEXT_DEVICE_MOTION,
    DATA_CONVENTION_ANDROID,
    (OSP_OutputReadyCallback_t)sigMotionOutputCallback,
    OSP_NO_NVM_WRITE_CALLBACK,
    OSP_NO_SENSOR_CONTROL_CALLBACK,
    SIG_MOTION_OUTPUT_RATE,
    OSP_NO_FLAGS,
    OSP_NO_OPTIONAL_DATA
};

static ResultOptionalData_t accelUnscaleFactor = { QFIXEDPOINTPRECISE, {ACCEL_UNSCALING_FACTOR, ACCEL_UNSCALING_FACTOR, ACCEL_UNSCALING_FACTOR }}; /* index per Axis */


/* Output result descriptor for subscribing to uncalibrated accelerometer data */
static SensorDescriptor_t UnCalAccelRequest = {
    SENSOR_ACCELEROMETER_UNCALIBRATED,
    DATA_CONVENTION_ANDROID,
    (OSP_OutputReadyCallback_t)UnCalAccelDataResultCallback,
    OSP_NO_NVM_WRITE_CALLBACK,
    OSP_NO_SENSOR_CONTROL_CALLBACK,
    ACCEL_OUTPUT_RATES,
    OSP_NO_FLAGS,
    OSP_NO_OPTIONAL_DATA
};


/* Output result descriptor for subscribing to uncalibrated accelerometer data */
static SensorDescriptor_t CalAccelRequest = {
    SENSOR_ACCELEROMETER_CALIBRATED,
    DATA_CONVENTION_ANDROID,
    (OSP_OutputReadyCallback_t)CalAccelDataResultCallback,
    OSP_NO_NVM_WRITE_CALLBACK,
    OSP_NO_SENSOR_CONTROL_CALLBACK,
    ACCEL_OUTPUT_RATES,
    OSP_NO_FLAGS,
    OSP_NO_OPTIONAL_DATA
};

static ResultOptionalData_t magUnscaleFactor = { QFIXEDPOINTEXTENDED, { MAG_UNSCALING_XY, MAG_UNSCALING_XY, MAG_UNSCALING_Z }}; /* index per Axis */

/* Output result descriptor for subscribing to uncalibrated magnetometer data */
static SensorDescriptor_t UnCalMagRequest = {
    SENSOR_MAGNETIC_FIELD_UNCALIBRATED,
    DATA_CONVENTION_ANDROID,
    (OSP_OutputReadyCallback_t)UnCalMagDataResultCallback,
    OSP_NO_NVM_WRITE_CALLBACK,
    OSP_NO_SENSOR_CONTROL_CALLBACK,
    MAG_OUTPUT_RATES,
    OSP_NO_FLAGS,
    OSP_NO_OPTIONAL_DATA
};

/* Output result descriptor for subscribing to uncalibrated magnetometer data */
static SensorDescriptor_t CalMagRequest = {
    SENSOR_MAGNETIC_FIELD_CALIBRATED,
    DATA_CONVENTION_ANDROID,
    (OSP_OutputReadyCallback_t)CalMagDataResultCallback,
    OSP_NO_NVM_WRITE_CALLBACK,
    OSP_NO_SENSOR_CONTROL_CALLBACK,
    MAG_OUTPUT_RATES,
    OSP_NO_FLAGS,
    OSP_NO_OPTIONAL_DATA
};

static ResultOptionalData_t gyroUnscaleFactor = { QFIXEDPOINTPRECISE, { GYRO_UNSCALING_1_FACTOR, GYRO_UNSCALING_2_FACTOR, 0 }}; /* index NOT per Axis - see computation */

/* Output result descriptor for subscribing to uncalibrated gyroscope data */
static SensorDescriptor_t UnCalGyroRequest = {
    SENSOR_GYROSCOPE_UNCALIBRATED,
    DATA_CONVENTION_ANDROID,
    (OSP_OutputReadyCallback_t)UnCalGyroDataResultCallback,
    OSP_NO_NVM_WRITE_CALLBACK,
    OSP_NO_SENSOR_CONTROL_CALLBACK,
    GYRO_OUTPUT_RATES,
    OSP_NO_FLAGS,
    OSP_NO_OPTIONAL_DATA
};

/* Output result descriptor for subscribing to uncalibrated gyroscope data */
static SensorDescriptor_t CalGyroRequest = {
    SENSOR_GYROSCOPE_CALIBRATED,
    DATA_CONVENTION_ANDROID,
    (OSP_OutputReadyCallback_t)CalGyroDataResultCallback,
    OSP_NO_NVM_WRITE_CALLBACK,
    OSP_NO_SENSOR_CONTROL_CALLBACK,
    GYRO_OUTPUT_RATES,
    OSP_NO_FLAGS,
    OSP_NO_OPTIONAL_DATA
};


/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/****************************************************************************************************
 * @fn      EnterCriticalSection/ ExitCriticalSection
 *          Helper routines for thread-safe operation of the FreeMotion Library
 *
 ***************************************************************************************************/
__inline void EnterCriticalSection(void)
{
    os_mut_wait( mutexCritSection, OS_WAIT_FOREVER );
}

__inline void ExitCriticalSection(void)
{
    os_mut_release( mutexCritSection );
}

#ifndef multiplyAndRound                                    // to be able to replace it with optimized implementation */
int16_t multiplyAndRound(int32_t x, int32_t y, uint8_t accutacy)
{
    int64_t temp;
    uint8_t flag;
    temp = (int64_t) x * (int64_t) y;                               // multiply
    flag = temp & 0x8000000000000000 ? 2 : 0;               // set bit 1 if negative
    flag +=  (temp & (1LL << ((accutacy * 2) -1))) ? 1 : 0; // set bit 0 if more than half (for positive) or less than haf (for negative)
    
    temp >>= (accutacy * 2);                              // keep integer portion
    switch (flag) {
        case 0:                                                     // positive less than half - leave as is
        case 3:                                                     // negative more than half - leave as is
            break;
        case 1:
            ++temp;                                                 // positive more than half - round up
            break;
        case 2:
            --temp;                                                 // negative less than half, round down
            break;
    }
    return temp & 0xffff;    
}
#endif

/****************************************************************************************************
 * @fn      UnCalAccelDataResultCallback
 *          Call back for Uncalibrated accelerometer data
 *
 ***************************************************************************************************/
static void UnCalAccelDataResultCallback(OutputSensorHandle_t outputHandle,
    Android_UncalibratedAccelOutputData_t* pOutput)
{
   	struct sh_sensor_broadcast_node hostBuffer;

	hostBuffer.sensorId = SENSOR_ACCELEROMETER_UNCALIBRATED;
	hostBuffer.compression = 0;
    
	hostBuffer.data.uncalibratedSensordata.timeStamp.timeStamp32 = pOutput->TickTimeStampLow;
	hostBuffer.data.uncalibratedSensordata.timeStamp.timeStamp40 = pOutput->TickTimeStampHigh & 0xff;

	hostBuffer.data.uncalibratedSensordata.Data[0] = multiplyAndRound(pOutput->X, accelUnscaleFactor.unscale.ntPreciseUnscaleFactor[0], QFIXEDPOINTPRECISE);
	hostBuffer.data.uncalibratedSensordata.Data[1] = multiplyAndRound(pOutput->Y, accelUnscaleFactor.unscale.ntPreciseUnscaleFactor[1], QFIXEDPOINTPRECISE);
	hostBuffer.data.uncalibratedSensordata.Data[2] = multiplyAndRound(pOutput->Z, accelUnscaleFactor.unscale.ntPreciseUnscaleFactor[2], QFIXEDPOINTPRECISE);
    
	hostBuffer.data.uncalibratedSensordata.Offset[0] = multiplyAndRound(pOutput->X_offset, accelUnscaleFactor.unscale.ntPreciseUnscaleFactor[0], QFIXEDPOINTPRECISE);
	hostBuffer.data.uncalibratedSensordata.Offset[1] = multiplyAndRound(pOutput->Y_offset, accelUnscaleFactor.unscale.ntPreciseUnscaleFactor[1], QFIXEDPOINTPRECISE);
	hostBuffer.data.uncalibratedSensordata.Offset[2] = multiplyAndRound(pOutput->Z_offset, accelUnscaleFactor.unscale.ntPreciseUnscaleFactor[2], QFIXEDPOINTPRECISE);
        

    post_on_boardcast_buffer((uint8_t *)&hostBuffer, sizeof(hostBuffer.data.uncalibratedSensordata), &hostBuffer.data.uncalibratedSensordata.timeStamp);

    if (g_logging & 0x40)  //Uncalibrated data, in Android conventions
    {
        Print_LIPS("RA,%.6f,%.6f,%.6f,%.6f", TOFLT_TIME(pOutput->TimeStamp), TOFLT_PRECISE(pOutput->X),
            TOFLT_PRECISE(pOutput->Y), TOFLT_PRECISE(pOutput->Z));
    }
}


/****************************************************************************************************
 * @fn      CalAccelDataResultCallback
 *          Call back for Calibrated accelerometer data
 *
 ***************************************************************************************************/
static void CalAccelDataResultCallback(OutputSensorHandle_t outputHandle,
    Android_CalibratedAccelOutputData_t* pOutput)
{
    struct sh_sensor_broadcast_node hostBuffer;

	hostBuffer.sensorId = SENSOR_ACCELEROMETER_CALIBRATED;
	hostBuffer.compression = 0;
    
	hostBuffer.data.sensorData.timeStamp.timeStamp32 = pOutput->TickTimeStampLow;
	hostBuffer.data.sensorData.timeStamp.timeStamp40 = pOutput->TickTimeStampHigh & 0xff;

	hostBuffer.data.sensorData.Data[0] = multiplyAndRound(pOutput->X, accelUnscaleFactor.unscale.ntPreciseUnscaleFactor[0], QFIXEDPOINTPRECISE);
	hostBuffer.data.sensorData.Data[1] = multiplyAndRound(pOutput->Y, accelUnscaleFactor.unscale.ntPreciseUnscaleFactor[1], QFIXEDPOINTPRECISE);
	hostBuffer.data.sensorData.Data[2] = multiplyAndRound(pOutput->Z, accelUnscaleFactor.unscale.ntPreciseUnscaleFactor[2], QFIXEDPOINTPRECISE);

    post_on_boardcast_buffer((uint8_t *)&hostBuffer, sizeof(hostBuffer.data.sensorData), &hostBuffer.data.sensorData.timeStamp);

    if (g_logging & 0x40)  //Uncalibrated data, in Android conventions
    {
        Print_LIPS("A,%.6f,%.6f,%.6f,%.6f", TOFLT_TIME(pOutput->TimeStamp), TOFLT_PRECISE(pOutput->X),
            TOFLT_PRECISE(pOutput->Y), TOFLT_PRECISE(pOutput->Z));
    }
}


/****************************************************************************************************
 * @fn      UnCalMagDataResultCallback
 *          Call back for Uncalibrated magnetometer data
 *
 ***************************************************************************************************/
static void UnCalMagDataResultCallback(OutputSensorHandle_t outputHandle,
    Android_UncalibratedMagOutputData_t* pOutput)
{
    struct sh_sensor_broadcast_node hostBuffer;

    hostBuffer.sensorId = SENSOR_MAGNETIC_FIELD_UNCALIBRATED;
    hostBuffer.compression = 0;

    hostBuffer.data.uncalibratedSensordata.timeStamp.timeStamp32  = pOutput->TickTimeStampLow;
    hostBuffer.data.uncalibratedSensordata.timeStamp.timeStamp40 = pOutput->TickTimeStampHigh & 0xff;
     
    hostBuffer.data.uncalibratedSensordata.Data[0] = multiplyAndRound(pOutput->X, magUnscaleFactor.unscale.ntExtendedUnscaleFactor[0], QFIXEDPOINTEXTENDED);
    hostBuffer.data.uncalibratedSensordata.Data[1] = multiplyAndRound(pOutput->Y, magUnscaleFactor.unscale.ntExtendedUnscaleFactor[1], QFIXEDPOINTEXTENDED);
    hostBuffer.data.uncalibratedSensordata.Data[2] = multiplyAndRound(pOutput->Z, magUnscaleFactor.unscale.ntExtendedUnscaleFactor[2], QFIXEDPOINTEXTENDED);

    hostBuffer.data.uncalibratedSensordata.Offset[0] = multiplyAndRound(pOutput->X_hardIron_offset, magUnscaleFactor.unscale.ntExtendedUnscaleFactor[0], QFIXEDPOINTEXTENDED);
    hostBuffer.data.uncalibratedSensordata.Offset[1] = multiplyAndRound(pOutput->Y_hardIron_offset, magUnscaleFactor.unscale.ntExtendedUnscaleFactor[1], QFIXEDPOINTEXTENDED);
    hostBuffer.data.uncalibratedSensordata.Offset[2] = multiplyAndRound(pOutput->Z_hardIron_offset, magUnscaleFactor.unscale.ntExtendedUnscaleFactor[2], QFIXEDPOINTEXTENDED);

    post_on_boardcast_buffer((uint8_t *)&hostBuffer, sizeof(hostBuffer.data.uncalibratedSensordata), &hostBuffer.data.uncalibratedSensordata.timeStamp);

    if (g_logging & 0x40)  //Uncalibrated data, in Android conventions
    {
        Print_LIPS("RM,%.6f,%.6f,%.6f,%.6f", TOFLT_TIME(pOutput->TimeStamp), TOFLT_EXTENDED(pOutput->X),
            TOFLT_EXTENDED(pOutput->Y), TOFLT_EXTENDED(pOutput->Z));
    }
}

/****************************************************************************************************
 * @fn      CalMagDataResultCallback
 *          Call back for Calibrated magnetometer data
 *
 ***************************************************************************************************/
static void CalMagDataResultCallback(OutputSensorHandle_t outputHandle,
    Android_CalibratedMagOutputData_t* pOutput)
{
   struct sh_sensor_broadcast_node hostBuffer;

    hostBuffer.sensorId = SENSOR_MAGNETIC_FIELD_CALIBRATED;
    hostBuffer.compression = 0;

    hostBuffer.data.sensorData.timeStamp.timeStamp32  = pOutput->TickTimeStampLow;
    hostBuffer.data.sensorData.timeStamp.timeStamp40 = pOutput->TickTimeStampHigh & 0xff;
     
    hostBuffer.data.sensorData.Data[0] = multiplyAndRound(pOutput->X, magUnscaleFactor.unscale.ntExtendedUnscaleFactor[0], QFIXEDPOINTEXTENDED);
    hostBuffer.data.sensorData.Data[1] = multiplyAndRound(pOutput->Y, magUnscaleFactor.unscale.ntExtendedUnscaleFactor[1], QFIXEDPOINTEXTENDED);
    hostBuffer.data.sensorData.Data[2] = multiplyAndRound(pOutput->Z, magUnscaleFactor.unscale.ntExtendedUnscaleFactor[2], QFIXEDPOINTEXTENDED);

    post_on_boardcast_buffer((uint8_t *)&hostBuffer, sizeof(hostBuffer.data.sensorData), &hostBuffer.data.sensorData.timeStamp);
    
    if (g_logging & 0x40)  //Uncalibrated data, in Android conventions
    {
        Print_LIPS("M,%.6f,%.6f,%.6f,%.6f", TOFLT_TIME(pOutput->TimeStamp), TOFLT_EXTENDED(pOutput->X),
            TOFLT_EXTENDED(pOutput->Y), TOFLT_EXTENDED(pOutput->Z));
    }
}


/****************************************************************************************************
 * @fn      UnCalGyroDataResultCallback
 *          Call back for Uncalibrated gyroscope data
 *
 ***************************************************************************************************/
static void UnCalGyroDataResultCallback(OutputSensorHandle_t outputHandle,
    Android_UncalibratedGyroOutputData_t* pOutput)
{
   	struct sh_sensor_broadcast_node hostBuffer;

	hostBuffer.sensorId = SENSOR_GYROSCOPE_UNCALIBRATED;
	hostBuffer.compression = 0;
    
	hostBuffer.data.uncalibratedSensordata.timeStamp.timeStamp32 = pOutput->TickTimeStampLow;
	hostBuffer.data.uncalibratedSensordata.timeStamp.timeStamp40 = pOutput->TickTimeStampHigh & 0xff;


	hostBuffer.data.uncalibratedSensordata.Data[0] = multiplyAndRound(
        ((int64_t) pOutput->X * (int64_t) gyroUnscaleFactor.unscale.ntPreciseUnscaleFactor[0]) >> QFIXEDPOINTPRECISE,
        gyroUnscaleFactor.unscale.ntPreciseUnscaleFactor[1],
        QFIXEDPOINTPRECISE);

	hostBuffer.data.uncalibratedSensordata.Data[1] = multiplyAndRound(
        ((int64_t) pOutput->Y * (int64_t) gyroUnscaleFactor.unscale.ntPreciseUnscaleFactor[0]) >> QFIXEDPOINTPRECISE,
        gyroUnscaleFactor.unscale.ntPreciseUnscaleFactor[1],
        QFIXEDPOINTPRECISE);

	hostBuffer.data.uncalibratedSensordata.Data[2] = multiplyAndRound(
        ((int64_t) pOutput->Z * (int64_t) gyroUnscaleFactor.unscale.ntPreciseUnscaleFactor[0]) >> QFIXEDPOINTPRECISE,
        gyroUnscaleFactor.unscale.ntPreciseUnscaleFactor[1],
        QFIXEDPOINTPRECISE);


	hostBuffer.data.uncalibratedSensordata.Offset[0] = multiplyAndRound(
        ((int64_t) pOutput->X_drift_offset * (int64_t) gyroUnscaleFactor.unscale.ntPreciseUnscaleFactor[0]) >> QFIXEDPOINTPRECISE,
        gyroUnscaleFactor.unscale.ntPreciseUnscaleFactor[1],
        QFIXEDPOINTPRECISE);

	hostBuffer.data.uncalibratedSensordata.Offset[1] = multiplyAndRound(
        ((int64_t) pOutput->Y_drift_offset * (int64_t) gyroUnscaleFactor.unscale.ntPreciseUnscaleFactor[0]) >> QFIXEDPOINTPRECISE,
        gyroUnscaleFactor.unscale.ntPreciseUnscaleFactor[1],
        QFIXEDPOINTPRECISE);

	hostBuffer.data.uncalibratedSensordata.Offset[2] = multiplyAndRound(
        ((int64_t) pOutput->Z_drift_offset * (int64_t) gyroUnscaleFactor.unscale.ntPreciseUnscaleFactor[0]) >> QFIXEDPOINTPRECISE,
        gyroUnscaleFactor.unscale.ntPreciseUnscaleFactor[1],
        QFIXEDPOINTPRECISE);
       
    post_on_boardcast_buffer((uint8_t *)&hostBuffer, sizeof(hostBuffer.data.uncalibratedSensordata), &hostBuffer.data.uncalibratedSensordata.timeStamp);
    
    if (g_logging & 0x40)  //Uncalibrated data, in Android conventions
    {
        Print_LIPS("RG,%.6f,%.6f,%.6f,%.6f", TOFLT_TIME(pOutput->TimeStamp), TOFLT_PRECISE(pOutput->X),
            TOFLT_PRECISE(pOutput->Y), TOFLT_PRECISE(pOutput->Z));
    }
}


/****************************************************************************************************
 * @fn      CalGyroDataResultCallback
 *          Call back for Calibrated gyroscope data
 *
 ***************************************************************************************************/
static void CalGyroDataResultCallback(OutputSensorHandle_t outputHandle,
    Android_CalibratedGyroOutputData_t* pOutput)
{
   	struct sh_sensor_broadcast_node hostBuffer;

	hostBuffer.sensorId = SENSOR_GYROSCOPE_CALIBRATED;
	hostBuffer.compression = 0;
    
	hostBuffer.data.sensorData.timeStamp.timeStamp32 = pOutput->TickTimeStampLow;
	hostBuffer.data.sensorData.timeStamp.timeStamp40 = pOutput->TickTimeStampHigh & 0xff;


	hostBuffer.data.sensorData.Data[0] = multiplyAndRound(
        ((int64_t) pOutput->X * (int64_t) gyroUnscaleFactor.unscale.ntPreciseUnscaleFactor[0]) >> QFIXEDPOINTPRECISE,
        gyroUnscaleFactor.unscale.ntPreciseUnscaleFactor[1],
        QFIXEDPOINTPRECISE);

	hostBuffer.data.sensorData.Data[1] = multiplyAndRound(
        ((int64_t) pOutput->Y * (int64_t) gyroUnscaleFactor.unscale.ntPreciseUnscaleFactor[0]) >> QFIXEDPOINTPRECISE,
        gyroUnscaleFactor.unscale.ntPreciseUnscaleFactor[1],
        QFIXEDPOINTPRECISE);

	hostBuffer.data.sensorData.Data[2] = multiplyAndRound(
        ((int64_t) pOutput->Z * (int64_t) gyroUnscaleFactor.unscale.ntPreciseUnscaleFactor[0]) >> QFIXEDPOINTPRECISE,
        gyroUnscaleFactor.unscale.ntPreciseUnscaleFactor[1],
        QFIXEDPOINTPRECISE);

    post_on_boardcast_buffer((uint8_t *)&hostBuffer, sizeof(hostBuffer.data.sensorData), &hostBuffer.data.sensorData.timeStamp);


    if (g_logging & 0x40)  //Uncalibrated data, in Android conventions
    {
        Print_LIPS("G,%.6f,%.6f,%.6f,%.6f", TOFLT_TIME(pOutput->TimeStamp), TOFLT_PRECISE(pOutput->X),
            TOFLT_PRECISE(pOutput->Y), TOFLT_PRECISE(pOutput->Z));
    }
}


/****************************************************************************************************
 * @fn      stepCounterOutputCallback
 *          Call back for Step Detector step counter
 *
 ***************************************************************************************************/
static void stepCounterOutputCallback(OutputSensorHandle_t OutputHandle,
    Android_StepCounterOutputData_t* pOutput)
{
    struct sh_sensor_broadcast_node hostBuffer;	
    hostBuffer.sensorId = SENSOR_STEP_COUNTER;
    hostBuffer.compression = 0;

    hostBuffer.data.stepCounterData.timeStamp.timeStamp32 = pOutput->TickTimeStampLow;
    hostBuffer.data.stepCounterData.timeStamp.timeStamp40 = pOutput->TickTimeStampHigh;
    
    hostBuffer.data.stepCounterData.numSteps = pOutput->StepCount;
    
    post_on_boardcast_buffer((uint8_t *)&hostBuffer, sizeof(hostBuffer.data.stepCounterData), &hostBuffer.data.stepCounterData.timeStamp);

    Print_LIPS("STC,%+03.4f,%d,0", TOFLT_TIME(pOutput->TimeStamp), pOutput->StepCount);
}


/****************************************************************************************************
 * @fn      stepDetectorResultCallback
 *          Call back for Step Detector results
 *
 ***************************************************************************************************/
static void stepDetectorOutputCallback(OutputSensorHandle_t OutputHandle,
    Android_StepCounterOutputData_t* pOutput)
{
    struct sh_sensor_broadcast_node hostBuffer;	
    hostBuffer.sensorId = SENSOR_STEP_DETECTOR;
    hostBuffer.compression = 0;

    hostBuffer.data.stepDetectorData.timeStamp.timeStamp32 = pOutput->TickTimeStampLow;
    hostBuffer.data.stepDetectorData.timeStamp.timeStamp40 = pOutput->TickTimeStampHigh;
        
    post_on_boardcast_buffer((uint8_t *)&hostBuffer, sizeof(hostBuffer.data.stepDetectorData), &hostBuffer.data.stepDetectorData.timeStamp);

    Print_LIPS("STC,%+03.4f,%d,0", TOFLT_TIME(pOutput->TimeStamp), pOutput->StepCount);
}


/****************************************************************************************************
 * @fn      sigMotionOutputCallback
 *          Call back for Significant Motion results
 *
 ***************************************************************************************************/
static void sigMotionOutputCallback(OutputSensorHandle_t outputHandle,
    Android_SignificantMotionOutputData_t* pOutput)
{
    Print_LIPS("SM,%+03.4f,%d",TOFLT_TIME(pOutput->TimeStamp), true);
}


/****************************************************************************************************
 * @fn      HandleSensorData
 *          Handles sensor input data and feeds it to the sensor algorithms
 *
 ***************************************************************************************************/
static void HandleSensorData( MessageBuffer *pRcvMsg )
{
    osp_status_t status;
    TriAxisSensorRawData_t sensorData;

    switch(pRcvMsg->msgId)
    {
    case MSG_ACC_DATA:
        sensorData.Data[0] = pRcvMsg->msg.msgAccelData.X;
        sensorData.Data[1] = pRcvMsg->msg.msgAccelData.Y;
        sensorData.Data[2] = pRcvMsg->msg.msgAccelData.Z;
        sensorData.TimeStamp = pRcvMsg->msg.msgAccelData.timeStamp;
        status = OSP_SetData(_AccHandle, &sensorData);
        ASF_assert(status == OSP_STATUS_OK);
        break;

    case MSG_MAG_DATA:
        sensorData.Data[0] = pRcvMsg->msg.msgMagData.X;
        sensorData.Data[1] = pRcvMsg->msg.msgMagData.Y;
        sensorData.Data[2] = pRcvMsg->msg.msgMagData.Z;
        sensorData.TimeStamp = pRcvMsg->msg.msgMagData.timeStamp;
        status = OSP_SetData(_MagHandle, &sensorData);
        ASF_assert(status == OSP_STATUS_OK);
        break;

    case MSG_GYRO_DATA:
        sensorData.Data[0] = pRcvMsg->msg.msgGyroData.X;
        sensorData.Data[1] = pRcvMsg->msg.msgGyroData.Y;
        sensorData.Data[2] = pRcvMsg->msg.msgGyroData.Z;
        sensorData.TimeStamp = pRcvMsg->msg.msgGyroData.timeStamp;
        status = OSP_SetData(_GyroHandle, &sensorData);
        ASF_assert(status == OSP_STATUS_OK);
        break;

    default:
        D1_printf("ALG: Bad message: %d\r\n", pRcvMsg->msgId);
        break;
    }

}


/****************************************************************************************************
 * @fn      SendBgTrigger
 *          Sends triggers to the algorithm background task to do processing
 *
 ***************************************************************************************************/
static void SendBgTrigger( void )
{
    MessageBuffer *pSendMsg = NULLP;
    ASF_assert( ASFCreateMessage( MSG_TRIG_ALG_BG, sizeof(MsgNoData), &pSendMsg ) == ASF_OK );
    ASFSendMessage( ALG_BG_TASK_ID, pSendMsg );
}


/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/****************************************************************************************************
 * @fn      AlgorithmTask
 *          This task is responsible for running the sensor algorithms on the incoming sensor
 *          data (could be raw or filtered) and processing output results
 *
 * @param   none
 *
 * @return  none
 *
 ***************************************************************************************************/
ASF_TASK  void AlgorithmTask ( ASF_TASK_ARG )
{
    MessageBuffer *rcvMsg = NULLP;
    osp_status_t OSP_Status;

    OSP_GetVersion(&version);
    D1_printf("OSP Version: %s\r\n", version->VersionString);

    OSP_Status = OSP_Initialize(&gSystemDesc);
    ASF_assert_msg(OSP_STATUS_OK == OSP_Status, "SensorManager: OSP_Initialize Failed");

    // Register the input sensors
    OSP_Status = OSP_RegisterInputSensor(&_AccSensDesc, &_AccHandle);
    ASF_assert_msg(OSP_STATUS_OK == OSP_Status, "SensorManager: OSP_RegisterSensor (accel) Failed");

    OSP_Status = OSP_RegisterInputSensor(&_MagSensDesc, &_MagHandle);
    ASF_assert_msg(OSP_STATUS_OK == OSP_Status, "SensorManager: OSP_RegisterSensor (mag) Failed");

    OSP_Status = OSP_RegisterInputSensor(&_GyroSensDesc, &_GyroHandle);
    ASF_assert_msg(OSP_STATUS_OK == OSP_Status, "SensorManager: OSP_RegisterSensor (gyro) Failed");


    while (1)
    {
        ASFReceiveMessage( ALGORITHM_TASK_ID, &rcvMsg );
        switch (rcvMsg->msgId)
        {
        case MSG_MAG_DATA:
            SendBgTrigger();
        case MSG_ACC_DATA:
        case MSG_GYRO_DATA:
            HandleSensorData(rcvMsg);
            do
            {
                OSP_Status = OSP_DoForegroundProcessing();
                ASF_assert(OSP_Status != OSP_STATUS_ERROR);
            } while(OSP_Status != OSP_STATUS_IDLE)
                ; //keep doing foreground computation until its finished
            break;
        case MSG_SENSOR_ENABLE_DATA:
            switch (rcvMsg->msg.msgSensorEnable.sensorId) {
            case SENSOR_ACCELEROMETER_UNCALIBRATED:
				if (rcvMsg->msg.msgSensorEnable.enabled) {
                    if (_unCalAccelHandle == NULL) {
                        OSP_Status =  OSP_SubscribeOutputSensor(&UnCalAccelRequest, &_unCalAccelHandle);
                        ASF_assert_msg(OSP_STATUS_OK == OSP_Status, "SensorManager: OSP_SubscribeResult (SENSOR_ACCELEROMETER_UNCALIBRATED) Failed");
                    }
                } else {
                    if (_unCalAccelHandle != NULL) {
                        OSP_UnsubscribeOutputSensor(&_unCalAccelHandle);
                    }
                }
                break;
            case SENSOR_ACCELEROMETER_CALIBRATED:
				if (rcvMsg->msg.msgSensorEnable.enabled) {
                    if (_calAccelHandle == NULL) {
                        OSP_Status =  OSP_SubscribeOutputSensor(&CalAccelRequest, &_calAccelHandle);
                        ASF_assert_msg(OSP_STATUS_OK == OSP_Status, "SensorManager: OSP_SubscribeResult (SENSOR_ACCELEROMETER_CALIBRATED) Failed");
                    }
                } else {
                    if (_calAccelHandle != NULL) {
                        OSP_UnsubscribeOutputSensor(&_calAccelHandle);
                    }
                }
                break;
            case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
				if (rcvMsg->msg.msgSensorEnable.enabled) {
                    if (_unCalMagHandle == NULL) {
                        OSP_Status =  OSP_SubscribeOutputSensor(&UnCalMagRequest, &_unCalMagHandle);
                        ASF_assert_msg(OSP_STATUS_OK == OSP_Status, "SensorManager: OSP_SubscribeResult (SENSOR_MAGNETIC_FIELD_UNCALIBRATED) Failed");
                    }
                } else {
                    if (_unCalMagHandle != NULL) {
                        OSP_UnsubscribeOutputSensor(&_unCalMagHandle);
                    }
                }
                break;
            case SENSOR_MAGNETIC_FIELD_CALIBRATED:
				if (rcvMsg->msg.msgSensorEnable.enabled) {
                    if (_calMagHandle == NULL) {
                        OSP_Status =  OSP_SubscribeOutputSensor(&CalMagRequest, &_calMagHandle);
                        ASF_assert_msg(OSP_STATUS_OK == OSP_Status, "SensorManager: OSP_SubscribeResult (SENSOR_MAGNETIC_FIELD_CALIBRATED) Failed");
                    }
                } else {
                    if (_calMagHandle != NULL) {
                        OSP_UnsubscribeOutputSensor(&_calMagHandle);
                    }
                }
                break;
            case SENSOR_GYROSCOPE_UNCALIBRATED:
				if (rcvMsg->msg.msgSensorEnable.enabled) {
                    if (_unCalGyroHandle == NULL) {
                        OSP_Status =  OSP_SubscribeOutputSensor(&UnCalGyroRequest, &_unCalGyroHandle);
                        ASF_assert_msg(OSP_STATUS_OK == OSP_Status, "SensorManager: OSP_SubscribeResult (SENSOR_GYROSCOPE_UNCALIBRATED) Failed");

                    }
                } else {
                    if (_unCalGyroHandle != NULL) {
                        OSP_UnsubscribeOutputSensor(&_unCalGyroHandle);
                    }
                }
                break;
            case SENSOR_GYROSCOPE_CALIBRATED:
				if (rcvMsg->msg.msgSensorEnable.enabled) {
                    if (_calGyroHandle == NULL) {
                        OSP_Status =  OSP_SubscribeOutputSensor(&CalGyroRequest, &_calGyroHandle);
                        ASF_assert_msg(OSP_STATUS_OK == OSP_Status, "SensorManager: OSP_SubscribeResult (SENSOR_GYROSCOPE_CALIBRATED) Failed");
                    }
                } else {
                    if (_calGyroHandle != NULL) {
                        OSP_UnsubscribeOutputSensor(&_calGyroHandle);
                    }
                }
                break;
            case SENSOR_STEP_COUNTER:
				if (rcvMsg->msg.msgSensorEnable.enabled) {
                    if (_stepCounterHandle == NULL) {
                        OSP_Status =  OSP_SubscribeOutputSensor(&stepCounterRequest, &_stepCounterHandle);
                        ASF_assert_msg(OSP_STATUS_OK == OSP_Status, "SensorManager: OSP_SubscribeResult (SENSOR_STEP_COUNTER) Failed");
                    }
                } else {
                    if (_stepCounterHandle != NULL) {
                        OSP_UnsubscribeOutputSensor(&_stepCounterHandle);
                    }
                }
                break;
            case SENSOR_STEP_DETECTOR:
				if (rcvMsg->msg.msgSensorEnable.enabled) {
                    if (_stepDetectorHandle == NULL) {
                        OSP_Status =  OSP_SubscribeOutputSensor(&stepDetectorRequest, &_stepDetectorHandle);
                        ASF_assert_msg(OSP_STATUS_OK == OSP_Status, "SensorManager: OSP_SubscribeResult (SENSOR_STEP_DETECTOR) Failed");
                    }
                } else {
                    if (_stepDetectorHandle != NULL) {
                        OSP_UnsubscribeOutputSensor(&_stepDetectorHandle);
                    }
                }
                break;
            case SENSOR_CONTEXT_DEVICE_MOTION:
				if (rcvMsg->msg.msgSensorEnable.enabled) {
                    if (_sigMotionHandle == NULL) {
                        OSP_Status =  OSP_SubscribeOutputSensor(&sigMotionRequest, &_sigMotionHandle);
                        ASF_assert_msg(OSP_STATUS_OK == OSP_Status, "SensorManager: OSP_SubscribeResult (SENSOR_CONTEXT_DEVICE_MOTION) Failed");
                    }
                } else {
                    if (_sigMotionHandle != NULL) {
                        OSP_UnsubscribeOutputSensor(&_sigMotionHandle);
                    }
                }
                break;
            }
            break;
        default:
            /* Unhandled messages */
            D1_printf("Alg-FG:!!!UNHANDLED MESSAGE:%d!!!\r\n", rcvMsg->msgId);
            break;
        }
    }
}


/****************************************************************************************************
 * @fn      AlgBackGndTask
 *          This task is responsible for running the background routines (e.g. calibration)
 *
 * @param   none
 *
 * @return  none
 *
 ***************************************************************************************************/
ASF_TASK  void AlgBackGndTask ( ASF_TASK_ARG )
{
    MessageBuffer *rcvMsg = NULLP;

    while (1)
    {
        ASFReceiveMessage( ALG_BG_TASK_ID, &rcvMsg );
        switch (rcvMsg->msgId)
        {
        case MSG_TRIG_ALG_BG:
            while(OSP_DoBackgroundProcessing() != OSP_STATUS_IDLE)
                ; //background compute. Note that it's safe to call background processing more often than needed
            break;

        default:
            /* Unhandled messages */
            D1_printf("Alg-BG:!!!UNHANDLED MESSAGE:%d!!!\r\n", rcvMsg->msgId);
            break;
        }
    }
}


/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
