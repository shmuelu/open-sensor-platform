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
#if !defined (OSP_ALG_TYPES)
#define   OSP_ALG_TYPES

/*-------------------------------------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
\*-------------------------------------------------------------------------------------------------*/
#include <stddef.h>
#include "osp-types.h"
#include "osp-sensors.h"
#include "osp-fixedpoint-types.h"

/*-------------------------------------------------------------------------------------------------*\
 |    C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/
#define NUM_ACCEL_AXES                  (3)
#define NUM_TRIAXIS_SENSOR_AXES         (3)

/*-------------------------------------------------------------------------------------------------*\
 |    T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
//! Enumeration typedef for segment type
typedef enum EStepSegmentType {
    firstStep,
    midStep,
    lastStep
} EStepSegmentType;

//! Struct definition for a step segment
typedef struct {
    //store start/stop times of the last step
    NTTIME startTime;
    NTTIME stopTime;
    EStepSegmentType type;
} StepSegment_t;

//struct for defining a step
typedef struct StepDataOSP_t{
    NTTIME startTime;
    NTTIME stopTime;
    osp_float_t stepFrequency;
    uint32_t numStepsTotal;
    uint32_t numStepsSinceWalking;
} StepDataOSP_t;

//! Enumeration for calibration sensor type
typedef enum {
       OSPCalTypeAccelerometer = 0,
       OSPCalTypeMagnetometer = 1,
       OSPCalTypeGyroscope = 2,
       OSPCalTypeGyroscopeExternal = 3
} OSPEnumCalType_t;

//! Enumeration for calibration complexity type
typedef enum {
       OSPCalibratorDefault = 0,
       OSPCalibratorInit = 1,
       OSPCalibratorRegular = 2,
       OSPCalibratorPremium = 3
} OSPEnumCalSource_t;

//! struct for storing all relevant calibration information
typedef struct {
    int32_t scale[NUM_TRIAXIS_SENSOR_AXES];
    int32_t skew[NUM_TRIAXIS_SENSOR_AXES];
    int32_t offset[NUM_TRIAXIS_SENSOR_AXES];
    int32_t rotation[NUM_TRIAXIS_SENSOR_AXES];
    int32_t quality[NUM_TRIAXIS_SENSOR_AXES*(NUM_TRIAXIS_SENSOR_AXES+1)];
    OSPEnumCalType_t sensortype;
    OSPEnumCalSource_t calsource;
    uint16_t crc;
} OSP_CalStorageStruct_t;

//! struct for storing any type of background alg result
typedef union {
    OSP_CalStorageStruct_t calstruct;
} OSP_BackgroundAlgResult_t;

//! enum for distinguishing between different types of background alg results
typedef enum {
    BKGALG_ACCELEROMETER_CALIBRATION          = 0,
    BKGALG_MAGNETOMETER_CALIBRATION           = 1,
    BKGALG_GYROSCOPE_CALIBRATION              = 2,
    BKGALG_ENUM_COUNT
} OSP_BackgroundAlgResultType_t;


typedef void (*OSP_StepSegmentResultCallback_t)(StepSegment_t * segment);
typedef void (*OSP_StepResultCallback_t)(StepDataOSP_t* stepData);
typedef void (*OSP_EventResultCallback_t)(NTTIME * eventTime);
typedef void (*OSP_BackgroundAlgResultCallback_t)(OSP_BackgroundAlgResultType_t resultType, const NTTIME time, OSP_BackgroundAlgResult_t * cal, osp_bool_t storeResult);
typedef void (*OSP_CalibratedSensorCallback_t)(enum SensorType_t sensorType, const NTTIME time, const int32_t calibratedData[NUM_TRIAXIS_SENSOR_AXES]);

/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/


#endif /* OSP_ALG_TYPES */
/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
