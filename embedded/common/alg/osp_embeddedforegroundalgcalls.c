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
#include "osp_embeddedforegroundalgcalls.h"
#include "stepdetector.h"
#include "signalgenerator.h"
#include "significantmotiondetector.h"
#include "osp-alg-types.h"
#include "osp-sensors.h"

/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
static OSP_StepResultCallback_t _fpStepResultCallback = NULL;
static OSP_StepSegmentResultCallback_t _fpStepSegmentResultCallback = NULL;
static OSP_EventResultCallback_t _fpSigMotCallback = NULL;
static OSP_CalibratedSensorCallback_t _fpCalibratedSensorDataCallback = NULL;

/*-------------------------------------------------------------------------------------------------*\
 |    F O R W A R D   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   A P I   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/****************************************************************************************************
 * @fn      OSPForegroundAlg_InitializeAlgorithms
 *          Call to initialize the algorithms implementation.
 *
 ***************************************************************************************************/
void OSPForegroundAlg_InitializeAlgorithms(void){
    //Initialize signal generator
    SignalGenerator_Init();

    //Initialize algs
    SignificantMotDetector_Init(_fpSigMotCallback);
    StepDetector_Init(_fpStepResultCallback, _fpStepSegmentResultCallback);
}


/****************************************************************************************************
 * @fn      OSPForegroundAlg_ResetAlgorithms
 *          Call this to reset the algorithms to initial startup state
 *
 ***************************************************************************************************/
void OSPForegroundAlg_ResetAlgorithms(void){
    SignalGenerator_Init();
    StepDetector_Reset();
    SignificantMotDetector_Reset();
}


/****************************************************************************************************
 * @fn      OSPForegroundAlg_DestroyAlgorithms
 *          Call this function before exit to shutdown the algorithms properly
 *
 ***************************************************************************************************/
void OSPForegroundAlg_DestroyAlgorithms(void){
    StepDetector_CleanUp();
    SignificantMotDetector_CleanUp();
}

/****************************************************************************************************
 * @fn      OSPForegroundAlg_SetBackgroundResult
 *          API to feed background algorithm result into the foreground algorithms
 *
 ***************************************************************************************************/
void OSPForegroundAlg_SetBackgroundResult(OSP_BackgroundAlgResultType_t resultType, const NTTIME time, OSP_BackgroundAlgResult_t * pCal){

}

/****************************************************************************************************
 * @fn      OSPForegroundAlg_SetAccelerometerMeasurement
 *          API to feed accelerometer data into the foreground algorithms
 *
 ***************************************************************************************************/
void OSPForegroundAlg_SetAccelerometerMeasurement(const NTTIME timeInSeconds, const NTPRECISE measurementInMetersPerSecondSquare[NUM_ACCEL_AXES]){
    //convert sensor data to floating point
    osp_float_t measurementFloat[NUM_ACCEL_AXES];
    osp_float_t measurementFiltered[NUM_ACCEL_AXES];
    //NTPRECISE calibratedAccelerometerMeasurement[NUM_ACCEL_AXES];
    NTTIME filterTime = timeInSeconds;

    measurementFloat[0] = TOFLT_PRECISE(measurementInMetersPerSecondSquare[0]);
    measurementFloat[1] = TOFLT_PRECISE(measurementInMetersPerSecondSquare[1]);
    measurementFloat[2] = TOFLT_PRECISE(measurementInMetersPerSecondSquare[2]);

    //update signal generator with raw accelerometer data
    if(SignalGenerator_SetAccelerometerData(measurementFloat, measurementFiltered)){

        filterTime -= SIGNAL_GENERATOR_DELAY;

        //update significant motion alg
        SignificantMotDetector_SetFilteredAccelerometerMeasurement(filterTime,
                                                                   measurementFiltered);
        //update step detector alg
        StepDetector_SetFilteredAccelerometerMeasurement(filterTime, measurementFiltered);
    }

    //compute calibrated data and send to listeners
    //ApplyCalibration(calibratedAccelerometerMeasurement,
    //                 &_foregroundCalibrations[OSPCalTypeAccelerometer],
    //                 measurementInMetersPerSecondSquare);
   
    if (_fpCalibratedSensorDataCallback) {
        _fpCalibratedSensorDataCallback(SENSOR_ACCELEROMETER, timeInSeconds,
                                        measurementInMetersPerSecondSquare);
    }
    
}


/****************************************************************************************************
 * @fn      OSPForegroundAlg_RegisterCalibratedSensorDataCallback
 *          Register for all calibrated sensor data callbacks with the algorithms
 *
 ***************************************************************************************************/
void OSPForegroundAlg_RegisterCalibratedSensorDataCallback(OSP_CalibratedSensorCallback_t fpCallback) {
    _fpCalibratedSensorDataCallback = fpCallback;
}

/****************************************************************************************************
 * @fn      OSPForegroundAlg_RegisterStepSegmentCallback
 *          Register step segment call back with the algorithms
 *
 ***************************************************************************************************/
void OSPForegroundAlg_RegisterStepSegmentCallback(OSP_StepSegmentResultCallback_t fpCallback){
    _fpStepSegmentResultCallback = fpCallback;
    StepDetector_Init(_fpStepResultCallback, _fpStepSegmentResultCallback);
}


/****************************************************************************************************
 * @fn      OSPForegroundAlg_RegisterStepCallback
 *          Register step result call back with the algorithms
 *
 ***************************************************************************************************/
void OSPForegroundAlg_RegisterStepCallback(OSP_StepResultCallback_t fpCallback){
    _fpStepResultCallback = fpCallback;
    StepDetector_Init(_fpStepResultCallback, _fpStepSegmentResultCallback);
}

/****************************************************************************************************
 * @fn      OSPForegroundAlg_RegisterSignificantMotionCallback
 *          Register significant motion call back with the algorithms
 *
 ***************************************************************************************************/
void OSPForegroundAlg_RegisterSignificantMotionCallback(OSP_EventResultCallback_t fpCallback){
    _fpSigMotCallback = fpCallback;
    SignificantMotDetector_Init(_fpSigMotCallback);
}

/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
