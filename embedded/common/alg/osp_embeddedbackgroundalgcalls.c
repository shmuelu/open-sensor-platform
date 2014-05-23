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
#include "osp_embeddedbackgroundalgcalls.h"
#include "osp-alg-types.h"

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
static OSP_BackgroundAlgResultCallback_t _fpBackgroundDataCallback = NULL;

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
 * @fn      OSPBackgroundAlg_InitializeAlgorithms
 *          Call to initialize the algorithms implementation.
 *
 ***************************************************************************************************/
void OSPBackgroundAlg_InitializeAlgorithms(void){

}


/****************************************************************************************************
 * @fn      OSPBackgroundAlg_ResetAlgorithms
 *          Call this to reset the algorithms to initial startup state
 *
 ***************************************************************************************************/
void OSPBackgroundAlg_ResetAlgorithms(void){

}


/****************************************************************************************************
 * @fn      OSPBackgroundAlg_DestroyAlgorithms
 *          Call this function before exit to shutdown the algorithms properly
 *
 ***************************************************************************************************/
void OSPBackgroundAlg_DestroyAlgorithms(void){

}

/****************************************************************************************************
 * @fn      OSPBackgroundAlg_SetStoredResult
 *          API to feed stored background result data into the background algorithms
 *          (for example, initial stored calibration values)
 *
 ***************************************************************************************************/
void OSPBackgroundAlg_SetStoredResult(OSP_BackgroundAlgResultType_t resultType, OSP_BackgroundAlgResult_t * pCal){

}

/****************************************************************************************************
 * @fn      OSPBackgroundAlg_SetAccelerometerMeasurement
 *          API to feed accelerometer data into the background algorithms
 *
 ***************************************************************************************************/
void OSPBackgroundAlg_SetAccelerometerMeasurement(const NTTIME timeInSeconds, const NTPRECISE measurementInMetersPerSecondSquare[NUM_ACCEL_AXES]){

    //No background algorithms implemented at this time
}

/****************************************************************************************************
 * @fn      OSPBackgroundAlg_RegisterBackgroundResultCallback
 *          Register for all background algorithm results
 *
 ***************************************************************************************************/
void OSPBackgroundAlg_RegisterBackgroundResultCallback(OSP_BackgroundAlgResultCallback_t fpCallback) {
    _fpBackgroundDataCallback = fpCallback;
}

/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
