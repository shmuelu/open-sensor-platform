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
#include "stepdetector.h"
#include "stepsegmenter.h"
#include "osp-alg-types.h"
#include <math.h>

/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
//struct for containing step generation data
typedef struct {
    //step segmenter
    StepSegmenter_t stepSegmenter;

    //step data
    StepDataOSP_t step;

    //first step time
    NTTIME startWalkTime;

    //callback variables
    OSP_StepResultCallback_t stepResultReadyCallback;
    OSP_StepSegmentResultCallback_t stepSegmentResultReadyCallback;
} StepDetector_t;

/*-------------------------------------------------------------------------------------------------*\
 |    S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
static StepDetector_t stepDetectData;

/*-------------------------------------------------------------------------------------------------*\
 |    F O R W A R D   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/****************************************************************************************************
 * @fn      SetNewStepSegment
 *          <brief>
 *
 ***************************************************************************************************/
static void SetNewStepSegment(StepSegment_t *segment)
{
    NTTIME dt;

    //Check for start of walk sequence
    if (segment->type == firstStep) {
        stepDetectData.startWalkTime = segment->startTime;
        stepDetectData.step.numStepsSinceWalking = 0;
    }

    //Set times and increment counters
    stepDetectData.step.startTime = segment->startTime;
    stepDetectData.step.stopTime = segment->stopTime;
    stepDetectData.step.numStepsTotal++;
    stepDetectData.step.numStepsSinceWalking++;

    //Estimate step frequency and length
    dt = segment->stopTime - stepDetectData.startWalkTime;
    stepDetectData.step.stepFrequency = ((osp_float_t)stepDetectData.step.numStepsSinceWalking) / TOFLT_TIME(dt);

    //Callback to subscribers if any
    if (stepDetectData.stepResultReadyCallback) {
        stepDetectData.stepResultReadyCallback(&stepDetectData.step);
    }
    if (stepDetectData.stepSegmentResultReadyCallback) {
        stepDetectData.stepSegmentResultReadyCallback(segment);
    }
}


/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/****************************************************************************************************
 * @fn      StepDetector_Init
 *          <brief>
 *
 ***************************************************************************************************/
void StepDetector_Init(OSP_StepResultCallback_t pStepResultReadyCallback,
    OSP_StepSegmentResultCallback_t pStepSegmentResultReadyCallback)
{
    //Set up callbacks
    stepDetectData.stepResultReadyCallback = pStepResultReadyCallback;
    stepDetectData.stepSegmentResultReadyCallback = pStepSegmentResultReadyCallback;

    //initialize sub-structs
    StepSegmenter_Init(&stepDetectData.stepSegmenter, &SetNewStepSegment);

    //reset
    StepDetector_Reset();
}


/****************************************************************************************************
 * @fn      StepDetector_CleanUp
 *          <brief>
 *
 ***************************************************************************************************/
void StepDetector_CleanUp(void)
{
    stepDetectData.stepResultReadyCallback = NULL;
    stepDetectData.stepSegmentResultReadyCallback = NULL;

    StepSegmenter_CleanUp(&stepDetectData.stepSegmenter);
}


/****************************************************************************************************
 * @fn      StepDetector_Reset
 *          <brief>
 *
 ***************************************************************************************************/
void StepDetector_Reset(void)
{
    //reset step data
    StepDataOSP_t *step = &stepDetectData.step;

    step->startTime = TOFIX_TIME(-1.f);
    step->stopTime = TOFIX_TIME(-1.f);
    step->stepFrequency = 0;
    step->numStepsTotal = 0;
    step->numStepsSinceWalking = 0;

    //reset signal generation and segmentation code
    StepSegmenter_Reset(&stepDetectData.stepSegmenter);
}


/****************************************************************************************************
 * @fn      StepDetector_SetAccelerometerMeasurement
 *          Set method
 *
 ***************************************************************************************************/
void StepDetector_SetFilteredAccelerometerMeasurement(const NTTIME tstamp, const osp_float_t filteredAcc[3])
{
    osp_float_t accNorm = sqrtf(filteredAcc[0] * filteredAcc[0] +
        filteredAcc[1] * filteredAcc[1] +
        filteredAcc[2] * filteredAcc[2]);
    NTTIME tFilter = tstamp;

    //Update step segmenter
    StepSegmenter_UpdateAndCheckForSegment(&stepDetectData.stepSegmenter, accNorm, tFilter);
}


/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
