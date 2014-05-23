/* Open Sensor Platform Project
 * https://github.com/sensorplatforms/open-sensor-platform
 *
 * Copyright (C) 2013 Sensor Platforms Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *	  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*-------------------------------------------------------------------------------------------------*\
 |	I N C L U D E   F I L E S
\*-------------------------------------------------------------------------------------------------*/
#include "common.h"
#include "osp-api.h"
#include <string.h>
#include "osp_embeddedforegroundalgcalls.h"
#include "osp_embeddedbackgroundalgcalls.h"
#include "osp-version.h"


/*-------------------------------------------------------------------------------------------------*\
 |	E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |	P R I V A T E   C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/
#define RESULT_FLAG_PAUSED				(1 << 0)
#define MAX_SENSOR_DESCRIPTORS			8
#define MAX_RESULT_DESCRIPTORS			5
#define SENSOR_FG_DATA_Q_SIZE			8
#define SENSOR_BG_DATA_Q_SIZE			8
#define MAX_SENSORS_PER_RESULT			5

//Sensor flags for internal use
#define SENSOR_FLAG_IN_USE				(1 << 0)
#define SENSOR_FLAG_HAVE_CTL_CALLBACK   (1 << 1)
#define SENSOR_FLAG_HAVE_CAL_CALLBACK   (1 << 2)
#define SENSOR_FLAG_NEEDS_DECIMATION	(1 << 3)

/* Result codes for local functions */
#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR						0

#ifdef ERROR
#undef ERROR
#endif
#define ERROR						   -1

#define Q32DIFF (32 - QFIXEDPOINTPRECISE)

/*-------------------------------------------------------------------------------------------------*\
 |	P R I V A T E   T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/


/* Local structure for keeping tab on active sensors and results */
typedef struct {
	SensorDescriptor_t *pSenDesc;
	uint16_t Flags;					// in-use, etc
} _SenDesc_t;

typedef struct {
	SensorDescriptor_t *pResDesc;
	uint16_t Flags;					// Paused, etc
} _ResDesc_t;

typedef struct {
	InputSensorHandle_t Handle;		// handle for this sensor
	TriAxisSensorRawData_t Data;	// raw data & time stamp from sensor
} _SensorDataBuffer_t;

typedef struct {
	struct SensorId_t  sensorId;	// sensir ID
	uint16_t SensorCount;			// number of sensors required
	struct SensorId_t Sensors[MAX_SENSORS_PER_RESULT];   // sensor types required for this result
} _ResultResourceMap_t;

typedef union {
	Android_UncalibratedAccelOutputData_t ucAccel;
	Android_UncalibratedMagOutputData_t   ucMag;
	Android_UncalibratedGyroOutputData_t  ucGyro;

} AndroidUnCalResult_t;

/* Common bridge between the different data types for base sensors (Accel/Mag/Gyro) */
typedef struct  {
	NTTIME TimeStamp;					// time stamp
	uint8_t accuracy;
	union {
		NTEXTENDED  extendedData[3];	// processed sensor data
		NTPRECISE   preciseData[3];		// processed sensor data
	} data;
} Common_3AxisResult_t;


/*-------------------------------------------------------------------------------------------------*\
 |	S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
static const OSP_Library_Version_t libVersion = {
	(OSP_VERSION_MAJOR << 16) | (OSP_VERSION_MINOR << 8) | (OSP_VERSION_PATCH),
	OSP_VERSION_STRING
};

static uint64_t _SubscribedResults[2];  // bit field of currently subscribed results,
										// bit positionscalculated using sensorType & sensorSubType
										// allocated space for 128 sensors.
										


// pointer to platform descriptor structure
SystemDescriptor_t const *_pPlatformDesc = NULL;

// pointers to sensor data structures, and local flags
static  _SenDesc_t _SensorTable[MAX_SENSOR_DESCRIPTORS];

// pointers to result data structures, and local flags
static _ResDesc_t _ResultTable[MAX_RESULT_DESCRIPTORS];

// Raw sensor data queue for foreground processing
static _SensorDataBuffer_t _SensorFgDataQueue[SENSOR_FG_DATA_Q_SIZE];
static int16_t _SensorFgDataQCnt;		  // number of data packets in the queue
static uint16_t _SensorFgDataNqPtr = SENSOR_FG_DATA_Q_SIZE - 1; // where the last data packet was put into the queue
static uint16_t _SensorFgDataDqPtr;		 // where to remove next data packet from the queue


// Raw sensor data queue for background processing
static _SensorDataBuffer_t _SensorBgDataQueue[SENSOR_BG_DATA_Q_SIZE];
static int16_t _SensorBgDataQCnt;								// number of data packets in the queue
static uint16_t _SensorBgDataNqPtr = SENSOR_BG_DATA_Q_SIZE - 1; // where the last data packet was put into the queue
static uint16_t _SensorBgDataDqPtr;								// where to remove next data packet from the queue

static uint32_t _accelLastForegroundTimeStamp = 0;				// keep the last time stamp here, we will use it to check for rollover
static uint32_t _accelLastForegroundTimeStampExtension = 0;		// we will re-create a larger raw time stamp here

static uint32_t _gyroLastForegroundTimeStamp = 0;				// keep the last time stamp here, we will use it to check for rollover
static uint32_t _gyroLastForegroundTimeStampExtension = 0;		// we will re-create a larger raw time stamp here

static uint32_t _magLastForegroundTimeStamp = 0;				// keep the last time stamp here, we will use it to check for rollover
static uint32_t _magLastForegroundTimeStampExtension = 0;		// we will re-create a larger raw time stamp here

static uint32_t _sensorLastBackgroundTimeStamp = 0;				// keep the last time stamp here, we will use it to check for rollover
static uint32_t _sensorLastBackgroundTimeStampExtension = 0;	// we will re-create a larger raw time stamp here

static NTPRECISE _accel_bias[3] = {0, 0, 0};					// bias in sensor ticks
static NTPRECISE _gyro_bias[3]  = {0, 0, 0};					// bias in sensor ticks
static NTEXTENDED _mag_bias[3]  = {0, 0, 0};					// bias in sensor ticks

// copy of last data that was sent to the alg. We will
// use this for when the user _polls_ for calibrated sensor data.
static Common_3AxisResult_t _LastAccelCookedData;
static Common_3AxisResult_t _LastMagCookedData;
static Common_3AxisResult_t _LastGyroCookedData;

// table of result to resource maps. 1 entry for each result that describes which sensor types
// that it needs, which callback routine to use, etc.
static const _ResultResourceMap_t _ResultResourceMap[] = {
	{
		{SENSOR_MAGNETIC_FIELD, SENSOR_MAGNETIC_FIELD_UNCALIBRATED},
		1,
		{
			{ SENSOR_MAGNETIC_FIELD, SENSOR_MAGNETIC_FIELD_RAW }
		}
	},
	{
		{SENSOR_GYROSCOPE, SENSOR_GYROSCOPE_UNCALIBRATED},
		1,
		{
			{ SENSOR_GYROSCOPE, SENSOR_GYROSCOPE_RAW }
		}
	},
	{
		{SENSOR_ACCELEROMETER, SENSOR_ACCELEROMETER_UNCALIBRATED},
		1,
		{
			{ SENSOR_ACCELEROMETER, SENSOR_ACCELEROMETER_RAW }
		}
	},
	{
		{SENSOR_MAGNETIC_FIELD, SENSOR_MAGNETIC_FIELD_CALIBRATED},
		1,
		{
			{SENSOR_MAGNETIC_FIELD, SENSOR_MAGNETIC_FIELD_UNCALIBRATED}
		}
	},
	{
		{SENSOR_GYROSCOPE, SENSOR_GYROSCOPE_CALIBRATED},
		1,
		{
			{SENSOR_GYROSCOPE, SENSOR_GYROSCOPE_RAW } 
		}
	},
	{
		{ SENSOR_ACCELEROMETER, SENSOR_ACCELEROMETER_CALIBRATED },
		1,
		{
			{ SENSOR_ACCELEROMETER, SENSOR_ACCELEROMETER_RAW }
		}
	},
	{
		{ SENSOR_CONTEXT_DEVICE_MOTION, CONTEXT_DEVICE_MOTION_SIGNIFICANT_MOTION },
		2,
		{
			{ SENSOR_MAGNETIC_FIELD, SENSOR_MAGNETIC_FIELD_UNCALIBRATED },
			{ SENSOR_ACCELEROMETER, SENSOR_ACCELEROMETER_RAW }
		}
	},
	{
		{SENSOR_STEP, SENSOR_STEP_COUNTER},
		1,
		{
			{ SENSOR_ACCELEROMETER, SENSOR_ACCELEROMETER_RAW }
		}
	},
	{
		{SENSOR_STEP, SENSOR_STEP_DETECTOR},
		1,
		{
			{ SENSOR_ACCELEROMETER, SENSOR_ACCELEROMETER_RAW }
		}
	}
};
#define RESOURCE_MAP_COUNT  (sizeof(_ResultResourceMap)/sizeof(_ResultResourceMap_t))

/*-------------------------------------------------------------------------------------------------*\
 |	F O R W A R D   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/
static osp_status_t NullRoutine(void);
/* callback for entering/exiting a critical section of code (i.e. disable/enable task switch) */
OSP_CriticalSectionCallback_t EnterCritical = (OSP_CriticalSectionCallback_t)&NullRoutine;
OSP_CriticalSectionCallback_t ExitCritical = (OSP_CriticalSectionCallback_t)&NullRoutine;
static int16_t FindResultTableIndexByType(const struct SensorId_t *sensorId);

/*-------------------------------------------------------------------------------------------------*\
 |	P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |	P R I V A T E	 F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/****************************************************************************************************
 * @fn	  mult_uint16_uint16
 *		  Unsigned 16-bit multiply with 32-bit result.
 *
 ***************************************************************************************************/
__inline static uint32_t mult_uint16_uint16(uint16_t a, uint16_t b)
{
	return ((uint32_t) a * (uint32_t)b);
}


/****************************************************************************************************
 * @fn	  OnStepResultsReady
 *		  Local callback used for Step Counter results from algorithm
 *
 ***************************************************************************************************/
static void OnStepResultsReady( StepDataOSP_t* stepData )
{
	struct SensorId_t sensorId;
    osp_bool_t isSubscribed;
    
    sensorId.sensorType = SENSOR_STEP;
    sensorId.sensorSubType = SENSOR_STEP_COUNTER;
    
    if ((isSensorSubscribed(&sensorId, &isSubscribed ) != ERROR) && (isSubscribed)) {
		int16_t index;
		Android_StepCounterOutputData_t callbackData;

		callbackData.StepCount = stepData->numStepsTotal;
		callbackData.TimeStamp = stepData->startTime; //!TODO - Double check if start time or stop time
		callbackData.TickTimeStampHigh = _accelLastForegroundTimeStampExtension;
		callbackData.TickTimeStampLow = _accelLastForegroundTimeStamp;

		index = FindResultTableIndexByType(&sensorId);
		_ResultTable[index].pResDesc->pOutputReadyCallback((OutputSensorHandle_t)&_ResultTable[index],
			&callbackData);
	}
    sensorId.sensorType = SENSOR_STEP;
    sensorId.sensorSubType = SENSOR_STEP_DETECTOR;
    
    if ((isSensorSubscribed(&sensorId, &isSubscribed ) != ERROR) && (isSubscribed)) {
		int16_t index;
		Android_StepDetectorOutputData_t callbackData;

		callbackData.TimeStamp = stepData->startTime; //!TODO - Double check if start time or stop time
		callbackData.TickTimeStampHigh = _accelLastForegroundTimeStampExtension;
		callbackData.TickTimeStampLow = _accelLastForegroundTimeStamp;


		index = FindResultTableIndexByType(&sensorId);
		_ResultTable[index].pResDesc->pOutputReadyCallback((OutputSensorHandle_t)&_ResultTable[index],
			&callbackData);
	}
}


/****************************************************************************************************
 * @fn	  OnSignificantMotionResult
 *		  Local callback used for Significant motion results from algorithm
 *
 ***************************************************************************************************/
static void OnSignificantMotionResult( NTTIME * eventTime )
{
	struct SensorId_t sensorId;
    osp_bool_t isSubscribed;

    sensorId.sensorType = SENSOR_CONTEXT_DEVICE_MOTION;
    sensorId.sensorSubType = CONTEXT_DEVICE_MOTION_SIGNIFICANT_MOTION;

    if ((isSensorSubscribed(&sensorId, &isSubscribed ) != ERROR) && (isSubscribed)) {
		int16_t index;
		Android_SignificantMotionOutputData_t callbackData;

		callbackData.TimeStamp = *eventTime;
		callbackData.TickTimeStampHigh = _accelLastForegroundTimeStampExtension;
		callbackData.TickTimeStampLow = _accelLastForegroundTimeStamp;

		index = FindResultTableIndexByType(&sensorId);
		_ResultTable[index].pResDesc->pOutputReadyCallback((OutputSensorHandle_t)&_ResultTable[index],
			&callbackData);
	}
}


/****************************************************************************************************
 * @fn	  NullRoutine
 *		  A valid routine that does nothing. It should be used instead of a NULL function pointer
 *		  for unused call backs so that we never try to execute a function pointed to by NULL
 *
 ***************************************************************************************************/
static osp_status_t NullRoutine(void)
{
	return OSP_STATUS_OK;
}


/****************************************************************************************************
 * @fn	  ValidateSystemDescriptor
 *		  Given a pointer to a system descriptor, validate it's contents. Return NO_ERROR if good,
 *		  otherwise return ERROR
 *
 ***************************************************************************************************/
static int16_t ValidateSystemDescriptor(SystemDescriptor_t const *pSystemDescriptor)
{
	// test that the ENUMs are in range
	return NO_ERROR;
}


/****************************************************************************************************
 * @fn	  FindSensorTableIndexByType
 *		  Given a sensor ID, return the index into the sensor table
 *
 ***************************************************************************************************/
static int16_t FindSensorTableIndexByType(const struct SensorId_t *sensorId)
{
	int16_t i;

	for(i = 0; i < MAX_SENSOR_DESCRIPTORS; i++) {
		if(_SensorTable[i].pSenDesc == NULL)
			continue;
		if((sensorId->sensorType == ((SensorDescriptor_t *)(_SensorTable[i].pSenDesc))->sensorId.sensorType) &&
		   (sensorId->sensorSubType == ((SensorDescriptor_t *)(_SensorTable[i].pSenDesc))->sensorId.sensorSubType))
			return i;
	}
	return ERROR;
}


/****************************************************************************************************
 * @fn	  FindEmptySensorTableIndex
 *		  Find 1st available empty sensor table slot, return the index into the sensor table
 *
 ***************************************************************************************************/
static int16_t FindEmptySensorTableIndex(void)
{
	int16_t i;

	for(i = 0; i < MAX_SENSOR_DESCRIPTORS; i++) {
		if(_SensorTable[i].pSenDesc == NULL)
			return i;
	}
	return ERROR;
}


/****************************************************************************************************
 * @fn	  FindSensorTableIndexByHandle
 *		  Given a sensor handle, return the index into the sensor table
 *
 ***************************************************************************************************/
static int16_t FindSensorTableIndexByHandle(InputSensorHandle_t Handle)
{
	int16_t i;

	for(i = 0; i < MAX_SENSOR_DESCRIPTORS; i++) {
		if(Handle == (InputSensorHandle_t)&_SensorTable[i])
			return i;
	}
	return ERROR;
}


/****************************************************************************************************
 * @fn	  FindResultTableIndexByType
 *		  Given a sensor ID , return the index into the result table
 *
 ***************************************************************************************************/
static int16_t FindResultTableIndexByType(const struct SensorId_t *sensorId)
{
	int16_t i;

	for(i = 0; i < MAX_RESULT_DESCRIPTORS; i++) {
		if(_ResultTable[i].pResDesc == NULL)
			continue;
		if((sensorId->sensorType == ((SensorDescriptor_t *)(_ResultTable[i].pResDesc))->sensorId.sensorType) &&
		   (sensorId->sensorSubType == ((SensorDescriptor_t *)(_ResultTable[i].pResDesc))->sensorId.sensorSubType))
			return i;
	}
	return ERROR;
}


/****************************************************************************************************
 * @fn	  FindEmptyResultTableIndex
 *		  Find 1st available empty result table slot, return the index into the sensor table
 *
 ***************************************************************************************************/
static int16_t FindEmptyResultTableIndex(void)
{
	int16_t i;

	for(i = 0; i < MAX_RESULT_DESCRIPTORS; i++) {
		if(_ResultTable[i].pResDesc == NULL)
			return i;
	}
	return ERROR;
}


/****************************************************************************************************
 * @fn	  FindResultTableIndexByHandle
 *		  Given a result handle, return the index into the result table
 *
 ***************************************************************************************************/
static int16_t FindResultTableIndexByHandle(OutputSensorHandle_t Handle)
{
	int16_t i;

	for(i = 0; i < MAX_RESULT_DESCRIPTORS; i++) {
		if(Handle == (OutputSensorHandle_t)&_ResultTable[i])
			return i;
	}
	return ERROR;
}


/****************************************************************************************************
 * @fn	  FindResourceMapIndexByType
 *		  Given a sensor ID, return the index into the resource map table. If not found, return
 *		  ERROR
 *
 ***************************************************************************************************/
static int16_t FindResourceMapIndexByType(const struct SensorId_t *sensorId)
{
	int16_t i;

	for (i = 0; i < RESOURCE_MAP_COUNT; i++) {
		if ((_ResultResourceMap[i].sensorId.sensorType == sensorId->sensorType) &&
			(_ResultResourceMap[i].sensorId.sensorSubType == sensorId->sensorSubType))
			return i;
	}
	return ERROR;
}


/****************************************************************************************************
 * @fn	  ValidateSensorDescriptor
 *		  Given a pointer to a sensor descriptor, validate it's contents
 *
 ***************************************************************************************************/
static int16_t ValidateSensorDescriptor(SensorDescriptor_t *pSensorDescriptor)
{
	InputSensorSpecificData_t *pSensSpecific;
	osp_bool_t haveGoodCalData = FALSE;

	// test that the ENUMs are in range
	if ( pSensorDescriptor->sensorId.sensorType >= SENSOR_ENUM_COUNT )
		return ERROR;

	// if we have sensor specific data, check if its valid
	pSensSpecific = pSensorDescriptor->pSensorSpecificData;
	if (pSensSpecific != NULL) {
		if ( pSensSpecific->AxisMapping[0] >= AXIS_MAP_ENUM_COUNT )
			return ERROR;
		if ( pSensSpecific->AxisMapping[1] >= AXIS_MAP_ENUM_COUNT )
			return ERROR;
		if ( pSensSpecific->AxisMapping[2] >= AXIS_MAP_ENUM_COUNT )
			return ERROR;

		if (pSensSpecific->pCalibrationData != NULL) {
			//!TODO Validate cal data provided
		}
	}

	// Do any other validation that is required here...

	switch(pSensorDescriptor->sensorId.sensorType) {

	case SENSOR_ACCELEROMETER:
		if (pSensorDescriptor->sensorId.sensorSubType >= SENSOR_ACCELEROMETER_ENUM_COUNT)
			return ERROR;
		break;

	case SENSOR_MAGNETIC_FIELD:
		if (pSensorDescriptor->sensorId.sensorSubType >= SENSOR_MAGNETIC_FIELD_ENUM_COUNT)
			return ERROR;
		break;

	case SENSOR_GYROSCOPE:
		if (pSensorDescriptor->sensorId.sensorSubType >= SENSOR_GYROSCOPE_ENUM_COUNT)
			return ERROR;
		break;

	default:
		break;
	}

	if (haveGoodCalData != TRUE) {
		return OSP_STATUS_CAL_NOT_VALID;
	}
	return NO_ERROR;
}


/****************************************************************************************************
 * @fn	  ValidateResultDescriptor
 *		  Given a pointer to a result descriptor, validate it's contents
 *
 ***************************************************************************************************/
static int16_t ValidateResultDescriptor(SensorDescriptor_t *pResultDescriptor)
{
	if(pResultDescriptor->sensorId.sensorType >= SENSOR_ENUM_COUNT )
		return ERROR;

	if(pResultDescriptor->pOutputReadyCallback == NULL)
		return ERROR;

	// Do any other validation that's required
	//...

	// Add currently supported output sensor results...
	switch (pResultDescriptor->sensorId.sensorType) {

	case SENSOR_MAGNETIC_FIELD:
		if (pResultDescriptor->sensorId.sensorSubType >= SENSOR_MAGNETIC_FIELD_ENUM_COUNT)
			return ERROR;
		break;
	case SENSOR_GYROSCOPE:
		if (pResultDescriptor->sensorId.sensorSubType >= SENSOR_GYROSCOPE_ENUM_COUNT)
			return ERROR;
		break;
	case SENSOR_ACCELEROMETER:
		if (pResultDescriptor->sensorId.sensorSubType >= SENSOR_ACCELEROMETER_ENUM_COUNT)
			return ERROR;
		break;
	case SENSOR_STEP:
		if (pResultDescriptor->sensorId.sensorSubType >= SENSOR_STEP_ENUM_COUNT)
			return ERROR;
		break;
	case SENSOR_CONTEXT_DEVICE_MOTION:
		if (pResultDescriptor->sensorId.sensorSubType >= CONTEXT_DEVICE_MOTION_ENUM_COUNT)
			return ERROR;
		break;
	default:
		return ERROR;

	}
	return NO_ERROR;
}


/****************************************************************************************************
 * @fn	  InvalidateQueuedDataByHandle
 *		  Invalidates the handles for the sensor data in the queue so that the data is discarded
 *
 ***************************************************************************************************/
void InvalidateQueuedDataByHandle(InputSensorHandle_t Handle)
{
	uint16_t i;

	EnterCritical();
	for(i = 0; i < SENSOR_FG_DATA_Q_SIZE; i++ ) {
		if(_SensorFgDataQueue[i].Handle == Handle)
			_SensorFgDataQueue[i].Handle = NULL;
	}
	for(i = 0; i < SENSOR_BG_DATA_Q_SIZE; i++ ) {
		if(_SensorBgDataQueue[i].Handle == Handle)
			_SensorBgDataQueue[i].Handle = NULL;
	}
	ExitCritical();
}


/****************************************************************************************************
 * @fn	  TurnOnSensors
 *		  Turns on sensors indicated by sensorsMask (bit mask based on enum SensorType_t bit position
 *
 ***************************************************************************************************/
static int16_t TurnOnSensors(uint32_t sensorsMask)
{
	SensorControl_t SenCtl;

	//  Check for control callback
	if(_pPlatformDesc->SensorsControl != NULL) {
		// send a sensor off command
		SenCtl.Handle = NULL;
		SenCtl.Command = SENSOR_CONTROL_SENSOR_ON;
		SenCtl.Data = sensorsMask;
		_pPlatformDesc->SensorsControl(&SenCtl);
	}
	return NO_ERROR;
}


/****************************************************************************************************
 * @fn	  TurnOffSensors
 *		  Turns off sensors indicated by sensorsMask (bit mask based on enum SensorType_t bit position
 *
 ***************************************************************************************************/
static int16_t TurnOffSensors(uint32_t sensorsMask)
{
	SensorControl_t SenCtl;

	//  does it have a control callback?
	if(_pPlatformDesc->SensorsControl != NULL) {
		// send a sensor off command
		SenCtl.Handle = NULL;
		SenCtl.Command = SENSOR_CONTROL_SENSOR_OFF;
		SenCtl.Data = sensorsMask;
		_pPlatformDesc->SensorsControl(&SenCtl);
	}
	return NO_ERROR;
}


/****************************************************************************************************
 * @fn	  ActivateResultSensors
 *		  Given a sensor ID, check to be sure that all of the sensors that are needed
 *		  for this result type are available. If any sensor that is needed is available and marked
 *		  as "not in-use", mark it so and send a "turn on" command through the control callback if
 *		  it is available.
 *
 ***************************************************************************************************/
static int16_t ActivateResultSensors(const struct SensorId_t *sensorId)
{
	int16_t i,j;
	int16_t index;
	uint32_t sensorsMask = 0;

	// use the result to sensor mapping table to see if all sensors that are needed are registered.
	// also check their "in-use" status. If any required sensor is not registered, return ERROR.

	// Find the index of the result type in the resource map table.
	for (i =  0; i < RESOURCE_MAP_COUNT; i++) {
		if ((_ResultResourceMap[i].sensorId.sensorType == sensorId->sensorType) &&
			(_ResultResourceMap[i].sensorId.sensorSubType == sensorId->sensorSubType)){
			for(j = 0; j < _ResultResourceMap[i].SensorCount; j++) {
				index = FindSensorTableIndexByType(&_ResultResourceMap[i].Sensors[j]);
				if(index == ERROR)
					return ERROR;				// sensor is not registered, exit with error
				// if this sensor is not active, mark it as such and send a command to it to go active.
				if((_SensorTable[index].Flags & SENSOR_FLAG_IN_USE) == 0) {
					_SensorTable[index].Flags |= SENSOR_FLAG_IN_USE; // mark sensor as "in use"
					sensorsMask |= (1 << _ResultResourceMap[i].Sensors[j].sensorType);
				}
			}
			break;
		}
	}

	if (sensorsMask) TurnOnSensors(sensorsMask);

	return NO_ERROR;
}


/****************************************************************************************************
 * @fn	  DeactivateResultSensors
 *		  Given a sensor ID, deactivate (mark as "not in use") all sensors that were used by
 *		  this result if they are not in use by any another active result.
 *
 ***************************************************************************************************/
static int16_t DeactivateResultSensors(const struct SensorId_t *sensorId)
{
	int16_t i,j,k,l;
	int16_t index;
	osp_bool_t NeedSensor;
	uint32_t sensorsMask = 0;


	// use the result to sensor mapping table to see if any sensors that are were used are still in use by another result.
	// We will send a control command to turn off the sensor if it supports it.

	// find our result resource table entry
	index = FindResourceMapIndexByType(sensorId);
	if (index == ERROR)
		return ERROR;

	// search active results to see if they use any of the sensors that we used
	for(i = 0; i < _ResultResourceMap[index].SensorCount; i++ ) { // for each of our sensors
		NeedSensor = FALSE;									   // assume no other result uses this sensor
		for(k = 0; k < MAX_RESULT_DESCRIPTORS; k++) {
			if((_ResultTable[k].pResDesc != NULL) && (_ResultTable[k].pResDesc->sensorId.sensorType != sensorId->sensorType)) { // search active results (but not ours)
				j = FindResourceMapIndexByType(&_ResultTable[k].pResDesc->sensorId);
				if(j == ERROR)
					return ERROR;
				for(l = 0; l < _ResultResourceMap[j].SensorCount; l++) { // for each sensor in this active result
					if ((_ResultResourceMap[j].Sensors[l].sensorType == _ResultResourceMap[index].Sensors[i].sensorType) &&
						(_ResultResourceMap[j].Sensors[l].sensorType == _ResultResourceMap[index].Sensors[i].sensorSubType))  {
						NeedSensor = TRUE; // found one, stop searching this result
						break;
					}
				}
				if(NeedSensor == TRUE)
					break;  // another result is using this sensor, no need to check more results for this sensor
			}
		}
		if(NeedSensor == FALSE) {
			// if we get here, no other result uses this sensor type, mark it "not in use" and send
			// a "turn off" command to it, and mark all data in the input queues as stale
			j = FindSensorTableIndexByType(&_ResultResourceMap[index].Sensors[i]);
			_SensorTable[j].Flags &= ~SENSOR_FLAG_IN_USE;   // Mark sensor "not in use"
			// mark all previously queued data for this sensor type as invalid
			InvalidateQueuedDataByHandle((InputSensorHandle_t)&_SensorTable[j]);
			sensorsMask |= (1 << _ResultResourceMap[index].Sensors[i].sensorType);
		}
	}
	if (sensorsMask) TurnOffSensors(sensorsMask);
	return NO_ERROR;
}


/****************************************************************************************************
 * @fn	  UMul32
 *		  Helper routine for 32-bit saturating multiply. This maybe optimized in assembly if needed
 *
 ***************************************************************************************************/
static void UMul32(uint32_t x,uint32_t y, uint32_t * pHigh, uint32_t * pLow)
{
	uint16_t xmsb;
	uint16_t ymsb;
	uint16_t xlsb;
	uint16_t ylsb;

	register uint32_t high;
	register uint32_t low;
	register uint32_t temp2;
	register uint32_t temp;


	xmsb = x >> 16;
	ymsb = y >> 16;

	xlsb = x & 0x0000FFFF;
	ylsb = y & 0x0000FFFF;

	high = mult_uint16_uint16(xmsb , ymsb);

	temp = mult_uint16_uint16(ymsb , xlsb);
	high += (temp & 0xFFFF0000) >> 16;

	low = temp << 16;

	temp = mult_uint16_uint16(xmsb , ylsb);
	high += (temp & 0xFFFF0000) >> 16;

	temp2 = low;
	low += temp << 16;

	if (low < temp2) {
		++high;
	}

	temp = low;
	low += mult_uint16_uint16(xlsb, ylsb);

	if (low < temp) {
		++high;
	}

	*pHigh = high;
	*pLow = low;
}


/****************************************************************************************************
 * @fn	  GetTimeFromCounter
 *		  Helper routine for time conversion
 *
 ***************************************************************************************************/
osp_bool_t GetTimeFromCounter(
	NTTIME * pTime,
	TIMECOEFFICIENT counterToTimeConversionFactor,
	uint32_t counterHigh,
	uint32_t counterLow)
{
	NTTIME ret = 0;
	uint32_t high1,low1;
	uint32_t high2,low2;
	const uint32_t roundfactor = (1 << (Q32DIFF-1)) ;
	if (counterToTimeConversionFactor & 0x80000000) {
		counterToTimeConversionFactor = ~counterToTimeConversionFactor + 1;
	}

	UMul32(counterToTimeConversionFactor, counterLow, &high1, &low1);
	UMul32(counterToTimeConversionFactor, counterHigh, &high2, &low2);

	low2 += high1;
	if (low2 < high1) {
		high2++;
	}

	//round things
	low1 += roundfactor;

	//check overflow
	if (low1 < roundfactor) {
		low2++;
		if (low2 ==0) {
			high2++;
		}
	}

	//right shift by Q32DIFF to make this into a Q24 number from a Q32 number
	low1 >>= Q32DIFF;
	low1 |= (low2 << (32 -  Q32DIFF) );
	low2 >>= Q32DIFF;
	low2 |= (high2 << (32 -  Q32DIFF) );
	high2 >>= Q32DIFF;

	if (high2 || low2 & 0x80000000) {
		//saturation!!!!!
		*pTime = 0x7FFFFFFFFFFFFFFFLL;
		return FALSE;
	}

	ret = low2;
	ret <<= 32;
	ret |= low1;

	*pTime = ret;

	return TRUE;
}


/****************************************************************************************************
 * @fn	  ScaleSensorData
 *		   Apply sign extension, offset and scaling to raw sensor data. NOTE: ScaleFactor may
 *		  contain either NTPRECISE or NTEXTENDED number, base of "accuracy". Return value will also
 *		  follow same logic.
 *
 ***************************************************************************************************/
static int32_t ScaleSensorData(
	int32_t Data,
	uint32_t Mask,
	int32_t Offset,
	int32_t ScaleFactor)
{
	int64_t llTemp;

	// apply offset
	Data -= Offset;

	Data &= Mask;			   // mask off non-used data bits
	// sign extend (we assume that the data is in 2s complement format)
	if((Data & (~Mask >> 1)) != 0 )
		Data |= ~Mask;

	llTemp = (int64_t) Data * (int64_t) ScaleFactor; // scale the data

	if(llTemp > SATURATE_INT_MAX )
		llTemp = SATURATE_INT_MAX;   //if overflow, make max
	if(llTemp < SATURATE_INT_MIN )
		llTemp = SATURATE_INT_MIN;   //if underflow, make min
	return (int32_t)llTemp;	 //return just the lower 32 bits
}


/****************************************************************************************************
 * @fn	  ConvertSensorData
 *		  Given a pointer to a raw sensor data packet from the input queue of type
 *		  _SensorDataBuffer_t and a pointer to a sensor output data packet of type
 *		  TriAxisSensorCookedData_t, apply translations and conversions into a format per Android
 *		  conventions. Accuracy must be either QFIXEDPOINTPRECISE or QFIXEDPOINTEXTENDED
 *
 ***************************************************************************************************/
static int16_t ConvertSensorData(
	_SensorDataBuffer_t *pRawData,
	Common_3AxisResult_t *pCookedData,
	uint8_t accuracy,
	uint32_t *sensorTimeStamp,
	uint32_t *sensorTimeStampExtension)
{
	uint16_t i;
	unsigned char negative;
	unsigned char source;
	InputSensorSpecificData_t *pInpSensData;

	switch( ((_SenDesc_t *)(pRawData->Handle))->pSenDesc->sensorId.sensorType ) {
	case SENSOR_ACCELEROMETER:
		switch( ((_SenDesc_t *)(pRawData->Handle))->pSenDesc->sensorId.sensorSubType ) {
		case SENSOR_ACCELEROMETER_RAW:
		case SENSOR_ACCELEROMETER_UNCALIBRATED:
		case SENSOR_ACCELEROMETER_CALIBRATED:
			break;
		default:
			return ERROR;
		}
		break;
	case SENSOR_MAGNETIC_FIELD:
		switch( ((_SenDesc_t *)(pRawData->Handle))->pSenDesc->sensorId.sensorSubType ) {
		case SENSOR_MAGNETIC_FIELD_RAW:
		case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
		case SENSOR_MAGNETIC_FIELD_CALIBRATED:
			break;
		default:
			return ERROR;
		}
		break;
	case SENSOR_GYROSCOPE:
		switch( ((_SenDesc_t *)(pRawData->Handle))->pSenDesc->sensorId.sensorSubType ) {
		case SENSOR_GYROSCOPE_RAW:
		case SENSOR_GYROSCOPE_UNCALIBRATED:
		case SENSOR_GYROSCOPE_CALIBRATED:
			break;
		default:
			return ERROR;
		}
		break;
	default:
		return ERROR;
	}

	// apply axis conversion and data width 1st, then offset, then gain (scaling), finally convert the time stamp.
	pCookedData->accuracy = accuracy;
	pInpSensData =  ((_SenDesc_t *)(pRawData->Handle))->pSenDesc->pSensorSpecificData;

	for (i = 0; i < 3; i++) {
		negative = 0;
		switch (pInpSensData->AxisMapping[i]) {

		case AXIS_MAP_UNUSED:
			switch (accuracy) {
			case QFIXEDPOINTPRECISE:
				pCookedData->data.preciseData[i] = TOFIX_PRECISE(0.0f);
				break;
			case QFIXEDPOINTEXTENDED:
				pCookedData->data.extendedData[i] = TOFIX_EXTENDED(0.0f);
				break;
			default:
				return ERROR;
			}
			continue;
		case AXIS_MAP_NEGATIVE_X:
			negative = 1;
		case AXIS_MAP_POSITIVE_X:
			source = 0;
			break;

		case AXIS_MAP_NEGATIVE_Y:
			negative = 1;
		case AXIS_MAP_POSITIVE_Y:
			source = 1;
			break;

		case AXIS_MAP_NEGATIVE_Z:
			negative = 1;
		case AXIS_MAP_POSITIVE_Z:
			source = 2;
			break;

		default:
			return ERROR;
		}

		switch (accuracy) {
		case QFIXEDPOINTPRECISE:
			// mask, sign extend (if needed), apply offset and scale factor
			pCookedData->data.preciseData[i] = ScaleSensorData(pRawData->Data.Data[source],
				pInpSensData->DataWidthMask,
				pInpSensData->ConversionOffset[i],
				pInpSensData->ConversionScale[i]);
			if (negative)
				pCookedData->data.preciseData[i] = -pCookedData->data.preciseData[i];

			break;
		case QFIXEDPOINTEXTENDED:
			// mask, sign extend (if needed), apply offset and scale factor
			pCookedData->data.extendedData[i] = ScaleSensorData(pRawData->Data.Data[source],
				pInpSensData->DataWidthMask,
				pInpSensData->ConversionOffset[i],
				pInpSensData->ConversionScale[i]);
			if (negative)
				pCookedData->data.extendedData[i] = -pCookedData->data.extendedData[i];
			break;
		default:
			return ERROR;
		}
	}

	// scale time stamp into seconds
	// check for user timestamp rollover, if so bump our timestamp extension word
	// !!WARNING!!: The time stamp extension scheme will need to be changed if timer capture is used
	// for sensor time-stamping. Current scheme will cause time jumps if two sensors are timer-captured
	// before & after rollover but the sensor that was captured after rollover is queued before the
	// sensor that was captured before timer rollover

	if( ((int32_t)(*sensorTimeStamp) < 0) && ((int32_t)pRawData->Data.TimeStamp >= 0) ) {
		(*sensorTimeStampExtension)++;
	}
	*sensorTimeStamp = pRawData->Data.TimeStamp;

	GetTimeFromCounter(&pCookedData->TimeStamp, _pPlatformDesc->TstampConversionToSeconds,
		*sensorTimeStampExtension, *sensorTimeStamp);
	return NO_ERROR;
}

/*-------------------------------------------------------------------------------------------------*\
 |	A P I	 F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/****************************************************************************************************
 * @fn	  OSP_Initialize
 *		  Does internal initializations that the library requires.
 *
 * @param   pSystemDesc - INPUT pointer to a struct that describes things like time tick conversion
 *		  value
 *
 * @return  status as specified in OSP_Types.h
 *
 ***************************************************************************************************/
osp_status_t OSP_Initialize(const SystemDescriptor_t* pSystemDesc)
{
	_SubscribedResults[0] = 0;								// by definition, we are not subscribed to any results
	_SubscribedResults[1] = 0;
	memset(_SensorTable, 0, sizeof(_SensorTable));	  // init the sensor table
	memset(_ResultTable, 0, sizeof(_ResultTable));	  // init the result table also

	if(ValidateSystemDescriptor(pSystemDesc) == ERROR)
		return (osp_status_t)OSP_STATUS_DESCRIPTOR_INVALID;
	_pPlatformDesc = pSystemDesc;
	if((pSystemDesc->EnterCritical != NULL) && (pSystemDesc->ExitCritical != NULL)) {
		EnterCritical = pSystemDesc->EnterCritical;
		ExitCritical = pSystemDesc->ExitCritical;
	}

	OSPForegroundAlg_InitializeAlgorithms();
	OSPBackgroundAlg_InitializeAlgorithms();

	return OSP_STATUS_OK;
}


/****************************************************************************************************
 * @fn	  OSP_RegisterInputSensor
 *		  Tells the Open-Sensor-Platform Library what kind of sensor inputs it has to work with.
 *
 * @param   pSensorDescriptor INPUT pointer to data which describes all the details of this sensor
 *		  and its current operating mode; e.g. sensor type, SI unit conversion factor
 * @param   pReturnedHandle OUTPUT a handle to use when feeding data in via OSP_SetData()
 *
 * @return  status as specified in OSP_Types.h
 *
 ***************************************************************************************************/
osp_status_t OSP_RegisterInputSensor(SensorDescriptor_t *pSensorDescriptor,
	InputSensorHandle_t *pReturnedHandle)
{
	int16_t status;
	int16_t index;
	osp_bool_t haveCalData = FALSE;

	// Find 1st available slot in the sensor descriptor table, insert descriptor pointer, clear flags
	// and return pointer to this table entry. If no room in the sensor table, return OSP_STATUS_NO_MORE_HANDLES
	// If this sensor type is already registered, return OSP_STATUS_ALREADY_REGISTERED.
	if((pSensorDescriptor == NULL) || (pReturnedHandle == NULL))	  // just in case
		return OSP_STATUS_NULL_POINTER;

	if(FindSensorTableIndexByType(&pSensorDescriptor->sensorId) != ERROR) { // is this sensor type already registered?
		*pReturnedHandle = NULL;
		return OSP_STATUS_ALREADY_REGISTERED;
	}

	// we have a new sensor, validate the sensor descriptor here before entering it into our table.
	status = ValidateSensorDescriptor(pSensorDescriptor);

	if (status == ERROR) {
		return OSP_STATUS_DESCRIPTOR_INVALID;
	} else if (status == NO_ERROR) {
		haveCalData = TRUE;
	}

	haveCalData = haveCalData; //Avoid compiler warning for now!

	// If room in the sensor table, enter it and return the handle, else return OSP_STATUS_NO_MORE_HANDLES
	index = FindEmptySensorTableIndex();
	if(index != ERROR) {
		_SensorTable[index].pSenDesc = pSensorDescriptor;
		_SensorTable[index].Flags = 0;
		*pReturnedHandle = (InputSensorHandle_t)&_SensorTable[index];
	} else {
		return OSP_STATUS_NO_MORE_HANDLES;
	}

	// setup any flags for this sensor
	if (pSensorDescriptor->pOptionalWriteCalDataCallback != NULL)  // set the flag for the optional sensor calibration changed callback
		_SensorTable[index].Flags |= SENSOR_FLAG_HAVE_CAL_CALLBACK;
	else
		_SensorTable[index].Flags &= ~SENSOR_FLAG_HAVE_CAL_CALLBACK;

	_SensorTable[index].Flags &= ~SENSOR_FLAG_IN_USE;			   // by definition, this sensor isn't in use yet.

	return OSP_STATUS_OK;
}


/****************************************************************************************************
 * @fn	  OSP_UnregisterInputSensor
 *		  Call to remove an sensor from OSP's known set of inputs.
 *
 * @param   handle INPUT a handle to the input sensor you want to unregister
 *
 * @return  status as specified in OSP_Types.h
 *
 ***************************************************************************************************/
osp_status_t OSP_UnregisterInputSensor(InputSensorHandle_t sensorHandle)
{
	int16_t index;
	// Check the sensor table to be sure we have a valid entry, if so we need to check
	// to be sure that the sensor type is the same. Clear sensor table entry, and return a NULL
	// SensorHandle and the appropriate error code.
	// We also need to mark all data for this sensor that is in the input queue as invalid (make handle = NULL).

	index = FindSensorTableIndexByHandle(sensorHandle);
	if(index == ERROR) {							// test for valid handle
		return OSP_STATUS_NOT_REGISTERED;
	}

	//Invalidate queued data for this sensor
	InvalidateQueuedDataByHandle(sensorHandle);

	// Invalidate the descriptor entry
	_SensorTable[index].pSenDesc = NULL;


	return OSP_STATUS_OK;
}


/****************************************************************************************************
 * @fn	  OSP_SetData
 *		  Queues sensor data which will be processed by OSP_DoForegroundProcessing() and
 *		  OSP_DoBackgroundProcessing()
 *
 * @param   sensorHandle INPUT requires a valid handle as returned by OSP_RegisterInputSensor()
 * @param   data INPUT pointer to timestamped raw sensor data
 *
 * @return  status as specified in OSP_Types.h
 *
 ***************************************************************************************************/
osp_status_t OSP_SetData(InputSensorHandle_t sensorHandle, TriAxisSensorRawData_t *data)
{
	register osp_status_t FgStatus = 0;
	register osp_status_t BgStatus = 0;


	if (data == NULL)										   // just in case
		return OSP_STATUS_NULL_POINTER;
	if(sensorHandle == NULL)									// just in case
		return OSP_STATUS_INVALID_HANDLE;
	if(FindSensorTableIndexByHandle(sensorHandle) == ERROR)
		return OSP_STATUS_INVALID_HANDLE;

	if( ((_SenDesc_t *)sensorHandle)->Flags & SENSOR_FLAG_IN_USE ) { // if this sensor is not used by a result, ignore data

		// put sensor data into the foreground queue
		FgStatus = OSP_STATUS_QUEUE_FULL;						// assume queue full
		EnterCritical();										// no interrupts while we diddle the queues
		if(_SensorFgDataQCnt < SENSOR_FG_DATA_Q_SIZE) {			// check for room in the foreground queue
			_SensorFgDataQCnt++;								//  if so, show one more in the queue
			if(++_SensorFgDataNqPtr == SENSOR_FG_DATA_Q_SIZE) { //  bump the enqueue pointer and check for pointer wrap, rewind if so
				_SensorFgDataNqPtr = 0;
			}
			FgStatus = OSP_STATUS_OK;							// FG queue isn't full
		}
		_SensorFgDataQueue[_SensorFgDataNqPtr].Handle = sensorHandle;
		memcpy(&_SensorFgDataQueue[_SensorFgDataNqPtr].Data, data, sizeof(TriAxisSensorRawData_t)); // put data in queue (room or not)

		// put sensor data into the background queue
		BgStatus = OSP_STATUS_QUEUE_FULL;						//	assume queue full
		if(_SensorBgDataQCnt < SENSOR_BG_DATA_Q_SIZE) {			// check for room in the background queue
			_SensorBgDataQCnt++;								//  if so, show one more in the queue
			if(++_SensorBgDataNqPtr == SENSOR_BG_DATA_Q_SIZE) { //  bump the enqueue pointer and check for pointer wrap, rewind if so
				_SensorBgDataNqPtr = 0;
			}
			BgStatus = OSP_STATUS_OK;							//	FG queue isn't full
		}
		_SensorBgDataQueue[_SensorBgDataNqPtr].Handle = sensorHandle;
		memcpy(&_SensorBgDataQueue[_SensorBgDataNqPtr].Data, data, sizeof(TriAxisSensorRawData_t)); // put data in queue (room or not)
		ExitCritical();
	}

	if((FgStatus == OSP_STATUS_QUEUE_FULL) || (BgStatus == OSP_STATUS_QUEUE_FULL))
		return OSP_STATUS_QUEUE_FULL;
	else
		return OSP_STATUS_OK;
}


/****************************************************************************************************
 * @fn	  OSP_DoForegroundProcessing
 *		  Triggers computation for primary algorithms  e.g ROTATION_VECTOR
 *
 * @param   none
 *
 * @return  status as specified in OSP_Types.h
 *
 ***************************************************************************************************/
osp_status_t OSP_DoForegroundProcessing(void)
{
	_SensorDataBuffer_t data;
	Common_3AxisResult_t AndoidProcessedData;
	AndroidUnCalResult_t AndoidUncalProcessedData;
	int16_t index;
	Common_3AxisResult_t algConvention;
    struct SensorId_t sensorId;
    osp_bool_t isSubscribed;

	// Get next sensor data packet from the queue. If nothing in the queue, return OSP_STATUS_IDLE.
	// If we get a data packet that has a sensor handle of NULL, we should drop it and get the next one,
	// a NULL handle is an indicator that the data is from a sensor that has been replaced or that the data is stale.

	EnterCritical();										// no interrupts while we diddle the queue

	// ignore any data marked as stale.
	while( (_SensorFgDataQueue[_SensorFgDataDqPtr].Handle == NULL) && (_SensorFgDataQCnt != 0) ) {
		_SensorFgDataQCnt--;								// stale data, show one less in the queue
		if(++_SensorFgDataDqPtr == SENSOR_FG_DATA_Q_SIZE)   //  and check for pointer wrap, rewind if so
			_SensorFgDataDqPtr = 0;
	}

	// now see if there is any data to process.
	if(_SensorFgDataQCnt == 0) {					// check for queue empty
		ExitCritical();
		return OSP_STATUS_IDLE;						// nothing left in the queue, let the caller know that
	}

	// There is at least 1 data packet in the queue, get it.
	memcpy(&data, &_SensorFgDataQueue[_SensorFgDataDqPtr], sizeof(_SensorDataBuffer_t)); // remove data from queue
	_SensorFgDataQCnt--;								// show one less in the queue
	if(++_SensorFgDataDqPtr == SENSOR_FG_DATA_Q_SIZE)   //  and check for pointer wrap, rewind if so
		_SensorFgDataDqPtr = 0;
	ExitCritical();

	sensorId.sensorType = (enum SensorType_t) ((_SenDesc_t*)data.Handle)->pSenDesc->sensorId.sensorType;
	sensorId.sensorSubType = ((_SenDesc_t*)data.Handle)->pSenDesc->sensorId.sensorSubType;
	// now send the processed data to the appropriate entry points in the alg code.
	switch(sensorId.sensorType) {
	case SENSOR_ACCELEROMETER:
		switch (sensorId.sensorSubType) {
		case SENSOR_ACCELEROMETER_RAW:
		case SENSOR_ACCELEROMETER_UNCALIBRATED:
			// Now we have a copy of the data to be processed. We need to apply any and all input conversions.
			if (sensorId.sensorSubType == SENSOR_ACCELEROMETER_RAW) {
				ConvertSensorData(
					&data,
					&AndoidProcessedData,
					QFIXEDPOINTPRECISE,
					&_accelLastForegroundTimeStamp,
					&_accelLastForegroundTimeStampExtension);
			}           
            // after conversion, RAW becomes Uncalibrated.
            sensorId.sensorSubType = SENSOR_ACCELEROMETER_UNCALIBRATED;
            
            if ((isSensorSubscribed(&sensorId, &isSubscribed ) != ERROR) && (isSubscribed)) {
                // Do uncalibrated accel call back here (Android conventions)
                index = FindResultTableIndexByType(&sensorId);
                if (index == ERROR) {
                    return OSP_STATUS_ERROR;
                }
                memcpy(&AndoidUncalProcessedData.ucAccel.X,
                    AndoidProcessedData.data.preciseData,
                    (sizeof(NTPRECISE)*3));
                memcpy(&AndoidUncalProcessedData.ucAccel.X_offset,
                    _accel_bias, (sizeof(NTPRECISE)*3));
                AndoidUncalProcessedData.ucAccel.TimeStamp = AndoidProcessedData.TimeStamp;

                AndoidUncalProcessedData.ucAccel.TickTimeStampHigh = _accelLastForegroundTimeStampExtension;
                AndoidUncalProcessedData.ucAccel.TickTimeStampLow = _accelLastForegroundTimeStamp;

                _ResultTable[index].pResDesc->pOutputReadyCallback(
                    (OutputSensorHandle_t)&_ResultTable[index], &AndoidUncalProcessedData.ucAccel);
            }

			// convert to algorithm convention before feeding data to algorithms.
			algConvention.accuracy = QFIXEDPOINTPRECISE;
			algConvention.data.preciseData[0] = AndoidProcessedData.data.preciseData[1];  // x (ALG) =  Y (Android)
			algConvention.data.preciseData[1] = -AndoidProcessedData.data.preciseData[0]; // y (ALG) = -X (Android)
			algConvention.data.preciseData[2] = AndoidProcessedData.data.preciseData[2];  // z (ALG) =  Z (Android)
			algConvention.TimeStamp = AndoidProcessedData.TimeStamp;

			memcpy(&_LastAccelCookedData, &algConvention, sizeof(Common_3AxisResult_t));

			// Send data on to algorithms
			OSPForegroundAlg_SetAccelerometerMeasurement(algConvention.TimeStamp, algConvention.data.preciseData);

			// Do linear accel and gravity processing if needed
			// ... TODO
			break;
		}
		break;
	case SENSOR_MAGNETIC_FIELD:
		switch (sensorId.sensorSubType) {
		case SENSOR_MAGNETIC_FIELD_RAW:
		case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
			// Now we have a copy of the data to be processed. We need to apply any and all input conversions.
			if (sensorId.sensorSubType == SENSOR_MAGNETIC_FIELD_RAW) {
                ConvertSensorData(
                    &data,
                    &AndoidProcessedData,
                    QFIXEDPOINTEXTENDED,
                    &_magLastForegroundTimeStamp,
                    &_magLastForegroundTimeStampExtension);
            }
             // after conversion, RAW becomes Uncalibrated.
            sensorId.sensorSubType = SENSOR_MAGNETIC_FIELD_UNCALIBRATED;
            
            if ((isSensorSubscribed(&sensorId, &isSubscribed ) != ERROR) && (isSubscribed)) {
				index = FindResultTableIndexByType(&sensorId);
				if (index == ERROR) {
					return OSP_STATUS_ERROR;
				}
				memcpy(&AndoidUncalProcessedData.ucMag.X,
					AndoidProcessedData.data.extendedData,
					(sizeof(NTEXTENDED)*3));
				memcpy(&AndoidUncalProcessedData.ucMag.X_hardIron_offset,
					_mag_bias, (sizeof(NTEXTENDED)*3));
				AndoidUncalProcessedData.ucMag.TimeStamp = AndoidProcessedData.TimeStamp;

				AndoidUncalProcessedData.ucMag.TickTimeStampHigh = _magLastForegroundTimeStampExtension;
				AndoidUncalProcessedData.ucMag.TickTimeStampLow = _magLastForegroundTimeStamp;


				_ResultTable[index].pResDesc->pOutputReadyCallback(
					(OutputSensorHandle_t)&_ResultTable[index], &AndoidUncalProcessedData.ucMag);
			}

			// convert to algorithm convention before feeding data to algs.
			algConvention.accuracy = QFIXEDPOINTEXTENDED;
			algConvention.data.extendedData[0] = AndoidProcessedData.data.extendedData[1];  // x (ALG) =  Y (Android)
			algConvention.data.extendedData[1] = -AndoidProcessedData.data.extendedData[0]; // y (ALG) = -X (Android)
			algConvention.data.extendedData[2] = AndoidProcessedData.data.extendedData[2];  // z (ALG) =  Z (Android)
			algConvention.TimeStamp = AndoidProcessedData.TimeStamp;

			memcpy(&_LastMagCookedData, &algConvention, sizeof(Common_3AxisResult_t));

			//OSPForegroundAlg_SetMagnetometerMeasurement(AndoidProcessedData.TimeStamp, algConvention.data.extendedData);
			break;
		}
		break;
	case SENSOR_GYROSCOPE:
		switch (sensorId.sensorSubType) {
		case SENSOR_GYROSCOPE_RAW:
		case SENSOR_GYROSCOPE_UNCALIBRATED:
			// Now we have a copy of the data to be processed. We need to apply any and all input conversions.
            if (sensorId.sensorSubType == SENSOR_GYROSCOPE_RAW) {
                ConvertSensorData(
                    &data,
                    &AndoidProcessedData,
                    QFIXEDPOINTPRECISE,
                    &_gyroLastForegroundTimeStamp,
                    &_gyroLastForegroundTimeStampExtension);
            }
             // after conversion, RAW becomes Uncalibrated.
            sensorId.sensorSubType = SENSOR_MAGNETIC_FIELD_UNCALIBRATED;
            
            if ((isSensorSubscribed(&sensorId, &isSubscribed ) != ERROR) && (isSubscribed)) {
				index = FindResultTableIndexByType(&sensorId);
				if (index != ERROR) {
					return OSP_STATUS_ERROR;
				}
				memcpy(&AndoidUncalProcessedData.ucGyro.X,
					AndoidProcessedData.data.preciseData,
					(sizeof(NTPRECISE)*3));
				memcpy(&AndoidUncalProcessedData.ucGyro.X_drift_offset,
					_gyro_bias, (sizeof(NTPRECISE)*3));
				AndoidUncalProcessedData.ucGyro.TimeStamp = AndoidProcessedData.TimeStamp;

				AndoidUncalProcessedData.ucGyro.TickTimeStampHigh = _gyroLastForegroundTimeStampExtension;
				AndoidUncalProcessedData.ucGyro.TickTimeStampLow = _gyroLastForegroundTimeStamp;

				_ResultTable[index].pResDesc->pOutputReadyCallback(
					(OutputSensorHandle_t)&_ResultTable[index], &AndoidUncalProcessedData.ucGyro);
			}

			// convert to algorithm convention before feeding data to algs.
			algConvention.accuracy = QFIXEDPOINTPRECISE;
			algConvention.data.preciseData[0] = AndoidProcessedData.data.preciseData[1];  // x (ALG) =  Y (Android)
			algConvention.data.preciseData[1] = -AndoidProcessedData.data.preciseData[0]; // y (ALG) = -X (Android)
			algConvention.data.preciseData[2] = AndoidProcessedData.data.preciseData[2];  // z (ALG) =  Z (Android)
			algConvention.TimeStamp = AndoidProcessedData.TimeStamp;

			memcpy(&_LastGyroCookedData, &algConvention, sizeof(Common_3AxisResult_t));

			//OSPForegroundAlg_SetGyroscopeMeasurement(AndoidProcessedData.TimeStamp, algConvention.data.preciseData);
			break;
		}
		break;
	default:
		break;
	}

	// all done for now, return OSP_STATUS_IDLE if no more data in the queue, else return OSP_STATUS_OK

	if(_SensorFgDataQCnt == 0)
		return OSP_STATUS_IDLE;				 // nothing left in the queue, let the caller know that
	else
		return OSP_STATUS_OK;				 // more to process
}


/****************************************************************************************************
 * @fn	  OSP_DoBackgroundProcessing
 *		  Triggers computation for less time critical background algorithms, e.g. sensor calibration
 *
 * @param   none
 *
 * @return  status as specified in OSP_Types.h
 *
 ***************************************************************************************************/
osp_status_t OSP_DoBackgroundProcessing(void)
{
	_SensorDataBuffer_t data;
	Common_3AxisResult_t AndoidProcessedData;

	enum SensorType_t sensorType;
	uint8_t sensorSubType;

	//Common_3AxisResult_t algConvention;

	// Get next sensor data packet from the queue. If nothing in the queue, return OSP_STATUS_IDLE.
	// If we get a data packet that has a sensor handle of NULL, we should drop it and get the next one,
	// a NULL handle is an indicator that the data is from a sensor that has been replaced and that the data is stale.

	EnterCritical();										// no interrupts while we diddle the queue

	// ignore any data marked as stale.
	while( (_SensorBgDataQueue[_SensorBgDataDqPtr].Handle == NULL) && (_SensorBgDataQCnt != 0) ) {
		_SensorBgDataQCnt--;								// stale data, show one less in the queue
		if(++_SensorBgDataDqPtr == SENSOR_BG_DATA_Q_SIZE)   //  and check for pointer wrap, rewind if so
			_SensorBgDataDqPtr = 0;
	}

	// now see if there is any data to process.
	if(_SensorBgDataQCnt == 0) {				// check for queue empty
		ExitCritical();
		return OSP_STATUS_IDLE;				 // nothing left in the queue, let the caller know that
	}

	// There is at least 1 data packet in the queue, get it.
	memcpy(&data, &_SensorBgDataQueue[_SensorBgDataDqPtr], sizeof(_SensorDataBuffer_t)); // remove data from queue
	_SensorBgDataQCnt--;								// show one less in the queue
	if(++_SensorBgDataDqPtr == SENSOR_BG_DATA_Q_SIZE)   //  and check for pointer wrap, rewind if so
		_SensorBgDataDqPtr = 0;

	ExitCritical();

	// now send the processed data to the appropriate entry points in the alg calibration code.
	sensorType = (enum SensorType_t)((_SenDesc_t*)data.Handle)->pSenDesc->sensorId.sensorType;
	sensorSubType = ((_SenDesc_t*)data.Handle)->pSenDesc->sensorId.sensorSubType;

	switch(sensorType) {
	case SENSOR_ACCELEROMETER:
		switch (sensorSubType) {
		case SENSOR_ACCELEROMETER_RAW:
		case SENSOR_ACCELEROMETER_UNCALIBRATED:
			// Now we have a copy of the data to be processed. We need to apply any and all input conversions.
			ConvertSensorData(
				&data,
				&AndoidProcessedData,
				QFIXEDPOINTPRECISE,
				&_sensorLastBackgroundTimeStamp,
				&_sensorLastBackgroundTimeStampExtension);

	#if 0 //Nothing to be done for background processing at this time!
			// convert to algorithm convention.
			algConvention.accuracy = QFIXEDPOINTPRECISE;
			algConvention.data.preciseData[0] = AndoidProcessedData.data.preciseData[1];  // x (ALG) =  Y (Android)
			algConvention.data.preciseData[1] = -AndoidProcessedData.data.preciseData[0]; // y (ALG) = -X (Android)
			algConvention.data.preciseData[2] = AndoidProcessedData.data.preciseData[2];  // z (ALG) =  Z (Android)
			algConvention.TimeStamp = AndoidProcessedData.TimeStamp;

			OSPBackgroundAlg_SetAccelerometerMeasurement(algConvention.TimeStamp, algConvention.data.preciseData);
	#endif
			break;
		}
		break;
	case SENSOR_MAGNETIC_FIELD:
		switch (sensorSubType) {
		case SENSOR_MAGNETIC_FIELD_RAW:
		case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
			// Now we have a copy of the data to be processed. We need to apply any and all input conversions.
			ConvertSensorData(
				&data,
				&AndoidProcessedData,
				QFIXEDPOINTEXTENDED,
				&_sensorLastBackgroundTimeStamp,
				&_sensorLastBackgroundTimeStampExtension);

	#if 0 //Nothing to be done for background processing at this time!
			// convert to algorithm convention.
			algConvention.accuracy = QFIXEDPOINTEXTENDED;
			algConvention.data.extendedData[0] = AndoidProcessedData.data.extendedData[1];   // x (ALG) =  Y (Android)
			algConvention.data.extendedData[1] = -AndoidProcessedData.data.extendedData[0];  // y (ALG) = -X (Android)
			algConvention.data.extendedData[2] = AndoidProcessedData.data.extendedData[2];   // z (ALG) =  Z (Android)
			algConvention.TimeStamp = AndoidProcessedData.TimeStamp;

			//OSPBackgroundAlg_SetMagnetometerMeasurement(algConvention.TimeStamp, algConvention.data.extendedData);
	#endif
			break;
		}
		break;
	case SENSOR_GYROSCOPE:
		switch (sensorSubType) {
		case SENSOR_GYROSCOPE_UNCALIBRATED:
		case SENSOR_GYROSCOPE_CALIBRATED:
			// Now we have a copy of the data to be processed. We need to apply any and all input conversions.
			ConvertSensorData(
				&data,
				&AndoidProcessedData,
				QFIXEDPOINTPRECISE,
				&_sensorLastBackgroundTimeStamp,
				&_sensorLastBackgroundTimeStampExtension);

	#if 0 //Nothing to be done for background processing at this time!
			// convert to algorithm convention.
			algConvention.accuracy = QFIXEDPOINTPRECISE;
			algConvention.data.preciseData[0] = AndoidProcessedData.data.preciseData[1];  // x (ALG) =  Y (Android)
			algConvention.data.preciseData[1] = -AndoidProcessedData.data.preciseData[0]; // y (ALG) = -X (Android)
			algConvention.data.preciseData[2] = AndoidProcessedData.data.preciseData[2];  // z (ALG) =  Z (Android)
			algConvention.TimeStamp = AndoidProcessedData.TimeStamp;

			//OSPBackgroundAlg_SetGyroscopeMeasurement(algConvention.TimeStamp, algConvention.data.preciseData);
	#endif
			break;
		}
		break;
	default:
		break;
	}

	// all done for now, return OSP_STATUS_IDLE if no more data in the queue, else return OSP_STATUS_OK
	if(_SensorBgDataQCnt == 0)
		return OSP_STATUS_IDLE;					// nothing left in the queue, let the caller know that
	else
		return OSP_STATUS_OK;					// more to process
}


/****************************************************************************************************
 * @fn	  OSP_SubscribeOutputSensor
 *		  Call for each Open-Sensor-Platform result (STEP_COUNT, ROTATION_VECTOR, etc) you want
 *		  computed and output
 *
 * @param   pSensorDescriptor INPUT pointer to data which describes the details of how the fusion
 *		  should be computed: e.g output rate, sensors to use, etc.
 * @param   pOutputHandle OUTPUT a handle to be used for OSP_UnsubscribeOutputSensor()
 *
 * @return  status as specified in OSP_Types.h. OSP_UNSUPPORTED_FEATURE for results that aren't
 *		  available or licensed
 *
 ***************************************************************************************************/
osp_status_t OSP_SubscribeOutputSensor(SensorDescriptor_t *pSensorDescriptor,
	OutputSensorHandle_t *pOutputHandle)
{
	int16_t index;
    struct SensorId_t sensorId;
    osp_bool_t subscribe = TRUE;
    
	if((pSensorDescriptor == NULL) || (pOutputHandle == NULL) ||
		(pSensorDescriptor->pOutputReadyCallback == NULL)) // just in case
		return OSP_STATUS_NULL_POINTER;

	if(FindResultTableIndexByType(&pSensorDescriptor->sensorId) != ERROR) { // is this result type already subscribed?
		*pOutputHandle = NULL;
		return OSP_STATUS_ALREADY_SUBSCRIBED;
	}

	// we have a new request, validate the result descriptor here before entering it into our table.

	if(ValidateResultDescriptor(pSensorDescriptor) == ERROR)
		return OSP_STATUS_DESCRIPTOR_INVALID;

	// Check for room in the result table, if no room, return OSP_STATUS_NO_MORE_HANDLES

	index = FindEmptyResultTableIndex();
	if(index == ERROR) {									// if no room in the result table, return the error
		*pOutputHandle = NULL;								//  and set the handle to NULL, so we can check for it later
		return OSP_STATUS_NO_MORE_HANDLES;
	}

	// check to be sure that we have all sensors registered that we need for this result. If so, mark them as "in use".
	// if not, return OSP_STATUS_NOT_REGISTERED, and set the result handle to NULL.

	if(ActivateResultSensors(&pSensorDescriptor->sensorId) == ERROR) {
		*pOutputHandle = NULL;								//  and set the handle to NULL, so we can check for it later
		return OSP_STATUS_NOT_REGISTERED;
	}
	sensorId.sensorType = pSensorDescriptor->sensorId.sensorType;
	sensorId.sensorSubType = pSensorDescriptor->sensorId.sensorSubType;

	// Setup the alg callbacks, and any thing else that is needed for this result.
	switch (sensorId.sensorType) {
	case SENSOR_ACCELEROMETER:
		switch (sensorId.sensorSubType) {
		case SENSOR_ACCELEROMETER_UNCALIBRATED:
		case SENSOR_ACCELEROMETER_CALIBRATED:
            controlSensorSubscription(&sensorId, &subscribe );
			//Note: Calibrated or uncalibrated result is specified in the descriptor flags
			//For Uncalibrated result no callback needs to be registered with the algorithms
			break;
		}
		break;
	case SENSOR_MAGNETIC_FIELD:
		switch (sensorId.sensorSubType) {
		case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
		case SENSOR_MAGNETIC_FIELD_CALIBRATED:
            controlSensorSubscription(&sensorId, &subscribe );
			//Note: Calibrated or uncalibrated result is specified in the descriptor flags
			//For Uncalibrated result no callback needs to be registered with the algorithms
			break;
		default:
			return OSP_STATUS_UNKNOWN_REQUEST;
		}
		break;
	case SENSOR_GYROSCOPE:
		switch (sensorId.sensorSubType) {
		case SENSOR_GYROSCOPE_UNCALIBRATED:
		case SENSOR_GYROSCOPE_CALIBRATED:
            controlSensorSubscription(&sensorId, &subscribe );
			//Note: Calibrated or uncalibrated result is specified in the descriptor flags
			//For Uncalibrated result no callback needs to be registered with the algorithms
			break;
		default:
			return OSP_STATUS_UNKNOWN_REQUEST;
		}
		break;
	case SENSOR_CONTEXT_DEVICE_MOTION:
		switch (sensorId.sensorSubType) {
		case SENSOR_CONTEXT_DEVICE_MOTION:
            controlSensorSubscription(&sensorId, &subscribe );
			OSPForegroundAlg_RegisterSignificantMotionCallback(OnSignificantMotionResult);
			break;
		default:
			return OSP_STATUS_UNKNOWN_REQUEST;
		}
		break;
	case SENSOR_STEP:
		switch (sensorId.sensorSubType) {
		case SENSOR_STEP_COUNTER:
            controlSensorSubscription(&sensorId, &subscribe );
			OSPForegroundAlg_RegisterStepCallback(OnStepResultsReady);
			break;
		case SENSOR_STEP_DETECTOR:
            controlSensorSubscription(&sensorId, &subscribe );
			OSPForegroundAlg_RegisterStepCallback(OnStepResultsReady);
			break;
		default:
			return OSP_STATUS_UNKNOWN_REQUEST;
		}
		break;
	default:
		return OSP_STATUS_UNKNOWN_REQUEST;
	}

	// Everything is setup, update our result table and return a handle
	_ResultTable[index].pResDesc = pSensorDescriptor;
	_ResultTable[index].Flags = 0;
	*pOutputHandle = (OutputSensorHandle_t *)&_ResultTable[index];

	return OSP_STATUS_OK;
}


/****************************************************************************************************
 * @fn	  OSP_UnsubscribeOutputSensor
 *		  Stops the chain of computation for a registered result
 *
 * @param   OutputHandle INPUT OutputSensorHandle_t that was received from
 *		  OSP_SubscribeOutputSensor()
 *
 * @return  status as specified in OSP_Types.h.
 *
 ***************************************************************************************************/
osp_status_t OSP_UnsubscribeOutputSensor(OutputSensorHandle_t OutputHandle)
{
	int16_t index;
    struct SensorId_t sensorId;
    osp_bool_t subscribe = FALSE;

	// Check the result table to be sure that this is a valid handle, if not return OSP_STATUS_INVALID_HANDLE error.
	// Also check that the handle points to a currently subscribed result, if not return OSP_STATUS_NOT_SUBSCRIBED.
	if(OutputHandle == NULL)	  // just in case
		return OSP_STATUS_INVALID_HANDLE;
	index = FindResultTableIndexByHandle(OutputHandle);
	if((index == ERROR) || (_ResultTable[index].pResDesc == NULL))  // test for active subscription for this handle
		return OSP_STATUS_NOT_SUBSCRIBED;

	// Check to see if any of the other results that are still subscribed needs to use the sensors
	// that we used. If not, mark those sensors as unused so that data from them will not be processed.
	// All data in the input queues from sensors that we mark as unused should be marked as stale. We
	// will also send a "sensor off" if that facility is available.
	DeactivateResultSensors(&_ResultTable[index].pResDesc->sensorId);

	sensorId.sensorType = _ResultTable[index].pResDesc->sensorId.sensorType;
	sensorId.sensorSubType= _ResultTable[index].pResDesc->sensorId.sensorSubType;

	// Now make sure that we won't call the users callback for this result
	switch (sensorId.sensorType) {
	case SENSOR_ACCELEROMETER:
		switch (sensorId.sensorSubType) {
		case SENSOR_ACCELEROMETER_UNCALIBRATED:
            controlSensorSubscription(&sensorId, &subscribe );
			break;
		case SENSOR_ACCELEROMETER_CALIBRATED:
            controlSensorSubscription(&sensorId, &subscribe );
			break;
		default:
			return OSP_STATUS_UNKNOWN_REQUEST;
		}
		break;
	case SENSOR_MAGNETIC_FIELD:
		switch (sensorId.sensorSubType) {
		case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
            controlSensorSubscription(&sensorId, &subscribe );
			break;
		case SENSOR_MAGNETIC_FIELD_CALIBRATED:
            controlSensorSubscription(&sensorId, &subscribe );
			break;
		default:
			return OSP_STATUS_UNKNOWN_REQUEST;
		}
		break;
	case SENSOR_GYROSCOPE:
		switch (sensorId.sensorSubType) {
		case SENSOR_GYROSCOPE_UNCALIBRATED:
            controlSensorSubscription(&sensorId, &subscribe );
			break;
		case SENSOR_GYROSCOPE_CALIBRATED:
            controlSensorSubscription(&sensorId, &subscribe );
			break;
		default:
			return OSP_STATUS_UNKNOWN_REQUEST;
		}
		break;
	case SENSOR_CONTEXT_DEVICE_MOTION:
		switch (sensorId.sensorSubType) {
		case CONTEXT_DEVICE_MOTION_SIGNIFICANT_MOTION:
            controlSensorSubscription(&sensorId, &subscribe );
			break;
		default:
			return OSP_STATUS_UNKNOWN_REQUEST;
		}
		break;
	case SENSOR_STEP:
		switch (sensorId.sensorSubType) {
		case SENSOR_STEP_COUNTER:
            controlSensorSubscription(&sensorId, &subscribe );
			break;
		case SENSOR_STEP_DETECTOR:
            controlSensorSubscription(&sensorId, &subscribe );
			break;
		default:
			return OSP_STATUS_UNKNOWN_REQUEST;
		}
		break;
	default:
		return OSP_STATUS_UNKNOWN_REQUEST;
	}

	// remove result table entry.
	_ResultTable[index].pResDesc = NULL;
	_ResultTable[index].Flags = 0;

	return OSP_STATUS_OK;
}


/****************************************************************************************************
 * @fn	  OSP_GetVersion
 *		  Provides version number and version string of the library implementation
 *
 * @param   pVersionStruct OUTPUT pointer to a pointer that will receive the version data.
 *
 * @return  status as specified in OSP_Types.h
 *
 ***************************************************************************************************/
osp_status_t OSP_GetVersion(const OSP_Library_Version_t **pVersionStruct)
{
	*pVersionStruct = &libVersion;
	return OSP_STATUS_OK;
}



/****************************************************************************************************
 * @fn	  calculateSensorNumber
 * @brief  This helper function returns an a unique serial number for specified sensor
 * @param  sensorId: Sensor identifier
 * @param uint16_t pointer- place to store sensor serial number result
 * @return ERROR if invalid sensor id, NO_ERROR id results generate
 *
 ***************************************************************************************************/
static osp_status_t calculateSensorNumber(const struct SensorId_t *sensorId, uint16_t *sensorSerialNumber) {
	uint16_t serialNumber = 0;
	uint8_t checked = 0;
    if ((sensorId == NULL) || (sensorSerialNumber == NULL)) return ERROR;
	switch (sensorId->sensorType) {
	case SENSOR_HEART_RATE:
		 if (!checked) {
			 if (sensorId->sensorSubType >= SENSOR_HEART_RATE_ENUM_COUNT)
				return ERROR;
			checked = 1;
		 }
		 serialNumber += GESTURE_ENUM_COUNT;				// falls into next case
	case SENSOR_GESTURE_EVENT:	
		 if (!checked) {
			 if (sensorId->sensorSubType >= GESTURE_ENUM_COUNT)
				return ERROR;
			checked = 1;
		 }
		 serialNumber += CONTEXT_TRANSPORT_ENUM_COUNT;	  // falls into next case
	case SENSOR_CONTEXT_TRANSPORT:	
		 if (!checked) {
			 if (sensorId->sensorSubType >= CONTEXT_TRANSPORT_ENUM_COUNT)
				return ERROR;
			checked = 1;
		 }
		 serialNumber += CONTEXT_POSTURE_ENUM_COUNT;		// falls into next case
	case SENSOR_CONTEXT_POSTURE:	
		 if (!checked) {
			 if (sensorId->sensorSubType >= CONTEXT_POSTURE_ENUM_COUNT)
				return ERROR;
			checked = 1;
		 }
		 serialNumber += CONTEXT_CARRY_ENUM_COUNT;		  // falls into next case
	case SENSOR_CONTEXT_CARRY:	
		 if (!checked) {
			 if (sensorId->sensorSubType >= CONTEXT_CARRY_ENUM_COUNT)
				return ERROR;
			checked = 1;
		 }
		 serialNumber += CONTEXT_DEVICE_MOTION_ENUM_COUNT;  // falls into next case
	case SENSOR_CONTEXT_DEVICE_MOTION:	
		 if (!checked) {
			 if (sensorId->sensorSubType >= CONTEXT_DEVICE_MOTION_ENUM_COUNT)
				return ERROR;
			checked = 1;
		 }
		 serialNumber += SENSOR_ROTATION_ENUM_COUNT;   // falls into next case
	case SENSOR_ROTATION_VECTOR:	
		 if (!checked) {
			 if (sensorId->sensorSubType >= SENSOR_ROTATION_ENUM_COUNT)
				return ERROR;
			checked = 1;
		 }
		 serialNumber += SENSOR_ORIENTATION_ENUM_COUNT;	 // falls into next case
	case SENSOR_ORIENTATION:	
		  if (!checked) {
			 if (sensorId->sensorSubType >= SENSOR_ORIENTATION_ENUM_COUNT)
				return ERROR;
			checked = 1;
		 }
		serialNumber += SENSOR_HUMIDITY_ENUM_COUNT;		// falls into next case
	case SENSOR_HUMIDITY:	
		  if (!checked) {
			 if (sensorId->sensorSubType >= SENSOR_HUMIDITY_ENUM_COUNT)
				return ERROR;
			checked = 1;
		 }
		serialNumber += SENSOR_STEP_ENUM_COUNT;			// falls into next case
	case SENSOR_STEP:	
		 if (!checked) {
			 if (sensorId->sensorSubType >= SENSOR_STEP_ENUM_COUNT)
				return ERROR;
			checked = 1;
		 }
		 serialNumber += SENSOR_LIGHT_ENUM_COUNT;		   // falls into next case
	case SENSOR_LIGHT:	
		  if (!checked) {
			 if (sensorId->sensorSubType >= SENSOR_LIGHT_ENUM_COUNT)
				return ERROR;
			checked = 1;
		 }
		serialNumber += SENSOR_PRESSURE_ENUM_COUNT;		// falls into next case
	case SENSOR_PRESSURE:	
		 if (!checked) {
			 if (sensorId->sensorSubType >= SENSOR_PRESSURE_ENUM_COUNT)
				return ERROR;
			checked = 1;
		 }
		 serialNumber += SENSOR_GYROSCOPE_ENUM_COUNT;	   // falls into next case
	case SENSOR_GYROSCOPE:
		 if (!checked) {
			 if (sensorId->sensorSubType >= SENSOR_GYROSCOPE_ENUM_COUNT)
				return ERROR;
			checked = 1;
		 }
		serialNumber += SENSOR_MAGNETIC_FIELD_ENUM_COUNT;   // falls into next case
	case SENSOR_MAGNETIC_FIELD :
		if (!checked) {
			if (sensorId->sensorSubType >= SENSOR_MAGNETIC_FIELD_ENUM_COUNT)
				return ERROR;
			checked = 1;
		}
		serialNumber += SENSOR_ACCELEROMETER_ENUM_COUNT;	// falls into next case
	case SENSOR_ACCELEROMETER:
		if (!checked) {
			if (sensorId->sensorSubType >= SENSOR_ACCELEROMETER_ENUM_COUNT)
				return ERROR;
			checked = 1;
		}
		serialNumber += SENSOR_MESSAGE_ENUM_COUNT;		  // falls into next case
	case SENSOR_MESSAGE:
		if (!checked) {
			if (sensorId->sensorSubType >= SENSOR_MESSAGE_ENUM_COUNT)
				return ERROR;
			checked = 1;
		}
		serialNumber += sensorId->sensorSubType;
		break;
	default:
		return ERROR;
	}

    *sensorSerialNumber = serialNumber;
    return NO_ERROR;
}

/****************************************************************************************************
 * @fn	  isSensorSubscribed
 * @brief  This helper function a boolean if specified sensor is already subscribed
 * @param  sensorId: Sensor identifier
 * @param  isSubscribed - pointer to store result
 * @return ERROR if invalid sensor id, NO_ERROR if results generate
 *
 ***************************************************************************************************/
osp_status_t isSensorSubscribed(const struct SensorId_t *sensorId, osp_bool_t *isSubscribed ) {

    uint16_t sensorSerialNumber;
    
    if (calculateSensorNumber(sensorId,&sensorSerialNumber) == ERROR) return ERROR;
    
    if (sensorSerialNumber < 64) {
        if (_SubscribedResults[0] & (1LL << sensorSerialNumber)) {
            *isSubscribed = TRUE;
        } else {
            *isSubscribed = FALSE;
        }
        return NO_ERROR;
    }
    if (sensorSerialNumber < 127) {
        if (_SubscribedResults[1] & (1LL << (sensorSerialNumber - 64))) {
            *isSubscribed = TRUE;
        } else {
            *isSubscribed = FALSE;
        }
        return NO_ERROR;
    }
    return ERROR;
}

/****************************************************************************************************
 * @fn	  controlSensorSubscription
 * @brief  This helper function a boolean if specified sensor is already subscribed
 * @param  sensorId: Sensor identifier
 * @param  subscribe - IO : IN: TRUE to subscribe, false to unsubscribe, OUT: TRUE if new state, FALSE if same state
 * @return ERROR if invalid sensor id, NO_ERROR if done
 *
 ***************************************************************************************************/
osp_status_t controlSensorSubscription(const struct SensorId_t *sensorId, osp_bool_t *subscribe ) {

    uint16_t sensorSerialNumber;
    
    if (calculateSensorNumber(sensorId, &sensorSerialNumber) == ERROR) return ERROR;
    
    if (sensorSerialNumber < 64) {
        if (_SubscribedResults[0] & (1LL << sensorSerialNumber)) {              // if currently subscribed
            if (*subscribe) {                                                   //   if attempt to re-subscribe
                *subscribe = FALSE;                                             //     indicate no new state
            } else {
                *subscribe = TRUE;                                              //    otherwise indicate new state
                _SubscribedResults[0] &= ~(1LL << sensorSerialNumber);          //    unsubscribe
            }
        } else {                                                                // if currently unsubscribed
            if (*subscribe) {                                                   //   if attempt to subscribe
                _SubscribedResults[0] |= (1LL << sensorSerialNumber);           //     subscribe
                *subscribe = TRUE;                                              //     indicate new state
            } else {
                *subscribe = FALSE;                                             //   otherwise indicate no new state
            }
        }
        return NO_ERROR;
    }
    if (sensorSerialNumber < 127) {
        if (_SubscribedResults[1] & (1LL << (sensorSerialNumber - 64))) {       // if currently subscribed
            if (*subscribe) {                                                   //   if attempt to re-subscribe
                *subscribe = FALSE;                                             //     indicate no new state
            } else {
                *subscribe = TRUE;                                              //    otherwise indicate new state
                _SubscribedResults[1] &= ~(1LL << (sensorSerialNumber - 64));   //    unsubscribe
            }
        } else {                                                                // if currently unsubscribed
            if (*subscribe) {                                                   //   if attempt to subscribe
                _SubscribedResults[1] |= (1LL << (sensorSerialNumber - 64));    //     subscribe
                *subscribe = TRUE;                                              //     indicate new state
            } else {
                *subscribe = FALSE;                                             //   otherwise indicate no new state
            }
        }
        return NO_ERROR;
    }
    return ERROR;
}

/*-------------------------------------------------------------------------------------------------*\
 |	E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
