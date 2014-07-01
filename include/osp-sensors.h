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
#if !defined (OSP_SENSORS_H)
#define   OSP_SENSORS_H

/*-------------------------------------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
\*-------------------------------------------------------------------------------------------------*/
/* This file is meant to provide a common definition of sensor related enumerations/defines and
 * generally should not depend on any other includes
 */

#ifndef __KERNEL__
# include <stdint.h>
#endif

/*-------------------------------------------------------------------------------------------------*\
 |    C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
//! use to specify the kind of sensor input or output
/*!
 * \sa OSP_RegisterInputSensor
 * \sa OSP_SubscribeOutputSensor
 *
 *  Final units of input/outputs are defined by the sensor data convention field of the sensor descriptor.
 *  Flags in the descriptor specify if sensor is calibrated/uncalibrated and/or used as input
 *  If a sensor type not is supported by the library implementation, an error will be returned on its usage
 */

#define osp_pack __attribute__ ((__packed__))

struct osp_pack SensorId_t {
    uint8_t sensorType;
    uint8_t sensorSubType;
};


enum SensorType_t {
    SENSOR_ENUM_FIRST_SENSOR               =  0,

    SENSOR_MESSAGE                         =  SENSOR_ENUM_FIRST_SENSOR, //!< warnings from the library: e.g. excessive timestamp jitter, need calibration
    SENSOR_ACCELEROMETER                   =  1, //!< accelerometer data
    SENSOR_MAGNETIC_FIELD                  =  2, //!< magnetometer data
    SENSOR_GYROSCOPE                       =  3, //!< calibrated gyroscope data
    SENSOR_PRESSURE                        =  4, //!< barometer pressure data
    SENSOR_LIGHT                           =  5, //!< light data
    SENSOR_STEP                            =  6, //!< step data
    SENSOR_HUMIDITY                        =  7, //!< relative humidity data
    SENSOR_ORIENTATION                     =  8, //!< yaw, pitch, roll (also use this for Win8 Inclinometer)
    SENSOR_ROTATION_VECTOR                 =  9, //!< accel+mag+gyro quaternion
    SENSOR_CONTEXT_DEVICE_MOTION           = 10, //!< context of device relative to world frame
    SENSOR_CONTEXT_CARRY                   = 11, //!< context of device relative to user
    SENSOR_CONTEXT_POSTURE                 = 12, //!< context of user relative to world frame
    SENSOR_CONTEXT_TRANSPORT               = 13, //!< context of environment relative to world frame
    SENSOR_GESTURE_EVENT                   = 15, //!< gesture event such as a double-tap or shake
    SENSOR_HEART_RATE                      = 16, //!< heart-rate data

    SENSOR_ENUM_COUNT
};

enum MessageType_t {
    SENSOR_MESSAGE_STD = 0,

    SENSOR_MESSAGE_ENUM_COUNT
};


enum AccelerometerType_t {
    SENSOR_ACCELEROMETER_RAW                    = 0, //!< raw accelerometer data
    SENSOR_ACCELEROMETER_UNCALIBRATED           = 1, //!< calibrated accelerometer data
    SENSOR_ACCELEROMETER_CALIBRATED             = 2, //!< uncalibrated accelerometer data
    SENSOR_ACCELEROMETER_GRAVITY                = 3, //!< gravity part of acceleration in body frame
    SENSOR_ACCELEROMETER_LINEAR_ACCELERATION    = 4, //!< dynamic acceleration

    SENSOR_ACCELEROMETER_ENUM_COUNT
};


enum MagneticFieldType_t {
    SENSOR_MAGNETIC_FIELD_RAW                   =  0, //!< raw magnetometer data
    SENSOR_MAGNETIC_FIELD_UNCALIBRATED          =  1, //!< calibrated magnetometer data
    SENSOR_MAGNETIC_FIELD_CALIBRATED            =  2, //!< uncalibrated magnetometer data

    SENSOR_MAGNETIC_FIELD_ENUM_COUNT
};

enum GyroscopeType_t {
    SENSOR_GYROSCOPE_RAW                   =  0, //!< raw gyroscope data
    SENSOR_GYROSCOPE_UNCALIBRATED          =  1, //!< calibrated gyroscope data
    SENSOR_GYROSCOPE_CALIBRATED            =  2, //!< uncalibrated gyroscope data
    SENSOR_VIRTUAL_GYROSCOPE               =  3, //!< virtual gyroscope data from accel+mag

    SENSOR_GYROSCOPE_ENUM_COUNT
};


enum PressureType_t {
    SENSOR_PRESSURE_STD                    =  0, //!< barometer pressure data

    SENSOR_PRESSURE_ENUM_COUNT
};


enum LightType_t {
    SENSOR_LIGHT_AMBIENT             =  0, //!<  ambient light data
    SENSOR_LIGHT_RGB                 =  1, //!< RGB light data
    SENSOR_LIGHT_UV                  =  2, //!< UV light data
    SENSOR_LIGHT_PROXIMITY           =  3, //!< proximity data

    SENSOR_LIGHT_ENUM_COUNT
};


//!  Use these enums as a sub-result for  STEP result
enum StepType_t {
    SENSOR_CONTEXT_STEP             = 0,  //!< only one kind of step now
    SENSOR_STEP_DETECTOR            = 1, //!< precise time a step occured
    SENSOR_STEP_COUNTER             = 2, //!< count of steps
    SENSOR_STEP_SEGMENT_DETECTOR    = 3, //!< low compute trigger for analyzing if step may have occured

    SENSOR_STEP_ENUM_COUNT
};

enum HumidityType_t {
    SENSOR_RELATIVE_HUMIDITY        =  0, //!< relative humidity data

    SENSOR_HUMIDITY_ENUM_COUNT
};

enum Orientation {
    SENSOR_ORIENTATION_STD                  =  0, //!< yaw, pitch, roll (also use this for Win8 Inclinometer)
    SENSOR_ORIENTATION_AUG_REALITY_COMPASS  =  1, //!< heading which switches to aug-reality mode when camera towards horizon (Win8 compass)

    SENSOR_ORIENTATION_ENUM_COUNT
};

enum RotationVectorType_t {
    SENSOR_ROTATION_VECTOR_QUATERNION      = 0, //!< accel+mag+gyro quaternion
    SENSOR_GEOMAGNETIC_ROTATION_VECTOR     = 1, //!< accel+mag quaternion
    SENSOR_GAME_ROTATION_VECTOR            = 2, //!< accel+gyro quaternion

    SENSOR_ROTATION_VECTOR_ENUM_COUNT
};

//! Use these enums as a sub-result for CONTEXT_DEVICE_MOTION result
enum ContextDeviceMotionType_t {
    CONTEXT_DEVICE_MOTION_STILL                 = 0,
    CONTEXT_DEVICE_MOTION_ACCELERATING          = 1,
    CONTEXT_DEVICE_MOTION_ROTATING              = 2,
    CONTEXT_DEIVCE_MOTION_TRANSLATING           = 3,
    CONTEXT_DEVICE_MOTION_FREE_FALLING          = 4,
    CONTEXT_DEVICE_MOTION_SIGNIFICANT_MOTION    = 5, //!< significant motion (as specified by Android HAL 1.0)
    CONTEXT_DEVICE_MOTION_SIGNIFICANT_STILLNESS = 6, //!< complement to significant motion
    CONTEXT_DEVICE_MOTION_CHANGE_DETECTOR       = 7, //!< low compute trigger for seeing if context may have changed

    CONTEXT_DEVICE_MOTION_ENUM_COUNT
};

//!  Use these enums as a sub-result for  CONTEXT_CARRY result
enum ContextCarryType_t {
    CONTEXT_CARRY_IN_POCKET     = 0,
    CONTEXT_CARRY_IN_HAND       = 1,
    CONTEXT_CARRY_NOT_ON_PERSON = 2,
    CONTEXT_CARRY_IN_HAND_FRONT = 3,
    CONTEXT_CARRY_IN_HAND_SIDE  = 4,

    CONTEXT_CARRY_ENUM_COUNT
};

//!  Use these enums as a sub-result for CONTEXT_POSTURE result
enum ContextPostureType_t {
    CONTEXT_POSTURE_WALKING     = 0,
    CONTEXT_POSTURE_STANDING    = 1,
    CONTEXT_POSTURE_SITTING     = 2,
    CONTEXT_POSTURE_JOGGING     = 3,
    CONTEXT_POSTURE_RUNNING     = 4,
    CONTEXT_POSTURE_ENUM_COUNT
};


//!  Use these enums as a sub-result for  CONTEXT_TRANSPORT result
enum ContextTransportType_t {
    CONTEXT_TRANSPORT_VEHICLE        = 0,
    CONTEXT_TRANSPORT_CAR            = 1,
    CONTEXT_TRANSPORT_TRAIN          = 2,
    CONTEXT_TRANSPORT_AIRPLANE       = 3,
    CONTEXT_TRANSPORT_UP_STAIRS      = 4,
    CONTEXT_TRANSPORT_DOWN_STAIRS    = 5,
    CONTEXT_TRANSPORT_UP_ELEVATOR    = 6,
    CONTEXT_TRANSPORT_DOWN_ELEVATOR  = 7,
    CONTEXT_TRANSPORT_UP_ESCALATOR   = 8,
    CONTEXT_TRANSPORT_DOWN_ESCALATOR = 9,
    CONTEXT_TRANSPORT_MOVING_WALKWAY = 10,
    CONTEXT_TRANSPORT_ON_BIKE        = 11,
    CONTEXT_TRANSPORT_ENUM_COUNT
};

//!  Use these enums as a sub-result for  GESTURE_EVENT result
enum GestureType_t {
    SENSOR_GESTURE_TAP         = 0,
    SENSOR_GESTURE_DOUBLE_TAP  = 1,
    SENSOR_GESTURE_SHAKE       = 2,
    SENSOR_GESTURE_ENUM_COUNT
};

enum HeartRateType_t {
    SENSOR_HEART_RATE_STD        = 0,

    SENSOR_HEART_RATE_ENUM_COUNT
};


/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/


#endif /* OSP_SENSORS_H */
/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
