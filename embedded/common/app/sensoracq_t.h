/* Open Sensor Platform Project
 * https://github.com/sensorplatforms/open-sensor-platform
 *
 * Copyright (C) 2013 Sensor Platforms Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef __SENSOR_ACQ_T_H__
#define __SENSOR_ACQ_T_H__

#include <stdint.h>

#include "osp-sensors.h"

#ifdef __cplusplus
extern "C" {
#endif


/****************************************************************************************************
 * @fn      SendInputSensorControlIndication
 * @brief  This helper function sends input sensor control indication to Sensor Acq task
 * @param  command  - SensorControlCommand_t (currently only SENSOR_CONTROL_SENSOR_ON & SENSOR_CONTROL_SENSOR_OFF supported
 * @param  mask - mask per index to _SensorTable for each sensor to act on
 * @return None
 *
 ***************************************************************************************************/
void SendInputSensorControlIndication(uint16_t command, uint16_t mask);

/****************************************************************************************************
 * @fn      SendDataReadyIndication
 * @brief  This helper function sends data ready indication to Sensor Acq task. Called from ISR
 * @param  sensorId: Sensor identifier whose data is ready to be read
 * @return None
 *
 ***************************************************************************************************/
void SendDataReadyIndication( const struct SensorId_t *sensorId, uint32_t timeStamp );

/****************************************************************************************************
 * @fn      SendInputSensorControlIndication
 * @brief  This helper function sends input sensor control indication to Sensor Acq task
 * @param  command  - SensorControlCommand_t (currently only SENSOR_CONTROL_SENSOR_ON & SENSOR_CONTROL_SENSOR_OFF supported
 * @param  mask - mask per index to _SensorTable for each sensor to act on
 * @return None
 *
 ***************************************************************************************************/
void SendInputSensorControlIndication(uint16_t command, uint16_t mask);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_ACQ_T_H__ */
