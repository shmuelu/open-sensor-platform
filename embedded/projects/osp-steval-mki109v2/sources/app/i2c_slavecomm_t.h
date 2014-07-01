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
#if !defined (I2C_SLAVECOMM_T_H)
#define   I2C_SLAVECOMM_T_H

#include <stdint.h>

typedef enum {
    HOST_PROCESSING_NONE = 0,
    HOST_PROCESSING_IN_PROGRESS,
    HOST_PROCESSING_STRETCHED_TX,
    HOST_PROCESSING_STRETCHED_RX
} HostProcessingState;

void SH_Host_Slave_init(void);
void SH_Slave_setup_I2c_Tx(uint8_t *address, uint16_t size);

/****************************************************************************************************
 * @fn      SH_Host_Slave_cmd_processing_active
 * @brief   Signal that command processing is active. this will enable i2C clock stretching if need be
 *
 ***************************************************************************************************/
void SH_Host_Slave_cmd_processing_active(void);

/****************************************************************************************************
 * @fn      SH_Host_Slave_terminate_cmd_processing
 * @brief   Signal termination of command processing. terminates i2C clock stretching if in progress.
 *
 ***************************************************************************************************/
void SH_Host_Slave_terminate_cmd_processing(void);


#endif /* I2C_SLAVECOMM_T_H */
