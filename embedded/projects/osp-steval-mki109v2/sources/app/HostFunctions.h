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
#ifndef HOST_FUNCTIONS_H
#define HOST_FUNCTIONS_H

/*-------------------------------------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
\*-------------------------------------------------------------------------------------------------*/

#include <stdint.h>
//#include <timers.h>

#include "osp-sensors.h"
#include "osp_HostInterface.h"

/*-------------------------------------------------------------------------------------------------*\
 |    C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/
#define SLAVE_NUM_TX_BUFFERS 25


extern uint64_t sensorEnable;      // bit map of (1 << sensorId) to show if sensor is enabled (1) or disabled (0)

/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

extern uint16_t sensorDelay[SENSOR_ENUM_COUNT];
uint16_t getSensorDelay(const struct SensorId_t *sensorId);
extern uint16_t commited_length;

extern uint16_t broadcast_buf_wr;
extern uint16_t broadcast_buf_rd;

extern uint8_t broadcast_buf[SLAVE_NUM_TX_BUFFERS][OSP_HOST_MAX_BROADCAST_BUFFER_SIZE];
extern uint8_t broadcast_buf_flags[SLAVE_NUM_TX_BUFFERS];
extern uint16_t broadcast_buf_used[SLAVE_NUM_TX_BUFFERS];
extern uint16_t last_transmitted_offset[SLAVE_NUM_TX_BUFFERS];

extern uint16_t timeStampExpansion;

/* An array to hold handles to the created timers. */
//extern xTimerHandle android_Timers[ ANDROID_NUM_TIMERS ];

extern uint8_t androidBroadcastTrigger;

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/

uint8_t getLastCommand(void);

void init_android_broadcast_buffers(void);
uint8_t  process_command(uint8_t *rx_buf, uint16_t length);
uint8_t post_on_boardcast_buffer(uint8_t *buffer, uint16_t length, struct Timestamp40_t *timeStamp);

uint8_t isHostInterruptAsserted(void);
void activateHostInterrupt(void);
void deactivateHostInterrupt(void);
void hostCommitDataTx(void);
void configureTimeCaptureTimers (void);
void configureSensorsTimeCapture (void);
void getTimeCapture(enum SensorType_t sensorId, uint32_t *result);
void calculate_commited_tx_buffer_size(void);
uint8_t isSensorEnable(const struct SensorId_t *sensorId);
uint16_t getSensorDelay(const struct SensorId_t *sensorId);

#endif /* HOST_FUNCTIONS_H */
/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
