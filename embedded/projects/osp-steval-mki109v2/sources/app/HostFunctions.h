/*
 * androidHostinterface.h
 *
 *  Created on: Mar 8, 2013
 *      Author: sungerfeld
 */

#ifndef ANDROIDHOSTINTERFACE_H_
#define ANDROIDHOSTINTERFACE_H_

#include <stdint.h>
//#include <timers.h>

#include "osp-sensors.h"
#include "HostInterface.h"

#define SLAVE_NUM_TX_BUFFERS 25


extern uint64_t sensorEnable;      // bit map of (1 << sensorId) to show if sensor is enabled (1) or disabled (0)
uint8_t isSensorEnable(uint8_t sensorId);
uint8_t areSensorEnable(uint64_t mask);


extern uint16_t sensorDelay[SENSOR_ENUM_COUNT];
uint16_t getSensorDelay(uint8_t sensorId);

//typedef enum
//{
//	ANDROID_BROADCAST_TIMER = 0,


//	ANDROID_NUM_TIMERS,
//} ANDROID__TIMERS;

//#define ANDROID_BROADCAST_RATE_MILISECONDS 5000

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



uint8_t getLastCommand(void);


void init_android_broadcast_buffers(void);
uint8_t  process_command(uint8_t *rx_buf, uint16_t length);
uint8_t post_on_boardcast_buffer(uint8_t *buffer, uint16_t length, uint32_t timeStamp);

uint8_t isHostInterruptAsserted(void);
void activateHostInterrupt(void);
void deactivateHostInterrupt(void);
void hostCommitDataTx(void);
void configureTimeCaptureTimers (void);
void configureSensorsTimeCapture (void);
void getTimeCapture(SensorType_t sensorId, uint32_t *result);
void calculate_commited_tx_buffer_size(void);

#endif /* ANDROIDHOSTINTERFACE_H_ */
