/*
 * androidHostinterface.c
 *
 *  Created on: Mar 8, 2013
 *      Author: sungerfeld
 */
#include <stdint.h>
#include <string.h>

#include "common.h"
#include "osp-api.h"

#include "HostInterface.h"
#include "HostFunctions.h"
#include "i2c_slavecomm_t.h"
//#include "Algorithm_T.h"
//#include "SensorAcq_T.h"

//#include "msp430_board.h"


uint64_t sensorEnable = 0;


/****************************************************************************************************
 * @fn      SendSensorEnabledIndication
 * @brief  This helper function sends Sensor Enabled indication to Sensor Acq task. Called from ISR
 * @param  sensorId: Sensor identifier whose data is ready to be read
 * @param  enabled: boolean (0/1) to indicate sensor is disabled/enabled
 * @return None
 *
 ***************************************************************************************************/
static void AlgSendSensorEnabledIndication( uint8_t sensorId, uint8_t enabled)
{
    MessageBuffer *pSendMsg = NULLP;

    ASF_assert( ASFCreateMessage( MSG_SENSOR_ENABLE_DATA, sizeof(MsgSensorEnable), &pSendMsg ) == ASF_OK );
    pSendMsg->msg.msgSensorEnable.sensorId = sensorId;
    pSendMsg->msg.msgSensorEnable.enabled = enabled ? 1 : 0;
    ASFSendMessage( ALGORITHM_TASK_ID, pSendMsg);
}


static void controlSensorEnable(uint8_t sensorId, uint8_t enable) {
    uint8_t update = 0;
    if ((sensorId < sizeof(sensorEnable) * 8) && (sensorId < SENSOR_ENUM_COUNT)) {
        if (enable) {
            if (!(sensorEnable & (1L << sensorId))) {
                sensorEnable |= (1L << sensorId);
                update = 1;
            }
        } else {
            if (sensorEnable & (1L << sensorId)) {
                sensorEnable &= ~(1L << sensorId);
                update = 1;
            }
        }
        if (update) {
            AlgSendSensorEnabledIndication( sensorId,  enable ? 1 : 0);
        }
    }
}

uint8_t isSensorEnable(uint8_t sensorId) {
    if ((sensorId < sizeof(sensorEnable) * 8) && (sensorId < SENSOR_ENUM_COUNT)) {
        return (sensorEnable & (1L << sensorId)) ? 1 : 0;
    }
    return 0;
}

uint8_t areSensorEnable(uint64_t mask) {
    return (sensorEnable & mask) ? 1 : 0;
}


/****************************************************************************************************
 * @fn      SendSensorDelayIndication
 * @brief  This helper function sends Sensor Delay setting indication to Sensor Acq task. Called from ISR
 * @param  sensorId: Sensor identifier whose data is ready to be read
 * @param  delayMiliSec: sample time in mili-seconds (aka delay)
 * @return None
 *
 ***************************************************************************************************/
static void SensorAcqSendSensorDelayIndication( uint8_t sensorId, uint16_t delayMiliSec)
{
    MessageBuffer *pSendMsg = NULLP;

    ASF_assert( ASFCreateMessage( MSG_SENSOR_DELAY_DATA, sizeof(MsgSensorDelay), &pSendMsg)  == ASF_OK );
    pSendMsg->msg.msgSensorDelay.sensorId = sensorId;
    pSendMsg->msg.msgSensorDelay.delayMiliSec = delayMiliSec;
    ASFSendMessage( SENSOR_ACQ_TASK_ID, pSendMsg);
}


uint16_t sensorDelay[SENSOR_ENUM_COUNT] = {0};

static void controlSensorDelay(uint8_t sensorId, uint16_t miliSecondsDelay) {
    if ((sensorId < sizeof(sensorEnable) * 8) && (sensorId < SENSOR_ENUM_COUNT)) {
        if (miliSecondsDelay != sensorDelay[sensorId]) {
            sensorDelay[sensorId] = miliSecondsDelay;
            SensorAcqSendSensorDelayIndication( sensorId,  miliSecondsDelay);
        }
    }
}

uint16_t getSensorDelay(uint8_t sensorId) {
    if ((sensorId < sizeof(sensorEnable) * 8) && (sensorId < SENSOR_ENUM_COUNT)) {
        return sensorDelay[sensorId] ;
    }
    return 0;
}

uint16_t timeStampExpansion;
uint16_t broadcast_buf_wr = 0, broadcast_buf_rd = 0;


static SH_RegArea_t SlaveRegMap;

uint8_t broadcast_buf[SLAVE_NUM_TX_BUFFERS][OSP_HOST_MAX_BROADCAST_BUFFER_SIZE];
uint8_t broadcast_buf_flags[SLAVE_NUM_TX_BUFFERS];
uint16_t broadcast_buf_used[SLAVE_NUM_TX_BUFFERS];
uint16_t last_transmitted_offset[SLAVE_NUM_TX_BUFFERS];
uint8_t broadcastPacketNextTransmittedIndex[SLAVE_NUM_TX_BUFFERS];



// 256 / 5 (spi_sh_motion_sensor_broadcast_delta_time1_data_node (size of 5) is smallest packet possible)
// leave one extra entry for end of used buffer.
#define MAX_SENSOR_PACKETS_IN_BROADCAST_BUFFER \
	((OSP_HOST_MAX_BROADCAST_BUFFER_SIZE / (sizeof(struct spi_sh_motion_sensor_broadcast_delta_time1_data_node))) + 1)



uint16_t commited_length;
uint16_t commited_index;
uint8_t hostInterruptState;

uint8_t sendfullNode;

uint8_t androidBroadcastTrigger = 0;

union spi_pack
{
	uint16_t shortVal[2];
	uint32_t val;
}lastTimeValue[SENSOR_ENUM_COUNT];

void init_android_broadcast_buffers(void)
{
    

	// mark all TX buffers empty
#ifdef USE_I2C_SLAVE_DRIVER
	memset(broadcast_buf_flags,       0, sizeof(broadcast_buf_flags));			// all buffers unused
#endif
	commited_length = 0;

	broadcast_buf_wr = 0;
	broadcast_buf_rd = 0;

	memset(broadcast_buf_used,        0, sizeof(broadcast_buf_used));			// all buffers have no data

	memset(last_transmitted_offset, 0, sizeof(last_transmitted_offset));		// nothing TX from any buffer
	memset(broadcastPacketNextTransmittedIndex,    0, sizeof(broadcastPacketNextTransmittedIndex));		// nothing commited in any buffer

	memset(lastTimeValue,         0, sizeof(lastTimeValue));					// no time reported



#
	androidBroadcastTrigger = 0;

	timeStampExpansion = 0;
    deactivateHostInterrupt();

	hostInterruptState = 0;

	sendfullNode = 0;
}


uint8_t post_on_boardcast_buffer(uint8_t *buffer, uint16_t length, uint32_t timeStamp)
{
	uint8_t err = 0;

    OS_SETUP_CRITICAL();
	OS_ENTER_CRITICAL();
   
	if (broadcast_buf_used[broadcast_buf_wr] < last_transmitted_offset[broadcast_buf_wr] )
	{
//		boardCountPort6Bit(DEBUG6_GPIO_WR_PKT, broadcast_buf_wr + 1);
		while(1);
	}

	// if current WR buffer not full
	if (broadcast_buf_flags[broadcast_buf_wr])
	{
		// no space in buffer, record is loss
//		boardToggleLostPacketBit();
		err = 1;
	}
	else
	{
		if (broadcast_buf_used[broadcast_buf_wr] == last_transmitted_offset[broadcast_buf_wr])
		{
			sendfullNode = 0;
		}

        length += 2;      // add sensorId & compression bytes

		// if current WR buffer not full
		if ((broadcast_buf_used[broadcast_buf_wr] + length) > OSP_HOST_MAX_BROADCAST_BUFFER_SIZE)
		{
			// if no room for new record, mark current WR buffer as full
			broadcast_buf_flags[broadcast_buf_wr] = 1;

			// point to next circular TX buffer
			broadcast_buf_wr = (++broadcast_buf_wr) % SLAVE_NUM_TX_BUFFERS;

//			boardCountPort6Bit(DEBUG6_GPIO_WR_PKT, broadcast_buf_wr + 1);

			// if current WR buffer not full
			if (broadcast_buf_flags[broadcast_buf_wr])
			{
				// no space in buffer, record is loss
//				boardToggleLostPacketBit();
				err = 1;

           		activateHostInterrupt();
			}
		}
		else
		{
//			boardCountPort6Bit(DEBUG6_GPIO_WR_PKT, broadcast_buf_wr + 1);
		}

		if (!err)
		{
			// copy new record into buffer
			memcpy(&broadcast_buf[broadcast_buf_wr][broadcast_buf_used[broadcast_buf_wr]], buffer, length);
			// set current used size to length
			broadcast_buf_used[broadcast_buf_wr] += length;

#ifdef TIMER_TRIGGER_BROADCAST        
            if (androidBroadcastTrigger)
            {
                androidBroadcastTrigger = 0;
                if (commited_length == 0)
                {
                    calculate_commited_tx_buffer_size();
                }
            }
#else
            activateHostInterrupt();
#endif  // TIMER_TRIGGER_BROADCAST
		}
	}
  	OS_LEAVE_CRITICAL();
	return err;
}



void  calculate_commited_tx_buffer_size(void)
{
	commited_length = 0;

//	boardCountPort6Bit(DEBUG6_GPIO_RD_PKT, broadcast_buf_rd + 1);

	if (broadcast_buf_used[broadcast_buf_rd] < last_transmitted_offset[broadcast_buf_rd] )
	{
		while(1);
	}

	// if buffer writting is done, and nothing left in it, goto next buffer
	if ((broadcast_buf_flags[broadcast_buf_rd]) &&
		(last_transmitted_offset[broadcast_buf_rd] == broadcast_buf_used[broadcast_buf_rd]))
	{
		last_transmitted_offset[broadcast_buf_rd] = 0;
		broadcastPacketNextTransmittedIndex[broadcast_buf_rd] = 0;

		broadcast_buf_used[broadcast_buf_rd] = 0;

		broadcast_buf_flags[broadcast_buf_rd] = 0;

		// point to next circular TX buffer
		broadcast_buf_rd = (++broadcast_buf_rd) % SLAVE_NUM_TX_BUFFERS;

//		boardCountPort6Bit(DEBUG6_GPIO_RD_PKT, broadcast_buf_rd + 1);
	}



	// if buffer has some records
	if (broadcast_buf_used[broadcast_buf_rd] <= last_transmitted_offset[broadcast_buf_rd])
	{
		commited_length = 0;
		commited_index = 0;
	}
	else
		commited_length = broadcast_buf_used[broadcast_buf_rd] - last_transmitted_offset[broadcast_buf_rd];

	if (commited_length > 0)
		activateHostInterrupt();
	else
		deactivateHostInterrupt();
}


void send_commited_tx_buffer_size(void)
{

//	boardCountPort6Bit(DEBUG6_GPIO_RD_PKT, broadcast_buf_rd + 1);

	if (broadcast_buf_used[broadcast_buf_rd] < last_transmitted_offset[broadcast_buf_rd] )
	{
		while(1);
	}

	// if buffer writting is done, and nothing left in it, goto next buffer
	if ((broadcast_buf_flags[broadcast_buf_rd]) &&
		(last_transmitted_offset[broadcast_buf_rd] == broadcast_buf_used[broadcast_buf_rd]))
	{
		last_transmitted_offset[broadcast_buf_rd] = 0;
		broadcastPacketNextTransmittedIndex[broadcast_buf_rd] = 0;

		broadcast_buf_used[broadcast_buf_rd] = 0;

		broadcast_buf_flags[broadcast_buf_rd] = 0;

		// point to next circular TX buffer
		broadcast_buf_rd = (++broadcast_buf_rd) % SLAVE_NUM_TX_BUFFERS;

//		boardCountPort6Bit(DEBUG6_GPIO_RD_PKT, broadcast_buf_rd + 1);
	}

	// if buffer has some records
	if (broadcast_buf_used[broadcast_buf_rd] <= last_transmitted_offset[broadcast_buf_rd])
	{
		commited_length = 0;
		commited_index = 0;
	}
#ifdef USE_I2C_SLAVE_DRIVER
	setup_i2c_Tx((uint8_t *)&commited_length, sizeof(commited_length));
#endif

#ifdef USE_SPI_SLAVE_DRIVER
	setup_spi_Tx((uint8_t *)&commited_length, sizeof(commited_length));
#endif
}


void hostCommitDataTx(void)
{
//	boardCountPort6Bit(DEBUG6_GPIO_RD_PKT, broadcast_buf_rd + 1);

	if (broadcast_buf_used[broadcast_buf_rd] < last_transmitted_offset[broadcast_buf_rd] )
	{
		while(1);
	}

	last_transmitted_offset[broadcast_buf_rd] += commited_length;

	// set current used size to length
	if (last_transmitted_offset[broadcast_buf_rd] >= broadcast_buf_used[broadcast_buf_rd])
	{
		if (broadcast_buf_flags[broadcast_buf_rd])
		{
			last_transmitted_offset[broadcast_buf_rd] = 0;
			broadcastPacketNextTransmittedIndex[broadcast_buf_rd] = 0;

			broadcast_buf_used[broadcast_buf_rd] = 0;
			broadcast_buf_flags[broadcast_buf_rd] = 0;

			// point to next circular TX buffer
			broadcast_buf_rd = (++broadcast_buf_rd) % SLAVE_NUM_TX_BUFFERS;

//			boardCountPort6Bit(DEBUG6_GPIO_RD_PKT, broadcast_buf_rd + 1);
		}
	}
}

/**
* @brief <b>Description:</b> Processes any pending read requests
*
**/ 

static uint8_t currentOpCode;

uint8_t  process_command(uint8_t *rx_buf, uint16_t length)
{
    uint8_t remaining = 0;
	int sensorId = 0;
	
//	WorkerMessage message;

	if  (length)
	{
		currentOpCode = rx_buf[0];
    }
    switch (length ) {
    case 1:
        switch (currentOpCode)
        {
        case OSP_HOST_GET_BROADCAST_LENGTH:
			calculate_commited_tx_buffer_size();
			setup_I2c_Tx((uint8_t *)&commited_length, sizeof(commited_length));
            break;
        case OSP_HOST_GET_BROADCAST_DATA:
			setup_I2c_Tx(&broadcast_buf[broadcast_buf_rd][last_transmitted_offset[broadcast_buf_rd]], commited_length);
            break;
        case OSP_HOST_GET_WHO_AM_I:
            //D0_printf("I2C:WHO_AM_I\r\n");
            setup_I2c_Tx((uint8_t *)&SlaveRegMap.whoami, (uint16_t) sizeof(SlaveRegMap.whoami));
             break;
		case OSP_HOST_RESET:
			{
				for (sensorId = 0; sensorId < SENSOR_ENUM_COUNT; sensorId++)
					controlSensorEnable(sensorId, false);

				init_android_broadcast_buffers();
			}
			break;
         case OSP_HOST_GET_VERSION:
            setup_I2c_Tx((uint8_t *)&SlaveRegMap.version, (uint16_t) sizeof(SlaveRegMap.version));
            break;
        case OSP_HOST_SENSOR_SET_ENABLE:
            setup_I2c_Tx(NULL, 0);
            remaining = 2;              // sensorId, enable boolean byte
            break;
        case OSP_HOST_SENSOR_GET_DELAY:
        case OSP_HOST_SENSOR_GET_ENABLE:
            setup_I2c_Tx(NULL, 0);
            remaining = 1;              // sensor id
            break;
        case OSP_HOST_SENSOR_SET_DELAY:
            setup_I2c_Tx(NULL, 0);
            remaining = 3;              // sensor id, 16 bit delay
            break;

# if defined TRANSMIT_CAL_TO_SH

        case SPI_SH_SENSOR_SET_CALIBRATE:
        case SPI_SH_SENSOR_GET_CALIBRATE:
# endif
        default:
            setup_I2c_Tx(NULL, 0);
            break;
        }
        break;
    case 2:
        switch (currentOpCode)
        {
        case OSP_HOST_SENSOR_GET_DELAY:
            SlaveRegMap.shortResult = getSensorDelay(rx_buf[1]);
            setup_I2c_Tx((uint8_t *)&SlaveRegMap.shortResult, sizeof(SlaveRegMap.shortResult));
            remaining = 1;
            break;
        case OSP_HOST_SENSOR_GET_ENABLE:
            SlaveRegMap.booleanResult = isSensorEnable(rx_buf[1]);
            setup_I2c_Tx(&SlaveRegMap.booleanResult, sizeof(SlaveRegMap.booleanResult));
            remaining = 1;
            break;
        }
        break;
    case 3:
        switch (currentOpCode)
        {
        case OSP_HOST_SENSOR_SET_ENABLE:
            controlSensorEnable(rx_buf[1], rx_buf[2] ? 1 : 0);
            setup_I2c_Tx(NULL, 0);
            remaining = 1;
            break;
        }
        break;
    case 4:
        switch (currentOpCode)
        {
        case OSP_HOST_SENSOR_SET_DELAY:
            controlSensorDelay(rx_buf[1], (uint16_t) rx_buf[2] | ((uint16_t) rx_buf[3] << 8));
            setup_I2c_Tx(NULL, 0);
            remaining = 1;
            break;
        }
        break;
    default:
        remaining = 1;
        break;
    }
    return remaining;
}

#ifdef TIMER_TRIGGER_BROADCAST        
/* Define a callback function that will be used by multiple timer instances.
The callback function does nothing but count the number of times the
associated timer expires, and stop the timer once the timer has expired
10 times. */
void androidTimerCallback( xTimerHandle pxTimer )
{
	long lArrayIndex;

    /* Which timer expired? */
    lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

    switch (lArrayIndex)
    {
    case ANDROID_BROADCAST_TIMER:
//    	boardToggleHostWakeupBit();
    	if (!isHostInterruptAsserted())
    	{
    		androidBroadcasetrigger = 1;
    	}
    	break;
    }

    /* If the timer has expired 10 times then stop it from running. */
}
#endif


void activateHostInterrupt(void)
{
	if (!isHostInterruptAsserted())
	{
		SensorHubIntHigh();
		hostInterruptState = 1;
	}
}


void deactivateHostInterrupt(void)
{
	SensorHubIntLow();
	hostInterruptState = 0;
}

uint8_t isHostInterruptAsserted(void)
{
    return isSensorHubIntHigh();
}

uint8_t getLastCommand(void)
{
	return currentOpCode;
}

