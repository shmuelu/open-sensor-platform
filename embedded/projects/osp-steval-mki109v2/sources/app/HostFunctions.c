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

#include <stdint.h>
#include <string.h>

#include "common.h"
#include "osp-api.h"
#include "osp-sensors.h"

#include "osp_HostInterface.h"
#include "HostFunctions.h"
#include "i2c_slavecomm_t.h"


/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

uint16_t timeStampExpansion;
uint16_t broadcast_buf_wr = 0, broadcast_buf_rd = 0;



uint8_t broadcast_buf[SLAVE_NUM_TX_BUFFERS][OSP_HOST_MAX_BROADCAST_BUFFER_SIZE];
uint8_t broadcast_buf_flags[SLAVE_NUM_TX_BUFFERS];
uint16_t broadcast_buf_used[SLAVE_NUM_TX_BUFFERS];
uint16_t last_transmitted_offset[SLAVE_NUM_TX_BUFFERS];
uint8_t broadcastPacketNextTransmittedIndex[SLAVE_NUM_TX_BUFFERS];

uint16_t commited_length;
uint16_t commited_index;
uint8_t hostInterruptState;

uint8_t sendfullNode;

uint8_t androidBroadcastTrigger = 0;

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/
// 256 / 5 (spi_sh_motion_sensor_broadcast_delta_time1_data_node (size of 5) is smallest packet possible)
// leave one extra entry for end of used buffer.
#define MAX_SENSOR_PACKETS_IN_BROADCAST_BUFFER \
	((OSP_HOST_MAX_BROADCAST_BUFFER_SIZE / (sizeof(struct spi_sh_motion_sensor_broadcast_delta_time1_data_node))) + 1)

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/* Register Area for slave device */
typedef struct SH_RegArea_tag
{
    uint8_t whoami;
    uint16_t version;
    struct  ShCmdGetEnableHeader_t enable;
    struct ShCmdGetHeader_get_16bits_param_t delay;
} SH_RegArea_t;

/*-------------------------------------------------------------------------------------------------*\
 |    S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
static SH_RegArea_t SlaveRegMap;
static uint8_t currentOpCode;

/****************************************************************************************************
 * @fn     AlgSendSensorEnabledIndication
 * @brief  This helper function sends Sensor Enabled indication to Sensor Acq task. Called from ISR
 * @param  sensorId: Sensor identifier whose data is ready to be read
 * @param  enabled: boolean (0/1) to indicate sensor is disabled/enabled
 * @return None
 *
 ***************************************************************************************************/
static void AlgSendSensorEnabledIndication( const struct SensorId_t *sensorId, uint8_t enabled)
{
    MessageBuffer *pSendMsg = NULLP;

    ASF_assert( ASFCreateMessage( MSG_SENSOR_ENABLE_DATA, sizeof(MsgSensorEnable), &pSendMsg ) == ASF_OK );
    pSendMsg->msg.msgSensorEnable.sensorId.sensorType = sensorId->sensorType;
    pSendMsg->msg.msgSensorEnable.sensorId.sensorSubType = sensorId->sensorSubType;
    pSendMsg->msg.msgSensorEnable.enabled = enabled ? 1 : 0;

    ASFSendMessage( ALGORITHM_TASK_ID, pSendMsg);
}


/****************************************************************************************************
 * @fn      controlSensorEnable
 *          Helper function check for current enable status on a sensor and generates request if
 *          change of state is needed.
 * @return  returns 1 if message sent to task, 0 otherwise
 ***************************************************************************************************/
static uint8_t controlSensorEnable(const struct SensorId_t *sensorId, osp_bool_t enable) {
    uint8_t update = 0;

    if (isSensorSubscribed(sensorId ) == OSP_STATUS_OK) {
        if (!enable) {
                update = 1;
        }
    } else {
        if (enable) {
            update = 1;
        }
    }
    if (update) {
        AlgSendSensorEnabledIndication( sensorId,  enable ? 1 : 0);
        return (1);
    }
    return (0);
}





/****************************************************************************************************
 * @fn      AlgSendSendSensorDelayIndication
 * @brief  This helper function sends Sensor Delay setting indication to ALG task. Called from ISR
 * @param  sensorId: Sensor identifier whose data is ready to be read
 * @param  delayMiliSec: sample time in mili-seconds (aka delay)
 * @return None
 *
 ***************************************************************************************************/
static void AlgSendSendSensorDelayIndication( const struct SensorId_t *sensorId, uint16_t delayMiliSec)
{
    MessageBuffer *pSendMsg = NULLP;

    ASF_assert( ASFCreateMessage( MSG_SENSOR_DELAY_DATA, sizeof(MsgSensorDelay), &pSendMsg)  == ASF_OK );
    pSendMsg->msg.msgSensorDelay.sensorId.sensorType = sensorId->sensorType;
    pSendMsg->msg.msgSensorDelay.sensorId.sensorSubType = sensorId->sensorSubType;
    pSendMsg->msg.msgSensorDelay.delayMiliSec = delayMiliSec;
    ASFSendMessage( ALGORITHM_TASK_ID, pSendMsg);
}


/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/****************************************************************************************************
 * @fn      init_android_broadcast_buffers
 *          Initializer for android host communication buffers
 *
 ***************************************************************************************************/
void init_android_broadcast_buffers(void)
{
    

	// mark all TX buffers empty
#ifdef USE_I2C_SLAVE_DRIVER
	memset(
        broadcast_buf_flags,
        0,
        sizeof(broadcast_buf_flags));			// all buffers unused
#endif
    memset(
        &SlaveRegMap,
        0,
        sizeof(SlaveRegMap));

    SlaveRegMap.version = (uint16_t)SH_VERSION0 | ((uint16_t) SH_VERSION1 << 8);
    SlaveRegMap.whoami   = SH_WHO_AM_I;

	commited_length = 0;

	broadcast_buf_wr = 0;
	broadcast_buf_rd = 0;

	memset(broadcast_buf_used,        0, sizeof(broadcast_buf_used));			// all buffers have no data

	memset(last_transmitted_offset, 0, sizeof(last_transmitted_offset));		// nothing TX from any buffer
	memset(broadcastPacketNextTransmittedIndex,    0, sizeof(broadcastPacketNextTransmittedIndex));		// nothing commited in any buffer

	androidBroadcastTrigger = 0;

	timeStampExpansion = 0;
    deactivateHostInterrupt();

	hostInterruptState = 0;

	sendfullNode = 0;
}



/****************************************************************************************************
 * @fn      post_on_broadcast_buffer
 *          desc
 *
 ***************************************************************************************************/
uint8_t post_on_boardcast_buffer(uint8_t *buffer, uint16_t length, struct Timestamp40_t *timeStamp)
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

/****************************************************************************************************
 * @fn      calculate_commited_tx_buffer_size
 *          desc
 *
 ***************************************************************************************************/
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


/****************************************************************************************************
 * @fn      hostCommitDataTx
 *          desc
 *
 ***************************************************************************************************/
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


/****************************************************************************************************
 * @fn      process_command
 *          Processes any pending read requests
 *
 ***************************************************************************************************/
uint8_t  process_command(uint8_t *rx_buf, uint16_t length)
{
    uint8_t cmdSize = 1;
    struct SensorId_t sensorId;
    struct ShSensorSetEnableCmdHeader_param_t *enableCommand;
    struct ShSensorSetDelayCmdHeader_t *delayCommand;
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
			SH_Slave_setup_I2c_Tx((uint8_t *)&commited_length, sizeof(commited_length));
            break;
        case OSP_HOST_GET_BROADCAST_DATA:
			SH_Slave_setup_I2c_Tx(&broadcast_buf[broadcast_buf_rd][last_transmitted_offset[broadcast_buf_rd]], commited_length);
            break;
        case OSP_HOST_GET_WHO_AM_I:
            //D0_printf("I2C:WHO_AM_I\r\n");
            SH_Slave_setup_I2c_Tx((uint8_t *)&SlaveRegMap.whoami, (uint16_t) sizeof(SlaveRegMap.whoami));
             break;
		case OSP_HOST_RESET:
			{
                uint8_t requestSentToTask = 0;
                SH_Host_Slave_cmd_processing_active();
				for (sensorId.sensorType = 0; sensorId.sensorType < SENSOR_ENUM_COUNT; sensorId.sensorType++) {
                    for (sensorId.sensorSubType = 0; ; sensorId.sensorSubType++) {
                        if (validateDeviceId(&sensorId) == OSP_STATUS_OK) {
                            if (controlSensorEnable(&sensorId, false)) {
                                requestSentToTask = 1;
                            }
                        } else {
                            break;
                        }
                    }
                }
				init_android_broadcast_buffers();
                /* if no message sent to task - terminate here */
                if (requestSentToTask == 0) {
                    SH_Host_Slave_terminate_cmd_processing();
                }
			}
			break;
         case OSP_HOST_GET_VERSION:
            SH_Slave_setup_I2c_Tx((uint8_t *)&SlaveRegMap.version, (uint16_t) sizeof(SlaveRegMap.version));
            break;
        case OSP_HOST_SENSOR_SET_ENABLE:
            SH_Slave_setup_I2c_Tx(NULL, 0);
            cmdSize = sizeof(struct ShSensorSetEnableCmdHeader_param_t);              // sensorType, sensorSubType, enable boolean byte
            break;
        case OSP_HOST_SENSOR_GET_DELAY:
        case OSP_HOST_SENSOR_GET_ENABLE:
            SH_Slave_setup_I2c_Tx(NULL, 0);
            cmdSize = sizeof(struct ShSensorCmdHeader_t);                             // sensorType, sensorSubType,
            break;
        case OSP_HOST_SENSOR_SET_DELAY:
            SH_Slave_setup_I2c_Tx(NULL, 0);
            cmdSize = sizeof(struct ShSensorSetDelayCmdHeader_t); // sensortype, sensorSubType, 16 bit delay
            break;

# if defined TRANSMIT_CAL_TO_SH

        case SPI_SH_SENSOR_SET_CALIBRATE:
        case SPI_SH_SENSOR_GET_CALIBRATE:
# endif
        default:
            SH_Slave_setup_I2c_Tx(NULL, 0);
            break;
        }
        break;
    case sizeof(struct ShSensorCmdHeader_t):
        switch (currentOpCode)
        {
        case OSP_HOST_SENSOR_GET_DELAY:
            {
                struct ShSensorCmdHeader_t *command = (struct ShSensorCmdHeader_t *)rx_buf;
                getSensorDelayMilliSeconds(&command->sensorId, &SlaveRegMap.delay.param );
                
                SH_Slave_setup_I2c_Tx((uint8_t *)&SlaveRegMap.delay, sizeof(SlaveRegMap.delay));
                cmdSize = COMMAND_PROCESS_GET;
            }
            break;
        case OSP_HOST_SENSOR_GET_ENABLE:
            {
                struct ShSensorCmdHeader_t *command = (struct ShSensorCmdHeader_t *)rx_buf;

                SlaveRegMap.enable.enable = (isSensorSubscribed(&command->sensorId) == OSP_STATUS_ERROR) ? TRUE : FALSE;     
                SH_Slave_setup_I2c_Tx((uint8_t *) &SlaveRegMap.enable.enable, sizeof(SlaveRegMap.enable.enable));
                cmdSize = COMMAND_PROCESS_GET;
            }
            break;
        default:
            cmdSize = COMMAND_PROCESS_INVALID;
            break;
        }
        break;
    case sizeof(struct ShSensorSetEnableCmdHeader_param_t):
        switch (currentOpCode)
        {
        case OSP_HOST_SENSOR_SET_ENABLE:
            {
                SH_Host_Slave_cmd_processing_active();  /* mark potential clock stretch */

                enableCommand = (struct ShSensorSetEnableCmdHeader_param_t *)rx_buf;
                if (controlSensorEnable(&enableCommand->sensorId, enableCommand->enable ? 1 : 0) == 0) {
                    SH_Host_Slave_terminate_cmd_processing(); /* if nothing was sent, terminate clock stretch posibility */
                }
                SH_Slave_setup_I2c_Tx(NULL, 0);
                cmdSize = COMMAND_PROCESS_SET;
            }
            break;
        default:
            cmdSize = COMMAND_PROCESS_INVALID;
            break;
        }
        break;
    case sizeof(struct ShSensorSetDelayCmdHeader_t):
        switch (currentOpCode)
        {
        case OSP_HOST_SENSOR_SET_DELAY:
            {
                SH_Host_Slave_cmd_processing_active();  /* mark potential clock stretch */

                delayCommand = (struct ShSensorSetDelayCmdHeader_t *)rx_buf;

                AlgSendSendSensorDelayIndication( &delayCommand->sensorId, delayCommand->delay_milisec);
                SH_Slave_setup_I2c_Tx(NULL, 0);
                cmdSize = COMMAND_PROCESS_SET;
            }
            break;
        default:
            cmdSize = COMMAND_PROCESS_INVALID;
            break;
        }
        break;
    default:
        cmdSize = COMMAND_PROCESS_INVALID;
        break;
    }
    return cmdSize;
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


/****************************************************************************************************
 * @fn      activateHostInterrupt
 *          Checks for interrupt state & asserts if not already asserted.
 *
 ***************************************************************************************************/
void activateHostInterrupt(void)
{
	if (!isHostInterruptAsserted())
	{
		SensorHubIntHigh();
		hostInterruptState = 1;
	}
}


/****************************************************************************************************
 * @fn      deactivateHostInterrupt
 *          De-asserts the host interrupt line
 *
 ***************************************************************************************************/
void deactivateHostInterrupt(void)
{
	SensorHubIntLow();
	hostInterruptState = 0;
}

/****************************************************************************************************
 * @fn      isHostInterruptAsserted
 *          Checks for interrupt state & returns the state
 *
 ***************************************************************************************************/
uint8_t isHostInterruptAsserted(void)
{
    return isSensorHubIntHigh();
}


/****************************************************************************************************
 * @fn      getLastCommand
 *          returns the last command
 *
 ***************************************************************************************************/
uint8_t getLastCommand(void)
{
	return currentOpCode;
}

