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
#include <string.h>

#include "common.h"
#include "osp_HostInterface.h"
#include "HostFunctions.h"
#include "osp-sensors.h"
#include "i2c_slavecomm_t.h"

/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/
extern AsfTaskHandle asfTaskHandleTable[];
extern uint32_t AccelTimeExtend;
extern uint32_t MagTimeExtend;
extern uint32_t GyroTimeExtend;
extern uint32_t QuatTimeExtend;

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/
#define TX_LENGTH                   32
#define RX_LENGTH                   32
#define I2C_SLAVE_ADDR_8BIT         (I2C_SLAVE_ADDRESS << 1)

#define I2C_SLAVE_XFER_DONE         0x10
#define I2C_OVERREACH_VAL           0xCC //This value is sent for access beyond register area

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
typedef struct I2CXferTag {
    const uint8_t *txBuff;  /* Pointer to array of bytes to be transmitted */
    uint32_t  txCnt;        /* Current transfer transmitted bytes */
    uint32_t  txCntMax;     /* Number of bytes in transmit array,
                               if 0 only receive transfer will be carried on */

    uint32_t  rxCnt;        /* Received bytes count */
    uint32_t  rxCntMax;			/* max bytes expected, max of  RX_LENGTH */
    uint8_t   rxData[RX_LENGTH];	/* storage for Rx data */
    
    HostProcessingState hostProcessState;
} I2CXfer_t;

/*-------------------------------------------------------------------------------------------------*\
 |    S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
static I2CXfer_t slave_xfer;

/*-------------------------------------------------------------------------------------------------*\
 |    F O R W A R D   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/
static void I2C_Slave_Handler_Process_TX(I2C_TypeDef *pI2C);
static void I2C_Slave_Handler_Process_RX(I2C_TypeDef *pI2C);

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/
/****************************************************************************************************
 * @fn      I2C_Slave_Initialise
 *          Call this function to set up the I2C slave to its initial standby state.
 *
 ***************************************************************************************************/
static void I2C_Slave_Initialise(void)
{
    I2C_InitTypeDef   I2C_InitStructure;

    /* Enable the bus */
    I2C_Cmd(I2C_SLAVE_BUS, ENABLE);

    /* Configure the I2C interface */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDR_8BIT;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C_SLAVE_BUS_CLOCK;
    I2C_Init(I2C_SLAVE_BUS, &I2C_InitStructure);
    I2C_StretchClockCmd(I2C_SLAVE_BUS, ENABLE);

    /* Enable interrupts and clear flags */
    I2C_ITConfig( I2C_SLAVE_BUS, (I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR), ENABLE );
    I2C_ClearITPendingBit( I2C_SLAVE_BUS, I2C_IT_SMBALERT | I2C_IT_TIMEOUT | I2C_IT_PECERR | I2C_IT_OVR
        | I2C_IT_AF | I2C_IT_ARLO | I2C_IT_BERR );
}


/****************************************************************************************************
 * @fn      I2C_Slave_hardware
 *          Initialize the Sensor Hub I2C Slave hardware
 *
 ***************************************************************************************************/
static void SH_Host_Slave_hardware_init( void )
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    /* Reset the peripheral (this allows soft restarts to recover the bus in case of hangups) */
    I2C_DeInit(I2C_SLAVE_BUS);

    /* Enable Clocks to the peripheral and GPIOs used */
    RCC_APB1PeriphClockCmd(RCC_Periph_I2C_SLAVE_BUS, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_Periph_I2C_SLAVE_BUS_GPIO, ENABLE ); //for I2C port GPIO

    /* NVIC/Interrupt config */
    /* Configure and enable I2C event interrupt -------------------------------*/
    NVIC_InitStructure.NVIC_IRQChannel = I2C_SLAVE_BUS_EVENT_IRQ_CH;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_SLAVE_BUS_INT_PREEMPT_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_SLAVE_BUS_EVENT_INT_SUB_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Configure and enable I2C error interrupt -------------------------------*/  
    NVIC_InitStructure.NVIC_IRQChannel = I2C_SLAVE_BUS_ERROR_IRQ_CH;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_SLAVE_BUS_ERROR_INT_SUB_PRIORITY;
    NVIC_Init(&NVIC_InitStructure);

    /* GPIO Configuration for CLK and SDA signals */
    GPIO_InitStructure.GPIO_Pin = I2C_SLAVE_BUS_CLK_PIN  | I2C_SLAVE_BUS_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( I2C_SLAVE_BUS_GPIO_GRP, &GPIO_InitStructure );

    /* Sensor Hub Host interrupt pin */
    RCC_APB2PeriphClockCmd(SH_INT_RCC, ENABLE );
    GPIO_InitStructure.GPIO_Pin = SH_INT_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //Because we are interfacing 3.3V to 1.8V
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( SH_INT_GPIO_GRP, &GPIO_InitStructure );

    /* Initialize the I2C slave interface */
    I2C_Slave_Initialise();
}

/****************************************************************************************************
 * @fn      SH_Host_Slave_software_init
 *          Initialize the Sensor Hub I2C Slave register interface
 *
 ***************************************************************************************************/
static void SH_Host_Slave_software_init(void)
{
    memset(&slave_xfer, 0, sizeof(slave_xfer));
    slave_xfer.hostProcessState = HOST_PROCESSING_NONE;    
}

/****************************************************************************************************
 * @fn      SH_Slave_init
 *          Initialize the Sensor Hub I2C Slave register interface
 *
 ***************************************************************************************************/
void SH_Host_Slave_init(void)
{
    SH_Host_Slave_software_init();
    SH_Host_Slave_hardware_init();
}

/****************************************************************************************************
 * @fn      SH_Host_Slave_cmd_processing_active
 * @brief   Signal that command processing is active. this will enable i2C clock stretching if need be
 *
 ***************************************************************************************************/
void SH_Host_Slave_cmd_processing_active(void) {
    slave_xfer.hostProcessState = HOST_PROCESSING_IN_PROGRESS;
}


/****************************************************************************************************
 * @fn      SH_Host_Slave_terminate_cmd_processing
 * @brief   Signal termination of command processing. terminates i2C clock stretching if in progress.
 *
 ***************************************************************************************************/
void SH_Host_Slave_terminate_cmd_processing(void) {

    OS_SETUP_CRITICAL();

    /* block interrupts to make sure clock stretching can be avoided if it did not occur yet */
    OS_ENTER_CRITICAL();
    
    switch (slave_xfer.hostProcessState) {              // look into current state
        /* nothing firther to do if clock stretch did not start */
    case HOST_PROCESSING_IN_PROGRESS:
        slave_xfer.hostProcessState = HOST_PROCESSING_NONE;
        /* falls through */
    case HOST_PROCESSING_NONE:
        OS_LEAVE_CRITICAL();
        break;
    case HOST_PROCESSING_STRETCHED_TX:
        I2C_Slave_Handler_Process_TX(I2C_SLAVE_BUS);
        slave_xfer.hostProcessState = HOST_PROCESSING_NONE;
        OS_LEAVE_CRITICAL();
        break;
    case HOST_PROCESSING_STRETCHED_RX:
        I2C_Slave_Handler_Process_RX(I2C_SLAVE_BUS);
        slave_xfer.hostProcessState = HOST_PROCESSING_NONE;
        OS_LEAVE_CRITICAL();
        break;
    }
}

/****************************************************************************************************
 * @fn      SH_Slave_setup_I2c_Tx
 *          sets up the I2C Slave TX buffer for next Slave Read operation
 *
 *
 * @param   address - address of buffer to transmit
 * @param   size    - size of buffer to transmit
 *
 * @return  none
 *
 ***************************************************************************************************/
void SH_Slave_setup_I2c_Tx(uint8_t *address, uint16_t size) {
	slave_xfer.txBuff = address;
    slave_xfer.txCnt = 0;
    slave_xfer.txCntMax = size;
}

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

uint32_t i2cslvstates[50] = {0};
int vs = 0;



static void I2C_Slave_Handler_Process_RX(I2C_TypeDef *pI2C) {
    
    if ((slave_xfer.rxCnt > 0) && (slave_xfer.rxCnt >= slave_xfer.rxCntMax)) {
            //Not supported at this time so just NACK it for now
        uint8_t tmp = (uint8_t) I2C_ReceiveData(pI2C);
        slave_xfer.rxCnt = 0;
        I2C_AcknowledgeConfig(pI2C, DISABLE);
        return;
    }
    slave_xfer.rxData[slave_xfer.rxCnt++] = (uint8_t) I2C_ReceiveData(pI2C);

    /* if first byte - it contains the command opcode */
    if (slave_xfer.rxCnt == 1) {
        /* decode command - calculate length of command structure */
        slave_xfer.rxCntMax = process_command(slave_xfer.rxData, slave_xfer.rxCnt);
        /* if it is a single byte command - we are done, otherwise wait until all bytes received */
        if (slave_xfer.rxCnt >= slave_xfer.rxCntMax) {
            slave_xfer.rxCnt = 0;
            slave_xfer.rxCntMax = 0;
        }
    } else {
        /* if all command bytes received */
        if (slave_xfer.rxCnt == slave_xfer.rxCntMax) {
            /* process the incomming conmmand */
            process_command(slave_xfer.rxData, slave_xfer.rxCnt);
            /* done with command */
            slave_xfer.rxCnt = 0;
            slave_xfer.rxCntMax = 0;
        }
    }
}

static void I2C_Slave_Handler_Process_TX(I2C_TypeDef *pI2C) {
    /* In transmit we will do not want to send data beyond the register set */

    /* In transmit we will do not want to send data beyond specified region */
    if ((slave_xfer.txBuff != NULL) && (slave_xfer.txCnt < slave_xfer.txCntMax))
    {
        I2C_SendData(pI2C, slave_xfer.txBuff[slave_xfer.txCnt++]);
        if (slave_xfer.txCnt >= slave_xfer.txCntMax) {
            SH_Slave_setup_I2c_Tx(NULL, 0);
            if (getLastCommand() == OSP_HOST_GET_BROADCAST_DATA) {
                hostCommitDataTx();
                calculate_commited_tx_buffer_size();
            }
        }
    }
    else
    {
        I2C_SendData(pI2C, I2C_OVERREACH_VAL); //This value signifies read beyond allowed range
        SH_Slave_setup_I2c_Tx(NULL, 0);
    }
}

/****************************************************************************************************
 * @fn      I2C_Slave_Handler
 *          Handles the I2C Slave communication events
 *
 ***************************************************************************************************/
void I2C_Slave_Handler(I2C_TypeDef *pI2C, uint8_t irqCh)
{
    uint32_t i2cslvstate = I2C_GetLastEvent(pI2C);
    
    if (vs < 50) i2cslvstates[vs++] =  i2cslvstate;

    /* If error event - clear it */
    if ((irqCh == I2C_SLAVE_BUS_ERROR_IRQ_CH) && ((i2cslvstate & 0xFF00) != 0))
    {
        pI2C->SR1 &= 0x00FF;

        /* EV 3-2 while slave is TX , this is when master generates STOP*/
        if (i2cslvstate == (I2C_FLAG_BUSY | I2C_FLAG_TRA | I2C_EVENT_SLAVE_ACK_FAILURE))
        {
            /* Master NAKed - this was end of transaction when slave is transmitting */
            slave_xfer.rxCnt = 0;
            slave_xfer.rxCntMax = 0;
            SH_Slave_setup_I2c_Tx(NULL, 0);
            I2C_GetITStatus(pI2C, I2C_IT_STOPF);
            I2C_Cmd(I2C_SLAVE_BUS, ENABLE);
            return;
        }
    }

    /* Below three states are for Slave mode: Address Received, TX, and RX. */
    switch ( i2cslvstate )
    {
        /* stop of one command followed by start and slave address tightly. */
        /* result is getting both STOPF & ADDR , with BUSY */
    case (I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED | I2C_EVENT_SLAVE_STOP_DETECTED): /* 00020012 */
        I2C_GetITStatus(pI2C, I2C_IT_STOPF);    /* clear STOPF */
        I2C_Cmd(I2C_SLAVE_BUS, ENABLE);
        // falls through !!!
    case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED: /* 00020002 */
        slave_xfer.rxCnt = 0; /* I2C Read Mode - no more receive */
        slave_xfer.rxCntMax = 0;

        /* Nothing to do except continue */
        break;
        
    case (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_SR1_BTF): /*  00020044 RxNE + BTF - used when clock stretch is enabled    */
        if (slave_xfer.hostProcessState == HOST_PROCESSING_IN_PROGRESS) {   /* if command is being processed, cause clock stretch by delaying of byte read until processing is done */
            slave_xfer.hostProcessState = HOST_PROCESSING_STRETCHED_RX;
            break;
        }
    case  I2C_EVENT_SLAVE_BYTE_RECEIVED:                /* 00020040  RxNE */
        I2C_Slave_Handler_Process_RX(pI2C);
        break;

    case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED:  /* 00060082 */
        slave_xfer.rxCnt = 0; /* I2C Read Mode - no more receive */
        slave_xfer.rxCntMax = 0;
        
        /* Fall thru */
    case I2C_EVENT_SLAVE_BYTE_TRANSMITTED:  /* 00060084 TxE + BTF - used when clock stretch is enabled*/
        if (slave_xfer.hostProcessState == HOST_PROCESSING_IN_PROGRESS) {   /* if command is being processed, cause clock stretch by delaying of byte write until processing is done */
            slave_xfer.hostProcessState = HOST_PROCESSING_STRETCHED_TX;
            break;
        }
    case I2C_EVENT_SLAVE_BYTE_TRANSMITTING: /* 00060080 TxE       */
    case 0x00060004: //Sometimes we get this event and the driver is stuck in ISR!
        I2C_Slave_Handler_Process_TX(pI2C);
      
        break;

    case I2C_EVENT_SLAVE_STOP_DETECTED: /* This is end of transaction when slave is receiving */
        /* if (STOPF==1) Read SR1;Write CR1 */
       slave_xfer.rxCnt = 0;
        slave_xfer.rxCntMax = 0;
        /* Re-enable ACK (in case it was disabled) */
        I2C_GetITStatus(pI2C, I2C_IT_STOPF);
        I2C_Cmd(I2C_SLAVE_BUS, ENABLE);
        I2C_AcknowledgeConfig(pI2C, ENABLE);
        break;

    default:
        break;
    }
}


/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
