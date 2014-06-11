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
#include "common.h"
#include "hw_setup.h"
#include "i2c_slavecomm_t.h"


/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
#ifdef DEBUG_BUILD
char _errBuff[ERR_LOG_MSG_SZ];
#endif

/* Clock information */
RCC_ClocksTypeDef gRccClockInfo;

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    F O R W A R D   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/****************************************************************************************************
 * @fn      main
 *          Main entry point to the application firmware
 *
 * @param   none
 *
 * @return  none
 *
 ***************************************************************************************************/
int main( void )
{
    /**
     Get the clocks going and setup UART1 as the basic debug port for now. Debug messages will be
     handled in a separate task when the RTOS takes over.
     */

    /* NVIC configuration */
    SystemInterruptConfig();

    /* System clock configuration for regular mode */
    /* SystemClkConfig(0); */
    /* Clock is set up in "system_stm32L1xx.c" which is generated by an automatic clock configuration
      system and can be easily customized.
      To select different clock setup refer to Clock configuration tool for STM32L1xx microcontrollers (AN3309)
    */
    SystemCoreClockUpdate();

    /* Configure the GPIO ports (non module specific) */
    SystemGPIOConfig();

    /* Set startup state of LEDs */
    LED_Init();                   /* Initialize Debug LEDs */
    LED_On(FRONT_LED); /* Visual indication that we powered up */

    /* Configure RTC */
    RTC_Configuration();

    /* Configure debug UART port - we do it here to enable assert messages early in the system */
    DebugUARTConfig( DBG_UART_BAUD, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No );
    DebugPortInit();

    /* initialize the Host Slave I2C port */
    SH_Host_Slave_init();

    /* Print version number */
    printf("\r\n*** Open Sensor Platform Application Version: %s Date: %s - %s ***\r\n",
        APP_VERSION_STRING, __DATE__, __TIME__);

    /* Display System clock information */
    RCC_GetClocksFreq( &gRccClockInfo );
    printf("System Clocks:\r\n");
    printf("\tSYSCLK - %ld\r\n", gRccClockInfo.SYSCLK_Frequency);
    printf("\tHCLK   - %ld\r\n", gRccClockInfo.HCLK_Frequency);
    printf("\tPCLK1  - %ld\r\n", gRccClockInfo.PCLK1_Frequency);
    printf("\tPCLK2  - %ld\r\n", gRccClockInfo.PCLK2_Frequency);
    printf("\tADCCLK - %ld\r\n\n", gRccClockInfo.ADCCLK_Frequency);
    D0_printf("Chip UID: %08X-%08X-%08X\r\n", gDevUniqueId->uidWords[2],
        gDevUniqueId->uidWords[1], gDevUniqueId->uidWords[0]);

    /* Get the OS going - This must be the last call */
    AsfInitialiseTasks();

    /* If it got here something bad happened */
    ASF_assert_fatal(false);
}


/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
