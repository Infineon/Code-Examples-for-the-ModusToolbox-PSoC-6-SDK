/*******************************************************************************
* File Name: debug.h
*
* Version: 1.00
*
* Description:
*  Contains the function prototypes and constants available to the code example
*  for debugging purposes.
*
********************************************************************************
* Copyright 2018-2019, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef DEBUG_H

    #define DEBUG_H

    #include <stdio.h>
	#include "stdio_user.h"
	#include "cycfg.h"
    #include "LED.h"

    #define DISABLED                    0
    #define ENABLED                     (!DISABLED)

    /***************************************
    * Conditional Compilation Parameters
    ***************************************/
    #define DEBUG_UART_ENABLED          DISABLED

    /***************************************
    *        External Function Prototypes
    ***************************************/
    void ShowError(void);

    /***************************************
    *        Macros
    ***************************************/
	#define UART_DEBUG_HW		KIT_UART_HW

    #if (DEBUG_UART_ENABLED == ENABLED)

		/** The instance-specific context structure.
		* It is used while the driver operation for internal configuration and
		* data keeping for the UART. The user should not modify anything in this
		* structure.
		*/
		cy_stc_scb_uart_context_t KIT_UART_context;

        __STATIC_INLINE void UART_DEBUG_START(void)
        {
            (void) Cy_SCB_UART_Init(KIT_UART_HW, &KIT_UART_config, &KIT_UART_context);
            Cy_SCB_UART_Enable(UART_DEBUG_HW);
        }

        #define DEBUG_PRINTF(...)               (printf(__VA_ARGS__))

        #define UART_DEBUG_GET_TX_BUFF_SIZE(...)  (Cy_SCB_GetNumInTxFifo(UART_DEBUG_HW) + Cy_SCB_GetTxSrValid(UART_DEBUG_HW))

        #define DEBUG_WAIT_UART_TX_COMPLETE()     while(UART_DEBUG_GET_TX_BUFF_SIZE() != 0);

    #else
        #define UART_DEBUG_START()

        #define DEBUG_PRINTF(...)

        #ifndef UART_DEBUG_GET_TX_FIFO_SR_VALID
            #define UART_DEBUG_GET_TX_FIFO_SR_VALID   (0u)
        #endif

        #define UART_DEBUG_GET_TX_BUFF_SIZE(...)      (0u)

        #define DEBUG_WAIT_UART_TX_COMPLETE()

    #endif /* (DEBUG_UART_ENABLED == ENABLED) */

#endif

/* [] END OF FILE */
