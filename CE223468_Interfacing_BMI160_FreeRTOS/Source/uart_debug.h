/******************************************************************************
* File Name: uart_debug.h
*
* Version 1.0
*
* Description: This file contains the macros that are used for UART based
*              debug
*
* Related Document: CE223468_Interfacing_BMI160_FreeRTOS.pdf
*
* Hardware Dependency: See CE223468_Interfacing_BMI160_FreeRTOS.pdf
*
*******************************************************************************
* Copyright (2019), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death (“High Risk Product”). By
* including Cypress’s product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

/* Include guard */
#ifndef SOURCE_UART_DEBUG_H_
#define SOURCE_UART_DEBUG_H_

/* Header file includes */
#include "cycfg_peripherals.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* (true) enables UART based debug and (false) disables it */
#define UART_DEBUG_ENABLE    (true)

/* Declare the macros used for UART based debug */
#if (UART_DEBUG_ENABLE)

    extern void DEBUG_UART_FastStart();

    /* UART component used for STDIO functions */
    #define UART_STDIO          (KIT_UART_HW)

    /* Following macros are not RTOS thread-safe, therefore, should not be
     * called from RTOS Tasks */

    /* Initializes the UART component used for STDIO functions */
    #define DebugPrintfInit()  (DEBUG_UART_FastStart())
    /* Function macro that sends formatted output to STDOUT */
    #define DebugPrintf(...)     (printf(__VA_ARGS__))

	/* RTOS thread-safe macros for initializing and printing debug
	 * information from RTOS Tasks */

	/* Initializes the underlying Task and Queue used for printing debug
	 * messages */
	#define Task_DebugInit()   (InitDebugPrintf())

    /* Prints a constant string and an optional error code from a separate
     * Task */
	#define Task_DebugPrintf(constString, errorCode) (SendToDebugPrintTask(\
													(char*)constString,\
													(uint32_t)errorCode))

	/* Definition of internal macros, datatypes and the inline functions
	 * used by the RTOS thread-safe macros for initializing and printing debug
	 * information from RTOS Tasks */

	/* Maximum number of debug messages that can be queued */
	#define DEBUG_QUEUE_SIZE    16u

	/* Datatype used for debug queue */
	typedef struct
	{
		char* stringPointer;
		uint32_t errorCode;
	}   debug_print_data_t;

	/* Queue handle for debug message Queue */
	extern QueueHandle_t debugMessageQ;

	/* Task that performs thread safe debug message printing */
	void Task_Debug(void *pvParameters);

	/* Inline function that creates the underlying Task and Queue  */
	void inline static InitDebugPrintf()
	{
		debugMessageQ = xQueueCreate(DEBUG_QUEUE_SIZE,
										sizeof(debug_print_data_t));
		xTaskCreate(Task_Debug, "Debug Task", configMINIMAL_STACK_SIZE, NULL,
					(tskIDLE_PRIORITY + 1u), NULL);
	}

	/* Inline function that sends messages to the debug Queue */
	void inline static SendToDebugPrintTask(char* stringPtr, uint32_t errCode)
	{
		debug_print_data_t printData = {.stringPointer = stringPtr,
										.errorCode     = errCode};
		xQueueSend(debugMessageQ, &printData,0u);
	}

/* Declaration of empty or default value macros if the debug is not enabled
 * for efficient code generation */
#else
    #define UART_STDIO              (NULL)
    #define DebugPrintfInit()
    #define DebugPrintf(...)
    #define Task_DebugInit()
    #define Task_DebugPrintf(...)
#endif /* UART_DEBUG_ENABLE */

#endif /* SOURCE_UART_DEBUG_H_ */
