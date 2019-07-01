/******************************************************************************
* File Name: uart_debug.c
*
* Version 1.0
*
* Description: This file contains the task that is used for thread-safe UART
*              based debug.
*
* Related Document: CE225909_PSoC6_BatteryLevel_RTOS.pdf
*
* Hardware Dependency: See CE225909_PSoC6_BatteryLevel_RTOS.pdf
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

/* Header file includes */
#include "cycfg.h"
#include "stdio.h"
#include <stdarg.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "uart_debug.h"

/* Queue length of message queue used for debug message printing */
#define DEBUG_QUEUE_SIZE 		(16u)

/* Maximum allowed length of debug message */
#define DEBUG_MESSAGE_MAX_LEN	(100u)

/* UART context */
cy_stc_scb_uart_context_t KIT_UART_context;

#if DEBUG_ENABLE

/* Queue handle for debug message Queue */
QueueHandle_t debugMessageQ;

/* Structure for storing debug message data */
typedef struct
{
	const char* stringPtr;
	debug_message_type_t messageType;
} debug_messaage_data_t;

/*******************************************************************************
* Function Name: void Task_Debug(void* pvParameters)
********************************************************************************
* Summary:
*  Task that prints debug messages.
*
* Parameters:
*  void *pvParameters : Task parameter defined during task creation (unused)
*
* Return:
*  None
*
*******************************************************************************/
void Task_Debug(void* pvParameters)
{
	debug_messaage_data_t messageData;

	/* Variable used to store the return values of RTOS APIs */
	BaseType_t rtosApiResult;

	/* Remove warning for unused parameter */
	(void) pvParameters;

	/* Repeatedly running part of the task */
	for(;;)
	{
		rtosApiResult = xQueueReceive(debugMessageQ, &messageData, portMAX_DELAY);

		if(rtosApiResult == pdPASS)
		{
			switch(messageData.messageType)
			{
				case none:
				{
					printf("%s", (char *)messageData.stringPtr);
					break;
				}
				case info:
				{
					printf("[Info]    : %s\r\n", (char *)messageData.stringPtr);
					break;
				}
				case warning:
				{
					printf("[Warning] : %s\r\n", (char *)messageData.stringPtr);
					break;
				}
				case error:
				{
					printf("[Error]   : %s\r\n", (char *)messageData.stringPtr);
					break;
				}
				default:
				{
					break;
				}
			}

			/* free the message buffer allocated by the message sender */
			vPortFree((char*)messageData.stringPtr);
		}
		/** Task has timed out and received no commands during an interval of
		 * portMAXDELAY ticks */
		else
		{
		}
	}
}

/*******************************************************************************
* Function Name: void Task_DebugPrintf(debug_message_type_t messageType,
* 					char* stringPtr, ...)
********************************************************************************
*
* Description: This function sends messages to the debug Queue.
*
*******************************************************************************/
void Task_DebugPrintf(debug_message_type_t messageType, char* stringPtr, ...)
{
	debug_messaage_data_t messageData;
	char* messageBuffer;
	char* taskName;
	uint32_t length = 0;
	va_list args;

	/* Allocate the message buffer */
	messageBuffer = pvPortMalloc(DEBUG_MESSAGE_MAX_LEN);

	if(messageBuffer)
	{
		va_start(args, stringPtr);
		if(messageType != none)
		{
			taskName = pcTaskGetName(xTaskGetCurrentTaskHandle());
			length = snprintf(messageBuffer, DEBUG_MESSAGE_MAX_LEN, "%-16s : ",
				taskName);
		}

		vsnprintf((messageBuffer + length), (DEBUG_MESSAGE_MAX_LEN - length),
				stringPtr, args);

		va_end(args);

		messageData.messageType = messageType;
		messageData.stringPtr = messageBuffer;

		/* The receiver task is responsible to free the memory from here on */
		if(pdPASS != xQueueSendToBack(debugMessageQ, &messageData, 0u))
		{
			/* Failed to send the message into the queue */
			vPortFree(messageBuffer);
		}
	}
	else
	{
		/* pvPortMalloc failed. Handle error */
	}
}

/*******************************************************************************
* Function Name: void Task_DebugInit(void)
********************************************************************************
*
* Description: Initializes the underlying Task and Queue used for printing debug
*  messages.
*
* Parameters: None
*
* Return: None
*
*******************************************************************************/
void Task_DebugInit(void)
{
	debugMessageQ = xQueueCreate(DEBUG_QUEUE_SIZE, sizeof(debug_messaage_data_t));

	xTaskCreate(Task_Debug, "Debug Task", configMINIMAL_STACK_SIZE,
			NULL, (tskIDLE_PRIORITY + 1u), NULL);
};

/*******************************************************************************
* Function Name: void UART_FastStart(void)
********************************************************************************
*
* Description: This function initializes the SCB in UART mode for debug message
*  printing.
*
* Parameters: None
*
* Return: None
*
*******************************************************************************/
void UART_FastStart(void)
{
	Cy_SCB_UART_Init(KIT_UART_HW, &KIT_UART_config, &KIT_UART_context);
	Cy_SCB_UART_Enable(KIT_UART_HW);
}

#endif
