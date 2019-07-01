/******************************************************************************
* File Name: main.c
*
* Version 1.0
*
* Description: This code example demonstrates the implementation of a BLE Battery
*  Service using PSoC 6 MCU with Bluetooth Low Energy (BLE) Connectivity.
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

/* Include header files */
#include "cy_device_headers.h"
#include "cycfg.h"
#include "cycfg_ble.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ble_task.h"
#include "battery_task.h"
#include "status_led_task.h"
#include "uart_debug.h"

/** Priority of user tasks in this application. Valid range of task priory is
 * priority is 0 to ( configMAX_PRIORITIES - 1 ), where configMAX_PRIORITIES is
 * defined within FreeRTOSConfig.h. Higher number means higher priority task.
 **/
#define TASK_BLE_PRIORITY			(configMAX_PRIORITIES - 1)
#define TASK_BATTERY_PRIORITY		(configMAX_PRIORITIES - 2)
#define TASK_STATUS_LED_PRIORITY	(configMAX_PRIORITIES - 3)


/* Task names used in this application */
#define TASK_BLE_NAME			("BLE Task")
#define TASK_STATUS_LED_NAME	("Status LED Task")
#define TASK_BATTERY_NAME		("Battery Task")

/* Stack sized used for task */
#define TASK_BLE_STACK_SIZE			(1024u)
#define TASK_STATUS_LED_STACK_SIZE	(configMINIMAL_STACK_SIZE)
#define TASK_BATTERY_STACK_SIZE		(configMINIMAL_STACK_SIZE)
#define TASK_LED_STACK_SIZE			(configMINIMAL_STACK_SIZE)

/* Queue lengths of message queues used in this application  */
#define BLE_QUEUE_LENGTH		(10u)
#define STATUS_LED_QUEUE_LENGTH	(1u)


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function sets up user tasks and then starts
*  the RTOS scheduler.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    /* Set up the device based on configurator selections */
    init_cycfg_all();

    __enable_irq();

    /* Enable UART for debug message printing */
    UART_FastStart();
    /* Create a task to do thread-safe debug message printing */
	Task_DebugInit();

	/* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
	DebugPrintf(CLEAR_SCREEN);
	DebugPrintf("PSoC 6 MCU with BLE connectivity - Battery Level\r\n");

	/* Create the queues. See the respective data-types for details of queue
	 * contents */
	bleCommandDataQ = xQueueCreate(BLE_QUEUE_LENGTH, \
			sizeof(ble_command_data_t));
	statusLedDataQ = xQueueCreate(STATUS_LED_QUEUE_LENGTH, \
				sizeof(status_led_data_t));

	/* Create the user tasks. See the respective Task definition for more
	 * details of these tasks */
	xTaskCreate(Task_Ble, TASK_BLE_NAME, TASK_BLE_STACK_SIZE, NULL,\
			TASK_BLE_PRIORITY, NULL);
	xTaskCreate(Task_StatusLed, TASK_STATUS_LED_NAME, \
			TASK_STATUS_LED_STACK_SIZE, NULL, TASK_STATUS_LED_PRIORITY, NULL);
	xTaskCreate(Task_Battery, TASK_BATTERY_NAME, TASK_BATTERY_STACK_SIZE, \
			NULL, TASK_BATTERY_PRIORITY, NULL);

	/* Start the RTOS scheduler. This function should never return */
	vTaskStartScheduler();
	/* Program should never reach here! */
	DebugPrintf("[Error]   : FreeRTOS scheduler failed to start \r\n");
	CY_ASSERT(0u);

    for(;;)
    {
    }
}

/*******************************************************************************
* Function Name: void vApplicationStackOverflowHook(TaskHandle_t *pxTask,
*                                                   signed char *pcTaskName)
********************************************************************************
*
* Summary:
*  This function is called when a stack overflow has been detected by the RTOS
*
* Parameters:
*  TaskHandle_t  : Handle to the task
*  signed char   : Name of the task
*
* Return:
*  None
*
*******************************************************************************/
void vApplicationStackOverflowHook(TaskHandle_t *pxTask,
                                   signed char *pcTaskName)
{
    /* Remove warning for unused parameters */
    (void)pxTask;

    /* Print the error message with task name if debug is enabled in
     * uart_debug.h file */
    DebugPrintf("[Error]   : RTOS - stack overflow in %s \r\n", pcTaskName);

    /* Halt the CPU */
    CY_ASSERT(0);
}

/*******************************************************************************
* Function Name: void vApplicationMallocFailedHook(void)
********************************************************************************
*
* Summary:
*  This function is called when a memory allocation operation by the RTOS
*  has failed
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void vApplicationMallocFailedHook(void)
{
    /* Print the error message if debug is enabled in uart_debug.h file */
	DebugPrintf("[Error]   : RTOS - Memory allocation failed \r\n");

    /* Halt the CPU */
    CY_ASSERT(0);
}

/* [] END OF FILE */
