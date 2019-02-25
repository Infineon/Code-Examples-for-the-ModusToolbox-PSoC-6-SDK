/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.00
*
* Description: This code example shows how to create a user-interface
*              solution using an EINK display and CapSense.
*
* Related Document: CE218136_EINK_CapSense_RTOS.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*                      CY8CKIT-028-EPD E-INK Display Shield
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
/******************************************************************************
* This code example demonstrates how to create a user-interface solution
* using EINK display and CapSense sliders and buttons with gesture detection.
* EINK displays consume no power for image retention. Together with PSoC 6 BLE’s
* CapSense touch sensing, EINK display can create user interfaces that have
* “always-on” functionality.
*
* This code example uses FreeRTOS. For documentation and API references of
* FreeRTOS, visit : https://www.freertos.org
*
* This code example uses emWin middleware for graphics. For documentation and API
* references of emWin, visit:
* https://www.segger.com/products/user-interface/emwin/
*******************************************************************************/

/* Header file includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cycfg.h"
#include "touch_task.h"
#include "display_task.h"
#include "uart_debug.h"

/* Priorities of user tasks in this project */
#define TASK_TOUCH_PRIORITY         (10u)
#define TASK_DISPLAY_PRIORITY       (5u)

/* Stack sizes of user tasks in this project */
#define TASK_DISPLAY_STACK_SIZE     (1536u)
#define TASK_TOUCH_STACK_SIZE       (configMINIMAL_STACK_SIZE)

/* Queue lengths of message queues used in this project */
#define TOUCH_ELEMENT_QUEUE_LEN     (1u)

/* API to initialize system components */
void InitializeSystem(void);

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
int main()
{
    /* Initialize the system components */
    InitializeSystem();

    /*~~~~~~~~~~~~~~~~~ Create FreeRTOS queues and tasks ~~~~~~~~~~~~~~~~~~~~~*/

	/* Create the queues. See the respective data-types for details of queue
       contents */
    touchDataQ = xQueueCreate(TOUCH_ELEMENT_QUEUE_LEN, sizeof(touch_data_t));

    /* Create the user tasks. See the respective Task definition for more
       details of these tasks */
    xTaskCreate(Task_Touch, "Touch Task", TASK_TOUCH_STACK_SIZE,
                NULL, TASK_TOUCH_PRIORITY, NULL);
    xTaskCreate(Task_Display, "Display task", TASK_DISPLAY_STACK_SIZE,
                NULL, TASK_DISPLAY_PRIORITY, NULL);

    /* Initialize thread-safe debug message printing. See uart_debug.h header file
       to enable / disable this feature */
    Task_DebugInit();

    /* Start the RTOS scheduler. This function should never return */
    vTaskStartScheduler();

    /*~~~~~~~~~~~~~~~~~~~~~ Should never get here! ~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /* RTOS scheduler exited */
    DebugPrintf("Error!   : RTOS - scheduler crashed \r\n");

    /* Halt the CPU if scheduler exits */
    CY_ASSERT(0);

    for(;;)
    {
    }
}

/*******************************************************************************
* Function Name: void vApplicationIdleHook(void)
********************************************************************************
*
* Summary:
*  This function is called when the RTOS in idle mode
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void vApplicationIdleHook(void)
{
    /* Enter sleep-mode */
    Cy_SysPm_Sleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
}

/*******************************************************************************
* Function Name: void vApplicationStackOverflowHook(TaskHandle_t *pxTask,
                                                    signed char *pcTaskName)
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
    (void)pcTaskName;

    /* Print the error message with task name if debug is enabled in
       uart_debug.h file */
    DebugPrintf("Error!   : RTOS - stack overflow in %s \r\n", pcTaskName);

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
    DebugPrintf("Error!   : RTOS - Memory allocation failed \r\n");

    /* Halt the CPU */
    CY_ASSERT(0);
}

/*******************************************************************************
* Function Name: void InitializeSystem(void)
********************************************************************************
*
* Summary:
*  This function initializes system components
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void InitializeSystem(void)
{
	/* Enable global interrupts. */
	__enable_irq();

	/* Configure the system components */
    init_cycfg_all();

    /* Initialize the hardware used to send debug messages, if debug is enabled
       in uart_debug.h header file */
    DebugPrintfInit();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen */
    DebugPrintf("\x1b[2J\x1b[;H");
}

/* [] END OF FILE */
