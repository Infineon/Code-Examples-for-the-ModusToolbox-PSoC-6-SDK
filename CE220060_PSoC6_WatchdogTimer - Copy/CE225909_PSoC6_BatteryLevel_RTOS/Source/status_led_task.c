/******************************************************************************
* File Name: status_led_task.c
*
* Version 1.0
*
* Description: This file contains the task that that controls the status LEDs.
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
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "status_led_task.h"
#include "stdio.h"
#include "uart_debug.h"

/* LED refresh interval of 500ms is used when continuously toggling a
 * status LED */
#define STATUS_LED_TOGGLE_INTERVAL (pdMS_TO_TICKS(500u))
/* LED refresh interval of 4s is used for blinking a status LED once */
#define STATUS_LED_BLINK_INTERVAL  (pdMS_TO_TICKS(4000u))
/* Idle interval is used for static LED states or when no LEDs are ON */
#define STATUS_LED_IDLE_INTERVAL   (portMAX_DELAY)

/* Queue handle used for commands to Task_StatusLed */
QueueHandle_t statusLedDataQ;

/* Timer handles used to control LED blink / toggle intervals */
TimerHandle_t timerHandle_led;

/* Functions that start and control the timers used for LED blink / toggle
 * intervals */
static void StatusLedTimerStart(void);
static void StatusLedTimerUpdate(TickType_t period);

/*******************************************************************************
* Function Name: void Task_StatusLed(void *pvParameters)
********************************************************************************
* Summary:
*  Task that controls the status LED.
*
* Parameters:
*  void *pvParameters : Task parameter defined during task creation (unused)
*
* Return:
*  None
*
*******************************************************************************/
void Task_StatusLed(void *pvParameters)
{
    /* Variable that stores the data received over queue */
    status_led_data_t statusLedData;

    /* Variables used to detect changes to the LED states */
    static status_led_command_t userLedState = LED_TURN_OFF;

    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;

    /* Remove warning for unused parameter */
    (void)pvParameters ;

    /* Start the timers that control LED blink / toggle intervals */
    StatusLedTimerStart();

    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Block until a command has been received over statusLedDataQ */
        rtosApiResult = xQueueReceive(statusLedDataQ, &statusLedData,
                        	portMAX_DELAY);
        /* Command has been received from statusLedDataQ */
        if(rtosApiResult == pdTRUE)
        {
            /* Take an action based on the command received for userLed1 LED */
            switch(statusLedData.userLed1)
            {
                /* No change to the LED state */
                case LED_NO_CHANGE:
                {
                    break;
                }
                /* Turn ON the LED */
                case LED_TURN_ON:
                {
                    Cy_GPIO_Clr(KIT_LED_PORT, KIT_LED_NUM);
                    /* Set the timer to idle mode */
                    StatusLedTimerUpdate(STATUS_LED_IDLE_INTERVAL);
                    userLedState = LED_TURN_ON;
                    break;
                }
                /* Turn OFF the LED */
                case LED_TURN_OFF:
                {
                    Cy_GPIO_Set(KIT_LED_PORT, KIT_LED_NUM);
                    /* Set the timer to idle mode */
                    StatusLedTimerUpdate(STATUS_LED_IDLE_INTERVAL);
                    userLedState = LED_TURN_OFF;
                    break;
                }
                /* Continuously toggle the LED */
                case LED_TOGGLE_EN:
                {
                    /* Update the timer period to toggle interval */
                	StatusLedTimerUpdate(STATUS_LED_TOGGLE_INTERVAL);
                    userLedState = LED_TOGGLE_EN;
                    break;
                }
                /* Blink the LED once */
                case LED_BLINK_ONCE:
                {
                    Cy_GPIO_Clr(KIT_LED_PORT, KIT_LED_NUM);
                    /* Update the timer period to blink interval */
                    StatusLedTimerUpdate(STATUS_LED_BLINK_INTERVAL);
                    userLedState = LED_BLINK_ONCE;
                    break;
                }
                /* Refresh the LED based on the current state */
                case LED_TIMER_EXPIRED:
                {
                    if(userLedState == LED_TOGGLE_EN)
                    {
                        /* Toggle the LED */
                        Cy_GPIO_Inv(KIT_LED_PORT, KIT_LED_NUM);
                    }
                    else if (userLedState == LED_BLINK_ONCE)
                    {
                        /* Turn off the LED */
                        Cy_GPIO_Set(KIT_LED_PORT, KIT_LED_NUM);
                        /* Set the timer to idle mode */
                        StatusLedTimerUpdate(STATUS_LED_IDLE_INTERVAL);
                        userLedState = LED_TURN_OFF;
                    }
                    else
                    {
                    	Task_PrintError("Refresh command for LED during invalid state");
                    }
                    break;
                }
                /* Invalid command received */
                default:
                {
                	Task_PrintError("Invalid  command for LED received. Error Code: %u",\
                			statusLedData.userLed1);
                    break;
                }
            }
        }
        /* Task has timed out and received no commands during an interval of
         * portMAXDELAY ticks */
        else
        {

        }
    }
}

/*******************************************************************************
* Function Name: static void StatusLedTimerCallback(TimerHandle_t xTimer)
********************************************************************************
* Summary:
*  This function is called when the LED Timer expires
*
* Parameters:
*  TimerHandle_t xTimer :  Current timer value (unused)
*
* Return:
*  None
*
*******************************************************************************/
static void StatusLedTimerCallback(TimerHandle_t xTimer)
{
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;

    /* Remove warning for unused parameter */
    (void)xTimer;

    /* Send command to refresh the LED state */
    status_led_data_t ledRefreshData =
    {
		.userLed1     = LED_TIMER_EXPIRED,
    };

    rtosApiResult = xQueueOverwrite(statusLedDataQ, &ledRefreshData);

    /* Check if the operation has been successful */
    if(rtosApiResult != pdTRUE)
    {
        Task_PrintError("Sending data to Status LED queue");
    }
}

/*******************************************************************************
* Function Name: static void StatusLedTimerStart(void)
********************************************************************************
* Summary:
*  This function starts the timer that provides timing to LED toggle / blink
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void StatusLedTimerStart(void)
{
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;

    /* Create an RTOS timer */
    timerHandle_led =  xTimerCreate ("Status LED Timer", \
    		STATUS_LED_IDLE_INTERVAL, pdTRUE,NULL, StatusLedTimerCallback);

    /* Make sure that timer handle is valid */
    if (timerHandle_led != NULL)
    {
        /* Start the timer */
        rtosApiResult = xTimerStart(timerHandle_led, 0u);

        /* Check if the operation has been successful */
        if(rtosApiResult  != pdPASS)
        {
        	Task_PrintError("LED Timer initialization");
        }
    }
    else
    {
    	Task_PrintError("LED Timer creation");
    }
}


/*******************************************************************************
* Function Name: static void StatusLedTimerUpdate(TickType_t period)
********************************************************************************
* Summary:
*  This function updates the timer period per the parameter
*
* Parameters:
*  TickType_t period : Period of the timer in ticks
*
* Return:
*  None
*
*******************************************************************************/
static void StatusLedTimerUpdate(TickType_t period)
{
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;

    /* Change the timer period */
    rtosApiResult = xTimerChangePeriod(timerHandle_led, period, 0u);

    /* Check if the operation has been successful */
    if(rtosApiResult != pdPASS)
    {
        Task_PrintError("LED Timer update");
    }
}

/* [] END OF FILE */

