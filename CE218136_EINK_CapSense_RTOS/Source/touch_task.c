/******************************************************************************
* File Name: touch_task.c
*
* Version: 1.00
*
* Description: This file contains the task that handles touch sensing and
*              gesture detection
*
* Related Document: CE218136_EINK_CapSense_RTOS.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*
******************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation.
******************************************************************************
* This software, including source code, documentation and related materials
* ("Software") is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress source code and derivative works for the sole purpose of creating
* custom software in support of licensee product, such licensee product to be
* used only in conjunction with Cypress's integrated circuit as specified in the
* applicable agreement. Any reproduction, modification, translation, compilation,
* or representation of this Software except as specified above is prohibited
* without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes to the Software without notice.
* Cypress does not assume any liability arising out of the application or use
* of Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use as critical components in any products
* where a malfunction or failure may reasonably be expected to result in
* significant injury or death ("ACTIVE Risk Product"). By including Cypress's
* product in a ACTIVE Risk Product, the manufacturer of such system or application
* assumes all risk of such use and in doing so indemnifies Cypress against all
* liability. Use of this Software may be limited by and subject to the applicable
* Cypress software license agreement.
*****************************************************************************/
/*******************************************************************************
* Task that handles touch sensing and gesture detection
*******************************************************************************/

/* Header file includes */
#include "touch_task.h"
#include "uart_debug.h"
#include "task.h" 
#include "cy_pdl.h"
#include "cy_capsense.h"
#include "cycfg.h"
#include "cycfg_capsense.h"

/* Scanning interval of 10ms is used to get 100 scans per second */
#define TOUCH_SCAN_INTERVAL  (pdMS_TO_TICKS(10u))
/* Gesture time increment at each scan interval */
#define GESTURE_TIME_INCREMENT  (10u)

/* Macros used for gesture detection. See the function header of
   Cy_CapSense_DecodeWidgetGestures for the details of each
   bit position */
#define LEFT_FLICK_MASK		(0x03004080u)
#define RIGHT_FLICK_MASK	(0x02004080u)

/* Queue handle used for touch data */
QueueHandle_t touchDataQ;

/* CapSense ISR */
static void CapSense_Interrupt(void);

/* CapSense ISR configuration parameters */
const cy_stc_sysint_t CapSense_ISR_cfg =
{
    .intrSrc = csd_interrupt_IRQn,
    .intrPriority = 7u,
};

/*******************************************************************************
* Function Name: void Task_Touch(void *pvParameters)
********************************************************************************
* Summary:
*  Task that reads touch data from CapSense button and slider widgets and
*  calculates gesture information
*
* Parameters:
*  void *pvParameters : Task parameter defined during task creation (unused)
*
* Return:
*  void
*
*******************************************************************************/
void Task_Touch(void *pvParameters)    
{
    /* Local variables used for touch and gesture detection */
    touch_data_t previousTouchData = NO_TOUCH;
    touch_data_t currentTouchData;
    uint32_t gestureTimestamp = 0;
    uint32_t gestureStatus;

    /* Remove warning for unused parameter */
    (void)pvParameters ;

	/*Initialize CapSense Data structures */
	Cy_CapSense_Init(&cy_capsense_context);

	/* Initialize CapSense interrupt */
	Cy_SysInt_Init(&CapSense_ISR_cfg, &CapSense_Interrupt);
	NVIC_ClearPendingIRQ(CapSense_ISR_cfg.intrSrc);
	NVIC_EnableIRQ(CapSense_ISR_cfg.intrSrc);

	/* Start CapSense block and perform first scan to set up sensor baselines */
	Cy_CapSense_Enable(&cy_capsense_context);

    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Check if CapSense is busy with a previous scan */
        if(CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
        {
            /* Process all widgets and read touch information */
            Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);
            gestureStatus = Cy_CapSense_DecodeWidgetGestures(
            						CY_CAPSENSE_LINEARSLIDER0_WDGT_ID,
									&cy_capsense_context);

            /* Button0 is active */
            if(Cy_CapSense_IsWidgetActive(CY_CAPSENSE_BUTTON0_WDGT_ID,
            							  &cy_capsense_context))
            {
                    currentTouchData = BUTTON0_TOUCHED;
            }
            /* Button1 is active */
            else if (Cy_CapSense_IsWidgetActive (CY_CAPSENSE_BUTTON1_WDGT_ID,
            									&cy_capsense_context))
            {
            	currentTouchData = BUTTON1_TOUCHED;
            }
            /* Slider left flick detected */
            else if (gestureStatus == LEFT_FLICK_MASK)
            {
            	currentTouchData = SLIDER_FLICK_LEFT;
            }
            /* Slider right flick detected */
        	else if (gestureStatus == RIGHT_FLICK_MASK)
        	{
        		currentTouchData = SLIDER_FLICK_RIGHT;
            }
            else
            {
            	currentTouchData = NO_TOUCH;
            }

            /* Start the next CapSense scan */
            Cy_CapSense_ScanAllWidgets(&cy_capsense_context);

            /* Check if there is any data to be sent */
            if(currentTouchData != previousTouchData)
            {
            	previousTouchData = currentTouchData;

            	if(currentTouchData != NO_TOUCH)
            	{
            		/* Send the processed touch data */
            		xQueueOverwrite(touchDataQ, &currentTouchData);
            	}
          }
        }

        /* Increment the gesture time stamp */
        gestureTimestamp += GESTURE_TIME_INCREMENT;
        Cy_CapSense_SetGestureTimestamp(gestureTimestamp, &cy_capsense_context);

        /* Block until next scan interval */
        vTaskDelay(TOUCH_SCAN_INTERVAL);
    }
}

/*******************************************************************************
* Function Name: static void CapSense_Interrupt(void)
********************************************************************************
*
* Summary:
*  CapSense interrupt service routine
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void CapSense_Interrupt(void)
{
    Cy_CapSense_InterruptHandler(CSD0, &cy_capsense_context);
}

/* [] END OF FILE */
