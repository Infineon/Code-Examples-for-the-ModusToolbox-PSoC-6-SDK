/******************************************************************************
* File Name: main.c
*
* Version 1.0
*
* Description: This code example demonstrates how to generate a daily alarm
*              using RTC alarm interrupt.
*
* Related Document: CE218964_PSoC6_RTC_DailyAlarm.pdf
*
* Hardware Dependency: See CE218964_PSoC6_RTC_DailyAlarm.pdf
*
*******************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation. All rights reserved.
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
/* Header files */
#include "cy_device_headers.h"
#include "cycfg.h"
#include "cy_sysint.h"
#include "stdio_user.h"
#include <stdio.h>
#include "rtc_user.h"

/* Interrupt priorities */
#define RTC_INTERRUPT_PRIORITY		(7UL)
#define SWITCH_INTERRUPT_PRIORITY 	(7UL)

/* UART context structure */
cy_stc_scb_uart_context_t KIT_UART_context;

/* Variables used for storing interrupt status */
bool rtcAlarmInterrupt = false;
bool switchInterrupt = false;

/* RTC interrupt configuration */
const cy_stc_sysint_t rtc_intr_config =
{
	.intrSrc = srss_interrupt_backup_IRQn,
	.intrPriority  = RTC_INTERRUPT_PRIORITY
};

/* Switch interrupt configuration */
const cy_stc_sysint_t switch_intr_config =
{
	.intrSrc = ioss_interrupts_gpio_0_IRQn,
	.intrPriority = SWITCH_INTERRUPT_PRIORITY
};

/* Function prototype */
void RtcIsr(void);
void SwitchIsr(void);

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary: This is the system entrance point for Cortex-M4. This function
* - Set up the device based on configurator selections
* - Initialize the RTC and setup the alarm
* - Handle RTC and switch interrupts
* - Display UART messages in terminal application
*
* Parameters:
*  None
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
	/* Variable used for storing date and time information */
	cy_stc_rtc_config_t dateTime;

	/* Variable used to store return status of RTC API */
	cy_en_rtc_status_t rtcApiStatus;

	/* Variable used to store previous second value */
	uint32_t secPrev = CY_RTC_MAX_SEC_OR_MIN + 1;

    /* Set up the device based on configurator selections */
    init_cycfg_all();

    /* Enable UART component */
	Cy_SCB_UART_Init(KIT_UART_HW, &KIT_UART_config, &KIT_UART_context);
    Cy_SCB_UART_Enable(KIT_UART_HW);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
	printf("\x1b[2J\x1b[;H");
	/* Display starting message*/
	printf("CE218964 - PSoC 6 MCU RTC Daily Alarm\r\n");

    /* Initialize RTC */
	rtcApiStatus = Rtc_Init();
	if (CY_RTC_SUCCESS != rtcApiStatus)
	{
		printf("RTC initialization failed \r\n");
		CY_ASSERT(0);
	}

	/* Enable RTC interrupt */
    Cy_RTC_SetInterruptMask(CY_RTC_INTR_ALARM1);

    /* Configure RTC interrupt ISR */
	Cy_SysInt_Init(&rtc_intr_config, RtcIsr);
	/* Clear pending interrupt */
	NVIC_ClearPendingIRQ(rtc_intr_config.intrSrc);
	/* Enable RTC interrupt */
	NVIC_EnableIRQ(rtc_intr_config.intrSrc);

	/* Configure switch interrupt ISR */
	Cy_SysInt_Init(&switch_intr_config, SwitchIsr);
	/* Clear pending interrupt */
	NVIC_ClearPendingIRQ(switch_intr_config.intrSrc);
	/* Enable switch interrupt */
	NVIC_EnableIRQ(switch_intr_config.intrSrc);

	/* Enable interrupt */
	__enable_irq();

	/* Set initial alarm */
	rtcApiStatus = Rtc_SetNextAlarm();
	if(CY_RTC_SUCCESS != rtcApiStatus)
	{
		printf("Failed to set Alarm\r\n");
	}

    for(;;)
    {
    	if(rtcAlarmInterrupt == true) /* Process RTC interrupt */
    	{
    		if(switchInterrupt == true) /* Process switch interrupt */
    		{
    			/* Reset RTC interrupt flag */
    			rtcAlarmInterrupt = false;
    			/* Reset switch interrupt flag */
    			switchInterrupt = false;
    			/* Set next alarm */
    			rtcApiStatus = Rtc_SetNextAlarm();
				if(CY_RTC_SUCCESS != rtcApiStatus)
				{
					printf("Failed to set Alarm\r\n");
				}
    		}
    	}

    	/* Get current date and time */
    	Cy_RTC_GetDateAndTime(&dateTime);

    	if(dateTime.sec != secPrev)
    	{
    		secPrev = dateTime.sec;
    		printf("\r%02lu/%02lu/%02lu %s %02lu:%02lu:%02lu\r\n",	\
    				dateTime.date, dateTime.month, dateTime.year, 	\
					day_str[dateTime.dayOfWeek - 1],				\
					dateTime.hour, dateTime.min, dateTime.sec);

    		/* Display alarm expire message until user switch is pressed */
    		if((rtcAlarmInterrupt == true ) && (switchInterrupt == false))
    		{
    			/* Toggle LED */
    			Cy_GPIO_Inv(KIT_LED2_PORT, KIT_LED2_NUM);
    			printf("Alarm Expired !! Press switch (SW2) to set up alarm for the next day\r\n");
    		}
    	}
    }
}

/*******************************************************************************
* Function Name: void RtcIsr(void)
********************************************************************************
* Summary: Interrupt service routine for RTC interrupt
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void RtcIsr(void)
{
	Cy_RTC_Interrupt(NULL, false);
}

/*******************************************************************************
* Function Name: void SwitchIsr(void)
********************************************************************************
* Summary: Interrupt service routine for switch interrupt. It sets
* switchInterrupt flag to notify the main processing routine.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void SwitchIsr(void)
{
	/* Clear interrupt source */
	Cy_GPIO_ClearInterrupt(KIT_BTN2_PORT, KIT_BTN2_NUM);
	/* Set the interrupt flag */
	switchInterrupt = true;
}

/*******************************************************************************
* Function Name: void Cy_RTC_Alarm1Interrupt(void)
********************************************************************************
*  This function is called when the CY_RTC_INTR_ALARM1 is expired.
*  It sets rtcAlarmInterrupt variable to notify the main processing routine.
*  This function overrides the weakly linked Cy_RTC_Alarm1Interrupt()
*  in cy_rtc.c
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Cy_RTC_Alarm1Interrupt(void)
{
	rtcAlarmInterrupt = true;
}

/* [] END OF FILE */
