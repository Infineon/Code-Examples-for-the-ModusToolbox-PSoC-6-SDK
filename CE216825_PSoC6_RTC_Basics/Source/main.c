/******************************************************************************
* File Name: main.c
*
* Version 1.0
*
* Description: Demonstrates how to read and write the current time using a RTC
*              component. The UART interface is used to input a command and
*              print the result on the terminal.
*
* Related Document: CE216825_PSoC6_RTC_Basics.pdf
*
* Hardware Dependency: See CE216825_PSoC6_RTC_Basics.pdf
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
* *******************************************************************************/

/* Header files */
#include "cy_pdl.h"
#include "cycfg.h"
#include "stdio_user.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/* Macros used in this program */
#define UART_NO_DATA 		(char8)CY_SCB_UART_RX_NO_DATA
#define RTC_CMD_GET_DATE_TIME   ('1')
#define RTC_CMD_SET_DATE_TIME   ('2')
#define MAX_LENGTH              (4u)
#define VALID_DAY_LENGTH        (2u)
#define VALID_MONTH_LENGTH      (2u)
#define VALID_SHORT_YEAR_LENGTH (2u)
#define VALID_LONG_YEAR_LENGTH  (4u)

#define RTC_ACCESS_RETRY    (50u)
#define RTC_RETRY_DELAY     (5u)     /* 5 millisecond */

/* Variable used for storing UART context */
static cy_stc_scb_uart_context_t KIT_UART_context;

/* See the respective function definitions for more details */
static inline bool Rtc_IsLeapYear(uint32_t year);
static void Rtc_PrintDateTime(void);
static void Rtc_SetTime(void);
static bool ValidateDateTime(uint32_t sec, uint32_t min, uint32_t hour, \
		uint32_t date, uint32_t month, uint32_t year);
static void PrintAvailableCommand(void);

/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary: This is the system entrance point for Cortex-M4. This function
*  initializes the UART and RTC component and process the input commands.
*
* Parameters:
*  None
*
* Return:
*  int
*
* Side Effects:
*  None
*
*******************************************************************************/
int main(void)
{
	/* Variable used for storing command */
	char8 command;

    /* Set up internal routing, pins, and clock-to-peripheral connections */
	init_cycfg_all();

	/* Enable global interrupts. */
    __enable_irq();
 
    /* Initialize and enable UART */
	Cy_SCB_UART_Init(KIT_UART_HW, &KIT_UART_config, &KIT_UART_context);
	Cy_SCB_UART_Enable(KIT_UART_HW);

	/* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
	printf("\x1b[2J\x1b[;H");
	/* Display code example title */
	printf("PSoC 6 MCU: RTC Basics\r\n");

	/* Display available options in terminal application */
	PrintAvailableCommand();

	for(;;)
	{
		command = Cy_SCB_UART_Get(KIT_UART_HW);
		if(command != UART_NO_DATA)
		{
			switch(command)
			{
				case'1':
				{
					printf("\rGet Time\n");
					Rtc_PrintDateTime();
					break;
				}
				case '2':
				{
					printf("\rSetTime\n");
					Rtc_SetTime();
					break;
				}
				default:
				{
					printf("\rInvalid Command\r\n");
					break;
				}
			}
		}
	}
}

/*******************************************************************************
* Function Name: static void Rtc_PrintDateTime(void)
********************************************************************************
* Summary:
*  This function prints current date and time to UART
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void Rtc_PrintDateTime(void)
{
	cy_stc_rtc_config_t dateTime;

	/* Get the current RTC date and time */
	Cy_RTC_GetDateAndTime(&dateTime);
	/* Display current date and time */
	printf("\rCurrent Time: %02lu : %02lu : %02lu  %02lu/%02lu/%02lu\r\n", \
			dateTime.hour, dateTime.min, dateTime.sec, \
			dateTime.date, dateTime.month, dateTime.year);
}

/*******************************************************************************
* Function Name: static void Rtc_SetTime(void)
********************************************************************************
* Summary:
*  This function sets new date and time.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void Rtc_SetTime(void)
{
	/* Variables used to store user input */
	char dateStr[MAX_LENGTH], monthStr[MAX_LENGTH], yearStr[MAX_LENGTH];
	char secStr[MAX_LENGTH], minStr[MAX_LENGTH], hourStr[MAX_LENGTH];

	/* Variables used to store date and time information */
	uint32_t date, month, year, sec, min, hour;

	/* Variable used to store return status of RTC API */
	cy_en_rtc_status_t rtcApiStatus;

	uint32_t rtcAccessRetry = RTC_ACCESS_RETRY;

	printf("\r\nEnter new date (DD MM YY)\r\n");
	scanf("%s %s %s", dateStr, monthStr, yearStr);

	/* validate user input */
	if(strlen(dateStr)<= VALID_DAY_LENGTH && strlen(monthStr)<= VALID_MONTH_LENGTH  && \
	(strlen(yearStr)<= VALID_SHORT_YEAR_LENGTH || strlen(yearStr)== VALID_LONG_YEAR_LENGTH ))
	{
		printf("\rEnter new time in 24-hour format (HH MM SS)\r\n");
		scanf("%s %s %s", hourStr, minStr, secStr);

		/* Convert string input to decimal */
		date    = atoi(dateStr);
		month   = atoi(monthStr);
		year    = atoi(yearStr);
		sec     = atoi(secStr);
		min     = atoi(minStr);
		hour    = atoi(hourStr);

		/* If user input 4 digits Year information, set 2 digits Year */
		if(year > CY_RTC_MAX_YEAR)
		{
			year = year % 100u;
		}

		if(ValidateDateTime(sec, min, hour, date, month, year))
		{
			/* Set date and time.
			 * RTC block doesn't allow to access, when synchronizing the user registers
			 * and the internal actual RTC registers. It will return RTC_BUSY value, if
			 * it is not available to update the configuration values. Needs to retry,
			 * if it doesn't return CY_RTC_SUCCESS.
			 */
			do
			{
				rtcApiStatus = Cy_RTC_SetDateAndTimeDirect( \
						sec, min, hour, date, month, year);
				rtcAccessRetry --;
			} while ((rtcApiStatus != CY_RTC_SUCCESS) && (rtcAccessRetry != 0));

			if(rtcApiStatus != CY_RTC_SUCCESS)
			{
				printf("\r\nFailed to update date and time\r\n");
				PrintAvailableCommand();
			}
			else
			{
				printf("\r\nDate and Time updated !!\r\n");
				Rtc_PrintDateTime();
			}
		}
		else
		{
			printf("\r\nInvalid values! Please enter the values in specified format\r\n");
			PrintAvailableCommand();
		}
	}
	else
	{
		printf("\r\nInvalid values! Please enter the values in specified format\r\n");
		PrintAvailableCommand();
	}
}

/*******************************************************************************
* Function Name: static inline bool IsLeapYear(uint32_t year)
********************************************************************************
* Summary:
*  This function checks whether the year passed through the parameter is leap or
*  not.  Leap year is identified as a year that is a multiple of 4 or 400 but not
*  100.
*
* Parameters:
*  uint32_t year: The year to be checked
*
* Return:
*  bool: false - The year is not leap; true - The year is leap.
*
*******************************************************************************/
static inline bool Rtc_IsLeapYear(uint32_t year)
{
	return(((0U == (year % 4UL)) && (0U != (year % 100UL))) || (0U == (year % 400UL)));
}

/*******************************************************************************
* Function Name: static bool ValidateDateTime( \
* 							uint32_t sec, uint32_t min, uint32_t hour, \
*	                     	uint32_t date, uint32_t month, uint32_t year)
********************************************************************************
* Summary:
*  This function validates date and time value.

*
* Parameters:
*  uint32_t sec:  The second valid range is [0-59].
*  uint32_t min:  The minute valid range is [0-59].
*  uint32_t hour: The hour valid range is [0-23].
*  uint32_t date: The date valid range is [1-31], if the month of February is
*        selected as the Month parameter, then the valid range is [0-29].
*  uint32_t month: The month valid range is [1-12].
*  uint32_t year: The year valid range is [0-99].
*
* Return:
*  false - invalid ; true - valid
*
*******************************************************************************/
static bool ValidateDateTime(uint32_t sec, uint32_t min, uint32_t hour, \
					  uint32_t date, uint32_t month, uint32_t year)
{
	uint8_t daysInMonth;
	/* Variable used to store days in months table */
	static uint8_t daysInMonthTable[CY_RTC_MONTHS_PER_YEAR] = {
		CY_RTC_DAYS_IN_JANUARY,
		CY_RTC_DAYS_IN_FEBRUARY,
		CY_RTC_DAYS_IN_MARCH,
		CY_RTC_DAYS_IN_APRIL,
		CY_RTC_DAYS_IN_MAY,
		CY_RTC_DAYS_IN_JUNE,
		CY_RTC_DAYS_IN_JULY,
		CY_RTC_DAYS_IN_AUGUST,
		CY_RTC_DAYS_IN_SEPTEMBER,
		CY_RTC_DAYS_IN_OCTOBER,
		CY_RTC_DAYS_IN_NOVEMBER,
		CY_RTC_DAYS_IN_DECEMBER};

	bool status = CY_RTC_IS_SEC_VALID(sec) & CY_RTC_IS_MIN_VALID(min) & \
			CY_RTC_IS_HOUR_VALID(hour) & CY_RTC_IS_MONTH_VALID(month) & \
			CY_RTC_IS_YEAR_SHORT_VALID(year);


	if(status)
	{
		daysInMonth = daysInMonthTable[month - 1];

		if(Rtc_IsLeapYear(year + CY_RTC_TWO_THOUSAND_YEARS) && \
			(month == CY_RTC_FEBRUARY))
		{
			daysInMonth++;
		}
		status &= (date > 0U) && (date <= daysInMonth);
	}
	return status;
}


/*******************************************************************************
* Function Name: static void PrintAvailableCommand(void)
********************************************************************************
* Summary:
*  This function displays available commands.
*
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void PrintAvailableCommand(void)
{
	printf("\r\nAvailable commands \r\n");
	printf("1: Get current time and date\r\n");
	printf("2: Set new time and date\r\n\n");
}

/* [] END OF FILE */
