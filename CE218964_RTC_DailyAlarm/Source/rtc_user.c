/******************************************************************************
* File Name: rtc_user.c
*
* Version 1.0
*
* Description:
*  This file contains RTC related functions.
*
* Related Document: CE218964_PSoC6_RTC_DailyAlarm.pdf
*
* Hardware Dependency: See CE218964_PSoC6_RTC_DailyAlarm.pdf
*
*******************************************************************************
* Copyright (2018-2019), Cypress Semiconductor Corporation. All rights reserved.
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
#include "cycfg.h"
#include "rtc_user.h"
#include "stdio_user.h"
#include <stdio.h>

#define RTC_ACCESS_RETRY    (500u)
#define RTC_RETRY_DELAY     (5u)     /* 5 msec   */

/* RTC initial alarm delay */
#define RTC_INITIAL_ALARM_DELAY	(3u) /* 3 sec */

/* Date and time on start */
#define RTC_INITIAL_TIME_SEC	(0UL)
#define RTC_INITIAL_TIME_MIN	(0UL)
#define RTC_INITIAL_TIME_HOUR	(0UL)
#define RTC_INITIAL_DATE_DOW	(CY_RTC_MONDAY)
#define RTC_INITIAL_DATE_DOM	(1UL)
#define RTC_INITIAL_DATE_MONTH 	(1UL)
#define RTC_INITIAL_DATE_YEAR	(18UL)
#define RTC_HOUR_FORMAT			(CY_RTC_24_HOURS)

/* Set Mon to Fri - The alarm will expire after RTC_INITIAL_ALARM_DELAY from
 * start time and then every Monday to Friday at initialized time
 */
#define ALARM_ENABLE_SUNDAY     (0u)
#define ALARM_ENABLE_MONDAY     (1u)
#define ALARM_ENABLE_TUESDAY    (1u)
#define ALARM_ENABLE_WEDNESDAY  (1u)
#define ALARM_ENABLE_THURSDAY   (1u)
#define ALARM_ENABLE_FRIDAY     (1u)
#define ALARM_ENABLE_SATURDAY   (0u)


/* Daily mask compares the day of week after the alarm expires to find the next
 * alarm
 *
 * | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
 * |------|------|------|------|------|------|------|------|
 * | Sat  | Fri  | Thu  | Wed  | Tue  | Mon  | Sun  | X    |
 */
uint8_t dailyAlarmMask = (((1u & ALARM_ENABLE_SUNDAY)   << CY_RTC_SUNDAY)   | \
                          ((1u & ALARM_ENABLE_MONDAY)   << CY_RTC_MONDAY)   | \
                          ((1u & ALARM_ENABLE_TUESDAY)  << CY_RTC_TUESDAY)  | \
                          ((1u & ALARM_ENABLE_WEDNESDAY)<< CY_RTC_WEDNESDAY)| \
                          ((1u & ALARM_ENABLE_THURSDAY) << CY_RTC_THURSDAY) | \
                          ((1u & ALARM_ENABLE_FRIDAY)   << CY_RTC_FRIDAY)   | \
                          ((1u & ALARM_ENABLE_SATURDAY) << CY_RTC_SATURDAY));


/* Variable used for storing days in the week */
char_t day_str[CY_RTC_DAYS_PER_WEEK][4] = {
		"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};

/*******************************************************************************
* Function Name: void Rtc_Init(void)
********************************************************************************
* Summary: This function initializes the RTC driver
*
* Parameters:
*  None
*
* Return:
*  cy_en_rtc_status_t :
*******************************************************************************/
cy_en_rtc_status_t Rtc_Init(void)
{
	/* Variable used to store return status of RTC API */
	cy_en_rtc_status_t rtcApiStatus;

	/* Initial date and time */
	cy_stc_rtc_config_t initialDateTime = {
		.sec = 	RTC_INITIAL_TIME_SEC,
		.min = 	RTC_INITIAL_TIME_MIN,
		.hour = RTC_INITIAL_TIME_HOUR,
		.hrFormat = RTC_HOUR_FORMAT,
		.date = RTC_INITIAL_DATE_DOM,
		.month = RTC_INITIAL_DATE_MONTH,
		.year = RTC_INITIAL_DATE_YEAR,
		.dayOfWeek = RTC_INITIAL_DATE_DOW
	};

	uint32_t rtcAccessRetry = RTC_ACCESS_RETRY;

	/* RTC block doesn't allow to access, when synchronizing the user registers
	 * and the internal actual RTC registers. It will return RTC_BUSY value, if
	 * it is not available to update the configuration values. Needs to retry,
	 * if it doesn't return CY_RTC_SUCCESS.
	 */
	do
	{
		rtcApiStatus = Cy_RTC_Init(&initialDateTime);
		rtcAccessRetry--;
		Cy_SysLib_Delay(RTC_RETRY_DELAY);
	}while((rtcApiStatus != CY_RTC_SUCCESS) && (rtcAccessRetry != 0));

	return rtcApiStatus;
}

/*******************************************************************************
* Function Name: static uint32_t GetNextAlarmDay(uint32_t alarmDay)
********************************************************************************
* Summary: This function initializes the RTC driver
*
* Parameters:
*  uint32_t alarmDay: current alarm day
*
* Return:
*  uint32_t: Next alarm day
*******************************************************************************/
static uint32_t GetNextAlarmDay(uint32_t alarmDay)
{
	uint32_t nextAlarmDay = 0;
	uint32_t i;

	/* Find next alarm day */
	for(i = 0; i < CY_RTC_DAYS_PER_WEEK; i++)
	{
		alarmDay = (alarmDay % CY_RTC_DAYS_PER_WEEK) + 1;
		if(dailyAlarmMask & (1 << alarmDay))
		{
			nextAlarmDay = alarmDay;
			break;
		}
	}
	return nextAlarmDay;
}

/*******************************************************************************
* Function Name: void Rtc_SetNextAlarm(void)
********************************************************************************
* Summary: This function sets the next RTC alarm
*
* Parameters:
*  None
*
* Return:
*  cy_en_rtc_status_t
*******************************************************************************/
cy_en_rtc_status_t Rtc_SetNextAlarm(void)
{
	/* Variable used to store return status of RTC API */
	cy_en_rtc_status_t rtcApiStatus;

	/* Alarm configuration structure */
	cy_stc_rtc_alarm_t alarmDateTime;

	/* Variable used for storing date and time */
	cy_stc_rtc_config_t dateTime;

	uint32_t rtcAccessRetry = RTC_ACCESS_RETRY;

	static bool firstAlarm = true;

	/* First RTC alarm is set to RTC_INITIAL_TIME_SEC + RTC_INITIAL_ALARM_DELAY */
	if(firstAlarm)
	{
		firstAlarm = false;

		/* Configure alarm */
		alarmDateTime.almEn = CY_RTC_ALARM_ENABLE;
		alarmDateTime.sec = RTC_INITIAL_TIME_SEC + RTC_INITIAL_ALARM_DELAY;
		alarmDateTime.secEn = CY_RTC_ALARM_ENABLE;
		alarmDateTime.min = RTC_INITIAL_TIME_MIN;
		alarmDateTime.minEn = CY_RTC_ALARM_ENABLE;
		alarmDateTime.hour = RTC_INITIAL_TIME_HOUR;
		alarmDateTime.hourEn = CY_RTC_ALARM_ENABLE;
		alarmDateTime.date = RTC_INITIAL_DATE_DOM;
		alarmDateTime.dateEn = CY_RTC_ALARM_ENABLE;
		alarmDateTime.month = RTC_INITIAL_DATE_MONTH;
		alarmDateTime.monthEn = CY_RTC_ALARM_DISABLE;
		alarmDateTime.dayOfWeek = RTC_INITIAL_DATE_DOW;
		alarmDateTime.dayOfWeekEn = CY_RTC_ALARM_ENABLE;
	}
	else
	{
		/* Get the current date and time */
		Cy_RTC_GetDateAndTime(&dateTime);

		/* Configure alarm */
		alarmDateTime.almEn = CY_RTC_ALARM_ENABLE;
		alarmDateTime.sec = RTC_INITIAL_TIME_SEC;
		alarmDateTime.secEn = CY_RTC_ALARM_ENABLE;
		alarmDateTime.min = RTC_INITIAL_TIME_MIN;
		alarmDateTime.minEn = CY_RTC_ALARM_ENABLE;
		alarmDateTime.hour = RTC_INITIAL_TIME_HOUR;
		alarmDateTime.hourEn = CY_RTC_ALARM_ENABLE;
		alarmDateTime.date = dateTime.date;
		alarmDateTime.dateEn = CY_RTC_ALARM_DISABLE;
		alarmDateTime.month = dateTime.month;
		alarmDateTime.monthEn = CY_RTC_ALARM_DISABLE;
		alarmDateTime.dayOfWeek = GetNextAlarmDay(dateTime.dayOfWeek);
		alarmDateTime.dayOfWeekEn = CY_RTC_ALARM_ENABLE;
	}

	/* Set alarm */
	do
	{
		rtcApiStatus = Cy_RTC_SetAlarmDateAndTime(&alarmDateTime, CY_RTC_ALARM_1);
		rtcAccessRetry--;
		Cy_SysLib_Delay(RTC_RETRY_DELAY);
	} while((rtcApiStatus != CY_RTC_SUCCESS) && (rtcAccessRetry != 0));

	if(rtcApiStatus == CY_RTC_SUCCESS)
	{
		printf("\r\nAlarm is set for: %s %02lu:%02lu:%02lu \r\n\n", \
				day_str[alarmDateTime.dayOfWeek -1],
				alarmDateTime.hour, alarmDateTime.min, alarmDateTime.sec);
	}

	return rtcApiStatus;
}

/* [] END OF FILE */
