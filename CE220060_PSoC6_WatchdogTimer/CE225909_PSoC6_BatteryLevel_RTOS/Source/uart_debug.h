/******************************************************************************
* File Name: uart_debug.h
*
* Version 1.0
*
* Description: This file contains the macros that are used for UART based
*              debug.
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

/* Include guard */
#ifndef UART_DEBUG_H_
#define UART_DEBUG_H_

/* Header file includes */
#include "stdio.h"

/** (true) enables UART based debug and (false) disables it.
 * Note that enabling debug reduces performance, power efficiency and
 * increases code size. */
#define DEBUG_ENABLE	(false)

/* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
#define CLEAR_SCREEN ("\x1b[2J\x1b[;H")

/* Debug message type */
typedef enum
{
	none = 0,
	info = 1,
	warning = 2,
	error = 3
} debug_message_type_t;

#if DEBUG_ENABLE

#define Task_Print(...)			Task_DebugPrintf(none, __VA_ARGS__)
#define Task_PrintInfo(...)		Task_DebugPrintf(info, __VA_ARGS__)
#define Task_PrintWarning(...) 	Task_DebugPrintf(warning, __VA_ARGS__)
#define Task_PrintError(...)	Task_DebugPrintf(error, __VA_ARGS__)

/* Function prototypes */
void UART_FastStart(void);
void Task_DebugPrintf(debug_message_type_t messageType, char* stringPtr, ...);
void Task_DebugInit(void);

/* DebugPrintf is not thread-safe, and should not be used inside task */
#define DebugPrintf(...)		printf(__VA_ARGS__)

/** Declaration of empty or default value macros if the debug is not enabled
 * for efficient code generation. */
#else

#define UART_FastStart()
#define Task_DebugInit(void)
#define Task_DebugPrintf(...)
#define Task_Print(...)
#define Task_PrintInfo(...)
#define Task_PrintWarning(...)
#define Task_PrintError(...)
#define DebugPrintf(...)

#endif /* DEBUG_ENABLE */

#endif /* UART_DEBUG_H_ */
