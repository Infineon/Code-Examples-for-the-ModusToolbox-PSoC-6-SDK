/*****************************************************************************
* File Name: ipc_common.h
*
* Version: 1.10
*
* Description: Contains common IPC utility functions for use by both CPUs.
*
* Note: These functions may be executed simultaneously by both CPUs.
*
* Related Document: Code example CE216795
*
* Hardware Dependency: See code example CE216795
*
******************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation.
******************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a Cypress integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the
* materials described herein. Cypress does not assume any liability arising out
* of the application or use of any product or circuit described herein. Cypress
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*****************************************************************************/
#ifndef INCLUDED_CE216795_COMMMON_H
#define INCLUDED_CE216795_COMMMON_H

#include "cycfg.h"
#include "cy_syslib.h"
#include "cy_ipc_sema.h"
#include <stdint.h>
#include <stdbool.h>

/* Select which IPC channel is used for this application.
   The channel is used only for initialization, where the address of the 
   shared variable is passed from one CPU to the other. */
#define MY_IPC_CHANNEL 8u

/* Select which IPC semaphore is used for this application. 
   The semaphore is used in both CPUs' do forever loops, for mutex-based access
   to the shared variable.*/
#define MY_SEMANUM 0u

/* function prototypes */
uint32_t ReadSharedVar(const uint8_t *sharedVar, uint8_t *copy);
uint32_t WriteSharedVar(uint8_t *sharedVar, uint8_t value);

#endif /* INCLUDED_CE216795_COMMON_H */

/* [] END OF FILE */
