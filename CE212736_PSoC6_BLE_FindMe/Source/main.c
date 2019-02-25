/*******************************************************************************
* File Name: main.c
*
* Version: 1.00
*
* Description:
*   This is source code for the PSoC 6 MCU with BLE Find Me code example.
*
* Note:
*
* Owners:
*   snvn@cypress.com
*
* Related Documents:
*   AN210781 - Getting Started with PSoC 6 MCU with
*              Bluetooth Low Energy BLE) Connectivity
*   CE212736 - PSoC 6 MCU with Bluetooth Low Energy
*              (BLE) Connectivity - Find Me Using ModusToolbox
*
* Hardware Dependency:
*  1. PSoC 6 MCU with BLE device
*  2. CY8CKIT-062-BLE Pioneer Kit
*
* Code Tested With:
*  1. ModusToolbox 1.1
*
********************************************************************************
* Copyright 2018-2019, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "BLEFindMe.h"

int main(void)
{
    /* Initialize BLE */
    BleFindMe_Init();

    __enable_irq();
 
    for(;;)
    {
        BleFindMe_Process();
    }
}


