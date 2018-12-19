/***************************************************************************//**
* \file main.c
* \version 1.0
*
* \brief
* Objective:
*  This is a CM0+ main() template. It starts the Cortex-M4 and enters deep-sleep.
*
********************************************************************************
* \copyright
* Copyright 2017-2018, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "cy_device_headers.h"
#include "cy_syslib.h"
#include "cy_syspm.h"

#include "cycfg.h"  // add this, needed for pin definitions

int main(void)
{
    init_cycfg_all(); // add this, needed for correct timing

    /* enable interrupts, and the CM4 */
    __enable_irq();
    Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR);

    for (;;)
    {
        Cy_GPIO_Inv(KIT_RGB_B_PORT, KIT_RGB_B_PIN); /* toggle the pin */
        Cy_SysLib_Delay(1023/*msec*/);
    }
}
