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
*  1. ModusToolbox 1.0
*
********************************************************************************
* Copyright 2018, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "cy_device_headers.h"
#include "cycfg.h"
#include "cycfg_ble.h"
#include "cy_syslib.h"
#include "cy_sysint.h"

/* BLESS interrupt configuration structure */
const cy_stc_sysint_t  blessIsrCfg =
{
    /* the BLESS interrupt */
    .cm0pSrc       = bless_interrupt_IRQn,

    /* CM0+ NVIC Mux input. Must be Deep Sleep capable, the valid range: 0..7 */
    .intrSrc       = (IRQn_Type) 7u,

    /* The interrupt priority number */
    .intrPriority  = 1u
};

/*******************************************************************************
* Function Name: BlessInterrupt
*******************************************************************************/
void BlessInterrupt(void)
{
    Cy_BLE_BlessIsrHandler();
}

int main(void)
{
    cy_en_ble_api_result_t          apiResult;

	/* Set up the device based on configurator selections */
	init_cycfg_all();

    /* enable interrupts, and the CM4 */
    __enable_irq();

    /* Unfreeze IO if device is waking up from hibernate */
    if(Cy_SysPm_GetIoFreezeStatus())
    {
        Cy_SysPm_IoUnfreeze();
    }

    /* Initialize the BLESS interrupt */
    cy_ble_config.hw->blessIsrConfig = &blessIsrCfg;
    Cy_SysInt_Init(cy_ble_config.hw->blessIsrConfig, BlessInterrupt);

    /* Initialize the Controller portion of BLE. Host runs on the CM4 */
    apiResult = Cy_BLE_Init(&cy_ble_config);

    /* Enable the Controller portion of the BLE. */
    if(apiResult == CY_BLE_SUCCESS)
    {
    	apiResult = Cy_BLE_Enable();
    }

    if(apiResult == CY_BLE_SUCCESS)
    {
        /* Enable CM4 only if BLE Controller started successfully.
        *  CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout
        *  is changed. */
        Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR);
    }
    else
    {
        /* Halt CPU */
        CY_ASSERT(0u != 0u);
    }

    /* Enable BLE Low Power Mode (LPM) */
    Cy_BLE_EnableLowPowerMode();


    for (;;)
    {
        /* Cy_Ble_ProcessEvents() allows the BLE stack to process pending events */
        /* The BLE Controller automatically wakes up the host if required */
        Cy_BLE_ProcessEvents();

        /* Put CM0p to Deep Sleep to achieve low power in the device */
        Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
    }
}
