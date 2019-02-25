/*******************************************************************************
* File Name: debug.c
*
* Version: 1.00
*
* Description:
*  This file contains functions for debug functionality.
*
*
********************************************************************************
* Copyright 2018-2019, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "debug.h"

/*******************************************************************************
* Function Name: ShowError()
********************************************************************************
*
* Summary:
*   Shows error condition: Turn ON all leds - white colour will indicate error.
*
*******************************************************************************/
void ShowError(void)
{
    Cy_GPIO_Write(Advertising_LED_PORT, Advertising_LED_PIN, LED_ON);
    Cy_GPIO_Write(Alert_LED_PORT, Alert_LED_PIN, LED_ON);
    Cy_GPIO_Write(Disconnect_LED_PORT, Disconnect_LED_PIN, LED_ON);

    /* Halt CPU */
    CY_ASSERT(0u != 0u);
}


/* [] END OF FILE */
