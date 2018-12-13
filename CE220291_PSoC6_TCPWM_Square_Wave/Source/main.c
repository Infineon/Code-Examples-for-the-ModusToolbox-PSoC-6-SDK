/*******************************************************************************
* \file main.c
* \version 1.0
*
* \brief
*    This is the source code for the TCPWM (square wave) code example.
*    See CE224876_TCPWM_Square_Wave.pdf for details.
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


#include "cy_pdl.h"
#include "cycfg.h"


/*******************************************************************************
* Function Name: main
********************************************************************************
*
* \brief This is the system entrance point for Cortex-M4.
* This function initializes the TCPWM (configured as PWM) and puts the Cortex-M4
* CPU in Sleep mode to save power.
*
* \param None
*
* \return None
*
*******************************************************************************/
int main(void)
{
    /* Variable to store API status */
    cy_en_tcpwm_status_t apiStatus;

    /* Set up internal routing, pins, and clock-to-peripheral connections
     * based on configurator selections */
    init_cycfg_all();

    /* Enable global interrupts */
    __enable_irq();

    /* Configure the TCPWM for PWM operation.
     * The ModusToolbox generates macros for parameters like TCPWM instance and count
     * which are used in PDL APIs (e.g. PWM_HW and PWM_NUM) and can be found in the
     * cycfg_peripherals.h file. See PSoC PDL API Reference document for API details. */
    apiStatus = Cy_TCPWM_PWM_Init(PWM_HW, PWM_NUM, &PWM_config);
    if(apiStatus != CY_TCPWM_SUCCESS)
    {
        /* Disable all interrupts */
        __disable_irq();
        /* Loop here infinitely - fault indication */
        for(;;);
    }
    Cy_TCPWM_Enable_Multiple(PWM_HW, PWM_MASK);
    Cy_TCPWM_TriggerStart(PWM_HW, PWM_MASK);

    for(;;)
    {
        /* Put the CPU into Sleep mode to save power */
        while(Cy_SysPm_Sleep(CY_SYSPM_WAIT_FOR_INTERRUPT) != CY_SYSPM_SUCCESS);
    }
}


/* [] END OF FILE */
