/******************************************************************************
* \file main.c
*
* \version 1.0
*
* \brief
* Objective:
*     This is the source code for the SCB EZI2C code example.
*
* Compatible Kits:
*    CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*    CY8CKIT-062-WIFI-BT PSoC 6 WiFi-BT Kit
*    CY8CPROTO-062-4343W PSoC 6 WiFi-BT Prototyping Kit
*
* Related Document: CE220541.pdf
*
*******************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
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
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_device_headers.h"
#include "cycfg.h"
#include "cy_scb_ezi2c.h"
#include "cy_syslib.h"
#include "cy_sysint.h"
#include "cy_tcpwm_counter.h"

/* Size of the total EZI2C packet */
#define TOTAL_PACKET_SIZE 		(uint8) (0x02u)

/* Size of the writable EZI2C packet */
#define WRITE_PACKET_SIZE 		(uint8) (0x01u)

/* Respective indices of EZI2C data
 * 1st byte LED brightness, 2nd byte is a counter */
#define LED_INDEX   			(uint8) (0x00u)
#define COUNT_INDEX             (uint8) (0x01u)

/* TCPWM compare value for the LED OFF state */
#define LED_OFF 			    (uint8) (0x00u)

/************************ Variables Declaration *******************************/

/* I2C buffer for communication with master */
uint8 ezi2cBuffer[TOTAL_PACKET_SIZE];

/** The instance-specific context structure.
* It is used by the driver for internal configuration and
* data keeping for the I2C. Do not modify anything in this structure.
*/
cy_stc_scb_ezi2c_context_t KIT_I2C_context;

/* EZI2C_SCB_IRQ */
const cy_stc_sysint_t KIT_I2C_IRQ_config = {
    .intrSrc = (IRQn_Type) KIT_I2C_IRQ,
    .intrPriority = 7u
};

/******************************************************************************/

/*******************************************************************************
* Function Name: EZI2C_InterruptHandler
********************************************************************************
* Summary:
*  This function executes interrupt service routine.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*
*******************************************************************************/
void EZI2C_InterruptHandler(void)
{
    /* ISR implementation for EZI2C */
    Cy_SCB_EZI2C_Interrupt(KIT_I2C_HW, &KIT_I2C_context);
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU. It enables the EZI2C and TCPWM
* peripherals. The TCPWM is configured as counter that drives the LED.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    /* Variable to store EZI2C status */
    uint32 ezi2cState;

    /* Set up the device based on configurator selections */
    init_cycfg_all();

    __enable_irq();

    /* Initialize SCB for EZI2C operation */
    (void) Cy_SCB_EZI2C_Init(KIT_I2C_HW, &KIT_I2C_config, &KIT_I2C_context);

    /* Hook interrupt service routine and enable interrupt */
    Cy_SysInt_Init(&KIT_I2C_IRQ_config, &EZI2C_InterruptHandler);
    NVIC_EnableIRQ(KIT_I2C_IRQ_config.intrSrc);

    /* Enable global interrupts */
    __enable_irq();

    /* Configure buffer for communication with master */
    Cy_SCB_EZI2C_SetBuffer1(KIT_I2C_HW, ezi2cBuffer, TOTAL_PACKET_SIZE, WRITE_PACKET_SIZE, &KIT_I2C_context);

    /* Enable SCB for the EZI2C operation */
    Cy_SCB_EZI2C_Enable(KIT_I2C_HW);

    /* Start the PWM component corresponding to the LED control */
    (void) Cy_TCPWM_PWM_Init(KIT_LED2_PWM_HW, KIT_LED2_PWM_NUM, &KIT_LED2_PWM_config);
    Cy_TCPWM_Enable_Multiple(KIT_LED2_PWM_HW, KIT_LED2_PWM_MASK);
    Cy_TCPWM_TriggerStart(KIT_LED2_PWM_HW, KIT_LED2_PWM_MASK);

    for(;;)
    {
        /* Disable the EZI2C interrupts so that ISR is not serviced while
           checking for EZI2C status */
        NVIC_DisableIRQ(KIT_I2C_IRQ_config.intrSrc);

        /* Read the EZI2C status */
        ezi2cState = Cy_SCB_EZI2C_GetActivity(KIT_I2C_HW, &KIT_I2C_context);

        /* Write complete without errors: parse packets, otherwise ignore */
        if((0u != (ezi2cState & CY_SCB_EZI2C_STATUS_WRITE1)) && (0u == (ezi2cState & CY_SCB_EZI2C_STATUS_ERR)))
        {
            /* Count the number of writes */
            ezi2cBuffer[COUNT_INDEX]++;

            /* Update the compare value of the TCPWMs for color control */
			Cy_TCPWM_Counter_SetCompare0(KIT_LED2_PWM_HW, KIT_LED2_PWM_NUM, ezi2cBuffer[LED_INDEX]);

        }

        /* Enable interrupts for servicing ISR */
        NVIC_EnableIRQ(KIT_I2C_IRQ_config.intrSrc);
    }

    return (0);
}
