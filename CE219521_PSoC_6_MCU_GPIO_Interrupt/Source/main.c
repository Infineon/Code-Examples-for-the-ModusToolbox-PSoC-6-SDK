/******************************************************************************
* File Name: main.c
*
* Version 1.0
*
* Description:
*  This code example demonstrates the use of GPIO configured as an input pin to
*  generate interrupts on CM4 CPU in PSoC 6 MCU.
*
* Related Document: CE219521_PSoC_6_MCU_GPIO_Interrupt.pdf
*
* Hardware Dependency: See CE219521_PSoC_6_MCU_GPIO_Interrupt.pdf
*
*******************************************************************************
* Copyright (2017-2018), Cypress Semiconductor Corporation. All rights reserved.
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

#include "cy_device_headers.h"
#include "cycfg.h"
#include "cy_sysint.h"
#include "stdbool.h"

/* Macros for configuring project functionality */
#define DELAY_SHORT             (250)   /* milliseconds */
#define DELAY_LONG              (500)   /* milliseconds */

#define LED_BLINK_COUNT		    (4)

#define LED_ON                  (0)
#define LED_OFF                 (1)

#define SWITCH_INTR_PRIORITY	(3u)

/* Switch interrupt configuration structure */
const cy_stc_sysint_t switch_intr_config = {
	.intrSrc = ioss_interrupts_gpio_0_IRQn,		/* Source of interrupt signal */
	.intrPriority = SWITCH_INTR_PRIORITY		/* Interrupt priority */
};

/* Global variable */
uint32_t interrupt_flag = false;

/*******************************************************************************
* Function Name: void Switch_ISR(void)
********************************************************************************
*
* Summary:
*  This function is executed when GPIO interrupt is triggered.
*
* Parameters: None
*
* Return: None
*
*******************************************************************************/
void Isr_switch(void)
{
	/* Clears the triggered pin interrupt */
	Cy_GPIO_ClearInterrupt(KIT_BTN1_PORT, KIT_BTN1_NUM);
	NVIC_ClearPendingIRQ(switch_intr_config.intrSrc);

	/* Set interrupt flag */
	interrupt_flag = true;
}

/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  System entrance point. This function configures and initializes the GPIO
*  interrupt, update the delay on every GPIO interrupt, blinks the LED and enter
*  in deepsleep mode.
*
* Parameters: None
*
* Return: int
*
*******************************************************************************/

int main(void)
{
	uint32_t count = 0;
	uint32_t delayMs = DELAY_LONG;

    /* Set up the device based on configurator selections */
    init_cycfg_all();

    /* Enable interrupt */
    __enable_irq();
 
    /* Initialize and enable GPIO interrupt assigned to CM0+ */
	Cy_SysInt_Init(&switch_intr_config, Isr_switch);
	NVIC_ClearPendingIRQ(switch_intr_config.intrSrc);
	NVIC_EnableIRQ(switch_intr_config.intrSrc);
	
	for(;;)
	{
		/* Check the interrupt status */
		if(true == interrupt_flag)
		{
			interrupt_flag = false;

			if(DELAY_LONG == delayMs)
			{
				delayMs = DELAY_SHORT;
			}
			else
			{
				delayMs = DELAY_LONG;
			}
		}

		/* Blink LED four times */
		for (count = 0; count < LED_BLINK_COUNT; count++)
		{
			Cy_GPIO_Write(KIT_LED2_PORT, KIT_LED2_NUM, LED_ON);
			Cy_SysLib_Delay(delayMs);
			Cy_GPIO_Write(KIT_LED2_PORT, KIT_LED2_NUM, LED_OFF);
			Cy_SysLib_Delay(delayMs);
		}

		/* Enter deep sleep mode */
		Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
	}
}

/* [] END OF FILE */