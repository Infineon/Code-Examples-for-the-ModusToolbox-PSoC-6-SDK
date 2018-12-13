/******************************************************************************
* File Name: main.c
*
* Version 1.0
*
* Description: Demonstrates how to configure the register setting for waking up
*  from hibernate.
*
* Related Document: CE218129_LPComp_WakeupFromHibernate.pdf
*
* Hardware Dependency: See CE218129_LPComp_WakeupFromHibernate.pdf
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
/* Header files */
#include "cy_device_headers.h"
#include "cycfg.h"

/* LPComp ultra-low power settling time */
#define LPCOMP_ULP_SETTLE       (50u)

/* USER LED 2 toggle delay */
#define LED_TOGGLE_DELAY		(500u) /* 500 ms */

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  The main function configures the LPComp Channel 0 for waking up from the
*  hibernate and continuously toggles LED when system is in active state.
*
* Parameters:
*  None
*
* Return:
*  int
*
* Side Effects:
*  None
*
*******************************************************************************/
int main(void)
{
	/* Check the IO status. If current status is frozen, unfreeze the system. */
	if(Cy_SysPm_GetIoFreezeStatus())
	{
		/* Unfreeze the system */
		Cy_SysPm_IoUnfreeze();
	}

    /* Set up internal routing, pins, and clock-to-peripheral connections */
    init_cycfg_all();

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize and enable LPcomp channel 0 */
    Cy_LPComp_Init(LPCOMP, CY_LPCOMP_CHANNEL_0, &LPComp_config);
	Cy_LPComp_Enable(LPCOMP, CY_LPCOMP_CHANNEL_0);

	/* Enable the local reference voltage */
	Cy_LPComp_UlpReferenceEnable(LPCOMP);

	/* Set the local reference voltage to the negative terminal and set
	 * a GPIO input on the positive terminal for the wake up signal */
	Cy_LPComp_SetInputs(
			LPCOMP,					/* LPCOMP register structure pointer */
			CY_LPCOMP_CHANNEL_0,	/* LPCOMP channel index */
			CY_LPCOMP_SW_GPIO, 		/* LPCOMP positive input */
			CY_LPCOMP_SW_LOCAL_VREF /* LPCOMP negative input */
			);

	for(;;)
	{
		/* If the comparison result is high, toggles LED every 500 ms */
		if(Cy_LPComp_GetCompare(LPCOMP, CY_LPCOMP_CHANNEL_0) == 1)
		{
			/* Toggle LED every 500 ms */
			Cy_GPIO_Inv(KIT_LED2_PORT, KIT_LED2_NUM);
			Cy_SysLib_Delay(LED_TOGGLE_DELAY);
		}
		/* If the comparison result is low, goes to the hibernate mode */
		else
		{
			/* Turn off Red LED */
			Cy_GPIO_Set(KIT_LED2_PORT, KIT_LED2_NUM);

			/* Set hibernate wake up source */
			Cy_SysPm_SetHibWakeupSource(CY_SYSPM_LPCOMP0_HIGH);
			/* Enter hibernate mode */
			Cy_SysPm_Hibernate();
		}
	}
}

/* [] END OF FILE */

