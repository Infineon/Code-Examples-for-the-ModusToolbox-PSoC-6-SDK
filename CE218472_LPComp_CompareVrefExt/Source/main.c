/******************************************************************************
* File Name: main.c
*
* Version 1.0
*
* Description:
*   This code example demonstrates how to set the Low-Power Comparator for
*   comparing the internal reference voltage and external voltage.
*
* Related Document: CE218472_LPComp_VrefExt.pdf
*
* Hardware Dependency: See CE218472_LPComp_VrefExt.pdf
*
*******************************************************************************
* Copyright (2018-2019), Cypress Semiconductor Corporation. All rights reserved.
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

/* Include header */
#include "cy_pdl.h"
#include "cycfg.h"

/* LPComp interrupt initialization structure. */
#define LPCOMP_INTR_PRIORITY   7

/* LPComp interrupt status flag */
bool lpcompInterruptFlag = false;

/* LPComp interrupt configuration structure */
const cy_stc_sysint_t lpcomp_IRQ_cfg = {
	.intrSrc 		= lpcomp_interrupt_IRQn,
	.intrPriority 	= LPCOMP_INTR_PRIORITY
};

/* ISR for LPComp interrupt */
void LPComp_ISR_Callback(void);

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary: System entrance point. This function configures the LPComp for
*  comparing external voltage and internal reference voltage. LPComp is configured
*  in the ultra-low power(ULP) mode for LPComp because only the ULP can compare
*  an external voltage and the internal reference in the deep sleep mode.
*  The main function continuously checks the LPComp interrupt flag and indicate
*  the comparison result on the LED.
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
    /* Set up internal routing, pins, and clock-to-peripheral connections */
    init_cycfg_all();

    /* Enable global interrupt */
    __enable_irq();

    /* Configure LPComp interrupt */
	Cy_SysInt_Init(&lpcomp_IRQ_cfg, LPComp_ISR_Callback);
	NVIC_ClearPendingIRQ(lpcomp_IRQ_cfg.intrSrc);
	NVIC_EnableIRQ(lpcomp_IRQ_cfg.intrSrc);

	/* Configure LPComp channel 1 */
	Cy_LPComp_Init(LPCOMP, CY_LPCOMP_CHANNEL_1, &LPComp_config);
	Cy_LPComp_Enable(LPCOMP, CY_LPCOMP_CHANNEL_1);

	/* Enable ULP local VREF */
	Cy_LPComp_UlpReferenceEnable(LPCOMP);

	/* Set the positive terminal to GPIO input and the negative terminal to the internal reference */
	Cy_LPComp_SetInputs(LPCOMP, CY_LPCOMP_CHANNEL_1, CY_LPCOMP_SW_GPIO, CY_LPCOMP_SW_LOCAL_VREF);

	/* Set the interrupt for LPComp channel 1 */
	Cy_LPComp_SetInterruptMask(LPCOMP, CY_LPCOMP_CHANNEL_1);

	/* Update the status of LED */
	Cy_GPIO_Write(KIT_LED2_PORT, KIT_LED2_NUM, Cy_LPComp_GetCompare(LPCOMP, CY_LPCOMP_CHANNEL_1));

	for(;;)
	{
		if(lpcompInterruptFlag)
		{
			/* Reset interrupt flag */
			lpcompInterruptFlag = false;
			/* Update the status of LED */
			Cy_GPIO_Write(KIT_LED2_PORT, KIT_LED2_NUM, Cy_LPComp_GetCompare(LPCOMP, CY_LPCOMP_CHANNEL_1));
		}

		/* Go to deep sleep mode and only wake up if an interrupt occurs */
		Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
	}
}

/*******************************************************************************
* Function Name: LPComp_ISR_Callback
********************************************************************************
* Summary: This interrupt service routine sets the flag to notify the LPComp
* triggering to the main loop.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void LPComp_ISR_Callback(void)
{
	/* Clear pending LPComp interrupt */
	Cy_LPComp_ClearInterrupt(LPCOMP, CY_LPCOMP_CHANNEL_1);

	/* Set interrupt flag */
	lpcompInterruptFlag = true;
}

/* [] END OF FILE */
