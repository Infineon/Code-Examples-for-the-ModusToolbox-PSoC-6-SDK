/******************************************************************************
* File Name: main.c
*
* Version 1.0
*
* Description: This example demonstrates the Watchdog Timer (WDT)
* on the PSoC 6 MCU, using ModusToolbox IDE
*
* Related Document: CE220060_PSoC6_WatchdogTimer.pdf
*
* Hardware Dependency: PSoC 6 BLE Pioneer Kit, PSoC 6 WiFi-BT Pioneer Kit,
* 					   PSoC 6 Wi-Fi BT Prototyping Kit
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
* ****************************************************************************/

#include "cy_device_headers.h"
#include "cycfg.h"

#include "cy_wdt.h"
#include "cy_sysint.h"

/* WDT demo options */
#define WDT_RESET_DEMO						1
#define WDT_INTERRUPT_DEMO 					2
#define WDT_DEMO 							WDT_INTERRUPT_DEMO

/* WDT Interrupt Number */
#define WDT_IRQ__INTC_NUMBER 				srss_interrupt_IRQn
#define WDT_IRQ__INTC_CORTEXM4_PRIORITY 	7

/* WDT periodic interrupt timing */
#define WDT_INTERRUPT_INTERVAL				1000  //ms

/* Match count =  Desired interrupt interval in seconds x ILO Frequency in Hz */
#define WDT_MATCH_COUNT 					WDT_INTERRUPT_INTERVAL*32000/1000

/* WDT interrupt configuration structure */
const cy_stc_sysint_t WDT_IRQ_cfg = {
	.intrSrc = (IRQn_Type)WDT_IRQ__INTC_NUMBER,
	.intrPriority = WDT_IRQ__INTC_CORTEXM4_PRIORITY
};

/* Function prototypes */
void InitializeWDT(void);
void WdtInterruptHandler(void);

/******************************************************************************
* Function Name: main
*******************************************************************************
*
* Summary: This is the main function for CM4 CPU. It configures the WDT in either
*          reset mode or periodic interrupt mode.
*
* Parameters:
*  None
*
* Return:
* int
*
******************************************************************************/
int main(void)
{
	/* Set up the device based on configurator selections */
	init_cycfg_all();

	/* Check the reason for device restart */
	if(CY_SYSLIB_RESET_HWWDT == Cy_SysLib_GetResetReason())
	{
		/* It's WDT reset event - blink LED twice */
		Cy_GPIO_Clr(KIT_LED2_PORT, KIT_LED2_NUM);
		Cy_SysLib_Delay(100);
		Cy_GPIO_Set(KIT_LED2_PORT, KIT_LED2_NUM);
		Cy_SysLib_Delay(200);
		Cy_GPIO_Clr(KIT_LED2_PORT, KIT_LED2_NUM);
		Cy_SysLib_Delay(100);
		Cy_GPIO_Set(KIT_LED2_PORT, KIT_LED2_NUM);
	}
	else
	{
		/* It's Power-On reset or XRES event - blink LED once */
		Cy_GPIO_Clr(KIT_LED2_PORT, KIT_LED2_NUM);
		Cy_SysLib_Delay(100);
		Cy_GPIO_Set(KIT_LED2_PORT, KIT_LED2_NUM);
		Cy_SysLib_Delay(100);
	}

	/* Initialize WDT */
	InitializeWDT();

	/* Enable global interrupt */
	__enable_irq();

	for(;;)
	{
		#if(WDT_DEMO == WDT_RESET_DEMO)
			/* Clear WDT match event */
			Cy_WDT_ClearWatchdog();

			/* Uncomment this blocking code to cause device reset */
			//while(1);

			/* Constant delay of 1000ms */
			Cy_SysLib_Delay(1000);

			/* Invert the state of LED */
			Cy_GPIO_Inv(KIT_LED2_PORT, KIT_LED2_NUM);
		#endif

		#if(WDT_DEMO == WDT_INTERRUPT_DEMO)
			/* Uncomment below code to demonstrate wake up from deep sleep */
			//Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
		#endif
	}
}

/******************************************************************************
* Function Name: InitializeWDT
*******************************************************************************
*
* Summary: This function initializes the WDT block
*
* Parameters:
*  None
*
* Return:
*  None
*
******************************************************************************/
void InitializeWDT()
{
	 /* Step 1- Unlock WDT */
	Cy_WDT_Unlock();

	/* Step 2- Write the ignore bits - operate with full 16 bits */
	Cy_WDT_SetIgnoreBits(0);

	/* Step 3- Write match value */
	#if(WDT_DEMO == WDT_INTERRUPT_DEMO)
		Cy_WDT_SetMatch(WDT_MATCH_COUNT);
	#else
		Cy_WDT_SetMatch(0);
	#endif

	/* Step 4- Clear match event interrupt, if any */
	Cy_WDT_ClearInterrupt();

	/* Step 5- Enable ILO */
	Cy_SysClk_IloEnable();

	/* Step 6 - Enable interrupt if periodic interrupt mode selected */
	#if(WDT_DEMO == WDT_INTERRUPT_DEMO)
		Cy_SysInt_Init(&WDT_IRQ_cfg, WdtInterruptHandler);
		NVIC_EnableIRQ(WDT_IRQ_cfg.intrSrc);
		Cy_WDT_UnmaskInterrupt();
	#endif

	/* Step 7- Enable WDT */
	Cy_WDT_Enable();

	/* Step 8- Lock WDT configuration */
	Cy_WDT_Lock();
}

/******************************************************************************
* Function Name: WdtInterruptHandler
*******************************************************************************
*
* Summary: This function is the handler for the WDT interrupt
*
* Parameters:
*  None
*
* Return:
*  None
*
******************************************************************************/
void WdtInterruptHandler(void)
{
	/* Check if the interrupt is from WDT */
	if(SRSS_SRSS_INTR & SRSS_SRSS_INTR_WDT_MATCH_Msk)
	{
		/* Clear WDT Interrupt */
		Cy_WDT_ClearInterrupt();

		/* Update the match count */
		Cy_WDT_Unlock();

		/* Update the match count. Note that, if the ignore bits in the match is set
		 * to a non-zero value, the below code requires a modification. */
		Cy_WDT_SetMatch((uint16)(Cy_WDT_GetMatch() + WDT_MATCH_COUNT));
		Cy_WDT_Lock();

		/* Invert the state of LED */
		Cy_GPIO_Inv(KIT_LED2_PORT, KIT_LED2_NUM);
	}
}

