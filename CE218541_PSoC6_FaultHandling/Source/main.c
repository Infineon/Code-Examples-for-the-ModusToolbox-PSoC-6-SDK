/******************************************************************************
* File Name: main.c
*
* Version 1.0
*
* Description:
*  This example demonstrates the fault handling functionality of PSoC 6 MCU
*  and how to add a custom fault handling function to find the fault location.
*  The UART interface will be used for showing ARM register information.
*
* Related Document: CE218541_PSoC6_FaultHandling.pdf
*
* Hardware Dependency: See CE218541_PSoC6_FaultHandling.pdf
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

/* Header files */
#include "cy_pdl.h"
#include "cycfg.h"
#include "stdio_user.h"
#include <stdio.h>

/* Constants */
#define SWITCH_PRESSED      (0u)
#define SWITCH_RELEASED     (!SWITCH_PRESSED)
#define SWITCH_PRESS_DELAY  (50u)   /* 50 msec */
#define LONG_SWITCH_PRESS   (40u)   /* 40 * 50 msec = 2 sec   */
#define SHORT_SWITCH_PRRESS (3u)    /* 3 * 50 msec  = 150 msec */

/* Macro used to read switch state */
#define SWITCH_STATE         Cy_GPIO_Read(KIT_BTN1_PORT, KIT_BTN1_NUM)

/* Variable used for generating Bus Fault by modifying the constant variable */
const uint32_t writeToCauseFaultCM4 = 0u;

/* Function prototype */
static void InitializeFaultRegister(void);
static void ForceBusFaultCM4(void);
static uint32_t ForceUsageFaultCM4(uint32_t*);

/* UART context structure */
static cy_stc_scb_uart_context_t KIT_UART_context;

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function calls the initializing fault registers
*  and processes switch press event.
*
* Parameters:
*  None
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
	/* Local variables */
	uint32_t prevState = SWITCH_RELEASED;
	uint32_t pressCount = 0;
	uint32_t var = 0;

    /* Set up internal routing, pins, and clock-to-peripheral connections */
    init_cycfg_all();

    /* Enable global interrupt */
    __enable_irq();

    Cy_SCB_UART_Init(KIT_UART_HW, &KIT_UART_config, &KIT_UART_context);
	Cy_SCB_UART_Enable(KIT_UART_HW);

	/* Configure necessary fault handling register */
	InitializeFaultRegister();

	printf("PSoC 6 MCU Fault Handling Basics\r\n");
	printf("Press SW2 and release once quickly - Force usage fault for CM4\r\n");
	printf("Press the RST button to start over once a fault has been triggered\r\n");
	printf("Press and hold SW2 for at least 2 seconds, then release - Force a CM4 bus fault\r\n");

	for(;;)
	{
		if(SWITCH_PRESSED == SWITCH_STATE)
		{
			if(SWITCH_PRESSED == prevState)
			{
				pressCount++;
			}
			else
			{
				prevState = SWITCH_PRESSED;
			}
		}
		else
		{
			if(pressCount >= LONG_SWITCH_PRESS)
			{
				printf("\r\nForce CM4 Bus Fault!\r\n");
				ForceBusFaultCM4();
			}
			else if(pressCount >= SHORT_SWITCH_PRRESS)
			{
				printf("\r\nForce CM4 Usage Fault!\r\n");
				if (0 != ForceUsageFaultCM4(&var))
				{
					printf("\rFailed to create CM4 Usage Fault!");
				}
			}
			prevState = SWITCH_RELEASED;
		}
		Cy_SysLib_Delay(SWITCH_PRESS_DELAY);
	}
}

/*******************************************************************************
* Function Name: static void InitializeFaultRegister(void)
********************************************************************************
* Summary:
*  This function configures the fault registers(bus fault and usage fault). See
*  the Arm documentation for more details on the registers.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void InitializeFaultRegister(void)
{
	/* Set SCB->SHCSR.BUSFAULTENA so that BusFault handler instead of the
	 * HardFault handler handles the BusFault.
	 */
	SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA_Msk;

	/* If ACTLR.DISDEFWBUF is not set to 1, the imprecise BusFault will occur.
	 * For the imprecise BusFault, the fault stack information won't be accurate.
	 * Setting ACTLR.DISDEFWBUF bit to 1 so that bus faults will be precise.
	 * Refer Arm documentation for detailed explanation on precise and imprecise
	 * BusFault.
	 * WARNING: This will decrease the performance because any store to memory
	 * must complete before the processor can execute the next instruction.
	 * Don't enable always, if it is not necessary.
	 */
	SCnSCB->ACTLR |= SCnSCB_ACTLR_DISDEFWBUF_Msk;

	/* Enable UsageFault when processor executes an divide by 0 */
	SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;

	/* Set SCB->SHCSR.USGFAULTENA so that faults such as DIVBYZERO, UNALIGNED,
	 * UNDEFINSTR etc are handled by UsageFault handler instead of the HardFault
	 * handler.
	 */
	SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk;
}

/*******************************************************************************
* Function Name: static void ForceBusFaultCM4(void)
********************************************************************************
* Summary:
*  The BusFault exception will be generated intentionally by writing a value
*  into read-only area.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void ForceBusFaultCM4(void)
{
	*(uint32_t *)&writeToCauseFaultCM4 = 10u;
}

/*******************************************************************************
* Function Name: static uint32_t ForceUsageFaultCM4(uint32_t* intVal)
********************************************************************************
* Summary:
*  This function divides 100 by intVal. It will generate the usage fault, if the
*  intVal is zero.
*  This simple logic is intentionally used a function to avoid the code
*  optimized out by a compiler.
*
* Parameters:
*  uint32_t* intVal : variable used to create usage fault
*
* Return:
*  uint32_t
*
*******************************************************************************/
static uint32_t ForceUsageFaultCM4(uint32_t* intVal)
{
	uint32_t faultNum = 100u;

	/* If *intVal = 0u then it triggers a fault because of DIVBYZERO (Divide by
	 * zero). The SCB->UFSR bit 9(=CFSR bit 25) will be set to 1, once the fault
	 * has occurred.
	 */
	faultNum /= *intVal;

	return faultNum;
}

/*******************************************************************************
* Function Name: void Cy_SysLib_ProcessingFault(void)
********************************************************************************
* Summary:
*  This function prints out the stacked register at the moment the hard fault
*  occurred. cy_syslib.c include same function as __WEAK option, this function
*  will replace the weak function. Cy_SysLib_ProcessingFault() is called at the
*  end of Cy_SysLib_FaultHandler() function which is the default exception
*  handler set for hard faults.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Cy_SysLib_ProcessingFault(void)
{

	printf("\r\nCM4 FAULT!!\r\n");
	printf("SCB->CFSR = 0x%08lx\r\n", (uint32_t) cy_faultFrame.cfsr.cfsrReg);

	/* If MemManage fault valid bit is set to 1, print MemManage fault address */
	if ((cy_faultFrame.cfsr.cfsrReg & SCB_CFSR_MMARVALID_Msk)
			== SCB_CFSR_MMARVALID_Msk)
	{
		printf("MemManage Fault! Fault address = 0x%08lx\r\n", SCB->MMFAR);
	}

	/* If Bus Fault valid bit is set to 1, print BusFault Address */
	if ((cy_faultFrame.cfsr.cfsrReg & SCB_CFSR_BFARVALID_Msk)
			== SCB_CFSR_BFARVALID_Msk)
	{
		printf("Bus Fault! \r\nFault address = 0x%08lx\r\n", SCB->BFAR);
	}

	/* Print Fault Frame */
	printf("r0 = 0x%08lx\r\n", cy_faultFrame.r0);
	printf("r1 = 0x%08lx\r\n", cy_faultFrame.r1);
	printf("r2 = 0x%08lx\r\n", cy_faultFrame.r2);
	printf("r3 = 0x%08lx\r\n", cy_faultFrame.r3);
	printf("r12 = 0x%08lx\r\n", cy_faultFrame.r12);
	printf("lr = 0x%08lx\r\n", cy_faultFrame.lr);
	printf("pc = 0x%08lx\r\n", cy_faultFrame.pc);
	printf("psr = 0x%08lx\r\n", cy_faultFrame.psr);

	while(1);
}

/* [] END OF FILE */
