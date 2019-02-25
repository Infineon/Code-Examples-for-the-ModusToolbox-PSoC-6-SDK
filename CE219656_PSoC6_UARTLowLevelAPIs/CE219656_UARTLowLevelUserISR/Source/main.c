/******************************************************************************
* File Name: main.c
*
* Version: 1.0
*
* Description: This example demonstrates the UART transmit and receive
*              operation using an ISR.
*
* Related Document: CE219656_PSoC6_UARTLowLevelAPIs.pdf
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

#include "cy_pdl.h"
#include "cycfg.h"

/*******************************************************************************
*        Forward Declaration
*******************************************************************************/
void handle_error(void);
void Isr_UART(void);

/*******************************************************************************
*        Constants
*******************************************************************************/
#define LED_ON  (0u)
#define LED_OFF (!LED_ON)

/*******************************************************************************
*        Global variables
*******************************************************************************/
uint32_t data_received;
uint8_t uart_error;
uint32_t read_data;

/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  The main function performs the following actions:
*   1. Sets up UART component.
*   2. If initialization of UART component fails then switch on RED_LED_ERROR.
*   3. UART sends text header into the serial terminal.
*   4. Unmask UART RX fifo not empty interrupt
*   5. Enable UART interrupts
*   6. Wait for UART RX fifo not empty interrupt
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
int main(void)
{
    /* Set up the device based on configurator selections */
    init_cycfg_all();

    __enable_irq();
    cy_en_scb_uart_status_t init_status;

    /* Populate configuration structure (code specific for CM4) */
    cy_stc_sysint_t KIT_UART_SCB_IRQ_cfg =
    {
        .intrSrc      = KIT_UART_IRQ,
        .intrPriority = 7u,
    };
    cy_stc_scb_uart_context_t KIT_UART_context;

    /* Start UART operation. */
    init_status = Cy_SCB_UART_Init(KIT_UART_HW, &KIT_UART_config, &KIT_UART_context);
    if(init_status!=CY_SCB_UART_SUCCESS)
    {
        handle_error();
    }
    Cy_SCB_UART_Enable(KIT_UART_HW);

    /* Transmit header to the terminal. */
    Cy_SCB_UART_PutString(KIT_UART_HW, "\r\n**********************************************************************************\r\n");
    Cy_SCB_UART_PutString(KIT_UART_HW, "This is an UART example, which uses User ISR to demonstrate UART operation.\r\n");
    Cy_SCB_UART_PutString(KIT_UART_HW, "If you are able to read this text the terminal connection is configured correctly.\r\n");
    Cy_SCB_UART_PutString(KIT_UART_HW, "Start typing the characters to see an echo in the terminal.\r\n");
    Cy_SCB_UART_PutString(KIT_UART_HW, "\r\n**********************************************************************************\r\n");

    /* Interrupt Settings for UART */
    Cy_SysInt_Init(&KIT_UART_SCB_IRQ_cfg, Isr_UART);

    /* Enable the interrupt */
    NVIC_EnableIRQ(KIT_UART_SCB_IRQ_cfg.intrSrc);

    /* Initialize flags */
    data_received = 0;
    uart_error = 0;
 
    for(;;)
    {
    	/* Handle received data from terminal */
		if(data_received == 1)
		{
	        /* Get the character from terminal */
			//read_data = Cy_SCB_UART_Get(KIT_UART_HW);
			data_received = 0;

			/* Echo the character to terminal */
			while (0UL == Cy_SCB_UART_Put(KIT_UART_HW,read_data))
			{
			}
		}

		/* Handle UART error */
		if(uart_error == 1)
		{
			handle_error();
		}
    }
}


/***************************************************************************//**
* Function Name: handle_error
********************************************************************************
*
* Summary:
* This function processes unrecoverable errors such as any component
* initialization errors etc. In case of such error the system will switch on
* RED_LED_ERROR and stay in the infinite loop of this function.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void handle_error(void)
{
     /* Disable all interrupts */
    __disable_irq();

    /* Switch on error LED */
    Cy_GPIO_Write(KIT_LED2_PORT, KIT_LED2_NUM, LED_ON);
    while(1u) {}
}

/***************************************************************************//**
* Function Name: Isr_UART
********************************************************************************
*
* Summary:
*  This function is registered to be called when UART interrupt occurs.
*  (Note that only RX fifo not empty interrupt is unmasked)
*  Whenever there is a data in UART RX fifo, Get that data and Put it into
*  UART TX fifo which will be transmitted to terminal
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
void Isr_UART(void)
{
    /* Check for "RX fifo not empty interrupt" */
    if((KIT_UART_HW->INTR_RX_MASKED) & SCB_INTR_RX_MASKED_NOT_EMPTY_Msk)
    {

        /* Clear UART "RX fifo not empty interrupt" */
		KIT_UART_HW->INTR_RX = KIT_UART_HW->INTR_RX & SCB_INTR_RX_NOT_EMPTY_Msk;

		/* Get the character from terminal */
		read_data = Cy_SCB_UART_Get(KIT_UART_HW);

        /* Update data_received flag */
        data_received = 1;

	}
    else
    {
        /* Error if any other interrupt occurs */
        uart_error = 1;
    }
}

