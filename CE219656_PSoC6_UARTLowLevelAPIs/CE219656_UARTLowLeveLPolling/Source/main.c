/******************************************************************************
* File Name: main.c
*
* Version: 1.0
*
* Description: This example demonstrates the UART transmit and receive
*              operation using low level polling APIs.
*
* Related Document: CE219656_PSoC6_UARTLowLevelAPIs.pdf
*
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
*        Function Prototypes
*******************************************************************************/
void handle_error(void);

/*******************************************************************************
*            Constants
*******************************************************************************/
#define LED_ON              (0u)
#define LED_OFF             (!LED_ON)

/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  The main function performs the following actions:
*   1. Sets up KIT_UART component.
*   2. If initialization of KIT_UART component fails then switch on RED_LED_ERROR.
*   3. KIT_UART sends text header into the serial terminal.
*   4. Re-transmits whatever the user types on the console.
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

    cy_stc_scb_uart_context_t KIT_UART_context;
    uint32_t read_data;
	cy_en_scb_uart_status_t initstatus;

	Cy_GPIO_Write(KIT_LED2_PORT, KIT_LED2_NUM, LED_OFF);

	/* Start KIT_UART operation. */
	initstatus = Cy_SCB_UART_Init(KIT_UART_HW, &KIT_UART_config, &KIT_UART_context);
	if(initstatus!=CY_SCB_UART_SUCCESS)
	{
		handle_error();
	}

	Cy_SCB_UART_Enable(KIT_UART_HW);

	/* Transmit header to the terminal. */
	Cy_SCB_UART_PutString(KIT_UART_HW, "\r\n**********************************************************************************\r\n");
	Cy_SCB_UART_PutString(KIT_UART_HW, "This is an UART example, which uses low level polling APIs to demonstrate UART operation.\r\n");
	Cy_SCB_UART_PutString(KIT_UART_HW, "If you are able to read this text the terminal connection is configured correctly.\r\n");
	Cy_SCB_UART_PutString(KIT_UART_HW, "Start transmitting the characters to see an echo in the terminal.\r\n");
	Cy_SCB_UART_PutString(KIT_UART_HW, "\r\n**********************************************************************************\r\n");
 
    for(;;)
    {
        /* Check if there is a received character from user console */
        if (0UL != Cy_SCB_UART_GetNumInRxFifo(KIT_UART_HW))
        {
            /* Re-transmit whatever the user types on the console */
            read_data = Cy_SCB_UART_Get(KIT_UART_HW);
            while (0UL == Cy_SCB_UART_Put(KIT_UART_HW,read_data))
            {
            }
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
