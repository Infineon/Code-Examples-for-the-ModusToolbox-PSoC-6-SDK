/******************************************************************************
* File Name: main.c
*
* Version 2.0
*
* Description:
* 	Demonstrates interfacing with an external memory by using the Serial Memory
* 	Interface (SMIF) hardware block in Quad Serial Peripheral Interface (QSPI)
* 	mode.
*
* Related Document: CE220823_PSoC6_SMIFMemWriteAndRead.pdf
*
*******************************************************************************
* Copyright (2018-2019), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions.  Therefore, you may use this Software only as
* provided in the license agreement accompanying the software package from which
* you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source code
* solely for use in connection with Cypress’s integrated circuit products.  Any
* reproduction, modification, translation, compilation, or representation of
* this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does not
* authorize its products for use in any products where a malfunction or failure
* of the Cypress product may reasonably be expected to result in significant
* property damage, injury or death ("High Risk Product"). By including Cypress's
* product in a High Risk Product, the manufacturer of such system or application
* assumes all risk of such use and in doing so agrees to indemnify Cypress
* against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cycfg.h"
#include "cycfg_qspi_memslot.h"
#include "smif_mem.h"
#include "stdio.h"
#include "string.h"


/***************************************************************************
* Global constants
***************************************************************************/
#define SMIF_PRIORITY       	(1u)      /* SMIF interrupt priority */
#define TIMEOUT_1_MS        	(1000ul)  /* 1 ms timeout for all blocking SMIF functions */
#define PACKET_SIZE         	(64u)     /* Memory Read/Write size */
#define MAX_ADDRESS_SIZE    	(4u)      /* Memory address size */
#define NUM_BYTES_PER_LINE		(16u)	  /* Used when array of data is printed on the console */
#define LED_TOGGLE_DELAY_MSEC	(1000u)	  /* LED blink delay */


/***************************************************************************
* Global variables
***************************************************************************/
cy_stc_smif_context_t KIT_QSPI_context;


/*******************************************************************************
* Function Name: Isr_SMIF
********************************************************************************
*
* The ISR for the SMIF interrupt. All Read/Write transfers to/from the external
* memory are processed inside the SMIF ISR.
*
*******************************************************************************/
void Isr_SMIF(void)
{
    Cy_SMIF_Interrupt(KIT_QSPI_HW, &KIT_QSPI_context);
}


/*******************************************************************************
* Function Name: CheckStatus
****************************************************************************//**
*
* Prints the message, indicates the non-zero status by turning the LED on, and
* asserts the non-zero status.
*
* \param message - message to print if status is non-zero.
*
* \param status - status for evaluation.
*
*******************************************************************************/
void CheckStatus(char *message, uint32_t status)
{
    if(0u != status)
    {
    	printf("\n================================================================================\n");
        printf("\nFAIL: %s\n", message);
        printf("Error Code: 0x%08lX\n", status);
        printf("\n================================================================================\n");
        Cy_GPIO_Clr(KIT_LED1_PORT, KIT_LED1_PIN); /* On failure, turn the LED ON */
        while(true); /* Wait forever here when error occurs. */
    }
}


/*******************************************************************************
* Function Name: PrintArray
****************************************************************************//**
*
* Prints the content of the buffer to the UART console.
*
* \param message - message to print before array output
*
* \param buf - buffer to print on the console.
*
* \param size - size of the buffer.
*
*******************************************************************************/
void PrintArray(char *message, uint8_t *buf, uint32_t size)
{
    printf("\n%s (%lu bytes):\n", message, size);
    printf("-------------------------\n");

    for(uint32_t index = 0; index < size; index++)
    {
        printf("0x%02X ", buf[index]);

        if(0u == ((index + 1) % NUM_BYTES_PER_LINE))
        {
        	printf("\n");
        }
    }
}


/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Initializes UART for console output and SMIF for interfacing an external
* memory, performs erase followed by write, verifies the written data by reading
* it back.
*
*******************************************************************************/
int main(void)
{
    /* Initialize clocks, pins, and assign clocks to peripherals */
    init_cycfg_all();

    cy_stc_scb_uart_context_t UART_context;
    Cy_SCB_UART_Init(KIT_UART_HW, &KIT_UART_config, &UART_context);
    Cy_SCB_UART_Enable(KIT_UART_HW);

    /* Enable interrupts to CM4 CPU */
    __enable_irq();
    printf("\nSMIF code example has started\n");

    /* Initialize SMIF interrupt */
    cy_stc_sysint_t smifIntConfig =
    {
        .intrSrc = KIT_QSPI_IRQ,
		.intrPriority = SMIF_PRIORITY
    };

    cy_en_sysint_status_t intrStatus = Cy_SysInt_Init(&smifIntConfig, Isr_SMIF);
    CheckStatus("SMIF interrupt initialization failed", (uint32_t)intrStatus);
    printf("SMIF interrupt is initialized\n");

    /* Initialize SMIF */
    cy_en_smif_status_t smifStatus;
    smifStatus = Cy_SMIF_Init(KIT_QSPI_HW, &KIT_QSPI_config, TIMEOUT_1_MS, &KIT_QSPI_context);
    CheckStatus("SMIF initialization failed", smifStatus);

    /* Configure slave select and data select. These settings depend on the pins
     * selected in the Device and QSPI configurators.
     */
    Cy_SMIF_SetDataSelect(KIT_QSPI_HW, smifMemConfigs[0]->slaveSelect, smifMemConfigs[0]->dataSelect);
    Cy_SMIF_Enable(KIT_QSPI_HW, &KIT_QSPI_context);
    NVIC_EnableIRQ(smifIntConfig.intrSrc); /* Finally, Enable the SMIF interrupt */

    printf("SMIF hardware block is initialized\n");
    printf("\n================================================================================\n");

    /* Initialize the transfer buffers */
    uint8_t txBuffer[PACKET_SIZE];
    uint8_t rxBuffer[PACKET_SIZE];

    /* Initialize tx buffer and rx buffer */
    for(uint32_t index = 0; index < PACKET_SIZE; index++)
	{
		txBuffer[index] = (uint8_t) (index & 0xFF);
		rxBuffer[index] = 0;
	}

    /* Enable Quad mode (1-1-4 or 1-4-4 modes) to use all the four I/Os during
     * communication.
     */
    bool isQuadEnabled = false;
    smifStatus = IsQuadEnabled(smifMemConfigs[0], &isQuadEnabled);
    CheckStatus("Checking QE bit failed", smifStatus);

    /* Though CheckStatus() will not return if the status indicates failure, the
     * status is checked here to show that the value of isQuadEnabled is valid
     * only when the return value indicates success.
     */
    if((CY_SMIF_SUCCESS == smifStatus) && !isQuadEnabled)
    {
    	/* Enabling Quad mode is a non-volatile write that should not be
    	 * interrupted by resets (e.g. from debugger during programming or
    	 * debugging) or by unstable power. Otherwise the configuration register
    	 * may be corrupted. Therefore, user confirmation is prompted through a
    	 * button press.
    	 */
    	printf("\r\nQuad mode is NOT enabled. Press & release the USER button to enable it.\r\n");
		while (true)
		{
			if (Cy_GPIO_Read(KIT_BTN1_PORT, KIT_BTN1_PIN) == 0u)
			{
				Cy_SysLib_Delay(50u); /* Button debounce delay */
				if (Cy_GPIO_Read(KIT_BTN1_PORT, KIT_BTN1_PIN) == 0u)
				{
					/* Wait until user releases the button */
					while (Cy_GPIO_Read(KIT_BTN1_PORT, KIT_BTN1_PIN) == 0u)
					{
						Cy_SysLib_Delay(50u);
					}
					break;
				}
			}

			Cy_SysLib_Delay(50u);
		}
    	smifStatus = EnableQuadMode(smifMemConfigs[0]);
    	CheckStatus("Enabling Quad mode failed", smifStatus);
    	printf("\r\nQuad mode is enabled.\r\n");
    }

	/* MSB of the address should be stored first.
	 * extMemAddress[0] = address[31:24]
	 * extMemAddress[1] = address[23:16]
	 * extMemAddress[2] = address[15:8]
	 * extMemAddress[3] = address[7:0]
	 */
    uint8_t extMemAddress[MAX_ADDRESS_SIZE] = {0x00, 0x00, 0x00, 0x00};

    /* Erase before write */
    printf("\n1. Erasing %lu bytes of memory.\n", smifMemConfigs[0]->deviceCfg->eraseSize);
    smifStatus = EraseMemory(smifMemConfigs[0], extMemAddress);
    CheckStatus("Erasing memory failed", smifStatus);

    /* Read after Erase to confirm that all data is 0xFF */
    printf("\n2. Reading after Erase. Ensure that the data read is 0xFF for each byte.\n");
    smifStatus = ReadMemory(smifMemConfigs[0], extMemAddress, rxBuffer, PACKET_SIZE);
    CheckStatus("Reading memory failed", smifStatus);
    PrintArray("Received Data", rxBuffer, PACKET_SIZE);

    /* Write the content of the txBuffer to the memory */
    printf("\n3. Writing data to memory.\n");
    smifStatus = WriteMemory(smifMemConfigs[0], extMemAddress, txBuffer, PACKET_SIZE);
    CheckStatus("Writing to memory failed", smifStatus);
    PrintArray("Written Data", txBuffer, PACKET_SIZE);

    /* Read back after Write for verification */
    printf("\n4. Reading back for verification.\n");
    smifStatus = ReadMemory(smifMemConfigs[0], extMemAddress, rxBuffer, PACKET_SIZE);
    CheckStatus("Reading memory failed", smifStatus);
    PrintArray("Received Data", rxBuffer, PACKET_SIZE);

    /* Check if the transmitted and received arrays are equal */
    CheckStatus("Read data does not match with written data. Read/Write operation failed.",
    		memcmp(txBuffer, rxBuffer, PACKET_SIZE));

    printf("\n================================================================================\n");
    printf("\nSUCCESS: Read data matches with written data!\n");
    printf("\n================================================================================\n");

    /* Blink LED to indicate success */
    while(true)
    {
    	Cy_GPIO_Inv(KIT_LED1_PORT, KIT_LED1_PIN);
		Cy_SysLib_Delay(LED_TOGGLE_DELAY_MSEC);
    }
}


