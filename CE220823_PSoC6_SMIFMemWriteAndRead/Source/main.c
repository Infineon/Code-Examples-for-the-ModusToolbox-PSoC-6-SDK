/******************************************************************************
* File Name: main.c
*
* Version 1.0
*
* Description:  Demonstrates read/write operation to external memory by using
* Serial memory interface(SMIF) in Quad Serial peripheral interface (QSPI) mode.
* This example also checks the integrity of the read data against written data.
*
* Related Document: CE220823_PSoC6_SMIFMemWriteAndRead.pdf
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
#include "smif_mem.h"

/***************************************************************************
* Global constants
***************************************************************************/
#define SMIF_PRIORITY       (1u)      /* SMIF interrupt priority */
#define TIMEOUT_1_MS        (1000ul)  /* 1 ms timeout for all blocking functions */
#define PACKET_SIZE         (64u)     /* The memory Read/Write packet */
#define LED_BLINK_DELAY		(500u)
#define ADDRESS_SIZE        (3u)      /* Memory address size */

/***************************************************************************
* Global variables
***************************************************************************/
cy_stc_smif_context_t KIT_QSPI_context;

/***************************************************************************
* Forward declaration
***************************************************************************/
void Isr_SMIF(void);
bool TxRxEqualCheck(uint8_t txBuffer[],uint8_t rxBuffer[]);

/***************************************************************************
* Function Name: main.c
***************************************************************************/
int main(void)
{
    /* Set up internal routing, pins, and clock-to-peripheral connections */
    init_cycfg_all();
    cy_stc_scb_uart_context_t UART_context;
    bool status;

    Cy_SCB_UART_Init(KIT_UART_HW, &KIT_UART_config, &UART_context);
    Cy_SCB_UART_Enable(KIT_UART_HW);

    /* enable interrupts, and the CM4 */
    __enable_irq();

    printf("\r\n\r\nUART initialization complete\r\n");
    printf("\r\n\r\nSMIF code example started\r\n");

    /* Configure SMIF interrupt */
    cy_stc_sysint_t smifIntConfig =
    {
        .intrSrc = smif_interrupt_IRQn,     /* SMIF interrupt */
        .intrPriority = SMIF_PRIORITY       /* SMIF interrupt priority */
    };

    /* SMIF interrupt initialization status */
    cy_en_sysint_status_t intr_init_status;
    intr_init_status = Cy_SysInt_Init(&smifIntConfig, Isr_SMIF);
    CheckStatus("\r\n\r\nSMIF interrupt initialization failed\r\n", (uint32_t)intr_init_status);

    printf("SMIF interrupt initialization complete\r\n");

    /* Initialize SMIF */
    cy_en_smif_status_t smif_status;
    smif_status = Cy_SMIF_Init(KIT_QSPI_HW, &KIT_QSPI_config, TIMEOUT_1_MS, &KIT_QSPI_context);
    CheckStatus("\r\n\r\nSMIF initialization failed\r\n", smif_status);

    /* Configure data select, refer the kit schematics to choose the right configuration */
    Cy_SMIF_SetDataSelect(KIT_QSPI_HW, CY_SMIF_SLAVE_SELECT_0, CY_SMIF_DATA_SEL0);
    Cy_SMIF_Enable(KIT_QSPI_HW, &KIT_QSPI_context);
    printf("SMIF initialization complete\r\n");

    /* Initialize the transfer buffers */
    uint8_t txBuffer[PACKET_SIZE] = {0};
    uint8_t rxBuffer[PACKET_SIZE] = {0};

    /* Filling the txBuffer with sample data to be transmitted */
    for(uint32_t index=0; index < PACKET_SIZE; index++)
	{
		txBuffer[index] = (uint8_t) (index & 0xFF);
	}

    /* Enable the SMIF interrupt */
    NVIC_EnableIRQ(smif_interrupt_IRQn);
    printf("=========================================================\r\n");
    printf("\r\nSMIF operation in Quad SPI mode\r\n");

    /* Erase before write */
    uint8_t extMemAddress[ADDRESS_SIZE] = {0x00, 0x00, 0x00};

    printf("\r\nErasing 64 bytes of memory \r\n");
    EraseMemory(KIT_QSPI_HW, (cy_stc_smif_mem_config_t*)smifMemConfigs[0], extMemAddress, &KIT_QSPI_context);

    printf("\r\nReading data from memory after Erase, ensure that the data read is 0xFF for each byte\r\n");
    ReadMemory(KIT_QSPI_HW, &KIT_QSPI_context, rxBuffer, PACKET_SIZE, extMemAddress);

    /* Reinitialize the buffers */
    for(uint32_t index=0; index < PACKET_SIZE; index++)
	{
		rxBuffer[index] = (uint8_t)0;
	}

    /* Write the sample bytes in txBuffer to the external memory address specified */
    WriteMemory(KIT_QSPI_HW, &KIT_QSPI_context, txBuffer, PACKET_SIZE, extMemAddress);

    /* Read data from external memory into rxBuffer */
    ReadMemory(KIT_QSPI_HW, &KIT_QSPI_context, rxBuffer, PACKET_SIZE, extMemAddress);

    status = TxRxEqualCheck(txBuffer, rxBuffer);
    /* Check if the transmitted and received arrays are equal */
    if (status)
    {
        printf("\r\nRead data matches with written data\r\n");
    }
    else
    {
        printf("\r\nRead data does not match with written data\r\n");
        printf("\r\nEither write or read operation failed\r\n");
    }

    while (1)
    {
        if (status)
        {
            Cy_GPIO_Inv(KIT_LED2_PORT, KIT_LED2_PIN); /* toggle the LED */
            Cy_SysLib_Delay(LED_BLINK_DELAY);
        }
        else
        {
            Cy_GPIO_Clr(KIT_LED2_PORT, KIT_LED2_PIN); /* Turn on the LED */
        }
    }
}

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
* Function Name: TxRxEqualCheck
****************************************************************************//**
*
* This function checks if the transmitted and received arrays are equal
*
* \param txBuffer - The buffer for Write data.
*
* \param rxBuffer - The buffer for Read data.
*
*******************************************************************************/
bool TxRxEqualCheck(uint8_t txBuffer[],uint8_t rxBuffer[])
{
    return(0U == memcmp(txBuffer, rxBuffer, PACKET_SIZE));
}

