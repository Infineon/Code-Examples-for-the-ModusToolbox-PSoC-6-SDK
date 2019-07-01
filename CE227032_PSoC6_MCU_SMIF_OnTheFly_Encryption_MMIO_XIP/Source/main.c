/******************************************************************************
* File Name: main.c
*
* Version 1.0
*
* Description:  This code example demonstrates on-the-fly encryption of data
* using Serial Memory Interface(SMIF) in MMIO and XIP modes. This example uses
* the CY15B104QSN Excelon™-Ultra 4-Mbit (512K × 8) Quad SPI F-RAM in SPI mode.
*
* Related Document: CE227032_PSoC6_MCU_SMIF_OnTheFly_Encryption_MMIO_XIP.pdf
*
* Compatible Kits:
*    CY8CKIT-062-WIFI-BT PSoC 6 Pioneer Kit
*
*******************************************************************************
* Copyright 2019, Cypress Semiconductor Corporation. All rights reserved.
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
#include <smif_fram.h>
#include "cy_pdl.h"
#include "cycfg.h"

/***************************************************************************
* Global constants
***************************************************************************/

#define SMIF_PRIORITY       (7u)      /* SMIF interrupt priority */
#define TIMEOUT_1_MS        (1000ul)  /* 1 ms timeout for all SMIF functions */
#define PACKET_SIZE         (32u)     /* The memory Read/Write packet */
#define LED_BLINK_DELAY		(500u)    /* LED blink delay in ms */
#define ADDRESS_SIZE        (3u)      /* External memory address size */
#define CRYPTO_KEY_SIZE     (16u)     /* Crypto key size in bytes - Do not edit */

/***************************************************************************
* Global variables
***************************************************************************/
/* Context structure for QSPI */
cy_stc_smif_context_t KIT_QSPI_context;

/***************************************************************************
* Function prototypes
***************************************************************************/
void Isr_SMIF(void);

/***************************************************************************
* Function Name: main.c
*
* Initializes and enables all peripherals.
*
* The function encrypts plain data and writes to F-RAM; reads encrypted
* data and decrypts it using MMIO mode. It also encrypts and decrypts data
* using XIP mode.
*
***************************************************************************/
int main(void)
{
	uint8_t cryptoKey[CRYPTO_KEY_SIZE] = {'C', 'y', 'p', 'r', 'e', 's', 's', \
			' ', 'P', 'S', 'o', 'C', '6', 'M', 'C', 'U'};

	/* Transmit and receive buffers */
	/* txBuffer -> plain data
	 * encryptedtxBuffer -> encrypted data
	 * rxBuffer -> received data
	 * rxBuffer_XIP -> pointer to data in FRAM using XIP memory range
	 */
    uint8_t txBuffer[PACKET_SIZE];
    uint8_t encryptedtxBuffer[PACKET_SIZE];
    uint8_t rxBuffer[PACKET_SIZE];

    /* Set rxBuffer_XIP address to point to the base address of
     * CY15B104QSN FRAM in PSoC 6 memory map */
    uint8_t *rxBuffer_XIP = (uint8_t *) (CY15B104QSN_SlaveSlot_2.baseAddress);

    /* Context structure for UART */
    cy_stc_scb_uart_context_t UART_context;

    /* Configure SMIF interrupt */
    cy_stc_sysint_t smifIntConfig =
        {
            .intrSrc = KIT_QSPI_IRQ,         /* SMIF interrupt */
            .intrPriority = SMIF_PRIORITY    /* SMIF interrupt priority */
        };

    /* Status */
    uint32_t errorStatus;
    cy_en_smif_status_t smifStatus;

    /* External memory address - 3 bytes */
    uint8_t extMemAddress[ADDRESS_SIZE] = {0x00, 0x00, 0x00};

    /* Set up internal routing, pins, and clock-to-peripheral connections */
    init_cycfg_all();

    Cy_SCB_UART_Init(KIT_UART_HW, &KIT_UART_config, &UART_context);
    Cy_SCB_UART_Enable(KIT_UART_HW);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("[Info] UART initialization complete\r\n\r\n");
    printf("CE227032 - PSoC 6 MCU SMIF On-The-Fly Encryption in MMIO and XIP Modes\r\n");
    printf("**********************************************************************\r\n\r\n");

    /* SMIF interrupt initialization status */
    cy_en_sysint_status_t intr_init_status;
    intr_init_status = Cy_SysInt_Init(&smifIntConfig, Isr_SMIF);
    CheckStatus("[Error] SMIF interrupt initialization failed\r\n\r\n", (uint32_t) intr_init_status);
    printf("[Info] SMIF interrupt initialization complete\r\n\r\n");

    /* Initialize SMIF */
    smifStatus = Cy_SMIF_Init(KIT_QSPI_HW, &KIT_QSPI_config, TIMEOUT_1_MS, &KIT_QSPI_context);
    CheckStatus("[Error] SMIF initialization failed\r\n\r\n", smifStatus);
    printf("[Info] SMIF initialization complete\r\n\r\n");

    /* Initialize F-RAM memory slot */
    smifStatus = Cy_SMIF_Memslot_Init(KIT_QSPI_HW, (cy_stc_smif_block_config_t *) &smifBlockConfig, &KIT_QSPI_context);
    CheckStatus("[Error] F-RAM memory slot initialization failed\r\n\r\n", smifStatus);
    printf("[Info] F-RAM memory slot initialization complete\r\n\r\n");
    printf("[Info] SMIF operation in SPI mode\r\n\r\n");

    /* Configure data select, refer the kit schematics to choose the right configuration */
    Cy_SMIF_SetDataSelect(KIT_QSPI_HW, CY_SMIF_SLAVE_SELECT_2, CY_SMIF_DATA_SEL0);
    Cy_SMIF_Enable(KIT_QSPI_HW, &KIT_QSPI_context);

    /* Load encryption key into SMIF registers */
    KIT_QSPI_HW->CRYPTO_KEY0 = Cy_SMIF_PackBytesArray(&cryptoKey[CY_SMIF_CRYPTO_FIRST_WORD], true);
    KIT_QSPI_HW->CRYPTO_KEY1 = Cy_SMIF_PackBytesArray(&cryptoKey[CY_SMIF_CRYPTO_SECOND_WORD], true);
    KIT_QSPI_HW->CRYPTO_KEY2 = Cy_SMIF_PackBytesArray(&cryptoKey[CY_SMIF_CRYPTO_THIRD_WORD], true);
    KIT_QSPI_HW->CRYPTO_KEY3 = Cy_SMIF_PackBytesArray(&cryptoKey[CY_SMIF_CRYPTO_FOURTH_WORD], true);

    /* Fill the txBuffer and encryptedtxBuffer with sample data to be transmitted */
    for(uint32_t index=0; index < PACKET_SIZE; index++)
	{
		txBuffer[index] = (uint8_t) (index);
		encryptedtxBuffer[index] = txBuffer[index];
	}

    /* Enable the SMIF interrupt */
    NVIC_EnableIRQ(smif_interrupt_IRQn);

    /* Enable global interrupts*/
    __enable_irq();

    /* Encrypt the sample data */
    printf("===================================================================\r\n\r\n");
    printf("[Info] Performing encryption in MMIO mode and writing data to F-RAM\r\n\r\n");
    PrintArray("[Info] Plain Data:\t", txBuffer, PACKET_SIZE);

    /* The AES-128 encryption of the address with the key,
     * and XOR of result with the plain data are performed by the function.
     *
     * The algorithm is the following:
     * encrypted data = XOR(AES128(address, key), plain data)
     *
     * Address here is the base address of the plain data array. */
    smifStatus = Cy_SMIF_Encrypt(KIT_QSPI_HW, CY15B104QSN_SlaveSlot_2.baseAddress, encryptedtxBuffer, PACKET_SIZE,  &KIT_QSPI_context);
    CheckStatus("[Error] Data encryption failed\r\n\r\n", smifStatus);
    printf("[Info] Data encryption complete\r\n\r\n");
    PrintArray("[Info] Encrypted Data:\t", encryptedtxBuffer, PACKET_SIZE);

    /* Write the sample bytes in encryptedtxBuffer to the external memory address specified */
    WriteMemory(KIT_QSPI_HW, (cy_stc_smif_mem_config_t*) smifMemConfigs[0], &KIT_QSPI_context, encryptedtxBuffer, PACKET_SIZE, extMemAddress);

    /* Read data from external memory into rxBuffer and decrypt it*/
    ReadMemory(KIT_QSPI_HW, (cy_stc_smif_mem_config_t*) smifMemConfigs[0], &KIT_QSPI_context, rxBuffer, PACKET_SIZE, extMemAddress);

    /* The AES-128 decryption of the address with the key,
     * and XOR of result with the encrypted data are performed by the function.
     *
     * The algorithm is the following:
     * plain data = XOR(AES128(address, key), encrypted data)
     *
     * Address here is the base address of the encrypted data array. */
    smifStatus = Cy_SMIF_Encrypt(KIT_QSPI_HW, CY15B104QSN_SlaveSlot_2.baseAddress, rxBuffer, PACKET_SIZE,  &KIT_QSPI_context);
    CheckStatus("[Error] Data decryption failed\r\n\r\n", smifStatus);
    printf("[Info] Data decryption complete\r\n\r\n");
    PrintArray("[Info] Decrypted Data:\t", rxBuffer, PACKET_SIZE);

    /* Reset error status */
    errorStatus = 0;

    /* Check if the transmitted and received arrays are equal */
    if (memcmp(txBuffer, rxBuffer, PACKET_SIZE) == 0)
    {
        printf("[Info] Received data matches with written data\r\n\r\n");
    }
    else
    {
        printf("[Error] Received data does not match with written data\r\n\r\n");
        errorStatus++;
    }

    printf("===================================================================\r\n\r\n");
    printf("[Info] Writing zeros to F-RAM\r\n\r\n");

    /* Fill the txBuffer with zeroes to be transmitted */
    for(uint32_t index=0; index < PACKET_SIZE; index++)
	{
		txBuffer[index] = (uint8_t) 0;
	}

    /* Write the sample bytes in txBuffer to the external memory address specified */
    WriteMemory(KIT_QSPI_HW, (cy_stc_smif_mem_config_t*) smifMemConfigs[0], &KIT_QSPI_context, txBuffer, PACKET_SIZE, extMemAddress);

    /* Read data from external memory into rxBuffer */
    ReadMemory(KIT_QSPI_HW, (cy_stc_smif_mem_config_t*) smifMemConfigs[0], &KIT_QSPI_context, rxBuffer, PACKET_SIZE, extMemAddress);

    /* Check if the transmitted and received arrays are equal */
    if (memcmp(txBuffer, rxBuffer, PACKET_SIZE) == 0)
    {
        printf("[Info] Received data matches with written data\r\n\r\n");
    }
    else
    {
        printf("[Error] Received data does not match with written data\r\n\r\n");
        errorStatus++;
    }

    printf("===================================================================\r\n\r\n");
    printf("[Info] Setting SMIF to XIP mode and writing data to F-RAM. Data is encrypted on-the-fly\r\n\r\n");


    /* Set XIP mode */
    Cy_SMIF_SetMode(KIT_QSPI_HW, CY_SMIF_MEMORY);

    /* Reinitialize the buffers and write directly to F-RAM using XIP mode*/
    for(uint32_t index=0; index < PACKET_SIZE; index++)
    {
    	txBuffer[index] = (uint8_t) (index & 0xFF);
    	*(rxBuffer_XIP + index) = txBuffer[index];
    }

    PrintArray("[Info] Plain Data:\t", txBuffer, PACKET_SIZE);
    printf("[Info] Reading on-the-fly encrypted data using MMIO mode\r\n\r\n");

    /* Set Normal mode */
    Cy_SMIF_SetMode(KIT_QSPI_HW, CY_SMIF_NORMAL);
    ReadMemory(KIT_QSPI_HW, (cy_stc_smif_mem_config_t*) smifMemConfigs[0], &KIT_QSPI_context, rxBuffer, PACKET_SIZE, extMemAddress);

    printf("[Info] Setting SMIF back to XIP mode and reading data from F-RAM. Data is decrypted on-the-fly\r\n\r\n");

    /* Set XIP mode */
    Cy_SMIF_SetMode(KIT_QSPI_HW, CY_SMIF_MEMORY);

    for(uint32_t index=0; index < PACKET_SIZE; index++)
    {
    	rxBuffer[index] = *(rxBuffer_XIP + index);
    }
    PrintArray("[Info] Decrypted Data:\t", rxBuffer, PACKET_SIZE);

    /* Check if the transmitted and received arrays are equal */
    if (memcmp(txBuffer, rxBuffer, PACKET_SIZE) == 0)
    {
        printf("[Info] Received data matches with written data\r\n\r\n");
    }
    else
    {
        printf("[Error] Received data does not match with written data\r\n\r\n");
        errorStatus++;

    }
    printf("===================================================================\r\n\r\n");

    /* Indicate status of all F-RAM operations using LED */
    while (1)
    {
        if (errorStatus == 0)
        {
            Cy_GPIO_Inv(KIT_LED2_PORT, KIT_LED2_PIN); /* Toggle the LED */
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

