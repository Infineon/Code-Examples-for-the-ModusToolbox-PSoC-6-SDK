/******************************************************************************
* File Name: smif_fram.c
*
* Version: 1.0
*
* Description: This file contains the helper functions for writing and reading
*              F-RAM data.
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

/***************************************************************************
* Global variables
***************************************************************************/
extern cy_stc_smif_context_t KIT_QSPI_context;

/***************************************************************************
* Private Function Prototypes
***************************************************************************/
void handle_error(void);

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
*
* This function processes unrecoverable errors
*
*******************************************************************************/
void handle_error(void)
{
     /* Disable all interrupts */
    __disable_irq();

    /* On failure, turn the LED ON */
    Cy_GPIO_Clr(KIT_LED2_PORT, KIT_LED2_PIN);
    while(true) {}
}

/*******************************************************************************
* Function Name: PrintArray
****************************************************************************//**
*
* This function prints the content of the RX buffer to the UART console.
*
* \param msg - message print before array output
*
* \param rxBuffer - The buffer to the console output.
*
* \param size - The size of the buffer to the console output.
*
*******************************************************************************/
void PrintArray(char * msg, uint8_t * buff, uint32_t size)
{
    printf("%s", msg);
    for(uint32_t index=0; index<size; index++)
    {
        printf("0x%02X ", (unsigned int) buff[index]);
    }
    printf("\r\n\r\n");
}


/*******************************************************************************
* Function Name: CheckStatus
****************************************************************************//**
*
* Check if status is SUCCESS and call handle error function if otherwise
*
* \param msg - message print in case of error
*
* \param status - status to check
*
*******************************************************************************/
void CheckStatus(char * msg, uint32_t status)
{
    if(0u != status)
    {
        printf("%s", msg);
        handle_error();
    }
}

/*******************************************************************************
* Function Name: ReadMemory
****************************************************************************//**
*
* This function reads data from the external F-RAM in SPI mode.
* The function sends the I/O Read: 0x03 command to the external F-RAM.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param memConfig
* The configuration data for external F-RAM.
*
* \param smifContext
* The internal SMIF context data.
*
* \param rxBuffer
* The buffer for read data.
*
* \param rxSize
* The size of data to read.
*
* \param address
* The address to read data from.
*
*******************************************************************************/
void ReadMemory(SMIF_Type *baseaddr, cy_stc_smif_mem_config_t *memConfig,
                            cy_stc_smif_context_t *smifContext,
                            uint8_t rxBuffer[],
                            uint32_t rxSize,
                            uint8_t *address)
{
    cy_en_smif_status_t status;

    /* Send the read command */
    status = Cy_SMIF_Memslot_CmdRead(baseaddr, memConfig, address, rxBuffer, rxSize, NULL, &KIT_QSPI_context);
    CheckStatus("[Error] SMIF Cy_SMIF_Memslot_CmdRead failed\r\n\r\n",status);

    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the read operation is completed. */
    }

    /* Send received data to the console */
    PrintArray("[Info] Received Data:\t",rxBuffer, rxSize);
}

/*******************************************************************************
* Function Name: WriteMemory
********************************************************************************
*
* This function writes data to the external memory in the quad mode.
* The function sends the Quad Page Program: 0x38 command to the external memory.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param memConfig
* The configuration data for external F-RAM.
*
* \param smifContext
* The internal SMIF context data.
*
* \param txBuffer
* Data to write in the external memory.
*
* \param txSize
* The size of data.
*
* \param address
* The address to write data to.
*
*******************************************************************************/
void WriteMemory(SMIF_Type *baseaddr, cy_stc_smif_mem_config_t *memConfig,
                    cy_stc_smif_context_t *smifContext,
                    uint8_t txBuffer[],
                    uint32_t txSize,
                    uint8_t *address)
{
    cy_en_smif_status_t status;
    uint8_t rxBuffer_reg;
    cy_stc_smif_mem_device_cfg_t *device = memConfig->deviceCfg;
    cy_stc_smif_mem_cmd_t *cmdreadStsRegWip = device->readStsRegWipCmd;

    /* Send Write Enable to external F-RAM */
    status = Cy_SMIF_Memslot_CmdWriteEnable(baseaddr, memConfig, &KIT_QSPI_context);
    CheckStatus("[Error] SMIF Cy_SMIF_Memslot_CmdWriteEnable failed\r\n\r\n", status);

    /* Send Write command with data*/
    status = Cy_SMIF_Memslot_CmdProgram(KIT_QSPI_HW, memConfig, address, txBuffer, txSize, NULL, &KIT_QSPI_context);
    CheckStatus("Error] SMIF Cy_SMIF_Memslot_CmdProgram failed\r\n\r\n", status);

    PrintArray("[Info] Written Data:\t", txBuffer, txSize);

    /* Read data from the external F-RAM status register */
    status = Cy_SMIF_Memslot_CmdReadSts(baseaddr, memConfig, &rxBuffer_reg,
                             (uint8_t)cmdreadStsRegWip->command , smifContext);
    CheckStatus("[Error] SMIF ReadStatusReg failed\r\n\r\n", status);

    printf("[Info] Received Status:\t0x%X\r\n\r\n", (unsigned int) rxBuffer_reg);
}

