/******************************************************************************
* File Name: I2CMaster.c
*
* Version: 1.0
*
* Description: This file contains all the functions and variables required for proper
* 			   operation of I2C Master SCB
*
*******************************************************************************
* Copyright (2018-19), Cypress Semiconductor Corporation. All rights reserved.
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
#include "I2CMaster.h"
#include "Interface.h"

/***************************************
*            Constants
****************************************/
/* I2C slave address to communicate with */
#define I2C_SLAVE_ADDR        (0x24UL)

/* Buffer and packet size */
#define RX_PACKET_SIZE  (3UL)

/* Buffer and packet size */
#define RX_PACKET_SIZE        (3UL)
#define BUFFER_SIZE           (PACKET_SIZE)

/* Command valid status */
#define STS_CMD_DONE          (0x00UL)
#define STS_CMD_FAIL          (0xFFUL)

/* Command valid status */
#define TRANSFER_ERROR        (0xFFUL)
#define READ_ERROR            (TRANSFER_ERROR)

/* Timeout */
#define LOOP_FOREVER          (0UL)

/* Packet positions */
#define RX_PACKET_SOP_POS     (0UL)
#define RX_PACKET_STS_POS     (1UL)
#define RX_PACKET_EOP_POS     (2UL)

#define I2C_TIMEOUT           (100UL)

/*******************************************************************************
* Global variables
*******************************************************************************/
cy_stc_scb_i2c_context_t mI2C_context;

/*******************************************************************************
* Function Name: WritePacket
****************************************************************************//**
*
* Buffer is assigned with data to be sent to slave.
* Low level PDL APIs are used to control I2C SCB to send data.
* Errors are handled depending on the return value from the appropriate function.
*
* \param buffer
*
* \param bufferSize
*
* \return
* returns the status after command is written to slave.
* TRANSFER_ERROR is returned if any error occurs.
* TRANSFER_CMPLT is returned if write is successful.
* \ref uint32_t
*
*******************************************************************************/
uint8_t WritePacket(uint8_t* buffer, uint32_t bufferSize)
{
    uint8_t status = TRANSFER_ERROR;
    cy_en_scb_i2c_status_t  errorStatus;

    /* Sends packets to slave using low level PDL library functions. */
    errorStatus = Cy_SCB_I2C_MasterSendStart(mI2C_HW, I2C_SLAVE_ADDR, CY_SCB_I2C_WRITE_XFER, I2C_TIMEOUT, &mI2C_context);
    if(errorStatus == CY_SCB_I2C_SUCCESS)
    {

        uint32_t cnt = 0UL;

        /* Read data from the slave into the buffer */
        do
        {
            /* Write byte and receive ACK/NACK response */
            errorStatus = Cy_SCB_I2C_MasterWriteByte(mI2C_HW, buffer[cnt], I2C_TIMEOUT, &mI2C_context);
            ++cnt;
        }
        while((errorStatus == CY_SCB_I2C_SUCCESS) && (cnt < bufferSize));
    }

    /* Check status of transaction */
    if ((errorStatus == CY_SCB_I2C_SUCCESS)           ||
        (errorStatus == CY_SCB_I2C_MASTER_MANUAL_NAK) ||
        (errorStatus == CY_SCB_I2C_MASTER_MANUAL_ADDR_NAK))
    {
        /* Send Stop condition on the bus */
        if (Cy_SCB_I2C_MasterSendStop(mI2C_HW, I2C_TIMEOUT, &mI2C_context) == CY_SCB_I2C_SUCCESS)
        {
            status = TRANSFER_CMPLT;
        }
    }

    return (status);
}
/*******************************************************************************
* Function Name: ReadStatusPacket
****************************************************************************//**
*
*  Master initiates to read status packet from the slave.
*  The status of the transfer is returned.
*
* \return
* Checks the status packet and returns the status.
* ref uint32_t
*
* \note
*   If the status packet read is correct function returns TRANSFER_CMPLT and
*   if status packet is incorrect function returns TRANSFER_ERROR.
*
*******************************************************************************/
uint8_t ReadStatusPacket(void)
{
    /* Buffer to copy RX messages. */
    uint8_t rxBuffer[RX_PACKET_SIZE];
    cy_en_scb_i2c_status_t  errorStatus;
    uint32_t status = TRANSFER_ERROR;

    /* Using low level function initiating master to read data. */
    errorStatus = Cy_SCB_I2C_MasterSendStart(mI2C_HW, I2C_SLAVE_ADDR, CY_SCB_I2C_READ_XFER, I2C_TIMEOUT, &mI2C_context);
    if(errorStatus == CY_SCB_I2C_SUCCESS)
    {
         uint32_t cnt = 0UL;
        cy_en_scb_i2c_command_t cmd = CY_SCB_I2C_ACK;

        /* Read data from the slave into the buffer */
        do
        {
            if (cnt == (RX_PACKET_SIZE - 1UL))
            {
                /* The last byte must be NACKed */
                cmd = CY_SCB_I2C_NAK;
            }

            /* Read byte and generate ACK / or prepare for NACK */
            errorStatus = Cy_SCB_I2C_MasterReadByte(mI2C_HW, cmd, rxBuffer + cnt, I2C_TIMEOUT, &mI2C_context);
            ++cnt;
        }
        while ((errorStatus == CY_SCB_I2C_SUCCESS) && (cnt < RX_PACKET_SIZE));
    }

    /* Check status of transaction */
    if ((errorStatus == CY_SCB_I2C_SUCCESS)           ||
        (errorStatus == CY_SCB_I2C_MASTER_MANUAL_NAK) ||
        (errorStatus == CY_SCB_I2C_MASTER_MANUAL_ADDR_NAK))
    {
        /* Send Stop condition on the bus */
        if (Cy_SCB_I2C_MasterSendStop(mI2C_HW, I2C_TIMEOUT, &mI2C_context) == CY_SCB_I2C_SUCCESS)
        {
            /* Check packet structure */
            if ((PACKET_SOP   == rxBuffer[RX_PACKET_SOP_POS]) &&
                (PACKET_EOP   == rxBuffer[RX_PACKET_EOP_POS]) &&
                (STS_CMD_DONE == rxBuffer[RX_PACKET_STS_POS]))
            {
                status = TRANSFER_CMPLT;
            }
        }
    }

    return (status);
}
/*******************************************************************************
* Function Name: initMaster
********************************************************************************
*
* This function initiates and enables master SCB
*
* \param None
*
* \return
* Status of initialization
*
*******************************************************************************/
uint32_t initMaster(void)
{
	/* Initialize the master I2C. */

    cy_en_scb_i2c_status_t initStatus;

	/* Configure component. */
	initStatus = Cy_SCB_I2C_Init(mI2C_HW, &mI2C_config, &mI2C_context);
	if(initStatus!=CY_SCB_I2C_SUCCESS)
	{
		return I2C_FAILURE;
	}

	/* Enable I2C master hardware. */
	Cy_SCB_I2C_Enable(mI2C_HW);
	return I2C_SUCCESS;
}
