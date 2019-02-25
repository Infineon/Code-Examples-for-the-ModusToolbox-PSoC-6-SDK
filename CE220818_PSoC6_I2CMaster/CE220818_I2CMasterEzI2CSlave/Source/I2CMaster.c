/******************************************************************************
* File Name: I2CMaster.c
*
* Version: 1.0
*
* Description: This file contains all the functions and variables required for proper
* 			   operation of I2C Master SCB
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
#include "I2CMaster.h"
#include "Interface.h"
/***************************************
*            Constants
****************************************/
/* I2C slave address to communicate with */
#define I2C_SLAVE_ADDR  (0x24)

/* Buffer and packet size */
#define READ_PACKET_SIZE  (0x08UL)
#define PACKET_SIZE       (3UL)
#define RX_PACKET_SIZE    (3UL)
#define BUFFER_SIZE       (PACKET_SIZE)

/* Command valid status */
#define STS_CMD_DONE    (0x00UL)
#define STS_CMD_FAIL    (0xFFUL)

/* Command valid status */
#define TRANSFER_ERROR    (0xFFUL)
#define READ_ERROR        (TRANSFER_ERROR)

/* Timeout */
#define LOOP_FOREVER        (0UL)

/* Packet positions */
#define EZI2C_RPLY_SOP_POS  (5UL)
#define EZI2C_RPLY_STS_POS  (6UL)
#define EZI2C_RPLY_EOP_POS  (7UL)

/* Combine master error statuses in single mask  */
#define MASTER_ERROR_MASK  (CY_SCB_I2C_MASTER_DATA_NAK | CY_SCB_I2C_MASTER_ADDR_NAK    | \
                            CY_SCB_I2C_MASTER_ARB_LOST | CY_SCB_I2C_MASTER_ABORT_START | \
                            CY_SCB_I2C_MASTER_BUS_ERR)

/*******************************************************************************
* Global variables
*******************************************************************************/
/* Structure for master transfer configuration */
cy_stc_scb_i2c_master_xfer_config_t masterTransferCfg =
{
    .slaveAddress = I2C_SLAVE_ADDR,
    .buffer       = NULL,
    .bufferSize   = 0U,
    .xferPending  = false
};
cy_stc_scb_i2c_context_t mI2C_context;

/***************************************
*         Forward Declaration
****************************************/
void mI2C_Interrupt(void);
/*******************************************************************************
* Function Name: mI2C_Interrupt
****************************************************************************//**
*
* Invokes the Cy_SCB_I2C_Interrupt() PDL driver function.
*
*******************************************************************************/
void mI2C_Interrupt(void)
{
    Cy_SCB_I2C_Interrupt(mI2C_HW, &mI2C_context);
}

/*******************************************************************************
* Function Name: WritePacketToEzI2C
****************************************************************************//**
*
* Buffer is assigned with data to be sent to slave.
* high level PDL library function is used to control I2C SCB to send data to EzI2C slave.
* Errors are handled depend on the return value from the appropriate function.
*
* \param writebuffer
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
uint8_t WritePacketToEzI2C(uint8_t* writebuffer, uint32_t bufferSize)
{
    uint8_t status = TRANSFER_ERROR;
    cy_en_scb_i2c_status_t  errorStatus;
    uint32_t masterStatus;
    /* Timeout 1 sec (one unit is us) */
    uint32_t timeout = 1000000UL;

    /* Setup transfer specific parameters */
    masterTransferCfg.buffer     = writebuffer;
    masterTransferCfg.bufferSize = bufferSize;

    /* Initiate write transaction */
    errorStatus = Cy_SCB_I2C_MasterWrite(mI2C_HW, &masterTransferCfg, &mI2C_context);
    if(errorStatus == CY_SCB_I2C_SUCCESS)
    {
        /* Wait until master complete read transfer or time out has occured */
        do
        {
            masterStatus  = Cy_SCB_I2C_MasterGetStatus(mI2C_HW, &mI2C_context);
            Cy_SysLib_DelayUs(CY_SCB_WAIT_1_UNIT);
            timeout--;

        } while ((0UL != (masterStatus & CY_SCB_I2C_MASTER_BUSY)) && (timeout > 0));

        if (timeout <= 0)
        {
            /* Timeout recovery */
            Cy_SCB_I2C_Disable(mI2C_HW, &mI2C_context);
            Cy_SCB_I2C_Enable (mI2C_HW);
        }
        else
        {
            if ((0u == (MASTER_ERROR_MASK & masterStatus)) &&
                (WRITE_PACKET_SIZE == Cy_SCB_I2C_MasterGetTransferCount(mI2C_HW, &mI2C_context)))
            {
                status = TRANSFER_CMPLT;
            }
        }
    }

    return (status);
}

/*******************************************************************************
* Function Name: ReadStatusPacketFromEzI2C
****************************************************************************//**
*
*  Master initiates the read from EzI2C buffer.
*  The status of the transfer is returned by comparing the data in EzI2C buffer.
*
* \return
*  Returns status of the transfer by checking packets read.
* ref uint32_t
*
* \note
* * If the status packet read is correct function returns TRANSFER_CMPLT anf
*   if status packet is incorrect function returns TRANSFER_ERROR.
*
*******************************************************************************/
uint8_t ReadStatusPacketFromEzI2C(void)
{
    uint8_t status = TRANSFER_ERROR;
    cy_en_scb_i2c_status_t errorStatus;
    uint32_t masterStatus;
    /* Timeout 1 sec (one unit is us) */
    uint32_t timeout = 1000000UL;
    uint8_t buffer[READ_PACKET_SIZE];

    /* Setup transfer specific parameters */
    masterTransferCfg.buffer     = buffer;
    masterTransferCfg.bufferSize = READ_PACKET_SIZE;

    /* Initiate read transaction */
    errorStatus = Cy_SCB_I2C_MasterRead(mI2C_HW, &masterTransferCfg, &mI2C_context);
    if(errorStatus == CY_SCB_I2C_SUCCESS)
    {
        /* Wait until master complete read transfer or time out has occurred */
        do
        {
            masterStatus  = Cy_SCB_I2C_MasterGetStatus(mI2C_HW, &mI2C_context);
            Cy_SysLib_DelayUs(CY_SCB_WAIT_1_UNIT);
            timeout--;

        } while ((0UL != (masterStatus & CY_SCB_I2C_MASTER_BUSY)) && (timeout > 0));

        if (timeout <= 0)
        {
            /* Timeout recovery */
            Cy_SCB_I2C_Disable(mI2C_HW, &mI2C_context);
            Cy_SCB_I2C_Enable (mI2C_HW);
        }
        else
        {
            /* Check transfer status */
            if (0u == (MASTER_ERROR_MASK & masterStatus))
            {
                /* Check packet structure and status */
                if((PACKET_SOP   == buffer[EZI2C_RPLY_SOP_POS]) &&
                    (PACKET_EOP   == buffer[EZI2C_RPLY_EOP_POS]) &&
                    (STS_CMD_DONE == buffer[EZI2C_RPLY_STS_POS]) )
                {
                    status = TRANSFER_CMPLT;
                }
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
*
* \param None
*
* \return
* Status of initialization
*
*******************************************************************************/
uint32_t initMaster(void)
{
    cy_en_scb_i2c_status_t initStatus;
    cy_en_sysint_status_t sysStatus;
    cy_stc_sysint_t mI2C_SCB_IRQ_cfg =
    {
    		/*.intrSrc =*/ mI2C_IRQ,
    		/*.intrPriority =*/ 7u
    };

    /*Initialize and enable the I2C in master mode*/
    initStatus = Cy_SCB_I2C_Init(mI2C_HW, &mI2C_config, &mI2C_context);
    if(initStatus != CY_SCB_I2C_SUCCESS)
    {
    	return I2C_FAILURE;
    }

    /* Hook interrupt service routine */
    sysStatus = Cy_SysInt_Init(&mI2C_SCB_IRQ_cfg, &mI2C_Interrupt);
    if(sysStatus != CY_SYSINT_SUCCESS)
    {
    	return I2C_FAILURE;
    }
    NVIC_EnableIRQ((IRQn_Type) mI2C_SCB_IRQ_cfg.intrSrc);
    Cy_SCB_I2C_Enable(mI2C_HW);
    return I2C_SUCCESS;
}
