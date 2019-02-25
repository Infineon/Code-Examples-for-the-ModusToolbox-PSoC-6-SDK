/******************************************************************************
* File Name: SPIMaster.c
*
* Version: 1.0
*
* Description: 	This file contains function definitions for SPI Master.
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
#include "SPIMaster.h"
#include "Interface.h"

/***************************************
*                Macros
****************************************/
#define MASTER_ERROR_MASK  (CY_SCB_SPI_SLAVE_TRANSFER_ERR  | CY_SCB_SPI_TRANSFER_OVERFLOW    | \
                            CY_SCB_SPI_TRANSFER_UNDERFLOW)

/***************************************
*            Global Variables
****************************************/
cy_stc_scb_spi_context_t mSPI_context;

/***************************************
*          Forward Declaration
****************************************/
uint32 checkTranferStatus(uint32 transferSize);
void mSPI_Interrupt(void);
uint32 initMaster(void);
uint32 sendPacket(uint32 *txBuffer, uint32 transferSize);
/*******************************************************************************
* Function Name: mSPI_Interrupt
****************************************************************************//**
*
* Invokes the Cy_SCB_SPI_Interrupt() PDL driver function.
*
*******************************************************************************/
void mSPI_Interrupt(void)
{
    Cy_SCB_SPI_Interrupt(mSPI_HW, &mSPI_context);
}

/******************************************************************************
* Function Name: initMaster
*******************************************************************************
*
* Summary: 		This function initializes the SPI Master based on the
* 				configuration done in design.modus file.
*
* Parameters: 	None
*
* Return:		(uint32) INIT_SUCCESS or INIT_FAILURE
*
******************************************************************************/
uint32 initMaster(void)
{
    cy_en_scb_spi_status_t initStatus;
    cy_en_sysint_status_t sysSpistatus;

    const cy_stc_sysint_t mSPI_SCB_IRQ_cfg =
    {
        .intrSrc      = mSPI_IRQ,
        .intrPriority = 7u
    };

	/* Configure SPI block */
	initStatus = Cy_SCB_SPI_Init(mSPI_HW, &mSPI_config, &mSPI_context);

	/* If the initialization fails, return failure status */
	if(initStatus != CY_SCB_SPI_SUCCESS)
	{
		return(INIT_FAILURE);
	}

	/* Set active slave select to line 0 */
	Cy_SCB_SPI_SetActiveSlaveSelect(mSPI_HW, CY_SCB_SPI_SLAVE_SELECT1);

	/* Enable SPI master block. */
	Cy_SCB_SPI_Enable(mSPI_HW);

	/* Hook interrupt service routine */
	sysSpistatus = Cy_SysInt_Init(&mSPI_SCB_IRQ_cfg, &mSPI_Interrupt);
	if(sysSpistatus != CY_SYSINT_SUCCESS)
	{
		return(INIT_FAILURE);
	}
	/* Enable interrupt in NVIC */
	NVIC_EnableIRQ((IRQn_Type) mSPI_SCB_IRQ_cfg.intrSrc);

	/* Initialization completed */

	return(INIT_SUCCESS);
}


/******************************************************************************
* Function Name: sendPacket
*******************************************************************************
*
* Summary: 		This function sends the data to the slave.
*
* Parameters:  	(uint32 *) txBuffer - Pointer to the transmit buffer
* 				(uint32)   transferSize - Number of bytes to be transmitted
*
* Return:		\ref cy_en_scb_spi_status_t
*
******************************************************************************/
uint32 sendPacket(uint32 *txBuffer, uint32 transferSize)
{
    cy_en_scb_spi_status_t errorStatus;

    /* Initiate SPI Master write and read transaction. */
    errorStatus = Cy_SCB_SPI_Transfer(mSPI_HW, txBuffer, NULL, transferSize, &mSPI_context);

    return (errorStatus);
}

/******************************************************************************
* Function Name: checkTranferStatus
*******************************************************************************
*
* Summary: 		This function checks for master transfer completion status
*
* Parameters:  	(uint32) transferSize - Size of command packet
*
* Return:		Status of transfer completion
*
******************************************************************************/
uint32 checkTranferStatus(uint32 transferSize)
{
    uint32 masterStatus = TRANSFER_FAILURE;

	/* Wait until master complete the transfer */
	do
	{
		masterStatus  = Cy_SCB_SPI_GetTransferStatus(mSPI_HW, &mSPI_context);

	} while (0UL != (masterStatus & CY_SCB_SPI_TRANSFER_ACTIVE));

	/* Check for any errors */
	if ((0UL == (MASTER_ERROR_MASK & masterStatus)) &&
	(transferSize == Cy_SCB_SPI_GetNumTransfered(mSPI_HW, &mSPI_context)))
	{
		/* No error */
		masterStatus = TRANSFER_COMPLETE;
	}
	else
	{
		/* Error encountered in the transfer */
		masterStatus = TRANSFER_FAILURE;
	}

    return (masterStatus);
}

