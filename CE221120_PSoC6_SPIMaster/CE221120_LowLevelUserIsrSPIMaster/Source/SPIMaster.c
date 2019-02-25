/******************************************************************************
* File Name: SPIMaster.c
*
* Version: 1.0
*
* Description: This file contains function definitions for SPI Master.
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

/* SPI Interrupt configuration structure */
const cy_stc_sysint_t mSPI_SCB_IRQ_cfg =
{
    .intrSrc      = mSPI_IRQ,
    .intrPriority = 7u
};

/***************************************************************************//**
* Function Name: isr_SPI
********************************************************************************
*
* Summary:
*  This function is registered to be called when SPI interrupt occurs
*  (Note that only SPI Master Done interrupt is enabled).
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
void isr_SPI(void)
{
    uint32_t masterStatus;

    /* Check the status of master */
    masterStatus  = Cy_SCB_SPI_GetSlaveMasterStatus(mSPI_HW);

    if(masterStatus == CY_SCB_SPI_MASTER_DONE)
    {
        /*Check number of elements in RX FIFO */
        if(TRANSFER_SIZE == Cy_SCB_SPI_GetNumInRxFifo(mSPI_HW))
        {
			masterTranStatus = TRANSFER_COMPLETE;
		}
        else
        {
        	masterTranStatus = TRANSFER_FAILURE;
        }
    }

    /*Clear status and RX FIFO */
    Cy_SCB_SPI_ClearSlaveMasterStatus(mSPI_HW, masterStatus);
    Cy_SCB_SPI_ClearRxFifo(mSPI_HW);
}

/******************************************************************************
* Function Name: initMaster
*******************************************************************************
*
* Summary: 		This function initializes the SPI Master based on the
* 				configuration done in design.modus file. SPI Master interrupt
* 				is enabled.
*
* Parameters: 	None
*
* Return:		(uint32) INIT_SUCCESS or INIT_FAILURE
*
******************************************************************************/
uint32 initMaster(void)
{
	cy_stc_scb_spi_context_t mSPI_context;
    cy_en_scb_spi_status_t initStatus;

    /* Configure SPI block */
	initStatus = Cy_SCB_SPI_Init(mSPI_HW, &mSPI_config, &mSPI_context);

	/* If the initialization fails, return failure status */
	if(initStatus != CY_SCB_SPI_SUCCESS)
	{
		return(INIT_FAILURE);
	}

	/* Set active slave select to line 0 */
	Cy_SCB_SPI_SetActiveSlaveSelect(mSPI_HW, CY_SCB_SPI_SLAVE_SELECT1);

	/* Unmask only the SPI_DONE interrupt bit */
	mSPI_HW->INTR_M_MASK = SCB_INTR_M_SPI_DONE_Msk;

	/* Configure User ISR */
	Cy_SysInt_Init(&mSPI_SCB_IRQ_cfg, &isr_SPI);

	/* Enable the interrupt */
	NVIC_EnableIRQ(mSPI_SCB_IRQ_cfg.intrSrc);

	/* Enable SPI master block. */
	Cy_SCB_SPI_Enable(mSPI_HW);

	/* Initialization completed */

	return(INIT_SUCCESS);
}

/******************************************************************************
* Function Name: sendPacket
*******************************************************************************
*
* Summary: 		This function sends the data to the slave. Note that the below
* 				function is blocking until all the bytes are put in the transmit
* 				FIFO.
*
* Parameters:  	(uint32 *) txBuffer - Pointer to the transmit buffer
* 				(uint32)	 transferSize - Number of bytes to be transmitted
*
* Return:		None
*
******************************************************************************/
void sendPacket(uint32 *txBuffer, uint32 transferSize)
{
	/* Send the data. The below function waits until all the bytes are placed in
	 * the transmit FIFO */
	Cy_SCB_SPI_WriteArrayBlocking(mSPI_HW, txBuffer, transferSize);

    masterTranStatus = TRANSFER_IN_PROGRESS;
}






