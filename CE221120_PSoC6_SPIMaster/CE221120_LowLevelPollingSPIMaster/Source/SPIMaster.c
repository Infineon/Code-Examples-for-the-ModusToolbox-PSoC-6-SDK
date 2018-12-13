/******************************************************************************
* File Name: SPIMaster.c
*
* Version: 1.0
*
* Description: 	This file contains function definitions for SPI Master.
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
#include "SPIMaster.h"
#include "Interface.h"

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

	/* Configure SPI block */
	initStatus = Cy_SCB_SPI_Init(mSPI_HW, &mSPI_config, NULL);

	/* If the initialization fails, return failure status */
	if(initStatus != CY_SCB_SPI_SUCCESS)
	{
		return(INIT_FAILURE);
	}

	/* Set active slave select to line 0 */
	Cy_SCB_SPI_SetActiveSlaveSelect(mSPI_HW, CY_SCB_SPI_SLAVE_SELECT1);

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
* 				function is blocking until all the bytes are transferred.
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

}

/******************************************************************************
* Function Name: checkTranferStatus
*******************************************************************************
*
* Summary: 		This function checks for master transfer completion status
*
* Parameters:  	None
*
* Return:		Status of transfer completion
*
******************************************************************************/
uint32 checkTranferStatus(void)
{
	volatile uint32 masterStatus;
	uint32 transferStatus;
    /* Wait until master complete the transfer */
	do
	{
		masterStatus  = Cy_SCB_SPI_GetSlaveMasterStatus(mSPI_HW);

	} while (0UL == (masterStatus & (CY_SCB_SPI_MASTER_DONE | CY_SCB_SPI_SLAVE_ERR)));

	/* Check for any errors */
	if(CY_SCB_SPI_MASTER_DONE & masterStatus)
	{
		/* No error */
		transferStatus = TRANSFER_COMPLETE;
	}
	else
	{
		/* Error encountered in the transfer */
		transferStatus = TRANSFER_FAILURE;
	}

	/* Clear the SPI master status */
	Cy_SCB_SPI_ClearSlaveMasterStatus(mSPI_HW, masterStatus);

	return(transferStatus);
}
