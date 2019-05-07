/******************************************************************************
* File Name: smif_mem.c
*
* Version: 2.0
*
* Description:
* 	This file contains functions to perform memory operations such as read,
* 	write, and erase on an external memory device.
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

#include "smif_mem.h"
#include "cycfg.h"
#include "cycfg_qspi_memslot.h"


/***************************************************************************
* Global variables
***************************************************************************/
/* All the functions in this module use the timeout parameter of the following
 * context variable for timing out on memory operations.
 */
extern cy_stc_smif_context_t KIT_QSPI_context;


/*******************************************************************************
* Function Name: PollTransferStatus
****************************************************************************//**
*
* Polls the SMIF block until the transfer status equals the expected value or
* timeout occurs.
*
* \param transferStatus
* Transfer status value to be checked.
*
* \return Status of the operation.
* CY_SMIF_SUCCESS 	     - SMIF block has completed the transfer
* CY_SMIF_EXCEED_TIMEOUT - Timeout occurred.
*
*******************************************************************************/
static cy_en_smif_status_t PollTransferStatus(cy_en_smif_txfr_status_t transferStatus)
{
	cy_en_smif_status_t status = CY_SMIF_SUCCESS;
	uint32_t timeout = SMIF_TRANSFER_TIMEOUT;

	while (((uint32_t)transferStatus != Cy_SMIF_GetTxfrStatus(KIT_QSPI_HW, &KIT_QSPI_context)) && ( timeout > 0u))
	{
		Cy_SysLib_DelayUs(CY_SMIF_WAIT_1_UNIT);
		timeout--;
	}

	if((uint32_t)transferStatus != Cy_SMIF_GetTxfrStatus(KIT_QSPI_HW, &KIT_QSPI_context))
	{
		status = CY_SMIF_EXCEED_TIMEOUT;
	}

	return status;
}


/*******************************************************************************
* Function Name: IsMemoryReady
****************************************************************************//**
*
* Polls the memory device to check whether it is ready to accept new commands or
* not until either it is ready or the retries have exceeded the limit.
*
* \param memConfig
* memory device configuration
*
* \return Status of the operation.
* CY_SMIF_SUCCESS 	     - Memory is ready to accept new commands.
* CY_SMIF_EXCEED_TIMEOUT - Memory is busy.
*
*******************************************************************************/
cy_en_smif_status_t IsMemoryReady(cy_stc_smif_mem_config_t const *memConfig)
{
	uint32_t retries = 0;
	bool isBusy;

	do
	{
		isBusy = Cy_SMIF_Memslot_IsBusy(KIT_QSPI_HW, (cy_stc_smif_mem_config_t* )memConfig, &KIT_QSPI_context);
		Cy_SysLib_Delay(CY_SMIF_WAIT_1_UNIT);
		retries++;
	}while(isBusy && (retries < MEMORY_BUSY_CHECK_RETRIES));

	return (isBusy ? CY_SMIF_EXCEED_TIMEOUT : CY_SMIF_SUCCESS);
}


/*******************************************************************************
* Function Name: IsQuadEnabled
****************************************************************************//**
*
* Checks whether QE (Quad Enable) bit is set or not in the configuration
* register of the memory.
*
* \param memConfig
* Memory device configuration
*
* \param isQuadEnabled
* This parameter is updated to indicate whether Quad mode is enabled (true) or
* not (false). The value is valid only when the function returns
* CY_SMIF_SUCCESS.
*
* \return Status of the operation. See cy_en_smif_status_t.
*
*******************************************************************************/
cy_en_smif_status_t IsQuadEnabled(cy_stc_smif_mem_config_t const *memConfig, bool *isQuadEnabled)
{
	cy_en_smif_status_t status;
	uint8_t readStatus = 0;
	uint32_t statusCmd = memConfig->deviceCfg->readStsRegQeCmd->command;
	uint8_t maskQE = (uint8_t) memConfig->deviceCfg->stsRegQuadEnableMask;

	status = Cy_SMIF_Memslot_CmdReadSts(KIT_QSPI_HW, memConfig, &readStatus, statusCmd, &KIT_QSPI_context);

	*isQuadEnabled = false;
	if(CY_SMIF_SUCCESS == status)
	{
		/* Check whether Quad mode is already enabled or not */
		*isQuadEnabled = (maskQE == (readStatus & maskQE));
	}

	return status;
}


/*******************************************************************************
* Function Name: EnableQuadMode
****************************************************************************//**
*
* This function sets the QE (QUAD Enable) bit in the external memory
* configuration register to enable Quad SPI mode.
*
* \param memConfig
* Memory device configuration
*
* \return Status of the operation. See cy_en_smif_status_t.
*
*******************************************************************************/
cy_en_smif_status_t EnableQuadMode(cy_stc_smif_mem_config_t const *memConfig)
{
	cy_en_smif_status_t status;

	/* Send Write Enable to external memory */
	status = Cy_SMIF_Memslot_CmdWriteEnable(KIT_QSPI_HW, smifMemConfigs[0], &KIT_QSPI_context);

	if(CY_SMIF_SUCCESS == status)
	{
		status = Cy_SMIF_Memslot_QuadEnable(KIT_QSPI_HW, (cy_stc_smif_mem_config_t* )memConfig, &KIT_QSPI_context);

		if(CY_SMIF_SUCCESS == status)
		{
			/* Poll memory for the completion of operation */
			status = IsMemoryReady(memConfig);
		}
	}

	return status;
}


/*******************************************************************************
* Function Name: ReadMemory
****************************************************************************//**
*
* This function reads data from the external memory and blocks until the read
* transfer is complete or timeout occurs.
*
* \param memConfig
* Memory device configuration
*
* \param address
* The address to read data from.
*
* \param rxBuffer
* The buffer for storing the read data.
*
* \param rxSize
* The size of data to read.
*
* \return Status of the operation. See cy_en_smif_status_t.
*
*******************************************************************************/
cy_en_smif_status_t ReadMemory(cy_stc_smif_mem_config_t const *memConfig, uint8_t address[], uint8_t rxBuffer[], uint32_t rxSize)
{
    cy_en_smif_status_t status = Cy_SMIF_Memslot_CmdRead(KIT_QSPI_HW, memConfig, address, rxBuffer, rxSize, NULL, &KIT_QSPI_context);

    if(CY_SMIF_SUCCESS == status)
	{
    	/* Wait until the SMIF block completes receiving data */
    	status = PollTransferStatus(CY_SMIF_REC_CMPLT);
	}

    return status;
}


/*******************************************************************************
* Function Name: WriteMemory
********************************************************************************
*
* This function writes data to the external memory.
*
* \param memConfig
* Memory device configuration
*
* \param address
* The address to write data at.
*
* \param txBuffer
* Buffer holding the data to write in the external memory.
*
* \param txSize
* The size of data to write.
*
* \return Status of the operation. See cy_en_smif_status_t.
*
*******************************************************************************/
cy_en_smif_status_t WriteMemory(cy_stc_smif_mem_config_t const *memConfig, uint8_t address[], uint8_t txBuffer[], uint32_t txSize)
{
    cy_en_smif_status_t status = Cy_SMIF_Memslot_CmdWriteEnable(KIT_QSPI_HW, memConfig, &KIT_QSPI_context);

    if(CY_SMIF_SUCCESS == status)
    {
		status = Cy_SMIF_Memslot_CmdProgram(KIT_QSPI_HW, memConfig, address, txBuffer, txSize, NULL, &KIT_QSPI_context);

		if(CY_SMIF_SUCCESS == status)
		{
			/* Wait until the SMIF block completes transmitting data */
			status = PollTransferStatus(CY_SMIF_SEND_CMPLT);

			if(CY_SMIF_SUCCESS == status)
			{
				/* Wait until the write operation is completed or timeout occurs */
				status = IsMemoryReady(memConfig);
			}
		}
    }

	return status;
}


/*******************************************************************************
* Function Name: EraseMemory
********************************************************************************
*
* Erases a block/sector of external memory
*
* \param memConfig
* Memory device configuration
*
* \param address
* The address of the block to be erased.
*
* \return Status of the operation. See cy_en_smif_status_t.
*
*******************************************************************************/
cy_en_smif_status_t EraseMemory(cy_stc_smif_mem_config_t const *memConfig, uint8_t address[])
{
    cy_en_smif_status_t status = Cy_SMIF_Memslot_CmdWriteEnable(KIT_QSPI_HW, memConfig, &KIT_QSPI_context);

    if(CY_SMIF_SUCCESS == status)
    {
		status = Cy_SMIF_Memslot_CmdSectorErase(KIT_QSPI_HW, (cy_stc_smif_mem_config_t* )memConfig, address, &KIT_QSPI_context);

		if(CY_SMIF_SUCCESS == status)
		{
			/* Wait until the erase operation is completed or timeout occurs. */
			status = IsMemoryReady(memConfig);
		}
    }

    return status;
}


