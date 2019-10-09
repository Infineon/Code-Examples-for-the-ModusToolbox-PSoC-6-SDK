/****************************************************************************
*File Name: spi_fram_apis.c
*
* Version: 1.0
*
* Description: 
* This file contains high-level functions for the F-RAM access in SPI mode.
*
*
*******************************************************************************
* Copyright (2019), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
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

#include "spi_fram_apis.h"

cy_smif_event_cb_t RxCmpltCallback;

/*******************************************************************************
* Function Name: FramCmdWRSR (0x01)
****************************************************************************//**
*
* This function writes data to the FRAM status register.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data. 
*
* \param cmdParam
* Data for the status register
* 
* \param cmdSize
* The size of data for the status register. Can be 1 or 2.
*
*******************************************************************************/
cy_en_sysint_status_t FramCmdWRSR(  cy_en_smif_slave_select_t fram_slave_select,
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t cmdParam[], 
                    uint32_t cmdSize)
{
    cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;

    /* Write Enable */
    FramCmdWREN(fram_slave_select, baseaddr, smifContext);
    
    /* Write Status */ 
    command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_WRSR,				  
                            CY_SMIF_WIDTH_SINGLE,
                            cmdParam, 
                            cmdSize, 
                            CY_SMIF_WIDTH_SINGLE, 
							fram_slave_select,
                            TX_LAST_BYTE, 
                            smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }

    return (command_Status);

}

/*******************************************************************************
* Function Name: FramCmdRDSR (0x05)
****************************************************************************//**
*
* This function reads data from FRAM status register.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_rxBuffer 
* The buffer for read status register data.
* 
* \param rxSize 
* The size of data to read.
* 
*******************************************************************************/
cy_en_sysint_status_t FramCmdRDSR(cy_en_smif_slave_select_t fram_slave_select,
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext,
                    uint8_t tst_rxBuffer[], 
                    uint32_t rxSize)
{
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;

    /* The Status Register Read command */
	command_Status = Cy_SMIF_TransmitCommand(baseaddr,
                            MEM_CMD_RDSR,				
                            CY_SMIF_WIDTH_SINGLE,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH_SINGLE, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);

	command_Status = Cy_SMIF_ReceiveData( baseaddr,
                        tst_rxBuffer, 					
                        rxSize, 
                        CY_SMIF_WIDTH_SINGLE, 
                        RxCmpltCallback,
                        smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed */
    }

    return (command_Status);
}

/*******************************************************************************
* Function Name: FramCmdWRITE (0x02)
****************************************************************************//**
*
* This function writes to FRAM.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_txBuffer 
* Data to write in the external memory.
* 
* \param txSize 
* The size of data.
* 
* \param address 
* The address to write data to.  
*
*******************************************************************************/
cy_en_sysint_status_t FramCmdWRITE(cy_en_smif_slave_select_t fram_slave_select,
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t tst_txBuffer[], 
                    uint32_t txSize, 
                    uint8_t *address)
{     
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;

    /* Write Enable */
	FramCmdWREN(fram_slave_select, baseaddr, smifContext);
	
    /* The memory write command */
    command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_WRITE,		
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                            ADDRESS_SIZE, 
                            CY_SMIF_WIDTH_SINGLE, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);

	command_Status = Cy_SMIF_TransmitData(baseaddr,
                            tst_txBuffer,
                            txSize, 
                            CY_SMIF_WIDTH_SINGLE, 
                            RxCmpltCallback, 
                            smifContext);
    
    /* Check if the SMIF IP is busy */
   while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed */
    }

   return (command_Status);
}

/*******************************************************************************
* Function Name: FramCmdSSWR (0x42)
****************************************************************************//**
*
* This function writes to FRAM special sector
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_txBuffer 
* Data to write in the external memory.
* 
* \param txSize 
* The size of data.
* 
* \param address 
* The address to write data to.  
*
*******************************************************************************/
cy_en_sysint_status_t FramCmdSSWR(cy_en_smif_slave_select_t fram_slave_select,
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t tst_txBuffer[], 
                    uint32_t txSize, 
                    uint8_t *address)
{  
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;

	/* Write Enable */
	FramCmdWREN(fram_slave_select, baseaddr, smifContext);
	
    /* The memory write command */
    command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_SSWR,		
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                            ADDRESS_SIZE, 
                            CY_SMIF_WIDTH_SINGLE, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);

	command_Status = Cy_SMIF_TransmitData(baseaddr,
                            tst_txBuffer,
                            txSize, 
                            CY_SMIF_WIDTH_SINGLE, 
                            RxCmpltCallback, 
                            smifContext);
    
    /* Check if the SMIF IP is busy */
   while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed */
    }
   return (command_Status);
}


/*******************************************************************************
* Function Name: FramCmdWRSN (0xC2)
****************************************************************************//**
*
* This function writes to FRAM serial number register
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_txBuffer 
* Data to write in the external memory.
* 
* \param txSize 
* The size of data.
* 
* \param address 
* The address to write data to.  
*
*******************************************************************************/
cy_en_sysint_status_t FramCmdWRSN(cy_en_smif_slave_select_t fram_slave_select,
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t cmdParam[], 
                    uint32_t cmdSize)
{
    
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;

	/* Write Enable */
	FramCmdWREN(fram_slave_select, baseaddr, smifContext);
    
    /* Write Status */    
    command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_WRSN,				  
                            CY_SMIF_WIDTH_SINGLE,
                            cmdParam, 
                            cmdSize, 
                            CY_SMIF_WIDTH_SINGLE, 
							fram_slave_select,
                            TX_LAST_BYTE, 
                            smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }
    return (command_Status);
}

/*******************************************************************************
* Function Name: FramCmdREAD (0x03)
****************************************************************************//**
*
* This function reads from FRAM serial number register
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_rxBuffer 
* The buffer for read data.
* 
* \param rxSize 
* The size of data to read.
* 
* \param address 
* The address to read data from.   
*
*******************************************************************************/
cy_en_sysint_status_t FramCmdREAD(cy_en_smif_slave_select_t fram_slave_select,
		                SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_rxBuffer[], 
                        uint32_t rxSize, 
                        uint8_t *address)
{   
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;

	/* Read memory data */
	command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_READ,				
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                            ADDRESS_SIZE, 
                            CY_SMIF_WIDTH_SINGLE, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);

	command_Status = Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH_SINGLE, 
                            RxCmpltCallback,
                            smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }

		return (command_Status);
}

/*******************************************************************************
* Function Name: FramCmdFSTRD (0x0B)
****************************************************************************//**
*
* This function reads data from the FRAM in Fast Read mode.
* The fast read command requires 8-clock cycle latency before valid data is
* driven on the output pin
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_rxBuffer 
* The buffer for read data.
* 
* \param rxSize 
* The size of data to read.
* 
* \param address 
* The address to read data from.   
*
*******************************************************************************/
cy_en_sysint_status_t FramCmdFSTRD(cy_en_smif_slave_select_t fram_slave_select,
		                SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_rxBuffer[], 
                        uint32_t rxSize, 
                        uint8_t *address)
{   
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;

	/* Read memory data */
	command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_FAST_READ,				
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                            ADDRESS_SIZE, 
                            CY_SMIF_WIDTH_SINGLE, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);    

	command_Status = Cy_SMIF_SendDummyCycles (baseaddr, 0x08);

	command_Status = Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH_SINGLE, 
                            RxCmpltCallback,
                            smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }

    return (command_Status);
}

/*******************************************************************************
* Function Name: FramCmdWREN (0x06)
****************************************************************************//**
*
* This function sets the write enable latch (WEL = 1) in the FRAM status register.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
*******************************************************************************/
cy_en_sysint_status_t FramCmdWREN(cy_en_smif_slave_select_t fram_slave_select,
		                    SMIF_Type *baseaddr,
                            cy_stc_smif_context_t *smifContext)
{    
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;

	/* Memory Write Enable */
	command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_WREN,				  
                            CY_SMIF_WIDTH_SINGLE,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH_SINGLE, 
							fram_slave_select,
                            TX_LAST_BYTE, 
                            smifContext); 
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }

    return (command_Status);
}

/*******************************************************************************
* Function Name: FramCmdWRDI (0x04)
****************************************************************************//**
*
* This function disables the write enable latch (WEL = 0) in the FRAM status register.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
*******************************************************************************/
cy_en_sysint_status_t FramCmdWRDI(cy_en_smif_slave_select_t fram_slave_select,
		                    SMIF_Type *baseaddr,
                            cy_stc_smif_context_t *smifContext)
{    
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;

	 /* Memory Write Enable */
	command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_WRDI,				  
                            CY_SMIF_WIDTH_SINGLE,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH_SINGLE, 
							fram_slave_select,
                            TX_LAST_BYTE, 
                            smifContext); 
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }

    return (command_Status);
}

/*******************************************************************************
* Function Name: FramCmdRDID (0x9F)
****************************************************************************//**
*
* This function reads device ID from the FRAM
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_rxBuffer 
* The buffer for read data.
* 
* \param rxSize 
* The size of data to read.
* 
*******************************************************************************/
cy_en_sysint_status_t FramCmdRDID(cy_en_smif_slave_select_t fram_slave_select,
		          SMIF_Type *baseaddr,
                  cy_stc_smif_context_t *smifContext,
                  uint8_t tst_rxBuffer[], 
                  uint32_t rxSize)
{
  
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;

	command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_RDID,				
                            CY_SMIF_WIDTH_SINGLE,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH_SINGLE, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);

	command_Status = Cy_SMIF_ReceiveData( baseaddr,
                        tst_rxBuffer, 					
                        rxSize, 
                        CY_SMIF_WIDTH_SINGLE, 
                        RxCmpltCallback,
                        smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed */
    }

    return (command_Status);
 }

/*******************************************************************************
* Function Name: FramCmdRUID (0x4C)
****************************************************************************//**
*
* This function reads unique device ID of FRAM
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_rxBuffer 
* The buffer for read data.
* 
* \param rxSize 
* The size of data to read.
* 
*******************************************************************************/
cy_en_sysint_status_t FramCmdRUID(cy_en_smif_slave_select_t fram_slave_select,
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext,
                    uint8_t tst_rxBuffer[], 
                    uint32_t rxSize)
{
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;

	command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_RUID,				
                            CY_SMIF_WIDTH_SINGLE,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH_SINGLE, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);

	command_Status = Cy_SMIF_ReceiveData( baseaddr,
                        tst_rxBuffer, 					
                        rxSize, 
                        CY_SMIF_WIDTH_SINGLE, 
                        RxCmpltCallback,
                        smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed */
    }

    return (command_Status);
}

/*******************************************************************************
* Function Name: FramCmdRDSN (0xC3)
****************************************************************************//**
*
* This function reads 8-byte serial number from FRAM
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param tst_rxBuffer 
* The buffer for read data.
* 
* \param rxSize 
* The size of data to read.
* 
*******************************************************************************/
cy_en_sysint_status_t FramCmdRDSN(cy_en_smif_slave_select_t fram_slave_select,
		           SMIF_Type *baseaddr,
                   cy_stc_smif_context_t *smifContext,
                   uint8_t tst_rxBuffer[],
                   uint32_t rxSize)
{
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;

	command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_RDSN,				
                            CY_SMIF_WIDTH_SINGLE,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH_SINGLE, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);

	command_Status = Cy_SMIF_ReceiveData( baseaddr,
                        tst_rxBuffer, 					
                        rxSize, 
                        CY_SMIF_WIDTH_SINGLE, 
                        RxCmpltCallback,
                        smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed */
    }

    return (command_Status);
}

/*******************************************************************************
* Function Name: FramCmdSSRD (0x4B)
****************************************************************************//**
*
* This function reads special sector of FRAM
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
*******************************************************************************/
cy_en_sysint_status_t FramCmdSSRD(cy_en_smif_slave_select_t fram_slave_select,
		                SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_rxBuffer[], 
                        uint32_t rxSize, 
                        uint8_t *address)
{   
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;

	/* Read memory data */
	command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_SSRD,				
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                            ADDRESS_SIZE, 
                            CY_SMIF_WIDTH_SINGLE, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);    

	command_Status = Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH_SINGLE, 
                            RxCmpltCallback,
                            smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }

    return (command_Status);
}

/********************************************************
******Low Power Mode Commands****************************
*********************************************************/

/*****************************************************************************
* Function Name: FramCmdHBN (0xB9)
***************************************************************************//*
*
* This function puts FRAM in low power Hibernate mode.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
*******************************************************************************/
cy_en_sysint_status_t FramCmdHBN(cy_en_smif_slave_select_t fram_slave_select,
		              SMIF_Type *baseaddr,
                      cy_stc_smif_context_t *smifContext)
{    
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;

	/* Memory Write Enable */
	command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_ENTHBN,				  
                            CY_SMIF_WIDTH_SINGLE,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH_SINGLE, 
							fram_slave_select,
                            TX_LAST_BYTE, 
                            smifContext); 
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }


 	return (command_Status);
}

/*******************************************************************************
* Function Name: FramCmdDPD (0xBA)
****************************************************************************//**
*
* This function puts FRAM in low power DPD mode
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
*******************************************************************************/
cy_en_sysint_status_t FramCmdDPD(cy_en_smif_slave_select_t fram_slave_select,
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext)
{    
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;

	/* Memory Write Enable */
	command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_ENTDPD,				  
                            CY_SMIF_WIDTH_SINGLE,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH_SINGLE, 
							fram_slave_select,
                            TX_LAST_BYTE, 
                            smifContext); 
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }

    return (command_Status);
}

/* [] END OF FILE */
