/****************************************************************************
*File Name: qspi_fram_apis.c
*
* Version: 1.0
*
* Description: 
* This file contains high-level functions for the F-RAM access in SPI, DPI, QPI, and extended SPI modes.  
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
#include "qspi_fram_apis.h"

cy_smif_event_cb_t RxCmpltCallback;


/*******************************************************************************
* Function Name: FramCmdWREN (0x06)
****************************************************************************//**
*
* This function enables the Write bit in the status register. 
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
*******************************************************************************/
cy_en_sysint_status_t  FramCmdWREN(cy_en_smif_slave_select_t fram_slave_select,
		                    SMIF_Type *baseaddr,
                            cy_stc_smif_context_t *smifContext,
                            uint8_t spimode)
{    
    
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;
	cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;

    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;     
    
    /* Transmit command */
    command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_WREN,				  
                            CY_SMIF_WIDTH,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH, 
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
* This function disables the Write enable bit in the status register. 
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
*******************************************************************************/
cy_en_sysint_status_t  FramCmdWRDI(cy_en_smif_slave_select_t fram_slave_select,
		                    SMIF_Type *baseaddr,
                            cy_stc_smif_context_t *smifContext,
                            uint8_t spimode)
{    

	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;
	cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE; 
    
     /* Transmit command */
    command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_WRDI,				  
                            CY_SMIF_WIDTH,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH, 
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
* Function Name: FramCmdWRSR (0x01)
****************************************************************************//**
*
* This function writes data to the external memory status register 1 (SR1). 
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data. 
*
* \param cmdParam
* Data for the status register 1
* 
* \param spimode
* Determines SMIF width single,dual,or quads
*
*******************************************************************************/
cy_en_sysint_status_t  FramCmdWRSR(cy_en_smif_slave_select_t fram_slave_select,
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t cmdParam[], 
                    uint8_t spimode)
{
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;
	cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
    else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;   
    
    /* Write Enable */
    FramCmdWREN(fram_slave_select, baseaddr, smifContext,spimode);
    
     /* Transmit command and data byte(s) */
    command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_WRSR,				  
                            CY_SMIF_WIDTH,
                            cmdParam, 
                            SR_SIZE, 
                            CY_SMIF_WIDTH, 
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
* Function Name: FramCmdRDSR (0x05 or 0x07)
****************************************************************************//**
*
* This function reads data from the external memory status register
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
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param latency
* Memory latency cycle during read.
* 
*******************************************************************************/
cy_en_sysint_status_t  FramCmdReadSRx(cy_en_smif_slave_select_t fram_slave_select,
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext,
                    uint8_t tst_rxBuffer[], 
                    uint32_t rxSize,
                    uint8_t spimode,
                    uint8_t cmdtype,
                    uint8_t latency)
{
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;
    cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;

    uint8_t CMD_TYPE =0x00;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;  
    
    if (cmdtype==MEM_CMD_RDSR1)
    CMD_TYPE = MEM_CMD_RDSR1;
   
    else if (cmdtype==MEM_CMD_RDSR2)
    CMD_TYPE = MEM_CMD_RDSR2;
    
     /* Transmit command */
    command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            CMD_TYPE,				
                            CY_SMIF_WIDTH,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    
    /* Sends extra dummy clocks to adde clock cycle latency */
    command_Status = Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);
      
    /* Receive data */
    command_Status = Cy_SMIF_ReceiveData( baseaddr,
                        tst_rxBuffer, 					
                        rxSize, 
                        CY_SMIF_WIDTH, 
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
* Function Name: FramCmdSPIWriteAnyReg (0x71)
****************************************************************************//**
*
* This function writes one byte (no burst write) to the FRAM status or configuration register.
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
* \param spimode
* Determines SMIF width single,dual,or quad.
*
*******************************************************************************/
cy_en_sysint_status_t  FramCmdSPIWriteAnyReg(cy_en_smif_slave_select_t fram_slave_select,
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t tst_txBuffer[], 
                    uint32_t txSize, 
                    uint8_t *address,
                    uint8_t spimode)

{         
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;
	cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
      (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;  
    else
     CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    /* Set the write enable (WEL) bit in SR1 */
    FramCmdWREN(fram_slave_select, baseaddr, smifContext, spimode);
    
    /* Transmit command */
    command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_WRAR,		
                            CY_SMIF_WIDTH,
                            address, 
                            ADDRESS_SIZE, 
                            CY_SMIF_WIDTH, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    /* Transmit data */
    command_Status = Cy_SMIF_TransmitData(baseaddr,
                            tst_txBuffer,
                            txSize, 
                            CY_SMIF_WIDTH, 
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
* Function Name: FramCmdSPIReadAnyReg (0x65)
****************************************************************************//**
*
* This function reads one byte (no burst read) from the status or configuration register of FRAM.
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
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param latency
* Register latency cycle during read.
*
*******************************************************************************/
cy_en_sysint_status_t  FramCmdSPIReadAnyReg(cy_en_smif_slave_select_t fram_slave_select,
		                SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_rxBuffer[], 
                        uint32_t rxSize, 
                        uint8_t *address,
                        uint8_t spimode,
                        uint8_t latency)
{   
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;
	cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;    
    
    /* Transmit command */
    command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_RDAR,				
                            CY_SMIF_WIDTH,
                            address, 
                            ADDRESS_SIZE, 
                            CY_SMIF_WIDTH, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);

    /* Sends extra dummy clocks to adde clock cycle latency */
    command_Status = Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);
    
    /* Receive command */
    command_Status = Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH, 
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
* Function Name: FramCmdReadCRx (0x35/0x3F/0x45/0x4E)
****************************************************************************//**
*
* This function reads data from FRAM configuration registers
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
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param cmdtype
* Determines cmd type to tranmit in the command cycle
*
* \param latency
* Memory latency cycle during read.
* 
*******************************************************************************/
cy_en_sysint_status_t  FramCmdReadCRx(cy_en_smif_slave_select_t fram_slave_select,
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext,
                    uint8_t tst_rxBuffer[], 
                    uint32_t rxSize,
                    uint8_t spimode,
                    uint8_t crtype,
                    uint8_t latency)
{
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;
	cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;

	uint8_t CMD_TYPE = 1U;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;  
    
    if (crtype==MEM_CMD_RDCR1)
        CMD_TYPE = MEM_CMD_RDCR1;
   
     else if (crtype==MEM_CMD_RDCR2)
              CMD_TYPE = MEM_CMD_RDCR2;
  
     else if (crtype==MEM_CMD_RDCR4)
              CMD_TYPE = MEM_CMD_RDCR4;
  
     else if (crtype==MEM_CMD_RDCR5)
             CMD_TYPE = MEM_CMD_RDCR5;

     else
     CMD_TYPE = MEM_CMD_RDCR1;
    
    /* Transmit command */
    command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            CMD_TYPE,				
                            CY_SMIF_WIDTH,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    
    /* Sends extra dummy clocks to adde clock cycle latency */
    command_Status = Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);
    
    /* Receive data */
    command_Status = Cy_SMIF_ReceiveData( baseaddr,
                        tst_rxBuffer, 					
                        rxSize, 
                        CY_SMIF_WIDTH, 
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
* Function Name: FramCmdSPIWrite (0x02)
****************************************************************************//**
*
* This function writes data to FRAM in SPI mode
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
* \param spimode
* Determines SMIF width single,dual,or quad.
*
*******************************************************************************/
cy_en_sysint_status_t  FramCmdSPIWrite(cy_en_smif_slave_select_t fram_slave_select,
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t tst_txBuffer[], 
                    uint32_t txSize, 
                    uint8_t *address,
                    uint8_t spimode)
{       
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;
	cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
      (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;  
    else
     CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    /* Set the write enable (WEL) bit in SR1 */
    FramCmdWREN(fram_slave_select, baseaddr, smifContext, spimode);
	
    /* Transmit command and address */
    command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_WRITE,		
                            CY_SMIF_WIDTH,
                            address, 
                            ADDRESS_SIZE, 
                            CY_SMIF_WIDTH, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    
     /* Transmit data */
    command_Status = Cy_SMIF_TransmitData(baseaddr,
                            tst_txBuffer,
                            txSize, 
                            CY_SMIF_WIDTH, 
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
* Function Name: FramCmdSPIRead (0x03)
****************************************************************************//**
*
* This function reads data from FRAM in SPI mode.
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
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param latency
* Memory latency cycle during read.
*
*******************************************************************************/
cy_en_sysint_status_t  FramCmdSPIRead(cy_en_smif_slave_select_t fram_slave_select,
		                SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_rxBuffer[], 
                        uint32_t rxSize, 
                        uint8_t *address,
                        uint8_t spimode,
                        uint8_t latency)
{   
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;
	cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;    
    
    /* Transmit command and address */
    command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_READ,				
                            CY_SMIF_WIDTH,
                            address, 
                            ADDRESS_SIZE, 
                            CY_SMIF_WIDTH, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext); 
    
    /* Sends extra dummy clocks to adde clock cycle latency */
    command_Status = Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);
    
    /* Receive data */
    command_Status = Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH, 
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
* Function Name: FramCmdSPIFastWrite (0xDA)
****************************************************************************//**
*
* This function writes data to FRAM in Fast Write mode.
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
* The address to write data to. It combines the mode byte, following address byte  
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
*******************************************************************************/
cy_en_sysint_status_t  FramCmdSPIFastWrite(cy_en_smif_slave_select_t fram_slave_select,
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t tst_txBuffer[], 
                    uint32_t txSize, 
                    uint8_t *address,
                    uint8_t spimode)
{       
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;
	cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
      (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;  
    else
     CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    /* Set the write enable (WEL) bit in SR1 */
    FramCmdWREN(fram_slave_select, baseaddr, smifContext, spimode);
    
    /* Transmit command and address */
    command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_FASTWRITE,		
                            CY_SMIF_WIDTH,
                            address, 
                            (ADDRESS_SIZE+1),                      /*ADDRESS_PLUS_MODE_SIZE*/  
                            CY_SMIF_WIDTH, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    /* Transmit data */
    command_Status = Cy_SMIF_TransmitData(baseaddr,
                            tst_txBuffer,
                            txSize, 
                            CY_SMIF_WIDTH, 
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
* Function Name: FramCmdSPIFastRead (0x0B)
****************************************************************************//**
*
* This function reads data from FRAM in the SPI mode.
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
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param latency
* Memory latency cycle during read.
*
*******************************************************************************/
cy_en_sysint_status_t  FramCmdSPIFastRead(cy_en_smif_slave_select_t fram_slave_select,
		                SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_rxBuffer[], 
                        uint32_t rxSize, 
                        uint8_t *address,
                        uint8_t spimode,
                        uint8_t latency)
{   
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;
	cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;    
    
    /* Transmit command and address */
    command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_FAST_READ,				
                            CY_SMIF_WIDTH,
                            address, 
                           (ADDRESS_SIZE+1),                      /*ADDRESS_PLUS_MODE_SIZE*/ 
                            CY_SMIF_WIDTH, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext); 
    
    /* Sends extra dummy clocks to adde clock cycle latency */
    command_Status = Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);
    
    /* Receive data */ 
    command_Status = Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH, 
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
* Function Name: FramCmdSSWR (0x42)
****************************************************************************//**
*
* This function writes data to the special sector (256-byte max) of FRAM.
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
* \param spimode
* Determines SMIF width single,dual,or quad.
*
*******************************************************************************/
cy_en_sysint_status_t  FramCmdSSWR(cy_en_smif_slave_select_t fram_slave_select,
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t tst_txBuffer[], 
                    uint32_t txSize, 
                    uint8_t *address,
                    uint8_t spimode)
{  
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;
	cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
      (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;  
    else
     CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;  
    
    /* Set the write enable (WEL) bit in SR1 */
    FramCmdWREN(fram_slave_select, baseaddr, smifContext,spimode);
	
    /* Transmit command and address */
    command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_SSWR,		
                            CY_SMIF_WIDTH,
                            address, 
                            ADDRESS_SIZE, 
                            CY_SMIF_WIDTH, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    
     /* Transmit data */
    command_Status = Cy_SMIF_TransmitData(baseaddr,
                            tst_txBuffer,
                            txSize, 
                            CY_SMIF_WIDTH, 
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
* This function writes 8-byte into FRAM serial number registers.
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
cy_en_sysint_status_t  FramCmdWRSN(cy_en_smif_slave_select_t fram_slave_select,
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t tst_txBuffer[], 
                    uint32_t txSize, 
                    uint8_t spimode)
{
    
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;
	cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
      (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;  
    else
     CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE; 
    
    /* Set the write enable (WEL) bit in SR1 */
    FramCmdWREN(fram_slave_select, baseaddr, smifContext, spimode);
    
    /* Transmit command */    
    command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_WRSN,				  
                            CY_SMIF_WIDTH,
                            tst_txBuffer, 
                            txSize, 
                            CY_SMIF_WIDTH, 
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
* Function Name: FramCmdSPIWrite_DIOW_QIOW (0xA1/0xA2)
****************************************************************************//**
*
* This function writes data to FRAM in the extended SPI mode in DIO or QIO.
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
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param latency
* Memory latency cycle during read.
*
*******************************************************************************/
cy_en_sysint_status_t  FramCmdSPIWrite_DIOW_QIOW(cy_en_smif_slave_select_t fram_slave_select,
		                SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_txBuffer[], 
                        uint32_t txSize, 
                        uint8_t *address,
                        uint8_t CMDtype)

{
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;

	/* Set the write enable (WEL) bit in SR1 */
    FramCmdWREN(fram_slave_select, baseaddr, smifContext, SPI_MODE);
    
    /* Transmit command and address in DIO or QIO */
      if( CMDtype==MEM_CMD_DIOW)
    	  command_Status =  Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_DIOW ,				
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                           (ADDRESS_SIZE+1),                      /*ADDRESS_PLUS_MODE_SIZE*/ 
                            CY_SMIF_WIDTH_DUAL, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);         
    
      if( CMDtype==MEM_CMD_QIOW)
    	  command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_QIOW,				
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                           (ADDRESS_SIZE+1),                      /*ADDRESS_PLUS_MODE_SIZE*/ 
                            CY_SMIF_WIDTH_QUAD, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);     
    
    /* Transmit data byte(s) in DIO or QIO */ 
    if( CMDtype==MEM_CMD_DIOW)    
    	command_Status = Cy_SMIF_TransmitData(baseaddr,
                            tst_txBuffer,
                            txSize, 
                            CY_SMIF_WIDTH_DUAL, 
                            RxCmpltCallback, 
                            smifContext);
       
    if( CMDtype==MEM_CMD_QIOW)
    	command_Status = Cy_SMIF_TransmitData(baseaddr,
                            tst_txBuffer, 
                            txSize, 
                            CY_SMIF_WIDTH_QUAD, 
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
* Function Name: FramCmdSPIRead_DIOR_QIOR (0xBB/0xEB)
****************************************************************************//**
*
* This function reads data from FRAM in extended SPI mode DIO or QIO.
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
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param latency
* Memory latency cycle during read.
*
*******************************************************************************/
cy_en_sysint_status_t  FramCmdSPIRead_DIOR_QIOR(cy_en_smif_slave_select_t fram_slave_select,
		                SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_rxBuffer[], 
                        uint32_t rxSize, 
                        uint8_t *address,
                        uint8_t spimode,
                        uint8_t latency,
                        uint8_t CMDtype)
{       
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;

	/* Transmit command and address in DIO or QIO */
    if (spimode==SPI_MODE)      
     { 
      if( CMDtype==MEM_CMD_DIOR)
    	  command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_DIOR,				
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                           (ADDRESS_SIZE+1),                      /*ADDRESS_PLUS_MODE_SIZE*/ 
                            CY_SMIF_WIDTH_DUAL, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);         
    
      if( CMDtype==MEM_CMD_QIOR)
    	  command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_QIOR,				
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                           (ADDRESS_SIZE+1),                      /*ADDRESS_PLUS_MODE_SIZE*/ 
                            CY_SMIF_WIDTH_QUAD, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);  
    
      }
    
    if (spimode==QPI_MODE)                                      
     { 	
    	command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_QIOR,				
                            CY_SMIF_WIDTH_QUAD,
                            address, 
                           (ADDRESS_SIZE+1),                      /*ADDRESS_PLUS_MODE_SIZE*/ 
                            CY_SMIF_WIDTH_QUAD, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);
      } 
 
    /* Sends extra dummy clocks to adde clock cycle latency */
    command_Status = Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);
   
    /* Transmit data in DIO or QIO */
    if( CMDtype==MEM_CMD_DIOR)
    	command_Status = Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH_DUAL, 
                            RxCmpltCallback,
                            smifContext);
    
    if( CMDtype==MEM_CMD_QIOR)    
    	command_Status = Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH_QUAD, 
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
* Function Name: FramCmdSPIWrite_DIW_QIW (0xA2/0x32)
****************************************************************************//**
*
* This function writes data to FRAM in extended SPI mode DIW, QIW.
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
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param latency
* Memory latency cycle during read.
*
*******************************************************************************/
cy_en_sysint_status_t  FramCmdSPIWrite_DIW_QIW(cy_en_smif_slave_select_t fram_slave_select,
		                SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_txBuffer[], 
                        uint32_t txSize, 
                        uint8_t *address,
                        uint8_t CMDtype)

{
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;

	/* Set the write enable (WEL) bit in SR1 */
    FramCmdWREN(fram_slave_select, baseaddr, smifContext, SPI_MODE);
    
    /* Transmit command and address in DIW or QIW */
    if( CMDtype==MEM_CMD_DIW)
    	command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_DIW ,				
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                           (ADDRESS_SIZE+1),                      /*ADDRESS_PLUS_MODE_SIZE*/ 
                            CY_SMIF_WIDTH_SINGLE, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);         
    
    if( CMDtype==MEM_CMD_QIW)
    	command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_QIW,				
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                           (ADDRESS_SIZE+1),                      /*ADDRESS_PLUS_MODE_SIZE*/ 
                            CY_SMIF_WIDTH_SINGLE, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);     
    
   /* Transmit data byte(s) in DIW or QIW */ 
   if( CMDtype==MEM_CMD_DIW)    
	   command_Status = Cy_SMIF_TransmitData(baseaddr,
                            tst_txBuffer,
                            txSize, 
                            CY_SMIF_WIDTH_DUAL, 
                            RxCmpltCallback, 
                            smifContext);
       
   if( CMDtype==MEM_CMD_QIW)    
	   command_Status = Cy_SMIF_TransmitData(  baseaddr,
                            tst_txBuffer, 
                            txSize, 
                            CY_SMIF_WIDTH_QUAD, 
                            RxCmpltCallback,
                            smifContext);
    
    /* Check if the SMIF IP is busy */
    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }

    return (command_Status);
}

/* ****************************************************************************//**
* Function Name: FramCmdSPIRead_DOR_QOR (0x3B/0x6B)
****************************************************************************//**
*
* This function reads data from FRAM in extended SPI DOR, QOR mode.
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
* \IOMode
* Determines extended SPI mode dual or quad
*
* \param latency
* Memory latency cycle during read.
*
*******************************************************************************/
cy_en_sysint_status_t  FramCmdSPIRead_DOR_QOR(cy_en_smif_slave_select_t fram_slave_select,
		                SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_rxBuffer[], 
                        uint32_t rxSize, 
                        uint8_t *address,
                        uint8_t CMDtype,
                        uint8_t latency)
{   
    
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;

	/* Transmit command and data in DO */
    if (CMDtype==MEM_CMD_DOR)      
     { 	
    	command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_DOR,				
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                           (ADDRESS_SIZE+1),                      /*ADDRESS_PLUS_MODE_SIZE*/ 
                            CY_SMIF_WIDTH_SINGLE, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    
       /* Sends extra dummy clocks to adde clock cycle latency */
    	command_Status = Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);
    
    	command_Status = Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH_DUAL, 
                            RxCmpltCallback,
                            smifContext);
      }
    
    if (CMDtype==MEM_CMD_QOR)      
     { 	
    	command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_QOR,				
                            CY_SMIF_WIDTH_SINGLE,
                            address, 
                           (ADDRESS_SIZE+1),                      /*ADDRESS_PLUS_MODE_SIZE*/ 
                            CY_SMIF_WIDTH_SINGLE, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    
    /* Sends extra dummy clocks to adde clock cycle latency */
    	command_Status = Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);
    
    /* Transmit data in DO */
    if( CMDtype==MEM_CMD_DOR)
    	command_Status = Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH_DUAL, 
                            RxCmpltCallback,
                            smifContext);
    
    if( CMDtype==MEM_CMD_QOR)    
    	command_Status = Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH_QUAD, 
                            RxCmpltCallback,
                            smifContext);
      } 
 
    
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
* This function reads 8-byte device ID of FRAM
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
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param latency
* Register latency cycle during register read.
*
*******************************************************************************/
cy_en_sysint_status_t  FramCmdRDID(cy_en_smif_slave_select_t fram_slave_select,
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext,
                     uint8_t tst_rxBuffer[],
                    uint8_t spimode,
                    uint8_t latency)
{
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;
	cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE; 
    
    /* Transmit command */
    command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_RDID,				
                            CY_SMIF_WIDTH,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    
    /* Sends extra dummy clocks to adde clock cycle latency */
    command_Status = Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);
    
    /* Receive data */
    command_Status = Cy_SMIF_ReceiveData( baseaddr,
                        tst_rxBuffer, 					
                        DID_REG_SIZE, 
                        CY_SMIF_WIDTH, 
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
* Function Name: FramCmdRDUID (0x4C)
****************************************************************************//**
*
* This function reads 8-byte unique device ID of the external FRAM
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
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param latency
* Register latency cycle during register read.
*
*******************************************************************************/
cy_en_sysint_status_t  FramCmdRDUID(cy_en_smif_slave_select_t fram_slave_select,
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext,
                    uint8_t tst_rxBuffer[],
                    uint8_t spimode,
                    uint8_t latency)
{
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;
	cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;    
    
    /* Transmit command */
    command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_RUID,				
                            CY_SMIF_WIDTH,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    
    /* Sends extra dummy clocks to adde clock cycle latency */
    command_Status = Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);
    
    /* Receive data */
    command_Status = Cy_SMIF_ReceiveData( baseaddr,
                        tst_rxBuffer, 					
                        UID_BUF_SIZE, 
                        CY_SMIF_WIDTH, 
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
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param latency
* Register latency cycle during read.
*
*******************************************************************************/
cy_en_sysint_status_t  FramCmdRDSN(cy_en_smif_slave_select_t fram_slave_select,
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext,
                    uint8_t tst_rxBuffer[],
                    uint32_t txSize, 
                    uint8_t spimode,
                    uint8_t latency)
{
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;
	cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;    
     
    /* Transmit command */
    command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_RDSN,				
                            CY_SMIF_WIDTH,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    
    /* Sends extra dummy clocks to adde clock cycle latency */
    command_Status = Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);
    
    /* Receive data */
    command_Status = Cy_SMIF_ReceiveData( baseaddr,
                        tst_rxBuffer, 					
                        txSize, 
                        CY_SMIF_WIDTH, 
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
* This function reads special sector (max 256 bytes) of FRAM
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
* The address to read special sector data from. 
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
* \param latency
* Memory latency cycle during read.
*
*******************************************************************************/
cy_en_sysint_status_t  FramCmdSSRD(cy_en_smif_slave_select_t fram_slave_select,
		                SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_rxBuffer[], 
                        uint32_t rxSize, 
                        uint8_t *address,
                        uint8_t spimode,
                        uint8_t latency)
{   
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;
	cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;   
    
    /* Transmit command */
    command_Status = Cy_SMIF_TransmitCommand( baseaddr,
                            MEM_CMD_SSRD,				
                            CY_SMIF_WIDTH,
                            address, 
                            ADDRESS_SIZE, 
                            CY_SMIF_WIDTH, 
							fram_slave_select,
                            TX_NOT_LAST_BYTE, 
                            smifContext);
    
    /* Sends extra dummy clocks to adde clock cycle latency */
    command_Status = Cy_SMIF_SendDummyCycles(baseaddr, (uint32_t)latency);
     
    /* Receive data */
    command_Status = Cy_SMIF_ReceiveData(  baseaddr,
                            tst_rxBuffer, 
                            rxSize, 
                            CY_SMIF_WIDTH, 
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
* Function Name: FramCmdEnterLPMode (0xBA (Hibernate)/0xB9(DPD))
****************************************************************************//**
*
* This function enters the device into low power DPD or hibernate mode. 
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param cmdtype
* Determines cmd type to send in the command cycle
*
* \param spimode
* Determines SMIF width single,dual,or quad.
*
*******************************************************************************/
cy_en_sysint_status_t  FramCmdEnterLPMode(cy_en_smif_slave_select_t fram_slave_select,
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext,
                    uint8_t cmdtype, 
                    uint8_t spimode)
{    
	cy_en_sysint_status_t command_Status = CY_SYSINT_SUCCESS;
	cy_en_smif_txfr_width_t CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
    
	uint8_t CMD_TYPE=0x00;

    if (spimode==SPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;
     else if
       (spimode==DPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_DUAL;    
     else if
       (spimode==QPI_MODE)
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_QUAD;   
    else
       CY_SMIF_WIDTH = CY_SMIF_WIDTH_SINGLE;      
    
    if (cmdtype==MEM_CMD_ENTDPD)
      CMD_TYPE = MEM_CMD_ENTDPD;
   
     else if (cmdtype==MEM_CMD_ENTHBN)
           CMD_TYPE = MEM_CMD_ENTHBN;  
    else
    CMD_TYPE = MEM_CMD_RDSR1; 
    
    /* Transmit command */
    command_Status = Cy_SMIF_TransmitCommand(baseaddr,
                            CMD_TYPE,				  
                            CY_SMIF_WIDTH,
                            CMD_WITHOUT_PARAM, 
                            CMD_WITHOUT_PARAM, 
                            CY_SMIF_WIDTH, 
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
