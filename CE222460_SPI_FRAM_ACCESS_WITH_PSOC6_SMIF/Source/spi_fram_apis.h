/****************************************************************************
*File Name: spi_fram_apis.h
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
#ifndef SPI_FRAM_APIS_H
#define SPI_FRAM_APIS_H


#include "cy_sysint.h"
#include "cy_device_headers.h"
#include "cy_smif_memslot.h"
#include "cy_pdl.h"
#include "cycfg.h"


/***************************************
*       SMIF Function Prototypes 
***************************************/

/*Register and buffer Length*/
#define SR_SIZE      	          (1u)	     /* Status register1 size */
#define DID_REG_SIZE      	      (9u)	     /* SPI F-RAM device ID status register size */
#define SN_BUF_SIZE      	      (8u)	     /* Serial Number register size */

/***************************************
*       SPIM specific constants
***************************************/
#define CMD_WITHOUT_PARAM         (0u)      /* Opcode only commands */
#define TX_LAST_BYTE       	      (1u) 	    /* The last byte in command transmission
									         * (SS is set to high after transmission)*/
#define TX_NOT_LAST_BYTE          (0u)   	/* Not the last byte in command transmission
									         * (SS remains low after transmission) */
#define ADDRESS_SIZE      	      (3u)	    /* F-RAM memory address size */
#define ADDRESS_PLUS_MODE_SIZE    (4u)	    /* F-RAM memory address size with mode byte combined*/

/***************************************
*       SPIM specific constants
***************************************/

/* Memory Read*/
#define MEM_CMD_READ        (0x03)  /* Read in the single mode */
#define MEM_CMD_FAST_READ   (0x0B)  /* Read in the single mode */
#define MEM_CMD_SSRD        (0x4B) 	/* Special sector -256 byte read */

/* Memory Write*/    
#define MEM_CMD_WRITE       (0x02) 	/* The memory write in the single mode */
#define MEM_CMD_WREN        (0x06) 	/* Write Enable */
#define MEM_CMD_SSWR        (0x42) 	/* Special sector -256 byte write */

/* Status*/
#define MEM_CMD_RDSR        (0x05) 	/* Read the status register */
#define MEM_CMD_WRSR        (0x01) 	/* Write the status register */
#define MEM_CMD_WRDI        (0x04) 	/* Write disable */

/* Device ID Write*/
#define MEM_CMD_RDID        (0x9F) 	/* Read device ID */
#define MEM_CMD_RDSN        (0xC3) 	/* Read serial number */
#define MEM_CMD_RUID        (0x4C) 	/* Read unique device ID  */
#define MEM_CMD_WRSN        (0xC2) 	/* Write tserial number */   
    
/* Power Mode Commands*/
#define MEM_CMD_ENTDPD      (0xBA) 	/* Enter DPD - Ramtron mode*/
#define MEM_CMD_ENTHBN      (0xB9) 	/* Enter Hibernate - Ramtron Mode */

/* Constants */
#define PACKET_SIZE         (256u)  /* The emory Read/Write packet */  
    
#define TX_RX_EQUAL       	(1u) 	/* The transmitted and received arrays are equal */
#define TX_RX_NOT_EQUAL     (0u) 	/* The transmitted and received arrays are not equal */
#define TIMEOUT_1_MS        (1000ul)/* 1 ms timeout for all blocking functions */

cy_en_sysint_status_t FramCmdWRSR(cy_en_smif_slave_select_t fram_slave_select, /* Change the Status Register */
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t cmdParam[], 	
                    uint32_t cmdSize);		    		

cy_en_sysint_status_t FramCmdRDSR(cy_en_smif_slave_select_t fram_slave_select,/* Read the status register */
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContextuint8_t,
                    uint8_t tst_rxBuffer[], 
                    uint32_t rxSize);	

cy_en_sysint_status_t FramCmdWRITE(cy_en_smif_slave_select_t fram_slave_select,/* Write to F-RAM memory */
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext, 	
                    uint8_t tst_txBuffer[], 	
                    uint32_t txSize, 	
                    uint8_t *address);	    			

cy_en_sysint_status_t FramCmdREAD(cy_en_smif_slave_select_t fram_slave_select,/* Read data from F-RAM */
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext, 	
                    uint8_t tst_rxBuffer[], 	
                    uint32_t rxSize, 	
                    uint8_t *address);   	

cy_en_sysint_status_t FramCmdFSTRD(cy_en_smif_slave_select_t fram_slave_select,/*Fast Read data from F-RAM with Fast Read opcode*/
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext, 	
                    uint8_t tst_rxBuffer[], 	
                    uint32_t rxSize, 	
                    uint8_t *address); 

cy_en_sysint_status_t FramCmdWREN(cy_en_smif_slave_select_t fram_slave_select,/* Memory Write Enable */
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext);	

cy_en_sysint_status_t FramCmdWRDI(cy_en_smif_slave_select_t fram_slave_select,/* Memory Write Disable */
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext);				   


cy_en_sysint_status_t FramCmdSSWR(cy_en_smif_slave_select_t fram_slave_select,/* Wtite 256-byte Special Sector */
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t tst_txBuffer[], 
                    uint32_t txSize, 
                    uint8_t *address);

cy_en_sysint_status_t FramCmdSSRD(cy_en_smif_slave_select_t fram_slave_select,/* Read 256-byte Special Sector */
		                SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_rxBuffer[], 
                        uint32_t rxSize, 
                        uint8_t *address);

cy_en_sysint_status_t FramCmdWRSN(cy_en_smif_slave_select_t fram_slave_select,/* Wrire 8-byte Serial Number */
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t cmdParam[], 	
                    uint32_t cmdSize);

cy_en_sysint_status_t FramCmdRDSN(cy_en_smif_slave_select_t fram_slave_select, /* Read 8-byte Serial Number */
		          SMIF_Type *baseaddr,
                  cy_stc_smif_context_t *smifContext,   
                  uint8_t tst_rxBuffer[], 
                  uint32_t rxSize);

cy_en_sysint_status_t FramCmdRDID(cy_en_smif_slave_select_t fram_slave_select,/* Read Device ID */
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext,
                    uint8_t tst_rxBuffer[], 
                    uint32_t rxSize);

cy_en_sysint_status_t FramCmdRUID(cy_en_smif_slave_select_t fram_slave_select,/* Read Unique Device ID */
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext,
                    uint8_t tst_rxBuffer[], 
                    uint32_t rxSize);

cy_en_sysint_status_t FramCmdHBN(cy_en_smif_slave_select_t fram_slave_select,/* Enter DPD HIBERNATE */
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext);	

cy_en_sysint_status_t FramCmdDPD(cy_en_smif_slave_select_t fram_slave_select,/* Enter DPD Mode */
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext);			 	

#endif //SPI_FRAM_APIS_H

/* [] END OF FILE */
