/****************************************************************************
*File Name: qspi_fram_apis.h
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
#ifndef QSPI_FRAM_APIS_H
#define QSPI_FRAM_APIS_H

#include "cy_sysint.h"
#include "cy_device_headers.h"
#include "cy_smif_memslot.h"
#include "cy_pdl.h"
#include "cycfg.h"

/***************************************
*       SMIF Function Prototypes 
***************************************/

/*Register and buffer Length*/
#define SR_SIZE      	          (1u)	 /* Status register1 size */
#define DID_REG_SIZE      	      (8u)	 /* Excelon Ultra QSPI F-RAM Device ID status register size */
#define SN_BUF_SIZE      	      (8u)	 /* Serial Number register size */
#define UID_BUF_SIZE      	      (8u)	 /* Unique Serial Number register size */

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
#define TIMEOUT_1_MS              (1000ul)  /* 1 ms timeout for all blocking functions */

/* SPIM Access Modes */
#define SPI_MODE                  (0u)      /* Transmits in SPI SDR mode */
#define DPI_MODE                  (1u)      /* Transmits in DPI SDR mode */ 
#define QPI_MODE                  (2u)      /* Transmits in QPI SDR mode */


/***************************************
*     QSPI F-RAM specific Opcodes
***************************************/
/* QSPI F-RAM Read Opcodes*/
#define MEM_CMD_READ              (0x03)    /* Memory READ opcode */
#define MEM_CMD_FAST_READ         (0x0B)    /* Memory FastRead opcode */
#define MEM_CMD_DIOR              (0xBB)    /* Memory read opcode - Dual IO Read (DIOR) */
#define MEM_CMD_QIOR              (0xEB)    /* Memory read opcode - Quad IO Read (QIOR) */
#define MEM_CMD_DOR               (0x3B)    /* Memory read opcode - Dual output Read (DOR) */
#define MEM_CMD_QOR               (0x6B)    /* Memory read opcode - Quad output Read (QOR) */
#define MEM_CMD_SSRD              (0x4B) 	/* Special sector -256 byte read */

/* QSPI F-RAM Write Opcodes*/    
#define MEM_CMD_WRITE             (0x02) 	/* Memory WRITE opcode */
#define MEM_CMD_WREN              (0x06) 	/* Write Enable */
#define MEM_CMD_SSWR              (0x42) 	/* Special sector -256 byte write */
#define MEM_CMD_FASTWRITE         (0xDA)    /* Fast Write opcode */
#define MEM_CMD_DIW               (0xA2)    /* Dual input write - data on dual line */
#define MEM_CMD_DIOW              (0xA1)    /* Dual input input write - address and data on dual line */
#define MEM_CMD_QIW               (0x32)    /* Quad input write - data on quad line */
#define MEM_CMD_QIOW              (0xD2)    /* Quad input input write - address and data on quad line */

/* Status and Configuration Register Opcodes */
#define MEM_CMD_WRSR              (0x01) 	/* Write the status and configuration registers (SR1, CR1, CR2, CR4, CR5) */
#define MEM_CMD_WRDI              (0x04) 	/* Write disable the write enable latch */
#define MEM_CMD_RDSR1             (0x05) 	/* Read Status Register 1 (SR1) */
#define MEM_CMD_RDSR2             (0x07) 	/* Read Status Register 2 (SR2) */
#define MEM_CMD_RDCR1             (0x35) 	/* Read Configuration Register 1 (CR1) */
#define MEM_CMD_RDCR2             (0x3F) 	/* Read Configuration Register 2 (CR2) */
#define MEM_CMD_RDCR4             (0x45) 	/* Read Configuration Register 4 (CR4) */
#define MEM_CMD_RDCR5             (0x5E) 	/* Read Configuration Register 5 (CR5) */
#define MEM_CMD_WRAR              (0x71) 	/* Write to any one register (SR1, CR1, CR2, CR4, CR5), one byte at a time */
#define MEM_CMD_RDAR              (0x65) 	/* Read from any one register (SR1, CR1, CR2, CR4, CR5), one byte at a time */

/* Device ID Access Opcodes*/
#define MEM_CMD_RDID              (0x9F) 	/* Command to read 8-byte device ID */
#define MEM_CMD_RDSN              (0xC3) 	/* Command to read 8-byte Serial Number (SN) */
#define MEM_CMD_RUID              (0x4C) 	/* Command to read 8-byte unique ID */
#define MEM_CMD_WRSN              (0xC2) 	/* Command to write 8-byte Serial Number (SN) */
    
/* Low Power Mode Commands*/
#define MEM_CMD_ENTDPD            (0xB9) 	/* Enter DPD*/
#define MEM_CMD_ENTHBN            (0xBA) 	/* Enter Hibernate*/

/***************************************/
/*QSPI F-RAM Function Prototype         */
/***************************************/

/* Change the Status Register1, one byte */
cy_en_sysint_status_t  FramCmdWRSR(cy_en_smif_slave_select_t fram_slave_select,
		          SMIF_Type *baseaddr,
                  cy_stc_smif_context_t *smifContext, 
                  uint8_t cmdParam[], 	
                  uint8_t spimode);	

/* Read from Status Register (SR1 and SR2) */
cy_en_sysint_status_t  FramCmdReadSRx(cy_en_smif_slave_select_t fram_slave_select,
		          SMIF_Type *baseaddr,
                  cy_stc_smif_context_t *smifContext,
                  uint8_t tst_rxBuffer[], 
                  uint32_t rxSize,
                  uint8_t spimode,
                  uint8_t cmdtype,
                  uint8_t latency);	

/* Read Any Register (Status and Config) from register address, one byte */
cy_en_sysint_status_t  FramCmdSPIReadAnyReg(cy_en_smif_slave_select_t fram_slave_select,
		                   SMIF_Type *baseaddr,
                           cy_stc_smif_context_t *smifContext, 
                           uint8_t tst_rxBuffer[], 
                           uint32_t rxSize, 
                           uint8_t *address,
                           uint8_t spimode,
                           uint8_t latency);

/* Write Any Register (Status and Config) at register address, one byte */
cy_en_sysint_status_t  FramCmdSPIWriteAnyReg(cy_en_smif_slave_select_t fram_slave_select,
		                    SMIF_Type *baseaddr,
                            cy_stc_smif_context_t *smifContext, 
                            uint8_t tst_txBuffer[], 
                            uint32_t txSize, 
                            uint8_t *address,
                            uint8_t spimode);

/* Read Config Register (CR1, CR2, CR4, CR5)*/
cy_en_sysint_status_t  FramCmdReadCRx(cy_en_smif_slave_select_t fram_slave_select,
		            SMIF_Type *baseaddr,
                   cy_stc_smif_context_t *smifContext,
                   uint8_t tst_rxBuffer[], 
                   uint32_t rxSize,
                   uint8_t spimode,
                   uint8_t crtype,
                   uint8_t latency);

/* Write to memory */
cy_en_sysint_status_t  FramCmdSPIWrite(cy_en_smif_slave_select_t fram_slave_select,
					  SMIF_Type *baseaddr,
                      cy_stc_smif_context_t *smifContext, 	
                      uint8_t tst_txBuffer[], 	
                      uint32_t txSize, 	
                      uint8_t *address,
                      uint8_t spimode);	    			

/* Read from memory using Read command */
cy_en_sysint_status_t  FramCmdSPIRead(cy_en_smif_slave_select_t fram_slave_select,
		             SMIF_Type *baseaddr,
                     cy_stc_smif_context_t *smifContext, 	
                     uint8_t tst_rxBuffer[], 	
                     uint32_t rxSize, 	
                     uint8_t *address,
                     uint8_t spimode,
                     uint8_t latency);   

/* Fast Write to memory */
cy_en_sysint_status_t  FramCmdSPIFastWrite(cy_en_smif_slave_select_t fram_slave_select,
		            SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext, 
                    uint8_t tst_txBuffer[], 
                    uint32_t txSize, 
                    uint8_t *address,
                    uint8_t spimode);

/* Read from memory using FastRead command */
cy_en_sysint_status_t  FramCmdSPIFastRead(cy_en_smif_slave_select_t fram_slave_select,
		                 SMIF_Type *baseaddr,
                         cy_stc_smif_context_t *smifContext, 	
                         uint8_t tst_rxBuffer[], 	
                         uint32_t rxSize, 	
                         uint8_t *address,
                         uint8_t spimode,
                         uint8_t latency);

/* Write to memory using DIOW, QIOW in extended SPI dual IO, quad IO mode */
cy_en_sysint_status_t  FramCmdSPIWrite_DIOW_QIOW(cy_en_smif_slave_select_t fram_slave_select,
		                SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_txBuffer[], 
                        uint32_t txSize, 
                        uint8_t *address,
                        uint8_t CMDtype);

/* Read from memory using DIOR, QIOR in extended SPI dual IO, quad IO mode */
cy_en_sysint_status_t  FramCmdSPIRead_DIOR_QIOR(cy_en_smif_slave_select_t fram_slave_select,
		                       SMIF_Type *baseaddr,
                               cy_stc_smif_context_t *smifContext, 
                               uint8_t tst_rxBuffer[], 
                               uint32_t rxSize, 
                               uint8_t *address,
                               uint8_t spimode,
                               uint8_t latency,
                               uint8_t CMDtype);

/* Write to memory using DIW, QIW in extended SPI dual line data input, quad line input */
cy_en_sysint_status_t  FramCmdSPIWrite_DIW_QIW(cy_en_smif_slave_select_t fram_slave_select,
		                SMIF_Type *baseaddr,
                        cy_stc_smif_context_t *smifContext, 
                        uint8_t tst_txBuffer[], 
                        uint32_t txSize, 
                        uint8_t *address,
                        uint8_t CMDtype);

/* Read from memory using DOR, QOR in extended SPI dual line output, quad line output  */
cy_en_sysint_status_t  FramCmdSPIRead_DOR_QOR(cy_en_smif_slave_select_t fram_slave_select,
		                     SMIF_Type *baseaddr,
                             cy_stc_smif_context_t *smifContext, 
                             uint8_t tst_rxBuffer[], 
                             uint32_t rxSize, 
                             uint8_t *address,
                             uint8_t CMDtype,
                             uint8_t latency);

/* Memory Write Enable */
cy_en_sysint_status_t  FramCmdWREN(cy_en_smif_slave_select_t fram_slave_select,
		          SMIF_Type *baseaddr,
                  cy_stc_smif_context_t *smifContext,
                  uint8_t spimode);	

/* Memory Write Disable */
cy_en_sysint_status_t  FramCmdWRDI(cy_en_smif_slave_select_t fram_slave_select,
		          SMIF_Type *baseaddr,
                  cy_stc_smif_context_t *smifContext,
                  uint8_t spimode);	   

/* Wtite 256-byte Special Sector */
cy_en_sysint_status_t  FramCmdSSWR(cy_en_smif_slave_select_t fram_slave_select,
		          SMIF_Type *baseaddr,
                  cy_stc_smif_context_t *smifContext, 
                  uint8_t tst_txBuffer[], 
                  uint32_t txSize, 
                  uint8_t *address,
                  uint8_t spimode);

 /* Read 256-byte Special Sector */
 cy_en_sysint_status_t  FramCmdSSRD(cy_en_smif_slave_select_t fram_slave_select,
		           SMIF_Type *baseaddr,
                   cy_stc_smif_context_t *smifContext, 
                   uint8_t tst_rxBuffer[], 
                   uint32_t rxSize, 
                   uint8_t *address,
                   uint8_t spimode,
                   uint8_t latency);

/* Wrire 8-byte Serial Number */
 cy_en_sysint_status_t  FramCmdWRSN(cy_en_smif_slave_select_t fram_slave_select,
		          SMIF_Type *baseaddr,
                  cy_stc_smif_context_t *smifContext, 
                  uint8_t cmdParam[], 	
                  uint32_t cmdSize,
                  uint8_t spimode);

/* Read 8-byte Serial Number */
 cy_en_sysint_status_t  FramCmdRDSN(cy_en_smif_slave_select_t fram_slave_select,
		          SMIF_Type *baseaddr,
                  cy_stc_smif_context_t *smifContext,
                  uint8_t tst_rxBuffer[],
                  uint32_t txSize,
                  uint8_t spimode,
                  uint8_t latency);    

 /* Read 8-byte Device ID */
 cy_en_sysint_status_t  FramCmdRDID(cy_en_smif_slave_select_t fram_slave_select,
		          SMIF_Type *baseaddr,
                  cy_stc_smif_context_t *smifContext,
                  uint8_t tst_rxBuffer[], 
                  uint8_t spimode,
                  uint8_t latency);

/* Read 8-byte Unique Device ID */
 cy_en_sysint_status_t  FramCmdRDUID(cy_en_smif_slave_select_t fram_slave_select,
		           SMIF_Type *baseaddr,
                   cy_stc_smif_context_t *smifContext,
                   uint8_t tst_rxBuffer[], 
                   uint8_t spimode,
                   uint8_t latency);

 /* Enter low power modes (DPD and HIBNET) */
 cy_en_sysint_status_t  FramCmdEnterLPMode(cy_en_smif_slave_select_t fram_slave_select,
		          SMIF_Type *baseaddr,
                  cy_stc_smif_context_t *smifContext,
                  uint8_t powermode, 
                  uint8_t spimode);	

#endif //QSPI_FRAM_APIS_H
    
/* [] END OF FILE */
