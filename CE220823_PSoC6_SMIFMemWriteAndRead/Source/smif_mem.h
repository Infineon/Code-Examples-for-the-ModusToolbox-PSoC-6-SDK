/******************************************************************************
* File Name: smif_mem.c
*
* Version: 1.0
*
* Description: This file contains all the function prototypes required for
* 			   proper operation of QSPI component
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
#ifndef SOURCE_SMIF_MEM_H_
#define SOURCE_SMIF_MEM_H_

#include "cy_pdl.h"
#include "cycfg.h"
#include "cycfg_qspi_memslot.h"

#include <stdio.h>
#include <string.h>

/***************************************************************************
* Function Prototypes
***************************************************************************/
void CheckStatus(char * msg, uint32_t status);
void ReadMemory(SMIF_Type *baseaddr,
                            cy_stc_smif_context_t *smifContext,
                            uint8_t rxBuffer[],
                            uint32_t rxSize,
                            uint8_t *address);
void WriteMemory(SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext,
                    uint8_t txBuffer[],
                    uint32_t txSize,
                    uint8_t *address);
void EraseMemory(SMIF_Type *baseaddr, cy_stc_smif_mem_config_t *memConfig,
                 uint8_t *address,
                 cy_stc_smif_context_t const *smifContext);

#endif /* SOURCE_SMIF_MEM_H_ */
