/******************************************************************************
* File Name: smif_mem.h
*
* Version: 2.0
*
* Description:
* 	This is the public interface header for smif_mem.c. This file contains the
* 	functions to perform memory operations such as read, write, and erase on an
* 	external memory device.
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

#ifndef SOURCE_SMIF_MEM_H
#define SOURCE_SMIF_MEM_H

#include "cy_pdl.h"


/***************************************************************************
* Global Constants
***************************************************************************/
/* Set it high enough for the sector erase operation to complete */
#define MEMORY_BUSY_CHECK_RETRIES	(750ul)

/* Timeout used in polling of the transfer status of SMIF block */
#define SMIF_TRANSFER_TIMEOUT		(1000ul) 	/* microseconds */


/***************************************************************************
* Function Prototypes
***************************************************************************/
cy_en_smif_status_t IsMemoryReady(cy_stc_smif_mem_config_t const *memConfig);
cy_en_smif_status_t IsQuadEnabled(cy_stc_smif_mem_config_t const *memConfig, bool *isQuadEnabled);
cy_en_smif_status_t EnableQuadMode(cy_stc_smif_mem_config_t const *memConfig);
cy_en_smif_status_t ReadMemory(cy_stc_smif_mem_config_t const *memConfig, uint8_t address[], uint8_t rxBuffer[], uint32_t rxSize);
cy_en_smif_status_t WriteMemory(cy_stc_smif_mem_config_t const *memConfig, uint8_t address[], uint8_t txBuffer[], uint32_t txSize);
cy_en_smif_status_t EraseMemory(cy_stc_smif_mem_config_t const *memConfig, uint8_t address[]);

#endif /* SOURCE_SMIF_MEM_H */


