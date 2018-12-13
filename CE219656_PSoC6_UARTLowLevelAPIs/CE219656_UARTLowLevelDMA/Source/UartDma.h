/******************************************************************************
* File Name: DmaUartLib.h
*
* Version: 1.0
*
* Description: This file contains all the function prototypes required for proper
* 			   operation of UART/DMA for this CE
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
#ifndef SOURCE_UARTDMA_H_
#define SOURCE_UARTDMA_H_

#include "cy_pdl.h"
#include "cycfg.h"

/*******************************************************************************
*            Constants
*******************************************************************************/
#define DMA_DESCR0       (0u)
#define DMA_DESCR1       (1u)
#define BUFFER_SIZE      (1u)

/*******************************************************************************
*        Function Prototypes
*******************************************************************************/
void ConfigureTxDma(uint8_t*, cy_stc_sysint_t* );
void ConfigureRxDma(uint8_t*, uint8_t*, cy_stc_sysint_t*);
void RxDmaComplete(void);
void TxDmaComplete(void);

#endif /* SOURCE_UARTDMA_H_ */
