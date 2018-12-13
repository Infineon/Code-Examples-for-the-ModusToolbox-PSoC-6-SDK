/***************************************************************************//**
* \file transport_uart.h
* \version 3.0
*
* This file provides constants and parameter values of the DFU
* communication APIs for the SCB Component.
*
* Note that component name has to be UART.
*
********************************************************************************
* \copyright
* Copyright 2016-2018, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(TRANSPORT_UART_H)
#define TRANSPORT_UART_H

#include <stdint.h>
#include "cy_dfu.h"

/***************************************
*        Function Prototypes
***************************************/

/* UART DFU physical layer functions */
void UART_UartCyBtldrCommStart(void);
void UART_UartCyBtldrCommStop (void);
void UART_UartCyBtldrCommReset(void);
cy_en_dfu_status_t UART_UartCyBtldrCommRead (uint8_t pData[], uint32_t size, uint32_t *count, uint32_t timeout);
cy_en_dfu_status_t UART_UartCyBtldrCommWrite(uint8_t pData[], uint32_t size, uint32_t *count, uint32_t timeout);


#endif /* !defined(TRANSPORT_UART_H) */


/* [] END OF FILE */
