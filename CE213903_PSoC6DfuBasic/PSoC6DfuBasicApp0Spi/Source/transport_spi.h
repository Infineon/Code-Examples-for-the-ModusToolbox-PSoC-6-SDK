/***************************************************************************//**
* \file transport_spi.h
* \version 3.0
*
* This file provides constants and parameter values of the DFU
* communication APIs for the SCB Component.
*
* Note that component name has to be SPI.
*
********************************************************************************
* \copyright
* Copyright 2016-2018, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(TRANSPORT_SPI_H)
#define TRANSPORT_SPI_H

#include "cy_dfu.h"

    
/***************************************
*        Function Prototypes
***************************************/

/* SPI DFU physical layer functions */
void SPI_SpiCyBtldrCommStart(void);
void SPI_SpiCyBtldrCommStop (void);
void SPI_SpiCyBtldrCommReset(void);
cy_en_dfu_status_t SPI_SpiCyBtldrCommRead (uint8_t pData[], uint32_t size, uint32_t *count, uint32_t timeout);
cy_en_dfu_status_t SPI_SpiCyBtldrCommWrite(const uint8_t pData[], uint32_t size, uint32_t *count, uint32_t timeout);


#endif /* !defined(TRANSPORT_SPI_H) */


/* [] END OF FILE */
