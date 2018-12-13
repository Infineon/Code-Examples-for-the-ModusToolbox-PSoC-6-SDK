/******************************************************************************
* File Name: SpiDma.c
*
* Version: 1.0
*
* Description: This file contains function definitions for DMA operation.
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
#include "SpiDma.h"
#include "Interface.h"

uint32_t txDmaDone=0;

void handle_error(void);

uint32_t ConfigureTxDma(uint32_t* txBuffer)
 {
     cy_en_dma_status_t dma_init_status;
     const cy_stc_sysint_t intTxDma_cfg =
     {
         .intrSrc      = txDma_IRQ,
         .intrPriority = 7u
     };
     /* Initialize descriptor */
     dma_init_status = Cy_DMA_Descriptor_Init(&txDma_Descriptor_0, &txDma_Descriptor_0_config);
     if(dma_init_status!=CY_DMA_SUCCESS)
     {
         return INIT_FAILURE;
     }

     dma_init_status = Cy_DMA_Channel_Init(txDma_HW, txDma_CHANNEL, &txDma_channelConfig);
     if(dma_init_status!=CY_DMA_SUCCESS)
     {
    	 return INIT_FAILURE;
     }

     /* Set source and destination for descriptor 1 */
     Cy_DMA_Descriptor_SetSrcAddress(&txDma_Descriptor_0, (uint8_t *)txBuffer);
     Cy_DMA_Descriptor_SetDstAddress(&txDma_Descriptor_0, (void *)&mSPI_HW->TX_FIFO_WR);

      /* Initialize and enable the interrupt from TxDma */
     Cy_SysInt_Init(&intTxDma_cfg, &TxDmaComplete);
     NVIC_EnableIRQ((IRQn_Type)intTxDma_cfg.intrSrc);

      /* Enable DMA interrupt source. */
     Cy_DMA_Channel_SetInterruptMask(txDma_HW, txDma_CHANNEL, CY_DMA_INTR_MASK);

     Cy_DMA_Enable(txDma_HW);
     return INIT_SUCCESS;
 }

void TxDmaComplete(void)
 {
     /* Check tx DMA status */
     if ((CY_DMA_INTR_CAUSE_COMPLETION    != Cy_DMA_Channel_GetStatus(txDma_HW, txDma_CHANNEL)) &&
         (CY_DMA_INTR_CAUSE_CURR_PTR_NULL != Cy_DMA_Channel_GetStatus(txDma_HW, txDma_CHANNEL)))
     {
         /* DMA error occurred while TX operations */
         handle_error();
     }
     txDmaDone=1;
     /* Clear tx DMA interrupt */
     Cy_DMA_Channel_ClearInterrupt(txDma_HW, txDma_CHANNEL);

 }


