/******************************************************************************
* File Name: DmaUartLib.h
*
* Version: 1.0
*
* Description: This file contains all the functions and variables required for
* 			   proper operation of UART/DMA for this CE
*
*******************************************************************************
* Copyright (2018-2019), Cypress Semiconductor Corporation. All rights reserved.
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
#include "UartDma.h"

/*******************************************************************************
*            Forward declaration
*******************************************************************************/
void handle_error(void);

/*******************************************************************************
*            Global variables
*******************************************************************************/
extern uint8_t rx_dma_error;   /* RxDma error flag */
extern uint8_t tx_dma_error;   /* TxDma error flag */
extern uint8_t rx_dma_done;    /* RxDma done flag */

/*******************************************************************************
* Function Name: ConfigureRxDma
********************************************************************************
*
* Summary:
* Configures DMA Rx channel for operation.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void ConfigureRxDma(uint8_t* BufferA, uint8_t* BufferB, cy_stc_sysint_t* intConfig)
{
	cy_en_dma_status_t dma_init_status;

    /* Initialize descriptor 1 */
    dma_init_status = Cy_DMA_Descriptor_Init(&RxDma_Descriptor_0, &RxDma_Descriptor_0_config);
	if(dma_init_status!=CY_DMA_SUCCESS)
    {
        handle_error();
    }

    /* Initialize descriptor 2 */
    dma_init_status = Cy_DMA_Descriptor_Init(&RxDma_Descriptor_1, &RxDma_Descriptor_1_config);
	if(dma_init_status!=CY_DMA_SUCCESS)
    {
        handle_error();
    }

    dma_init_status = Cy_DMA_Channel_Init(RxDma_HW, RxDma_CHANNEL, &RxDma_channelConfig);
	if(dma_init_status!=CY_DMA_SUCCESS)
    {
        handle_error();
    }

    /* Set source and destination address for descriptor 1 */
    Cy_DMA_Descriptor_SetSrcAddress(&RxDma_Descriptor_0, (uint32_t *) &KIT_UART_HW->RX_FIFO_RD);
    Cy_DMA_Descriptor_SetDstAddress(&RxDma_Descriptor_0, (uint32_t *) BufferA);

    /* Set source and destination address for descriptor 2 */
    Cy_DMA_Descriptor_SetSrcAddress(&RxDma_Descriptor_1, (uint32_t *) &KIT_UART_HW->RX_FIFO_RD);
    Cy_DMA_Descriptor_SetDstAddress(&RxDma_Descriptor_1, (uint32_t *) BufferB);

    Cy_DMA_Channel_SetDescriptor(RxDma_HW, RxDma_CHANNEL, &RxDma_Descriptor_0);

    /* Initialize and enable interrupt from RxDma */
    Cy_SysInt_Init  (intConfig, &RxDmaComplete);
    NVIC_EnableIRQ(intConfig->intrSrc);

    /* Enable DMA interrupt source. */
    Cy_DMA_Channel_SetInterruptMask(RxDma_HW, RxDma_CHANNEL, CY_DMA_INTR_MASK);

    /* Enable channel and DMA block to start descriptor execution process */
    Cy_DMA_Channel_Enable(RxDma_HW, RxDma_CHANNEL);
    Cy_DMA_Enable(RxDma_HW);
}

/*******************************************************************************
* Function Name: ConfigureTxDma
********************************************************************************
*
* Summary:
* Configures DMA Tx channel for operation.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void ConfigureTxDma(uint8_t* BufferA, cy_stc_sysint_t* intConfig)
{
    cy_en_dma_status_t dma_init_status;

    /* Init descriptor */
    dma_init_status = Cy_DMA_Descriptor_Init(&TxDma_Descriptor_0, &TxDma_Descriptor_0_config);
	if(dma_init_status!=CY_DMA_SUCCESS)
    {
        handle_error();
    }
    dma_init_status = Cy_DMA_Channel_Init(TxDma_HW, TxDma_CHANNEL, &TxDma_channelConfig);
	if(dma_init_status!=CY_DMA_SUCCESS)
    {
        handle_error();
    }

    /* Set source and destination for descriptor 1 */
    Cy_DMA_Descriptor_SetSrcAddress(&TxDma_Descriptor_0, (uint32_t *) BufferA);
    Cy_DMA_Descriptor_SetDstAddress(&TxDma_Descriptor_0, (uint32_t *) &KIT_UART_HW->TX_FIFO_WR);

    /* Set next descriptor to NULL to stop the chain execution after descriptor 1
    *  is completed.
    */
    Cy_DMA_Descriptor_SetNextDescriptor(Cy_DMA_Channel_GetCurrentDescriptor(TxDma_HW, TxDma_CHANNEL), NULL);

    /* Initialize and enable the interrupt from TxDma */
    Cy_SysInt_Init  (intConfig, &TxDmaComplete);
    NVIC_EnableIRQ(intConfig->intrSrc);

    /* Enable DMA interrupt source */
    Cy_DMA_Channel_SetInterruptMask(TxDma_HW, TxDma_CHANNEL, CY_DMA_INTR_MASK);

    /* Enable Data Write block but keep channel disabled to not trigger
    *  descriptor execution because TX FIFO is empty and SCB keeps active level
    *  for DMA.
    */
    Cy_DMA_Enable(TxDma_HW);
}


/*******************************************************************************
* Function Name: RxDmaComplete
********************************************************************************
*
* Summary:
*  Handles Rx Dma descriptor completion interrupt source: triggers Tx Dma to
*  transfer back data received by the Rx Dma descriptor.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void RxDmaComplete(void)
{
    Cy_DMA_Channel_ClearInterrupt(RxDma_HW, RxDma_CHANNEL);

    /* Check interrupt cause to capture errors. */
    if (CY_DMA_INTR_CAUSE_COMPLETION == Cy_DMA_Channel_GetStatus(RxDma_HW, RxDma_CHANNEL))
    {
		rx_dma_done = 1;
	}
	else
	{
		/* DMA error occurred while RX operations */
		rx_dma_error = 1;
	}
}


/*******************************************************************************
* Function Name: TxDmaComplete
********************************************************************************
*
* Summary:
*  Handles Tx Dma descriptor completion interrupt source: only used for
*  indication.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void TxDmaComplete(void)
{
    /* Check interrupt cause to capture errors.
    *  Note that next descriptor is NULL to stop descriptor execution */
    if ((CY_DMA_INTR_CAUSE_COMPLETION    != Cy_DMA_Channel_GetStatus(TxDma_HW, TxDma_CHANNEL)) &&
        (CY_DMA_INTR_CAUSE_CURR_PTR_NULL != Cy_DMA_Channel_GetStatus(TxDma_HW, TxDma_CHANNEL)))
    {
        /* DMA error occurred while TX operations */
        tx_dma_error = 1;
    }
	Cy_DMA_Channel_ClearInterrupt(TxDma_HW, TxDma_CHANNEL);
}

/* [] END OF FILE */





