/***************************************************************************//**
* \file main.c
* \version 1.0
*
* \brief
This example demonstrates how a PSoC® 6 DMA channel transfers data received 
from the UART to a buffer in memory. When the buffer is filled, a second DMA 
channel drains the buffer to the UART, to be echoed back.
*
* Compatible Kits:
*	CY8CKPROTO-062-4343W
*   
*
* Migration to CY8CKIT-062-BLE and CY8CKIT-062-WIFI-BT kit:
*   Open device configuration and replicate the configuration of 
*	DMA0 Channel 26 and 27 to DMA0 Channel 0 and 1 respectively. 
*
********************************************************************************
* \copyright
* Copyright 2017-2018, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "cy_pdl.h"
#include "cycfg.h"


/***************************************
*        Function Prototypes
***************************************/

void ConfigureTxDma(void);
void ConfigureRxDma(void);

void RxDmaCmplt(void);

/***************************************
*            Constants
****************************************/

#define BUFFER_SIZE         (10u)
#define INTERRUPT_PRIORITY	 (7u)

uint8_t Buffer[BUFFER_SIZE];

cy_stc_scb_uart_context_t UART_context;
const cy_stc_sysint_t UART_SCB_IRQ_cfg = {
    .intrSrc = (IRQn_Type)UART_IRQ,
    .intrPriority = INTERRUPT_PRIORITY
};

const cy_stc_sysint_t RxDMA_Cmplt_cfg = {
    .intrSrc = (IRQn_Type)RxDMA_IRQ,
    .intrPriority = INTERRUPT_PRIORITY
};


/* UART interrupt function */
void UART_Interrupt(void)
{
    Cy_SCB_UART_Interrupt(UART_HW, &UART_context);
}

int main(void)
{
    /* Set up internal routing, pins, and clock-to-peripheral connections */
    init_cycfg_all();
    
    /* enable interrupts */
    __enable_irq();

    /* Configure UART. */
    /* Initialize UART with config set by configurator */
    Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);
 
 /* Initialize and enable the UART interrupts */
    Cy_SysInt_Init(&UART_SCB_IRQ_cfg, &UART_Interrupt);
    NVIC_EnableIRQ(UART_SCB_IRQ_cfg.intrSrc);

    /* Enable the SCB block that implements the UART */
    Cy_SCB_UART_Enable(UART_HW);

    /* Print a message on UART */
    Cy_SCB_UART_PutString(UART_HW, "UART to Memory Buffer using DMA code example project\n");
    Cy_SCB_UART_PutString(UART_HW, "Transmit 10 characters to see an echo in the terminal.\n");

    /* Configures DMA Rx and Tx channels for operation. */
    ConfigureTxDma();
    ConfigureRxDma();

    /* Initialize and enable the RxDMA interrupt*/
    Cy_SysInt_Init  (&RxDMA_Cmplt_cfg, &RxDmaCmplt);
    NVIC_EnableIRQ(RxDMA_Cmplt_cfg.intrSrc);

    for(;;)
    {

    }
}

/******************************************************************************
* Function Name: ConfigureRxDma
********************************************************************************
*
* Configures RxDMA channel for operation. This channel has a descriptor that is
* configured to transfer 1 byte at a time when it becomes available at the UART
* RxFIFO. The Descriptor is configured to tranfer 10 bytes. When 10 byte transfer
* is completed, the RxDMA_Cmplt interrupt is triggered.
*
******************************************************************************/
void ConfigureRxDma(void)
{

/*  set the source and destination of transfer in the descriptor config structure */
    RxDMA_Descriptor_0_config.srcAddress = (uint32_t *) &UART_HW->RX_FIFO_RD;
    RxDMA_Descriptor_0_config.dstAddress = Buffer;

/*  Initialize the Descriptor for the RxDMA channel */
    Cy_DMA_Descriptor_Init(&RxDMA_Descriptor_0, &RxDMA_Descriptor_0_config);

/*  Initialize the DMA channel */
    Cy_DMA_Channel_Init(RxDMA_HW, RxDMA_CHANNEL, &RxDMA_channelConfig);

/*  Enables the DMA channel interrupt mask */
    Cy_DMA_Channel_SetInterruptMask(RxDMA_HW, RxDMA_CHANNEL, CY_DMA_INTR_MASK);

/*  Enables DMA channel */
    Cy_DMA_Channel_Enable(RxDMA_HW, RxDMA_CHANNEL);

/*  Enable DMA hardware block */
    Cy_DMA_Enable(RxDMA_HW);
}


/*******************************************************************************
* Function Name: ConfigureTxDma
********************************************************************************
*
* Configures TxDMA which handles transmitting 10 bytes of data from the Buffer to
* UART TxFIFO. At the end of the transfer the TxDMA is configured to be disabled.
*
*
*******************************************************************************/
void ConfigureTxDma(void)
{


    /* set the source and destination of transfer in the descriptor config structure */
    TxDMA_Descriptor_0_config.srcAddress = Buffer;
    TxDMA_Descriptor_0_config.dstAddress = (uint32_t *) &UART_HW->TX_FIFO_WR;

    /* Initialize the Descriptor for the DMA channel */
    Cy_DMA_Descriptor_Init(&TxDMA_Descriptor_0, &TxDMA_Descriptor_0_config);

    /* Initialize the DMA channel */
    Cy_DMA_Channel_Init(TxDMA_HW, TxDMA_CHANNEL, &TxDMA_channelConfig);

}


/******************************************************************************
* Function Name: RxDmaCmplt
********************************************************************************
*
* Interrupt that is triggered at the completion of the RxDMA transfer.
* This interrupt reconfigures TxDMA for another 10 byte transfer
*
******************************************************************************/
void RxDmaCmplt(void)
{
/*  Clears the interupt source in RxDMA channel */
    Cy_DMA_Channel_ClearInterrupt(RxDMA_HW, RxDMA_CHANNEL);

	/*  Reconfigure the descriptor for the TxDMA. */
    Cy_DMA_Channel_SetDescriptor(TxDMA_HW, TxDMA_CHANNEL, &TxDMA_Descriptor_0);

/*  Enable the TxDMA channel. This channel is disabled automatically at the end
    of the transfer
*/
    Cy_DMA_Channel_Enable(TxDMA_HW, TxDMA_CHANNEL);
}

