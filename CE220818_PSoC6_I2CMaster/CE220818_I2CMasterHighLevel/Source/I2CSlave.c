/******************************************************************************
* File Name: I2CSlave.c
*
* Version: 1.0
*
* Description: This file contains all the functions and variables required for proper
* 			   operation of I2C slave SCB
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
#include "I2CSlave.h"
#include "Interface.h"
/*******************************************************************************
* Global constants
*******************************************************************************/
/* Buffer and packet size */
#define PACKET_SIZE          (3UL)

/* Buffer and packet size in the slave*/
#define SL_RD_BUFFER_SIZE    (03UL)
#define SL_WR_BUFFER_SIZE    (PACKET_SIZE)

/* Start and end of packet markers */
#define PACKET_SOP           (0x01UL)
#define PACKET_EOP           (0x17UL)

/* Command valid status */
#define STS_CMD_DONE         (0x00UL)
#define STS_CMD_FAIL         (0xFFUL)

/* Packet positions */
#define PACKET_SOP_POS       (0UL)
#define PACKET_STS_POS       (1UL)
#define PACKET_CMD_POS       (1UL)
#define PACKET_EOP_POS       (2UL)

#define ZERO                 (0UL)

/*******************************************************************************
* Global variables
*******************************************************************************/
/* I2C slave read and write buffers. */
uint8_t i2cReadBuffer [SL_RD_BUFFER_SIZE] = {PACKET_SOP, STS_CMD_FAIL, PACKET_EOP};
uint8_t i2cWriteBuffer[SL_WR_BUFFER_SIZE] ;
cy_stc_scb_i2c_context_t sI2C_context;

/*******************************************************************************
* Forward declaration
*******************************************************************************/
void SlaveExecuteCommand(void);
void HandleEventsSlave(uint32 event);
void sI2C_Interrupt(void);

/*******************************************************************************
* Function Name: HandleEventsSlave
****************************************************************************//**
*
* Handles slave events write and read completion events.
*
* \param event
* reports slave events.
* ref uint32_t
*
* \return
*  None
*
*******************************************************************************/
void HandleEventsSlave(uint32_t event)
{
    /* Check write complete event. */
    if (0UL != (CY_SCB_I2C_SLAVE_WR_CMPLT_EVENT & event))
    {

        /* Check for errors */
        if (0UL == (CY_SCB_I2C_SLAVE_ERR_EVENT & event))
        {
            /* Check packet length */
            if (PACKET_SIZE ==  Cy_SCB_I2C_SlaveGetWriteTransferCount(sI2C_HW, &sI2C_context))
            {
                /* Check start and end of packet markers. */
                if ((i2cWriteBuffer[PACKET_SOP_POS] == PACKET_SOP) &&
                    (i2cWriteBuffer[PACKET_EOP_POS] == PACKET_EOP))
                {
                    SlaveExecuteCommand();
                }
            }
        }

        /* Update status of received commend. */
        i2cReadBuffer[PACKET_STS_POS] = STS_CMD_DONE;

        /* Configure write buffer for the next write. */
        i2cWriteBuffer[PACKET_SOP_POS] = ZERO;
        i2cWriteBuffer[PACKET_EOP_POS] = ZERO;
        Cy_SCB_I2C_SlaveConfigWriteBuf(sI2C_HW, i2cWriteBuffer, SL_WR_BUFFER_SIZE, &sI2C_context);
    }

    /* Check write complete event. */
    if (0UL != (CY_SCB_I2C_SLAVE_RD_CMPLT_EVENT & event))
    {
        /* Configure read buffer for the next read. */
        i2cReadBuffer[PACKET_STS_POS] = STS_CMD_FAIL;
        Cy_SCB_I2C_SlaveConfigReadBuf(sI2C_HW, i2cReadBuffer, SL_RD_BUFFER_SIZE, &sI2C_context);
    }
}


/*******************************************************************************
* Function Name: SlaveExecuteCommand
****************************************************************************//**
*
*  Executes received command to control the LED state.
*
* \param None
*
* \return
*  None
*
*******************************************************************************/
void SlaveExecuteCommand( void )
{
    Cy_GPIO_Write(KIT_LED2_PORT, KIT_LED2_NUM, i2cWriteBuffer[PACKET_CMD_POS]);
}


/*******************************************************************************
* Function Name: handle_error
****************************************************************************//**
*
* This function processes unrecoverable errors such as any component
* initialization errors etc. In case of such error the system will
* stay in the infinite loop of this function..
*
*
* \note
* * If error occurs interrupts are disabled.
*
*******************************************************************************/
void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    /* Infinite loop. */
    while(1u) {}
}

/*******************************************************************************
* Function Name: msI2C_Interrupt
****************************************************************************//**
*
* Invokes the Cy_SCB_I2C_Interrupt() PDL driver function.
*
*******************************************************************************/
inline void sI2C_Interrupt(void)
{
    Cy_SCB_I2C_Interrupt(sI2C_HW, &sI2C_context);
}

/*******************************************************************************
* Function Name: initSlave
********************************************************************************
*
* This function initiates and enables slave SCB
*
* \param None
*
* \return
* Status of initialization
*
*******************************************************************************/
uint32_t initSlave(void)
{
	cy_en_scb_i2c_status_t initI2Cstatus;
	cy_en_sysint_status_t sysI2Cstatus;
	cy_stc_sysint_t sI2C_SCB_IRQ_cfg =
	{
			/*.intrSrc =*/ sI2C_IRQ,
			/*.intrPriority =*/ 2UL
	};
	/* Initialize the reply status packet. */
	i2cReadBuffer[PACKET_STS_POS] = STS_CMD_FAIL;

	/* Initialize and enable I2C component in slave mode. */
	initI2Cstatus = Cy_SCB_I2C_Init(sI2C_HW, &sI2C_config, &sI2C_context);
	if(initI2Cstatus != CY_SCB_I2C_SUCCESS)
	{
		return I2C_FAILURE;
	}
	sysI2Cstatus = Cy_SysInt_Init(&sI2C_SCB_IRQ_cfg, &sI2C_Interrupt);

	if(sysI2Cstatus != CY_SYSINT_SUCCESS)
	{
		return I2C_FAILURE;
	}
	Cy_SCB_I2C_SlaveConfigReadBuf(sI2C_HW, i2cReadBuffer, SL_RD_BUFFER_SIZE, &sI2C_context);
	Cy_SCB_I2C_SlaveConfigWriteBuf(sI2C_HW, i2cWriteBuffer, SL_WR_BUFFER_SIZE, &sI2C_context);
	Cy_SCB_I2C_RegisterEventCallback(sI2C_HW, (cy_cb_scb_i2c_handle_events_t) HandleEventsSlave, &sI2C_context);
	NVIC_EnableIRQ((IRQn_Type) sI2C_SCB_IRQ_cfg.intrSrc);
	Cy_SCB_I2C_Enable(sI2C_HW);
	return I2C_SUCCESS;
}

