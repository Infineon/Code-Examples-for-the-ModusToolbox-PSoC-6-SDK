/******************************************************************************
* \file main.c
*
* \version 1.0
*
* \brief
* Objective:
*     This code example demonstrates the basic operation of the I2C slave
*     (SCB mode) using polling method. The I2C slave accepts a packet from
*     the master with a command to control the LED brightness using PWM signal.
*     A status packet is sent back to master.
*
* Compatible Kits:
*    CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*    CY8CKIT-062-WIFI-BT PSoC 6 WiFi-BT Kit
*    CY8CPROTO-062-4343W PSoC 6 WiFi-BT Prototyping Kit
*
* Related Document: CE221119.pdf
*
*******************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
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
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_device_headers.h"
#include "cycfg.h"
#include "cy_scb_i2c.h"
#include "cy_sysint.h"
#include "cy_tcpwm_counter.h"
#include "stdbool.h"

/************************** Macro Definitions *********************************/

/* Valid command packet size of three bytes */
#define PACKET_SIZE          (0x03u)

/* Master write and read buffer of size three bytes */
#define SL_RD_BUFFER_SIZE    (PACKET_SIZE)
#define SL_WR_BUFFER_SIZE    (PACKET_SIZE)

/* Start and end of packet markers */
#define PACKET_SOP           (0x01u)
#define PACKET_EOP           (0x17u)

/* Command valid status */
#define STS_CMD_DONE         (0x00u)
#define STS_CMD_FAIL         (0xFFu)

/* Packet positions */
#define PACKET_SOP_POS       (0x00u)
#define PACKET_STS_POS       (0x01u)
#define PACKET_LED_POS       (0x01u)
#define PACKET_EOP_POS       (0x02u)

/******************************************************************************/

/*********************** Function prototypes **********************************/

void sI2C_InterruptHandler(void);
static void pollI2CwriteBuffer(void);
static void ExecuteCommand(void);
static void InitEnablePWM(void);
static void HandleError(void);

/******************************************************************************/

/************************ Variables Declaration *******************************/

/** The instance-specific context structure.
* It is used by the driver for internal configuration and
* data keeping for the I2C. Do not modify anything in this structure.
*/
cy_stc_scb_i2c_context_t KIT_I2C_context;

/* KIT_I2C_SCB_IRQ */
const cy_stc_sysint_t KIT_I2C_SCB_IRQ_config = {
    .intrSrc = (IRQn_Type) KIT_I2C_IRQ,
    .intrPriority = 7u
};

/* I2C read and write buffers */
uint8_t i2cReadBuffer [SL_RD_BUFFER_SIZE] = {PACKET_SOP, STS_CMD_FAIL, PACKET_EOP};
uint8_t i2cWriteBuffer[SL_WR_BUFFER_SIZE] ;

bool errorDetected = false;
/******************************************************************************/

/*******************************************************************************
* Function Name: main
********************************************************************************
*
*  The main function performs the following actions:
*   1. Sets up I2C to be in I2C slave mode.
*   2. If initialization of I2C fails, system will be in infinite loop.
*   3. Initializes PWM to control the LED. If initialization of PWM fails,
*      system will be in infinite loop.
*   4. I2C slave receives packets from master and configures the PWM to
*      drive the LED.
*   5. Slave responds with the acknowledgment packet.
*
*******************************************************************************/

int main(void)
{
    cy_en_scb_i2c_status_t initI2Cstatus;
    cy_en_sysint_status_t sysI2Cstatus;

    /* Set up the device based on configurator selections */
    init_cycfg_all();

    /* Initialize and enable TCPWM Components */
    InitEnablePWM();

    /* Initialize and enable I2C Component in slave mode. If initialization fails process error */
    initI2Cstatus = Cy_SCB_I2C_Init(KIT_I2C_HW, &KIT_I2C_config, &KIT_I2C_context);
    if(initI2Cstatus != CY_SCB_I2C_SUCCESS)
    {
        HandleError();
    }

    sysI2Cstatus = Cy_SysInt_Init(&KIT_I2C_SCB_IRQ_config, &sI2C_InterruptHandler);
    if(sysI2Cstatus != CY_SYSINT_SUCCESS)
    {
        HandleError();
    }

    /* Configure read buffer */
    Cy_SCB_I2C_SlaveConfigReadBuf(KIT_I2C_HW, i2cReadBuffer, SL_RD_BUFFER_SIZE, &KIT_I2C_context);

    /* Configure write buffer */
    Cy_SCB_I2C_SlaveConfigWriteBuf(KIT_I2C_HW, i2cWriteBuffer, SL_WR_BUFFER_SIZE, &KIT_I2C_context);

    /*  Enable interrupt and I2C block */
    NVIC_EnableIRQ((IRQn_Type) KIT_I2C_SCB_IRQ_config.intrSrc);
    Cy_SCB_I2C_Enable(KIT_I2C_HW);

    /* Enable global interrupts */
    __enable_irq();


    for(;;)
    {
        /* continuously poll to check whether master has written data to write buffer. */
        pollI2CwriteBuffer();

        /* Poll every 1ms */
        Cy_SysLib_Delay(CY_SCB_WAIT_1_UNIT);
    }

    return (0);
}

/*******************************************************************************
* Function Name: sI2C_InterruptHandler
********************************************************************************
* Summary:
*  This function executes interrupt service routine.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*
*******************************************************************************/
void sI2C_InterruptHandler(void)
{
    /* ISR implementation for I2C */
    Cy_SCB_I2C_Interrupt(KIT_I2C_HW, &KIT_I2C_context);
}

/*******************************************************************************
* Function Name: pollI2CwriteBuffer
********************************************************************************
*
* Checks the status of slave to know if any data is written by master into write
* buffer. If data is written, check for error. If no error while writing
* then check whether required number of bytes is written. If required number of
* bytes are written, if start and end packets are correct set the TCPWM compare
* value to the respective RGB LED TCPWM's to control the color.
*
* \param None
*
* \return
*  None
*
*******************************************************************************/
static void pollI2CwriteBuffer(void)
{
    if( 0UL == (Cy_SCB_I2C_SlaveGetStatus(KIT_I2C_HW, &KIT_I2C_context) &  CY_SCB_I2C_SLAVE_BUS_ERR))
    {
        if( 0UL != (Cy_SCB_I2C_SlaveGetStatus(KIT_I2C_HW, &KIT_I2C_context) &  CY_SCB_I2C_SLAVE_WR_CMPLT))
        {
            if (PACKET_SIZE ==  Cy_SCB_I2C_SlaveGetWriteTransferCount(KIT_I2C_HW, &KIT_I2C_context))
            {
                /* Check start and end of packet markers. */
                if ((i2cWriteBuffer[PACKET_SOP_POS] == PACKET_SOP) &&
                    (i2cWriteBuffer[PACKET_EOP_POS] == PACKET_EOP))
                {
                    /* Update reply status for received commend. */
                    ExecuteCommand( );
                    i2cReadBuffer[PACKET_STS_POS] = STS_CMD_DONE;
                }
            }
        }
        /* Configure write buffer for the next write. */
        Cy_SCB_I2C_SlaveClearWriteStatus(KIT_I2C_HW, &KIT_I2C_context);
        Cy_SCB_I2C_SlaveConfigWriteBuf(KIT_I2C_HW, i2cWriteBuffer, SL_WR_BUFFER_SIZE, &KIT_I2C_context);
    }
    else
    {
        HandleError();
    }

     /* Check read complete event. */
    if(0UL != (Cy_SCB_I2C_SlaveGetStatus(KIT_I2C_HW, &KIT_I2C_context) &  CY_SCB_I2C_SLAVE_RD_CMPLT))
    {
        /* Configure read buffer for next read. */
         /* Clear read status flags. */
        Cy_SCB_I2C_SlaveClearReadStatus(KIT_I2C_HW, &KIT_I2C_context);

        /* Configure read buffer for the next read. */
        i2cReadBuffer[PACKET_STS_POS] = STS_CMD_FAIL;
        Cy_SCB_I2C_SlaveConfigReadBuf(KIT_I2C_HW, i2cReadBuffer, SL_RD_BUFFER_SIZE, &KIT_I2C_context);
    }
}

/*******************************************************************************
* Function Name: ExecuteCommand
********************************************************************************
*
* Sets the compare value for the LED PWM received by slave.
*
* \param None
*
* \return
*  None
*
*******************************************************************************/
static void ExecuteCommand(void)
{
    /* Sets the compare value to control the brightness of theLED. */
    Cy_TCPWM_PWM_SetCompare0(KIT_LED2_PWM_HW, KIT_LED2_PWM_NUM, i2cWriteBuffer[PACKET_LED_POS]);
}

/*******************************************************************************
* Function Name: InitEnablePWM
********************************************************************************
*
* This function initializes and enables the TCPWM for controlling RGB LED.
*
* \param None
* \note
*
* \return
*  None
*
*******************************************************************************/
static void InitEnablePWM(void)
{
    cy_en_tcpwm_status_t initPWMstatus;

    /* Initialize and enables TCPWM to control red LED. If initialization fails process error.  */
    initPWMstatus = Cy_TCPWM_PWM_Init(KIT_LED2_PWM_HW, KIT_LED2_PWM_NUM, &KIT_LED2_PWM_config);
    if(initPWMstatus != CY_TCPWM_SUCCESS)
    {
        HandleError();
    }
    Cy_TCPWM_Enable_Multiple(KIT_LED2_PWM_HW, KIT_LED2_PWM_MASK);
    Cy_TCPWM_TriggerStart(KIT_LED2_PWM_HW, KIT_LED2_PWM_MASK);

 }

/*******************************************************************************
* Function Name: HandleError
********************************************************************************
*
* This function processes unrecoverable errors such as any
* initialization errors etc. In case of such error the system will
* stay in the infinite loop of this function.
*
*
* \note
* * If error occurs interrupts are disabled.
*
*******************************************************************************/
static void HandleError(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    /* Infinite loop. */
    while(1u) {}
}
