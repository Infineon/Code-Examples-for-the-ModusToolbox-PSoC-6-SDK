/*******************************************************************************
* File Name: i2cm_support.c
*
* Version 1.0
*
* Description: This file contains the functions for I2C read and write 
*              operations for interfacing with the BMI160 motion sensor.
*
* Related Document: CE223468_Interfacing_BMI160_FreeRTOS.pdf
*
* Hardware Dependency: See CE223468_Interfacing_BMI160_FreeRTOS.pdf
* 
*******************************************************************************
* Copyright (2019), Cypress Semiconductor Corporation. All rights reserved.
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

/* Header files include */ 
#include "i2cm_support.h"
#include "cycfg.h"
#include "cy_pdl.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* I2C buffer size */
#define I2C_BUFFER_SIZE (256u)

/* Semaphore handle */
SemaphoreHandle_t xSemaphoreI2C;

/* I2C context structure */
cy_stc_scb_i2c_context_t KIT_I2C_context;

/* I2C interrupt configuration structure */
const cy_stc_sysint_t KIT_I2C_SCB_IRQ_cfg =
{
	.intrSrc = scb_3_interrupt_IRQn,
	.intrPriority = 7UL
};

/*******************************************************************************
* Function Name: static inline void I2Cm_Interrupt(void)
********************************************************************************
* Summary:
*  I2C interrupt ISR
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static inline void I2C_Interrupt(void)
{
	/* Call interrupt function for the SCB configured in the I2C mode.
	 * The interrupt is mandatory for I2C operation */
	Cy_SCB_I2C_MasterInterrupt(KIT_I2C_HW, &KIT_I2C_context);
}

/*******************************************************************************
* Function Name: void I2C_Init(void)
********************************************************************************
* Summary:
*	Initialize and enable I2C and register event callback.
*
* Parameters:
*	None
*
* Return:
*	None
*
*******************************************************************************/
void I2C_Init(void)
{
	/* Start the I2C master interface for BMI160 motion sensor */
	Cy_SCB_I2C_Init(KIT_I2C_HW, &KIT_I2C_config, &KIT_I2C_context);

	Cy_SysInt_Init(&KIT_I2C_SCB_IRQ_cfg, &I2C_Interrupt);
	NVIC_ClearPendingIRQ( KIT_I2C_SCB_IRQ_cfg.intrSrc);
	NVIC_EnableIRQ(KIT_I2C_SCB_IRQ_cfg.intrSrc);

	Cy_SCB_I2C_Enable(KIT_I2C_HW);

	/* Register interrupt callback */
	Cy_SCB_I2C_RegisterEventCallback(KIT_I2C_HW, I2C_Callback, &KIT_I2C_context);
}


/*******************************************************************************
* Function Name: unsigned int I2C_WriteBytes(uint8_t slaveAddress,
* uint8_t registerAddress, uint8_t *registerValue, uint8_t registerLen)
********************************************************************************
* Summary:
*  Function that writes data to an I2C slave device 
*
* Parameters:
*  uint8_t slaveAddress		- I2C slave address
*  uint8_t registerAddress	- Register address to select gyro, accelerometer
*                                 data
*  uint8_t *registerValue 	- Pointer to data to be written to I2C slave
*  uint8_t registerLen   	- Length of sensor data in bytes
*
* Return:
*  unsigned long status		- status of I2C write operation
*
*******************************************************************************/
unsigned int I2C_WriteBytes(uint8_t slaveAddress, uint8_t registerAddress,
		uint8_t *registerValue, uint8_t registerLen)
{
    /* Variable used for status of I2C operation */
    unsigned int status;

    /* Temporary buffer used for I2C transfer */ 
    static uint8_t tempBuff[I2C_BUFFER_SIZE];
    
    if(registerValue == NULL)
    {
    	status = CY_SCB_I2C_BAD_PARAM;
    }
    else
    {
		tempBuff[0] = registerAddress;
		memcpy(tempBuff + 1, registerValue, registerLen);

		/* Local variables for storing I2C Master transfer configuration structure */
		cy_stc_scb_i2c_master_xfer_config_t xfer_config =
		{
			.slaveAddress = (uint32)slaveAddress,
			.buffer =  tempBuff,
			.bufferSize = registerLen + 1,
			.xferPending = false
		};

		/* Start I2C write and take the semaphore */
		status = (unsigned int)Cy_SCB_I2C_MasterWrite(
				KIT_I2C_HW, &xfer_config, &KIT_I2C_context);
		xSemaphoreTake(xSemaphoreI2C, portMAX_DELAY);
    }
    return status;
}

/*******************************************************************************
* Function Name: unsigned int I2C_ReadBytes(uint8_t Address,
* 	uint8_t RegisterAddr, uint8_t *RegisterValue, uint8_t RegisterLen)
********************************************************************************
* Summary:
*  Function that reads data from an I2C slave device 
*
* Parameters:
*  unsigned char Address        - I2C slave address
*  unsigned char RegisterAddr   - register address to select gyro, accelerometer
*                                 data
*  unsigned char *RegisterValue - pointer to sensor data
*  unsigned char  RegisterLen   - length of sensor data in bytes
* Return:
*  unsigned long status - status of I2C write operation
*
*******************************************************************************/ 
unsigned int I2C_ReadBytes(uint8_t address, uint8_t registerAddress,
		uint8_t *registerValue, uint8_t registerLen)
{
    /* Variable used for status of I2C operation */
    unsigned int status = CY_SCB_I2C_SUCCESS;

    /* Local variables for storing I2C Master transfer configuration structure */
    cy_stc_scb_i2c_master_xfer_config_t config;
    
    if(registerValue == NULL)
    {
    	status = CY_SCB_I2C_BAD_PARAM;
    }
    else
    {
		config.slaveAddress = (uint32)address;
		config.buffer       = &registerAddress;
		config.bufferSize   = 1;
		config.xferPending  = true;

		/* Start I2C write and take the semaphore */
		status = (unsigned int) Cy_SCB_I2C_MasterWrite(KIT_I2C_HW,
				&config, &KIT_I2C_context);
		xSemaphoreTake(xSemaphoreI2C, portMAX_DELAY);

		if(status == CY_SCB_I2C_SUCCESS)
		{
			config.slaveAddress = (uint32)address;
			config.buffer =  registerValue;
			config.bufferSize = registerLen;
			config.xferPending = false;

			/* Start I2C read and take the semaphore */
			status = (unsigned int)Cy_SCB_I2C_MasterRead(KIT_I2C_HW,
					&config, &KIT_I2C_context);
			xSemaphoreTake(xSemaphoreI2C, portMAX_DELAY);
		}
    }
    return status;
}

/*******************************************************************************
* Function Name: void I2C_Callback(uint32_t events)
********************************************************************************
*
* Summary:
*  Call back event function to handle interrupts from I2C
*
* Parameters:
*  uint32_t events: 
*   CY_SCB_I2C_MASTER_WR_IN_FIFO_EVENT - unused
*   CY_SCB_I2C_MASTER_WR_CMPLT_EVENT - I2C maser write completed
*   CY_SCB_I2C_MASTER_RD_CMPLT_EVENT - I2C master read completed
*   CY_SCB_I2C_MASTER_ERR_EVENT - I2C hardware error detected
*
* Return:
*  None
*
*******************************************************************************/
void I2C_Callback(uint32_t events)
{
    BaseType_t xHigherPriorityTaskWoken;
    /**
     * Unblock the task by releasing the semaphore only if no hardware error is 
     * detected and I2C master read or write is completed
     */
    if(events & CY_SCB_I2C_MASTER_ERR_EVENT)
    {
        printf("Failure!  : I2C hardware error detected\r\n");
        CY_ASSERT(0u); /* Halt CPU */
    }
    else if((events & CY_SCB_I2C_MASTER_WR_CMPLT_EVENT)|| 
            (events & CY_SCB_I2C_MASTER_RD_CMPLT_EVENT))
    {
        xSemaphoreGiveFromISR(xSemaphoreI2C, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(pdTRUE);
    }
}

/* [] END OF FILE */
