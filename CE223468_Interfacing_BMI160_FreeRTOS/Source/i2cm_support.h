/*******************************************************************************
* File Name: i2cm_support.h
*
* Version 1.0
*
* Description: This file is the public interface of i2cm_support.c
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

/* Include Guard */
#ifndef I2CM_SUPPORT_H_
#define I2CM_SUPPORT_H_

/* Header file included */
#include "cy_pdl.h"
#include "FreeRTOS.h"
#include "semphr.h"

/* Semaphore handle for I2C */
extern SemaphoreHandle_t xSemaphoreI2C;   
/* I2C context structure */
extern cy_stc_scb_i2c_context_t KIT_I2C_context;

/* Function Prototype */
void I2C_Init(void);
void I2C_Callback(uint32_t events);
unsigned int I2C_ReadBytes(uint8_t Address, uint8_t RegisterAddr, 
                            uint8_t *RegisterValue, uint8_t RegisterLen);
unsigned int I2C_WriteBytes(uint8_t Address, uint8_t RegisterAddr,
                            uint8_t *RegisterValue, uint8_t RegisterLen);

#endif /* I2CM_SUPPORT_H_ */

/* [] END OF FILE */
