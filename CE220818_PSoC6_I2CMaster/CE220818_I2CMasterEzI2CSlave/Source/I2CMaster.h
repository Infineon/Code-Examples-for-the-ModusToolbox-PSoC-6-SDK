/******************************************************************************
* File Name: I2CMaster.h
*
* Version: 1.0
*
* Description: This file contains all the function prototypes required for proper
* 			   operation of I2C Master SCB
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
#ifndef SOURCE_I2CMASTER_H_
#define SOURCE_I2CMASTER_H_

#include "cy_pdl.h"
#include "cycfg.h"

/***************************************
*              Constants
****************************************/
#define TRANSFER_CMPLT    (0x00UL)
#define READ_CMPLT        (TRANSFER_CMPLT)
/* Packet positions */
#define PACKET_ADDR_POS     (0UL)
#define PACKET_SOP_POS      (1UL)
#define PACKET_STS_POS      (1UL)
#define PACKET_CMD_POS    	(2UL)
#define PACKET_EOP_POS      (3UL)
/* Start and end of packet markers */
#define PACKET_SOP      (0x01UL)
#define PACKET_EOP      (0x17UL)
#define WRITE_PACKET_SIZE (0x04UL)
/* Start address of slave buffer */
#define EZI2C_BUFFER_ADDRESS   (0x00)

/***************************************
*         Forward Prototypes
****************************************/
uint8_t WritePacketToEzI2C(uint8_t* writebuffer, uint32_t bufferSize);
uint8_t ReadStatusPacketFromEzI2C(void);
uint32_t initMaster(void);

#endif /* SOURCE_I2CMASTER_H_ */
