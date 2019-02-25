/******************************************************************************
* File Name: main.c
*
* Version: 1.0
*
* Description: This example project demonstrates the basic operation of the I2C
* master resource using low level APIs. The I2C master SCB sends the
* command packets to the I2C slave SCB to control an user LED.
*
* Related Document: CE220818_PSoC6_I2CMaster.pdf
*
*******************************************************************************
* Copyright (2018-19), Cypress Semiconductor Corporation. All rights reserved.
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

#include "cy_pdl.h"
#include "cycfg.h"
#include "I2CMaster.h"
#include "I2CSlave.h"
#include "Interface.h"

/***************************************
*            Constants
****************************************/
#define OFF          			(1UL)
#define ON						(0UL)
#define CMD_TO_CMD_DELAY     	(1000UL)

int main(void)
{
	/* Set up internal routing, pins, and clock-to-peripheral connections */
	init_cycfg_all();
    uint8_t cmd = ON;
    uint32_t status;
    uint8_t  buffer[TX_PACKET_SIZE];

    /*Initiate and enable Slave and Master SCBs*/
    status = initSlave();
    if(status == I2C_FAILURE)
    {
    	handle_error();
    }
    status = initMaster();
    if(status == I2C_FAILURE)
	{
    	handle_error();
	}

    /* Enable interrupts */
    __enable_irq();
    for(;;)
    {
        /* Create packet to be sent to slave. */
        buffer[PACKET_SOP_POS] = PACKET_SOP;
        buffer[PACKET_EOP_POS] = PACKET_EOP;
        buffer[PACKET_CMD_POS] = cmd;

        /* Send packet with command to the slave. */
        if (TRANSFER_CMPLT == WritePacket(buffer, TX_PACKET_SIZE))
        {
        	/* The below code is for slave function. It is implemented in
        	   this code example so that the master function can be tested
        	   without the need of one more kit. */

            /* Read response packet from the slave. */
            if (READ_CMPLT == ReadStatusPacket())
            {
				/* Next command to be written. */
            	cmd = (cmd == ON) ? OFF : ON;
            }
            /* Give 1 Second delay between commands. */
            Cy_SysLib_Delay(CMD_TO_CMD_DELAY);
        }
    }
}
