/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.0
*
* Description: This example project demonstrates SPI communication between two
* SCB blocks of PSoC 6 device (one configured as Master and another as slave).
* Master uses interrupt method to check the status of data transfer.
*
* Hardware Dependency: PSoC 6 BLE Pioneer Kit, PSoC 6 WiFi-BT Pioneer Kit,
* 					   PSoC 6 Wi-Fi BT Prototyping Kit
*
* Related Document: CE221120_PSoC6_SPIMaster.pdf
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
* ****************************************************************************/
#include "cy_pdl.h"
#include "cycfg.h"
#include "SPIMaster.h"
#include "SPISlave.h"
#include "Interface.h"

/***************************** Macros ****************************************/

/* LED States. LEDs in the supported kits are in active low connection. */
#define LED_ON 				(0)
#define LED_OFF				(1)

/* Delay between successive SPI Master command transmissions */
#define CMD_DELAY			(1000)	//in milliseconds

/* Number of elements in the transmit and receive buffer */
/* There are three elements - one for head, one for command and one for tail */
#define NUMBER_OF_ELEMENTS	(3UL)
#define SIZE_OF_ELEMENT		(4UL)
#define SIZE_OF_PACKET		(NUMBER_OF_ELEMENTS * SIZE_OF_ELEMENT)


/************************ Function Prototypes ********************************/

/* Function to turn ON or OFF the LED based on the SPI Master command. */
void UpdateLED(uint32_t);

/* Function to handle the error */
void handle_error(void);

uint32 masterTranStatus = IDLE;

/******************************************************************************
* Function Name: main
*******************************************************************************
*
* Summary: 	1. 	Sets up two SPI components - one as master and another as slave
*   		2. 	SPI master sends commands to the slave to turn LED ON or OFF,
*   			every one second.
*   		3. 	SPI Master uses interrupt to check the status of data transfer.
*   		4.  Data received by the slave is used to turn LED ON or OFF
*
* Parameters:	None
*
* Return:		Int
*
******************************************************************************/
int main(void)
{
	/* Buffer to hold command packet to be sent to the slave by the master */
	uint32  txBuffer[NUMBER_OF_ELEMENTS];

	/* Buffer to save the received data by the slave */
	uint32  rxBuffer[NUMBER_OF_ELEMENTS];

	/* Local command variable */
	uint32 cmd = LED_OFF;

	uint32 status = 0;

	/* Set up internal routing, pins, and clock-to-peripheral connections */
	init_cycfg_all();

	/* Initialize the SPI Slave */
	status = initSlave();

	if(status == INIT_FAILURE)
	{
		/* NOTE: This function will block the CPU forever */
		handle_error();
	}

	/* Initialize the SPI Master */
	status = initMaster();

	if(status == INIT_FAILURE)
	{
		/* NOTE: This function will block the CPU forever */
		handle_error();
	}

	/* Enable global interrupt */
	__enable_irq();

    for(;;)
    {
    	if(masterTranStatus == IDLE)
    	{
    		/* Give delay between successive commands */
    		Cy_SysLib_Delay(CMD_DELAY);

    		/* Toggle the current LED state */
			cmd = (cmd == LED_ON) ? LED_OFF : LED_ON;

			/* Form the command packet */
			txBuffer[PACKET_SOP_POS] = PACKET_SOP;
			txBuffer[PACKET_CMD_POS] = cmd;
			txBuffer[PACKET_EOP_POS] = PACKET_EOP;

			/* Pass the command packet to the master along with the number of bytes to be
			 * sent to the slave */
			sendPacket(txBuffer, SIZE_OF_PACKET);
    	}
    	else
    	if(masterTranStatus == TRANSFER_COMPLETE)
		{
    		/*The below code is for slave function. It is implemented in this same code example so that the master
    		function can be tested without the need of one more kit. */

    		/* Poll for the required number of bytes to be received by the slave */
    		status = readPacket(rxBuffer,SIZE_OF_PACKET);

    		/* Check whether the slave succeeded in receiving the required number of bytes
			 * and in the right format */
			if(status == COMMUNICATION_SUCCESS)
			{
				/* Communication succeeded. Update the LED. */
				UpdateLED(rxBuffer[PACKET_CMD_POS]);
			}
			else
			{
				/* Communication failed */
				handle_error();
			}

			masterTranStatus =  IDLE;
		}
		else
		if(masterTranStatus == TRANSFER_FAILURE)
		{
			/* If any error occurs handle the error  */
			handle_error();
		}
    }
}

/******************************************************************************
* Function Name: UpdateLED
*******************************************************************************
*
* Summary: 		This function updates the LED based on the command received by
* 				the SPI Slave from Master.
*
* Parameters: 	(uint32) LED_Cmd - command to turn LED ON or OFF
*
* Return:		None
*
******************************************************************************/
void UpdateLED(uint32 LED_Cmd)
{
	/* Control the LED. Note that the LED on the supported kits is in active low
	   connection. */

	if(LED_Cmd == LED_ON)
	{
		/* Turn ON the LED */
		Cy_GPIO_Clr(KIT_LED2_PORT, KIT_LED2_NUM);
	}

	if(LED_Cmd == LED_OFF)
	{
		/* Turn OFF the LED */
		Cy_GPIO_Set(KIT_LED2_PORT, KIT_LED2_NUM);
	}
}

/******************************************************************************
* Function Name: handle_error
*******************************************************************************
*
* Summary: 		This is a blocking function. It disables the interrupt and waits
* 				in an infinite loop. This function is called when an error is
* 				encountered during initialization of the blocks or during
* 				SPI communication.
*
* Parameters: 	None
*
* Return:		None
*
******************************************************************************/
void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    /* Infinite loop. */
    while(1u) {}
}

