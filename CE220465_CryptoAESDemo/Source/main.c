/******************************************************************************
* File Name: main.c
*
* Version 1.0
*
* Description: This code example demonstrates encryption and decryption of data
*  using the Advanced Encryption Scheme (AES) algorithm in PSoC 6 MCU.
*
* Related Document: CE220465.pdf
*
* Hardware Dependency: See CE220465.pdf
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

#include "cy_device_headers.h"
#include "cycfg.h"
#include "cy_crypto.h"
#include "cy_crypto_server.h"
#include "stdio_user.h"
#include <stdio.h>
#include <string.h>

#define SCREEN_HEADER	  	"\r\n__________________________________________________"\
                  	  	  	"____________________________\r\n*\t\tCE220465 PSoC 6 "\
						  	"Cryptography: AES Demonstration\r\n*\r\n*\tIf you are able to "\
						  	"read the text the terminal connection is configured \n\r*"\
						  	"\tcorrectly\r\n*\r\n*\tBaud Rate : 115200 bps\r\n*"\
						  	"\r\n__________________________________________________"\
						  	"____________________________\r\n"

#define SCREEN_HEADER1 		"\r\n__________________________________________________"\
                  	   	   	"____________________________\r\n"

/* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
#define CLEAR_SCREEN 		"\x1b[2J\x1b[;H"

/* ASCII value for carriage return*/
#define CARRIAGE_RETURN_ASCII				(0x0D)

/* Macros to configure the Crypto block */
#define MY_CHAN_CRYPTO         				(uint32_t)(3u)    /* IPC data channel for the Crypto */
#define MY_INTR_CRYPTO_SRV     				(uint32_t)(1u)    /* IPC interrupt structure for the Crypto server */
#define MY_INTR_CRYPTO_CLI     				(uint32_t)(2u)    /* IPC interrupt structure for the Crypto client */
#define MY_INTR_CRYPTO_SRV_MUX 				(IRQn_Type)(2u)   /* CM0+ IPC interrupt mux number the Crypto server */
#define MY_INTR_CRYPTO_CLI_MUX 				(IRQn_Type)(3u)   /* CM0+ IPC interrupt mux number the Crypto client */
#define MY_INTR_CRYPTO_ERR_MUX 				(IRQn_Type)(4u)   /* CM0+ ERROR interrupt mux number the Crypto server */

#define AES128_KEY_LENGTH  					(uint32_t)(16u)
#define MAX_MESSAGE_SIZE   					(uint32_t)(16u)
#define AES128_ENCRYPTION_LENGTH 			(uint32_t)(16u)

/* Data type definition to store the data in text format as well as in hexadecimal
   format*/
typedef union datablock
{
    char text[16];
    uint32_t packedtext[4];
}datablock_t;

/* Variables to store Crypto internal states */
cy_stc_crypto_context_t cryptoScratch;
cy_stc_crypto_context_aes_t cryptoAES;
cy_stc_crypto_server_context_t cryptoServerContext;

/* Crypto configuration structure */
const cy_stc_crypto_config_t cryptoConfig =
    {
        /* .ipcChannel             */ MY_CHAN_CRYPTO,
        /* .acquireNotifierChannel */ MY_INTR_CRYPTO_SRV,
        /* .releaseNotifierChannel */ MY_INTR_CRYPTO_CLI,
        /* .releaseNotifierConfig */ {
        #if (CY_CPU_CORTEX_M0P)
            /* .intrSrc            */ MY_INTR_CRYPTO_CLI_MUX,
            /* .cm0pSrc            */ cpuss_interrupts_ipc_2_IRQn, /* depends on selected releaseNotifierChannel value */
        #else
            /* .intrSrc            */ cpuss_interrupts_ipc_2_IRQn, /* depends on selected releaseNotifierChannel value */
        #endif
            /* .intrPriority       */ 2u,
        },
        /* .userCompleteCallback   */ NULL,
        /* .userGetDataHandler     */ NULL,
        /* .userErrorHandler       */ NULL,
        /* .acquireNotifierConfig */ {
        #if (CY_CPU_CORTEX_M0P)
            /* .intrSrc            */ MY_INTR_CRYPTO_SRV_MUX,      /* to use with DeepSleep mode should be in DeepSleep capable muxer's range */
            /* .cm0pSrc            */ cpuss_interrupts_ipc_1_IRQn, /* depends on selected acquireNotifierChannel value */
        #else
            /* .intrSrc            */ cpuss_interrupts_ipc_1_IRQn, /* depends on selected acquireNotifierChannel value */
        #endif
            /* .intrPriority       */ 2u,
        },
        /* .cryptoErrorIntrConfig */ {
        #if (CY_CPU_CORTEX_M0P)
            /* .intrSrc            */ MY_INTR_CRYPTO_ERR_MUX,
            /* .cm0pSrc            */ cpuss_interrupt_crypto_IRQn,
        #else
            /* .intrSrc            */ cpuss_interrupt_crypto_IRQn,
        #endif
            /* .intrPriority       */ 2u,
        }
    };


cy_stc_scb_uart_context_t uartContext;

void PrintHeader(void);
void PrintData(uint8_t*);

/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  Main function.
*
* Parameters:
*  None
*
* Return:
*  int
*
*******************************************************************************/

int main(void)
{
	/* Key used for AES encryption*/
	uint8_t AES_Key[AES128_KEY_LENGTH]={0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,0xFF,0xEE,\
	                                     0xDD,0xCC,0xBB,0xAA,0xAA,0xBB,0xCC,0xDD,};

	datablock_t ciphertext;
	char message[MAX_MESSAGE_SIZE+1],Decoded_Message[MAX_MESSAGE_SIZE+1],temp;
	uint32_t message_size;

	/* Set up internal routing, pins, and clock-to-peripheral connections */
	init_cycfg_all();

	__enable_irq();

	/* Initialize and enable UART block */
	Cy_SCB_UART_Init(KIT_UART_HW, &KIT_UART_config, &uartContext);
	Cy_SCB_UART_Enable(KIT_UART_HW);

	printf(CLEAR_SCREEN);

	/* Start the Crypto Server */
	Cy_Crypto_Server_Start(&cryptoConfig, &cryptoServerContext);

	/*Initialization of Crypto Driver */
	Cy_Crypto_Init(&cryptoConfig, &cryptoScratch);


	/* Enable Crypto Hardware */
	Cy_Crypto_Enable();

	/* Wait for Crypto Block to be available */
	Cy_Crypto_Sync(CY_CRYPTO_SYNC_BLOCKING);

	PrintHeader();

	for(;;)
	{
		message_size = 0;
		printf("\r\nEnter the message( not more than 16 characters):\r\n\n");

		/* Read the user input message till the user press the "ENTER" Key
		(ASCII value ='0x0D')*/
		Cy_SCB_UART_GetArrayBlocking(KIT_UART_HW, &temp,1);

		while(temp != CARRIAGE_RETURN_ASCII)
		{
			Cy_SCB_UART_Put(KIT_UART_HW, temp);
			message[message_size] = temp;
			message_size++;
			Cy_SCB_UART_GetArrayBlocking(KIT_UART_HW, &temp, 1);
		}
		message[message_size] = '\0';

		/*Check whether message length exceeds 16 characters*/
		if(message_size > MAX_MESSAGE_SIZE)
		{
			printf("\r\n\nMessage size exceeds 16 characters! Please enter a "\
					"message of not more than 16 characters \r\n");
		}

		else
		{
			/* Initializes the AES operation by setting key and key length */
			Cy_Crypto_Aes_Init((uint32_t*)AES_Key, CY_CRYPTO_KEY_AES_128, &cryptoAES);


			/* Perform AES ECB Encryption mode of operation */
			Cy_Crypto_Aes_Ecb_Run(CY_CRYPTO_ENCRYPT, ciphertext.packedtext,\
									 (uint32_t*)message, &cryptoAES);

			/* Wait for Crypto Block to be available */
			Cy_Crypto_Sync(CY_CRYPTO_SYNC_BLOCKING);

			printf("\r\n\nKey used for Encryption:\r\n");
			PrintData(AES_Key);
			printf("\r\nResult of Encryption:\r\n");
			PrintData((uint8_t*) ciphertext.text);

			 /* Perform AES ECB Decryption mode of operation */
			Cy_Crypto_Aes_Ecb_Run(CY_CRYPTO_DECRYPT,( uint32_t*)Decoded_Message\
									 , ciphertext.packedtext,&cryptoAES);

			/* Wait for Crypto Block to be available */
			Cy_Crypto_Sync(CY_CRYPTO_SYNC_BLOCKING);

			Decoded_Message[message_size]='\0';
			/* Print the decrypted message on the UART terminal */
			printf("\r\nResult of Decryption:\r\n\n");
			printf("%s", Decoded_Message);
			printf("%s", SCREEN_HEADER1);
		}
	}
}

/***********************************************************************************
* Function Name: PrintHeader()
************************************************************************************
* Summary: Function used to display the main menu to the user
*
* Parameters:
*  None
*
* Return:
*  void
*
************************************************************************************/
void PrintHeader()
{
	/* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
	printf("\x1b[2J\x1b[;H");

	printf("%s",SCREEN_HEADER);
}

/***********************************************************************************
* Function Name: PrintData()
************************************************************************************
* Summary: Function used to display the data in hexadecimal format
*
* Parameters:
*  uint8_t*
*  Pointer to location of data to be printed
*
* Return:
*  void
*
************************************************************************************/
void PrintData(uint8_t* text)
{
	datablock_t printdata;

	memcpy(printdata.text, text, AES128_ENCRYPTION_LENGTH);
	printf("\r\n");
	char print[10];
	for(uint32 i=0; i < AES128_ENCRYPTION_LENGTH; i++)
	{
		sprintf(print,"0x%2X ", printdata.text[i]);
		printf("%s", print);
	}
	printf("\r\n");
}
