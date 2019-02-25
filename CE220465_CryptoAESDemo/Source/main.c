/******************************************************************************
* File Name: main.c
*
* Version 1.10
*
* Description: This code example demonstrates encryption and decryption of data
*  using the Advanced Encryption Scheme (AES) algorithm in PSoC 6 MCU.
*
* Related Document: CE220465.pdf
*
* Hardware Dependency: See CE220465.pdf
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

#include "cy_device_headers.h"
#include "cycfg.h"
#include "cy_crypto.h"
#include "cy_crypto_server.h"
#include "stdio_user.h"
#include <stdio.h>
#include <string.h>

/*******************************************************************************
* Macro definitions
********************************************************************************/
#define UART_INT_NUM						((IRQn_Type) scb_5_interrupt_IRQn)
#define UART_INTR_PRIORITY					(1u)

/* Macros to configure the Crypto block */
/* IPC data channel for the Crypto */
#define MY_CHAN_CRYPTO         				(uint32_t)(3u)
/* IPC interrupt structure for the Crypto server */
#define MY_INTR_CRYPTO_SRV     				(uint32_t)(1u)
/* IPC interrupt structure for the Crypto client */
#define MY_INTR_CRYPTO_CLI     				(uint32_t)(2u)
/* CM0+ IPC interrupt mux number the Crypto server */
#define MY_INTR_CRYPTO_SRV_MUX 				(IRQn_Type)(2u)
 /* CM0+ IPC interrupt mux number the Crypto client */
#define MY_INTR_CRYPTO_CLI_MUX 				(IRQn_Type)(3u)
/* CM0+ ERROR interrupt mux number the Crypto server */
#define MY_INTR_CRYPTO_ERR_MUX 				(IRQn_Type)(4u)

/* The input message size (inclusive of the string terminating character '\0').
 * Edit this macro to suit your message size */
#define MAX_MESSAGE_SIZE    				(100u)

/* Size of the message block that can be processed by Crypto hardware for
 * AES encryption */
#define AES128_ENCRYPTION_LENGTH 			(uint32_t)(16u)

#define AES128_KEY_LENGTH  					(uint32_t)(16u)

#define ASCII_RETURN_CARRIAGE            	(0x0D)

#define SCREEN_HEADER "\r\n__________________________________________________"\
                  	  "____________________________\r\n*\t\tCE220465 PSoC 6 "\
					  "Cryptography: AES Demonstration\r\n*\r\n*\tThis code example demonstrates"\
					  " encryption and decryption of data using\r\n*\tthe Advanced Encryption Scheme (AES)"\
					  "algorithm in PSoC 6 MCU."\
					  "\r\n*\r\n*\tUART Terminal Settings: Baud Rate - 115200 bps, 8N1\r\n*"\
					  "\r\n__________________________________________________"\
					  "____________________________\r\n"

#define SCREEN_HEADER1 "\r\n__________________________________________________"\
                  	   "____________________________\r\n"

/* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
#define CLEAR_SCREEN 		"\x1b[2J\x1b[;H"

/*******************************************************************************
* Data type definitions
********************************************************************************/

/* Data type definition to access the data in 1-byte format as well as 4-byte
 * aligned format. This is required since the Crypto block operates on 4-byte
 * aligned values. */

/* Data type definition to track the state machine accepting the user message */
typedef enum
{
	MESSAGE_ENTER_NEW,
	MESSAGE_READY,
	MESSAGE_NOT_READY
}message_status_t;

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

/* Key used for AES encryption*/
CY_ALIGN(4) uint8_t AES_Key[AES128_KEY_LENGTH]={0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,0xFF,0xEE,\
                                     0xDD,0xCC,0xBB,0xAA,0xAA,0xBB,0xCC,0xDD,};

/* UART interrupt configuration structure */
cy_stc_sysint_t uart_IntrConfig =
{
		.intrSrc 		= UART_INT_NUM,
		.intrPriority 	= UART_INTR_PRIORITY,
};

cy_stc_scb_uart_context_t uartContext;

/* Variables to hold the user message and the corresponding encrypted message */
	CY_ALIGN(4) char message[MAX_MESSAGE_SIZE];
	CY_ALIGN(4) char encrypted_msg[MAX_MESSAGE_SIZE];
	CY_ALIGN(4) char decrypted_msg[MAX_MESSAGE_SIZE];

/*******************************************************************************
* Function prototypes
*******************************************************************************/
void DisplayMenu(void);
void PrintData(uint8_t*, uint8_t len);
void UART_Isr(void);

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
	uint8_t messagesize = 0, AESBlock_count = 0;

	/* Variable to track the status of the message entered by the user */
	message_status_t msgStatus = MESSAGE_ENTER_NEW;

	uint32_t uartStatus;

	init_cycfg_all();

	Cy_SysInt_Init(&uart_IntrConfig, UART_Isr);
	NVIC_EnableIRQ(uart_IntrConfig.intrSrc);

	/* Initialize and enable UART block */
	Cy_SCB_UART_Init(KIT_UART_HW, &KIT_UART_config, &uartContext);
	Cy_SCB_UART_Enable(KIT_UART_HW);

	__enable_irq();

	printf(CLEAR_SCREEN);

	/* Start the Crypto Server */
	Cy_Crypto_Server_Start(&cryptoConfig, &cryptoServerContext);

	/*Initialization of Crypto Driver */
	Cy_Crypto_Init(&cryptoConfig, &cryptoScratch);

	/* Enable Crypto Hardware */
	Cy_Crypto_Enable();

	/* Wait for Crypto Block to be available */
	Cy_Crypto_Sync(CY_CRYPTO_SYNC_BLOCKING);

	DisplayMenu();

	for(;;)
	{
		switch(msgStatus)
		{
			case MESSAGE_ENTER_NEW:
				memset(message, 0, MAX_MESSAGE_SIZE);
				messagesize = 0;
				printf("\r\nEnter the message:\r\n");
				Cy_SCB_UART_Receive(KIT_UART_HW, &message[messagesize], 1, &uartContext);
				msgStatus = MESSAGE_NOT_READY;
				break;

			case MESSAGE_NOT_READY:

				uartStatus = Cy_SCB_UART_GetReceiveStatus(KIT_UART_HW, &uartContext);
				if(uartStatus != CY_SCB_UART_RECEIVE_ACTIVE)
				{
					/* Check if the ENTER Key is pressed. If pressed, set the message
					 * status as MESSAGE_READY */
					if(message[messagesize] == ASCII_RETURN_CARRIAGE)
					{
						message[messagesize]='\0';
						msgStatus = MESSAGE_READY;
					}
					else
					{
						Cy_SCB_UART_Put(KIT_UART_HW, message[messagesize]);
						messagesize++;
						/* Check if size of the message  exceeds MAX_MESSAGE_SIZE
						 * (inclusive of the string terminating character '\0')*/
						if(messagesize > (MAX_MESSAGE_SIZE - 1))
						{
							printf("\r\n\nMessage length exceeds 100 characters!!!"\
							" Please enter a shorter message\r\nor edit the macro MAX_MESSAGE_SIZE"\
							" to suit your message size\r\n");
							msgStatus = MESSAGE_ENTER_NEW;
							break;
						}
						else
						{
							Cy_SCB_UART_Receive(KIT_UART_HW, &message[messagesize], 1, &uartContext);
						}
					}
				}
				break;

			case MESSAGE_READY:

				AESBlock_count =  (messagesize % AES128_ENCRYPTION_LENGTH == 0) ? \
								  (messagesize/AES128_ENCRYPTION_LENGTH) \
								  : (1 + messagesize/AES128_ENCRYPTION_LENGTH);

				/* Initializes the AES operation by setting key and key length */
				Cy_Crypto_Aes_Init((uint32_t*)AES_Key, CY_CRYPTO_KEY_AES_128, &cryptoAES);

				/* Wait for Crypto Block to be available */
				Cy_Crypto_Sync(CY_CRYPTO_SYNC_BLOCKING);

				for(int i = 0; i < AESBlock_count ; i++)
				{
					/* Perform AES ECB Encryption mode of operation */
					Cy_Crypto_Aes_Ecb_Run(CY_CRYPTO_ENCRYPT,\
					(uint32_t*) (encrypted_msg + AES128_ENCRYPTION_LENGTH * i),\
					(uint32_t*) (message + AES128_ENCRYPTION_LENGTH * i), &cryptoAES);

					/* Wait for Crypto Block to be available */
					Cy_Crypto_Sync(CY_CRYPTO_SYNC_BLOCKING);
				}

				printf("\r\n\nKey used for Encryption:\r\n");
				PrintData(AES_Key, AES128_KEY_LENGTH);
				printf("\r\nResult of Encryption:\r\n");
				PrintData((uint8_t*) encrypted_msg, \
							AESBlock_count*AES128_ENCRYPTION_LENGTH);

				for(int i = 0; i < AESBlock_count ; i++)
				{
					/* Perform AES ECB Decryption mode of operation */
					Cy_Crypto_Aes_Ecb_Run(CY_CRYPTO_DECRYPT,\
					( uint32_t*) (decrypted_msg + AES128_ENCRYPTION_LENGTH * i), \
				    ( uint32_t*) (encrypted_msg + AES128_ENCRYPTION_LENGTH * i), \
					&cryptoAES);

					/* Wait for Crypto Block to be available */
					Cy_Crypto_Sync(CY_CRYPTO_SYNC_BLOCKING);
				}

				decrypted_msg[messagesize]='\0';
				/* Print the decrypted message on the UART terminal */
				printf("\r\nResult of Decryption:\r\n\n");
				printf("%s", decrypted_msg);
				printf("%s", SCREEN_HEADER1);

				msgStatus = MESSAGE_ENTER_NEW;
				break;

			default:

				break;
		}
	}
}

/*******************************************************************************
* Function Name: DisplayMenu()
********************************************************************************
* Summary: Function used to display the main menu to the user
*
* Parameters:
*  None
*
* Return:
*  void
*
*******************************************************************************/
void DisplayMenu()
{
	/* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
	printf("\x1b[2J\x1b[;H");

	printf("%s",SCREEN_HEADER);
}

/*******************************************************************************
* Function Name: PrintData()
********************************************************************************
* Summary: Function used to display the data in hexadecimal format
*
* Parameters:
*  uint8_t*
*  Pointer to location of data to be printed
*
* Return:
*  void
*
*******************************************************************************/
void PrintData(uint8_t* data, uint8_t len)
{
	printf("\r\n");
	char print[10];
	for(uint32 i=0; i < len; i++)
	{
		sprintf(print,"0x%02X ", *(data+i));
		printf("%s", print);
	}
	printf("\r\n");

}

/*******************************************************************************
* Function Name: UART_Isr
********************************************************************************
* Summary: Function used to handle the UART interrupt.
*
* Parameters:
*  None
*
* Return:
*  void
*
*******************************************************************************/
void UART_Isr(void)
{
	Cy_SCB_UART_Interrupt(KIT_UART_HW, &uartContext);
}
