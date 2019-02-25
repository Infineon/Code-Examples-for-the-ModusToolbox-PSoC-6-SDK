/******************************************************************************
* File Name: main.c
*
* Version 1.0
*
* Description: This example demonstrates generating a One-Time-Password (OTP)
* of 8 characters in length. Using the True Random Number generation feature
* of PSoC 6 MCU crypto block, this example generates a random number
* corresponding to each character of the OTP. The generated random number is
* such that it corresponds to alpha-numeric and special characters of the ASCII
* code. The generated OTP is then displayed on a UART terminal emulator.
*
* Related Document: CE221295.pdf
*
* Hardware Dependency: See CE221295.pdf
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

#include "cy_device_headers.h"
#include "cycfg.h"
#include "cy_crypto.h"
#include "cy_crypto_server.h"
#include <stdio.h>
#include "stdio_user.h"

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

/* Macros for the polynomial to configure the programmable Galois and Fibonacci
   ring oscillators */
#define CRYPTO_TRNG_GARO_POL             (0x42000000)
#define CRYPTO_TRNG_FIRO_POL             (0x43000000)

/* Macro for the maximum value of the random number generated in bits */
#define MAX_TRND_VAL                     (7u)

#define PASSWORD_LENGTH                  (8u)
#define ASCII_ALPHANUMERIC_LOWER_LIMIT   (33u)
#define ASCII_ALPHANUMERIC_UPPER_LIMIT   (127u)
#define ASCII_RETURN_CARRIAGE            (0x0D)

#define SCREEN_HEADER "\r\n__________________________________________________"\
           "____________________________\r\n*\tCE221295 PSoC 6 "\
           "MCU Cryptography: True Random Number Generation\r\n*\r\n*\t"\
           "This code example demonstrates generating a One-Time Password (OTP)\r\n*\t"\
		   "using the True Random Number generation feature of PSoC 6 MCU"\
		   "\r\n*\tcryptography block\r\n*\r\n*\tUART Terminal SettingsBaud Rate : 115200 bps 8N1 \r\n*"\
           "\r\n__________________________________________________"\
           "____________________________\r\n"

#define SCREEN_HEADER1 "\r\n================================================="\
           "=============================\r\n"

/* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
#define CLEAR_SCREEN 		"\x1b[2J\x1b[;H"

/* Crypto context structure to store and manipulate the global context */
cy_stc_crypto_context_t cryptoScratch;

cy_stc_crypto_server_context_t cryptoServerContext;

/* Crypto context structure to store and manipulate the TRNG operational context */
cy_stc_crypto_context_trng_t TRNGContext;

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

/* UART configuration structure */
cy_stc_scb_uart_context_t uartContext;

/*******************************************************************************
 * Local Function Declaration
 ******************************************************************************/
void GeneratePassword(int32_t size, uint8_t* buffer);

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
    /* Set up the device based on configurator selections */
    init_cycfg_all();

    __enable_irq();

    uint8_t TRNGData[PASSWORD_LENGTH + 1];

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

	printf(SCREEN_HEADER);
 
    for(;;)
    {
    	printf("Press Enter to Generate OTP\r\n\n");
		while(Cy_SCB_UART_Get(KIT_UART_HW)!= ASCII_RETURN_CARRIAGE);

		/* Generate a password of 8 characters in length */
		GeneratePassword(sizeof(TRNGData),TRNGData);

		/* Display the generated password on the UART Terminal */
		printf("One-Time Password: %s\r\n",TRNGData);
		printf(SCREEN_HEADER1);
    }
}

/*******************************************************************************
* Function Name: GeneratePassword
********************************************************************************
* Summary: This function generates a 8 character long password
*
* Parameters:
*  int32_t size    - size of the buffer to store the generated password
*  uint8_t* buffer - pointer to the location where the password is stored
*
* Return
*  void
*
*******************************************************************************/
void GeneratePassword(int32_t size, uint8_t* buffer)
{
    int32_t index;
    uint32_t temp;

    for(index = 0; index < (size-1); index++)
    {
        /* Generate a random number */
        Cy_Crypto_Trng_Generate(CRYPTO_TRNG_GARO_POL, CRYPTO_TRNG_FIRO_POL,\
                                MAX_TRND_VAL, &temp, &TRNGContext);

        /* Wait until crypto completes operation */
        Cy_Crypto_Sync(CY_CRYPTO_SYNC_BLOCKING);

        /* Check if the generated random number is in the range of alpha-numeric,
        special characters ASCII codes. If not, convert to that range */
        if(temp < ASCII_ALPHANUMERIC_LOWER_LIMIT)
        {
            temp += ASCII_ALPHANUMERIC_LOWER_LIMIT;
        }
        if(temp >= ASCII_ALPHANUMERIC_UPPER_LIMIT)
        {
            temp -= ASCII_ALPHANUMERIC_LOWER_LIMIT;
        }
        buffer[index] = temp;
    }
    /* Terminate the password with end of string character */
    buffer[index]='\0';
}
