/******************************************************************************
* \file main.c
* \version 1.0
*
* \brief
* Objective:
*    Demonstrates executing functions and using variables from external
*    memory using QSPI (SMIF) in XIP mode.
*
* Compatible Kits:
*    CY8CKIT-062-BLE
*    CY8CKIT-062-WIFI-BT
*
* Importing the Code Example into ModusToolbox:
*   Follow the steps in KBA225201-Importing Code Example into ModusToolbox IDE
*   https://community.cypress.com/docs/DOC-15968
*
* Migration to CY8CPROTO-062-4343W kit (ModusToolbox IDE):
*   1. Following KBA225201, create this project targeting the CY8CPROTO-062-4343W kit.
*   2. Add the following to line 376 of file "cy8cxx7_cm4_dual.ld":
*      .cy_xip_code :
*      {
*           KEEP(*(.cy_xip)) 
*      } > xip
*   3. Follow the instructions from step 1 under "Instructions" below.
*
*   Instructions:
*       1. Connect header J10 on the CY8CKIT-062-WiFi-BT kit board to a USB port on your PC.
*       2. Open a serial port communication program, such as Tera Term and
*          select the corresponding COM port. Configure the terminal to match
*          UART: 115200 baud rate, 8N1, and Flow control – None. These settings
*          must match the configuration of the SCB UART driver in the project.
*       3. Right-click the "CE224285_PSoC6_QSPI_XIP_mainapp" project in the project
*          explorer. Select Run > Run Configurations
*          a. In the Run Configurations window select GDB OpenOCD Debugging >
*             CE224285_PSoC6_QSPI_XIP Program (KitProg3)
*          b. Select the Debugger window and add the following options to Config Options:
*             1. Add '-s "${workspace_loc}/CE224285_PSoC6_QSPI_XIP_mainapp/GeneratedSource" '
*             2. After the line '-c "source [find target/psoc6.cfg]" add '-c "psoc6 sflash_restrictions 1" '
*       4. Add the following to line 376 of file "cy8cxx7_cm4_dual.ld":
*           .cy_xip_code :
*           {
*               KEEP(*(.cy_xip)) 
*           } > xip
*       5. Build and program the application into the kit.
*       6. Make sure that debug messages display in the terminal window as
*          expected.
*   NOTE: If the code example is run for first time and QE is not set, you need to allow
*   sending Quad Enable command manually by pressing SW2 button to ensure that
*   power is stable. Debug message will appear in terminal window.
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
#include "cy_pdl.h"
#include "cycfg.h"
#include "cycfg_qspi_memslot.h"
#include "smif_mem.h"
#include <stdio.h>

/***************************************************************************
* Global constants
***************************************************************************/
#define ADDRESS_SIZE      	    (3u)	  /* Memory address size */
#define PACKET_SIZE             (32u)     /* The memory Read/Write packet */
#define TIMEOUT_1_MS            (1000ul)  /* 1 ms timeout for all blocking functions */
#define SMIF_PRIORITY           (1u)      /* SMIF interrupt priority */
#define BYTES_PER_LINE		    (8u)
#define TEST_DATA				(0xBA) 	  /* Data to be written to external memory */

/***************************************************************************
* Global string that is placed in external memory
***************************************************************************/
const char hiWord[] CY_SECTION(".cy_xip") = "\r\n\rHello from the external string\r\n\r";

/***************************************************************************
* Global contexts
***************************************************************************/
cy_stc_scb_uart_context_t UARTContext;
cy_stc_smif_context_t SMIFContext;

/*******************************************************************************
* Function Name: SMIF_Interrupt_User
********************************************************************************
*
* The ISR for the SMIF interrupt. All Read/Write transfers to/from the external
* memory are processed inside the SMIF ISR.
*
*******************************************************************************/
void SMIF_Interrupt_User(void)
{
	Cy_SMIF_Interrupt(SMIF_HW, &SMIFContext);
}

/*******************************************************************************
* Function wrappers
*******************************************************************************/
cy_en_smif_status_t SMIF_Start(void);
void CheckStatus(char *message, uint32_t status);
void UART_Start(void);
void PrintArray(char *message, uint8_t *buf, uint32_t size);
#pragma long_calls
void PrintFromExternalMemory(const char buf[]);
#pragma long_calls_off

/*******************************************************************************
* NULL terminated array of SMIF structures for use in cyToc2
*******************************************************************************/
typedef struct
{
    const cy_stc_smif_block_config_t * smifCfg; /* Pointer to SMIF top-level configuration */
    const uint32 null_t;                        /* NULL termination */
} stc_smif_ipblocks_arr_t;

const stc_smif_ipblocks_arr_t smifIpBlocksArr = {&smifBlockConfig, 0x00000000};

/*******************************************************************************
*  Point to the SMIF block structure in the table of contents2 (TOC2).
*   The TOC2 structure supports user application validation. See
*   AN221111 for more information about the TOC2 and its use.
*
*   This enables memory reads using Cypress Programmer, without this
*   structure, external memory access from Cypress Programmer will not work
*******************************************************************************/
CY_SECTION(".cy_toc_part2") __attribute__((used))
const int cyToc[(512 / 4)] =
{
   0x200-4,            /* Offset=0x00: Object Size, bytes */
   0x01211220,         /* Offset=0x04: Magic Number (TOC Part 2, ID) */
   0,                  /* Offset=0x08: Key Storage Address */
   (int)&smifIpBlocksArr,    /* Offset=0x0C: This points to a null terminated array of SMIF structures. */
   0x10000000u,        /* Offset=0x10: App image start address */
   0,                  /* Offset=0x14: Application Format */
   0,                  /* Offset=0x18: App #2 image start address */
   0,                  /* Offset=0x1C: App #2 App Format */
   0,                  /* Offset=0x20: Number of the next objects to add to SECURE_CMAC */
   0,                  /* Offset=0x24: Public Key address */
   0,                  /* Offset=0x28-1F4: … (additional objects if needed or 0’s if none) */
   /* Offset=0x1F8: */
   [(512 / sizeof(int)) - 2] = (0x00 << 2) | /* Bits [4:2]: Listen Window Time index: */
                                             /*  0 = 20ms (default), 1 = 10ms, 2 = 1ms, 3 = 0ms, 4 = 100ms,  5-7 = reserved */
                               (0x01 << 0),  /* Bits [1:0]: IMO/FLL clock frequency index:   */
                                             /*  0 = 8MHz, 1 = 25MHz (default), 2 = 50MHz, 3 = Reserved */
   /* Offset=0x1FC: CRC16-CCITT (the upper 2 bytes contain the CRC and the lower 2 bytes are 0) */
};

/********************************************************************
 * Function Name: Main
 ********************************************************************
 *  Initializes all peripherals, enables all peripherals, calls SMIF
 *  R/W functions, switches to XIP and executes functions as normal
 *******************************************************************/
int main(void)
{
    uint16 loopCount;

    /* R/W buffers */
    uint8 rxBuffer[PACKET_SIZE];
    uint8 txBuffer[PACKET_SIZE];

	/* Initalization Status Holders */
	cy_en_smif_status_t SMIF_Status = CY_SMIF_BAD_PARAM;

	/* Start of second sector of external memory.
	 *  First sector (0x00000000) contains cy_xip and cy_xip_code
	 *  regions assigned in the cy8c6xx7_cm4_dual.ld. MSB is at index 0
	 */
#if(ADDRESS_SIZE == 3)
    uint8 extMemAddress[ADDRESS_SIZE];
    extMemAddress[0] = (smifMemConfigs[0]->deviceCfg->eraseSize & 0x00FF0000) >> 16;
    extMemAddress[1] = (smifMemConfigs[0]->deviceCfg->eraseSize & 0x0000FF00) >> 8;
    extMemAddress[2] = (smifMemConfigs[0]->deviceCfg->eraseSize & 0x000000FF);
#elif(ADDRESS_SIZE == 4)
    uint8 extMemAddress[ADDRESS_SIZE];
    extMemAddress[0] = (smifMemConfigs[0]->deviceCfg->eraseSize & 0xFF000000) >> 24;
    extMemAddress[1] = (smifMemConfigs[0]->deviceCfg->eraseSize & 0x00FF0000) >> 16;
    extMemAddress[2] = (smifMemConfigs[0]->deviceCfg->eraseSize & 0x0000FF00) >> 8;
    extMemAddress[3] = (smifMemConfigs[0]->deviceCfg->eraseSize & 0x000000FF);
#endif

    /* Initialize txBuffer */
    for(loopCount = 0; loopCount < PACKET_SIZE; loopCount++)
    {
    	txBuffer[loopCount] = TEST_DATA;
    }                                     
                                     
    /* Set up the device based on configurator selections */
    init_cycfg_all();

    UART_Start(); /* Start the UART */

    __enable_irq(); /* Enable global interrupts */

    /* Setup the SMIF Interrupt */
    cy_stc_sysint_t smifIntConfig =
    {
    		.intrSrc = smif_interrupt_IRQn,
			.intrPriority = SMIF_PRIORITY
    };

    /* Start the SMIF block */
    cy_en_sysint_status_t intrStatus = Cy_SysInt_Init(&smifIntConfig, SMIF_Interrupt_User);
    CheckStatus("SMIF Interrupt initialization failed", (uint32)intrStatus);
    
    SMIF_Status = SMIF_Start();
    CheckStatus("SMIF Initialzation failed", SMIF_Status);

    /* Enable the SMIF Interrupt */
    NVIC_EnableIRQ(smif_interrupt_IRQn);

    /* Map memory device to memory map */
    SMIF_Status = Cy_SMIF_Memslot_Init(SMIF_HW, (cy_stc_smif_block_config_t *) &smifBlockConfig, &SMIFContext);
    CheckStatus("SMIF memory slot initialization failed", SMIF_Status);

    bool isQuadEnabled = false;
    SMIF_Status = IsQuadEnabled(smifMemConfigs[0], &isQuadEnabled);
    CheckStatus("Checking QE bit failed", SMIF_Status);

    /* Though CheckStatus() will not return if the status indicates failure, the
     * status is checked here to show that the value of isQuadEnabled is valid
     * only when the return value indicates success.
     */
    if((CY_SMIF_SUCCESS == SMIF_Status) && !isQuadEnabled)
    {
    	/* Enabling Quad mode is a non-volatile write that should not be
    	 * interrupted by resets (e.g. from debugger during programming or
    	 * debugging) or by unstable power. Otherwise the configuration register
    	 * may be corrupted. Therefore, user confirmation is prompted through a
    	 * button press.
    	 */
    	printf("\n\rQuad mode is NOT enabled. Press & release the USER button to enable it.\n\r");
		while (true)
		{
			if (Cy_GPIO_Read(KIT_BTN1_PORT, KIT_BTN1_PIN) == 0u)
			{
				Cy_SysLib_Delay(50u); /* Button debounce delay */
				if (Cy_GPIO_Read(KIT_BTN1_PORT, KIT_BTN1_PIN) == 0u)
				{
					/* Wait until user releases the button */
					while (Cy_GPIO_Read(KIT_BTN1_PORT, KIT_BTN1_PIN) == 0u)
					{
						Cy_SysLib_Delay(50u);
					}
					break;
				}
			}

			Cy_SysLib_Delay(50u);
		}
    	SMIF_Status = EnableQuadMode(smifMemConfigs[0]);
    	CheckStatus("Enabling Quad mode failed", SMIF_Status);
    	printf("\n\rQuad mode is enabled.\n\r");
    }

    /* Erase before write */
    printf("\n\r1. Erasing %lu bytes of memory.\n\r", smifMemConfigs[0]->deviceCfg->eraseSize);
    SMIF_Status = EraseMemory(smifMemConfigs[0], extMemAddress);
    CheckStatus("Erasing memory failed", SMIF_Status);

   	/* Read and print external flash data */
    /* Read after Erase to confirm that all data is 0xFF */
    printf("\n\r2. Reading after Erase. Ensure that the data read is 0xFF for each byte.\n\r");
    SMIF_Status = ReadMemory(smifMemConfigs[0], extMemAddress, rxBuffer, PACKET_SIZE);
    CheckStatus("Reading memory failed", SMIF_Status);
	PrintArray("Received Data",rxBuffer, PACKET_SIZE);

    /* Write the content of the txBuffer to the memory */
    printf("\n\r3. Writing data to memory.\n\r");
    SMIF_Status = WriteMemory(smifMemConfigs[0], extMemAddress, txBuffer, PACKET_SIZE);
    CheckStatus("Writing to memory failed", SMIF_Status);
    PrintArray("Written Data", txBuffer, PACKET_SIZE);

    /* Read back after Write for verification */
    printf("\n\r4. Reading back for verification.\n\r");
    SMIF_Status = ReadMemory(smifMemConfigs[0], extMemAddress, rxBuffer, PACKET_SIZE);
    CheckStatus("Reading memory failed", SMIF_Status);
    PrintArray("Received Data", rxBuffer, PACKET_SIZE);

	/* Put the device in XIP mode */
	printf("\n\rEntering XIP Mode\n\r");
	Cy_SMIF_SetMode(SMIF_HW, CY_SMIF_MEMORY);

	/* Print our string from external memory */
	printf(hiWord);

	/* Print by calling function which lives in external memory */
	PrintFromExternalMemory("\n\rHello from the external function\n\r");

    for(;;)
    {
    	/* Loop forever */
    }
}

/*******************************************************************************
* Function Name: CheckStatus
****************************************************************************//**
*
* Prints the message, indicates the non-zero status by turning the LED on, and
* asserts the non-zero status.
*
* params
*  message: message to print if status is non-zero.
*
*  status: status for evaluation.
*
*******************************************************************************/
void CheckStatus(char *message, uint32_t status)
{
    if(0u != status)
    {
    	printf("\n\r================================================================================\n\r");
        printf("\n\rFAIL: %s\n\r", message);
        printf("Error Code: 0x%08lX\n\r", status);
        printf("\n\r================================================================================\n\r");
        Cy_GPIO_Clr(KIT_LED1_PORT, KIT_LED1_PIN); /* On failure, turn the LED ON */
        while(true); /* Wait forever here when error occurs. */
    }
}

/*******************************************************************************
* Function Name: PrintArray
****************************************************************************//**
*
* Prints the content of the buffer to the UART console.
*
* \param message - message to print before array output
*
* \param buf - buffer to print on the console.
*
* \param size - size of the buffer.
*
*******************************************************************************/
void PrintArray(char *message, uint8_t *buf, uint32_t size)
{
    printf("\n\r%s (%lu bytes):\n\r", message, size);
    printf("-------------------------\n\r");

    for(uint32_t index = 0; index < size; index++)
    {
        printf("0x%02X ", buf[index]);

        if(0u == ((index + 1) % BYTES_PER_LINE))
        {
        	printf("\n\r");
        }
    }
}

/********************************************************
* SMIF_Start
*********************************************************
* Initializes the SMIF hardware, sets the slave select
* and enables the SMIF block.
*
* returns: the status of the block during initialization
*
********************************************************/
cy_en_smif_status_t SMIF_Start(void)
{
	cy_en_smif_status_t SMIF_Status = CY_SMIF_BAD_PARAM;

	SMIF_Status = Cy_SMIF_Init(SMIF_HW, &SMIF_config, TIMEOUT_1_MS, &SMIFContext);

	if(SMIF_Status == CY_SMIF_SUCCESS)
	{
		Cy_SMIF_SetDataSelect(SMIF_HW, smifMemConfigs[0]->slaveSelect, smifMemConfigs[0]->dataSelect);
		Cy_SMIF_Enable(SMIF_HW, &SMIFContext);
		printf("SMIF Block Enabled\n\r");
	}

	return SMIF_Status;
}

/********************************************************
* UART_Start
*********************************************************
* Initializes and enables the SCB hardware.
********************************************************/
void UART_Start(void)
{
    cy_en_scb_uart_status_t uartStatus = CY_SCB_UART_BAD_PARAM;

    uartStatus = Cy_SCB_UART_Init(UART_HW, &UART_config, &UARTContext);
    if(uartStatus == CY_SCB_UART_SUCCESS)
    {
    	Cy_SCB_UART_Enable(UART_HW);
    	printf("UART Enabled\n\r");
    }
    else
    {
    	CheckStatus(NULL, uartStatus);
    }

}

/********************************************************
* PrintFromExternalMemory
*********************************************************
* prints the passed string to a UART from external memory
*
* parameters
* 	buf: The string to be printed
********************************************************/
CY_SECTION(".cy_xip_code") __attribute__((used))
void PrintFromExternalMemory(const char buf[])
{
	printf(buf);
}

/*******************************************************************************
* Function Name: _write
*******************************************************************************/
#if defined (__GNUC__)
    /* Add an explicit reference to the floating point printf library to allow
    the usage of the floating point conversion specifier. */
    asm (".global _printf_float");
    /* For GCC compiler revise _write() function for printf functionality */
    int _write(int file, char *ptr, int len)
    {
        int nChars = 0;

        /* Suppress the compiler warning about an unused variable. */
        if (0 != file)
        {
        }

        for (/* Empty */; len != 0; --len)
        {
            /* Block until there is space in the TX FIFO or buffer. */
            while (0UL == Cy_SCB_UART_Put(UART_HW, *ptr))
            {
            }

            ++nChars;
            ++ptr;
        }

        return (nChars);
    }
    #elif defined(__ARMCC_VERSION)
    /* For the MDK/RVDS compiler, revise the  fputc() function for the  printf functionality */
    struct __FILE
    {
        int handle;
    };

    enum
    {
        STDIN_HANDLE,
        STDOUT_HANDLE,
        STDERR_HANDLE
    };

    FILE __stdin = {STDIN_HANDLE};
    FILE __stdout = {STDOUT_HANDLE};
    FILE __stderr = {STDERR_HANDLE};

    int fputc(int ch, FILE *file)
    {
        int ret = EOF;
        switch(file->handle)
        {
            case STDOUT_HANDLE:
                /* Block until there is space in the TX FIFO or buffer. */
                while (0UL == Cy_SCB_UART_Put(ch))
                {
                }

                ret = ch;
            break;

            case STDERR_HANDLE:
                ret = ch;
            break;

            default:
                file = file;
            break;
        }
        return(ret);
    }

    #elif defined (__ICCARM__)
    /* For the IAR compiler, revise the  __write() function for the  printf functionality */
    size_t __write(int handle, const unsigned char * buffer, size_t size)
    {
        size_t nChars = 0;

        for (/* Empty */; size != 0; --size)
        {
            /* Block until there is space in the TX FIFO or buffer. */
            while (0UL == Cy_SCB_UART_Put(*buffer))
            {
            }

            ++nChars;
            ++buffer;
        }
    return (nChars);
}
#endif /* (__GNUC__) */

/* [] END OF FILE */
