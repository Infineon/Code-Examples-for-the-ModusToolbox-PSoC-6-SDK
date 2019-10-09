/****************************************************************************
*File Name: main_cm4.c
*
* Version: 1.0
*
* Description: This code example demonstrates the SPI F-RAM access using PSoC6 SMIF.
               The SPI F-RAM includes both legacy F-RAM and Excelon (SPI) F-RAM products.
*
* Related Document: CE222460_SPI_FRAM_ACCESS_WITH_PSOC6_SMIF.pdf
*
* Hardware Dependency: CE222460_SPI_FRAM_ACCESS_WITH_PSOC6_SMIF.pdf
*
*******************************************************************************
* Copyright (2019), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (â€œSoftwareâ€), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (â€œCypressâ€) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (â€œEULAâ€).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypressâ€™s integrated circuit products.
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
* significant property damage, injury or death (â€œHigh Risk Productâ€). By
* including Cypressâ€™s product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "stdio.h"
#include "string.h"
#include "spi_fram_apis.h"
#include "stdio_user.h"

/***************************************************************************
* Global constants
***************************************************************************/
#define SMIF_PRIORITY           (1u)     /* SMIF interrupt priority */
#define MODE_BYTE               (0x00)   /* non XIP */
#define TEST_PASS               (0x00)
#define TEST_FAIL               (0x01)
#define NUM_BYTES_PER_LINE		(16u)	 /* Used when array of data is printed on the console */
#define PACKET_SIZE             (256u)   /* The memory Read/Write packet */

/*RGB LED control*/
#define RGB_GLOW_RED          (0x01)
#define RGB_GLOW_GREEN        (0x02)
#define RGB_GLOW_OFF          (0x03)
#define LED_TOGGLE_DELAY_MSEC (1000u)	/* LED blink delay */

/***************************************************************************
* Global variables
***************************************************************************/
cy_stc_scb_uart_context_t KIT_UART_context;
cy_stc_smif_context_t KIT_FRAM_context;

uint32_t EXAMPLE_INITIAL_ADDR    = 0x00;     /* Example F-RAM Initial Address */
uint32_t EXAMPLE_INITIAL_ADDR_SS = 0x00;     /* Example F-RAM Initial Address for Special Sector */
uint32_t EXAMPLE_ANY_ADDR        = 0x123456; /* Example F-RAM Any Address */
uint8_t  EXAMPLE_DATA_BYTE       = 0xCA;     /* Example F-RAM Data */

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
*
* This function processes unrecoverable errors
*
*******************************************************************************/
void handle_error(void)
{
     /* Disable all interrupts */
    __disable_irq();
    while(1u) {}
}

/*******************************************************************************
* Function Name: status_led
********************************************************************************
*
* This function drives the red/green status of RGB
* *
* \param status - The led status (RGB_GLOW_OFF, RGB_GLOW_RED, or RGB_GLOW_GREEN)
*
*******************************************************************************/
void status_led (uint32_t status)
{
	  Cy_GPIO_Write (RGB_RED_PORT, RGB_RED_NUM, status>>1&1U );
	  Cy_GPIO_Write (RGB_GREEN_PORT, RGB_GREEN_NUM, status&1U);
}

/*******************************************************************************
* Function Name: Isr_SMIF
********************************************************************************
*
* The ISR for the SMIF interrupt. All Read/Write transfers to/from the external
* memory are processed inside the SMIF ISR.
*
*******************************************************************************/
void Isr_SMIF(void)
{
    Cy_SMIF_Interrupt(KIT_FRAM_HW, &KIT_FRAM_context);
}

/*******************************************************************************
* Function Name: Initialize_SMIF
********************************************************************************
*
* This function initializes the SMIF block
*
*******************************************************************************/
void Init_SMIF(void)
{
	/* Initialize SMIF interrupt */

	      cy_stc_sysint_t smifIntConfig =
		  {
		   .intrSrc = KIT_FRAM_IRQ,
		   .intrPriority = SMIF_PRIORITY
		  };

	      cy_en_sysint_status_t intrStatus = Cy_SysInt_Init(&smifIntConfig, Isr_SMIF);

	      if(0u != intrStatus)
		    {
		    	printf("\r\n================================================================================");
		        printf("\r\nSMIF interrupt initialization failed");
		        printf("\r\nError Code: 0x%08uX", (unsigned int) intrStatus);
		        printf("\r\n================================================================================\n");
		        status_led (RGB_GLOW_RED);

		        for(;;)
		          {
		             /*Waits forever when SMIF initialization error occurs*/
		          }
		    }

		  printf("\r\nSMIF interrupt is initialized\n");

		  /* Initialize SMIF */
		  cy_en_smif_status_t smifStatus;
		  smifStatus = Cy_SMIF_Init(KIT_FRAM_HW, &KIT_FRAM_config, TIMEOUT_1_MS, &KIT_FRAM_context);

		  if(0u != smifStatus)
		    {
		    	printf("\r\n================================================================================");
		        printf("\r\nSMIF interrupt is initialized");
		        printf("\r\nError Code: 0x%08uX", (unsigned int) smifStatus);
		        printf("\r\n================================================================================\n");
		        status_led (RGB_GLOW_RED);

		   for(;;)
		         {
		             /*Waits forever when SMIF initialization error occurs*/
		         }
		     }
	     /* Configure slave select and data select. These settings depend on the pins
	      * selected in the Device and QSPI configurators.
	      */

		   Cy_SMIF_SetDataSelect(KIT_FRAM_HW, CY_SMIF_SLAVE_SELECT_2, CY_SMIF_DATA_SEL0);
	       Cy_SMIF_Enable(KIT_FRAM_HW, &KIT_FRAM_context);
	       NVIC_EnableIRQ(smifIntConfig.intrSrc); /* Enable the SMIF interrupt */

		   printf("\r\nSMIF hardware block is initialized");
		   printf("\r\n===========================================================");
}

/*******************************************************************************
* Function Name: CheckStatus
****************************************************************************//**
*
* Prints the message, indicates the non-zero status by turning the LED on, and
* asserts the non-zero status.
*
* \param message - message to print if status is non-zero.
*
* \param status - status for evaluation.
*
*******************************************************************************/
void CheckStatus(char *message, uint32_t status)
{
    if(0u != status)
    {
    	printf("\r\n================================================================================");
        printf("\r\nFAIL: %s", message);
        printf("\r\nError Code: 0x%08uX", (unsigned int) status);
        printf("\r\n================================================================================\n");

        status_led (RGB_GLOW_RED);

        for(;;)
          {
             /*Waits forever when SMIF initialization error occurs*/
          }
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
    printf("\n%s (%lu-byte):", message, size);

    for(uint32_t index = 0; index < size; index++)
    {
        printf("0x%02X ", buf[index]);

        if(0u == ((index + 1) % NUM_BYTES_PER_LINE))
        {
        	printf("\r\n");
        }
    }
}

/*************************************************************************************
* Function Name: main
**************************************************************************************
********RGB LED turns ON (GREEN - pass, RED -fail) for 1 second after every
******** execution and then turns off to turn ON again for the next execution result**

*Turns ON GREEN LED after successful SMIF Initialization
*
*Executes write and read access to the Status Register -
*
*Executes random (1-Byte) memory write, read and verify from an address
*
*Executes burst (256-Byte) memory write, read and verify from a start address
*
*Executes burst (256-Byte) special sector write, burst read and verify from address 0x00
*
*Executes Serial Number write, read and verify
*
* Parameters:
*  None
*
* Return:
*  int
*
*************************************************************************************/

int main(void)
{
 	 uint8_t extMemAddress[ADDRESS_SIZE];          /* Memory address for write and read access */
     uint8_t write_fram_buffer[PACKET_SIZE];       /* Buffer for memory write in burst mode */
     uint8_t read_fram_buffer[PACKET_SIZE];        /* Buffer to read memory in burst mode */
     uint8_t regwrite_fram_buffer[SR_SIZE];        /* Buffer for register write */
     uint32 loopcount=0x00;                        /* Loop count for For loops */
     uint8_t testResult = TEST_PASS;               /* Test result status */
     cy_en_smif_slave_select_t fram_slave_select;  /* Slave select control for on-board FRAM */

	/* Set up the device based on configurator selections */
	 init_cycfg_all();

	 __enable_irq();

	  Cy_GPIO_Pin_Init (	RGB_RED_PORT, RGB_RED_PIN , &RGB_RED_config);
	  Cy_GPIO_Pin_Init (	RGB_GREEN_PORT, RGB_GREEN_PIN, &RGB_GREEN_config);
      Cy_SCB_UART_Init(KIT_UART_HW, &KIT_UART_config, &KIT_UART_context);

      Cy_SCB_UART_Enable(KIT_UART_HW );

      Init_SMIF(); //Initializes SMIF block
      fram_slave_select = CY_SMIF_SLAVE_SELECT_2;

      /* Status LED; starts with GREEN */
	   status_led (RGB_GLOW_GREEN); /* Turns GREEN LED ON */
	   CyDelay(LED_TOGGLE_DELAY_MSEC); /* STATUS LED ON TIME */

	   /******************************************/
	   /*******SPI F-RAM Access Examples****/
	   /******************************************/

	   /******************************************/
	   /***********Read - 9 Byte ID ***************/
	   /******************************************/
	   FramCmdRDID(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, read_fram_buffer, DID_REG_SIZE);
	   PrintArray("\r\n9-byte Device ID read - ", read_fram_buffer, DID_REG_SIZE);

       printf("\r\n============================================");

	   /******************************************/
	   /***********Write Status Register**********/
	   /******************************************/

       /* Write Status Register - 1-Byte */
	   /* ||Bit7||Bit6||Bit5||Bit4||Bit3) ||Bit2  ||Bit1  ||Bit0|| */
	   /* ||WPEN||X(0)||X(0)||X(1)||BP1(0)||BP0(0)||WEL(0)||X(0)|| */

	   printf("\r\n\r\nWrite Status Register (WRSR 0x01) ");

	   regwrite_fram_buffer [0] = 0x0C; /* Byte to write into status register */
	   FramCmdWRSR(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context,regwrite_fram_buffer,SR_SIZE);

	   PrintArray("\r\nWrite Data - ", regwrite_fram_buffer, SR_SIZE);

	   /******************************************/
	   /***********Read Status Register***********/
	   /******************************************/
	   printf("\r\n\r\nRead Status Register (RDSR 0x05) ");

	   FramCmdRDSR(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, read_fram_buffer, SR_SIZE);

	   PrintArray("\r\nStatus Reg Value - ", read_fram_buffer, SR_SIZE);

	   /*Clear the status register memory block protect setting to
        *enable memory write in follow on sections***************/
       printf("\r\n\r\nClear the Status Register (WRSR 0x01) ");

	   regwrite_fram_buffer [0] = 0x00;  /* ||WPEN||X(0)||X(0)||X(1)||BP1(0)||BP0(0)||WEL(0)||X(0)|| */
	   FramCmdWRSR(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context,regwrite_fram_buffer,SR_SIZE);

	   PrintArray("\r\nWrite Data - ",regwrite_fram_buffer, SR_SIZE);

	   /* Read Status Register - 1-Byte */
	   printf("\r\n\r\nRead Status Register (RDSR 0x05) ");

	   FramCmdRDSR(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, read_fram_buffer, SR_SIZE);

	   PrintArray("\r\nDefault Status Reg Value - ",read_fram_buffer, SR_SIZE);

	   /******************************************/
	   /***1-Byte Memory Write and Read Example***/
	   /******************************************/
	   status_led (RGB_GLOW_OFF); /* Turn off the Status LED before the next text */
	   CyDelay(LED_TOGGLE_DELAY_MSEC); /* STATUS LED ON TIME */

	   /* Write 1 byte EXAMPLE_DATA_BYTE at EXAMPLE_ANY_ADDRESS */
	   write_fram_buffer[0] = EXAMPLE_DATA_BYTE;

	   printf("\r\n\nMemory Write (WRITE 0x02)1-Byte");
	   extMemAddress [2]=EXAMPLE_ANY_ADDR;
	   extMemAddress [1]=EXAMPLE_ANY_ADDR>>8;
	   extMemAddress [0]=EXAMPLE_ANY_ADDR>>16;

	   FramCmdWRITE(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, write_fram_buffer ,0x01, extMemAddress);

	   /* Send the start address and write data to UART for display */
	   PrintArray("\r\nWrite Address: ",extMemAddress, 0x03);
       PrintArray("\r\nWrite Data (1-Byte): ", write_fram_buffer, 0x01);

      /* Read 1-byte from EXAMPLE_ANY_ADDRESS */
       printf("\r\n\nMemory Read (READ 0x03) 1-Byte");
	   extMemAddress [2]=EXAMPLE_ANY_ADDR;
	   extMemAddress [1]=EXAMPLE_ANY_ADDR>>8;
	   extMemAddress [0]=EXAMPLE_ANY_ADDR>>16;

	   FramCmdREAD(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, read_fram_buffer, 0x01, extMemAddress);

	   if ( read_fram_buffer[0]!=write_fram_buffer[0])
	        {
		      status_led (RGB_GLOW_RED); /*Turns RED LED ON*/
		      printf("\r\nRead 1-Byte Fail");
		      CyDelay(LED_TOGGLE_DELAY_MSEC);
		     }

		else
		    {
		     status_led (RGB_GLOW_GREEN); /* Turns GREEN LED ON */
		     /* Send the start address read data to UART for display */
		     PrintArray("\r\nRead Address: ",extMemAddress, 0x03);
		     PrintArray("\r\nRead Data (1-Byte): ",read_fram_buffer, 0x01);
		     printf("\r\nRead 1-Byte Pass");

		     CyDelay(LED_TOGGLE_DELAY_MSEC);
		     }

	   /*******************************************/
	   /***256 Byte Memory Write and Read Example***/
	   /*******************************************/
		status_led (RGB_GLOW_OFF); /* Turn off the Status LED before the next text */
		CyDelay(LED_TOGGLE_DELAY_MSEC); /* STATUS LED ON TIME */

		/* Write 256 bytes write_fram_buffer at extmemAddress */
		/* Set the write buffer and clear the read buffer */

       for(loopcount = 0; loopcount < PACKET_SIZE; loopcount++)
   	    {
		  read_fram_buffer[loopcount] = 0;
		  write_fram_buffer[loopcount] = loopcount;
		}

       extMemAddress [2]=EXAMPLE_INITIAL_ADDR;
	   extMemAddress [1]=EXAMPLE_INITIAL_ADDR>>8;
	   extMemAddress [0]=EXAMPLE_INITIAL_ADDR>>16;

	   FramCmdWRITE(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context,write_fram_buffer ,PACKET_SIZE, extMemAddress);
	   printf("\r\n\r\nWrite Data (256-Byte):");

	   /* Send the start address and write data bytes to UART for display */
	   PrintArray("\r\nWrite Address: ", extMemAddress, 0x03);
	   PrintArray("\r\nWrite Data: ",write_fram_buffer, PACKET_SIZE);

	   /* Read 256 bytes in read_fram_buffer from extmemAddress */
	   FramCmdREAD(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, read_fram_buffer, PACKET_SIZE, extMemAddress);

	   /* Send the start address and read data bytes to UART for display */
	   printf("\r\n\r\nRead Data (256-Byte): ");

	   PrintArray("\r\nRead Address: ",extMemAddress, 0x03);
	   PrintArray("\r\nRead Data: ", read_fram_buffer, PACKET_SIZE);

	   testResult = TEST_PASS;

	   for(loopcount=0;loopcount<PACKET_SIZE;loopcount++)
	   {
	     if ( read_fram_buffer[loopcount]!=write_fram_buffer[loopcount])
	        {
	          printf("\r\nRead Data Fail: ");
	          status_led (RGB_GLOW_RED); /* Turns RED LED ON */
	          CyDelay(LED_TOGGLE_DELAY_MSEC);
	          testResult = TEST_FAIL;
	          break;
	         }
	   }

	   if(!testResult)
	        {
	          printf("\r\nRead Data Pass ");
	          status_led (RGB_GLOW_GREEN); /* Turns GREEN LED ON */
	          CyDelay(LED_TOGGLE_DELAY_MSEC);
	         }

	   /*******************************************/
	   /********256 Byte Fast Read Example*********/
	   /*******************************************/
       FramCmdFSTRD(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, read_fram_buffer, PACKET_SIZE, extMemAddress);

       /* Send the start address and read data bytes to UART for display */
	   printf("\r\n\r\nFast Read Data (256-Byte):\n");

	   PrintArray("\r\nRead Address: ",extMemAddress, 0x03);
	   PrintArray("\r\nRead Data: ",read_fram_buffer, PACKET_SIZE);

	   testResult = TEST_PASS;

	   	   for(loopcount=0;loopcount<PACKET_SIZE;loopcount++)
	   	   {
	   	     if ( read_fram_buffer[loopcount]!=write_fram_buffer[loopcount])
	   	        {
	   	          printf("\r\nFast Read Data Fail: ");
	   	          status_led (RGB_GLOW_RED); /* Turns RED LED ON */
	   	          CyDelay(LED_TOGGLE_DELAY_MSEC);
	   	          testResult = TEST_FAIL;
	   	          break;
	   	         }
	   	   }

	   	   if(!testResult)
	   	        {
	   	          printf("\r\nFast Read Data Pass ");
	   	          status_led (RGB_GLOW_GREEN); /* Turns GREEN LED ON */
	   	          CyDelay(LED_TOGGLE_DELAY_MSEC);
	   	         }

	   /********************************************/
	   /***256-Byte Special Sector Write and Read ***/
	   /********************************************/
	   status_led (RGB_GLOW_OFF); /* Turn off the Status LED before the next text */
	   CyDelay(LED_TOGGLE_DELAY_MSEC);

	  /* Set the write buffer and clear the read buffer */
	   for(loopcount = 0; loopcount < PACKET_SIZE; loopcount++)
		{
    	  read_fram_buffer[loopcount] = 0;
		  write_fram_buffer[loopcount] = loopcount+0x0A;
		}

	   /* Write 256 bytes write_fram_buffer at extmemAddress */
	   extMemAddress [2]=EXAMPLE_INITIAL_ADDR_SS;
	   extMemAddress [1]=EXAMPLE_INITIAL_ADDR_SS>>8;
	   extMemAddress [0]=EXAMPLE_INITIAL_ADDR_SS>>16;

	   FramCmdSSWR(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context,write_fram_buffer,PACKET_SIZE, extMemAddress);

	   printf("\r\n\r\nWrite Special Sector (256-Byte): ");

	   /* Send the start address and write data bytes to UART for display */
       PrintArray("\r\nSpecial Sector Write Address: ",extMemAddress, 0x03);
       PrintArray("\r\nSpecial Sector Write Data: ",write_fram_buffer, PACKET_SIZE);

      /* Read 256 bytes in read_fram_buffer from extmemAddress */
      FramCmdSSRD(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, read_fram_buffer, PACKET_SIZE, extMemAddress);

      /* Send the start address and read data bytes to UART for display */
      printf("\r\n\r\nSpecial Sector Read (256-Byte): ");
      PrintArray("\r\nSpecia Sector Read Address: ",extMemAddress, 0x03);
      PrintArray("\r\nRead Special Sector: ",read_fram_buffer, PACKET_SIZE);

      testResult = TEST_PASS;
	  for(loopcount=0;loopcount<PACKET_SIZE;loopcount++)
	   {
	     if ( read_fram_buffer[loopcount]!=write_fram_buffer[loopcount])
	         {
	          printf("\r\nRead Special Sector Fail: ");
	          status_led (RGB_GLOW_RED); /* Turns RED LED ON */
	          CyDelay(LED_TOGGLE_DELAY_MSEC);
	          testResult = TEST_FAIL;
	          break;
	         }
	   }

	  if(!testResult)
		{
		 printf("\r\nRead Special Sector Pass ");
		 status_led (RGB_GLOW_GREEN); /* Turns GREEN LED ON */
		 CyDelay(LED_TOGGLE_DELAY_MSEC);
        }

	  /******************************************************/
	  /***********Write and Read 8-Byte Serial Number********/
	  /******************************************************/
	  status_led (RGB_GLOW_OFF); /* Turn off the Status LED before the next text */
	  CyDelay(LED_TOGGLE_DELAY_MSEC);

	  /* Set the write buffer and clear the read buffer */
      for(loopcount = 0; loopcount < SN_BUF_SIZE; loopcount++)
       {
		read_fram_buffer[loopcount] = 0;
		write_fram_buffer[loopcount] = loopcount+0xC0;
	   }

      /* Write 8 bytes write_fram_buffer into serial number registers */
      printf("\r\n\r\nWriteSerial Number (WRSN 0xC2)  8-Byte : ");
      FramCmdWRSN(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context,write_fram_buffer, SN_BUF_SIZE);
      PrintArray("\r\nSerial Number: ",write_fram_buffer, SN_BUF_SIZE);

      /* Read Device Serial No - 8-Byte */
      printf("\r\n\r\nRead Serial Number (RDSN 0xC3) 8-Byte : ");

      FramCmdRDSN(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context,&read_fram_buffer[0],SN_BUF_SIZE);
      PrintArray("\r\nSerial Number: ",&read_fram_buffer[0], SN_BUF_SIZE);

      for(loopcount=0;loopcount<SN_BUF_SIZE;loopcount++)
	   {
		if ( read_fram_buffer[loopcount]!=write_fram_buffer[loopcount])
		    {
		     printf("\r\nRead Serial Number Fail: ");
		     status_led (RGB_GLOW_RED); /*Turns RED LED ON*/
		     CyDelay(LED_TOGGLE_DELAY_MSEC);
		     testResult = TEST_FAIL;
		     break;
		     }
		}

      if(!testResult)
		    {
		     printf("\r\nRead Serial Number Pass ");
		     status_led (RGB_GLOW_GREEN); /*Turns GREEN LED ON*/
		     CyDelay(LED_TOGGLE_DELAY_MSEC);
		     }

      printf("\r\nEnd of Example Project: ");
      printf("\r\n=========================================================================\n");

      for(;;)
         {
          /*Loops forever*/
          /* CM4 does nothing after SMIF operation is complete. */
	      }

}
