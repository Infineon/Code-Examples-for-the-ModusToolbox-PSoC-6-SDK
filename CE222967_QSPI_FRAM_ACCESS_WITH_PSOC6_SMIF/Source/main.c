/****************************************************************************
*File Name: main_cm4.c
*
* Version: 1.0
*
* Description: This code example demonstrates the Excelon Ultra
*              QSPI F-RAM access using PSoC6 SMIF.
*
* Related Document: CE222967_QSPI_FRAM_ACCESS_WITH_PSOC6_SMIF.pdf
*
* Hardware Dependency: CE222967_SPI_FRAM_ACCESS_WITH_PSOC6_SMIF.pdf
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
#include "qspi_fram_apis.h"
#include "stdio_user.h"

/***************************************************************************
* Global constants
***************************************************************************/
#define SMIF_PRIORITY           (1u)     /* SMIF interrupt priority */
#define PACKET_SIZE             (256u)     /* The memory Read/Write packet */
#define NUM_BYTES_PER_LINE		(16u)	 /* Used when array of data is printed on the console */
#define MODE_BYTE               (0x00)   /* non XIP */
#define TEST_PASS               (0x00)
#define TEST_FAIL               (0x01)

/*RGB LED control*/
#define RGB_GLOW_OFF          (0x03)
#define RGB_GLOW_RED          (0x01)
#define RGB_GLOW_GREEN        (0x02)
#define LED_TOGGLE_DELAY_MSEC (1000u)	/* LED blink delay */


/***************************************************************************
* Global variables and functions
***************************************************************************/
cy_stc_scb_uart_context_t KIT_UART_context;
cy_stc_smif_context_t KIT_FRAM_context;
cy_en_smif_slave_select_t fram_slave_select;  /* Slave select control for on-board FRAM */

uint32_t EXAMPLE_INITIAL_ADDR = 0x00;    /* Example F-RAM Initial Address */
uint32_t EXAMPLE_INITIAL_ADDR_SS = 0x00; /* Example F-RAM Initial Address for Special Sector */
uint32_t EXAMPLE_ANY_ADDR =0x123456;     /* Example F-RAM Any Address */
uint32_t CONFIG_REG2_ADDR =0x700003;     /* Configuration Register 2 (CR2) address */
                                         /* Address = 0x700003 is a volatile write and doesn't survive power cycle or HW reset */
                                         /* Use address = 0x000003 for nonvolatile configuration setting */

uint32_t ACCESS_MODE = SPI_MODE;         /* Sets the host controller access mode */
uint8_t EXAMPLE_DATA_BYTE = 0xCA;       /* Example F-RAM Data */
uint8_t MLC=0x06;                       /* Sets max latency for memory read at 100 MHz*/
 int8_t RLC=0x01;                       /* Sets max latency for register read at 100 MHz*/
                                         /* Refer to QSPI F-RAM (CY15x104QSN)datasheet for details*/

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

/*******************************************************************************
* Function Name: PowerUpMemoryDefaultSPI
****************************************************************************//**
*
* This functions sets the F-RAM to SPI mode.
* Configures all user registers (status and configuration registers) to factory default.
*
*******************************************************************************/

void PowerUpMemoryDefaultSPI (void)
{
   /* Write CR2 for interface mode */
    {
        uint8_t CR2_Address[3] = {0x07, 0x00, 0x03}; /* CR2 register address 0x000003 */
        uint8_t CR2_Value[1] = {0x00};              /* CR2 value to set the SPI mode */

        /* Try in QPI mode*/
        FramCmdSPIWriteAnyReg(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, CR2_Value, sizeof CR2_Value, CR2_Address, QPI_MODE);

        /* Try in DPI mode*/
        FramCmdSPIWriteAnyReg(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, CR2_Value, sizeof CR2_Value, CR2_Address, DPI_MODE);
    }

    /* Write CR1 for memory latency setting */
    {
        uint8_t CR1_Address[3] = {0x07, 0x00, 0x02};  /* CR1 register address 0x000002 */
        uint8_t CR1_Value[1] = {(MLC<<4)|0x00};      /* CR1 value to set the memory access latency */

        FramCmdSPIWriteAnyReg(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, CR1_Value, sizeof CR1_Value, CR1_Address, SPI_MODE);
    }

     /* Write CR5 for register latency setting */
    {
        uint8_t CR5_Address[3] = {0x07, 0x00, 0x06};  /* CR5 register address 0x000006 */
        uint8_t CR5_Value[1] = {(RLC<<6)};           /* CR5 value to set the register access latency */

        FramCmdSPIWriteAnyReg(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, CR5_Value, sizeof CR5_Value, CR5_Address, SPI_MODE);
    }
}

/*******************************************************************************
* Function Name: FactoryDefault
****************************************************************************//**
*
* This functions resets the status and configuration register content to factory default.
*
*******************************************************************************/

void FactoryDefault (void)
{
   MLC =0x00;
   RLC =0x00;
   /* Write CR2 for interface mode */
    {
        uint8_t CR2_Address[3] = {0x00, 0x00, 0x03}; /* CR2 register address 0x000003 */
        uint8_t CR2_Value[1] = {0x00};              /* CR2 value to set the SPI mode */

        /* Try in QPI mode*/
        FramCmdSPIWriteAnyReg(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, CR2_Value, sizeof CR2_Value, CR2_Address, QPI_MODE);

        /* Try in DPI mode*/
        FramCmdSPIWriteAnyReg(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, CR2_Value, sizeof CR2_Value, CR2_Address, DPI_MODE);
    }

    /* Write CR1 for memory latency setting */
    {
        uint8_t CR1_Address[3] = {0x00, 0x00, 0x02};  /* CR1 register address 0x000002 */
        uint8_t CR1_Value[1] = {(MLC<<4)|0x00};      /* CR1 value to set the memory access latency */

        FramCmdSPIWriteAnyReg(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, CR1_Value, sizeof CR1_Value, CR1_Address, SPI_MODE);
    }

     /* Write CR5 for register latency setting */
    {
        uint8_t CR5_Address[3] = {0x00, 0x00, 0x06};  /* CR5 register address 0x000006 */
        uint8_t CR5_Value[1] = {(RLC<<6)};           /* CR5 value to set the register access latency */

        FramCmdSPIWriteAnyReg(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, CR5_Value, sizeof CR5_Value, CR5_Address, SPI_MODE);
    }
}


/*********************************************************************************************
*This code example tests the following features of QSPI F-RAM using PSoC6 SMIF
*********************************************************************************************
*
*Set the device access mode to default SPI mode. This ensure part starts with a known SPI mode
*
*Read two Status and four Configuration registers (SR1, SR2, CR1, CR2, CR4, CR5)
*
*Read 8-byte device ID read
*
*Write 256 bytes into F-RAM at a given address, in SPI mode
*
*Read 256 bytes from F-RAM at a given address, using READ (0x02) opcode in SPI mode
*
*Read 256 bytes from F-RAM at a given address, using READ (0x02) opcode in DPI mode
*
*Read 256 bytes from F-RAM at a given address, using READ (0x02) opcode in QPI mode
*
*Read 256 bytes from F-RAM at a given address, using FastRead (0x0B) opcode in QPI mode
*
*Write and read 256 bytes special sector using SSWR and SSRD commands in QPI mode
**********************************************************************************************/

int main(void)
{

 	 uint8_t extMemAddress[ADDRESS_SIZE];   /* Memory address for write and read access */
	 uint8_t extMemAddressWithMode[ADDRESS_PLUS_MODE_SIZE];/* Memory address for read access */
	                                                       /* with mode bye */
     uint8_t write_fram_buffer[PACKET_SIZE];   /* Buffer for memory writr in burst mode */
     uint8_t read_fram_buffer[PACKET_SIZE];    /* Buffer to read memory in burst mode */
     uint8_t regwrite_fram_buffer[SR_SIZE];   /* Buffer for register write */
     uint32 loopcount=0x00;                        /* Loop count for For loops */
     uint8_t testResult = TEST_PASS;               /* Test result status */

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

     /* Sets the device access mode to default SPI */
	 PowerUpMemoryDefaultSPI ();

	 ACCESS_MODE =SPI_MODE;   /*Set the SPI Access Mode*/

	 /******************************************/
	 /*******SPI/QSPI F-RAM Access Examples****/
	 /******************************************/

	 /******************************************/
	 /*Read-Status and Configuration Registers**/
	 /******************************************/

     FramCmdReadSRx(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context,&read_fram_buffer[0],1, ACCESS_MODE, MEM_CMD_RDSR1, RLC);
	 PrintArray("\r\n Read SR1 - ", &read_fram_buffer[0], 1);

	 FramCmdReadSRx(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context,&read_fram_buffer[0],1, ACCESS_MODE, MEM_CMD_RDSR2, RLC);
	 PrintArray("\r\n Read SR2 - ", &read_fram_buffer[0], 1);

	 FramCmdReadCRx(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context,&read_fram_buffer[0],1, ACCESS_MODE, MEM_CMD_RDCR1, RLC);
	 PrintArray("\r\n Read CR1 - ", &read_fram_buffer[0], 1);

	 FramCmdReadCRx(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context,&read_fram_buffer[0],1, ACCESS_MODE, MEM_CMD_RDCR2, RLC);
	 PrintArray("\r\n Read CR2 - ", &read_fram_buffer[0], 1);

	 FramCmdReadCRx(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context,&read_fram_buffer[0],1, ACCESS_MODE, MEM_CMD_RDCR4, RLC);
	 PrintArray("\r\n Read CR4 - ", &read_fram_buffer[0], 1);

	 FramCmdReadCRx(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context,&read_fram_buffer[0],1, ACCESS_MODE, MEM_CMD_RDCR5, RLC);
	 PrintArray("\r\n Read CR5 - ", &read_fram_buffer[0], 1);
	 printf("\r\n============================================\r\n");

	 /******************************************/
	 /***********Read - 8Byte ID ***************/
	 /******************************************/

	 FramCmdRDID(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context,&read_fram_buffer[0], ACCESS_MODE, RLC);
	 PrintArray("\r\n 8-byte Device ID read - ", &read_fram_buffer[0], DID_REG_SIZE);

	 FramCmdRDSN(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context,&read_fram_buffer[0],DID_REG_SIZE, ACCESS_MODE,RLC);
	 PrintArray("\r\n 8-byte Device ID read - ", &read_fram_buffer[0], SN_BUF_SIZE);

	 FramCmdRDUID(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context,&read_fram_buffer[0],ACCESS_MODE, RLC);
	 PrintArray("\r\n 8-byte UNIQUE ID read - ", &read_fram_buffer[0], UID_BUF_SIZE);
	 printf("\r\n============================================\r\n");

	 /******************************************************/
	 /***********Write and Read 8-Byte Serial Number********/
	 /******************************************************/

     status_led (RGB_GLOW_OFF); /* Turn off the Status LED before the next text */
     CyDelay(LED_TOGGLE_DELAY_MSEC); /* STATUS LED ON TIME */

     /* Set the write buffer and clear the read buffer */

     for(loopcount = 0; loopcount < SN_BUF_SIZE; loopcount++)
      {
		read_fram_buffer[loopcount] = 0;
		write_fram_buffer[loopcount] = loopcount+0x00;
	  }

      /* Write 8 bytes write_fram_buffer in SN register */

	  FramCmdWRSN(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context,&write_fram_buffer[0], SN_BUF_SIZE,ACCESS_MODE);
	  PrintArray("\r\n\r\nWriteSerial Number (WRSN 0xC2) : ", &write_fram_buffer[0], SN_BUF_SIZE);

	  /* Read 8-Byte Serial No - */

	  FramCmdRDSN(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context,&read_fram_buffer[0],SN_BUF_SIZE, ACCESS_MODE,RLC);
	  PrintArray("\r\nSerial Number: ", &read_fram_buffer[0], SN_BUF_SIZE);

	  testResult = 0x00;
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
		      printf("\r\n============================================\r\n");

     /********************************************/
	 /***256-Byte memory Write and Read in SPI***/
	 /********************************************/

	  status_led (RGB_GLOW_OFF); /* Turn off the Status LED before the next text */
	  CyDelay(LED_TOGGLE_DELAY_MSEC);

	 /* Fill the write buffer and clear the read buffer */

	 for(loopcount = 0; loopcount < PACKET_SIZE; loopcount++)
      {
		write_fram_buffer[loopcount] = 0xA0+loopcount;
		read_fram_buffer[loopcount] = 0x00;
	  }

     /* Set the memAddress with start address for write */

	 extMemAddress [2]=EXAMPLE_INITIAL_ADDR;
	 extMemAddress [1]=EXAMPLE_INITIAL_ADDR>>8;
	 extMemAddress [0]=EXAMPLE_INITIAL_ADDR>>16;

	/* Write 256 bytes from write_fram_buffer at extmemAddress */

	 FramCmdSPIWrite(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context,&write_fram_buffer[0],PACKET_SIZE, extMemAddress,ACCESS_MODE);
	 printf("\r\n\r\nWrite memery (WRITE 0x02) 256-Byte in SPI: ");

	/* Dispaly the start address on UART */

	PrintArray("\r\nMemory Write Address: ", extMemAddress, 0x03);

	PrintArray("\r\nMemory Write Data: ", &write_fram_buffer[0], PACKET_SIZE);

	/* Read 256 bytes in read_fram_buffer from extmemAddress */

	FramCmdSPIRead(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, &read_fram_buffer[0], PACKET_SIZE, extMemAddress, ACCESS_MODE, MLC);

	/* Send the start address and read data bytes to UART for display */

	printf("\r\n\r\nMemory Read (READ 0x03) in SPI 256-Byte: ");
	PrintArray("\r\nMemory Read Address: ", extMemAddress, 0x03);

	PrintArray("\r\nRead Memory in SPI: ",&read_fram_buffer[0], PACKET_SIZE);
	testResult = 0x00;

	for(loopcount=0;loopcount<PACKET_SIZE;loopcount++)
	 {
	  if ( read_fram_buffer[loopcount]!=write_fram_buffer[loopcount])
	      {
	       printf("\r\nRead Memory in SPI Fail: ");
		   status_led (RGB_GLOW_RED); /* Turns RED LED ON */
		   CyDelay(LED_TOGGLE_DELAY_MSEC);
		   testResult = TEST_FAIL;
		   break;
		   }
       }

		if(!testResult)
		  {
		   printf("\r\nRead Memory in SPI Pass ");
		   status_led (RGB_GLOW_GREEN); /* Turns GREEN LED ON */
		   CyDelay(LED_TOGGLE_DELAY_MSEC);
		  }

    /********************************************/
    /***256-Byte memory Read in DPI***/
    /********************************************/

	status_led (RGB_GLOW_OFF); /* Turn off the Status LED before the next test */
	CyDelay(LED_TOGGLE_DELAY_MSEC);

	extMemAddress [2]=CONFIG_REG2_ADDR;
	extMemAddress [1]=CONFIG_REG2_ADDR>>8;
	extMemAddress [0]=CONFIG_REG2_ADDR>>16;
	regwrite_fram_buffer[0] = 0x10;

	/* Write 0x10 to CR2 register to set the access mode mode to DPI (CR2[4] = 1)*/
	FramCmdSPIWriteAnyReg(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, &regwrite_fram_buffer[0],0x01, extMemAddress, ACCESS_MODE);

	/* Set the access mode for SMIF transfer*/
	ACCESS_MODE = DPI_MODE;

	extMemAddress [2]=EXAMPLE_INITIAL_ADDR;
	extMemAddress [1]=EXAMPLE_INITIAL_ADDR>>8;
	extMemAddress [0]=EXAMPLE_INITIAL_ADDR>>16;

	testResult = 0x00;

	for(loopcount = 0; loopcount < PACKET_SIZE; loopcount++)
	    read_fram_buffer[loopcount] = 0x00;

	FramCmdSPIRead(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, &read_fram_buffer[0], PACKET_SIZE, extMemAddress, ACCESS_MODE, MLC);

	/* Send the start address and read data bytes to UART for display */

	printf("\r\n\r\nMemory Read (READ 0x03) in DPI 256-Byte: ");
    PrintArray("\r\nMemory Read Address: ",extMemAddress, 0x03);

    PrintArray("\r\nRead Memory in DPI: ",&read_fram_buffer[0], PACKET_SIZE);

    for(loopcount=0;loopcount<PACKET_SIZE;loopcount++)
	 {
	  if ( read_fram_buffer[loopcount]!=write_fram_buffer[loopcount])
	      {
	        printf("\r\nRead Memory in DPI Fail: ");
	        status_led (RGB_GLOW_RED); /* Turns RED LED ON */
	        CyDelay(LED_TOGGLE_DELAY_MSEC);
	        testResult = TEST_FAIL;
	        break;
	       }
	 }

	  if(!testResult)
        {
         printf("\r\nRead Memory in DPI Pass ");
         status_led (RGB_GLOW_GREEN); /* Turns GREEN LED ON */
	     CyDelay(LED_TOGGLE_DELAY_MSEC);
	    }

	/******************************************/
	/*******256-Byte memory Read in QPI********/
	/******************************************/

	status_led (RGB_GLOW_OFF); /* Turn off the Status LED before the next text */
	CyDelay(LED_TOGGLE_DELAY_MSEC);

	extMemAddress [2]=CONFIG_REG2_ADDR;
	extMemAddress [1]=CONFIG_REG2_ADDR>>8;
	extMemAddress [0]=CONFIG_REG2_ADDR>>16;

	regwrite_fram_buffer[0] = 0x40;

	/* Write 0x40 to CR2 register to set the access mode mode to QPI (CR2[6] = 1)*/
	FramCmdSPIWriteAnyReg(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, &regwrite_fram_buffer[0],0x01, extMemAddress, ACCESS_MODE);

	/* Set the access mode for SMIF transfer*/
	ACCESS_MODE = QPI_MODE;

	extMemAddress [2]=EXAMPLE_INITIAL_ADDR;
	extMemAddress [1]=EXAMPLE_INITIAL_ADDR>>8;
	extMemAddress [0]=EXAMPLE_INITIAL_ADDR>>16;

	for(loopcount = 0; loopcount < PACKET_SIZE; loopcount++)
	    read_fram_buffer[loopcount] = 0x00;


    FramCmdSPIRead(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, &read_fram_buffer[0], PACKET_SIZE, extMemAddress, ACCESS_MODE, MLC);

    /* Send the start address and read data bytes to UART for display */

	printf("\r\n\r\nMemory Read (READ 0x03) in QPI 256-Byte: ");

	PrintArray("\r\nMemory Read Address: ",extMemAddress, 0x03);
    PrintArray("\r\nRead Memory in QPI: ",&read_fram_buffer[0], PACKET_SIZE);

	testResult = 0x00;
	for(loopcount=0;loopcount<PACKET_SIZE;loopcount++)
	 {
	  if ( read_fram_buffer[loopcount]!=write_fram_buffer[loopcount])
	      {
		   printf("\r\nRead Memory in QPI Fail: ");
		   status_led(RGB_GLOW_RED); /* Turns RED LED ON */
		   CyDelay(LED_TOGGLE_DELAY_MSEC);
		   testResult = TEST_FAIL;
		   break;
		   }
	  }

	  if(!testResult)
	   {
		printf("\r\nRead Memory in QPI Pass ");
		status_led (RGB_GLOW_GREEN); /* Turns GREEN LED ON */
		CyDelay(LED_TOGGLE_DELAY_MSEC);
		}

     /********************************************/
	 /*****256-Byte memory FastRead in QPI********/
	 /********************************************/

	  extMemAddressWithMode [3]=MODE_BYTE;
	  extMemAddressWithMode [2]=EXAMPLE_INITIAL_ADDR;
	  extMemAddressWithMode [1]=EXAMPLE_INITIAL_ADDR>>8;
	  extMemAddressWithMode [0]=EXAMPLE_INITIAL_ADDR>>16;

      for(loopcount = 0; loopcount < PACKET_SIZE; loopcount++)
	      read_fram_buffer[loopcount] = 0x00;

      FramCmdSPIFastRead(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, &read_fram_buffer[0], PACKET_SIZE, extMemAddressWithMode, ACCESS_MODE, MLC);

      /* Send the start address and read data bytes to UART for display */
      printf("\r\n\r\nMemory FastRead (FAST_READ 0x0B) 256-Byte in QPI : ");

      PrintArray("\r\nMemory FastRead Address: ",extMemAddress, 0x03);
      PrintArray("\r\nFastRead Memory in QPI: ",&read_fram_buffer[0], PACKET_SIZE);
      testResult = 0x00;

      for(loopcount=0;loopcount<PACKET_SIZE;loopcount++)
       {
        if ( read_fram_buffer[loopcount]!=write_fram_buffer[loopcount])
            {
             printf("\r\nFastRead Memory Fail in QPI: ");
             status_led (RGB_GLOW_RED); /* Turns RED LED ON */
             CyDelay(LED_TOGGLE_DELAY_MSEC);
             testResult = TEST_FAIL;
             break;
             }
        }
        if(!testResult)
          {
           printf("\r\nFastRead Memory Pass in QPI ");
           status_led (RGB_GLOW_GREEN); /* Turns GREEN LED ON */
           CyDelay(LED_TOGGLE_DELAY_MSEC);
           }

      /********************************************************/
	  /***256-Byte Special Sector Write and Read in QPI********/
	  /********************************************************/

	 status_led (RGB_GLOW_OFF); /* Turn off the Status LED before the next text */
	 CyDelay(LED_TOGGLE_DELAY_MSEC);

	 /* Set the write buffer and clear the read buffer */
     for(loopcount = 0; loopcount < PACKET_SIZE; loopcount++)
	 {
	  write_fram_buffer[loopcount] = loopcount+0x0A;
	  read_fram_buffer[loopcount] = 0x00;
	 }

     /* Write 256 bytes write_fram_buffer at memAddress */
     extMemAddress [2]=EXAMPLE_INITIAL_ADDR_SS;
     extMemAddress [1]=EXAMPLE_INITIAL_ADDR_SS>>8;
     extMemAddress [0]=EXAMPLE_INITIAL_ADDR_SS>>16;
     FramCmdSSWR(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context,&write_fram_buffer[0],PACKET_SIZE, extMemAddress,ACCESS_MODE);
     printf("\r\n\r\nWrite Special Sector (SSWR 0x42) 256-Byte in QPI: ");

     /* Send the start address and write data bytes to UART for display */

	 PrintArray("\r\nSpecial Sector Write Address: ", extMemAddress, 0x03);
	 PrintArray("\r\nSpecial Sector Write Data: ", &write_fram_buffer[0], PACKET_SIZE);

	/* Read 256 bytes in read_fram_buffer from memAddress */

	FramCmdSSRD(fram_slave_select, KIT_FRAM_HW, &KIT_FRAM_context, &read_fram_buffer[0], PACKET_SIZE, extMemAddress, ACCESS_MODE, MLC);

	/* Send the start address and read data bytes to UART for display */

	printf("\r\n\r\nSpecial Sector Read (SSRD 0x4B) 256-Byte in QPI: ");
    PrintArray("\r\nSpecia Sector Read Address: ", extMemAddress, 0x03);

    PrintArray("\r\nRead Special Sector: ", &read_fram_buffer[0], PACKET_SIZE);

    testResult = 0x00;
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

	   printf("\r\n\r\nReset Status and Configuration registers to their factory default values per datasheet ");
	   FactoryDefault();

	   printf("\r\nEnd of Code Example Project: ");
	   printf("\r\n=========================================================================\n");

       for(;;)
        {
         /*Loops forever*/
         /* CM4 does nothing after the example code execution is complete. */
        }

}
