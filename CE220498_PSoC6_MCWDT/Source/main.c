/******************************************************************************
* File Name: main.c
*
* Version 1.1
*
* Description: This example demonstrates Multi-counter watchdog timer (MCWDT)
* counters in free running mode on the PSoC® 6 MCU, using ModusToolbox IDE.
*
* Related Document: CE220498_PSoC6_MCWDT.pdf
*
* Hardware Dependency: PSoC 6 BLE Pioneer Kit, PSoC 6 WiFi-BT Pioneer Kit,
* 					   PSoC 6 Wi-Fi BT Prototyping Kit
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

#include "cy_device_headers.h"
#include "cycfg.h"
#include "cy_mcwdt.h"
#include "stdio.h"

/*******************************************************************************
*        Constants
*******************************************************************************/
#define LED_ON               				(0u)      	/* Value to turn LED ON  */
#define LED_OFF              				(!LED_ON) 	/* Value to turn LED OFF */

/* Switch press/release check interval in milliseconds for debouncing */
#define SWITCH_DEBOUNCE_CHECK_UNIT			(1u)

/* Number of debounce check units to count before considering that switch is pressed
 * or released */
#define SWITCH_DEBOUNCE_MAX_PERIOD_UNITS	(80u)

/* Delay before counter actually starts counting */
#define MCWDT0_THREE_LF_CLK_CYCLES_DELAY 	(94u)

/* LF_CLK Frequency - Source is set to ILO in design.modus. */
#define LF_CLK_FREQUENCY					(32000)		/* Hz (nominal) */

/*******************************************************************************
*        Function Prototypes
*******************************************************************************/
void handle_error(void);
static uint32_t read_switch_status(void);

/*******************************************************************************
*        Variables
*******************************************************************************/
cy_stc_scb_uart_context_t UART_context;


/******************************************************************************
* Function Name: main
*******************************************************************************
*
* Summary: This is the main function for CM4 CPU.
*
* Parameters:
*  None
*
* Return:
* int
*
******************************************************************************/
int main(void)
{
	/* Key press event count value */
	uint32_t event1_cnt, event2_cnt;

	/* The time between two presses of switch */
	uint32_t timegap;

    /* UART initialization status */
	cy_en_scb_uart_status_t uart_init_status;
	cy_en_mcwdt_status_t mcwdt_init_status;

	/* Set up the device based on configurator selections */
    init_cycfg_all();

	/* Configure UART */
	/* Initialize UART based on configuration set in design.modus */
	uart_init_status = Cy_SCB_UART_Init(KIT_UART_HW, &KIT_UART_config, &UART_context);

	if(uart_init_status!=CY_SCB_UART_SUCCESS)
	{
		handle_error();
	}

	/* Enable the SCB block that implements the UART */
	Cy_SCB_UART_Enable(KIT_UART_HW);

	/* Print a message on UART */
	Cy_SCB_UART_PutString(KIT_UART_HW, "\r\n\r\nUART initialization complete\r\n");

	/* Configure MCWDT */
	/* Initialize MCWDT based on configuration set in design.modus */
	/* Configuration to cascade Counter 0 and Counter 1 is done in design.modus */
	mcwdt_init_status = Cy_MCWDT_Init(MCWDT0_HW, &MCWDT0_config);

	if(mcwdt_init_status!=CY_MCWDT_SUCCESS)
	{
		handle_error();
	}

	/* Enable the MCWDT0 counters and give blocking delay of 3 LF_CLKs to
	 * allow configuration is complete */
	Cy_MCWDT_Enable(MCWDT0_HW, CY_MCWDT_CTR0|CY_MCWDT_CTR1, \
						MCWDT0_THREE_LF_CLK_CYCLES_DELAY);

    __enable_irq();

    /* Initialize event count value */
	event1_cnt = 0;
	event2_cnt = 0;
 
    for(;;)
    {
    	/* Check if the switch is pressed */
    	/* Note that, if the switch is pressed, the CPU will not return from
    	 * read_switch_status() function until the switch is released */
		if (0UL != read_switch_status())
		{
			/* Consider previous key press as 1st key press event */
			event1_cnt = event2_cnt;

			/* Consider current key press as 2nd key press event */
			/* Get live counter value from MCWDT0 */
			/* Note that MCWDT0 Counter0 is cascaded to MCWDT0 Counter1 */

			event2_cnt = Cy_MCWDT_GetCountCascaded(MCWDT0_HW);

			/* Calculate the time between two presses of switch and print on the terminal */
			/* MCWDT0 Counter0 and Counter1 are clocked by LFClk = ILO = 32000Hz +/- 10% */

			if(event2_cnt > event1_cnt)
			{
				timegap = (event2_cnt - event1_cnt)/LF_CLK_FREQUENCY;

				/* Print the timegap value */
				printf("\r\nThe time between two presses of switch = %ds\r\n", (unsigned int)timegap);
			}
			else//counter overflow
			{
				/* Print a message on overflow of counter */
				Cy_SCB_UART_PutString(KIT_UART_HW, "\r\n\r\nCounter overflow detected\r\n");
			}
		}
    }
}

/*******************************************************************************
* Function Name: read_switch_status
********************************************************************************
* Summary:
*  Reads and returns the current status of the switch. If the switch is pressed,
*  CPU will be blocking until the switch is released.
*
* Parameters:
*  None
*
* Return:
*  Returns non-zero value if switch is pressed and zero otherwise.
*
*******************************************************************************/
uint32_t read_switch_status(void)
{
    uint32_t delayCounter = 0;
    uint32_t sw_status = 0;

    /* Check if the switch is pressed */
    while (0UL == Cy_GPIO_Read(KIT_BTN1_PORT, KIT_BTN1_NUM))
    {
    	/* Switch is pressed. Proceed for debouncing. */

        Cy_SysLib_Delay(SWITCH_DEBOUNCE_CHECK_UNIT);
        ++delayCounter;

        /* Keep checking the switch status till the switch is pressed for a
         * minimum period of SWITCH_DEBOUNCE_CHECK_UNIT x SWITCH_DEBOUNCE_MAX_PERIOD_UNITS */
        if (delayCounter > SWITCH_DEBOUNCE_MAX_PERIOD_UNITS)
        {
            /* Wait till the switch is released */
            while (0UL == Cy_GPIO_Read(KIT_BTN1_PORT, KIT_BTN1_NUM))
            {
            }

            /* Debounce when the switch is being released */
            do
            {
            	delayCounter = 0;

                while(delayCounter < SWITCH_DEBOUNCE_MAX_PERIOD_UNITS)
                {
                    Cy_SysLib_Delay(SWITCH_DEBOUNCE_CHECK_UNIT);
                    ++delayCounter;
                }

            }while (0UL == Cy_GPIO_Read(KIT_BTN1_PORT, KIT_BTN1_NUM));

            /* Switch is pressed and released*/
            sw_status = 1u;
        }
    }

    return (sw_status);
}

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* This function processes unrecoverable errors such as UART initialization error.
* In case of such error the system will turn on LED and stay in an infinite
* loop of this function.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void handle_error(void)
{
     /* Disable all interrupts */
    __disable_irq();

    /* Turn on error LED */
    Cy_GPIO_Write(KIT_LED2_PORT, KIT_LED2_NUM, LED_ON);
    while(1u) {}
}

