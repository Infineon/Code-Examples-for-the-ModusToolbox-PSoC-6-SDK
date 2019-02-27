/******************************************************************************
* File Name: main.c
*
* Version: 1.00
*
* Description: This example demonstrates basic GPIO pins operation in ModusToolbox.
*
* Related Document: CE220263_GPIO_Pins.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit or
*                      CY8CKIT-062 PSoC 6 Pioneer Kit
*
******************************************************************************
* Copyright (2019), Cypress Semiconductor Corporation.
******************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a Cypress integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the
* materials described herein. Cypress does not assume any liability arising out
* of the application or use of any product or circuit described herein. Cypress
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*****************************************************************************/
#include "cy_device_headers.h"
#include "cycfg.h"
#include "cy_sysint.h"

/* This project assumes ModusToolbox automatically configures all GPIO */
/* pins of the device on a supported development kit. Some of the examples */
/* use #defines hard coded to specific pins to demonstrate the direct PDL use case. */
/* These hard coded pins may require manual re-mapping to work on all kits. */
/* To demonstrate how PDL drivers are used to manually configure GPIO pins, */
/* set the PDL_PIN_CONFIGURATION #define to 1, otherwise leave set to 0. */
#define PDL_PIN_CONFIGURATION   (0u)

#define GPIO_ISR_FLAG           (1u)
#define GPIO_ISR_MASKED         (1u)
#define PIN_DELAY               (0x00030000u)
#define INTCAUSE0_PORT0         (1u)


#if PDL_PIN_CONFIGURATION
	/* This structure is used to initialize a single GPIO pin using PDL configuration. */
	const cy_stc_gpio_pin_config_t P0_4_Pin_Init =
	{
		.outVal     = 1u,                   /* Pin output state */
		.driveMode  = CY_GPIO_DM_PULLUP,    /* Drive mode */
		.hsiom      = HSIOM_SEL_GPIO,       /* HSIOM selection */
		.intEdge    = CY_GPIO_INTR_FALLING, /* Interrupt Edge type */
		.intMask    = CY_GPIO_INTR_EN_MASK, /* Interrupt enable mask */
		.vtrip      = CY_GPIO_VTRIP_CMOS,   /* Input buffer voltage trip type */
		.slewRate   = CY_GPIO_SLEW_FAST,    /* Output buffer slew rate */
		.driveSel   = CY_GPIO_DRIVE_FULL,   /* Drive strength */
		.vregEn     = 0u,                   /* SIO pair output buffer mode */
		.ibufMode   = 0u,                   /* SIO pair input buffer mode */
		.vtripSel   = 0u,                   /* SIO pair input buffer trip point */
		.vrefSel    = 0u,                   /* SIO pair reference voltage for input buffer trip point */
		.vohSel     = 0u                    /* SIO pair regulated voltage output level */
	};
#endif
/* This structure is used to initialize a full GPIO Port using PDL configuration */
const cy_stc_gpio_prt_config_t port7_Init =
{
	.out        = 0x00000000u,  /* Initial output data for the IO pins in the port */
	.intrMask   = 0x00000000u,  /* Interrupt enable mask for the port interrupt */
	.intrCfg    = 0x00000000u,  /* Port interrupt edge detection configuration */
	.cfg        = 0x0EEEEEEEu,  /* Port drive modes and input buffer enable configuration */
	.cfgIn      = 0x00000000u,  /* Port input buffer configuration */
	.cfgOut     = 0x00000000u,  /* Port output buffer configuration */
	.cfgSIO     = 0x00000000u,  /* Port SIO configuration */
	.sel0Active = 0x00000000u,  /* HSIOM selection for port pins 0,1,2,3 */
	.sel1Active = 0x00000000u,  /* HSIOM selection for port pins 4,5,6,7 */
};
/* This structure initializes the Port0 interrupt for the NVIC */
cy_stc_sysint_t intrCfg =
{
	.intrSrc = ioss_interrupts_gpio_0_IRQn, /* Interrupt source is GPIO port 0 interrupt */
	.intrPriority = 2UL                     /* Interrupt priority is 2 */
};
uint32    pinState = 0ul;

void GPIO_Interrupt();


/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  The main function performs the following actions:
*   1. Initializes GPIO pins and interrupts
*   2. Reads value from button pin
*   3. Writes value from button pin to red LED pin. If button is pressed then red LED is on.
*   4. On button release, triggers interrupt to pulse blue LED for approximately 1 second.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
int main(void)
{
	volatile uint32 pinReadValue = 0ul; /* Volatile to prevent compiler optimizing out examples */
	uint32 pinStateCounter = 0ul;
	#if PDL_PIN_CONFIGURATION
		uint32 portReadValue = 0ul;
		uint32 portNumber = 0ul;
	#endif

	/* Set up the device based on configurator selections */
    init_cycfg_all();

    __enable_irq();

    /* ModusToolbox Device Configurator automatically generates GPIO configuration code and */
    /* executes it as part of the device boot process in cyfitter_cfg.c. The following GPIO */
    /* configuration methods are typically only used with manual PDL GPIO configuration. */
    /* They may also be used at run time to dynamically reconfigure GPIO pins independent */
    /* of how the initial configuration was performed. */
	#if PDL_PIN_CONFIGURATION
		/* Most IO pins only require their basic parameters to be set and can use default */
		/* values for all other settings. This allows use of a simplified Initialization function. */
		/* Cy_GPIO_Pin_FastInit() only supports parameterized configuration of drive mode, */
		/* output logic level, and HSIOM multiplexer setting. All other configuration settings */
		/* are untouched. Very useful at run time to dynamically change a pin's configuration. */
		/* For example, configure a pin to strong drive mode to write data, and then reconfigure */
		/* as high impedance to read data. */
		Cy_GPIO_Pin_FastInit(P1_5_PORT, P1_5_NUM, CY_GPIO_DM_STRONG, 1, HSIOM_SEL_GPIO);
		Cy_GPIO_Pin_FastInit(P13_7_PORT, P13_7_NUM, CY_GPIO_DM_STRONG, 1, HSIOM_SEL_GPIO);

		/* The method to configure all attributes of a single pin is to use the */
		/* Cy_GPIO_Pin_Init() function and configuration structure. While easy to use, it generates */
		/* larger code than other methods. This is the default method used by ModusToolbox Device */
		/* and can be seen in cycfg_pins.c */
		Cy_GPIO_Pin_Init(P0_4_PORT, P0_4_NUM, &P0_4_Pin_Init);
	#endif

	/* Individual pin configuration settings can also be changed at run time using */
	/* supplied driver API functions. An example of some of these functions are provided below. */
	/* The function parameters demonstrate use of pin specific #defines provided in */
	/* cycfg_pins.h when ModusToolbox Device Configurator is used. */
	Cy_GPIO_SetHSIOM(KIT_BTN1_PORT, KIT_BTN1_NUM, HSIOM_SEL_GPIO);
	Cy_GPIO_SetDrivemode(KIT_BTN1_PORT, KIT_BTN1_NUM, CY_GPIO_DM_PULLUP);
	Cy_GPIO_SetVtrip(KIT_BTN1_PORT, KIT_BTN1_NUM, CY_GPIO_VTRIP_CMOS);
	Cy_GPIO_SetSlewRate(KIT_BTN1_PORT, KIT_BTN1_NUM, CY_GPIO_SLEW_FAST);
	Cy_GPIO_SetDriveSel(KIT_BTN1_PORT, KIT_BTN1_NUM, CY_GPIO_DRIVE_FULL);

	/* The most code efficient method to configure all attributes for a full port of pins */
	/* is to use the Cy_GPIO_Port_Init() API function and configuration structure. It packs all */
	/* the configuration data into direct register writes for the whole port. Its limitation */
	/* is that it must configure all pins in a port and the user must calculate the */
	/* combined register values for all pins. */
	Cy_GPIO_Port_Init(GPIO_PRT7, &port7_Init);

    /* Pin Interrupts */
    /* Configure GPIO pin to generate interrupts */
    Cy_GPIO_SetInterruptEdge(KIT_BTN1_PORT, KIT_BTN1_NUM, CY_GPIO_INTR_RISING);
    Cy_GPIO_SetInterruptMask(KIT_BTN1_PORT, KIT_BTN1_NUM, CY_GPIO_INTR_EN_MASK);
    /* Configure CM4+ CPU GPIO interrupt vector for Port 0 */
    Cy_SysInt_Init(&intrCfg, GPIO_Interrupt);
    NVIC_ClearPendingIRQ(intrCfg.intrSrc);
    NVIC_EnableIRQ((IRQn_Type)intrCfg.intrSrc);
 
    for(;;)
    {
        /* Pin input read methods */
    	/*******************************************************************************/
        /* The following code examples all perform the same read from a GPIO using the */
        /* different read methods available. Please choose the most appropriate method */
        /* for your specific use case. All Read() functions are thread and multi-core */
        /* safe */

        /* Pin read using #defines provided by Device Configurator tool pin names. This is the */
        /* preferred method for use with the Device Configurator tool. #defines are located in */
        /* \\<ProjectName>_config\GeneratedSource\cycfg_pins.h */
        pinReadValue = Cy_GPIO_Read(KIT_BTN1_PORT, KIT_BTN1_NUM);

		#if PDL_PIN_CONFIGURATION
			/* Pin read with user defined custom #define pin name. This is the preferred */
			/* method for direct PDL use without using the Device Configurator tool. #defines are typically */
			/* placed in a .h file but included here for example simplicity and clarity */
			#define mySwPin_Port P0_4_PORT
			#define mySwPin_Num  P0_4_NUM
			pinReadValue = Cy_GPIO_Read(mySwPin_Port, mySwPin_Num);

			/* Pin read using default device pin name #defines located in */
			/* \\<ProjectName>_mainapp_(device series)pdl\Includes\(..devices/series/include)\gpio_(series+package).h */
			pinReadValue = Cy_GPIO_Read(P0_4_PORT, P0_4_NUM);

			/* Pin read using default port register name #defines and pin number #defines */
			/* located in \\<ProjectName>_mainapp_(device series)pdl\Includes\(..devices/series/include)\(part number).h */
			pinReadValue = Cy_GPIO_Read(GPIO_PRT0, 4);

			/* Pin read using port and pin numbers. Useful for algorithmically generated */
			/* port and pin numbers. Cy_GPIO_PortToAddr() is a helper function that converts */
			/* the port number into the required port register base address. */
			portNumber = 0;
			pinReadValue = Cy_GPIO_Read(Cy_GPIO_PortToAddr(portNumber), 4);

			/* Direct port IN register read with mask and shift of desired pin data */
			pinReadValue = (GPIO_PRT0->IN >> P0_4_NUM) & CY_GPIO_IN_MASK;
		#endif


        /* Pin output write methods */
        /*******************************************************************************/
        /* The following code examples all perform the same write to GPIO using the */
        /* different write methods available. Please choose the most appropriate */
        /* method for your specific use case. Cy_GPIO_Write() API is best used when */
        /* the desired pin state is not already known and is determined at run time. The */
        /* Write API uses atomic operations that directly affect only the selected pin */
        /* without using read-modify-write operations. The Write API is therefore */
        /* thread and multi-core safe */

        /* Pin write using #defines provided by Device Configurator pin name. This is */
        /* the preferred method for use with configuration tools. #defines are located in */
        /* \\<ProjectName>_config\GeneratedSource\cycfg_pins.h */
        Cy_GPIO_Write(KIT_LED1_PORT, KIT_LED1_NUM, pinReadValue);

		#if PDL_PIN_CONFIGURATION
			/* Pin write with user defined custom #define pin name. This is the preferred */
			/* method for direct PDL use without using the Device Configurator tool. #defines are typically */
			/* placed in .h file but included here for example simplicity and clarity */
			#define myLedPin_Port P1_5_PORT
			#define myLedPin_Num  P1_5_NUM
			Cy_GPIO_Write(myLedPin_Port, myLedPin_Num, pinReadValue);

			/* Pin write using default device pin name #defines located in */
			/* \\<ProjectName>_mainapp_(device series)pdl\Includes\(..devices/series/include)\gpio_(series+package).h */
			Cy_GPIO_Write(P1_5_PORT, P1_5_NUM, pinReadValue);

			/* Pin write using default port register name #defines and pin numbers. */
			/* #defines located in located in \\<ProjectName>_mainapp_(device series)pdl\Includes\(..devices/series/include)\(part number).h */
			Cy_GPIO_Write(GPIO_PRT1, 5, pinReadValue);

			/* Pin write using port and pin numbers. Useful for algorithmically generated */
			/* port and pin numbers. Cy_GPIO_PortToAddr() is a helper function that */
			/* converts the port number into the required port register base address */
			portNumber = 1;
			Cy_GPIO_Write(Cy_GPIO_PortToAddr(portNumber), 5, pinReadValue);
		#endif


        /* Pin output methods to directly Set, Clear, and Invert pin output state */
        /*******************************************************************************/
        /* These register writes are atomic operations that directly affect the */
        /* selected pin without using read-modify-write operations. They are therefore */
        /* thread and multi-core safe. These are the most efficient output methods */
        /* when the desired pin state is already known at compile time. The same */
        /* argument variations as demonstrated with the Cy_GPIO_Write() API can be used. */
        if(1u == pinReadValue)
        {
            Cy_GPIO_Set(KIT_LED1_PORT, KIT_LED1_NUM);
        }
        else
        {
            Cy_GPIO_Clr(KIT_LED1_PORT, KIT_LED1_NUM);
        }
		#if PDL_PIN_CONFIGURATION
        	Cy_GPIO_Inv(KIT_LED1_PORT, KIT_LED1_NUM);
        	GPIO_PRT1->OUT_INV = CY_GPIO_OUT_MASK << P1_5_NUM; /* Equivalent to Cy_GPIO_Inv() function */
		#endif


        /* Simultaneous Port Pin access */
        /*******************************************************************************/
        /* Direct register access is used to interface with multiple pins in one port */
        /* at the same time. May not be thread or multi-core safe due to possible */
        /* read-modify-write operations. All pins in a Port under direct register */
        /* control should only be accessed by a single CPU core. */
		#if PDL_PIN_CONFIGURATION
			portReadValue = GPIO_PRT7->IN;
			portReadValue++;
			GPIO_PRT7->OUT = portReadValue;
		#endif


        /* Service ISR flag */
        /*******************************************************************************/
        if(GPIO_ISR_FLAG == pinState)
        {
            pinStateCounter++;
            if(PIN_DELAY > pinStateCounter)
            {
                Cy_GPIO_Clr(KIT_LED2_PORT, KIT_LED2_NUM);
            }
            else
            {
                Cy_GPIO_Set(KIT_LED2_PORT, KIT_LED2_NUM);
                pinState = 0;
                pinStateCounter = 0;
            }
        }
    }
}


/*******************************************************************************
* Function Name: GPIO_Interrupt
********************************************************************************
*
*  Summary:
*  Interrupt service routine for the port interrupt triggered from KIT_BTN1.
*
*  Parameters:
*  None
*
*  Return:
*  None
*
**********************************************************************************/
void GPIO_Interrupt()
{
    uint32 portIntrStatus;

    /* Optional check to determine if correct Port generated interrupt. */
    /* Only used if a single interrupt vector is used for multiple Ports due to vector */
    /* limitations. Preferred method is to use unique interrupt vector for each Port. */
    if((Cy_GPIO_GetInterruptCause0() & INTCAUSE0_PORT0) == INTCAUSE0_PORT0)
    {
        /* Optional check to determine which pin in the port generated interrupt. */
        if(Cy_GPIO_GetInterruptStatus(KIT_BTN1_PORT, KIT_BTN1_NUM) == CY_GPIO_INTR_STATUS_MASK)
        {
            pinState = GPIO_ISR_FLAG;
        }
        /* Alternate optional check to determine which pin in the port generated interrupt. */
        portIntrStatus = KIT_BTN1_PORT->INTR;
        if(CY_GPIO_INTR_STATUS_MASK == ((portIntrStatus >> KIT_BTN1_NUM) & CY_GPIO_INTR_STATUS_MASK))
        {
            pinState = GPIO_ISR_FLAG;
        }
    }

    /* Clear pin interrupt logic. Required to detect next interrupt */
    Cy_GPIO_ClearInterrupt(KIT_BTN1_PORT, KIT_BTN1_NUM);
}

