/******************************************************************************
* File Name: screen_contents.c
*
* Version: 1.10
*
* Description: This is a storage file for the screen contents
*
* Related Document: CE218136_EINK_CapSense_RTOS.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*                      CY8CKIT-028-EPD E-INK Display Shield
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
/******************************************************************************
* This is a storage file for the screen contents (text and images)
* This file contains no executable code
*******************************************************************************/

/* Header file includes */
#include "./images_and_text/screen_contents.h"

/* Maximum page indexes of each text page array */
#define MCU_FEATURES_PAGES_MAX_INDEX    (NUMBER_OF_MCU_FEATURES_PAGES - 1)
#define KIT_FEATURES_PAGES_MAX_INDEX    (NUMBER_OF_KIT_FEATURES_PAGES - 1)
#define KIT_DOCUMENT_PAGES_MAX_INDEX    (NUMBER_OF_KIT_DOCUMENT_PAGES - 1)
#define KIT_EXAMPLES_PAGES_MAX_INDEX    (NUMBER_OF_KIT_EXAMPLES_PAGES - 1)

/* Constant variable that stores the maximum index of each text page array */
uint8_t const  maxTextPageIndexes[NUMBER_OF_MAIN_MENU_ITEMS] =
{
    MCU_FEATURES_PAGES_MAX_INDEX,
    KIT_FEATURES_PAGES_MAX_INDEX,
    KIT_DOCUMENT_PAGES_MAX_INDEX,
    KIT_EXAMPLES_PAGES_MAX_INDEX
};

/* Array that stores the text pages */   
char const textPage [TOTAL_TEXT_PAGES][TEXT_PAGE_CHARACTER_SIZE] =
{   
    /* Pages of "PSoC 6 MCU FEATURES" menu item */
    /* Page 1 */
    { 
    "PSoC 6 MCU FEATURES (Page 1 of 8)\n\n"\
    "CPU Subsystem:\n\n"\
    "150-MHz Arm Cortex-M4 CPU with single-cycle multiply (Floating Point and"\
	"Memory Protection Unit), 100-MHz Cortex M0+ CPU.\n\n"\
	"Two DMA controllers with sixteen channels each"
    },
    
    /* Page 2 */
    { 
    "PSoC 6 MCU FEATURES (Page 2 of 8)\n\n"\
    "Memory :\n\n"\
    "1-MB Application Flash with 32-KB EEPROM area. \n\n"\
    "288-KB integrated SRAM with 32-KB retention boundaries (can retain "\
    "32K to 288K in 32K increments.)"
    },
    
    /* Page 3 */
    { 
    "PSoC 6 MCU FEATURES (Page 3 of 8)\n\n"\
    "BLE:\n\n"\
    "Bluetooth Low Energy (Bluetooth Smart) BT 4.2 Subsystem that supports "\
	"2 Mbps LE data rate.\n\n"\
	" 2.4-GHz RF transceiver with 50-ohm antenna drive.\n\n"\
    "Link Layer engine supports four connections simultaneously."\
    },
    
    /* Page 4 */
    { 
    "PSoC 6 MCU FEATURES (Page 4 of 8)\n\n"\
    "Digital Peripherals:\n\n"\
    "Serial Communication Blocks (SCBs), that are configurable as I2C, SPI, "\
	"or UART.\n\n"\
    "Timer, Counter, Pulse-Width Modulator (TCPWM) blocks\n\n"\
    "Audio Subsystem with two PDM channels and I2S Interface."\
    },
    
    /* Page 5 */
    { 
    "PSoC 6 MCU FEATURES (Page 5 of 8)\n\n"\
    "Programmable Digital:\n\n"\
    "12 programmable logic blocks, each with 8 Macrocells and an 8-bit data "\
	"path (called universal digital blocks or UDBs).\n\n"\
    "Usable as drag-and-drop Boolean primitives (gates, registers), or as "\
	"Verilog programmable blocks."
    },
    
    /* Page 6*/
    { 
    "PSoC 6 MCU FEATURES (Page 6 of 8)\n\n"\
    "Programmable Analog:\n\n"\
    "12-bit, 1 Msps SAR ADC with 16-channel Sequencer.\n\n"\
    "12-bit voltage mode DAC.\n\n"\
    "Two opamps with low-power operation modes\n\n"\
    "Two low-power comparators."
    },
    
    /* Page 7*/
    { 
    "PSoC 6 MCU FEATURES (Page 7 of 8)\n\n"\
    "Capacitive Sensing:\n\n"\
    "Cypress Capacitive Sigma-Delta (CSD) provides best-in-class SNR,"\
    "liquid tolerance, and proximity sensing.\n\n"\
    "Mutual Capacitance sensing (CSX) with dynamic usage of both Self "\
    "and Mutual sensing."
    },
    
    /* Page 8*/
    { 
    "PSoC 6 MCU FEATURES (Page 8 of 8)\n\n"\
    "More information:\n\n"\
    "For a detailed list of PSoC 6 MCU features, visit the webpage:\n\n"\
    "www.cypress.com/PSoC6."
    },
    
    /* Pages of "KIT FEATURES" menu item */
    /* Page 1 */
    { 
    "KIT FEATURES (Page 1 of 7)\n\n"\
    "PSoC 6 MCU:\n\n"\
    "PSoC 6 MCU is a true programmable embedded system-on-chip, integrating "\
    "a 150-MHz Arm Cortex-M4, a 100-MHz Arm Cortex-M0+, up to 1MB Flash and "\
	"288KB SRAM, BLE 4.2 radio, CapSense touch-sensing, and programmable "\
	"peripherals that allow high flexibility."
    },
    
    /* Page 2 */
    { 
    "KIT FEATURES (Page 2 of 7)\n\n"\
    "KitProg:\n\n"\
    "KitProg on the Pioneer Board is a pre-programmed PSoC 5LP (CY8C5868LTI-LP039)"\
	"functioning as an on-board programmer/debugger with mass storage programming,"\
	"USB to UART/I2C/SPI bridge functionality and custom applications support."
    },
    
    /* Page 3 */
    { 
    "KIT FEATURES (Page 3 of 7)\n\n"\
    "Power supply:\n\n"\
    "Power supply on the Pioneer  Board is designed to support 1.8V to 3.3V "\
	"operation of the PSoC 6 MCU. A 330 mF super capacitor is provided for "\
	"backup domain supply (Vbackup). In addition, the Board has an EZ-PD CCG3 "\
	"USB Type-C power delivery system."
    },
    
    /* Page 4 */
    { 
    "KIT FEATURES (Page 4 of 7)\n\n"\
    "Serial NOR Flash:\n\n"\
    "The Pioneer Board has a 512-Mbit external Quad-SPI NOR Flash that "\
	"provides a fast, expandable memory for data as well as code."
    },
    
    /* Page 5 */
    { 
    "KIT FEATURES (Page 5 of 7)\n\n"\
    "BLE-USB Dongle:\n\n"\
    "The kit includes a CY5677 CySmart BLE 4.2 USB Dongle that is "\
	"factory-programmed to emulate a BLE GAP Central device, enabling "\
    "you to emulate a BLE host on your computer."
    },
    
    /* Page 6*/
    { 
    "KIT FEATURES (Page 6 of 7)\n\n"\
    "E-INK Display Shield:\n\n"\
    "This kit includes a CY8CKIT-028- EPD E-INK Display Shield, that "\
	"contains a 2.7-inch E-INK display, a thermistor, a PDM microphone "\
	"and a motion sensor."
    },
    
    /* Page 7*/
    { 
    "KIT FEATURES (Page 7 of 7)\n\n"\
    "More information:\n\n"\
    "For a detailed list of kit features, visit the webpage:\n\n"\
    "www.cypress.com/CY8CKIT-062-BLE "
    },

    /* Pages of "KIT DOCUMENTATION" menu item */
    /* Page 1 */
    { 
    "KIT DOCUMENTATION (Page 1 of 4)\n\n"\
    "Quick Start Guide (QSG):\n\n"\
    "The QSG provides details of the kit contents, instructions to use "\
    "the out-of-the box (OOB) project, kit features and pin-out of the "\
    "Pioneer Board."
    },
    
    /* Page 2 */
    { 
    "KIT DOCUMENTATION (Page 2 of 4)\n\n"\
    "Kit Guide:\n\n"\
    "Kit guide contains a detailed   explanation of kit hardware, "\
    "operation,  and  provides step by step instructions to install the "\
    "kit software."
    },

    /* Page 3 */
    { 
    "KIT DOCUMENTATION (Page 3 of 4)\n\n"\
    "Release Notes:\n\n"\
    "Release notes lists kit contents, installation requirements, kit"\
    "documentation, limitations and known issues."
    }, 
    
    /* Page 4 */
    { 
    "KIT DOCUMENTATION (Page 4 of 4)\n\n"\
    "Location:\n\n"\
    "The kit documentation explained in pages 1, 2 and 3 can be found "\
    "at the CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit webpage: \n\n"\
    "www.cypress.com/CY8CKIT-062-BLE"
    }, 
    
    /* Pages of "KIT CODE EXAMPLES" menu item */
    /* Page 1 */
    { 
    "KIT CODE EXAMPLE (Page 1 of 10)\n\n"\
    "CE218136_EINK_CapSense_RTOS:\n\n"\
    "This code example shows how to create a user-interface solution "\
    "using an E-INK display and CapSense."
    },
    
    /* Page 2 */
    { 
    "KIT CODE EXAMPLES (Page 2 of 10)\n\n"\
    "CE218137_BLE_Proximity_RTOS:\n\n"\
    "This code example demonstrates connectivity between the PSoC 6 "\
    "MCU with BLE and CySmart BLE host emulation tool or mobile device "\
    "running the CySmart mobile application, to transfer CapSense"\
    "proximity sensing information."
    }, 

    /* Page 3 */
    { 
    "KIT CODE EXAMPLES (Page 3 of 10)\n\n"\
    "CE219517_KitProg2_Power_Monitoring:\n\n"\
    "This code example demonstrates how to create a bootloadable PSoC 5LP "\
	"project that measures the power consumed by the PSoC 6 MCU on "\
	"CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit."
    },
    
    /* Page 4 */
    { 
    "KIT CODE EXAMPLES (Page 4 of 10)\n\n"\
    "CE220331_BLE_UI_RTOS:\n\n"\
    "This code example demonstrates interfacing PSoC 6 MCU with user "\
    "interface functions such as an E-INK display, RGB LED, and touch"\
    "sensors based on self and mutual capacitance (CSD and CSX) with "\
    "bi-directional BLE connectivity."
    }, 
    
        
    /* Page 5 */
    { 
    "KIT CODE EXAMPLES (Page 5 of 10)\n\n"\
    "CE222604_RTC_CTS_RTOS:\n\n"\
    "This code example demonstrates accurate time keeping with PSoC 6 "\
    "MCU's real time clock(RTC), which is synchronized with a current "\
    "time server such as an iPhone using the BLE Current Time Service (CTS)."
    }, 
          
    /* Page 6 */
    { 
    "KIT CODE EXAMPLES (Page 6 of 10)\n\n"\
    "CE220272_BLE_Direct_Test_Mode:\n\n"\
    "This code example demonstrates Direct Test Mode (DTM) over the "\
    "Host Controller Interface (HCI) using PSoC 6 MCU with BLE Connectivity."
    }, 
          
    /* Page 7 */
    { 
    "KIT CODE EXAMPLES (Page 7 of 10)\n\n"\
    "CE218139_BLE_Eddystone_RTOS:\n\n"\
    "This code example demonstrates a BLE beacon that broadcasts the "\
    "core frame types (UID, URL, and TLM) of Google's Eddystone beacon"\
    "profile."
    },

    /* Page 8 */
    { 
    "KIT CODE EXAMPLES (Page 8 of 10)\n\n"\
    "CE218138_BLE_Thermometer_RTOS:\n\n"\
    "This code example demonstrates interfacing PSoC 6 MCU with a thermistor"\
	"circuit to read temperature information and  sending the data over BLE "\
	"Health Thermometer Service (HTS) to a  mobile device running CySmart "\
    "mobile application."
    },
    
    /* Page 9 */
    { 
    "KIT CODE EXAMPLES (Page 9 of 10)\n\n"\
    "CE222793_MotionSensor_RTOS:\n\n"\
    "This code example demonstrates how to interface a PSoC 6 MCU "\
    "with a BMI160 motion sensor. This example reads steps counted by "\
    "the sensor to emulate a pedometer. Raw motion data is also read "\
    "and used to estimate the orientation of the board."
    },
    
    /* Page 10 */
    { 
    "KIT CODE EXAMPLES (Page 10 of 10)\n\n"\
    "CE222046_BLE_Throughput_Measurement:\n\n"\
    "This code example demonstrates  how to maximize the BLE  throughput "\
	"on PSoC 6 MCU with Bluetooth Low Energy (BLE) Connectivity device."
    }
}; 

/* Two dimensional array that stores the indexes of text page. This type of 
   indexing saves flash by avoiding the need for a ragged text page array */
uint8_t const textPageIndex [NUMBER_OF_MAIN_MENU_ITEMS][MAX_NUMBER_OF_TEXT_PAGES]=
{
    /* Indexes of "PSoC 6 MCU FEATURES" pages */
    {0x00u, 0x01u, 0x02u, 0x03u, 0x04u, 0x05u, 0x06u, 0x07u, INVALID_PAGE_INDEX,INVALID_PAGE_INDEX},
    
    
    /* Indexes of "KIT FEATURES" pages */
    {0x08u, 0x09u, 0x0Au, 0x0Bu, 0x0Cu, 0x0Du, 0x0Eu, INVALID_PAGE_INDEX, INVALID_PAGE_INDEX, INVALID_PAGE_INDEX},
    
    /* Indexes of "KIT DOCUMENTATION" pages */
    {0x0Fu, 0x10u, 0x11u, 0x12u, INVALID_PAGE_INDEX, INVALID_PAGE_INDEX, INVALID_PAGE_INDEX, INVALID_PAGE_INDEX, INVALID_PAGE_INDEX, INVALID_PAGE_INDEX},
    
    /* Indexes of "KIT CODE EXAMPLES" pages */
    {0x13u, 0x14u, 0x015u, 0x16u, 0x17u, 0x18u, 0x19u, 0x1Au, 0x1Bu, 0x1Cu}
};


/* [] END OF FILE */
