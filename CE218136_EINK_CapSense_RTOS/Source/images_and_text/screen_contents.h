/******************************************************************************
* File Name: screen_contents.h
*
* Version: 1.10
*
* Description: This file contains the macros and variable declarations that can
*              be used to access the text stored in screen_contents.c
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
* This file contains the macros and variable declarations that can be used to 
* access the text stored in screen_contents.c
*******************************************************************************/

/* Include Guard */
#ifndef SCREEN_CONTENTS_H
#define SCREEN_CONTENTS_H
    
/* Header file includes */
#include "./cy_cy8ckit_028_epd/cy_cy8ckit_028_epd.h"

/* Number of items in the main menu */
#define NUMBER_OF_MAIN_MENU_ITEMS       (uint8_t) (4u)

/* Number of text pages of each text page group, corresponding to a main menu
   item */
#define NUMBER_OF_MCU_FEATURES_PAGES    (uint8_t) (8u)
#define NUMBER_OF_KIT_FEATURES_PAGES    (uint8_t) (7u)
#define NUMBER_OF_KIT_DOCUMENT_PAGES    (uint8_t) (4u)
#define NUMBER_OF_KIT_EXAMPLES_PAGES    (uint8_t) (10u)

/* Total number of text pages, which is a sum of the text pages in each text 
   page group */
#define TOTAL_TEXT_PAGES                NUMBER_OF_MCU_FEATURES_PAGES + \
                                        NUMBER_OF_KIT_FEATURES_PAGES + \
                                        NUMBER_OF_KIT_DOCUMENT_PAGES + \
                                        NUMBER_OF_KIT_EXAMPLES_PAGES

/* Maximum number of text pages of in a page group */
#define MAX_NUMBER_OF_TEXT_PAGES        (uint8_t) (10u)

/* Size of a text page in number of ASCII characters */
#define TEXT_PAGE_CHARACTER_SIZE        (uint16)(397u)

/* Value returned from the page index array if an invalid page is accessed */
#define INVALID_PAGE_INDEX              (uint8_t) (0xFFu)
    
/* Variable from screen_contents.c that stores the maximum index of each text
   page array */
extern uint8_t const maxTextPageIndexes[NUMBER_OF_MAIN_MENU_ITEMS];

/* Variables from screen_contents.c that store the text page and the associated
   indexes in flash */
extern char    const textPage[TOTAL_TEXT_PAGES]
                             [TEXT_PAGE_CHARACTER_SIZE];
extern uint8_t const textPageIndex[NUMBER_OF_MAIN_MENU_ITEMS]
                                  [MAX_NUMBER_OF_TEXT_PAGES];



#endif
/* [] END OF FILE */
