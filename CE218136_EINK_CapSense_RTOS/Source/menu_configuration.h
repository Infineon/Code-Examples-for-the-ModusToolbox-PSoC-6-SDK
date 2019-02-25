/******************************************************************************
* File Name: menu_configuration.h
*
* Version: 1.00
*
* Description: This file contains the macros and datatypes that define the menu 
*              structure displayed
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
* This file contains the macros and datatypes that define the menu structure
* and other display related information
*******************************************************************************/

/* Include guard */
#ifndef MENU_CONFIGURATION_H
#define MENU_CONFIGURATION_H

/* Number of startup screens before the main menu image is displayed */
#define NUMBER_OF_STARTUP_SCREENS   (uint8_t) (1u)

/* Total number of touch input types */
#define NUMBER_OF_INPUT_TYPES       (uint8_t)(0x04u)

/* Number of screen types consisting of the main menu and text pages */
#define NUMBER_OF_SCREEN_TYPES      (uint8_t) (0x02u)

/* Start and maximum indexes of the main menu items */
#define MAIN_MENU_INDEX_START       (uint8_t) (0x00u)
#define MAIN_MENU_MAX_INDEX         (NUMBER_OF_MAIN_MENU_ITEMS - 1)

/* Start page number of the text pages */
#define TEXT_PAGE_INDEX_START       (uint8_t) (0x00u)

/* Time in mS during which the logo will be displayed during startup */
#define STARTUP_SCREEN_DELAY        (pdMS_TO_TICKS(1000u))

/* Enumerated data type used to identify the content of the screen */
typedef enum
{
    MAIN_MENU_CONTENT,
    TEXT_PAGE_CONTENT
}   screen_content_t;

/* Enumerated data-type for the screen type, consists of main menu and the
   text page */
typedef enum
{
    MAIN_MENU   = 0x00u,
    TEXT_PAGE   = 0x01u
}   screen_type_t;

/* Datatype for the screen information */
typedef struct
{
    screen_type_t   screen;
    uint8_t         menuItem;
    uint8_t         textPage;
}   screen_t;

#endif /* MENU_CONFIGURATION_H */
/* [] END OF FILE */
