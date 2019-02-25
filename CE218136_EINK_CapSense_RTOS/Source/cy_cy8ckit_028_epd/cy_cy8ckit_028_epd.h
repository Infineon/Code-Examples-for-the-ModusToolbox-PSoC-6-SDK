/******************************************************************************
* File Name: cy_cy8ckit_028_epd.h
*
* Version: 1.00
*
* Description: This file contains function declarations and macro definitions 
*              provided by the cy_cy8ckit_028_epd.c file.
*
* Hardware Dependency: CY8CKIT-028-EPD E-INK Display Shield
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
* This file contains function declarations and macro definitions provided by 
* the cy_cy8ckit_028_epd.c file.
*
* For the details of the E-INK display and library functions, see the code  
* example document of CE218136 - PSoC 6 MCU E-INK Display with CapSense (RTOS)
*
* For the details of E-INK display hardware and driver interface, see the 
* documents available at the following website:
* http://www.pervasivedisplays.com/products/271
*******************************************************************************/

/* Include Guard */
#ifndef CY_CY8CKIT_028_EPD_H
#define CY_CY8CKIT_028_EPD_H

/* Header file includes */
#include "pervasive_eink_hardware_driver.h"
#include <stdio.h>

/* Macros used for E-INK power control */
#define CY_EINK_OFF                (false)
#define CY_EINK_ON                 (true)
#define CY_EINK_OPERATION_SUCCESS  (true)
#define CY_EINK_OPERATION_FAILURE  (false)
#define CY_EINK_POWER_AUTO         (true)
#define CY_EINK_POWER_MANUAL       (false)

/* Size of an E-INK frame = size of an E-INK image = (264*176)/8 = 5808 bytes */
#define CY_EINK_FRAME_SIZE         PV_EINK_IMAGE_SIZE
#define CY_EINK_IMAGE_SIZE         PV_EINK_IMAGE_SIZE
    
/* Data types of E-INK frame / image */
typedef pv_eink_frame_data_t       cy_eink_frame_t;
typedef pv_eink_frame_data_t       cy_eink_image_t;

/* Data type of E-INK update types */
typedef enum
{ 
    CY_EINK_PARTIAL,
    CY_EINK_FULL_4STAGE,
    CY_EINK_FULL_2STAGE 
}   cy_eink_update_t;

/* Data type for E-INK API results */
typedef enum
{   CY_EINK_SUCCESS,
    CY_EINK_FAILURE
}   cy_eink_api_result;

/* Declarations of functions provided by cy_cy8ckit_028_epd.c. For the details of 
   E-INK library functions, see Appendix A of the CE218133 - PSoC 6 MCU E-INK 
   Display with CapSense code example document */

/* Power and initialization functions */
cy_eink_api_result  Cy_EINK_Start(int8_t temperature,
                                  cy_eink_delay_function_t delayFunction);
cy_eink_api_result  Cy_EINK_Power(bool powerCtrl);

/* Display update function */
void Cy_EINK_ShowFrame(cy_eink_frame_t* prevFrame, cy_eink_frame_t* newFrame,
                       cy_eink_update_t updateType, bool powerCycle);

#endif /* CY_CY8CKIT_028_EPD_H */
/* [] END OF FILE */
