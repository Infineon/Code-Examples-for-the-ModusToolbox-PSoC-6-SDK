/******************************************************************************
* File Name: cy_eink_psoc_interface.h
*
* Version: 1.00
*
* Description: This file contains function declarations and macro definitions
*              provided by the cy_eink_psocinterface.c file.
*
* Hardware Dependency: CY8CKIT-028-EPD E-INK Display Shield
*
*******************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation. All rights reserved.
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
* This file contains function declarations and macro definitions provided by the
* cy_eink_psoc_encapsulation.c file.
*
* For the details of the E-INK display and library functions, see the code  
* example document of CE218136 - PSoC 6 MCU E-INK Display with CapSense (RTOS)
*
* For the details of E-INK display control and communication protocols, see the
* driver document available at the following website:
* http://www.pervasivedisplays.com/products/271
*******************************************************************************/

/* Include Guard */
#ifndef CY_EINK_PSOC_INTERFACE_H
#define CY_EINK_PSOC_INTERFACE_H

/* Header file includes */
#include "cy_sysint.h"
#include "cy_syslib.h"
#include <stdbool.h>
#include "cycfg.h"

/* Macros used for byte level operations */
#define CY_EINK_BYTE_SIZE      (uint8_t)(0x08u)
#define CY_EINK_SINGLE_BYTE    (uint8_t)(0x01u)

/* Push the chip select pin to logic HIGH */
#define CY_EINK_CsHigh          Cy_GPIO_Set(CY_EINK_Ssel_PORT,\
                                            CY_EINK_Ssel_PIN)

/* Pull the chip select pin to logic LOW */
#define CY_EINK_CsLow           Cy_GPIO_Clr(CY_EINK_Ssel_PORT,\
                                            CY_EINK_Ssel_PIN)

/* Push the reset pin to logic HIGH */
#define CY_EINK_RstHigh         Cy_GPIO_Set(CY_EINK_DispRst_PORT,\
                                            CY_EINK_DispRst_PIN)

/* Pull the reset pin to logic LOW */
#define CY_EINK_RstLow          Cy_GPIO_Clr(CY_EINK_DispRst_PORT,\
                                            CY_EINK_DispRst_PIN)

/* Push the discharge pin to logic HIGH */
#define CY_EINK_DischargeHigh   Cy_GPIO_Set(CY_EINK_Discharge_PORT,\
                                            CY_EINK_Discharge_PIN)

/* Pull the discharge pin to logic LOW */
#define CY_EINK_DischargeLow    Cy_GPIO_Clr(CY_EINK_Discharge_PORT,\
                                            CY_EINK_Discharge_PIN)

/* Turn on the display by pushing the display enable pin to logic HIGH */
#define CY_EINK_TurnOnVcc       Cy_GPIO_Set(CY_EINK_DispEn_PORT,\
                                            CY_EINK_DispEn_PIN)

/* Turn off the display by pulling the display enable pin to logic LOW */
#define CY_EINK_TurnOffVcc      Cy_GPIO_Clr(CY_EINK_DispEn_PORT,\
                                            CY_EINK_DispEn_PIN)

/* Push the border pin to logic HIGH */
#define CY_EINK_BorderHigh      Cy_GPIO_Set(CY_EINK_Border_PORT,\
                                            CY_EINK_Border_PIN)

/* Pull the border  pin to logic LOW */
#define CY_EINK_BorderLow       Cy_GPIO_Clr(CY_EINK_Border_PORT,\
                                            CY_EINK_Border_PIN)

/* Pull the Enable I/O pin to logic LOW */
#define CY_EINK_EnableIO        Cy_GPIO_Clr(CY_EINK_DispIoEn_PORT,\
                                            CY_EINK_DispIoEn_PIN)

/* Push the Enable I/O pin to logic HIGH */
#define CY_EINK_DisableIO       Cy_GPIO_Set(CY_EINK_DispIoEn_PORT,\
                                            CY_EINK_DispIoEn_PIN)

/* Callback function prototype for EINK delay in milliseconds */
typedef void (* cy_eink_delay_function_t) (uint32_t delayMs); 

/* Function pointer for EINK delay in milliseconds */
extern cy_eink_delay_function_t Cy_EINK_Delay;
   
/* Functions used for E-INK driver communication */
void    Cy_EINK_InitSPI(void);
void    Cy_EINK_AttachSPI(void);
void    Cy_EINK_DetachSPI(void);
void    Cy_EINK_WriteSPI(uint8_t data);
uint8_t Cy_EINK_ReadSPI(uint8_t data);
bool    Cy_EINK_IsBusy(void);

/* Function used to register the EINK delay function */
void    Cy_EINK_RegisterDelayFunction(cy_eink_delay_function_t delayFunction);

#endif /* CY_EINK_PSOC_INTERFACE_H */  
/* [] END OF FILE */
