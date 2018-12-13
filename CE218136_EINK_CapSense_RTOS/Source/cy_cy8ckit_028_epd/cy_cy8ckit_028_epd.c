/******************************************************************************
* File Name: cy_cy8ckit_028_epd.c
*
* Version: 1.00
*
* Description: This file contains the library functions for controlling
*              the E-INK display.
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
* This file contains the library functions for controlling the E-INK display.
*
* For the details of the E-INK display and library functions, see the code  
* example document of CE218136 - PSoC 6 MCU E-INK Display with CapSense (RTOS)
*
* For the details of E-INK display hardware and driver interface, see the 
* documents available at the following website:
* http://www.pervasivedisplays.com/products/271
 *******************************************************************************/

/* Header file includes */
#include "./cy_cy8ckit_028_epd/cy_cy8ckit_028_epd.h"


/*******************************************************************************
* Function Name: cy_eink_api_result Cy_EINK_Start(int8_t temperature,
*                                       cy_eink_delay_function_t delayFunction)
********************************************************************************
*
* Summary: Initialize the E-INK display hardware, starts the required PSoC 
*  components, and also performs temperature compensation of E-INK parameters.
*
*  Note: This function does not turn on the E-INK display.
*
* Parameters:
*  int8_t temperature       : Ambient temperature in degree Celsius
*  cy_eink_delay_function_t : Pointer to a delay function that accepts
*                             delay in milliseconds (uint32_t type)                         
*
* Return:
*  None
*
* Side Effects:
*  Lower ambient temperature results in higher refresh times
*
*******************************************************************************/
cy_eink_api_result Cy_EINK_Start(int8_t temperature,
                                 cy_eink_delay_function_t delayFunction)
{
    /* Variable to store the return value */
    cy_eink_api_result returnValue;
    
    /* Make sure that the function pointer parameter is not NULL */
    if(delayFunction != NULL)
    {
        /* Register the callback function for EINK delay in milliseconds */
        Cy_EINK_RegisterDelayFunction(delayFunction);
        
        /* Initialize the E-INK display hardware and associated PSoC
           components */
        Pv_EINK_Init();
        /* Perform temperature compensation of E-INK parameters */
        Pv_EINK_SetTempFactor(temperature);
        
        /* Return success */
        returnValue = CY_EINK_SUCCESS;
    }
    else
    {
        /* Return failure */
        returnValue = CY_EINK_FAILURE;
    }
    
    return returnValue;
}

/*******************************************************************************
* Function Name: cy_eink_api_result Cy_EINK_Power(bool powerCtrl)
********************************************************************************
*
* Summary: This function is used to turn on/off the E-INK display power.
*
*  Note: This function can not be used to clear the E-INK display. The display 
*  will retain the previously written frame even when it's turned off.
*
* Parameters:
*  bool powerCtrl       : "False" turns off and "True" turns on the display.
*
* Return:
*  cy_eink_api_result   : "CY_EINK_SUCCESS" if operation was successful;
*                         "CY_EINK_FAILURE" otherwise
*
* Side Effects:
*  None
*
*******************************************************************************/
cy_eink_api_result Cy_EINK_Power(bool powerCtrl)
{
    /* Variable to store operation status */
    pv_eink_status_t pwrStatus;
    cy_eink_api_result returnValue;
    
    /* Turn on the E-INK power if powerCtrl is "true" */
    if (powerCtrl == CY_EINK_ON)
    {
        pwrStatus = Pv_EINK_HardwarePowerOn();
    }
    /* Turn off the E-INK power if powerCtrl is "false" */
    else
    {
        pwrStatus = Pv_EINK_HardwarePowerOff();
    }
    /* If the operation was successful, return "true" */
    if (pwrStatus == PV_EINK_RES_OK)
    {
        returnValue = CY_EINK_SUCCESS;
    }
    /* If the operation was not successful, return "false" */
    else
    {
        returnValue = CY_EINK_FAILURE;
    }
    
    /* Return the outcome of the power control operation */
    return (returnValue);
}

/*******************************************************************************
* Function Name: void Cy_EINK_ShowFrame(cy_eink_frame_t* prevFrame, 
*      cy_eink_frame_t* newFrame,CY_EINK_UpdateType updateType, bool powerCycle)
********************************************************************************
*
* Summary: Updates the E-INK display with a frame/image stored in the flash
*          or RAM.
*
*  Notes: This function requires the previous frame data as well as the new 
*  frame data. If the previous frame data changes from the actual frame 
*  previously written to the display, considerable ghosting may occur.
*
*  The E-INK display should be powered on (using Cy_EINK_Power function) before 
*  calling this function, if "powerCycle" parameter is false. Otherwise the 
*  display won't be updated.
*
* Parameters:
*  cy_eink_frame_t* prevFrame    : Pointer to the previous frame written on the 
*                                  display
*  cy_eink_frame_t* newFrame     : Pointer to the new frame that need to be
*                                  written
*  cy_eink_update_t              : Full update (2/4 stages) or  Partial update
*  bool powerCycle               : "true" for automatic power cycle, "false" 
*                                  for manual
*  
* Return:
*  None
*
* Side Effects:
*  This is a blocking function that can take as many as 2 seconds 
*
*******************************************************************************/
void Cy_EINK_ShowFrame(cy_eink_frame_t* prevFrame, cy_eink_frame_t* newFrame,
                       cy_eink_update_t updateType, bool powerCycle)
{
    /* If power cycle operation requested, turn on E-INK power */
    if (powerCycle)
    {
        Cy_EINK_Power(CY_EINK_ON);
    }
    /* Partial update stage */
    if (updateType == CY_EINK_PARTIAL)
    {
        /* Update the display with changes from previous frame */
        Pv_EINK_PartialStageHandler(prevFrame, newFrame);
    }
    /* Full update stages */
    else if ((updateType == CY_EINK_FULL_4STAGE) || 
             (updateType == CY_EINK_FULL_2STAGE))
    {
        /* Stage 1: update the display with the inverted version of the previous 
           frame */
        Pv_EINK_FullStageHandler(prevFrame, PV_EINK_STAGE1);
        
        /* Additional stages that reduce ghosting for a 4 stage full update */
        if (updateType == CY_EINK_FULL_4STAGE)
        {
            /* Stage 2: update the display with an all white frame */
            Pv_EINK_FullStageHandler(prevFrame, PV_EINK_STAGE2);
            /* Stage 3: update the display with the inverted version of the new 
               frame */
            Pv_EINK_FullStageHandler(newFrame, PV_EINK_STAGE3);
        }
        
        /* Stage 4: update the display with the new frame */
        Pv_EINK_FullStageHandler(newFrame, PV_EINK_STAGE4);
    }
    else
    {
    }
    
    /* If power cycle operation requested, turn off E-INK power */
    if (powerCycle)
    {
        Cy_EINK_Power(CY_EINK_OFF);
    }
}

/* [] END OF FILE */
