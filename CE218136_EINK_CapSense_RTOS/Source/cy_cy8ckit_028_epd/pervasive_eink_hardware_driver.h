/******************************************************************************
* Copyright (2018), Pervasive Displays Inc.
*******************************************************************************
*
*  Authors: Pervasive Displays Inc.
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
/******************************************************************************
* For the details of E-INK display hardware and driver interface, see the 
* documents available at the following website:
* http://www.pervasivedisplays.com/products/271
*******************************************************************************/

/* Include Guard */
#ifndef PERVASIVE_EINK_HARDWARE_DRIVER_H
#define PERVASIVE_EINK_HARDWARE_DRIVER_H

/* Header file includes */
#include "cy_eink_psoc_interface.h"
#include "pervasive_eink_configuration.h"
    
/* Macros of the basic frame (white and black) addresses.
   These addresses do not occur as pointers to actual images */
/* Null pointer is used to denote the white frame */    
#define PV_EINK_WHITE_FRAME_ADDRESS  NULL
/* Final address of the main Flash region is used to denote the black frame */
#define PV_EINK_BLACK_FRAME_ADDRESS  (uint8_t*)0x100FFFFFu
    
/* Data type of E-INK image */
typedef unsigned char                pv_eink_frame_data_t;

/* Data-type of E-INK status messages */
typedef enum
{
    PV_EINK_RES_OK,
    PV_EINK_ERROR_BUSY,
    PV_EINK_ERROR_ID,
    PV_EINK_ERROR_BREAKAGE,
    PV_EINK_ERROR_DC,
    PV_EINK_ERROR_CHARGEPUMP
}   pv_eink_status_t;

/* Data-type of E-INK update stages */
typedef enum 
{ 
    PV_EINK_STAGE1,
    PV_EINK_STAGE2,
    PV_EINK_STAGE3,
    PV_EINK_STAGE4
}   pv_eink_stage_t ;

/* Declarations of functions defined in pv_eink_hardware_driver.c */
/* Power control and initialization functions */
void             Pv_EINK_Init(void);
pv_eink_status_t Pv_EINK_HardwarePowerOn(void);
pv_eink_status_t Pv_EINK_HardwarePowerOff(void);

/* Set refresh time factor based on the ambient temperature */
void Pv_EINK_SetTempFactor(int8_t temperature);

/* Display update functions */
void Pv_EINK_FullStageHandler(pv_eink_frame_data_t* imagePtr, 
                              pv_eink_stage_t stageNumber);
void Pv_EINK_PartialStageHandler(pv_eink_frame_data_t* previousImagePtr, 
                                 pv_eink_frame_data_t* newImagePtr);

#endif  /* PERVASIVE_EINK_HARDWARE_DRIVER_H */
/* [] END OF FILE */
