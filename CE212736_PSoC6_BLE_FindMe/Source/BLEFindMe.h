/*******************************************************************************
* File Name: BLEFindMe.h
*
* Version: 1.00
*
* Description:
*  Contains the function prototypes and constants for the BLE functionality.
*
********************************************************************************
* Copyright 2018, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef BLEFINDME_H

    #define BLEFINDME_H

	#include "cy_device_headers.h"
	#include "cycfg.h"
	#include "cycfg_ble.h"
	#include "cy_syspm.h"
    #include "debug.h"
    #include "LED.h"

    /***************************************
    *       Function Prototypes
    ***************************************/
    void BleFindMe_Init(void);
    void BleFindMe_Process(void);

#endif

/* [] END OF FILE */
