/*******************************************************************************
* File Name: LED.h
*
* Version: 1.00
*
* Description:
*  Contains the function prototypes and constants available to the code example
*  for LED operation.
*
********************************************************************************
* Copyright 2018-2019, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef LED_H

    #include "cycfg.h"

    #define LED_H

    /***************************************
    *           Constants
    ***************************************/
    #define LED_OFF                     1u
    #define LED_ON                      0u

	#define Alert_LED					KIT_RGB_B
	#define Advertising_LED				KIT_RGB_G
	#define Disconnect_LED				KIT_RGB_R

	#define Alert_LED_PORT				KIT_RGB_B_PORT
	#define Advertising_LED_PORT		KIT_RGB_G_PORT
	#define Disconnect_LED_PORT			KIT_RGB_R_PORT

	#define Alert_LED_PIN				KIT_RGB_B_PIN
	#define Advertising_LED_PIN			KIT_RGB_G_PIN
	#define Disconnect_LED_PIN			KIT_RGB_R_PIN

#endif

/* [] END OF FILE */
