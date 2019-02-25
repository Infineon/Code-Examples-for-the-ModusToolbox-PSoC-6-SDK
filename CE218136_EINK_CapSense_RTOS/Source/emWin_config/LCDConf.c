/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2017  SEGGER Microcontroller GmbH & Co. KG       *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.46 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with the following terms:

The software  has been licensed to  Cypress Semiconductor Corporation,
whose registered  office is situated  at 198 Champion Ct. San Jose, CA 
95134 USA  solely for the  purposes of creating  libraries for Cypress
PSoC3 and  PSoC5 processor-based devices,  sublicensed and distributed
under  the  terms  and  conditions  of  the  Cypress  End User License
Agreement.
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
Licensing information
Licensor:                 SEGGER Microcontroller Systems LLC
Licensed to:              Cypress Semiconductor Corp, 198 Champion Ct., San Jose, CA 95134, USA
Licensed SEGGER software: emWin
License number:           GUI-00319
License model:            Services and License Agreement, signed June 10th, 2009
Licensed platform:        Any Cypress platform (Initial targets are: PSoC3, PSoC5)
----------------------------------------------------------------------
Support and Update Agreement (SUA)
SUA period:               2009-06-12 - 2022-07-27
Contact to extend SUA:    sales@segger.com
----------------------------------------------------------------------
File        : LCDConf.c
Purpose     : Display configuration file for GUIDRV_Bitplains to be
              used without any display controller. This implementation
              has been written and tested with the RSKR32C111 with
              Visualization Add On.
---------------------------END-OF-HEADER------------------------------
*/

#include <stddef.h>
#include "cy_syslib.h"

#include "GUI.h"

#include "GUIDRV_BitPlains.h"

/*********************************************************************
*
*       Layer configuration
*
**********************************************************************
*/
//
// Physical display size
//
/* Updated with E-INK display's x and y resolutions */
#define XSIZE_PHYS 264
#define YSIZE_PHYS 176

//
// Color conversion
//
#define COLOR_CONVERSION GUICC_1

/*********************************************************************
*
*       Configuration checking
*
**********************************************************************
*/
//
// Checking display driver configuration
//
#ifndef   XSIZE_PHYS
  #error Physical X size of display is not defined!
#endif
#ifndef   YSIZE_PHYS
  #error Physical Y size of display is not defined!
#endif
#ifndef   COLOR_CONVERSION
  #error Color conversion not defined!
#endif

//
// Number of bytes per line
//
#define BYTES_PER_LINE ((XSIZE_PHYS + 7) / 8)

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/
//
// Data arrays to be used by the display driver
//
static U8 _aPlain_0[BYTES_PER_LINE * YSIZE_PHYS];

//
// Structure to be passed to the driver
//
static struct {
  U8 * apVRAM[8];
} _VRAM_Desc = {{
  _aPlain_0,
}};

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       LCD_X_Config
*
* Purpose:
*   Called during the initialization process in order to set up the
*   display driver configuration.
*   
*/
void LCD_X_Config(void) {
	GUI_DEVICE * pDevice;
	CONFIG_BITPLAINS Config;
	
	//Config.Mirror = 1;
	
	
  //
  // Set display driver and color conversion for 1st layer
  //
  pDevice = GUI_DEVICE_CreateAndLink(GUIDRV_BITPLAINS, COLOR_CONVERSION, 0, 0);
  /* Configure Bitplains library */
  GUIDRV_BitPlains_Config(pDevice, &Config);

  //
  // Display driver configuration
  //
  if (LCD_GetSwapXY()) {
    LCD_SetSizeEx (0, YSIZE_PHYS, XSIZE_PHYS);
    LCD_SetVSizeEx(0, YSIZE_PHYS, XSIZE_PHYS);
  } else {
    LCD_SetSizeEx (0, XSIZE_PHYS, YSIZE_PHYS);
    LCD_SetVSizeEx(0, XSIZE_PHYS, YSIZE_PHYS);
  }
  //
  // Initialize VRAM access off driver
  //
  LCD_SetVRAMAddrEx(0, (void *)&_VRAM_Desc);
}

/*********************************************************************
*
*       LCD_X_DisplayDriver
*
* Purpose:
*   This function is called by the display driver for several purposes.
*   To support the according task, the routine needs to be adapted to
*   the display controller. Note that the commands marked
*   "optional" are not cogently required and should only be adapted if 
*   the display controller supports these features.
*
* Parameter:
*   LayerIndex - Zero based layer index
*   Cmd        - Command to be executed
*   pData      - Pointer to a data structure.
* 
* Return Value:
*   < -1 - Error
*     -1 - The command is not handled.
*      0 - OK.
*/
int LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void * pData) {
  int r;
  
  GUI_USE_PARA(LayerIndex);
  GUI_USE_PARA(pData);
  
  switch (Cmd) {
  case LCD_X_INITCONTROLLER: {
    //
    // Called during the initialization process in order to set up the
    // display controller and put it into operation. If the display
    // controller is not initialized by any external routine, this needs
    // to be adapted by the customer...
    //
    // ...
    return 0;
  }
  default:
    r = -1;
  }
  return r;
}
/*********************************************************************
*
*       LCD_CopyDisplayBuffer
*
* Purpose:
*   This function has been added to copy the Bitplains buffer
*   to user memory that can be used to refresh the E-INK
* Parameter:
*   destination - destination memory address
*   count       - Number of bytes to be copied
*   pData      - Pointer to a data structure.
*
* Return Value:
*   < -1 - Error
*     -1 - The command is not handled.
*      0 - OK.
*/
void LCD_CopyDisplayBuffer(uint8* destination, int count)
{
	int i;
	
	/* Copy the Bitplains display buffer to user memory */
	for(i=0; i<count; i++)
	{
		destination[i] = _aPlain_0[i];
	}
}
/*************************** End of file ****************************/
