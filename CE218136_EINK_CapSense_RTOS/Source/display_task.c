/******************************************************************************
* File Name: display_task.c
*
* Version: 1.00
*
* Description: This file contains the task that shows an interactive menu and
*              text pages based on the commands received
*
* Related Document: CE218136_EINK_CapSense_RTOS.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*                      CY8CKIT-028-EPD E-INK Display Shield
*
******************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation.
******************************************************************************
* This software, including source code, documentation and related materials
* ("Software") is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and 
* foreign), United States copyright laws and international treaty provisions. 
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the 
* Cypress source code and derivative works for the sole purpose of creating 
* custom software in support of licensee product, such licensee product to be
* used only in conjunction with Cypress's integrated circuit as specified in the
* applicable agreement. Any reproduction, modification, translation, compilation,
* or representation of this Software except as specified above is prohibited 
* without the express written permission of Cypress.
* 
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes to the Software without notice. 
* Cypress does not assume any liability arising out of the application or use
* of Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use as critical components in any products 
* where a malfunction or failure may reasonably be expected to result in 
* significant injury or death ("ACTIVE Risk Product"). By including Cypress's 
* product in a ACTIVE Risk Product, the manufacturer of such system or application
* assumes all risk of such use and in doing so indemnifies Cypress against all
* liability. Use of this Software may be limited by and subject to the applicable
* Cypress software license agreement.
*****************************************************************************/
/*******************************************************************************
* This file contains the task that shows an interactive menu and text pages 
* based on the commands received
*
* For the details of the E-INK display and library functions, see the code 
* example CE218136 - PSoC 6 MCU E-INK Display with CapSense (RTOS)
*
* This code example uses emWin middleware for graphics. For documentation and API
* references of emWin, visit:
* https://www.segger.com/products/user-interface/emwin/
*******************************************************************************/

/* Header file includes */
#include "./cy_cy8ckit_028_epd/cy_cy8ckit_028_epd.h"
#include "./images_and_text/screen_contents.h"
#include <math.h>
#include "display_task.h"
#include "menu_configuration.h"
#include "touch_task.h"
#include "uart_debug.h"
#include "cycfg.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "GUI.h"

/* Macros for temperature measurement using thermistor, which is
   used for the E-INK temperature compensation */
/* Reference resistor in series with the thermistor is of 10 kOhm */
#define R_REFERENCE         		(float)(10000)

/* Beta constant of this thermistor is 3380 Kelvin. See the thermistor
   (NCP18XH103F03RB) data sheet for more details.
 */
#define B_CONSTANT          		(float)(3380)

/* Resistance of the thermistor is 10K at 25 degrees C (from data sheet)
   Therefore R0 = 10000 Ohm, and T0 = 298.15 Kelvin, which gives
   R_INFINITY = R0 e^(-B_CONSTANT / T0) = 0.1192855
 */
#define R_INFINITY          		(float)(0.1192855)

/* Zero Kelvin in degree C */
#define ABSOLUTE_ZERO       		(float)(-273.15)

/* Frame buffers used by the display update functions  */
cy_eink_frame_t	currentFrameBuffer[CY_EINK_FRAME_SIZE];
cy_eink_frame_t	previousFrameBuffer[CY_EINK_FRAME_SIZE];

/* Reference to the bitmap image for the startup screen */
extern GUI_CONST_STORAGE GUI_BITMAP bmCypressLogo_1bpp;

/* Variable that stores the details of the current screen */
screen_t  currentScreen =
{
    .screen     = MAIN_MENU,
    .menuItem   = MAIN_MENU_INDEX_START,
    .textPage   = TEXT_PAGE_INDEX_START
};

/* emWin function hook to copy the display buffer */
extern void LCD_CopyDisplayBuffer(uint8* destination, int count);

/*  These static functions are not available outside this file. 
    See the respective function definitions for more details */
void static NoScreenChange(void);
void static MoveArrowUp(void);
void static MoveArrowDown(void);
void static GoToTextPage(void);
void static GoToMainMenu(void);
void static PreviousTextPage(void);
void static NextTextPage(void);
void static ShowStartupScreen(void);
int8_t static ReadAmbientTemperature(void);

/* Function used to register the E-INK delay call back */
void static DelayMs(uint32_t delayInMs)  {vTaskDelay(pdMS_TO_TICKS(delayInMs));}

/*******************************************************************************
* Function Name: void Task_Display (void *pvParameters)
********************************************************************************
* Summary:
*  Task that processes the touch command received and the updates the display
*  with the corresponding menu / text page
*
* Parameters:
*  void *pvParameters : Task parameter defined during task creation (unused)                            
*
* Return:
*  void
*
*******************************************************************************/
void Task_Display (void *pvParameters)
{  
    /* Flag that indicates if the E-INK display has been detected */
    cy_eink_api_result displayDetected;
    
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtosApiResult;
    
    /* Ambient temperature value in degree Celsius, that used to initialize
       E-INK. Display refreshes faster at higher ambient temperatures */
    int8_t ambientTemperature;

    /* Variable that stores the touch input received from the Touch Task */
    touch_data_t touchInput;

    /* Remove warning for unused parameter */
    (void)pvParameters ;
    
    /* Initialize emWin Graphics */
    GUI_Init();

    /* Read the ambient temperature value from the thermistor circuit */
    ambientTemperature = ReadAmbientTemperature();

    /* Initialize the E-INK display hardware with the ambient temperature 
       value and the delay function pointer */
    if(Cy_EINK_Start(ambientTemperature,DelayMs) == CY_EINK_SUCCESS)
    {
        Task_DebugPrintf("Success  : Display - Cy_EINK_Start API", 0u);

        /* Power on the display and check if the operation was successful */
        displayDetected = Cy_EINK_Power(CY_EINK_ON);

        if(displayDetected == CY_EINK_SUCCESS)
        {
            Task_DebugPrintf("Success  : Display - E-INK display power on", 0u);   
        } 
        else
        {
            Task_DebugPrintf("Failure! : Display - E-INK display power on ", 0u); 
            
            /* Turn the red LED on to indicate that the E-INK display is not 
            detected. Check the connection between the E-INK shield and the 
            Pioneer Baseboard if this happens, and then reset the PSoC 6 BLE */
            Cy_GPIO_Clr(KIT_LED2_PORT, KIT_LED2_PIN);
        }
    }
    else
    {
        Task_DebugPrintf("Failure! : Display - Cy_EINK_Start API", 0u);
    }

    /* Show the startup screen */
    ShowStartupScreen();

    /* Keep the logo on for a specific time and then load the menu */
    vTaskDelay(STARTUP_SCREEN_DELAY);
    GoToMainMenu();

    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Block until a touch input has been received over touchDataQ */
        rtosApiResult = xQueueReceive(touchDataQ, &touchInput,
                         portMAX_DELAY);
        
        /* Touch input has been received over touchDataQ */
        if(rtosApiResult == pdTRUE)
        {
       
        /* Function pointer that selects a screen update function based on the 
          current screen type and the touch input:
        _______________________________________________________________
        |                                     |                       |
        |          Parameters                 |   Selected Function   |
        |_____________________________________|_______________________|
        |                                     |                       |
        |   [MAIN_MENU][BUTTON0_TOUCHED])     |   GoToTextPage        |
        |   [MAIN_MENU][BUTTON1_TOUCHED])     |   NoScreenChange      |
        |   [MAIN_MENU][SLIDER_FLICK_LEFT])   |   MoveArrowUp         |
        |   [MAIN_MENU][SLIDER_FLICK_RIGHT])  |   MoveArrowDown       |
        |                                     |                       |
        |   [TEXT_PAGE][BUTTON0_TOUCHED])     |   NoScreenChange      |
        |   [TEXT_PAGE][BUTTON1_TOUCHED])     |   GoToMainMenu        |
        |   [TEXT_PAGE][SLIDER_FLICK_LEFT])   |   PreviousTextPage    |
        |   [TEXT_PAGE][SLIDER_FLICK_RIGHT])  |   NextTextPage        |
        |_____________________________________|_______________________|*/
            
        static void (*ScreenUpdatePointer[NUMBER_OF_SCREEN_TYPES]
                                         [NUMBER_OF_INPUT_TYPES]) (void) =
        {
            { GoToTextPage, NoScreenChange, MoveArrowUp, MoveArrowDown },
            { NoScreenChange, GoToMainMenu, PreviousTextPage,NextTextPage }
        };

        	/* Call the function pointer that selects a screen update function
            according to the touch input */     
            (*ScreenUpdatePointer[currentScreen.screen][touchInput])();

        }
        /* Task has timed out and received no inputs during an interval of 
           portMAXDELAY ticks */
        else
        {
            Task_DebugPrintf("Warning! : Display - Task Timed out ", 0u);   
        }
    }
}

/*******************************************************************************
********************************************************************************
*  Following are the static functions used for display updates. These          *
*  functions are not available outside this file.                              *
********************************************************************************
*******************************************************************************/

/*******************************************************************************
* Function Name: void static UpdateDisplay(void)
********************************************************************************
*
* Summary: This function updates the display with the data in the display
*            buffer.  The function first transfers the content of the EmWin
*            display buffer to the primary EInk display buffer.  Then it calls
*            the Cy_EINK_ShowFrame function to update the display, and then
*            it copies the EmWin display buffer to the E-INK display cache buffer
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  It takes about a second to refresh the display.  This is a blocking function
*  and only returns after the display refresh
*
*******************************************************************************/
void static UpdateDisplay(cy_eink_update_t updateMethod, bool powerCycle)
{
    /* Copy the EmWin display buffer to imageBuffer*/
    LCD_CopyDisplayBuffer(currentFrameBuffer, CY_EINK_FRAME_SIZE);

    /* Turn the Orange LED on to indicate that the E-INK display is refreshing */
    Cy_GPIO_Clr(KIT_LED1_PORT, KIT_LED1_PIN);

    /* Update the EInk display */
    Cy_EINK_ShowFrame(previousFrameBuffer, currentFrameBuffer, updateMethod,
    				  powerCycle);

    /* Turn off the Orange LED on to indicate that the E-INK refresh has finished */
    Cy_GPIO_Set(KIT_LED1_PORT, KIT_LED1_PIN);

    /* Copy the EmWin display buffer to the imageBuffer cache*/
    LCD_CopyDisplayBuffer(previousFrameBuffer, CY_EINK_FRAME_SIZE);
}

/*******************************************************************************
* Function Name: void static NoScreenChange(void)
********************************************************************************
*
* Summary:
*  Null function that is called by the "ScreenUpdatePointer" function pointer  
*  when no screen change is required
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*
*******************************************************************************/
void static NoScreenChange(void)
{
}

/*******************************************************************************
* Function Name: void static GoToTextPage(void)
********************************************************************************
*
* Summary:
*  Performs transition from the main menu to the text page
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*
*******************************************************************************/
void static GoToTextPage(void)
{
    /* Variable that stores the index of the character array, which in turn 
       stores the current text page as a string */
    uint8_t currentPageIndex;
    
    /* Change the current screen type to text page */
    currentScreen.screen = TEXT_PAGE;

    /* Re-initialize the text page index to point to the start page */
    currentScreen.textPage = TEXT_PAGE_INDEX_START;
    
    /* Access the index array and fetch the index of the character array that 
      stores the current text page as a string */
    currentPageIndex = textPageIndex[currentScreen.menuItem]
                                    [currentScreen.textPage];

    /* Check if the fetched index is valid */
    if (currentPageIndex != INVALID_PAGE_INDEX)
    {
    	/* Load the screen and font settings */
    	GUI_SetFont(GUI_FONT_13B_1);
    	GUI_SetColor(GUI_BLACK);
    	GUI_SetBkColor(GUI_WHITE);
    	GUI_SetTextMode(GUI_TM_NORMAL);
    	GUI_SetTextStyle(GUI_TS_NORMAL);

    	/* Clear the screen */
    	GUI_Clear();

    	/* Display string inside given margins with word wrap. Margin
    	   coordinates are selected in such a way that 2 pixels are
    	   left outside the margins in all 4 dimensions */
    	GUI_RECT textMargins = {2u, 2u, 262u, 174u};
    	GUI_DispStringInRectWrap(textPage[currentPageIndex], &textMargins,
    							 GUI_TA_LEFT, GUI_WRAPMODE_WORD);

    	/* Send the display buffer data to display*/
    	UpdateDisplay(CY_EINK_FULL_4STAGE, true);
    }
}

/*******************************************************************************
* Function Name: void static GoToMainMenu(void)
********************************************************************************
*
* Summary:
*  Enters the main menu from the startup screen or a text page
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*
*******************************************************************************/
void static GoToMainMenu(void)
{
    /* Change the current screen type to main menu */
    currentScreen.screen = MAIN_MENU;
    
    /* Set foreground and background color and font size */
	GUI_SetColor(GUI_BLACK);
	GUI_SetBkColor(GUI_WHITE);
	GUI_Clear();

	/* Render the menu. The coordinates are selected to align with the
	   cursor arrows and have the menu items at equal spacing */
	GUI_SetFont(GUI_FONT_24B_1);
	GUI_SetTextAlign(GUI_TA_LEFT);
	GUI_DispStringAt("1.PSoC 6 FEATURES", 25u, 5u);
	GUI_DispStringAt("2.KIT FEATURES", 25u, 40u);
	GUI_DispStringAt("3.KIT DOCUMENTATION", 25u, 75u);
	GUI_DispStringAt("4.KIT CODE EXAMPLES", 25u, 110u);

	/* Show the instructions at the bottom of the display, under a line that
	   spans from left to right */
	GUI_SetPenSize(1);
	GUI_DrawLine(0u, 140u, 263u, 140u);
	GUI_SetFont(GUI_FONT_13B_1);
	GUI_DispStringAt("Flick the slider left/right to move the cursor.",5u,145u);
	GUI_DispStringAt("Press BTN0 to select an option.",5u,160u);

	/* Calculate the vertical offset and display the cursor */
	uint8 vOff;
	vOff = currentScreen.menuItem*35u;
	GUI_SetPenSize(4u);
	GUI_DrawLine(5u, 5+vOff, 15u, 15+vOff);
	GUI_DrawLine(5u, 25+vOff, 15u, 15+vOff);
	GUI_DrawLine(5u, 5+vOff, 5u, 25+vOff);

	/* Send the display buffer data to display*/
	UpdateDisplay(CY_EINK_FULL_4STAGE, true);
}

/*******************************************************************************
* Function Name: void static MoveArrowUp(void)
********************************************************************************
*
* Summary:
*  Moves the selection arrow of the main menu upwards
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*
*******************************************************************************/
void static MoveArrowUp(void)
{
    
    /* If the beginning of the main menu is reached, then move the arrow to the
       final item of the main menu by selecting the maximum index */
    if (currentScreen.menuItem == MAIN_MENU_INDEX_START)
    {
        currentScreen.menuItem = MAIN_MENU_MAX_INDEX;
    }
    /* Otherwise, decrement the index to move the arrow up */
    else
    {
        currentScreen.menuItem--;
    }
    
	/* Clear the cursor area */
	GUI_SetColor(GUI_WHITE);
	GUI_FillRect(3u, 4u, 18u, 134u);

	/* Calculate the vertical offset and update the cursor */
	uint8 vOff;
	vOff = currentScreen.menuItem*35u;
	GUI_SetColor(GUI_BLACK);
	GUI_SetPenSize(4);
	GUI_DrawLine(5u, 5+vOff, 15u, 15+vOff);
	GUI_DrawLine(5u, 25+vOff, 15u, 15+vOff);
	GUI_DrawLine(5u, 5+vOff, 5u, 25+vOff);

	/* Send the display buffer data to display*/
	UpdateDisplay(CY_EINK_PARTIAL, true);
}

/*******************************************************************************
* Function Name: void static MoveArrowDown(void)
********************************************************************************
*
* Summary:
*  Moves the selection arrow of the main menu downwards
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*
*******************************************************************************/
void static MoveArrowDown(void)
{ 
    /* If the final item of the main menu is reached, then move the arrow to the
       beginning of the main menu by selecting the starting index */
    if (currentScreen.menuItem >= MAIN_MENU_MAX_INDEX)
    {
        currentScreen.menuItem = MAIN_MENU_INDEX_START;
    }
    /* Otherwise, increment the index to move the arrow down */
    else
    {
        currentScreen.menuItem++;
    }
    
	/* Clear the cursor area */
	GUI_SetColor(GUI_WHITE);
	GUI_FillRect(3u, 4u, 18u, 134u);

	/* Calculate the vertical offset and update the cursor */
	uint8 vOff;
	vOff = currentScreen.menuItem*35u;
	GUI_SetColor(GUI_BLACK);
	GUI_SetPenSize(4);
	GUI_DrawLine(5u, 5+vOff, 15u, 15+vOff);
	GUI_DrawLine(5u, 25+vOff, 15u, 15+vOff);
	GUI_DrawLine(5u, 5+vOff, 5u, 25+vOff);

	/* Send the display buffer data to display*/
	UpdateDisplay(CY_EINK_PARTIAL, true);
}

/*******************************************************************************
* Function Name: void static PreviousTextPage(void)
********************************************************************************
*
* Summary:
*  Performs the transition to the previous text page
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*
*******************************************************************************/
void static PreviousTextPage(void)
{
    
    /* Variable that stores the index of the character array, which in turn 
       stores the current text page as a string */
    uint8_t currentPageIndex;
    
    /* If the start page is reached, then go to the final page by selecting the 
       maximum index of the text pages */
    if (currentScreen.textPage == TEXT_PAGE_INDEX_START)
    {
        currentScreen.textPage = maxTextPageIndexes[currentScreen.menuItem];
    }
    /* Otherwise, select the previous page by decrementing the index */
    else
    {
        currentScreen.textPage--;
    }

    /* Access the index array and fetch the index of the character array that
        stores the current text page as a string */
    currentPageIndex = textPageIndex[currentScreen.menuItem]
                                    [currentScreen.textPage];
    
    /* Check if the fetched index is valid */
    if (currentPageIndex != INVALID_PAGE_INDEX)
    {
		/* Load the graphics settings for text pages */
    	GUI_RECT textMargins = {2u, 2u, 262u, 174u};
		GUI_SetFont(GUI_FONT_13B_1);
		GUI_SetColor(GUI_BLACK);
		GUI_SetBkColor(GUI_WHITE);
		GUI_SetTextMode(GUI_TM_NORMAL);
		GUI_SetTextStyle(GUI_TS_NORMAL);

		/* Clear the screen */
		GUI_Clear();

		/* Display string in middle rectangle with word wrap */
		GUI_DispStringInRectWrap(textPage[currentPageIndex], &textMargins,
								 GUI_TA_LEFT, GUI_WRAPMODE_WORD);

		/* Send the display buffer data to display*/
		UpdateDisplay(CY_EINK_FULL_4STAGE, true);
    }
}

/*******************************************************************************
* Function Name: void static NextTextPage(void)
********************************************************************************
*
* Summary:
*  Performs the transition to the next text page
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*
*******************************************************************************/
void static NextTextPage(void)
{
    /* Variable that stores the index of the character array, which in turn 
       stores the current text page as a string */
    uint8_t currentPageIndex;
    
    /* If the final page is reached, then go to the start page by selecting 
       the starting index of the text pages */
    if (currentScreen.textPage >= maxTextPageIndexes[currentScreen.menuItem])
    {
        currentScreen.textPage = TEXT_PAGE_INDEX_START;
    }
     /* Otherwise, select the next page by incrementing the index */
    else
    {
        currentScreen.textPage++;
    }

    /* Access the index array and fetch the index of the character array that
        stores the current text page as a string */
    currentPageIndex = textPageIndex[currentScreen.menuItem]
                                    [currentScreen.textPage];
    
    /* Check if the fetched index is valid */
    if (currentPageIndex != INVALID_PAGE_INDEX)
    {
		/* Load the graphics settings for text pages */
    	GUI_RECT textMargins = {2u, 2u, 262u, 174u};
		GUI_SetFont(GUI_FONT_13B_1);
		GUI_SetColor(GUI_BLACK);
		GUI_SetBkColor(GUI_WHITE);
		GUI_SetTextMode(GUI_TM_NORMAL);
		GUI_SetTextStyle(GUI_TS_NORMAL);

		/* Clear the screen */
		GUI_Clear();

		/* Display string in middle rectangle with word wrap */
		GUI_DispStringInRectWrap(textPage[currentPageIndex], &textMargins,
								 GUI_TA_LEFT, GUI_WRAPMODE_WORD);

		/* Send the display buffer data to display*/
		UpdateDisplay(CY_EINK_FULL_4STAGE, true);
    }
}

/*******************************************************************************
* Function Name: void static ShowStartupScreen(void)
********************************************************************************
*
* Summary: This function displays the startup screen with Cypress Logo and
*            the kit description text
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void static ShowStartupScreen(void)
{
    /* Set foreground and background color and font size */
    GUI_SetFont(GUI_FONT_16B_1);
    GUI_SetColor(GUI_BLACK);
    GUI_SetBkColor(GUI_WHITE);
    GUI_Clear();

    /* Render the logo and the text */
    GUI_DrawBitmap(&bmCypressLogo_1bpp, 2u, 2u);
    GUI_SetTextAlign(GUI_TA_HCENTER);
    GUI_DispStringAt("CY8CKIT-062-BLE", 132u, 105u);
    GUI_SetTextAlign(GUI_TA_HCENTER);
    GUI_DispStringAt("PSoC 6 BLE PIONEER KIT", 132u, 125u);

    /* Send the display buffer data to display*/
    UpdateDisplay(CY_EINK_FULL_4STAGE, true);
}

/*******************************************************************************
* Function Name: int8_t static ReadAmbientTemperature(void)
********************************************************************************
*
* Summary: Reads the ambient temperature using thermistor
*
* Parameters:
*  None
*
* Return:
*  int8_t : Temperature in degree Celsius
*
*******************************************************************************/
int8_t static ReadAmbientTemperature(void)
{
	/* Variables used to store ADC counts, thermistor resistance and
	the temperature */
	int16_t countThermistor;
	int16_t countReference;
	float rThermistor;
	float temperature = 0;
	int8_t tempInt;

	cy_en_sar_status_t adcStatus;

	/* Initialize and enable the ADC */
	Cy_SysAnalog_Enable();
	Cy_SAR_Init(KIT_ADC_HW, &KIT_ADC_config);
	Cy_SAR_Enable(KIT_ADC_HW);

	/* Start ADC conversion and wait for the result */
	Cy_SAR_StartConvert(KIT_ADC_HW, CY_SAR_START_CONVERT_SINGLE_SHOT);
	adcStatus = Cy_SAR_IsEndConversion(KIT_ADC_HW, CY_SAR_WAIT_FOR_RESULT);

	/* Check if the conversion was successful */
	if(adcStatus == CY_SAR_SUCCESS)
	{
		/* Read the ADC count values from channels 0 and 1 */
		countReference  = Cy_SAR_GetResult16(KIT_ADC_HW, 0u);
		countThermistor = Cy_SAR_GetResult16(KIT_ADC_HW, 1u);

		/* Calculate the thermistor resistance and the corresponding temperature */
		rThermistor = (R_REFERENCE*countThermistor)/countReference;
		temperature = (B_CONSTANT/(logf(rThermistor/R_INFINITY)))+ABSOLUTE_ZERO;
	}
	else
	{
		Task_DebugPrintf("Failure! : Display - Temperature readout", 0u);
	}

	/* Put the ADC to sleep so that entering low power modes won't affect
	   the ADC operation
	 */
	Cy_SAR_DeepSleep(KIT_ADC_HW);

	/* Clear the GPIO that drives the thermistor circuit's Vdd, to save power */
	Cy_GPIO_Clr(KIT_SAR_A0_PORT,KIT_SAR_A0_NUM);

	/* Return the temperature value (defaults to 0 if the temperature
	   readout has failed) */
	   
	tempInt =  (int8_t)temperature;
	
	return tempInt;
}

/* [] END OF FILE */
