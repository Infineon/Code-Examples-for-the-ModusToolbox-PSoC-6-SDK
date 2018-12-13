/*
* Copyright YOUR COMPANY, THE YEAR
* All Rights Reserved
* UNPUBLISHED, LICENSED SOFTWARE.
*
* CONFIDENTIAL AND PROPRIETARY INFORMATION
* WHICH IS THE PROPERTY OF your company.
*/

#include "cy_device_headers.h"
#include "cycfg.h"

#include "cy_dfu.h"

CY_SECTION(".cy_app_signature") __USED static const uint32_t cy_bootload_appSignature[1];

int main(void)
{
    /* Set up the device based on configurator selections */
    init_cycfg_all();

    __enable_irq();
 
    for(;;)
    {
        /* Blink twice per second */
         Cy_GPIO_Inv(KIT_RGB_R_PORT, KIT_RGB_R_PIN);
         Cy_SysLib_Delay(250u);

         /* If Button clicked and switch to App0 */
         if (Cy_GPIO_Read(KIT_BTN1_PORT, KIT_BTN1_PIN) == 0u)
         {
             /* 50 ms delay for button debounce on button press */
             Cy_SysLib_Delay(50u);

             if (Cy_GPIO_Read(KIT_BTN1_PORT, KIT_BTN1_PIN) == 0u)
             {
                 while (Cy_GPIO_Read(KIT_BTN1_PORT, KIT_BTN1_PIN) == 0u)
                 {   /* 50 ms delay for button debounce on button release */
                     Cy_SysLib_Delay(50u);
                 }

                 Cy_Bootload_ExecuteApp(0u);
             }
         }
    }
}
