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

int main(void)
{
    /* Set up the device based on configurator selections */
//    init_cycfg_all(); // move this to the CM0+

    __enable_irq();
 
    for(;;)
    {
        Cy_GPIO_Inv(KIT_RGB_R_PORT, KIT_RGB_R_PIN); /* toggle the pin */
        Cy_SysLib_Delay(997/*msec*/);
    }
}
