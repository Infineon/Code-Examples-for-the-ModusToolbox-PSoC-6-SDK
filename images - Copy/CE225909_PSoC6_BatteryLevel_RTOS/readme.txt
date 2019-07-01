-------------------------------------------------------------------------------
CE225909_PSoC6_BatteryLevel_RTOS
-------------------------------------------------------------------------------

Requirements
------------
Tool: ModusToolbox™ IDE
Programming Language: C
Associated Parts: All PSoC 6 MCU with BLE Connectivity parts

Supported Kits
--------------
PSoC 6 BLE Pioneer Kit

Overview
--------
This code example implements a GATT server with BLE standard Battery Service 
and Device Information Service. Battery level is simulated in the firmware 
and its value changes continuously from 0 to 100 percent. The design uses 
red LED (LED9) on the CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit for indication
(OFF, Flashing, or ON for no device connected, advertising, or connected 
respectively).

The USB-BLE dongle provided with the CY8CKIT-062-BLE Pioneer kit or an 
iOS/Android mobile device can act as the BLE Central device.


Build and Program
-----------------
Read the code example PDF file to ensure you have the right software set up.

Then select the <application name>_mainapp folder in the project explorer. 
In the Quick Panel click the <application name> Program (KitProg3) link.
The IDE builds the application, programs the kit, and starts execution.

Code Example Document
---------------------
The PDF file for this example is in the example folder. It has full details
on the design, how to use the code example, and what to look for when it runs.

-------------------------------------------------------------------------------