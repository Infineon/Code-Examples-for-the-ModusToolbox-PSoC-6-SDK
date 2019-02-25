-------------------------------------------------------------------------------
CE212736_PSoC6_BLE_FindMe
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
This design implements a Bluetooth Low Energy (BLE) Find Me Profile (FMP) that 
consists of an Immediate Alert Service (IAS). FMP and IAS are BLE standard 
Profile and Service respectively, as defined by the Bluetooth SIG. The design 
uses the RGB LED on the CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit. The blue LED 
displays the alert level (OFF, flashing, or ON for no alert, mild alert, or 
high alert respectively). Green and red LEDs indicate whether the Peripheral 
device (the Pioneer kit) is advertising or disconnected.

The USB-BLE dongle provided with the CY8CKIT-062-BLE Pioneer kit or an 
iOS/Android mobile device can act as the BLE Central device, which locates 
the Peripheral device.


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