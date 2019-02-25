-------------------------------------------------------------------------------
CE221773_HelloWorld
-------------------------------------------------------------------------------

Requirements
------------
Tool: ModusToolbox™ IDE
Programming Language: C
Associated Parts: All PSoC 6 MCU parts

Supported Kits
--------------
PSoC 6 BLE Pioneer Kit
PSoC 6 WiFi-BT Pioneer Kit
PSoC 6 WiFi-BT Prototyping Kit
PSoC 6 WiFi-BT-FRAM Pioneer Kit

Overview
--------
This example uses the Cortex-M4 (CM4) CPU of PSoC 6 MCU to execute two tasks: 
UART communication and LED control. At device reset, the Cortex-M0+ (CM0+) 
CPU enables the CM4 CPU. The CM4 CPU uses a UART resource to print a 
“Hello World” message in a UART terminal emulator. When the user presses the 
Enter key, the LED on the kit starts blinking.

Build and Program
-----------------
First, select the <application name>_mainapp folder in the project explorer. 
In the Quick Panel click the <application name> Program (KitProg3) link.
The IDE builds the application, programs the kit, and starts execution.

Code Example Document
---------------------
The PDF file for this example is in the example folder. It has full details
on the design, how to use the code example, and what to look for when it runs.

-------------------------------------------------------------------------------
