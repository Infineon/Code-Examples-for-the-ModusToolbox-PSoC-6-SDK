-------------------------------------------------------------------------------
CE219656_UARTLowLevelAPIs
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

Overview
--------
This example contains three applications that implement a UART using: polling, ISR, or DMA. 
Each application uses low level UART APIs and echoes what is received on the UART serial terminal. 
The UARTLowLevelPolling example polls repeatedly. The UARTLowLevelUserISR example uses a user interrupt. 
The UARTLowLevelDMA example uses DMA functions.

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