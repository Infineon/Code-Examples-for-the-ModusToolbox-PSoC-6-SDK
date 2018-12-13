-------------------------------------------------------------------------------
CE213903_PSoC6DfuBasic
-------------------------------------------------------------------------------

Requirements
------------
Tool: ModusToolbox™ IDE 1.0
Programming Language: C
Associated Parts: All PSoC® 6 MCU parts

Supported Kits
--------------
PSoC 6 BLE Pioneer Kit

Overview
--------
These examples demonstrate several basic DFU operations:
- Downloading an application from a host, using various ModusToolbox resources
  for host communication
- Installing the downloaded application into flash or external memory
- Validating an application, and then transferring control to that application
Multiple communication channels are supported, including UART, I2C, and SPI.

Build and Program
-----------------
First, select any App0 project in the application. 
  In the Quick Panel click the <application name> Program (KitProg3) link.
  The IDE builds the application, programs the kit, and starts execution.
Then, program the App1 application.
  Run the Device Firmware Update Host Tool and download App1 to the target kit.

Code Example Document
---------------------
The PDF file for this example is in the example folder. It has full details
on the design, how to use the code example, and what to look for when it runs.

-------------------------------------------------------------------------------