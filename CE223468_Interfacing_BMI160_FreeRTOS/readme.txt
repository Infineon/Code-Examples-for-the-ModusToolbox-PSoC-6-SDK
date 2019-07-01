-------------------------------------------------------------------------------
CE223468_Interfacing_BMI160(I2C)_FreeRTOS
-------------------------------------------------------------------------------

Requirements
------------
Tool: ModusToolbox™ IDE
Programming Language: C
Associated Parts: All PSoC®6 MCU parts

Software Setup
--------------
This code example requires a PC terminal emulator to display orientation  
information.

Open terminal software such as Tera Term and select the KitProg3's COM port 
with a baud rate setting of 115200 bps, data bits 8, parity none, and 
stop bit 1.

Supported Kits
--------------
PSoC 6 BLE Pioneer Kit
PSoC 6 WiFi-BT Pioneer Kit

Overview
--------
This code example demonstrates how to interface PSoC® 6 MCU with a BMI160
Motion Sensor over an I2C interface within a FreeRTOS task. This example
reads raw motion data and estimates the orientation of the board.

Build and Program
-----------------
Select any project in the application.
In the Quick Panel click the <application name> Program (KitProg3) link.
The IDE builds the application, programs the kit, and starts execution.

Code Example Document
---------------------
The PDF file for this example is in the example folder. It has full details
on the design, how to use the code example, and what to look for when it runs.

-------------------------------------------------------------------------------