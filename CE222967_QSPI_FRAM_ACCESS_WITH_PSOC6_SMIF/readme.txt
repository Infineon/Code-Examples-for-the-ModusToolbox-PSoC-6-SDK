-------------------------------------------------------------------------------
CE222967_QSPI_FRAM_ACCESS_WITH_PSOC6_SMIF
-------------------------------------------------------------------------------

Requirements
------------
Tool: ModusToolbox™ IDE
Programming Language: C
Associated Parts: All PSoC®6 MCU parts

Software Setup
--------------
This code example requires a PC terminal emulator to display orientation information.

Open terminal emulator/software such as Tera Term or PuTTY and select the KitProg3's COM port 
with a baud rate setting of 115200 bps, data bits 8, parity none, and stop bit 1.

Supported Kits
--------------
PSoC 6 BLE Pioneer Kit
PSoC 6 WiFi-BT Pioneer Kit

Overview
--------
This code example implements the QSPI F-RAM host controller using PSoC 6 Serial Memory Interface (SMIF) block
and demonstrates accessing the Excelon Ultra QSPI F-RAM in SPI, DPI, and QPI modes. The status LED (RGB) 
turns green when the CE output matches with the expected output, and turns red when the CE output differs 
from the expected output. The code example also enables the UART interface to connect to a PC to monitor the result.

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