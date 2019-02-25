-------------------------------------------------------------------------------
CE220060_PSoC6_WatchdogTimer
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
PSoC 6 Wi-Fi BT Prototyping Kit

Overview
--------
This example explains the two use cases of WDT – as a watchdog that causes 
a device reset in the case of a malfunction, and as a periodic interrupt 
source. A macro definition determines which mode to use. 

Out of the box, this example demonstrates the periodic interrupt. The red 
LED toggles on every interrupt at an interval of ~1s. 

For reset mode, you change the macro definition and enable an infinite 
loop in the main() function, to block execution. No LED turns on and the 
device resets in ~6s. The red LED blinks twice after the device comes 
out of reset. If you use reset mode without blocking execution, the 
device does not reset. The red LED toggles every 1s in the main loop 
to indicate the CPU is in action. 

In addition, the red LED blinks once for power cycling or external reset
event.

Build and Program
-----------------
First, select any project in the application. 
In the Quick Panel click the CE220060_PSoC6_WatchdogTimer Program (KitProg3) link.
The IDE builds the application, programs the kit, and starts execution.

Code Example Document
---------------------
The PDF file for this example is in the example folder. It has full details
on the design, how to use the code example, and what to look for when it runs.

-------------------------------------------------------------------------------