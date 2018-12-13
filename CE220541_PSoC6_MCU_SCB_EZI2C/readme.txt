-------------------------------------------------------------------------------
CE220541_PSoC6_MCU_SCB_EZI2C
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
This code example implements an I2C slave using an SCB Component (configured as 
EZI2C), which receives the data required to control an LED from an I2C master. 
In this example, a host PC running the Cypress’ Bridge Control Panel (BCP) 
software is used as an I2C master. The LED control is implemented using a TCPWM 
Resource (configured as PWM). The intensity of the LED is controlled by 
changing the duty cycle of the PWM signal.

Build and Program
-----------------
First, select any project in the application. 
In the Quick Panel click the <application name> Program (KitProg3) link.
The IDE builds the application, programs the kit, and starts execution.

Code Example Document
---------------------
The PDF file for this example is in the example folder. It has full details
on the design, how to use the code example, and what to look for when it runs.

-------------------------------------------------------------------------------