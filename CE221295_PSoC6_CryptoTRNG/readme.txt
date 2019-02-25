-------------------------------------------------------------------------------
CE221295_PSoC_6_MCU_CryptoTRNG
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
This example demonstrates generating a One-Time-Password (OTP) of eight 
characters in length. Using the True Random Number generation feature of PSoC 6 
MCU crypto block, a random number corresponding to each character of the OTP is
generated. The generated random number is such that it corresponds to
alpha-numeric and special characters of the ASCII code. The generated OTP is
then displayed on a UART terminal emulator. 

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