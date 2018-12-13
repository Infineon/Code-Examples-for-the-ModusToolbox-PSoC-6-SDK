# Code Examples for the ModusToolbox™ PSoC 6 SDK


# Table of Contents 

* [Overview](#overview)
* [Adding the Code Example to the IDE](#adding-the-code-example-to-the-ide)
* [MCU Code Examples](#mcu-code-examples)
* [ModusToolbox IDE](#modustoolbox-software)
* [Kits](#kits) 
* [Technical Resources](#technical-resources)
## Overview

These examples demonstrate the peripherals and basic functionality of the PSoC<sup>®</sup> 6 MCU. Some examples demonstrate more complex solutions. Each example has a readme.txt, and a comprehensive PDF document that explains what to do and what to observe when running the example.

The code examples in this repository all use [ModusToolbox IDE](http://www.cypress.com/ModusToolbox) and the C language.

The specific supported kits and parts vary per example. Review the readme.txt or the PDF file for each example. Supported kits may include [PSoC 6 BLE Pioneer Kit](http://www.cypress.com/cy8ckit-062-ble), [PSoC 6 WiFi-BT Pioneer Kit](http://www.cypress.com/CY8CKIT-062-WiFi-BT), and [PSoC 6 Wi-Fi BT Prototyping Kit](http://www.cypress.com/cy8cproto-062-4343w). Most examples support all [PSoC 6 MCU](http://www.cypress.com/PSoC6) parts.

You can browse the code examples in the repository, or use GitHub's search capabilities to find a particular symbol or term. Just type the term into the search box at the top of this page and search in this repository.

![](/images/SearchGitHub.png)

## Adding the Code Example to the IDE

Download and unzip this repository onto your local machine. You can also clone the repository into a location on your local machine. This puts all the code examples on your local machine.

In the ModusToolbox IDE, click the **New Application** link in the Quick Panel (or, use **File > New > ModusToolbox IDE Application**). Pick your kit or device. You must use a kit or device supported by the code example.

In the **Starter Application** window, click the **Browse** button and navigate to the *modus.mk* file for the example. Click **Next** and complete the application creation process.

## MCU Code Examples

| Code Example | Description |
| ----- | ----- |
|CE221773_PSoC6_HelloWorld | Demonstrates UART communication and blinks an LED using a TCPWM resource|
|CE219521_PSoC_6_MCU_GPIO_Interrupt| Demonstrates how to configure a GPIO to generate an interrupt.|
|CE218472_LPComp_CompareVrefExt| Demonstrates the voltage comparison functionality using the LPComp resource.|
|CE218129_LPComp_WakeupFromHibernate| Demonstrates wakeup from the Hibernate mode using the LPComp resource.|
|CE220060_WatchdogTimer| Demonstrates the two use cases of WDT – as a watchdog that causes a device reset in the case of a malfunction, and as a periodic interrupt source.|
|CE220498_PSoC6_MCWDT| Demonstrates the Multi-Counter Watchdog Timer (MCWDT) in free-running mode. The MCWDT measures the time between two successive switch press events and the result is displayed on the UART terminal.|
|CE218136_EINK_CapSense_RTOS | Shows how to create a user-interface solution using an E-INK display, CapSense®, and FreeRTOS |
|CE212736_PSoC6_BLE_FindMe| Introduces BLE software development and implements the BLE Find me profile|
|CE220291_PSoC6_TCPWM_Square_Wave| Generates a square wave using the TCPWM configured as a PWM. An LED connected to the PWM output pin blinks at 2 Hz.|
|CE213903_PSoC6DfuBasic| Four applications demonstrate device firmware update. This includes downloading an application from a host, installing it in device flash, and transferring control to that application.|
|CE219490_Ramping_LED_Using_SmartIO| Uses a PWM and Smart I/O to implement a ramping LED. There is no CPU usage except for the initialization of PWM and Smart I/O.|
|CE220465_CryptoAESDemo| Encrypts and decrypts user input data using the AES algorithm, using a 128-bit long key. The encrypted and decrypted data are displayed on a UART terminal emulator.|
|CE219656_PSoC6_UARTLowLevelAPIs| Three applications demonstrate the UART transmit and receive operation using low-level polling, DMA, or an interrupt.|
|CE220818_PSoC6_I2CMaster| Demonstrates the basic operation of the I2C master resource, which sends a command packet to the I2C slave SCB to control an user LED. Three applications developed in this example are: I2C master using high-level APIs, I2C master using low-level APIs and I2C master communication with EzI2C slave.|
|CE221119_PSoC6_MCU_SCB_I2C_Slave| Two applications demonstrate the I2C slave mode, using either polling or a callback routine.|
|CE221120_PSoC6_SPIMaster| Four applications demonstrate SPI communication between master and slave, using high-level API, low-level API, polling, and an interrupt.|
|CE220823_SMIFMemWriteAndRead| Demonstrates read/write operation to external memory by using Serial memory interface(SMIF) in Quad Serial peripheral interface (QSPI) mode.|



# ModusToolbox Software
![](/images/MTbanner.png)

ModusToolbox software is a complete Software Development Kit for PSoC 6 MCU. ModusToolbox IDE is installed as part of ModusToolbox software. The IDE is based on the industry-standard Eclipse IDE, and provides a single, coherent, and familiar design experience for the lowest power, most flexible MCUs with best-in-class sensing.

* [ModusToolbox Software](http://www.cypress.com/ModusToolbox)

Join the discussion at the [ModusToolbox Forum](https://community.cypress.com/community/modustoolbox/overview) on the Cypress Developer Community.

# Kits
Cypress provides low-cost development kits for the PSoC 6 MCU platform.

[PSoC 6 BLE Pioneer Kit](http://www.cypress.com/cy8ckit-062-ble): The PSoC 6 BLE Pioneer Kit  enables design and debug of the PSoC 63 Line. The kit includes an E-Ink display, and Cypress' industry-leading CapSense® technology.

[PSoC 6 WiFi-BT Pioneer Kit](http://www.cypress.com/CY8CKIT-062-WiFi-BT): The PSoC 6 WiFi-BT Pioneer Kit enables design and debug of the PSoC 62 MCU and the Murata LBEE5KL1DX Module (CYW4343W WiFi + Bluetooth Combo Chip). It includes a TFT display and Cypress' industry-leading CapSense technology.

[PSoC 6 Wi-Fi BT Prototyping Kit](http://www.cypress.com/cy8cproto-062-4343w) enables design and debug of PSoC 6 MCUs. It comes with industry-leading CapSense technology for touch buttons and slider, on-board debugger/programmer with KitProg3, μSD card interface, 512-Mb Quad-SPI NOR flash, PDM-PCM microphone, and a thermistor. It also includes a Murata LBEE5KL1DX module, based on the CYW4343W combo device.

# Technical Resources

Cypress provides a wealth of data at [www.cypress.com](http://www.cypress.com/) to help you select the right PSoC device and effectively integrate it into your design. Visit our [PSoC 6 MCU](http://www.cypress.com/psoc6) webpage to explore more about PSoC 6 MCU family of device.

For a comprehensive list of PSoC 6 MCU resources, see [KBA223067](https://community.cypress.com/docs/DOC-14644) in the Cypress community.

#### [PSoC 6 MCU Datasheets](http://www.cypress.com/psoc6ds)
Device datasheets list the features and electrical specifications of PSoC 6 families of devices.

#### [PSoC 6 MCU Application Notes](http://www.cypress.com/psoc6an)
Application notes are available on the Cypress website to assist you with designing your PSoC application.

#### [PSoC 6 MCU Technical Reference Manuals](http://www.cypress.com/psoc6trm)
The TRM provides detailed descriptions of the internal architecture of PSoC 6 devices.

### Cypress Developer Community ##

Need support for your design and development questions? Check out the [ModusToolbox Forum](https://community.cypress.com/community/modustoolbox/overview), or the [PSoC 6 forum](https://community.cypress.com/community/psoc-6) on the [Cypress Developer Community 3.0](https://community.cypress.com/welcome). Interact with technical experts in the embedded design community and receive answers verified by Cypress' very best applications engineers. You'll also have access to robust technical documentation, active conversation threads, and rich multimedia content.

[Community Forums](https://community.cypress.com/welcome) | [Videos](http://www.cypress.com/video-library) | [Blogs](http://www.cypress.com/blog) | [Training](http://www.cypress.com/training)

