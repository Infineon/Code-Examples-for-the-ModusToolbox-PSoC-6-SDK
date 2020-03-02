# Code Examples for ModusToolbox v1

## Overview - Obsolete

This repo is maintained for legacy purposes only. These examples do not work with ModusToolbox version 2.x. See all current examples at [Code Examples for Modus Toolbox](https://github.com/cypresssemiconductorco/Code-Examples-for-ModusToolbox-Software).

------

These examples demonstrate the peripherals and basic functionality of the PSoC<sup>®</sup> 6 MCU. Some examples demonstrate more complex solutions.

The code examples in *this* branch all use [ModusToolbox IDE](http://www.cypress.com/ModusToolbox) v1.1 and the C language. These examples support ModusToolbox v1.1. They do not work with ModusToolbox v1.0.

If you are using ModusToolbox v1.0, switch to the **MT1.0** branch for compatible code examples.

Each example has a readme.txt, and a comprehensive PDF document that explains what to do and what to observe when running the example.

The specific supported kits and parts vary per example. Review the readme.txt or the PDF file for each example. Supported kits may include [PSoC 6 BLE Pioneer Kit](http://www.cypress.com/cy8ckit-062-ble), [PSoC 6 WiFi-BT Pioneer Kit](http://www.cypress.com/CY8CKIT-062-WiFi-BT), and [PSoC 6 Wi-Fi BT Prototyping Kit](http://www.cypress.com/cy8cproto-062-4343w). Most examples support all [PSoC 6 MCU](http://www.cypress.com/PSoC6) parts.

You can browse the code examples in the repository, or use GitHub's search capabilities to find a particular symbol or term. Type the term into the search box at the top of this page and search in this repository.

![](/images/SearchGitHub.png)

## Adding the Code Example to the IDE

Download and unzip this repository onto your local machine. You can also clone the repository into a location on your local machine. This puts all the code examples on your local machine.

In the ModusToolbox IDE, click the **New Application** link in the Quick Panel (or, use **File > New > ModusToolbox IDE Application**). Pick your kit (or a custom board). You must use a kit or device supported by the code example.

In the **Starter Application** window, click the **Import** button and navigate to the *modus.mk* file for the example. Click **Next** and complete the application creation process.

## MCU Code Examples

| Code Example                                                 | Description                                                  |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| CE195313_Emulated_EEPROM                                     | Demonstrates the basic operation of the emulated EEPROM middleware |
| CE221773_PSoC6_HelloWorld                                    | Demonstrates UART communication and blinks an LED using a TCPWM resource |
| CE212736_PSoC6_BLE_FindMe                                    | Introduces BLE software development and implements the BLE Find me profile |
| CE218136_EINK_CapSense_RTOS                                  | Shows how to create a user-interface solution using an E-INK display, CapSense®, and FreeRTOS |
| CE218541_PSoC6_FaultHandling                                 | Shows how to find a fault location using the PDL SysLib API and the Arm® exception handler. |
| CE220823_SMIFMemWriteAndRead                                 | Demonstrates read/write operation to external memory by using Serial memory interface(SMIF) in Quad Serial peripheral interface (QSPI) mode. |
| CE218552_DMA_UART_Mem                                        | Demonstrates a UART-to-memory buffer data transfer using DMA, with no CPU usage. |
| CE219521_PSoC_6_MCU_GPIO_Interrupt                           | Demonstrates how to configure a GPIO to generate an interrupt. |
| CE219490_Ramping_LED_Using_SmartIO                           | Uses a PWM and Smart I/O to implement a ramping LED. There is no CPU usage except for the initialization of PWM and Smart I/O. |
| CE218472_LPComp_CompareVrefExt                               | Demonstrates the voltage comparison functionality using the LPComp resource. |
| CE218129_LPComp_WakeupFromHibernate                          | Demonstrates wakeup from the Hibernate mode using the LPComp resource. |
| CE220291_PSoC6_TCPWM_Square_Wave                             | Generates a square wave using the TCPWM configured as a PWM. An LED connected to the PWM output pin blinks at 2 Hz. |
| CE220060_WatchdogTimer                                       | Demonstrates the two use cases of WDT – as a watchdog that causes a device reset in the case of a malfunction, and as a periodic interrupt source. |
| CE220498_PSoC6_MCWDT                                         | Demonstrates the Multi-Counter Watchdog Timer (MCWDT) in free-running mode. The MCWDT measures the time between two successive switch press events and the result is displayed on the UART terminal. |
| CE216825_PSoC6_RTC_Basics                                    | Demonstrates the real-time clock (RTC) in PSoC 6 MCU.        |
| CE218964_RTC_DailyAlarm                                      | Configures RTC registers for a daily alarm using the RTC driver API in the Peripheral Driver Library. |
| CE220465_CryptoAESDemo                                       | Encrypts and decrypts user input data using the AES algorithm, using a 128-bit long key. The encrypted and decrypted data are displayed on a UART terminal emulator. |
| CE220511_PSoC6_CryptoSHADemo                                 | Generate a unique hash value for an arbitrary message using Secure Hash Algorithm (SHA). |
| CE221295_PSoC6_CryptoTRNG                                    | Create a One-Time Password (OTP) using the True Random Number generation feature of PSoC 6 MCU cryptography block. |
| CE227032_PSoC6_MCU_<br />SMIF_OnTheFly_Encryption_MMIO_XIP   | This code example demonstrates on-the-fly encryption of data using Serial Memory Interface(SMIF) in MMIO and XIP modes. |
| CE219656_PSoC6_UARTLowLevelAPIs                              | Three applications demonstrate the UART transmit and receive operation using low-level polling, DMA, or an interrupt. |
| CE220541_PSoC6_MCU_SCB_EZI2C                                 | Implements an EZI2C Slave using the Serial Communication Block (SCB) resource. |
| CE220818_PSoC6_I2CMaster                                     | Demonstrates the basic operation of the I2C master resource, which sends a command packet to the I2C slave SCB to control an user LED. Three applications developed in this example are: I2C master using high-level APIs, I2C master using low-level APIs and I2C master communication with EzI2C slave. |
| CE221119_PSoC6_MCU_SCB_I2C_Slave                             | Two applications demonstrate the I2C slave mode, using either polling or a callback routine. |
| CE221120_PSoC6_SPIMaster                                     | Four applications demonstrate SPI communication between master and slave, using high-level API, low-level API, polling, and an interrupt. |
| CE223468<br />Interfacing_BMI160(I2C)FreeRTOS                | This code example demonstrates how to interface PSoC® 6 MCU with a BMI160 motion aensor over an I2C interface within a FreeRTOS task. |
| CE225909_PSoC6_BatteryLevel_RTOS                             | This code example implements a GATT server with BLE standard Battery Service and Device Information Service. Battery level is simulated in the firmware and its value changes continuously from 0 to 100 percent. |
| CE224285_PSoC_6MCU_ExternalFlashAccess_in_XIP_Mode_with_QSPI | This example demonstrates how to use QSPI in execute-in-place (XIP) mode with external flash memory on the PSoC® 6 MCU |
| CE222460_SPI_FRAM_ACCESS_WITH_PSOC6_SMIF                     | This code example demonstrates implementing the SPI host controller using PSoC 6 SMIF to access the SPI F-RAM. |
| CE222967_QSPI_FRAM_ACCESS_WITH_PSOC6_SMIF                    | This code example implements the QSPI F-RAM host controller using PSoC 6 Serial Memory Interface (SMIF) block and demonstrates accessing the Excelon Ultra QSPI F-RAM in SPI, DPI, and QPI modes. |

|      |      |
| ----- | ----- |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
|      |      |
