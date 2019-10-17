This project shows how to use QSPI (SMIF) in execute-in-place (XIP) mode
to access functions and variables place in external memory. The QSPI
interface is configured to run at 50 MHz and a UART is provided to enable
visual feedback of the QSPI operation.

The following is a summary of the PSoC configuration in this project. To modify
the settings, click "Configure device" in the Quick Panel or double-click on the
"design.modus" file in the _config project.

CLOCKS

Platform clocks are set for high performance (144 MHz CLK_FAST for CM4 core
and 72 MHz CLK_SLOW for CM0+) but with a peripheral-friendly CLK_PERI
frequency (72 MHz).

FLL (Frequency-Locked Loop)       Enabled
  Source:						  IMO (8 MHz)
  Frequency:					  100 MHz
PLL (Phase-Locked Loop)           Enabled
  Source:                         IMO (8 MHz)
  Frequency:                      144 MHz
CLK_HF0 (hf_clock SMIF)
  Source:                         CLK_PATH1 / PLL (144 MHz)
CLK_HF2 (SMIF interface clock)
  Source:						  CLK_PATH0 / FLL (100 MHz)
  Frequency:					  50 MHz  
CLK_FAST (CM4 clock)
  Source:                         CLK_HF0 (144 MHz)
CLK_PERI (peripheral clock):
  Source:                         CLK_HF0 (144 MHz)
  Divider:                        2
  Frequency:                      72 MHz
CLK_SLOW (CM0+ clock):
  Source:                         CLK_PERI (72 MHz)
  
  
Quad Serial Memory Interface (QSPI)

The QSPI block is enabled and named SMIF (Serial Memory Interface)

  Alias:						SMIF
  
  Clocks  
    HF Clock:					CLK_HF0 root_clk  
    Interface Clock:			CLK_HF2 root_clk
    SPI Clock:			    	P11[7] digital_inout
  
  Data
  	SPI Data[0]:				P11[6] digital_out
  	SPI Data[1]:				P11[5] digital_out
  	SPI Data[2]:				P11[4] digital_out
  	SPI Data[3]:				P11[3] digital_out
  	
  Slave Select
  	SPI Slave Select 0:			P11[2] digital_out
  	
  	
UART Bridge

  Peripheral:                     SCB5
  Personality:					  UART-1.0
  Alias:                          UART
  Baud Rate (bps):                115200 (actual 115384)
  Clock:                          8 bit Divider 0 clk (923.1kHz)
    Divider:                      78
  TX:                             P5[1]
    Drive Mode:                   Strong Drive. Input buffer off
    Internal Connection:          Digital InOut
  