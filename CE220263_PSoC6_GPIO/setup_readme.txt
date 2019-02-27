Basic configuration for 150 MHz PSoC 6 devices.

CLOCKS

Platform clocks are set for high performance (144 MHz CLK_FAST for CM4 core
and 72 MHz CLK_SLOW for CM0+) but with a peripheral-friendly CLK_PERI
frequency (72 MHz).

FLL (Frequency-Locked Loop)       Disabled
PLL (Phase-Locked Loop)           Enabled
  Source:                         IMO (8 MHz)
  Frequency:                      144 MHz
CLK_HF0
  Source:                         CLK_PATH1 / PLL (144 MHz)
CLK_FAST (CM4 clock)
  Source:                         CLK_HF0 (144 MHz)
CLK_PERI (peripheral clock):
  Source:                         CLK_HF0 (144 MHz)
  Divider:                        2
  Frequency:                      72 MHz
CLK_SLOW (CM0+ clock):
  Source:                         CLK_PERI (72 MHz)
