//Copyright (C)2014-2025 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//Tool Version: V1.9.12 (64-bit) 
//Created Time: 2025-12-12 01:41:01
create_clock -name clkout0 -period 16.667 -waveform {0 8.334} [get_ports {usb_clk}]
create_clock -name clkin -period 20 -waveform {0 10} [get_ports {clk_osc}]
create_clock -name exflash_clk -period 33.333 -waveform {0 16.666} [get_nets {Cortex_M1/FLASH_SPI_CLK_in}]
create_clock -name SWCLK -period 250 -waveform {0 125} [get_ports {swclk}]
set_clock_groups -exclusive -group [get_clocks {clkout0}] -group [get_clocks {SWCLK}]
