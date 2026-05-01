//Copyright (C)2014-2026 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//Tool Version: V1.9.12 (64-bit) 
//Created Time: 2026-04-19 17:30:36
create_clock -name SWCLK -period 250 -waveform {0 125} [get_ports {MCU_SWCLK}]
create_clock -name CLKIN -period 20 -waveform {0 10} [get_ports {clk_osc}]
create_generated_clock -name SYSCLK -source [get_ports {clk_osc}] -master_clock CLKIN -divide_by 5 -multiply_by 6 -duty_cycle 50 [get_nets {clkout0}]
set_clock_groups -exclusive -group [get_clocks {CLKIN}] -group [get_clocks {SWCLK}]
