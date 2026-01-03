cmake_minimum_required(VERSION 3.22)

add_library(GOWIN_M1 INTERFACE)

target_sources(GOWIN_M1 INTERFACE
    CMSIS/Device/system_GOWIN_M1.c
    drivers/src/GOWIN_M1_dap.c
    drivers/src/GOWIN_M1_axisuart.c
    drivers/src/GOWIN_M1_can.c
    drivers/src/GOWIN_M1_ddr3.c
    drivers/src/GOWIN_M1_dualtimer.c
    drivers/src/GOWIN_M1_ethernet.c
    drivers/src/GOWIN_M1_gpio.c
    drivers/src/GOWIN_M1_i2c.c
    drivers/src/GOWIN_M1_misc.c
    drivers/src/GOWIN_M1_psram.c
    drivers/src/GOWIN_M1_qspi_flash.c
    drivers/src/GOWIN_M1_rtc.c
    drivers/src/GOWIN_M1_sdcard.c
    drivers/src/GOWIN_M1_spi_flash.c
    drivers/src/GOWIN_M1_spi.c
    drivers/src/GOWIN_M1_timer.c
    drivers/src/GOWIN_M1_trng.c
    drivers/src/GOWIN_M1_uart.c
    drivers/src/GOWIN_M1_wdog.c
    drivers/src/GOWIN_M1_usbd.c
)

target_include_directories(GOWIN_M1 INTERFACE 
    CMSIS_6/CMSIS/Core/Include
    CMSIS/Device
    drivers/inc
)

