cmake_minimum_required(VERSION 3.22)

if (WIN32)
    set(JLINK_DEIVCE_DIR $ENV{APPDATA}/Segger/JLinkDevices CACHE STRING "JlinkDevices目录")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

project(HFLink C CXX ASM)

add_subdirectory(Firmware)
add_subdirectory(Third_Party/SEGGER_RTT_V812)

add_subdirectory(Application)
# add_subdirectory(Bootloader)
# add_subdirectory(FlashAlgo)
