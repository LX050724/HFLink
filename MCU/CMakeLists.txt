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


# 删除自带配置文件，防止冲突
file(REMOVE FORCE Third_Party/EasyFlash/easyflash/inc/ef_cfg.h)
# 添加为接口库，APP可以重配置
add_library(EasyFlash INTERFACE)
target_sources(EasyFlash INTERFACE
    Third_Party/EasyFlash/easyflash/src/easyflash.c
    Third_Party/EasyFlash/easyflash/src/ef_env_legacy_wl.c
    Third_Party/EasyFlash/easyflash/src/ef_env_legacy.c
    Third_Party/EasyFlash/easyflash/src/ef_env.c
    Third_Party/EasyFlash/easyflash/src/ef_iap.c
    Third_Party/EasyFlash/easyflash/src/ef_log.c
    Third_Party/EasyFlash/easyflash/src/ef_utils.c
)

target_include_directories(EasyFlash INTERFACE Third_Party/EasyFlash/easyflash/inc)

add_subdirectory(Firmware)
add_subdirectory(Third_Party/SEGGER_RTT_V812)

add_subdirectory(Application)
# add_subdirectory(Bootloader)
# add_subdirectory(FlashAlgo)
