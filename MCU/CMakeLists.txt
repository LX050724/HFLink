cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(FPGA_M1 C CXX ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

include_directories(sources)


add_subdirectory(Firmware)
add_subdirectory(Third_Party/SEGGER_RTT_V812)

add_executable(${CMAKE_PROJECT_NAME} 
    startup.s
    sources/main.c
    sources/GOWIN_M1_it.c
    sources/usb/usbd.c
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE m SEGGER_RTT GOWIN_M1)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMENT "Building ${CMAKE_PROJECT_NAME}.bin"
)

# set(FPGA_FS_NAME ao_0.fs)
set(FPGA_FS_NAME hflink.fs)
set(FPGA_POSP_NAME hflink.posp)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND cmake -E copy_if_different ${CMAKE_CURRENT_LIST_DIR}/../FPGA/impl/pnr/${FPGA_FS_NAME} ${CMAKE_CURRENT_LIST_DIR}/../FPGA/impl/pnr/${FPGA_POSP_NAME} .
    COMMAND ${CMAKE_CURRENT_LIST_DIR}/tools/merge_bit.exe ${CMAKE_PROJECT_NAME}.bin ${FPGA_FS_NAME} 32 ${FPGA_POSP_NAME}
)

# target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE GD32F30X_HD)
