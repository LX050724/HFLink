add_library(SEGGER_RTT
    RTT/SEGGER_RTT.c
    RTT/SEGGER_RTT_printf.c
    Syscalls/SEGGER_RTT_Syscalls_GCC.c
)

target_include_directories(SEGGER_RTT PUBLIC RTT Config)
