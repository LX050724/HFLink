
add_executable(HFLinkAPP
    GOWIN_M1_APP_startup.s
    main.c
    GOWIN_M1_it.c
    usb/usbd.c
)
target_include_directories(HFLinkAPP PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_options(HFLinkAPP PRIVATE -T "${CMAKE_CURRENT_LIST_DIR}/GOWIN_M1_APP.ld")
target_link_libraries(HFLinkAPP PRIVATE m SEGGER_RTT GOWIN_M1)

add_custom_command(TARGET HFLinkAPP POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:HFLinkAPP> HFLinkAPP.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:HFLinkAPP> HFLinkAPP.hex
    COMMENT "Building HFLinkAPP.bin"
)
    
