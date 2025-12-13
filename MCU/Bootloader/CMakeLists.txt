
add_executable(HFLinkBootloader
    GOWIN_M1_it.c
    GOWIN_M1_APP_startup.s
    boot.c
)
target_include_directories(HFLinkBootloader PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_options(HFLinkBootloader PRIVATE -T "${CMAKE_CURRENT_LIST_DIR}/GOWIN_M1_BL.ld")
target_link_libraries(HFLinkBootloader PRIVATE GOWIN_M1)

add_custom_command(TARGET HFLinkBootloader POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:HFLinkBootloader> HFLinkBootloader.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:HFLinkBootloader> HFLinkBootloader.hex
    COMMENT "Building HFLinkBootloader.bin"
    )
    
add_custom_command(TARGET HFLinkBootloader POST_BUILD
    COMMAND python ${CMAKE_SOURCE_DIR}/tools/make_itcm.py HFLinkBootloader.bin -o .
    COMMENT "Building itcm"
)
