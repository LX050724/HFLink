
add_executable(HFLinkFlashAlgo
    FlashDev.c
    FlashPrg.c
)

target_include_directories(HFLinkFlashAlgo PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_options(HFLinkFlashAlgo PRIVATE -T "${CMAKE_CURRENT_LIST_DIR}/GOWIN_M1_FlashAlgo.ld")
target_link_directories(HFLinkFlashAlgo PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(HFLinkFlashAlgo PRIVATE GOWIN_M1 libSEGGER_OFL_Lib_CortexM_LE.a)

install(FILES JLinkDevices.xml
    DESTINATION ${JLINK_DEIVCE_DIR}/FPGA
)

install(
    TARGETS HFLinkFlashAlgo RUNTIME
    DESTINATION ${JLINK_DEIVCE_DIR}/FPGA
)