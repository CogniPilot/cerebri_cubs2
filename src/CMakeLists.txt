target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_sources(app PRIVATE
  main.c
)

if(CONFIG_CODE_DATA_RELOCATION AND CONFIG_BOARD_MR_VMU_TROPIC_MIMXRT1064)
  zephyr_code_relocate(
    FILES
      main.c
      ${CUBS2_EFMI_CONTROL_SOURCES}
      ${ZEPHYR_CSYN_MODULE_DIR}/src/csyn_codec.c
      ${ZEPHYR_CSYN_MODULE_DIR}/src/csyn_zros_bridge.c
    LOCATION ITCM_TEXT_RODATA
  )
endif()
