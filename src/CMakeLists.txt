target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_sources(app PRIVATE csyn_topics.c zros_topics.c)

if(CONFIG_CUBS2_REALTIME)
  target_sources(app PRIVATE main_realtime.c runtime_control.c)
else()
  target_sources(app PRIVATE
    main_sil.c
    native_sil_transport.c
  )
endif()

if(CONFIG_CODE_DATA_RELOCATION AND CONFIG_BOARD_MR_VMU_TROPIC_MIMXRT1064)
  zephyr_code_relocate(
    FILES
      main_realtime.c
      ${CUBS2_EFMI_CONTROL_SOURCES}
      ${ZEPHYR_CSYN_MODULE_DIR}/src/csyn_codec.c
      ${ZEPHYR_CSYN_MODULE_DIR}/src/csyn_zros_bridge.c
    LOCATION ITCM_TEXT_RODATA
  )
endif()
