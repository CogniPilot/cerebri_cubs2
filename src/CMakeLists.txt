target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR})

if(CONFIG_CUBS2_REALTIME)
  set(CUBS2_MAIN_SOURCE main_realtime.c)
else()
  set(CUBS2_MAIN_SOURCE main_sil.c)
endif()

target_sources(app PRIVATE
  ${CUBS2_MAIN_SOURCE}
  csyn_topics.c
  runtime_control.c
  zros_topics.c
)

target_sources_ifdef(CONFIG_CUBS2_FASTDYN app PRIVATE fastdyn_board.c)

if(CONFIG_CUBS2_FASTDYN)
  # app/libapp.a is linked before the board and SDK archives, so the FastDyn
  # implementations below replace hardware-only weak/board hooks.
  zephyr_link_libraries(-Wl,--allow-multiple-definition)
endif()

if(CONFIG_CODE_DATA_RELOCATION AND CONFIG_BOARD_MR_VMU_TROPIC_MIMXRT1064)
  zephyr_code_relocate(
    FILES
      ${CUBS2_MAIN_SOURCE}
      runtime_control.c
      ${CUBS2_EFMI_CONTROL_SOURCES}
      ${ZEPHYR_CSYN_MODULE_DIR}/src/csyn_codec.c
      ${ZEPHYR_CSYN_MODULE_DIR}/src/csyn_zros_bridge.c
    LOCATION ITCM_TEXT_RODATA
  )
endif()
