target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_sources(app PRIVATE
  main.c
  generated_fixed_wing/CubControl_FixedWingOuterLoop.c
)

if(CONFIG_CODE_DATA_RELOCATION AND CONFIG_BOARD_MR_VMU_TROPIC_MIMXRT1064)
  zephyr_code_relocate(
    FILES
      main.c
      generated_fixed_wing/CubControl_FixedWingOuterLoop.c
      ${ZEPHYR_CSYN_MODULE_DIR}/src/csyn_codec.c
      ${ZEPHYR_CSYN_MODULE_DIR}/src/csyn_zros_bridge.c
    LOCATION ITCM_TEXT_RODATA
  )
endif()
