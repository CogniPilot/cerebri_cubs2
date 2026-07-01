cmake_minimum_required(VERSION 3.20.0)

set(CUBS2_WORKSPACE_ROOT "" CACHE PATH "West workspace root containing Zephyr and module checkouts")
if(NOT CUBS2_WORKSPACE_ROOT AND DEFINED ENV{CUBS2_WORKSPACE_ROOT})
  set(CUBS2_WORKSPACE_ROOT "$ENV{CUBS2_WORKSPACE_ROOT}")
endif()

if(CUBS2_WORKSPACE_ROOT)
  get_filename_component(WORKSPACE_ROOT ${CUBS2_WORKSPACE_ROOT} ABSOLUTE)
else()
  get_filename_component(WORKSPACE_ROOT ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
endif()

set(ZEPHYR_MODULES
  ${WORKSPACE_ROOT}/modules/hal/cmsis
  ${WORKSPACE_ROOT}/modules/hal/cmsis_6
  ${WORKSPACE_ROOT}/modules/hal/nxp
  ${WORKSPACE_ROOT}/modules/lib/zenoh-pico
  ${WORKSPACE_ROOT}/modules/lib/zros
  ${WORKSPACE_ROOT}/modules/lib/csyn
  ${WORKSPACE_ROOT}/modules/lib/zephyr_boards
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cubs2 LANGUAGES C)

if(TARGET ..__modules__lib__zenoh-pico__zephyr)
  target_compile_options(..__modules__lib__zenoh-pico__zephyr PRIVATE
    -Wno-unused-function
    -Wno-unused-variable
  )
endif()

add_subdirectory(src)
