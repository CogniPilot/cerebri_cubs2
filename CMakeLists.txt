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

set(CUBS2_RUMOCA_VERSION "v0.9.11"
  CACHE STRING "Pinned Rumoca version used for eFMI control code generation"
)
set(CUBS2_RUMOCA_INSTALL_SCRIPT_SHA256
  "ac15aa5ec90cb974387610fae7594d2704f2a3bd2e080cfe2517e896d30eb161"
  CACHE STRING "SHA256 of the pinned Rumoca binary install script"
)
set(CUBS2_RUMOCA_INSTALL_SCRIPT_URL
  "https://raw.githubusercontent.com/CogniPilot/rumoca/${CUBS2_RUMOCA_VERSION}/infra/install/install.sh"
  CACHE STRING "Pinned Rumoca binary install script URL"
)
set(CUBS2_RUMOCA_BIN_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/tools/rumoca/${CUBS2_RUMOCA_VERSION}
)
set(CUBS2_RUMOCA_EXECUTABLE ${CUBS2_RUMOCA_BIN_DIR}/rumoca)

add_custom_command(
  OUTPUT ${CUBS2_RUMOCA_EXECUTABLE}
  COMMAND ${CMAKE_COMMAND}
          -DCUBS2_RUMOCA_VERSION=${CUBS2_RUMOCA_VERSION}
          -DCUBS2_RUMOCA_BIN_DIR=${CUBS2_RUMOCA_BIN_DIR}
          -DCUBS2_RUMOCA_INSTALL_SCRIPT_URL=${CUBS2_RUMOCA_INSTALL_SCRIPT_URL}
          -DCUBS2_RUMOCA_INSTALL_SCRIPT_SHA256=${CUBS2_RUMOCA_INSTALL_SCRIPT_SHA256}
          -P ${CMAKE_CURRENT_LIST_DIR}/cmake/InstallRumoca.cmake
  COMMENT "Installing pinned Rumoca ${CUBS2_RUMOCA_VERSION}"
  VERBATIM
)

add_custom_target(cubs2_rumoca_tool
  DEPENDS ${CUBS2_RUMOCA_EXECUTABLE}
)

set(CUBS2_RUMOCA_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated/rumoca)
set(CUBS2_EFMI_CONTROL_STAMPS)
set(CUBS2_EFMI_CONTROL_SOURCES)
set(CUBS2_EFMI_CONTROL_INCLUDE_DIRS)

function(cubs2_add_efmi_control_model model_name)
  set(_model_file ${CMAKE_CURRENT_SOURCE_DIR}/modelica/${model_name}.mo)
  set(_efmu_dir ${CUBS2_RUMOCA_GENERATED_DIR}/${model_name})
  set(_efmu_file ${CUBS2_RUMOCA_GENERATED_DIR}/${model_name}.efmu)
  set(_stamp ${CUBS2_RUMOCA_GENERATED_DIR}/${model_name}.galec-production.stamp)
  set(_source ${_efmu_dir}/ProductionCode/${model_name}.c)
  set(_header ${_efmu_dir}/ProductionCode/${model_name}.h)

  add_custom_command(
    OUTPUT ${_stamp}
    BYPRODUCTS
      ${_efmu_file}
      ${_efmu_dir}/AlgorithmCode/${model_name}.alg
      ${_efmu_dir}/AlgorithmCode/manifest.xml
      ${_source}
      ${_header}
      ${_efmu_dir}/ProductionCode/manifest.xml
      ${_efmu_dir}/__content.xml
    COMMAND ${CMAKE_COMMAND} -E rm -rf
            ${_efmu_dir}
            ${_efmu_file}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CUBS2_RUMOCA_GENERATED_DIR}
    COMMAND ${CUBS2_RUMOCA_EXECUTABLE}
            --cache-dir ${CMAKE_CURRENT_BINARY_DIR}/.rumoca-cache
            compile ${_model_file}
            --model ${model_name}
            --target galec-production
            --output ${CUBS2_RUMOCA_GENERATED_DIR}
    COMMAND ${CMAKE_COMMAND} -E touch ${_stamp}
    DEPENDS
      ${_model_file}
      ${CUBS2_RUMOCA_EXECUTABLE}
    COMMENT "Generating ${model_name} eFMI Production Code with Rumoca ${CUBS2_RUMOCA_VERSION}"
    VERBATIM
  )

  list(APPEND CUBS2_EFMI_CONTROL_STAMPS ${_stamp})
  list(APPEND CUBS2_EFMI_CONTROL_SOURCES ${_source})
  list(APPEND CUBS2_EFMI_CONTROL_INCLUDE_DIRS ${_efmu_dir}/ProductionCode)
  set(CUBS2_EFMI_CONTROL_STAMPS ${CUBS2_EFMI_CONTROL_STAMPS} PARENT_SCOPE)
  set(CUBS2_EFMI_CONTROL_SOURCES ${CUBS2_EFMI_CONTROL_SOURCES} PARENT_SCOPE)
  set(CUBS2_EFMI_CONTROL_INCLUDE_DIRS ${CUBS2_EFMI_CONTROL_INCLUDE_DIRS} PARENT_SCOPE)
endfunction()

cubs2_add_efmi_control_model(FixedWingOuterLoop)

add_custom_target(cubs2_efmi_control_codegen
  DEPENDS ${CUBS2_EFMI_CONTROL_STAMPS}
)

add_dependencies(app cubs2_efmi_control_codegen)

set_source_files_properties(${CUBS2_EFMI_CONTROL_SOURCES}
  PROPERTIES GENERATED TRUE
)

target_sources(app PRIVATE
  ${CUBS2_EFMI_CONTROL_SOURCES}
)

target_include_directories(app PRIVATE
  ${CUBS2_EFMI_CONTROL_INCLUDE_DIRS}
)

if(TARGET ..__modules__lib__zenoh-pico__zephyr)
  target_compile_options(..__modules__lib__zenoh-pico__zephyr PRIVATE
    -Wno-unused-function
    -Wno-unused-variable
  )
endif()

add_subdirectory(src)
