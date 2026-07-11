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

if(TARGET native_simulator AND NOT CONFIG_CUBS2_REALTIME)
  target_sources(native_simulator INTERFACE src/native_sil_host.c)
endif()

if(CONFIG_BOARD_MR_VMU_TROPIC_MIMXRT1064 AND CMAKE_C_COMPILER_ID STREQUAL "GNU"
   AND "${GNULD_VERSION_STRING}" VERSION_GREATER_EQUAL 2.39)
  # Zephyr's bare-metal GNU ld link uses -N, which creates a RWX load segment
  # in the ELF. This image is not a hosted executable, so silence ld.bfd's
  # generic RWX segment diagnostic for this firmware target. The flag itself
  # needs binutils >= 2.39 (SDK 0.17.3 ships 2.38, which errors on it).
  zephyr_link_libraries(${LINKERFLAGPREFIX},--no-warn-rwx-segments)
endif()

if(TARGET native_simulator AND CMAKE_C_COMPILER_ID STREQUAL "GNU" AND NOT CONFIG_64BIT)
  # Nix's multilib GCC wrapper searches both 32-bit and 64-bit library paths
  # while native_sim links with -m32. Tell ld.bfd not to warn when skipping the
  # incompatible 64-bit candidates it encounters before the matching libraries.
  target_link_options(native_simulator INTERFACE -Wl,--no-warn-search-mismatch)
endif()

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(CUBS2_RUMOCA_PYTHON "" CACHE FILEPATH
  "Python interpreter with the Rumoca binding used for Modelica code generation"
)

if(NOT CUBS2_RUMOCA_PYTHON
   AND DEFINED ENV{CUBS2_RUMOCA_PYTHON}
   AND NOT "$ENV{CUBS2_RUMOCA_PYTHON}" STREQUAL "")
  set(CUBS2_RUMOCA_PYTHON "$ENV{CUBS2_RUMOCA_PYTHON}" CACHE FILEPATH
    "Python interpreter with the Rumoca binding used for Modelica code generation"
    FORCE
  )
endif()

if(NOT CUBS2_RUMOCA_PYTHON)
  set(CUBS2_RUMOCA_PYTHON "${Python3_EXECUTABLE}" CACHE FILEPATH
    "Python interpreter with the Rumoca binding used for Modelica code generation"
    FORCE
  )
endif()

set(CUBS2_RUMOCA_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated/rumoca)
set(CUBS2_EFMI_CONTROL_STAMPS)
set(CUBS2_EFMI_CONTROL_SOURCES)
set(CUBS2_EFMI_CONTROL_INCLUDE_DIRS)

function(cubs2_add_efmi_control_model model_name)
  set(_model_file ${CMAKE_CURRENT_SOURCE_DIR}/src/${model_name}.mo)
  set(_efmu_dir ${CUBS2_RUMOCA_GENERATED_DIR}/${model_name})
  set(_stamp ${CUBS2_RUMOCA_GENERATED_DIR}/${model_name}.galec-production.stamp)
  set(_source ${_efmu_dir}/ProductionCode/${model_name}.c)
  set(_header ${_efmu_dir}/ProductionCode/${model_name}.h)

  add_custom_command(
    OUTPUT ${_stamp}
    BYPRODUCTS
      ${_efmu_dir}/AlgorithmCode/${model_name}.alg
      ${_efmu_dir}/AlgorithmCode/manifest.xml
      ${_source}
      ${_header}
      ${_efmu_dir}/ProductionCode/manifest.xml
      ${_efmu_dir}/__content.xml
    COMMAND ${CMAKE_COMMAND} -E rm -rf "${_efmu_dir}"
    COMMAND "${CUBS2_RUMOCA_PYTHON}"
            "-c"
            "from pathlib import Path; import sys; import rumoca as rum; model_file, model_name, target, output = sys.argv[1:5]; Path(output).mkdir(parents=True, exist_ok=True); generated = rum.Session().codegen_file(model_file, model_name, target, output); [print(f'wrote {path}') for path in generated]"
            "${_model_file}"
            "${model_name}"
            "galec-production"
            "${_efmu_dir}"
    COMMAND ${CMAKE_COMMAND} -E touch "${_stamp}"
    DEPENDS
      ${_model_file}
    COMMENT "Generating ${model_name} eFMI Production Code with Rumoca Python binding"
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

if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
  set_source_files_properties(${CUBS2_EFMI_CONTROL_SOURCES}
    PROPERTIES COMPILE_OPTIONS -fno-var-tracking-assignments
  )
endif()

target_sources(app PRIVATE
  ${CUBS2_EFMI_CONTROL_SOURCES}
  ${synapse_fbs_c_SOURCE_DIR}/src/flatcc-runtime/builder.c
  ${synapse_fbs_c_SOURCE_DIR}/src/flatcc-runtime/emitter.c
  ${synapse_fbs_c_SOURCE_DIR}/src/flatcc-runtime/refmap.c
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
