cmake_minimum_required(VERSION 3.20)

foreach(_var
    CUBS2_RUMOCA_EXECUTABLE
    CUBS2_RUMOCA_CACHE_DIR
    CUBS2_RUMOCA_MODEL_FILE
    CUBS2_RUMOCA_MODEL_NAME
    CUBS2_RUMOCA_TARGET
    CUBS2_RUMOCA_OUTPUT_DIR
    CUBS2_RUMOCA_EFMU_DIR
    CUBS2_RUMOCA_EFMU_FILE
    CUBS2_RUMOCA_STAMP)
  if(NOT DEFINED ${_var} OR "${${_var}}" STREQUAL "")
    message(FATAL_ERROR "${_var} must be set")
  endif()
endforeach()

file(REMOVE_RECURSE
  "${CUBS2_RUMOCA_EFMU_DIR}"
  "${CUBS2_RUMOCA_EFMU_FILE}"
)
file(MAKE_DIRECTORY "${CUBS2_RUMOCA_OUTPUT_DIR}")

set(_rumoca_command
  "${CUBS2_RUMOCA_EXECUTABLE}"
  --cache-dir "${CUBS2_RUMOCA_CACHE_DIR}"
  compile "${CUBS2_RUMOCA_MODEL_FILE}"
  --model "${CUBS2_RUMOCA_MODEL_NAME}"
  --target "${CUBS2_RUMOCA_TARGET}"
  --output "${CUBS2_RUMOCA_OUTPUT_DIR}"
)

execute_process(
  COMMAND ${_rumoca_command}
  RESULT_VARIABLE _rumoca_result
  OUTPUT_VARIABLE _rumoca_output
  ERROR_VARIABLE _rumoca_error
)

if(NOT _rumoca_result EQUAL 0)
  string(JOIN " " _rumoca_command_display ${_rumoca_command})
  message(FATAL_ERROR
    "Rumoca compile failed for ${CUBS2_RUMOCA_MODEL_NAME} "
    "using target ${CUBS2_RUMOCA_TARGET}\n"
    "command:\n${_rumoca_command_display}\n"
    "stdout:\n${_rumoca_output}\n"
    "stderr:\n${_rumoca_error}"
  )
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" -E touch "${CUBS2_RUMOCA_STAMP}"
  RESULT_VARIABLE _touch_result
  ERROR_VARIABLE _touch_error
)
if(NOT _touch_result EQUAL 0)
  message(FATAL_ERROR "Could not write ${CUBS2_RUMOCA_STAMP}: ${_touch_error}")
endif()
