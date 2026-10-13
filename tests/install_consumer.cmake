function(checked)
  execute_process(COMMAND ${ARGV} RESULT_VARIABLE result)
  if(NOT "${result}" STREQUAL "0")
    message(FATAL_ERROR "Consumer command failed (${result}): ${ARGV}")
  endif()
endfunction()

file(REMOVE_RECURSE "${INSTALL_PREFIX}" "${FIXTURE_BINARY}")
checked("${CMAKE_COMMAND}" --install "${SDK_BUILD}" --prefix "${INSTALL_PREFIX}"
  --config "${CONFIG}")
checked("${CMAKE_COMMAND}" -S "${FIXTURE_SOURCE}" -B "${FIXTURE_BINARY}"
  "-Dagx_motor_sdk_DIR=${INSTALL_PREFIX}/lib/cmake/agx_motor_sdk")
checked("${CMAKE_COMMAND}" --build "${FIXTURE_BINARY}" --config "${CONFIG}")
checked("${CMAKE_CTEST_COMMAND}" --test-dir "${FIXTURE_BINARY}"
  -C "${CONFIG}" --output-on-failure)