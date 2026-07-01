# Fetch prebuilt arm/chassis protocol libraries (shared by cmake and pip).

set(AGX_MOTOR_SDK_PROTOCOL_SRC_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/agx_motor_sdk/protocol")
set(AGX_MOTOR_SDK_PROTOCOL_BUILD_DIR
  "${CMAKE_BINARY_DIR}/protocol")

function(agx_motor_sdk_fetch_protocol_libs)
  if(NOT AGX_MOTOR_SDK_FETCH_PROTOCOL)
    message(STATUS "AGX_MOTOR_SDK_FETCH_PROTOCOL=OFF, skip protocol fetch")
    return()
  endif()

  if(DEFINED ENV{AGX_MOTOR_SDK_SKIP_DOWNLOAD}
     AND NOT "$ENV{AGX_MOTOR_SDK_SKIP_DOWNLOAD}" STREQUAL "")
    message(STATUS "AGX_MOTOR_SDK_SKIP_DOWNLOAD set, skip protocol fetch")
    return()
  endif()

  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  message(STATUS "Fetching protocol libraries via protocol_fetch.py ...")
  execute_process(
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/protocol_fetch.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE _agx_fetch_rc
    OUTPUT_VARIABLE _agx_fetch_out
    ERROR_VARIABLE _agx_fetch_err
  )
  if(_agx_fetch_out)
    string(STRIP "${_agx_fetch_out}" _agx_fetch_out)
    if(_agx_fetch_out)
      message(STATUS "${_agx_fetch_out}")
    endif()
  endif()
  if(NOT _agx_fetch_rc EQUAL 0)
    file(GLOB _agx_existing_protocol
      "${AGX_MOTOR_SDK_PROTOCOL_SRC_DIR}/*.so"
      "${AGX_MOTOR_SDK_PROTOCOL_SRC_DIR}/*.dll"
      "${AGX_MOTOR_SDK_PROTOCOL_SRC_DIR}/*.dylib")
    if(_agx_existing_protocol)
      message(WARNING
        "Protocol fetch failed; using existing libraries in "
        "${AGX_MOTOR_SDK_PROTOCOL_SRC_DIR}.\n${_agx_fetch_err}")
    else()
      message(WARNING
        "Protocol fetch failed and no local libraries found.\n"
        "${_agx_fetch_err}\n"
        "SDK will still build; set AGX_MOTOR_SDK_ARM_LIB / "
        "AGX_MOTOR_SDK_CHASSIS_LIB at runtime, or place libraries under "
        "${AGX_MOTOR_SDK_PROTOCOL_SRC_DIR} and re-run cmake.")
    endif()
  endif()
endfunction()

function(agx_motor_sdk_stage_protocol_libs)
  file(MAKE_DIRECTORY "${AGX_MOTOR_SDK_PROTOCOL_BUILD_DIR}")
  file(GLOB _agx_protocol_libs
    "${AGX_MOTOR_SDK_PROTOCOL_SRC_DIR}/*.so"
    "${AGX_MOTOR_SDK_PROTOCOL_SRC_DIR}/*.dll"
    "${AGX_MOTOR_SDK_PROTOCOL_SRC_DIR}/*.dylib")
  if(NOT _agx_protocol_libs)
    message(WARNING
      "No protocol libraries under ${AGX_MOTOR_SDK_PROTOCOL_SRC_DIR}. "
      "Runtime dlopen may fail unless AGX_MOTOR_SDK_ARM_LIB / "
      "AGX_MOTOR_SDK_CHASSIS_LIB are set.")
    return()
  endif()
  foreach(_agx_lib IN LISTS _agx_protocol_libs)
    get_filename_component(_agx_name "${_agx_lib}" NAME)
    configure_file("${_agx_lib}"
      "${AGX_MOTOR_SDK_PROTOCOL_BUILD_DIR}/${_agx_name}" COPYONLY)
  endforeach()
  message(STATUS "Staged protocol libraries in ${AGX_MOTOR_SDK_PROTOCOL_BUILD_DIR}")
endfunction()
