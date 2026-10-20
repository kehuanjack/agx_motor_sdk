set(_agx_motor_sdk_samples
  arm_enable_demo
  arm_disable_demo
  arm_go_zero_demo
  arm_get_high_speed_demo
  arm_get_low_speed_demo
  arm_head_from_yaml_demo
)

foreach(_demo ${_agx_motor_sdk_samples})
  set(_target "agx_motor_sdk_${_demo}")
  add_executable(${_target} "${_demo}.cpp")
  target_link_libraries(${_target} PRIVATE agx_motor_sdk)
  target_include_directories(${_target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
endforeach()
