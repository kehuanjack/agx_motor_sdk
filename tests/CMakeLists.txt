add_executable(agx_sdk_config_test config_test.cpp)
target_link_libraries(agx_sdk_config_test PRIVATE agx_motor_sdk)
target_compile_definitions(agx_sdk_config_test PRIVATE
  AGX_SDK_EXAMPLE_YAML="${PROJECT_SOURCE_DIR}/examples/mechanism/robot_motors.yaml")
add_test(NAME software.config COMMAND agx_sdk_config_test)
set_tests_properties(software.config PROPERTIES LABELS software TIMEOUT 15)

set(AGX_MOTOR_SDK_PROTOCOL_DIR "${AGX_MOTOR_SDK_PROTOCOL_BUILD_DIR}" CACHE PATH
  "Local protocol directory for software lifecycle tests (no download)")
add_executable(agx_sdk_lifecycle_test lifecycle_test.cpp)
target_link_libraries(agx_sdk_lifecycle_test PRIVATE agx_motor_sdk)
target_compile_definitions(agx_sdk_lifecycle_test PRIVATE
  AGX_SDK_TEST_PROTOCOL_DIR="${AGX_MOTOR_SDK_PROTOCOL_DIR}")
add_test(NAME software.weak_driver COMMAND agx_sdk_lifecycle_test expired)
add_test(NAME software.protocol_lifecycle COMMAND agx_sdk_lifecycle_test protocol)
set_tests_properties(software.weak_driver software.protocol_lifecycle PROPERTIES
  LABELS software TIMEOUT 15)
set_tests_properties(software.protocol_lifecycle PROPERTIES SKIP_RETURN_CODE 77)

add_executable(agx_sdk_socket_callback_test socket_callback_test.cpp)
target_link_libraries(agx_sdk_socket_callback_test PRIVATE agx_motor_sdk)
add_test(NAME software.socket_callback COMMAND agx_sdk_socket_callback_test)
set_tests_properties(software.socket_callback PROPERTIES LABELS software TIMEOUT 15)

if(BUILD_WITHOUT_ROS)
  add_test(NAME software.install_consumer COMMAND ${CMAKE_COMMAND}
    -DSDK_BUILD=${PROJECT_BINARY_DIR}
    -DFIXTURE_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/consumer
    -DFIXTURE_BINARY=${CMAKE_CURRENT_BINARY_DIR}/consumer-build
    -DINSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/consumer-prefix
    -DCONFIG=$<CONFIG>
    -P ${CMAKE_CURRENT_SOURCE_DIR}/install_consumer.cmake)
  set_tests_properties(software.install_consumer PROPERTIES LABELS software TIMEOUT 120)
endif()