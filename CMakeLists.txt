cmake_minimum_required(VERSION 3.14)
project(agx_motor_sdk VERSION 1.1.0)

# pip setuptools passes -DAGX_MOTOR_SDK_PIP_BUILD=ON (must not use ament_cmake).
# colcon builds into build/<package_name>/ → auto-enable ament integration.
option(AGX_MOTOR_SDK_PIP_BUILD "Building via pip setuptools" OFF)
option(AGX_MOTOR_SDK_AMENT_BUILD "ROS 2 ament/colcon integration" OFF)
if(NOT AGX_MOTOR_SDK_PIP_BUILD AND NOT AGX_MOTOR_SDK_AMENT_BUILD)
  get_filename_component(_agx_build_dir_name "${CMAKE_BINARY_DIR}" NAME)
  if(_agx_build_dir_name STREQUAL "${PROJECT_NAME}")
    set(AGX_MOTOR_SDK_AMENT_BUILD ON CACHE BOOL "ROS 2 ament/colcon integration" FORCE)
  endif()
endif()

if(AGX_MOTOR_SDK_AMENT_BUILD)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
endif()

option(BUILD_TESTING "Build tests" OFF)
option(PYTHON_BINDING "Build python binding for colcon / ROS Python" OFF)
if(AGX_MOTOR_SDK_AMENT_BUILD)
  set(PYTHON_BINDING ON CACHE BOOL "Build python binding for colcon / ROS Python" FORCE)
endif()
option(AGX_MOTOR_SDK_FETCH_PROTOCOL
  "Download prebuilt protocol libraries at configure time"
  ON)

if(CATKIN_DEVEL_PREFIX)
  set(BUILD_WITHOUT_ROS OFF)
  find_package(catkin REQUIRED)
  catkin_package(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
  )
else()
  set(BUILD_WITHOUT_ROS ON)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose build type" FORCE)
endif()

find_package(Threads REQUIRED)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/fetch_protocol_libs.cmake)
agx_motor_sdk_fetch_protocol_libs()
agx_motor_sdk_stage_protocol_libs()

add_library(${PROJECT_NAME}
  src/can/can_port.cpp
  src/can/socket_can.cpp
  src/motor/protocol_loader.cpp
  src/motor/arm_driver.cpp
  src/motor/chassis_driver.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC Threads::Threads ${CMAKE_DL_LIBS})
target_compile_options(${PROJECT_NAME} PRIVATE -fPIC)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src
)

if(BUILD_WITHOUT_ROS)
  include(GNUInstallDirs)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

  add_subdirectory(sample)

  if(PYTHON_BINDING)
    add_subdirectory(bindings/python)
  endif()

  install(TARGETS ${PROJECT_NAME}
    EXPORT agx_motor_sdkTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

  install(DIRECTORY include/agx_motor_sdk
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

  file(GLOB _agx_install_protocol_libs
    "${AGX_MOTOR_SDK_PROTOCOL_SRC_DIR}/*.so"
    "${AGX_MOTOR_SDK_PROTOCOL_SRC_DIR}/*.dll"
    "${AGX_MOTOR_SDK_PROTOCOL_SRC_DIR}/*.dylib")
  if(_agx_install_protocol_libs)
    install(FILES ${_agx_install_protocol_libs}
      DESTINATION ${CMAKE_INSTALL_LIBDIR}/agx_motor_sdk/protocol)
  endif()

  include(CMakePackageConfigHelpers)
  write_basic_package_version_file(
    agx_motor_sdkConfigVersion.cmake
    VERSION "${PROJECT_VERSION}"
    COMPATIBILITY SameMajorVersion)

  install(EXPORT agx_motor_sdkTargets
    FILE agx_motor_sdkTargets.cmake
    DESTINATION lib/cmake/agx_motor_sdk)

  configure_file(cmake/agx_motor_sdkConfig.cmake.in
    agx_motor_sdkConfig.cmake @ONLY)
  install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/agx_motor_sdkConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/agx_motor_sdkConfigVersion.cmake"
    DESTINATION lib/cmake/agx_motor_sdk)

  if(AGX_MOTOR_SDK_AMENT_BUILD)
    if(NOT TARGET _native)
      message(FATAL_ERROR
        "ament build requires PYTHON_BINDING=ON and target _native")
    endif()
    ament_get_python_install_dir(_agx_python_install_dir)
    install(TARGETS _native
      DESTINATION "${_agx_python_install_dir}/agx_motor_sdk")
    ament_python_install_package(agx_motor_sdk
      PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/agx_motor_sdk")
    ament_package()
  endif()
else()
  install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  install(DIRECTORY include/agx_motor_sdk
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
endif()
