if(AGX_MOTOR_SDK_PIP_BUILD)
  set(_agx_pybind11_ready FALSE)
else()
  find_package(pybind11 CONFIG QUIET)
  set(_agx_pybind11_ready ${pybind11_FOUND})
endif()

if(NOT _agx_pybind11_ready)
  include(FetchContent)
  FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v2.11.1
  )
  FetchContent_MakeAvailable(pybind11)
endif()

pybind11_add_module(_native
  module.cpp
)
target_link_libraries(_native PRIVATE agx_motor_sdk)
target_include_directories(_native PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include)

set(PYBIND_OUTPUT_LIBDIR ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR} CACHE PATH "")
if(PYBIND_OUTPUT_LIBDIR)
  set_target_properties(_native PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PYBIND_OUTPUT_LIBDIR})
endif()

# site-packages/agx_motor_sdk/_native.so -> install/lib/libagx_motor_sdk.so
set_target_properties(_native PROPERTIES
  INSTALL_RPATH "$ORIGIN/../../.."
  BUILD_WITH_INSTALL_RPATH FALSE)
