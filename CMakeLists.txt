cmake_minimum_required(VERSION 3.0.0)
project(librobcomm
    DESCRIPTION "Library for communication with RobCo robots."
    VERSION 0.0.0 # default, can be overridden by CI pipeline
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 98)  # ROS Melodic uses C++ 03, which is not supported as a value
                      # but C++ 98 is equivalent.

set(CMAKE_INSTALL_DEFAULT_COMPONENT_NAME runtime)

set(ROBCOMM_INCLUDE_INSTALL_DIR include/robcomm)
set(ROBCOMM_LIB_INSTALL_DIR lib)

file(GLOB ROBCOMM_SOURCES src/*.cpp)
file(GLOB ROBCOMM_HEADERS include/robcomm/*.hpp)

add_library(robcomm SHARED ${ROBCOMM_SOURCES})

if(BUILD_EXAMPLES)
  add_subdirectory(examples/basic_example)
endif()

target_include_directories(robcomm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include>)

message(STATUS "LIB_DIR: ${ROBCOMM_LIB_INSTALL_DIR}")

install(TARGETS robcomm EXPORT robcommConfig DESTINATION ${ROBCOMM_LIB_INSTALL_DIR})
install(EXPORT robcommConfig DESTINATION share/robcomm/cmake)
include(CMakePackageConfigHelpers)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  MyLibConfigVersion.cmake
  VERSION ${PACKAGE_VERSION}
  COMPATIBILITY AnyNewerVersion
)

install(FILES
  ${ROBCOMM_HEADERS}
  DESTINATION ${ROBCOMM_INCLUDE_INSTALL_DIR}
  COMPONENT devel)

# NOTE: Library has been split into 'runtime' and 'devel' components.
# 'runtime' contains shared library (.so) while 'devel' contains headers.
SET(CPACK_GENERATOR "DEB")
SET(CPACK_DEB_COMPONENT_INSTALL ON)
SET(CPACK_DEBIAN_RUNTIME_PACKAGE_NAME "librobcomm")
SET(CPACK_DEBIAN_DEVEL_PACKAGE_NAME "librobcomm-devel")
SET(CPACK_PACKAGE_VERSION "${CMAKE_PROJECT_VERSION}")
SET(CPACK_DEBIAN_RUNTIME_PACKAGE_SECTION "embedded")
SET(CPACK_DEBIAN_DEVEL_PACKAGE_SECTION "devel")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "RobCo GmbH")
SET(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
SET(CPACK_DEBIAN_RUNTIME_PACKAGE_DEPENDS "")
SET(CPACK_DEBIAN_DEVEL_PACKAGE_DEPENDS "librobcomm (=${CMAKE_PROJECT_VERSION})")
SET(CPACK_DEBIAN_PACKAGE_DESCRIPTION "This package contains a library for communicating with a RobCo robot.")
SET(CPACK_DEBIAN_FILE_NAME "DEB-DEFAULT")
INCLUDE(CPack)
