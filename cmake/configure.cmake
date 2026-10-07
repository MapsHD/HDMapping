include_guard()

# ------------------------------------------------------------
# Build configuration
# ------------------------------------------------------------

if(CMAKE_CONFIGURATION_TYPES)
    set(HDMAPPING_CONFIGURE_IS_MULTICONFIG "ON")
    list(JOIN CMAKE_CONFIGURATION_TYPES ";" HDMAPPING_CONFIGURE_BUILD_TYPES)
else()
    set(HDMAPPING_CONFIGURE_IS_MULTICONFIG "OFF")
    set(HDMAPPING_CONFIGURE_BUILD_TYPES "${CMAKE_BUILD_TYPE}")
endif()

# ------------------------------------------------------------
# Platform
# ------------------------------------------------------------

set(HDMAPPING_CONFIGURE_SYSTEM            "${CMAKE_SYSTEM_NAME}")
set(HDMAPPING_CONFIGURE_SYSTEM_VERSION    "${CMAKE_SYSTEM_VERSION}")
set(HDMAPPING_CONFIGURE_CPU               "${CMAKE_SYSTEM_PROCESSOR}")

# ------------------------------------------------------------
# Compiler
# ------------------------------------------------------------

set(HDMAPPING_CONFIGURE_COMPILER          "${CMAKE_CXX_COMPILER_ID}")
set(HDMAPPING_CONFIGURE_COMPILER_VERSION  "${CMAKE_CXX_COMPILER_VERSION}")
set(HDMAPPING_CONFIGURE_COMPILER_PATH     "${CMAKE_CXX_COMPILER}")

# ------------------------------------------------------------
# CMake
# ------------------------------------------------------------

set(HDMAPPING_CONFIGURE_CMAKE_VERSION     "${CMAKE_VERSION}")
set(HDMAPPING_CONFIGURE_GENERATOR         "${CMAKE_GENERATOR}")

# ------------------------------------------------------------
# Project
# ------------------------------------------------------------

set(HDMAPPING_CONFIGURE_PROJECT_NAME      "${PROJECT_NAME}")
set(HDMAPPING_CONFIGURE_PROJECT_VERSION   "${PROJECT_VERSION}")

# ------------------------------------------------------------
# Language
# ------------------------------------------------------------

set(HDMAPPING_CONFIGURE_CXX_STANDARD      "${CMAKE_CXX_STANDARD}")

# ------------------------------------------------------------
# Timestamp
# ------------------------------------------------------------

string(TIMESTAMP HDMAPPING_CONFIGURE_TIMESTAMP "%Y-%m-%d %H:%M:%S")

# ------------------------------------------------------------
# Git
# ------------------------------------------------------------

execute_process(
    COMMAND git rev-parse HEAD
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE HDMAPPING_CONFIGURE_GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

execute_process(
    COMMAND git rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE HDMAPPING_CONFIGURE_GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

foreach(VAR
    HDMAPPING_CONFIGURE_GIT_HASH
    HDMAPPING_CONFIGURE_GIT_BRANCH)
    if(NOT ${VAR})
        set(${VAR} "unknown")
    endif()
endforeach()

execute_process(
    COMMAND git status --porcelain
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE HDMAPPING_CONFIGURE_GIT_STATUS_PORCELAIN
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(HDMAPPING_CONFIGURE_GIT_STATUS_PORCELAIN STREQUAL "")
    set(HDMAPPING_CONFIGURE_GIT_IS_DIRTY "false")
else()
    set(HDMAPPING_CONFIGURE_GIT_IS_DIRTY "true")
endif()

# ------------------------------------------------------------
# Architecture and Flags
# ------------------------------------------------------------

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(HDMAPPING_CONFIGURE_IS_64_BIT "true")
else()
    set(HDMAPPING_CONFIGURE_IS_64_BIT "false")
endif()

set(HDMAPPING_CONFIGURE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(HDMAPPING_CONFIGURE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
set(HDMAPPING_CONFIGURE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

set(HDMAPPING_CONFIGURE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")

# ------------------------------------------------------------
# Host System
# ------------------------------------------------------------

set(HDMAPPING_CONFIGURE_HOST_SYSTEM "${CMAKE_HOST_SYSTEM_NAME}")
set(HDMAPPING_CONFIGURE_HOST_SYSTEM_VERSION "${CMAKE_HOST_SYSTEM_VERSION}")
set(HDMAPPING_CONFIGURE_HOST_CPU "${CMAKE_HOST_SYSTEM_PROCESSOR}")

configure_file(
    "${REPOSITORY_DIRECTORY}/cmake/configure/configure.hpp.in"
    "${REPOSITORY_DIRECTORY}/shared/include/HDMapping/HDMAPPING_ConfigureInfo.hpp"
    @ONLY
)