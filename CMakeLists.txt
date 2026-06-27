cmake_minimum_required(VERSION 4.0.0)

project(hd-mapping)

# Common settings
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Versioning settings
set(HDMAPPING_VERSION_MAJOR 0)
set(HDMAPPING_VERSION_MINOR 101)
set(HDMAPPING_VERSION_PATCH 0)

# Set up common paths
get_filename_component(REPOSITORY_DIRECTORY ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
set(THIRDPARTY_DIRECTORY ${REPOSITORY_DIRECTORY}/3rdparty)
set(THIRDPARTY_DIRECTORY_BINARY ${REPOSITORY_DIRECTORY}/3rdpartyBinary)
set(EXTERNAL_LIBRARIES_DIRECTORY ${REPOSITORY_DIRECTORY}/external)

# Set up CPU optimization config
set(HD_CPU_OPTIMIZATION "AUTO" CACHE STRING "CPU optimization target (AUTO, INTEL, AMD, ARM, GENERIC)")
set_property(CACHE HD_CPU_OPTIMIZATION PROPERTY STRINGS AUTO INTEL AMD ARM GENERIC)

# Options to enable / disable specific components
option(BUILD_WITH_CLI_TOOLS "Build with command line tools" ON)
option(BUILD_WITH_HD_MAPPER_APPLICATION "Build with hd_mapper tool" OFF)
option(BUILD_WITH_QUICK_START_DEMO "Build with quick_start_demo" OFF)

# Three-way profiler backend: NONE (zero overhead) | UTL (lightweight) | TRACY (full profiler)
set(HDMAPPING_PROFILER "NONE" CACHE STRING "Profiler backend: NONE | UTL | TRACY")
set_property(CACHE HDMAPPING_PROFILER PROPERTY STRINGS NONE UTL TRACY)
option(BUILD_WITH_BUNDLED_FREEGLUT "Build with bundled FreeGlut" ON)
option(BUILD_WITH_BUNDLED_GLEW "Build with bundled GLEW" ON)
option(BUILD_WITH_BUNDLED_LIBLASZIP "Build with bundled Lib LASZIP" ON)
option(BUILD_WITH_BUNDLED_ONETBB "Build with bundled oneTBB" ON)
option(BUILD_WITH_PYBIND "Enable pybind11 bindings" OFF)

# Add definitions for all projects
add_definitions(-DHDMAPPING_VERSION_MAJOR=${HDMAPPING_VERSION_MAJOR})
add_definitions(-DHDMAPPING_VERSION_MINOR=${HDMAPPING_VERSION_MINOR})
add_definitions(-DHDMAPPING_VERSION_PATCH=${HDMAPPING_VERSION_PATCH})

add_definitions(-D_USE_MATH_DEFINES)

if (MSVC)
    add_definitions(-DNOMINMAX)
    add_definitions(-DWIN32_LEAN_AND_MEAN)
endif()

if(APPLE AND CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    add_compile_definitions(USE_EXECUTION_PAR_UNSEQ=0)
else()
    add_compile_definitions(USE_EXECUTION_PAR_UNSEQ=1)
endif()

# Expand module seatch path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")

# TODO(mwlasiuk) : move
include_directories(shared/include)

# Packages setup and finding
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)

# Output DLLs and EXEs to  common library and LIBs to another
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# TODO(mwlasiuk) : fix
# if(WIN32)
#     set(CMAKE_PDB_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
#     set(CMAKE_COMPILE_PDB_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
# endif()

# Include cmake configs
include(cmake/cpu_optimizations.cmake)
include(cmake/disable_target_warnings.cmake)
include(cmake/platform.cmake)
include(cmake/imgui.cmake)
include(cmake/imguizmo.cmake)
include(cmake/implot.cmake)

# ============================================================================
# Configure all dependencies (3rdparty and external libraries)
# ============================================================================
include(cmake/dependencies.cmake)

# ============================================================================
# Core Library
# ============================================================================
add_subdirectory(core)

set(CORE_LIBRARIES core)
set(GUI_LIBRARIES imgui imguizmo implot)

if(BUILD_WITH_CLI_TOOLS)
    add_subdirectory(apps/console_tools)
endif()

if(BUILD_WITH_HD_MAPPER_APPLICATION)
    add_subdirectory(apps/hd_mapper)
endif()

if(BUILD_WITH_QUICK_START_DEMO)
    add_subdirectory(apps/quick_start_demo)
endif()

add_subdirectory(apps/lidar_odometry_step_1)    
add_subdirectory(apps/manual_color)
add_subdirectory(apps/multi_session_registration)
add_subdirectory(apps/multi_view_tls_registration)
add_subdirectory(apps/split_multi_livox)
add_subdirectory(apps/precision_forestry_tools)
add_subdirectory(apps/mandeye_raw_data_viewer)
add_subdirectory(apps/compare_trajectories)
add_subdirectory(apps/mandeye_mission_recorder_calibration)
add_subdirectory(apps/mandeye_single_session_viewer)
add_subdirectory(apps/livox_mid_360_intrinsic_calibration)
add_subdirectory(apps/single_session_manual_coloring)
add_subdirectory(apps/concatenate_multi_livox)

# NOTE(mwlasiuk) : disable warnings for third party libraries so they do not pollute build logs

###
### DISABLE WARNINGS FOR THIRDPARTY CODE - START
###

if(NOT WIN32)
    disable_target_warnings(laszip)
    disable_target_warnings(laszip_api)
else()
    disable_target_warnings(laszip3)
    disable_target_warnings(laszip_api3)
endif()

if(NOT WIN32)
    disable_target_warnings(tbb)
    disable_target_warnings(tbbbind_2_5)
else()
    disable_target_warnings(tbb)
endif()

disable_target_warnings(freeglut)
disable_target_warnings(freeglut_static)
disable_target_warnings(libglew_static)
disable_target_warnings(libglew_shared)
disable_target_warnings(spdlog)
disable_target_warnings(vqf)
disable_target_warnings(Fusion)

###
### DISABLE WARNINGS FOR THIRDPARTY CODE - END
###

# CPack configuration
set(CPACK_PACKAGE_NAME "hd_mapping")
set(CPACK_PACKAGE_VERSION "${HDMAPPING_VERSION_MAJOR}.${HDMAPPING_VERSION_MINOR}.${HDMAPPING_VERSION_PATCH}")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "HD Mapping debian package for Mandeye")
set(CPACK_PACKAGE_CONTACT "januszbedkowski@gmail.com")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Janusz Bedkowski <januszbedkowski@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.28), libstdc++6 (>= 8.3.0), freeglut3-dev, libeigen3-dev, liblaszip-dev")
if(WIN32)
    set(CPACK_GENERATOR "ZIP")
else()
    set(CPACK_GENERATOR "DEB")
endif()

set(CPACK_DEBIAN_PACKAGE_SECTION "utils")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")

include(CPack)