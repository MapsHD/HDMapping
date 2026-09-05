# cmake/dependencies.cmake
# Extract 3rdparty and external library configuration
# Included from root CMakeLists.txt

include_guard()

message(STATUS "==== Configuring Dependencies ====")

# ============================================================================
# Graphics Libraries (Bundled or System)
# ============================================================================

# Eigen3 - Linear algebra library (bundled)
add_subdirectory(${THIRDPARTY_DIRECTORY}/eigen)
set(EIGEN3_INCLUDE_DIR ${THIRDPARTY_DIRECTORY}/eigen)
message(STATUS "Using bundled Eigen3 : ${EIGEN3_INCLUDE_DIR}")

# FreeGLUT - OpenGL utilities
if (BUILD_WITH_BUNDLED_FREEGLUT)
    set(FREEGLUT_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
    add_subdirectory(${THIRDPARTY_DIRECTORY}/freeglut)
    set(FREEGLUT_INCLUDE_DIR ${THIRDPARTY_DIRECTORY}/freeglut/include)
    set(FREEGLUT_LIBRARY freeglut)
    
    # Suppress unused variable warnings from FreeGlut
    if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(freeglut PRIVATE -Wno-unused-const-variable)
    endif()
    message(STATUS "Using bundled FreeGlut")
else()
    find_package(GLUT REQUIRED)
    set(FREEGLUT_INCLUDE_DIR ${GLUT_INCLUDE_DIR})
    set(FREEGLUT_LIBRARY ${GLUT_glut_LIBRARY})
    message(STATUS "FreeGlut include dir: ${FREEGLUT_INCLUDE_DIR}")
endif()

# GLEW - OpenGL extension loader
if (BUILD_WITH_BUNDLED_GLEW)
    if(APPLE)
        # On modern macOS AGL framework is deprecated/removed.
        # Point AGL_LIBRARY to OpenGL to trick the submodule into linking against a valid framework
        find_library(AGL_LIBRARY OpenGL)
    endif()

    add_subdirectory(${THIRDPARTY_DIRECTORY}/glew-cmake)
    set(GLEW_INCLUDE_DIR ${THIRDPARTY_DIRECTORY}/glew-cmake/include)
    set(GLEW_LIBRARY libglew_static)
    message(STATUS "Using bundled GLEW")
else()
    find_package(GLEW REQUIRED)
    set(GLEW_INCLUDE_DIR ${GLEW_INCLUDE_DIRS})
    set(GLEW_LIBRARY ${GLEW_LIBRARIES})
    message(STATUS "GLEW include dir: ${GLEW_INCLUDE_DIR}, GLEW library: ${GLEW_LIBRARY}")
endif()

# ============================================================================
# Point Cloud & LiDAR Libraries
# ============================================================================

# LASzip - LAS file compression
if (BUILD_WITH_BUNDLED_LIBLASZIP)
    add_subdirectory(${THIRDPARTY_DIRECTORY}/LASzip)
    set(LASZIP_INCLUDE_DIR ${THIRDPARTY_DIRECTORY})
    set(LASZIP_LIBRARY laszip)
    message(STATUS "Using bundled LASzip")
    
    if(APPLE)
        target_compile_definitions(laszip PUBLIC off64_t=off_t)
    endif()
else()
    find_package(LASzip REQUIRED)
    set(LASZIP_INCLUDE_DIR ${LASZIP_INCLUDE_DIR})
    set(LASZIP_LIBRARY ${LASZIP_LIBRARY})
    set(PLATFORM_LASZIP_LIB ${LASZIP_LIBRARY})
    message(STATUS "LASzip include dir: ${LASZIP_INCLUDE_DIR}, LASzip library: ${LASZIP_LIBRARY}")
endif()

# E57 - Apache Xerces-C + libE57Format, both vendored as git submodules and
# built from source (no system packages). Xerces-C provides the XML parser
# libE57Format needs; cmake/FindXercesC.cmake redirects libE57Format's
# find_package(XercesC) to the in-tree `xerces-c` target defined here.
#
# NOTE: xerces-c's bundled tests/samples are kept out of CTest -- see the
# add_subdirectory() override further down. libE57Format's own tests are
# disabled via E57_BUILD_TEST=OFF.
#
# xerces-c/cmake/XercesDLL.cmake unconditionally runs
#   set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries")
# With no prior cache entry that creates BUILD_SHARED_LIBS=ON in the cache and
# flips EVERY dependency added afterwards (vqf, Fusion, glad, plycpp, ...) to
# a DLL with no exported symbols -- on Windows that means no import .lib and a
# project-wide link failure. Pin a cache entry to OFF up front so that set()
# becomes a no-op, then remove it so nothing else is affected. HDMapping builds
# all dependencies statically, so an existing BUILD_SHARED_LIBS is not
# expected; save and restore it anyway.
if(DEFINED CACHE{BUILD_SHARED_LIBS})
    set(_hd_bsl_prev "$CACHE{BUILD_SHARED_LIBS}")
    set(_hd_bsl_restore ON)
else()
    set(_hd_bsl_restore OFF)
endif()
set(BUILD_SHARED_LIBS OFF CACHE BOOL "HDMapping links Xerces-C + libE57Format statically" FORCE)

# Keep xerces-c's bundled `tests` and `samples` subdirectories out of the
# configure entirely: they unconditionally call enable_testing() + add_test()
# for ~80 sample-driven cases whose executables we never build
# (EXCLUDE_FROM_ALL), which then fail `ctest` in CI. Override add_subdirectory()
# to drop those two leaves while the _hd_skip_test_subdirs guard is set; the
# builtin stays reachable as _add_subdirectory() and the guard is cleared right
# after so every later add_subdirectory() (libE57Format, core, apps, HDMapping's
# own tests) behaves normally.
set(_hd_skip_test_subdirs ON)
macro(add_subdirectory _hd_dir)
    get_filename_component(_hd_leaf "${_hd_dir}" NAME)
    if(_hd_skip_test_subdirs AND (_hd_leaf STREQUAL "tests" OR _hd_leaf STREQUAL "samples"))
        message(STATUS "Skipping xerces-c '${_hd_leaf}' subdirectory (E57 tests disabled)")
    else()
        _add_subdirectory("${_hd_dir}" ${ARGN})
    endif()
endmacro()

# Xerces-C transcoder/netaccessor/message-loader defaults are the platform
# native, dependency-free choices (macOS: macosunicodeconverter/cfurl,
# Windows: windows/winsock, Linux: iconv/socket) -- no ICU, no libcurl.
add_subdirectory(${THIRDPARTY_DIRECTORY}/xerces-c ${CMAKE_BINARY_DIR}/3rdparty/xerces-c EXCLUDE_FROM_ALL)

set(_hd_skip_test_subdirs OFF)

# libE57Format: static lib target `E57Format`, headers at libE57Format/include.
set(E57_BUILD_TEST OFF CACHE BOOL "" FORCE)
set(E57_RELEASE_LTO OFF CACHE BOOL "" FORCE)   # don't force LTO into the wider build
if(WIN32)
    set(USING_STATIC_XERCES ON CACHE BOOL "" FORCE)   # adds XERCES_STATIC_LIBRARY define
endif()
add_subdirectory(${THIRDPARTY_DIRECTORY}/libE57Format ${CMAKE_BINARY_DIR}/3rdparty/libE57Format EXCLUDE_FROM_ALL)
set(LIBE57FORMAT_INCLUDE_DIR ${THIRDPARTY_DIRECTORY}/libE57Format/include)

if(_hd_bsl_restore)
    set(BUILD_SHARED_LIBS "${_hd_bsl_prev}" CACHE BOOL "Build shared libraries" FORCE)
else()
    unset(BUILD_SHARED_LIBS CACHE)
endif()
message(STATUS "Using bundled Xerces-C + libE57Format (static) for E57 support")

# ============================================================================
# Computer Vision & Geospatial Libraries (Pre-downloaded)
# ============================================================================

# OpenCV - Computer vision library
include(${THIRDPARTY_DIRECTORY_BINARY}/OpenCV/CMakeLists.txt)
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV include dir: ${OpenCV_INCLUDE_DIRS}, OpenCV libs: ${OpenCV_LIBS}")

# PROJ - Cartographic projections library
include(${THIRDPARTY_DIRECTORY_BINARY}/Proj/CMakeLists.txt)
message(STATUS "PROJ include dir: ${PROJ_INCLUDE_DIR}, PROJ library: ${PROJ_LIBRARY}")
message(STATUS "PROJ implementation library: ${PROJ_IMPLIB}, PROJ database: ${PROJ_DB}")

# ============================================================================
# Threading & Performance Libraries
# ============================================================================

# oneTBB - Intel Threading Building Blocks (Bundled or System)
if (BUILD_WITH_BUNDLED_ONETBB)
    # Disable oneTBB components we don't need
    set(TBB_TEST OFF CACHE BOOL "" FORCE)
    set(TBB_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(TBB_STRICT OFF CACHE BOOL "" FORCE)
    set(TBBMALLOC_BUILD OFF CACHE BOOL "" FORCE)
    set(TBB_CPF OFF CACHE BOOL "" FORCE)

    add_subdirectory(${THIRDPARTY_DIRECTORY}/oneTBB)
    message(STATUS "Using bundled oneTBB from: ${THIRDPARTY_DIRECTORY}/oneTBB")
else()
    find_package(TBB REQUIRED)
    message(STATUS "Found system TBB")
endif()

# ============================================================================
# Header-Only & Utility Libraries
# ============================================================================

# unordered_dense - Hash map library
add_subdirectory(${THIRDPARTY_DIRECTORY}/unordered_dense)
message(STATUS "Using bundled unordered_dense from: ${THIRDPARTY_DIRECTORY}/unordered_dense")

# spdlog - Logging library
set(SPDLOG_BUILD_PIC ON CACHE BOOL "Build spdlog with position-independent code")
add_subdirectory(${THIRDPARTY_DIRECTORY}/spdlog)
message(STATUS "Using bundled spdlog from: ${THIRDPARTY_DIRECTORY}/spdlog")

# UTL - Header-only utility library
add_subdirectory(${THIRDPARTY_DIRECTORY}/UTL)
message(STATUS "Using bundled UTL from: ${THIRDPARTY_DIRECTORY}/UTL")

# ============================================================================
# IMGui & Visualization Libraries
# ============================================================================

# Note: imgui, imguizmo, implot are configured separately in:
# cmake/imgui.cmake, cmake/imguizmo.cmake, cmake/implot.cmake

# VQF - Vector Quaternion Filter
add_subdirectory(${THIRDPARTY_DIRECTORY}/vqf/vqf/cpp)
message(STATUS "Using bundled VQF from: ${THIRDPARTY_DIRECTORY}/vqf/vqf/cpp")

# Fusion - AHRS filter implementation
add_subdirectory(${THIRDPARTY_DIRECTORY}/Fusion/Fusion)
message(STATUS "Using bundled Fusion from: ${THIRDPARTY_DIRECTORY}/Fusion/Fusion")

# ============================================================================
# Profiler Backend - Optional
# ============================================================================

if(HDMAPPING_PROFILER STREQUAL "TRACY")
    include(FetchContent)
    FetchContent_Declare(tracy
        GIT_REPOSITORY https://github.com/wolfpld/tracy.git
        GIT_TAG        v0.13.1
        GIT_SHALLOW    TRUE
    )
    set(TRACY_ENABLE    ON CACHE BOOL "" FORCE)
    set(TRACY_ON_DEMAND ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(tracy)
    message(STATUS "HDMapping profiler: Tracy v0.13.1 (on-demand)")
elseif(HDMAPPING_PROFILER STREQUAL "UTL")
    message(STATUS "HDMapping profiler: UTL")
else()
    message(STATUS "HDMapping profiler: NONE (disabled)")
endif()

# ============================================================================
# Python Bindings - Optional
# ============================================================================

if(BUILD_WITH_PYBIND)
    message(STATUS "BUILD_WITH_PYBIND is enabled: fetching pybind11")
    include(FetchContent)
    set(pybind11_SOURCE_DIR "${CMAKE_BINARY_DIR}/3rdparty/pybind11-src")
    set(pybind11_BINARY_DIR "${CMAKE_BINARY_DIR}/3rdparty/pybind11-build")
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11
        GIT_TAG        v2.12.1
        SOURCE_DIR     ${pybind11_SOURCE_DIR}
        BINARY_DIR     ${pybind11_BINARY_DIR}
    )
    FetchContent_GetProperties(pybind11)
    if(NOT pybind11_POPULATED)
        FetchContent_MakeAvailable(pybind11)
    endif()
    add_subdirectory(pybind)
else()
    message(STATUS "BUILD_WITH_PYBIND is disabled")
endif()

# ============================================================================
# External Coordinate Transformation Libraries
# ============================================================================

# Custom WGS84 projection utilities
add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/plycpp)
message(STATUS "Using external library: plycpp")

add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/wgs84_do_puwg92)
message(STATUS "Using external library: wgs84_do_puwg92")

add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/WGS84toCartesian)
message(STATUS "Using external library: WGS84toCartesian")


# mcap - support for ROS 2 bag export
add_subdirectory(${THIRDPARTY_DIRECTORY}/mcap)

message(STATUS "==== Dependencies Configuration Complete ====")

