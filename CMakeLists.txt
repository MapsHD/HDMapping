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

add_subdirectory(${THIRDPARTY_DIRECTORY}/eigen)
set(EIGEN3_INCLUDE_DIR ${THIRDPARTY_DIRECTORY}/eigen)
MESSAGE(STATUS "Using bundled Eigen3 : ${EIGEN3_INCLUDE_DIR}")

if (BUILD_WITH_BUNDLED_FREEGLUT)
    set(FREEGLUT_BUILD_DEMOS
            OFF
            CACHE BOOL "" FORCE)
    add_subdirectory(${THIRDPARTY_DIRECTORY}/freeglut)
    set(FREEGLUT_INCLUDE_DIR ${THIRDPARTY_DIRECTORY}/freeglut/include)
    set(FREEGLUT_LIBRARY freeglut)
    # Suppress unused variable warnings from FreeGlut
    if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(freeglut PRIVATE -Wno-unused-const-variable)
    endif()
    MESSAGE(STATUS "Using bundled FreeGlut")
else()
    find_package(GLUT REQUIRED)
    set(FREEGLUT_INCLUDE_DIR ${GLUT_INCLUDE_DIR})
    set(FREEGLUT_LIBRARY ${GLUT_glut_LIBRARY})
    MESSAGE(STATUS "FreeGlut include dir: ${FREEGLUT_INCLUDE_DIR}")
endif ()

if (BUILD_WITH_BUNDLED_GLEW)
    if(APPLE)
      # On modern macOS AGL framework is deprecated/removed.
      # We point AGL_LIBRARY to OpenGL to trick the submodule into linking against a valid framework instead of failing.
      find_library(AGL_LIBRARY OpenGL)
    endif()

    add_subdirectory(${THIRDPARTY_DIRECTORY}/glew-cmake)
    set(GLEW_INCLUDE_DIR ${THIRDPARTY_DIRECTORY}/glew-cmake/include)
    set(GLEW_LIBRARY libglew_static)

    MESSAGE(STATUS "Using bundled GLEW")
else()
    find_package(GLEW REQUIRED)
    set(GLEW_INCLUDE_DIR ${GLEW_INCLUDE_DIRS})
    set(GLEW_LIBRARY ${GLEW_LIBRARIES})
    MESSAGE(STATUS "GLEW include dir: ${GLEW_INCLUDE_DIR}, GLEW library: ${GLEW_LIBRARY}")
endif()

if (BUILD_WITH_BUNDLED_LIBLASZIP)
    add_subdirectory(${THIRDPARTY_DIRECTORY}/LASzip)
    set(LASZIP_INCLUDE_DIR ${THIRDPARTY_DIRECTORY})
    set(LASZIP_LIBRARY laszip)
    MESSAGE(STATUS "Using bundled LASzip")
    
    if(APPLE)
        target_compile_definitions(laszip PUBLIC off64_t=off_t)
    endif()
else()
    find_package(LASzip REQUIRED)
    set(LASZIP_INCLUDE_DIR ${LASZIP_INCLUDE_DIR})
    set(LASZIP_LIBRARY ${LASZIP_LIBRARY})
    set(PLATFORM_LASZIP_LIB ${LASZIP_LIBRARY})
    MESSAGE(STATUS "LASzip include dir: ${LASZIP_INCLUDE_DIR}, LASzip library: ${LASZIP_LIBRARY}")
endif ()

# Download or Search for OpenCV
include(${THIRDPARTY_DIRECTORY_BINARY}/OpenCV/CMakeLists.txt)
find_package(OpenCV REQUIRED)
MESSAGE(STATUS "OpenCV include dir: ${OpenCV_INCLUDE_DIRS}, OpenCV librarys: ${OpenCV_LIBS}")

# Download or Search for PROJ
include(${THIRDPARTY_DIRECTORY_BINARY}/Proj/CMakeLists.txt)
MESSAGE(STATUS "PROJ include dir: ${PROJ_INCLUDE_DIR}, PROJ library: ${PROJ_LIBRARY}")
MESSAGE(STATUS "PROJ implementation library: ${PROJ_IMPLIB}, PROJ database: ${PROJ_DB}")

# Option to build with bundled oneTBB
if (BUILD_WITH_BUNDLED_ONETBB)
    # Disable oneTBB components we don't need
    set(TBB_TEST OFF CACHE BOOL "" FORCE)
    set(TBB_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(TBB_STRICT OFF CACHE BOOL "" FORCE)
    set(TBBMALLOC_BUILD OFF CACHE BOOL "" FORCE)
    set(TBB_CPF OFF CACHE BOOL "" FORCE)

    add_subdirectory(${THIRDPARTY_DIRECTORY}/oneTBB)
    MESSAGE(STATUS "Using bundled oneTBB from: ${THIRDPARTY_DIRECTORY}/oneTBB")
else()
    # Find TBB from system
    find_package(TBB REQUIRED)
    MESSAGE(STATUS "Found system TBB")
endif()

# Add unordered_dense header-only library
add_subdirectory(${THIRDPARTY_DIRECTORY}/unordered_dense)
MESSAGE(STATUS "Using bundled unordered_dense from: ${THIRDPARTY_DIRECTORY}/unordered_dense")

# Add spdlog logging library
set(SPDLOG_BUILD_PIC ON CACHE BOOL "Build spdlog with position-independent code")
add_subdirectory(${THIRDPARTY_DIRECTORY}/spdlog)
MESSAGE(STATUS "Using bundled spdlog from: ${THIRDPARTY_DIRECTORY}/spdlog")

# Add UTL header-only utility library
add_subdirectory(${THIRDPARTY_DIRECTORY}/UTL)
MESSAGE(STATUS "Using bundled UTL from: ${THIRDPARTY_DIRECTORY}/UTL")

# ---- Profiler backend ----
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

if(BUILD_WITH_PYBIND)
    message(STATUS "BUILD_WITH_PYBIND is enabled: fetching pybind.")
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
    message(STATUS "BUILD_WITH_PYBIND is disabled.")
endif()


add_subdirectory(${THIRDPARTY_DIRECTORY}/vqf/vqf/cpp)
add_subdirectory(${THIRDPARTY_DIRECTORY}/Fusion/Fusion)

add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/plycpp)
add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/wgs84_do_puwg92)
add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/WGS84toCartesian)

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