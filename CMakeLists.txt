cmake_minimum_required(VERSION 3.15.0)

project(hd-mapping)

set (HDMAPPING_VERSION_MAJOR 0)
set (HDMAPPING_VERSION_MINOR 60)
set (HDMAPPING_VERSION_PATCH 0)

add_definitions(-DHDMAPPING_VERSION_MAJOR=${HDMAPPING_VERSION_MAJOR})
add_definitions(-DHDMAPPING_VERSION_MINOR=${HDMAPPING_VERSION_MINOR})
add_definitions(-DHDMAPPING_VERSION_PATCH=${HDMAPPING_VERSION_PATCH})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")

include_directories(shared/include)

set(OpenGL_GL_PREFERENCE GLVND)
set(CMAKE_CXX_STANDARD 20)

find_package(OpenGL REQUIRED)

if(NOT WIN32)
    find_package(TBB REQUIRED)
endif()

add_definitions(-D_USE_MATH_DEFINES)

get_filename_component(REPOSITORY_DIRECTORY ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
set(EXTERNAL_LIBRARIES_DIRECTORY ${REPOSITORY_DIRECTORY}/3rdparty)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include(cmake/platform.cmake)
include(cmake/imgui.cmake)
include(cmake/imguizmo.cmake)

# Option to build with bundled Eigen
option(BUILD_WITH_BUNDLED_EIGEN "Build with bundled Eigen" ON)
if (BUILD_WITH_BUNDLED_EIGEN)
    set(EIGEN_BUILD_DOC
            OFF
            CACHE BOOL "" FORCE)
    set(EIGEN_BUILD_TESTING
            OFF
            CACHE BOOL "" FORCE)

    add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/eigen)
    set(EIGEN3_INCLUDE_DIR ${EXTERNAL_LIBRARIES_DIRECTORY}/eigen)
    MESSAGE("Using bundled Eigen3 : "${EIGEN_INCLUDE_DIR})
else()
    find_package(Eigen3 REQUIRED)
    MESSAGE(STATUS "Found Eigen3: ${EIGEN3_INCLUDE_DIR}")
endif()

option(BUILD_WITH_BUNDLED_FREEGLUT "Build with bundled FreeGlut" ON)
if (BUILD_WITH_BUNDLED_FREEGLUT)
    set(FREEGLUT_BUILD_DEMOS
            OFF
            CACHE BOOL "" FORCE)
    add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/freeglut-3.2.2)
    set(FREEGLUT_INCLUDE_DIR ${EXTERNAL_LIBRARIES_DIRECTORY}/freeglut-3.2.2/include)
    set(FREEGLUT_LIBRARY freeglut)
    MESSAGE("Using bundled FreeGlut")
else()
    find_package(GLUT REQUIRED)
    set(FREEGLUT_INCLUDE_DIR ${GLUT_INCLUDE_DIR})
    set(FREEGLUT_LIBRARY ${GLUT_glut_LIBRARY})
    MESSAGE(STATUS "FreeGlut include dir: ${FREEGLUT_INCLUDE_DIR}")
endif ()

option(BUILD_WITH_BUNDLED_LIBLASZIP "Build with bundled Lib LASZIP" ON)
if (BUILD_WITH_BUNDLED_LIBLASZIP)
    add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/LASzip)
    set(LASZIP_INCLUDE_DIR ${EXTERNAL_LIBRARIES_DIRECTORY})
    set(LASZIP_LIBRARY laszip)
    MESSAGE("Using bundled LASzip")
else()
    find_package(LASzip REQUIRED)
    set(LASZIP_INCLUDE_DIR ${LASZIP_INCLUDE_DIR})
    set(LASZIP_LIBRARY ${LASZIP_LIBRARY})
    set(PLATFORM_LASZIP_LIB ${LASZIP_LIBRARY})
    MESSAGE(STATUS "LASzip include dir: ${LASZIP_INCLUDE_DIR}", "LASzip library: ${LASZIP_LIBRARY}")
endif ()

add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/Fusion/Fusion)

add_subdirectory(core)
add_subdirectory(core_hd_mapping)

set(CORE_LIBRARIES core core-hd-mapping)
set(GUI_LIBRARIES imgui imguizmo)

add_subdirectory(apps/hd_mapper)
add_subdirectory(apps/lidar_odometry_step_1)
add_subdirectory(apps/manual_color)
add_subdirectory(apps/matrix_mul)
add_subdirectory(apps/multi_session_registration)
add_subdirectory(apps/multi_view_tls_registration)
add_subdirectory(apps/split_multi_livox)
add_subdirectory(apps/precision_forestry_tools)
add_subdirectory(apps/mandeye_raw_data_viewer)

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