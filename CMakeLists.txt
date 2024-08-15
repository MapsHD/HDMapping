cmake_minimum_required(VERSION 3.15.0)

project(hd-mapping)

add_definitions(-DHDMAPPING_VERSION_MAJOR=0)
add_definitions(-DHDMAPPING_VERSION_MINOR=54)
add_definitions(-DHDMAPPING_VERSION_PATCH=0)

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

set(EIGEN_BUILD_DOC
    OFF
    CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING
    OFF
    CACHE BOOL "" FORCE)
add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/eigen)

set(FREEGLUT_BUILD_DEMOS
    OFF
    CACHE BOOL "" FORCE)
add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/freeglut-3.2.2)

add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/LASzip)
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
