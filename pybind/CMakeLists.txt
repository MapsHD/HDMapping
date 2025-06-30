cmake_minimum_required(VERSION 3.15)

project(bindings)

set(PYTHON_EXECUTABLE "" CACHE STRING "Path to a specific Python executable")

if(PYTHON_EXECUTABLE STREQUAL "")
    find_package(Python3 COMPONENTS Interpreter Development)
    if(Python3_FOUND)
        set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
        message(STATUS "Found system Python3: ${PYTHON_EXECUTABLE}")
    else()
        message(WARNING "No Python executable specified and Python3 not found. Skipping pybind targets.")
        return()
    endif()
else()
    message(STATUS "Using custom Python executable: ${PYTHON_EXECUTABLE}")
endif()

execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c "import sys; sys.exit(0)"
    RESULT_VARIABLE PYTHON_EXECUTABLE_VALID
)
if(NOT PYTHON_EXECUTABLE_VALID EQUAL 0)
    message(FATAL_ERROR "Invalid Python executable: ${PYTHON_EXECUTABLE}")
endif()

message(STATUS "Configuring pybind with Python executable: ${PYTHON_EXECUTABLE}")

add_library(lidar_odometry_py MODULE lidar_odometry_binding.cpp)
target_compile_definitions(lidar_odometry_py PRIVATE -DWITH_GUI=0)
if (MSVC)
    target_compile_options(lidar_odometry_py PRIVATE /bigobj)
endif()

add_library(multi_view_tls_registration_py MODULE tls_registration_binding.cpp)
target_compile_definitions(multi_view_tls_registration_py PRIVATE -DWITH_GUI=0)
if (MSVC)
    target_compile_options(multi_view_tls_registration_py PRIVATE /bigobj)
endif()

target_sources(lidar_odometry_py
    PRIVATE
        ${CMAKE_SOURCE_DIR}/apps/lidar_odometry_step_1/lidar_odometry.cpp
        ${CMAKE_SOURCE_DIR}/apps/lidar_odometry_step_1/lidar_odometry_utils.cpp
        ${CMAKE_SOURCE_DIR}/apps/lidar_odometry_step_1/lidar_odometry_utils_optimizers.cpp
)

target_sources(multi_view_tls_registration_py
    PRIVATE
        ${CMAKE_SOURCE_DIR}/apps/multi_view_tls_registration/multi_view_tls_registration.cpp
)

target_include_directories(lidar_odometry_py
    PRIVATE ${pybind11_SOURCE_DIR}/include
            ${CMAKE_SOURCE_DIR}/apps/lidar_odometry_step_1
            ${REPOSITORY_DIRECTORY}/core/include
            ${REPOSITORY_DIRECTORY}/core_hd_mapping/include
            ${EIGEN3_INCLUDE_DIR}
            ${LASZIP_INCLUDE_DIR}/LASzip/include
            ${EXTERNAL_LIBRARIES_DIRECTORY}/Fusion/Fusion
            ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
            ${EXTERNAL_LIBRARIES_DIRECTORY}
            ${EXTERNAL_LIBRARIES_DIRECTORY}/glm
            ${EXTERNAL_LIBRARIES_DIRECTORY}/observation_equations/codes
)

target_include_directories(multi_view_tls_registration_py
    PRIVATE ${pybind11_SOURCE_DIR}/include
            ${CMAKE_SOURCE_DIR}/apps/multi_view_tls_registration
            ${REPOSITORY_DIRECTORY}/core/include
            ${REPOSITORY_DIRECTORY}/core_hd_mapping/include
            ${EIGEN3_INCLUDE_DIR}
            ${LASZIP_INCLUDE_DIR}/LASzip/include
            ${EXTERNAL_LIBRARIES_DIRECTORY}/Fusion/Fusion
            ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
            ${EXTERNAL_LIBRARIES_DIRECTORY}
            ${EXTERNAL_LIBRARIES_DIRECTORY}/glm
            ${EXTERNAL_LIBRARIES_DIRECTORY}/observation_equations/codes
)

target_link_libraries(lidar_odometry_py
    PRIVATE pybind11::module 
            core_no_gui
            ${PLATFORM_LASZIP_LIB} 
            Fusion
            ${PLATFORM_MISCELLANEOUS_LIBS}
)

target_link_libraries(multi_view_tls_registration_py
    PRIVATE pybind11::module 
            core_no_gui
            ${PLATFORM_LASZIP_LIB} 
            Fusion
            ${PLATFORM_MISCELLANEOUS_LIBS}
)

if(WIN32)
    set_target_properties(lidar_odometry_py PROPERTIES SUFFIX ".pyd")
    set_target_properties(multi_view_tls_registration_py PROPERTIES SUFFIX ".pyd")
else()
    set_target_properties(lidar_odometry_py PROPERTIES SUFFIX ".so")
    set_target_properties(multi_view_tls_registration_py PROPERTIES SUFFIX ".so")
endif()

set_target_properties(lidar_odometry_py PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/pybind"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/pybind"
)

set_target_properties(multi_view_tls_registration_py PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/pybind"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/pybind"
)

set(DLL_SOURCE_DIR "${CMAKE_BINARY_DIR}/bin/Release")
set(DLL_DEST_DIR "${CMAKE_SOURCE_DIR}/pybind/Release")

file(GLOB DLL_FILES "${CMAKE_BINARY_DIR}/bin/Release/*.dll")
foreach(DLL ${DLL_FILES})
    add_custom_command(
        TARGET lidar_odometry_py POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DLL} "${CMAKE_SOURCE_DIR}/pybind/Release"
    )
endforeach()