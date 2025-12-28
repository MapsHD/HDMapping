cmake_minimum_required(VERSION 4.0.0)

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

add_library(core_py MODULE core_binding.cpp)
target_compile_definitions(core_py PRIVATE -DWITH_GUI=0)
if (MSVC)
    target_compile_options(core_py PRIVATE /bigobj)
endif()

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

add_custom_target(pybind_all
    DEPENDS core_py lidar_odometry_py multi_view_tls_registration_py
    COMMENT "Building all Python bindings"
)

target_sources(core_py
    PRIVATE
        ${CMAKE_SOURCE_DIR}/core/src/session.cpp
)

target_sources(lidar_odometry_py
    PRIVATE
        ${CMAKE_SOURCE_DIR}/apps/lidar_odometry_step_1/lidar_odometry.cpp
        ${CMAKE_SOURCE_DIR}/apps/lidar_odometry_step_1/lidar_odometry_utils.cpp
        ${CMAKE_SOURCE_DIR}/apps/lidar_odometry_step_1/lidar_odometry_utils_optimizers.cpp
        ${CMAKE_SOURCE_DIR}/apps/lidar_odometry_step_1/toml_io.cpp
)

target_sources(multi_view_tls_registration_py
    PRIVATE
        ${CMAKE_SOURCE_DIR}/apps/multi_view_tls_registration/multi_view_tls_registration.cpp
)

target_include_directories(core_py
    PRIVATE ${pybind11_SOURCE_DIR}/include
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

target_link_libraries(core_py
    PRIVATE pybind11::module 
            core_no_gui
            ${PLATFORM_LASZIP_LIB} 
            Fusion
            ${PLATFORM_MISCELLANEOUS_LIBS}
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
    set_target_properties(core_py PROPERTIES SUFFIX ".pyd")
    set_target_properties(lidar_odometry_py PROPERTIES SUFFIX ".pyd")
    set_target_properties(multi_view_tls_registration_py PROPERTIES SUFFIX ".pyd")
else()
    set_target_properties(core_py PROPERTIES 
        PREFIX ""
        SUFFIX ".so")
    set_target_properties(lidar_odometry_py PROPERTIES 
        PREFIX ""
        SUFFIX ".so")
    set_target_properties(multi_view_tls_registration_py PROPERTIES 
        PREFIX ""
        SUFFIX ".so")
endif()

set_target_properties(core_py PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>"
)

set_target_properties(lidar_odometry_py PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>"
)

set_target_properties(multi_view_tls_registration_py PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>"
)

if(WIN32)
    add_custom_command(
        TARGET core_py
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:core_py>
                "${CMAKE_BINARY_DIR}/bin/$<CONFIG>"
        COMMAND_EXPAND_LISTS
    )
    
    add_custom_command(
        TARGET lidar_odometry_py
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:lidar_odometry_py>
                "${CMAKE_BINARY_DIR}/bin/$<CONFIG>"
        COMMAND_EXPAND_LISTS
    )
    
    add_custom_command(
        TARGET multi_view_tls_registration_py
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:multi_view_tls_registration_py>
                "${CMAKE_BINARY_DIR}/bin/$<CONFIG>"
        COMMAND_EXPAND_LISTS
    )
endif()

