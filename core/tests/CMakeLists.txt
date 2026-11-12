cmake_minimum_required(VERSION 4.0.0)

project(hdmapping_core_tests)

# Unit tests for core/ (core_no_gui, the GUI-less build of the core library).
# Uses doctest, same as shared/tests.
add_executable(hdmapping_core_tests
    test_pair_wise_icp.cpp
)

target_include_directories(hdmapping_core_tests PRIVATE
    ${THIRDPARTY_DIRECTORY}/doctest
    ${REPOSITORY_DIRECTORY}/core/include
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(hdmapping_core_tests PRIVATE
    core_no_gui
    ${PLATFORM_MISCELLANEOUS_LIBS}
)

if (MSVC)
    target_compile_definitions(hdmapping_core_tests PRIVATE _USE_MATH_DEFINES)
endif()

include(CTest)
add_test(NAME hdmapping_core_tests COMMAND hdmapping_core_tests)
