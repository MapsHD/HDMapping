cmake_minimum_required(VERSION 4.0.0)

project(hdmapping_shared_tests)

# Unit tests for shared/include/HDMapping/*.h -- header-only utilities used
# across multiple apps (see PoseInterpolation.h's own top comment). Uses
# doctest (3rdparty/doctest/doctest.h, vendored single-header, MIT) rather
# than GoogleTest/Catch2: neither is vendored for offline use elsewhere in
# this repo (3rdparty/manif/test/gtest fetches GoogleTest live via
# ExternalProject_Add), and doctest compiles fast enough not to add
# noticeably to build time.
add_executable(hdmapping_shared_tests
    test_pose_interpolation.cpp
)

target_include_directories(hdmapping_shared_tests PRIVATE
    ${THIRDPARTY_DIRECTORY}/doctest
    ${EIGEN3_INCLUDE_DIR}
)

if (MSVC)
    target_compile_definitions(hdmapping_shared_tests PRIVATE _USE_MATH_DEFINES)
endif()

include(CTest)
add_test(NAME hdmapping_shared_tests COMMAND hdmapping_shared_tests)
