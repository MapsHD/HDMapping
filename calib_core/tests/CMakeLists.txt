cmake_minimum_required(VERSION 4.0.0)

project(calib_core_tests)

# Unit tests for calib_core's camera model. calib_core deliberately depends on
# nothing but Eigen/LASzip/std (see calib_core/CMakeLists.txt), so its
# projection math is directly testable without a GL context -- which is the
# whole reason the equirectangular model lives there rather than in an app.
# Uses doctest (3rdparty/doctest/doctest.h) for the same reason shared/tests
# does; see that directory's CMakeLists.txt.
add_executable(calib_core_tests
    test_camera.cpp
)

target_link_libraries(calib_core_tests PRIVATE calib_core)

# calib_core's Eigen include is PRIVATE, so it isn't inherited by linking.
target_include_directories(calib_core_tests PRIVATE
    ${THIRDPARTY_DIRECTORY}/doctest
    ${EIGEN3_INCLUDE_DIR}
)

if (MSVC)
    target_compile_definitions(calib_core_tests PRIVATE _USE_MATH_DEFINES)
endif()

include(CTest)
add_test(NAME calib_core_tests COMMAND calib_core_tests)