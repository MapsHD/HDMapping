cmake_minimum_required(VERSION 4.0.0)

project(calib_core_tests)

# Unit tests for calib_core's camera model and solvers. calib_core
# deliberately depends on nothing but Eigen/LASzip/std, plus OpenCV/yaml-cpp
# for the Mei model and (optionally, see CALIB_ENABLE_CERES) Ceres for its
# extrinsics solver (see calib_core/CMakeLists.txt) -- no raylib/imgui/GL --
# so its projection math is directly testable without a GL context, which is
# the whole reason the equirectangular and Mei models live there rather than
# in an app.
# Uses doctest (3rdparty/doctest/doctest.h) for the same reason shared/tests
# does; see that directory's CMakeLists.txt. test_camera.cpp owns doctest's
# main() (DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN); test_solver.cpp just adds more
# TEST_CASEs to the same registry, same multi-TU doctest setup as shared/tests.
add_executable(calib_core_tests
    test_camera.cpp
    test_solver.cpp
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