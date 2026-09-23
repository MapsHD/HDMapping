cmake_minimum_required(VERSION 4.0.0)

project(calib_core_tests)

# Unit tests for calib_core's camera models and solvers. calib_core pulls in
# no raylib/imgui/GL, so its projection math is testable without a GL context
# -- the reason the equirectangular and Mei models live there, not in an app.
# Uses doctest, like shared/tests. test_camera.cpp owns doctest's main()
# (DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN); test_solver.cpp adds more TEST_CASEs
# to the same registry.
add_executable(calib_core_tests
    test_camera.cpp
    test_solver.cpp
    test_trajectory.cpp
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
