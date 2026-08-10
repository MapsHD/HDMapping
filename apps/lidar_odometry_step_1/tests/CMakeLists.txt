cmake_minimum_required(VERSION 4.0.0)

project(lidar_odometry_step_1_tests)

# Unit tests for the MLvxCalib namespace (lidar_odometry_utils.h/.cpp), which
# parses Livox extrinsic/IMU calibration files (.json/.mjc and .sn). Compiles
# the real lidar_odometry_utils.cpp (no GUI code path, WITH_GUI left
# undefined) instead of the whole lidar_odometry_step_1 app, so the include
# dirs/link libraries below are the subset that TU actually needs: laszip and
# TBB because load_point_cloud()/decimate() (compiled into the same TU) use
# them even though the tests never call those functions, core_no_gui for
# Core/ndt.h & Core/hash_utils.h symbols, and vqf/Fusion/unordered_dense/
# spdlog/UTL for the rest of lidar_odometry_utils.h's includes. Uses doctest,
# same as shared/tests.
add_executable(lidar_odometry_step_1_tests
    test_mlvx_calib.cpp
    ../lidar_odometry_utils.cpp
)

target_include_directories(lidar_odometry_step_1_tests PRIVATE
    ${THIRDPARTY_DIRECTORY}/doctest
    ${REPOSITORY_DIRECTORY}/core/include
    ${THIRDPARTY_DIRECTORY}
    ${EIGEN3_INCLUDE_DIR}
    ${THIRDPARTY_DIRECTORY}/json/include
    ${LASZIP_INCLUDE_DIR}/LASzip/include
    ${THIRDPARTY_DIRECTORY}/observation_equations/codes
    ${THIRDPARTY_DIRECTORY}/vqf/vqf/cpp
    ${THIRDPARTY_DIRECTORY}/Fusion/Fusion
)

target_link_libraries(lidar_odometry_step_1_tests PRIVATE
    core_no_gui
    vqf
    Fusion
    unordered_dense::unordered_dense
    spdlog::spdlog
    UTL::include
    ${PLATFORM_LASZIP_LIB}
    ${PLATFORM_MISCELLANEOUS_LIBS}
)

if (MSVC)
    target_compile_definitions(lidar_odometry_step_1_tests PRIVATE _USE_MATH_DEFINES)
    target_compile_options(lidar_odometry_step_1_tests PRIVATE /bigobj)
endif()

include(CTest)
add_test(NAME lidar_odometry_step_1_tests COMMAND lidar_odometry_step_1_tests)
