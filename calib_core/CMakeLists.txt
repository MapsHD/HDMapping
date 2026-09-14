cmake_minimum_required(VERSION 4.0.0)

project(calib_core)

# Shared, GUI-independent logic for the camera_lidar_calibration app family
# (camera_lidar_calibration, camera_lidar_trajectory_viewer,
# camera_lidar_intrinsics_calib) -- LiDAR-camera projection math, LAS/LAZ
# point cloud loading, Mandeye trajectory CSV parsing, and CLI argument
# parsing. Deliberately depends on nothing but Eigen/LASzip/std plus, for the
# Mei camera model only (MeiCamera.h/.cpp), OpenCV's header-only Point2d/
# Point3d and yaml-cpp for camera_info.yaml loading -- no raylib/imgui, and
# no other OpenCV usage, here -- so it stays reusable and cheap to build for
# tools (like camera_lidar_intrinsics_calib) that don't need the others.
# File dialogs are a GUI concern, not calibration logic, so they live in
# core's core_pfd target (mandeye::fd) instead -- apps link it directly.
#
# All public types live in namespace calib (see include/CalibCore/*.h) to
# avoid colliding with core's own global (non-namespaced) PointCloud
# (core/include/Core/point_cloud.h), in case a future consumer links both
# calib_core and core/core_raylib in the same binary.
add_library(calib_core STATIC
    src/Camera.cpp
    src/MeiCamera.cpp
    src/PointCloud.cpp
    src/Trajectory.cpp
    src/CliArgs.cpp
    src/CameraCalibrationSolver.cpp
    src/CameraCalibrationSolverMei.cpp
)

# yaml-cpp is a system package everywhere else in this repo already assumes
# is available for it (MeiCamera.cpp's #include <yaml-cpp/yaml.h> predates
# this CMakeLists wiring) -- same "find it on the system" treatment as
# OpenCV gets in 3rdpartyBinary/OpenCV/CMakeLists.txt, just scoped to this
# target since calib_core is its only consumer.
find_package(yaml-cpp REQUIRED)

# ── Optional Ceres-based Mei extrinsics solver ────────────────────────────────
# OFF by default: HDMapping's own README advertises depending on nothing but
# Eigen for its optimization (no Ceres/g2o/GTSAM/manif/Sophus), unlike
# yaml-cpp/OpenCV above this is a real optional feature, not something every
# build needs -- CameraCalibrationSolverMei.cpp's #ifdef falls back to a stub
# that explains the gap instead of solving, so callers
# (apps/camera_lidar_calibration's AppState::solvePairs) don't need their own
# #ifdef, just to check solveExtrinsicsMeiCeres()'s return value. Enable with
#   cmake -DCALIB_ENABLE_CERES=ON   (needs libceres-dev or equivalent).
option(CALIB_ENABLE_CERES "Enable the Ceres-based extrinsics solver for the Mei camera model" OFF)
if(CALIB_ENABLE_CERES)
    find_package(Ceres REQUIRED)
    # PUBLIC (not PRIVATE, unlike Ceres::ceres's own link below): lets
    # calib_core_tests -- calib_core's only other consumer that cares --
    # #ifdef on this to build the real-solve test only when it can actually
    # run, without duplicating this option's value into a second place.
    target_compile_definitions(calib_core PUBLIC CALIB_ENABLE_CERES)
    message(STATUS "calib_core Mei Ceres solver: ENABLED")
else()
    message(STATUS "calib_core Mei Ceres solver: disabled (set -DCALIB_ENABLE_CERES=ON to enable)")
endif()

target_include_directories(calib_core PUBLIC
    include
    # MeiCamera.h (a public calib_core header) includes <opencv2/core.hpp>
    # directly for cv::Point2d/Point3d, so anything that includes it needs
    # OpenCV's headers on its include path too. OpenCV_INCLUDE_DIRS/
    # OpenCV_LIBS themselves come from find_package(OpenCV) in
    # cmake/dependencies.cmake, included by the top-level CMakeLists.txt
    # before add_subdirectory(calib_core) -- no find_package(OpenCV) needed
    # here.
    ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(calib_core PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${LASZIP_INCLUDE_DIR}/LASzip/include
    # laszip_api_version.h is generated at configure time into LASzip's own
    # binary dir (see 3rdparty/LASzip/CMakeLists.txt's configure_file), not
    # exposed via any of its targets' usage requirements -- core's own LASzip
    # usage (color_las_loader.cpp) goes through the C++ LASzip/LASreader
    # classes instead and never needed this path, so nothing else in the repo
    # wires it up.
    ${CMAKE_BINARY_DIR}/3rdparty/LASzip/include
    # Reuses the vendored analytic camera observation equations directly
    # (CameraCalibrationSolver.cpp #includes the header from here) instead
    # of copying the math -- same include-the-generated-header convention
    # core/CMakeLists.txt already uses for its own observation equations.
    ${THIRDPARTY_DIRECTORY}/observation_equations/codes
    # Camera.cpp reuses core's own pose_tait_bryan_from_affine_matrix /
    # affine_matrix_from_pose_tait_bryan (core/include/Core/transformations.h)
    # for the om/fi/ka<->matrix conversion instead of duplicating that math.
    # Header-only and pulls in nothing but Eigen/std (see structures.h) --
    # doesn't violate calib_core's no-raylib/imgui rule above, and nothing
    # here links the core/core_math library, just includes headers.
    ${REPOSITORY_DIRECTORY}/core/include
)

target_link_libraries(calib_core PUBLIC
    ${PLATFORM_LASZIP_LIB}
    # For MeiCamera.cpp/.h -- see the target_include_directories comment
    # above. PUBLIC (like PLATFORM_LASZIP_LIB) because calib_core is a
    # STATIC library: CMake does not propagate a static library's own link
    # dependencies to its consumers unless they're PUBLIC/INTERFACE, so a
    # PRIVATE keyword here would leave executables linking calib_core with
    # unresolved yaml-cpp symbols.
    ${OpenCV_LIBS}
    yaml-cpp::yaml-cpp
)

if(CALIB_ENABLE_CERES)
    # PRIVATE, unlike the PUBLIC block above -- CameraCalibrationSolver.h
    # never exposes a Ceres type, so no consumer's own compilation needs
    # Ceres' include dirs, just the symbols to link (CMake still records a
    # STATIC library's PRIVATE link libraries as $<LINK_ONLY:...> for
    # whoever finally links an executable against calib_core, so this alone
    # is enough for that final link to resolve solveExtrinsicsMeiCeres's
    # ceres:: calls -- verified against this exact target during development).
    target_link_libraries(calib_core PRIVATE Ceres::ceres)
endif()

if(MSVC)
    target_compile_options(calib_core PRIVATE /W4)
    target_compile_definitions(calib_core PRIVATE _USE_MATH_DEFINES LASZIP_API_VERSION)
else()
    target_compile_options(calib_core PRIVATE -Wall -Wextra)
    target_compile_definitions(calib_core PRIVATE LASZIP_API_VERSION)
endif()
