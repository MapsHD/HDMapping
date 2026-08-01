cmake_minimum_required(VERSION 4.0.0)

project(calib_core)

# Shared, GUI-independent logic for the camera_lidar_calibration app family
# (camera_lidar_calibration, camera_lidar_trajectory_viewer,
# camera_lidar_intrinsics_calib) -- LiDAR-camera projection math, LAS/LAZ
# point cloud loading, Mandeye trajectory CSV parsing, and CLI argument
# parsing. Deliberately depends on nothing but Eigen/LASzip/std -- no
# raylib/imgui/OpenCV here -- so it stays reusable and cheap to build for
# tools (like camera_lidar_intrinsics_calib) that don't need the others.
#
# All public types live in namespace calib (see include/CalibCore/*.h) to
# avoid colliding with core's own global (non-namespaced) PointCloud
# (core/include/Core/point_cloud.h), in case a future consumer links both
# calib_core and core/core_raylib in the same binary.
add_library(calib_core STATIC
    src/Camera.cpp
    src/PointCloud.cpp
    src/Trajectory.cpp
    src/CliArgs.cpp
)

target_include_directories(calib_core PUBLIC
    include
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
)

target_link_libraries(calib_core PUBLIC ${PLATFORM_LASZIP_LIB})

if(MSVC)
    target_compile_options(calib_core PRIVATE /W4)
    target_compile_definitions(calib_core PRIVATE _USE_MATH_DEFINES LASZIP_API_VERSION)
else()
    target_compile_options(calib_core PRIVATE -Wall -Wextra)
    target_compile_definitions(calib_core PRIVATE LASZIP_API_VERSION)
endif()
