cmake_minimum_required(VERSION 4.0.0)

project(camera_lidar_trajectory_viewer)

# Multi-camera trajectory/point-cloud viewer with LAZ export, plus optional
# COLMAP sparse-model and ROS 2 bag export. Ported from the sibling
# mandeye-colors project (see calib_core/CMakeLists.txt for the shared
# non-GUI logic).
#
# NOTE on rendering: this app's core feature is per-point "which camera
# colored this point" RGB assignment plus a per-camera isolation toggle
# (see the kFS fragment shader's colorCameraId/selectedCamera uniform and
# TrajectoryViewer.cpp's isolateCamera option) over a single merged point
# buffer built from all trajectory chunks. core's ScanRenderer
# (core/include/Core/raylib_render.hpp, used by core_raylib) is a per-scan
# renderer -- Flat/Intensity/Elevation/Distance color modes with one pose
# per scan -- with no equivalent for that per-point camera-source
# attribute, which is load-bearing here, not cosmetic. Re-fitting it would
# mean either dropping that feature or extending core's shared renderer
# contract (also used by multi_view_tls_registration) for a need specific
# to this app, so this app keeps its own GPU shader code (like
# camera_lidar_calibration's Renderer.cpp) and links raylib/imgui_raylib/
# rlimgui directly rather than core_raylib.
add_executable(camera_lidar_trajectory_viewer
    TrajectoryViewer.cpp
    RosExport.h RosExport.cpp
)

target_include_directories(camera_lidar_trajectory_viewer PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${LASZIP_INCLUDE_DIR}/LASzip/include
    # laszip_api_version.h is generated at configure time into LASzip's own
    # binary dir (see calib_core/CMakeLists.txt's matching comment) -- this
    # app includes <laszip/laszip_api.h> directly, same as calib_core's
    # PointCloud.cpp.
    ${CMAKE_BINARY_DIR}/3rdparty/LASzip/include
    ${THIRDPARTY_DIRECTORY}/json/include
)

target_compile_definitions(camera_lidar_trajectory_viewer PRIVATE WITH_GUI=1)

target_link_libraries(camera_lidar_trajectory_viewer PRIVATE
    calib_core
    core_pfd
    core_e57
    raylib_widgets
    raylib
    imgui_raylib
    rlimgui
    ${OpenCV_LIBS}
    ${PLATFORM_LASZIP_LIB}
    ${PLATFORM_MISCELLANEOUS_LIBS}
)

if(MSVC)
    target_compile_options(camera_lidar_trajectory_viewer PRIVATE /W4)
    target_compile_definitions(camera_lidar_trajectory_viewer PRIVATE _USE_MATH_DEFINES LASZIP_API_VERSION)
else()
    target_compile_options(camera_lidar_trajectory_viewer PRIVATE -Wall -Wextra)
    target_compile_definitions(camera_lidar_trajectory_viewer PRIVATE LASZIP_API_VERSION)
endif()

# ── Optional ROS 2 bag export ─────────────────────────────────────────────────
# OFF by default so the project still builds on machines without ROS. Enable with
#   cmake -DCALIB_ENABLE_ROS_EXPORT=ON   (source a ROS 2 install first).
option(CALIB_ENABLE_ROS_EXPORT "Enable ROS 2 bag export in camera_lidar_trajectory_viewer (needs ROS 2)" OFF)
if(CALIB_ENABLE_ROS_EXPORT)
    find_package(rclcpp        REQUIRED)
    find_package(rosbag2_cpp   REQUIRED)
    find_package(rosbag2_storage REQUIRED)
    find_package(builtin_interfaces REQUIRED)
    find_package(std_msgs      REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(sensor_msgs   REQUIRED)
    find_package(tf2_msgs      REQUIRED)
    # Link the exported targets with the keyword (PRIVATE) form so it matches the
    # rest of this target's link calls (ament_target_dependencies uses the plain
    # form, which CMake forbids mixing).
    target_link_libraries(camera_lidar_trajectory_viewer PRIVATE
        rclcpp::rclcpp
        rosbag2_cpp::rosbag2_cpp
        rosbag2_storage::rosbag2_storage
        ${builtin_interfaces_TARGETS}
        ${std_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${tf2_msgs_TARGETS})
    target_compile_definitions(camera_lidar_trajectory_viewer PRIVATE CALIB_ENABLE_ROS_EXPORT)
    message(STATUS "camera_lidar_trajectory_viewer ROS 2 bag export: ENABLED")
else()
    message(STATUS "camera_lidar_trajectory_viewer ROS 2 bag export: disabled (set -DCALIB_ENABLE_ROS_EXPORT=ON to enable)")
endif()

if(WIN32)
    add_custom_command(
        TARGET camera_lidar_trajectory_viewer
        POST_BUILD
        COMMAND
            ${CMAKE_COMMAND} -E copy
            $<TARGET_RUNTIME_DLLS:camera_lidar_trajectory_viewer>
            $<TARGET_FILE_DIR:camera_lidar_trajectory_viewer>
        COMMAND_EXPAND_LISTS)
endif()

if(MSVC)
    target_compile_options(camera_lidar_trajectory_viewer PRIVATE /bigobj)
endif()

install(TARGETS camera_lidar_trajectory_viewer DESTINATION bin)
