cmake_minimum_required(VERSION 4.0.0)

project(camera_lidar_intrinsics_calib)

# Checkerboard-based camera intrinsic calibration (OpenCV rational distortion
# model). Ported from the sibling mandeye-colors project. Doesn't touch point
# clouds at all, so it only needs calib_core for CliArgs and core_pfd for
# file dialogs, plus the same light raylib/imgui_raylib/rlimgui/OpenCV stack
# as camera_lidar_calibration -- no LASzip, no core_raylib.
add_executable(camera_lidar_intrinsics_calib
    IntrinsicsCalib.cpp
)

target_include_directories(camera_lidar_intrinsics_calib PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${THIRDPARTY_DIRECTORY}/json/include
)

target_compile_definitions(camera_lidar_intrinsics_calib PRIVATE WITH_GUI=1)

target_link_libraries(camera_lidar_intrinsics_calib PRIVATE
    calib_core
    core_pfd
    raylib
    imgui_raylib
    rlimgui
    ${OpenCV_LIBS}
)

if(MSVC)
    target_compile_options(camera_lidar_intrinsics_calib PRIVATE /W4)
    target_compile_definitions(camera_lidar_intrinsics_calib PRIVATE _USE_MATH_DEFINES)
else()
    target_compile_options(camera_lidar_intrinsics_calib PRIVATE -Wall -Wextra)
endif()

if(WIN32)
    add_custom_command(
        TARGET camera_lidar_intrinsics_calib
        POST_BUILD
        COMMAND
            ${CMAKE_COMMAND} -E copy
            $<TARGET_RUNTIME_DLLS:camera_lidar_intrinsics_calib>
            $<TARGET_FILE_DIR:camera_lidar_intrinsics_calib>
        COMMAND_EXPAND_LISTS)
endif()

if(MSVC)
    target_compile_options(camera_lidar_intrinsics_calib PRIVATE /bigobj)
endif()

install(TARGETS camera_lidar_intrinsics_calib DESTINATION bin)
