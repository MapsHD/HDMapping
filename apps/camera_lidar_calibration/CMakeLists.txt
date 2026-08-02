cmake_minimum_required(VERSION 4.0.0)

project(camera_lidar_calibration)

# Interactive LiDAR-camera extrinsic calibration: aligns a point cloud (LAZ/LAS)
# to a camera image, with GPU-shader reprojection feedback. Ported from the
# sibling mandeye-colors project (see calib_core/CMakeLists.txt for the shared
# non-GUI logic). Its image-space projection/distortion overlay shader
# (Renderer.cpp) has no equivalent in core's ScanRenderer (core_raylib), so
# rendering stays bespoke here rather than reusing core_raylib -- this app
# links raylib/imgui_raylib/rlimgui directly instead, without pulling in
# core/core_math.
add_executable(camera_lidar_calibration
    main.cpp
    App.h App.cpp
    UI.h UI.cpp
    Renderer.h Renderer.cpp
)

target_include_directories(camera_lidar_calibration PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${LASZIP_INCLUDE_DIR}/LASzip/include
    ${THIRDPARTY_DIRECTORY}/json/include
)

target_compile_definitions(camera_lidar_calibration PRIVATE WITH_GUI=1)

target_link_libraries(camera_lidar_calibration PRIVATE
    calib_core
    core_pfd
    raylib
    imgui_raylib
    rlimgui
    ${OpenCV_LIBS}
    ${PLATFORM_LASZIP_LIB}
    ${PLATFORM_MISCELLANEOUS_LIBS}
)

if(MSVC)
    target_compile_options(camera_lidar_calibration PRIVATE /W4)
    target_compile_definitions(camera_lidar_calibration PRIVATE _USE_MATH_DEFINES LASZIP_API_VERSION)
else()
    target_compile_options(camera_lidar_calibration PRIVATE -Wall -Wextra)
    target_compile_definitions(camera_lidar_calibration PRIVATE LASZIP_API_VERSION)
endif()

if(WIN32)
    add_custom_command(
        TARGET camera_lidar_calibration
        POST_BUILD
        COMMAND
            ${CMAKE_COMMAND} -E copy
            $<TARGET_RUNTIME_DLLS:camera_lidar_calibration>
            $<TARGET_FILE_DIR:camera_lidar_calibration>
        COMMAND_EXPAND_LISTS)
endif()

if(MSVC)
    target_compile_options(camera_lidar_calibration PRIVATE /bigobj)
endif()

install(TARGETS camera_lidar_calibration DESTINATION bin)
