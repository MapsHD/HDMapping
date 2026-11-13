cmake_minimum_required(VERSION 4.0.0)

project(camera_lidar_depth_images)

# Command-line tool: renders a session's lidar points into the camera every N
# seconds and writes depth and intensity images. Same inputs as
# camera_lidar_trajectory_viewer (--mjs, --calib, --camera_dir); no GUI.
add_executable(camera_lidar_depth_images
    DepthImages.cpp
)

target_include_directories(camera_lidar_depth_images PRIVATE
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(camera_lidar_depth_images PRIVATE
    calib_core
    ${OpenCV_LIBS}
    ${PLATFORM_MISCELLANEOUS_LIBS}
)

if(MSVC)
    target_compile_options(camera_lidar_depth_images PRIVATE /W4)
    target_compile_definitions(camera_lidar_depth_images PRIVATE _USE_MATH_DEFINES)
else()
    target_compile_options(camera_lidar_depth_images PRIVATE -Wall -Wextra)
endif()

if(WIN32)
    add_custom_command(
        TARGET camera_lidar_depth_images
        POST_BUILD
        COMMAND
            ${CMAKE_COMMAND} -E copy
            $<TARGET_RUNTIME_DLLS:camera_lidar_depth_images>
            $<TARGET_FILE_DIR:camera_lidar_depth_images>
        COMMAND_EXPAND_LISTS)
endif()

hdmapping_install_app(camera_lidar_depth_images)
