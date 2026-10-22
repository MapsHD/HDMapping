cmake_minimum_required(VERSION 4.0.0)

# Legacy GLUT + legacy immediate-mode OpenGL build of the raw data viewer,
# kept side by side with apps/mandeye_raw_data_viewer (raylib-based since it
# was ported off GLUT), the same way step2/step3 keep their *_legacy builds.
project(mandeye_raw_data_viewer_legacy)

# Source files
set(SOURCES
  mandeye_raw_data_viewer.cpp
  ../lidar_odometry_step_1/toml_io.cpp
  ../lidar_odometry_step_1/lidar_odometry.cpp
  ../lidar_odometry_step_1/lidar_odometry_utils.cpp
  ../lidar_odometry_step_1/lidar_odometry_utils_optimizers.cpp
  ../lidar_odometry_step_1/points_global_spill.cpp
  ../lidar_odometry_step_1/raw_cloud_loader.cpp
    "../../core/src/utils.cpp"
)

# Windows: add resource file
if(WIN32)
    list(APPEND SOURCES resource.rc)
endif()

add_executable(
  mandeye_raw_data_viewer_legacy ${SOURCES}
)

target_include_directories(
  mandeye_raw_data_viewer_legacy
  PRIVATE include
  ${REPOSITORY_DIRECTORY}/core/include
  ${REPOSITORY_DIRECTORY}/core_hd_mapping/include
  ${THIRDPARTY_DIRECTORY}
  ${THIRDPARTY_DIRECTORY}/glm
  ${EIGEN3_INCLUDE_DIR}
  ${THIRDPARTY_DIRECTORY}/tomlplusplus/include
  ${THIRDPARTY_DIRECTORY}/imgui
  ${THIRDPARTY_DIRECTORY}/imgui/backends
  ${THIRDPARTY_DIRECTORY}/ImGuizmo
  ${THIRDPARTY_DIRECTORY}/glew-cmake/include
  ${FREEGLUT_INCLUDE_DIR}
  ${THIRDPARTY_DIRECTORY}/json/include
  ${THIRDPARTY_DIRECTORY}/portable-file-dialogs-master
  ${LASZIP_INCLUDE_DIR}/LASzip/include
  ${THIRDPARTY_DIRECTORY}/observation_equations/codes
  ${THIRDPARTY_DIRECTORY}/vqf/vqf/cpp
  ${THIRDPARTY_DIRECTORY}/Fusion/Fusion)

target_link_libraries(
  mandeye_raw_data_viewer_legacy
  PRIVATE vqf Fusion
  unordered_dense::unordered_dense
  spdlog::spdlog
  UTL::include
  ${FREEGLUT_LIBRARY}
  ${OPENGL_gl_LIBRARY}
  OpenGL::GLU
  ${PLATFORM_LASZIP_LIB}
  ${CORE_LIBRARIES}
  ${GUI_LIBRARIES}
  ${PLATFORM_MISCELLANEOUS_LIBS}
  ${OpenCV_LIBS})

if(WIN32)
  add_custom_command(
    TARGET mandeye_raw_data_viewer_legacy
    POST_BUILD
    COMMAND
    ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:mandeye_raw_data_viewer_legacy>
    $<TARGET_FILE_DIR:mandeye_raw_data_viewer_legacy>
    COMMAND_EXPAND_LISTS)
endif()

if (MSVC)
    target_compile_options(mandeye_raw_data_viewer_legacy PRIVATE /bigobj)
endif()

hdmapping_install_app(mandeye_raw_data_viewer_legacy)
