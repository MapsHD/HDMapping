cmake_minimum_required(VERSION 4.0.0)

# Legacy GLUT + legacy immediate-mode OpenGL build of step1, kept side by
# side with apps/lidar_odometry_step_1 (raylib-based since it was ported off
# GLUT), the same way step2/step3 keep their *_legacy builds. Only the GUI
# translation unit is copied: the odometry sources it builds on carry no
# GL/GLUT code, so they are compiled from apps/lidar_odometry_step_1 as-is.
project(lidar_odometry_step_1_legacy)

set(STEP1_DIR ${REPOSITORY_DIRECTORY}/apps/lidar_odometry_step_1)

set(SOURCES
  lidar_odometry_gui.cpp
  ${STEP1_DIR}/lidar_odometry_utils.cpp
  ${STEP1_DIR}/lidar_odometry_utils_optimizers.cpp
  ${STEP1_DIR}/lidar_odometry.cpp
  ${STEP1_DIR}/points_global_spill.cpp
  ${STEP1_DIR}/raw_cloud_loader.cpp
  ${STEP1_DIR}/toml_io.cpp
  "../../core/src/utils.cpp"
)

# Windows: add resource file
if(WIN32)
    list(APPEND SOURCES "resource.rc")
endif()

add_executable(lidar_odometry_step_1_legacy ${SOURCES})

target_compile_definitions(lidar_odometry_step_1_legacy PRIVATE WITH_GUI=1)
# target_apply_hdmapping_profiler() is defined by apps/lidar_odometry_step_1.
target_apply_hdmapping_profiler(lidar_odometry_step_1_legacy)

target_include_directories(
  lidar_odometry_step_1_legacy
  PRIVATE ${STEP1_DIR}
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
          lidar_odometry_step_1_legacy
          PRIVATE
          vqf
          Fusion
          unordered_dense::unordered_dense
          spdlog::spdlog
          UTL::include
          OpenGL::GLU
          ${FREEGLUT_LIBRARY}
          ${OPENGL_gl_LIBRARY}
          ${PLATFORM_LASZIP_LIB}
          ${PLATFORM_MISCELLANEOUS_LIBS}
          ${CORE_LIBRARIES}
          ${GUI_LIBRARIES})

if(WIN32)
  add_custom_command(
    TARGET lidar_odometry_step_1_legacy
    POST_BUILD
    COMMAND
      ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:lidar_odometry_step_1_legacy>
      $<TARGET_FILE_DIR:lidar_odometry_step_1_legacy>
    COMMAND_EXPAND_LISTS)
endif()

if (MSVC)
    target_compile_options(lidar_odometry_step_1_legacy PRIVATE /bigobj)
endif()

hdmapping_install_app(lidar_odometry_step_1_legacy)
