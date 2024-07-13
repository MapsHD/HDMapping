cmake_minimum_required(VERSION 3.15.0)

project(lidar_odometry_step_1)

add_executable(
  lidar_odometry_step_1
  lidar_odometry.cpp lidar_odometry_utils.h lidar_odometry_utils.cpp
  lidar_odometry_utils_optimizers.cpp)

target_include_directories(
  lidar_odometry_step_1
  PRIVATE include
          ${REPOSITORY_DIRECTORY}/core/include
          ${REPOSITORY_DIRECTORY}/core_hd_mapping/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glm
          ${EXTERNAL_LIBRARIES_DIRECTORY}/eigen
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui/backends
          ${EXTERNAL_LIBRARIES_DIRECTORY}/ImGuizmo
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glew-2.2.0/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/freeglut-3.2.2/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/portable-file-dialogs-master
          ${EXTERNAL_LIBRARIES_DIRECTORY}/LASzip/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/observation_equations/codes
          ${EXTERNAL_LIBRARIES_DIRECTORY}/Fusion/Fusion)

target_link_libraries(
  lidar_odometry_step_1
  PRIVATE Fusion
          freeglut
          ${OPENGL_gl_LIBRARY}
          OpenGL::GLU
          ${PLATFORM_LASZIP_LIB}
          ${PLATFORM_MISCELLANEOUS_LIBS}
          ${CORE_LIBRARIES}
          ${GUI_LIBRARIES})

if(WIN32)
  add_custom_command(
    TARGET lidar_odometry_step_1
    POST_BUILD
    COMMAND
      ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:lidar_odometry_step_1>
      $<TARGET_FILE_DIR:lidar_odometry_step_1>
    COMMAND_EXPAND_LISTS)
endif()
