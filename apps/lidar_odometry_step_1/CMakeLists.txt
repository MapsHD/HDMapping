cmake_minimum_required(VERSION 3.15.0)

project(lidar_odometry_step_1)

add_executable(
  lidar_odometry_step_1 
  lidar_odometry_gui.cpp lidar_odometry_utils.h lidar_odometry_utils.cpp
  lidar_odometry_utils_optimizers.cpp
  lidar_odometry.cpp lidar_odometry.h
  lidar_odometry_gui_utils.cpp lidar_odometry_gui_utils.h
  toml.hpp
  toml_io.h
  toml_io.cpp
  )

target_compile_definitions(lidar_odometry_step_1 PRIVATE -DWITH_GUI=1)

target_include_directories(
  lidar_odometry_step_1
  PRIVATE include
          ${REPOSITORY_DIRECTORY}/core/include
          ${REPOSITORY_DIRECTORY}/core_hd_mapping/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glm
          ${EIGEN3_INCLUDE_DIR}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui/backends
          ${EXTERNAL_LIBRARIES_DIRECTORY}/ImGuizmo
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glew-2.2.0/include
          ${FREEGLUT_INCLUDE_DIR}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/portable-file-dialogs-master
          ${LASZIP_INCLUDE_DIR}/LASzip/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/observation_equations/codes
          ${EXTERNAL_LIBRARIES_DIRECTORY}/Fusion/Fusion)

target_link_libraries(
  lidar_odometry_step_1
  PRIVATE Fusion
          ${FREEGLUT_LIBRARY}
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

if (MSVC)
    target_compile_options(lidar_odometry_step_1 PRIVATE /bigobj)
endif()

install(TARGETS lidar_odometry_step_1 DESTINATION bin)