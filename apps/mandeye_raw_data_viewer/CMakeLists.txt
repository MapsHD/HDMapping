cmake_minimum_required(VERSION 3.15.0)

project(mandeye_raw_data_viewer)

add_executable(
  mandeye_raw_data_viewer
  mandeye_raw_data_viewer.cpp
  ../lidar_odometry_step_1/lidar_odometry_utils.cpp
)

target_include_directories(
  mandeye_raw_data_viewer
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
  mandeye_raw_data_viewer
  PRIVATE Fusion
  freeglut
  ${OPENGL_gl_LIBRARY}
  OpenGL::GLU
  ${PLATFORM_LASZIP_LIB}
  ${CORE_LIBRARIES}
  ${GUI_LIBRARIES}
  ${PLATFORM_MISCELLANEOUS_LIBS})

if(WIN32)
  add_custom_command(
    TARGET mandeye_raw_data_viewer
    POST_BUILD
    COMMAND
    ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:mandeye_raw_data_viewer>
    $<TARGET_FILE_DIR:mandeye_raw_data_viewer>
    COMMAND_EXPAND_LISTS)
endif()
