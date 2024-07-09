cmake_minimum_required(VERSION 3.15.0)

project(mandeye_with_gopro_max_manual_coloring)

add_executable(mandeye_with_gopro_max_manual_coloring manual_color.cpp)
target_include_directories(
  mandeye_with_gopro_max_manual_coloring
  PRIVATE include
          ${REPOSITORY_DIRECTORY}/core/include
          ${REPOSITORY_DIRECTORY}/core_hd_mapping/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glm
          ${EXTERNAL_LIBRARIES_DIRECTORY}/eigen
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui/backends
          ${EXTERNAL_LIBRARIES_DIRECTORY}/ImGuizmo
          ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/portable-file-dialogs-master
          ${EXTERNAL_LIBRARIES_DIRECTORY}/LASzip/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glew-2.2.0/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/observation_equations/codes
          ${EXTERNAL_LIBRARIES_DIRECTORY}/freeglut-3.2.2/include)
target_link_libraries(
  mandeye_with_gopro_max_manual_coloring
  PRIVATE freeglut ${OPENGL_gl_LIBRARY} OpenGL::GLU ${PLATFORM_LASZIP_LIB}
          ${CORE_LIBRARIES} ${GUI_LIBRARIES})

if(WIN32)
  add_custom_command(
    TARGET mandeye_with_gopro_max_manual_coloring
    POST_BUILD
    COMMAND
      ${CMAKE_COMMAND} -E copy
      $<TARGET_RUNTIME_DLLS:mandeye_with_gopro_max_manual_coloring>
      $<TARGET_FILE_DIR:mandeye_with_gopro_max_manual_coloring>
    COMMAND_EXPAND_LISTS)
endif()
