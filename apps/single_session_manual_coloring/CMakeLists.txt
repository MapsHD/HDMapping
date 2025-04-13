cmake_minimum_required(VERSION 3.15.0)

project(single_session_manual_coloring)

add_executable(single_session_manual_coloring single_session_manual_coloring.cpp)

target_include_directories(
  single_session_manual_coloring
  PRIVATE include
          ${REPOSITORY_DIRECTORY}/core/include
          ${REPOSITORY_DIRECTORY}/core_hd_mapping/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glm
          ${EIGEN3_INCLUDE_DIR}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui/backends
          ${EXTERNAL_LIBRARIES_DIRECTORY}/ImGuizmo
          ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/portable-file-dialogs-master
          ${LASZIP_INCLUDE_DIR}/LASzip/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glew-2.2.0/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/observation_equations/codes
          ${FREEGLUT_INCLUDE_DIR})

target_link_libraries(
  single_session_manual_coloring
  PRIVATE ${FREEGLUT_LIBRARY}
          ${OPENGL_gl_LIBRARY}
          OpenGL::GLU
          ${PLATFORM_LASZIP_LIB}
          ${PLATFORM_MISCELLANEOUS_LIBS}
          ${CORE_LIBRARIES}
          ${GUI_LIBRARIES})

if(WIN32)
  add_custom_command(
    TARGET single_session_manual_coloring
    POST_BUILD
    COMMAND
      ${CMAKE_COMMAND} -E copy
      $<TARGET_RUNTIME_DLLS:single_session_manual_coloring>
      $<TARGET_FILE_DIR:single_session_manual_coloring>
    COMMAND_EXPAND_LISTS)
endif()

install(TARGETS single_session_manual_coloring DESTINATION bin)