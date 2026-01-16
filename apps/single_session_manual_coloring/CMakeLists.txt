cmake_minimum_required(VERSION 4.0.0)

project(single_session_manual_coloring)

add_executable(single_session_manual_coloring single_session_manual_coloring.cpp)

target_include_directories(
  single_session_manual_coloring
  PRIVATE include
          ${REPOSITORY_DIRECTORY}/core/include
          ${REPOSITORY_DIRECTORY}/core_hd_mapping/include
          ${THIRDPARTY_DIRECTORY}
          ${THIRDPARTY_DIRECTORY}/glm
          ${EIGEN3_INCLUDE_DIR}
          ${THIRDPARTY_DIRECTORY}/imgui
          ${THIRDPARTY_DIRECTORY}/imgui/backends
          ${THIRDPARTY_DIRECTORY}/ImGuizmo
          ${THIRDPARTY_DIRECTORY}/json/include
          ${THIRDPARTY_DIRECTORY}/portable-file-dialogs-master
          ${LASZIP_INCLUDE_DIR}/LASzip/include
          ${THIRDPARTY_DIRECTORY}/glew-cmake/include
          ${THIRDPARTY_DIRECTORY}/observation_equations/codes
          ${FREEGLUT_INCLUDE_DIR})

target_compile_definitions(single_session_manual_coloring PRIVATE WITH_GUI=1)

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

if (MSVC)
    target_compile_options(single_session_manual_coloring PRIVATE /bigobj)
endif()

install(TARGETS single_session_manual_coloring DESTINATION bin)