cmake_minimum_required(VERSION 3.15.0)

project(multi_view_tls_registration_step_2)

add_executable(multi_view_tls_registration_step_2
               multi_view_tls_registration.cpp perform_experiment.cpp)

target_link_libraries(
  multi_view_tls_registration_step_2
  PRIVATE freeglut
          ${OPENGL_gl_LIBRARY}
          OpenGL::GLU
          ${PLATFORM_LASZIP_LIB}
          ${PLATFORM_MISCELLANEOUS_LIBS}
          ${CORE_LIBRARIES}
          ${GUI_LIBRARIES})

target_include_directories(
  multi_view_tls_registration_step_2
  PRIVATE include
          ${REPOSITORY_DIRECTORY}/core/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glm
          ${EXTERNAL_LIBRARIES_DIRECTORY}/eigen
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui/backends
          ${EXTERNAL_LIBRARIES_DIRECTORY}/ImGuizmo
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glew-2.2.0/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/freeglut-3.2.2/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/portable-file-dialogs-master
          ${EXTERNAL_LIBRARIES_DIRECTORY}/LASzip/include)

if(WIN32)
  add_custom_command(
    TARGET multi_view_tls_registration_step_2
    POST_BUILD
    COMMAND
      ${CMAKE_COMMAND} -E copy
      $<TARGET_RUNTIME_DLLS:multi_view_tls_registration_step_2>
      $<TARGET_FILE_DIR:multi_view_tls_registration_step_2>
    COMMAND_EXPAND_LISTS)
endif()
