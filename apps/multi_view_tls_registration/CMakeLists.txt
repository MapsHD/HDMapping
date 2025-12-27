cmake_minimum_required(VERSION 3.25.0)

project(multi_view_tls_registration_step_2)

# Source files
set(SOURCES
    multi_view_tls_registration.cpp perform_experiment.cpp 
    multi_view_tls_registration_gui.cpp multi_view_tls_registration.h
    ../lidar_odometry_step_1/lidar_odometry_utils.cpp
    "../../core/src/utils.cpp"
)

# Windows: add resource file
if(WIN32)
    list(APPEND SOURCES "resource.rc")
endif()

add_executable(
  multi_view_tls_registration_step_2 ${SOURCES}
  )

target_compile_definitions(multi_view_tls_registration_step_2 PRIVATE -DWITH_GUI=1)

target_include_directories(
          multi_view_tls_registration_step_2
          PRIVATE include
          ${REPOSITORY_DIRECTORY}/core/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glm
          ${EIGEN3_INCLUDE_DIR}
          ${EXTERNAL_LIBRARIES_DIRECTORY}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui/backends
          ${EXTERNAL_LIBRARIES_DIRECTORY}/ImGuizmo
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glew-cmake/include
          ${FREEGLUT_INCLUDE_DIR}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/portable-file-dialogs-master
          ${LASZIP_INCLUDE_DIR}/LASzip/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/observation_equations/codes
          ${EXTERNAL_LIBRARIES_DIRECTORY}/Fusion/Fusion)

  target_link_libraries(
          multi_view_tls_registration_step_2
          # PRIVATE ${EXTERNAL_LIBRARIES_DIRECTORY}/glew-2.2.0/lib/Release/x64/glew32s.lib
          PRIVATE ${FREEGLUT_LIBRARY}
          ${OPENGL_gl_LIBRARY}
          OpenGL::GLU
          ${PLATFORM_LASZIP_LIB}
          ${PLATFORM_MISCELLANEOUS_LIBS}
          ${CORE_LIBRARIES}
          ${GUI_LIBRARIES})

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

if (MSVC)
    target_compile_options(multi_view_tls_registration_step_2 PRIVATE /bigobj)
endif()

install (TARGETS multi_view_tls_registration_step_2 DESTINATION bin)