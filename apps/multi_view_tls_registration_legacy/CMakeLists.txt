cmake_minimum_required(VERSION 4.0.0)

# Legacy GLUT + legacy immediate-mode OpenGL build of step2, kept side by
# side with apps/multi_view_tls_registration (raylib-based since it was
# ported off GLUT) at a stakeholder's request. This is a full, independent
# copy of the app as it was before that port -- not sharing translation
# units with apps/multi_view_tls_registration -- so the two can diverge or
# be retired independently.
project(multi_view_tls_registration_step_2_legacy)

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
  multi_view_tls_registration_step_2_legacy ${SOURCES}
  )

target_compile_definitions(multi_view_tls_registration_step_2_legacy PRIVATE -DWITH_GUI=1)

target_include_directories(
          multi_view_tls_registration_step_2_legacy
          PRIVATE include
          ${REPOSITORY_DIRECTORY}/core/include
          ${THIRDPARTY_DIRECTORY}/glm
          ${EIGEN3_INCLUDE_DIR}
          ${THIRDPARTY_DIRECTORY}
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
          ${EXTERNAL_LIBRARIES_DIRECTORY}/include)

  target_link_libraries(
          multi_view_tls_registration_step_2_legacy
          PRIVATE
          WGS84toCartesian
          wgs84_do_puwg92
          unordered_dense::unordered_dense
          spdlog::spdlog
          OpenGL::GLU
          ${FREEGLUT_LIBRARY}
          ${OPENGL_gl_LIBRARY}
          ${PLATFORM_LASZIP_LIB}
          ${PLATFORM_MISCELLANEOUS_LIBS}
          ${CORE_LIBRARIES}
          ${GUI_LIBRARIES}
          PROJ::proj)

if(WIN32)
  add_custom_command(
    TARGET multi_view_tls_registration_step_2_legacy
    POST_BUILD
    COMMAND
      ${CMAKE_COMMAND} -E copy
      $<TARGET_RUNTIME_DLLS:multi_view_tls_registration_step_2_legacy>
      $<TARGET_FILE_DIR:multi_view_tls_registration_step_2_legacy>
    COMMAND_EXPAND_LISTS)
endif()

if (MSVC)
    target_compile_options(multi_view_tls_registration_step_2_legacy PRIVATE /bigobj)
endif()

install (TARGETS multi_view_tls_registration_step_2_legacy DESTINATION bin)
