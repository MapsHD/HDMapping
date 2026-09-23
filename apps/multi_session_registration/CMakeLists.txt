cmake_minimum_required(VERSION 4.0.0)

project(multi_session_registration_step_3)

# raylib-based, like step 2 (apps/multi_view_tls_registration): no core/src/utils.cpp,
# GLUT or legacy OpenGL. The GLUT build is kept as apps/multi_session_registration_legacy.
# multi_session_registration_no_gui.cpp is not part of this app; pybind builds it.
set(SOURCES
    multi_session_registration.cpp
    multi_session_factor_graph.cpp
)

# Windows: add resource file
if(WIN32)
    list(APPEND SOURCES "resource.rc")
endif()

add_executable(
  multi_session_registration_step_3 ${SOURCES}
  )

target_compile_definitions(multi_session_registration_step_3 PRIVATE -DWITH_GUI=1)

target_include_directories(
  multi_session_registration_step_3
  PRIVATE include
          ${REPOSITORY_DIRECTORY}/core/include
          ${REPOSITORY_DIRECTORY}/core_hd_mapping/include
          ${THIRDPARTY_DIRECTORY}
          ${EIGEN3_INCLUDE_DIR}
          ${THIRDPARTY_DIRECTORY}/json/include
          ${THIRDPARTY_DIRECTORY}/portable-file-dialogs-master
          ${THIRDPARTY_DIRECTORY}/observation_equations/codes
          ${LASZIP_INCLUDE_DIR}/LASzip/include)

target_link_libraries(
  multi_session_registration_step_3
  PRIVATE
  # core_raylib brings in core + raylib (and links FREEGLUT after core for libcore.a's
  # remaining glutBitmap* references) -- see core/CMakeLists.txt.
  core_raylib
  raylib_widgets
  imgui_raylib
  rlimgui
  imguizmo_raylib
  spdlog::spdlog
  ${PLATFORM_LASZIP_LIB}
  ${PLATFORM_MISCELLANEOUS_LIBS})

if(WIN32)
  add_custom_command(
    TARGET multi_session_registration_step_3
    POST_BUILD
    COMMAND
      ${CMAKE_COMMAND} -E copy
      $<TARGET_RUNTIME_DLLS:multi_session_registration_step_3>
      $<TARGET_FILE_DIR:multi_session_registration_step_3>
    COMMAND_EXPAND_LISTS)
endif()

if (MSVC)
    target_compile_options(multi_session_registration_step_3 PRIVATE /bigobj)
endif()

hdmapping_install_app(multi_session_registration_step_3)
