cmake_minimum_required(VERSION 4.0.0)

project(multi_session_registration_step_3)

# Source files
set(SOURCES
    multi_session_registration.cpp
    multi_session_edges.cpp
    multi_session_factor_graph.cpp
    raylib_utils.cpp
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
          ${THIRDPARTY_DIRECTORY}/glm
          ${EIGEN3_INCLUDE_DIR}
          ${THIRDPARTY_DIRECTORY}/imgui
          ${THIRDPARTY_DIRECTORY}/imgui/backends
          ${THIRDPARTY_DIRECTORY}/ImGuizmo
          ${THIRDPARTY_DIRECTORY}/json/include
          ${THIRDPARTY_DIRECTORY}/portable-file-dialogs-master
          ${THIRDPARTY_DIRECTORY}/glew-cmake/include
          ${THIRDPARTY_DIRECTORY}/observation_equations/codes
          ${THIRDPARTY_DIRECTORY}/freeglut/include
          ${LASZIP_INCLUDE_DIR}/LASzip/include)

target_link_libraries(
  multi_session_registration_step_3
  # PRIVATE ${THIRDPARTY_DIRECTORY}/glew-2.2.0/lib/Release/x64/glew32s.lib
  # raylib-based like step 2; core_raylib brings in core, raylib and ScanRenderer, core_raylib_utils the
  # shared raylib replacement for <Core/utils.hpp>.
  core_raylib_utils
  core_raylib
  raylib_widgets
  imgui_raylib
  rlimgui
  imguizmo_raylib
  ${PLATFORM_LASZIP_LIB}
  ${PLATFORM_MISCELLANEOUS_LIBS}
  spdlog::spdlog)

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

# Headless step 3 (multi_session_registration_cli.cpp): propose and register edges, optimize, save -- no GUI. Session's
# point cloud layout branches on WITH_GUI, so it is built WITH_GUI=0 against core_no_gui, which exists only with the
# command line tools (as session_to_mcap).
if(BUILD_WITH_CLI_TOOLS)
  add_executable(
    multi_session_registration_step_3_cli
    multi_session_registration_cli.cpp
    multi_session_edges.cpp
    multi_session_factor_graph.cpp)

  target_compile_definitions(multi_session_registration_step_3_cli PRIVATE WITH_GUI=0)

  # imgui: headers only, for <Core/utils.hpp> (DEG_TO_RAD), which multi_session_factor_graph.cpp includes; nothing
  # of imgui is linked.
  target_include_directories(
    multi_session_registration_step_3_cli
    PRIVATE ${REPOSITORY_DIRECTORY}/core/include
            ${THIRDPARTY_DIRECTORY}
            ${THIRDPARTY_DIRECTORY}/glm
            ${THIRDPARTY_DIRECTORY}/imgui
            ${EIGEN3_INCLUDE_DIR}
            ${THIRDPARTY_DIRECTORY}/json/include
            ${THIRDPARTY_DIRECTORY}/observation_equations/codes
            ${LASZIP_INCLUDE_DIR}/LASzip/include)

  target_link_libraries(
    multi_session_registration_step_3_cli
    PRIVATE core_no_gui
            ${PLATFORM_LASZIP_LIB}
            ${PLATFORM_MISCELLANEOUS_LIBS}
            spdlog::spdlog)

  if(WIN32)
    add_custom_command(
      TARGET multi_session_registration_step_3_cli
      POST_BUILD
      COMMAND
        ${CMAKE_COMMAND} -E copy
        $<TARGET_RUNTIME_DLLS:multi_session_registration_step_3_cli>
        $<TARGET_FILE_DIR:multi_session_registration_step_3_cli>
      COMMAND_EXPAND_LISTS)
  endif()

  if (MSVC)
    target_compile_options(multi_session_registration_step_3_cli PRIVATE /bigobj)
  endif()

  hdmapping_install_app(multi_session_registration_step_3_cli)
endif()
