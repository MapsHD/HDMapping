cmake_minimum_required(VERSION 4.0.0)

project(session_to_session_rigid_registration)

# Source files
set(SOURCES
    main.cpp
)


add_executable(
  session_to_session_rigid_registration ${SOURCES}
  )

target_compile_definitions(session_to_session_rigid_registration PRIVATE -DWITH_GUI=0)

target_include_directories(
  session_to_session_rigid_registration
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
  session_to_session_rigid_registration
  # PRIVATE ${THIRDPARTY_DIRECTORY}/glew-2.2.0/lib/Release/x64/glew32s.lib
  # raylib-based like step 2; core_raylib brings in core, raylib and ScanRenderer.
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
    TARGET session_to_session_rigid_registration
    POST_BUILD
    COMMAND
      ${CMAKE_COMMAND} -E copy
      $<TARGET_RUNTIME_DLLS:session_to_session_rigid_registration>
      $<TARGET_FILE_DIR:session_to_session_rigid_registration>
    COMMAND_EXPAND_LISTS)
endif()

if (MSVC)
    target_compile_options(session_to_session_rigid_registration PRIVATE /bigobj)
endif()

hdmapping_install_app(session_to_session_rigid_registration)