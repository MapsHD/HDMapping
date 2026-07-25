cmake_minimum_required(VERSION 4.0.0)

project(multi_view_tls_registration_step_2)

# Source files
#
# multi_view_tls_registration_gui.cpp is raylib-based: camera/picking/mini-
# compass/GLUT-window-and-input-loop code that used to come from
# core/src/utils.cpp (shared with the remaining GLUT apps, so it can't be
# changed) is now implemented locally in the .cpp itself, using rlgl's rl*()
# legacy-GL-emulation API and Core/raylib_render.hpp's ScanRenderer for the
# point-cloud GPU rendering that used to go through core's own legacy-GL
# PointCloud::render()/PointClouds::render().
#
# perform_experiment.cpp still #includes GLUT/glew/imgui-GLUT-backend headers,
# but never calls anything from them (verified: no glut*/GL_*/ImGui_Impl*
# symbols referenced in its body) -- those are dead includes left as-is, kept
# resolvable via the include dirs below rather than editing that file.
set(SOURCES
    multi_view_tls_registration.cpp perform_experiment.cpp
    multi_view_tls_registration_gui.cpp multi_view_tls_registration.h
    ../lidar_odometry_step_1/lidar_odometry_utils.cpp
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
          ${EIGEN3_INCLUDE_DIR}
          ${THIRDPARTY_DIRECTORY}
          ${THIRDPARTY_DIRECTORY}/glm
          # For perform_experiment.cpp's unused-but-still-#included glm/GLUT/
          # glew/imgui-GLUT-backend headers (see note above) -- not otherwise
          # needed now that rendering goes through core_raylib.
          ${THIRDPARTY_DIRECTORY}/imgui
          ${THIRDPARTY_DIRECTORY}/imgui/backends
          ${THIRDPARTY_DIRECTORY}/glew-cmake/include
          ${FREEGLUT_INCLUDE_DIR}
          ${THIRDPARTY_DIRECTORY}/json/include
          ${THIRDPARTY_DIRECTORY}/portable-file-dialogs-master
          ${LASZIP_INCLUDE_DIR}/LASzip/include
          ${THIRDPARTY_DIRECTORY}/observation_equations/codes
          ${THIRDPARTY_DIRECTORY}/vqf/vqf/cpp
          ${EXTERNAL_LIBRARIES_DIRECTORY}/include)

  target_link_libraries(
          multi_view_tls_registration_step_2
          PRIVATE
          # core_raylib brings in core + raylib transitively (PUBLIC link,
          # see core/CMakeLists.txt) -- ScanRenderer (the GPU point-cloud
          # renderer this app uses) lives there. FREEGLUT_LIBRARY is linked
          # PUBLIC by core_raylib rather than here, so it lands correctly
          # ordered relative to libcore.a on the final link line -- see the
          # comment there for why.
          core_raylib
          imgui_raylib
          rlimgui
          imguizmo_raylib
          WGS84toCartesian
          wgs84_do_puwg92
          unordered_dense::unordered_dense
          spdlog::spdlog
          ${PLATFORM_LASZIP_LIB}
          ${PLATFORM_MISCELLANEOUS_LIBS}
          PROJ::proj)

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
