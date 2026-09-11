cmake_minimum_required(VERSION 4.0.0)

project(hd_mapper)

add_executable(hd_mapper
  hd_mapper.cpp
  laz_wrapper.cpp
  odo_with_gnss_fusion.cpp
  project_settings.cpp
  roi_exporter.cpp
  single_trajectory_viewer.cpp
  ${EXTERNAL_LIBRARIES_DIRECTORY}/src/plycpp.cpp)

target_link_libraries(
  hd_mapper
  PRIVATE spdlog::spdlog
          OpenGL::GLU
          ${FREEGLUT_LIBRARY}
          ${OPENGL_gl_LIBRARY}
          ${PLATFORM_LASZIP_LIB}
          ${PLATFORM_MISCELLANEOUS_LIBS}
          ${CORE_LIBRARIES}
          ${GUI_LIBRARIES})

target_include_directories(
  hd_mapper
  PRIVATE include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/include
          ${REPOSITORY_DIRECTORY}/core/include
          ${EIGEN3_INCLUDE_DIR}
          ${FREEGLUT_INCLUDE_DIR}
          ${THIRDPARTY_DIRECTORY}/glm
          ${THIRDPARTY_DIRECTORY}/imgui
          ${THIRDPARTY_DIRECTORY}/imgui/backends
          ${THIRDPARTY_DIRECTORY}/ImGuizmo
          ${THIRDPARTY_DIRECTORY}/glew-cmake/include
          ${THIRDPARTY_DIRECTORY}/json/include
          ${THIRDPARTY_DIRECTORY}/portable-file-dialogs-master
          ${LASZIP_INCLUDE_DIR}/LASzip/include
          ${THIRDPARTY_DIRECTORY}/observation_equations/codes)

if(WIN32)
  add_custom_command(
    TARGET hd_mapper
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:hd_mapper>
            $<TARGET_FILE_DIR:hd_mapper>
    COMMAND_EXPAND_LISTS)
endif()

hdmapping_install_app(hd_mapper)