cmake_minimum_required(VERSION 3.25.0)

project(quick_start_demo)

add_executable(
  quick_start_demo
  quick_start_demo.cpp
  ../lidar_odometry_step_1/lidar_odometry_utils.cpp
  ../lidar_odometry_step_1/lidar_odometry_utils_optimizers.cpp
)

target_include_directories(
  quick_start_demo
  PRIVATE include
  ${REPOSITORY_DIRECTORY}/core/include
  ${REPOSITORY_DIRECTORY}/core_hd_mapping/include
  ${EXTERNAL_LIBRARIES_DIRECTORY}
  ${EXTERNAL_LIBRARIES_DIRECTORY}/glm
  ${EIGEN3_INCLUDE_DIR}
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
  quick_start_demo
  PRIVATE Fusion
  ${FREEGLUT_LIBRARY}
  ${OPENGL_gl_LIBRARY}
  OpenGL::GLU
  ${PLATFORM_LASZIP_LIB}
  ${CORE_LIBRARIES}
  ${GUI_LIBRARIES}
  ${PLATFORM_MISCELLANEOUS_LIBS})

if(WIN32)
  add_custom_command(
    TARGET quick_start_demo
    POST_BUILD
    COMMAND
    ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:quick_start_demo>
    $<TARGET_FILE_DIR:quick_start_demo>
    COMMAND_EXPAND_LISTS)
endif()

if (MSVC)
    target_compile_options(quick_start_demo PRIVATE /bigobj)
endif()

install(TARGETS quick_start_demo DESTINATION bin)