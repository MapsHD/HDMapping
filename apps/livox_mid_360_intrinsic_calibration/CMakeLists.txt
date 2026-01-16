cmake_minimum_required(VERSION 4.0.0)

project(livox_mid_360_intrinsic_calibration)

add_executable(
  livox_mid_360_intrinsic_calibration
  livox_mid_360_intrinsic_calibration.cpp
  ../lidar_odometry_step_1/lidar_odometry_utils.cpp
  ../lidar_odometry_step_1/lidar_odometry_utils_optimizers.cpp
)

target_include_directories(
  livox_mid_360_intrinsic_calibration
  PRIVATE include
  ${REPOSITORY_DIRECTORY}/core/include
  ${REPOSITORY_DIRECTORY}/core_hd_mapping/include
  ${THIRDPARTY_DIRECTORY}
  ${THIRDPARTY_DIRECTORY}/glm
  ${EIGEN3_INCLUDE_DIR}
  ${THIRDPARTY_DIRECTORY}/imgui
  ${THIRDPARTY_DIRECTORY}/imgui/backends
  ${THIRDPARTY_DIRECTORY}/ImGuizmo
  ${THIRDPARTY_DIRECTORY}/glew-cmake/include
  ${FREEGLUT_INCLUDE_DIR}
  ${THIRDPARTY_DIRECTORY}/json/include
  ${THIRDPARTY_DIRECTORY}/portable-file-dialogs-master
  ${LASZIP_INCLUDE_DIR}/LASzip/include
  ${THIRDPARTY_DIRECTORY}/observation_equations/codes
  ${THIRDPARTY_DIRECTORY}/Fusion/Fusion)

target_link_libraries(
  livox_mid_360_intrinsic_calibration
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
    TARGET livox_mid_360_intrinsic_calibration
    POST_BUILD
    COMMAND
    ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:livox_mid_360_intrinsic_calibration>
    $<TARGET_FILE_DIR:livox_mid_360_intrinsic_calibration>
    COMMAND_EXPAND_LISTS)
endif()

if (MSVC)
    target_compile_options(livox_mid_360_intrinsic_calibration PRIVATE /bigobj)
endif()

install(TARGETS livox_mid_360_intrinsic_calibration DESTINATION bin)