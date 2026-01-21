cmake_minimum_required(VERSION 4.0.0)

project(mandeye_raw_data_viewer)

# Source files
set(SOURCES
  mandeye_raw_data_viewer.cpp
  ../lidar_odometry_step_1/toml_io.cpp
  ../lidar_odometry_step_1/lidar_odometry.cpp
  ../lidar_odometry_step_1/lidar_odometry_utils.cpp
  ../lidar_odometry_step_1/lidar_odometry_utils_optimizers.cpp
    "../../core/src/utils.cpp"
)

# Windows: add resource file
if(WIN32)
    list(APPEND SOURCES resource.rc)
endif()

add_executable(
  mandeye_raw_data_viewer ${SOURCES}
)

target_include_directories(
  mandeye_raw_data_viewer
  PRIVATE include
  ${REPOSITORY_DIRECTORY}/core/include
  ${REPOSITORY_DIRECTORY}/core_hd_mapping/include
  ${THIRDPARTY_DIRECTORY}
  ${THIRDPARTY_DIRECTORY}/glm
  ${EIGEN3_INCLUDE_DIR}
  ${THIRDPARTY_DIRECTORY}/tomlplusplus/include
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
  mandeye_raw_data_viewer
  PRIVATE Fusion
  unordered_dense::unordered_dense
  ${FREEGLUT_LIBRARY}
  ${OPENGL_gl_LIBRARY}
  OpenGL::GLU
  ${PLATFORM_LASZIP_LIB}
  ${CORE_LIBRARIES}
  ${GUI_LIBRARIES}
  ${PLATFORM_MISCELLANEOUS_LIBS}
  ${OpenCV_LIBS})

if(WIN32)
  add_custom_command(
    TARGET mandeye_raw_data_viewer
    POST_BUILD
    COMMAND
    ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:mandeye_raw_data_viewer>
    $<TARGET_FILE_DIR:mandeye_raw_data_viewer>
    COMMAND_EXPAND_LISTS)
endif()

if (MSVC)
    target_compile_options(mandeye_raw_data_viewer PRIVATE /bigobj)
endif()

install(TARGETS mandeye_raw_data_viewer DESTINATION bin)