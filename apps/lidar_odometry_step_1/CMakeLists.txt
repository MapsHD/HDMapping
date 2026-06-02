cmake_minimum_required(VERSION 4.0.0)

project(lidar_odometry_step_1)

# Apply the chosen profiler backend to a target.
function(target_apply_hdmapping_profiler TARGET)
    if(HDMAPPING_PROFILER STREQUAL "TRACY")
        target_compile_definitions(${TARGET} PRIVATE
            HDMAPPING_PROFILER_TRACY TRACY_ENABLE TRACY_ON_DEMAND
            UTL_PROFILER_DISABLE)   # silence any remaining UTL_PROFILER_* calls
        target_link_libraries(${TARGET} PRIVATE TracyClient)
    elseif(HDMAPPING_PROFILER STREQUAL "UTL")
        target_compile_definitions(${TARGET} PRIVATE HDMAPPING_PROFILER_UTL)
    else()
        target_compile_definitions(${TARGET} PRIVATE UTL_PROFILER_DISABLE)
    endif()
endfunction()

# Source files
set(SOURCES
  lidar_odometry_utils.h lidar_odometry_utils.cpp
  lidar_odometry_utils_optimizers.cpp
  lidar_odometry.cpp lidar_odometry.h
  toml_io.h toml_io.cpp
  "../../core/src/utils.cpp"
)

# Windows: add resource file
if(WIN32)
    list(APPEND SOURCES "resource.rc")
endif()

add_executable(
  lidar_odometry_step_1 lidar_odometry_gui.cpp ${SOURCES}
  )

target_compile_definitions(lidar_odometry_step_1 PRIVATE WITH_GUI=1)
target_apply_hdmapping_profiler(lidar_odometry_step_1)

target_include_directories(
  lidar_odometry_step_1
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
          ${THIRDPARTY_DIRECTORY}/vqf/vqf/cpp
          ${THIRDPARTY_DIRECTORY}/Fusion/Fusion)

target_link_libraries(
          lidar_odometry_step_1
          PRIVATE 
          vqf 
          Fusion
          unordered_dense::unordered_dense
          spdlog::spdlog
          UTL::include
          OpenGL::GLU
          ${FREEGLUT_LIBRARY}
          ${OPENGL_gl_LIBRARY}
          ${PLATFORM_LASZIP_LIB}
          ${PLATFORM_MISCELLANEOUS_LIBS}
          ${CORE_LIBRARIES}
          ${GUI_LIBRARIES})

if(WIN32)
  add_custom_command(
    TARGET lidar_odometry_step_1
    POST_BUILD
    COMMAND
      ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:lidar_odometry_step_1>
      $<TARGET_FILE_DIR:lidar_odometry_step_1>
    COMMAND_EXPAND_LISTS)
endif()

if (MSVC)
    target_compile_options(lidar_odometry_step_1 PRIVATE /bigobj)
endif()

add_executable(
  drag_folder_with_mandeye_data_and_drop_here-precision_forestry drag_folder_with_mandeye_data_and_drop_here-precision_forestry.cpp ${SOURCES}
  )

target_compile_definitions(drag_folder_with_mandeye_data_and_drop_here-precision_forestry PRIVATE WITH_GUI=1)
target_apply_hdmapping_profiler(drag_folder_with_mandeye_data_and_drop_here-precision_forestry)

target_include_directories(
  drag_folder_with_mandeye_data_and_drop_here-precision_forestry
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
          ${THIRDPARTY_DIRECTORY}/vqf/vqf/cpp
          ${THIRDPARTY_DIRECTORY}/Fusion/Fusion)

target_link_libraries(
          drag_folder_with_mandeye_data_and_drop_here-precision_forestry
          PRIVATE 
          vqf 
          Fusion
          unordered_dense::unordered_dense
          spdlog::spdlog
          UTL::include
          OpenGL::GLU
          ${FREEGLUT_LIBRARY}
          ${OPENGL_gl_LIBRARY}
          ${PLATFORM_LASZIP_LIB}
          ${PLATFORM_MISCELLANEOUS_LIBS}
          ${CORE_LIBRARIES}
          ${GUI_LIBRARIES})

if(WIN32)
  add_custom_command(
    TARGET drag_folder_with_mandeye_data_and_drop_here-precision_forestry
    POST_BUILD
    COMMAND
      ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:drag_folder_with_mandeye_data_and_drop_here-precision_forestry>
      $<TARGET_FILE_DIR:drag_folder_with_mandeye_data_and_drop_here-precision_forestry>
    COMMAND_EXPAND_LISTS)
endif()

if (MSVC)
    target_compile_options(drag_folder_with_mandeye_data_and_drop_here-precision_forestry PRIVATE /bigobj)
endif()

install(TARGETS drag_folder_with_mandeye_data_and_drop_here-precision_forestry DESTINATION bin)