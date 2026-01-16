cmake_minimum_required(VERSION 4.0.0)

project(concatenate_multi_livox)

add_executable(concatenate_multi_livox concatenate_multi_livox.cpp ../lidar_odometry_step_1/lidar_odometry_utils.cpp)

target_include_directories(
        concatenate_multi_livox
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
          ${LASZIP_INCLUDE_DIR}/LASzip/include
          ${THIRDPARTY_DIRECTORY}/glew-cmake/include
          ${THIRDPARTY_DIRECTORY}/observation_equations/codes
          ${THIRDPARTY_DIRECTORY}/Fusion/Fusion
          ${FREEGLUT_INCLUDE_DIR})

target_link_libraries(concatenate_multi_livox PRIVATE core ${PLATFORM_LASZIP_LIB}
                                                ${PLATFORM_MISCELLANEOUS_LIBS})

if(WIN32)
  add_custom_command(
    TARGET concatenate_multi_livox
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:concatenate_multi_livox>
            $<TARGET_FILE_DIR:concatenate_multi_livox>
    COMMAND_EXPAND_LISTS)
endif()

if (MSVC)
    target_compile_options(concatenate_multi_livox PRIVATE /bigobj)
endif()

install (TARGETS concatenate_multi_livox DESTINATION bin)