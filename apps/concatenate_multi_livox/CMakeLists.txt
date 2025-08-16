cmake_minimum_required(VERSION 3.15.0)

project(concatenate_multi_livox)

add_executable(concatenate_multi_livox concatenate_multi_livox.cpp ../lidar_odometry_step_1/lidar_odometry_utils.cpp)

target_include_directories(
        concatenate_multi_livox
  PRIVATE include
          ${REPOSITORY_DIRECTORY}/core/include
          ${REPOSITORY_DIRECTORY}/core_hd_mapping/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glm
          ${EIGEN3_INCLUDE_DIR}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui/backends
          ${EXTERNAL_LIBRARIES_DIRECTORY}/ImGuizmo
          ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/portable-file-dialogs-master
          ${LASZIP_INCLUDE_DIR}/LASzip/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glew-2.2.0/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/observation_equations/codes
          ${EXTERNAL_LIBRARIES_DIRECTORY}/Fusion/Fusion
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

install (TARGETS concatenate_multi_livox DESTINATION bin)