cmake_minimum_required(VERSION 3.15.0)

project(split_multi_livox)

add_executable(split_multi_livox split_multi_livox.cpp)

target_include_directories(
  split_multi_livox
  PRIVATE include
          ${REPOSITORY_DIRECTORY}/core/include
          ${REPOSITORY_DIRECTORY}/core_hd_mapping/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glm
          ${EXTERNAL_LIBRARIES_DIRECTORY}/eigen
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui/backends
          ${EXTERNAL_LIBRARIES_DIRECTORY}/ImGuizmo
          ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/portable-file-dialogs-master
          ${EXTERNAL_LIBRARIES_DIRECTORY}/LASzip/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glew-2.2.0/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/observation_equations/codes
          ${EXTERNAL_LIBRARIES_DIRECTORY}/freeglut-3.2.2/include)

target_link_libraries(split_multi_livox PRIVATE core ${PLATFORM_LASZIP_LIB}
                                                ${PLATFORM_MISCELLANEOUS_LIBS})

if(WIN32)
  add_custom_command(
    TARGET split_multi_livox
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:split_multi_livox>
            $<TARGET_FILE_DIR:split_multi_livox>
    COMMAND_EXPAND_LISTS)
endif()
