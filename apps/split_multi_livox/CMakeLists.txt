cmake_minimum_required(VERSION 4.0.0)

project(split_multi_livox)

add_executable(split_multi_livox split_multi_livox.cpp)

target_include_directories(
  split_multi_livox
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
          ${FREEGLUT_INCLUDE_DIR})

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

if (MSVC)
    target_compile_options(split_multi_livox PRIVATE /bigobj)
endif()

install (TARGETS split_multi_livox DESTINATION bin)