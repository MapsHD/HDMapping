cmake_minimum_required(VERSION 3.15.0)

project(hd_mapper)

add_executable(hd_mapper hd_mapper.cpp)
target_link_libraries(
  hd_mapper PRIVATE freeglut ${OPENGL_gl_LIBRARY} OpenGL::GLU
                    ${PLATFORM_LASZIP_LIB} ${CORE_LIBRARIES} ${GUI_LIBRARIES})
target_include_directories(
  hd_mapper
  PRIVATE include
          ${REPOSITORY_DIRECTORY}/core/include
          ${REPOSITORY_DIRECTORY}/core_hd_mapping/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glm
          ${EXTERNAL_LIBRARIES_DIRECTORY}/eigen
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui/backends
          ${EXTERNAL_LIBRARIES_DIRECTORY}/ImGuizmo
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glew-2.2.0/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/freeglut-3.2.2/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/portable-file-dialogs-master
          ${EXTERNAL_LIBRARIES_DIRECTORY}/LASzip/include)

if(WIN32)
  add_custom_command(
    TARGET hd_mapper
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:hd_mapper>
            $<TARGET_FILE_DIR:hd_mapper>
    COMMAND_EXPAND_LISTS)
endif()
