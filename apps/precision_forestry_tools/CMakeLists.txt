cmake_minimum_required(VERSION 3.15.0)

project(precision_forestry_tools)

add_executable(precision_forestry_tools
  precision_forestry_tools.cpp)
target_link_libraries(
  precision_forestry_tools
  PRIVATE ${PLATFORM_LASZIP_LIB} freeglut ${OPENGL_gl_LIBRARY} OpenGL::GLU
  ${GUI_LIBRARIES} ${PLATFORM_MISCELLANEOUS_LIBS} ${CORE_LIBRARIES})
target_include_directories(
  precision_forestry_tools
  PRIVATE include
  ${REPOSITORY_DIRECTORY}/core/include
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
    TARGET precision_forestry_tools
    POST_BUILD
    COMMAND
    ${CMAKE_COMMAND} -E copy
    $<TARGET_RUNTIME_DLLS:precision_forestry_tools>
    $<TARGET_FILE_DIR:precision_forestry_tools>
    COMMAND_EXPAND_LISTS)
endif()
