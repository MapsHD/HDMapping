cmake_minimum_required(VERSION 4.0.0)

project(precision_forestry_tools)

add_executable(precision_forestry_tools 
  precision_forestry_tools.cpp 
  ${REPOSITORY_DIRECTORY}/core/src/local_shape_features.cpp
  ${REPOSITORY_DIRECTORY}/core/src/surface.cpp
  ${REPOSITORY_DIRECTORY}/core/src/hash_utils.cpp)

target_compile_definitions(precision_forestry_tools PRIVATE -DWITH_GUI=1)

target_link_libraries(
  precision_forestry_tools
  PRIVATE ${PLATFORM_LASZIP_LIB}
          ${FREEGLUT_LIBRARY}
          ${OPENGL_gl_LIBRARY}
          OpenGL::GLU
          ${GUI_LIBRARIES}
          ${PLATFORM_MISCELLANEOUS_LIBS}
          ${CORE_LIBRARIES})

target_include_directories(
  precision_forestry_tools
  PRIVATE include
          ${REPOSITORY_DIRECTORY}/core/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glm
          ${EIGEN3_INCLUDE_DIR}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui/backends
          ${EXTERNAL_LIBRARIES_DIRECTORY}/ImGuizmo
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glew-2.2.0/include
          ${FREEGLUT_INCLUDE_DIR}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/portable-file-dialogs-master
          ${LASZIP_INCLUDE_DIR}/LASzip/include)

if(WIN32)
  add_custom_command(
    TARGET precision_forestry_tools
    POST_BUILD
    COMMAND
      ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:precision_forestry_tools>
      $<TARGET_FILE_DIR:precision_forestry_tools>
    COMMAND_EXPAND_LISTS)
endif()

if (MSVC)
    target_compile_options(precision_forestry_tools PRIVATE /bigobj)
endif()

install (TARGETS precision_forestry_tools DESTINATION bin)