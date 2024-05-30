cmake_minimum_required(VERSION 3.10.0)

project(multi_view_tls_registration_step_2)

add_executable(multi_view_tls_registration_step_2
               multi_view_tls_registration.cpp perform_experiment.cpp)
target_link_libraries(
  multi_view_tls_registration_step_2 PRIVATE freeglut laszip3 ${CORE_LIBRARIES}
                                             ${GUI_LIBRARIES})
target_include_directories(
  multi_view_tls_registration_step_2
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
