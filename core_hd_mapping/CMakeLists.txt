cmake_minimum_required(VERSION 3.15.0)

project(core-hd-mapping)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

set(CORE_HD_MAPPING_SOURCE_FILES
    src/laz_wrapper.cpp src/odo_with_gnss_fusion.cpp src/project_settings.cpp
    src/roi_exporter.cpp src/single_trajectory_viewer.cpp)

set(CORE_HD_MAPPING_INCLUDE_FILES
    include/laz_wrapper.h include/odo_with_gnss_fusion.h
    include/project_settings.h include/roi_exporter.h
    include/single_trajectory_viewer.h)

set(CORE_HD_MAPPING_FILES ${CORE_HD_MAPPING_SOURCE_FILES}
                          ${CORE_HD_MAPPING_INCLUDE_FILES})

add_library(core-hd-mapping STATIC ${CORE_HD_MAPPING_FILES})

target_link_libraries(core-hd-mapping PRIVATE core ${PLATFORM_MISCELLANEOUS_LIBS} Threads::Threads)

target_include_directories(
  core-hd-mapping
  PRIVATE include
          ${REPOSITORY_DIRECTORY}/core/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui/backends
          ${EXTERNAL_LIBRARIES_DIRECTORY}/ImGuizmo
          ${EXTERNAL_LIBRARIES_DIRECTORY}/observation_equations/codes
          ${EIGEN3_INCLUDE_DIR}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/portable-file-dialogs-master
          ${EXTERNAL_LIBRARIES_DIRECTORY}/freeglut/include
          ${LASZIP_INCLUDE_DIR}/LASzip/include)
