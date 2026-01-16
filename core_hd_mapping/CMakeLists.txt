cmake_minimum_required(VERSION 4.0.0)

project(core-hd-mapping)

add_library(core-hd-mapping STATIC
    src/laz_wrapper.cpp
    src/odo_with_gnss_fusion.cpp
    src/project_settings.cpp
    src/roi_exporter.cpp
    src/single_trajectory_viewer.cpp
)

target_precompile_headers(core-hd-mapping
    PRIVATE include/pch/pch.h
)

target_include_directories(core-hd-mapping
    PRIVATE
        include
        ${EIGEN3_INCLUDE_DIR}
        ${THIRDPARTY_DIRECTORY}/freeglut/include
        ${THIRDPARTY_DIRECTORY}/ImGuizmo
        ${THIRDPARTY_DIRECTORY}/imgui
        ${THIRDPARTY_DIRECTORY}/imgui/backends
        ${THIRDPARTY_DIRECTORY}/json/include
        ${THIRDPARTY_DIRECTORY}/observation_equations/codes
        ${THIRDPARTY_DIRECTORY}/portable-file-dialogs-master
        ${LASZIP_INCLUDE_DIR}/LASzip/include
        ${REPOSITORY_DIRECTORY}/core/include
        ${EXTERNAL_LIBRARIES_DIRECTORY}/include
)

target_link_libraries(core-hd-mapping
    PRIVATE
        core
        ${PLATFORM_MISCELLANEOUS_LIBS}
        Threads::Threads
)
