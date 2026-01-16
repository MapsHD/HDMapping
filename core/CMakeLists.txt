cmake_minimum_required(VERSION 4.0.0)

project(core)

set(CORE_BASE_SOURCES
    src/color_las_loader.cpp
    src/control_points.cpp
    src/gnss.cpp
    src/ground_control_points.cpp
    src/hash_utils.cpp
    src/icp.cpp
    src/ndt.cpp
    src/nmea.cpp
    src/optimization_point_to_point_source_to_target.cpp
    src/optimize_distance_point_to_plane_source_to_target.cpp
    src/optimize_plane_to_plane_source_to_target.cpp
    src/optimize_point_to_plane_source_to_target.cpp
    src/optimize_point_to_projection_onto_plane_source_to_target.cpp
    src/pair_wise_iterative_closest_point.cpp
    src/point_cloud.cpp
    src/point_clouds.cpp
    src/pose_graph_loop_closure.cpp
    src/pose_graph_slam.cpp
    src/registration_plane_feature.cpp
    src/session.cpp
    ${EXTERNAL_LIBRARIES_DIRECTORY}/src/wgs84_do_puwg92.cc
    ${EXTERNAL_LIBRARIES_DIRECTORY}/src/plycpp.cpp
)

set(CORE_GUI_SOURCES
    src/manual_pose_graph_loop_closure.cpp
    src/observation_picking.cpp
    src/pfd_wrapper.cpp
)

function(add_core_target target_name with_gui)
    if(${with_gui})
        set(SOURCES ${CORE_BASE_SOURCES} ${CORE_GUI_SOURCES})
        set(DEFINES WITH_GUI=1)
    else()
        set(SOURCES ${CORE_BASE_SOURCES})
        set(DEFINES WITH_GUI=0)
    endif()

    add_library(${target_name} STATIC ${SOURCES})
    target_compile_definitions(${target_name} PRIVATE ${DEFINES})
    target_link_libraries(${target_name} PRIVATE ${PLATFORM_LASZIP_LIB} ${PLATFORM_MISCELLANEOUS_LIBS})
    target_include_directories(${target_name} PRIVATE
        include
        ${EIGEN3_INCLUDE_DIR}
        ${LASZIP_INCLUDE_DIR}/LASzip/include
        ${THIRDPARTY_DIRECTORY}/json/include
        ${THIRDPARTY_DIRECTORY}/observation_equations/codes
        ${EXTERNAL_LIBRARIES_DIRECTORY}/include
    )

    if(${with_gui})
        target_include_directories(${target_name} PRIVATE
            ${THIRDPARTY_DIRECTORY}/imgui
            ${THIRDPARTY_DIRECTORY}/imgui/backends
            ${THIRDPARTY_DIRECTORY}/ImGuizmo
            ${THIRDPARTY_DIRECTORY}/freeglut/include
            ${THIRDPARTY_DIRECTORY}/portable-file-dialogs-master
        )
    endif()
endfunction()

add_core_target(core_no_gui FALSE)
add_core_target(core TRUE)

target_precompile_headers(core_no_gui
    PRIVATE include/pch/pch.h
)

target_precompile_headers(core_no_gui
    PRIVATE include/pch/pch.h
)

set_target_properties(core_no_gui PROPERTIES
    POSITION_INDEPENDENT_CODE ON)