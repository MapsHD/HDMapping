cmake_minimum_required(VERSION 3.15.0)

project(core)

# Source files and headers
set(CORE_SOURCE_FILES
    src/color_las_loader.cpp
    src/gnss.cpp
    src/icp.cpp
    src/ndt.cpp
    src/nns.cpp
    src/optimization_point_to_point_source_to_target.cpp
    src/optimize_distance_point_to_plane_source_to_target.cpp
    src/optimize_plane_to_plane_source_to_target.cpp
    src/optimize_point_to_plane_source_to_target.cpp
    src/optimize_point_to_projection_onto_plane_source_to_target.cpp
    src/pcl_wrapper.cpp
    src/plycpp.cpp
    src/point_cloud.cpp
    src/point_clouds.cpp
    src/pose_graph_slam.cpp
    src/registration_plane_feature.cpp
    src/session.cpp
    src/wgs84_do_puwg92.cc
    src/pair_wise_iterative_closest_point.cpp
    src/hash_utils.cpp
    src/minmea.c
)

set(CORE_SOURCE_FILES_GUI
    src/pfd_wrapper.cpp
    src/ground_control_points.cpp
    src/control_points.cpp
    src/manual_pose_graph_loop_closure.cpp
    src/observation_picking.cpp
)

set(CORE_INCLUDE_FILES
    include/color_las_loader.h
    include/gnss.h
    include/icp.h
    include/m_estimators.h
    include/pose_graph_loop_closure.h
    include/ndt.h
    include/nns.h
    include/pcl_wrapper.h
    include/plycpp.h
    include/point_cloud.h
    include/point_clouds.h
    include/pose_graph_slam.h
    include/registration_plane_feature.h
    include/session.h
    include/structures.h
    include/transformations.h
    include/wgs84_do_puwg92.h
    include/WGS84toCartesian.hpp
    include/minmea.h
)

set(CORE_INCLUDE_FILES_GUI
    include/manual_pose_graph_loop_closure.h
    include/observation_picking.h
    include/pfd_wrapper.hpp
)

# GUI-independent target
add_library(core_no_gui STATIC ${CORE_SOURCE_FILES} ${CORE_INCLUDE_FILES})
target_compile_definitions(core_no_gui PRIVATE WITH_GUI=0)
target_link_libraries(core_no_gui PRIVATE ${PLATFORM_LASZIP_LIB} ${PLATFORM_MISCELLANEOUS_LIBS})
target_include_directories(core_no_gui PRIVATE 
    include
    ${EIGEN3_INCLUDE_DIR}
    ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
    ${LASZIP_INCLUDE_DIR}/LASzip/include
    ${EXTERNAL_LIBRARIES_DIRECTORY}/observation_equations/codes
)

# GUI target
add_library(core STATIC ${CORE_SOURCE_FILES} ${CORE_SOURCE_FILES_GUI} ${CORE_INCLUDE_FILES} ${CORE_INCLUDE_FILES_GUI})
target_compile_definitions(core PRIVATE WITH_GUI=1)
target_link_libraries(core PRIVATE ${PLATFORM_LASZIP_LIB} ${PLATFORM_MISCELLANEOUS_LIBS})
target_include_directories(core PRIVATE 
    include
    ${EIGEN3_INCLUDE_DIR}
    ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
    ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui
    ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui/backends
    ${EXTERNAL_LIBRARIES_DIRECTORY}/ImGuizmo
    ${LASZIP_INCLUDE_DIR}/LASzip/include
    ${EXTERNAL_LIBRARIES_DIRECTORY}/freeglut/include
    ${EXTERNAL_LIBRARIES_DIRECTORY}/observation_equations/codes
    ${EXTERNAL_LIBRARIES_DIRECTORY}/portable-file-dialogs-master
)

