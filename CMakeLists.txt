cmake_minimum_required(VERSION 3.1.0)
project(terrestrial-laser-scanning-registration)
set(CMAKE_CXX_STANDARD 20)

if (WIN32)
	add_compile_definitions(_USE_MATH_DEFINES)
	include_directories(3rdparty/eigen)
else()
	find_package( Eigen3 REQUIRED)
	if(EIGEN3_FOUND)
		message("EIGEN3 found")
		message ("EIGEN3_INCLUDE_DIR: " ${EIGEN3_INCLUDE_DIR})
		message ("EIGEN3_INCLUDE_DIRS: " ${EIGEN3_INCLUDE_DIRS})
		message ("Eigen3_DIR: " ${Eigen3_DIR})
		include_directories(	
			${EIGEN3_INCLUDE_DIR} 
			${EIGEN3_INCLUDE_DIRS} 
			${Eigen3_DIR})
	endif()
	
	find_package(GLEW REQUIRED)
	include_directories(${GLEW_INCLUDE_DIRS})
	link_libraries(${GLEW_LIBRARIES})

	find_package(OpenGL REQUIRED) 
	find_package(GLUT REQUIRED) 
	include_directories(library ${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})

	set(CMAKE_THREAD_LIBS_INIT "-lpthread")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
	set(CMAKE_HAVE_THREADS_LIBRARY 1)
	set(CMAKE_USE_WIN32_THREADS_INIT 0)
	set(CMAKE_USE_PTHREADS_INIT 1)
	set(THREADS_PREFER_PTHREAD_FLAG ON)

	add_definitions("-DWITH_MANIF")
endif()

find_package(PCL)
if(PCL_FOUND)
    message("PCL found")
    message ("PCL_LIBRARY_DIRS: " ${PCL_LIBRARY_DIRS})
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    add_definitions("-DWITH_PCL")
endif()

find_package(GTSAM)
if (GTSAM_FOUND)
    message("GTSAM found")
    message("GTSAM include dirs" ${GTSAM_INCLUDE_DIR})
    add_definitions("-DWITH_GTSAM")
endif()

include_directories(
3rdparty/imgui 
3rdparty/imgui/backends
3rdparty/ImGuizmo
3rdparty/freeglut/include
3rdparty/glew-2.2.0/include/GL
3rdparty/glm
core/include
3rdparty/observation_equations/codes
3rdparty/portable-file-dialogs-master
3rdparty/json/include/nlohmann
3rdparty/json/include
core/hd_mapping_include
3rdparty/manif/include
3rdparty/manif/external/tl
)

add_subdirectory(3rdparty/LASzip/)
include_directories(
3rdparty/LASzip/include
)

set(IMGUI_SRCS
3rdparty/imgui/imgui.cpp 
3rdparty/imgui/imgui_demo.cpp 
3rdparty/imgui/imgui_draw.cpp 
3rdparty/imgui/imgui_tables.cpp 
3rdparty/imgui/imgui_widgets.cpp
3rdparty/imgui/backends/imgui_impl_glut.cpp 
3rdparty/imgui/backends/imgui_impl_opengl2.cpp
)

set(CORE_SRCS
core/include/structures.h
core/include/transformations.h
core/include/point_cloud.h
core/src/point_cloud.cpp
core/include/point_clouds.h
core/src/point_clouds.cpp
core/include/ndt.h
core/src/ndt.cpp
core/include/icp.h
core/src/icp.cpp
core/include/nns.h
core/src/nns.cpp
core/include/pose_graph_slam.h
core/src/pose_graph_slam.cpp
core/include/m_estimators.h
core/include/registration_plane_feature.h
core/src/registration_plane_feature.cpp
core/include/plycpp.h
core/src/plycpp.cpp
core/src/optimize_point_to_projection_onto_plane_source_to_target.cpp
core/src/optimize_point_to_plane_source_to_target.cpp
core/src/optimize_distance_point_to_plane_source_to_target.cpp
core/src/optimize_plane_to_plane_source_to_target.cpp
core/include/pcl_wrapper.h
core/src/pcl_wrapper.cpp
core/src/optimization_point_to_point_source_to_target.cpp
core/include/observation_picking.h
core/src/observation_picking.cpp
core/include/manual_pose_graph_loop_closure.h
core/src/manual_pose_graph_loop_closure.cpp
core/include/gnss.h
core/src/gnss.cpp
core/include/wgs84_do_puwg92.h
core/src/wgs84_do_puwg92.cc
core/include/session.h
core/src/session.cpp
core/src/pfd_wrapper.cpp
)

set(CORE_HD_MAPPING_SRCS
core/hd_mapping_include/odo_with_gnss_fusion.h
core/hd_mapping_src/odo_with_gnss_fusion.cpp
core/hd_mapping_include/project_settings.h
core/hd_mapping_src/project_settings.cpp
core/hd_mapping_include/single_trajectory_viewer.h
core/hd_mapping_src/single_trajectory_viewer.cpp
core/hd_mapping_include/roi_exporter.h
core/hd_mapping_src/roi_exporter.cpp
core/hd_mapping_include/laz_wrapper.h
core/hd_mapping_src/laz_wrapper.cpp
)

set(GIZMO 
3rdparty/ImGuizmo/ImGuizmo.cpp
)

set(FUSION 
3rdparty/Fusion/Fusion/FusionAhrs.c
3rdparty/Fusion/Fusion/FusionCompass.c
3rdparty/Fusion/Fusion/FusionOffset.c
)

include_directories(3rdparty/)
include_directories(3rdparty/freeglut-3.2.2/include/)
add_subdirectory(3rdparty/freeglut-3.2.2)

add_executable(multi_view_tls_registration_step_2 src/multi_view_tls_registration.cpp src/perform_experiment.cpp
${IMGUI_SRCS}
${CORE_SRCS}
${GIZMO}
)

if (WIN32)
target_link_libraries(multi_view_tls_registration_step_2 ${OPENGL_LIBRARIES} ../3rdparty/glew-2.2.0/lib/Release/x64/glew32 freeglut_static laszip3)	
else()
target_link_libraries(multi_view_tls_registration_step_2 ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} freeglut_static laszip tbb)
endif()
if (PCL_FOUND)
    target_link_libraries (multi_view_tls_registration_step_2 ${PCL_LIBRARIES})
endif()
if (GTSAM_FOUND)
    target_link_libraries (multi_view_tls_registration_step_2 gtsam)
endif()

add_executable(hd_mapper src/hd_mapper.cpp
${IMGUI_SRCS}
${CORE_SRCS}
${GIZMO}
${CORE_HD_MAPPING_SRCS}
)

if (WIN32)
target_link_libraries(hd_mapper ${OPENGL_LIBRARIES} ../3rdparty/glew-2.2.0/lib/Release/x64/glew32 freeglut_static laszip3)	
else()
target_link_libraries(hd_mapper ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} freeglut_static laszip tbb)
endif()
if (PCL_FOUND)
    target_link_libraries (hd_mapper ${PCL_LIBRARIES})
endif()
if (GTSAM_FOUND)
    target_link_libraries (hd_mapper gtsam)
endif()

add_executable(matrix_mul src/matrix_mul.cpp)

include_directories(
3rdparty/Fusion/Fusion
)

add_executable(lidar_odometry_step_1 src/lidar_odometry.cpp src/lidar_odometry_utils.h src/lidar_odometry_utils.cpp src/lidar_odometry_utils_optimizers.cpp
${IMGUI_SRCS}
${CORE_SRCS}
${GIZMO}
${FUSION}
)

if (WIN32)
target_link_libraries(lidar_odometry_step_1 ${OPENGL_LIBRARIES} ../3rdparty/glew-2.2.0/lib/Release/x64/glew32 freeglut_static laszip3)
else()
target_link_libraries(lidar_odometry_step_1 ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} freeglut_static laszip tbb)
endif()

if (PCL_FOUND)
    target_link_libraries (lidar_odometry_step_1 ${PCL_LIBRARIES})
endif()
if (GTSAM_FOUND)
    target_link_libraries (lidar_odometry_step_1 gtsam)
endif()

add_executable(multi_session_registration_step_3 src/multi_session_registration.cpp
${IMGUI_SRCS}
${CORE_SRCS}
${GIZMO}
)

add_executable(mandeye_with_gopro_max_manual_coloring src/manual_color.cpp 
							core/src/color_las_loader.cpp 
							core/src/pfd_wrapper.cpp 
							${IMGUI_SRCS}
							) 


if (WIN32)
	target_link_libraries(mandeye_with_gopro_max_manual_coloring ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} freeglut_static laszip3)
	target_link_libraries(multi_session_registration_step_3 ${OPENGL_LIBRARIES} ../3rdparty/glew-2.2.0/lib/Release/x64/glew32 freeglut_static laszip3)	
else()
	target_link_libraries(mandeye_with_gopro_max_manual_coloring ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} freeglut_static laszip tbb)
	target_link_libraries(multi_session_registration_step_3 ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} freeglut_static laszip tbb)
endif()
if (PCL_FOUND)
    target_link_libraries (multi_session_registration_step_3 ${PCL_LIBRARIES})
endif()
if (GTSAM_FOUND)
    target_link_libraries (multi_session_registration_step_3 gtsam)
endif()



