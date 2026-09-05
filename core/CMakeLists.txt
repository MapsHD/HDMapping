cmake_minimum_required(VERSION 4.0.0)

project(core)

set(CORE_BASE_SOURCES
    src/color_las_loader.cpp
    src/control_points.cpp
    src/gnss.cpp
    src/ground_control_points.cpp
    src/imu_preintegration.cpp
    src/nmea.cpp
    src/point_cloud.cpp
    src/point_clouds.cpp
    src/session.cpp
    src/tum.cpp
   # # src/utils.cpp # TODO(mwlasiuk) : broken AF ...
)

# core_math holds the registration/optimization sources with auto-generated Jacobian headers (up to ~24k chars/line, expensive to compile); built once as a static lib shared by core and core_no_gui since none of it branches on WITH_GUI.
# hash_utils.cpp lives here (not CORE_BASE_SOURCES) because pair_wise_iterative_closest_point.cpp needs get_rgd_index_3d() from it -- keeping both in the same archive avoids a circular static-lib link dependency between core_math and core/core_no_gui.
set(CORE_MATH_SOURCES
    src/hash_utils.cpp
    src/icp.cpp
    src/ndt.cpp
    src/optimization_point_to_point_source_to_target.cpp
    src/optimize_distance_point_to_plane_source_to_target.cpp
    src/optimize_plane_to_plane_source_to_target.cpp
    src/optimize_point_to_plane_source_to_target.cpp
    src/optimize_point_to_projection_onto_plane_source_to_target.cpp
    src/pair_wise_iterative_closest_point.cpp
    src/pose_graph_loop_closure.cpp
    src/pose_graph_slam.cpp
    src/registration_plane_feature.cpp
    src/point_cloud_evaluation.cpp
)

add_library(core_math STATIC ${CORE_MATH_SOURCES})
target_compile_definitions(core_math PRIVATE WITH_GUI=0)
target_link_libraries(core_math PRIVATE PROJ::proj spdlog::spdlog vqf Fusion wgs84_do_puwg92 plycpp WGS84toCartesian)
target_include_directories(core_math PRIVATE
    include
    ${EIGEN3_INCLUDE_DIR}
    ${LASZIP_INCLUDE_DIR}/LASzip/include
    ${THIRDPARTY_DIRECTORY}/json/include
    ${THIRDPARTY_DIRECTORY}/observation_equations/codes
    ${EXTERNAL_LIBRARIES_DIRECTORY}/include
    ${THIRDPARTY_DIRECTORY}/vqf/vqf/cpp
    ${THIRDPARTY_DIRECTORY}/Fusion/Fusion
)
set_target_properties(core_math PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(NOT MSVC)
    # DWARF generation for these auto-generated Jacobian expression trees dominates -g compile time far more than -O2 codegen, so RelWithDebInfo builds this target without -g.
    target_compile_options(core_math PRIVATE $<$<CONFIG:RelWithDebInfo>:-g0>)
endif()

set(CORE_GUI_SOURCES
    src/manual_pose_graph_loop_closure.cpp
    src/observation_picking.cpp
)

# core_pfd -- native file/folder picker dialogs (mandeye::fd, wrapping
# portable-file-dialogs), split out of CORE_GUI_SOURCES into its own minimal
# target so non-core GUI consumers (the camera_lidar_* apps, via calib_core)
# can reuse it without linking core_math/PROJ/spdlog/vqf/Fusion/plycpp/
# WGS84toCartesian/imgui/ImGuizmo/freeglut, none of which pfd_wrapper.cpp
# actually needs.
add_library(core_pfd STATIC src/pfd_wrapper.cpp)
target_include_directories(core_pfd PUBLIC include)
target_include_directories(core_pfd PRIVATE ${THIRDPARTY_DIRECTORY}/portable-file-dialogs-master)
set_target_properties(core_pfd PROPERTIES POSITION_INDEPENDENT_CODE ON)

# core_e57 -- E57 point-cloud read/write (mandeye::e57io, wrapping libE57Format
# + Xerces-C). Split out like core_pfd so the camera_lidar_* apps (which link
# calib_core, not core) can use E57 export without pulling in core_math/PROJ/
# the rest. Linked PUBLIC into core/core_no_gui below, so existing core
# consumers still get mandeye::e57io transparently.
add_library(core_e57 STATIC src/e57_utils.cpp)
target_include_directories(core_e57 PUBLIC include ${LIBE57FORMAT_INCLUDE_DIR})
target_include_directories(core_e57 PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(core_e57 PUBLIC E57Format PRIVATE spdlog::spdlog)
set_target_properties(core_e57 PROPERTIES POSITION_INDEPENDENT_CODE ON)

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
    # core_e57 (PUBLIC) provides mandeye::e57io + pulls E57Format/Xerces-C onto
    # the final link line for consumers that call it.
    target_link_libraries(${target_name} PRIVATE core_math ${PLATFORM_LASZIP_LIB} ${PLATFORM_MISCELLANEOUS_LIBS} PROJ::proj spdlog::spdlog vqf Fusion wgs84_do_puwg92 plycpp WGS84toCartesian PUBLIC core_e57)
    target_include_directories(${target_name} PRIVATE
        include
        ${EIGEN3_INCLUDE_DIR}
        ${LASZIP_INCLUDE_DIR}/LASzip/include
        ${THIRDPARTY_DIRECTORY}/json/include
        ${THIRDPARTY_DIRECTORY}/observation_equations/codes
        ${EXTERNAL_LIBRARIES_DIRECTORY}/include
        ${THIRDPARTY_DIRECTORY}/vqf/vqf/cpp
        ${THIRDPARTY_DIRECTORY}/Fusion/Fusion
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
target_link_libraries(core PUBLIC core_pfd)

target_precompile_headers(core_no_gui
    PRIVATE include/pch/pch.h
)

target_precompile_headers(core_no_gui
    PRIVATE include/pch/pch.h
)

set_target_properties(core_no_gui PROPERTIES
    POSITION_INDEPENDENT_CODE ON)

# ============================================================================
# core_raylib -- shared raylib rendering/camera/picking infrastructure
# (ScanRenderer, OrbitCamera, picking, the compass overlay), used by
# apps/multi_view_tls_registration (step2, raylib-based) and any future
# raylib-based HDMapping GUI. This is the raylib-app equivalent of
# core/src/utils.cpp's role for the remaining GLUT apps. Kept as a separate
# target (not merged into `core`/`core_no_gui` above) so that raylib is never
# a dependency of apps that still use GLUT.
# ============================================================================
add_library(core_raylib STATIC src/raylib_render.cpp)
target_compile_definitions(core_raylib PRIVATE WITH_GUI=1)
target_include_directories(core_raylib PRIVATE
    include
    ${EIGEN3_INCLUDE_DIR}
    ${LASZIP_INCLUDE_DIR}/LASzip/include
    ${THIRDPARTY_DIRECTORY}/json/include
    ${THIRDPARTY_DIRECTORY}/observation_equations/codes
    ${EXTERNAL_LIBRARIES_DIRECTORY}/include
    ${THIRDPARTY_DIRECTORY}/vqf/vqf/cpp
    ${THIRDPARTY_DIRECTORY}/Fusion/Fusion
    # raylib_widgets/include/RaylibWidgets/Shaders.h -- shared GLSL colormap
    # snippet spliced into raylib_render.cpp's fragment shader. Header-only,
    # so just the include dir is needed here, not a full raylib_widgets link
    # (which would add imgui_raylib as a new transitive core_raylib dependency).
    ${REPOSITORY_DIRECTORY}/raylib_widgets/include
)
# PUBLIC: consumers (apps/multi_view_tls_registration) get core's
# Session/PointCloud types and raylib/rlgl/raymath transitively just by
# linking core_raylib. This target only needs raylib itself (rlgl,
# raymath, glad) -- rlImGui/Dear ImGui are used by the app's own panel
# code, not by raylib_render.cpp, so they're linked by the app directly.
#
# FREEGLUT_LIBRARY is linked here (not by the consuming app) so that
# CMake's link-order computation places it immediately alongside `core`:
# core.a's own object files (manual_pose_graph_loop_closure.cpp.o,
# observation_picking.cpp.o) reference glutBitmap*() symbols transitively
# (same translation unit as the Gui()/data methods raylib consumers do
# call, even though the ::Render() methods that use GLUT text are never
# actually invoked at runtime here -- no glutInit/window exists). Linking
# it as a separate PRIVATE entry on the final executable instead let
# CMake's static-library dependency reordering sort libglut.so before
# libcore.a, leaving those symbols unresolved.
target_link_libraries(core_raylib PUBLIC core raylib ${FREEGLUT_LIBRARY})