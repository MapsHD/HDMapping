cmake_minimum_required(VERSION 4.0.0)

project(raylib_widgets)

# Small UI overlay helpers (compass/ruler, DPI-aware window fit-to-screen,
# a generic shortcuts-help table, an orbit camera + its center-of-rotation
# dialog, app-shell scaffolding) shared between apps/multi_view_tls_registration
# and the camera_lidar_* apps. Depends on raylib + imgui_raylib + Eigen
# (header-only) -- no `core`/`calib_core` -- so it's linkable from both the
# core_raylib side (already coupled to core/core_math) and the calib_core
# side (which links Eigen itself already, but deliberately not `core`)
# without adding `core` coupling either way; every current/planned consumer
# is a raylib+ImGui app already, so depending on imgui_raylib (not the
# separate GLUT-backed `imgui` target) doesn't add anything new for them.
# Only RayPlaneD.h actually uses Eigen -- everything else stays exactly as
# dependency-light as before.
add_library(raylib_widgets STATIC
    src/CompassRuler.cpp
    src/WindowFit.cpp
    src/ShortcutsTable.cpp
    src/OrbitCamera.cpp
    src/CenterOfRotationWindow.cpp
    src/PointPicking.cpp
    src/AppShell.cpp
    src/RayPlaneD.cpp
)

target_include_directories(raylib_widgets PUBLIC include ${EIGEN3_INCLUDE_DIR})
target_link_libraries(raylib_widgets PUBLIC raylib imgui_raylib)

set_target_properties(raylib_widgets PROPERTIES POSITION_INDEPENDENT_CODE ON)
