cmake_minimum_required(VERSION 4.0.0)

project(raylib_widgets)

# Small UI overlay helpers (compass/ruler, DPI-aware window fit-to-screen,
# a generic shortcuts-help table) shared between
# apps/multi_view_tls_registration and the camera_lidar_* apps. Depends on
# raylib + imgui_raylib only -- no Eigen, no core, no calib_core -- so it's
# linkable from both the core_raylib side (already coupled to core/core_math)
# and the calib_core side (deliberately not) without adding coupling either
# way; every current/planned consumer is a raylib+ImGui app already, so
# depending on imgui_raylib (not the separate GLUT-backed `imgui` target)
# doesn't add anything new for them.
add_library(raylib_widgets STATIC
    src/CompassRuler.cpp
    src/WindowFit.cpp
    src/ShortcutsTable.cpp
)

target_include_directories(raylib_widgets PUBLIC include)
target_link_libraries(raylib_widgets PUBLIC raylib imgui_raylib)

set_target_properties(raylib_widgets PROPERTIES POSITION_INDEPENDENT_CODE ON)
