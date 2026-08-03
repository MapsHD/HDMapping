#pragma once
#include "raylib.h"

// Bottom-left axis compass + zoom-tied ruler overlay, shared between
// apps/multi_view_tls_registration (rl_utils.cpp's drawMiniCompassWithRuler)
// and the camera_lidar_* apps. Depends on nothing but raylib -- no Eigen, no
// core -- so it stays linkable from both the core_raylib side (which already
// pulls in core/core_math) and the calib_core side (which deliberately
// doesn't) without adding coupling either way.
namespace raylib_widgets {

struct CompassAxisLabels {
    const char* x = "X";
    const char* y = "Y";
    const char* z = "Z";
};

// right/up: the current camera's screen-right/up directions in world space
// (e.g. cross(forward, camUp) / cross(right, forward) for a raylib Camera3D,
// or the first two rows of a world-to-eye rotation matrix).
// zoomDistance: current camera distance/zoom, used to size the ruler.
// rulerColor: contrast color for the ruler ticks/label (axis lines are
// always drawn in RGB = X/Y/Z). Call after 3D drawing ends (2D screen-space
// overlay).
void drawCompassRuler(
    Vector3 right,
    Vector3 up,
    float zoomDistance,
    Color rulerColor,
    CompassAxisLabels labels = {});

} // namespace raylib_widgets
