#pragma once
#include "raylib.h"
#include <cstddef>

// Mouse-driven picking of a single point out of a 3D point cloud. Depends on
// nothing but raylib -- no Eigen, no core, no calib_core -- see
// raylib_widgets/CMakeLists.txt. Callers own their own coordinate
// conventions (e.g. LiDAR-frame vs raylib-frame axes) and pass already
// raylib-world-space points in.
//
// Ray construction is deliberately NOT provided here: raylib's BeginMode3D
// derives its projection's aspect ratio from the *full* window framebuffer
// (CORE.Window.currentFbo.width/height in rcore.c), with no awareness of any
// BeginScissorMode() sub-region a caller might be confining the 3D view to
// -- so a "viewport-aware" ray helper scoped to that sub-region would
// actually be WRONG for apps (like this one) that use scissor-only
// sub-views without a matching custom rlViewport()/projection. Callers
// should build the ray with plain GetScreenToWorldRay(mouse, cam), matching
// what OrbitCamera::pickGroundPlaneTarget already does.
namespace raylib_widgets
{
    // Finds the point in `points` (raylib world-space coordinates, `count`
    // entries) whose perpendicular distance to `ray` is smallest, rejecting
    // it if that distance exceeds `pixelThreshold` screen pixels at the
    // point's depth (converted via the camera's vertical FOV and the
    // viewport's pixel height, so the tolerance stays roughly constant in
    // screen space regardless of zoom). Returns true and sets outIndex on a
    // hit; false (outIndex untouched) if `points` is empty or nothing is
    // within tolerance.
    bool pickNearestPoint(
        const Vector3* points,
        size_t count,
        const Ray& ray,
        float cameraFovYDeg,
        float viewportHeightPx,
        float pixelThreshold,
        size_t& outIndex);
} // namespace raylib_widgets
