#pragma once
#include "raylib.h"

// Mouse-driven orbit/pan/zoom camera, shared between
// apps/camera_lidar_calibration and apps/camera_lidar_trajectory_viewer (was
// byte-for-byte duplicated as camera_lidar_calibration's OrbitCamera and
// camera_lidar_trajectory_viewer's Orbit). Depends on nothing but raylib --
// no Eigen, no core -- so it stays linkable from both the core_raylib side
// (which already pulls in core/core_math) and the calib_core side (which
// deliberately doesn't) without adding coupling either way.
namespace raylib_widgets {

struct OrbitCamera {
    float azimuth   =  30.f;  // degrees
    float elevation =  25.f;  // degrees
    float distance  =  30.f;
    Vector3 target  = {0.f, 0.f, 0.f};

    // Center-of-rotation ("target") smooth-transition state, mirroring
    // multi_view_tls_registration's rotation_center/new_rotation_center/
    // camera_transition_active animation -- only target eases, azimuth/
    // elevation/distance are left as-is (matching that app's actual
    // click-picked/typed-center behavior).
    Vector3 transitionTarget = target;
    bool transitionActive    = false;
    float transitionSpeed    = 1.f;

    Camera3D toRaylib() const;
    // Processes mouse input when active (mouse not over ImGui): left-drag
    // orbits, right-drag pans, wheel zooms. A manual drag cancels any
    // in-flight transition.
    void update(bool active);

    // Starts (or retargets) an eased transition of `target` to newTarget.
    void moveTargetTo(Vector3 newTarget);
    // Eases `target` toward `transitionTarget`; call once per frame with the
    // frame's delta time. No-op when no transition is active.
    void updateTransition(float dt);
    // Immediately snaps `target` to `transitionTarget` and ends the transition.
    void cancelTransition();

    // Casts a ray from `mouse` through `cam` and intersects the y = groundY
    // plane, starting a transition of `target` to the hit point on success.
    // Returns false (no-op) when the ray is ~parallel to the plane.
    bool pickGroundPlaneTarget(Vector2 mouse, Camera3D cam, float groundY = 0.f);
};

// Small 3-axis crosshair marking the current center of rotation. Call inside
// BeginMode3D/EndMode3D, every frame -- drawing at camera.target (which
// updateTransition() eases each frame) makes the cross visibly slide to a
// newly-picked/typed center along with the camera.
void drawRotationCenterCross(Vector3 center, float size, Color color);

} // namespace raylib_widgets
