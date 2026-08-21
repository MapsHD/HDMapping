#pragma once
#include <RaylibWidgets/OrbitCamera.h>

// "Center of rotation" modal dialog, shared between the camera_lidar_* apps
// (mirrors multi_view_tls_registration's cor_window(), adapted from its
// Eigen rotation_center to OrbitCamera's Vector3 target). Depends on raylib
// + ImGui + OrbitCamera.h only -- no Eigen, no core -- same dependency story
// as OrbitCamera.h itself.
namespace raylib_widgets {

// Call once per frame. `open` is an edge-triggered request: the caller sets
// it true (e.g. from a keyboard shortcut or menu item) to pop the dialog;
// this function opens the ImGui popup and immediately clears `open` back to
// false (the popup then manages its own visibility until Set/Cancel). On
// "Set", starts an eased transition of camera.target to the typed X/Y/Z via
// camera.moveTargetTo().
void showCenterOfRotationWindow(bool& open, OrbitCamera& camera);

// Same dialog, for OrbitCamera's Euler mode instead of its azimuth/
// elevation/target mode above -- was multi_view_tls_registration's local
// cor_window(). Binds to camera.eulerGoal.rotationCenter (edited in place,
// matching cor_window()'s original behavior of editing its own persistent
// new_rotation_center field rather than a freshly-seeded copy) and calls
// camera.moveEulerRotationCenterTo() on Set. xTooltip/yTooltip/zTooltip are
// optional per-field hover tooltips (multi_view_tls_registration's original
// had these -- e.g. "Longitudinal (forward/backward)"); pass nullptr (the
// default) to skip.
void showEulerCenterOfRotationWindow(
    bool& open, OrbitCamera& camera, const char* xTooltip = nullptr, const char* yTooltip = nullptr,
    const char* zTooltip = nullptr);

} // namespace raylib_widgets
