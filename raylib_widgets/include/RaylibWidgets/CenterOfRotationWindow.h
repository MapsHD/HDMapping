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

} // namespace raylib_widgets
