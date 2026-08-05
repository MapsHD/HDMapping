#include "RaylibWidgets/OrbitCamera.h"

#include "raymath.h"

#include <algorithm>
#include <cmath>

namespace raylib_widgets {

Camera3D OrbitCamera::toRaylib() const {
    float az  = azimuth   * (float)DEG2RAD;
    float el  = elevation * (float)DEG2RAD;
    Vector3 pos = {
        target.x + distance * std::cos(el) * std::sin(az),
        target.y + distance * std::sin(el),
        target.z + distance * std::cos(el) * std::cos(az)
    };
    Camera3D cam;
    cam.position   = pos;
    cam.target     = target;
    cam.up         = {0.f, 1.f, 0.f};
    cam.fovy       = 45.f;
    cam.projection = CAMERA_PERSPECTIVE;
    return cam;
}

void OrbitCamera::update(bool active) {
    if (!active) return;

    // Left-drag → orbit
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Vector2 d = GetMouseDelta();
        azimuth   -= d.x * 0.4f;
        elevation += d.y * 0.4f;
        elevation  = std::max(-89.f, std::min(89.f, elevation));
        cancelTransition();
    }
    // Right-drag → pan
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        Camera3D cam = toRaylib();
        Vector3 fwd   = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
        Vector3 right = Vector3Normalize(Vector3CrossProduct(fwd, cam.up));
        Vector3 up    = Vector3CrossProduct(right, fwd);
        Vector2 d     = GetMouseDelta();
        float   speed = distance * 0.002f;
        target = Vector3Add(target, Vector3Scale(right, -d.x * speed));
        target = Vector3Add(target, Vector3Scale(up,     d.y * speed));
        cancelTransition();
    }
    // Scroll → zoom
    float wheel = GetMouseWheelMove();
    if (wheel != 0.f) {
        distance -= wheel * distance * 0.1f;
        distance  = std::max(0.5f, distance);
    }
}

void OrbitCamera::moveTargetTo(Vector3 newTarget) {
    transitionTarget = newTarget;
    transitionActive = true;
}

void OrbitCamera::updateTransition(float dt) {
    if (!transitionActive) return;

    float t = 1.f - std::pow(1.f - std::min(dt * transitionSpeed, 1.f), 3.f);

    bool doneX = std::fabs(transitionTarget.x - target.x) < 0.01f;
    bool doneY = std::fabs(transitionTarget.y - target.y) < 0.01f;
    bool doneZ = std::fabs(transitionTarget.z - target.z) < 0.01f;

    if (!doneX) target.x += (transitionTarget.x - target.x) * t;
    if (!doneY) target.y += (transitionTarget.y - target.y) * t;
    if (!doneZ) target.z += (transitionTarget.z - target.z) * t;

    transitionActive = !(doneX && doneY && doneZ);
    if (!transitionActive)
        target = transitionTarget;
}

void OrbitCamera::cancelTransition() {
    if (!transitionActive) return;
    target = transitionTarget;
    transitionActive = false;
}

bool OrbitCamera::pickGroundPlaneTarget(Vector2 mouse, Camera3D cam, float groundY) {
    Ray ray = GetScreenToWorldRay(mouse, cam);

    const float kTolerance = 0.0001f;
    if (ray.direction.y > -kTolerance && ray.direction.y < kTolerance)
        return false; // ray ~parallel to the ground plane

    float t = (groundY - ray.position.y) / ray.direction.y;
    Vector3 hit = Vector3Add(ray.position, Vector3Scale(ray.direction, t));
    moveTargetTo(hit);
    return true;
}

void drawRotationCenterCross(Vector3 center, float size, Color color) {
    DrawLine3D(Vector3{ center.x - size, center.y, center.z }, Vector3{ center.x + size, center.y, center.z }, color);
    DrawLine3D(Vector3{ center.x, center.y - size, center.z }, Vector3{ center.x, center.y + size, center.z }, color);
    DrawLine3D(Vector3{ center.x, center.y, center.z - size }, Vector3{ center.x, center.y, center.z + size }, color);
}

} // namespace raylib_widgets
