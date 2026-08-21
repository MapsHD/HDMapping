#include "RaylibWidgets/OrbitCamera.h"

#include "raymath.h"
#include "rlgl.h"

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
    if (sphericalOrtho) {
        cam.fovy       = sphericalOrthoHeight;
        cam.projection = CAMERA_ORTHOGRAPHIC;
    } else {
        cam.fovy       = 45.f;
        cam.projection = CAMERA_PERSPECTIVE;
    }
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
    // Scroll → zoom (distance in perspective, sphericalOrthoHeight in ortho
    // -- mirrors multi_view_tls_registration's own perspective-dolly vs.
    // ortho-zoom split in OrbitCamera::zoom()).
    float wheel = GetMouseWheelMove();
    if (wheel != 0.f) {
        if (sphericalOrtho) {
            sphericalOrthoHeight -= wheel * sphericalOrthoHeight * 0.1f;
            sphericalOrthoHeight  = std::max(0.5f, sphericalOrthoHeight);
        } else {
            distance -= wheel * distance * 0.1f;
            distance  = std::max(0.5f, distance);
        }
    }
}

void OrbitCamera::moveTargetTo(Vector3 newTarget) {
    transitionTarget = newTarget;
    // Freeze the others to their current value so they ease "toward
    // themselves" (a no-op) -- see this field's declaration comment.
    transitionAzimuth = azimuth;
    transitionElevation = elevation;
    transitionDistance = distance;
    transitionSphericalOrthoHeight = sphericalOrthoHeight;
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

    // Azimuth is an unbounded, unwrapped angle (unlike elevation, which
    // update() clamps to +-89) -- after enough manual dragging it can sit
    // far outside [0,360), so the raw goal-minus-current delta can be a
    // near-full-circle the "long way around". Wrapping it into (-180,180]
    // first makes the eased rotation always take the shorter path.
    float deltaAz = transitionAzimuth - azimuth;
    while (deltaAz > 180.f) deltaAz -= 360.f;
    while (deltaAz < -180.f) deltaAz += 360.f;
    bool doneAz = std::fabs(deltaAz) < 0.01f;
    if (!doneAz) azimuth += deltaAz * t;

    bool doneEl = std::fabs(transitionElevation - elevation) < 0.01f;
    if (!doneEl) elevation += (transitionElevation - elevation) * t;

    bool doneDist = std::fabs(transitionDistance - distance) < 0.01f;
    if (!doneDist) distance += (transitionDistance - distance) * t;

    bool doneOrtho = std::fabs(transitionSphericalOrthoHeight - sphericalOrthoHeight) < 0.01f;
    if (!doneOrtho) sphericalOrthoHeight += (transitionSphericalOrthoHeight - sphericalOrthoHeight) * t;

    transitionActive = !(doneX && doneY && doneZ && doneAz && doneEl && doneDist && doneOrtho);
    if (!transitionActive) {
        target = transitionTarget;
        azimuth = transitionAzimuth;
        elevation = transitionElevation;
        distance = transitionDistance;
        sphericalOrthoHeight = transitionSphericalOrthoHeight;
    }
}

void OrbitCamera::cancelTransition() {
    if (!transitionActive) return;
    target = transitionTarget;
    azimuth = transitionAzimuth;
    elevation = transitionElevation;
    distance = transitionDistance;
    sphericalOrthoHeight = transitionSphericalOrthoHeight;
    transitionActive = false;
}

void OrbitCamera::setPreset(ViewPreset preset) {
    // Only Front/Back/Left/Right/Top/Bottom/Iso change azimuth/elevation;
    // Reset changes target/distance/sphericalOrthoHeight too -- see this
    // method's declaration comment.
    transitionTarget = target;
    transitionDistance = distance;
    transitionSphericalOrthoHeight = sphericalOrthoHeight;

    switch (preset) {
    case ViewPreset::Front:  transitionAzimuth =   0.f; transitionElevation =  0.f; break;
    case ViewPreset::Back:   transitionAzimuth = 180.f; transitionElevation =  0.f; break;
    case ViewPreset::Left:   transitionAzimuth =  90.f; transitionElevation =  0.f; break;
    case ViewPreset::Right:  transitionAzimuth = -90.f; transitionElevation =  0.f; break;
    // +-89, not +-90: exactly 90 sits right on update()'s own elevation
    // clamp boundary -- landing a hair inside it means a manual drag right
    // after the transition completes doesn't immediately re-clamp/jump.
    case ViewPreset::Top:    transitionAzimuth = azimuth; transitionElevation =  89.f; break;
    case ViewPreset::Bottom: transitionAzimuth = azimuth; transitionElevation = -89.f; break;
    // Matches multi_view_tls_registration's CAMERA_ISO angle (35.264 deg
    // -- true isometric) for consistency between the two apps.
    case ViewPreset::Iso:    transitionAzimuth =  45.f; transitionElevation = 35.264f; break;
    case ViewPreset::Reset:
        transitionAzimuth = 30.f;
        transitionElevation = 25.f;
        transitionDistance = 30.f;
        transitionTarget = { 0.f, 0.f, 0.f };
        transitionSphericalOrthoHeight = 20.f;
        break;
    }

    transitionActive = true;
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

// ---------------------------------------------------------------------
// Euler-angle + orthographic mode -- see OrbitCamera.h's comment on the
// `euler`/`isOrtho`/... fields for what this ports from.
// ---------------------------------------------------------------------

void OrbitCamera::applyPerspectiveProjection(int w, int h) const {
    // GetRenderWidth/Height(), not w/h: the GL viewport must be sized in
    // actual framebuffer pixels, which can differ from logical w/h under
    // DPI scaling -- matches the original reshape().
    rlViewport(0, 0, GetRenderWidth(), GetRenderHeight());
    rlMatrixMode(RL_PROJECTION);
    rlLoadIdentity();

    const double fovy = 60.0;
    const double aspect = (double)w / (double)h;
    const double nearP = 0.01, farP = 10000.0;
    const double top = nearP * tan(fovy * 0.5 * (double)DEG2RAD);
    const double bottom = -top;
    const double right = top * aspect;
    const double left = -right;
    rlFrustum(left, right, bottom, top, nearP, farP);

    rlMatrixMode(RL_MODELVIEW);
    rlLoadIdentity();
}

void OrbitCamera::applyOrthoProjection(float aspect) const {
    rlViewport(0, 0, GetRenderWidth(), GetRenderHeight());
    rlMatrixMode(RL_PROJECTION);
    rlLoadIdentity();
    rlOrtho(-orthoZoom, orthoZoom, -orthoZoom / aspect, orthoZoom / aspect, -100000, 100000);
    rlMatrixMode(RL_MODELVIEW);
    rlLoadIdentity();
}

void OrbitCamera::updateOrtho(float aspect) {
    // Real rlgl projection (right-handed, [-1,1] depth) -- what the scene
    // actually renders through.
    rlOrtho(-orthoZoom, orthoZoom, -orthoZoom / aspect, orthoZoom / aspect, -100000, 100000);

    // Second, independent projection matrix for ImGuizmo only -- ported
    // from the original's glm::orthoLH_ZO(-zoom, zoom, -zoom/aspect,
    // zoom/aspect, -100, 100) (left-handed, [0,1] depth -- distinct from
    // rlOrtho's own convention above). Written directly as flat
    // column-major floats (index = col*4+row) rather than through raylib's
    // Matrix type: Matrix's m0..m15 field *names* follow that same
    // indexing, but its C++ declaration order groups them by row, so a raw
    // `&matrix.m0` reinterpret-as-float* would silently scramble this.
    for (float& f : orthoProjection) f = 0.f;
    orthoProjection[0] = 1.f / orthoZoom;
    orthoProjection[5] = aspect / orthoZoom;
    orthoProjection[10] = 1.f / 200.f; // 1/(zFar - zNear), zNear=-100, zFar=100
    orthoProjection[14] = 0.5f;        // -zNear/(zFar - zNear)
    orthoProjection[15] = 1.f;

    // Ortho "camera" for the gizmo view: looks straight down at
    // (orthoShiftX, orthoShiftY, orthoZCenterH) from 10 units above, with
    // "up" rotated to match the current in-plane rotation
    // (rotateX + rotateY, same yaw the perspective mode's rlMultMatrixf
    // view applies) -- ported from the original's TaitBryan-yaw-only
    // rotation of (0,1,0), which reduces to this plain 2D rotation.
    Vector3 eye = { -orthoShiftX, orthoShiftY, orthoZCenterH + 10.f };
    Vector3 center = { -orthoShiftX, orthoShiftY, orthoZCenterH };
    float ka = -(euler.rotateX + euler.rotateY) * DEG2RAD;
    Vector3 up = { -std::sin(ka), std::cos(ka), 0.f };

    Matrix lookat = MatrixLookAt(eye, center, up);
    float16 flat = MatrixToFloatV(lookat);
    for (int i = 0; i < 16; ++i) orthoGizmoView[i] = flat.v[i];

    // Folds the lookAt into rlgl's *projection* stack (not modelview) --
    // matches the original's "was glOrtho + gluLookAt folded into
    // GL_PROJECTION" comment: this app drives its ortho camera entirely
    // through the projection matrix, leaving modelview identity (see the
    // rlMatrixMode(RL_MODELVIEW) below). MatrixToFloat(), not `&lookat.m0`
    // -- see this function's comment above on why the latter is unsafe.
    rlMultMatrixf(MatrixToFloat(lookat));

    rlMatrixMode(RL_MODELVIEW);
    rlLoadIdentity();
}

void OrbitCamera::captureFrameMatrices() {
    frameView3D = rlGetMatrixModelview();
    frameProj3D = rlGetMatrixProjection();
}

Ray OrbitCamera::eulerScreenRay(int x, int y, int screenW, int screenH) const {
    float ndcX = (2.0f * (float)x) / (float)screenW - 1.0f;
    float ndcY = 1.0f - (2.0f * (float)y) / (float)screenH;

    // Far point uses NDC z=1 (the actual far plane) rather than an
    // out-of-range hack -- only direction, not magnitude, matters to
    // callers, and this matches raylib's own GetScreenToWorldRayEx.
    Vector3 nearPoint = Vector3Unproject(Vector3{ ndcX, ndcY, 0.0f }, frameProj3D, frameView3D);
    Vector3 farPoint = Vector3Unproject(Vector3{ ndcX, ndcY, 1.0f }, frameProj3D, frameView3D);

    Ray ray;
    ray.position = nearPoint;
    ray.direction = Vector3Subtract(farPoint, nearPoint);
    return ray;
}

void OrbitCamera::dragOrbit(float dx, float dy) {
    euler.rotateX += dy * 0.2f;
    euler.rotateY += dx * 0.2f;
    breakEulerTransition();
}

void OrbitCamera::dragPanPerspective(float dx, float dy) {
    euler.translate.x += dx * 0.1f * eulerMouseSensitivity;
    euler.translate.y -= dy * 0.1f * eulerMouseSensitivity;
    breakEulerTransition();
}

void OrbitCamera::dragPanOrtho(float dx, float dy, float displayW, float displayH) {
    if (displayW <= 0.f || displayH <= 0.f) return;

    float ratio = displayW / displayH;
    float vx = dx * (orthoZoom / displayW * 2.f);
    float vy = dy * (orthoZoom / displayH * 2.f / ratio);

    // Rotate the pan vector by the current in-plane rotation (rotateX +
    // rotateY), matching the original's TaitBryan-yaw-only rotation
    // (opposite sign convention from updateOrtho()'s gizmo-up rotation --
    // this is a different vector, ported independently from the same
    // original source).
    float ka = (euler.rotateX + euler.rotateY) * DEG2RAD;
    float ck = std::cos(ka), sk = std::sin(ka);

    orthoShiftX += ck * vx - sk * vy;
    orthoShiftY += sk * vx + ck * vy;
    // Deliberately no breakEulerTransition() call here -- matches the
    // original, which didn't break the transition on ortho pan either.
}

void OrbitCamera::zoom(float wheelDelta, bool shiftHeld) {
    if (wheelDelta == 0.f) return;

    // wheelDelta=+-1 (a real mouse wheel's per-notch magnitude) reproduces
    // the original's fixed per-notch step exactly; any other magnitude
    // (trackpad smooth-scroll) scales it proportionally -- see this
    // function's declaration comment.
    if (isOrtho) {
        orthoZoom -= 0.1f * orthoZoom * wheelDelta;
        if (orthoZoom < 0.1f) orthoZoom = 0.1f;
    } else {
        float step = shiftHeld ? 5.f : 1.f;
        euler.translate.z += step * wheelDelta;
    }

    eulerMouseSensitivity = std::fabs(euler.translate.z) / 100.f;
    // Not breakEulerTransition(): matches the original, which just cleared
    // camera_transition_active here without snapping rotation_center.
    eulerTransitionActive = false;
}

void OrbitCamera::setEulerPreset(EulerPreset preset) {
    bool triggered = false;

    switch (preset) {
    case EulerPreset::Front:
        eulerGoal.rotateX = -90.0f;
        eulerGoal.rotateY = +90.0f;
        triggered = true;
        break;
    case EulerPreset::Back:
        eulerGoal.rotateX = -90.0f;
        eulerGoal.rotateY = -90.0f;
        triggered = true;
        break;
    case EulerPreset::Left:
        eulerGoal.rotateX = -90.0f;
        eulerGoal.rotateY = 180.0f;
        triggered = true;
        break;
    case EulerPreset::Right:
        eulerGoal.rotateX = -90.0f;
        eulerGoal.rotateY = 0.0f;
        triggered = true;
        break;
    case EulerPreset::Top:
        eulerGoal.rotateX = 0.0f;
        eulerGoal.rotateY = 90.0f;
        triggered = true;
        break;
    case EulerPreset::Bottom:
        eulerGoal.rotateX = 180.0f;
        eulerGoal.rotateY = -90.0f;
        triggered = true;
        break;
    case EulerPreset::Iso:
        eulerGoal.rotateX = -35.264f;
        eulerGoal.rotateY = 135.0f;
        triggered = true;
        break;
    case EulerPreset::Reset:
        eulerGoal.rotationCenter = { 0.f, 0.f, 0.f };
        eulerGoal.rotateX = 0.f;
        eulerGoal.rotateY = 0.f;
        eulerGoal.translate = { 0.f, 0.f, -50.0f };
        eulerMouseSensitivity = std::fabs(euler.translate.z) / 100.f;

        orthoZoom = 10.f;
        orthoShiftX = 0.f;
        orthoShiftY = 0.f;
        orthoZCenterH = 0.f;

        triggered = false;
        break;
    }

    if (triggered) {
        eulerGoal.rotationCenter = euler.rotationCenter;
        eulerGoal.translate = euler.translate;
    }

    eulerTransitionActive = true;
}

void OrbitCamera::startEulerTransition(float rotateX, float rotateY, Vector3 translate, Vector3 rotationCenter) {
    eulerGoal.rotateX = rotateX;
    eulerGoal.rotateY = rotateY;
    eulerGoal.translate = translate;
    eulerGoal.rotationCenter = rotationCenter;
    eulerTransitionActive = true;
}

void OrbitCamera::moveEulerRotationCenterTo(Vector3 center) {
    startEulerTransition(euler.rotateX, euler.rotateY, Vector3{ -center.x, -center.y, euler.translate.z }, center);
}

void OrbitCamera::updateEulerTransition(float dt) {
    if (!eulerTransitionActive) return;

    float t = 1.0f - std::pow(1.0f - std::min(dt * eulerTransitionSpeed, 1.0f), 3.0f);

    auto ease = [t](float& cur, float goal) -> bool {
        if (std::fabs(goal - cur) < 0.01f) return true;
        cur += (goal - cur) * t;
        return false;
    };

    bool doneRcX = ease(euler.rotationCenter.x, eulerGoal.rotationCenter.x);
    bool doneRcY = ease(euler.rotationCenter.y, eulerGoal.rotationCenter.y);
    bool doneRcZ = ease(euler.rotationCenter.z, eulerGoal.rotationCenter.z);
    bool doneRx = ease(euler.rotateX, eulerGoal.rotateX);
    bool doneRy = ease(euler.rotateY, eulerGoal.rotateY);
    bool doneTx = ease(euler.translate.x, eulerGoal.translate.x);
    bool doneTy = ease(euler.translate.y, eulerGoal.translate.y);
    bool doneTz = ease(euler.translate.z, eulerGoal.translate.z);

    eulerTransitionActive = !(doneRcX && doneRcY && doneRcZ && doneRx && doneRy && doneTx && doneTy && doneTz);

    if (!eulerTransitionActive) euler = eulerGoal;
}

void OrbitCamera::breakEulerTransition() {
    if (!eulerTransitionActive) return;
    euler.rotationCenter = eulerGoal.rotationCenter;
    eulerTransitionActive = false;
}

void drawRotationCenterCross(Vector3 center, float size, Color color) {
    DrawLine3D(Vector3{ center.x - size, center.y, center.z }, Vector3{ center.x + size, center.y, center.z }, color);
    DrawLine3D(Vector3{ center.x, center.y - size, center.z }, Vector3{ center.x, center.y + size, center.z }, color);
    DrawLine3D(Vector3{ center.x, center.y, center.z - size }, Vector3{ center.x, center.y, center.z + size }, color);
}

void end3DMatrixStack(float displayW, float displayH) {
    rlDrawRenderBatchActive();
    rlMatrixMode(RL_PROJECTION);
    rlLoadIdentity();
    rlOrtho(0, displayW, displayH, 0, 0.0f, 1.0f);
    rlMatrixMode(RL_MODELVIEW);
    rlLoadIdentity();
    rlDisableDepthTest();
}

} // namespace raylib_widgets
