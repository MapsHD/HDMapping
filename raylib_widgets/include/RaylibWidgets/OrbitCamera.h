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
    // camera_transition_active animation. moveTargetTo() (the only way
    // pickGroundPlaneTarget()/CenterOfRotationWindow trigger a transition)
    // only ever moves `target` -- it freezes transitionAzimuth/Elevation/
    // Distance/SphericalOrthoHeight to their current values, so those ease
    // "toward themselves" (a no-op) whenever only the target changes,
    // exactly like before this struct grew orientation/zoom transitions.
    // setPreset() below is what actually drives those too, as one combined
    // transition.
    Vector3 transitionTarget            = target;
    float transitionAzimuth             = azimuth;
    float transitionElevation           = elevation;
    float transitionDistance            = distance;
    float transitionSphericalOrthoHeight = 20.f; // matches sphericalOrthoHeight's default below
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

    // Orthographic toggle for this (azimuth/elevation/distance) camera --
    // raylib's own Camera3D/BeginMode3D already supports
    // CAMERA_ORTHOGRAPHIC natively, so unlike the Euler mode below this
    // doesn't need any manual rlgl matrix-stack work: toRaylib() just sets
    // cam.projection/cam.fovy accordingly. sphericalOrthoHeight is the
    // view's vertical extent in world units (raylib's ortho fovy
    // convention) -- update()'s wheel-zoom branch adjusts this instead of
    // `distance` while sphericalOrtho is set, mirroring
    // multi_view_tls_registration's separate ortho zoom. Named distinctly
    // from the Euler mode's own isOrtho/orthoProjection/etc. below --
    // that's a separate camera model bolted onto this same struct, not
    // shared state.
    bool sphericalOrtho = false;
    float sphericalOrthoHeight = 20.f;

    // Fixed preset views, ported from multi_view_tls_registration's
    // CameraPreset/setCameraPreset() (see EulerPreset/setEulerPreset()
    // below) but expressed in this camera's own azimuth/elevation terms --
    // the two apps use different world "up" conventions (this one is
    // raylib's standard Y-up; multi_view_tls_registration's Euler mode is
    // Z-up), so the *numbers* differ, but each preset names the same
    // logical view. Front/Back/Left/Right/Top/Bottom/Iso only change
    // azimuth/elevation -- target/distance/sphericalOrthoHeight (pan/zoom)
    // carry over unchanged, matching the original's behavior of preserving
    // pan and zoom across a preset switch. Reset also restores target/
    // distance/sphericalOrthoHeight to this struct's own defaults.
    enum class ViewPreset { Front, Back, Left, Right, Top, Bottom, Iso, Reset };
    void setPreset(ViewPreset preset);

    // ------------------------------------------------------------------
    // Euler-angle + orthographic mode, ported from
    // multi_view_tls_registration's original rotate_x/rotate_y/
    // translate_x/y/z/rotation_center/is_ortho camera (driven through
    // rlgl's manual matrix stack -- rlMatrixMode/rlFrustum/rlOrtho/
    // rlMultMatrixf -- rather than raylib's Camera3D/BeginMode3D). Kept
    // entirely separate from the azimuth/elevation/distance/target fields
    // and methods above, which remain the camera_lidar_* apps' own model
    // and are untouched by any of this.
    // ------------------------------------------------------------------

    struct EulerState {
        float rotateX = -35.264f;
        float rotateY = 135.0f;
        Vector3 translate = { 0.f, 0.f, -50.f };
        Vector3 rotationCenter = { 0.f, 0.f, 0.f };
    };

    EulerState euler;

    // Eased-transition goal state for the Euler mode (separate from
    // transitionTarget/transitionActive above, which only ever ease
    // `target`).
    EulerState eulerGoal = euler;
    bool eulerTransitionActive = false;
    float eulerTransitionSpeed = 1.f;

    bool lockZ = false;
    bool isOrtho = false;
    float orthoZoom = 10.f;
    float orthoShiftX = 0.f, orthoShiftY = 0.f;
    float orthoZCenterH = 0.f;
    // Mirrors the original's app_state.mouse_sensitivity: fabs(translate.z)/100,
    // recomputed by zoom()/setEulerPreset(Reset) -- scales keyboard-shortcut
    // and perspective-pan step sizes so they still feel proportional at any
    // zoom level.
    float eulerMouseSensitivity = 1.f;

    // Ortho projection + ortho "gizmo view" matrices, rebuilt each frame by
    // updateOrtho() -- flat column-major float[16] (index = col*4+row), the
    // layout ImGuizmo::Manipulate expects. Distinct from the real rlgl
    // projection updateOrtho() also sets up (right-handed, [-1,1] depth,
    // via rlOrtho): these mirror the original's GLM-computed
    // left-handed/[0,1]-depth pair, used only for ImGuizmo's own matrices.
    float orthoProjection[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
    float orthoGizmoView[16]  = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

    // This frame's captured 3D view/projection -- set by captureFrameMatrices()
    // right after applyEuler.../updateOrtho() build them, and read back by
    // picking code (e.g. multi_view_tls_registration's GetLaserBeam) to build
    // an unproject ray. Defaults to identity so an out-of-sequence call
    // before the first captureFrameMatrices() degrades gracefully instead of
    // unprojecting through a zero matrix.
    Matrix frameView3D = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
    Matrix frameProj3D = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

    enum class EulerPreset { Front, Back, Left, Right, Top, Bottom, Iso, Reset };

    // Sets up rlgl's projection matrix stack for a perspective frame -- was
    // multi_view_tls_registration's reshape()'s !is_ortho branch. w/h should
    // be logical window size (the GL viewport itself is still sized from the
    // actual framebuffer, matching the original).
    void applyPerspectiveProjection(int w, int h) const;
    // Was reshape()'s is_ortho branch. `aspect` is the caller's display
    // width/height (e.g. ImGui's io.DisplaySize.x/y) -- kept as a plain
    // parameter rather than reading ImGui here, since OrbitCamera otherwise
    // depends on nothing but raylib.
    void applyOrthoProjection(float aspect) const;

    // Builds this frame's ortho projection + folds an eye/center/up lookAt
    // (derived from euler.rotateX/rotateY and the ortho pan/height state)
    // into rlgl's current (projection) matrix -- was updateOrthoView(),
    // minus the caller's own app-specific compass-rotation bookkeeping
    // (that line stays with the caller, which already owns an Eigen
    // dependency this widget deliberately doesn't have). Must be called
    // with RL_PROJECTION active and freshly loaded identity (display()'s
    // job, same as today), and leaves RL_MODELVIEW active + identity after,
    // like the original.
    void updateOrtho(float aspect);

    // Stores rlGetMatrixModelview()/rlGetMatrixProjection() into
    // frameView3D/frameProj3D. Call once per frame right after
    // applyPerspectiveProjection()+the view multiply, or updateOrtho() --
    // before any 2D/ImGui drawing resets the matrix stack.
    void captureFrameMatrices();

    // Unprojects screen point (x,y) through frameView3D/frameProj3D into a
    // world-space ray -- was multi_view_tls_registration's GetLaserBeam().
    // Pass the actual framebuffer width/height (e.g. GetScreenWidth()/
    // GetScreenHeight()), not a cached value, so a mid-frame resize can't
    // desync it from the viewport applyPerspectiveProjection()/
    // applyOrthoProjection() actually set up. Like GetLaserBeam(), this
    // reads whatever frameView3D/frameProj3D last captured -- typically the
    // *previous* frame's matrices when called from input handling that runs
    // before this frame's captureFrameMatrices(), which is deliberate (see
    // frameView3D's declaration comment).
    Ray eulerScreenRay(int x, int y, int screenW, int screenH) const;

    // Left-drag: orbit. Was motion()'s `mouse_buttons & 1` branch.
    void dragOrbit(float dx, float dy);
    // Right-drag in perspective mode: pan. Was motion()'s `mouse_buttons & 4`
    // branch, !is_ortho case.
    void dragPanPerspective(float dx, float dy);
    // Right-drag in ortho mode: pan. Was motion()'s `mouse_buttons & 4`
    // branch, is_ortho case. displayW/displayH: caller's display size (e.g.
    // ImGui's io.DisplaySize), needed for the same screen-space-to-world
    // scaling the original used.
    void dragPanOrtho(float dx, float dy, float displayW, float displayH);
    // Mouse-wheel zoom/dolly -- was wheel()'s camera-mutating branch, now
    // scaled by the actual per-frame scroll magnitude (raylib's
    // GetMouseWheelMove()) instead of only its sign. A physical mouse wheel
    // notch reports ~1.0, so passing that through reproduces the original's
    // fixed per-notch step exactly; a trackpad's continuous smooth-scroll
    // deltas (fractions of 1.0, many per frame during a gesture) now move
    // the camera proportionally instead of snapping a full step on every
    // one of those frames -- which is what made a trackpad pinch/scroll
    // feel like it "teleported" before this took magnitude into account.
    // Deliberately does NOT call breakEulerTransition() (which also snaps
    // rotationCenter) -- matches the original, which just cleared
    // camera_transition_active here, leaving rotationCenter wherever it had
    // eased to.
    void zoom(float wheelDelta, bool shiftHeld);

    // Was setCameraPreset(). Note: the original's CAMERA_RESET also reset
    // an app-specific viewer_decimate_point_cloud setting outside the
    // camera's own state -- callers still need to do that themselves.
    void setEulerPreset(EulerPreset preset);
    // General eased-transition start -- was the repeated
    // `new_rotation_center = ...; new_rotate_x = rotate_x; ...;
    // camera_transition_active = true;` pattern at each call site.
    void startEulerTransition(float rotateX, float rotateY, Vector3 translate, Vector3 rotationCenter);
    // Convenience for the common "recenter on a picked point" pattern (was
    // repeated in getClosestTrajectoryPoint()/setNewRotationCenter()/
    // cor_window()'s Set button): keeps the current rotate angles and Z
    // distance, moves rotationCenter to `center`, and sets translate.xy to
    // -center.xy so the new center recentres under the (unchanged) view.
    void moveEulerRotationCenterTo(Vector3 center);
    // Eases euler toward eulerGoal; call once per frame with the frame's
    // delta time. Was updateCameraTransition().
    void updateEulerTransition(float dt);
    // Was breakCameraTransition(): snaps euler.rotationCenter to
    // eulerGoal.rotationCenter and ends the transition. Note this leaves
    // rotateX/rotateY/translate wherever they had already eased to -- matches
    // the original exactly (asymmetric with updateEulerTransition's
    // full-snap-on-completion).
    void breakEulerTransition();
};

// Small 3-axis crosshair marking the current center of rotation. Call inside
// BeginMode3D/EndMode3D, every frame -- drawing at camera.target (which
// updateTransition() eases each frame) makes the cross visibly slide to a
// newly-picked/typed center along with the camera.
void drawRotationCenterCross(Vector3 center, float size, Color color);

// Restores rlgl's default 2D screen-space projection (matches what raylib's
// own EndMode3D() does) -- for apps that drive the rlgl matrix stack
// manually each frame (rlMatrixMode/rlFrustum/rlOrtho/rlMultMatrixf, e.g.
// via OrbitCamera's Euler-mode applyPerspectiveProjection()/updateOrtho())
// instead of using raylib's BeginMode3D/EndMode3D wrapper, which can't be
// mixed with that -- was multi_view_tls_registration's end3DMatrixStack().
// Must be called after all 3D drawing and before any 2D drawing (a mini-
// compass, ImGui) each frame. displayW/displayH should be the caller's
// logical display size (e.g. ImGui's io.DisplaySize.x/y), not
// GetScreenWidth()/Height() -- those can differ under DPI scaling, and this
// has to match whatever the viewport was actually set from.
void end3DMatrixStack(float displayW, float displayH);

} // namespace raylib_widgets
