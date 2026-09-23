#pragma once

// raylib replacement for the parts of <Core/utils.hpp> (GLUT + legacy OpenGL) that step 3 uses:
// same names, so multi_session_registration.cpp keeps its code and only its GL calls change.
// Camera state lives in raylib_widgets::OrbitCamera; the old globals are references into it.

#include "raylib.h"
#include "rlgl.h"

#include <imgui.h>

#include <Eigen/Eigen>

#include <Core/raylib_render.hpp>
#include <Core/registration_plane_feature.h>
#include <Core/session.h>
#include <Core/structures.h>

#include <RaylibWidgets/AppShell.h>
#include <RaylibWidgets/OrbitCamera.h>
#include <RaylibWidgets/ShortcutsTable.h>

#include <string>
#include <vector>

using raylib_widgets::ShortcutEntry;
using raylib_widgets::ShowMainDockSpace;

constexpr float DEG_TO_RAD = M_PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / M_PI;
constexpr float ImGuiNumberWidth = 120.0f;
constexpr const char* xText = "Longitudinal (forward/backward)";
constexpr const char* yText = "Lateral (left/right)";
constexpr const char* zText = "Vertical (up/down)";

// GLUT mouse codes, so mouse() keeps its GLUT-callback shape.
constexpr int GLUT_LEFT_BUTTON = 0;
constexpr int GLUT_MIDDLE_BUTTON = 1;
constexpr int GLUT_RIGHT_BUTTON = 2;
constexpr int GLUT_DOWN = 0;
constexpr int GLUT_UP = 1;

extern raylib_widgets::OrbitCamera camera;

extern int viewer_decimate_point_cloud;
extern int mouse_old_x, mouse_old_y;
extern int mouse_buttons;
extern bool& is_ortho;
extern bool& lock_z;
extern bool show_axes;
extern ImVec4 bg_color;
extern int point_size;
extern bool info_gui;
extern bool compass_ruler;
extern bool cor_gui;
extern Eigen::Affine3f viewLocal;

extern Eigen::Map<Eigen::Vector3f> rotation_center;
extern float& rotate_x;
extern float& rotate_y;
extern float& translate_x;
extern float& translate_y;
extern float& translate_z;
extern Eigen::Map<Eigen::Vector3f> new_rotation_center;
extern float& new_rotate_x;
extern float& new_rotate_y;
extern float& new_translate_x;
extern float& new_translate_y;
extern float& new_translate_z;
extern bool& camera_transition_active;
extern float* const m_ortho_projection;
extern float* const m_ortho_gizmo_view;

//! Same as Core/utils.hpp; the GL/GLUT ones are implemented with raylib/rlgl.
std::string truncPath(const std::string& fullPath);
void wheel(int button, int dir, int x, int y);
void motion(int x, int y);
void reshape(int w, int h);
bool initGL(int* argc, char** argv, const std::string& winTitle, void (*display)(), void (*mouse)(int, int, int, int));
void showAxes();
void updateCameraTransition();
void updateOrthoView();
void camMenu();
void view_kbd_shortcuts();
void cor_window();
void info_window(const std::vector<std::string>& infoLines, const std::vector<ShortcutEntry>& appShortcuts);
void drawMiniCompassWithRuler();
Eigen::Vector3d rayIntersection(const LaserBeam& laser_beam, const RegistrationPlaneFeature::Plane& plane);
LaserBeam GetLaserBeam(int x, int y);
double distance_point_to_line(const Eigen::Vector3d& point, const LaserBeam& line);
void getClosestTrajectoriesPoint(
    std::vector<Session>& sessions,
    int x,
    int y,
    const int first_session_index,
    const int second_session_index,
    const int number_visible_sessions,
    int& index_loop_closure_source,
    int& index_loop_closure_target,
    bool KeyShift,
    double& time_stamp_offset);
void setNewRotationCenter(int x, int y);
bool checkClHelp(int argc, char** argv);

//! Replaces glutMainLoop(): polls raylib input into the mouse()/motion()/wheel() callbacks, calls display() per frame.
void mainLoop();
//! Releases GPU buffers and closes the window (replaces the ImGui GLUT/OpenGL2 backend shutdown).
void shutdownGL();

//! Copies the current rlgl projection/modelview (column-major, for ImGuizmo); replaces glGetFloatv(GL_*_MATRIX).
void getProjectionMatrix(float m[16]);
void getModelviewMatrix(float m[16]);
//! Stores this frame's 3D matrices for picking and labels; call once the camera transform is set.
void captureFrameMatrices();

//! GL-style immediate helpers: color3f() also sets the color labels and line strips pick up (like glColor3f).
void color3f(float r, float g, float b);
//! rlgl has no line strips; these collect a strip and emit it as RL_LINES segments.
void beginLineStrip();
void lineStripVertex3f(float x, float y, float z);
void endLineStrip();
//! Replace glRasterPos3f + glutBitmapString: queue a text label at a world position, in the current color3f().
void labelPos3f(float x, float y, float z);
void labelText(const std::string& text);
//! Switches to 2D and draws the queued labels; call after all 3D drawing, before the compass.
void end3DAndDrawLabels();

//! Per-session GPU buffers (ScanRenderer). syncSessionRenderers() runs once per frame; it re-uploads after
//! invalidateSessionRenderers() or when the number of sessions changes, else only scans whose pose changed.
void syncSessionRenderers(const std::vector<Session>& sessions);
void invalidateSessionRenderers();
//! Replaces PointClouds::render(): session.visible is checked by the caller.
void renderSession(const Session& session, const ObservationPicking& observation_picking, int decimate, int reduce_trajectory);
//! Replaces PointCloud::render(show_with_initial_pose, ...) for scan `index` of session `session_index`.
void renderScan(
    int session_index,
    int index,
    bool show_with_initial_pose,
    const ObservationPicking& observation_picking,
    int decimate,
    int reduce_trajectory,
    bool xz_intersection,
    bool yz_intersection,
    bool xy_intersection,
    double intersection_width,
    bool visible_imu_diff);
//! Replaces PointCloud::render(pose, ...): points drawn at `pose`, trajectory at the scan's m_pose.
void renderScanAtPose(int session_index, int index, const Eigen::Affine3d& pose, int decimate, int reduce_trajectory, const float color[3]);
//! Replace GroundControlPoints::render() / ControlPoints::render(pcs, false), which use legacy GL.
void renderGroundControlPoints(const GroundControlPoints& ground_control_points, const PointClouds& point_clouds_container);
void renderControlPoints(const ControlPoints& control_points, const PointClouds& point_clouds_container);

//! View > Points color: ScanRenderer's shader color modes (default: session color shaded by intensity).
void pointsColorMenu();
//! After loading: red, blue, orange, ... for the first 8 sessions, then a random vivid hue (seeded by index).
void assignDistinctSessionColors(std::vector<Session>& sessions);
