#pragma once

// raylib replacement for the parts of <Core/utils.hpp> (GLUT + legacy OpenGL) that the raylib-based
// apps use: same names, so a ported app keeps its code and only its GL calls change. Camera state
// lives in raylib_widgets::OrbitCamera; the old globals are references into it.
//
// Shared by lidar_odometry_step_1, mandeye_raw_data_viewer and multi_session_registration (step 3,
// whose own raylib_utils.h adds the session-rendering helpers on top). Built as core_raylib_utils
// (core/CMakeLists.txt), separate from core_raylib, which deliberately does not depend on ImGui.

#include "raylib.h"
#include "rlgl.h"

#include <imgui.h>

#include <Eigen/Eigen>

#include <Core/registration_plane_feature.h>
#include <Core/structures.h>

#include <RaylibWidgets/AppShell.h>
#include <RaylibWidgets/OrbitCamera.h>
#include <RaylibWidgets/PointBufferPart.h>
#include <RaylibWidgets/ShortcutsTable.h>

#include <string>
#include <vector>

using raylib_widgets::ShortcutEntry;
using raylib_widgets::ShowMainDockSpace;

constexpr float DEG_TO_RAD = M_PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / M_PI;
constexpr float ImGuiNumberWidth = 120.0f;
constexpr const char* omText = "Roll (left/right)";
constexpr const char* fiText = "Pitch (up/down)";
constexpr const char* kaText = "Yaw (turning left/right)";
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
//! Like utils.cpp's reshape(): sets the perspective projection, or the plain ortho one while is_ortho.
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
//! Was utils.cpp's draw_ellipse() (GL_LINE_LOOP per lat/long cell); emitted as RL_LINES.
void draw_ellipse(const Eigen::Matrix3d& covar, const Eigen::Vector3d& mean, Eigen::Vector3f color, float nstd);
Eigen::Vector3d rayIntersection(const LaserBeam& laser_beam, const RegistrationPlaneFeature::Plane& plane);
LaserBeam GetLaserBeam(int x, int y);
double distance_point_to_line(const Eigen::Vector3d& point, const LaserBeam& line);
void setNewRotationCenter(int x, int y);
//! Moves the rotation center directly to a known world point (e.g. the gizmo's position),
//! skipping the screen-pick raycast the (x, y) overload does.
void setNewRotationCenter(const Eigen::Vector3d& center);
bool checkClHelp(int argc, char** argv);

//! Replaces glutMainLoop(): polls raylib input into the mouse()/motion()/wheel() callbacks, calls display() per frame.
//! @param close_requested called when the window is asked to close; returning false keeps it open. Null = always close.
void mainLoop(bool (*close_requested)() = nullptr);
//! Releases GPU buffers and closes the window (replaces the ImGui GLUT/OpenGL2 backend shutdown).
void shutdownGL();

//! Copies the current rlgl projection/modelview (column-major, for ImGuizmo); replaces glGetFloatv(GL_*_MATRIX).
void getProjectionMatrix(float m[16]);
void getModelviewMatrix(float m[16]);
//! Stores this frame's 3D matrices for picking and labels; call once the camera transform is set.
void captureFrameMatrices();

//! GL-style immediate helpers: color3f() also sets the color labels, line strips and points pick up (like glColor3f).
void color3f(float r, float g, float b);
//! rlgl has no line strips; these collect a strip and emit it as RL_LINES segments.
void beginLineStrip();
void lineStripVertex3f(float x, float y, float z);
void endLineStrip();
//! rlgl's batch has no points either; these replace glPointSize + glBegin(GL_POINTS) ... glEnd(). Points are
//! collected with their current color3f() and drawn in one go by endPoints() with a per-vertex-color shader.
void beginPoints(float size);
void pointVertex3f(float x, float y, float z);
void endPoints();
//! The same position/color vertex used by beginPoints()/endPoints().
struct PointVertex
{
    float x, y, z;
    float r, g, b;
};
//! Retained geometry using the existing point shader and raylib buffer parts. Upload only when data changes.
void uploadPointVertices(std::vector<raylib_widgets::PointBufferPart>& parts, const std::vector<PointVertex>& vertices);
void unloadPointVertices(std::vector<raylib_widgets::PointBufferPart>& parts);
void drawPointVertices(
    const std::vector<raylib_widgets::PointBufferPart>& parts, float size, const Eigen::Affine3d& transform = Eigen::Affine3d::Identity());
void drawLineVertices(
    const std::vector<raylib_widgets::PointBufferPart>& parts,
    const Eigen::Affine3d& transform = Eigen::Affine3d::Identity(),
    Color tint = WHITE);
//! Draw a unit covariance mesh with a precomputed covariance/translation transform.
void draw_ellipse(const Eigen::Affine3d& transform, Eigen::Vector3f color);
//! Intensity range of a point set, for color3fIntensityShaded(). stride > 1 scans every stride-th point,
//! matching a decimated draw loop.
struct IntensityRange
{
    float min = 0.f;
    float max = 0.f;
    float normalize(float intensity) const;
};
IntensityRange intensityRange(const std::vector<Point3Di>& points, size_t stride = 1);
//! color3f() of (r, g, b) shaded by the point's intensity, normalized over `range` -- the same shading as
//! ScanRenderer's ScanColorMode::FlatIntensity, so these points look like the step 2/3 renders.
void color3fIntensityShaded(float r, float g, float b, float intensity, const IntensityRange& range);
//! Replace glRasterPos3f + glutBitmapString: queue a text label at a world position, in the current color3f().
void labelPos3f(float x, float y, float z);
void labelText(const std::string& text);
//! Queues a text label at a world position in an explicit color.
void addLabel(const Vector3& position, const std::string& text, Color color);
//! Switches to 2D and draws the queued labels; call after all 3D drawing, before the compass.
void end3DAndDrawLabels();

//! Paths dropped onto the window since the last call (empty if none); call once per frame.
std::vector<std::string> takeDroppedFiles();

#ifdef _WIN32
//! Same as Core/utils.hpp: Windows taskbar progress for this app's window.
void InitTaskbarProgress();
void SetTaskbarProgress(double progress01);
void ClearTaskbarProgress();
#endif
