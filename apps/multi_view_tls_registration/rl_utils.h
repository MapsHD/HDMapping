#pragma once

// raylib-based replacement for the app-agnostic camera/picking/mini-compass/
// misc-ImGui-widget API that this app used to get from <Core/utils.hpp>
// (core/src/utils.cpp). That file is shared by several other GLUT apps and
// can't be changed, and raylib's context here is OpenGL 3.3 core profile (no
// fixed-function pipeline), so this header/its .cpp are a from-scratch
// reimplementation of the same API surface -- same names, same call shape --
// backed by rlgl's rl*() legacy-GL-emulation API (a software matrix stack +
// immediate-mode layer that mirrors gl*()'s call shape but works under core
// profile) instead of real gl*()/glu*()/glut*() calls. See rl_utils.cpp's
// top comment for the function-by-function porting notes.
//
// Deliberately not shared with any other app (unlike Core/utils.hpp): this
// is multi_view_tls_registration's own local header, analogous to how
// core/src/utils.cpp served the same role for the GLUT apps.

#include "raylib.h"

#include <imgui.h>

#include <Core/registration_plane_feature.h>
#include <Core/session.h>
#include <Core/structures.h>

#include <Eigen/Geometry>

#include <string>
#include <vector>

///////////////////////////////////////////////////////////////////////////////////

const float DEG_TO_RAD = M_PI / 180.0f;
const float RAD_TO_DEG = 180.0f / M_PI;

const ImVec4 orangeBorder(1.0f, 0.5f, 0.0f, 1.0f);

const std::string out_fn = "Output file name";

constexpr float ImGuiNumberWidth = 120.0f;
constexpr const char* omText = "Roll (left/right)";
constexpr const char* fiText = "Pitch (up/down)";
constexpr const char* kaText = "Yaw (turning left/right)";
constexpr const char* xText = "Longitudinal (forward/backward)";
constexpr const char* yText = "Lateral (left/right)";
constexpr const char* zText = "Vertical (up/down)";

const uint32_t window_width = 1600;
const uint32_t window_height = 900;

const float camera_transition_speed = 1.0f; // higher = faster

enum CameraPreset
{
    CAMERA_FRONT,
    CAMERA_BACK,
    CAMERA_LEFT,
    CAMERA_RIGHT,
    CAMERA_TOP,
    CAMERA_BOTTOM,
    CAMERA_ISO,
    CAMERA_RESET
};

enum ColorScheme
{
    CS_SOLID, // fixed color
    CS_RANDOM, // random
    CS_GRAD_INTENS, // gradient based on intensity
    CS_GRAD_ELEV, // gradient based on elevation
    CS_GRAD_DIST, // gradient based on distance from rotation center
    CS_FOLLOW // valid for trajectory
};

///////////////////////////////////////////////////////////////////////////////////

extern int viewer_decimate_point_cloud;

extern int mouse_old_x, mouse_old_y;
extern int mouse_buttons;
extern float mouse_sensitivity;

extern bool is_ortho;
extern bool lock_z;
extern bool show_axes;
extern ImVec4 bg_color;
extern int point_size;

extern bool info_gui;
extern bool compass_ruler;

extern Eigen::Affine3f viewLocal;

extern Eigen::Vector3f rotation_center;
extern float rotate_x, rotate_y;
extern float translate_x, translate_y, translate_z;

extern double camera_ortho_xy_view_zoom;
extern double camera_ortho_xy_view_shift_x;
extern double camera_ortho_xy_view_shift_y;
extern double camera_mode_ortho_z_center_h;

// Target camera state for smooth transitions
extern Eigen::Vector3f new_rotation_center;
extern float new_rotate_x;
extern float new_rotate_y;
extern float new_translate_x;
extern float new_translate_y;
extern float new_translate_z;

// Transition timing
extern bool camera_transition_active;

// The 3D view/projection rlgl had active during this frame's scene render,
// cached by display() right before end3DMatrixStack() resets rlgl's matrix
// stack to the 2D screen-space ortho used for the mini-compass/ImGui pass.
// GetLaserBeam() (called from mouse(), which runs *before* display() each
// frame -- see main()) needs these: querying rlGetMatrixModelview()/
// rlGetMatrixProjection() live at that point would still see the previous
// frame's post-end3DMatrixStack() state (identity modelview, 2D ortho
// projection), not the 3D camera, producing a meaningless pick ray.
extern Matrix frame_view_3d;
extern Matrix frame_proj_3d;

// Unlike the original (which probed GL_LINE_WIDTH_RANGE), rlgl's line width
// support is uniform enough here not to need a runtime check -- always true.
extern bool glLineWidthSupport;

extern float m_ortho_projection[];
extern float m_ortho_gizmo_view[];

struct ShortcutEntry
{
    std::string type;
    std::string shortcut;
    std::string description;
};

///////////////////////////////////////////////////////////////////////////////////

std::string truncPath(const std::string& fullPath);

void wheel(int button, int dir, int x, int y);
void reshape(int w, int h);
void motion(int x, int y);
void ShowMainDockSpace();

void showAxes();
void updateCameraTransition();
void breakCameraTransition();
void setCameraPreset(CameraPreset preset);
void camMenu();
void view_kbd_shortcuts();
void cor_window();

void ImGuiHyperlink(const char* url, ImVec4 color = ImVec4(0.2f, 0.4f, 0.8f, 1.0f));
void ShowShortcutsTable(const std::vector<ShortcutEntry> appShortcuts);
void info_window(const std::vector<std::string>& infoLines, const std::vector<ShortcutEntry>& appShortcuts);

void drawMiniCompassWithRuler();

float distanceToPlane(const RegistrationPlaneFeature::Plane& plane, const Eigen::Vector3d& p);
Eigen::Vector3d rayIntersection(const LaserBeam& laser_beam, const RegistrationPlaneFeature::Plane& plane);
LaserBeam GetLaserBeam(int x, int y);
double distance_point_to_line(const Eigen::Vector3d& point, const LaserBeam& line);
void getClosestTrajectoryPoint(Session& session_, int x, int y, bool gcpPicking, int& picked_index);

void setNewRotationCenter(int x, int y);

bool checkClHelp(int argc, char** argv);

void updateOrthoView();

// New (no equivalent in the original <Core/utils.hpp>): restores rlgl's
// default 2D screen-space projection (matches what raylib's own
// EndMode3D() does), since this app drives the rlgl matrix stack manually
// (rlMatrixMode/rlFrustum/rlMultMatrixf in reshape()/display()) instead of
// using raylib's BeginMode3D/EndMode3D wrapper. Must be called after all 3D
// drawing and before any 2D drawing (the mini-compass, ImGui) each frame.
void end3DMatrixStack();
