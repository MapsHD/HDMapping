#pragma once
#include <Eigen/Geometry>
#include <imgui.h>
#include <structures.h>
#include <registration_plane_feature.h>

///////////////////////////////////////////////////////////////////////////////////

const std::string out_fn = "Output file name";

constexpr float ImGuiNumberWidth = 120.0f;
constexpr const char* omText = "Roll (left/right)";
constexpr const char* fiText = "Pitch (up/down)";
constexpr const char* kaText = "Yaw (turning left/right)";
constexpr const char* xText = "Longitudinal (forward/backward)";
constexpr const char* yText = "Lateral (left/right)";
constexpr const char* zText = "Vertical (up/down)";

const unsigned int window_width = 800;
const unsigned int window_height = 600;

const float camera_transition_speed = 1.0f; // higher = faster

enum CameraPreset {
    CAMERA_FRONT,
    CAMERA_BACK,
    CAMERA_LEFT,
    CAMERA_RIGHT,
    CAMERA_TOP,
    CAMERA_BOTTOM,
    CAMERA_ISO,
    CAMERA_RESET
};

///////////////////////////////////////////////////////////////////////////////////

extern int viewer_decimate_point_cloud;

extern int mouse_old_x, mouse_old_y;
extern bool gui_mouse_down;
extern int mouse_buttons;
extern float mouse_sensitivity;

extern bool is_ortho;
extern bool show_axes;
extern ImVec4 clear_color;
extern int point_size;

extern bool info_gui;
extern bool compass_ruler;

extern Eigen::Vector3f rotation_center;
extern float rotate_x, rotate_y;
extern float translate_x, translate_y;
extern float translate_z;

extern double camera_ortho_xy_view_zoom;
extern double camera_ortho_xy_view_shift_x;
extern double camera_ortho_xy_view_shift_y;
extern double camera_ortho_xy_view_rotation_angle_deg;
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

struct ShortcutEntry {
    std::string type;
    std::string shortcut;
    std::string description;
};

#define IDI_ICON1                        101 //application icon (double defined in resource.h)

///////////////////////////////////////////////////////////////////////////////////

std::string truncPath(const std::string& fullPath);

void wheel(int button, int dir, int x, int y);
void reshape(int w, int h);
void motion(int x, int y);
void specialDown(int key, int x, int y);
void specialUp(int key, int x, int y);
bool initGL(int* argc, char** argv, const std::string& winTitle, void (*display)(), void (*mouse)(int, int, int, int));

void showAxes();
void updateCameraTransition();
void breakCameraTransition();
void setCameraPreset(CameraPreset preset);
void camMenu();
void view_kbd_shortcuts();

void ImGuiHyperlink(const char* url, ImVec4 color = ImVec4(0.2f, 0.4f, 0.8f, 1.0f));
void info_window(const std::vector<std::string>& infoLines, const std::vector<ShortcutEntry>& appShortcuts, bool* open);

void drawMiniCompassWithRuler(
    const Eigen::Affine3f& viewLocal,
    float translate_z,
    const ImVec4& bg_color,
    ImVec2 compassSize = ImVec2(200, 200));

Eigen::Vector3d rayIntersection(const LaserBeam& laser_beam, const RegistrationPlaneFeature::Plane& plane);
LaserBeam GetLaserBeam(int x, int y);
void setNewRotationCenter(int x, int y);