#pragma once
#include <Eigen/Geometry>
#include <imgui.h>

const std::string out_fn = "Output file name";

static constexpr float ImGuiNumberWidth = 120.0f;
static constexpr const char* omText = "Roll (left/right)";
static constexpr const char* fiText = "Pitch (up/down)";
static constexpr const char* kaText = "Yaw (turning left/right)";
static constexpr const char* xText = "Longitudinal (forward/backward)";
static constexpr const char* yText = "Lateral (left/right)";
static constexpr const char* zText = "Vertical (up/down)";

void drawMiniCompassWithRuler(
    const Eigen::Affine3f& viewLocal,
    float translate_z,
    const ImVec4& bg_color,
    ImVec2 compassSize = ImVec2(200, 200));



// Declare globals for updateCameraTransition
extern bool camera_transition_active;
extern const float camera_transition_speed;
extern float rotate_x, rotate_y;
extern float target_rotate_x, target_rotate_y;
extern float translate_x, translate_y, translate_z;
extern float target_translate_x, target_translate_y, target_translate_z;

void updateCameraTransition(float deltaTime);



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
#ifdef ENABLE_ORTHO_SETTINGS
extern int viewer_decimate_point_cloud;
extern double camera_ortho_xy_view_zoom;
extern double camera_ortho_xy_view_shift_x;
extern double camera_ortho_xy_view_shift_y;
extern double camera_ortho_xy_view_rotation_angle_deg;
extern double camera_mode_ortho_z_center_h;
#endif

void setCameraPreset(CameraPreset preset);


void ImGuiHyperlink(const char* url, ImVec4 color = ImVec4(0.2f, 0.4f, 0.8f, 1.0f));



void info_window(const std::vector<std::string>& infoLines, bool* open = nullptr);



std::string truncPath(const std::string& fullPath);