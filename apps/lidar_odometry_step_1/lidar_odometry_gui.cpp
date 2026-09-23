// raylib + rlImGui GUI (was GLUT + legacy immediate-mode OpenGL via core/src/utils.cpp). raylib's context is
// OpenGL 3.3 core profile, so the scene is drawn with rlgl's rl*() immediate-mode emulation (lines) and
// ScanRenderer::PointsGPU buffers (points); camera, compass, docking and dialogs come from raylib_widgets,
// the same way apps/multi_view_tls_registration (step 2) does it.
#include "raylib.h"
#include "raymath.h"
#include "rlImGui.h"
#include "rlgl.h"

#include <imgui.h>
#include <imgui_internal.h>

#include <ImGuizmo.h>

#ifdef _WIN32
// portable-file-dialogs.h pulls in windows.h, whose CloseWindow(HWND)/ShowCursor(BOOL) collide with raylib's
// -- see multi_view_tls_registration_gui.cpp for the full story.
#define CloseWindow CloseWindow_win32
#define ShowCursor ShowCursor_win32
#endif
#include <portable-file-dialogs.h>
#ifdef _WIN32
#undef CloseWindow
#undef ShowCursor
#undef DrawText
#endif

#include "lidar_odometry.h"
#include "lidar_odometry_utils.h"
#include "taskbar_progress.h"

#include <Core/export_laz.h>
#include <Core/hash_utils.h>
#include <Core/pfd_wrapper.hpp>
#include <Core/raylib_render.hpp>
#include <Core/registration_plane_feature.h>
#include <Core/session.h>

#include <RaylibWidgets/AppShell.h>
#include <RaylibWidgets/CenterOfRotationWindow.h>
#include <RaylibWidgets/CompassRuler.h>
#include <RaylibWidgets/OrbitCamera.h>
#include <RaylibWidgets/RayPlaneD.h>
#include <RaylibWidgets/ShortcutsTable.h>
#include <RaylibWidgets/WindowFit.h>

#include "toml_io.h"
#include <HDMapping/Version.hpp>
#include <chrono>
#include <ctime>
#include <mutex>
#include <spdlog/cfg/env.h>
#include <spdlog/spdlog.h>

///////////////////////////////////////////////////////////////////////////////////

// This is LiDAR odometry (step 1)
// This program calculates trajectory based on IMU and LiDAR data provided by MANDEYE mobile mapping system
// https://github.com/JanuszBedkowski/mandeye_controller The output is a session proving trajekctory and point clouds that can be  further
// processed by "multi_view_tls_registration" program.

// #define SAMPLE_PERIOD (1.0 / 200.0)

using raylib_widgets::ShortcutEntry;

// Formerly from <Core/utils.hpp>, which is GLUT-only.
const float DEG_TO_RAD = M_PI / 180.0f;

constexpr float ImGuiNumberWidth = 120.0f;
constexpr const char* omText = "Roll (left/right)";
constexpr const char* fiText = "Pitch (up/down)";
constexpr const char* kaText = "Yaw (turning left/right)";
constexpr const char* xText = "Longitudinal (forward/backward)";
constexpr const char* yText = "Lateral (left/right)";
constexpr const char* zText = "Vertical (up/down)";

const uint32_t window_width = 1600;
const uint32_t window_height = 900;

std::string winTitle = std::string("Step 1 (Lidar odometry) ") + HDMAPPING_VERSION_STRING;

std::vector<std::string> infoLines = {
    "This program is first step in MANDEYE process.",
    "",
    "It results trajectory and point clouds as single session for 'multi_view_tls_registration_step_2' program.",
    "",
    "Next step will be to load session.json file with 'multi_view_tls_registration_step_2' program."
};

// App specific shortcuts (Type and Shortcut are just for easy reference)
static const std::vector<ShortcutEntry> appShortcuts = { { "Normal keys", "A", "" },
                                                         { "", "Ctrl+A", "" },
                                                         { "", "B", "" },
                                                         { "", "Ctrl+B", "" },
                                                         { "", "C", "" },
                                                         { "", "Ctrl+C", "" },
                                                         { "", "D", "" },
                                                         { "", "Ctrl+D", "" },
                                                         { "", "E", "" },
                                                         { "", "Ctrl+E", "" },
                                                         { "", "F", "" },
                                                         { "", "Ctrl+F", "" },
                                                         { "", "G", "" },
                                                         { "", "Ctrl+G", "" },
                                                         { "", "H", "" },
                                                         { "", "Ctrl+H", "" },
                                                         { "", "I", "" },
                                                         { "", "Ctrl+I", "" },
                                                         { "", "J", "" },
                                                         { "", "Ctrl+K", "" },
                                                         { "", "K", "" },
                                                         { "", "Ctrl+K", "" },
                                                         { "", "L", "" },
                                                         { "", "Ctrl+L", "" },
                                                         { "", "M", "" },
                                                         { "", "Ctrl+M", "" },
                                                         { "", "N", "" },
                                                         { "", "Ctrl+N", "" },
                                                         { "", "O", "" },
                                                         { "", "Ctrl+O", "Open data" },
                                                         { "", "P", "" },
                                                         { "", "Ctrl+P", "" },
                                                         { "", "Q", "" },
                                                         { "", "Ctrl+Q", "" },
                                                         { "", "R", "" },
                                                         { "", "Ctrl+R", "" },
                                                         { "", "Shift+R", "" },
                                                         { "", "S", "" },
                                                         { "", "Ctrl+S", "" },
                                                         { "", "Ctrl+Shift+S", "" },
                                                         { "", "T", "" },
                                                         { "", "Ctrl+T", "" },
                                                         { "", "U", "" },
                                                         { "", "Ctrl+U", "" },
                                                         { "", "V", "" },
                                                         { "", "Ctrl+V", "" },
                                                         { "", "W", "" },
                                                         { "", "Ctrl+W", "" },
                                                         { "", "X", "" },
                                                         { "", "Ctrl+X", "" },
                                                         { "", "Y", "" },
                                                         { "", "Ctrl+Y", "" },
                                                         { "", "Z", "" },
                                                         { "", "Ctrl+Z", "" },
                                                         { "", "Shift+Z", "" },
                                                         { "", "1-9", "" },
                                                         { "Special keys", "Up arrow", "" },
                                                         { "", "Shift + up arrow", "" },
                                                         { "", "Ctrl + up arrow", "" },
                                                         { "", "Down arrow", "" },
                                                         { "", "Shift + down arrow", "" },
                                                         { "", "Ctrl + down arrow", "" },
                                                         { "", "Left arrow", "" },
                                                         { "", "Shift + left arrow", "" },
                                                         { "", "Ctrl + left arrow", "" },
                                                         { "", "Right arrow", "" },
                                                         { "", "Shift + right arrow", "" },
                                                         { "", "Ctrl + right arrow", "" },
                                                         { "", "Pg down", "" },
                                                         { "", "Pg up", "" },
                                                         { "", "- key", "" },
                                                         { "", "+ key", "" },
                                                         { "Mouse related", "Left click + drag", "" },
                                                         { "", "Right click + drag", "n" },
                                                         { "", "Scroll", "" },
                                                         { "", "Shift + scroll", "" },
                                                         { "", "Shift + drag", "" },
                                                         { "", "Ctrl + left click", "" },
                                                         { "", "Ctrl + right click", "" },
                                                         { "", "Ctrl + middle click", "" } };

namespace fs = std::filesystem;

bool is_settings_gui = false;
bool full_debug_messages = false;
NDT ndt;
bool show_reference_buckets_indoor = false;
bool show_reference_buckets_outdoor = false;
bool show_reference_points = false;
int dec_reference_points = 100;
bool show_initial_points = true;
bool show_trajectory = true;
bool show_trajectory_as_axes = false;
bool show_prediction_vectors = false;
bool intermediate_trajectory_prediction_axes = false;
bool simple_gui = true;
bool step_1_done = false;
bool step_2_done = false;
bool step_3_done = false;
bool calculations_failed = false;
bool show_without_filtered_buckets = false;
bool show_normal_vectors_indoor = false;
bool show_normal_vectors_outdoor = false;

int lastPar = 1;

std::vector<WorkerData> worker_data;

std::string working_directory = "";
bool initial_transformation_gizmo = false;

float m_gizmo[] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

float x_displacement = 0.01;

int index_begin = -1;
int index_end = -1;
bool gizmo_stretch_interval = false;
Eigen::Affine3d stretch_gizmo_m = Eigen::Affine3d::Identity();

LidarOdometryParams params;
std::vector<std::string> csv_files;
std::vector<std::string> sn_files;
std::string imuSnToUse;
Session session;

std::vector<std::vector<Point3Di>> pointsPerFile;
Imu imu_data;
Trajectory trajectory;
std::atomic<bool> loRunning{ false };
std::atomic<float> loProgress{ 0.0 };
std::atomic<bool> loPause{ false };
std::chrono::time_point<std::chrono::system_clock> loStartTime;
std::atomic<double> loElapsedSeconds{ 0.0 };
std::atomic<double> loEstimatedTimeRemaining{ 0.0 };

fs::path outwd;

#if _WIN32
#define DEFAULT_PATH "C:\\"
#else
#define DEFAULT_PATH "~"
#endif

///////////////////////////////////////////////////////////////////////////////////
// View state and camera/input helpers (formerly the globals and functions of core/src/utils.cpp)

struct AppState
{
    int mouse_old_x = 0, mouse_old_y = 0;
    int mouse_buttons = 0; // bit 0: left, bit 2: right (GLUT numbering, as motion() expects)
    bool show_axes = true;
    ImVec4 bg_color = ImVec4(0.65f, 0.65f, 0.65f, 1.00f);
    int point_size = 1;

    bool info_gui = false;
    bool compass_ruler = true;

    Eigen::Affine3f viewLocal = Eigen::Affine3f::Identity(); // rebuilt from camera every frame
    raylib_widgets::OrbitCamera camera; // Euler/ortho mode only
};

AppState app_state;

bool cor_gui = false; // edge-triggered request to open the center-of-rotation dialog (Shift+R)

bool scroll_hint_enabled = true;
bool scroll_hint_active = false;
int scroll_hint_count = 0;
float scroll_hint_accu = 0.0f;
double scroll_hint_lastT = 0.0;

ScanRenderer scan_renderer; // only its PointsGPU helpers are used here -- this app renders no PointCloud scans

// GPU buffers for the scene's point sets. The static ones are re-uploaded only when their source changes; the
// ones that grow while odometry runs in the background are re-uploaded every frame they are shown.
struct ScenePoints
{
    ScanRenderer::PointsGPU initial;
    size_t initial_count = 0;
    Eigen::Matrix4d initial_m_g = Eigen::Matrix4d::Zero();
    ScanRenderer::PointsGPU reference;
    size_t reference_count = 0;
    int reference_dec = 0;
    ScanRenderer::PointsGPU trajectory;
    std::vector<Eigen::Vector3d> trajectory_uploaded;
    ScanRenderer::PointsGPU buckets[4]; // indoor, indoor filtered, outdoor, outdoor filtered
    std::vector<Eigen::Vector3d> buckets_uploaded[4];
} scene_points;

//! Uploads `pts` to `gpu` only if they differ from what was last uploaded.
//! @param uploaded copy of the points currently in `gpu`, updated on upload.
void uploadIfChanged(ScanRenderer::PointsGPU& gpu, std::vector<Eigen::Vector3d>& uploaded, std::vector<Eigen::Vector3d>& pts)
{
    if (pts == uploaded)
        return;
    scan_renderer.uploadPoints(gpu, pts);
    uploaded.swap(pts);
}

void unloadScenePoints()
{
    scan_renderer.unloadPoints(scene_points.initial);
    scan_renderer.unloadPoints(scene_points.reference);
    scan_renderer.unloadPoints(scene_points.trajectory);
    for (auto& b : scene_points.buckets)
        scan_renderer.unloadPoints(b);
}

bool checkClHelp(int argc, char** argv)
{
    for (int i = 1; i < argc; ++i)
    {
        std::string arg(argv[i]);
        if (arg == "-h" || arg == "/h" || arg == "--help" || arg == "/?")
            return true;
    }
    return false;
}

void wheel(float wheelMove)
{
    ImGuiIO& io = ImGui::GetIO();

    if (ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow))
        return;

    app_state.camera.zoom(wheelMove, io.KeyShift);

    if (scroll_hint_enabled)
    {
        if (!scroll_hint_active)
        {
            scroll_hint_accu += 1.0f;
            if (scroll_hint_accu > 30.0f)
            {
                scroll_hint_accu = 0.0f;
                scroll_hint_active = true;
                scroll_hint_count++;
            }
        }

        if (scroll_hint_active)
            scroll_hint_lastT = ImGui::GetTime();

        if (io.KeyShift || scroll_hint_count > 3)
        {
            scroll_hint_active = false;
            scroll_hint_enabled = false;
        }
    }
}

void motion(int x, int y)
{
    ImGuiIO& io = ImGui::GetIO();

    if (io.WantCaptureMouse)
        return;

    const float dx = (float)(x - app_state.mouse_old_x);
    const float dy = (float)(y - app_state.mouse_old_y);

    // Ctrl/Shift clicks pick a new rotation center; a stray drag on the same click must not break that transition.
    if (!io.KeyCtrl && !io.KeyShift)
    {
        if (app_state.mouse_buttons & 1)
            app_state.camera.dragOrbit(dx, dy);

        if (app_state.mouse_buttons & 4)
        {
            if (app_state.camera.isOrtho)
                app_state.camera.dragPanOrtho(dx, dy, io.DisplaySize.x, io.DisplaySize.y);
            else
                app_state.camera.dragPanPerspective(dx, dy);
        }
    }

    app_state.mouse_old_x = x;
    app_state.mouse_old_y = y;
}

void setNewRotationCenter(int x, int y)
{
    const Ray ray = app_state.camera.eulerScreenRay(x, y, GetScreenWidth(), GetScreenHeight());
    const Eigen::Vector3d origin(ray.position.x, ray.position.y, ray.position.z);
    const Eigen::Vector3d direction(ray.direction.x, ray.direction.y, ray.direction.z);

    Eigen::Vector3d center = origin;
    raylib_widgets::intersectPlane(origin, direction, 0.0, 0.0, 1.0, 0.0, center);

    spdlog::info("Setting new rotation center to: {}, {}, {}", center.x(), center.y(), center.z());

    app_state.camera.moveEulerRotationCenterTo(
        Vector3{ static_cast<float>(center.x()), static_cast<float>(center.y()), static_cast<float>(center.z()) });
}

void showAxes()
{
    if (!app_state.show_axes && !ImGui::GetIO().KeyCtrl)
        return;

    const auto& rc = app_state.camera.euler.rotationCenter;
    rlBegin(RL_LINES);
    rlColor3f(1.f, 1.f, 1.f);
    rlVertex3f(rc.x - 1.f, rc.y, rc.z);
    rlVertex3f(rc.x + 1.f, rc.y, rc.z);
    rlVertex3f(rc.x, rc.y - 1.f, rc.z);
    rlVertex3f(rc.x, rc.y + 1.f, rc.z);
    rlVertex3f(rc.x, rc.y, rc.z - 1.f);
    rlVertex3f(rc.x, rc.y, rc.z + 1.f);

    rlColor3f(1.0f, 0.0f, 0.0f);
    rlVertex3f(0.0f, 0.0f, 0.0f);
    rlVertex3f(100.0f, 0.0f, 0.0f);
    rlColor3f(0.0f, 1.0f, 0.0f);
    rlVertex3f(0.0f, 0.0f, 0.0f);
    rlVertex3f(0.0f, 100.0f, 0.0f);
    rlColor3f(0.0f, 0.0f, 1.0f);
    rlVertex3f(0.0f, 0.0f, 0.0f);
    rlVertex3f(0.0f, 0.0f, 100.0f);
    rlEnd();
}

void camMenu()
{
    using raylib_widgets::OrbitCamera;

    if (ImGui::BeginMenu("Camera"))
    {
        if (ImGui::MenuItem("Front (yz view)", "key F"))
            app_state.camera.setEulerPreset(OrbitCamera::EulerPreset::Front);
        if (ImGui::MenuItem("Back", "key B"))
            app_state.camera.setEulerPreset(OrbitCamera::EulerPreset::Back);
        if (ImGui::MenuItem("Left (xz view)", "key L"))
            app_state.camera.setEulerPreset(OrbitCamera::EulerPreset::Left);
        if (ImGui::MenuItem("Right", "key R"))
            app_state.camera.setEulerPreset(OrbitCamera::EulerPreset::Right);
        if (ImGui::MenuItem("Top (xy view)", "key T"))
            app_state.camera.setEulerPreset(OrbitCamera::EulerPreset::Top);
        if (ImGui::MenuItem("Bottom", "key U"))
            app_state.camera.setEulerPreset(OrbitCamera::EulerPreset::Bottom);
        if (ImGui::MenuItem("Isometric", "key I"))
            app_state.camera.setEulerPreset(OrbitCamera::EulerPreset::Iso);
        ImGui::Separator();
        if (ImGui::MenuItem("Reset", "key Z"))
            app_state.camera.setEulerPreset(OrbitCamera::EulerPreset::Reset);

        ImGui::EndMenu();
    }
    if (ImGui::IsItemHovered())
    {
        const auto& e = app_state.camera.euler;
        ImGui::BeginTooltip();
        ImGui::Text("Change camera view to fixed positions");
        ImGui::Separator();
        ImGui::Text("rotate:     %.3f %.3f", e.rotateX, e.rotateY);
        ImGui::Text("translate:  %.3f %.3f %.3f", e.translate.x, e.translate.y, e.translate.z);
        ImGui::Text("rot center: %.3f %.3f %.3f", e.rotationCenter.x, e.rotationCenter.y, e.rotationCenter.z);
        ImGui::Text("Mouse sensitivity: %.4f", app_state.camera.eulerMouseSensitivity);
        ImGui::EndTooltip();
    }

    if (scroll_hint_active)
    {
        ImVec2 mousePos = ImGui::GetMousePos();
        ImGui::SetNextWindowPos(ImVec2(mousePos.x + 20, mousePos.y - 40));
        ImGui::SetNextWindowBgAlpha(0.7f);
        ImGui::BeginTooltip();
        ImGui::Text("Tip: To accelerate hold Shift + scroll");
        ImGui::EndTooltip();

        if (ImGui::GetTime() - scroll_hint_lastT > 1)
            scroll_hint_active = false;
    }
}

void view_kbd_shortcuts()
{
    using raylib_widgets::OrbitCamera;

    ImGuiIO& io = ImGui::GetIO();
    auto& cam = app_state.camera;

    if (io.WantCaptureKeyboard)
        return;

    const float step = 0.5f * cam.eulerMouseSensitivity;
    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_RightArrow, true))
    {
        cam.euler.translate.x += step;
        cam.breakEulerTransition();
    }
    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_LeftArrow, true))
    {
        cam.euler.translate.x -= step;
        cam.breakEulerTransition();
    }
    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_UpArrow, true))
    {
        cam.euler.translate.y += step;
        cam.breakEulerTransition();
    }
    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_DownArrow, true))
    {
        cam.euler.translate.y -= step;
        cam.breakEulerTransition();
    }

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_RightArrow, true))
    {
        cam.euler.rotateY -= 0.6f;
        cam.breakEulerTransition();
    }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_LeftArrow, true))
    {
        cam.euler.rotateY += 0.6f;
        cam.breakEulerTransition();
    }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_UpArrow, true))
    {
        cam.euler.rotateX -= 0.6f;
        cam.breakEulerTransition();
    }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_DownArrow, true))
    {
        cam.euler.rotateX += 0.6f;
        cam.breakEulerTransition();
    }

    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_R, false))
        cor_gui = true;

    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_Z, false) && !cam.isOrtho)
        cam.lockZ = !cam.lockZ;

    if (io.KeyCtrl || io.KeyAlt || io.KeyShift)
        return;

    if (ImGui::IsKeyPressed(ImGuiKey_B))
        cam.setEulerPreset(OrbitCamera::EulerPreset::Back);
    if (ImGui::IsKeyPressed(ImGuiKey_F))
        cam.setEulerPreset(OrbitCamera::EulerPreset::Front);
    if (ImGui::IsKeyPressed(ImGuiKey_I))
        cam.setEulerPreset(OrbitCamera::EulerPreset::Iso);
    if (ImGui::IsKeyPressed(ImGuiKey_L))
        cam.setEulerPreset(OrbitCamera::EulerPreset::Left);
    if (ImGui::IsKeyPressed(ImGuiKey_R))
        cam.setEulerPreset(OrbitCamera::EulerPreset::Right);
    if (ImGui::IsKeyPressed(ImGuiKey_T))
        cam.setEulerPreset(OrbitCamera::EulerPreset::Top);
    if (ImGui::IsKeyPressed(ImGuiKey_U))
        cam.setEulerPreset(OrbitCamera::EulerPreset::Bottom);
    if (ImGui::IsKeyPressed(ImGuiKey_Z))
        cam.setEulerPreset(OrbitCamera::EulerPreset::Reset);

    if (ImGui::IsKeyPressed(ImGuiKey_C, false))
        app_state.compass_ruler = !app_state.compass_ruler;
    if (ImGui::IsKeyPressed(ImGuiKey_O, false))
        cam.isOrtho = !cam.isOrtho;
    if (ImGui::IsKeyPressed(ImGuiKey_X, false))
        app_state.show_axes = !app_state.show_axes;

    for (int k = 1; k <= 9; k++)
        if (ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_0 + k)))
            app_state.point_size = k;
}

// Rows 0/1 of the world-to-eye rotation are the world-space directions of screen right/up.
void drawMiniCompassWithRuler()
{
    const Eigen::Matrix3f& R = app_state.viewLocal.rotation();
    Vector3 right = { R(0, 0), R(0, 1), R(0, 2) };
    Vector3 up = { R(1, 0), R(1, 1), R(1, 2) };
    Color rulerColor =
        ColorFromNormalized(Vector4{ 1.0f - app_state.bg_color.x, 1.0f - app_state.bg_color.y, 1.0f - app_state.bg_color.z, 1.0f });
    raylib_widgets::drawCompassRuler(
        right,
        up,
        app_state.camera.euler.translate.z,
        rulerColor,
        raylib_widgets::CompassAxisLabels{ "X (long.)", "Y (lat.)", "Z (vert.)" });
}

// Runs ImGuizmo on `m` (column-major 4x4) against this frame's 3D camera.
void manipulateGizmo(float* m)
{
    ImGuiIO& io = ImGui::GetIO();
    ImGuizmo::BeginFrame();
    ImGuizmo::Enable(true);
    ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

    if (!app_state.camera.isOrtho)
    {
        float16 view = MatrixToFloatV(app_state.camera.frameView3D);
        float16 projection = MatrixToFloatV(app_state.camera.frameProj3D);
        ImGuizmo::Manipulate(
            view.v,
            projection.v,
            ImGuizmo::TRANSLATE | ImGuizmo::ROTATE_Z | ImGuizmo::ROTATE_X | ImGuizmo::ROTATE_Y,
            ImGuizmo::WORLD,
            m,
            NULL);
    }
    else
    {
        ImGuizmo::Manipulate(
            app_state.camera.orthoGizmoView,
            app_state.camera.orthoProjection,
            ImGuizmo::TRANSLATE_X | ImGuizmo::TRANSLATE_Y | ImGuizmo::ROTATE_Z,
            ImGuizmo::WORLD,
            m,
            NULL);
    }
}

void drawCross(const Eigen::Vector3d& p, double size)
{
    rlVertex3f(p.x() - size, p.y(), p.z());
    rlVertex3f(p.x() + size, p.y(), p.z());
    rlVertex3f(p.x(), p.y() - size, p.z());
    rlVertex3f(p.x(), p.y() + size, p.z());
    rlVertex3f(p.x(), p.y(), p.z() - size);
    rlVertex3f(p.x(), p.y(), p.z() + size);
}

// Draws axes of `rotation`'s columns scaled by `length`, starting at `origin`.
void drawAxesAt(const Eigen::Vector3d& origin, const Eigen::Matrix3d& rotation, double length)
{
    const float colors[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
    for (int k = 0; k < 3; k++)
    {
        const Eigen::Vector3d end = origin + rotation.col(k) * length;
        rlColor3f(colors[k][0], colors[k][1], colors[k][2]);
        rlVertex3f(origin.x(), origin.y(), origin.z());
        rlVertex3f(end.x(), end.y(), end.z());
    }
}

///////////////////////////////////////////////////////////////////////////////////

void set_lidar_odometry_default_params(LidarOdometryParams& params)
{
    params.decimation = 0.01;
    params.in_out_params_indoor.resolution_X = 0.1;
    params.in_out_params_indoor.resolution_Y = 0.1;
    params.in_out_params_indoor.resolution_Z = 0.1;

    params.in_out_params_outdoor.resolution_X = 0.3;
    params.in_out_params_outdoor.resolution_Y = 0.3;
    params.in_out_params_outdoor.resolution_Z = 0.3;

    params.filter_threshold_xy_inner = 0.3;
    params.filter_threshold_xy_outer = 70.0;
    params.threshould_output_filter = 0.3;

    params.use_robust_and_accurate_lidar_odometry = false;
    params.distance_bucket = 0.2;
    params.polar_angle_deg = 10.0;
    params.azimutal_angle_deg = 10.0;
    params.robust_and_accurate_lidar_odometry_iterations = 20;

    params.max_distance_lidar = 70.0;
    params.nr_iter = 1000;
    params.sliding_window_trajectory_length_threshold = 200;
    params.real_time_threshold_seconds = 10;
}

// Helper function to format time in human-readable format
std::string formatTime(double seconds)
{
    if (seconds < 0)
        return "Calculating...";

    int hours = static_cast<int>(seconds / 3600);
    int minutes = static_cast<int>((seconds - hours * 3600) / 60);
    int secs = static_cast<int>(seconds - hours * 3600 - minutes * 60);

    std::ostringstream oss;

    if (hours > 0)
    {
        oss << std::setw(2) << std::setfill('0') << hours << "h ";
    }
    if (minutes > 0 || hours > 0)
    {
        oss << std::setw(2) << std::setfill('0') << minutes << "m ";
    }

    oss << std::setw(2) << std::setfill('0') << secs << "s";

    return oss.str();
}

// Helper function to format estimated completion time
std::string formatCompletionTime(double remainingSeconds)
{
    if (remainingSeconds < 0)
        return "Calculating...";

    auto now = std::chrono::system_clock::now();
    auto estimatedCompletion = now + std::chrono::seconds(static_cast<int64_t>(remainingSeconds));
    auto completion_time_t = std::chrono::system_clock::to_time_t(estimatedCompletion);

    // Format as HH:MM:SS - Cross-platform time formatting
    struct tm completion_tm;
#ifdef _WIN32
    localtime_s(&completion_tm, &completion_time_t);
#else
    localtime_r(&completion_time_t, &completion_tm);
#endif

    char timeStr[32];
    strftime(timeStr, sizeof(timeStr), "%H:%M", &completion_tm);

    return std::string(timeStr);
}

#if 0
std::vector<std::vector<Point3Di>> get_batches_of_points(std::string laz_file, int point_count_threshold, conststd::vector<Point3Di>& prev_points)
{
    std::vector<std::vector<Point3Di>> res_points;
    std::vector<Point3Di> points = load_point_cloud(laz_file, false, 0, 10000, {});

    std::vector<Point3Di> tmp_points = prev_points;
    int counter = tmp_points.size();
    for (size_t i = 0; i < points.size(); i++)
    {
        counter++;
        tmp_points.push_back(points[i]);
        if (counter > point_count_threshold)
        {
            res_points.push_back(tmp_points);
            tmp_points.clear();
            counter = 0;
        }
    }

    if (tmp_points.size() > 0)
    {
        res_points.push_back(tmp_points);
    }
    return res_points;
}
#endif

void find_best_stretch(
    std::vector<Point3Di> points, std::vector<double> timestamps, std::vector<Eigen::Affine3d> poses, std::string fn1, std::string fn2)
{
    for (size_t i = 0; i < points.size(); i++)
    {
        auto lower = std::lower_bound(timestamps.begin(), timestamps.end(), points[i].timestamp);
        points[i].index_pose = std::distance(timestamps.begin(), lower);
    }

    std::set<int> indexes;

    for (size_t i = 0; i < points.size(); i++)
    {
        indexes.insert(points[i].index_pose);
    }
    // build trajectory
    std::vector<Eigen::Affine3d> trajectory;
    std::vector<double> ts;

    for (auto& s : indexes)
    {
        trajectory.push_back(poses[s]);
        ts.push_back(timestamps[s]);
    }

    std::cout << "trajectory.size() " << trajectory.size() << std::endl;
    // Sleep(2000);

    std::vector<Point3Di> points_reindexed = points;
    for (size_t i = 0; i < points_reindexed.size(); i++)
    {
        points_reindexed[i].index_pose = get_index_in_set(indexes, points[i].index_pose);
    }
    ///
    std::vector<Eigen::Affine3d> best_trajectory = trajectory;
    uint64_t min_buckets = ULLONG_MAX;

    for (double x = 0.0; x < 0.2; x += 0.0005)
    {
        std::vector<Eigen::Affine3d> trajectory_stretched;

        Eigen::Affine3d m_x_offset = Eigen::Affine3d::Identity();
        m_x_offset(0, 3) = x;

        Eigen::Affine3d m = trajectory[0];
        trajectory_stretched.push_back(m);
        for (size_t i = 1; i < trajectory.size(); i++)
        {
            Eigen::Affine3d m_update = trajectory[i - 1].inverse() * trajectory[i] * (m_x_offset);
            m = m * m_update;
            trajectory_stretched.push_back(m);
        }

        NDT::GridParameters rgd_params;
        rgd_params.resolution_X = 0.3;
        rgd_params.resolution_Y = 0.3;
        rgd_params.resolution_Z = 0.3;
        NDTBucketMapType my_buckets;

        std::vector<Point3Di> points_global = points_reindexed;

        std::vector<Point3Di> points_global2;

        for (auto& p : points_global)
        {
            if (p.point.z() > 0)
            {
                p.point = trajectory_stretched[p.index_pose] * p.point;
                points_global2.push_back(p);
                // if (p.point.norm() > 6 && p.point.norm() < 15)
                //{
                // points_global.push_back(p);
                // }
            }
        }
        std::scoped_lock lock(params.mutex_buckets_indoor, params.mutex_buckets_outdoor);

        update_rgd(rgd_params, my_buckets, points_global2, trajectory_stretched[0].translation());

        std::cout << "number of buckets [" << x << "]: " << my_buckets.size() << std::endl;
        if (my_buckets.size() < min_buckets)
        {
            min_buckets = my_buckets.size();
            best_trajectory = trajectory_stretched;
        }
    }

    std::map<double, Eigen::Matrix4d> trajectory_for_interpolation;
    for (size_t i = 0; i < best_trajectory.size(); i++)
    {
        trajectory_for_interpolation[ts[i]] = best_trajectory[i].matrix();
    }

    std::vector<Eigen::Vector3d> pointcloud_global;
    std::vector<unsigned short> intensity;
    std::vector<double> timestamps_;
    for (const auto& p : points_reindexed)
    {
        Eigen::Matrix4d pose = getInterpolatedPose(trajectory_for_interpolation, p.timestamp);
        Eigen::Affine3d b;
        b.matrix() = pose;
        Eigen::Vector3d vec = b * p.point;
        pointcloud_global.push_back(vec);
        intensity.push_back(p.intensity);
        timestamps_.push_back(p.timestamp);
    }

    std::cout << "saving file: " << fn1 << std::endl;
    exportLaz(fn1, pointcloud_global, intensity, timestamps_);

    pointcloud_global.clear();
    for (const auto& p : points_reindexed)
    {
        Eigen::Vector3d vec = trajectory[p.index_pose] * p.point;
        pointcloud_global.push_back(vec);
    }
    exportLaz(fn2, pointcloud_global, intensity, timestamps_);
}

#if 0
void alternative_approach()
{
    int point_count_threshold = 10000;

    std::cout << "aternative_approach" << std::endl;

    std::vector<std::string> input_file_names;
    input_file_names = mandeye::fd::OpenFileDialog("Load las files", {}, true);
    std::sort(std::begin(input_file_names), std::end(input_file_names));

    std::vector<std::string> csv_files;
    std::vector<std::string> laz_files;
    std::for_each(std::begin(input_file_names), std::end(input_file_names), [&](const std::string& fileName)
        {
            if (fileName.ends_with(".laz") || fileName.ends_with(".las"))
            {
                laz_files.push_back(fileName);
            }
            if (fileName.ends_with(".csv"))
            {
                csv_files.push_back(fileName);
            } });

            std::cout << "imu files: " << std::endl;
            for (const auto& fn : csv_files)
            {
                std::cout << fn << std::endl;
            }

            std::cout << "loading imu" << std::endl;
            Imu imu_data;

            std::for_each(std::begin(csv_files), std::end(csv_files), [&imu_data](const std::string& fn)
                {
                    auto imu = load_imu(fn.c_str(), 0);
                    std::cout << fn << std::endl;
                    imu_data.insert(std::end(imu_data), std::begin(imu), std::end(imu)); });

            std::map<double, Eigen::Matrix4d> trajectory;
            int counter = 1;
            const float RAD_TO_DEG = 180.0f / static_cast<float>(M_PI);

            if (params.use_vqf)
            {
                double avg_dt = 1.0 / 200.0;
                if (imu_data.size() >= 2)
                {
                    double t0 = std::get<0>(imu_data.front()).first;
                    double t1 = std::get<0>(imu_data.back()).first;
                    if (t1 > t0)
                        avg_dt = (t1 - t0) / static_cast<double>(imu_data.size() - 1);
                }

                VQFParams vqf_params = buildVQFParams(params);
                VQF vqf(vqf_params, avg_dt);

                for (const auto& [timestamp_pair, gyr, acc] : imu_data)
                {
                    const double g = 9.80665;
                    vqf_real_t gyr_vqf[3] = { static_cast<double>(gyr.x()), static_cast<double>(gyr.y()), static_cast<double>(gyr.z()) };
                    vqf_real_t acc_vqf[3] = { static_cast<double>(acc.x()) * g, static_cast<double>(acc.y()) * g, static_cast<double>(acc.z()) * g };

                    vqf.update(gyr_vqf, acc_vqf);

                    vqf_real_t quat[4];
                    if (params.vqf_useMagnetometer)
                        vqf.getQuat9D(quat);
                    else
                        vqf.getQuat6D(quat);
                    Eigen::Quaterniond d(quat[0], quat[1], quat[2], quat[3]);
                    Eigen::Affine3d t{ Eigen::Matrix4d::Identity() };
                    t.rotate(d);
                    trajectory[timestamp_pair.first] = t.matrix();
                    counter++;
                    if (counter % 100 == 0)
                    {
                        Eigen::Vector3d euler = d.toRotationMatrix().eulerAngles(0, 1, 2) * (180.0 / M_PI);
                        std::cout << "Roll " << euler.x() << ", Pitch " << euler.y() << ", Yaw " << euler.z() << " [" << counter << " of " << imu_data.size() << "]" << std::endl;
                    }
                }
            }
            else
            {
                FusionAhrs ahrs;
                FusionAhrsInitialise(&ahrs);
                if (params.fusionConventionNwu) ahrs.settings.convention = FusionConventionNwu;
                else if (params.fusionConventionEnu) ahrs.settings.convention = FusionConventionEnu;
                else if (params.fusionConventionNed) ahrs.settings.convention = FusionConventionNed;
                ahrs.settings.gain = params.fusion_gain;

                bool first = true;
                double last_ts = 0.0;

                for (const auto& [timestamp_pair, gyr, acc] : imu_data)
                {
                    const FusionVector gyroscope = { static_cast<float>(gyr.x() * RAD_TO_DEG),
                                                     static_cast<float>(gyr.y() * RAD_TO_DEG),
                                                     static_cast<float>(gyr.z() * RAD_TO_DEG) };
                    const FusionVector accelerometer = { acc.x(), acc.y(), acc.z() };

                    if (first)
                    {
                        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, 1.0f / 200.0f);
                        first = false;
                    }
                    else
                    {
                        float ts_diff = static_cast<float>(timestamp_pair.first - last_ts);
                        if (ts_diff < 0.01f)
                            FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, ts_diff);
                        else
                            FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, 1.0f / 200.0f);
                    }
                    last_ts = timestamp_pair.first;

                    FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);
                    Eigen::Quaterniond d{ quat.element.w, quat.element.x, quat.element.y, quat.element.z };
                    Eigen::Affine3d t{ Eigen::Matrix4d::Identity() };
                    t.rotate(d);
                    trajectory[timestamp_pair.first] = t.matrix();
                    counter++;
                    if (counter % 100 == 0)
                    {
                        const FusionEuler euler = FusionQuaternionToEuler(quat);
                        std::cout << "Roll " << euler.angle.roll << ", Pitch " << euler.angle.pitch << ", Yaw " << euler.angle.yaw
                                  << " [" << counter << " of " << imu_data.size() << "]" << std::endl;
                    }
                }
            }

            ///////////////////////////////////////////////////////////////////////////////
            std::cout << "point cloud file names" << std::endl;
            for (const auto& fn : laz_files)
            {
                std::cout << fn << std::endl;
            }

            std::vector<Point3Di> prev_points;
            std::vector<std::vector<Point3Di>> all_points;
            std::vector<std::vector<Point3Di>> tmp_points = get_batches_of_points(laz_files[0], point_count_threshold, prev_points);

            for (size_t i = 0; i < tmp_points.size() - 1; i++)
            {
                all_points.push_back(tmp_points[i]);
            }

            for (size_t i = 1; i < laz_files.size(); i++)
            {
                prev_points = tmp_points[tmp_points.size() - 1];
                tmp_points = get_batches_of_points(laz_files[i], point_count_threshold, prev_points);
                for (size_t j = 0; j < tmp_points.size() - 1; j++)
                {
                    all_points.push_back(tmp_points[j]);
                }
            }

            //////////
            std::vector<double> timestamps;
            std::vector<Eigen::Affine3d> poses;
            for (const auto& t : trajectory)
            {
                timestamps.push_back(t.first);
                Eigen::Affine3d m;
                m.matrix() = t.second;
                poses.push_back(m);
            }

            for (size_t i = 0; i < all_points.size(); i++)
            {
                std::cout << all_points[i].size() << std::endl;
                std::string fn1 = "C:/data/tmp/" + std::to_string(i) + "_best.laz";
                std::string fn2 = "C:/data/tmp/" + std::to_string(i) + "_original.laz";

                find_best_stretch(all_points[i], timestamps, poses, fn1, fn2);
            }
}
#endif

//! Loads Mandeye data and runs the first processing stage.
//! @param folder data folder; when empty, the user is asked to select one.
void step1(const std::atomic<bool>& loPause, const std::string& folder = "")
{
    std::string input_folder_name = folder;
    std::vector<std::string> input_file_names;
    if (input_folder_name.empty())
        input_folder_name = mandeye::fd::SelectFolder("Select Mandeye data folder");

    std::cout << "Selected folder: '" << input_folder_name << std::endl;

    if (fs::exists(input_folder_name))
    {
        std::string newTitle = winTitle + " - ..\\" + std::filesystem::path(input_folder_name).filename().string();
        SetWindowTitle(newTitle.c_str());

        for (const auto& entry : fs::directory_iterator(input_folder_name))
            if (entry.is_regular_file())
                input_file_names.push_back(entry.path().string());

        if (load_data(input_file_names, params, pointsPerFile, imu_data, full_debug_messages))
        {
            working_directory = fs::path(input_file_names[0]).parent_path().string();
            calculate_trajectory(trajectory, imu_data, params, full_debug_messages);
            compute_step_1(pointsPerFile, params, trajectory, worker_data, loPause);
            step_1_done = true;
        }
        else
        {
            SetWindowTitle(winTitle.c_str());

            std::string message_info = "Problem with loading data from folder '" + input_folder_name +
                "'. Please check that it contains Mandeye IMU *.csv and LiDAR *.laz files, then select another folder.";
            std::cout << message_info << std::endl;
            [[maybe_unused]] pfd::message message("Information", message_info.c_str(), pfd::choice::ok, pfd::icon::warning);
            message.result();
        }
    }
}

void step2(const std::atomic<bool>& loPause)
{
    double ts_failure = 0.0;
    if (compute_step_2(worker_data, params, ts_failure, loProgress, loPause, full_debug_messages))
        step_2_done = true;
    else
    {
        for (size_t fileNo = 0; fileNo < csv_files.size(); fileNo++)
        {
            const std::string& imufn = csv_files.at(fileNo);
            const std::string snFn = (fileNo >= sn_files.size()) ? ("") : (sn_files.at(fileNo));
            const auto idToSn = MLvxCalib::GetIdToSnMapping(snFn);
            // GetId of Imu to use
            int imuNumberToUse = MLvxCalib::GetImuIdToUse(idToSn, imuSnToUse);
            auto imu = load_imu(imufn.c_str(), imuNumberToUse);

            if (imu.size() > 0)
            {
                if (std::get<0>(imu[imu.size() - 1]).first > ts_failure)
                    break;
            }
            std::cout << "file: '" << imufn << "' [OK]" << std::endl;
        }
        calculations_failed = true;
    }
}

void save_results(bool info, double elapsed_seconds)
{
    outwd = get_next_result_path(working_directory);
    save_result(worker_data, params, outwd, elapsed_seconds);
    if (info)
    {
        std::string message_info = "Results saved to folder: '" + outwd.string() + "'";
        std::cout << message_info << std::endl;
        [[maybe_unused]] pfd::message message("Information", message_info.c_str(), pfd::choice::ok, pfd::icon::info);
        message.result();
    }
}

void settings_gui()
{
    if (ImGui::Begin("Settings", &is_settings_gui))
    {
        ImGui::Checkbox("simple_gui", &simple_gui);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Show even more parameters");

        ImGui::NewLine();

        if (!loRunning)
        {
            // TaitBryanPose motion_model_correction;
            ImGui::Text("motion_model_correction [deg]:");
            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputDouble("om, rotation via X", &params.motion_model_correction.om);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(omText);
            ImGui::InputDouble("fi, rotation via Y", &params.motion_model_correction.fi);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(fiText);
            ImGui::InputDouble("ka, rotation via Z", &params.motion_model_correction.ka);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(kaText);
            ImGui::PopItemWidth();

            if (ImGui::Button("Set example"))
            {
                params.motion_model_correction.om = 0.0;
                params.motion_model_correction.fi = 0.05;
                params.motion_model_correction.ka = 0.0;
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("motion_model_corrections for LiDAR X-axis (forward direction)");

            ImGui::NewLine();

            ImGui::Text("lidar_odometry_motion_model sigmas [m] / [deg]:");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("position/rotational uncertainties");

            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputDouble("x_1", &params.lidar_odometry_motion_model_x_1_sigma_m, 0.0, 0.0, "%.4f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(xText);
            ImGui::SameLine();
            ImGui::InputDouble("om_1", &params.lidar_odometry_motion_model_om_1_sigma_deg, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(omText);

            ImGui::InputDouble("y_1", &params.lidar_odometry_motion_model_y_1_sigma_m, 0.0, 0.0, "%.4f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(yText);
            ImGui::SameLine();
            ImGui::InputDouble("fi_1", &params.lidar_odometry_motion_model_fi_1_sigma_deg, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(fiText);

            ImGui::InputDouble("z_1", &params.lidar_odometry_motion_model_z_1_sigma_m, 0.0, 0.0, "%.4f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(zText);
            ImGui::SameLine();
            ImGui::InputDouble("ka_1", &params.lidar_odometry_motion_model_ka_1_sigma_deg, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(kaText);

            ImGui::PopItemWidth();

            ImGui::Text("lidar_odometry_motion_model_fix_origin sigmas [m] / [deg]:");

            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputDouble("x_1##o", &params.lidar_odometry_motion_model_fix_origin_x_1_sigma_m);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(xText);
            ImGui::SameLine();
            ImGui::InputDouble("om_1##o", &params.lidar_odometry_motion_model_fix_origin_om_1_sigma_deg);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(omText);

            ImGui::InputDouble("y_1##o", &params.lidar_odometry_motion_model_fix_origin_y_1_sigma_m);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(yText);
            ImGui::SameLine();
            ImGui::InputDouble("fi_1##o", &params.lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(fiText);

            ImGui::InputDouble("z_1##o", &params.lidar_odometry_motion_model_fix_origin_z_1_sigma_m);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(zText);
            ImGui::SameLine();
            ImGui::InputDouble("ka_1##o", &params.lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(kaText);

            ImGui::PopItemWidth();
        }

        ImGui::NewLine();

        if (!simple_gui)
        {
            ImGui::SetNextItemWidth(ImGuiNumberWidth);
            ImGui::InputInt("Threshold nr poses", &params.threshold_nr_poses);
            if (params.threshold_nr_poses < 1)
            {
                params.threshold_nr_poses = 1;
            }

            ImGui::NewLine();
        }

        // ImGui::Checkbox("show_all_points", &show_all_points);

        if (calculations_failed)
        {
            ImGui::Text("CALCULATIONS FAILED... please read information in console");
            ImGui::End();
            return;
        }

        ImGui::PushItemWidth(ImGuiNumberWidth);
        ImGui::InputDouble("Filter threshold XY inner [m]", &params.filter_threshold_xy_inner, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
        {
            ImGui::BeginTooltip();
            ImGui::Text("all local points inside lidar xy_circle radius will be removed during load");
            ImGui::Text("Minimum value is given by Lidar's specs (Close Proximity Blind Zone)");
            ImGui::Text("Value can be higher to filter out close range permanent obstacles");
            ImGui::Text("e.g.: 0.1[m] for Livox Mid-360");
            ImGui::EndTooltip();
        }
        ImGui::InputDouble("Filter threshold XY outer [m]", &params.filter_threshold_xy_outer, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
        {
            ImGui::BeginTooltip();
            ImGui::Text("all local points outside lidar xy_circle radius will be removed during load");
            ImGui::Text("Maximum value is given by Lidar's specs (Detection Range)");
            ImGui::Text("Value can be lower to adapt for different reflectivity or real world contrains");
            ImGui::Text("e.g.: 70[m] @ 80%% reflectivity for Livox Mid-360");
            ImGui::EndTooltip();
        }
        ImGui::InputDouble("Threshold output filter [m]", &params.threshould_output_filter, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("all local points inside lidar xy_circle radius will be removed during save");
        ImGui::PopItemWidth();

        if (!simple_gui)
        {
            ImGui::NewLine();
            ImGui::Text("NDT bucket size (inner/outer)");
            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputDouble("##x", &params.in_out_params_indoor.resolution_X, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(xText);
            if (params.in_out_params_indoor.resolution_X < 0.01)
            {
                params.in_out_params_indoor.resolution_X = 0.01;
            }
            ImGui::SameLine();
            ImGui::InputDouble("X##ndt", &params.in_out_params_outdoor.resolution_X, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(xText);
            if (params.in_out_params_outdoor.resolution_X < 0.01)
            {
                params.in_out_params_outdoor.resolution_X = 0.01;
            }

            ImGui::InputDouble("##y", &params.in_out_params_indoor.resolution_Y, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(yText);
            if (params.in_out_params_indoor.resolution_Y < 0.01)
            {
                params.in_out_params_indoor.resolution_Y = 0.01;
            }
            ImGui::SameLine();
            ImGui::InputDouble("Y##ndt", &params.in_out_params_outdoor.resolution_Y, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(yText);
            if (params.in_out_params_outdoor.resolution_Y < 0.01)
            {
                params.in_out_params_outdoor.resolution_Y = 0.01;
            }

            ImGui::InputDouble("##z", &params.in_out_params_indoor.resolution_Z, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(zText);
            if (params.in_out_params_indoor.resolution_Z < 0.01)
            {
                params.in_out_params_indoor.resolution_Z = 0.01;
            }
            ImGui::SameLine();
            ImGui::InputDouble("Z##ndt", &params.in_out_params_outdoor.resolution_Z, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(zText);
            if (params.in_out_params_outdoor.resolution_Z < 0.01)
            {
                params.in_out_params_outdoor.resolution_Z = 0.01;
            }

            ImGui::NewLine();

            ImGui::InputDouble("Downsampling", &params.decimation, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Larger value of downsampling better performance, but worse accuracy");

            ImGui::InputDouble("Max distance of processed points [m]", &params.max_distance_lidar, 0.0, 0.0, "%.2f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Local LiDAR coordinates");

            ImGui::InputInt("Number of iterations", &params.nr_iter);
            ImGui::InputDouble(
                "Sliding window trajectory length threshold [m]", &params.sliding_window_trajectory_length_threshold, 0.0, 0.0, "%.2f");
            ImGui::InputInt("Threshold initial points", &params.threshold_initial_points);

            ImGui::NewLine();

            // AHRS type selection
            ImGui::Checkbox("Use VQF (instead of Fusion/Madgwick)", &params.use_vqf);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Unchecked = Fusion (Madgwick complementary filter, default)\nChecked = VQF (Versatile Quaternion-based Filter)");

            if (!params.use_vqf)
            {
                // Fusion-specific parameters
                static int fusionConvention; // 0=NWU, 1=ENU, 2=NED
                if (fusionConvention < 0 || fusionConvention > 2)
                    fusionConvention = 0;

                ImGui::Text("Fusion convention: ");
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        "Coordinate system conventions for sensor fusion defining how the axes are oriented relative to world frame");

                ImGui::SameLine();
                ImGui::RadioButton("NWU", &fusionConvention, 0);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("North West Up");
                ImGui::SameLine();
                ImGui::RadioButton("ENU", &fusionConvention, 1);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("East North Up");
                ImGui::SameLine();
                ImGui::RadioButton("NED", &fusionConvention, 2);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("North East Down");

                params.fusionConventionNwu = (fusionConvention == 0);
                params.fusionConventionEnu = (fusionConvention == 1);
                params.fusionConventionNed = (fusionConvention == 2);

                ImGui::InputDouble("Fusion gain", &params.fusion_gain, 0.0, 0.0, "%.3f");
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Complementary filter gain (0-1). Higher = more accelerometer trust.");
            }

            ImGui::NewLine();

            ImGui::Checkbox("Use motion from previous step", &params.use_motion_from_previous_step);
            ImGui::Checkbox("Use IMU preintegration", &params.use_imu_preintegration);
            if (params.use_imu_preintegration)
            {
                const char* methods[] = { "Euler, no gravity comp., SM velocity",      "Trapezoidal, no gravity comp., SM velocity",
                                          "Euler, gravity comp., SM velocity",         "Trapezoidal, gravity comp., SM velocity",
                                          "Kalman, gravity comp., SM velocity",        "Euler, gravity comp., AHRS velocity",
                                          "Trapezoidal, gravity comp., AHRS velocity", "Kalman, gravity comp., AHRS velocity" };
                ImGui::Combo("IMU preintegration method", &params.imu_preintegration_method, methods, IM_ARRAYSIZE(methods));
            }
            if (params.use_vqf)
            {
                ImGui::InputDouble("VQF tauAcc [s]", &params.vqf_tauAcc, 0.0, 0.0, "%.3f");
                if (ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    ImGui::Text("VQF accelerometer time constant (tauAcc) in seconds.");
                    ImGui::Text("Controls how strongly accelerometer corrects the gyroscope-based orientation.");
                    ImGui::Text("Higher = more gyro trust (stable but may drift). Lower = more acc trust (noisy but no drift).");
                    ImGui::EndTooltip();
                }

                if (ImGui::TreeNode("VQF Gyro Bias Estimation"))
                {
                    ImGui::Checkbox("Motion bias estimation", &params.vqf_motionBiasEstEnabled);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Enables gyroscope bias estimation during motion phases,\nbased on the inclination correction only (without "
                            "magnetometer).");

                    if (params.vqf_motionBiasEstEnabled)
                    {
                        ImGui::InputDouble("Bias sigma motion [deg/s]", &params.vqf_biasSigmaMotion, 0.0, 0.0, "%.4f");
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip(
                                "Std dev of converged bias estimation uncertainty during motion.\nDetermines trust on motion bias "
                                "estimation "
                                "updates.\nSmall value leads to fast convergence. Default: 0.1");
                        ImGui::InputDouble("Bias vertical forgetting", &params.vqf_biasVerticalForgettingFactor, 0.0, 0.0, "%.6f");
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip(
                                "Forgetting factor for unobservable bias in vertical direction during motion.\nGyro bias is not observable "
                                "vertically without magnetometer.\nRelative weight of artificial zero measurement ensuring\nbias estimate "
                                "decays to zero. Default: 0.0001");
                    }

                    ImGui::InputDouble("Bias sigma init [deg/s]", &params.vqf_biasSigmaInit, 0.0, 0.0, "%.3f");
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Std dev of the initial bias estimation uncertainty. Default: 0.5 deg/s");
                    ImGui::InputDouble("Bias forgetting time [s]", &params.vqf_biasForgettingTime, 0.0, 0.0, "%.1f");
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Time in which bias estimation uncertainty increases from 0 to 0.1 deg/s.\nDetermines the system noise assumed "
                            "by "
                            "the Kalman filter. Default: 100.0");
                    ImGui::InputDouble("Bias clip [deg/s]", &params.vqf_biasClip, 0.0, 0.0, "%.2f");
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Maximum expected gyroscope bias.\nUsed to clip bias estimate and measurement error in update step.\nAlso used "
                            "by "
                            "rest detection to not regard large constant angular rate as rest.\nDefault: 2.0");

                    ImGui::Checkbox("Rest bias estimation", &params.vqf_restBiasEstEnabled);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Enables rest detection and gyroscope bias estimation during rest phases.\nDuring rest, gyro bias is estimated "
                            "from low-pass filtered gyro readings.");

                    if (params.vqf_restBiasEstEnabled)
                    {
                        ImGui::InputDouble("Bias sigma rest [deg/s]", &params.vqf_biasSigmaRest, 0.0, 0.0, "%.4f");
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip(
                                "Std dev of converged bias estimation uncertainty during rest.\nDetermines trust on rest bias estimation "
                                "updates.\nSmall value leads to fast convergence. Default: 0.03");
                        ImGui::InputDouble("Rest min time [s]", &params.vqf_restMinT, 0.0, 0.0, "%.2f");
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip(
                                "Time threshold for rest detection.\nRest is detected when measurements have been close to\nthe low-pass "
                                "filtered reference for the given time. Default: 1.5");
                        ImGui::InputDouble("Rest filter tau [s]", &params.vqf_restFilterTau, 0.0, 0.0, "%.2f");
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip(
                                "Time constant for the second-order Butterworth low-pass filter\nused to obtain the reference for rest "
                                "detection. Default: 0.5");
                        ImGui::InputDouble("Rest threshold gyro [deg/s]", &params.vqf_restThGyr, 0.0, 0.0, "%.2f");
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip(
                                "Angular velocity threshold for rest detection.\nDeviation norm between measurement and reference must be "
                                "below threshold.\nEach component must also be below biasClip. Default: 2.0");
                        ImGui::InputDouble("Rest threshold acc [m/s2]", &params.vqf_restThAcc, 0.0, 0.0, "%.2f");
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip(
                                "Acceleration threshold for rest detection.\nDeviation norm between measurement and reference must be "
                                "below threshold.\nDefault: 0.5");
                    }

                    ImGui::TreePop();
                }

                ImGui::Checkbox("Use magnetometer", &params.vqf_useMagnetometer);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        "Enable 9D mode (gyro+acc+mag) for absolute heading correction.\nDefault: off (6D mode, gyro+acc only, heading "
                        "from "
                        "gyro integration).");

                if (params.vqf_useMagnetometer && ImGui::TreeNode("VQF Magnetometer"))
                {
                    ImGui::InputDouble("tauMag [s]", &params.vqf_tauMag, 0.0, 0.0, "%.2f");
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Time constant for magnetometer update.\nSmall values imply trust on magnetometer, large values trust on "
                            "gyroscope.\nCorresponds to cutoff frequency of first-order LP filter\nfor heading correction. Default: 9.0");
                    ImGui::Checkbox("Mag disturbance rejection", &params.vqf_magDistRejectionEnabled);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Enables magnetic disturbance detection and rejection.\nFor short disturbances, mag correction is fully "
                            "disabled.\nFor long disturbances (>magMaxRejectionTime), correction uses\nincreased time constant "
                            "(magRejectionFactor).");
                    ImGui::InputDouble("Mag current tau [s]", &params.vqf_magCurrentTau, 0.0, 0.0, "%.3f");
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Time constant for current norm/dip value in disturbance detection.\nFast LP filter for robustness with noisy "
                            "or "
                            "async mag measurements.\nSet to -1 to disable. Default: 0.05");
                    ImGui::InputDouble("Mag ref tau [s]", &params.vqf_magRefTau, 0.0, 0.0, "%.1f");
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Time constant for adjustment of the magnetic field reference.\nAllows reference to converge to observed "
                            "undisturbed field. Default: 20.0");
                    ImGui::InputDouble("Mag norm threshold", &params.vqf_magNormTh, 0.0, 0.0, "%.3f");
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Relative threshold for magnetic field strength for disturbance detection.\nRelative to the reference norm. "
                            "Default: 0.1 (10%%)");
                    ImGui::InputDouble("Mag dip threshold [deg]", &params.vqf_magDipTh, 0.0, 0.0, "%.1f");
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Threshold for the magnetic field dip angle for disturbance detection. Default: 10.0");
                    ImGui::InputDouble("Mag new time [s]", &params.vqf_magNewTime, 0.0, 0.0, "%.1f");
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Duration after which to accept a different homogeneous magnetic field.\nNew reference accepted when within "
                            "magNormTh and magDipTh for this time.\nOnly phases with sufficient movement (magNewMinGyr) count. Default: "
                            "20.0");
                    ImGui::InputDouble("Mag new first time [s]", &params.vqf_magNewFirstTime, 0.0, 0.0, "%.1f");
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Duration after which to accept a homogeneous magnetic field for the first time.\nUsed instead of magNewTime "
                            "when "
                            "no current estimate exists,\nto allow faster initial reference acquisition. Default: 5.0");
                    ImGui::InputDouble("Mag new min gyro [deg/s]", &params.vqf_magNewMinGyr, 0.0, 0.0, "%.1f");
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Minimum angular velocity needed to count time for new mag field acceptance.\nPeriods with angular velocity "
                            "norm "
                            "below this threshold\ndo not count towards magNewTime. Default: 20.0");
                    ImGui::InputDouble("Mag min undisturbed [s]", &params.vqf_magMinUndisturbedTime, 0.0, 0.0, "%.2f");
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Minimum duration within thresholds after which to regard\nthe field as undisturbed again. Default: 0.5");
                    ImGui::InputDouble("Mag max rejection [s]", &params.vqf_magMaxRejectionTime, 0.0, 0.0, "%.1f");
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Maximum duration of full magnetic disturbance rejection.\nUp to this duration, heading correction is fully "
                            "disabled\nand tracked by gyroscope only. After this, correction uses\nincreased time constant "
                            "(magRejectionFactor). Default: 60.0");
                    ImGui::InputDouble("Mag rejection factor", &params.vqf_magRejectionFactor, 0.0, 0.0, "%.2f");
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Factor by which to slow heading correction during long disturbed phases.\nAfter magMaxRejectionTime of full "
                            "rejection, correction uses\nthis factor to increase the time constant. Default: 2.0");
                    ImGui::TreePop();
                }
            } // end if (params.use_vqf)

            ImGui::PopItemWidth();
        }
        ImGui::NewLine();
        if (!step_1_done)
        {
            if (ImGui::Button("Load data"))
            {
                loStartTime = std::chrono::system_clock::now();

                step1(loPause);
                std::cout << "Load data done please click 'Compute all' to continue calculations" << std::endl;

                std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - loStartTime;
                double elapsedSeconds = elapsed.count();

                std::cout << "Elapsed time: " << formatTime(elapsedSeconds).c_str() << std::endl;
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Select folder containing IMU *.csv and LiDAR *.laz files produced by MANDEYE (e.g.: 'continousScanning_*')");
        }
        if (step_1_done && !step_2_done)
        {
            if (ImGui::Button("Compute all"))
                step2(loPause);

            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Press this button for automatic LiDAR odometry calculation -> it will produce trajectory");
        }

        if (step_1_done && step_2_done)
        {
            if (ImGui::Button("Point cloud consistency and trajectory smoothness"))
                run_consistency(worker_data, params);
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                ImGui::Text("This process makes trajectory smooth, point cloud will be more consistent");
                ImGui::Text("Press optionally before pressing 'Save result'");
                ImGui::EndTooltip();
            }

            ImGui::SameLine();
            ImGui::Checkbox("Use multiple Gaussians for each bucket", &params.use_mutliple_gaussian);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Multiple Gaussians suppose to work better in floor plan indoor scenarios (multiple neighbouring rooms)");

            if (ImGui::Button("Save result"))
                save_results(true, 0.0);
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                ImGui::Text("Press this button for saving resulting trajectory and point clouds");
                ImGui::Text("as single session for 'multi_view_tls_registration_step_2' program");
                ImGui::EndTooltip();
            }
        }
        if (step_1_done && step_2_done)
        {
            if (ImGui::Button("Save all point clouds to single las/laz file"))
            {
                const auto output_file_name = mandeye::fd::SaveFileDialog("Save las/laz file", mandeye::fd::LAS_LAZ_filter, ".laz");

                if (output_file_name.size() > 0)
                {
                    session.fill_session_from_worker_data(worker_data, false, true, true, params.threshould_output_filter);
                    // save_all_to_las(session, output_file_name, false);
                    save_all_to_las(session, output_file_name, true, true);
                }
            }
        }
        if (!simple_gui)
        {
            ImGui::NewLine();
            ImGui::Checkbox("Use robust and accurate lidar odometry", &params.use_robust_and_accurate_lidar_odometry);

            if (params.use_robust_and_accurate_lidar_odometry)
            {
                ImGui::PushItemWidth(ImGuiNumberWidth);
                ImGui::InputDouble("Distance bucket [m]", &params.distance_bucket, 0.0, 0.0, "%.3f");
                ImGui::InputDouble("Polar angle [deg]", &params.polar_angle_deg, 0.0, 0.0, "%.3f");
                ImGui::InputDouble("Azimutal angle [deg]", &params.azimutal_angle_deg, 0.0, 0.0, "%.3f");
                ImGui::InputInt("Number of iterations", &params.robust_and_accurate_lidar_odometry_iterations);
                // ImGui::InputDouble("Max distance lidar", &params.max_distance_lidar);
                ImGui::PopItemWidth();
            }

            ImGui::NewLine();

            ImGui::Text("Rigid ICP using spherical coordinates");
            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputDouble("Distance bucket [m]##ricp", &params.distance_bucket_rigid_sf, 0.0, 0.0, "%.3f");
            ImGui::InputDouble("Polar angle [deg]##ricp", &params.polar_angle_deg_rigid_sf, 0.0, 0.0, "%.3f");
            ImGui::InputDouble("azimutal angle [deg]##ricp", &params.azimutal_angle_deg_rigid_sf, 0.0, 0.0, "%.3f");
            ImGui::InputInt("Number of iterations##ricp", &params.robust_and_accurate_lidar_odometry_rigid_sf_iterations);
            ImGui::InputDouble("Max distance [m]##ricp", &params.max_distance_lidar_rigid_sf, 0.0, 0.0, "%.2f");
            ImGui::PopItemWidth();

            ImGui::NewLine();

            ImGui::Text("Rigid spherical feature sigmas [m] / [deg]:");
            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputDouble("x##rgdsfs", &params.rgd_sf_sigma_x_m, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(xText);
            ImGui::SameLine();
            ImGui::InputDouble("om##rgdsfs", &params.rgd_sf_sigma_om_deg, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(omText);

            ImGui::InputDouble("y##rgdsfs", &params.rgd_sf_sigma_y_m, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(yText);
            ImGui::SameLine();
            ImGui::InputDouble("fi##rgdsfs", &params.rgd_sf_sigma_fi_deg, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(fiText);

            ImGui::InputDouble("z##rgdsfs", &params.rgd_sf_sigma_z_m, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(zText);
            ImGui::SameLine();
            ImGui::InputDouble("ka##rgdsfs", &params.rgd_sf_sigma_ka_deg, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(kaText);

            ImGui::PopItemWidth();

            ImGui::NewLine();

            if (ImGui::Button("Save trajectory to ASCII (x y z)"))
            {
                std::string output_file_name = "";
                output_file_name = mandeye::fd::SaveFileDialog("Save trajectory", {}, "");
                std::cout << "File to save: '" << output_file_name << "'" << std::endl;

                if (output_file_name.size() > 0)
                {
                    save_trajectory_to_ascii(worker_data, output_file_name);
                }
            }

            ImGui::NewLine();
            ImGui::Text("Reference point clouds for initial alignment:");

            static std::vector<std::string> input_file_names;

            if (ImGui::Button("Load laz/las files"))
            {
                input_file_names = mandeye::fd::OpenFileDialog("Load laz/las files", mandeye::fd::LAS_LAZ_filter, true);
                if (input_file_names.size() > 0)
                {
                    show_reference_points = true;
                    load_reference_point_clouds(input_file_names, params);
                }
            }

            if (ImGui::Button("Set parameters for drone with Ouster (https://ntu-aris.github.io/ntu_viral_dataset/)"))
            {
                params.decimation = 1.0;
                params.in_out_params_indoor.resolution_X = 1.0;
                params.in_out_params_indoor.resolution_Y = 1.0;
                params.in_out_params_indoor.resolution_Z = 1.0;

                params.in_out_params_outdoor.resolution_X = 2.0;
                params.in_out_params_outdoor.resolution_Y = 2.0;
                params.in_out_params_outdoor.resolution_Z = 2.0;
            }

            if (!input_file_names.empty())
            {
                ImGui::Checkbox("Show points", &show_reference_points);
                ImGui::SameLine();
                // ImGui::Checkbox("Show buckets ", &show_reference_buckets);
                ImGui::SetNextItemWidth(ImGuiNumberWidth);
                ImGui::InputInt("Downsamplingn###ref", &dec_reference_points);

                if (ImGui::Button("Filter reference buckets"))
                {
                    filter_reference_buckets(params);
                }
            }

            if (params.initial_points.size() > 0)
            {
                ImGui::NewLine();
                ImGui::Text("Manipulate initial transformation:");
                ImGui::Checkbox("Show gizmo", &initial_transformation_gizmo);
                // gizmo_stretch_interval
                if (initial_transformation_gizmo)
                {
                    m_gizmo[0] = (float)params.m_g(0, 0);
                    m_gizmo[1] = (float)params.m_g(1, 0);
                    m_gizmo[2] = (float)params.m_g(2, 0);
                    m_gizmo[3] = (float)params.m_g(3, 0);
                    m_gizmo[4] = (float)params.m_g(0, 1);
                    m_gizmo[5] = (float)params.m_g(1, 1);
                    m_gizmo[6] = (float)params.m_g(2, 1);
                    m_gizmo[7] = (float)params.m_g(3, 1);
                    m_gizmo[8] = (float)params.m_g(0, 2);
                    m_gizmo[9] = (float)params.m_g(1, 2);
                    m_gizmo[10] = (float)params.m_g(2, 2);
                    m_gizmo[11] = (float)params.m_g(3, 2);
                    m_gizmo[12] = (float)params.m_g(0, 3);
                    m_gizmo[13] = (float)params.m_g(1, 3);
                    m_gizmo[14] = (float)params.m_g(2, 3);
                    m_gizmo[15] = (float)params.m_g(3, 3);
                }
                else
                {
                    if (ImGui::Button("Align to reference"))
                    {
                        for (int i = 0; i < 30; i++)
                        {
                            align_to_reference(params.in_out_params_indoor, params.initial_points, params.m_g, params.buckets_indoor);
                        }
                    }
                }
            }

            if (worker_data.size() > 0)
            {
                ImGui::NewLine();

                ImGui::Text("Scans selection:");

                ImGui::Text("index from: ");
                ImGui::SameLine();
                ImGui::PushItemWidth(ImGuiNumberWidth);
                ImGui::SliderInt("##fs", &index_begin, 0, index_end);
                ImGui::SameLine();
                ImGui::InputInt("##fi", &index_begin, 1, 5);
                if (index_begin < 0)
                    index_begin = 0;
                if (index_begin >= index_end)
                    index_begin = std::min(index_end, static_cast<int>(worker_data.size() - 1));

                ImGui::SameLine();
                ImGui::Text(" to: ");
                ImGui::SameLine();

                int prev = index_end;
                ImGui::SliderInt("##ts", &index_end, index_begin, static_cast<int>(worker_data.size() - 1));
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("max %zu", worker_data.size() - 1);
                ImGui::SameLine();
                ImGui::InputInt("##ti", &index_end, 1, 5);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("max %zu", worker_data.size() - 1);
                if (index_end < index_begin)
                    index_end = index_begin;
                if (index_end >= worker_data.size() - 1)
                    index_end = worker_data.size() - 1;
                if (prev != index_end)
                {
                    stretch_gizmo_m = worker_data[index_end].intermediate_trajectory[0];
                }

                ImGui::PopItemWidth();

                ImGui::Text("Selection: ");
                if (ImGui::Button("Select all"))
                {
                    for (size_t k = 0; k < worker_data.size(); k++)
                    {
                        worker_data[k].show = true;
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("Unselect"))
                {
                    for (size_t k = 0; k < worker_data.size(); k++)
                    {
                        worker_data[k].show = false;
                    }
                }

                ImGui::Text("Select scans from range:");
                ImGui::SameLine();
                if (ImGui::Button("<from, to>"))
                {
                    for (size_t k = 0; k < worker_data.size(); k++)
                        worker_data[k].show = (k >= index_begin && k <= index_end);
                }

                if (ImGui::Button("<from - 1, to - 1>"))
                {
                    if (index_begin > 1 && index_end > 1)
                    {
                        index_begin--;
                        index_end--;

                        for (size_t k = 0; k < worker_data.size(); k++)
                            worker_data[k].show = (k >= index_begin && k <= index_end);
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("<from + 1, to + 1>"))
                {
                    if (index_begin + 1 < worker_data.size() && index_end + 1 < worker_data.size())
                    {
                        index_begin++;
                        index_end++;

                        for (size_t k = 0; k < worker_data.size(); k++)
                            worker_data[k].show = (k >= index_begin && k <= index_end);
                    }
                }

                if (ImGui::Button("from - 10, to - 10>"))
                {
                    if (index_begin > 10 && index_end > 10)
                    {
                        index_begin -= 10;
                        index_end -= 10;

                        for (size_t k = 0; k < worker_data.size(); k++)
                            worker_data[k].show = (k >= index_begin && k <= index_end);
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("<from + 10, to + 10>"))
                {
                    if (index_begin + 10 < worker_data.size() && index_end + 10 < worker_data.size())
                    {
                        index_begin += 10;
                        index_end += 10;

                        for (size_t k = 0; k < worker_data.size(); k++)
                            worker_data[k].show = (k >= index_begin && k <= index_end);
                    }
                }

                if (ImGui::Button("Export selected scans"))
                {
                    auto output_file_name = mandeye::fd::SaveFileDialog("Save las/laz file", mandeye::fd::LAS_LAZ_filter, ".laz");
                    Eigen::Affine3d pose;
                    if (output_file_name.size() > 0)
                    {
                        session.fill_session_from_worker_data(worker_data, true, false, false, params.threshould_output_filter);
                        save_all_to_las(session, output_file_name, false, true);
                    }
                    // TODO: give value to pose even if output_file_name is wrong

                    std::cout << "----------------------------------------" << std::endl;
                    std::cout << "please add following lines to RESSO file:" << std::endl;
                    std::cout << fs::path(output_file_name).filename() << "(!!!please remove brackets!!!)" << std::endl;
                    std::cout << pose.matrix() << std::endl;

                    std::cout << "example RESSO file" << std::endl;
                    std::cout << ".................................................." << std::endl;
                    std::cout << "3" << std::endl
                              << "scan_0.laz" << std::endl
                              << "0.999775 0.000552479 -0.0212158 -0.0251188" << std::endl
                              << "0.000834612 0.997864 0.0653156 -0.0381429" << std::endl
                              << "0.0212066 - 0.0653186 0.997639 -0.000757752" << "0 0 0 1" << std::endl
                              << "scan_1.laz" << std::endl
                              << "0.999783 0.00178963 -0.0207603 -0.0309683" << std::endl
                              << "-0.000467341 0.99798 0.0635239 -0.0517512" << std::endl
                              << "0.0208321 -0.0635004 0.997764 0.00331449" << std::endl
                              << "0 0 0 1" << std::endl
                              << "scan_2.laz" << std::endl
                              << "0.999783 0.00163449 -0.0207736 -0.0309985" << std::endl
                              << "-0.000312224 0.997982 0.0634957 -0.0506113" << std::endl
                              << "0.0208355 -0.0634754 0.997766 0.0028499" << std::endl
                              << "0 0 0 1" << std::endl;
                    std::cout << "................................................." << std::endl;
                }

                ImGui::SameLine();

                if (ImGui::Button("Save RESSO file"))
                {
                    auto output_file_name = mandeye::fd::SaveFileDialog("Save RESSO file", {}, "");
                    std::cout << "RESSO file to save: '" << output_file_name << "'" << std::endl;

                    if (output_file_name.size() > 0)
                    {
                        session.point_clouds_container.save_poses(fs::path(output_file_name).string(), false);
                    }
                }

                if (index_end > index_begin)
                {
                    ImGui::Separator();
                    ImGui::Checkbox("Gizmo_stretch_interval", &gizmo_stretch_interval);
                }

                // gizmo_stretch_interval
                if (gizmo_stretch_interval)
                {
                    if (index_end < worker_data.size())
                    {
                        if (worker_data[index_end].intermediate_trajectory.size() > 0)
                        {
                            m_gizmo[0] = (float)stretch_gizmo_m(0, 0);
                            m_gizmo[1] = (float)stretch_gizmo_m(1, 0);
                            m_gizmo[2] = (float)stretch_gizmo_m(2, 0);
                            m_gizmo[3] = (float)stretch_gizmo_m(3, 0);
                            m_gizmo[4] = (float)stretch_gizmo_m(0, 1);
                            m_gizmo[5] = (float)stretch_gizmo_m(1, 1);
                            m_gizmo[6] = (float)stretch_gizmo_m(2, 1);
                            m_gizmo[7] = (float)stretch_gizmo_m(3, 1);
                            m_gizmo[8] = (float)stretch_gizmo_m(0, 2);
                            m_gizmo[9] = (float)stretch_gizmo_m(1, 2);
                            m_gizmo[10] = (float)stretch_gizmo_m(2, 2);
                            m_gizmo[11] = (float)stretch_gizmo_m(3, 2);
                            m_gizmo[12] = (float)stretch_gizmo_m(0, 3);
                            m_gizmo[13] = (float)stretch_gizmo_m(1, 3);
                            m_gizmo[14] = (float)stretch_gizmo_m(2, 3);
                            m_gizmo[15] = (float)stretch_gizmo_m(3, 3);
                        }
                    }

                    if (ImGui::Button("Accept Gizmo (only translation)"))
                    {
                        if (index_end < worker_data.size())
                        {
                            if (worker_data[index_end].intermediate_trajectory.size() > 0)
                            {
                                Eigen::Affine3d current_gizmo = Eigen::Affine3d::Identity();
                                current_gizmo(0, 0) = m_gizmo[0];
                                current_gizmo(1, 0) = m_gizmo[1];
                                current_gizmo(2, 0) = m_gizmo[2];
                                current_gizmo(3, 0) = m_gizmo[3];
                                current_gizmo(0, 1) = m_gizmo[4];
                                current_gizmo(1, 1) = m_gizmo[5];
                                current_gizmo(2, 1) = m_gizmo[6];
                                current_gizmo(3, 1) = m_gizmo[7];
                                current_gizmo(0, 2) = m_gizmo[8];
                                current_gizmo(1, 2) = m_gizmo[9];
                                current_gizmo(2, 2) = m_gizmo[10];
                                current_gizmo(3, 2) = m_gizmo[11];
                                current_gizmo(0, 3) = m_gizmo[12];
                                current_gizmo(1, 3) = m_gizmo[13];
                                current_gizmo(2, 3) = m_gizmo[14];
                                current_gizmo(3, 3) = m_gizmo[15];

                                auto first_pose = worker_data[index_begin].intermediate_trajectory[0];

                                Eigen::Vector3d translation = current_gizmo.translation() - first_pose.translation();

                                float number_all_nodes_inside_interval = 0;
                                for (int i = index_begin; i < index_end; i++)
                                {
                                    number_all_nodes_inside_interval += (float)worker_data[i].intermediate_trajectory.size();
                                }

                                std::vector<std::vector<Eigen::Affine3d>> all_poses;
                                for (size_t i = 0; i < worker_data.size(); i++)
                                {
                                    std::vector<Eigen::Affine3d> poses;
                                    for (size_t j = 0; j < worker_data[i].intermediate_trajectory.size(); j++)
                                    {
                                        poses.push_back(worker_data[i].intermediate_trajectory[j]);
                                    }
                                    all_poses.push_back(poses);
                                }

                                float counter = 0;

                                Eigen::Affine3d last_m = Eigen::Affine3d::Identity();

                                for (int i = index_begin; i < index_end; i++)
                                {
                                    for (size_t j = 0; j < worker_data[i].intermediate_trajectory.size(); j++)
                                    {
                                        TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(worker_data[i].intermediate_trajectory[j]);

                                        pose.px =
                                            first_pose.translation().x() + translation.x() * (counter / number_all_nodes_inside_interval);
                                        pose.py =
                                            first_pose.translation().y() + translation.y() * (counter / number_all_nodes_inside_interval);
                                        pose.pz =
                                            first_pose.translation().z() + translation.z() * (counter / number_all_nodes_inside_interval);

                                        counter += 1.0f;

                                        worker_data[i].intermediate_trajectory[j] = affine_matrix_from_pose_tait_bryan(pose);

                                        last_m = worker_data[i].intermediate_trajectory[j];
                                    }
                                }

                                for (size_t i = index_end; i < worker_data.size(); i++)
                                {
                                    for (size_t j = 0; j < worker_data[i].intermediate_trajectory.size(); j++)
                                    {
                                        Eigen::Affine3d m_update = Eigen::Affine3d::Identity();

                                        if (j == 0)
                                        {
                                            m_update = all_poses[i - 1][all_poses[i - 1].size() - 1].inverse() * all_poses[i][0];
                                        }
                                        else
                                        {
                                            m_update = all_poses[i][j - 1].inverse() * all_poses[i][j];
                                        }
                                        last_m = last_m * m_update;
                                        worker_data[i].intermediate_trajectory[j] = last_m;
                                    }
                                }
                            }
                        }
                    }
                    ImGui::Separator();
                }

                ImGui::NewLine();
                ImGui::Text("Show/hide individual scans:");

                int n_items = (int)worker_data.size();
                int digits = (n_items > 0) ? (int)std::log10(n_items - 1) + 1 : 1;

                for (int i = 0; i < n_items; i++)
                {
                    // std::string text = "[" + std::to_string(i) + "]";
                    // ImGui::Checkbox(text.c_str(), &worker_data[i].show);
                    std::stringstream ss;
                    ss << "[" << std::setw(digits) << std::setfill('0') << i << "]";

                    ImGui::Checkbox(ss.str().c_str(), &worker_data[i].show);

                    // Arrange checkboxes in rows of 5
                    if ((i + 1) % 5 != 0)
                        ImGui::SameLine();
                }
            }
        }
    }

    ImGui::End();
}

void progress_window()
{
    ImGui::Begin("Progress", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::Text("Working directory:\n'%s'", working_directory.c_str());

    // Calculate elapsed time and ETA
    auto currentTime = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = currentTime - loStartTime;
    double elapsedSeconds = elapsed.count();
    loElapsedSeconds.store(elapsedSeconds);

    float progress = loProgress.load();
    double estimatedTimeRemaining = 0.0;

    if (progress > 0.01f && progress < 100.0f)
    { // Only estimate when we have meaningful progress
        double totalEstimatedTime = elapsedSeconds / progress;
        estimatedTimeRemaining = totalEstimatedTime - elapsedSeconds;
        loEstimatedTimeRemaining.store(estimatedTimeRemaining);
    }

    // Format progress text with time information
    char progressText[256];
    char timeInfo[512];

    if (progress > 0.01f)
    {
        std::string completionTime = formatCompletionTime(estimatedTimeRemaining);
        snprintf(progressText, sizeof(progressText), "Processing: %.1f%%", progress * 100.0f);
        snprintf(
            timeInfo,
            sizeof(timeInfo),
            "Elapsed: %s | Remaining: %s | Estimated finish: ~%s",
            formatTime(elapsedSeconds).c_str(),
            formatTime(estimatedTimeRemaining).c_str(),
            completionTime.c_str());
    }
    else
    {
        snprintf(progressText, sizeof(progressText), "Processing: %.1f%%", progress * 100.0f);
        snprintf(timeInfo, sizeof(timeInfo), "Elapsed: %s | Calculating completion time...", formatTime(elapsedSeconds).c_str());
    }

    ImGui::ProgressBar(progress, ImVec2(-1.0f, 0.0f), progressText);
    ImGui::Text("%s", timeInfo);

#ifdef _WIN32
    // Update Windows taskbar progress
    if (progress > 0.01f && progress < 1.0f)
    {
        SetTaskbarProgress(progress);
    }
    else if (progress >= 1.0f)
    {
        ClearTaskbarProgress();
    }
#endif

    ImGui::NewLine();

    if (!loPause)
    {
        if (ImGui::Button("Pause"))
            loPause.store(true);

        if (ImGui::IsItemHovered())
        {
            ImGui::BeginTooltip();
            ImGui::Text("Pausing calculation will not save intermediate state!");
            ImGui::Text("You can't resume if you close the program!");
            ImGui::EndTooltip();
        }
    }
    else
    {
        if (ImGui::Button("Resume"))
            loPause.store(false);
    }
    ImGui::SameLine();
    ImGui::Text("Also check console for progress..");

    ImGui::Checkbox("Show reference buckets indoor", &show_reference_buckets_indoor);
    ImGui::Checkbox("Show reference buckets outdoor", &show_reference_buckets_outdoor);
    ImGui::Checkbox("Show initial points", &show_initial_points);
    ImGui::Checkbox("Show trajectory", &show_trajectory);
    ImGui::Checkbox("Show trajectory as axes", &show_trajectory_as_axes);
    ImGui::Checkbox("Show prediction vectors", &show_prediction_vectors);
    ImGui::Checkbox("Show intermediate trajectory prediction axes", &intermediate_trajectory_prediction_axes);
    ImGui::Checkbox("Show without filtered buckets", &show_without_filtered_buckets);
    ImGui::Checkbox("Show normal vectors indoor", &show_normal_vectors_indoor);
    ImGui::Checkbox("Show normal vectors outdoor", &show_normal_vectors_outdoor);

    ImGui::End();
}

//! Loads data and starts processing in the background.
//! @param folder data folder; when empty, the user is asked to select one.
void openData(const std::string& folder = "")
{
    is_settings_gui = false;
    app_state.info_gui = false;

    loRunning.store(true);
    loProgress.store(0.0f);
    loElapsedSeconds.store(0.0);
    loEstimatedTimeRemaining.store(0.0);
    loStartTime = std::chrono::system_clock::now();

    step1(loPause, folder);

    if (step_1_done)
    {
        std::thread loThread(
            []()
            {
                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();

                step2(loPause);

                end = std::chrono::system_clock::now();
                std::chrono::duration<double> elapsed_seconds = end - start;

                save_results(false, elapsed_seconds.count());

                loRunning = false;

                std::ostringstream oss;
                oss << "Data saved to folder:\n'" << outwd.string() << "'\n"
                    << "Calculated trajectory length: " << std::fixed << std::setprecision(1)
                    << params.total_length_of_calculated_trajectory << "[m]\n"
                    << "Elapsed time: " << formatTime(elapsed_seconds.count()).c_str();

                [[maybe_unused]] pfd::message message("Information", oss.str(), pfd::choice::ok, pfd::icon::info);
                message.result();
            });

        loThread.detach();
    }
    else // no data loaded
        loRunning = false;
}

void step1(
    const std::string& folder,
    LidarOdometryParams& params,
    std::vector<std::vector<Point3Di>>& pointsPerFile,
    Imu& imu_data,
    std::string& working_directory,
    Trajectory& trajectory,
    std::vector<WorkerData>& worker_data,
    const std::atomic<bool>& loPause)
{
    std::vector<std::string> input_file_names;

    for (const auto& entry : fs::directory_iterator(folder))
    {
        if (!entry.is_directory())
        {
            std::cout << entry.path() << std::endl;
            input_file_names.push_back(entry.path().string());
        }
    }

    if (load_data(input_file_names, params, pointsPerFile, imu_data, full_debug_messages))
    {
        working_directory = fs::path(input_file_names[0]).parent_path().string();
        calculate_trajectory(trajectory, imu_data, params, full_debug_messages);
        compute_step_1(pointsPerFile, params, trajectory, worker_data, loPause);
        std::cout << "step_1_done" << std::endl;
    }
}

void step2(std::vector<WorkerData>& worker_data, LidarOdometryParams& params, const std::atomic<bool>& loPause)
{
    double ts_failure = 0.0;
    std::atomic<float> loProgress;
    compute_step_2(worker_data, params, ts_failure, loProgress, loPause, full_debug_messages);
}

void save_results(
    bool info,
    double elapsed_seconds,
    std::string& working_directory,
    std::vector<WorkerData>& worker_data,
    LidarOdometryParams& params,
    fs::path outwd)
{
    save_result(worker_data, params, outwd, elapsed_seconds);
}

// Pure colors -- raylib's named RED/GREEN/BLUE are tinted, unlike the original glColor3f values.
constexpr Color kRed{ 255, 0, 0, 255 };
constexpr Color kGreen{ 0, 255, 0, 255 };
constexpr Color kBlue{ 0, 0, 255, 255 };
constexpr Color kCyan{ 0, 255, 255, 255 };
constexpr Color kMagenta{ 255, 0, 255, 255 };

// Collects bucket means into the two point sets drawn per bucket map (regular and number_of_points == -1).
void collectBucketMeans(const NDTBucketMapType& buckets, std::vector<Eigen::Vector3d>& regular, std::vector<Eigen::Vector3d>& marked)
{
    regular.clear();
    marked.clear();
    for (const auto& b : buckets)
    {
        if (show_without_filtered_buckets && b.second.number_of_hits >= 20)
            continue;
        (b.second.number_of_points == -1 ? marked : regular).push_back(b.second.mean);
    }
}

void drawBucketNormals(const NDTBucketMapType& buckets)
{
    rlBegin(RL_LINES);
    for (const auto& b : buckets)
    {
        const auto& n = b.second.normal_vector;
        const auto& m = b.second.mean;
        rlColor3f(fabs(n.x()), fabs(n.y()), fabs(n.z()));
        rlVertex3f(m.x(), m.y(), m.z());
        rlVertex3f(m.x() + n.x(), m.y() + n.y(), m.z() + n.z());
    }
    rlEnd();
}

void renderScene()
{
    const float point_size = static_cast<float>(app_state.point_size);

    showAxes();

    if (show_initial_points && !params.initial_points.empty())
    {
        // Re-uploaded only when the points or params.m_g (moved by the initial-transformation gizmo) change.
        if (scene_points.initial_count != params.initial_points.size() || scene_points.initial_m_g != params.m_g.matrix())
        {
            std::vector<Eigen::Vector3d> pts;
            pts.reserve(params.initial_points.size());
            for (const auto& p : params.initial_points)
                pts.push_back(params.m_g * p.point);
            scan_renderer.uploadPoints(scene_points.initial, pts);
            scene_points.initial_count = params.initial_points.size();
            scene_points.initial_m_g = params.m_g.matrix();
        }
        scan_renderer.drawPoints(scene_points.initial, kGreen, point_size);
    }

    if (show_reference_points)
    {
        const int dec = std::max(1, dec_reference_points);
        if (scene_points.reference_count != params.reference_points.size() || scene_points.reference_dec != dec)
        {
            std::vector<Eigen::Vector3d> pts;
            for (size_t i = 0; i < params.reference_points.size(); i += dec)
                pts.push_back(params.reference_points[i].point);
            scan_renderer.uploadPoints(scene_points.reference, pts);
            scene_points.reference_count = params.reference_points.size();
            scene_points.reference_dec = dec;
        }
        scan_renderer.drawPoints(scene_points.reference, kRed, point_size);
    }

    if (show_trajectory_as_axes)
    {
        rlBegin(RL_LINES);
        for (const auto& wd : worker_data)
            for (const auto& it : wd.intermediate_trajectory)
                drawAxesAt(it.translation(), it.linear(), 0.1);
        rlEnd();
    }

    if (show_prediction_vectors)
    {
        // Depth test and line width are GL state, applied when rlgl flushes its batch -- flush around the change.
        rlDrawRenderBatchActive();
        rlDisableDepthTest();
        rlSetLineWidth(3.0f);
        rlBegin(RL_LINES);
        rlColor3f(1.0f, 0.0f, 1.0f);
        for (const auto& wd : worker_data)
        {
            if (wd.intermediate_trajectory.empty() || wd.imu_prediction_vector.norm() < 1e-6)
                continue;

            const Eigen::Vector3d start = wd.intermediate_trajectory.front().translation();
            const Eigen::Vector3d end = start + wd.imu_prediction_vector;
            rlVertex3f(start.x(), start.y(), start.z());
            rlVertex3f(end.x(), end.y(), end.z());
        }
        rlEnd();
        rlDrawRenderBatchActive();
        rlSetLineWidth(1.0f);
        rlEnableDepthTest();
    }

    if (intermediate_trajectory_prediction_axes)
    {
        rlBegin(RL_LINES);
        for (const auto& wd : worker_data)
        {
            const size_t n = std::min(wd.intermediate_trajectory.size(), wd.intermediate_trajectory_prediction.size());
            for (size_t i = 0; i < n; i++)
                drawAxesAt(wd.intermediate_trajectory[i].translation(), wd.intermediate_trajectory_prediction[i].linear(), 0.05);
        }
        rlEnd();
    }

    if (show_trajectory)
    {
        std::vector<Eigen::Vector3d> pts;
        for (const auto& wd : worker_data)
            for (const auto& it : wd.intermediate_trajectory)
                pts.push_back(it.translation());
        uploadIfChanged(scene_points.trajectory, scene_points.trajectory_uploaded, pts);
        scan_renderer.drawPoints(scene_points.trajectory, kCyan, 3.0f);

        if (!worker_data.empty() && !worker_data.back().intermediate_trajectory.empty())
        {
            rlBegin(RL_LINES);
            rlColor3f(1, 0, 0);
            drawCross(worker_data.back().intermediate_trajectory.back().translation(), 1.0);
            rlEnd();
        }
    }

    std::vector<Eigen::Vector3d> regular, marked;

    if (show_reference_buckets_indoor)
    {
        {
            std::scoped_lock lock(params.mutex_buckets_indoor);
            collectBucketMeans(params.buckets_indoor, regular, marked);
        }
        uploadIfChanged(scene_points.buckets[0], scene_points.buckets_uploaded[0], regular);
        uploadIfChanged(scene_points.buckets[1], scene_points.buckets_uploaded[1], marked);
        scan_renderer.drawPoints(scene_points.buckets[0], kRed, 1.0f);
        scan_renderer.drawPoints(scene_points.buckets[1], kCyan, 1.0f);
    }

    if (show_reference_buckets_outdoor)
    {
        {
            std::scoped_lock lock(params.mutex_buckets_outdoor);
            collectBucketMeans(params.buckets_outdoor, regular, marked);
        }
        uploadIfChanged(scene_points.buckets[2], scene_points.buckets_uploaded[2], regular);
        uploadIfChanged(scene_points.buckets[3], scene_points.buckets_uploaded[3], marked);
        scan_renderer.drawPoints(scene_points.buckets[2], kBlue, 1.0f);
        scan_renderer.drawPoints(scene_points.buckets[3], kMagenta, 1.0f);
    }

    if (show_normal_vectors_indoor)
    {
        std::scoped_lock lock(params.mutex_buckets_indoor);
        drawBucketNormals(params.buckets_indoor);
    }

    if (show_normal_vectors_outdoor)
    {
        std::scoped_lock lock(params.mutex_buckets_outdoor);
        drawBucketNormals(params.buckets_outdoor);
    }

    // Scan selection range markers: yellow at index_begin, cyan at index_end.
    const auto drawIndexMarker = [](int index, float r, float g, float b)
    {
        if (index < 0 || index >= static_cast<int>(worker_data.size()) || worker_data[index].intermediate_trajectory.empty())
            return;
        rlBegin(RL_LINES);
        rlColor3f(r, g, b);
        drawCross(worker_data[index].intermediate_trajectory[0].translation(), 1.0);
        rlEnd();
    };
    drawIndexMarker(index_begin, 1, 1, 0);
    drawIndexMarker(index_end, 0, 1, 1);
}

void display()
{
    ImGuiIO& io = ImGui::GetIO();
    // Framebuffer pixels, not io.DisplaySize: they differ on HiDPI displays.
    rlViewport(0, 0, GetRenderWidth(), GetRenderHeight());

    const ImVec4& bg = app_state.bg_color;
    ClearBackground(ColorFromNormalized(Vector4{ bg.x * bg.w, bg.y * bg.w, bg.z * bg.w, bg.w }));
    rlEnableDepthTest();

    rlMatrixMode(RL_PROJECTION);
    rlLoadIdentity();
    const float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);

    auto& cam = app_state.camera;
    cam.updateEulerTransition(io.DeltaTime);

    app_state.viewLocal = Eigen::Affine3f::Identity();

    if (!cam.isOrtho)
    {
        cam.applyPerspectiveProjection((int)io.DisplaySize.x, (int)io.DisplaySize.y);

        const Eigen::Vector3f rotationCenter(cam.euler.rotationCenter.x, cam.euler.rotationCenter.y, cam.euler.rotationCenter.z);
        app_state.viewLocal.translate(rotationCenter);
        app_state.viewLocal.translate(Eigen::Vector3f(cam.euler.translate.x, cam.euler.translate.y, cam.euler.translate.z));
        if (!cam.lockZ)
            app_state.viewLocal.rotate(Eigen::AngleAxisf(cam.euler.rotateX * DEG_TO_RAD, Eigen::Vector3f::UnitX()));
        else
            app_state.viewLocal.rotate(Eigen::AngleAxisf(-90.0 * DEG_TO_RAD, Eigen::Vector3f::UnitX()));
        app_state.viewLocal.rotate(Eigen::AngleAxisf(cam.euler.rotateY * DEG_TO_RAD, Eigen::Vector3f::UnitZ()));
        app_state.viewLocal.translate(-rotationCenter);

        rlMultMatrixf(app_state.viewLocal.matrix().data());
    }
    else
    {
        app_state.viewLocal.rotate(Eigen::AngleAxisf((cam.euler.rotateX + cam.euler.rotateY) * DEG_TO_RAD, Eigen::Vector3f::UnitZ()));
        cam.updateOrtho(ratio);
    }

    cam.captureFrameMatrices();

    renderScene();

    // Only polls input into ImGui and starts its frame; the 3D matrices above stay active until end3DMatrixStack().
    rlImGuiBegin();

    raylib_widgets::ShowMainDockSpace();

    view_kbd_shortcuts();

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_O, false))
    {
        openData();

        // workaround
        io.AddKeyEvent(ImGuiKey_O, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
    }

    if (!loRunning)
    {
        if (ImGui::BeginMainMenuBar())
        {
            if (ImGui::BeginMenu("Presets"))
            {
                if (ImGui::MenuItem("1 Velocity < 8km/h, tiny spaces (default)", nullptr, (lastPar == 1)))
                {
                    lastPar = 1;

                    set_lidar_odometry_default_params(params);

                    std::cout << "clicked: Set parameters for velocity up to 8km/h, tiny spaces" << std::endl;
                }
                if (ImGui::MenuItem("2 Velocity < 8km/h, larger spaces, precise forestry (generic)", nullptr, (lastPar == 2)))
                {
                    lastPar = 2;

                    params.decimation = 0.01;
                    params.in_out_params_indoor.resolution_X = 0.1;
                    params.in_out_params_indoor.resolution_Y = 0.1;
                    params.in_out_params_indoor.resolution_Z = 0.1;

                    params.in_out_params_outdoor.resolution_X = 0.3;
                    params.in_out_params_outdoor.resolution_Y = 0.3;
                    params.in_out_params_outdoor.resolution_Z = 0.3;

                    params.filter_threshold_xy_inner = 1.0;
                    params.filter_threshold_xy_outer = 70.0;
                    params.threshould_output_filter = 1.5;

                    params.use_robust_and_accurate_lidar_odometry = false;
                    params.distance_bucket = 0.2;
                    params.polar_angle_deg = 10.0;
                    params.azimutal_angle_deg = 10.0;
                    params.robust_and_accurate_lidar_odometry_iterations = 20;

                    params.max_distance_lidar = 70.0;
                    params.nr_iter = 500;
                    params.sliding_window_trajectory_length_threshold = 10000;
                    params.real_time_threshold_seconds = 10;

                    std::cout << "clicked: Set parameters for velocity < 8km/h, larger spaces, precise forestry (generic)" << std::endl;
                }
                if (ImGui::MenuItem("3 Velocity < 30 km/h, largest open spaces, fast motion", nullptr, (lastPar == 3)))
                {
                    lastPar = 3;

                    params.decimation = 0.03;
                    params.in_out_params_indoor.resolution_X = 0.3;
                    params.in_out_params_indoor.resolution_Y = 0.3;
                    params.in_out_params_indoor.resolution_Z = 0.3;

                    params.in_out_params_outdoor.resolution_X = 0.5;
                    params.in_out_params_outdoor.resolution_Y = 0.5;
                    params.in_out_params_outdoor.resolution_Z = 0.5;

                    params.filter_threshold_xy_inner = 1.0;
                    params.filter_threshold_xy_outer = 70.0;
                    params.threshould_output_filter = 3.0;

                    params.use_robust_and_accurate_lidar_odometry = false;
                    params.distance_bucket = 0.2;
                    params.polar_angle_deg = 10.0;
                    params.azimutal_angle_deg = 10.0;
                    params.robust_and_accurate_lidar_odometry_iterations = 20;

                    params.max_distance_lidar = 70.0;
                    params.nr_iter = 500;
                    params.sliding_window_trajectory_length_threshold = 200;
                    params.real_time_threshold_seconds = 10;

                    std::cout << "clicked: Set parameters for velocity < 30 km/h, largest open spaces, fast motion" << std::endl;
                }
                ImGui::Separator();
                if (ImGui::MenuItem(
                        "4: Best accuracy, precision, and robustness (uncheck to calculate faster)",
                        nullptr,
                        params.ablation_study_use_anisotropic_weighting))
                {
                    params.ablation_study_use_anisotropic_weighting = !params.ablation_study_use_anisotropic_weighting;
                }

                ImGui::EndMenu();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Set parameters for MANDEYE data processing");

            if (ImGui::BeginMenu("Parameters"))
            {
                ImGui::MenuItem(
                    "Remove IMU bias from first stationary scan", nullptr, &params.use_removie_imu_bias_from_first_stationary_scan);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("IMU bias will be removed given there's a stationary period at the start of the recording");

                ImGui::MenuItem("Multithread", nullptr, &params.useMultithread);
                ImGui::SetNextItemWidth(ImGuiNumberWidth / 2);
                ImGui::InputDouble("Time threshold [s]", &params.real_time_threshold_seconds, 0.0, 0.0, "%.3f");
                if (ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    ImGui::Text("Optimization timeout");
                    ImGui::Text("smaller value gives faster calculations, less precise results");
                    ImGui::Text("e.g.: 0.1 [s] will make processing time aprox equal to scan time");
                    ImGui::Text("0.3 [s] will make processing time aprox equal to 3x scan time");
                    ImGui::EndTooltip();
                }

                if (ImGui::MenuItem("Set real time performance"))
                    params.real_time_threshold_seconds = 0.1;
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("For IMU 200Hz, 20 nodes in optimization window");

                ImGui::Separator();

                ImGui::Text("Debug:");

                ImGui::MenuItem("Full processing messages", nullptr, &full_debug_messages);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Show more messages during processing in console window");
                ImGui::MenuItem("Save calibration validation file", nullptr, &params.save_calibration_validation);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Save calibrated points from first laz file in 'calibrationValidation.asc' file");
                ImGui::SetNextItemWidth(ImGuiNumberWidth);
                ImGui::InputInt("Number of calibration validation points", &params.calibration_validation_points);

                ImGui::Separator();
                if (ImGui::BeginMenu("Ablation study"))
                {
                    ImGui::MenuItem("Use planarity", nullptr, &params.ablation_study_use_planarity);
                    ImGui::MenuItem("Use norm", nullptr, &params.ablation_study_use_norm);
                    ImGui::MenuItem("Use hierarchical RGD (outer RGD turned on)", nullptr, &params.ablation_study_use_hierarchical_rgd);
                    ImGui::MenuItem("Use view point and normal vectors", nullptr, &params.ablation_study_use_view_point_and_normal_vectors);
                    ImGui::MenuItem("Use threshold '1e-6' outer RGD", nullptr, &params.ablation_study_use_threshold_outer_rgd);
                    ImGui::MenuItem("Use anisotropic weighting", nullptr, &params.ablation_study_use_anisotropic_weighting);
                    ImGui::EndMenu();
                }

                ImGui::Separator();

                ImGui::MenuItem("Saving results with index pose", nullptr, &params.save_index_pose);

                ImGui::Separator();

                if (ImGui::MenuItem("Load parameters"))
                {
                    auto input_file_names = mandeye::fd::OpenFileDialog("Load parameters file", mandeye::fd::Toml_filter, ".toml");

                    if (input_file_names.size() > 0)
                    {
                        try
                        {
                            // Use the original TomlIO class for loading parameters in GUI
                            TomlIO toml_io;
                            toml_io.LoadParametersFromTomlFile(input_file_names[0], params);
                            std::cout << "Parameters loaded from: " << input_file_names[0] << std::endl;
                        } catch (const std::exception& e)
                        {
                            std::cerr << "Error loading TOML file: " << e.what() << std::endl;
                        }
                    }
                }

                if (ImGui::MenuItem("Save parameters"))
                {
                    auto output_file_name = mandeye::fd::SaveFileDialog("Save parameters file", mandeye::fd::Toml_filter, ".toml");
                    std::cout << "Parameters file to save: '" << output_file_name << "'" << std::endl;

                    if (!output_file_name.empty())
                    {
                        // Use the original TomlIO class for saving parameters in GUI
                        TomlIO toml_io;
                        bool success = toml_io.SaveParametersToTomlFile(output_file_name, params);
                        if (success)
                            std::cout << "Parameters file generated: " << output_file_name << std::endl;
                        else
                            std::cerr << "Failed to save parameters." << std::endl;
                    }
                }

                ImGui::EndMenu();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Processing parameter list");

            ImGui::SameLine();
            ImGui::Dummy(ImVec2(20, 0));
            ImGui::SameLine();

            if (ImGui::Button("Load & process scanning"))
                openData();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Select folder from where to load data and start processing (Ctrl+O),\nor drop the folder onto the window");

            ImGui::SameLine();
            ImGui::Dummy(ImVec2(20, 0));
            ImGui::SameLine();

            if (ImGui::BeginMenu("View"))
            {
                ImGui::BeginDisabled(!(session.point_clouds_container.point_clouds.size() > 0));
                {
                    int& point_size = app_state.point_size;
                    auto tmp = point_size;
                    ImGui::SetNextItemWidth(ImGuiNumberWidth);
                    ImGui::InputInt("Points size", &point_size);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("keyboard 1-9 keys");
                    if (point_size < 1)
                        point_size = 1;
                    else if (point_size > 10)
                        point_size = 10;

                    if (tmp != point_size)
                        for (auto& point_cloud : session.point_clouds_container.point_clouds)
                            point_cloud.point_size = point_size;

                    ImGui::Separator();
                }
                ImGui::EndDisabled();

                ImGui::MenuItem("Show initial points", nullptr, &show_initial_points);
                ImGui::MenuItem("Show trajectory", nullptr, &show_trajectory);
                ImGui::MenuItem("Show trajectory as axes", nullptr, &show_trajectory_as_axes);
                ImGui::MenuItem("Show prediction vectors", nullptr, &show_prediction_vectors);
                ImGui::MenuItem("Show intermediate trajectory prediction axes", nullptr, &intermediate_trajectory_prediction_axes);

                ImGui::MenuItem("Show compass/ruler", "key C", &app_state.compass_ruler);

                ImGui::MenuItem("Lock Z", "Shift + Z", &app_state.camera.lockZ, !app_state.camera.isOrtho);

                // ImGui::MenuItem("show_covs", nullptr, &show_covs);

                ImGui::Separator();

                ImGui::Text("Colors:");

                ImGui::ColorEdit3("Background", (float*)&params.clear_color, ImGuiColorEditFlags_NoInputs);

                app_state.bg_color = params.clear_color;

                ImGui::Separator();

                ImGui::MenuItem("Settings", nullptr, &is_settings_gui);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Show power user settings window with more parameters");

                ImGui::EndMenu();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Scene view relevant parameters");

            camMenu();

            ImGui::SameLine(
                ImGui::GetWindowWidth() - ImGui::CalcTextSize("Info").x - ImGui::GetStyle().ItemSpacing.x * 2 -
                ImGui::GetStyle().FramePadding.x * 2);

            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4, 2));
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetStyleColorVec4(ImGuiCol_HeaderHovered));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImGui::GetStyleColorVec4(ImGuiCol_Header));
            if (ImGui::SmallButton("Info"))
                app_state.info_gui = !app_state.info_gui;

            ImGui::PopStyleVar(2);
            ImGui::PopStyleColor(3);

            ImGui::EndMainMenuBar();
        }
    }

    // m_gizmo is column-major, like Eigen's default storage.
    if (initial_transformation_gizmo)
    {
        manipulateGizmo(m_gizmo);
        params.m_g.matrix() = Eigen::Map<const Eigen::Matrix4f>(m_gizmo).cast<double>();
    }

    if (gizmo_stretch_interval)
    {
        manipulateGizmo(m_gizmo);
        stretch_gizmo_m.matrix() = Eigen::Map<const Eigen::Matrix4f>(m_gizmo).cast<double>();
    }

    if (is_settings_gui)
    {
        lastPar = 0;

        settings_gui();
    }

    if (loRunning)
        progress_window();

    raylib_widgets::showEulerCenterOfRotationWindow(cor_gui, app_state.camera, xText, yText, zText);

    if (app_state.info_gui)
    {
        infoLines[infoLines.size() - 2] = "It saves session file in " + working_directory + "\\lio_result_*";
        raylib_widgets::ShowInfoWindow(app_state.info_gui, infoLines, appShortcuts, HDMAPPING_VERSION_STRING, __DATE__);
    }

    // 3D drawing is done -- switch to the 2D screen-space projection the compass and ImGui need.
    raylib_widgets::end3DMatrixStack(io.DisplaySize.x, io.DisplaySize.y);

    if (app_state.compass_ruler)
        drawMiniCompassWithRuler();

    rlImGuiEnd();
}

//! Starts processing a folder dropped onto the window. A dropped file stands for the folder that contains it.
void loadDroppedPaths(const std::vector<std::string>& paths)
{
    if (paths.empty())
        return;

    if (loRunning || step_1_done)
    {
        std::string message_info = "Data is already loaded. Restart the program to process another folder.";
        std::cout << message_info << std::endl;
        [[maybe_unused]] pfd::message message("Information", message_info.c_str(), pfd::choice::ok, pfd::icon::info);
        message.result();
        return;
    }

    fs::path folder(paths.front());
    if (!fs::is_directory(folder))
        folder = folder.parent_path();

    std::cout << "Dropped folder: '" << folder.string() << "'" << std::endl;
    openData(folder.string());
}

void on_exit()
{
    // remove cache
    std::cout << "remove cache: '" << params.working_directory_cache << "' START" << std::endl;
    std::filesystem::remove_all(params.working_directory_cache);
    std::cout << "remove cache: '" << params.working_directory_cache << "' FINISHED" << std::endl;
}

//! Handles a raylib mouse button transition (`button` is a raylib MouseButton).
void mouse(int button, bool down, int x, int y)
{
    ImGuiIO& io = ImGui::GetIO();

    if (io.WantCaptureMouse)
        return;

    if ((button == MOUSE_BUTTON_MIDDLE || button == MOUSE_BUTTON_RIGHT) && down && io.KeyCtrl)
        setNewRotationCenter(x, y);

    if (down)
        app_state.mouse_buttons |= (button == MOUSE_BUTTON_LEFT) ? 1 : (button == MOUSE_BUTTON_MIDDLE) ? 2 : 4;
    else
        app_state.mouse_buttons = 0;

    app_state.mouse_old_x = x;
    app_state.mouse_old_y = y;
}

bool initGL()
{
    // FLAG_WINDOW_HIGHDPI scales the ImGui menu bar incorrectly on Windows, so it is only used elsewhere.
    unsigned int flags = FLAG_WINDOW_RESIZABLE;
#ifndef _WIN32
    flags |= FLAG_WINDOW_HIGHDPI;
#endif

    SetConfigFlags(flags);
    InitWindow(static_cast<int>(window_width), static_cast<int>(window_height), winTitle.c_str());
    if (!IsWindowReady())
        return false;

    // Startup info (GL version, GPU) is still logged above; later per-resource INFO lines are noise.
    SetTraceLogLevel(LOG_WARNING);

    SetExitKey(KEY_NULL); // Esc must not close the window
    SetTargetFPS(60);
    raylib_widgets::fitWindowToScreen(/*marginW=*/100, /*marginH=*/100, /*centerVertically=*/true);

    rlImGuiSetup(true);
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard | ImGuiConfigFlags_NavEnableGamepad | ImGuiConfigFlags_DockingEnable;
    io.ConfigDockingWithShift = true;

    scan_renderer.init();

    return true;
}

void runGui()
{
    if (!initGL())
    {
        spdlog::error("Could not create the application window");
        return;
    }

#ifdef _WIN32
    InitTaskbarProgress(GetWindowHandle());
#endif

    while (!WindowShouldClose())
    {
        const int mx = GetMouseX();
        const int my = GetMouseY();

        for (int button : { MOUSE_BUTTON_LEFT, MOUSE_BUTTON_RIGHT, MOUSE_BUTTON_MIDDLE })
        {
            if (IsMouseButtonPressed(button))
                mouse(button, true, mx, my);
            if (IsMouseButtonReleased(button))
                mouse(button, false, mx, my);
        }

        motion(mx, my);

        const float wheelMove = GetMouseWheelMove();
        if (wheelMove != 0.0f)
            wheel(wheelMove);

        // raylib's GLFW backend reports OS drag & drop the same way on Windows, Linux and macOS.
        if (IsFileDropped())
        {
            FilePathList dropped_files = LoadDroppedFiles();
            std::vector<std::string> paths(dropped_files.paths, dropped_files.paths + dropped_files.count);
            UnloadDroppedFiles(dropped_files);
            loadDroppedPaths(paths);
        }

        BeginDrawing();
        display();
        EndDrawing();
    }

    on_exit();

    unloadScenePoints();
    scan_renderer.shutdown();
    rlImGuiShutdown();
    CloseWindow();
}

int main(int argc, char* argv[])
{
    spdlog::cfg::load_env_levels();
    spdlog::flush_on(spdlog::level::warn);
    set_lidar_odometry_default_params(params);

    try
    {
        if (checkClHelp(argc, argv))
        {
            std::cout << winTitle << "\n\n"
                      << "USAGE:\n"
                      << std::filesystem::path(argv[0]).stem().string() << " <input_folder> <parameter_file> <output_folder> /?\n"
                      << std::filesystem::path(argv[0]).stem().string() << " <input_folder>\n"
                      << std::filesystem::path(argv[0]).stem().string() << " --dump-default-params <file.toml>\n\n"
                      << "where\n"
                      << "   <input_folder>       Path where scan files are located (*.csv, *.laz, *.sn)\n"
                      << "   <parameter_file>     Path to TOML parameter file (*.toml)\n"
                      << "   <output_folder>      Path where processed session should be stored\n"
                      << "                        (with <input_folder> alone, default parameters are used and the\n"
                      << "                        session is stored in the next free <input_folder>/lio_result_N,\n"
                      << "                        as the GUI does)\n"
                      << "   --dump-default-params <file.toml>\n"
                      << "                        Write the default parameters to <file.toml> and exit\n"
                      << "   -h, /h, --help, /?   Show this help and exit\n\n";

            return 0;
        }

        for (int i = 1; i < argc; ++i)
        {
            if (std::string(argv[i]) == "--dump-default-params")
            {
                if (i + 1 >= argc)
                {
                    std::cerr << "--dump-default-params requires an output file (*.toml)" << std::endl;
                    return 1;
                }
                const fs::path out_file(argv[i + 1]);
                // Value-initialised, as the static `params` is: the NDT grid members carry no
                // initialisers, and a default-initialised local would dump stack garbage.
                LidarOdometryParams default_params{};
                set_lidar_odometry_default_params(default_params);
                TomlIO toml_io;
                toml_io.SaveParametersToTomlFile(out_file.string(), default_params);
                std::error_code ec;
                if (!fs::exists(out_file, ec) || fs::file_size(out_file, ec) == 0 || ec)
                {
                    std::cerr << "Could not write default parameters to: " << out_file.string() << std::endl;
                    return 1;
                }
                std::cout << "Default parameters saved to: " << out_file.string() << std::endl;
                return 0;
            }
        }

        if (argc == 2) // running from command line
        {
            auto path = fs::path(argv[1]);
            if (is_directory(path))
            {
                std::string working_directory;
                std::vector<WorkerData> worker_data;

                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();

                std::atomic<bool> loPause{ false };
                step1(path.string(), params, pointsPerFile, imu_data, working_directory, trajectory, worker_data, loPause);

                step2(worker_data, params, loPause);

                end = std::chrono::system_clock::now();
                std::chrono::duration<double> elapsed_seconds = end - start;
                std::time_t end_time = std::chrono::system_clock::to_time_t(end);
                std::cout << "calculations finished computation at " << std::ctime(&end_time)
                          << "Elapsed time: " << formatTime(elapsed_seconds.count()).c_str() << "s\n";

                // Only the input folder was given: store the session where the GUI would,
                // in the next free lio_result_N under the working directory.
                if (working_directory.empty())
                {
                    std::cerr << "No Mandeye data could be loaded from: '" << path.string() << "'" << std::endl;
                    return 1;
                }
                const fs::path result_dir = get_next_result_path(working_directory);
                save_results(false, elapsed_seconds.count(), working_directory, worker_data, params, result_dir);
                std::cout << "Results saved to folder: '" << result_dir.string() << "'" << std::endl;
            }
        }
        else if (argc == 4) // runnning from command line with custom params
        {
            // Load parameters from file using original TomlIO class
            TomlIO toml_io;
            toml_io.LoadParametersFromTomlFile(argv[2], params);
            std::cout << "Parameters loaded OK from: " << argv[2] << std::endl;

            std::string working_directory;
            std::vector<WorkerData> worker_data;

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();

            std::atomic<bool> loPause{ false };
            step1(argv[1], params, pointsPerFile, imu_data, working_directory, trajectory, worker_data, loPause);

            step2(worker_data, params, loPause);

            end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;
            std::time_t end_time = std::chrono::system_clock::to_time_t(end);
            std::cout << "calculations finished computation at " << std::ctime(&end_time)
                      << "Elapsed time: " << formatTime(elapsed_seconds.count()).c_str() << "s\n";

            save_results(false, elapsed_seconds.count(), working_directory, worker_data, params, argv[3]);
        }
        else // full GUI mode
        {
            std::cout << argv[0] << " input_folder parameters(*.toml) output_folder" << std::endl;

            runGui();
        }
    } catch (const std::bad_alloc& e)
    {
        std::cerr << "System is out of memory : " << e.what() << std::endl;
        mandeye::fd::OutOfMemMessage();
    } catch (const std::exception& e)
    {
        std::cout << e.what();
    } catch (...)
    {
        std::cerr << "Unknown fatal error occurred." << std::endl;
    }

    return 0;
}