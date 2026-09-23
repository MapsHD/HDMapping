#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>
#include <memory>

// Step 3 used to be built on GLUT + legacy immediate-mode OpenGL via
// core/src/utils.cpp (the GLUT build is kept as
// apps/multi_session_registration_legacy). Like step 2
// (apps/multi_view_tls_registration), it now runs on raylib: the camera,
// input and picking helpers it used to get from <Core/utils.hpp> are
// re-implemented below on top of raylib_widgets::OrbitCamera, and point
// clouds are drawn with Core/raylib_render.hpp's ScanRenderer (one per
// session) instead of core's legacy-GL PointCloud::render().
#include "raylib.h"
#include "raymath.h"
#include "rlImGui.h"
#include "rlgl.h"

#include <imgui.h>
#include <imgui_internal.h>

#include <ImGuizmo.h>

#include <nlohmann/json.hpp>

#include <spdlog/spdlog.h>

#include <Eigen/Eigen>

#include <Core/export_laz.h>
#include <Core/icp.h>
#include <Core/ndt.h>
#include <Core/observation_picking.h>
#include <Core/pair_wise_iterative_closest_point.h>
#include <Core/pfd_wrapper.hpp>
#include <Core/raylib_render.hpp>
#include <Core/registration_plane_feature.h>
#include <Core/session.h>
#include <Core/structures.h>
#include <Core/transformations.h>

#ifdef _WIN32
// Same windows.h/raylib clash as step 2 (see multi_view_tls_registration_gui.cpp):
// rename windows.h's CloseWindow/ShowCursor so raylib's stay callable.
#define CloseWindow CloseWindow_win32
#define ShowCursor ShowCursor_win32
#endif
#include <portable-file-dialogs.h>
#ifdef _WIN32
#undef CloseWindow
#undef ShowCursor
#endif

#include <HDMapping/Version.hpp>

#ifdef _WIN32
#include "resource.h"
#endif

#include <RaylibWidgets/AppShell.h>
#include <RaylibWidgets/CenterOfRotationWindow.h>
#include <RaylibWidgets/CompassRuler.h>
#include <RaylibWidgets/OrbitCamera.h>
#include <RaylibWidgets/PointPicking.h>
#include <RaylibWidgets/RayPlaneD.h>
#include <RaylibWidgets/ShortcutsTable.h>
#include <RaylibWidgets/WindowFit.h>

#ifdef _WIN32
// windows.h #defines DrawText as DrawTextA; restore raylib's DrawText.
#undef DrawText
#endif

#include "multi_session_factor_graph.h"

using raylib_widgets::ShortcutEntry;
using raylib_widgets::ShowMainDockSpace;

const float DEG_TO_RAD = M_PI / 180.0f;
const float RAD_TO_DEG = 180.0f / M_PI;

constexpr float ImGuiNumberWidth = 120.0f;
constexpr const char* xText = "Longitudinal (forward/backward)";
constexpr const char* yText = "Lateral (left/right)";
constexpr const char* zText = "Vertical (up/down)";

const uint32_t window_width = 1600;
const uint32_t window_height = 900;

// GLUT/mouse-button codes kept so mouse() keeps its GLUT-callback shape (as in step 2).
constexpr int GLUT_LEFT_BUTTON = 0;
constexpr int GLUT_MIDDLE_BUTTON = 1;
constexpr int GLUT_RIGHT_BUTTON = 2;
constexpr int GLUT_DOWN = 0;
constexpr int GLUT_UP = 1;

// Point downsampling default and camera-Reset value, as in the GLUT step 3 (utils.cpp).
constexpr int kDefaultDecimate = 1000;

std::string winTitle = std::string("Step 3 (Multi session registration) ") + HDMAPPING_VERSION_STRING;

std::vector<std::string> infoLines = {
    "This program is third/final step in MANDEYE process",
    "",
    "First step: create project by adding sessions (result of 'multi_view_tls_registration_step_2' program)",
    "Last step: save project",
    "To produce map use 'multi_view_tls_registration_step_2' export functionality"
};

// App specific shortcuts (Type and Shortcut are just for easy reference)
static const std::vector<ShortcutEntry> appShortcuts = { { "Normal keys", "A", "" },
                                                         { "", "Ctrl+A", "Add session(s)" },
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
                                                         { "", "Ctrl+L", "Load sessions" },
                                                         { "", "M", "" },
                                                         { "", "Ctrl+M", "" },
                                                         { "", "N", "" },
                                                         { "", "Ctrl+N", "" },
                                                         { "", "O", "" },
                                                         { "", "Ctrl+O", "Open project" },
                                                         { "", "P", "" },
                                                         { "", "Ctrl+P", "" },
                                                         { "", "Q", "" },
                                                         { "", "Ctrl+Q", "" },
                                                         { "", "R", "" },
                                                         { "", "Ctrl+R", "Remove session(s)" },
                                                         { "", "Shift+R", "" },
                                                         { "", "S", "" },
                                                         { "", "Ctrl+S", "Save project" },
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

float m_gizmo[] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

bool is_decimate = true;
double bucket_x = 0.1;
double bucket_y = 0.1;
double bucket_z = 0.1;
bool calculate_offset = false;
ObservationPicking observation_picking;
int index_loop_closure_source = -1;
int index_loop_closure_target = -1;
int first_session_index = -1;
int second_session_index = -1;
double search_radius = 0.3;
bool loaded_sessions = false;
bool optimized = false;
bool gizmo_all_sessions = false;
bool is_ndt_gui = false;
bool is_loop_closure_gui = false;
bool remove_gui = false;
NDT ndt;

bool update_rotation_center = false;

bool is_settings_gui = true;

int number_visible_sessions = 0;
int index_gt = -1;
int old_index_gt = -1;
int index_gizmo = -1;
int old_index_gizmo = -1;

double time_stamp_offset = 0.0;

struct ProjectSettings
{
    std::vector<std::string> session_file_names;
};

std::vector<Edge> edges;
int index_active_edge = -1;
bool manipulate_active_edge = false;
bool edge_gizmo = false;

ProjectSettings project_settings;
std::vector<Session> sessions;

int viewer_reduce_rendered_trajectory = 1;
namespace fs = std::filesystem;

int num_edge_extended_before = 0;
int num_edge_extended_after = 0;

TaitBryanPose motion_model_weights = { 0.01, 0.01, 0.01, 0.1, 0.1, 0.1 };
///////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////

// Camera/view state that used to be <Core/utils.hpp> globals.
struct AppStateBase
{
    int viewer_decimate_point_cloud = kDefaultDecimate;

    int mouse_old_x = 0, mouse_old_y = 0;
    int mouse_buttons = 0;
    bool show_axes = true;
    ImVec4 bg_color = ImVec4(0.65f, 0.65f, 0.65f, 1.00f);
    // Single point size for all sessions. The GLUT app had two (View menu/1-9 keys and the
    // loop closure window's gui_point_size), and the latter silently overrode the former every frame.
    int point_size = 2;

    bool info_gui = false;
    bool compass_ruler = true;

    // Rebuilt from `camera` every frame; used by the compass and the perspective modelview.
    Eigen::Affine3f viewLocal = Eigen::Affine3f::Identity();

    raylib_widgets::OrbitCamera camera;
};

inline AppStateBase app_state;

// Edge-triggered request to open the Center of rotation dialog (Shift+R).
bool cor_gui = false;

bool scroll_hint_enabled = true;
bool scroll_hint_active = false;
int scroll_hint_count = 0;
float scroll_hint_accu = 0.0f;
double scroll_hint_lastT = 0.0;

// One GPU renderer per session, parallel to `sessions` (ScanRenderer is indexed by a single
// std::vector<PointCloud>). Rebuilt whenever the session list changes -- see syncSessionRenderers().
std::vector<std::unique_ptr<ScanRenderer>> session_renderers;

// Point coloring, using the same ScanRenderer shader modes as step 2's color schemes. Flat (each
// scan's render_color, i.e. the session color) is the default, since step 3 compares sessions.
ScanColorMode points_color_mode = ScanColorMode::Flat;

// Bounds of all loaded sessions, for the height and distance gradients; updated with session_renderers.
PointClouds::PointCloudDimensions scene_dims{ 0, 0, 0, 0, 0, 1, 1, 1, 1 };

// This frame's 3D model-view-projection, captured before the matrix stack is switched to 2D,
// so the 2D label pass can project world points to the screen.
Matrix frame_mvp_3d{};

void display();
void mouse(int glut_button, int state, int x, int y);

///////////////////////////////////////////////////////////////////////////////////

// Camera/input/picking helpers, same as step 2 (apps/multi_view_tls_registration).

std::string truncPath(const std::string& fullPath)
{
    namespace fspath = std::filesystem;
    fspath::path path(fullPath);

    auto parent1 = path.parent_path().filename().string();
    auto parent2 = path.parent_path().parent_path().filename().string(); // second to last folder
    auto filename = path.filename().string();

    return "..\\" + parent2 + "\\" + parent1 + "\\" + filename;
}

void wheel(int button, int dir, int x, int y)
{
    ImGuiIO& io = ImGui::GetIO();
    io.MouseWheel += dir; // or direction * 1.0f depending on your setup

    if (!ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow))
    {
        // GetMouseWheelMove(), not `dir`: dir is already quantized to +-1 by
        // main()'s caller (see its comment), which discards a trackpad's
        // fractional per-frame scroll magnitude -- reading it again here
        // (stable within the same frame, since raylib only updates it once
        // per PollInputEvents()) lets zoom() scale the step by how much was
        // actually scrolled instead of always taking a full step.
        app_state.camera.zoom(GetMouseWheelMove(), io.KeyShift);

        if (scroll_hint_enabled)
        {
            if (!scroll_hint_active)
            {
                scroll_hint_accu += fabs(dir);

                if (scroll_hint_accu > 30.0f) // tweak threshold
                {
                    scroll_hint_accu = 0.0f;
                    scroll_hint_active = true;
                    scroll_hint_count++;
                }
            }

            if (scroll_hint_active)
                scroll_hint_lastT = ImGui::GetTime();

            // Reset and disable hint if Shift is pressed while scrolling
            if (io.KeyShift || scroll_hint_count > 3)
            {
                scroll_hint_active = false;
                scroll_hint_enabled = false;
            }
        }
    }
}

void motion(int x, int y)
{
    ImGuiIO& io = ImGui::GetIO();
    io.MousePos = ImVec2((float)x, (float)y);

    if (!io.WantCaptureMouse)
    {
        float dx, dy;
        dx = (float)(x - app_state.mouse_old_x);
        dy = (float)(y - app_state.mouse_old_y);

        // Ctrl/Shift held: reserved for the discrete click actions and the
        // keyboard shortcuts in view_kbd_shortcuts() -- mouse() sets
        // mouse_buttons for *every* button-down, including a Ctrl/Shift+
        // click used to pick a new rotation center (which starts a camera
        // transition -- see getClosestTrajectoryPoint()/
        // setNewRotationCenter()/the Center of rotation dialog). Without
        // this guard, any stray sub-pixel movement on the same click
        // (trackpads are far more prone to this than a physical mouse
        // button) got read as an ordinary orbit/pan drag and immediately
        // broke that transition via dragOrbit()/dragPanPerspective()'s
        // breakEulerTransition() call.
        if (!io.KeyCtrl && !io.KeyShift)
        {
            if (app_state.mouse_buttons & 1) // left button
            {
                app_state.camera.dragOrbit(dx, dy);
            }

            if (app_state.mouse_buttons & 4) // right button
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
}

void showAxes()
{
    if (app_state.show_axes || ImGui::GetIO().KeyCtrl) // rotation center axes
    {
        const auto& rc = app_state.camera.euler.rotationCenter;
        rlBegin(RL_LINES);
        rlColor3f(1.f, 1.f, 1.f);
        rlVertex3f(rc.x, rc.y, rc.z);
        rlVertex3f(rc.x + 1.f, rc.y, rc.z);
        rlVertex3f(rc.x, rc.y, rc.z);
        rlVertex3f(rc.x - 1.f, rc.y, rc.z);
        rlVertex3f(rc.x, rc.y, rc.z);
        rlVertex3f(rc.x, rc.y - 1.f, rc.z);
        rlVertex3f(rc.x, rc.y, rc.z);
        rlVertex3f(rc.x, rc.y + 1.f, rc.z);
        rlVertex3f(rc.x, rc.y, rc.z);
        rlVertex3f(rc.x, rc.y, rc.z - 1.f);
        rlVertex3f(rc.x, rc.y, rc.z);
        rlVertex3f(rc.x, rc.y, rc.z + 1.f);
        rlEnd();
    }

    if (app_state.show_axes || ImGui::GetIO().KeyCtrl) // origin axes
    {
        rlBegin(RL_LINES);
        rlColor3f(1.0f, 0.0f, 0.0f);
        rlVertex3f(0.0f, 0.0f, 0.0f);
        rlVertex3f(100, 0.0f, 0.0f);

        rlColor3f(0.0f, 1.0f, 0.0f);
        rlVertex3f(0.0f, 0.0f, 0.0f);
        rlVertex3f(0.0f, 100, 0.0f);

        rlColor3f(0.0f, 0.0f, 1.0f);
        rlVertex3f(0.0f, 0.0f, 0.0f);
        rlVertex3f(0.0f, 0.0f, 100);
        rlEnd();
    }
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
        {
            app_state.camera.setEulerPreset(OrbitCamera::EulerPreset::Reset);
            app_state.viewer_decimate_point_cloud = kDefaultDecimate;
        }

        ImGui::EndMenu();
    }
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::Text("Change camera view to fixed positions");
        ImGui::Separator();
        ImGui::Text("Metrics:");
        if (ImGui::BeginTable("Metrics", 4))
        {
            ImGui::TableSetupColumn("Coord");
            ImGui::TableSetupColumn("rotate");
            ImGui::TableSetupColumn("translate");
            ImGui::TableSetupColumn("rot center");
            ImGui::TableHeadersRow();

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);

            std::string text = "X";
            float centered = ImGui::GetColumnWidth() - ImGui::CalcTextSize(text.c_str()).x;
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + centered * 0.5f);
            ImGui::Text("X");

            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.3f", app_state.camera.euler.rotateX);
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%.3f", app_state.camera.euler.translate.x);
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%.3f", app_state.camera.euler.rotationCenter.x);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + centered * 0.5f);
            ImGui::Text("Y");

            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.3f", app_state.camera.euler.rotateY);
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%.3f", app_state.camera.euler.translate.y);
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%.3f", app_state.camera.euler.rotationCenter.y);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + centered * 0.5f);
            ImGui::Text("Z");

            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%.3f", app_state.camera.euler.translate.z);
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%.3f", app_state.camera.euler.rotationCenter.y);

            ImGui::EndTable();
        }
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

    if (io.WantCaptureKeyboard)
        return;

    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_RightArrow, true))
    {
        app_state.camera.euler.translate.x += 0.5f * app_state.camera.eulerMouseSensitivity;
        app_state.camera.breakEulerTransition();
    }
    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_LeftArrow, true))
    {
        app_state.camera.euler.translate.x -= 0.5f * app_state.camera.eulerMouseSensitivity;
        app_state.camera.breakEulerTransition();
    }

    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_UpArrow, true))
    {
        app_state.camera.euler.translate.y += 0.5f * app_state.camera.eulerMouseSensitivity;
        app_state.camera.breakEulerTransition();
    }
    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_DownArrow, true))
    {
        app_state.camera.euler.translate.y -= 0.5f * app_state.camera.eulerMouseSensitivity;
        app_state.camera.breakEulerTransition();
    }

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_RightArrow, true))
    {
        app_state.camera.euler.rotateY -= 0.6f;
        app_state.camera.breakEulerTransition();
    }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_LeftArrow, true))
    {
        app_state.camera.euler.rotateY += 0.6f;
        app_state.camera.breakEulerTransition();
    }

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_UpArrow, true))
    {
        app_state.camera.euler.rotateX -= 0.6f;
        app_state.camera.breakEulerTransition();
    }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_DownArrow, true))
    {
        app_state.camera.euler.rotateX += 0.6f;
        app_state.camera.breakEulerTransition();
    }

    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_R, false))
        cor_gui = true;

    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_Z, false) && !app_state.camera.isOrtho)
        app_state.camera.lockZ = !app_state.camera.lockZ;

    if (io.KeyCtrl || io.KeyAlt || io.KeyShift)
        return;

    if (ImGui::IsKeyPressed(ImGuiKey_B))
        app_state.camera.setEulerPreset(OrbitCamera::EulerPreset::Back);
    if (ImGui::IsKeyPressed(ImGuiKey_F))
        app_state.camera.setEulerPreset(OrbitCamera::EulerPreset::Front);
    if (ImGui::IsKeyPressed(ImGuiKey_I))
        app_state.camera.setEulerPreset(OrbitCamera::EulerPreset::Iso);
    if (ImGui::IsKeyPressed(ImGuiKey_L))
        app_state.camera.setEulerPreset(OrbitCamera::EulerPreset::Left);
    if (ImGui::IsKeyPressed(ImGuiKey_R))
        app_state.camera.setEulerPreset(OrbitCamera::EulerPreset::Right);
    if (ImGui::IsKeyPressed(ImGuiKey_T))
        app_state.camera.setEulerPreset(OrbitCamera::EulerPreset::Top);
    if (ImGui::IsKeyPressed(ImGuiKey_U))
        app_state.camera.setEulerPreset(OrbitCamera::EulerPreset::Bottom);
    if (ImGui::IsKeyPressed(ImGuiKey_Z))
    {
        app_state.camera.setEulerPreset(OrbitCamera::EulerPreset::Reset);
        app_state.viewer_decimate_point_cloud = kDefaultDecimate;
    }

    if (ImGui::IsKeyPressed(ImGuiKey_C, false))
        app_state.compass_ruler = !app_state.compass_ruler;
    if (ImGui::IsKeyPressed(ImGuiKey_O, false))
        app_state.camera.isOrtho = !app_state.camera.isOrtho;
    if (ImGui::IsKeyPressed(ImGuiKey_X, false))
        app_state.show_axes = !app_state.show_axes;

    if (ImGui::IsKeyPressed(ImGuiKey_1))
        app_state.point_size = 1;
    if (ImGui::IsKeyPressed(ImGuiKey_2))
        app_state.point_size = 2;
    if (ImGui::IsKeyPressed(ImGuiKey_3))
        app_state.point_size = 3;
    if (ImGui::IsKeyPressed(ImGuiKey_4))
        app_state.point_size = 4;
    if (ImGui::IsKeyPressed(ImGuiKey_5))
        app_state.point_size = 5;
    if (ImGui::IsKeyPressed(ImGuiKey_6))
        app_state.point_size = 6;
    if (ImGui::IsKeyPressed(ImGuiKey_7))
        app_state.point_size = 7;
    if (ImGui::IsKeyPressed(ImGuiKey_8))
        app_state.point_size = 8;
    if (ImGui::IsKeyPressed(ImGuiKey_9))
        app_state.point_size = 9;
}

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

Eigen::Vector3d rayIntersection(const LaserBeam& laser_beam, const RegistrationPlaneFeature::Plane& plane)
{
    Eigen::Vector3d hit = laser_beam.position;
    raylib_widgets::intersectPlane(laser_beam.position, laser_beam.direction, plane.a, plane.b, plane.c, plane.d, hit);
    return hit;
}

LaserBeam GetLaserBeam(int x, int y)
{
    Ray ray = app_state.camera.eulerScreenRay(x, y, GetScreenWidth(), GetScreenHeight());

    LaserBeam laser_beam;
    laser_beam.position = Eigen::Vector3d(ray.position.x, ray.position.y, ray.position.z);
    laser_beam.direction = Eigen::Vector3d(ray.direction.x, ray.direction.y, ray.direction.z);

    return laser_beam;
}

double distance_point_to_line(const Eigen::Vector3d& point, const LaserBeam& line)
{
    return raylib_widgets::distancePointToLine(point, line.position, line.direction);
}

void setNewRotationCenter(int x, int y)
{
    const auto laser_beam = GetLaserBeam(x, y);

    RegistrationPlaneFeature::Plane pl;

    pl.a = 0;
    pl.b = 0;
    pl.c = 1;
    pl.d = 0;
    Eigen::Vector3f center_eigen = rayIntersection(laser_beam, pl).cast<float>();

    spdlog::info("Setting new rotation center to: {}, {}, {}", center_eigen.x(), center_eigen.y(), center_eigen.z());

    app_state.camera.moveEulerRotationCenterTo(Vector3{ center_eigen.x(), center_eigen.y(), center_eigen.z() });
}

bool checkClHelp(int argc, char** argv)
{
    for (int i = 1; i < argc; ++i)
    {
        std::string arg(argv[i]);

        if (arg == "-h" || arg == "/h" || arg == "--help" || arg == "/?")
        {
            return true;
        }
    }
    return false;
}

// Was utils.cpp's getClosestTrajectoriesPoint(): picks the trajectory node nearest to the mouse ray
// and moves the rotation center there. Ctrl picks the loop-closure source (and time_stamp_offset),
// Shift the target; with more than two visible sessions it searches all visible ones.
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
    double& time_stamp_offset)
{
    const auto laser_beam = GetLaserBeam(x, y);
    double min_distance = std::numeric_limits<double>::max();
    Vector3 center = app_state.camera.eulerGoal.rotationCenter;

    auto visit = [&](int s, bool update_source_target)
    {
        if (s < 0 || s >= static_cast<int>(sessions.size()))
            return;
        const auto& pcs = sessions[s].point_clouds_container.point_clouds;
        for (size_t i = 0; i < pcs.size(); i++)
        {
            for (size_t j = 0; j < pcs[i].local_trajectory.size(); j++)
            {
                Eigen::Vector3d vp = pcs[i].m_pose * pcs[i].local_trajectory[j].m_pose.translation();
                double dist = distance_point_to_line(vp, laser_beam);
                if (dist >= min_distance)
                    continue;
                min_distance = dist;

                if (!update_source_target)
                {
                    center = Vector3{ static_cast<float>(vp.x()), static_cast<float>(vp.y()), static_cast<float>(vp.z()) };
                    time_stamp_offset = pcs[i].local_trajectory[j].timestamps.first;
                }
                else if (!KeyShift) // Ctrl
                {
                    center = Vector3{ static_cast<float>(vp.x()), static_cast<float>(vp.y()), static_cast<float>(vp.z()) };
                    index_loop_closure_source = static_cast<int>(i);
                    time_stamp_offset = pcs[i].local_trajectory[j].timestamps.first;
                }
                else // Shift
                {
                    index_loop_closure_target = static_cast<int>(i);
                }
            }
        }
    };

    if (number_visible_sessions == 1)
        visit(first_session_index, true);
    else if (number_visible_sessions == 2)
        visit(KeyShift ? second_session_index : first_session_index, true);
    else
        for (size_t s = 0; s < sessions.size(); s++)
            if (sessions[s].visible)
                visit(static_cast<int>(s), false);

    app_state.camera.moveEulerRotationCenterTo(center);
}

// Keeps session_renderers parallel to `sessions` and each renderer's GPU buffers in sync with its
// scans' poses. A size mismatch means the session list changed (load/remove), so everything is
// re-uploaded; otherwise only scans whose m_pose changed are rebuilt.
void syncSessionRenderers()
{
    if (session_renderers.size() != sessions.size())
    {
        session_renderers.clear();
        bool first = true;
        for (const auto& s : sessions)
        {
            auto renderer = std::make_unique<ScanRenderer>();
            renderer->init();
            renderer->rebuildAll(s.point_clouds_container.point_clouds);
            session_renderers.push_back(std::move(renderer));

            if (s.point_clouds_container.point_clouds.empty())
                continue;
            const auto d = s.point_clouds_container.compute_point_cloud_dimension();
            if (first)
                scene_dims = d;
            scene_dims.x_min = std::min(scene_dims.x_min, d.x_min);
            scene_dims.x_max = std::max(scene_dims.x_max, d.x_max);
            scene_dims.y_min = std::min(scene_dims.y_min, d.y_min);
            scene_dims.y_max = std::max(scene_dims.y_max, d.y_max);
            scene_dims.z_min = std::min(scene_dims.z_min, d.z_min);
            scene_dims.z_max = std::max(scene_dims.z_max, d.z_max);
            first = false;
        }
        scene_dims.length = scene_dims.x_max - scene_dims.x_min;
        scene_dims.width = scene_dims.y_max - scene_dims.y_min;
        scene_dims.height = scene_dims.z_max - scene_dims.z_min;
        return;
    }

    for (size_t i = 0; i < sessions.size(); i++)
        session_renderers[i]->syncPoses(sessions[i].point_clouds_container.point_clouds);
}

// Draws the given session's visible scans (points + trajectories), colored by points_color_mode.
// `only` restricts drawing to scans whose index it accepts (used by loop closure mode).
template<typename Pred>
void drawSession(size_t session_index, Pred only)
{
    if (session_index >= sessions.size() || session_index >= session_renderers.size())
        return;

    auto& pcc = sessions[session_index].point_clouds_container;
    auto& pcs = pcc.point_clouds;

    std::vector<bool> was_visible(pcs.size());
    for (size_t i = 0; i < pcs.size(); i++)
    {
        was_visible[i] = pcs[i].visible;
        pcs[i].visible = pcs[i].visible && only(static_cast<int>(i));
    }

    const auto& rc = app_state.camera.euler.rotationCenter;
    session_renderers[session_index]->draw(
        pcs,
        static_cast<float>(app_state.point_size),
        points_color_mode,
        static_cast<float>(scene_dims.z_min),
        static_cast<float>(scene_dims.z_max),
        Eigen::Vector3d(rc.x, rc.y, rc.z),
        static_cast<float>(std::max({ scene_dims.length, scene_dims.width, scene_dims.height, 1.0 })),
        app_state.viewer_decimate_point_cloud,
        pcc.xz_intersection,
        pcc.yz_intersection,
        pcc.xy_intersection,
        static_cast<float>(pcc.intersection_width),
        pcc.show_with_initial_pose);
    session_renderers[session_index]->drawTrajectories(
        pcs,
        viewer_reduce_rendered_trajectory,
        pcc.show_imu_to_lio_diff,
        pcc.xz_intersection,
        pcc.yz_intersection,
        pcc.xy_intersection,
        pcc.show_with_initial_pose,
        pcc.imu_to_lio_diff_scale);

    for (size_t i = 0; i < pcs.size(); i++)
        pcs[i].visible = was_visible[i];
}

void drawSession(size_t session_index)
{
    drawSession(
        session_index,
        [](int)
        {
            return true;
        });
}

// Was PointCloud::render(pose, ...): previews scan `index` of a session at `pose` (points only, from
// the cached GPU buffer), plus its trajectory at its real m_pose, as the GLUT version drew it.
void drawScanAtPose(size_t session_index, int index, const Eigen::Affine3d& pose, const float color[3])
{
    if (session_index >= sessions.size() || session_index >= session_renderers.size())
        return;
    const auto& pcs = sessions[session_index].point_clouds_container.point_clouds;
    if (index < 0 || index >= static_cast<int>(pcs.size()) || !pcs[index].visible)
        return;
    const auto& pc = pcs[index];

    Color c = ColorFromNormalized(Vector4{ color[0], color[1], color[2], 1.f });
    session_renderers[session_index]->drawCachedWithTransform(
        static_cast<size_t>(index), pose * pc.m_pose.inverse(), c, static_cast<float>(app_state.point_size), false);

    const int stride = std::max(1, viewer_reduce_rendered_trajectory);
    rlBegin(RL_LINES);
    rlColor3f(color[0], color[1], color[2]);
    for (size_t i = stride; i < pc.local_trajectory.size(); i += stride)
    {
        Eigen::Vector3d a = (pc.m_pose * pc.local_trajectory[i - stride].m_pose).translation();
        Eigen::Vector3d b = (pc.m_pose * pc.local_trajectory[i].m_pose).translation();
        rlVertex3f(static_cast<float>(a.x()), static_cast<float>(a.y()), static_cast<float>(a.z()));
        rlVertex3f(static_cast<float>(b.x()), static_cast<float>(b.y()), static_cast<float>(b.z()));
    }
    rlEnd();
}

void vertex(const Eigen::Vector3d& v)
{
    rlVertex3f(static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z()));
}

// Polyline through every scan pose of a session, colored per scan (was a GL_LINE_STRIP).
void drawPosePolyline(const Session& session)
{
    const auto& pcs = session.point_clouds_container.point_clouds;
    rlBegin(RL_LINES);
    for (size_t i = 1; i < pcs.size(); i++)
    {
        rlColor3f(pcs[i - 1].render_color[0], pcs[i - 1].render_color[1], pcs[i - 1].render_color[2]);
        vertex(pcs[i - 1].m_pose.translation());
        rlColor3f(pcs[i].render_color[0], pcs[i].render_color[1], pcs[i].render_color[2]);
        vertex(pcs[i].m_pose.translation());
    }
    rlEnd();
}

// Edge line between two poses plus a 10 m vertical flagpole at its midpoint (label drawn in the 2D pass).
void drawEdge(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, float r, float g, float b)
{
    const Eigen::Vector3d mid = (v1 + v2) * 0.5;
    rlBegin(RL_LINES);
    rlColor3f(r, g, b);
    vertex(v1);
    vertex(v2);
    vertex(mid);
    vertex(mid + Eigen::Vector3d(0, 0, 10));
    rlEnd();
}

bool validScan(int session_index, int scan_index)
{
    return session_index >= 0 && session_index < static_cast<int>(sessions.size()) && scan_index >= 0 &&
        scan_index < static_cast<int>(sessions[session_index].point_clouds_container.point_clouds.size());
}

void drawUncertaintyEllipse(const Eigen::Matrix3d& covar, const Eigen::Vector3d& mean, Color color)
{
    Eigen::LLT<Eigen::Matrix<double, 3, 3>> cholSolver(covar);
    Eigen::Matrix3d transform = cholSolver.matrixL();

    const double pi = 3.141592;
    const double di = 0.02;
    const double dj = 0.04;
    const double du = di * 2 * pi;
    const double dv = dj * pi;

    rlBegin(RL_LINES);
    rlColor4ub(color.r, color.g, color.b, color.a);
    for (double i = 0; i < 1.0; i += di)
    {
        for (double j = 0; j < 1.0; j += dj)
        {
            double u = i * 2 * pi;
            double v = (j - 0.5) * pi;

            const Eigen::Vector3d tp0 = transform * Eigen::Vector3d(cos(v) * cos(u), cos(v) * sin(u), sin(v)) + mean;
            const Eigen::Vector3d tp1 = transform * Eigen::Vector3d(cos(v) * cos(u + du), cos(v) * sin(u + du), sin(v)) + mean;
            const Eigen::Vector3d tp2 =
                transform * Eigen::Vector3d(cos(v + dv) * cos(u + du), cos(v + dv) * sin(u + du), sin(v + dv)) + mean;
            const Eigen::Vector3d tp3 = transform * Eigen::Vector3d(cos(v + dv) * cos(u), cos(v + dv) * sin(u), sin(v + dv)) + mean;

            vertex(tp0);
            vertex(tp1);
            vertex(tp1);
            vertex(tp2);
            vertex(tp2);
            vertex(tp3);
            vertex(tp3);
            vertex(tp0);
        }
    }
    rlEnd();
}

// Was GroundControlPoints::render() (legacy GL in core); same drawing as step 2's port. Labels are
// drawn in the 2D pass.
void renderGroundControlPoints(const GroundControlPoints& ground_control_points, const PointClouds& point_clouds_container)
{
    const Color markColor{ 179, 77, 128, 255 };
    const Color connectorColor{ 0, 77, 153, 255 };

    for (const auto& gcp : ground_control_points.gpcs)
    {
        if (gcp.index_to_node_inner < 0 || static_cast<size_t>(gcp.index_to_node_inner) >= point_clouds_container.point_clouds.size())
            continue;
        const auto& pc = point_clouds_container.point_clouds[gcp.index_to_node_inner];
        if (gcp.index_to_node_outer < 0 || static_cast<size_t>(gcp.index_to_node_outer) >= pc.local_trajectory.size())
            continue;

        Eigen::Vector3d c = pc.m_pose * pc.local_trajectory[gcp.index_to_node_outer].m_pose.translation();
        float h = static_cast<float>(gcp.lidar_height_above_ground);
        Vector3 g{ static_cast<float>(gcp.x), static_cast<float>(gcp.y), static_cast<float>(gcp.z) };

        DrawLine3D(Vector3{ g.x - 0.05f, g.y, g.z }, Vector3{ g.x + 0.05f, g.y, g.z }, markColor);
        DrawLine3D(Vector3{ g.x, g.y - 0.05f, g.z }, Vector3{ g.x, g.y + 0.05f, g.z }, markColor);
        DrawLine3D(Vector3{ g.x - 0.01f, g.y, g.z + h }, Vector3{ g.x + 0.01f, g.y, g.z + h }, markColor);
        DrawLine3D(Vector3{ g.x, g.y - 0.01f, g.z + h }, Vector3{ g.x, g.y + 0.01f, g.z + h }, markColor);
        DrawLine3D(g, Vector3{ g.x, g.y, g.z + h }, markColor);
        DrawLine3D(
            Vector3{ static_cast<float>(c.x()), static_cast<float>(c.y()), static_cast<float>(c.z()) },
            Vector3{ g.x, g.y, g.z + h },
            connectorColor);

        if (ground_control_points.draw_uncertainty)
        {
            Eigen::Matrix3d covar = Eigen::Matrix3d::Zero();
            covar(0, 0) = gcp.sigma_x * gcp.sigma_x;
            covar(1, 1) = gcp.sigma_y * gcp.sigma_y;
            covar(2, 2) = gcp.sigma_z * gcp.sigma_z;
            drawUncertaintyEllipse(covar, Eigen::Vector3d(gcp.x, gcp.y, gcp.z + h), GRAY);
        }
    }
}

// Was ControlPoints::render(pcs, show_pc = false) (legacy GL in core): markers only; step 3 never
// opens the control points editor, so step 2's editor branch is not needed.
void renderControlPoints(const ControlPoints& control_points, const PointClouds& point_clouds_container)
{
    const Color markColor{ 179, 77, 128, 255 };
    const Color connectorColor{ 0, 77, 153, 255 };
    const auto& pcs = point_clouds_container.point_clouds;

    for (const auto& cp : control_points.cps)
    {
        if (cp.index_to_pose < 0 || static_cast<size_t>(cp.index_to_pose) >= pcs.size())
            continue;

        Eigen::Vector3d c = pcs[cp.index_to_pose].m_pose * Eigen::Vector3d(cp.x_source_local, cp.y_source_local, cp.z_source_local);
        Vector3 g{ static_cast<float>(cp.x_target_global), static_cast<float>(cp.y_target_global), static_cast<float>(cp.z_target_global) };

        DrawLine3D(Vector3{ g.x - 0.05f, g.y, g.z }, Vector3{ g.x + 0.05f, g.y, g.z }, markColor);
        DrawLine3D(Vector3{ g.x, g.y - 0.05f, g.z }, Vector3{ g.x, g.y + 0.05f, g.z }, markColor);
        DrawLine3D(Vector3{ g.x - 0.01f, g.y, g.z }, Vector3{ g.x + 0.01f, g.y, g.z }, markColor);
        DrawLine3D(Vector3{ g.x, g.y - 0.01f, g.z }, Vector3{ g.x, g.y + 0.01f, g.z }, markColor);
        DrawLine3D(Vector3{ static_cast<float>(c.x()), static_cast<float>(c.y()), static_cast<float>(c.z()) }, g, connectorColor);

        if (control_points.draw_uncertainty)
        {
            Eigen::Matrix3d covar = Eigen::Matrix3d::Zero();
            covar(0, 0) = cp.is_z_0 ? 0.01 * 0.01 : cp.sigma_x * cp.sigma_x;
            covar(1, 1) = cp.is_z_0 ? 0.01 * 0.01 : cp.sigma_y * cp.sigma_y;
            covar(2, 2) = cp.sigma_z * cp.sigma_z;
            drawUncertaintyEllipse(covar, Eigen::Vector3d(cp.x_target_global, cp.y_target_global, cp.z_target_global), GRAY);
        }
    }
}

namespace
{
    // Outlined so labels stay readable over same-colored geometry; `line` stacks labels above one anchor.
    void drawOutlinedText(const char* text, Vector2 anchor, int fontSize, Color color, int line = 0)
    {
        int x = static_cast<int>(anchor.x) + 6;
        int y = static_cast<int>(anchor.y) - fontSize - 6 - line * (fontSize + 4);
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                if (dx != 0 || dy != 0)
                    DrawText(text, x + dx, y + dy, fontSize, BLACK);
        DrawText(text, x, y, fontSize, color);
    }

    // Projects a world point with frame_mvp_3d (needs w for the perspective divide, so not Vector3Transform).
    Vector2 worldToScreen(const Eigen::Vector3d& world)
    {
        const ImGuiIO& io = ImGui::GetIO();
        const Matrix& m = frame_mvp_3d;
        float x = static_cast<float>(world.x());
        float y = static_cast<float>(world.y());
        float z = static_cast<float>(world.z());
        float clipX = m.m0 * x + m.m4 * y + m.m8 * z + m.m12;
        float clipY = m.m1 * x + m.m5 * y + m.m9 * z + m.m13;
        float clipW = m.m3 * x + m.m7 * y + m.m11 * z + m.m15;
        if (clipW < 1e-6f) // behind the camera, or degenerate
            return Vector2{ -1000.f, -1000.f };
        float ndcX = clipX / clipW;
        float ndcY = clipY / clipW;
        return Vector2{ (ndcX * 0.5f + 0.5f) * io.DisplaySize.x, (1.0f - (ndcY * 0.5f + 0.5f)) * io.DisplaySize.y };
    }

    Color colorOf(const float c[3])
    {
        return ColorFromNormalized(Vector4{ c[0], c[1], c[2], 1.f });
    }
} // namespace

void renderGroundControlPointsLabels(const GroundControlPoints& ground_control_points, const PointClouds& point_clouds_container)
{
    const Color markColor{ 179, 77, 128, 255 };
    const Color connectorColor{ 0, 77, 153, 255 };

    for (size_t i = 0; i < ground_control_points.gpcs.size(); ++i)
    {
        const auto& gcp = ground_control_points.gpcs[i];
        Vector2 anchor = worldToScreen(Eigen::Vector3d(gcp.x, gcp.y, gcp.z));
        drawOutlinedText(gcp.name, anchor, 22, WHITE, 2);
        drawOutlinedText(TextFormat("GCP_%d: LiDAR center", static_cast<int>(i)), anchor, 14, markColor, 1);
        drawOutlinedText(TextFormat("GCP_%d: 'plane on the ground'", static_cast<int>(i)), anchor, 14, markColor, 0);

        if (gcp.index_to_node_inner < 0 || static_cast<size_t>(gcp.index_to_node_inner) >= point_clouds_container.point_clouds.size())
            continue;
        const auto& pc = point_clouds_container.point_clouds[gcp.index_to_node_inner];
        if (gcp.index_to_node_outer < 0 || static_cast<size_t>(gcp.index_to_node_outer) >= pc.local_trajectory.size())
            continue;

        Eigen::Vector3d c = pc.m_pose * pc.local_trajectory[gcp.index_to_node_outer].m_pose.translation();
        drawOutlinedText(TextFormat("GCP_%d: assigned trajectory node", static_cast<int>(i)), worldToScreen(c), 14, connectorColor);
    }
}

void renderControlPointsLabels(const ControlPoints& control_points, const PointClouds& point_clouds_container)
{
    const Color markColor{ 179, 77, 128, 255 };

    for (size_t i = 0; i < control_points.cps.size(); ++i)
    {
        const auto& cp = control_points.cps[i];
        Vector2 anchor = worldToScreen(Eigen::Vector3d(cp.x_target_global, cp.y_target_global, cp.z_target_global));
        drawOutlinedText(cp.name, anchor, 22, WHITE, 1);
        drawOutlinedText(TextFormat("CP_%d", static_cast<int>(i)), anchor, 14, WHITE, 0);

        if (cp.index_to_pose < 0 || static_cast<size_t>(cp.index_to_pose) >= point_clouds_container.point_clouds.size())
            continue;

        Eigen::Vector3d c = point_clouds_container.point_clouds[cp.index_to_pose].m_pose *
            Eigen::Vector3d(cp.x_source_local, cp.y_source_local, cp.z_source_local);
        drawOutlinedText(TextFormat("CP_%d: initial location", static_cast<int>(i)), worldToScreen(c), 14, markColor);
    }
}

// Was the glRasterPos3f + glutBitmapString labels of loop closure mode: scan indices of the first and
// second session (in scan color), per-session pose graph edges (blue) and inter-session edges
// (cyan if a ground truth session is involved, otherwise yellow), at the top of each edge's flagpole.
void renderLoopClosureLabels()
{
    for (int s : { first_session_index, second_session_index })
    {
        if (s < 0 || s >= static_cast<int>(sessions.size()))
            continue;
        const auto& pcs = sessions[s].point_clouds_container.point_clouds;
        for (size_t i = 0; i < pcs.size(); i++)
            drawOutlinedText(
                TextFormat("%d", static_cast<int>(i)),
                worldToScreen(pcs[i].m_pose.translation() + Eigen::Vector3d(0, 0, 0.1)),
                20,
                colorOf(pcs[i].render_color));
        if (first_session_index == second_session_index)
            break;
    }

    for (size_t i = 0; i < sessions.size(); i++)
    {
        const auto& pcs = sessions[i].point_clouds_container.point_clouds;
        const auto& pg_edges = sessions[i].pose_graph_loop_closure.edges;
        for (size_t j = 0; j < pg_edges.size(); j++)
        {
            if (!validScan(static_cast<int>(i), pg_edges[j].index_from) || !validScan(static_cast<int>(i), pg_edges[j].index_to))
                continue;
            Eigen::Vector3d mid = (pcs[pg_edges[j].index_from].m_pose.translation() + pcs[pg_edges[j].index_to].m_pose.translation()) * 0.5;
            drawOutlinedText(TextFormat("%d", static_cast<int>(j)), worldToScreen(mid + Eigen::Vector3d(0, 0, 10.1)), 22, BLUE);
        }
    }

    for (size_t i = 0; i < edges.size(); i++)
    {
        const auto& e = edges[i];
        if (!validScan(e.index_session_from, e.index_from) || !validScan(e.index_session_to, e.index_to))
            continue;
        Eigen::Vector3d v1 = sessions[e.index_session_from].point_clouds_container.point_clouds[e.index_from].m_pose.translation();
        Eigen::Vector3d v2 = sessions[e.index_session_to].point_clouds_container.point_clouds[e.index_to].m_pose.translation();
        bool gt = sessions[e.index_session_from].is_ground_truth || sessions[e.index_session_to].is_ground_truth;
        drawOutlinedText(
            TextFormat("%d", static_cast<int>(i)), worldToScreen((v1 + v2) * 0.5 + Eigen::Vector3d(0, 0, 10.1)), 22, gt ? SKYBLUE : YELLOW);
    }
}

// Copies the current rlgl modelview/projection into column-major float[16] for ImGuizmo.
void currentGizmoMatrices(float modelview[16], float projection[16])
{
    Matrix p = rlGetMatrixProjection();
    Matrix m = rlGetMatrixModelview();
    const float pv[16] = { p.m0, p.m1, p.m2, p.m3, p.m4, p.m5, p.m6, p.m7, p.m8, p.m9, p.m10, p.m11, p.m12, p.m13, p.m14, p.m15 };
    const float mv[16] = { m.m0, m.m1, m.m2, m.m3, m.m4, m.m5, m.m6, m.m7, m.m8, m.m9, m.m10, m.m11, m.m12, m.m13, m.m14, m.m15 };
    std::copy(pv, pv + 16, projection);
    std::copy(mv, mv + 16, modelview);
}

// ImGuizmo on m_gizmo with this app's usual operation sets: full 3D in perspective, planar in ortho.
void manipulateGizmo()
{
    if (!app_state.camera.isOrtho)
    {
        float modelview[16], projection[16];
        currentGizmoMatrices(modelview, projection);
        ImGuizmo::Manipulate(
            modelview,
            projection,
            ImGuizmo::TRANSLATE | ImGuizmo::ROTATE_Z | ImGuizmo::ROTATE_X | ImGuizmo::ROTATE_Y,
            ImGuizmo::WORLD,
            m_gizmo,
            NULL);
    }
    else
        ImGuizmo::Manipulate(
            app_state.camera.orthoGizmoView,
            app_state.camera.orthoProjection,
            ImGuizmo::TRANSLATE_X | ImGuizmo::TRANSLATE_Y | ImGuizmo::ROTATE_Z,
            ImGuizmo::WORLD,
            m_gizmo,
            NULL);
}

///////////////////////////////////////////////////////////////////////////////////

void ndt_gui()
{
    static bool compute_mean_and_cov_for_bucket = false;
    if (ImGui::Begin("Normal Distributions Transform", &is_ndt_gui, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::InputFloat3("Bucket size [m] (x, y,z)", ndt.bucket_size);
        if (ndt.bucket_size[0] < 0.01)
            ndt.bucket_size[0] = 0.01f;
        if (ndt.bucket_size[1] < 0.01)
            ndt.bucket_size[1] = 0.01f;
        if (ndt.bucket_size[2] < 0.01)
            ndt.bucket_size[2] = 0.01f;

        ImGui::PushItemWidth(ImGuiNumberWidth);
        ImGui::InputInt("Number of threads", &ndt.number_of_threads);
        if (ndt.number_of_threads < 1)
            ndt.number_of_threads = 1;
        ImGui::SameLine();
        ImGui::InputInt("Number of iterations", &ndt.number_of_iterations);
        if (ndt.number_of_iterations < 1)
            ndt.number_of_iterations = 1;
        ImGui::PopItemWidth();

        if (ImGui::Button("NDT optimization"))
        {
            for (auto& s : sessions)
            {
                s.is_gizmo = false;
            }

            double rms_initial = 0.0;
            double rms_final = 0.0;
            double mui = 0.0;

            ndt.optimize(sessions, false, compute_mean_and_cov_for_bucket);
        }
        ImGui::End();
    }

#if 0
    ImGui::Checkbox("ndt fix_first_node (add I to first pose in Hessian)", &ndt.is_fix_first_node);

    ImGui::Checkbox("ndt Gauss-Newton", &ndt.is_gauss_newton);
    if (ndt.is_gauss_newton)
    {
        ndt.is_levenberg_marguardt = false;
    }

    ImGui::SameLine();
    ImGui::Checkbox("ndt Levenberg-Marguardt", &ndt.is_levenberg_marguardt);
    if (ndt.is_levenberg_marguardt)
    {
        ndt.is_gauss_newton = false;
    }

    ImGui::Checkbox("ndt poses expressed as camera<-world (cw)", &ndt.is_cw);
    if (ndt.is_cw)
    {
        ndt.is_wc = false;
    }
    ImGui::SameLine();
    ImGui::Checkbox("ndt poses expressed as camera->world (wc)", &ndt.is_wc);
    if (ndt.is_wc)
    {
        ndt.is_cw = false;
    }

    ImGui::Checkbox("ndt Tait-Bryan angles (om fi ka: RxRyRz)", &ndt.is_tait_bryan_angles);
    if (ndt.is_tait_bryan_angles)
    {
        ndt.is_quaternion = false;
        ndt.is_rodrigues = false;
    }

    ImGui::SameLine();
    ImGui::Checkbox("ndt Quaternion (q0 q1 q2 q3)", &ndt.is_quaternion);
    if (ndt.is_quaternion)
    {
        ndt.is_tait_bryan_angles = false;
        ndt.is_rodrigues = false;
    }

    ImGui::SameLine();
    ImGui::Checkbox("ndt Rodrigues (sx sy sz)", &ndt.is_rodrigues);
    if (ndt.is_rodrigues)
    {
        ndt.is_tait_bryan_angles = false;
        ndt.is_quaternion = false;
    }

    if (ImGui::Button("compute mean mahalanobis distance"))
    {
        double rms_initial = 0.0;
        double rms_final = 0.0;
        double mui = 0.0;
        ndt.optimize(session.point_clouds_container.point_clouds, true, compute_mean_and_cov_for_bucket);
    }

    ImGui::Text("--------------------------------------------------------------------------------------------------------");

    if (ImGui::Button("ndt_optimization(Lie-algebra left Jacobian)"))
    {
        // icp.optimize_source_to_target_lie_algebra_left_jacobian(point_clouds_container);
        ndt.optimize_lie_algebra_left_jacobian(session.point_clouds_container.point_clouds, compute_mean_and_cov_for_bucket);
    }
    if (ImGui::Button("ndt_optimization(Lie-algebra right Jacobian)"))
    {
        // icp.optimize_source_to_target_lie_algebra_right_jacobian(point_clouds_container);
        ndt.optimize_lie_algebra_right_jacobian(session.point_clouds_container.point_clouds, compute_mean_and_cov_for_bucket);
    }

    ImGui::Text("--------------------------------------------------------------------------------------------------------");

    ImGui::Checkbox("generalized", &ndt.is_generalized);

    if (ndt.is_generalized)
    {
        ImGui::InputDouble("sigma_r", &ndt.sigma_r, 0.01, 0.01);
        ImGui::InputDouble("sigma_polar_angle_rad", &ndt.sigma_polar_angle, 0.0001, 0.0001);
        ImGui::InputDouble("sigma_azimuthal_angle_rad", &ndt.sigma_azimuthal_angle, 0.0001, 0.0001);
        ImGui::InputInt("num_extended_points", &ndt.num_extended_points, 1, 1);

        ImGui::Checkbox("compute_mean_and_cov_for_bucket", &compute_mean_and_cov_for_bucket);
    }

    if (ImGui::Button("Set Zoller+Fröhlich TLS Imager 5006i errors"))
    {
        ndt.sigma_r = 0.0068;
        ndt.sigma_polar_angle = 0.007 * DEG_TO_RAD;
        ndt.sigma_azimuthal_angle = 0.007 * DEG_TO_RAD;
    }

    if (ImGui::Button("Set Zoller+Fröhlich TLS Imager 5010C errors"))
    {
        ndt.sigma_r = 0.01;
        ndt.sigma_polar_angle = 0.007 * DEG_TO_RAD;
        ndt.sigma_azimuthal_angle = 0.007 * DEG_TO_RAD;
    }

    if (ImGui::Button("Set Zoller+Fröhlich TLS Imager 5016 errors"))
    {
        ndt.sigma_r = 0.00025;
        ndt.sigma_polar_angle = 0.004 * DEG_TO_RAD;
        ndt.sigma_azimuthal_angle = 0.004 * DEG_TO_RAD;
    }
    if (ImGui::Button("Set Faro Focus3D errors"))
    {
        ndt.sigma_r = 0.001;
        ndt.sigma_polar_angle = 19.0 * (1.0 / 3600.0) * DEG_TO_RAD;
        ndt.sigma_azimuthal_angle = 19.0 * (1.0 / 3600.0) * DEG_TO_RAD;
    }
    if (ImGui::Button("Set Leica ScanStation C5 C10 errors"))
    {
        ndt.sigma_r = 0.006;
        ndt.sigma_polar_angle = 0.00006;
        ndt.sigma_azimuthal_angle = 0.00006;
    }
    if (ImGui::Button("Set Riegl VZ400 errors"))
    {
        ndt.sigma_r = 0.005;
        ndt.sigma_polar_angle = 0.0005 * DEG_TO_RAD + 0.0003;     // Laser Beam Dicvergence
        ndt.sigma_azimuthal_angle = 0.0005 * DEG_TO_RAD + 0.0003; // Laser Beam Dicvergence
    }
    if (ImGui::Button("Set Leica HDS6100 errors"))
    {
        ndt.sigma_r = 0.009;
        ndt.sigma_polar_angle = 0.000125;
        ndt.sigma_azimuthal_angle = 0.000125;
    }
    if (ImGui::Button("Set Leica P40 errors"))
    {
        ndt.sigma_r = 0.0012;
        ndt.sigma_polar_angle = 8.0 / 3600;
        ndt.sigma_azimuthal_angle = 8.0 / 3600;
    }
#endif
}

void loop_closure_gui()
{
    if (ImGui::Begin("Manual Pose Graph Loop Closure Mode", &is_loop_closure_gui, ImGuiWindowFlags_AlwaysAutoResize))
    {
        if (ImGui::Button("Optimize GRAPH"))
        {
            for (int i = 0; i < 100; i++)
            {
                std::cout << "Iteration [" << i + 1 << "] of: " << 100 << std::endl;
                optimize(sessions, edges, motion_model_weights);
            }
            optimized = true;
        }

        ImGui::Checkbox("update_rotation_center", &update_rotation_center);

        //
        auto point_cloud_upper = sessions[first_session_index].point_clouds_container.point_clouds.size() - 1;

        ImGui::InputInt("gui_point_size", &app_state.point_size);
        if (app_state.point_size < 1)
            app_state.point_size = 1;

        ImGui::Text("Num edge extended:");

        ImGui::Text("before: ");
        ImGui::SameLine();
        ImGui::PushItemWidth(ImGuiNumberWidth);
        ImGui::SliderInt("##fs", &num_edge_extended_before, 0, point_cloud_upper);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("min 0; max %zu", point_cloud_upper);
        ImGui::SameLine();
        ImGui::InputInt("##fi", &num_edge_extended_before, 1, 5);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("min 0; max %zu", point_cloud_upper);
        if (num_edge_extended_before < 0)
            num_edge_extended_before = 0;
        if (num_edge_extended_before >= point_cloud_upper)
            num_edge_extended_before = point_cloud_upper;

        point_cloud_upper = sessions[second_session_index].point_clouds_container.point_clouds.size() - 1;

        ImGui::Text(" after: ");
        ImGui::SameLine();

        ImGui::SliderInt("##ts", &num_edge_extended_after, index_loop_closure_target, static_cast<int>(point_cloud_upper));
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("min 0; max %zu", point_cloud_upper);
        ImGui::SameLine();
        ImGui::InputInt("##ti", &num_edge_extended_after, 1, 5);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("min 0; max %zu", point_cloud_upper);
        if (num_edge_extended_after < 0)
            num_edge_extended_after = 0;
        if (num_edge_extended_after >= point_cloud_upper)
            num_edge_extended_after = point_cloud_upper;
        ImGui::PopItemWidth();
        //

        if (!manipulate_active_edge)
        {
            ImGui::InputInt("index_loop_closure_source", &index_loop_closure_source);
            if (index_loop_closure_source < 0)
                index_loop_closure_source = 0;
            if (index_loop_closure_source >= sessions[first_session_index].point_clouds_container.point_clouds.size() - 1)
                index_loop_closure_source = sessions[first_session_index].point_clouds_container.point_clouds.size() - 1;
            ImGui::InputInt("index_loop_closure_target", &index_loop_closure_target);
            if (index_loop_closure_target < 0)
                index_loop_closure_target = 0;
            if (index_loop_closure_target >= sessions[second_session_index].point_clouds_container.point_clouds.size() - 1)
                index_loop_closure_target = sessions[second_session_index].point_clouds_container.point_clouds.size() - 1;
        }
        if (ImGui::Button("Add edge"))
        {
            Edge edge;
            edge.index_from = index_loop_closure_source;
            edge.index_to = index_loop_closure_target;
            edge.index_session_from = first_session_index;
            edge.index_session_to = second_session_index;

            edge.relative_pose_tb = pose_tait_bryan_from_affine_matrix(
                sessions[first_session_index].point_clouds_container.point_clouds[index_loop_closure_source].m_pose.inverse() *
                sessions[second_session_index].point_clouds_container.point_clouds[index_loop_closure_target].m_pose);

            edge.relative_pose_tb_weights.px = 1000000.0;
            edge.relative_pose_tb_weights.py = 1000000.0;
            edge.relative_pose_tb_weights.pz = 1000000.0;
            edge.relative_pose_tb_weights.om = 1000000.0;
            edge.relative_pose_tb_weights.fi = 1000000.0;
            edge.relative_pose_tb_weights.ka = 1000000.0;

            edges.push_back(edge);

            index_active_edge = edges.size() - 1;
        }

        std::string number_active_edges = "number_edges: " + std::to_string(edges.size());
        ImGui::Text(number_active_edges.c_str());
        if (edges.size() > 0)
        {
            ImGui::Checkbox("manipulate_active_edge", &manipulate_active_edge);
            if (manipulate_active_edge)
            {
                int remove_edge_index = -1;
                if (ImGui::Button("remove active edge"))
                {
                    edge_gizmo = false;
                    remove_edge_index = index_active_edge;
                }

                int prev_index_active_edge = index_active_edge;

                if (!edge_gizmo)
                {
                    bool is_gizmo = false;

                    for (const auto& s : sessions)
                    {
                        if (s.is_gizmo)
                            is_gizmo = true;
                    }

                    if (!is_gizmo)
                    {
                        ImGui::InputInt("index_active_edge", &index_active_edge);

                        if (index_active_edge < 0)
                            index_active_edge = 0;
                        if (index_active_edge >= (int)edges.size())
                            index_active_edge = (int)edges.size() - 1;
                    }
                }

                std::string txt = "index_session_from: " + std::to_string(edges[index_active_edge].index_session_from);
                ImGui::Text(txt.c_str());
                txt = "index_session_to: " + std::to_string(edges[index_active_edge].index_session_to);
                ImGui::Text(txt.c_str());
                txt = "index_from: " + std::to_string(edges[index_active_edge].index_from);
                ImGui::Text(txt.c_str());
                txt = "index_to: " + std::to_string(edges[index_active_edge].index_to);
                ImGui::Text(txt.c_str());

                if (remove_edge_index != -1)
                {
                    std::vector<Edge> new_edges;
                    for (size_t i = 0; i < edges.size(); i++)
                    {
                        if (remove_edge_index != i)
                            new_edges.push_back(edges[i]);
                    }
                    edges = new_edges;

                    index_active_edge = remove_edge_index - 1;
                    manipulate_active_edge = false;
                }

                bool prev_gizmo = edge_gizmo;
                ImGui::Checkbox("gizmo", &edge_gizmo);

                if (prev_gizmo != edge_gizmo)
                {
                    auto m_to = sessions[edges[index_active_edge].index_session_from]
                                    .point_clouds_container.point_clouds[edges[index_active_edge].index_from]
                                    .m_pose *
                        affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                    m_gizmo[0] = (float)m_to(0, 0);
                    m_gizmo[1] = (float)m_to(1, 0);
                    m_gizmo[2] = (float)m_to(2, 0);
                    m_gizmo[3] = (float)m_to(3, 0);
                    m_gizmo[4] = (float)m_to(0, 1);
                    m_gizmo[5] = (float)m_to(1, 1);
                    m_gizmo[6] = (float)m_to(2, 1);
                    m_gizmo[7] = (float)m_to(3, 1);
                    m_gizmo[8] = (float)m_to(0, 2);
                    m_gizmo[9] = (float)m_to(1, 2);
                    m_gizmo[10] = (float)m_to(2, 2);
                    m_gizmo[11] = (float)m_to(3, 2);
                    m_gizmo[12] = (float)m_to(0, 3);
                    m_gizmo[13] = (float)m_to(1, 3);
                    m_gizmo[14] = (float)m_to(2, 3);
                    m_gizmo[15] = (float)m_to(3, 3);
                }
                if (!edge_gizmo)
                {
                    if (ImGui::Button("ICP"))
                    {
                        std::cout << "Iterative Closest Point" << std::endl;
                        if (sessions[edges[index_active_edge].index_session_from].is_ground_truth &&
                            sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                        {
                            std::cout << "Two sessions are ground truth!!! ICP is disabled" << std::endl;
                        }
                        else
                        {
                            bool is_with_ground_truth = false;
                            if (sessions[edges[index_active_edge].index_session_from].is_ground_truth ||
                                sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                            {
                                is_with_ground_truth = true;
                            }

                            if (is_with_ground_truth)
                            {
                                int index_session_from = -1;
                                int index_session_to = -1;
                                int index_from = -1;
                                int index_to = -1;

                                if (sessions[edges[index_active_edge].index_session_from].is_ground_truth)
                                {
                                    index_session_from = edges[index_active_edge].index_session_from;
                                    index_session_to = edges[index_active_edge].index_session_to;
                                    index_from = edges[index_active_edge].index_from;
                                    index_to = edges[index_active_edge].index_to;
                                }
                                else
                                {
                                    index_session_from = edges[index_active_edge].index_session_to;
                                    index_session_to = edges[index_active_edge].index_session_from;
                                    index_from = edges[index_active_edge].index_to;
                                    index_to = edges[index_active_edge].index_from;
                                }

                                double x_min = 1000000000000.0;
                                double y_min = 1000000000000.0;
                                double z_min = 1000000000000.0;
                                double x_max = -1000000000000.0;
                                double y_max = -1000000000000.0;
                                double z_max = -1000000000000.0;

                                auto& points_to = sessions[index_session_to].point_clouds_container.point_clouds[index_to];

                                for (const auto& p : points_to.points_local)
                                {
                                    auto pg = points_to.m_pose * p;
                                    if (pg.x() < x_min)
                                        x_min = pg.x();
                                    if (pg.y() < y_min)
                                        y_min = pg.y();
                                    if (pg.z() < z_min)
                                        z_min = pg.z();

                                    if (pg.x() > x_max)
                                        x_max = pg.x();
                                    if (pg.y() > y_max)
                                        y_max = pg.y();
                                    if (pg.z() > z_max)
                                        z_max = pg.z();
                                }
                                auto& points_from = sessions[index_session_from].point_clouds_container.point_clouds[index_from];
                                std::vector<Eigen::Vector3d> ground_truth;
                                for (const auto& p : points_from.points_local)
                                {
                                    auto pg = points_from.m_pose * p;
                                    if (pg.x() > x_min && pg.x() < x_max)
                                    {
                                        if (pg.y() > y_min && pg.y() < y_max)
                                        {
                                            if (pg.z() > z_min && pg.z() < z_max)
                                                ground_truth.push_back(p);
                                        }
                                    }
                                }

                                int number_of_iterations = 10;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                std::vector<Eigen::Vector3d> source =
                                    sessions[edges[index_active_edge].index_session_to]
                                        .point_clouds_container.point_clouds[edges[index_active_edge].index_to]
                                        .points_local;
                                std::vector<Eigen::Vector3d> target =
                                    ground_truth; // sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from].points_local;

                                if (icp.compute(source, target, search_radius, number_of_iterations, m_pose))
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                            }
                            else
                            {
                                int number_of_iterations = 10;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                std::vector<Eigen::Vector3d> source;
                                auto& e = edges[index_active_edge];
                                for (int i = -num_edge_extended_before; i <= num_edge_extended_after; i++)
                                {
                                    int index_src = e.index_to + i;
                                    if (index_src >= 0 &&
                                        index_src <
                                            sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds.size())
                                    {
                                        Eigen::Affine3d m_src = sessions[edges[index_active_edge].index_session_to]
                                                                    .point_clouds_container.point_clouds.at(index_src)
                                                                    .m_pose;
                                        for (int k = 0; k < sessions[edges[index_active_edge].index_session_to]
                                                                .point_clouds_container.point_clouds[index_src]
                                                                .points_local.size();
                                             k++)
                                        {
                                            Eigen::Vector3d p_g = m_src *
                                                sessions[edges[index_active_edge].index_session_to]
                                                    .point_clouds_container.point_clouds[index_src]
                                                    .points_local[k];
                                            source.push_back(p_g);
                                        }
                                        // point_clouds_container.point_clouds.at(index_src).render(m_src, 1);
                                    }
                                }
                                std::vector<Eigen::Vector3d> target;

                                for (int i = -num_edge_extended_before; i <= num_edge_extended_after; i++)
                                {
                                    int index_trg = e.index_from + i;
                                    if (index_trg >= 0 &&
                                        index_trg < sessions[edges[index_active_edge].index_session_from]
                                                        .point_clouds_container.point_clouds.size())
                                    {
                                        Eigen::Affine3d m_trg = sessions[edges[index_active_edge].index_session_from]
                                                                    .point_clouds_container.point_clouds.at(index_trg)
                                                                    .m_pose;
                                        for (int k = 0; k < sessions[edges[index_active_edge].index_session_from]
                                                                .point_clouds_container.point_clouds[index_trg]
                                                                .points_local.size();
                                             k++)
                                        {
                                            Eigen::Vector3d p_g = m_trg *
                                                sessions[edges[index_active_edge].index_session_from]
                                                    .point_clouds_container.point_clouds[index_trg]
                                                    .points_local[k];
                                            target.push_back(p_g);
                                        }
                                        // point_clouds_container.point_clouds.at(index_src).render(m_src, 1);
                                    }
                                }

                                Eigen::Affine3d m_src_inv = sessions[edges[index_active_edge].index_session_to]
                                                                .point_clouds_container.point_clouds[e.index_to]
                                                                .m_pose.inverse();

                                for (auto& p : source)
                                {
                                    p = m_src_inv * p;
                                }

                                Eigen::Affine3d m_trg_inv = sessions[edges[index_active_edge].index_session_from]
                                                                .point_clouds_container.point_clouds[e.index_from]
                                                                .m_pose.inverse();

                                for (auto& p : target)
                                {
                                    p = m_trg_inv * p;
                                }

                                if (icp.compute(source, target, search_radius, number_of_iterations, m_pose))
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                            }
                        }
                    }
                    ImGui::SameLine();
                    ImGui::InputDouble("search_radius", &search_radius);
                    if (search_radius < 0.01)
                        search_radius = 0.01;

                    /////////////////////////////////
                    if (ImGui::Button("ICP [search radius 2m]"))
                    {
                        float sr = 2.0;
                        std::cout << "Iterative Closest Point" << std::endl;
                        if (sessions[edges[index_active_edge].index_session_from].is_ground_truth &&
                            sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                        {
                            std::cout << "Two sessions are ground truth!!! ICP is disabled" << std::endl;
                        }
                        else
                        {
                            bool is_with_ground_truth = false;
                            if (sessions[edges[index_active_edge].index_session_from].is_ground_truth ||
                                sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                            {
                                is_with_ground_truth = true;
                            }

                            if (is_with_ground_truth)
                            {
                                int index_session_from = -1;
                                int index_session_to = -1;
                                int index_from = -1;
                                int index_to = -1;

                                if (sessions[edges[index_active_edge].index_session_from].is_ground_truth)
                                {
                                    index_session_from = edges[index_active_edge].index_session_from;
                                    index_session_to = edges[index_active_edge].index_session_to;
                                    index_from = edges[index_active_edge].index_from;
                                    index_to = edges[index_active_edge].index_to;
                                }
                                else
                                {
                                    index_session_from = edges[index_active_edge].index_session_to;
                                    index_session_to = edges[index_active_edge].index_session_from;
                                    index_from = edges[index_active_edge].index_to;
                                    index_to = edges[index_active_edge].index_from;
                                }

                                double x_min = 1000000000000.0;
                                double y_min = 1000000000000.0;
                                double z_min = 1000000000000.0;
                                double x_max = -1000000000000.0;
                                double y_max = -1000000000000.0;
                                double z_max = -1000000000000.0;

                                auto& points_to = sessions[index_session_to].point_clouds_container.point_clouds[index_to];

                                for (const auto& p : points_to.points_local)
                                {
                                    auto pg = points_to.m_pose * p;
                                    if (pg.x() < x_min)
                                        x_min = pg.x();
                                    if (pg.y() < y_min)
                                        y_min = pg.y();
                                    if (pg.z() < z_min)
                                        z_min = pg.z();

                                    if (pg.x() > x_max)
                                        x_max = pg.x();
                                    if (pg.y() > y_max)
                                        y_max = pg.y();
                                    if (pg.z() > z_max)
                                        z_max = pg.z();
                                }
                                auto& points_from = sessions[index_session_from].point_clouds_container.point_clouds[index_from];
                                std::vector<Eigen::Vector3d> ground_truth;
                                for (const auto& p : points_from.points_local)
                                {
                                    auto pg = points_from.m_pose * p;
                                    if (pg.x() > x_min && pg.x() < x_max)
                                    {
                                        if (pg.y() > y_min && pg.y() < y_max)
                                        {
                                            if (pg.z() > z_min && pg.z() < z_max)
                                                ground_truth.push_back(p);
                                        }
                                    }
                                }

                                int number_of_iterations = 30;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                const std::vector<Eigen::Vector3d>& source =
                                    sessions[edges[index_active_edge].index_session_to]
                                        .point_clouds_container.point_clouds[edges[index_active_edge].index_to]
                                        .points_local;
                                const std::vector<Eigen::Vector3d>& target =
                                    ground_truth; // sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from].points_local;

                                if (icp.compute(source, target, sr, number_of_iterations, m_pose))
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                            }
                            else
                            {
                                int number_of_iterations = 30;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                const std::vector<Eigen::Vector3d>& source =
                                    sessions[edges[index_active_edge].index_session_to]
                                        .point_clouds_container.point_clouds[edges[index_active_edge].index_to]
                                        .points_local;
                                const std::vector<Eigen::Vector3d>& target =
                                    sessions[edges[index_active_edge].index_session_from]
                                        .point_clouds_container.point_clouds[edges[index_active_edge].index_from]
                                        .points_local;

                                if (icp.compute(source, target, sr, number_of_iterations, m_pose))
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                            }
                        }
                    }

                    ImGui::SameLine();
                    if (ImGui::Button("ICP [search radius 1m]"))
                    {
                        float sr = 1.0;
                        std::cout << "Iterative Closest Point" << std::endl;
                        if (sessions[edges[index_active_edge].index_session_from].is_ground_truth &&
                            sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                        {
                            std::cout << "Two sessions are ground truth!!! ICP is disabled" << std::endl;
                        }
                        else
                        {
                            bool is_with_ground_truth = false;
                            if (sessions[edges[index_active_edge].index_session_from].is_ground_truth ||
                                sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                            {
                                is_with_ground_truth = true;
                            }

                            if (is_with_ground_truth)
                            {
                                int index_session_from = -1;
                                int index_session_to = -1;
                                int index_from = -1;
                                int index_to = -1;

                                if (sessions[edges[index_active_edge].index_session_from].is_ground_truth)
                                {
                                    index_session_from = edges[index_active_edge].index_session_from;
                                    index_session_to = edges[index_active_edge].index_session_to;
                                    index_from = edges[index_active_edge].index_from;
                                    index_to = edges[index_active_edge].index_to;
                                }
                                else
                                {
                                    index_session_from = edges[index_active_edge].index_session_to;
                                    index_session_to = edges[index_active_edge].index_session_from;
                                    index_from = edges[index_active_edge].index_to;
                                    index_to = edges[index_active_edge].index_from;
                                }

                                double x_min = 1000000000000.0;
                                double y_min = 1000000000000.0;
                                double z_min = 1000000000000.0;
                                double x_max = -1000000000000.0;
                                double y_max = -1000000000000.0;
                                double z_max = -1000000000000.0;

                                auto& points_to = sessions[index_session_to].point_clouds_container.point_clouds[index_to];

                                for (const auto& p : points_to.points_local)
                                {
                                    auto pg = points_to.m_pose * p;
                                    if (pg.x() < x_min)
                                        x_min = pg.x();
                                    if (pg.y() < y_min)
                                        y_min = pg.y();
                                    if (pg.z() < z_min)
                                        z_min = pg.z();

                                    if (pg.x() > x_max)
                                        x_max = pg.x();
                                    if (pg.y() > y_max)
                                        y_max = pg.y();
                                    if (pg.z() > z_max)
                                        z_max = pg.z();
                                }
                                auto& points_from = sessions[index_session_from].point_clouds_container.point_clouds[index_from];
                                std::vector<Eigen::Vector3d> ground_truth;
                                for (const auto& p : points_from.points_local)
                                {
                                    auto pg = points_from.m_pose * p;
                                    if (pg.x() > x_min && pg.x() < x_max)
                                    {
                                        if (pg.y() > y_min && pg.y() < y_max)
                                        {
                                            if (pg.z() > z_min && pg.z() < z_max)
                                                ground_truth.push_back(p);
                                        }
                                    }
                                }

                                int number_of_iterations = 30;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                const std::vector<Eigen::Vector3d>& source =
                                    sessions[edges[index_active_edge].index_session_to]
                                        .point_clouds_container.point_clouds[edges[index_active_edge].index_to]
                                        .points_local;
                                const std::vector<Eigen::Vector3d>& target =
                                    ground_truth; // sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from].points_local;

                                if (icp.compute(source, target, sr, number_of_iterations, m_pose))
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                            }
                            else
                            {
                                int number_of_iterations = 30;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                const std::vector<Eigen::Vector3d>& source =
                                    sessions[edges[index_active_edge].index_session_to]
                                        .point_clouds_container.point_clouds[edges[index_active_edge].index_to]
                                        .points_local;
                                const std::vector<Eigen::Vector3d>& target =
                                    sessions[edges[index_active_edge].index_session_from]
                                        .point_clouds_container.point_clouds[edges[index_active_edge].index_from]
                                        .points_local;

                                if (icp.compute(source, target, sr, number_of_iterations, m_pose))
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                            }
                        }
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("ICP [search radius 0.5m]"))
                    {
                        float sr = 0.5;
                        std::cout << "Iterative Closest Point" << std::endl;
                        if (sessions[edges[index_active_edge].index_session_from].is_ground_truth &&
                            sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                        {
                            std::cout << "Two sessions are ground truth!!! ICP is disabled" << std::endl;
                        }
                        else
                        {
                            bool is_with_ground_truth = false;
                            if (sessions[edges[index_active_edge].index_session_from].is_ground_truth ||
                                sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                            {
                                is_with_ground_truth = true;
                            }

                            if (is_with_ground_truth)
                            {
                                int index_session_from = -1;
                                int index_session_to = -1;
                                int index_from = -1;
                                int index_to = -1;

                                if (sessions[edges[index_active_edge].index_session_from].is_ground_truth)
                                {
                                    index_session_from = edges[index_active_edge].index_session_from;
                                    index_session_to = edges[index_active_edge].index_session_to;
                                    index_from = edges[index_active_edge].index_from;
                                    index_to = edges[index_active_edge].index_to;
                                }
                                else
                                {
                                    index_session_from = edges[index_active_edge].index_session_to;
                                    index_session_to = edges[index_active_edge].index_session_from;
                                    index_from = edges[index_active_edge].index_to;
                                    index_to = edges[index_active_edge].index_from;
                                }

                                double x_min = 1000000000000.0;
                                double y_min = 1000000000000.0;
                                double z_min = 1000000000000.0;
                                double x_max = -1000000000000.0;
                                double y_max = -1000000000000.0;
                                double z_max = -1000000000000.0;

                                auto& points_to = sessions[index_session_to].point_clouds_container.point_clouds[index_to];

                                for (const auto& p : points_to.points_local)
                                {
                                    auto pg = points_to.m_pose * p;
                                    if (pg.x() < x_min)
                                        x_min = pg.x();
                                    if (pg.y() < y_min)
                                        y_min = pg.y();
                                    if (pg.z() < z_min)
                                        z_min = pg.z();

                                    if (pg.x() > x_max)
                                        x_max = pg.x();
                                    if (pg.y() > y_max)
                                        y_max = pg.y();
                                    if (pg.z() > z_max)
                                        z_max = pg.z();
                                }
                                auto& points_from = sessions[index_session_from].point_clouds_container.point_clouds[index_from];
                                std::vector<Eigen::Vector3d> ground_truth;
                                for (const auto& p : points_from.points_local)
                                {
                                    auto pg = points_from.m_pose * p;
                                    if (pg.x() > x_min && pg.x() < x_max)
                                    {
                                        if (pg.y() > y_min && pg.y() < y_max)
                                        {
                                            if (pg.z() > z_min && pg.z() < z_max)
                                                ground_truth.push_back(p);
                                        }
                                    }
                                }

                                int number_of_iterations = 30;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                const std::vector<Eigen::Vector3d>& source =
                                    sessions[edges[index_active_edge].index_session_to]
                                        .point_clouds_container.point_clouds[edges[index_active_edge].index_to]
                                        .points_local;
                                const std::vector<Eigen::Vector3d>& target =
                                    ground_truth; // sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from].points_local;

                                if (icp.compute(source, target, sr, number_of_iterations, m_pose))
                                {
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                                }
                            }
                            else
                            {
                                int number_of_iterations = 30;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                const std::vector<Eigen::Vector3d>& source =
                                    sessions[edges[index_active_edge].index_session_to]
                                        .point_clouds_container.point_clouds[edges[index_active_edge].index_to]
                                        .points_local;
                                const std::vector<Eigen::Vector3d>& target =
                                    sessions[edges[index_active_edge].index_session_from]
                                        .point_clouds_container.point_clouds[edges[index_active_edge].index_from]
                                        .points_local;

                                if (icp.compute(source, target, sr, number_of_iterations, m_pose))
                                {
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                                }
                            }
                        }
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("ICP [search radius 0.25m]"))
                    {
                        float sr = 0.25;
                        std::cout << "Iterative Closest Point" << std::endl;
                        if (sessions[edges[index_active_edge].index_session_from].is_ground_truth &&
                            sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                        {
                            std::cout << "Two sessions are ground truth!!! ICP is disabled" << std::endl;
                        }
                        else
                        {
                            bool is_with_ground_truth = false;
                            if (sessions[edges[index_active_edge].index_session_from].is_ground_truth ||
                                sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                            {
                                is_with_ground_truth = true;
                            }

                            if (is_with_ground_truth)
                            {
                                int index_session_from = -1;
                                int index_session_to = -1;
                                int index_from = -1;
                                int index_to = -1;

                                if (sessions[edges[index_active_edge].index_session_from].is_ground_truth)
                                {
                                    index_session_from = edges[index_active_edge].index_session_from;
                                    index_session_to = edges[index_active_edge].index_session_to;
                                    index_from = edges[index_active_edge].index_from;
                                    index_to = edges[index_active_edge].index_to;
                                }
                                else
                                {
                                    index_session_from = edges[index_active_edge].index_session_to;
                                    index_session_to = edges[index_active_edge].index_session_from;
                                    index_from = edges[index_active_edge].index_to;
                                    index_to = edges[index_active_edge].index_from;
                                }

                                double x_min = 1000000000000.0;
                                double y_min = 1000000000000.0;
                                double z_min = 1000000000000.0;
                                double x_max = -1000000000000.0;
                                double y_max = -1000000000000.0;
                                double z_max = -1000000000000.0;

                                auto& points_to = sessions[index_session_to].point_clouds_container.point_clouds[index_to];

                                for (const auto& p : points_to.points_local)
                                {
                                    auto pg = points_to.m_pose * p;
                                    if (pg.x() < x_min)
                                        x_min = pg.x();
                                    if (pg.y() < y_min)
                                        y_min = pg.y();
                                    if (pg.z() < z_min)
                                        z_min = pg.z();

                                    if (pg.x() > x_max)
                                        x_max = pg.x();
                                    if (pg.y() > y_max)
                                        y_max = pg.y();
                                    if (pg.z() > z_max)
                                        z_max = pg.z();
                                }
                                auto& points_from = sessions[index_session_from].point_clouds_container.point_clouds[index_from];
                                std::vector<Eigen::Vector3d> ground_truth;
                                for (const auto& p : points_from.points_local)
                                {
                                    auto pg = points_from.m_pose * p;
                                    if (pg.x() > x_min && pg.x() < x_max)
                                    {
                                        if (pg.y() > y_min && pg.y() < y_max)
                                        {
                                            if (pg.z() > z_min && pg.z() < z_max)
                                                ground_truth.push_back(p);
                                        }
                                    }
                                }

                                int number_of_iterations = 30;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                const std::vector<Eigen::Vector3d>& source =
                                    sessions[edges[index_active_edge].index_session_to]
                                        .point_clouds_container.point_clouds[edges[index_active_edge].index_to]
                                        .points_local;
                                const std::vector<Eigen::Vector3d>& target =
                                    ground_truth; // sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from].points_local;

                                if (icp.compute(source, target, sr, number_of_iterations, m_pose))
                                {
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                                }
                            }
                            else
                            {
                                int number_of_iterations = 30;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                const std::vector<Eigen::Vector3d>& source =
                                    sessions[edges[index_active_edge].index_session_to]
                                        .point_clouds_container.point_clouds[edges[index_active_edge].index_to]
                                        .points_local;
                                const std::vector<Eigen::Vector3d>& target =
                                    sessions[edges[index_active_edge].index_session_from]
                                        .point_clouds_container.point_clouds[edges[index_active_edge].index_from]
                                        .points_local;

                                if (icp.compute(source, target, sr, number_of_iterations, m_pose))
                                {
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                                }
                            }
                        }
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("ICP [search radius 0.1m]"))
                    {
                        float sr = 0.1;
                        std::cout << "Iterative Closest Point" << std::endl;
                        if (sessions[edges[index_active_edge].index_session_from].is_ground_truth &&
                            sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                        {
                            std::cout << "Two sessions are ground truth!!! ICP is disabled" << std::endl;
                        }
                        else
                        {
                            bool is_with_ground_truth = false;
                            if (sessions[edges[index_active_edge].index_session_from].is_ground_truth ||
                                sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                            {
                                is_with_ground_truth = true;
                            }

                            if (is_with_ground_truth)
                            {
                                int index_session_from = -1;
                                int index_session_to = -1;
                                int index_from = -1;
                                int index_to = -1;

                                if (sessions[edges[index_active_edge].index_session_from].is_ground_truth)
                                {
                                    index_session_from = edges[index_active_edge].index_session_from;
                                    index_session_to = edges[index_active_edge].index_session_to;
                                    index_from = edges[index_active_edge].index_from;
                                    index_to = edges[index_active_edge].index_to;
                                }
                                else
                                {
                                    index_session_from = edges[index_active_edge].index_session_to;
                                    index_session_to = edges[index_active_edge].index_session_from;
                                    index_from = edges[index_active_edge].index_to;
                                    index_to = edges[index_active_edge].index_from;
                                }

                                double x_min = 1000000000000.0;
                                double y_min = 1000000000000.0;
                                double z_min = 1000000000000.0;
                                double x_max = -1000000000000.0;
                                double y_max = -1000000000000.0;
                                double z_max = -1000000000000.0;

                                auto& points_to = sessions[index_session_to].point_clouds_container.point_clouds[index_to];

                                for (const auto& p : points_to.points_local)
                                {
                                    auto pg = points_to.m_pose * p;
                                    if (pg.x() < x_min)
                                        x_min = pg.x();
                                    if (pg.y() < y_min)
                                        y_min = pg.y();
                                    if (pg.z() < z_min)
                                        z_min = pg.z();

                                    if (pg.x() > x_max)
                                        x_max = pg.x();
                                    if (pg.y() > y_max)
                                        y_max = pg.y();
                                    if (pg.z() > z_max)
                                        z_max = pg.z();
                                }
                                auto& points_from = sessions[index_session_from].point_clouds_container.point_clouds[index_from];
                                std::vector<Eigen::Vector3d> ground_truth;
                                for (const auto& p : points_from.points_local)
                                {
                                    auto pg = points_from.m_pose * p;
                                    if (pg.x() > x_min && pg.x() < x_max)
                                    {
                                        if (pg.y() > y_min && pg.y() < y_max)
                                        {
                                            if (pg.z() > z_min && pg.z() < z_max)
                                                ground_truth.push_back(p);
                                        }
                                    }
                                }

                                int number_of_iterations = 30;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                const std::vector<Eigen::Vector3d>& source =
                                    sessions[edges[index_active_edge].index_session_to]
                                        .point_clouds_container.point_clouds[edges[index_active_edge].index_to]
                                        .points_local;
                                const std::vector<Eigen::Vector3d>& target =
                                    ground_truth; // sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from].points_local;

                                if (icp.compute(source, target, sr, number_of_iterations, m_pose))
                                {
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                                }
                            }
                            else
                            {
                                int number_of_iterations = 30;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                const std::vector<Eigen::Vector3d>& source =
                                    sessions[edges[index_active_edge].index_session_to]
                                        .point_clouds_container.point_clouds[edges[index_active_edge].index_to]
                                        .points_local;
                                const std::vector<Eigen::Vector3d>& target =
                                    sessions[edges[index_active_edge].index_session_from]
                                        .point_clouds_container.point_clouds[edges[index_active_edge].index_from]
                                        .points_local;

                                if (icp.compute(source, target, sr, number_of_iterations, m_pose))
                                {
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                                }
                            }
                        }
                    }
#if 0
                    if (ImGui::Button("Save src"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog("Output file name", mandeye::fd::LAS_LAZ_filter, ".laz");
                        std::cout << "laz file to save: '" << output_file_name << "'" << std::endl;

                        if (output_file_name.size() > 0)
                        {
                            std::vector<Eigen::Vector3d> source = sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds[edges[index_active_edge].index_to].points_local;
                            std::vector<Eigen::Vector3d> pointcloud;
                            std::vector<unsigned short> intensity;
                            std::vector<double> timestamps;

                            for (size_t i = 0; i < source.size(); i++)
                            {
                                pointcloud.push_back(source[i]);
                                intensity.push_back(0);
                                timestamps.push_back(0.0);
                            }

                            exportLaz(
                                output_file_name[0],
                                pointcloud,
                                intensity,
                                timestamps);
                        }

                        /*int number_of_iterations = 30;
                        PairWiseICP icp;
                        auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                        std::vector<Eigen::Vector3d> source = sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds[edges[index_active_edge].index_to].points_local;
                        std::vector<Eigen::Vector3d> target = sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from].points_local;

                        if (icp.compute(source, target, sr, number_of_iterations, m_pose))
                        {
                            edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                        }*/
                        //save
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Save trg (transfromed only by rotation)"))
                    {
                    }
#endif
                    //////////////////////////////////
                }
            }
        }

        ImGui::End();
    }
}

void save_trajectories_to_laz(
    const Session& session,
    const std::string& output_file_name,
    float curve_consecutive_distance_meters,
    float not_curve_consecutive_distance_meters,
    bool is_trajectory_export_downsampling)
{
    std::vector<Eigen::Vector3d> pointcloud;
    std::vector<unsigned short> intensity;
    std::vector<double> timestamps;

    float consecutive_distance = 0;
    for (auto& p : session.point_clouds_container.point_clouds)
    {
        if (p.visible)
        {
            for (size_t i = 0; i < p.local_trajectory.size(); i++)
            {
                const auto& pp = p.local_trajectory[i].m_pose.translation();
                Eigen::Vector3d vp;
                vp = p.m_pose * pp; // + session.point_clouds_container.offset;

                if (i > 0)
                {
                    double dist = (p.local_trajectory[i].m_pose.translation() - p.local_trajectory[i - 1].m_pose.translation()).norm();
                    consecutive_distance += dist;
                }

                bool is_curve = false;

                if (i > 100 && i < p.local_trajectory.size() - 100)
                {
                    Eigen::Vector3d position_prev = p.local_trajectory[i - 100].m_pose.translation();
                    Eigen::Vector3d position_curr = p.local_trajectory[i].m_pose.translation();
                    Eigen::Vector3d position_next = p.local_trajectory[i + 100].m_pose.translation();

                    Eigen::Vector3d v1 = position_curr - position_prev;
                    Eigen::Vector3d v2 = position_next - position_curr;

                    if (v1.norm() > 0 && v2.norm() > 0)
                    {
                        double angle_deg = fabs(acos(v1.dot(v2) / (v1.norm() * v2.norm())) * RAD_TO_DEG);

                        if (angle_deg > 10.0)
                        {
                            is_curve = true;
                        }
                    }
                }
                double tol = not_curve_consecutive_distance_meters;

                if (is_curve)
                {
                    tol = curve_consecutive_distance_meters;
                }

                if (!is_trajectory_export_downsampling)
                {
                    pointcloud.push_back(vp);
                    intensity.push_back(0);
                    timestamps.push_back(p.local_trajectory[i].timestamps.first);
                }
                else
                {
                    if (consecutive_distance >= tol)
                    {
                        consecutive_distance = 0;
                        pointcloud.push_back(vp);
                        intensity.push_back(0);
                        timestamps.push_back(p.local_trajectory[i].timestamps.first);
                    }
                }
            }
        }
    }
    // if (!exportLaz(output_file_name, pointcloud, intensity, gnss.offset_x, gnss.offset_y, gnss.offset_alt))
    if (!exportLaz(
            output_file_name,
            pointcloud,
            intensity,
            timestamps,
            session.point_clouds_container.offset.x(),
            session.point_clouds_container.offset.y(),
            session.point_clouds_container.offset.z()))
    {
        std::cout << "problem with saving file: " << output_file_name << std::endl;
    }
}

void createDXFPolyline(const std::string& filename, const std::vector<Eigen::Vector3d>& points)
{
    std::ofstream dxfFile(filename);
    dxfFile << std::setprecision(20);
    if (!dxfFile.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    // DXF header
    dxfFile << "0\nSECTION\n2\nHEADER\n0\nENDSEC\n";
    dxfFile << "0\nSECTION\n2\nTABLES\n0\nENDSEC\n";

    // Start the ENTITIES section
    dxfFile << "0\nSECTION\n2\nENTITIES\n";

    // Start the POLYLINE entity
    dxfFile << "0\nPOLYLINE\n";
    dxfFile << "8\n0\n"; // Layer 0
    dxfFile << "66\n1\n"; // Indicates the presence of vertices
    dxfFile << "70\n8\n"; // 1 = Open polyline

    // Write the VERTEX entities
    for (const auto& point : points)
    {
        dxfFile << "0\nVERTEX\n";
        dxfFile << "8\n0\n"; // Layer 0
        dxfFile << "10\n" << point.x() << "\n"; // X coordinate
        dxfFile << "20\n" << point.y() << "\n"; // Y coordinate
        dxfFile << "30\n" << point.z() << "\n"; // Z coordinate
    }

    // End the POLYLINE
    dxfFile << "0\nSEQEND\n";

    // End the ENTITIES section
    dxfFile << "0\nENDSEC\n";

    // End the DXF file
    dxfFile << "0\nEOF\n";

    dxfFile.close();
    std::cout << "DXF file created: " << filename << std::endl;
}

void save_trajectories(
    Session& session,
    const std::string& output_file_name,
    float curve_consecutive_distance_meters,
    float not_curve_consecutive_distance_meters,
    bool is_trajectory_export_downsampling,
    bool write_lidar_timestamp,
    bool write_unix_timestamp,
    bool use_quaternions,
    bool save_to_dxf)
{
    std::ofstream outfile;
    if (!save_to_dxf)
    {
        outfile.open(output_file_name);
    }
    if (save_to_dxf || outfile.good())
    {
        float consecutive_distance = 0;
        std::vector<Eigen::Vector3d> polylinePoints;
        for (auto& p : session.point_clouds_container.point_clouds)
        {
            if (p.visible)
            {
                for (size_t i = 0; i < p.local_trajectory.size(); i++)
                {
                    const auto& m = p.local_trajectory[i].m_pose;
                    Eigen::Affine3d pose = p.m_pose * m;
                    pose.translation() += session.point_clouds_container.offset;

                    if (i > 0)
                    {
                        double dist = (p.local_trajectory[i].m_pose.translation() - p.local_trajectory[i - 1].m_pose.translation()).norm();
                        consecutive_distance += dist;
                    }

                    bool is_curve = false;

                    if (i > 100 && i < p.local_trajectory.size() - 100)
                    {
                        Eigen::Vector3d position_prev = p.local_trajectory[i - 100].m_pose.translation();
                        Eigen::Vector3d position_curr = p.local_trajectory[i].m_pose.translation();
                        Eigen::Vector3d position_next = p.local_trajectory[i + 100].m_pose.translation();

                        Eigen::Vector3d v1 = position_curr - position_prev;
                        Eigen::Vector3d v2 = position_next - position_curr;

                        if (v1.norm() > 0 && v2.norm() > 0)
                        {
                            double angle_deg = fabs(acos(v1.dot(v2) / (v1.norm() * v2.norm())) * RAD_TO_DEG);

                            if (angle_deg > 10.0)
                                is_curve = true;
                        }
                    }
                    double tol = not_curve_consecutive_distance_meters;

                    if (is_curve)
                        tol = curve_consecutive_distance_meters;

                    if (!is_trajectory_export_downsampling || (is_trajectory_export_downsampling && consecutive_distance >= tol))
                    {
                        if (is_trajectory_export_downsampling)
                            consecutive_distance = 0;
                        if (save_to_dxf)
                            polylinePoints.push_back(pose.translation());
                        else
                        {
                            outfile << std::setprecision(20);

                            if (write_lidar_timestamp)
                                outfile << p.local_trajectory[i].timestamps.first << ",";
                            if (write_unix_timestamp)
                                outfile << p.local_trajectory[i].timestamps.second << ",";

                            outfile << pose(0, 3) << "," << pose(1, 3) << "," << pose(2, 3) << ",";
                            if (use_quaternions)
                            {
                                Eigen::Quaterniond q(pose.rotation());
                                outfile << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << std::endl;
                            }
                            else
                                outfile << pose(0, 0) << "," << pose(0, 1) << "," << pose(0, 2) << "," << pose(1, 0) << "," << pose(1, 1)
                                        << "," << pose(1, 2) << "," << pose(2, 0) << "," << pose(2, 1) << "," << pose(2, 2) << std::endl;
                        }
                    }
                }
            }
        }
        if (!save_to_dxf)
            outfile.close();
        else
            createDXFPolyline(output_file_name, polylinePoints);
    }
}

bool save_project_settings(const std::string& file_name, const ProjectSettings& _project_settings)
{
    std::cout << "saving file: '" << file_name << "'" << std::endl;

    nlohmann::json jj;

    nlohmann::json jsession_file_names;
    for (const auto& pc : _project_settings.session_file_names)
    {
        nlohmann::json jfn{ { "session_file_name", pc } };
        jsession_file_names.push_back(jfn);
    }
    jj["session_file_names"] = jsession_file_names;

    nlohmann::json jloop_closure_edges;
    for (const auto& edge : edges)
    {
        nlohmann::json jloop_closure_edge{
            { "px", edge.relative_pose_tb.px },
            { "py", edge.relative_pose_tb.py },
            { "pz", edge.relative_pose_tb.pz },
            { "om", edge.relative_pose_tb.om },
            { "fi", edge.relative_pose_tb.fi },
            { "ka", edge.relative_pose_tb.ka },
            { "w_px", edge.relative_pose_tb_weights.px },
            { "w_py", edge.relative_pose_tb_weights.py },
            { "w_pz", edge.relative_pose_tb_weights.pz },
            { "w_om", edge.relative_pose_tb_weights.om },
            { "w_fi", edge.relative_pose_tb_weights.fi },
            { "w_ka", edge.relative_pose_tb_weights.ka },
            { "index_from", edge.index_from },
            { "index_to", edge.index_to },
            { "is_fixed_px", edge.is_fixed_px },
            { "is_fixed_py", edge.is_fixed_py },
            { "is_fixed_pz", edge.is_fixed_pz },
            { "is_fixed_om", edge.is_fixed_om },
            { "is_fixed_fi", edge.is_fixed_fi },
            { "is_fixed_ka", edge.is_fixed_ka },
            { "index_session_from", edge.index_session_from },
            { "index_session_to", edge.index_session_to },
        };
        jloop_closure_edges.push_back(jloop_closure_edge);
    }
    jj["loop_closure_edges"] = jloop_closure_edges;

    std::ofstream fs(file_name);
    if (!fs.good())
        return false;
    fs << jj.dump(2);
    fs.close();

    return true;
}

void update_timestamp_offset()
{
    std::cout << "update_timestamp" << std::endl;
    time_stamp_offset = std::numeric_limits<double>::max();

    for (const auto& s : sessions)
    {
        if (!s.point_clouds_container.point_clouds.empty() && !s.point_clouds_container.point_clouds[0].local_trajectory.empty())
        {
            double ts = s.point_clouds_container.point_clouds[0].local_trajectory[0].timestamps.first;
            if (ts < time_stamp_offset)
                time_stamp_offset = ts;
        }
    }

    std::cout << "new time_stamp_offset = " << time_stamp_offset << std::endl;
}

bool revert(std::vector<Session>& sessions)
{
    for (auto& session : sessions)
    {
        for (auto& pc : session.point_clouds_container.point_clouds)
            pc.m_pose = pc.m_pose_temp;
    }
    return true;
}

bool revert_to_initial(std::vector<Session>& sessions)
{
    for (auto& session : sessions)
    {
        for (auto& pc : session.point_clouds_container.point_clouds)
            pc.m_pose = pc.m_initial_pose;
    }
    return true;
}

bool save_results(std::vector<Session>& sessions)
{
    for (auto& session : sessions)
    {
        if (!session.is_ground_truth)
        {
            std::cout << "saving result to: " << session.point_clouds_container.poses_file_name << std::endl;
            session.point_clouds_container.save_poses(fs::path(session.point_clouds_container.poses_file_name).string(), false);
        }
    }
    return true;
}

Eigen::Vector3d GLWidgetGetOGLPos(int x, int y, const ObservationPicking& observation_picking)
{
    const auto laser_beam = GetLaserBeam(x, y);

    RegistrationPlaneFeature::Plane pl;

    pl.a = 0;
    pl.b = 0;
    pl.c = 1;
    pl.d = -observation_picking.picking_plane_height;

    Eigen::Vector3d pos = rayIntersection(laser_beam, pl);

    std::cout << "intersection: " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;

    return pos;
}

bool loadProject(const std::string& file_name, ProjectSettings& _project_settings)
{
    std::cout << "Opening project file: '" << file_name << "'\n";

    try
    {
        std::ifstream fs(file_name);
        if (!fs.good())
            return false;
        nlohmann::json data = nlohmann::json::parse(fs);
        fs.close();

        _project_settings.session_file_names.clear();

        std::cout << "Contained sessions:\n";

        for (const auto& fn_json : data["session_file_names"])
        {
            const std::string fn = fn_json["session_file_name"];
            _project_settings.session_file_names.push_back(fn);
            std::cout << "'" << fn << "'";
            if (!fs::exists(fn))
                std::cout << "  (WARNING: session file does not exist! Please manually adapt path)";
            std::cout << "\n";
        }

        edges.clear();
        for (const auto& edge_json : data["loop_closure_edges"])
        {
            Edge edge;
            edge.index_from = edge_json["index_from"];
            edge.index_to = edge_json["index_to"];
            edge.is_fixed_fi = edge_json["is_fixed_fi"];
            edge.is_fixed_ka = edge_json["is_fixed_ka"];
            edge.is_fixed_om = edge_json["is_fixed_om"];
            edge.is_fixed_px = edge_json["is_fixed_px"];
            edge.is_fixed_py = edge_json["is_fixed_py"];
            edge.is_fixed_pz = edge_json["is_fixed_pz"];
            edge.relative_pose_tb.fi = edge_json["fi"];
            edge.relative_pose_tb.ka = edge_json["ka"];
            edge.relative_pose_tb.om = edge_json["om"];
            edge.relative_pose_tb.px = edge_json["px"];
            edge.relative_pose_tb.py = edge_json["py"];
            edge.relative_pose_tb.pz = edge_json["pz"];
            edge.relative_pose_tb_weights.fi = edge_json["w_fi"];
            edge.relative_pose_tb_weights.ka = edge_json["w_ka"];
            edge.relative_pose_tb_weights.om = edge_json["w_om"];
            edge.relative_pose_tb_weights.px = edge_json["w_px"];
            edge.relative_pose_tb_weights.py = edge_json["w_py"];
            edge.relative_pose_tb_weights.pz = edge_json["w_pz"];
            edge.index_session_from = edge_json["index_session_from"];
            edge.index_session_to = edge_json["index_session_to"];
            edges.push_back(edge);
        }

        std::cout << "Found " << edges.size() << "edges\nOpening done\n";

        return true;
    } catch (std::exception& e)
    {
        std::cout << "can't load project settings: " << e.what() << std::endl;
        return false;
    }

    std::string newTitle = winTitle + " - " + truncPath(file_name);
    SetWindowTitle(newTitle.c_str());

    loaded_sessions = false;
    time_stamp_offset = 0.0;

    return true;
}

void openProject()
{
    std::string input_file_name = "";
    input_file_name = mandeye::fd::OpenFileDialogOneFile("Open project", mandeye::fd::Project_filter);

    if (input_file_name.size() > 0)
    {
        loadProject(fs::path(input_file_name).string(), project_settings);
    }
}

void saveProject()
{
    std::string output_file_name = "";
    output_file_name = mandeye::fd::SaveFileDialog("Save project file", mandeye::fd::Project_filter, ".mjp", "project");

    if (output_file_name.size() > 0)
        if (save_project_settings(fs::path(output_file_name).string(), project_settings))
        {
            std::string newTitle = winTitle + " - " + truncPath(output_file_name);
            SetWindowTitle(newTitle.c_str());
        }
}

void addSession()
{
    auto input_file_names = mandeye::fd::OpenFileDialog("Add session(s)", mandeye::fd::Session_filter, true);

    if (input_file_names.size() > 0)
    {
        for (const auto& input_file_name : input_file_names)
        {
            std::cout << "Adding session file: '" << input_file_name << "'" << std::endl;
            project_settings.session_file_names.push_back(input_file_name);
        }

        loaded_sessions = false;
        time_stamp_offset = 0.0;
    }
}

void loadSessions()
{
    sessions.clear();
    for (const auto& ps : project_settings.session_file_names)
    {
        Session session;
        session.load(fs::path(ps).string(), is_decimate, bucket_x, bucket_y, bucket_z, calculate_offset);

        // making sure irelevant session specific settings that could affect rendering are off
        session.point_clouds_container.xz_intersection = false;
        session.point_clouds_container.yz_intersection = false;
        session.point_clouds_container.xy_intersection = false;
        session.point_clouds_container.xz_grid_10x10 = false;
        session.point_clouds_container.xz_grid_1x1 = false;
        session.point_clouds_container.xz_grid_01x01 = false;
        session.point_clouds_container.yz_grid_10x10 = false;
        session.point_clouds_container.yz_grid_1x1 = false;
        session.point_clouds_container.yz_grid_01x01 = false;
        session.point_clouds_container.xy_grid_10x10 = false;
        session.point_clouds_container.xy_grid_1x1 = false;
        session.point_clouds_container.xy_grid_01x01 = false;

        sessions.push_back(session);
        if (session.is_ground_truth)
            index_gt = sessions.size() - 1;
    }
    loaded_sessions = true;

    // reorder
    std::vector<Session> sessions_reorder;
    std::vector<std::string> session_file_names_reordered;

    std::map<int, int> map_reorder;
    // project_settings.session_file_names.
    int new_index = 0;
    for (size_t i = 0; i < sessions.size(); i++)
    {
        if (sessions[i].is_ground_truth)
        {
            sessions_reorder.push_back(sessions[i]);
            session_file_names_reordered.push_back(project_settings.session_file_names[i]);
            map_reorder[i] = new_index++;
        }
    }
    for (size_t i = 0; i < sessions.size(); i++)
    {
        if (!sessions[i].is_ground_truth)
        {
            sessions_reorder.push_back(sessions[i]);
            session_file_names_reordered.push_back(project_settings.session_file_names[i]);
            map_reorder[i] = new_index++;
        }
    }
    sessions = sessions_reorder;
    project_settings.session_file_names = session_file_names_reordered;

    for (auto& e : edges)
    {
        e.index_session_from = map_reorder[e.index_session_from];
        e.index_session_to = map_reorder[e.index_session_to];
    }

    std::cout << "sessions reordered, ground truth should be in front" << std::endl;
    for (const auto& s : sessions)
    {
        std::cout << "session: '" << s.session_file_name << "' ground truth [" << int(s.is_ground_truth) << "]" << std::endl;
    }

    // update time_stamp_offset
    std::cout << "update time_stamp_offset" << std::endl;
    for (const auto& s : sessions)
    {
        if (s.point_clouds_container.point_clouds.size() > 0)
        {
            if (s.point_clouds_container.point_clouds[0].local_trajectory.size() > 0)
            {
                if (s.point_clouds_container.point_clouds[0].local_trajectory[0].timestamps.first > time_stamp_offset)
                {
                    time_stamp_offset = s.point_clouds_container.point_clouds[0].local_trajectory[0].timestamps.first;
                }
            }
        }
    }
}

void generate_loop_closures(const std::vector<Session>& sessions, std::vector<Edge>& edges)
{
    edges.clear();
    // Implementation for generating loop closures

    // bool found_edge = false;

    for (int i1 = 0; i1 < sessions.size(); i1++)
    {
        for (int j1 = 0; j1 < sessions[i1].point_clouds_container.point_clouds.size(); j1++)
        {
            Eigen::Affine3d pose_i = sessions[i1].point_clouds_container.point_clouds[j1].m_pose;

            for (int i2 = i1 + 1; i2 < sessions.size(); i2++)
            {
                for (int j2 = 0; j2 < sessions[i2].point_clouds_container.point_clouds.size(); j2++)
                {
                    Eigen::Affine3d pose_j = sessions[i2].point_clouds_container.point_clouds[j2].m_pose;

                    if ((pose_i.translation() - pose_j.translation()).norm() < 10.0) // Example threshold for proximity
                    {
                        // found_edge = true;
                        //  Create a loop closure edge between pose_i and pose_j
                        /*Edge edge;
                        edge.index_session_from = i1;
                        edge.index_from = j1;
                        edge.index_session_to = i2;
                        edge.index_to = j2;

                            // Initialize other edge parameters as needed
                            edges.push_back(edge);*/

                        Edge edge;

                        edge.index_session_from = i1;
                        edge.index_session_to = i2;

                        edge.index_from = j1;
                        edge.index_to = j2;

                        std::cout << "Found loop closure edge between session " << i1 << " (pose " << j1 << ") and session " << i2
                                  << " (pose " << j2 << ")" << std::endl;

                        edge.relative_pose_tb = pose_tait_bryan_from_affine_matrix(
                            sessions[edge.index_session_from].point_clouds_container.point_clouds[edge.index_from].m_pose.inverse() *
                            sessions[edge.index_session_to].point_clouds_container.point_clouds[edge.index_to].m_pose);

                        edge.relative_pose_tb_weights.px = 1.0;
                        edge.relative_pose_tb_weights.py = 1.0;
                        edge.relative_pose_tb_weights.pz = 1.0;
                        edge.relative_pose_tb_weights.om = 1.0;
                        edge.relative_pose_tb_weights.fi = 1.0;
                        edge.relative_pose_tb_weights.ka = 1.0;

                        edges.push_back(edge);

                        // j1 += 20;
                        // j2 += 20;
                    }
                }
            } // for(int i2 = i1 + 1; i2 < sessions.size(); i2++)
        } // for(int j1 = 0; j1 < sessions[i1].point_clouds_container.point_clouds.size(); j1++)
    } // for(int i1 = 0; i1 < sessions.size(); i1++)
}

void icp_all_edges(std::vector<Session>& sessions, std::vector<Edge>& edges, float sr)
{
    std::cout << "icp_all_edges" << std::endl;

    int number_of_iterations = 10;

    for (size_t index_active_edge = 0; index_active_edge < edges.size(); index_active_edge++)
    {
        PairWiseICP icp;
        auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

        std::vector<Eigen::Vector3d> source;
        auto& e = edges[index_active_edge];
        for (int i = -num_edge_extended_before; i <= num_edge_extended_after; i++)
        {
            int index_src = e.index_to + i;
            if (index_src >= 0 &&
                index_src < sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds.size())
            {
                Eigen::Affine3d m_src =
                    sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds.at(index_src).m_pose;
                for (int k = 0; k <
                     sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds[index_src].points_local.size();
                     k++)
                {
                    Eigen::Vector3d p_g = m_src *
                        sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds[index_src].points_local[k];
                    source.push_back(p_g);
                }
                // point_clouds_container.point_clouds.at(index_src).render(m_src, 1);
            }
        }
        std::vector<Eigen::Vector3d> target;

        for (int i = -num_edge_extended_before; i <= num_edge_extended_after; i++)
        {
            int index_trg = e.index_from + i;
            if (index_trg >= 0 &&
                index_trg < sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds.size())
            {
                Eigen::Affine3d m_trg =
                    sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds.at(index_trg).m_pose;
                for (int k = 0; k < sessions[edges[index_active_edge].index_session_from]
                                        .point_clouds_container.point_clouds[index_trg]
                                        .points_local.size();
                     k++)
                {
                    Eigen::Vector3d p_g = m_trg *
                        sessions[edges[index_active_edge].index_session_from]
                            .point_clouds_container.point_clouds[index_trg]
                            .points_local[k];
                    target.push_back(p_g);
                }
                // point_clouds_container.point_clouds.at(index_src).render(m_src, 1);
            }
        }

        Eigen::Affine3d m_src_inv =
            sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds[e.index_to].m_pose.inverse();

        for (auto& p : source)
        {
            p = m_src_inv * p;
        }

        Eigen::Affine3d m_trg_inv =
            sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[e.index_from].m_pose.inverse();

        for (auto& p : target)
        {
            p = m_trg_inv * p;
        }

        if (icp.compute(source, target, sr, number_of_iterations, m_pose))
            edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
    }
}

void settings_gui()
{
    if (ImGui::Begin("Settings", &is_settings_gui))
    {
        ImGui::Checkbox("Downsample during load", &is_decimate);
        ImGui::SameLine();
        ImGui::Text("Bucket [m]:");
        ImGui::PushItemWidth(ImGuiNumberWidth);
        ImGui::InputDouble("X##b", &bucket_x, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(xText);
        ImGui::SameLine();
        ImGui::InputDouble("Y##b", &bucket_y, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(yText);
        ImGui::SameLine();
        ImGui::InputDouble("Z##b", &bucket_z, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(zText);
        ImGui::PopItemWidth();

        ImGui::NewLine();

        ImGui::Separator();

        ImGui::Text("Benchmark settings:");

        ImGui::PushItemWidth(ImGuiNumberWidth * 2);

        static double fast_plus = 100000000.0;
        static double fast_plus_plus = 1000000000.0;

        ImGui::InputDouble("Increment", &fast_plus);
        ImGui::InputDouble("Fast increment", &fast_plus_plus);
        ImGui::InputDouble("Timestamp offset", &time_stamp_offset, fast_plus, fast_plus_plus);
        ImGui::PopItemWidth();
        ImGui::SameLine();
        if (ImGui::Button("Set to origin"))
        {
            bool is_first_gt = false;

            if (sessions.size() > 0)
            {
                if (sessions[0].is_ground_truth)
                    is_first_gt = true;
            }

            Eigen::Affine3d m_gt = Eigen::Affine3d::Identity();
            if (sessions.size() > 0)
            {
                int index_point_clouds = -1;
                int index_local_trajectory = -1;
                bool found = false;
                for (size_t a = 0; a < sessions[0].point_clouds_container.point_clouds.size(); a++)
                {
                    for (size_t b = 0; b < sessions[0].point_clouds_container.point_clouds[a].local_trajectory.size(); b++)
                    {
                        if (sessions[0].point_clouds_container.point_clouds[a].local_trajectory[b].timestamps.first > time_stamp_offset)
                        {
                            if (!found)
                            {
                                found = true;
                                index_point_clouds = a;
                                index_local_trajectory = b;
                                break;
                            }
                        }
                    }
                }

                if (index_point_clouds != -1 && index_local_trajectory != -1)
                {
                    m_gt = sessions[0].point_clouds_container.point_clouds[index_point_clouds].m_pose *
                        sessions[0].point_clouds_container.point_clouds[index_point_clouds].local_trajectory[index_local_trajectory].m_pose;
                }
            }

            // for (auto& session : sessions)
            for (auto& session : sessions)
            {
                if (is_first_gt)
                {
                    if (session.is_ground_truth)
                        continue;
                }

                int index_point_clouds = -1;
                int index_local_trajectory = -1;
                bool found = false;
                for (size_t a = 0; a < session.point_clouds_container.point_clouds.size(); a++)
                {
                    for (size_t b = 0; b < session.point_clouds_container.point_clouds[a].local_trajectory.size(); b++)
                    {
                        if (session.point_clouds_container.point_clouds[a].local_trajectory[b].timestamps.first > time_stamp_offset)
                        {
                            if (!found)
                            {
                                found = true;
                                index_point_clouds = a;
                                index_local_trajectory = b;
                                break;
                            }
                        }
                    }
                }

                if (index_point_clouds != -1 && index_local_trajectory != -1)
                {
                    auto m1 = session.point_clouds_container.point_clouds[index_point_clouds].m_pose;
                    auto m2 =
                        session.point_clouds_container.point_clouds[index_point_clouds].local_trajectory[index_local_trajectory].m_pose;

                    auto inv = Eigen::Affine3d::Identity();
                    inv = (m1 * m2).inverse();
                    for (size_t index = 0; index < session.point_clouds_container.point_clouds.size(); index++)
                        session.point_clouds_container.point_clouds[index].m_pose =
                            inv * session.point_clouds_container.point_clouds[index].m_pose;

                    for (size_t index = 0; index < session.point_clouds_container.point_clouds.size(); index++)
                        session.point_clouds_container.point_clouds[index].m_pose =
                            m_gt * session.point_clouds_container.point_clouds[index].m_pose;
                }
            }
        }

        static bool open_import_popup = false;
        static bool show_instruction = false;

        ImGui::Dummy(ImVec2(0, 10));

        if (ImGui::Button("Import Benchmark Output Folders"))
        {
            open_import_popup = true;
            ImGui::OpenPopup("Import Benchmark");
        }

        if (ImGui::BeginPopupModal("Import Benchmark", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("Import benchmark sessions");
            ImGui::Separator();
            ImGui::SetNextWindowSize(ImVec2(740, 340), ImGuiCond_Once);
            if (ImGui::Button("Instruction"))
            {
                show_instruction = !show_instruction;
            }

            ImGui::Dummy(ImVec2(0, 10));
            if (ImGui::Button("Select Folder"))
            {
                const std::string algorithms[] = { "ct-icp",    "glim",      "super-lio", "dlio",
                                                   "i2ekf-lo",  "superOdom", "lego-loam", "faster-lio",
                                                   "kiss-icp",  "fast-lio",  "lio-ekf",   "genz-icp",
                                                   "point-lio", "ig-lio",    "dlo",       "lidar_odometry_ros_wrapper" };

                std::vector<fs::path> missing;
                std::unordered_map<std::string, std::string> algo_map;

                for (const auto& algo : algorithms)
                {
                    if (algo == "kiss-icp")
                    {
                        algo_map[algo] = "output_hdmapping-kiss";
                    }
                    else if (algo == "genz-icp")
                    {
                        algo_map[algo] = "output_hdmapping-genz";
                    }
                    else if (algo == "lidar_odometry_ros_wrapper")
                    {
                        algo_map[algo] = "output_hdmapping-lidar-odometry-ros";
                    }
                    else
                    {
                        algo_map[algo] = "output_hdmapping-" + algo;
                    }
                }

                fs::path path = fs::path(mandeye::fd::SelectFolder("Add sessions"));

                for (const auto& algo : algorithms)
                {
                    fs::path output_folder = path / algo / algo_map[algo];
                    fs::path session_file = output_folder / "session.json";

                    if (fs::is_directory(output_folder))
                    {
                        auto it =
                            std::find(project_settings.session_file_names.begin(), project_settings.session_file_names.end(), session_file);

                        if (it == project_settings.session_file_names.end())
                        {
                            std::cout << "Adding session file: '" << session_file << "'" << std::endl;
                            project_settings.session_file_names.push_back(session_file.string());
                        }
                    }
                    else
                    {
                        missing.push_back(output_folder);
                    }
                }

                for (const auto& miss : missing)
                {
                    std::cout << miss << " doesn't exist" << std::endl;
                }
            }

            ImGui::SameLine();

            if (ImGui::Button("Close"))
            {
                ImGui::CloseCurrentPopup();
                show_instruction = false;
            }

            if (show_instruction)
            {
                ImGui::BeginChild("InstructionChild", ImVec2(720, 300), true, ImGuiWindowFlags_HorizontalScrollbar);
                ImGui::Separator();

                ImGui::TextWrapped("Required folder structure:");

                ImGui::Spacing();

                ImGui::TextWrapped("Folders must follow the structure generated in benchmark-HDMapping-Orchestration (step 3):");
                ImGui::TextWrapped("https://github.com/MapsHD/benchmark-HDMapping-Orchestration");

                ImGui::Spacing();
                ImGui::Separator();

                ImGui::BulletText("chosen_folder/");
                ImGui::BulletText("  ct-icp/output_hdmapping-ct-icp/session.json");
                ImGui::BulletText("  glim/output_hdmapping-glim/session.json");
                ImGui::BulletText("  kiss-icp/output_hdmapping-kiss/session.json");
                ImGui::BulletText("  fast-lio/output_hdmapping-fast-lio/session.json");
                ImGui::BulletText("  ... (same pattern for other algorithms)");

                ImGui::Spacing();
                ImGui::EndChild();
            }

            ImGui::EndPopup();
        }
        if (project_settings.session_file_names.size() > 0)
        {
            ImGui::Separator();

            ImGui::Text("Sessions:");

            for (size_t i = 0; i < project_settings.session_file_names.size(); i++)
            {
                ImGui::Text(truncPath(project_settings.session_file_names[i]).c_str());
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(project_settings.session_file_names[i].c_str());

                if (project_settings.session_file_names.size() == sessions.size())
                {
                    ImGui::BeginDisabled(is_loop_closure_gui);
                    {
                        ImGui::SameLine();
                        ImGui::Checkbox(("Visible##" + std::to_string(i)).c_str(), &sessions[i].visible);
                    }
                    ImGui::EndDisabled();

                    ImGui::SameLine();
                    if (ImGui::RadioButton(("Ground truth##" + std::to_string(i)).c_str(), &index_gt, i))
                        if (old_index_gt == i)
                            index_gt = -1; // unselect
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Select session as unmovable reference");

                    ImGui::BeginDisabled(!sessions[i].visible);
                    {
                        ImGui::BeginDisabled(sessions[i].is_ground_truth);
                        {
                            ImGui::SameLine();
                            if (ImGui::RadioButton(("Gizmo##" + std::to_string(i)).c_str(), &index_gizmo, i))
                                if (old_index_gizmo == i)
                                    index_gizmo = -1; // unselect
                        }
                        ImGui::EndDisabled();

                        ImGui::SameLine();
                        ImGui::ColorEdit3(
                            ("Color##" + std::to_string(i)).c_str(), (float*)&sessions[i].render_color, ImGuiColorEditFlags_NoInputs);
                        for (auto& pc : sessions[i].point_clouds_container.point_clouds)
                        {
                            pc.traj_color[0] = sessions[i].render_color[0];
                            pc.traj_color[1] = sessions[i].render_color[1];
                            pc.traj_color[2] = sessions[i].render_color[2];
                            pc.render_color[0] = sessions[i].render_color[0];
                            pc.render_color[1] = sessions[i].render_color[1];
                            pc.render_color[2] = sessions[i].render_color[2];
                        }
                    }
                    ImGui::EndDisabled();

                    //
                    if (sessions[i].point_clouds_container.point_clouds.size() > 0)
                    {
                        if (sessions[i].point_clouds_container.point_clouds[0].local_trajectory.size() > 0)
                        {
                            if (sessions[i]
                                    .point_clouds_container.point_clouds[sessions[i].point_clouds_container.point_clouds.size() - 1]
                                    .local_trajectory.size() > 0)
                            {
                                ImGui::SameLine();

                                int index_last = sessions[i].point_clouds_container.point_clouds.size() - 1;
                                int index_last2 = sessions[i].point_clouds_container.point_clouds[index_last].local_trajectory.size() - 1;

                                ImGui::Text(
                                    "Timestamp range: <%.0f, %.0f>",
                                    sessions[i].point_clouds_container.point_clouds[0].local_trajectory[0].timestamps.first,
                                    sessions[i]
                                        .point_clouds_container.point_clouds[index_last]
                                        .local_trajectory[index_last2]
                                        .timestamps.first);
                            }
                        }
                    }
                }
            }

            if (project_settings.session_file_names.size() == sessions.size())
            {
                if ((old_index_gt != index_gt) || (old_index_gizmo != index_gizmo))
                {
                    for (size_t i = 0; i < sessions.size(); i++)
                    {
                        sessions[i].is_ground_truth = (i == index_gt);
                        sessions[i].is_gizmo = (i == index_gizmo);
                    }

                    old_index_gt = index_gt;
                    old_index_gizmo = index_gizmo;
                }

                if (index_gizmo != -1 && index_gizmo < sessions.size())
                {
                    // sessions[index_gizmo].is_gizmo = true;
                    m_gizmo[0] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(0, 0);
                    m_gizmo[1] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(1, 0);
                    m_gizmo[2] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(2, 0);
                    m_gizmo[3] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(3, 0);
                    m_gizmo[4] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(0, 1);
                    m_gizmo[5] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(1, 1);
                    m_gizmo[6] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(2, 1);
                    m_gizmo[7] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(3, 1);
                    m_gizmo[8] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(0, 2);
                    m_gizmo[9] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(1, 2);
                    m_gizmo[10] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(2, 2);
                    m_gizmo[11] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(3, 2);
                    m_gizmo[12] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(0, 3);
                    m_gizmo[13] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(1, 3);
                    m_gizmo[14] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(2, 3);
                    m_gizmo[15] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(3, 3);
                }
            }

            ImGui::BeginDisabled((project_settings.session_file_names.size() < 2) || (index_gizmo == -1));
            {
                ImGui::Checkbox("Gizmo all sessions", &gizmo_all_sessions);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Gizmo will move all sessions except ground truth one");
            }
            ImGui::EndDisabled();

            ImGui::Separator();

            ImGui::NewLine();

            if (project_settings.session_file_names.size() == sessions.size())
            {
                number_visible_sessions = 0;

                bool first_session_index_found = false;
                for (size_t index = 0; index < sessions.size(); index++)
                {
                    if (sessions[index].visible)
                    {
                        number_visible_sessions++;
                        if (!first_session_index_found)
                        {
                            first_session_index = index;
                            second_session_index = index;
                            first_session_index_found = true;
                        }
                        else
                        {
                            second_session_index = index;
                        }
                    }
                }

                if (!is_loop_closure_gui)
                {
                    static int nr_iter = 100;
                    ImGui::SetNextItemWidth(ImGuiNumberWidth);
                    ImGui::InputInt("Number of iterations", &nr_iter);
                    if (nr_iter < 1)
                        nr_iter = 1;

                    ImGui::InputDouble("Motion Model Weight (1 sigma m): position x [px]", &motion_model_weights.px, 0.0, 0.0, "%.3f");
                    ImGui::InputDouble("Motion Model Weight (1 sigma m): position y [py]", &motion_model_weights.py, 0.0, 0.0, "%.3f");
                    ImGui::InputDouble("Motion Model Weight (1 sigma m): position z [pz]", &motion_model_weights.pz, 0.0, 0.0, "%.3f");
                    ImGui::InputDouble(
                        "Motion Model Weight (1 sigma degree): orientation om [om]", &motion_model_weights.om, 0.0, 0.0, "%.3f");
                    ImGui::InputDouble(
                        "Motion Model Weight (1 sigma degree): orientation fi [fi]", &motion_model_weights.fi, 0.0, 0.0, "%.3f");
                    ImGui::InputDouble(
                        "Motion Model Weight (1 sigma degree): orientation ka [ka]", &motion_model_weights.ka, 0.0, 0.0, "%.3f");

                    std::string bn = "Optimize (number of iterations: " + std::to_string(nr_iter) + ")";

                    if (ImGui::Button(bn.c_str()))
                    {
                        for (int i = 0; i < nr_iter; i++)
                        {
                            std::cout << "Iteration [" << i + 1 << "] of: " << nr_iter << std::endl;
                            optimize(sessions, edges, motion_model_weights);
                        }
                        optimized = true;
                    }

                    // if (optimized)
                    //{
                    ImGui::SameLine();
                    if (ImGui::Button("Revert"))
                        revert(sessions);
                    ImGui::SameLine();
                    if (ImGui::Button("Save results"))
                        save_results(sessions);
                    ImGui::SameLine();
                    if (ImGui::Button("Revert to initial"))
                        revert_to_initial(sessions);
                    //}

                    if (ImGui::Button("Generate loop closures"))
                    {
                        generate_loop_closures(sessions, edges);
                    }

                    if (ImGui::Button("ICP all edges"))
                    {
                        icp_all_edges(sessions, edges, search_radius);
                    }
                    ImGui::SameLine();
                    ImGui::InputDouble("search_radius", &search_radius);
                    if (search_radius < 0.01)
                        search_radius = 0.01;
                }

                // if (!is_loop_closure_gui && prev_is_loop_closure_gui)
                //{
                //     exit(1);
                // }
            }
        }
    }

    ImGui::End();
}

void display()
{
    syncSessionRenderers();

    ImGuiIO& io = ImGui::GetIO();
    // Framebuffer pixels, not io.DisplaySize (they differ on HiDPI) -- see step 2's display().
    rlViewport(0, 0, GetRenderWidth(), GetRenderHeight());

    ClearBackground(ColorFromNormalized(
        Vector4{ app_state.bg_color.x * app_state.bg_color.w,
                 app_state.bg_color.y * app_state.bg_color.w,
                 app_state.bg_color.z * app_state.bg_color.w,
                 app_state.bg_color.w }));
    rlEnableDepthTest();

    rlMatrixMode(RL_PROJECTION);
    rlLoadIdentity();
    float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);

    auto& camera = app_state.camera;
    camera.updateEulerTransition(io.DeltaTime);

    app_state.viewLocal = Eigen::Affine3f::Identity();

    if (!camera.isOrtho)
    {
        camera.applyPerspectiveProjection((int)io.DisplaySize.x, (int)io.DisplaySize.y);

        // In loop closure mode the rotation center follows the source scan / active edge (when enabled).
        if (is_loop_closure_gui && update_rotation_center)
        {
            auto follow = [&](const Eigen::Vector3d& t)
            {
                camera.euler.rotationCenter = Vector3{ static_cast<float>(t.x()), static_cast<float>(t.y()), static_cast<float>(t.z()) };
                camera.eulerGoal.rotationCenter = camera.euler.rotationCenter;
            };

            if (validScan(first_session_index, index_loop_closure_source))
                follow(sessions[first_session_index].point_clouds_container.point_clouds[index_loop_closure_source].m_pose.translation());

            if (manipulate_active_edge && index_active_edge >= 0 && index_active_edge < static_cast<int>(edges.size()))
            {
                const auto& e = edges[index_active_edge];
                if (validScan(e.index_session_from, e.index_from))
                    follow(sessions[e.index_session_from].point_clouds_container.point_clouds[e.index_from].m_pose.translation());
            }
        }

        Eigen::Vector3f rotationCenter(camera.euler.rotationCenter.x, camera.euler.rotationCenter.y, camera.euler.rotationCenter.z);
        app_state.viewLocal.translate(rotationCenter);
        app_state.viewLocal.translate(Eigen::Vector3f(camera.euler.translate.x, camera.euler.translate.y, camera.euler.translate.z));
        if (!camera.lockZ)
            app_state.viewLocal.rotate(Eigen::AngleAxisf(camera.euler.rotateX * DEG_TO_RAD, Eigen::Vector3f::UnitX()));
        else
            app_state.viewLocal.rotate(Eigen::AngleAxisf(-90.0 * DEG_TO_RAD, Eigen::Vector3f::UnitX()));
        app_state.viewLocal.rotate(Eigen::AngleAxisf(camera.euler.rotateY * DEG_TO_RAD, Eigen::Vector3f::UnitZ()));
        app_state.viewLocal.translate(-rotationCenter);

        rlMultMatrixf(app_state.viewLocal.matrix().data());
    }
    else
    {
        app_state.viewLocal.rotate(Eigen::AngleAxisf((camera.euler.rotateX + camera.euler.rotateY) * DEG_TO_RAD, Eigen::Vector3f::UnitZ()));
        camera.updateOrtho(ratio);
    }

    camera.captureFrameMatrices();
    frame_mvp_3d = MatrixMultiply(camera.frameView3D, camera.frameProj3D);

    showAxes();

    if (is_loop_closure_gui)
    {
        // Scans within [index - before, index + after] of the loop closure source/target.
        auto in_range = [](int center)
        {
            return [center](int i)
            {
                return i >= center - num_edge_extended_before && i <= center + num_edge_extended_after;
            };
        };

        const bool edge_ok = manipulate_active_edge && index_active_edge >= 0 && index_active_edge < static_cast<int>(edges.size()) &&
            validScan(edges[index_active_edge].index_session_from, edges[index_active_edge].index_from);

        if (edge_ok && validScan(first_session_index, index_loop_closure_source) &&
            validScan(second_session_index, index_loop_closure_target))
        {
            // Preview the active edge: the source range placed at the edge's source pose, the target range
            // at source * relative_pose. Like the GLUT version, the ranges are taken around
            // index_loop_closure_source/target of the first/second visible session.
            const auto& e = edges[index_active_edge];
            Eigen::Affine3d edge_src = sessions[e.index_session_from].point_clouds_container.point_clouds[e.index_from].m_pose;
            Eigen::Affine3d edge_trg = edge_src * affine_matrix_from_pose_tait_bryan(e.relative_pose_tb);

            const auto& first_pcs = sessions[first_session_index].point_clouds_container.point_clouds;
            Eigen::Affine3d src_0 = first_pcs[index_loop_closure_source].m_pose;
            for (int i = index_loop_closure_source - num_edge_extended_before; i <= index_loop_closure_source + num_edge_extended_after;
                 i++)
                if (i >= 0 && i < static_cast<int>(first_pcs.size()))
                    drawScanAtPose(first_session_index, i, edge_src * (src_0.inverse() * first_pcs[i].m_pose), first_pcs[i].render_color);

            const auto& second_pcs = sessions[second_session_index].point_clouds_container.point_clouds;
            Eigen::Affine3d trg_0 = second_pcs[index_loop_closure_target].m_pose;
            for (int i = index_loop_closure_target - num_edge_extended_before; i <= index_loop_closure_target + num_edge_extended_after;
                 i++)
                if (i >= 0 && i < static_cast<int>(second_pcs.size()))
                    drawScanAtPose(
                        second_session_index, i, edge_trg * (trg_0.inverse() * second_pcs[i].m_pose), second_pcs[i].render_color);
        }
        else if (!manipulate_active_edge)
        {
            if (first_session_index >= 0)
                drawSession(first_session_index, in_range(index_loop_closure_source));
            if (second_session_index >= 0 && second_session_index != first_session_index)
                drawSession(second_session_index, in_range(index_loop_closure_target));
            else if (second_session_index >= 0)
                drawSession(
                    second_session_index,
                    [&](int i)
                    {
                        return in_range(index_loop_closure_source)(i) || in_range(index_loop_closure_target)(i);
                    });
        }

        for (int s : { first_session_index, second_session_index })
            if (s >= 0 && s < static_cast<int>(sessions.size()))
                drawPosePolyline(sessions[s]);

        for (size_t i = 0; i < sessions.size(); i++)
        {
            const auto& pcs = sessions[i].point_clouds_container.point_clouds;
            for (const auto& pg_edge : sessions[i].pose_graph_loop_closure.edges)
                if (validScan(static_cast<int>(i), pg_edge.index_from) && validScan(static_cast<int>(i), pg_edge.index_to))
                    drawEdge(pcs[pg_edge.index_from].m_pose.translation(), pcs[pg_edge.index_to].m_pose.translation(), 0.f, 0.f, 1.f);
        }

        for (const auto& e : edges)
        {
            if (!validScan(e.index_session_from, e.index_from) || !validScan(e.index_session_to, e.index_to))
                continue;
            bool gt = sessions[e.index_session_from].is_ground_truth || sessions[e.index_session_to].is_ground_truth;
            drawEdge(
                sessions[e.index_session_from].point_clouds_container.point_clouds[e.index_from].m_pose.translation(),
                sessions[e.index_session_to].point_clouds_container.point_clouds[e.index_to].m_pose.translation(),
                gt ? 0.f : 1.f, // cyan with a ground truth session, otherwise yellow
                1.f,
                gt ? 1.f : 0.f);
        }
    }
    else
    {
        for (size_t s = 0; s < sessions.size(); s++)
        {
            auto& session = sessions[s];
            if (!session.visible)
                continue;

            drawSession(s);
            renderGroundControlPoints(session.ground_control_points, session.point_clouds_container);
            renderControlPoints(session.control_points, session.point_clouds_container);

            // +-5 m cross at the session's first trajectory node after time_stamp_offset.
            const auto& pcs = session.point_clouds_container.point_clouds;
            bool found = false;
            for (size_t a = 0; a < pcs.size() && !found; a++)
            {
                for (size_t b = 0; b < pcs[a].local_trajectory.size(); b++)
                {
                    if (pcs[a].local_trajectory[b].timestamps.first > time_stamp_offset)
                    {
                        found = true;
                        Eigen::Vector3d v1 = (pcs[a].m_pose * pcs[a].local_trajectory[b].m_pose).translation();
                        rlBegin(RL_LINES);
                        rlColor3f(pcs[a].render_color[0], pcs[a].render_color[1], pcs[a].render_color[2]);
                        vertex(v1 - Eigen::Vector3d(5, 0, 0));
                        vertex(v1 + Eigen::Vector3d(5, 0, 0));
                        vertex(v1 - Eigen::Vector3d(0, 5, 0));
                        vertex(v1 + Eigen::Vector3d(0, 5, 0));
                        vertex(v1 - Eigen::Vector3d(0, 0, 5));
                        vertex(v1 + Eigen::Vector3d(0, 0, 5));
                        rlEnd();
                        break;
                    }
                }
            }
        }
    }

    // rlImGuiBegin() only feeds input to ImGui and starts its frame; it leaves the rlgl 3D matrices
    // active, so the gizmo code below still sees this frame's camera.
    rlImGuiBegin();

    ShowMainDockSpace();

    if (!is_loop_closure_gui)
    {
        Eigen::Affine3d prev_pose_manipulated = Eigen::Affine3d::Identity();
        Eigen::Affine3d prev_pose_after_gismo = Eigen::Affine3d::Identity();

        for (size_t i = 0; i < sessions.size(); i++)
        {
            // guizmo_all_sessions;
            if (sessions[i].is_gizmo && !sessions[i].is_ground_truth)
            {
                if (sessions[i].point_clouds_container.point_clouds.size() > 0)
                {
                    prev_pose_manipulated = sessions[i].point_clouds_container.point_clouds[0].m_pose;
                    std::vector<Eigen::Affine3d> all_m_poses;
                    for (size_t j = 0; j < sessions[i].point_clouds_container.point_clouds.size(); j++)
                        all_m_poses.push_back(sessions[i].point_clouds_container.point_clouds[j].m_pose);

                    ImGuiIO& io = ImGui::GetIO();

                    ImGuizmo::BeginFrame();
                    ImGuizmo::Enable(true);
                    ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

                    manipulateGizmo();

                    sessions[i].point_clouds_container.point_clouds[0].m_pose = Eigen::Map<const Eigen::Matrix4f>(m_gizmo).cast<double>();
                    prev_pose_after_gismo = sessions[i].point_clouds_container.point_clouds[0].m_pose;
                    sessions[i].point_clouds_container.point_clouds[0].pose =
                        pose_tait_bryan_from_affine_matrix(sessions[i].point_clouds_container.point_clouds[0].m_pose);

                    sessions[i].point_clouds_container.point_clouds[0].gui_translation[0] =
                        (float)sessions[i].point_clouds_container.point_clouds[0].pose.px;
                    sessions[i].point_clouds_container.point_clouds[0].gui_translation[1] =
                        (float)sessions[i].point_clouds_container.point_clouds[0].pose.py;
                    sessions[i].point_clouds_container.point_clouds[0].gui_translation[2] =
                        (float)sessions[i].point_clouds_container.point_clouds[0].pose.pz;

                    sessions[i].point_clouds_container.point_clouds[0].gui_rotation[0] =
                        (float)(sessions[i].point_clouds_container.point_clouds[0].pose.om * RAD_TO_DEG);
                    sessions[i].point_clouds_container.point_clouds[0].gui_rotation[1] =
                        (float)(sessions[i].point_clouds_container.point_clouds[0].pose.fi * RAD_TO_DEG);
                    sessions[i].point_clouds_container.point_clouds[0].gui_rotation[2] =
                        (float)(sessions[i].point_clouds_container.point_clouds[0].pose.ka * RAD_TO_DEG);

                    Eigen::Affine3d curr_m_pose = sessions[i].point_clouds_container.point_clouds[0].m_pose;
                    for (size_t j = 1; j < sessions[i].point_clouds_container.point_clouds.size(); j++)
                    {
                        curr_m_pose = curr_m_pose * (all_m_poses[j - 1].inverse() * all_m_poses[j]);
                        sessions[i].point_clouds_container.point_clouds[j].m_pose = curr_m_pose;
                        sessions[i].point_clouds_container.point_clouds[j].pose =
                            pose_tait_bryan_from_affine_matrix(sessions[i].point_clouds_container.point_clouds[j].m_pose);

                        sessions[i].point_clouds_container.point_clouds[j].gui_translation[0] =
                            (float)sessions[i].point_clouds_container.point_clouds[j].pose.px;
                        sessions[i].point_clouds_container.point_clouds[j].gui_translation[1] =
                            (float)sessions[i].point_clouds_container.point_clouds[j].pose.py;
                        sessions[i].point_clouds_container.point_clouds[j].gui_translation[2] =
                            (float)sessions[i].point_clouds_container.point_clouds[j].pose.pz;

                        sessions[i].point_clouds_container.point_clouds[j].gui_rotation[0] =
                            (float)(sessions[i].point_clouds_container.point_clouds[j].pose.om * RAD_TO_DEG);
                        sessions[i].point_clouds_container.point_clouds[j].gui_rotation[1] =
                            (float)(sessions[i].point_clouds_container.point_clouds[j].pose.fi * RAD_TO_DEG);
                        sessions[i].point_clouds_container.point_clouds[j].gui_rotation[2] =
                            (float)(sessions[i].point_clouds_container.point_clouds[j].pose.ka * RAD_TO_DEG);
                    }
                    //}
                }
            }
        }
        if (gizmo_all_sessions)
        {
            for (size_t i = 0; i < sessions.size(); i++)
            {
                // guizmo_all_sessions;
                if (!sessions[i].is_gizmo && !sessions[i].is_ground_truth)
                {
                    std::vector<Eigen::Affine3d> all_m_poses;
                    for (size_t j = 0; j < sessions[i].point_clouds_container.point_clouds.size(); j++)
                        all_m_poses.push_back(sessions[i].point_clouds_container.point_clouds[j].m_pose);

                    Eigen::Affine3d m_rel_org = prev_pose_manipulated.inverse() * sessions[i].point_clouds_container.point_clouds[0].m_pose;

                    Eigen::Affine3d m_new = prev_pose_after_gismo * m_rel_org;

                    sessions[i].point_clouds_container.point_clouds[0].m_pose = m_new;
                    sessions[i].point_clouds_container.point_clouds[0].pose =
                        pose_tait_bryan_from_affine_matrix(sessions[i].point_clouds_container.point_clouds[0].m_pose);

                    sessions[i].point_clouds_container.point_clouds[i].gui_translation[0] =
                        (float)sessions[i].point_clouds_container.point_clouds[0].pose.px;
                    sessions[i].point_clouds_container.point_clouds[i].gui_translation[1] =
                        (float)sessions[i].point_clouds_container.point_clouds[0].pose.py;
                    sessions[i].point_clouds_container.point_clouds[i].gui_translation[2] =
                        (float)sessions[i].point_clouds_container.point_clouds[0].pose.pz;

                    sessions[i].point_clouds_container.point_clouds[i].gui_rotation[0] =
                        (float)(sessions[i].point_clouds_container.point_clouds[0].pose.om * RAD_TO_DEG);
                    sessions[i].point_clouds_container.point_clouds[i].gui_rotation[1] =
                        (float)(sessions[i].point_clouds_container.point_clouds[0].pose.fi * RAD_TO_DEG);
                    sessions[i].point_clouds_container.point_clouds[i].gui_rotation[2] =
                        (float)(sessions[i].point_clouds_container.point_clouds[0].pose.ka * RAD_TO_DEG);

                    Eigen::Affine3d curr_m_pose = sessions[i].point_clouds_container.point_clouds[0].m_pose;
                    for (size_t j = 1; j < sessions[i].point_clouds_container.point_clouds.size(); j++)
                    {
                        curr_m_pose = curr_m_pose * (all_m_poses[j - 1].inverse() * all_m_poses[j]);
                        sessions[i].point_clouds_container.point_clouds[j].m_pose = curr_m_pose;
                        sessions[i].point_clouds_container.point_clouds[j].pose =
                            pose_tait_bryan_from_affine_matrix(sessions[i].point_clouds_container.point_clouds[j].m_pose);

                        sessions[i].point_clouds_container.point_clouds[j].gui_translation[0] =
                            (float)sessions[i].point_clouds_container.point_clouds[j].pose.px;
                        sessions[i].point_clouds_container.point_clouds[j].gui_translation[1] =
                            (float)sessions[i].point_clouds_container.point_clouds[j].pose.py;
                        sessions[i].point_clouds_container.point_clouds[j].gui_translation[2] =
                            (float)sessions[i].point_clouds_container.point_clouds[j].pose.pz;

                        sessions[i].point_clouds_container.point_clouds[j].gui_rotation[0] =
                            (float)(sessions[i].point_clouds_container.point_clouds[j].pose.om * RAD_TO_DEG);
                        sessions[i].point_clouds_container.point_clouds[j].gui_rotation[1] =
                            (float)(sessions[i].point_clouds_container.point_clouds[j].pose.fi * RAD_TO_DEG);
                        sessions[i].point_clouds_container.point_clouds[j].gui_rotation[2] =
                            (float)(sessions[i].point_clouds_container.point_clouds[j].pose.ka * RAD_TO_DEG);
                    }
                }
            }
        }
    }
    else
    {
        // ImGuizmo -----------------------------------------------
        if (edge_gizmo && edges.size() > 0)
        {
            ImGuizmo::BeginFrame();
            ImGuizmo::Enable(true);
            ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

            manipulateGizmo();

            Eigen::Affine3d m_g = Eigen::Affine3d::Identity();

            m_g.matrix() = Eigen::Map<const Eigen::Matrix4f>(m_gizmo).cast<double>();

            const int& index_src = edges[index_active_edge].index_from;

            const Eigen::Affine3d& m_src =
                sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds.at(index_src).m_pose;
            edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_src.inverse() * m_g);
        }
    }

    view_kbd_shortcuts();

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_A, false))
    {
        addSession();

        // workaround
        io.AddKeyEvent(ImGuiKey_A, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
    }
    if ((project_settings.session_file_names.size() > 0) && !loaded_sessions)
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_L, false))
        {
            loadSessions();

            // workaround
            io.AddKeyEvent(ImGuiKey_L, false);
            io.AddKeyEvent(ImGuiMod_Ctrl, false);
        }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_O, false))
    {
        openProject();

        // workaround
        io.AddKeyEvent(ImGuiKey_O, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
    }

    if (project_settings.session_file_names.size() > 0)
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_R, false))
        {
            remove_gui = true;

            // workaround
            io.AddKeyEvent(ImGuiKey_R, false);
            io.AddKeyEvent(ImGuiMod_Ctrl, false);
        }

    if (sessions.size() > 0)
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S, false))
        {
            saveProject();

            // workaround
            io.AddKeyEvent(ImGuiKey_S, false);
            io.AddKeyEvent(ImGuiMod_Ctrl, false);
        }

    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Open project", "Ctrl+O"))
                openProject();
            if (ImGui::MenuItem("Save project", "Ctrl+S", nullptr, project_settings.session_file_names.size() > 0))
                saveProject();

            ImGui::Separator();

            if (ImGui::MenuItem("Add session(s)", "Ctrl+A"))
                addSession();
            if (ImGui::MenuItem("Remove session(s)", "Ctrl+R", nullptr, project_settings.session_file_names.size() > 0))
                remove_gui = true;

            if (ImGui::MenuItem("Load sessions", "Ctrl+L", nullptr, (project_settings.session_file_names.size() > 0) && !loaded_sessions))
                loadSessions();

            ImGui::Separator();

            if (ImGui::BeginMenu("Save all marked trajectories", sessions.size() > 0))
            {
                if (ImGui::MenuItem("Save all as las/laz files"))
                {
                    for (size_t i = 0; i < project_settings.session_file_names.size(); ++i)
                    {
                        const auto& session_path = project_settings.session_file_names[i];

                        if (i >= sessions.size())
                        {
                            std::cerr << "No loaded session for: " << session_path << std::endl;
                            continue;
                        }

                        Session& session = sessions[i];

                        std::filesystem::path dir = std::filesystem::path(session_path).parent_path();
                        std::string folder_name = dir.filename().string();
                        std::string laz_path = (dir / (folder_name + "_trajectory_laz.laz")).string();

                        std::cout << "Saving trajectory to LAZ: " << laz_path << std::endl;

                        save_trajectories_to_laz(session, laz_path, 0.0f, 0.0f, false);
                    }

                    std::cout << "Finished saving all trajectories to .laz files." << std::endl;
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("As one global scan");

                ImGui::Separator();

                ImGui::Text("(x,y,z,r00,r01,r02,r10,r11,r12,r20,r21,r22)");
                if (ImGui::MenuItem("Save all as csv (timestamp Lidar)##1"))
                {
                    for (size_t i = 0; i < project_settings.session_file_names.size(); ++i)
                    {
                        const auto& session_path = project_settings.session_file_names[i];

                        if (i >= sessions.size())
                        {
                            std::cerr << "No loaded session for: " << session_path << std::endl;
                            continue;
                        }

                        Session& session = sessions[i];
                        std::filesystem::path dir = std::filesystem::path(session_path).parent_path();
                        std::string folder_name = dir.filename().string();
                        std::string csv_path = (dir / (folder_name + "_trajectory_timestampLidar_r.csv")).string();

                        std::cout << "Saving trajectory to CSV: " << csv_path << std::endl;

                        try
                        {
                            std::ofstream outfile(csv_path);
                            if (!outfile.is_open())
                            {
                                std::cerr << "Failed to create file: " << csv_path << std::endl;
                                continue;
                            }

                            outfile << "timestampLidar,x,y,z," << "r00,r01,r02," << "r10,r11,r12," << "r20,r21,r22\n";

                            for (const auto& pc : session.point_clouds_container.point_clouds)
                            {
                                if (!pc.visible)
                                    continue;

                                for (const auto& traj : pc.local_trajectory)
                                {
                                    Eigen::Affine3d pose = pc.m_pose * traj.m_pose;
                                    Eigen::Vector3d pos = pose.translation();
                                    Eigen::Matrix3d rot = pose.rotation();

                                    outfile << std::fixed << std::setprecision(0) << traj.timestamps.first << "," << std::setprecision(10)
                                            << pos.x() << "," << pos.y() << "," << pos.z() << "," << rot(0, 0) << "," << rot(0, 1) << ","
                                            << rot(0, 2) << "," << rot(1, 0) << "," << rot(1, 1) << "," << rot(1, 2) << "," << rot(2, 0)
                                            << "," << rot(2, 1) << "," << rot(2, 2) << "\n";
                                }
                            }

                            outfile.close();
                            std::cout << "Saved: " << csv_path << std::endl;
                        } catch (const std::exception& e)
                        {
                            std::cerr << "Error creating " << csv_path << ": " << e.what() << std::endl;
                        }
                    }

                    std::cout << "Finished saving all trajectories to CSV files." << std::endl;
                }
                if (ImGui::MenuItem("Save all as csv (timestamp Unix)##1"))
                {
                    for (size_t i = 0; i < project_settings.session_file_names.size(); ++i)
                    {
                        const auto& session_path = project_settings.session_file_names[i];
                        if (i >= sessions.size())
                        {
                            std::cerr << "No loaded session for: " << session_path << std::endl;
                            continue;
                        }

                        Session& session = sessions[i];
                        std::filesystem::path dir = std::filesystem::path(session_path).parent_path();
                        std::string folder_name = dir.filename().string();
                        std::string csv_path = (dir / (folder_name + "_trajectory_timestampUnix_r.csv")).string();

                        try
                        {
                            std::ofstream outfile(csv_path);
                            if (!outfile.is_open())
                            {
                                std::cerr << "Failed to create file: " << csv_path << std::endl;
                                continue;
                            }

                            outfile << "timestampUnix,x,y,z," << "r00,r01,r02,r10,r11,r12,r20,r21,r22\n";

                            for (const auto& pc : session.point_clouds_container.point_clouds)
                            {
                                if (!pc.visible)
                                    continue;
                                for (const auto& traj : pc.local_trajectory)
                                {
                                    Eigen::Affine3d pose = pc.m_pose * traj.m_pose;
                                    Eigen::Vector3d pos = pose.translation();
                                    Eigen::Matrix3d rot = pose.rotation();
                                    outfile << std::fixed << std::setprecision(0) << traj.timestamps.second << "," // Unix timestamp
                                            << std::setprecision(10) << pos.x() << "," << pos.y() << "," << pos.z() << "," << rot(0, 0)
                                            << "," << rot(0, 1) << "," << rot(0, 2) << "," << rot(1, 0) << "," << rot(1, 1) << ","
                                            << rot(1, 2) << "," << rot(2, 0) << "," << rot(2, 1) << "," << rot(2, 2) << "\n";
                                }
                            }

                            outfile.close();
                            std::cout << "Saved: " << csv_path << std::endl;
                        } catch (const std::exception& e)
                        {
                            std::cerr << "Error creating " << csv_path << ": " << e.what() << std::endl;
                        }
                    }
                }
                if (ImGui::MenuItem("Save all as csv (timestamp Lidar, Unix)##1"))
                {
                    for (size_t i = 0; i < project_settings.session_file_names.size(); ++i)
                    {
                        const auto& session_path = project_settings.session_file_names[i];
                        if (i >= sessions.size())
                        {
                            std::cerr << "No loaded session for: " << session_path << std::endl;
                            continue;
                        }

                        Session& session = sessions[i];
                        std::filesystem::path dir = std::filesystem::path(session_path).parent_path();
                        std::string folder_name = dir.filename().string();
                        std::string csv_path = (dir / (folder_name + "_trajectory_timestampLidarUnix_r.csv")).string();

                        try
                        {
                            std::ofstream outfile(csv_path);
                            if (!outfile.is_open())
                            {
                                std::cerr << "Failed to create file: " << csv_path << std::endl;
                                continue;
                            }

                            outfile << "timestampLidar,timestampUnix,x,y,z," << "r00,r01,r02,r10,r11,r12,r20,r21,r22\n";

                            for (const auto& pc : session.point_clouds_container.point_clouds)
                            {
                                if (!pc.visible)
                                    continue;
                                for (const auto& traj : pc.local_trajectory)
                                {
                                    Eigen::Affine3d pose = pc.m_pose * traj.m_pose;
                                    Eigen::Vector3d pos = pose.translation();
                                    Eigen::Matrix3d rot = pose.rotation();
                                    outfile << std::fixed << std::setprecision(0) << traj.timestamps.first << "," // Lidar timestamp
                                            << traj.timestamps.second << "," // Unix timestamp
                                            << std::setprecision(10) << pos.x() << "," << pos.y() << "," << pos.z() << "," << rot(0, 0)
                                            << "," << rot(0, 1) << "," << rot(0, 2) << "," << rot(1, 0) << "," << rot(1, 1) << ","
                                            << rot(1, 2) << "," << rot(2, 0) << "," << rot(2, 1) << "," << rot(2, 2) << "\n";
                                }
                            }

                            outfile.close();
                            std::cout << "Saved: " << csv_path << std::endl;
                        } catch (const std::exception& e)
                        {
                            std::cerr << "Error creating " << csv_path << ": " << e.what() << std::endl;
                        }
                    }
                }

                ImGui::Separator();
                ImGui::Text("(x,y,z,qx,qy,qz,qw)");

                if (ImGui::MenuItem("Save all as csv (timestamp Lidar)##2"))
                {
                    for (size_t i = 0; i < project_settings.session_file_names.size(); ++i)
                    {
                        const auto& session_path = project_settings.session_file_names[i];

                        if (i >= sessions.size())
                        {
                            std::cerr << "No loaded session for: " << session_path << std::endl;
                            continue;
                        }

                        Session& session = sessions[i];
                        std::filesystem::path dir = std::filesystem::path(session_path).parent_path();
                        std::string folder_name = dir.filename().string();
                        std::string csv_path = (dir / (folder_name + "_trajectory_timestampLidar_q.csv")).string();

                        std::cout << "Saving trajectory to CSV: " << csv_path << std::endl;

                        try
                        {
                            std::ofstream outfile(csv_path);
                            if (!outfile.is_open())
                            {
                                std::cerr << "Failed to create file: " << csv_path << std::endl;
                                continue;
                            }

                            outfile << "timestampLidar,x,y,z,qx,qy,qz,qw\n";

                            for (const auto& pc : session.point_clouds_container.point_clouds)
                            {
                                if (!pc.visible)
                                    continue;

                                for (const auto& traj : pc.local_trajectory)
                                {
                                    Eigen::Affine3d pose = pc.m_pose * traj.m_pose;
                                    Eigen::Vector3d pos = pose.translation();
                                    Eigen::Quaterniond q(pose.rotation());

                                    outfile << std::fixed << std::setprecision(0) << traj.timestamps.first << "," // Lidar timestamp
                                            << std::setprecision(10) << pos.x() << "," << pos.y() << "," << pos.z() << "," << q.x() << ","
                                            << q.y() << "," << q.z() << "," << q.w() << "\n";
                                }
                            }

                            outfile.close();
                            std::cout << "Saved: " << csv_path << std::endl;
                        } catch (const std::exception& e)
                        {
                            std::cerr << "Error creating " << csv_path << ": " << e.what() << std::endl;
                        }
                    }

                    std::cout << "Finished saving all trajectories to CSV files." << std::endl;
                }
                if (ImGui::MenuItem("Save all as csv (timestamp Unix)##2"))
                {
                    for (size_t i = 0; i < project_settings.session_file_names.size(); ++i)
                    {
                        const auto& session_path = project_settings.session_file_names[i];

                        if (i >= sessions.size())
                        {
                            std::cerr << "No loaded session for: " << session_path << std::endl;
                            continue;
                        }

                        Session& session = sessions[i];
                        std::filesystem::path dir = std::filesystem::path(session_path).parent_path();
                        std::string folder_name = dir.filename().string();
                        std::string csv_path = (dir / (folder_name + "_trajectory_timestampUnix_q.csv")).string();

                        std::cout << "Saving trajectory to CSV: " << csv_path << std::endl;

                        try
                        {
                            std::ofstream outfile(csv_path);
                            if (!outfile.is_open())
                            {
                                std::cerr << "Failed to create file: " << csv_path << std::endl;
                                continue;
                            }

                            outfile << "timestampUnix,x,y,z,qx,qy,qz,qw\n";

                            for (const auto& pc : session.point_clouds_container.point_clouds)
                            {
                                if (!pc.visible)
                                    continue;

                                for (const auto& traj : pc.local_trajectory)
                                {
                                    Eigen::Affine3d pose = pc.m_pose * traj.m_pose;
                                    Eigen::Vector3d pos = pose.translation();
                                    Eigen::Quaterniond q(pose.rotation());

                                    outfile << std::fixed << std::setprecision(0) << traj.timestamps.second << "," // Unix timestamp
                                            << std::setprecision(10) << pos.x() << "," << pos.y() << "," << pos.z() << "," << q.x() << ","
                                            << q.y() << "," << q.z() << "," << q.w() << "\n";
                                }
                            }

                            outfile.close();
                            std::cout << "Saved: " << csv_path << std::endl;
                        } catch (const std::exception& e)
                        {
                            std::cerr << "Error creating " << csv_path << ": " << e.what() << std::endl;
                        }
                    }

                    std::cout << "Finished saving all trajectories to CSV files." << std::endl;
                }
                if (ImGui::MenuItem("Save all as csv (timestamp Lidar, Unix)##2"))
                {
                    for (size_t i = 0; i < project_settings.session_file_names.size(); ++i)
                    {
                        const auto& session_path = project_settings.session_file_names[i];

                        if (i >= sessions.size())
                        {
                            std::cerr << "No loaded session for: " << session_path << std::endl;
                            continue;
                        }

                        Session& session = sessions[i];
                        std::filesystem::path dir = std::filesystem::path(session_path).parent_path();
                        std::string folder_name = dir.filename().string();
                        std::string csv_path = (dir / (folder_name + "_trajectory_timestampLidarUnix_q.csv")).string();

                        std::cout << "Saving trajectory to CSV: " << csv_path << std::endl;

                        try
                        {
                            std::ofstream outfile(csv_path);
                            if (!outfile.is_open())
                            {
                                std::cerr << "Failed to create file: " << csv_path << std::endl;
                                continue;
                            }

                            outfile << "timestampLidar,timestampUnix,x,y,z,qx,qy,qz,qw\n";

                            for (const auto& pc : session.point_clouds_container.point_clouds)
                            {
                                if (!pc.visible)
                                    continue;

                                for (const auto& traj : pc.local_trajectory)
                                {
                                    Eigen::Affine3d pose = pc.m_pose * traj.m_pose;
                                    Eigen::Vector3d pos = pose.translation();
                                    Eigen::Quaterniond q(pose.rotation());

                                    outfile << std::fixed << std::setprecision(0) << traj.timestamps.first << "," // Lidar timestamp
                                            << traj.timestamps.second << "," // Unix timestamp
                                            << std::setprecision(10) << pos.x() << "," << pos.y() << "," << pos.z() << "," << q.x() << ","
                                            << q.y() << "," << q.z() << "," << q.w() << "\n";
                                }
                            }

                            outfile.close();
                            std::cout << "Saved: " << csv_path << std::endl;
                        } catch (const std::exception& e)
                        {
                            std::cerr << "Error creating " << csv_path << ": " << e.what() << std::endl;
                        }
                    }

                    std::cout << "Finished saving all trajectories to CSV files." << std::endl;
                }
                if (ImGui::MenuItem("Save all as TUM TXT"))
                {
                    for (size_t i = 0; i < project_settings.session_file_names.size(); ++i)
                    {
                        const auto& session_path = project_settings.session_file_names[i];
                        if (i >= sessions.size())
                        {
                            std::cerr << "No loaded session for: " << session_path << std::endl;
                            continue;
                        }
                        Session& session = sessions[i];
                        std::filesystem::path dir = std::filesystem::path(session_path).parent_path();
                        std::string folder_name = dir.filename().string();
                        std::string txt_path = (dir / (folder_name + "_trajectory_tum.txt")).string();

                        std::cout << "Saving trajectory to TUM TXT: " << txt_path << std::endl;
                        try
                        {
                            std::ofstream outfile(txt_path);
                            if (!outfile.is_open())
                            {
                                std::cerr << "Failed to create file: " << txt_path << std::endl;
                                continue;
                            }

                            for (const auto& pc : session.point_clouds_container.point_clouds)
                            {
                                if (!pc.visible)
                                    continue;
                                for (const auto& traj : pc.local_trajectory)
                                {
                                    Eigen::Affine3d pose = pc.m_pose * traj.m_pose;
                                    Eigen::Vector3d pos = pose.translation();
                                    Eigen::Quaterniond q(pose.rotation());

                                    double t_s = static_cast<double>(traj.timestamps.first) / 1e9;

                                    outfile << std::fixed << std::setprecision(9) << t_s << " " << std::setprecision(10) << pos.x() << " "
                                            << pos.y() << " " << pos.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
                                            << "\n";
                                }
                            }

                            outfile.close();
                            std::cout << "Saved: " << txt_path << std::endl;
                        } catch (const std::exception& e)
                        {
                            std::cerr << "Error creating " << txt_path << ": " << e.what() << std::endl;
                        }
                    }
                    std::cout << "Finished saving all trajectories to TUM TXT files." << std::endl;
                }
                if (ImGui::MenuItem("Save all as TUM TXT (single folder)"))
                {
                    for (size_t i = 0; i < project_settings.session_file_names.size(); ++i)
                    {
                        const auto& session_path = project_settings.session_file_names[i];

                        if (i >= sessions.size())
                        {
                            std::cerr << "No loaded session for: " << session_path << std::endl;
                            continue;
                        }

                        Session& session = sessions[i];

                        std::filesystem::path session_dir = std::filesystem::path(session_path).parent_path();

                        std::filesystem::path base_dir = session_dir.parent_path();

                        std::filesystem::path output_dir = base_dir / "all_tum_files";

                        try
                        {
                            std::filesystem::create_directories(output_dir);
                        } catch (const std::exception& e)
                        {
                            std::cerr << "Failed to create export directory: " << e.what() << std::endl;
                            continue;
                        }

                        std::string folder_name = session_dir.filename().string();

                        std::filesystem::path txt_path = output_dir / (folder_name + "_trajectory_tum.txt");

                        std::cout << "Saving trajectory to TUM TXT: " << txt_path << std::endl;

                        try
                        {
                            std::ofstream outfile(txt_path);

                            if (!outfile.is_open())
                            {
                                std::cerr << "Failed to create file: " << txt_path << std::endl;
                                continue;
                            }

                            for (const auto& pc : session.point_clouds_container.point_clouds)
                            {
                                if (!pc.visible)
                                    continue;

                                for (const auto& traj : pc.local_trajectory)
                                {
                                    Eigen::Affine3d pose = pc.m_pose * traj.m_pose;

                                    Eigen::Vector3d pos = pose.translation();

                                    Eigen::Quaterniond q(pose.rotation());

                                    double t_s = static_cast<double>(traj.timestamps.first) / 1e9;

                                    outfile << std::fixed << std::setprecision(9) << t_s << " " << std::setprecision(10) << pos.x() << " "
                                            << pos.y() << " " << pos.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
                                            << "\n";
                                }
                            }

                            outfile.close();

                            std::cout << "Saved: " << txt_path << std::endl;
                        } catch (const std::exception& e)
                        {
                            std::cerr << "Error creating " << txt_path << ": " << e.what() << std::endl;
                        }
                    }
                    std::cout << "Finished saving all trajectories to single folder." << std::endl;
                }

                ImGui::EndMenu();
            }

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Tools"))
        {
            ImGui::MenuItem("Normal Distributions Transform", nullptr, &is_ndt_gui, !is_loop_closure_gui && (sessions.size() > 0));
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                ImGui::Text("Point cloud alignment (registration) algorithm");
                ImGui::Text(
                    "Probabilistic alternative to ICP that models one cloud (the target)\nas a set of Gaussian distributions "
                    "rather than raw points");
                ImGui::Text(
                    "Robust for rough initial poses but can converge to a local optimum\nif the initial misalignment is very large");
                ImGui::Text(
                    "Known for being faster and smoother in optimization because\nit replaces discrete point-point correspondences "
                    "with continuous probability density functions.");
                ImGui::EndTooltip();
            }

            // bool prev_is_loop_closure_gui
            ImGui::MenuItem(
                "Manual Loop Closure", "Ctrl+L", &is_loop_closure_gui, (number_visible_sessions == 1 || number_visible_sessions == 2));
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Manually connect overlapping scan sections");

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View"))
        {
            ImGui::BeginDisabled(!(sessions.size() > 0));
            {
                auto tmp = app_state.point_size;
                ImGui::SetNextItemWidth(ImGuiNumberWidth);
                ImGui::InputInt("Points size", &app_state.point_size);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("keyboard 1-9 keys");
                if (app_state.point_size < 1)
                    app_state.point_size = 1;
                else if (app_state.point_size > 10)
                    app_state.point_size = 10;

                if (tmp != app_state.point_size)
                    for (auto& session : sessions)
                        for (auto& point_cloud : session.point_clouds_container.point_clouds)
                            point_cloud.point_size = app_state.point_size;

                ImGui::Separator();
            }
            ImGui::EndDisabled();

            if (ImGui::MenuItem("Orthographic", "key O", &app_state.camera.isOrtho))
            {
                if (app_state.camera.isOrtho)
                    app_state.camera.startEulerTransition(
                        0.0f, 0.0f, app_state.camera.euler.translate, app_state.camera.euler.rotationCenter);
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Switch between perspective view (3D) and orthographic view (2D/flat)");

            ImGui::MenuItem("Show axes", "key X", &app_state.show_axes);
            ImGui::MenuItem("Show compass/ruler", "key C", &app_state.compass_ruler);

            ImGui::MenuItem("Lock Z", "Shift + Z", &app_state.camera.lockZ, !app_state.camera.isOrtho);

            // ImGui::MenuItem("show_covs", nullptr, &show_covs);

            ImGui::Separator();

            ImGui::Text("Colors:");

            ImGui::ColorEdit3("Background", (float*)&app_state.bg_color, ImGuiColorEditFlags_NoInputs);

            // Same shader color modes as step 2's point cloud color schemes (ScanRenderer).
            if (ImGui::BeginMenu("Points color"))
            {
                if (ImGui::MenuItem("> Session color", nullptr, points_color_mode == ScanColorMode::Flat))
                    points_color_mode = ScanColorMode::Flat;
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Each session in its own color (Settings window)");

                ImGui::Separator();

                if (ImGui::MenuItem("> By intensity (gradient)", nullptr, points_color_mode == ScanColorMode::Intensity))
                    points_color_mode = ScanColorMode::Intensity;
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Per-point jet colormap from LAS/LAZ intensity");

                if (ImGui::MenuItem("> By height (gradient)", nullptr, points_color_mode == ScanColorMode::Elevation))
                    points_color_mode = ScanColorMode::Elevation;
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Per-point jet colormap from world Z, over all sessions' [z_min, z_max]");

                if (ImGui::MenuItem("> By distance (gradient)", nullptr, points_color_mode == ScanColorMode::Distance))
                    points_color_mode = ScanColorMode::Distance;
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Per-point jet colormap from distance to the rotation center");

                ImGui::EndMenu();
            }

            ImGui::Separator();

            ImGui::MenuItem("Settings", nullptr, &is_settings_gui);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Show power user settings window with more parameters");

            ImGui::EndMenu();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Scene view relevant parameters");

        camMenu();

        ImGui::BeginDisabled(sessions.size() <= 0);
        {
            ImGui::SameLine();
            ImGui::Dummy(ImVec2(20, 0));
            ImGui::SameLine();

            ImGui::SetNextItemWidth(ImGuiNumberWidth);
            ImGui::InputInt("Points render downsampling", &app_state.viewer_decimate_point_cloud, 10, 100);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("increase for better performance, decrease for rendering more points");
            // ImGui::SameLine();

            if (app_state.viewer_decimate_point_cloud < 1)
                app_state.viewer_decimate_point_cloud = 1;

            ImGui::SameLine();

            ImGui::InputInt("Trajectory reduce render", &viewer_reduce_rendered_trajectory, 10, 100);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("increase for better performance, decrease for rendering more nodes in the trajectory");
            // ImGui::SameLine();

            if (viewer_reduce_rendered_trajectory < 1)
                viewer_reduce_rendered_trajectory = 1;

            ImGui::SameLine();

            ImGui::Text("(%d FPS)", GetFPS());
        }
        ImGui::EndDisabled();

        ImGui::SameLine();
        ImGui::Dummy(ImVec2(20, 0));
        ImGui::SameLine();

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

    if (remove_gui)
    {
        ImGui::OpenPopup("Remove session(s)");
        remove_gui = false;
    }

    if (ImGui::BeginPopupModal("Remove session(s)", NULL, ImGuiWindowFlags_AlwaysAutoResize))
    {
        static std::vector<bool> session_marked_for_removal;
        if (session_marked_for_removal.size() != project_settings.session_file_names.size())
            session_marked_for_removal.resize(project_settings.session_file_names.size(), false);

        ImGui::Text("Select session(s) to remove:");
        ImGui::Separator();

        for (size_t i = 0; i < project_settings.session_file_names.size(); i++)
        {
            bool checked = session_marked_for_removal[i];
            if (ImGui::Checkbox(project_settings.session_file_names[i].c_str(), &checked))
                session_marked_for_removal[i] = checked;
        }

        ImGui::Separator();

        if (ImGui::Button("Remove"))
        {
            for (size_t i = project_settings.session_file_names.size(); i > 0; --i)
            {
                size_t idx = i - 1;

                if (session_marked_for_removal[idx])
                {
                    std::cout << "Removing session: " << project_settings.session_file_names[idx] << std::endl;

                    project_settings.session_file_names.erase(project_settings.session_file_names.begin() + idx);

                    if (idx < sessions.size())
                        sessions.erase(sessions.begin() + idx);
                }
            }
            session_marked_for_removal.clear();

            if (!sessions.empty())
                update_timestamp_offset();
            else
            {
                loaded_sessions = false;
                time_stamp_offset = 0.0;
            }

            ImGui::CloseCurrentPopup();
        }

        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
        {
            session_marked_for_removal.clear();
            ImGui::CloseCurrentPopup();
        }

        ImGui::EndPopup();
    }

    if (is_ndt_gui)
        ndt_gui();

    if (is_loop_closure_gui)
        loop_closure_gui();

    raylib_widgets::showEulerCenterOfRotationWindow(cor_gui, app_state.camera, xText, yText, zText);

    raylib_widgets::ShowInfoWindow(app_state.info_gui, infoLines, appShortcuts, HDMAPPING_VERSION_STRING, __DATE__);

    if (is_settings_gui)
        settings_gui();

    // Switch to 2D screen space for text labels, the compass and ImGui's own draw pass.
    raylib_widgets::end3DMatrixStack(io.DisplaySize.x, io.DisplaySize.y);

    if (is_loop_closure_gui)
        renderLoopClosureLabels();
    else
        for (const auto& session : sessions)
            if (session.visible)
            {
                renderGroundControlPointsLabels(session.ground_control_points, session.point_clouds_container);
                renderControlPointsLabels(session.control_points, session.point_clouds_container);
            }

    if (app_state.compass_ruler)
        drawMiniCompassWithRuler();

    rlImGuiEnd();
}

void mouse(int glut_button, int state, int x, int y)
{
    ImGuiIO& io = ImGui::GetIO();

    // GLUT's wheel-as-button-3/4 fallback is gone: main() polls GetMouseWheelMove() and calls wheel().

    if (!io.WantCaptureMouse)
    {
        if ((glut_button == GLUT_MIDDLE_BUTTON || glut_button == GLUT_RIGHT_BUTTON) && state == GLUT_DOWN && (io.KeyCtrl || io.KeyShift) &&
            !manipulate_active_edge)
        {
            if ((sessions.size() > 0) && (number_visible_sessions > 0) && update_rotation_center)
            {
                getClosestTrajectoriesPoint(
                    sessions,
                    x,
                    y,
                    first_session_index,
                    second_session_index,
                    number_visible_sessions,
                    index_loop_closure_source,
                    index_loop_closure_target,
                    io.KeyShift,
                    time_stamp_offset);
            }
            else if (update_rotation_center)
            {
                setNewRotationCenter(x, y);
            }
        }

        if (state == GLUT_DOWN)
            app_state.mouse_buttons |= 1 << glut_button;
        else if (state == GLUT_UP)
            app_state.mouse_buttons = 0;

        app_state.mouse_old_x = x;
        app_state.mouse_old_y = y;
    }
}

// Was utils.cpp's GLUT initGL(): raylib window + rlImGui, same setup as step 2.
bool initGL(const std::string& winTitleArg)
{
    // HiDPI breaks ImGui scaling on Windows, so it is only enabled on macOS and Linux (as in step 2).
    unsigned int flags = FLAG_WINDOW_RESIZABLE;
#ifdef __APPLE__
    flags |= FLAG_WINDOW_HIGHDPI;
#endif
#if __LINUX__
    flags |= FLAG_WINDOW_HIGHDPI;
#endif

    SetConfigFlags(flags);
    InitWindow(static_cast<int>(window_width), static_cast<int>(window_height), winTitleArg.c_str());
    SetExitKey(KEY_NULL); // Esc must not close the window (e.g. while cancelling a dialog)
    SetTargetFPS(60);
    raylib_widgets::fitWindowToScreen(/*marginW=*/100, /*marginH=*/100, /*centerVertically=*/true);

    rlImGuiSetup(true);
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard | ImGuiConfigFlags_NavEnableGamepad | ImGuiConfigFlags_DockingEnable;
    io.ConfigDockingWithShift = true;

    app_state.camera.applyPerspectiveProjection(static_cast<int>(window_width), static_cast<int>(window_height));

    return true;
}

// Drag & drop: a project (*.mjp) replaces the current one; session files (*.mjs/*.json) are added to it.
void loadDroppedFiles(const std::vector<std::string>& paths)
{
    for (const auto& path : paths)
    {
        std::string ext = fs::path(path).extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == ".mjp")
        {
            loadProject(path, project_settings);
            return;
        }
    }

    bool added = false;
    for (const auto& path : paths)
    {
        std::string ext = fs::path(path).extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == ".mjs" || ext == ".json")
        {
            std::cout << "Adding session file: '" << path << "'" << std::endl;
            project_settings.session_file_names.push_back(path);
            added = true;
        }
    }

    if (added)
    {
        loaded_sessions = false;
        time_stamp_offset = 0.0;
    }
    else
        pfd::message("Unsupported file", "Drop a project (*.mjp) or session files (*.mjs, *.json).", pfd::choice::ok, pfd::icon::warning);
}

int main(int argc, char* argv[])
{
    try
    {
        if (checkClHelp(argc, argv))
        {
            std::cout << winTitle << "\n\n"
                      << "USAGE:\n"
                      << std::filesystem::path(argv[0]).stem().string() << " <input_file> /?\n\n"
                      << "where\n"
                      << "   <input_file>         Path to Mandeye JSON Project file (*.mjp)\n"
                      << "   -h, /h, --help, /?   Show this help and exit\n\n";

            return 0;
        }

        initGL(winTitle);

        if (argc > 1)
        {
            for (int i = 1; i < argc; i++)
            {
                std::string ext = fs::path(argv[i]).extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

                if (ext == ".mjp")
                {
                    loadProject(argv[i], project_settings);

                    break;
                }
            }
        }

        // Was glutMainLoop(): the GLUT callbacks are called directly, on raylib's input transitions.
        while (!WindowShouldClose())
        {
            int mx = static_cast<int>(GetMouseX());
            int my = static_cast<int>(GetMouseY());

            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
                mouse(GLUT_LEFT_BUTTON, GLUT_DOWN, mx, my);
            if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
                mouse(GLUT_LEFT_BUTTON, GLUT_UP, mx, my);
            if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))
                mouse(GLUT_RIGHT_BUTTON, GLUT_DOWN, mx, my);
            if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT))
                mouse(GLUT_RIGHT_BUTTON, GLUT_UP, mx, my);
            if (IsMouseButtonPressed(MOUSE_BUTTON_MIDDLE))
                mouse(GLUT_MIDDLE_BUTTON, GLUT_DOWN, mx, my);
            if (IsMouseButtonReleased(MOUSE_BUTTON_MIDDLE))
                mouse(GLUT_MIDDLE_BUTTON, GLUT_UP, mx, my);

            motion(mx, my);

            float wheelMove = GetMouseWheelMove();
            if (wheelMove != 0.0f)
                wheel(0, wheelMove > 0.0f ? 1 : -1, mx, my);

            if (IsFileDropped())
            {
                FilePathList dropped_files = LoadDroppedFiles();
                std::vector<std::string> paths;
                for (unsigned int i = 0; i < dropped_files.count; i++)
                    paths.emplace_back(dropped_files.paths[i]);
                UnloadDroppedFiles(dropped_files);
                if (!paths.empty())
                    loadDroppedFiles(paths);
            }

            BeginDrawing();
            display();
            EndDrawing();
        }

        // GPU buffers must be released while the GL context still exists.
        session_renderers.clear();
        rlImGuiShutdown();
        CloseWindow();
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
