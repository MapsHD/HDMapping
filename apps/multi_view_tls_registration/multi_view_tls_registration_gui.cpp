#include <algorithm>
#include <cmath>

// This app used to be built on GLUT + legacy immediate-mode OpenGL
// (glBegin/glVertex/gluPerspective/gluUnProject/glMatrixMode/...) via
// core/src/utils.cpp. utils.cpp is shared by several other GLUT apps and
// can't be changed, and raylib's context here is OpenGL 3.3 core profile
// (no fixed-function pipeline), so this file no longer includes
// <Core/utils.hpp> -- instead it locally re-declares the same globals and
// re-implements the same functions it used to get from there (same names,
// same call sites throughout this file), backed by rlgl's rl*() legacy-GL
// emulation API (rlMatrixMode/rlBegin/rlVertex3f/... -- a software matrix
// stack + immediate-mode layer that works under core profile) instead of
// real gl*() calls. The handful of call sites that used to reach into
// core's own legacy-GL .render()/::Render() methods (PointClouds::render(),
// ManualPoseGraphLoopClosure::Render(), etc. -- compiled once into `core`,
// shared with GLUT apps, so they can't be changed either) are replaced with
// Core/raylib_render.hpp's ScanRenderer instead. See each local
// function/global below for what it replaces.
#include "external/glad.h"
#include "raylib.h"
#include "raymath.h"
#include "rlImGui.h"
#include "rlgl.h"

#include <imgui.h>
#include <imgui_internal.h>

#include <ImGuizmo.h>

#include <spdlog/spdlog.h>

#include <Eigen/Eigen>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <Core/e57_utils.h>
#include <Core/export_laz.h>
#include <Core/fmt_filesystem.hpp>
#include <Core/gnss.h>
#include <Core/icp.h>
#include <Core/manual_pose_graph_loop_closure.h>
#include <Core/ndt.h>
#include <Core/observation_picking.h>
#include <Core/pfd_wrapper.hpp>
#include <Core/pose_graph_slam.h>
#include <Core/raylib_render.hpp>
#include <Core/registration_plane_feature.h>
#include <Core/session.h>
#include <Core/structures.h>
#include <Core/transformations.h>
#include <Core/tum.h>
#include <RaylibWidgets/WindowFit.h>

#ifdef _WIN32
// portable-file-dialogs.h pulls in real windows.h, whose CloseWindow(HWND)/
// ShowCursor(BOOL) collide with raylib.h's already-declared CloseWindow(void)/
// ShowCursor(void) (both extern "C", so this is a hard redeclaration error,
// not just a macro-textual one -- and unlike rl_utils.cpp, which only needs
// ShellExecuteA, this file also needs portable-file-dialogs.h's own
// winuser.h functionality, i.e. SendMessage/DispatchMessage/MessageBoxW/
// GetActiveWindow, so suppressing all of winuser.h via NOUSER isn't an
// option here). Renaming raylib's versions doesn't work either: the
// compiled raylib library still only exports the symbol under its real
// name, so a renamed *declaration* just becomes an unresolved symbol at
// link time. windows.h's versions are renamed instead -- safe because
// portable-file-dialogs.h itself never calls CloseWindow/ShowCursor
// (verified: neither name appears in its source) -- leaving raylib's
// real CloseWindow/ShowCursor callable normally everywhere in this file.
#define CloseWindow CloseWindow_win32
#define ShowCursor ShowCursor_win32
#endif
#include <portable-file-dialogs.h>
#ifdef _WIN32
#undef CloseWindow
#undef ShowCursor
#endif

#include <laszip/laszip_api.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <tuple>

#include "../lidar_odometry_step_1/lidar_odometry_utils.h"
#include "multi_view_tls_registration.h"

#include <HDMapping/Version.hpp>

#include "WGS84toCartesian/WGS84toCartesian.hpp"
#include "wgs84_do_puwg92/wgs84_do_puwg92.h"

#include <proj.h>
#ifdef _WIN32
// Just numeric resource IDs (IDI_ICON1 etc.) -- no windows.h needed to parse
// it, and this file makes no direct WinAPI calls itself, so windows.h isn't
// included directly here (it still arrives transitively, via
// portable-file-dialogs.h above -- see the CloseWindow/ShowCursor rename
// near the top of this file, and the #undef DrawText below, for how that's
// handled).
#include "resource.h"
#endif

// Camera/picking/mini-compass/misc-ImGui-widget API this app used to get
// from <Core/utils.hpp>, then from its own local rl_utils.h/.cpp (see this
// section's comments for what replaced what) -- folded directly into this
// file since everything in rl_utils.h/.cpp turned out to only ever be used
// here, plus raylib_widgets (shared with the camera_lidar_* apps: OrbitCamera's
// Euler mode, the Euler center-of-rotation dialog, app-shell scaffolding,
// double-precision ray/plane math).
#include <RaylibWidgets/AppShell.h>
#include <RaylibWidgets/CenterOfRotationWindow.h>
#include <RaylibWidgets/CompassRuler.h>
#include <RaylibWidgets/OrbitCamera.h>
#include <RaylibWidgets/PointPicking.h>
#include <RaylibWidgets/RayPlaneD.h>
#include <RaylibWidgets/ShortcutsTable.h>

#ifdef _WIN32
// windows.h (pulled in transitively above, via portable-file-dialogs.h)
// #defines DrawText as DrawTextA -- undefining it restores plain DrawText
// calls below to mean raylib's function again (its declaration, parsed
// before windows.h's macro existed, is unaffected either way; this only
// affects how later *source text* that writes the identifier "DrawText"
// gets preprocessed).
#undef DrawText
#endif

///////////////////////////////////////////////////////////////////////////////////

// Now shared with the camera_lidar_* apps -- see
// raylib_widgets/include/RaylibWidgets/ShortcutsTable.h/AppShell.h.
using raylib_widgets::ImGuiHyperlink;
using raylib_widgets::ShortcutEntry;
using raylib_widgets::ShowMainDockSpace;

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

enum ColorScheme
{
    CS_SOLID, // fixed color
    CS_RANDOM, // random
    CS_GRAD_INTENS, // gradient based on intensity
    CS_GRAD_ELEV, // gradient based on elevation
    CS_GRAD_DIST, // gradient based on distance from rotation center
    CS_FOLLOW // valid for trajectory
};

struct AppStateBase
{
    int viewer_decimate_point_cloud = 2;

    int mouse_old_x = 0, mouse_old_y = 0;
    int mouse_buttons = 0;
    bool show_axes = true;
    ImVec4 bg_color = ImVec4(0.65f, 0.65f, 0.65f, 1.00f);
    int point_size = 1;

    bool info_gui = false;
    bool compass_ruler = true;

    // Still Eigen/rlgl-driven directly (not folded into
    // raylib_widgets::OrbitCamera, which deliberately stays Eigen-free for
    // its azimuth/elevation half -- only OrbitCamera's own RayPlaneD-using
    // pieces gained an Eigen dependency) -- used only by
    // drawMiniCompassWithRuler() below and display()'s own rlMultMatrixf
    // call. Rebuilt from `camera` every frame.
    Eigen::Affine3f viewLocal;

    // Camera state (rotate/translate/rotation-center/ortho/presets/
    // transitions/frame matrices), shared with the camera_lidar_* apps via
    // raylib_widgets::OrbitCamera's Euler/ortho mode -- see its header.
    raylib_widgets::OrbitCamera camera;

    // Unlike the original (which probed GL_LINE_WIDTH_RANGE), rlgl's line width
    // support is uniform enough here not to need a runtime check -- always true.
    bool glLineWidthSupport = true;
};

inline AppStateBase app_state;

// Edge-triggered "please open the Center of rotation dialog" request --
// set by view_kbd_shortcuts()'s Shift+R, consumed once by
// showEulerCenterOfRotationWindow() in display() below.
bool cor_gui = false;

bool scroll_hint_enabled = true;
bool scroll_hint_active = false;
int scroll_hint_count = 0;
float scroll_hint_accu = 0.0f;
double scroll_hint_lastT = 0.0;

std::string truncPath(const std::string& fullPath);

void wheel(int button, int dir, int x, int y);
void motion(int x, int y);

void showAxes();
void drawIntersectionGrids(const PointClouds& point_clouds_container, const PointClouds::PointCloudDimensions& dims);
void camMenu();
void view_kbd_shortcuts();

void drawMiniCompassWithRuler();

Eigen::Vector3d rayIntersection(const LaserBeam& laser_beam, const RegistrationPlaneFeature::Plane& plane);
LaserBeam GetLaserBeam(int x, int y);
double distance_point_to_line(const Eigen::Vector3d& point, const LaserBeam& line);
void getClosestTrajectoryPoint(Session& session_, int x, int y, bool gcpPicking, int& picked_index);

void setNewRotationCenter(int x, int y);

bool checkClHelp(int argc, char** argv);

// GPU (rlgl-based) point cloud renderer -- replaces core's legacy-GL
// PointCloud::render()/PointClouds::render() (see Core/raylib_render.hpp).
// Rebuilt on session load and whenever a scan's pose changes; syncPoses()
// is called once per frame in display() as a safety net for pose-mutating
// code paths that don't explicitly call rebuild().
ScanRenderer scan_renderer;

// This frame's 3D model-view-projection matrix, captured right after the
// camera transform is set up in display() (before the projection/modelview
// stack gets reset to the 2D screen ortho for ImGui -- see
// raylib_widgets::end3DMatrixStack()). renderLoopClosureLabels() uses it to
// project pose world positions to screen space for DrawText, since it runs
// after that reset (2D text needs the 2D ortho active, but still needs to
// know where each 3D point landed on screen).
Matrix frame_mvp_3d{};

// Forward declarations for this file's own functions defined near
// display() below -- panel functions earlier in this file call some of these.
void observationPickingRender(const ObservationPicking& observation_picking);
void renderLoopClosure(
    PointClouds& point_clouds_container, int index_loop_closure_source, int index_loop_closure_target, int before, int after);
void renderLoopClosureLabels(PointClouds& point_clouds_container);
void renderGroundControlPoints(const GroundControlPoints& ground_control_points, const PointClouds& point_clouds_container);
void renderGroundControlPointsLabels(const GroundControlPoints& ground_control_points, const PointClouds& point_clouds_container);
void renderGNSS(const GNSS& gnss, const PointClouds& point_clouds_container);
void renderTUM(const TUM& tum, const PointClouds& point_clouds_container);
void renderControlPoints(const ControlPoints& control_points, PointClouds& point_clouds_container);
void renderControlPointsLabels(const ControlPoints& control_points, const PointClouds& point_clouds_container);
void display();
void mouse(int glut_button, int state, int x, int y);

///////////////////////////////////////////////////////////////////////////////////

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

// Ported from PointClouds::draw_grids() (core/src/point_clouds.cpp, legacy
// immediate-mode GL, shared with the GLUT apps so it can't be changed) --
// rl*() rename, one helper per cutting plane instead of one copy-pasted
// block per grid density. Spans the session's bounding box (dims), snapped
// outward to whole grid steps, same as the original.
void drawGridXZ(float step, Color color, const PointClouds::PointCloudDimensions& dims)
{
    float x_min = std::floor(dims.x_min / step) * step;
    float x_max = std::ceil(dims.x_max / step) * step;
    float z_min = std::floor(dims.z_min / step) * step;
    float z_max = std::ceil(dims.z_max / step) * step;

    rlBegin(RL_LINES);
    rlColor3f(color.r / 255.f, color.g / 255.f, color.b / 255.f);
    for (float x = x_min; x <= x_max; x += step)
    {
        rlVertex3f(x, 0.0f, z_min);
        rlVertex3f(x, 0.0f, z_max);
    }
    for (float z = z_min; z <= z_max; z += step)
    {
        rlVertex3f(x_min, 0.0f, z);
        rlVertex3f(x_max, 0.0f, z);
    }
    rlEnd();
}

void drawGridYZ(float step, Color color, const PointClouds::PointCloudDimensions& dims)
{
    float y_min = std::floor(dims.y_min / step) * step;
    float y_max = std::ceil(dims.y_max / step) * step;
    float z_min = std::floor(dims.z_min / step) * step;
    float z_max = std::ceil(dims.z_max / step) * step;

    rlBegin(RL_LINES);
    rlColor3f(color.r / 255.f, color.g / 255.f, color.b / 255.f);
    for (float y = y_min; y <= y_max; y += step)
    {
        rlVertex3f(0.0f, y, z_min);
        rlVertex3f(0.0f, y, z_max);
    }
    for (float z = z_min; z <= z_max; z += step)
    {
        rlVertex3f(0.0f, y_min, z);
        rlVertex3f(0.0f, y_max, z);
    }
    rlEnd();
}

void drawGridXY(float step, Color color, const PointClouds::PointCloudDimensions& dims)
{
    float x_min = std::floor(dims.x_min / step) * step;
    float x_max = std::ceil(dims.x_max / step) * step;
    float y_min = std::floor(dims.y_min / step) * step;
    float y_max = std::ceil(dims.y_max / step) * step;

    rlBegin(RL_LINES);
    rlColor3f(color.r / 255.f, color.g / 255.f, color.b / 255.f);
    for (float x = x_min; x <= x_max; x += step)
    {
        rlVertex3f(x, y_min, 0.0f);
        rlVertex3f(x, y_max, 0.0f);
    }
    for (float y = y_min; y <= y_max; y += step)
    {
        rlVertex3f(x_min, y, 0.0f);
        rlVertex3f(x_max, y, 0.0f);
    }
    rlEnd();
}

// Draws whichever of the 9 grid-density/plane checkboxes (View menu, next to
// the xz/yz/xy_intersection toggles) are on -- was the unconditional
// draw_grids() call at the top of the legacy PointClouds::render(). Not
// gated on xz/yz/xy_intersection itself (matching the original): a grid can
// be shown independent of whether its plane's intersection slab is active.
void drawIntersectionGrids(const PointClouds& point_clouds_container, const PointClouds::PointCloudDimensions& dims)
{
    const Color light = ColorFromNormalized(Vector4{ 0.7f, 0.7f, 0.7f, 1.0f });
    const Color dark = ColorFromNormalized(Vector4{ 0.3f, 0.3f, 0.3f, 1.0f });

    if (point_clouds_container.xz_grid_10x10)
        drawGridXZ(10.0f, light, dims);
    if (point_clouds_container.xz_grid_1x1)
        drawGridXZ(1.0f, dark, dims);
    if (point_clouds_container.xz_grid_01x01)
        drawGridXZ(0.1f, dark, dims);

    if (point_clouds_container.yz_grid_10x10)
        drawGridYZ(10.0f, light, dims);
    if (point_clouds_container.yz_grid_1x1)
        drawGridYZ(1.0f, dark, dims);
    if (point_clouds_container.yz_grid_01x01)
        drawGridYZ(0.1f, dark, dims);

    if (point_clouds_container.xy_grid_10x10)
        drawGridXY(10.0f, light, dims);
    if (point_clouds_container.xy_grid_1x1)
        drawGridXY(1.0f, dark, dims);
    if (point_clouds_container.xy_grid_01x01)
        drawGridXY(0.1f, dark, dims);
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
            app_state.viewer_decimate_point_cloud = 2;
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
        app_state.viewer_decimate_point_cloud = 2;
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

// Drawing itself lives in raylib_widgets::drawCompassRuler (shared with the
// camera_lidar_* apps' identical overlay) -- this just adapts this app's own
// camera/background state (app_state.viewLocal's rotation matrix,
// app_state.camera.euler.translate.z zoom, app_state.bg_color) into that
// function's right/up/zoomDistance/rulerColor parameters. Row 0/1 of a
// world-to-eye rotation matrix R are exactly the world-space directions that
// map to eye-space +X/+Y (screen right/up): (R * dir).x() == dot(R.row(0), dir).
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

// Was distanceToPlane()+its own loop -- both now delegate to the shared
// raylib_widgets::intersectPlane() (Eigen double precision, matching this
// app's world coordinates). Falls back to laser_beam.position itself (like
// the original) when the ray is ~parallel to the plane.
Eigen::Vector3d rayIntersection(const LaserBeam& laser_beam, const RegistrationPlaneFeature::Plane& plane)
{
    Eigen::Vector3d hit = laser_beam.position;
    raylib_widgets::intersectPlane(laser_beam.position, laser_beam.direction, plane.a, plane.b, plane.c, plane.d, hit);
    return hit;
}

// Delegates the actual unprojection to the shared
// raylib_widgets::OrbitCamera::eulerScreenRay() (also used by
// camera_lidar_trajectory_viewer) and adapts its raylib Ray into this app's
// own Eigen-based LaserBeam type, which rayIntersection()/
// distance_point_to_line()/callers throughout this file still expect.
LaserBeam GetLaserBeam(int x, int y)
{
    Ray ray = app_state.camera.eulerScreenRay(x, y, GetScreenWidth(), GetScreenHeight());

    LaserBeam laser_beam;
    laser_beam.position = Eigen::Vector3d(ray.position.x, ray.position.y, ray.position.z);
    laser_beam.direction = Eigen::Vector3d(ray.direction.x, ray.direction.y, ray.direction.z);

    return laser_beam;
}

// Delegates to the shared raylib_widgets::distancePointToLine().
double distance_point_to_line(const Eigen::Vector3d& point, const LaserBeam& line)
{
    return raylib_widgets::distancePointToLine(point, line.position, line.direction);
}

// Shared with camera_lidar_trajectory_viewer's equivalent
// nearestTrajectoryPoint() -- both now delegate the actual nearest-point-
// to-ray search to raylib_widgets::pickNearestPointOnLine() instead of
// each keeping its own copy of this loop.
void getClosestTrajectoryPoint(Session& session_, int x, int y, bool gcpPicking, int& picked_index)
{
    picked_index = -1;

    const auto laser_beam = GetLaserBeam(x, y);
    Ray ray;
    ray.position = Vector3{ static_cast<float>(laser_beam.position.x()),
                            static_cast<float>(laser_beam.position.y()),
                            static_cast<float>(laser_beam.position.z()) };
    ray.direction = Vector3{ static_cast<float>(laser_beam.direction.x()),
                             static_cast<float>(laser_beam.direction.y()),
                             static_cast<float>(laser_beam.direction.z()) };

    std::vector<Vector3> pts;
    std::vector<std::pair<int, int>> ptOwners; // (point cloud index, local_trajectory index), parallel to pts
    for (int i = 0; i < session_.point_clouds_container.point_clouds.size(); i++)
    {
        for (int j = 0; j < session_.point_clouds_container.point_clouds[i].local_trajectory.size(); j++)
        {
            const auto& p = session_.point_clouds_container.point_clouds[i].local_trajectory[j].m_pose.translation();
            Eigen::Vector3d vp = session_.point_clouds_container.point_clouds[i].m_pose * p;
            pts.push_back(Vector3{ static_cast<float>(vp.x()), static_cast<float>(vp.y()), static_cast<float>(vp.z()) });
            ptOwners.push_back({ i, j });
        }
    }

    // Defaults to the still-pending transition target (mirrors the
    // original, which read/wrote its own persistent new_rotation_center
    // field here rather than a fresh local -- so a call that finds no
    // point still re-triggers a transition toward whatever that field last
    // held).
    Vector3 center = app_state.camera.eulerGoal.rotationCenter;

    size_t bestIdx;
    if (raylib_widgets::pickNearestPointOnLine(pts.data(), pts.size(), ray, bestIdx))
    {
        center = pts[bestIdx];
        const auto [index_i, index_j] = ptOwners[bestIdx];

        if (gcpPicking)
        {
            session_.ground_control_points.picking_mode_index_to_node_inner = index_i;
            session_.ground_control_points.picking_mode_index_to_node_outer = index_j;
        }

        picked_index = index_i;
    }

    app_state.camera.moveEulerRotationCenterTo(center);
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

///////////////////////////////////////////////////////////////////////////////////

#ifdef _WIN32
bool consWin = true;
#endif
bool consImGui = false;

std::string winTitle = std::string("Step 2 (Multi view TSL registration) ") + HDMAPPING_VERSION_STRING;

std::vector<std::string> infoLines = {
    "This program is second step in MANDEYE process",
    "",
    "It refines trajectory (e.g with loop closure)",
    "It refines trajectory with many approaches (e.g. Iterative Closest Point, Normal Distributions Transform)",
    "It exports session as rigid point cloud to single LAZ file",
    "LAZ files are the product of MANDEYE process (open them with Cloud Compare)",
};

// App specific shortcuts. Used to be two overlaid lists (this one, plus a
// "generic" scaffold in rl_utils.cpp that ShowShortcutsTable() fell back to
// for blank descriptions) -- merged into one here since raylib_widgets'
// shared ShowShortcutsTable() just takes a single complete list. The merge
// also fixes two bugs the old indirection was hiding: this list was missing
// a "Ctrl+J" entry, silently shifting every entry after "J" by one row
// against the generic list's descriptions; and "Right click + drag" had a
// stray "n" instead of its real "camera pan" description.
static const std::vector<ShortcutEntry> appShortcuts = { { "Normal keys", "A", "" },
                                                         { "", "Ctrl+A", "point cloud Alignment" },
                                                         { "", "B", "camera Back" },
                                                         { "", "Ctrl+B", "" },
                                                         { "", "C", "Compass/ruler" },
                                                         { "", "Ctrl+C", "Control points" },
                                                         { "", "D", "" },
                                                         { "", "Ctrl+D", "" },
                                                         { "", "E", "" },
                                                         { "", "Ctrl+E", "lio segments Editor" },
                                                         { "", "F", "camera Front" },
                                                         { "", "Ctrl+F", "" },
                                                         { "", "G", "" },
                                                         { "", "Ctrl+G", "Ground control points" },
                                                         { "", "H", "" },
                                                         { "", "Ctrl+H", "" },
                                                         { "", "I", "camera Isometric" },
                                                         { "", "Ctrl+I", "" },
                                                         { "", "J", "" },
                                                         { "", "Ctrl+J", "" },
                                                         { "", "K", "" },
                                                         { "", "Ctrl+K", "" },
                                                         { "", "L", "camera Left" },
                                                         { "", "Ctrl+L", "manual Loop closure" },
                                                         { "", "M", "" },
                                                         { "", "Ctrl+M", "" },
                                                         { "", "N", "" },
                                                         { "", "Ctrl+N", "" },
                                                         { "", "O", "Ortographic view" },
                                                         { "", "Ctrl+O", "Open/load session/data" },
                                                         { "", "P", "" },
                                                         { "", "Ctrl+P", "Pose graph slam" },
                                                         { "", "Q", "" },
                                                         { "", "Ctrl+Q", "" },
                                                         { "", "R", "camera Right" },
                                                         { "", "Ctrl+R", "Random cloud colors" },
                                                         { "", "Shift+R", "Rotation center" },
                                                         { "", "S", "" },
                                                         { "", "Ctrl+S", "Save session" },
                                                         { "", "Ctrl+Shift+S", "Save subsession" },
                                                         { "", "T", "camera Top" },
                                                         { "", "Ctrl+T", "Solid cloud color" },
                                                         { "", "U", "camera bottom (Under)" },
                                                         { "", "Ctrl+U", "" },
                                                         { "", "V", "" },
                                                         { "", "Ctrl+V", "" },
                                                         { "", "W", "" },
                                                         { "", "Ctrl+W", "" },
                                                         { "", "X", "show aXes" },
                                                         { "", "Ctrl+X", "" },
                                                         { "", "Y", "" },
                                                         { "", "Ctrl+Y", "" },
                                                         { "", "Z", "camera reset" },
                                                         { "", "Ctrl+Z", "" },
                                                         { "", "Shift+Z", "Lock Z" },
                                                         { "", "1-9", "point size" },
                                                         { "Special keys", "Up arrow", "" },
                                                         { "", "Shift + up arrow", "camera translate Up" },
                                                         { "", "Ctrl + up arrow", "" },
                                                         { "", "Down arrow", "" },
                                                         { "", "Shift + down arrow", "camera translate Down" },
                                                         { "", "Ctrl + down arrow", "" },
                                                         { "", "Left arrow", "" },
                                                         { "", "Shift + left arrow", "camera translate Left" },
                                                         { "", "Ctrl + left arrow", "" },
                                                         { "", "Right arrow", "" },
                                                         { "", "Shift + right arrow", "camera translate Right" },
                                                         { "", "Ctrl + right arrow", "" },
                                                         { "", "Pg down", "" },
                                                         { "", "Pg up", "" },
                                                         { "", "- key", "" },
                                                         { "", "+ key", "" },
                                                         { "Mouse related", "Left click + drag", "camera rotate" },
                                                         { "", "Right click + drag", "camera pan" },
                                                         { "", "Scroll", "camera zoom" },
                                                         { "", "Shift + scroll", "camera 5x zoom" },
                                                         { "", "Shift + drag", "Dock window to screen edges" },
                                                         { "", "Ctrl + left click", "" },
                                                         { "", "Ctrl + right click", "change center of rotation" },
                                                         { "", "Ctrl + middle click", "change center of rotation (if no CP GUI active)" } };

namespace fs = std::filesystem;

static bool show_demo_window = true;
static bool show_another_window = false;

bool gnssWithOffset = false;
bool tumSubtractFirstPose = false;

// radio button selectors
static int NDTnomSelection = 0;
static int NDTpeSelection = 0;
static int NDT3dSelection = 0;
static int ICPnomSelection = 0;
static int ICPpeSelection = 0;
static int ICP3dSelection = 0;
static int RPFnomSelection = 0;
static int RPFpeSelection = 0;
static int RPF3dSelection = 0;
static int PGSnomSelection = 0;
static int PGSpeSelection = 0;
static int PGS3dSelection = 0;
static int PGSpwmtSelection = 0;

std::string session_file_name = "";
int session_total_number_of_points = 0;
// bool dynamicSubsampling = true;
// static double lastAdjustTime = 0.0;  // last time we changed subsampling
// const double cooldownSeconds = 1;  // wait between auto adjustments
// static float fps_avg = 60.0f;

bool is_pca_gui = false;
bool is_ndt_gui = true;
bool is_icp_gui = false;
bool is_rpf_gui = false;
bool is_pose_graph_slam = false;
bool is_manual_analisys = false;
bool is_loop_closure_gui = false;
bool is_lio_segments_gui = false;
bool is_settings_gui = true;
bool is_translate_gui = false;

struct TranslateTool
{
    enum class Step
    {
        Idle,
        PickOrigin,
        PickXAxis,
        PickYHint,
        Ready
    };
    Step step = Step::Idle;
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    Eigen::Vector3d x_point = Eigen::Vector3d::Zero();
    Eigen::Vector3d y_hint = Eigen::Vector3d::Zero();
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    bool has_transform = false;
    float plane_z = 0.0f;
};
TranslateTool translate_tool;

bool fillInSession = true;

TLSRegistration tls_registration;
ObservationPicking observation_picking;
std::vector<Eigen::Vector3d> picked_points;

bool new_loop_closure_index = false;
int num_edge_extended_before = 0;
int num_edge_extended_after = 0;
int index_loop_closure_source = 0;
int index_loop_closure_target = 0;
int index_begin = 0;
int index_end = 0;

ColorScheme csPointCloud = CS_GRAD_INTENS;
ColorScheme csTrajectory = CS_SOLID;

// New (not in the original GLUT app): CS_GRAD_INTENS/CS_GRAD_ELEV/CS_GRAD_DIST
// were declared in the original's ColorScheme enum but never actually wired
// to a menu item or the renderer -- this hooks them up to scan_renderer's
// per-point jet-colormap shader modes (see Core/raylib_render.hpp's
// ScanColorMode), alongside the two modes (Solid/Random) the original did
// implement.
ScanColorMode scanColorModeFromScheme(ColorScheme cs)
{
    switch (cs)
    {
    case CS_GRAD_INTENS:
        return ScanColorMode::Intensity;
    case CS_GRAD_ELEV:
        return ScanColorMode::Elevation;
    case CS_GRAD_DIST:
        return ScanColorMode::Distance;
    default:
        return ScanColorMode::Flat;
    }
}

float m_gizmo[] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

bool manipulate_only_marked_gizmo = false;

Session session;
PointClouds::PointCloudDimensions session_dims;
bool session_loaded = false;
std::vector<std::string> geoids;
std::string selected_geoid_model;

// these functions performs experiment from paper
//@article
//{BEDKOWSKI2023113199,
//      title = {Benchmark of multi-view Terrestrial Laser Scanning Point Cloud data registration algorithms},
//      journal = {Measurement},
//      pages = {113199},
//      year = {2023},
//      issn = {0263-2241},
//      doi = {https://doi.org/10.1016/j.measurement.2023.113199},
//      url = {https://www.sciencedirect.com/science/article/pii/S0263224123007637},
//      author = {Janusz Będkowski},
//      keywords = {TLS, Point cloud, Open-source, Multi-view data registration, LiDAR data metrics, Robust loss function, Tait-bryan
//      angles, Quaternions, Rodrigues’ formula, Lie algebra, Rotation matrix parameterization}, abstract = {This study addresses multi-view
//      Terrestrial Laser Scanning Point Cloud data registration methods. Multiple rigid point cloud data registration is mandatory for
//      aligning all scans into a common reference frame and it is still considered a challenge looking from a large-scale surveys point of
//      view. The goal of this work is to support the development of cutting-edge registration methods in geoscience and mobile robotics
//      domains. This work evaluates 3 data sets of total 20 scenes available in the literature. This paper provides a novel open-source
//      framework for multi-view Terrestrial Laser Scanning Point Cloud data registration benchmarks. The goal was to verify experimentally
//      which registration variant can improve the open-source data looking from the quantitative and qualitative points of view. In
//      particular, the following scanners provided measurement data: Z+F TLS Imager 5006i, Z+F TLS Imager 5010C, Leica ScanStation C5,
//      Leica ScanStation C10, Leica P40 and Riegl VZ-400. The benchmark shows an impact of the metric e.g. point to point, point to
//      projection onto a plane, plane to plane etc..., rotation matrix parameterization (Tait-Bryan, quaternion, Rodrigues) and other
//      implementation variations (e.g. multi-view Normal Distributions Transform, Pose Graph SLAM approach) onto the multi-view data
//      registration accuracy and performance. An open-source project is created and it can be used for improving existing data sets
//      reported in the literature, it is the added value of the presented research. The combination of metrics, rotation matrix
//      parameterization and optimization algorithms creates hundreds of possible approaches. It is shown that chosen metric is a dominant
//      factor in data registration. The rotation parameterization and other degrees of freedom of proposed variants are rather negligible
//      compared with chosen metric. Most of the proposed approaches improve registered reference data provided by other researchers. Only
//      for 2 from 20 scenes it was not possible to provide significant improvement. The largest improvements are evident for large-scale
//      scenes. The project is available and maintained at https://github.com/MapsHD/HDMapping.}
// }

void export_result_to_folder(std::string output_folder_name, ObservationPicking& observation_picking, Session& session);
void perform_experiment_on_windows(
    Session& session,
    ObservationPicking& observation_picking,
    ICP& icp,
    NDT& ndt,
    RegistrationPlaneFeature& registration_plane_feature,
    PoseGraphSLAM& pose_graph_slam);
void perform_experiment_on_linux(
    Session& session,
    ObservationPicking& observation_picking,
    ICP& icp,
    NDT& ndt,
    RegistrationPlaneFeature& registration_plane_feature,
    PoseGraphSLAM& pose_graph_slam);
double compute_rms(bool initial, Session& session, ObservationPicking& observation_picking);
void reset_poses(Session& session);
void translate_gui();
void draw_translate_preview();

///////////////////////////////////////////////////////////////////////////////////

void ndt_gui()
{
    ImGui::InputFloat3("Bucket size (x,y,z) [m]", tls_registration.ndt.bucket_size);
    if (tls_registration.ndt.bucket_size[0] < 0.01)
        tls_registration.ndt.bucket_size[0] = 0.01f;
    if (tls_registration.ndt.bucket_size[1] < 0.01)
        tls_registration.ndt.bucket_size[1] = 0.01f;
    if (tls_registration.ndt.bucket_size[2] < 0.01)
        tls_registration.ndt.bucket_size[2] = 0.01f;

    ImGui::PushItemWidth(ImGuiNumberWidth);
    ImGui::InputInt("Number of threads", &tls_registration.ndt.number_of_threads);
    if (tls_registration.ndt.number_of_threads < 1)
        tls_registration.ndt.number_of_threads = 1;
    ImGui::SameLine();
    ImGui::InputInt("Number of iterations", &tls_registration.ndt.number_of_iterations);
    if (tls_registration.ndt.number_of_iterations < 1)
        tls_registration.ndt.number_of_iterations = 1;
    ImGui::PopItemWidth();

    ImGui::Checkbox("Fix first node (add I to first pose in Hessian)", &tls_registration.ndt.is_fix_first_node);

    ImGui::Text("Nonlinear optimization method:");
    ImGui::SameLine();
    ImGui::RadioButton("Gauss-Newton", &NDTnomSelection, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Levenberg-Marguardt", &NDTnomSelection, 1);

    tls_registration.ndt.is_gauss_newton = (NDTnomSelection == 0);
    tls_registration.ndt.is_levenberg_marguardt = (NDTnomSelection == 1);

    ImGui::Text("Poses expressed as:");
    ImGui::SameLine();
    ImGui::RadioButton("camera<-world (cw)", &NDTpeSelection, 0);
    ImGui::SameLine();
    ImGui::RadioButton("camera->world (wc)", &NDTpeSelection, 1);

    tls_registration.ndt.is_cw = (NDTpeSelection == 0);
    tls_registration.ndt.is_wc = (NDTpeSelection == 1);

    ImGui::Text("Parameterizations of 3D rotation:");
    ImGui::RadioButton("Tait-Bryan angles (om fi ka: RxRyRz)", &NDT3dSelection, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Quaternion (q0 q1 q2 q3)", &NDT3dSelection, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Rodrigues (sx sy sz)", &NDT3dSelection, 2);

    tls_registration.ndt.is_tait_bryan_angles = (NDT3dSelection == 0);
    tls_registration.ndt.is_quaternion = (NDT3dSelection == 1);
    tls_registration.ndt.is_rodrigues = (NDT3dSelection == 2);

    if (ImGui::Button("Optimization"))
    {
        double rms_initial = 0.0;
        double rms_final = 0.0;
        double mui = 0.0;
        // ndt.optimize(point_clouds_container.point_clouds, rms_initial, rms_final, mui);
        // spdlog::info("mui: " << mui << " rms_initial: " << rms_initial << " rms_final: " << rms_final << std::endl;
        tls_registration.ndt.optimize(session.point_clouds_container.point_clouds, false, tls_registration.compute_mean_and_cov_for_bucket);
    }

    if (ImGui::Button("Compute mean Mahalanobis distance"))
    {
        double rms_initial = 0.0;
        double rms_final = 0.0;
        double mui = 0.0;

        tls_registration.ndt.optimize(session.point_clouds_container.point_clouds, true, tls_registration.compute_mean_and_cov_for_bucket);
    }
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::Text(
            "Average of all Mahalanobis distances between transformed source points\nand their corresponding Gaussian cells, where:");
        ImGui::Text(
            "Mahalanobis distance measures how far a point is from the mean of a multivariate Gaussian distribution,\ntaking into "
            "account the covariance (shape and orientation) of that distribution");
        ImGui::EndTooltip();
    }

    ImGui::Separator();

    ImGui::Text("NDT optimization Lie algebra:");
    ImGui::SameLine();
    if (ImGui::Button("left Jacobian"))
    {
        tls_registration.ndt.optimize_lie_algebra_left_jacobian(
            session.point_clouds_container.point_clouds, tls_registration.compute_mean_and_cov_for_bucket);
    }
    ImGui::SameLine();
    if (ImGui::Button("right Jacobian"))
    {
        tls_registration.ndt.optimize_lie_algebra_right_jacobian(
            session.point_clouds_container.point_clouds, tls_registration.compute_mean_and_cov_for_bucket);
    }

    ImGui::Separator();

    ImGui::Checkbox("Generalized", &tls_registration.ndt.is_generalized);
    ImGui::BeginDisabled(!tls_registration.ndt.is_generalized);
    {
        ImGui::PushItemWidth(ImGuiNumberWidth);
        ImGui::InputDouble("sigma_r", &tls_registration.ndt.sigma_r, 0.01, 0.01);
        ImGui::InputDouble("sigma_polar_angle_rad", &tls_registration.ndt.sigma_polar_angle, 0.0001, 0.0001);
        ImGui::InputDouble("sigma_azimuthal_angle_rad", &tls_registration.ndt.sigma_azimuthal_angle, 0.0001, 0.0001);
        ImGui::InputInt("num_extended_points", &tls_registration.ndt.num_extended_points, 1, 1);
        ImGui::PopItemWidth();

        ImGui::Checkbox("compute_mean_and_cov_for_bucket", &tls_registration.compute_mean_and_cov_for_bucket);
    }
    ImGui::EndDisabled();

    ImGui::Text("Set error presets:");
    ImGui::Text("Zoller+Fröhlich TLS Imager");
    ImGui::SameLine();
    if (ImGui::Button("5006i"))
        tls_registration.set_zoller_frohlich_tls_imager_5006i_errors();
    ImGui::SameLine();
    if (ImGui::Button("5010C"))
        tls_registration.set_zoller_frohlich_tls_imager_5010c_errors();
    ImGui::SameLine();
    if (ImGui::Button("5016"))
        tls_registration.set_zoller_frohlich_tls_imager_5016_errors();

    ImGui::Text("Leica");
    ImGui::SameLine();
    if (ImGui::Button("ScanStation C5 C10"))
        tls_registration.set_leica_scanstation_c5_c10_errors();
    ImGui::SameLine();
    if (ImGui::Button("Leica HDS6100"))
        tls_registration.set_leica_hds6100_errors();
    ImGui::SameLine();
    if (ImGui::Button("Leica P40"))
        tls_registration.set_leica_p40_errors();

    if (ImGui::Button("Faro Focus3D"))
        tls_registration.set_faro_focus3d_errors();
    ImGui::SameLine();
    if (ImGui::Button("Riegl VZ400"))
        tls_registration.set_riegl_vz400_errors();
    ImGui::SameLine();
    if (ImGui::Button("Livox mid360"))
        tls_registration.set_livox_mid360_errors();
}

void icp_gui()
{
    ImGui::PushItemWidth(ImGuiNumberWidth);
    ImGui::InputFloat("Search radius", &tls_registration.icp.search_radius, 0.01f, 0.1f);
    if (tls_registration.icp.search_radius < 0.01f)
        tls_registration.icp.search_radius = 0.01f;
    if (tls_registration.icp.search_radius > 2.0f)
        tls_registration.icp.search_radius = 2.0f;

    ImGui::InputInt("Number of threads", &tls_registration.icp.number_of_threads);
    if (tls_registration.icp.number_of_threads < 1)
        tls_registration.icp.number_of_threads = 1;
    ImGui::SameLine();
    ImGui::InputInt("Number of iterations", &tls_registration.icp.number_of_iterations);
    if (tls_registration.icp.number_of_iterations < 1)
        tls_registration.icp.number_of_iterations = 1;
    ImGui::PopItemWidth();

    ImGui::Checkbox("Adaptive robust kernel", &tls_registration.icp.is_adaptive_robust_kernel);
    ImGui::SameLine();
    ImGui::Checkbox("Fix first node (add I to first pose in Hessian)", &tls_registration.icp.is_fix_first_node);

    ImGui::Text("Nonlinear optimization method:");
    ImGui::SameLine();
    ImGui::RadioButton("Gauss-Newton", &ICPnomSelection, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Levenberg-Marguardt", &ICPnomSelection, 1);

    tls_registration.icp.is_gauss_newton = (ICPnomSelection == 0);
    tls_registration.icp.is_levenberg_marguardt = (ICPnomSelection == 1);

    ImGui::Text("Poses expressed as:");
    ImGui::SameLine();
    ImGui::RadioButton("camera<-world (cw)", &ICPpeSelection, 0);
    ImGui::SameLine();
    ImGui::RadioButton("camera->world (wc)", &ICPpeSelection, 1);

    tls_registration.icp.is_cw = (ICPpeSelection == 0);
    tls_registration.icp.is_wc = (ICPpeSelection == 1);

    ImGui::Text("Parameterizations of 3D rotation:");
    ImGui::RadioButton("Tait-Bryan angles (om fi ka: RxRyRz)", &ICP3dSelection, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Quaternion (q0 q1 q2 q3)", &ICP3dSelection, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Rodrigues (sx sy sz)", &ICP3dSelection, 2);

    tls_registration.icp.is_tait_bryan_angles = (ICP3dSelection == 0);
    tls_registration.icp.is_quaternion = (ICP3dSelection == 1);
    tls_registration.icp.is_rodrigues = (ICP3dSelection == 2);

    if (ImGui::Button("Optimization point to point source to target"))
        tls_registration.icp.optimization_point_to_point_source_to_target(session.point_clouds_container);

    ImGui::Separator();

    ImGui::Text("Optimization source to target Lie-algebra:");
    ImGui::SameLine();
    if (ImGui::Button("left Jacobian"))
        tls_registration.icp.optimize_source_to_target_lie_algebra_left_jacobian(session.point_clouds_container);

    ImGui::SameLine();
    if (ImGui::Button("right Jacobian"))
        tls_registration.icp.optimize_source_to_target_lie_algebra_right_jacobian(session.point_clouds_container);

    ImGui::Separator();

    if (ImGui::Button("Compute RMS (optimization_point_to_point_source_to_target)"))
    {
        double rms = 0.0;
        tls_registration.icp.optimization_point_to_point_source_to_target_compute_rms(session.point_clouds_container, rms);
        spdlog::info("RMS (optimization_point_to_point_source_to_target): {}", rms);
    }
}

void rpf_gui()
{
    ImGui::PushItemWidth(ImGuiNumberWidth);
    ImGui::InputFloat("Search radius", &tls_registration.registration_plane_feature.search_radius, 0.01, 2.0);
    if (tls_registration.registration_plane_feature.search_radius < 0.01)
        tls_registration.registration_plane_feature.search_radius = 0.01;
    if (tls_registration.registration_plane_feature.search_radius > 2.0)
        tls_registration.registration_plane_feature.search_radius = 2.0;

    ImGui::InputInt("Number of threads", &tls_registration.registration_plane_feature.number_of_threads);
    if (tls_registration.registration_plane_feature.number_of_threads < 1)
        tls_registration.registration_plane_feature.number_of_threads = 1;
    ImGui::SameLine();
    ImGui::InputInt("Number of iterations", &tls_registration.registration_plane_feature.number_of_iterations);
    if (tls_registration.registration_plane_feature.number_of_iterations < 1)
        tls_registration.registration_plane_feature.number_of_iterations = 1;
    ImGui::PopItemWidth();

    ImGui::Checkbox("Adaptive robust kernel", &tls_registration.registration_plane_feature.is_adaptive_robust_kernel);
    ImGui::SameLine();
    ImGui::Checkbox("Fix first node (add I to first pose in Hessian)", &tls_registration.registration_plane_feature.is_fix_first_node);

    ImGui::Text("Nonlinear optimization method:");
    ImGui::SameLine();
    ImGui::RadioButton("Gauss-Newton", &RPFnomSelection, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Levenberg-Marguardt", &RPFnomSelection, 1);

    tls_registration.registration_plane_feature.is_gauss_newton = (RPFnomSelection == 0);
    tls_registration.registration_plane_feature.is_levenberg_marguardt = (RPFnomSelection == 1);

    ImGui::Text("Poses expressed as:");
    ImGui::SameLine();
    ImGui::RadioButton("camera<-world (cw)", &RPFpeSelection, 0);
    ImGui::SameLine();
    ImGui::RadioButton("camera->world (wc)", &RPFpeSelection, 1);

    tls_registration.registration_plane_feature.is_cw = (RPFpeSelection == 0);
    tls_registration.registration_plane_feature.is_wc = (RPFpeSelection == 1);

    ImGui::Text("Parameterizations of 3D rotation:");
    ImGui::RadioButton("Tait-Bryan angles (om fi ka: RxRyRz)", &RPF3dSelection, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Quaternion (q0 q1 q2 q3)", &RPF3dSelection, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Rodrigues (sx sy sz)", &RPF3dSelection, 2);

    tls_registration.registration_plane_feature.is_tait_bryan_angles = (RPF3dSelection == 0);
    tls_registration.registration_plane_feature.is_quaternion = (RPF3dSelection == 1);
    tls_registration.registration_plane_feature.is_rodrigues = (RPF3dSelection == 2);

    ImGui::Separator();
    ImGui::Text("Optimize point to projection onto plane source to target:");
    if (ImGui::Button("Basic Jacobian"))
        tls_registration.registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(
            session.point_clouds_container);
    ImGui::SameLine();
    if (ImGui::Button("Lie-algebra left Jacobian"))
        tls_registration.registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_left_jacobian(
            session.point_clouds_container);
    ImGui::SameLine();
    if (ImGui::Button("Lie-algebra right Jacobian"))
        tls_registration.registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_right_jacobian(
            session.point_clouds_container);

    ImGui::Separator();

    ImGui::Text("Optimize source to target:");
    if (ImGui::Button("point to plane (using dot product)"))
        tls_registration.registration_plane_feature.optimize_point_to_plane_source_to_target(session.point_clouds_container);
    ImGui::SameLine();
    if (ImGui::Button("distance point to plane"))
        tls_registration.registration_plane_feature.optimize_distance_point_to_plane_source_to_target(session.point_clouds_container);
    ImGui::SameLine();
    if (ImGui::Button("plane to plane"))
        tls_registration.registration_plane_feature.optimize_plane_to_plane_source_to_target(session.point_clouds_container);
}

void pca_gui()
{
    ImGui::Begin("Point cloud alignment", &is_pca_gui, ImGuiWindowFlags_MenuBar);
    {
        if (ImGui::BeginMenuBar())
        {
            bool justPushed = false;

            if (is_ndt_gui)
                ImGui::PushStyleColor(ImGuiCol_Button, orangeBorder);
            if (ImGui::Button("Normal Distributions Transform"))
            {
                if (!is_ndt_gui)
                {
                    is_ndt_gui = true;
                    is_icp_gui = false;
                    is_rpf_gui = false;
                    justPushed = true;
                }
            }
            if (is_ndt_gui && !justPushed)
                ImGui::PopStyleColor();
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
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

            ImGui::SameLine();

            if (is_icp_gui)
                ImGui::PushStyleColor(ImGuiCol_Button, orangeBorder);
            if (ImGui::Button("Iterative Closest Point"))
            {
                if (!is_icp_gui)
                {
                    is_ndt_gui = false;
                    is_icp_gui = true;
                    is_rpf_gui = false;
                    justPushed = true;
                }
            }
            if (is_icp_gui && !justPushed)
                ImGui::PopStyleColor();
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                ImGui::Text(
                    "Geometric registration algorithm that aligns two point clouds\nby minimizing the Euclidean distances between "
                    "corresponding points");
                ImGui::Text(
                    "Very precise at local refinement, especially point-to-plane ICP,\nbut it struggles if the starting alignment "
                    "is too far off");
                ImGui::EndTooltip();
            }

            ImGui::SameLine();

            if (is_rpf_gui)
                ImGui::PushStyleColor(ImGuiCol_Button, orangeBorder);
            if (ImGui::Button("Registration Plane Feature"))
            {
                if (!is_rpf_gui)
                {
                    is_ndt_gui = false;
                    is_icp_gui = false;
                    is_rpf_gui = true;
                    justPushed = true;
                }
            }
            if (is_rpf_gui && !justPushed)
                ImGui::PopStyleColor();
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                ImGui::Text(
                    "Feature based registration technique that uses detected\nplanar surfaces in the environment (walls, floors, "
                    "ceilings, etc.) as constraints for alignment");
                ImGui::Text(
                    "Can be much more robust to noise and partial overlap,\nrequire far fewer correspondences (just a few planes "
                    "can define a full 6 DOF pose),\nhandle low texture regions better than ICP");
                ImGui::EndTooltip();
            }

            ImGui::EndMenuBar();
        }

        if (is_ndt_gui)
            ndt_gui();
        if (is_icp_gui)
            icp_gui();
        if (is_rpf_gui)
            rpf_gui();
    }

    ImGui::End();
}

void pose_graph_slam_gui()
{
    ImGui::Begin("Pose Graph SLAM", &is_pose_graph_slam);
    {
        ImGui::PushItemWidth(ImGuiNumberWidth);
        ImGui::InputFloat("Search radius", &tls_registration.pose_graph_slam.search_radius, 0.01f, 2.0f);
        if (tls_registration.pose_graph_slam.search_radius < 0.01f)
            tls_registration.pose_graph_slam.search_radius = 0.01f;

        ImGui::InputInt("Number of threads", &tls_registration.pose_graph_slam.number_of_threads);
        if (tls_registration.pose_graph_slam.number_of_threads < 1)
            tls_registration.pose_graph_slam.number_of_threads = 1;

        ImGui::InputInt(
            "Number of iterations (pair wise matching)", &tls_registration.pose_graph_slam.number_of_iterations_pair_wise_matching);
        if (tls_registration.pose_graph_slam.number_of_iterations_pair_wise_matching < 1)
            tls_registration.pose_graph_slam.number_of_iterations_pair_wise_matching = 1;

        ImGui::InputFloat("Overlap threshold", &tls_registration.pose_graph_slam.overlap_threshold, 0.1f, 0.8f);
        if (tls_registration.pose_graph_slam.overlap_threshold < 0.1f)
            tls_registration.pose_graph_slam.overlap_threshold = 0.1f;
        ImGui::PopItemWidth();

        // ImGui::Checkbox("pgslam adaptive_robust_kernel", &pose_graph_slam.icp.is_adaptive_robust_kernel);

        //--
        ImGui::Checkbox("Adaptive robust kernel", &tls_registration.pose_graph_slam.is_adaptive_robust_kernel);
        ImGui::SameLine();
        ImGui::Checkbox("Fix first node (add I to first pose in Hessian)", &tls_registration.pose_graph_slam.is_fix_first_node);

        ImGui::Text("Nonlinear optimization method:");
        ImGui::SameLine();
        ImGui::RadioButton("Gauss-Newton", &PGSnomSelection, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Levenberg-Marguardt", &PGSnomSelection, 1);

        tls_registration.pose_graph_slam.is_gauss_newton = (PGSnomSelection == 0);
        tls_registration.pose_graph_slam.is_levenberg_marguardt = (PGSnomSelection == 1);

        ImGui::Text("Poses expressed as:");
        ImGui::SameLine();
        ImGui::RadioButton("camera<-world (cw)", &PGSpeSelection, 0);
        ImGui::SameLine();
        ImGui::RadioButton("camera->world (wc)", &PGSpeSelection, 1);

        tls_registration.pose_graph_slam.is_cw = (PGSpeSelection == 0);
        tls_registration.pose_graph_slam.is_wc = (PGSpeSelection == 1);

        ImGui::Text("Parameterizations of 3D rotation:");
        ImGui::RadioButton("Tait-Bryan angles (om fi ka: RxRyRz)", &PGS3dSelection, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Quaternion (q0 q1 q2 q3)", &PGS3dSelection, 1);
        ImGui::SameLine();
        ImGui::RadioButton("Rodrigues (sx sy sz)", &PGS3dSelection, 2);

        tls_registration.pose_graph_slam.is_tait_bryan_angles = (PGS3dSelection == 0);
        tls_registration.pose_graph_slam.is_quaternion = (PGS3dSelection == 1);
        tls_registration.pose_graph_slam.is_rodrigues = (PGS3dSelection == 2);

        ImGui::Separator();

        ImGui::Text("Method for pair wise matching (general):");
        ImGui::RadioButton("NDT", &PGSpwmtSelection, 0);
        ImGui::RadioButton("Optimization_point_to_point_source_to_target", &PGSpwmtSelection, 1);
        ImGui::RadioButton("Optimize_point_to_projection_onto_plane_source_to_target", &PGSpwmtSelection, 2);
        ImGui::RadioButton("Optimize_point_to_plane_source_to_target", &PGSpwmtSelection, 3);
        ImGui::RadioButton("Optimize_distance_point_to_plane_source_to_target", &PGSpwmtSelection, 4);
        ImGui::RadioButton("Optimize_plane_to_plane_source_to_target", &PGSpwmtSelection, 5);

        ImGui::Separator();

        ImGui::Text("Method for pair wise matching (with Lie-algebra):");
        ImGui::RadioButton("Optimize NDT (Lie-algebra left Jacobian)", &PGSpwmtSelection, 6);
        ImGui::RadioButton("Optimize NDT (Lie-algebra right Jacobian)", &PGSpwmtSelection, 7);
        ImGui::RadioButton("Optimize point to point source to target (Lie-algebra left Jacobian)", &PGSpwmtSelection, 8);
        ImGui::RadioButton("Optimize point to point source to target (Lie-algebra right Jacobian)", &PGSpwmtSelection, 9);
        ImGui::RadioButton("Optimize point to projection onto plane source to target (Lie-algebra left Jacobian)", &PGSpwmtSelection, 10);
        ImGui::RadioButton("Optimize point to projection onto plane source to target (Lie-algebra right Jacobian)", &PGSpwmtSelection, 11);

#ifdef WITH_PCL
        ImGui::Separator();
        ImGui::Text("Method for pair wise matching (with PCL):");
        ImGui::RadioButton("Optimize with PCL (NDT based pair wise matching)", &PGSpwmtSelection, 12);
        ImGui::RadioButton("Optimize with PCL (ICP based pair wise matching)", &PGSpwmtSelection, 13);
#endif

        // tls_registration.pose_graph_slam.set_all_to_false();
        tls_registration.pose_graph_slam.is_ndt = (PGSpwmtSelection == 0);
        tls_registration.pose_graph_slam.is_optimization_point_to_point_source_to_target = (PGSpwmtSelection == 1);
        tls_registration.pose_graph_slam.is_optimize_point_to_projection_onto_plane_source_to_target = (PGSpwmtSelection == 2);
        tls_registration.pose_graph_slam.is_optimize_point_to_plane_source_to_target = (PGSpwmtSelection == 3);
        tls_registration.pose_graph_slam.is_optimize_distance_point_to_plane_source_to_target = (PGSpwmtSelection == 4);
        tls_registration.pose_graph_slam.is_optimize_plane_to_plane_source_to_target = (PGSpwmtSelection == 5);

        tls_registration.pose_graph_slam.is_ndt_lie_algebra_left_jacobian = (PGSpwmtSelection == 6);
        tls_registration.pose_graph_slam.is_ndt_lie_algebra_right_jacobian = (PGSpwmtSelection == 7);
        tls_registration.pose_graph_slam.is_optimize_point_to_point_source_to_target_lie_algebra_left_jacobian = (PGSpwmtSelection == 8);
        tls_registration.pose_graph_slam.is_optimize_point_to_point_source_to_target_lie_algebra_right_jacobian = (PGSpwmtSelection == 9);
        tls_registration.pose_graph_slam.is_optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_left_jacobian =
            (PGSpwmtSelection == 10);
        tls_registration.pose_graph_slam.is_optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_right_jacobian =
            (PGSpwmtSelection == 11);

        tls_registration.pose_graph_slam.is_optimize_pcl_ndt = (PGSpwmtSelection == 12);
        tls_registration.pose_graph_slam.is_optimize_pcl_icp = (PGSpwmtSelection == 13);

        if (PGSpwmtSelection >= 0 && PGSpwmtSelection <= 11)
            tls_registration.pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
        if (PGSpwmtSelection == 6)
            tls_registration.pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::pcl_ndt;
        if (PGSpwmtSelection == 7)
            tls_registration.pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::pcl_icp;

        ImGui::Separator();
        if (ImGui::Button("Optimize"))
        {
            tls_registration.pose_graph_slam.ndt_bucket_size[0] = tls_registration.ndt.bucket_size[0];
            tls_registration.pose_graph_slam.ndt_bucket_size[1] = tls_registration.ndt.bucket_size[1];
            tls_registration.pose_graph_slam.ndt_bucket_size[2] = tls_registration.ndt.bucket_size[2];
            // double rms_initial = 0.0;
            // double rms_final = 0.0;
            // double mui = 0.0;
            tls_registration.pose_graph_slam.optimize(session.point_clouds_container);
            // pose_graph_slam.optimize(point_clouds_container, rms_initial, rms_final, mui);
            // spdlog::info("mean uncertainty impact: " << mui << " rms_initial: " << rms_initial << " rms_final: " << rms_final <<
            // std::endl;
        }

#if WITH_GTSAM
        if (ImGui::Button("Optimize with GTSAM"))
        {
            ImGui::Separator();
            ImGui::Text("With GTSAM:");
            tls_registration.pose_graph_slam.ndt_bucket_size[0] = tls_registration.ndt.bucket_size[0];
            tls_registration.pose_graph_slam.ndt_bucket_size[1] = tls_registration.ndt.bucket_size[1];
            tls_registration.pose_graph_slam.ndt_bucket_size[2] = tls_registration.ndt.bucket_size[2];
            double rms_initial = 0.0;
            double rms_final = 0.0;
            double mui = 0.0;
            tls_registration.pose_graph_slam.optimize_with_GTSAM(session.point_clouds_container);
            // spdlog::info("mean uncertainty impact: " << mui << " rms_initial: " << rms_initial << " rms_final: " << rms_final <<
            // std::endl;
        }
#endif

#if WITH_MANIF
        ImGui::Separator();
        ImGui::Text("With MANIF:");
        if (ImGui::Button("Optimize with manif (a small header-only library for Lie theory)"))
        {
            tls_registration.pose_graph_slam.optimize_with_manif(session.point_clouds_container);
            spdlog::info("Optimize with manif (a small header-only library for Lie theory) DONE" << std::endl;
        }
#endif
    }

    ImGui::End();
}

void observation_picking_gui()
{
    static std::string observations_file_name = "";

    ImGui::Begin("Observations", &is_manual_analisys);
    {
        ImGui::Checkbox("Observation picking mode", &observation_picking.is_observation_picking_mode);
        ImGui::BeginDisabled(!observation_picking.is_observation_picking_mode);
        {
            ImGui::Text("Grid [m]:");
            ImGui::Checkbox("10x10", &observation_picking.grid10x10m);
            ImGui::SameLine();
            ImGui::Checkbox("1x1", &observation_picking.grid1x1m);
            ImGui::SameLine();
            ImGui::Checkbox("0.1x0.1", &observation_picking.grid01x01m);
            ImGui::SameLine();
            ImGui::Checkbox("0.01x0.01", &observation_picking.grid001x001m);

            ImGui::Text("Picking plane:");
            ImGui::PushItemWidth(ImGuiNumberWidth);
            // ImGui::SliderFloat("picking_plane_height", &observation_picking.picking_plane_height, -20.0f, 20.0f);
            ImGui::InputFloat("Height [m]", &observation_picking.picking_plane_height);
            // ImGui::SliderFloat("picking_plane_threshold", &observation_picking.picking_plane_threshold, 0.01f, 200.0f);
            ImGui::InputFloat("Threshold [m]", &observation_picking.picking_plane_threshold);
            // ImGui::SliderFloat("picking_plane_max_xy", &observation_picking.max_xy, 10.0f, 1000.0f);
            ImGui::InputFloat("Grid size [m]", &observation_picking.max_xy);
            // ImGui::SliderInt("point_size", &observation_picking.point_size, 1, 10);
            ImGui::InputInt("Point size", &observation_picking.point_size);
            ImGui::PopItemWidth();
            if (observation_picking.point_size < 1)
                observation_picking.point_size = 1;
            if (observation_picking.point_size > 20)
                observation_picking.point_size = 20;

            if (ImGui::Button("Accept current observation"))
            {
                std::vector<Eigen::Affine3d> m_poses;
                for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
                    m_poses.push_back(session.point_clouds_container.point_clouds[i].m_pose);
                observation_picking.accept_current_observation(m_poses);
            }
            ImGui::SameLine();
            if (ImGui::Button("Clear current observation"))
                observation_picking.current_observation.clear();

            if (ImGui::Button("Reset view"))
            {
                app_state.camera.startEulerTransition(0.0f, 0.0f, app_state.camera.euler.translate, app_state.camera.euler.rotationCenter);
            }
        }
        ImGui::EndDisabled();

        ImGui::Text((std::string("Number of observations: ") + std::to_string(observation_picking.observations.size())).c_str());

        if (ImGui::Button("Load observations"))
        {
            std::string input_file_name = "";
            input_file_name = mandeye::fd::OpenFileDialogOneFile("Load observations", {});

            if (input_file_name.size() > 0)
            {
                observations_file_name = input_file_name;
                observation_picking.import_observations(input_file_name);

                for (const auto& obs : observation_picking.observations)
                {
                    for (const auto& [key, value] : obs)
                    {
                        if (session.point_clouds_container.show_with_initial_pose)
                        {
                            auto p = session.point_clouds_container.point_clouds[key].m_initial_pose * value;
                            observation_picking.add_intersection(p);
                        }
                        else
                        {
                            auto p = session.point_clouds_container.point_clouds[key].m_pose * value;
                            observation_picking.add_intersection(p);
                        }
                        break;
                    }
                }
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Save observations"))
        {
            const auto output_file_name = mandeye::fd::SaveFileDialog("Save observations", {}, ".json");
            spdlog::info("JSON file to save: '{}'", output_file_name);

            if (output_file_name.size() > 0)
                observation_picking.export_observation(output_file_name);
        }

        ImGui::Text((std::string("Loaded observations from file: '") + observations_file_name + std::string("'")).c_str());

        if (ImGui::Button("Compute RMS (xy)"))
        {
            double rms = compute_rms(true, session, observation_picking);
            spdlog::info("RMS (initial poses): {}", rms);
            rms = compute_rms(false, session, observation_picking);
            spdlog::info("RMS (current poses): {}", rms);
        }

        ImGui::Separator();
        if (ImGui::Button("Add intersection"))
            observation_picking.add_intersection(Eigen::Vector3d(0.0, 0.0, 0.0));

        int index_intersection_to_remove = -1;
        for (size_t i = 0; i < observation_picking.intersections.size(); i++)
        {
            ImGui::Separator();
            ImGui::SetWindowFontScale(1.25f);
            ImGui::Text("Intersection %zu", i);
            ImGui::SetWindowFontScale(1.0f);
            ImGui::SameLine();
            ImGui::ColorEdit3(
                std::string("Color##" + std::to_string(i)).c_str(),
                observation_picking.intersections[i].color,
                ImGuiColorEditFlags_NoInputs);
            ImGui::SameLine();
            if (ImGui::Button(std::string("Remove##" + std::to_string(i)).c_str()))
                index_intersection_to_remove = i;

            ImGui::InputFloat3(
                std::string("Translation [m]##" + std::to_string(i)).c_str(), observation_picking.intersections[i].translation);
            ImGui::InputFloat3(std::string("Rotation [deg]##" + std::to_string(i)).c_str(), observation_picking.intersections[i].rotation);
            ImGui::InputFloat3(
                std::string("Width length height [m]##" + std::to_string(i)).c_str(),
                observation_picking.intersections[i].width_length_height);
        }

        if (index_intersection_to_remove != -1)
        {
            std::vector<Intersection> intersections;
            for (size_t i = 0; i < observation_picking.intersections.size(); i++)
            {
                if (i != index_intersection_to_remove)
                    intersections.push_back(observation_picking.intersections[i]);
            }
            observation_picking.intersections = intersections;
        }

        ImGui::Separator();

        ImGui::BeginDisabled(observation_picking.intersections.size() <= 0);
        {
            if (ImGui::Button("Export point clouds inside intersections, RMS and poses"))
            {
                std::string output_folder_name = "";
                output_folder_name = mandeye::fd::SelectFolder("Choose folder");
                spdlog::info("folder: '{}'", output_folder_name);

                if (output_folder_name.size() > 0)
                    export_result_to_folder(output_folder_name, observation_picking, session);
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("in RESSO format to folder");
            ImGui::SetNextItemWidth(ImGuiNumberWidth);
            ImGui::InputFloat("Label distance [m]", &observation_picking.label_dist);
        }
        ImGui::EndDisabled();
    }

    ImGui::End();
}

void loop_closure_gui()
{
    ImGui::Begin("Manual Pose Graph Loop Closure", &is_loop_closure_gui);
    {
        const auto point_cloud_upper = session.point_clouds_container.point_clouds.size() - 1;

        ImGui::Checkbox("Render source as red target as blue", &session.pose_graph_loop_closure.render_source_as_red_target_as_blue);

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

        int prev_index_active_edge = session.pose_graph_loop_closure.index_active_edge;
        session.pose_graph_loop_closure.Gui(
            session.point_clouds_container,
            index_loop_closure_source,
            index_loop_closure_target,
            m_gizmo,
            tls_registration.gnss,
            tls_registration.tum,
            session.ground_control_points,
            session.control_points,
            num_edge_extended_before,
            num_edge_extended_after);

        new_loop_closure_index = (prev_index_active_edge != session.pose_graph_loop_closure.index_active_edge);
    }

    ImGui::End();
}

void lio_segments_gui()
{
    ImGui::Begin("LIO segments editor", &is_lio_segments_gui);
    {
        ImGui::Text("index from: ");
        ImGui::SameLine();
        ImGui::PushItemWidth(ImGuiNumberWidth);
        ImGui::SliderInt("##fs", &index_begin, 0, index_end);
        ImGui::SameLine();
        ImGui::InputInt("##fi", &index_begin, 1, 5);
        if (index_begin < 0)
            index_begin = 0;
        if (index_begin >= index_end)
            index_begin = index_end;

        ImGui::SameLine();
        ImGui::Text(" to: ");
        ImGui::SameLine();

        ImGui::SliderInt("##ts", &index_end, index_begin, static_cast<int>(session.point_clouds_container.point_clouds.size() - 1));
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("max %zu", session.point_clouds_container.point_clouds.size() - 1);
        ImGui::SameLine();
        ImGui::InputInt("##ti", &index_end, 1, 5);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("max %zu", session.point_clouds_container.point_clouds.size() - 1);
        if (index_end < index_begin)
            index_end = index_begin;
        if (index_end >= session.point_clouds_container.point_clouds.size() - 1)
            index_end = session.point_clouds_container.point_clouds.size() - 1;
        ImGui::PopItemWidth();

        ImGui::Text("Selection: ");
        ImGui::SameLine();
        if (ImGui::Button("show <index from, index to>"))
            session.point_clouds_container.show_all_from_range(index_begin, index_end);
        ImGui::SameLine();
        if (ImGui::Button("shift -"))
        {
            int step = index_end - index_begin;
            index_begin -= step;
            index_end -= step;

            if (index_begin < 0)
                index_begin = 0;
            if (index_end < 0)
                index_end = 0;

            app_state.camera.euler.rotationCenter.x = (float)session.point_clouds_container.point_clouds[index_begin].m_pose(0, 3);
            app_state.camera.euler.rotationCenter.y = (float)session.point_clouds_container.point_clouds[index_begin].m_pose(1, 3);
            app_state.camera.euler.rotationCenter.z = (float)session.point_clouds_container.point_clouds[index_begin].m_pose(2, 3);
            session.point_clouds_container.show_all_from_range(index_begin, index_end);
        }
        ImGui::SameLine();
        if (ImGui::Button("shift +"))
        {
            int step = index_end - index_begin;
            index_begin += step;
            index_end += step;

            if (index_begin > session.point_clouds_container.point_clouds.size() - 1)
                index_begin = session.point_clouds_container.point_clouds.size() - 1;
            if (index_end > session.point_clouds_container.point_clouds.size() - 1)
                index_end = session.point_clouds_container.point_clouds.size() - 1;

            app_state.camera.euler.rotationCenter.x = (float)session.point_clouds_container.point_clouds[index_begin].m_pose(0, 3);
            app_state.camera.euler.rotationCenter.y = (float)session.point_clouds_container.point_clouds[index_begin].m_pose(1, 3);
            app_state.camera.euler.rotationCenter.z = (float)session.point_clouds_container.point_clouds[index_begin].m_pose(2, 3);
            session.point_clouds_container.show_all_from_range(index_begin, index_end);
        }
        ImGui::SameLine();
        if (ImGui::Button("Show all"))
            session.point_clouds_container.show_all();
        ImGui::SameLine();
        if (ImGui::Button("Hide all"))
            session.point_clouds_container.hide_all();
        ImGui::SameLine();
        if (ImGui::Button("Reset poses"))
            reset_poses(session);

        ImGui::Checkbox("Show with initial pose", &session.point_clouds_container.show_with_initial_pose);
        ImGui::SameLine();
        ImGui::Checkbox("Manipulate only marked gizmo", &manipulate_only_marked_gizmo);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("false: move also succesive nodes");

        ImGui::SameLine();

        if (ImGui::Button("Set fuse IMU inclination for picked trajectory node"))
        {
            if (index_loop_closure_target >= 0 && index_loop_closure_target < session.point_clouds_container.point_clouds.size())
            {
                session.point_clouds_container.point_clouds[index_loop_closure_target].fuse_inclination_from_IMU = true;
            }

            // picked_index
            /*int tmp = -1;
            getClosestTrajectoryPoint(session, x, y, false, tmp);

            if (io.KeyCtrl)
            {
                if (tmp != -1)
                    index_loop_closure_target = tmp;
            }
            else if (io.KeyShift)
            {
                if (tmp != -1)
                    index_loop_closure_source = tmp;
            }

            januszjanusz*/
        }

        ImGui::Text("Fuse IMU inclination: ");
        ImGui::SameLine();

        static double angle_diff = 5.0;

        if (ImGui::Button("Set those that satisfy acceptable angle"))
        {
            for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
            {
                double om = session.point_clouds_container.point_clouds[i].local_trajectory[0].imu_om_fi_ka.x() * RAD_TO_DEG;
                double fi = session.point_clouds_container.point_clouds[i].local_trajectory[0].imu_om_fi_ka.y() * RAD_TO_DEG;

                spdlog::info("om: {}, fi {}", om, fi);
                if (fabs(om) > angle_diff || fabs(fi) > angle_diff)
                {
                }
                else
                    session.point_clouds_container.point_clouds[i].fuse_inclination_from_IMU = true;
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("unset all"))
        {
            for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
                session.point_clouds_container.point_clouds[i].fuse_inclination_from_IMU = false;
        }

        ImGui::SameLine();
        ImGui::SetNextItemWidth(ImGuiNumberWidth);
        ImGui::InputDouble("acceptable angle [deg]", &angle_diff);

        ImGui::Separator();
        // ImGui::Text("motion model");

        // session.pose_graph_loop_closure.edges.

        // ImGui::InputDouble("motion_model_w_px_1_sigma_m", &session.pose_graph_loop_closure.motion_model_w_px_1_sigma_m);
        // ImGui::InputDouble("motion_model_w_py_1_sigma_m", &session.pose_graph_loop_closure.motion_model_w_py_1_sigma_m);
        // ImGui::InputDouble("motion_model_w_pz_1_sigma_m", &session.pose_graph_loop_closure.motion_model_w_pz_1_sigma_m);
        // ImGui::InputDouble("motion_model_w_om_1_sigma_deg", &session.pose_graph_loop_closure.motion_model_w_om_1_sigma_deg);
        // ImGui::InputDouble("motion_model_w_fi_1_sigma_deg", &session.pose_graph_loop_closure.motion_model_w_fi_1_sigma_deg);
        // ImGui::InputDouble("motion_model_w_ka_1_sigma_deg", &session.pose_graph_loop_closure.motion_model_w_ka_1_sigma_deg);

        // ImGui::Separator();

        ImGui::BeginChild("LIO segments", ImVec2(0, 0), true);
        {
            for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
            {
                if (i > 0)
                    ImGui::Separator();
                ImGui::SetWindowFontScale(1.25f);
                ImGui::Checkbox(
                    std::filesystem::path(session.point_clouds_container.point_clouds[i].file_name).filename().string().c_str(),
                    &session.point_clouds_container.point_clouds[i].visible);
                ImGui::SetWindowFontScale(1.0f);
                ImGui::SameLine();
                ImGui::Checkbox(("gizmo##" + std::to_string(i)).c_str(), &session.point_clouds_container.point_clouds[i].gizmo);

#if 0
                ImGui::SameLine();
                ImGui::Checkbox(("fixed##" + std::to_string(i)).c_str(), &session.point_clouds_container.point_clouds[i].fixed);
                ImGui::SameLine();
                ImGui::PushButtonRepeat(true);
                float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
                if (ImGui::ArrowButton(("left##" + std::to_string(i)).c_str(), ImGuiDir_Left))
                {
                    (session.point_clouds_container.point_clouds[i].point_size)--;
                }
                ImGui::SameLine(0.0f, spacing);
                if (ImGui::ArrowButton(("right##" + std::to_string(i)).c_str(), ImGuiDir_Right))
                {
                    (session.point_clouds_container.point_clouds[i].point_size)++;
                }
                ImGui::PopButtonRepeat();
                ImGui::SameLine();
                ImGui::Text("point size %d", session.point_clouds_container.point_clouds[i].point_size);
                if (session.point_clouds_container.point_clouds[i].point_size < 1)
                {
                    session.point_clouds_container.point_clouds[i].point_size = 1;
                }

                ImGui::SameLine();
                if (ImGui::Button(std::string("#" + std::to_string(i) + " save scan(global reference frame)").c_str()))
                {
                    const auto output_file_name = mandeye::fd::SaveFileDialog("Choose folder", {});
                    spdlog::info("Scan file to save: '" << output_file_name << "'" << std::endl;
                    if (output_file_name.size() > 0)
                    {
                        session.point_clouds_container.point_clouds[i].save_as_global(output_file_name);
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button(std::string("#" + std::to_string(i) + " shift points to center").c_str()))
                {
                    session.point_clouds_container.point_clouds[i].shift_to_center();
                }
#endif
                if (session.point_clouds_container.point_clouds[i].gizmo)
                {
                    for (size_t j = 0; j < session.point_clouds_container.point_clouds.size(); j++)
                    {
                        if (i != j)
                        {
                            session.point_clouds_container.point_clouds[j].gizmo = false;
                        }
                    }
                    m_gizmo[0] = (float)session.point_clouds_container.point_clouds[i].m_pose(0, 0);
                    m_gizmo[1] = (float)session.point_clouds_container.point_clouds[i].m_pose(1, 0);
                    m_gizmo[2] = (float)session.point_clouds_container.point_clouds[i].m_pose(2, 0);
                    m_gizmo[3] = (float)session.point_clouds_container.point_clouds[i].m_pose(3, 0);
                    m_gizmo[4] = (float)session.point_clouds_container.point_clouds[i].m_pose(0, 1);
                    m_gizmo[5] = (float)session.point_clouds_container.point_clouds[i].m_pose(1, 1);
                    m_gizmo[6] = (float)session.point_clouds_container.point_clouds[i].m_pose(2, 1);
                    m_gizmo[7] = (float)session.point_clouds_container.point_clouds[i].m_pose(3, 1);
                    m_gizmo[8] = (float)session.point_clouds_container.point_clouds[i].m_pose(0, 2);
                    m_gizmo[9] = (float)session.point_clouds_container.point_clouds[i].m_pose(1, 2);
                    m_gizmo[10] = (float)session.point_clouds_container.point_clouds[i].m_pose(2, 2);
                    m_gizmo[11] = (float)session.point_clouds_container.point_clouds[i].m_pose(3, 2);
                    m_gizmo[12] = (float)session.point_clouds_container.point_clouds[i].m_pose(0, 3);
                    m_gizmo[13] = (float)session.point_clouds_container.point_clouds[i].m_pose(1, 3);
                    m_gizmo[14] = (float)session.point_clouds_container.point_clouds[i].m_pose(2, 3);
                    m_gizmo[15] = (float)session.point_clouds_container.point_clouds[i].m_pose(3, 3);
                }

                if (session.point_clouds_container.point_clouds[i].visible)
                {
                    ImGui::SameLine();
                    ImGui::ColorEdit3(
                        ("color##" + std::to_string(i)).c_str(),
                        session.point_clouds_container.point_clouds[i].render_color,
                        ImGuiColorEditFlags_NoInputs);

#if 0
                    ImGui::SameLine();
                    if (ImGui::Button(std::string(("ICP##" + std::to_string(i)).c_str())
                    {
                        size_t index_target = i;
                        PointClouds pcs;
                        for (size_t k = 0; k < index_target; k++)
                        {
                            if (session.point_clouds_container.point_clouds[k].visible)
                            {
                                pcs.point_clouds.push_back(session.point_clouds_container.point_clouds[k]);
                            }
                        }

                        if (pcs.point_clouds.size() > 0)
                        {
                            for (size_t k = 0; k < pcs.point_clouds.size(); k++)
                            {
                                pcs.point_clouds[k].fixed = true;
                            }
                        }
                        pcs.point_clouds.push_back(session.point_clouds_container.point_clouds[index_target]);
                        pcs.point_clouds[pcs.point_clouds.size() - 1].fixed = false;

                        ICP icp;
                        icp.search_radious = 0.3; // ToDo move to params
                        for (auto& pc : pcs.point_clouds)
                        {
                            pc.rgd_params.resolution_X = icp.search_radious;
                            pc.rgd_params.resolution_Y = icp.search_radious;
                            pc.rgd_params.resolution_Z = icp.search_radious;

                            pc.build_rgd();
                            pc.cout_rgd();
                            pc.compute_normal_vectors(0.5);
                        }

                        icp.number_of_threads = std::thread::hardware_concurrency();

                        icp.number_of_iterations = 10;
                        icp.is_adaptive_robust_kernel = false;

                        icp.is_ballanced_horizontal_vs_vertical = false;
                        icp.is_fix_first_node = false;
                        icp.is_gauss_newton = true;
                        icp.is_levenberg_marguardt = false;
                        icp.is_cw = false;
                        icp.is_wc = true;
                        icp.is_tait_bryan_angles = true;
                        icp.is_quaternion = false;
                        icp.is_rodrigues = false;
                        spdlog::info("optimization_point_to_point_source_to_target" << std::endl;

                        icp.optimization_point_to_point_source_to_target(pcs);

                        spdlog::info("pose before: " << session.point_clouds_container.point_clouds[index_target].m_pose.matrix() << std::endl;

                        std::vector<Eigen::Affine3d> all_m_poses;
                        for (size_t j = 0; j < session.point_clouds_container.point_clouds.size(); j++)
                        {
                            all_m_poses.push_back(session.point_clouds_container.point_clouds[j].m_pose);
                        }

                        session.point_clouds_container.point_clouds[index_target].m_pose = pcs.point_clouds[pcs.point_clouds.size() - 1].m_pose;

                        spdlog::info("pose after ICP: " << session.point_clouds_container.point_clouds[index_target].m_pose.matrix() << std::endl;

                        // like gizmo
                        if (!manipulate_only_marked_gizmo)
                        {
                            spdlog::info("Update all poses after current pose" << std::endl;

                            Eigen::Affine3d curr_m_pose = session.point_clouds_container.point_clouds[index_target].m_pose;
                            for (size_t j = index_target + 1; j < session.point_clouds_container.point_clouds.size(); j++)
                            {
                                curr_m_pose = curr_m_pose * (all_m_poses[j - 1].inverse() * all_m_poses[j]);
                                session.point_clouds_container.point_clouds[j].m_pose = curr_m_pose;
                            }
                        }
                    }
#endif

                    ImGui::SameLine();
                    ImGui::Checkbox(
                        ("fuse IMU inclination##" + std::to_string(i)).c_str(),
                        &session.point_clouds_container.point_clouds[i].fuse_inclination_from_IMU);

                    ImGui::SameLine();
                    ImGui::Checkbox(("show IMU##" + std::to_string(i)).c_str(), &session.point_clouds_container.point_clouds[i].show_IMU);

                    ImGui::SameLine();
                    ImGui::Checkbox(("show pose##" + std::to_string(i)).c_str(), &session.point_clouds_container.point_clouds[i].show_pose);

                    /*ImGui::SameLine();
                    if (ImGui::Button(("set IMU inclination##" + std::to_string(i)).c_str()))
                    {
                        //session.point_clouds_container.point_clouds[i].m_initial_pose
                        //session.point_clouds_container.point_clouds[i].m_pose
                        //session.point_clouds_container.point_clouds[i].m_pose_temp

                        TaitBryanPose target_pose =
                    pose_tait_bryan_from_affine_matrix(session.point_clouds_container.point_clouds[i].m_pose);

                        target_pose.om = session.point_clouds_container.point_clouds[i].local_trajectory[0].imu_om_fi_ka.x();
                        target_pose.fi = session.point_clouds_container.point_clouds[i].local_trajectory[0].imu_om_fi_ka.y();

                        Eigen::Affine3d m_pose = affine_matrix_from_pose_tait_bryan(target_pose);

                        session.point_clouds_container.point_clouds[i].m_initial_pose = m_pose;
                        session.point_clouds_container.point_clouds[i].m_pose = m_pose;
                        session.point_clouds_container.point_clouds[i].m_pose_temp = m_pose;

                        //session.point_clouds_container.point_clouds[i].m_pose = m_pose;
                        //session.point_clouds_container.point_clouds[i].pose =
                    pose_tait_bryan_from_affine_matrix(session.point_clouds_container.point_clouds[i].m_pose);
                        //session.point_clouds_container.point_clouds[i].gui_translation[0] =
                    session.point_clouds_container.point_clouds[i].pose.px;
                        //session.point_clouds_container.point_clouds[i].gui_translation[1] =
                    session.point_clouds_container.point_clouds[i].pose.py;
                        //session.point_clouds_container.point_clouds[i].gui_translation[2] =
                    session.point_clouds_container.point_clouds[i].pose.pz;
                        //session.point_clouds_container.point_clouds[i].gui_rotation[0] =
                    rad2deg(session.point_clouds_container.point_clouds[i].pose.om);
                        //session.point_clouds_container.point_clouds[i].gui_rotation[1] =
                    rad2deg(session.point_clouds_container.point_clouds[i].pose.fi);
                        //session.point_clouds_container.point_clouds[i].gui_rotation[2] =
                    rad2deg(session.point_clouds_container.point_clouds[i].pose.ka);
                    }*/

                    ImGui::Text("fixed: ");

                    ImGui::SameLine();
                    ImGui::Checkbox(("X##" + std::to_string(i)).c_str(), &session.point_clouds_container.point_clouds[i].fixed_x);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(xText);

                    ImGui::SameLine();
                    ImGui::Checkbox(("Y##" + std::to_string(i)).c_str(), &session.point_clouds_container.point_clouds[i].fixed_y);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(yText);

                    ImGui::SameLine();
                    ImGui::Checkbox(("Z##" + std::to_string(i)).c_str(), &session.point_clouds_container.point_clouds[i].fixed_z);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(zText);

                    ImGui::SameLine();
                    ImGui::Checkbox(("om##" + std::to_string(i)).c_str(), &session.point_clouds_container.point_clouds[i].fixed_om);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(omText);

                    ImGui::SameLine();
                    ImGui::Checkbox(("fi##" + std::to_string(i)).c_str(), &session.point_clouds_container.point_clouds[i].fixed_fi);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(fiText);

                    ImGui::SameLine();
                    ImGui::Checkbox(("ka##" + std::to_string(i)).c_str(), &session.point_clouds_container.point_clouds[i].fixed_ka);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(kaText);
                }
#if 0
                ImGui::SameLine();
                if (ImGui::Button(std::string("#" + std::to_string(i) + " print frame to console").c_str()))
                {
                    spdlog::info(session.point_clouds_container.point_clouds[i].m_pose.matrix() << std::endl;
                }
#endif
            }
        }
        ImGui::EndChild();
    }

    ImGui::End();
}

void loadSession(const std::string& session_file_name)
{
    spdlog::info("Session file: '{}'", session_file_name);

    if (session.load(
            fs::path(session_file_name).string(),
            tls_registration.is_decimate,
            tls_registration.bucket_x,
            tls_registration.bucket_y,
            tls_registration.bucket_z,
            tls_registration.calculate_offset))
    {
        session_loaded = true;
        index_begin = 0;
        index_end = session.point_clouds_container.point_clouds.size() - 1;

        std::string newTitle = winTitle + " - " + truncPath(session_file_name);
        SetWindowTitle(newTitle.c_str());

        for (const auto& pc : session.point_clouds_container.point_clouds)
            session_total_number_of_points += pc.points_local.size();

        session_dims = session.point_clouds_container.compute_point_cloud_dimension();

        scan_renderer.rebuildAll(session.point_clouds_container.point_clouds);
    }
}

// Accepts a Mandeye JSON Session file (*.mjs/*.json) -- shared by the drag & drop handler in main()'s loop and
// the CLI argv handling below, so both accept the same input and report unsupported drops the same way.
void loadSessionFromPath(const std::string& path)
{
    std::string ext = fs::path(path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    if (ext != ".mjs" && ext != ".json")
    {
        spdlog::error("Unsupported file dropped: '{}'", path);

        [[maybe_unused]] pfd::message message(
            "Load session", "Unsupported file:\n" + path + "\n\nDrop a session file (*.mjs/*.json).", pfd::choice::ok, pfd::icon::error);
        message.result();
        return;
    }

    session_file_name = path;
    loadSession(path);
}

void openSession()
{
    session_file_name = mandeye::fd::OpenFileDialogOneFile("Open session", mandeye::fd::Session_filter);

    if (session_file_name.size() > 0)
    {
        loadSession(session_file_name);
    }
}

std::string saveSession()
{
    const std::string output_file_name =
        mandeye::fd::SaveFileDialog("Save session as", mandeye::fd::Session_filter, ".mjs", session_file_name);

    if (output_file_name.size() > 0)
    {
        spdlog::info("Session file to save: '{}'", output_file_name);

        // creating filenames proposal based on current selection
        std::filesystem::path path(output_file_name);
        // Extract parts
        const auto dir = path.parent_path();
        const auto stem = path.stem().string();

        // Build new names
        std::string initial_poses_file_name = (dir / (stem + "_ini_poses.mri")).string();
        std::string poses_file_name = (dir / (stem + "_poses.mrp")).string();

        if (session.point_clouds_container.initial_poses_file_name.empty())
        {
            spdlog::info("Please assign initial_poses_file_name to session");
            spdlog::warn("Session is not saved!");

            [[maybe_unused]] pfd::message message(
                "Please assign initial_poses_file_name to session",
                "Session is not saved. Please assign initial_poses_file_name to session. "
                "Follow guidlines available here : "
                "https://github.com/MapsHD/HDMapping/tree/main/doc/, "
                "You can do this using button 'update initial poses from RESSO file'",
                pfd::choice::ok,
                pfd::icon::error);
            message.result();

            initial_poses_file_name =
                mandeye::fd::SaveFileDialog("Initial poses file name", mandeye::fd::IniPoses_filter, initial_poses_file_name);
            spdlog::info("Resso file to save: '{}'", initial_poses_file_name);

            if (initial_poses_file_name.size() > 0)
            {
                spdlog::info("Saving initial poses to: '{}'", initial_poses_file_name);
                session.point_clouds_container.save_poses(initial_poses_file_name, false);
            }
        }

        if (session.point_clouds_container.poses_file_name.empty())
        {
            spdlog::info("Please assign poses_file_name to session");
            spdlog::warn("Session is not saved!");

            [[maybe_unused]] pfd::message message(
                "Please assign poses_file_name to session",
                "Session is not saved. Please assign poses_file_name to session. "
                "Follow guidlines available here : "
                "https://github.com/MapsHD/HDMapping/tree/main/doc/,"
                "You can do this using button 'update poses from RESSO file'",
                pfd::choice::ok,
                pfd::icon::error);
            message.result();

            poses_file_name = mandeye::fd::SaveFileDialog("Poses file name", mandeye::fd::Poses_filter, poses_file_name);
            spdlog::info("Resso file to save: '{}'", poses_file_name);
            if (poses_file_name.size() > 0)
            {
                spdlog::info("Saving poses to: '{}'", poses_file_name);
                session.point_clouds_container.save_poses(poses_file_name, false);
            }
        }

        session.save(output_file_name, poses_file_name, initial_poses_file_name, false);
        spdlog::info("Saving result to: '{}'", poses_file_name);
        session.point_clouds_container.save_poses(poses_file_name, false);

        try
        {
            fs::copy_file(poses_file_name, initial_poses_file_name, fs::copy_options::overwrite_existing);
        } catch (const fs::filesystem_error& e)
        {
            spdlog::error("Error copying poses file: {}", e.what());
        }

        return output_file_name;
    }
    else
    {
        spdlog::info("Saving canceled");

        return "";
    }
}

// Shared tail of openLaz()/openE57(): once session.point_clouds_container is
// populated, wire up the viewer state and -- when fillInSession is set --
// materialize a fresh on-disk session (result folder + per-scan .laz +
// trajectory_lio_*.csv). `sourceDir` is only used for the window title.
//
// NOTE: the fillInSession branch mean-centres every cloud, which only makes
// sense for las/laz where the clouds have no pose. openE57() always passes
// fillInSession = false (E57 scans keep their embedded poses and never write a
// session folder) and does its own in-memory fill.
void finalizeScanSession(const std::string& sourceDir, bool fillInSession)
{
    session_loaded = true;
    index_begin = 0;
    index_end = session.point_clouds_container.point_clouds.size() - 1;

    std::string newTitle = winTitle + " - " + sourceDir;
    SetWindowTitle(newTitle.c_str());

    for (const auto& pc : session.point_clouds_container.point_clouds)
        session_total_number_of_points += pc.points_local.size();

    session_dims = session.point_clouds_container.compute_point_cloud_dimension();

    if (fillInSession && !session.point_clouds_container.point_clouds.empty())
    {
        {
            int counter = 1;
            Eigen::Vector3d mean(session.point_clouds_container.point_clouds[0].points_local[0]);
            for (auto& pc : session.point_clouds_container.point_clouds)
            {
                if (pc.points_local.size() > 100)
                {
                    // spdlog::info("mean " << mean << std::endl;
                    for (size_t i = 100; i < pc.points_local.size(); i += 100)
                    {
                        mean += pc.points_local[i];
                        counter++;
                    }
                }
            }
            mean /= counter;

            for (auto& pc : session.point_clouds_container.point_clouds)
            {
                Eigen::Affine3d m = Eigen::Affine3d::Identity();
                if (pc.points_local.size() > 100)
                {
                    // int counter = 1;

                    // spdlog::info("mean " << mean << std::endl;
                    // for (size_t i = 100; i < pc.points_local.size(); i += 100)
                    //{
                    //    mean += pc.points_local[i];
                    //    counter++;
                    //}

                    // mean /= counter;
                    m.translation() = mean;

                    PointCloud::LocalTrajectoryNode node;
                    node.imu_diff_angle_om_fi_ka_deg = { 0, 0, 0 };
                    node.imu_om_fi_ka = { 0, 0, 0 };
                    node.m_pose = Eigen::Affine3d::Identity();
                    node.timestamps = { 0, 0 };

                    pc.local_trajectory.push_back(node);

                    for (auto& p : pc.points_local)
                    {
                        p -= mean;
                    }
                }

                pc.m_initial_pose = m;
                pc.m_pose = m;
                pc.m_pose_temp = m;

                pc.pose = pose_tait_bryan_from_affine_matrix(m);
            }
        }

        {
            std::string session_fn = get_next_result_path(session.working_directory).string();
            std::filesystem::create_directory(session_fn);

            session_file_name = (fs::path(session_fn) / "newSession.mjs").string();
            session.point_clouds_container.initial_poses_file_name =
                "dummy"; // non empty file names signal poses present, new file names will be created on save
            session.point_clouds_container.poses_file_name =
                "dummy"; // non empty file names signal poses present, new file names will be created on save

            std::string output_file_name = saveSession();

            if (output_file_name.size() > 0)
            {
                // creating filenames proposal based on current selection
                std::filesystem::path path(output_file_name);
                // Extract parts
                const auto dir = path.parent_path();
                const auto stem = path.stem().string();

                // save to folder
                for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
                {
                    auto pc = session.point_clouds_container.point_clouds[i];
                    auto fpath = dir / fs::path(pc.file_name).filename();

                    session.point_clouds_container.point_clouds[i].file_name = fpath.string();
                    exportLaz(fpath.string(), pc.points_local, pc.intensities, pc.timestamps);

                    // saving terajectory file
                    auto pathtrj = dir / ("trajectory_lio_" + std::to_string(i) + ".csv");
                    spdlog::info("Saving trajectory: '{}'", pathtrj);

                    std::ofstream outfile;
                    outfile.open(pathtrj);
                    if (!outfile.good())
                    {
                        spdlog::error("Error saving file: '{}'", pathtrj);
                        return;
                    }

                    outfile << "timestamp_nanoseconds pose00 pose01 pose02 pose03 pose10 pose11 pose12 pose13 pose20 pose21 pose22 pose23 "
                               "timestampUnix_nanoseconds om_rad fi_rad ka_rad"
                            << std::endl;
                    for (int j = 0; j < 1; j++)
                    {
                        auto pose = Eigen::Affine3d::Identity();

                        outfile << std::setprecision(20) << 0.0 << " " << std::setprecision(10) << pose(0, 0) << " " << pose(0, 1) << " "
                                << pose(0, 2) << " " << pose(0, 3) << " " << pose(1, 0) << " " << pose(1, 1) << " " << pose(1, 2) << " "
                                << pose(1, 3) << " " << pose(2, 0) << " " << pose(2, 1) << " " << pose(2, 2) << " " << pose(2, 3) << " "
                                << std::setprecision(20) << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << std::endl;
                    }
                    outfile.close();
                }
            }

            [[maybe_unused]] pfd::message message(
                "Information",
                "If you can not see point cloud --> 1. Change 'Points render subsampling', 2. Check console 'min max coordinates "
                "should be "
                "small numbers to see points in our local coordinate system'. 3. Set checkbox 'calculate_offset for WHU-TLS'. 4. Later "
                "on "
                "You can change offset directly in session json file.",
                pfd::choice::ok,
                pfd::icon::info);
            message.result();

            // std::string mes = "Session saved to folder '" + path_ground_truth_session_folder.string() + "'";
            //[[maybe_unused]] pfd::message message(
            //    "Ground truth session info", mes,
            //    pfd::choice::ok, pfd::icon::info);
            // message.result();
        }
    }
}

void openLaz(bool fillInSession)
{
    std::vector<std::string> input_file_names;
    input_file_names = mandeye::fd::OpenFileDialog("Load las/laz files", mandeye::fd::LAS_LAZ_filter, true);
    if (input_file_names.size() == 0)
        return; // dialog cancelled -- leave any already-loaded session untouched

    session.point_clouds_container.point_clouds.clear();
    session.working_directory = fs::path(input_file_names[0]).parent_path().string();

    spdlog::info("Creating session from las/laz files:");
    for (size_t i = 0; i < input_file_names.size(); i++)
        spdlog::info("{}", input_file_names[i]);

    if (!session.point_clouds_container.load_whu_tls(
            input_file_names,
            tls_registration.is_decimate,
            tls_registration.bucket_x,
            tls_registration.bucket_y,
            tls_registration.bucket_z,
            tls_registration.calculate_offset,
            session.load_cache_mode))
        spdlog::error("Error loading session! Check input files laz/las");
    else
        spdlog::info("Loaded: {} point_clouds", session.point_clouds_container.point_clouds.size());

    finalizeScanSession(fs::path(input_file_names[0]).parent_path().string(), fillInSession);
}

void openE57(bool fillInSession)
{
    std::vector<std::string> input_file_names = mandeye::fd::OpenFileDialog("Load e57 files", mandeye::fd::E57_filter, true);
    if (input_file_names.size() == 0)
        return; // dialog cancelled -- leave any already-loaded session untouched

    spdlog::info("Creating session from e57 files:");
    for (const auto& fn : input_file_names)
        spdlog::info("{}", fn);

    // Build into a local list first so a full failure leaves the current session intact.
    std::vector<PointCloud> loaded;
    for (const auto& fn : input_file_names)
    {
        std::vector<mandeye::e57io::E57Scan> scans;
        std::string err;
        if (!mandeye::e57io::load_e57(fn, scans, err))
        {
            spdlog::error("Failed to load e57 '{}': {}", fn, err);
            [[maybe_unused]] pfd::message message(
                "E57 load error", "Could not read:\n" + fn + "\n\n" + err, pfd::choice::ok, pfd::icon::error);
            message.result();
            continue;
        }

        const std::string stem = fs::path(fn).stem().string();
        std::string abs_src;
        try
        {
            abs_src = fs::absolute(fn).string();
        } catch (const std::exception&)
        {
            abs_src = fn;
        }

        for (size_t si = 0; si < scans.size(); si++)
        {
            auto& scan = scans[si];

            PointCloud pc;
            // Name the cloud so it exports to a sensible .laz in fillInSession mode.
            pc.file_name = scans.size() > 1 ? (stem + "_" + std::to_string(si) + ".laz") : (stem + ".laz");
            // Provenance for "Update e57 poses".
            pc.e57_source_path = abs_src;
            pc.e57_scan_index = static_cast<int>(si);
            pc.points_local = std::move(scan.points);
            pc.intensities = std::move(scan.intensities);
            pc.colors = std::move(scan.colors);
            pc.timestamps = std::move(scan.timestamps);

            // exportLaz() and PointCloud::decimate() index intensities/timestamps
            // per point, so pad the ones the E57 scan didn't provide.
            if (pc.intensities.size() != pc.points_local.size())
                pc.intensities.assign(pc.points_local.size(), 0);
            if (pc.timestamps.size() != pc.points_local.size())
                pc.timestamps.assign(pc.points_local.size(), 0.0);

            pc.m_initial_pose = scan.pose;
            pc.m_pose = scan.pose;
            pc.m_pose_temp = scan.pose;
            pc.pose = pose_tait_bryan_from_affine_matrix(scan.pose);
            pc.gui_translation[0] = static_cast<float>(pc.pose.px);
            pc.gui_translation[1] = static_cast<float>(pc.pose.py);
            pc.gui_translation[2] = static_cast<float>(pc.pose.pz);
            pc.gui_rotation[0] = rad2deg(pc.pose.om);
            pc.gui_rotation[1] = rad2deg(pc.pose.fi);
            pc.gui_rotation[2] = rad2deg(pc.pose.ka);

            if (tls_registration.is_decimate && pc.points_local.size() > 0)
                pc.decimate(tls_registration.bucket_x, tls_registration.bucket_y, tls_registration.bucket_z);

            loaded.push_back(std::move(pc));
        }
    }

    if (loaded.empty())
    {
        spdlog::error("No scans loaded from the selected e57 file(s)");
        return;
    }

    session.point_clouds_container.point_clouds = std::move(loaded);
    session.working_directory = fs::path(input_file_names[0]).parent_path().string();
    spdlog::info(
        "Loaded: {} point_clouds from {} e57 file(s)", session.point_clouds_container.point_clouds.size(), input_file_names.size());

    // E57 scans carry their own poses, so the session is already complete in
    // memory. When "Fill in session" is set, just add the identity local
    // trajectory node the fill flow expects -- but never write a session/result
    // folder to disk here. The user saves explicitly via "Save session as".
    if (fillInSession)
    {
        for (auto& pc : session.point_clouds_container.point_clouds)
        {
            if (!pc.local_trajectory.empty())
                continue;
            PointCloud::LocalTrajectoryNode node;
            node.imu_diff_angle_om_fi_ka_deg = { 0, 0, 0 };
            node.imu_om_fi_ka = { 0, 0, 0 };
            node.m_pose = Eigen::Affine3d::Identity();
            node.timestamps = { 0, 0 };
            pc.local_trajectory.push_back(node);
        }
    }

    finalizeScanSession(fs::path(input_file_names[0]).parent_path().string(), false /* never create session files for e57 */);
}

// Write the current (registered) poses of E57-imported scans back into the
// originating .e57 file(s), replacing each Data3D `pose` element. All other
// content (points, colors, intensity, line groups, 2D images) is copied
// verbatim. Only scans that still carry their e57 provenance are considered.
void updateE57Poses()
{
    // file -> { Data3D index -> refined pose }
    std::map<std::string, std::map<int, Eigen::Affine3d>> by_file;
    for (const auto& pc : session.point_clouds_container.point_clouds)
    {
        if (pc.e57_source_path.empty() || pc.e57_scan_index < 0)
            continue;
        // openE57 keeps points scan-local and never applies a session offset, so
        // the refined m_pose is already the file-level Data3D pose to write back.
        by_file[pc.e57_source_path][pc.e57_scan_index] = pc.m_pose;
    }

    if (by_file.empty())
    {
        [[maybe_unused]] pfd::message m(
            "Update e57 poses", "No scans in this session were loaded from an e57 file.", pfd::choice::ok, pfd::icon::warning);
        m.result();
        return;
    }

    const pfd::button choice = pfd::message(
                                   "Update e57 poses",
                                   "Write updated Data3D poses into " + std::to_string(by_file.size()) +
                                       " e57 file(s)?\n\n"
                                       "Yes    - overwrite the original file(s) in place\n"
                                       "No     - write copies as <name>_updated.e57\n"
                                       "Cancel - abort",
                                   pfd::choice::yes_no_cancel,
                                   pfd::icon::question)
                                   .result();

    if (choice == pfd::button::cancel)
        return;
    const bool in_place = (choice == pfd::button::yes);

    int ok = 0;
    int failed = 0;
    std::string errors;
    for (const auto& [src, poses] : by_file)
    {
        const fs::path srcp(src);
        const fs::path tmp = srcp.parent_path() / (srcp.stem().string() + ".e57.tmp");

        std::string err;
        if (!mandeye::e57io::rewrite_e57_poses(src, tmp.string(), poses, err))
        {
            failed++;
            errors += "\n- " + src + "\n    " + err;
            std::error_code ec;
            fs::remove(tmp, ec);
            continue;
        }

        const fs::path dst = in_place ? srcp : srcp.parent_path() / (srcp.stem().string() + "_updated.e57");
        std::error_code ec;
        fs::rename(tmp, dst, ec);
        if (ec)
        {
            ec.clear();
            fs::copy_file(tmp, dst, fs::copy_options::overwrite_existing, ec);
            std::error_code ec2;
            fs::remove(tmp, ec2);
        }
        if (ec)
        {
            failed++;
            errors += "\n- " + src + "\n    could not place result: " + ec.message();
            continue;
        }

        ok++;
        spdlog::info("Update e57 poses: wrote '{}' ({} scan pose(s))", dst.string(), poses.size());
    }

    [[maybe_unused]] pfd::message summary(
        "Update e57 poses",
        std::to_string(ok) + " file(s) updated, " + std::to_string(failed) + " failed." +
            (errors.empty() ? std::string() : ("\n\nErrors:" + errors)),
        pfd::choice::ok,
        failed > 0 ? pfd::icon::error : pfd::icon::info);
    summary.result();
}

// Write the whole session out as one multi-scan .e57 file: one Data3D block per
// point cloud, points in their scan-local frame, each block carrying the current
// (registered) pose. Independent of the "never write files on e57 load" rule --
// this is an explicit, user-driven export.
void saveSessionAsE57()
{
    if (session.point_clouds_container.point_clouds.empty())
    {
        [[maybe_unused]] pfd::message m(
            "Save session as e57", "The session has no point clouds to save.", pfd::choice::ok, pfd::icon::warning);
        m.result();
        return;
    }

    std::string default_name = "session.e57";
    if (!session_file_name.empty())
        default_name = fs::path(session_file_name).replace_extension(".e57").string();

    const std::string out = mandeye::fd::SaveFileDialog("Save session as e57", mandeye::fd::E57_filter, ".e57", default_name);
    if (out.empty())
        return;

    const std::string description = std::string("HDMapping ") + HDMAPPING_VERSION_STRING + " session";

    std::vector<mandeye::e57io::E57WriteScan> scans;
    scans.reserve(session.point_clouds_container.point_clouds.size());
    for (const auto& pc : session.point_clouds_container.point_clouds)
    {
        mandeye::e57io::E57WriteScan s;
        s.name = fs::path(pc.file_name).stem().string();
        s.description = description;
        s.points = &pc.points_local;
        if (pc.intensities.size() == pc.points_local.size())
            s.intensities = &pc.intensities;
        if (pc.colors.size() == pc.points_local.size())
            s.colors = &pc.colors;
        if (pc.timestamps.size() == pc.points_local.size())
            s.timestamps = &pc.timestamps;

        // File-level pose = session offset (a pure translation) composed with the
        // scan's registered pose. For e57-loaded sessions the offset is zero.
        s.pose = pc.m_pose;
        s.pose.translation() += session.point_clouds_container.offset;

        scans.push_back(std::move(s));
    }

    std::string err;
    if (mandeye::e57io::save_e57(out, scans, err))
    {
        spdlog::info("Saved session as e57: '{}' ({} scans)", out, scans.size());
        [[maybe_unused]] pfd::message m(
            "Save session as e57", "Saved " + std::to_string(scans.size()) + " scan(s) to:\n" + out, pfd::choice::ok, pfd::icon::info);
        m.result();
    }
    else
    {
        spdlog::error("Save session as e57 failed: {}", err);
        [[maybe_unused]] pfd::message m("Save session as e57", "Failed:\n" + err, pfd::choice::ok, pfd::icon::error);
        m.result();
    }
}

void saveSubsession()
{
    int inx_begin = 0;
    int inx_end = 0;

    for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
    {
        if (session.point_clouds_container.point_clouds[i].visible)
        {
            inx_begin = i;
            break;
        }
    }

    for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
        if (session.point_clouds_container.point_clouds[i].visible)
            inx_end = i;

    // creating filename proposal based on current selection
    fs::path path(session_file_name);
    // Extract parts
    fs::path dir = path.parent_path();
    std::string stem = path.stem().string();
    const auto ext = ".mjs"; // forcing new extension even if original session was .json
    const std::string indexpart = " " + std::to_string(inx_begin) + "-" + std::to_string(inx_end);

    // Build new name
    std::string indexed_file_name = (dir / (stem + indexpart + ext)).string();

    const auto output_file_name = mandeye::fd::SaveFileDialog("Save subsession", mandeye::fd::Session_filter, ".mjs", indexed_file_name);

    if (output_file_name.size() > 0)
    {
        spdlog::info("Subsession file to save: '{}'", output_file_name);

        path = fs::path(output_file_name);
        dir = path.parent_path();
        stem = path.stem().string();

        const auto initial_poses_file_name = (dir / (stem + "_ini_poses" + ".mri")).string();
        const auto poses_file_name = (dir / (stem + "_poses" + ".mrp")).string();

        session.save(fs::path(output_file_name).string(), poses_file_name, initial_poses_file_name, true);
        spdlog::info("Saving poses to: '{}'", poses_file_name);
        session.point_clouds_container.save_poses(fs::path(poses_file_name).string(), true);

        try
        {
            fs::copy_file(poses_file_name, initial_poses_file_name, fs::copy_options::overwrite_existing);
        } catch (const fs::filesystem_error& e)
        {
            spdlog::error("Error copying poses file: {}", e.what());
        }
    }
    else
        spdlog::info("Saving canceled");
}

void settings_gui()
{
    ImGui::Begin("Settings", &is_settings_gui);
    {
        std::string wd = "Working directory: '" + session.working_directory + "'";
        ImGui::Text(wd.c_str());

        ImGui::NewLine();

        ImGui::InputFloat("camera_x", &app_state.camera.eulerGoal.rotationCenter.x);
        ImGui::InputFloat("camera_y", &app_state.camera.eulerGoal.rotationCenter.y);
        ImGui::InputFloat("camera_z", &app_state.camera.eulerGoal.rotationCenter.z);

        if (ImGui::Button("set camera"))
        {
            // app_state.camera.eulerGoal.rotateX = app_state.camera.euler.rotateX;
            // app_state.camera.eulerGoal.rotateY = app_state.camera.euler.rotateY;
            // app_state.camera.eulerGoal.translate.x = -app_state.camera.eulerGoal.rotationCenter.x;
            // app_state.camera.eulerGoal.translate.y = -app_state.camera.eulerGoal.rotationCenter.y;
            // app_state.camera.eulerGoal.translate.z = -app_state.camera.eulerGoal.rotationCenter.z;
            app_state.camera.eulerTransitionActive = true;
        }

        if (ImGui::Button("Set initial pose to Identity and update other poses"))
            initial_pose_to_identity(session);

        ImGui::NewLine();

        ImGui::Checkbox("Downsample during load", &tls_registration.is_decimate);
        ImGui::Checkbox("Loading Point Cloud Cache Mode", &session.load_cache_mode);

        ImGui::Text("Bucket [m]:");
        ImGui::PushItemWidth(ImGuiNumberWidth);
        ImGui::InputDouble("X##b", &tls_registration.bucket_x, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(xText);
        ImGui::SameLine();
        ImGui::InputDouble("Y##b", &tls_registration.bucket_y, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(yText);
        ImGui::SameLine();
        ImGui::InputDouble("Z##b", &tls_registration.bucket_z, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(zText);
        ImGui::PopItemWidth();

        ImGui::Separator();
        // common_data
        // manual_pose_graph_loop_closure_mode

        if (ImGui::Button("Load RESSO file (transformation_GroundTruth.reg)"))
        {
            std::string input_file_name = "";
            input_file_name = mandeye::fd::OpenFileDialogOneFile("Load RESSO", mandeye::fd::Resso_filter);
            spdlog::info("RESSO file: '{}'", input_file_name);

            if (input_file_name.size() > 0)
            {
                session.working_directory = fs::path(input_file_name).parent_path().string();

                if (!session.point_clouds_container.load(
                        session.working_directory.c_str(),
                        input_file_name.c_str(),
                        tls_registration.is_decimate,
                        tls_registration.bucket_x,
                        tls_registration.bucket_y,
                        tls_registration.bucket_z,
                        session.load_cache_mode))
                {
                    spdlog::error("Error loading file!");
                    return;
                }
                else
                    spdlog::info("Loaded: {} point_clouds", session.point_clouds_container.point_clouds.size());
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Save RESSO file"))
        {
            const auto output_file_name = mandeye::fd::SaveFileDialog("Save RESSO file", mandeye::fd::Resso_filter);
            spdlog::info("RESSO file to save: '{}'", output_file_name);
            if (output_file_name.size() > 0)
                session.point_clouds_container.save_poses(fs::path(output_file_name).string(), false);
        }
        ImGui::Text("RESSO dataset: ");
        ImGui::SameLine();
        ImGuiHyperlink("https://3d.bk.tudelft.nl/liangliang/publications/2019/plade/resso.html");

        ImGui::NewLine();

        if (ImGui::Button("Load ETH file (pairs.txt)"))
        {
            std::string input_file_name = "";
            input_file_name = mandeye::fd::OpenFileDialogOneFile("Load ETH file", {});
            if (input_file_name.size() > 0)
            {
                session.working_directory = fs::path(input_file_name).parent_path().string();

                if (!session.point_clouds_container.load_eth(
                        session.working_directory.c_str(),
                        input_file_name.c_str(),
                        tls_registration.is_decimate,
                        tls_registration.bucket_x,
                        tls_registration.bucket_y,
                        tls_registration.bucket_z))
                {
                    spdlog::error("Error loading file!");
                    return;
                }
                else
                    spdlog::info("Loaded: {} point_clouds", session.point_clouds_container.point_clouds.size());
            }
        }
        ImGui::Text("ETH dataset: ");
        ImGui::SameLine();
        ImGuiHyperlink("https://prs.igp.ethz.ch/research/completed_projects/automatic_registration_of_point_clouds.html");

        ImGui::NewLine();

        if (ImGui::Button("Load 3DTK files (select all *.txt files)"))
        {
            session.point_clouds_container.point_clouds.clear();
            std::vector<std::string> input_file_names;
            input_file_names = mandeye::fd::OpenFileDialog("Load txt files", {}, true);

            if (input_file_names.size() > 0)
            {
                session.working_directory = fs::path(input_file_names[0]).parent_path().string();

                spdlog::info("TXT files:");
                for (size_t i = 0; i < input_file_names.size(); i++)
                    spdlog::info("{}", input_file_names[i]);

                if (!session.point_clouds_container.load_3DTK_tls(
                        input_file_names,
                        tls_registration.is_decimate,
                        tls_registration.bucket_x,
                        tls_registration.bucket_y,
                        tls_registration.bucket_z))
                {
                    spdlog::error("Error loading file!");
                    return;
                }
                else
                    spdlog::info("Loaded: {} point_clouds", session.point_clouds_container.point_clouds.size());
            }
        }
        ImGui::Text("3DTK dataset (18: the campus of the Jacobs University Bremen)");
        ImGui::SameLine();
        ImGuiHyperlink("http://kos.informatik.uni-osnabrueck.de/3Dscans/");

        ImGui::NewLine();

        if (ImGui::Button("Update initial poses from RESSO file"))
        {
            std::string input_file_name;
            input_file_name = mandeye::fd::OpenFileDialogOneFile("Load RESSO file", {});
            if (input_file_name.size() > 0)
            {
                session.working_directory = fs::path(input_file_name).parent_path().string();

                if (!session.point_clouds_container.update_initial_poses_from_RESSO(
                        session.working_directory.c_str(), input_file_name.c_str()))
                {
                    spdlog::error("Error loading file!");
                    return;
                }
                else
                {
                    session.point_clouds_container.initial_poses_file_name = input_file_name;
                    spdlog::info("Updated: {} point_clouds", session.point_clouds_container.point_clouds.size());
                }
            }
        }
        ImGui::SameLine();
        ImGui::Text(session.point_clouds_container.initial_poses_file_name.c_str());

        if (ImGui::Button("Update poses from RESSO file"))
        {
            std::string input_file_name;
            input_file_name = mandeye::fd::OpenFileDialogOneFile("Load RESSO file", {});

            if (input_file_name.size() > 0)
            {
                session.working_directory = fs::path(input_file_name).parent_path().string();

                if (!session.point_clouds_container.update_poses_from_RESSO(session.working_directory.c_str(), input_file_name.c_str()))
                {
                    spdlog::error("Error loading file!");
                    return;
                }
                else
                {
                    spdlog::info("Updated: {} point_clouds", session.point_clouds_container.point_clouds.size());
                    session.point_clouds_container.poses_file_name = input_file_name;
                }
            }
        }
        ImGui::SameLine();

        if (ImGui::Button("Update poses from RESSO file (inverse)"))
        {
            std::string input_file_name;
            input_file_name = mandeye::fd::OpenFileDialogOneFile("Load RESSO file", {});

            if (input_file_name.size() > 0)
            {
                session.working_directory = fs::path(input_file_name).parent_path().string();

                if (!session.point_clouds_container.update_poses_from_RESSO_inverse(
                        session.working_directory.c_str(), input_file_name.c_str()))
                {
                    spdlog::error("Error loading file!");
                    return;
                }
                else
                {
                    spdlog::info("Updated: {} point_clouds", session.point_clouds_container.point_clouds.size());
                    session.point_clouds_container.poses_file_name = input_file_name;
                }
            }
        }
        ImGui::SameLine();
        ImGui::Text(session.point_clouds_container.poses_file_name.c_str());

        ImGui::Separator();

        if (!is_loop_closure_gui)
        {
            static double x_origin = 0.0;
            static double y_origin = 0.0;
            static double z_origin = 0.0;

            ImGui::Text("Origin [m]: ");
            ImGui::SameLine();
            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputDouble("X##o", &x_origin, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(xText);
            ImGui::SameLine();
            ImGui::InputDouble("Y##o", &y_origin, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(yText);
            ImGui::SameLine();
            ImGui::InputDouble("Z##o", &z_origin, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(zText);
            ImGui::PopItemWidth();
            ImGui::SameLine();
            if (ImGui::Button("Set XYZ origin"))
            {
                if (session.point_clouds_container.point_clouds.size() != 0)
                {
                    std::vector<Eigen::Affine3d> all_m_poses2;
                    for (size_t j = 0; j < session.point_clouds_container.point_clouds.size(); j++)
                    {
                        all_m_poses2.push_back(session.point_clouds_container.point_clouds[j].m_pose);
                    }

                    session.point_clouds_container.point_clouds[0].m_pose(0, 3) = x_origin;
                    session.point_clouds_container.point_clouds[0].m_pose(1, 3) = y_origin;
                    session.point_clouds_container.point_clouds[0].m_pose(2, 3) = z_origin;

                    session.point_clouds_container.point_clouds[0].pose =
                        pose_tait_bryan_from_affine_matrix(session.point_clouds_container.point_clouds[0].m_pose);

                    session.point_clouds_container.point_clouds[0].gui_translation[0] =
                        (float)session.point_clouds_container.point_clouds[0].pose.px;
                    session.point_clouds_container.point_clouds[0].gui_translation[1] =
                        (float)session.point_clouds_container.point_clouds[0].pose.py;
                    session.point_clouds_container.point_clouds[0].gui_translation[2] =
                        (float)session.point_clouds_container.point_clouds[0].pose.pz;

                    session.point_clouds_container.point_clouds[0].gui_rotation[0] =
                        (float)(session.point_clouds_container.point_clouds[0].pose.om * RAD_TO_DEG);
                    session.point_clouds_container.point_clouds[0].gui_rotation[1] =
                        (float)(session.point_clouds_container.point_clouds[0].pose.fi * RAD_TO_DEG);
                    session.point_clouds_container.point_clouds[0].gui_rotation[2] =
                        (float)(session.point_clouds_container.point_clouds[0].pose.ka * RAD_TO_DEG);

                    Eigen::Affine3d curr_m_pose2 = session.point_clouds_container.point_clouds[0].m_pose;
                    for (size_t j = 1; j < session.point_clouds_container.point_clouds.size(); j++)
                    {
                        curr_m_pose2 = curr_m_pose2 * (all_m_poses2[j - 1].inverse() * all_m_poses2[j]);

                        // spdlog::info(curr_m_pose2.matrix() << std::endl;
                        session.point_clouds_container.point_clouds[j].m_pose = curr_m_pose2;
                        session.point_clouds_container.point_clouds[j].pose =
                            pose_tait_bryan_from_affine_matrix(session.point_clouds_container.point_clouds[j].m_pose);

                        session.point_clouds_container.point_clouds[j].gui_translation[0] =
                            (float)session.point_clouds_container.point_clouds[j].pose.px;
                        session.point_clouds_container.point_clouds[j].gui_translation[1] =
                            (float)session.point_clouds_container.point_clouds[j].pose.py;
                        session.point_clouds_container.point_clouds[j].gui_translation[2] =
                            (float)session.point_clouds_container.point_clouds[j].pose.pz;

                        session.point_clouds_container.point_clouds[j].gui_rotation[0] =
                            (float)(session.point_clouds_container.point_clouds[j].pose.om * RAD_TO_DEG);
                        session.point_clouds_container.point_clouds[j].gui_rotation[1] =
                            (float)(session.point_clouds_container.point_clouds[j].pose.fi * RAD_TO_DEG);
                        session.point_clouds_container.point_clouds[j].gui_rotation[2] =
                            (float)(session.point_clouds_container.point_clouds[j].pose.ka * RAD_TO_DEG);
                    }
                }
            }

            ImGui::Separator();

            ImGui::Text("Set offsets to export point cloud in global coordinate system [m]:");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("each local coordinate of the point += offset");
            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputDouble("X##t", &session.point_clouds_container.offset.x(), 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(xText);
            ImGui::SameLine();
            ImGui::InputDouble("Y##t", &session.point_clouds_container.offset.y(), 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(yText);
            ImGui::SameLine();
            ImGui::InputDouble("Z##t", &session.point_clouds_container.offset.z(), 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(zText);
            ImGui::PopItemWidth();

            if (ImGui::Button("Set offset_to_apply --> from session (X, Y, Z)"))
                session.point_clouds_container.offset = session.point_clouds_container.offset_to_apply;
            /*
            ImGui::NewLine();
            ImGui::NewLine();
            ImGui::NewLine();
            ImGui::NewLine();
            ImGui::NewLine();
            ImGui::NewLine();
            ImGui::Separator();
            ImGui::Text("Perform experiment on:");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Experiments to compare methods and approaches for multi-view TLS registration");
            ImGui::SameLine();
            if (ImGui::Button("WINDOWS"))
                perform_experiment_on_windows(session, observation_picking, tls_registration.icp, tls_registration.ndt,
            tls_registration.registration_plane_feature, tls_registration.pose_graph_slam); ImGui::SameLine(); if (ImGui::Button("LINUX"))
                perform_experiment_on_linux(session, observation_picking, tls_registration.icp, tls_registration.ndt,
            tls_registration.registration_plane_feature, tls_registration.pose_graph_slam);
            */
        }
    }

    ImGui::End();
}

// Camera/picking/mini-compass/misc-ImGui-widget functions (truncPath,
// wheel/reshape/motion, showAxes, updateCameraTransition/breakCameraTransition/
// setCameraPreset, camMenu/view_kbd_shortcuts/cor_window/ImGuiHyperlink/
// ShowShortcutsTable/info_window, drawMiniCompassWithRuler, rayIntersection/
// GetLaserBeam/distance_point_to_line/getClosestTrajectoryPoint/
// setNewRotationCenter, checkClHelp, updateOrthoView, end3DMatrixStack) now
// live in rl_utils.cpp/rl_utils.h -- see rl_utils.h's top comment.

// Was ObservationPicking::render() (core/src/observation_picking.cpp) --
// legacy-GL, compiled once into `core`, shared with the remaining GLUT
// apps, so it can't be changed. Reimplemented here via rl* renames (the
// original is pure immediate-mode grid/point/line drawing, no matrix-stack
// work of its own). The per-intersection wireframe boxes (Intersection::
// render(), also in `core`) and the GLUT-bitmap-font index labels are not
// ported (a niche sub-feature of an already-niche picking mode) -- picking
// itself and the current/committed observation markers below are.
void observationPickingRender(const ObservationPicking& observation_picking)
{
    if (observation_picking.is_observation_picking_mode)
    {
        auto drawGrid = [&](float step, float r, float g, float b)
        {
            rlColor3f(r, g, b);
            rlBegin(RL_LINES);
            for (float x = -observation_picking.max_xy; x <= observation_picking.max_xy; x += step)
            {
                rlVertex3f(x, -observation_picking.max_xy, observation_picking.picking_plane_height);
                rlVertex3f(x, observation_picking.max_xy, observation_picking.picking_plane_height);
            }
            for (float y = -observation_picking.max_xy; y <= observation_picking.max_xy; y += step)
            {
                rlVertex3f(-observation_picking.max_xy, y, observation_picking.picking_plane_height);
                rlVertex3f(observation_picking.max_xy, y, observation_picking.picking_plane_height);
            }
            rlEnd();
        };

        if (observation_picking.grid10x10m)
            drawGrid(10.0f, 0.7f, 0.7f, 0.7f);
        if (observation_picking.grid1x1m)
            drawGrid(1.0f, 0.3f, 0.3f, 0.3f);
        if (observation_picking.grid01x01m)
            drawGrid(0.1f, 0.1f, 0.1f, 0.1f);
        if (observation_picking.grid001x001m)
            drawGrid(0.01f, 0.8f, 0.8f, 0.8f);
    }

    // rlgl's rlBegin() only supports RL_LINES/RL_TRIANGLES/RL_QUADS (no
    // point-mode immediate drawing), so point markers use small spheres.
    for (const auto& [key, value] : observation_picking.current_observation)
    {
        DrawSphere(Vector3{ static_cast<float>(value.x()), static_cast<float>(value.y()), static_cast<float>(value.z()) }, 0.05f, WHITE);
    }

    rlColor3f(1.0f, 0.2f, 0.2f);
    rlBegin(RL_LINES);
    for (const auto& [key1, value1] : observation_picking.current_observation)
    {
        for (const auto& [key2, value2] : observation_picking.current_observation)
        {
            if (key1 != key2)
            {
                rlVertex3f(value1.x(), value1.y(), value1.z());
                rlVertex3f(value2.x(), value2.y(), value2.z());
            }
        }
    }
    rlEnd();
}

// Was ManualPoseGraphLoopClosure::Render() (core/src/manual_pose_graph_loop_closure.cpp) --
// legacy-GL, compiled once into `core`, shared with the remaining GLUT
// apps, so it can't be changed. Reimplemented here using scan_renderer
// (marks for source/target highlighting, straight from each scan's already-
// cached GPU buffer) plus DrawSphere/DrawCylinderEx/DrawLine3D for the
// green pose-sequence trail and the per-edge lines/flagpole markers (these
// work against whatever rlgl projection/modelview is currently active, same
// as this app's own manually-driven matrix stack -- no BeginMode3D needed).
// Coordinates are used directly (Z-up, no remap -- see raylib_render.hpp).
void renderLoopClosure(
    PointClouds& point_clouds_container, int index_loop_closure_source, int index_loop_closure_target, int before, int after)
{
    auto& pointClouds = point_clouds_container.point_clouds;
    if (pointClouds.empty())
    {
        return;
    }

    scan_renderer.clearMarks();

    // Matches the original: while loop closure editing is active, only the
    // source/target (or active-edge) range renders -- not the whole
    // session -- since ManualPoseGraphLoopClosure::Render() drew just those
    // scans directly rather than going through the bulk point-cloud render
    // call (which display() only makes when !is_loop_closure_gui).
    for (auto& pc : pointClouds)
    {
        pc.visible = false;
    }

    auto markRange = [&](int center, Color color)
    {
        for (int i = center - before; i <= center + after; ++i)
        {
            if (i >= 0 && static_cast<size_t>(i) < pointClouds.size())
            {
                pointClouds[i].visible = true;
                if (session.pose_graph_loop_closure.render_source_as_red_target_as_blue)
                {
                    scan_renderer.setMarkColor(static_cast<size_t>(i), color);
                }
            }
        }
    };

    if (!session.pose_graph_loop_closure.manipulate_active_edge)
    {
        markRange(index_loop_closure_source, RED);
        markRange(index_loop_closure_target, BLUE);
    }
    else if (!session.pose_graph_loop_closure.edges.empty())
    {
        const auto& activeEdge = session.pose_graph_loop_closure.edges[session.pose_graph_loop_closure.index_active_edge];
        markRange(activeEdge.index_from, RED);

        // Live preview of the target side at the edge's in-progress
        // (not-yet-committed) relative_pose_tb, drawn straight from each
        // scan's cached GPU buffer with the delta folded into the MVP
        // rather than re-transforming points on the CPU.
        int indexSrcEdge = activeEdge.index_from;
        int indexTrgEdge = activeEdge.index_to;
        if (indexSrcEdge >= 0 && static_cast<size_t>(indexSrcEdge) < pointClouds.size() && indexTrgEdge >= 0 &&
            static_cast<size_t>(indexTrgEdge) < pointClouds.size())
        {
            const Eigen::Affine3d& mSrc = pointClouds[indexSrcEdge].m_pose;
            for (int i = -before; i <= after; ++i)
            {
                int idx = indexTrgEdge + i;
                if (idx < 0 || static_cast<size_t>(idx) >= pointClouds.size())
                {
                    continue;
                }
                Eigen::Affine3d mTrg = mSrc * affine_matrix_from_pose_tait_bryan(activeEdge.relative_pose_tb);
                Eigen::Affine3d mRel = pointClouds[indexTrgEdge].m_pose.inverse() * pointClouds[idx].m_pose;
                mTrg = mTrg * mRel;

                Eigen::Affine3d delta = mTrg * pointClouds[idx].m_pose.inverse();
                Color c = session.pose_graph_loop_closure.render_source_as_red_target_as_blue
                    ? BLUE
                    : Color{ static_cast<unsigned char>(pointClouds[idx].render_color[0] * 255.f),
                             static_cast<unsigned char>(pointClouds[idx].render_color[1] * 255.f),
                             static_cast<unsigned char>(pointClouds[idx].render_color[2] * 255.f),
                             255 };
                scan_renderer.drawCachedWithTransform(
                    static_cast<size_t>(idx), delta, c, static_cast<float>(pointClouds[idx].point_size), false);
            }
        }
    }

    // The marked/visible-restricted range set above (source/target or
    // active-edge scans, at their normal stored pose).
    scan_renderer.draw(
        pointClouds,
        static_cast<float>(app_state.point_size),
        scanColorModeFromScheme(csPointCloud),
        static_cast<float>(session_dims.z_min),
        static_cast<float>(session_dims.z_max),
        Eigen::Vector3d(
            app_state.camera.euler.rotationCenter.x, app_state.camera.euler.rotationCenter.y, app_state.camera.euler.rotationCenter.z),
        static_cast<float>(std::max({ session_dims.length, session_dims.width, session_dims.height, 1.0 })),
        1,
        point_clouds_container.xz_intersection,
        point_clouds_container.yz_intersection,
        point_clouds_container.xy_intersection,
        static_cast<float>(point_clouds_container.intersection_width));

    // Pose-sequence trail across the whole session, as a chain of thick
    // green cylinders (sphere at each joint), sized relative to the current
    // zoom (app_state.camera.euler.translate.z) so it stays visible next to the point cloud.
    const float tubeRadius = std::max(0.005f, fabsf(app_state.camera.euler.translate.z) * 0.001f);
    bool first = true;
    Vector3 prev{};
    for (const auto& pc : pointClouds)
    {
        Vector3 p = Vector3{ static_cast<float>(pc.m_pose.translation().x()),
                             static_cast<float>(pc.m_pose.translation().y()),
                             static_cast<float>(pc.m_pose.translation().z()) };
        DrawSphere(p, tubeRadius, GREEN);
        if (!first)
        {
            DrawCylinderEx(prev, p, tubeRadius, tubeRadius, 8, GREEN);
        }
        prev = p;
        first = false;
    }

    // Edge lines + flagpole markers (red = active edge, blue = others).
    for (size_t i = 0; i < session.pose_graph_loop_closure.edges.size(); ++i)
    {
        const auto& edge = session.pose_graph_loop_closure.edges[i];
        if (edge.index_from < 0 || static_cast<size_t>(edge.index_from) >= pointClouds.size() || edge.index_to < 0 ||
            static_cast<size_t>(edge.index_to) >= pointClouds.size())
        {
            continue;
        }

        Color c = (static_cast<int>(i) == session.pose_graph_loop_closure.index_active_edge) ? RED : BLUE;

        Eigen::Vector3d worldSrc = pointClouds[edge.index_from].m_pose.translation();
        Eigen::Vector3d worldTrg = pointClouds[edge.index_to].m_pose.translation();
        Vector3 pSrc = Vector3{ static_cast<float>(worldSrc.x()), static_cast<float>(worldSrc.y()), static_cast<float>(worldSrc.z()) };
        Vector3 pTrg = Vector3{ static_cast<float>(worldTrg.x()), static_cast<float>(worldTrg.y()), static_cast<float>(worldTrg.z()) };
        DrawCylinderEx(pSrc, pTrg, tubeRadius, tubeRadius, 8, c);

        Eigen::Vector3d mid = (worldSrc + worldTrg) * 0.5;
        Eigen::Vector3d midUp = mid + Eigen::Vector3d(0, 0, 10);
        Vector3 pMid = Vector3{ static_cast<float>(mid.x()), static_cast<float>(mid.y()), static_cast<float>(mid.z()) };
        Vector3 pMidUp = Vector3{ static_cast<float>(midUp.x()), static_cast<float>(midUp.y()), static_cast<float>(midUp.z()) };
        DrawCylinderEx(pMid, pMidUp, tubeRadius, tubeRadius, 8, c);
    }
}

// Was GroundControlPoints::draw_ellipse() (core/src/ground_control_points.cpp)
// -- a GL_LINE_LOOP quad per lat/long grid cell. rlgl's rlBegin() has no
// LOOP/STRIP mode (only RL_LINES/RL_TRIANGLES/RL_QUADS -- see
// renderLoopClosureLabels()'s comment for the same constraint elsewhere in
// this file), so each cell's 4-vertex loop is emitted as 4 independent line
// segments instead. nstd is dropped: the only caller always passed 1.0.
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

    for (double i = 0; i < 1.0; i += di) // horizontal
    {
        for (double j = 0; j < 1.0; j += dj) // vertical
        {
            double u = i * 2 * pi; // 0     to  2pi
            double v = (j - 0.5) * pi; //-pi/2 to pi/2

            const Eigen::Vector3d pp0(cos(v) * cos(u), cos(v) * sin(u), sin(v));
            const Eigen::Vector3d pp1(cos(v) * cos(u + du), cos(v) * sin(u + du), sin(v));
            const Eigen::Vector3d pp2(cos(v + dv) * cos(u + du), cos(v + dv) * sin(u + du), sin(v + dv));
            const Eigen::Vector3d pp3(cos(v + dv) * cos(u), cos(v + dv) * sin(u), sin(v + dv));
            Eigen::Vector3d tp0 = transform * pp0 + mean;
            Eigen::Vector3d tp1 = transform * pp1 + mean;
            Eigen::Vector3d tp2 = transform * pp2 + mean;
            Eigen::Vector3d tp3 = transform * pp3 + mean;

            rlVertex3f(static_cast<float>(tp0.x()), static_cast<float>(tp0.y()), static_cast<float>(tp0.z()));
            rlVertex3f(static_cast<float>(tp1.x()), static_cast<float>(tp1.y()), static_cast<float>(tp1.z()));

            rlVertex3f(static_cast<float>(tp1.x()), static_cast<float>(tp1.y()), static_cast<float>(tp1.z()));
            rlVertex3f(static_cast<float>(tp2.x()), static_cast<float>(tp2.y()), static_cast<float>(tp2.z()));

            rlVertex3f(static_cast<float>(tp2.x()), static_cast<float>(tp2.y()), static_cast<float>(tp2.z()));
            rlVertex3f(static_cast<float>(tp3.x()), static_cast<float>(tp3.y()), static_cast<float>(tp3.z()));

            rlVertex3f(static_cast<float>(tp3.x()), static_cast<float>(tp3.y()), static_cast<float>(tp3.z()));
            rlVertex3f(static_cast<float>(tp0.x()), static_cast<float>(tp0.y()), static_cast<float>(tp0.z()));
        }
    }

    rlEnd();
}

// Was GroundControlPoints::render() (core/src/ground_control_points.cpp) --
// legacy-GL, compiled once into `core` and shared with the remaining GLUT
// apps, so it can't be touched; reimplemented here with raylib's own
// DrawLine3D (works against whatever rlgl projection/modelview is currently
// active, same as renderLoopClosure()'s DrawSphere/DrawCylinderEx calls --
// no BeginMode3D needed). Name/height text labels are handled separately by
// renderGroundControlPointsLabels() (2D screen-space DrawText, same
// reasoning as renderLoopClosureLabels()).
void renderGroundControlPoints(const GroundControlPoints& ground_control_points, const PointClouds& point_clouds_container)
{
    for (const auto& gcp : ground_control_points.gpcs)
    {
        if (gcp.index_to_node_inner < 0 || static_cast<size_t>(gcp.index_to_node_inner) >= point_clouds_container.point_clouds.size())
        {
            continue;
        }
        const auto& pc = point_clouds_container.point_clouds[gcp.index_to_node_inner];
        if (gcp.index_to_node_outer < 0 || static_cast<size_t>(gcp.index_to_node_outer) >= pc.local_trajectory.size())
        {
            continue;
        }

        Eigen::Vector3d c = pc.m_pose * pc.local_trajectory[gcp.index_to_node_outer].m_pose.translation();
        float h = static_cast<float>(gcp.lidar_height_above_ground);
        Vector3 g{ static_cast<float>(gcp.x), static_cast<float>(gcp.y), static_cast<float>(gcp.z) };

        const Color markColor{ 179, 77, 128, 255 }; // was glColor3f(0.7f, 0.3f, 0.5f)

        DrawLine3D(Vector3{ g.x - 0.05f, g.y, g.z }, Vector3{ g.x + 0.05f, g.y, g.z }, markColor);
        DrawLine3D(Vector3{ g.x, g.y - 0.05f, g.z }, Vector3{ g.x, g.y + 0.05f, g.z }, markColor);

        DrawLine3D(Vector3{ g.x - 0.01f, g.y, g.z + h }, Vector3{ g.x + 0.01f, g.y, g.z + h }, markColor);
        DrawLine3D(Vector3{ g.x, g.y - 0.01f, g.z + h }, Vector3{ g.x, g.y + 0.01f, g.z + h }, markColor);

        DrawLine3D(g, Vector3{ g.x, g.y, g.z + h }, markColor);

        const Color connectorColor{ 0, 77, 153, 255 }; // was glColor3f(0.0f, 0.3f, 0.6f)
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

            Eigen::Vector3d mean(gcp.x, gcp.y, gcp.z + h);
            drawUncertaintyEllipse(covar, mean, GRAY); // was Eigen::Vector3f(0.5, 0.5, 0.5)
        }
    }
}

// Was GNSS::render() (core/src/gnss.cpp) -- legacy-GL, compiled once into
// `core` and shared with the remaining GLUT apps, so it can't be touched;
// reimplemented here with rlgl's rl*() legacy-emulation API. GL_LINE_STRIP
// has no rlBegin() equivalent (only RL_LINES/RL_TRIANGLES/RL_QUADS -- see
// drawUncertaintyEllipse()'s comment for the same constraint elsewhere in
// this file), so the polyline is emitted as one RL_LINES segment per
// consecutive pair of poses instead. No text labels here (the original had
// none), so unlike GroundControlPoints this needs no separate 2D-pass
// function.
void renderGNSS(const GNSS& gnss, const PointClouds& point_clouds_container)
{
    if (gnss.gnss_poses.size() >= 2)
    {
        rlBegin(RL_LINES);
        rlColor3f(1.0f, 1.0f, 1.0f);
        for (size_t i = 0; i + 1 < gnss.gnss_poses.size(); ++i)
        {
            const auto& a = gnss.gnss_poses[i];
            const auto& b = gnss.gnss_poses[i + 1];
            rlVertex3f(
                static_cast<float>(a.enu_x - point_clouds_container.offset.x()),
                static_cast<float>(a.enu_y - point_clouds_container.offset.y()),
                static_cast<float>(a.enu_z - point_clouds_container.offset.z()));
            rlVertex3f(
                static_cast<float>(b.enu_x - point_clouds_container.offset.x()),
                static_cast<float>(b.enu_y - point_clouds_container.offset.y()),
                static_cast<float>(b.enu_z - point_clouds_container.offset.z()));
        }
        rlEnd();
    }

    if (gnss.show_correspondences)
    {
        rlBegin(RL_LINES);
        rlColor3f(1.0f, 0.0f, 0.0f);
        for (const auto& pc : point_clouds_container.point_clouds)
        {
            for (size_t i = 0; i < gnss.gnss_poses.size(); ++i)
            {
                double time_stamp = gnss.gnss_poses[i].timestamp;

                auto it = std::lower_bound(
                    pc.local_trajectory.begin(),
                    pc.local_trajectory.end(),
                    time_stamp,
                    [](const PointCloud::LocalTrajectoryNode& lhs, const double& time) -> bool
                    {
                        return lhs.timestamps.first < time;
                    });

                size_t index = static_cast<size_t>(it - pc.local_trajectory.begin());

                if (index > 0 && index < pc.local_trajectory.size())
                {
                    if (fabs(time_stamp - pc.local_trajectory[index].timestamps.first) < 10e12)
                    {
                        auto m = pc.m_pose * pc.local_trajectory[index].m_pose;
                        rlVertex3f(static_cast<float>(m(0, 3)), static_cast<float>(m(1, 3)), static_cast<float>(m(2, 3)));

                        rlVertex3f(
                            static_cast<float>(gnss.gnss_poses[i].enu_x - point_clouds_container.offset.x()),
                            static_cast<float>(gnss.gnss_poses[i].enu_y - point_clouds_container.offset.y()),
                            static_cast<float>(gnss.gnss_poses[i].enu_z - point_clouds_container.offset.z()));
                    }
                }
            }
        }
        rlEnd();
    }
}

// TUM trajectories are treated like a second GNSS-style external track (see
// renderGNSS() above, whose structure this mirrors): a polyline through
// tum_poses plus, when show_correspondences is set, lines to the nearest
// local_trajectory sample of every loaded scan by timestamp. Unlike GNSS,
// TumPose::x/y/z are already Cartesian in the trajectory's own frame, so
// only the point_clouds_container offset is subtracted, with no ENU/PROJ
// conversion.
void renderTUM(const TUM& tum, const PointClouds& point_clouds_container)
{
    // Cached across frames -- re-uploaded only when tum_poses actually
    // changed (tum.version) or the point cloud recentering offset shifted,
    // not every frame (see ScanRenderer::PointsGPU's own comment for why
    // that matters).
    static ScanRenderer::PointsGPU tumPointsGPU;
    static size_t tumPointsGPUVersion = SIZE_MAX;
    static Eigen::Vector3d tumPointsGPUOffset = Eigen::Vector3d::Zero();

    if (!tum.tum_poses.empty())
    {
        bool stale = tumPointsGPUVersion != tum.version || !tumPointsGPUOffset.isApprox(point_clouds_container.offset, 1e-9);
        if (stale)
        {
            std::vector<Eigen::Vector3d> positions;
            positions.reserve(tum.tum_poses.size());
            for (const auto& p : tum.tum_poses)
            {
                positions.emplace_back(
                    p.x - point_clouds_container.offset.x(),
                    p.y - point_clouds_container.offset.y(),
                    p.z - point_clouds_container.offset.z());
            }
            scan_renderer.uploadPoints(tumPointsGPU, positions);
            tumPointsGPUVersion = tum.version;
            tumPointsGPUOffset = point_clouds_container.offset;
        }
        scan_renderer.drawPoints(tumPointsGPU, YELLOW, tum.point_size);
    }

    if (tum.show_correspondences)
    {
        rlBegin(RL_LINES);
        rlColor3f(1.0f, 0.0f, 0.0f);
        for (const auto& pc : point_clouds_container.point_clouds)
        {
            for (size_t i = 0; i < tum.tum_poses.size(); ++i)
            {
                // TUM timestamps are Unix-epoch seconds; local_trajectory's
                // timestamps.first is the LIO trajectory CSV's
                // "timestamp_nanoseconds" column -- Unix-epoch nanoseconds,
                // same epoch, 1e9x the scale. timestamps.second
                // ("timestampUnix_nanoseconds") looks like the more obvious
                // match by name, but is 0 for every node unless that column
                // was actually captured during LIO (commonly isn't), so
                // matching against it silently finds nothing -- .first with
                // the unit conversion below is the field that's actually
                // populated.
                double time_stamp_ns = tum.tum_poses[i].timestamp * 1.0e9;

                auto it = std::lower_bound(
                    pc.local_trajectory.begin(),
                    pc.local_trajectory.end(),
                    time_stamp_ns,
                    [](const PointCloud::LocalTrajectoryNode& lhs, const double& time) -> bool
                    {
                        return lhs.timestamps.first < time;
                    });

                size_t index = static_cast<size_t>(it - pc.local_trajectory.begin());

                if (index > 0 && index < pc.local_trajectory.size())
                {
                    if (fabs(time_stamp_ns - pc.local_trajectory[index].timestamps.first) < 5.0e8) // 0.5s, in ns
                    {
                        auto m = pc.m_pose * pc.local_trajectory[index].m_pose;
                        rlVertex3f(static_cast<float>(m(0, 3)), static_cast<float>(m(1, 3)), static_cast<float>(m(2, 3)));

                        rlVertex3f(
                            static_cast<float>(tum.tum_poses[i].x - point_clouds_container.offset.x()),
                            static_cast<float>(tum.tum_poses[i].y - point_clouds_container.offset.y()),
                            static_cast<float>(tum.tum_poses[i].z - point_clouds_container.offset.z()));
                    }
                }
            }
        }
        rlEnd();
    }
}

// Was ControlPoints::render() (core/src/control_points.cpp) -- legacy-GL,
// compiled once into `core` and shared with the remaining GLUT apps, so it
// can't be touched; reimplemented here. Two parts, like the original's
// show_pc flag:
//  - While editing (control_points.is_imgui), the bulk multi-scan
//    scan_renderer.draw() call in display() is skipped entirely (see this
//    function's caller), so the original's per-point GL_POINTS draw of just
//    the active (index_pose) scan, colored by intensity, is replaced here
//    by restricting a scan_renderer.draw() call to that one scan instead of
//    reimplementing per-point immediate-mode drawing (rlBegin() has no
//    RL_POINTS mode -- see drawUncertaintyEllipse()'s comment for the same
//    rlBegin() constraint elsewhere in this file). Its GL_LINE_STRIP
//    trajectory becomes RL_LINES segments, same reasoning.
//  - The per-control-point crosshair/connector/ellipse markers, drawn
//    unconditionally in the original (both show_pc branches called this),
//    are always drawn regardless of is_imgui -- text labels are handled
//    separately by renderControlPointsLabels().
void renderControlPoints(const ControlPoints& control_points, PointClouds& point_clouds_container)
{
    auto& pointClouds = point_clouds_container.point_clouds;

    if (control_points.is_imgui && control_points.index_pose >= 0 && static_cast<size_t>(control_points.index_pose) < pointClouds.size())
    {
        std::vector<bool> wasVisible(pointClouds.size());
        for (size_t i = 0; i < pointClouds.size(); ++i)
        {
            wasVisible[i] = pointClouds[i].visible;
            pointClouds[i].visible = (static_cast<int>(i) == control_points.index_pose);
        }

        scan_renderer.draw(
            pointClouds,
            static_cast<float>(app_state.point_size),
            ScanColorMode::Intensity,
            static_cast<float>(session_dims.z_min),
            static_cast<float>(session_dims.z_max),
            Eigen::Vector3d(
                app_state.camera.euler.rotationCenter.x, app_state.camera.euler.rotationCenter.y, app_state.camera.euler.rotationCenter.z),
            static_cast<float>(std::max({ session_dims.length, session_dims.width, session_dims.height, 1.0 })),
            1,
            point_clouds_container.xz_intersection,
            point_clouds_container.yz_intersection,
            point_clouds_container.xy_intersection,
            static_cast<float>(point_clouds_container.intersection_width));

        for (size_t i = 0; i < pointClouds.size(); ++i)
        {
            pointClouds[i].visible = wasVisible[i];
        }

        const auto& activePc = pointClouds[control_points.index_pose];
        if (activePc.local_trajectory.size() >= 2)
        {
            rlBegin(RL_LINES);
            rlColor3f(0.0f, 1.0f, 0.0f);
            for (size_t i = 0; i + 1 < activePc.local_trajectory.size(); ++i)
            {
                auto poseA = activePc.m_pose * activePc.local_trajectory[i].m_pose;
                auto poseB = activePc.m_pose * activePc.local_trajectory[i + 1].m_pose;
                rlVertex3f(static_cast<float>(poseA(0, 3)), static_cast<float>(poseA(1, 3)), static_cast<float>(poseA(2, 3)));
                rlVertex3f(static_cast<float>(poseB(0, 3)), static_cast<float>(poseB(1, 3)), static_cast<float>(poseB(2, 3)));
            }
            rlEnd();
        }
    }

    const Color markColor{ 179, 77, 128, 255 }; // was glColor3f(0.7f, 0.3f, 0.5f)
    const Color connectorColor{ 0, 77, 153, 255 }; // was glColor3f(0.0f, 0.3f, 0.6f)

    for (const auto& cp : control_points.cps)
    {
        if (cp.index_to_pose < 0 || static_cast<size_t>(cp.index_to_pose) >= pointClouds.size())
        {
            continue;
        }

        Eigen::Vector3d p(cp.x_source_local, cp.y_source_local, cp.z_source_local);
        Eigen::Vector3d c = pointClouds[cp.index_to_pose].m_pose * p;
        Vector3 g{ static_cast<float>(cp.x_target_global), static_cast<float>(cp.y_target_global), static_cast<float>(cp.z_target_global) };

        DrawLine3D(Vector3{ g.x - 0.05f, g.y, g.z }, Vector3{ g.x + 0.05f, g.y, g.z }, markColor);
        DrawLine3D(Vector3{ g.x, g.y - 0.05f, g.z }, Vector3{ g.x, g.y + 0.05f, g.z }, markColor);
        DrawLine3D(Vector3{ g.x - 0.01f, g.y, g.z }, Vector3{ g.x + 0.01f, g.y, g.z }, markColor);
        DrawLine3D(Vector3{ g.x, g.y - 0.01f, g.z }, Vector3{ g.x, g.y + 0.01f, g.z }, markColor);
        // Original's 5th line pair was glVertex3f(g,g) twice -- a
        // degenerate zero-length segment that draws nothing. Dropped.

        DrawLine3D(Vector3{ static_cast<float>(c.x()), static_cast<float>(c.y()), static_cast<float>(c.z()) }, g, connectorColor);

        if (control_points.draw_uncertainty)
        {
            Eigen::Matrix3d covar = Eigen::Matrix3d::Zero();
            if (cp.is_z_0)
            {
                covar(0, 0) = 0.01 * 0.01;
                covar(1, 1) = 0.01 * 0.01;
            }
            else
            {
                covar(0, 0) = cp.sigma_x * cp.sigma_x;
                covar(1, 1) = cp.sigma_y * cp.sigma_y;
            }
            covar(2, 2) = cp.sigma_z * cp.sigma_z;

            Eigen::Vector3d mean(cp.x_target_global, cp.y_target_global, cp.z_target_global);
            drawUncertaintyEllipse(covar, mean, GRAY); // was Eigen::Vector3f(0.5, 0.5, 0.5)
        }
    }
}

// Was ManualPoseGraphLoopClosure::Render()'s per-pose glRasterPos3f +
// glutBitmapString(std::to_string(i)) labels (one per point cloud, plus one
// per edge at its flagpole top) -- neither has an rlgl/raylib equivalent
// (no matrix-anchored bitmap fonts under core profile), so this projects
// each world position to screen space by hand (using frame_mvp_3d, captured
// in display() while the 3D projection/modelview was still active -- see
// its declaration) and draws with DrawText instead. Must run after
// end3DMatrixStack() (2D screen-space drawing).
namespace
{
    // Plain DrawText at a point sitting exactly on top of a same-size, often
    // same-color trajectory marker is easy to lose visually -- outlined in
    // black and nudged up-right of the anchor so it reads clearly regardless
    // of what's directly underneath. `line` stacks additional labels above
    // the same anchor (one line height per unit) -- needed wherever several
    // labels sit at world points too close together to separate on screen
    // by their 3D position alone (e.g. GCP's LiDAR-center/ground-plane/name
    // labels, which differ by only lidar_height_above_ground/0.1m).
    void drawOutlinedText(const char* text, Vector2 anchor, int fontSize, Color color, int line = 0)
    {
        int x = static_cast<int>(anchor.x) + 6;
        int y = static_cast<int>(anchor.y) - fontSize - 6 - line * (fontSize + 4);
        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                if (dx != 0 || dy != 0)
                {
                    DrawText(text, x + dx, y + dy, fontSize, BLACK);
                }
            }
        }
        DrawText(text, x, y, fontSize, color);
    }

    // Manual clip-space transform (mat * [x,y,z,1]^T) -- raymath's
    // Vector3Transform computes the same x/y/z but drops w, which the
    // perspective divide below needs, so it can't be reused here.
    Vector2 worldToScreen(const Eigen::Vector3d& world, float screenW, float screenH)
    {
        const Matrix& m = frame_mvp_3d;
        float x = static_cast<float>(world.x());
        float y = static_cast<float>(world.y());
        float z = static_cast<float>(world.z());
        float clipX = m.m0 * x + m.m4 * y + m.m8 * z + m.m12;
        float clipY = m.m1 * x + m.m5 * y + m.m9 * z + m.m13;
        float clipW = m.m3 * x + m.m7 * y + m.m11 * z + m.m15;
        if (fabsf(clipW) < 1e-6f)
        {
            return Vector2{ -1000.f, -1000.f };
        }
        float ndcX = clipX / clipW;
        float ndcY = clipY / clipW;
        return Vector2{ (ndcX * 0.5f + 0.5f) * screenW, (1.0f - (ndcY * 0.5f + 0.5f)) * screenH };
    }
} // namespace

void renderLoopClosureLabels(PointClouds& point_clouds_container)
{
    auto& pointClouds = point_clouds_container.point_clouds;

    // io.DisplaySize, not GetScreenWidth()/GetScreenHeight(): must match
    // whatever reshape() last set the actual GL viewport to (see
    // end3DMatrixStack()'s comment) -- the two can differ under DPI
    // scaling, which would silently throw this off-screen.
    ImGuiIO& io = ImGui::GetIO();
    float screenW = io.DisplaySize.x;
    float screenH = io.DisplaySize.y;

    for (size_t i = 0; i < pointClouds.size(); ++i)
    {
        Vector2 screen = worldToScreen(pointClouds[i].m_pose.translation(), screenW, screenH);
        drawOutlinedText(TextFormat("%d", static_cast<int>(i)), screen, 20, WHITE);
    }

    for (size_t i = 0; i < session.pose_graph_loop_closure.edges.size(); ++i)
    {
        const auto& edge = session.pose_graph_loop_closure.edges[i];
        if (edge.index_from < 0 || static_cast<size_t>(edge.index_from) >= pointClouds.size() || edge.index_to < 0 ||
            static_cast<size_t>(edge.index_to) >= pointClouds.size())
        {
            continue;
        }

        Eigen::Vector3d worldSrc = pointClouds[edge.index_from].m_pose.translation();
        Eigen::Vector3d worldTrg = pointClouds[edge.index_to].m_pose.translation();
        Eigen::Vector3d midUp = (worldSrc + worldTrg) * 0.5 + Eigen::Vector3d(0, 0, 10);

        Vector2 screen = worldToScreen(midUp, screenW, screenH);
        Color c = (static_cast<int>(i) == session.pose_graph_loop_closure.index_active_edge) ? RED : SKYBLUE;
        drawOutlinedText(TextFormat("%d", static_cast<int>(i)), screen, 22, c);
    }
}

// Was GroundControlPoints::render()'s glRasterPos3f + glutBitmapString
// calls (core/src/ground_control_points.cpp) -- same reasoning as
// renderLoopClosureLabels() (no matrix-anchored bitmap fonts under core
// profile), projected to screen space and drawn with DrawText instead.
// Must run after end3DMatrixStack(), like renderLoopClosureLabels().
void renderGroundControlPointsLabels(const GroundControlPoints& ground_control_points, const PointClouds& point_clouds_container)
{
    ImGuiIO& io = ImGui::GetIO();
    float screenW = io.DisplaySize.x;
    float screenH = io.DisplaySize.y;

    const Color markColor{ 179, 77, 128, 255 }; // was glColor3f(0.7f, 0.3f, 0.5f)
    const Color connectorColor{ 0, 77, 153, 255 }; // was glColor3f(0.0f, 0.3f, 0.6f)

    for (size_t i = 0; i < ground_control_points.gpcs.size(); ++i)
    {
        const auto& gcp = ground_control_points.gpcs[i];

        // LiDAR center (z+h), ground plane (z) and name (z+h+0.1) sit
        // world-space centimeters apart -- at any normal zoom that's the
        // same handful of screen pixels, so unlike renderLoopClosureLabels()
        // (one label per anchor) these three share a single screen anchor
        // and stack via drawOutlinedText's `line` instead of relying on
        // their (invisible-on-screen) 3D separation.
        Vector2 anchor = worldToScreen(Eigen::Vector3d(gcp.x, gcp.y, gcp.z), screenW, screenH);

        // was glColor3f(0, 0, 0) -- plain black text with no outline in the
        // GLUT original; drawOutlinedText always outlines in black, so
        // black text would vanish. WHITE instead, matching
        // renderLoopClosureLabels()'s index labels.
        drawOutlinedText(gcp.name, anchor, 22, WHITE, 2);
        drawOutlinedText(TextFormat("GCP_%d: LiDAR center", static_cast<int>(i)), anchor, 14, markColor, 1);
        drawOutlinedText(TextFormat("GCP_%d: 'plane on the ground'", static_cast<int>(i)), anchor, 14, markColor, 0);

        if (gcp.index_to_node_inner < 0 || static_cast<size_t>(gcp.index_to_node_inner) >= point_clouds_container.point_clouds.size())
        {
            continue;
        }
        const auto& pc = point_clouds_container.point_clouds[gcp.index_to_node_inner];
        if (gcp.index_to_node_outer < 0 || static_cast<size_t>(gcp.index_to_node_outer) >= pc.local_trajectory.size())
        {
            continue;
        }

        Eigen::Vector3d c = pc.m_pose * pc.local_trajectory[gcp.index_to_node_outer].m_pose.translation();
        Vector2 nodeScreen = worldToScreen(c, screenW, screenH);
        drawOutlinedText(TextFormat("GCP_%d: assigned trajectory node", static_cast<int>(i)), nodeScreen, 14, connectorColor);
    }
}

// Was ControlPoints::render()'s glRasterPos3f + glutBitmapString calls
// (core/src/control_points.cpp) -- same reasoning as
// renderGroundControlPointsLabels() (no matrix-anchored bitmap fonts under
// core profile, and name/"CP_i" sit only 0.1m apart in world space -- too
// close to separate on screen at normal zoom -- so they share one anchor
// and stack via drawOutlinedText's `line`). Must run after
// end3DMatrixStack(), like renderGroundControlPointsLabels().
void renderControlPointsLabels(const ControlPoints& control_points, const PointClouds& point_clouds_container)
{
    ImGuiIO& io = ImGui::GetIO();
    float screenW = io.DisplaySize.x;
    float screenH = io.DisplaySize.y;

    const Color markColor{ 179, 77, 128, 255 }; // was glColor3f(0.7f, 0.3f, 0.5f)

    for (size_t i = 0; i < control_points.cps.size(); ++i)
    {
        const auto& cp = control_points.cps[i];

        Vector2 anchor = worldToScreen(Eigen::Vector3d(cp.x_target_global, cp.y_target_global, cp.z_target_global), screenW, screenH);

        // was glColor3f(0, 0, 0) -- plain black text with no outline in the
        // GLUT original; drawOutlinedText always outlines in black, so
        // black text would vanish. WHITE instead, matching
        // renderGroundControlPointsLabels()'s name label.
        drawOutlinedText(cp.name, anchor, 22, WHITE, 1);
        drawOutlinedText(TextFormat("CP_%d", static_cast<int>(i)), anchor, 14, WHITE, 0);

        if (cp.index_to_pose < 0 || static_cast<size_t>(cp.index_to_pose) >= point_clouds_container.point_clouds.size())
        {
            continue;
        }

        Eigen::Vector3d p(cp.x_source_local, cp.y_source_local, cp.z_source_local);
        Eigen::Vector3d c = point_clouds_container.point_clouds[cp.index_to_pose].m_pose * p;
        Vector2 sourceScreen = worldToScreen(c, screenW, screenH);
        drawOutlinedText(TextFormat("CP_%d: initial location", static_cast<int>(i)), sourceScreen, 14, markColor);
    }
}

void display()
{
    // Safety net: rebuilds any scan whose m_pose no longer matches its
    // cached GPU buffer, regardless of what changed it (registration
    // panels, gizmo, translate tool, settings, scan editor, loop closure
    // Gui() -- which can move point cloud poses through paths that don't
    // individually call scan_renderer.rebuild()).
    scan_renderer.syncPoses(session.point_clouds_container.point_clouds);

    ImGuiIO& io = ImGui::GetIO();
    // GetRenderWidth/Height(), not io.DisplaySize: the GL viewport must be
    // sized in actual framebuffer pixels, which on a Retina Mac are a
    // multiple of io.DisplaySize's logical points (see initGL()'s
    // FLAG_WINDOW_HIGHDPI comment) -- using io.DisplaySize directly here
    // left the 3D scene rendered only into the bottom-left quarter of the
    // window, the rest showing just the clear color.
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

    app_state.camera.updateEulerTransition(io.DeltaTime);

    app_state.viewLocal = Eigen::Affine3f::Identity();

    if (!app_state.camera.isOrtho)
    {
        app_state.camera.applyPerspectiveProjection((int)io.DisplaySize.x, (int)io.DisplaySize.y);

        // janusz
        if (is_loop_closure_gui)
        {
            if (new_loop_closure_index)
            {
                if (index_loop_closure_source < session.point_clouds_container.point_clouds.size())
                {
                    const auto& t = session.point_clouds_container.point_clouds[index_loop_closure_source].m_pose.translation();
                    app_state.camera.moveEulerRotationCenterTo(
                        Vector3{ static_cast<float>(t.x()), static_cast<float>(t.y()), static_cast<float>(t.z()) });
                }

                if (session.pose_graph_loop_closure.manipulate_active_edge)
                {
                    Vector3 center = app_state.camera.eulerGoal.rotationCenter;

                    if (session.pose_graph_loop_closure.edges.size() > 0)
                    {
                        if (session.pose_graph_loop_closure.index_active_edge < session.pose_graph_loop_closure.edges.size())
                        {
                            const auto& t =
                                session.point_clouds_container
                                    .point_clouds[session.pose_graph_loop_closure.edges[session.pose_graph_loop_closure.index_active_edge]
                                                      .index_from]
                                    .m_pose.translation();
                            center = Vector3{ static_cast<float>(t.x()), static_cast<float>(t.y()), static_cast<float>(t.z()) };
                        }
                    }

                    app_state.camera.moveEulerRotationCenterTo(center);
                }

                new_loop_closure_index = false;
            }
        }

        Eigen::Vector3f rotationCenter(
            app_state.camera.euler.rotationCenter.x, app_state.camera.euler.rotationCenter.y, app_state.camera.euler.rotationCenter.z);
        app_state.viewLocal.translate(rotationCenter);

        app_state.viewLocal.translate(
            Eigen::Vector3f(app_state.camera.euler.translate.x, app_state.camera.euler.translate.y, app_state.camera.euler.translate.z));
        if (!app_state.camera.lockZ)
            app_state.viewLocal.rotate(Eigen::AngleAxisf(app_state.camera.euler.rotateX * DEG_TO_RAD, Eigen::Vector3f::UnitX()));
        else
            app_state.viewLocal.rotate(Eigen::AngleAxisf(-90.0 * DEG_TO_RAD, Eigen::Vector3f::UnitX()));
        app_state.viewLocal.rotate(Eigen::AngleAxisf(app_state.camera.euler.rotateY * DEG_TO_RAD, Eigen::Vector3f::UnitZ()));

        app_state.viewLocal.translate(-rotationCenter);

        rlMultMatrixf(app_state.viewLocal.matrix().data());
    }
    else
    {
        // Still updating app_state.viewLocal for the compass -- the rest of
        // the original updateOrthoView() (rlOrtho + the ortho gizmo lookAt)
        // now lives in raylib_widgets::OrbitCamera::updateOrtho().
        app_state.viewLocal.rotate(
            Eigen::AngleAxisf((app_state.camera.euler.rotateX + app_state.camera.euler.rotateY) * DEG_TO_RAD, Eigen::Vector3f::UnitZ()));
        app_state.camera.updateOrtho(ratio);
    }

    app_state.camera.captureFrameMatrices();
    frame_mvp_3d = MatrixMultiply(app_state.camera.frameView3D, app_state.camera.frameProj3D);

    showAxes();
    drawIntersectionGrids(session.point_clouds_container, session_dims);

    // renderLoopClosure() hides every scan except the current source/target
    // range while loop closure editing is active (see its comment) --
    // restore full visibility the moment the panel closes (X button, menu
    // toggle, or Ctrl+L), or every scan but that last-shown range stays
    // hidden. Checked unconditionally (not just when the below block runs)
    // so this still fires even if control_points.is_imgui happens to be
    // true on the closing frame.
    static bool was_loop_closure_gui = false;
    if (was_loop_closure_gui && !is_loop_closure_gui)
    {
        for (auto& pc : session.point_clouds_container.point_clouds)
        {
            pc.visible = true;
        }
    }
    was_loop_closure_gui = is_loop_closure_gui;

    // renderControlPoints() draws its markers regardless of is_imgui (like
    // the original's render() -- both its show_pc=true/false call sites
    // drew them) and, while is_imgui is true, also substitutes for the bulk
    // scan_renderer.draw() call below (skipped entirely in that case -- see
    // its own comment).
    renderControlPoints(session.control_points, session.point_clouds_container);

    if (!session.control_points.is_imgui)
    {
        renderGroundControlPoints(session.ground_control_points, session.point_clouds_container);
        renderGNSS(tls_registration.gnss, session.point_clouds_container);
        renderTUM(tls_registration.tum, session.point_clouds_container);

        if (is_loop_closure_gui)
            renderLoopClosure(
                session.point_clouds_container,
                index_loop_closure_source,
                index_loop_closure_target,
                num_edge_extended_before,
                num_edge_extended_after);
    }

    int prev_index_pose = session.control_points.index_pose;

    if (prev_index_pose != session.control_points.index_pose)
    {
        session.control_points.index_picked_point = -1; // reset picked point when pose changes

        const auto& t = session.point_clouds_container.point_clouds[session.control_points.index_pose].m_pose.translation();
        Vector3 center = { static_cast<float>(t.x()), static_cast<float>(t.y()), static_cast<float>(t.z()) };

        Vector3 translate = app_state.camera.euler.translate;
        if (session.control_points.track_pose_with_camera)
        {
            translate.x = -center.x;
            translate.y = -center.y;
        }

        app_state.camera.startEulerTransition(app_state.camera.euler.rotateX, app_state.camera.euler.rotateY, translate, center);
    }

    // rlImGuiBegin() only polls raylib input into ImGui's IO and calls
    // ImGui::NewFrame() -- it doesn't touch rlgl's matrix stack, so the 3D
    // projection/modelview set up above stays active through all the
    // interleaved 3D drawing + ImGui panel-building code below, exactly
    // like the original (glBegin/glVertex calls and ImGui:: calls building
    // up a draw list are independent of each other either way -- the ImGui
    // draw list only actually hits the GPU once, at rlImGuiEnd() near the
    // end of this function).
    rlImGuiBegin();

    ShowMainDockSpace();

    if (session.control_points.is_imgui)
        session.control_points.imgui(
            session.point_clouds_container,
            Eigen::Vector3f(
                app_state.camera.euler.rotationCenter.x, app_state.camera.euler.rotationCenter.y, app_state.camera.euler.rotationCenter.z));

    if (session.ground_control_points.is_imgui)
        session.ground_control_points.imgui(session.point_clouds_container);

    if (!session.control_points.is_imgui)
    {
        if (!is_loop_closure_gui)
        {
            for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
            {
                if (session.point_clouds_container.point_clouds[i].gizmo)
                {
                    std::vector<Eigen::Affine3d> all_m_poses;
                    for (size_t j = 0; j < session.point_clouds_container.point_clouds.size(); j++)
                    {
                        all_m_poses.push_back(session.point_clouds_container.point_clouds[j].m_pose);
                    }

                    ImGuiIO& io = ImGui::GetIO();
                    // ImGuizmo -----------------------------------------------
                    ImGuizmo::BeginFrame();
                    ImGuizmo::Enable(true);
                    ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

                    if (!app_state.camera.isOrtho)
                    {
                        // Named-field copy (not a raw struct memcpy): Matrix's
                        // declared field order isn't guaranteed to match the
                        // m0..m15 column-major numbering its names imply.
                        Matrix projMat = rlGetMatrixProjection();
                        Matrix modelMat = rlGetMatrixModelview();
                        float projection[16] = { projMat.m0,  projMat.m1,  projMat.m2,  projMat.m3, projMat.m4,  projMat.m5,
                                                 projMat.m6,  projMat.m7,  projMat.m8,  projMat.m9, projMat.m10, projMat.m11,
                                                 projMat.m12, projMat.m13, projMat.m14, projMat.m15 };
                        float modelview[16] = { modelMat.m0,  modelMat.m1,  modelMat.m2,  modelMat.m3, modelMat.m4,  modelMat.m5,
                                                modelMat.m6,  modelMat.m7,  modelMat.m8,  modelMat.m9, modelMat.m10, modelMat.m11,
                                                modelMat.m12, modelMat.m13, modelMat.m14, modelMat.m15 };

                        ImGuizmo::Manipulate(
                            &modelview[0],
                            &projection[0],
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

                    session.point_clouds_container.point_clouds[i].m_pose(0, 0) = m_gizmo[0];
                    session.point_clouds_container.point_clouds[i].m_pose(1, 0) = m_gizmo[1];
                    session.point_clouds_container.point_clouds[i].m_pose(2, 0) = m_gizmo[2];
                    session.point_clouds_container.point_clouds[i].m_pose(3, 0) = m_gizmo[3];
                    session.point_clouds_container.point_clouds[i].m_pose(0, 1) = m_gizmo[4];
                    session.point_clouds_container.point_clouds[i].m_pose(1, 1) = m_gizmo[5];
                    session.point_clouds_container.point_clouds[i].m_pose(2, 1) = m_gizmo[6];
                    session.point_clouds_container.point_clouds[i].m_pose(3, 1) = m_gizmo[7];
                    session.point_clouds_container.point_clouds[i].m_pose(0, 2) = m_gizmo[8];
                    session.point_clouds_container.point_clouds[i].m_pose(1, 2) = m_gizmo[9];
                    session.point_clouds_container.point_clouds[i].m_pose(2, 2) = m_gizmo[10];
                    session.point_clouds_container.point_clouds[i].m_pose(3, 2) = m_gizmo[11];
                    session.point_clouds_container.point_clouds[i].m_pose(0, 3) = m_gizmo[12];
                    session.point_clouds_container.point_clouds[i].m_pose(1, 3) = m_gizmo[13];
                    session.point_clouds_container.point_clouds[i].m_pose(2, 3) = m_gizmo[14];
                    session.point_clouds_container.point_clouds[i].m_pose(3, 3) = m_gizmo[15];
                    session.point_clouds_container.point_clouds[i].pose =
                        pose_tait_bryan_from_affine_matrix(session.point_clouds_container.point_clouds[i].m_pose);

                    session.point_clouds_container.point_clouds[i].gui_translation[0] =
                        (float)session.point_clouds_container.point_clouds[i].pose.px;
                    session.point_clouds_container.point_clouds[i].gui_translation[1] =
                        (float)session.point_clouds_container.point_clouds[i].pose.py;
                    session.point_clouds_container.point_clouds[i].gui_translation[2] =
                        (float)session.point_clouds_container.point_clouds[i].pose.pz;

                    session.point_clouds_container.point_clouds[i].gui_rotation[0] =
                        (float)(session.point_clouds_container.point_clouds[i].pose.om * RAD_TO_DEG);
                    session.point_clouds_container.point_clouds[i].gui_rotation[1] =
                        (float)(session.point_clouds_container.point_clouds[i].pose.fi * RAD_TO_DEG);
                    session.point_clouds_container.point_clouds[i].gui_rotation[2] =
                        (float)(session.point_clouds_container.point_clouds[i].pose.ka * RAD_TO_DEG);

                    if (!manipulate_only_marked_gizmo)
                    {
                        Eigen::Affine3d curr_m_pose = session.point_clouds_container.point_clouds[i].m_pose;
                        for (size_t j = i + 1; j < session.point_clouds_container.point_clouds.size(); j++)
                        {
                            curr_m_pose = curr_m_pose * (all_m_poses[j - 1].inverse() * all_m_poses[j]);
                            session.point_clouds_container.point_clouds[j].m_pose = curr_m_pose;
                            session.point_clouds_container.point_clouds[j].pose =
                                pose_tait_bryan_from_affine_matrix(session.point_clouds_container.point_clouds[j].m_pose);

                            session.point_clouds_container.point_clouds[j].gui_translation[0] =
                                (float)session.point_clouds_container.point_clouds[j].pose.px;
                            session.point_clouds_container.point_clouds[j].gui_translation[1] =
                                (float)session.point_clouds_container.point_clouds[j].pose.py;
                            session.point_clouds_container.point_clouds[j].gui_translation[2] =
                                (float)session.point_clouds_container.point_clouds[j].pose.pz;

                            session.point_clouds_container.point_clouds[j].gui_rotation[0] =
                                (float)(session.point_clouds_container.point_clouds[j].pose.om * RAD_TO_DEG);
                            session.point_clouds_container.point_clouds[j].gui_rotation[1] =
                                (float)(session.point_clouds_container.point_clouds[j].pose.fi * RAD_TO_DEG);
                            session.point_clouds_container.point_clouds[j].gui_rotation[2] =
                                (float)(session.point_clouds_container.point_clouds[j].pose.ka * RAD_TO_DEG);
                        }
                    }
                }
            }

            // Was PointClouds::render() (legacy-GL, in `core`, shared with
            // GLUT apps) -- replaced with scan_renderer, which is kept in
            // sync via rebuildAll()/syncPoses() at session-load/pose-change
            // sites and each frame (see main()/loadSession() below).
            scan_renderer.draw(
                session.point_clouds_container.point_clouds,
                static_cast<float>(app_state.point_size),
                scanColorModeFromScheme(csPointCloud),
                static_cast<float>(session_dims.z_min),
                static_cast<float>(session_dims.z_max),
                Eigen::Vector3d(
                    app_state.camera.euler.rotationCenter.x,
                    app_state.camera.euler.rotationCenter.y,
                    app_state.camera.euler.rotationCenter.z),
                static_cast<float>(std::max({ session_dims.length, session_dims.width, session_dims.height, 1.0 })),
                app_state.viewer_decimate_point_cloud,
                session.point_clouds_container.xz_intersection,
                session.point_clouds_container.yz_intersection,
                session.point_clouds_container.xy_intersection,
                static_cast<float>(session.point_clouds_container.intersection_width));
            scan_renderer.drawTrajectories(
                session.point_clouds_container.point_clouds,
                1,
                session.point_clouds_container.show_imu_to_lio_diff,
                session.point_clouds_container.xz_intersection,
                session.point_clouds_container.yz_intersection,
                session.point_clouds_container.xy_intersection);

            observationPickingRender(observation_picking);

            for (const auto& obs : observation_picking.observations)
            {
                for (const auto& [key1, value1] : obs)
                {
                    for (const auto& [key2, value2] : obs)
                    {
                        if (key1 != key2)
                        {
                            Eigen::Vector3d p1, p2;
                            if (session.point_clouds_container.show_with_initial_pose)
                            {
                                p1 = session.point_clouds_container.point_clouds[key1].m_initial_pose * value1;
                                p2 = session.point_clouds_container.point_clouds[key2].m_initial_pose * value2;
                            }
                            else
                            {
                                p1 = session.point_clouds_container.point_clouds[key1].m_pose * value1;
                                p2 = session.point_clouds_container.point_clouds[key2].m_pose * value2;
                            }
                            DrawSphere(
                                Vector3{ static_cast<float>(p1.x()), static_cast<float>(p1.y()), static_cast<float>(p1.z()) },
                                0.05f,
                                GREEN);
                            DrawSphere(
                                Vector3{ static_cast<float>(p2.x()), static_cast<float>(p2.y()), static_cast<float>(p2.z()) },
                                0.05f,
                                GREEN);
                            rlColor3f(1, 0, 0);
                            rlBegin(RL_LINES);
                            rlVertex3f(p1.x(), p1.y(), p1.z());
                            rlVertex3f(p2.x(), p2.y(), p2.z());
                            rlEnd();
                        }
                    }
                }
            }

            for (const auto& obs : observation_picking.observations)
            {
                Eigen::Vector3d mean(0, 0, 0);
                int counter = 0;
                for (const auto& [key1, value1] : obs)
                {
                    mean += session.point_clouds_container.point_clouds[key1].m_initial_pose * value1;
                    counter++;
                }
                if (counter > 0)
                {
                    mean /= counter;

                    // RL_LINE_STRIP isn't supported by rlBegin() (only
                    // RL_LINES/RL_TRIANGLES/RL_QUADS are) -- each edge of
                    // the square drawn as its own line segment instead.
                    rlColor3f(1, 0, 0);
                    rlBegin(RL_LINES);
                    rlVertex3f(mean.x() - 1, mean.y() - 1, mean.z());
                    rlVertex3f(mean.x() + 1, mean.y() - 1, mean.z());
                    rlVertex3f(mean.x() + 1, mean.y() - 1, mean.z());
                    rlVertex3f(mean.x() + 1, mean.y() + 1, mean.z());
                    rlVertex3f(mean.x() + 1, mean.y() + 1, mean.z());
                    rlVertex3f(mean.x() - 1, mean.y() + 1, mean.z());
                    rlVertex3f(mean.x() - 1, mean.y() + 1, mean.z());
                    rlVertex3f(mean.x() - 1, mean.y() - 1, mean.z());
                    rlEnd();
                }
            }

            for (auto p : picked_points)
            {
                DrawSphere(Vector3{ static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()) }, 0.05f, MAGENTA);
            }
        }
        else
        {
            // ImGuizmo -----------------------------------------------
            if (session.pose_graph_loop_closure.gizmo && session.pose_graph_loop_closure.edges.size() > 0)
            {
                ImGuizmo::BeginFrame();
                ImGuizmo::Enable(true);
                ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

                if (!app_state.camera.isOrtho)
                {
                    Matrix projMat = rlGetMatrixProjection();
                    Matrix modelMat = rlGetMatrixModelview();
                    float projection[16] = { projMat.m0,  projMat.m1,  projMat.m2,  projMat.m3, projMat.m4,  projMat.m5,
                                             projMat.m6,  projMat.m7,  projMat.m8,  projMat.m9, projMat.m10, projMat.m11,
                                             projMat.m12, projMat.m13, projMat.m14, projMat.m15 };
                    float modelview[16] = { modelMat.m0,  modelMat.m1,  modelMat.m2,  modelMat.m3, modelMat.m4,  modelMat.m5,
                                            modelMat.m6,  modelMat.m7,  modelMat.m8,  modelMat.m9, modelMat.m10, modelMat.m11,
                                            modelMat.m12, modelMat.m13, modelMat.m14, modelMat.m15 };

                    ImGuizmo::Manipulate(
                        &modelview[0],
                        &projection[0],
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

                Eigen::Affine3d m_g = Eigen::Affine3d::Identity();

                m_g(0, 0) = m_gizmo[0];
                m_g(1, 0) = m_gizmo[1];
                m_g(2, 0) = m_gizmo[2];
                m_g(3, 0) = m_gizmo[3];
                m_g(0, 1) = m_gizmo[4];
                m_g(1, 1) = m_gizmo[5];
                m_g(2, 1) = m_gizmo[6];
                m_g(3, 1) = m_gizmo[7];
                m_g(0, 2) = m_gizmo[8];
                m_g(1, 2) = m_gizmo[9];
                m_g(2, 2) = m_gizmo[10];
                m_g(3, 2) = m_gizmo[11];
                m_g(0, 3) = m_gizmo[12];
                m_g(1, 3) = m_gizmo[13];
                m_g(2, 3) = m_gizmo[14];
                m_g(3, 3) = m_gizmo[15];

                const int& index_src = session.pose_graph_loop_closure.edges[session.pose_graph_loop_closure.index_active_edge].index_from;

                const Eigen::Affine3d& m_src = session.point_clouds_container.point_clouds.at(index_src).m_pose;
                session.pose_graph_loop_closure.edges[session.pose_graph_loop_closure.index_active_edge].relative_pose_tb =
                    pose_tait_bryan_from_affine_matrix(m_src.inverse() * m_g);
            }
        }
    }

    view_kbd_shortcuts();

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_A, false))
    {
        is_pca_gui = !is_pca_gui;

        // workaround
        io.AddKeyEvent(ImGuiKey_A, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
    }

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_C, false))
    {
        session.control_points.is_imgui = !session.control_points.is_imgui;

        // workaround
        io.AddKeyEvent(ImGuiKey_C, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
    }

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_E, false))
    {
        is_lio_segments_gui = !is_lio_segments_gui;

        // workaround
        io.AddKeyEvent(ImGuiKey_E, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
    }

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_G, false))
    {
        session.ground_control_points.is_imgui = !session.ground_control_points.is_imgui;

        // workaround
        io.AddKeyEvent(ImGuiKey_G, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
    }

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_L, false))
    {
        is_loop_closure_gui = !is_loop_closure_gui;

        // workaround
        io.AddKeyEvent(ImGuiKey_L, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
    }

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_O, false))
    {
        openSession();

        // workaround
        io.AddKeyEvent(ImGuiKey_O, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
    }

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_P, false))
    {
        is_pose_graph_slam = !is_pose_graph_slam;

        // workaround
        io.AddKeyEvent(ImGuiKey_P, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
    }

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_R)) // random colors
    {
        for (auto& pc : session.point_clouds_container.point_clouds)
        {
            pc.render_color[0] = float(rand() % 255) / 255.0f;
            pc.render_color[1] = float(rand() % 255) / 255.0f;
            pc.render_color[2] = float(rand() % 255) / 255.0f;

            if (csTrajectory == CS_FOLLOW)
            {
                pc.traj_color[0] = pc.render_color[0];
                pc.traj_color[1] = pc.render_color[1];
                pc.traj_color[2] = pc.render_color[2];
            }
        }

        // workaround
        io.AddKeyEvent(ImGuiKey_R, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
    }

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S, false))
    {
        if (io.KeyShift)
            saveSubsession();
        else
            saveSession();

        // workaround
        io.AddKeyEvent(ImGuiKey_S, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
    }

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_T)) // solid colors
    {
        csPointCloud = CS_SOLID;

        float color[3];
        if (session_loaded)
        {
            color[0] = session.point_clouds_container.point_clouds[0].render_color[0];
            color[1] = session.point_clouds_container.point_clouds[0].render_color[1];
            color[2] = session.point_clouds_container.point_clouds[0].render_color[2];
        }

        for (auto& pc : session.point_clouds_container.point_clouds)
        {
            pc.render_color[0] = color[0];
            pc.render_color[1] = color[1];
            pc.render_color[2] = color[2];

            if (csTrajectory == CS_FOLLOW)
            {
                pc.traj_color[0] = pc.render_color[0];
                pc.traj_color[1] = pc.render_color[1];
                pc.traj_color[2] = pc.render_color[2];
            }
        }

        // workaround
        io.AddKeyEvent(ImGuiKey_T, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
    }

    if (ImGui::BeginMainMenuBar())
    {
        if (!session_loaded)
        {
            if (ImGui::Button("Open session"))
                openSession();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Select session to open (Ctrl+O)");

            ImGui::SameLine();

            if (ImGui::ArrowButton("##menuArrow", ImGuiDir_Down))
                ImGui::OpenPopup("OpenMenu");

            if (ImGui::BeginPopup("OpenMenu"))
            {
                ImGui::MenuItem("Calculate_offset", nullptr, &tls_registration.calculate_offset);
                ImGui::MenuItem("Fill in session", nullptr, &fillInSession);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Fill in data for trajectory and pose to create complete session");

                if (ImGui::MenuItem("Open las/laz"))
                    openLaz(fillInSession);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Create session from las/laz file(s)");

                if (ImGui::MenuItem("Open e57"))
                    openE57(fillInSession);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Create session from e57 file(s); embedded per-scan poses are used as initial poses");

                ImGui::EndPopup();
            }

            ImGui::SameLine();
            ImGui::Dummy(ImVec2(20, 0));
            ImGui::SameLine();
        }
        else
        {
            if (ImGui::BeginMenu("File"))
            {
                if (ImGui::BeginMenu("Open"))
                {
                    ImGui::MenuItem("Calculate_offset", nullptr, &tls_registration.calculate_offset);
                    ImGui::MenuItem("Fill in session", nullptr, &fillInSession);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Fill in data for trajectory and pose to create complete session");

                    ImGui::Separator();

                    if (ImGui::MenuItem("Open session"))
                        openSession();
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Replace the current session with another one (Ctrl+O)");

                    if (ImGui::MenuItem("Open las/laz"))
                        openLaz(fillInSession);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Replace the current session with one created from las/laz file(s)");

                    if (ImGui::MenuItem("Open e57"))
                        openE57(fillInSession);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Replace the current session with one created from e57 file(s); "
                            "embedded per-scan poses are used as initial poses");

                    ImGui::EndMenu();
                }

                {
                    bool has_e57 = false;
                    for (const auto& pc : session.point_clouds_container.point_clouds)
                    {
                        if (!pc.e57_source_path.empty())
                        {
                            has_e57 = true;
                            break;
                        }
                    }
                    ImGui::BeginDisabled(!has_e57);
                    if (ImGui::MenuItem("Update e57 poses"))
                        updateE57Poses();
                    ImGui::EndDisabled();
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Write the current registered poses back into the Data3D headers "
                            "of the source e57 file(s)");
                }

                ImGui::Separator();

                if (ImGui::MenuItem("Save session as", "Ctrl+S"))
                    saveSession();
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Save changes of full session with posibility to change filename");

                // ImGui::BeginDisabled(!((index_begin > 0) || (index_end <
                // static_cast<int>(session.point_clouds_container.point_clouds.size() - 1))));
                //{
                if (ImGui::MenuItem("Save subsession", "Ctrl+Shift+S"))
                    saveSubsession();
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Save session according to selections from 'LIO segments editor' window");
                //}
                // ImGui::EndDisabled();

                if (ImGui::MenuItem("Save session as e57"))
                    saveSessionAsE57();
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        "Write every point cloud of the session to one multi-scan .e57 file, "
                        "each scan carrying its current (registered) pose");

                ImGui::Separator();
                if (ImGui::BeginMenu("Save all marked scans"))
                {
                    static bool skip_ts_0 = true;
                    ImGui::Checkbox("Skip points with zero ts", &skip_ts_0);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Invalid points with zero timestamp will be skipped during export");

                    if (ImGui::MenuItem("Local scan"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                        spdlog::info("laz file to save: '{}'", output_file_name);

                        if (output_file_name.size() > 0)
                            save_all_to_las(session, output_file_name, true, skip_ts_0);
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("As one local scan transformed via inverse pose of first scan");

                    if (ImGui::MenuItem("Global scan"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                        spdlog::info("laz file to save: '{}'", output_file_name);

                        if (output_file_name.size() > 0)
                            save_all_to_las(session, output_file_name, false, skip_ts_0);
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "To export in full resolution, close the program and open again and unmark 'downsample during "
                            "load' before loading session");
                    ImGui::Separator();
                    if (ImGui::MenuItem("Separate global scans (laz)"))
                    {
                        std::string output_folder_name_separately = "";
                        output_folder_name_separately = mandeye::fd::SelectFolder("Choose folder");
                        save_separately_to_las(session, output_folder_name_separately, ".laz");
                    }

                    if (ImGui::MenuItem("Separate global scans (las)"))
                    {
                        std::string output_folder_name_separately = "";
                        output_folder_name_separately = mandeye::fd::SelectFolder("Choose folder");
                        save_separately_to_las(session, output_folder_name_separately, ".las");
                    }

                    ImGui::EndMenu();
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Save all marked scans as las/laz files");

                if (ImGui::BeginMenu("Save all marked trajectories"))
                {
                    ImGui::MenuItem("is_trajectory_export_downsampling", nullptr, &tls_registration.is_trajectory_export_downsampling);
                    ImGui::PushItemWidth(ImGuiNumberWidth);
                    ImGui::InputFloat("curve_consecutive_distance [m]", &tls_registration.curve_consecutive_distance_meters);
                    ImGui::InputFloat("not_curve_consecutive_distance [m]", &tls_registration.not_curve_consecutive_distance_meters);
                    ImGui::PopItemWidth();

                    if (ImGui::MenuItem("Save all as las/laz files"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                        spdlog::info("laz file to save: '{}'", output_file_name);
                        if (output_file_name.size() > 0)
                            save_trajectories_to_laz(
                                session,
                                output_file_name,
                                tls_registration.curve_consecutive_distance_meters,
                                tls_registration.not_curve_consecutive_distance_meters,
                                tls_registration.is_trajectory_export_downsampling);
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("As one global scan");

                    ImGui::Separator();

                    ImGui::Text("(x,y,z,r00,r01,r02,r10,r11,r12,r20,r21,r22)");
                    if (ImGui::MenuItem("Save all as csv (timestamp Lidar)##1"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::Csv_filter, ".csv");
                        spdlog::info("csv file to save: '{}'", output_file_name);

                        if (output_file_name.size() > 0)
                            save_trajectories(
                                session,
                                output_file_name,
                                tls_registration.curve_consecutive_distance_meters,
                                tls_registration.not_curve_consecutive_distance_meters,
                                tls_registration.is_trajectory_export_downsampling,
                                true,
                                false,
                                false,
                                false);
                    }
                    if (ImGui::MenuItem("Save all as csv (timestamp Unix)##1"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::Csv_filter, ".csv");
                        spdlog::info("csv file to save: '{}'", output_file_name);

                        if (output_file_name.size() > 0)
                            save_trajectories(
                                session,
                                output_file_name,
                                tls_registration.curve_consecutive_distance_meters,
                                tls_registration.not_curve_consecutive_distance_meters,
                                tls_registration.is_trajectory_export_downsampling,
                                false,
                                true,
                                false,
                                false);
                    }
                    if (ImGui::MenuItem("Save all as csv (timestamp Lidar, Unix)##1"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::Csv_filter, ".csv");
                        spdlog::info("csv file to save: '{}'", output_file_name);

                        if (output_file_name.size() > 0)
                            save_trajectories(
                                session,
                                output_file_name,
                                tls_registration.curve_consecutive_distance_meters,
                                tls_registration.not_curve_consecutive_distance_meters,
                                tls_registration.is_trajectory_export_downsampling,
                                true,
                                true,
                                false,
                                false);
                    }

                    ImGui::Separator();
                    ImGui::Text("(x,y,z,qx,qy,qz,qw)");

                    if (ImGui::MenuItem("Save all as csv (timestamp Lidar)##2"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::Csv_filter, ".csv");
                        spdlog::info("csv file to save: '{}'", output_file_name);

                        if (output_file_name.size() > 0)
                            save_trajectories(
                                session,
                                output_file_name,
                                tls_registration.curve_consecutive_distance_meters,
                                tls_registration.not_curve_consecutive_distance_meters,
                                tls_registration.is_trajectory_export_downsampling,
                                true,
                                false,
                                true,
                                false);
                    }
                    if (ImGui::MenuItem("Save all as csv (timestamp Unix)##2"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::Csv_filter, ".csv");
                        spdlog::info("csv file to save: '{}'", output_file_name);

                        if (output_file_name.size() > 0)
                            save_trajectories(
                                session,
                                output_file_name,
                                tls_registration.curve_consecutive_distance_meters,
                                tls_registration.not_curve_consecutive_distance_meters,
                                tls_registration.is_trajectory_export_downsampling,
                                false,
                                true,
                                true,
                                false);
                    }
                    if (ImGui::MenuItem("Save all as csv (timestamp Lidar, Unix)##2"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::Csv_filter, ".csv");
                        spdlog::info("csv file to save: '{}'", output_file_name);

                        if (output_file_name.size() > 0)
                            save_trajectories(
                                session,
                                output_file_name,
                                tls_registration.curve_consecutive_distance_meters,
                                tls_registration.not_curve_consecutive_distance_meters,
                                tls_registration.is_trajectory_export_downsampling,
                                true,
                                true,
                                true,
                                false);
                    }

                    ImGui::Separator();

                    if (ImGui::MenuItem("Save all as dxf as polyline"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog("Ouput file name", mandeye::fd::Dxf_filter, ".dxf");
                        spdlog::info("dxf file to save: '{}'", output_file_name);

                        if (output_file_name.size() > 0)
                            save_trajectories(
                                session,
                                output_file_name,
                                tls_registration.curve_consecutive_distance_meters,
                                tls_registration.not_curve_consecutive_distance_meters,
                                tls_registration.is_trajectory_export_downsampling,
                                false,
                                false,
                                false,
                                true);
                    }

                    ImGui::EndMenu();
                }

                if (ImGui::BeginMenu("Save scale board"))
                {
                    ImGui::Text("For all marked trajectories as one global scan to laz");

                    if (ImGui::MenuItem("> dec 0.1"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                        spdlog::info("laz file to save: '{}'", output_file_name);

                        if (output_file_name.size() > 0)
                            save_scale_board_to_laz(session, output_file_name, 0.1);
                    }

                    if (ImGui::MenuItem("> dec 1.0"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                        spdlog::info("laz file to save: '{}'", output_file_name);

                        if (output_file_name.size() > 0)
                            save_scale_board_to_laz(session, output_file_name, 1.0);
                    }

                    if (ImGui::MenuItem("> dec 10.0"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                        spdlog::info("laz file to save: '{}'", output_file_name);

                        if (output_file_name.size() > 0)
                            save_scale_board_to_laz(session, output_file_name, 10.0);
                    }

                    ImGui::Separator();
                    ImGui::Text("10km x 10km to laz");

                    if (ImGui::MenuItem("> 10m"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                        spdlog::info("laz file to save: '{}'", output_file_name);

                        if (output_file_name.size() > 0)
                            save_scale_board_to_laz(session, output_file_name, 10.0, 10000.0);
                    }

                    if (ImGui::MenuItem("> 100m"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                        spdlog::info("laz file to save: '{}'", output_file_name);

                        if (output_file_name.size() > 0)
                            save_scale_board_to_laz(session, output_file_name, 100.0, 10000.0);
                    }

                    if (ImGui::MenuItem("> 1000m"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                        spdlog::info("laz file to save: '{}'", output_file_name);

                        if (output_file_name.size() > 0)
                            save_scale_board_to_laz(session, output_file_name, 1000.0, 10000.0);
                    }

                    ImGui::EndMenu();
                }

                if (ImGui::BeginMenu("GNSS"))
                {
                    ImGui::MenuItem("Load with offset -> move to (0,0,0)", nullptr, &gnssWithOffset);

                    if (ImGui::MenuItem("Load GNSS files and convert WGS84 to PUWG92"))
                    {
                        std::vector<std::string> input_file_names;
                        input_file_names = mandeye::fd::OpenFileDialog("Load gnss files", mandeye::fd::Gnss_filter, true);

                        if (input_file_names.size() > 0)
                        {
                            Eigen::Vector3d out_offset(0.0, 0.0, 0.0);
                            if (!tls_registration.gnss.load_data_from_gnss_and_convert_to_92(input_file_names, out_offset, gnssWithOffset))
                            {
                                spdlog::error("Error loading GNSS files!");
                            }
                            else
                            {
                                session.point_clouds_container.offset_to_apply = out_offset;
                            }
                        }
                    }

                    ImGui::MenuItem("Set WGS84 reference from 1st pose", nullptr, &tls_registration.gnss.setWGS84ReferenceFromFirstPose);

                    ImGui::Text("Load & convert WGS84 to Cartesian by Mercator projection");

                    if (ImGui::MenuItem("Load GNSS (deprecated)"))
                    {
                        std::vector<std::string> input_file_names;
                        input_file_names = mandeye::fd::OpenFileDialog("Load gnss files", mandeye::fd::Gnss_filter, true);

                        if (input_file_names.size() > 0)
                        {
                            if (!tls_registration.gnss.load_raw_data_from_gnss(input_file_names))
                            {
                                spdlog::error("Error loading GNSS files!");
                            }
                            if (!tls_registration.gnss.project_to_mercator_projection())
                            {
                                spdlog::error("Error converting WGS84 to Mercator projection!");
                            }
                        }
                    }
                    if (ImGui::MenuItem("Load GNSS"))
                    {
                        std::vector<std::string> input_file_names;
                        input_file_names = mandeye::fd::OpenFileDialog("Load gnss files", mandeye::fd::Gnss_filter, true);

                        if (input_file_names.size() > 0)
                        {
                            if (!tls_registration.gnss.load_raw_data_from_gnss(input_file_names))
                            {
                                spdlog::error("Error loading GNSS files!");
                            }
                            if (!tls_registration.gnss.project_using_proj())
                            {
                                spdlog::error("Error converting WGS84 to PROJ projection!");
                            }
                        }
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Load structured GNSS dataset and decode it into coordinates, using PROJ library");

                    if (ImGui::MenuItem("Load NMEA (deprecated)"))
                    {
                        std::vector<std::string> input_file_names;
                        input_file_names = mandeye::fd::OpenFileDialog("Load nmea files", mandeye::fd::Nmea_filter, true);

                        if (input_file_names.size() > 0)
                        {
                            if (!tls_registration.gnss.load_raw_data_from_nmea(input_file_names))
                                spdlog::error("Error loading NMEA files!");
                        }
                        if (!tls_registration.gnss.project_to_mercator_projection())
                        {
                            spdlog::error("Error converting WGS84 to Mercator projection!");
                        }
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Load raw GNSS serial output and decode it into coordinates");
                    if (ImGui::MenuItem("Load NMEA"))
                    {
                        std::vector<std::string> input_file_names;
                        input_file_names = mandeye::fd::OpenFileDialog("Load nmea files", mandeye::fd::Nmea_filter, true);

                        if (input_file_names.size() > 0)
                        {
                            if (!tls_registration.gnss.load_raw_data_from_nmea(input_file_names))
                                spdlog::error("Error loading NMEA files!");
                        }
                        if (!tls_registration.gnss.project_using_proj())
                        {
                            spdlog::error("Error converting WGS84 to PROJ projection!");
                        }
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Load raw GNSS serial output and decode it into coordinates, using PROJ library");

                    ImGui::Separator();

                    if (ImGui::MenuItem("Save GNSS data to las/laz file"))
                    {
                        const auto output_file_name =
                            mandeye::fd::SaveFileDialog("Save las or laz file", mandeye::fd::LAS_LAZ_filter, ".laz");
                        spdlog::info("las or laz file to save: '{}'", output_file_name);

                        if (output_file_name.size() > 0)
                            tls_registration.gnss.save_to_laz(
                                output_file_name,
                                session.point_clouds_container.offset.x(),
                                session.point_clouds_container.offset.y(),
                                session.point_clouds_container.offset.z());
                    }
                    const auto prepareVisibleData = [&]()
                    {
                        std::vector<Eigen::Vector3d> pointcloud;
                        std::vector<unsigned short> intensity;
                        std::vector<double> timestamps;

                        for (auto& p : session.point_clouds_container.point_clouds)
                        {
                            if (p.visible)
                            {
                                for (size_t i = 0; i < p.points_local.size(); i++)
                                {
                                    const auto& pp = p.points_local[i];
                                    Eigen::Vector3d vp;
                                    vp = p.m_pose * pp;

                                    pointcloud.push_back(vp);
                                    if (i < p.intensities.size())
                                        intensity.push_back(p.intensities[i]);
                                    else
                                        intensity.push_back(0);
                                    if (i < p.timestamps.size())
                                        timestamps.push_back(p.timestamps[i]);
                                }
                            }
                        }
                        return std::tuple(pointcloud, intensity, timestamps);
                    };

                    if (ImGui::MenuItem("Save metascan points in PUWG92(dep!)"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                        const auto [pointcloud, intensity, timestamps] = prepareVisibleData();

                        const auto lat = tls_registration.gnss.WGS84ReferenceLatitude;
                        const auto lon = tls_registration.gnss.WGS84ReferenceLongitude;
                        const auto alt = tls_registration.gnss.gnss_poses[0].alt;

                        double Xpuwg92 = 0.0;
                        double Ypuwg92 = 0.0;
                        wgs84_do_puwg92(lat, lon, &Xpuwg92, &Ypuwg92);
                        Eigen::Vector3d offset(Ypuwg92, Xpuwg92, alt);
                        exportLaz(output_file_name, pointcloud, intensity, timestamps, offset.x(), offset.y(), offset.z());
                    }
                    ImGui::Separator();
                    for (const auto& geoid : geoids)
                    {
                        if (ImGui::MenuItem(std::string("Set geoid to " + geoid).c_str(), nullptr, selected_geoid_model == geoid))
                        {
                            selected_geoid_model = geoid;
                        }
                    }
                    ImGui::Separator();

                    for (const auto& crtName : CRTs::SupportedCRTs)
                    {
                        std::string itemName = "Save metascan points in " + crtName + " (PROJ)";
                        if (ImGui::MenuItem(itemName.c_str()))
                        {
                            const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");

                            const auto [pointcloud, intensity, timestamps] = prepareVisibleData();
                            const auto lla_points = tls_registration.gnss.unproject_using_proj(pointcloud);
                            auto crt_points = tls_registration.gnss.CRTConvert(lla_points, crtName, selected_geoid_model);
                            Eigen::Vector3d offset = crt_points.front();
                            for (auto& p : crt_points)
                            {
                                p = p - offset;
                            }
                            exportLaz(output_file_name, crt_points, intensity, timestamps, offset.x(), offset.y(), offset.z());
                        }
                    }

                    for (const auto& crtName : CRTs::SupportedCRTs)
                    {
                        std::string itemName = "Save GNSS data to las/laz in " + crtName + " (PROJ)";
                        if (ImGui::MenuItem(itemName.c_str()))
                        {
                            const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");

                            std::vector<Eigen::Vector3d> lla_points;
                            std::vector<unsigned short> intensity;
                            std::vector<double> timestamps;
                            for (const auto& gnss : tls_registration.gnss.gnss_poses)
                            {
                                lla_points.emplace_back(gnss.lat, gnss.lon, gnss.h_wgs84);
                                intensity.push_back(gnss.hdop);
                                timestamps.push_back(gnss.timestamp);
                            }

                            auto crt_points = tls_registration.gnss.CRTConvert(lla_points, crtName, selected_geoid_model);
                            Eigen::Vector3d offset = crt_points.front();
                            for (auto& p : crt_points)
                            {
                                p = p - offset;
                            }
                            exportLaz(output_file_name, crt_points, intensity, timestamps, offset.x(), offset.y(), offset.z());
                        }
                    }

                    ImGui::EndMenu();
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("GNSS (GPS, etc.) related open/save commands");

                if (ImGui::BeginMenu("TUM"))
                {
                    ImGui::MenuItem("Subtract 1st pose transform -> move to (0,0,0)", nullptr, &tumSubtractFirstPose);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Re-express every pose relative to the first one, so the trajectory starts at "
                            "identity (0,0,0, no rotation) instead of the file's raw coordinates");

                    if (ImGui::MenuItem("Load TUM trajectory"))
                    {
                        std::vector<std::string> input_file_names;
                        input_file_names = mandeye::fd::OpenFileDialog("Load TUM trajectory files", mandeye::fd::Tum_filter, true);

                        if (input_file_names.size() > 0)
                        {
                            if (!tls_registration.tum.load_data_from_tum(input_file_names, tumSubtractFirstPose))
                            {
                                spdlog::error("Error loading TUM trajectory files!");
                            }
                            else
                            {
                                spdlog::info(
                                    "point_clouds_container.offset = ({}, {}, {})",
                                    session.point_clouds_container.offset.x(),
                                    session.point_clouds_container.offset.y(),
                                    session.point_clouds_container.offset.z());
                            }
                        }
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Load a trajectory in the TUM RGB-D format (timestamp tx ty tz qx qy qz qw), treated like a GNSS track");

                    ImGui::BeginDisabled(tls_registration.tum.tum_poses.size() == 0);
                    if (ImGui::MenuItem("Center camera on TUM trajectory"))
                    {
                        Eigen::Vector3d centroid(0, 0, 0);
                        for (const auto& p : tls_registration.tum.tum_poses)
                        {
                            centroid += Eigen::Vector3d(p.x, p.y, p.z);
                        }
                        centroid /= static_cast<double>(tls_registration.tum.tum_poses.size());
                        centroid -= session.point_clouds_container.offset;

                        Eigen::Vector3f centroid_f = centroid.cast<float>();
                        app_state.camera.eulerGoal.rotationCenter = Vector3{ centroid_f.x(), centroid_f.y(), centroid_f.z() };
                        app_state.camera.eulerTransitionActive = true;
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Jump the camera to the loaded TUM trajectory -- use this if the trajectory doesn't "
                            "appear where you expect it (e.g. it's far from the loaded point clouds)");
                    ImGui::EndDisabled();

                    ImGui::EndMenu();
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("TUM-format external trajectory open commands");

                ImGui::EndMenu();
            }
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                ImGui::Text("Loaded session:");
                ImGui::Text(std::string(session.session_file_name).c_str());
                ImGui::Separator();
                ImGui::Text("Total number of points: %zu", session_total_number_of_points);

                if (ImGui::BeginTable("Dimensions", 4))
                {
                    ImGui::TableSetupColumn("Coord [m]");
                    ImGui::TableSetupColumn("min");
                    ImGui::TableSetupColumn("max");
                    ImGui::TableSetupColumn("size");
                    ImGui::TableHeadersRow();

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);

                    std::string text = "X";
                    float centered = ImGui::GetColumnWidth() - ImGui::CalcTextSize(text.c_str()).x;
                    // Set cursor so text is centered
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + centered * 0.5f);

                    ImGui::Text("X");

                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%.3f", session_dims.x_min);
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%.3f", session_dims.x_max);
                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("%.3f", session_dims.length);

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + centered * 0.5f);
                    ImGui::Text("Y");

                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%.3f", session_dims.y_min);
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%.3f", session_dims.y_max);
                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("%.3f", session_dims.width);

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + centered * 0.5f);
                    ImGui::Text("Z");

                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%.3f", session_dims.z_min);
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%.3f", session_dims.z_max);
                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("%.3f", session_dims.height);

                    ImGui::EndTable();
                }

                ImGui::EndTooltip();
            }

            if (ImGui::BeginMenu("Tools"))
            {
                ImGui::MenuItem("Point Cloud Alignment", "Ctrl+A", &is_pca_gui);
                if (ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    ImGui::Text("Point cloud alignment (registration) algorithms:");
                    ImGui::Text(
                        "(aligning two 3D point sets from LiDAR scans, by estimating\nthe relative pose(translation and rotation) "
                        "between them)");
                    ImGui::Text("- Normal Distributions Transform");
                    ImGui::Text("- Iterative Closest Point");
                    ImGui::Text("- Registration Plane Feature");
                    ImGui::EndTooltip();
                }
                ImGui::Separator();
                ImGui::MenuItem("Pose Graph SLAM", "Ctrl+P", &is_pose_graph_slam);
                ImGui::MenuItem("Observations", nullptr, &is_manual_analisys);

                ImGui::Separator();

                ImGui::MenuItem("Control Points", "Ctrl+C", &session.control_points.is_imgui, !session.ground_control_points.is_imgui);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Known reference points used to align or verify the scan");

                ImGui::MenuItem(
                    "Ground Control Points", "Ctrl+G", &session.ground_control_points.is_imgui, !session.control_points.is_imgui);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Accurate real-world points used to georeference the scan");

                ImGui::MenuItem("Manual Loop Closure", "Ctrl+L", &is_loop_closure_gui, !is_lio_segments_gui);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Manually connect overlapping scan sections");

                ImGui::Separator();
                ImGui::MenuItem("LIO segments editor", "Ctrl+E", &is_lio_segments_gui, !is_loop_closure_gui);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Manually adjust or review Lidar Inertial Odometry trajectory segments");

                ImGui::Separator();
                ImGui::MenuItem("Translate", nullptr, &is_translate_gui);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Define new coordinate frame from 3 picked points and transform all clouds");

                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Intersections"))
            {
                ImGui::SetNextItemWidth(ImGuiNumberWidth);
                ImGui::InputDouble("Intersection width [m]", &session.point_clouds_container.intersection_width, 0.0, 0.0, "%.2f");
                if (session.point_clouds_container.intersection_width < 0.001)
                    session.point_clouds_container.intersection_width = 0.001;

                ImGui::Separator();

                ImGui::MenuItem("xz_intersection", nullptr, &session.point_clouds_container.xz_intersection);
                ImGui::MenuItem("10m grid##xz", nullptr, &session.point_clouds_container.xz_grid_10x10);
                ImGui::MenuItem("1m grid##xz", nullptr, &session.point_clouds_container.xz_grid_1x1);
                ImGui::MenuItem("0.1m grid##xz", nullptr, &session.point_clouds_container.xz_grid_01x01);

                if (ImGui::MenuItem("Export xz intersection", nullptr, false, session.point_clouds_container.xz_intersection))
                {
                    const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                    spdlog::info("laz file to save: '{}'", output_file_name);

                    if (output_file_name.size() > 0)
                    {
                        save_intersection(
                            session,
                            output_file_name,
                            session.point_clouds_container.xz_intersection,
                            session.point_clouds_container.yz_intersection,
                            session.point_clouds_container.xy_intersection,
                            session.point_clouds_container.intersection_width);
                    }
                }

                ImGui::Separator();

                ImGui::MenuItem("yz_intersection", nullptr, &session.point_clouds_container.yz_intersection);
                ImGui::MenuItem("10m grid##yz", nullptr, &session.point_clouds_container.yz_grid_10x10);
                ImGui::MenuItem("1m grid##yz", nullptr, &session.point_clouds_container.yz_grid_1x1);
                ImGui::MenuItem("0.1m grid##yz", nullptr, &session.point_clouds_container.yz_grid_01x01);

                if (ImGui::MenuItem("Export yz intersection", nullptr, false, session.point_clouds_container.yz_intersection))
                {
                    const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                    spdlog::info("laz file to save: '{}'", output_file_name);

                    if (output_file_name.size() > 0)
                    {
                        save_intersection(
                            session,
                            output_file_name,
                            session.point_clouds_container.xz_intersection,
                            session.point_clouds_container.yz_intersection,
                            session.point_clouds_container.xy_intersection,
                            session.point_clouds_container.intersection_width);
                    }
                }

                ImGui::Separator();

                ImGui::MenuItem("xy_intersection", nullptr, &session.point_clouds_container.xy_intersection);
                ImGui::MenuItem("10m grid##xy", nullptr, &session.point_clouds_container.xy_grid_10x10);
                ImGui::MenuItem("1m grid##xy", nullptr, &session.point_clouds_container.xy_grid_1x1);
                ImGui::MenuItem("0.1m grid##xy", nullptr, &session.point_clouds_container.xy_grid_01x01);

                if (ImGui::MenuItem("Export xy intersection", nullptr, false, session.point_clouds_container.xy_intersection))
                {
                    const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                    spdlog::info("laz file to save: '{}'", output_file_name);

                    if (output_file_name.size() > 0)
                    {
                        save_intersection(
                            session,
                            output_file_name,
                            session.point_clouds_container.xz_intersection,
                            session.point_clouds_container.yz_intersection,
                            session.point_clouds_container.xy_intersection,
                            session.point_clouds_container.intersection_width);
                    }
                }

                ImGui::EndMenu();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Intersection menu");
        }

        if (ImGui::BeginMenu("View"))
        {
            ImGui::BeginDisabled(!session_loaded);
            {
                if (ImGui::BeginMenu("Point cloud"))
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
                        for (auto& point_cloud : session.point_clouds_container.point_clouds)
                            point_cloud.point_size = app_state.point_size;

                    ImGui::Separator();

                    ImGui::Text("Color:");

                    float color[3];
                    if (session_loaded)
                    {
                        color[0] = session.point_clouds_container.point_clouds[0].render_color[0];
                        color[1] = session.point_clouds_container.point_clouds[0].render_color[1];
                        color[2] = session.point_clouds_container.point_clouds[0].render_color[2];
                    }

                    if (ImGui::ColorEdit3("", (float*)&color, ImGuiColorEditFlags_NoInputs))
                    {
                        csPointCloud = CS_SOLID;

                        for (auto& pc : session.point_clouds_container.point_clouds)
                        {
                            pc.render_color[0] = color[0];
                            pc.render_color[1] = color[1];
                            pc.render_color[2] = color[2];

                            if (csTrajectory == CS_FOLLOW)
                            {
                                pc.traj_color[0] = pc.render_color[0];
                                pc.traj_color[1] = pc.render_color[1];
                                pc.traj_color[2] = pc.render_color[2];
                            }
                        }
                    }
                    ImGui::SameLine();
                    if (ImGui::MenuItem("> Solid", nullptr, (csPointCloud == CS_SOLID)))
                        csPointCloud = CS_SOLID;

                    if (ImGui::MenuItem("> Random per segment", "Ctrl+R", (csPointCloud == CS_RANDOM)))
                    {
                        csPointCloud = CS_RANDOM;

                        for (auto& pc : session.point_clouds_container.point_clouds)
                        {
                            pc.render_color[0] = float(rand() % 255) / 255.0f;
                            pc.render_color[1] = float(rand() % 255) / 255.0f;
                            pc.render_color[2] = float(rand() % 255) / 255.0f;

                            if (csTrajectory == CS_FOLLOW)
                            {
                                pc.traj_color[0] = pc.render_color[0];
                                pc.traj_color[1] = pc.render_color[1];
                                pc.traj_color[2] = pc.render_color[2];
                            }
                        }
                    }

                    ImGui::Separator();

                    // Gradient modes -- declared in ColorScheme since the
                    // original, but never actually wired to a menu item or
                    // the renderer there; hooked up here to
                    // scan_renderer's per-point jet-colormap shader.
                    if (ImGui::MenuItem("> By intensity (gradient)", nullptr, (csPointCloud == CS_GRAD_INTENS)))
                        csPointCloud = CS_GRAD_INTENS;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Per-point jet colormap from LAS/LAZ intensity");

                    if (ImGui::MenuItem("> By height (gradient)", nullptr, (csPointCloud == CS_GRAD_ELEV)))
                        csPointCloud = CS_GRAD_ELEV;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Per-point jet colormap from world Z, over the session's [z_min, z_max]");

                    if (ImGui::MenuItem("> By distance (gradient)", nullptr, (csPointCloud == CS_GRAD_DIST)))
                        csPointCloud = CS_GRAD_DIST;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Per-point jet colormap from distance to the rotation center");

                    ImGui::EndMenu();
                }

                if (ImGui::BeginMenu("Trajectory"))
                {
                    if (session_loaded)
                    {
                        auto tmp = session.point_clouds_container.point_clouds[0].line_width;

                        ImGui::BeginDisabled(!app_state.glLineWidthSupport);
                        {
                            ImGui::SetNextItemWidth(ImGuiNumberWidth);
                            ImGui::InputInt("Line width", &tmp);
                        }
                        ImGui::EndDisabled();

                        if (tmp < 0)
                            tmp = 0;
                        else if (tmp > 50)
                            tmp = 50;

                        if (tmp != session.point_clouds_container.point_clouds[0].line_width)
                            for (auto& point_cloud : session.point_clouds_container.point_clouds)
                                point_cloud.line_width = tmp;
                    }

                    ImGui::MenuItem("Show IMU to LIO difference", nullptr, &session.point_clouds_container.show_imu_to_lio_diff);

                    ImGui::Separator();

                    ImGui::Text("Color:");

                    float color[3];
                    if (session_loaded)
                    {
                        color[0] = session.point_clouds_container.point_clouds[0].traj_color[0];
                        color[1] = session.point_clouds_container.point_clouds[0].traj_color[1];
                        color[2] = session.point_clouds_container.point_clouds[0].traj_color[2];
                    }

                    if (ImGui::ColorEdit3("", (float*)&color, ImGuiColorEditFlags_NoInputs))
                    {
                        csTrajectory = CS_SOLID;

                        for (auto& pc : session.point_clouds_container.point_clouds)
                        {
                            pc.traj_color[0] = color[0];
                            pc.traj_color[1] = color[1];
                            pc.traj_color[2] = color[2];
                        }
                    }
                    ImGui::SameLine();
                    if (ImGui::MenuItem("> Solid", nullptr, (csTrajectory == CS_SOLID)))
                        csTrajectory = CS_SOLID;

                    if (ImGui::MenuItem("> Random per segment", nullptr, (csTrajectory == CS_RANDOM)))
                    {
                        csTrajectory = CS_RANDOM;

                        for (auto& pc : session.point_clouds_container.point_clouds)
                        {
                            pc.traj_color[0] = float(rand() % 255) / 255.0f;
                            pc.traj_color[1] = float(rand() % 255) / 255.0f;
                            pc.traj_color[2] = float(rand() % 255) / 255.0f;
                        }
                    }

                    if (ImGui::MenuItem("> Follow cloud color", nullptr, (csTrajectory == CS_FOLLOW)))
                    {
                        csTrajectory = CS_FOLLOW;

                        for (auto& pc : session.point_clouds_container.point_clouds)
                        {
                            pc.traj_color[0] = pc.render_color[0];
                            pc.traj_color[1] = pc.render_color[1];
                            pc.traj_color[2] = pc.render_color[2];
                        }
                    }

                    ImGui::EndMenu();
                }

                ImGui::ColorEdit3("Background color", (float*)&app_state.bg_color, ImGuiColorEditFlags_NoInputs);

                ImGui::BeginDisabled(tls_registration.gnss.gnss_poses.size() <= 0);
                {
                    ImGui::MenuItem("Show GNSS correspondences", nullptr, &tls_registration.gnss.show_correspondences);
                }
                ImGui::EndDisabled();

                ImGui::BeginDisabled(tls_registration.tum.tum_poses.size() <= 0);
                {
                    if (ImGui::BeginMenu("TUM GT Trajectory"))
                    {
                        ImGui::MenuItem("Show TUM correspondences", nullptr, &tls_registration.tum.show_correspondences);
                        ImGui::SetNextItemWidth(ImGuiNumberWidth);
                        ImGui::SliderFloat("TUM point size", &tls_registration.tum.point_size, 1.0f, 20.0f);
                        ImGui::EndMenu();
                    }
                }
                ImGui::EndDisabled();

                ImGui::Separator();
            }
            ImGui::EndDisabled();

            if (ImGui::MenuItem("Orthographic", "key O", &app_state.camera.isOrtho))
            {
                if (app_state.camera.isOrtho)
                {
                    app_state.camera.startEulerTransition(
                        0.0f, 0.0f, app_state.camera.euler.translate, app_state.camera.euler.rotationCenter);
                }
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Switch between perspective view (3D) and orthographic view (2D/flat)");

            ImGui::MenuItem("Show axes", "key X", &app_state.show_axes);
            ImGui::MenuItem("Show compass/ruler", "key C", &app_state.compass_ruler);

            ImGui::MenuItem("Lock Z", "Shift + Z", &app_state.camera.lockZ, !app_state.camera.isOrtho);

            ImGui::Separator();

            ImGui::MenuItem("Settings", nullptr, &is_settings_gui);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Show power user settings window with more parameters");

            if (ImGui::BeginMenu("Console"))
            {
#ifdef _WIN32
                if (ImGui::MenuItem("Use Windows console", nullptr, &consWin))
                {
                    if (consWin)
                    {
                        AllocConsole();
                        freopen("CONOUT$", "w", stdout);
                        freopen("CONOUT$", "w", stderr);
                        freopen("CONIN$", "r", stdin);
                    }
                    else
                        FreeConsole();
                }
                if (ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    ImGui::Text("!!! If not used.. !!!");
                    ImGui::Text("- old console output is lost");
                    ImGui::Text("- new console output can only be seen in subwindow");
                    ImGui::Text("- app might run faster");
                    ImGui::EndTooltip();
                }
#endif
                // ImGui::MenuItem("Subwindow", nullptr, &consImGui);
                // if (ImGui::IsItemHovered())
                //    ImGui::SetTooltip("Show/hide console output as GUI window");

                ImGui::EndMenu();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Control console output");

            ImGui::EndMenu();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Scene view relevant parameters");

        camMenu();

        ImGui::BeginDisabled(session.point_clouds_container.point_clouds.size() <= 0);
        {
            ImGui::SameLine();
            ImGui::Dummy(ImVec2(20, 0));
            ImGui::SameLine();

            ImGui::SetNextItemWidth(ImGuiNumberWidth);
            ImGui::InputInt("Points render downsampling", &app_state.viewer_decimate_point_cloud, 2, 10);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("increase for better performance, decrease for rendering more points");
            ImGui::SameLine();

            // fps_avg = fps_avg * 0.7f + ImGui::GetIO().Framerate * 0.3f;  // exponential smoothing

            // double now = ImGui::GetTime();  // ImGui’s built-in timer (in seconds)

            // ImGui::Checkbox("dynamic", &dynamicSubsampling);
            // if (ImGui::IsItemHovered())
            //    ImGui::SetTooltip("automatically control subsampling vs FPS: increase bellow 10, decrease above 60");
            // if (dynamicSubsampling && (fps_avg < 15) && (now - lastAdjustTime > cooldownSeconds))
            //{
            //    app_state.viewer_decimate_point_cloud += 1;
            //    lastAdjustTime = now;
            //}
            // ImGui::SameLine();
            // ImGui::Text("(avg %.1f)", fps_avg);

            if (app_state.viewer_decimate_point_cloud < 1)
                app_state.viewer_decimate_point_cloud = 1;

            ImGui::SameLine();
            // GetFPS()/point-cloud draw-call/vertex count via raylib/ScanRenderer,
            // rather than ImGui's own Framerate tracker -- raylib doesn't
            // expose a general "draw calls" counter (rlgl's own internal one
            // only tracks its immediate-mode batch renderer, not custom
            // glDrawArrays calls like ScanRenderer's), so these are scan_renderer's
            // own per-frame counts of the calls/points it issued in draw().
            ImGui::Text(
                "(%d FPS, %d draw calls, %d vertices)", GetFPS(), scan_renderer.lastDrawCallCount(), scan_renderer.lastVertexCount());
        }
        ImGui::EndDisabled();

        ImGui::SameLine();
        // GL_RENDERER has no raylib wrapper (raylib doesn't expose GPU
        // vendor/renderer strings), so this is the same plain GL query
        // info_window()'s tooltip already uses, just surfaced directly in
        // the bar instead of hidden behind a hover.
        ImGui::TextDisabled("| %s", reinterpret_cast<const char*>(glGetString(GL_RENDERER)));

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

    if (is_settings_gui)
        settings_gui();

    // my_display_code();

    if (is_pca_gui)
        pca_gui();

    if (is_pose_graph_slam)
        pose_graph_slam_gui();

    if (is_manual_analisys)
        observation_picking_gui();

    if (is_loop_closure_gui)
        loop_closure_gui();

    if (is_lio_segments_gui)
        lio_segments_gui();

    if (is_translate_gui)
    {
        translate_gui();
    }
    else if (translate_tool.step != TranslateTool::Step::Idle)
    {
        translate_tool.step = TranslateTool::Step::Idle;
        translate_tool.has_transform = false;
        translate_tool.transform = Eigen::Affine3d::Identity();
        SetMouseCursor(MOUSE_CURSOR_DEFAULT);
    }

    raylib_widgets::showEulerCenterOfRotationWindow(cor_gui, app_state.camera, xText, yText, zText);

    raylib_widgets::ShowInfoWindow(app_state.info_gui, infoLines, appShortcuts, HDMAPPING_VERSION_STRING, __DATE__);

    draw_translate_preview();

    // 3D drawing is done -- switch to the 2D screen-space projection the
    // mini-compass (DrawLineEx/DrawText) and rlImGuiEnd()'s UI render both
    // need (see raylib_widgets::end3DMatrixStack()'s comment).
    raylib_widgets::end3DMatrixStack(io.DisplaySize.x, io.DisplaySize.y);

    if (is_loop_closure_gui)
        renderLoopClosureLabels(session.point_clouds_container);

    if (!session.control_points.is_imgui)
        renderGroundControlPointsLabels(session.ground_control_points, session.point_clouds_container);

    renderControlPointsLabels(session.control_points, session.point_clouds_container);

    if (app_state.compass_ruler)
        drawMiniCompassWithRuler();

    rlImGuiEnd();
}

void draw_translate_preview()
{
    if (translate_tool.step != TranslateTool::Step::PickXAxis && translate_tool.step != TranslateTool::Step::PickYHint)
        return;

    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse)
        return;

    const auto laser_beam = GetLaserBeam((int)io.MousePos.x, (int)io.MousePos.y);
    RegistrationPlaneFeature::Plane pl;
    pl.a = 0;
    pl.b = 0;
    pl.c = 1;
    pl.d = -translate_tool.plane_z;
    Eigen::Vector3d cursor_world = rayIntersection(laser_beam, pl);

    const Eigen::Vector3d O = translate_tool.origin;

    Eigen::Vector3d x_dir;
    if (translate_tool.step == TranslateTool::Step::PickXAxis)
        x_dir = cursor_world - O;
    else
        x_dir = translate_tool.x_point - O;
    x_dir.z() = 0.0;
    if (x_dir.norm() < 1e-9)
        return;
    double x_len = x_dir.norm();
    Eigen::Vector3d x_n = x_dir / x_len;
    Eigen::Vector3d y_n(-x_n.y(), x_n.x(), 0.0);

    if (translate_tool.step == TranslateTool::Step::PickYHint)
    {
        Eigen::Vector3d v = cursor_world - O;
        if (v.dot(y_n) < 0.0)
            y_n = -y_n;
    }

    double y_len = x_len * 0.5;
    double z_len = x_len * 0.25;

    rlSetLineWidth(3.0f);
    rlBegin(RL_LINES);
    rlColor3f(1.0f, 0.0f, 0.0f);
    rlVertex3f(O.x(), O.y(), O.z());
    rlVertex3f(O.x() + x_n.x() * x_len, O.y() + x_n.y() * x_len, O.z());

    rlColor3f(0.0f, 1.0f, 0.0f);
    rlVertex3f(O.x(), O.y(), O.z());
    rlVertex3f(O.x() + y_n.x() * y_len, O.y() + y_n.y() * y_len, O.z());

    rlColor3f(0.0f, 0.0f, 1.0f);
    rlVertex3f(O.x(), O.y(), O.z());
    rlVertex3f(O.x(), O.y(), O.z() + z_len);
    rlEnd();
    rlSetLineWidth(1.0f);
}

Eigen::Affine3d compute_translate_matrix(const Eigen::Vector3d& O, const Eigen::Vector3d& X, const Eigen::Vector3d& Y_hint)
{
    Eigen::Vector3d dx = X - O;
    dx.z() = 0.0;
    if (dx.norm() < 1e-9)
        return Eigen::Affine3d::Identity();
    dx.normalize();

    double theta = -std::atan2(dx.y(), dx.x());

    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.prerotate(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
    T.pretranslate(-(T.linear() * O));

    Eigen::Vector3d y_h_new = T * Y_hint;
    if (y_h_new.y() < 0.0)
    {
        Eigen::Affine3d flip = Eigen::Affine3d::Identity();
        Eigen::Matrix3d R;
        R << 1, 0, 0, 0, -1, 0, 0, 0, -1;
        flip.linear() = R;
        T = flip * T;
    }
    return T;
}

void translate_gui()
{
    ImGui::Begin("Translate", &is_translate_gui);

    ImGui::PushItemWidth(ImGuiNumberWidth);
    ImGui::InputFloat("Plane Z [m]", &translate_tool.plane_z);
    ImGui::PopItemWidth();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Height of the horizontal pick plane used to project mouse clicks into 3D");

    if (ImGui::Button("Start picking"))
    {
        translate_tool.step = TranslateTool::Step::PickOrigin;
        translate_tool.has_transform = false;
        translate_tool.transform = Eigen::Affine3d::Identity();

        app_state.camera.isOrtho = true;
        app_state.camera.startEulerTransition(0.0f, 0.0f, app_state.camera.euler.translate, app_state.camera.euler.rotationCenter);

        SetMouseCursor(MOUSE_CURSOR_CROSSHAIR);
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset"))
    {
        translate_tool.step = TranslateTool::Step::Idle;
        translate_tool.has_transform = false;
        translate_tool.transform = Eigen::Affine3d::Identity();
        SetMouseCursor(MOUSE_CURSOR_DEFAULT);
    }

    const char* step_text = "Idle";
    switch (translate_tool.step)
    {
    case TranslateTool::Step::PickOrigin:
        step_text = "Pick origin (new 0,0)";
        break;
    case TranslateTool::Step::PickXAxis:
        step_text = "Pick point on +X axis";
        break;
    case TranslateTool::Step::PickYHint:
        step_text = "Pick point on +Y side";
        break;
    case TranslateTool::Step::Ready:
        step_text = "Ready - press Translate";
        break;
    default:
        break;
    }
    ImGui::Text("Step: %s", step_text);

    ImGui::Text("Origin : %.3f %.3f %.3f", translate_tool.origin.x(), translate_tool.origin.y(), translate_tool.origin.z());
    ImGui::Text("X point: %.3f %.3f %.3f", translate_tool.x_point.x(), translate_tool.x_point.y(), translate_tool.x_point.z());
    ImGui::Text("Y hint : %.3f %.3f %.3f", translate_tool.y_hint.x(), translate_tool.y_hint.y(), translate_tool.y_hint.z());

    if (translate_tool.has_transform)
    {
        ImGui::Separator();
        ImGui::Text("Transformation matrix:");
        const Eigen::Matrix4d M = translate_tool.transform.matrix();
        for (int r = 0; r < 4; r++)
            ImGui::Text("%8.4f %8.4f %8.4f %8.4f", M(r, 0), M(r, 1), M(r, 2), M(r, 3));
    }

    ImGui::Separator();
    ImGui::BeginDisabled(!translate_tool.has_transform);
    {
        if (ImGui::Button("Translate"))
        {
            for (auto& pc : session.point_clouds_container.point_clouds)
                pc.m_pose = translate_tool.transform * pc.m_pose;

            for (auto& cp : session.control_points.cps)
            {
                Eigen::Vector3d g(cp.x_target_global, cp.y_target_global, cp.z_target_global);
                g = translate_tool.transform * g;
                cp.x_target_global = g.x();
                cp.y_target_global = g.y();
                cp.z_target_global = g.z();
            }

            translate_tool.step = TranslateTool::Step::Idle;
            translate_tool.has_transform = false;
            translate_tool.transform = Eigen::Affine3d::Identity();
            SetMouseCursor(MOUSE_CURSOR_DEFAULT);
        }
    }
    ImGui::EndDisabled();

    ImGui::End();
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

    spdlog::info("intersection: {}, {}, {}", pos.x(), pos.y(), pos.z());

    return pos;
}

// Button/state constants formerly from <GL/freeglut.h> (matching GLUT's own
// values), so mouse()'s body below -- ported verbatim from GLUT's
// glutMouseFunc callback shape -- needed no changes. Called manually from
// the main loop on raylib button-state transitions (see main() below)
// instead of via glutMouseFunc registration.
constexpr int GLUT_LEFT_BUTTON = 0;
constexpr int GLUT_MIDDLE_BUTTON = 1;
constexpr int GLUT_RIGHT_BUTTON = 2;
constexpr int GLUT_DOWN = 0;
constexpr int GLUT_UP = 1;

void mouse(int glut_button, int state, int x, int y)
{
    ImGuiIO& io = ImGui::GetIO();

    // The GLUT-version-gated legacy mouse-wheel-as-button-3/4 fallback is
    // dropped -- raylib's GetMouseWheelMove() (polled in main()'s loop,
    // calling wheel() directly) covers this unconditionally.

    if (!io.WantCaptureMouse)
    {
        if (glut_button == GLUT_LEFT_BUTTON && state == GLUT_DOWN && !io.KeyCtrl && !io.KeyShift &&
            translate_tool.step != TranslateTool::Step::Idle && translate_tool.step != TranslateTool::Step::Ready)
        {
            const auto laser_beam = GetLaserBeam(x, y);
            RegistrationPlaneFeature::Plane pl;
            pl.a = 0;
            pl.b = 0;
            pl.c = 1;
            pl.d = -translate_tool.plane_z;
            Eigen::Vector3d p = rayIntersection(laser_beam, pl);

            switch (translate_tool.step)
            {
            case TranslateTool::Step::PickOrigin:
                translate_tool.origin = p;
                translate_tool.step = TranslateTool::Step::PickXAxis;
                break;
            case TranslateTool::Step::PickXAxis:
                translate_tool.x_point = p;
                translate_tool.step = TranslateTool::Step::PickYHint;
                break;
            case TranslateTool::Step::PickYHint:
                translate_tool.y_hint = p;
                translate_tool.transform = compute_translate_matrix(translate_tool.origin, translate_tool.x_point, translate_tool.y_hint);
                translate_tool.has_transform = true;
                translate_tool.step = TranslateTool::Step::Ready;
                break;
            default:
                break;
            }

            app_state.mouse_old_x = x;
            app_state.mouse_old_y = y;
            return;
        }

        if ((glut_button == GLUT_MIDDLE_BUTTON || glut_button == GLUT_LEFT_BUTTON) && state == GLUT_DOWN && (io.KeyCtrl || io.KeyShift))
        {
            if (session.ground_control_points.is_imgui)
            {
                spdlog::info("GCP picking");
                int tmp;
                getClosestTrajectoryPoint(session, x, y, true, tmp);
            }
            else if (session.control_points.is_imgui)
            {
                spdlog::info("Control point picking");
                const auto laser_beam = GetLaserBeam(x, y);
                double min_distance = std::numeric_limits<double>::max();

                session.control_points.index_picked_point = -1;

                int i = session.control_points.index_pose;
                if (session.control_points.index_pose >= 0 &&
                    session.control_points.index_pose < session.point_clouds_container.point_clouds.size())
                {
                    Vector3 center = app_state.camera.eulerGoal.rotationCenter;

                    for (size_t j = 0; j < session.point_clouds_container.point_clouds[i].points_local.size(); j++)
                    {
                        const auto& p = session.point_clouds_container.point_clouds[i].points_local[j];
                        Eigen::Vector3d vp = session.point_clouds_container.point_clouds[i].m_pose * p;

                        double dist = distance_point_to_line(vp, laser_beam);

                        if (dist < min_distance)
                        {
                            min_distance = dist;

                            center = Vector3{ static_cast<float>(vp.x()), static_cast<float>(vp.y()), static_cast<float>(vp.z()) };

                            session.control_points.index_picked_point = j;
                        }
                    }

                    app_state.camera.moveEulerRotationCenterTo(center);
                }
            }
            else
            {
                if (glut_button == GLUT_MIDDLE_BUTTON)
                    if (session_loaded)
                    {
                        int tmp = -1;
                        getClosestTrajectoryPoint(session, x, y, false, tmp);

                        if (io.KeyCtrl)
                        {
                            if (tmp != -1)
                                index_loop_closure_target = tmp;
                        }
                        else if (io.KeyShift)
                        {
                            if (tmp != -1)
                                index_loop_closure_source = tmp;
                        }
                    }
                    else
                        setNewRotationCenter(x, y);
            }
        }

        if (glut_button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN && io.KeyCtrl)
        {
            int tmp;
            if (session_loaded)
                getClosestTrajectoryPoint(session, x, y, false, tmp);
            else
                setNewRotationCenter(x, y);
        }

        if (state == GLUT_DOWN)
        {
            app_state.mouse_buttons |= 1 << glut_button;

            if (observation_picking.is_observation_picking_mode)
            {
                Eigen::Vector3d p = GLWidgetGetOGLPos(x, y, observation_picking);
                int number_active_pcs = 0;
                int index_picked = -1;
                for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
                {
                    if (session.point_clouds_container.point_clouds[i].visible)
                    {
                        number_active_pcs++;
                        index_picked = i;
                    }
                }
                if (number_active_pcs == 1)
                    observation_picking.add_picked_to_current_observation(index_picked, p);
            }
        }
        else if (state == GLUT_UP)
            app_state.mouse_buttons = 0;

        app_state.mouse_old_x = x;
        app_state.mouse_old_y = y;
    }
}

// Was glutInit/glutInitDisplayMode/glutInitWindowSize/glutCreateWindow +
// ImGui_ImplGLUT_Init/ImGui_ImplOpenGL2_Init + glutDisplayFunc/glutMouseFunc/
// glutMotionFunc/glutMouseWheelFunc/glutKeyboardFunc/glutKeyboardUpFunc --
// rewritten with raylib's InitWindow + rlImGuiSetup. Kept as a same-named,
// same-signature function (called the same way from main() below) even
// though the display/mouse function pointers are no longer registered as
// GLUT callbacks -- main()'s own loop calls display()/mouse() directly
// instead (see below), and rlImGuiSetup()/raylib's input polling already
// cover what keyboardDown/keyboardUp/motion/wheel used to need GLUT
// callback registration for.
bool initGL(int* argc, char** argv, const std::string& winTitleArg, void (*)(), void (*)(int, int, int, int))
{
    (void)argc;
    (void)argv;

    // FLAG_WINDOW_HIGHDPI on Windows causes issues with the ImGui menu bar and other UI elements being scaled incorrectly, so it's only
    // enabled on macOS and Linux. On Windows, raylib's default DPI scaling is used instead.

    unsigned int flags = FLAG_WINDOW_RESIZABLE;
#ifdef __APPLE__
    flags |= FLAG_WINDOW_HIGHDPI;
#endif //__APPLE__
#if __LINUX__
    flags |= FLAG_WINDOW_HIGHDPI;
#endif // __LINUX__

    SetConfigFlags(flags);
    InitWindow(static_cast<int>(window_width), static_cast<int>(window_height), winTitleArg.c_str());
    // raylib's default exit key (Esc) closes the window outright -- too easy
    // to hit by accident while e.g. cancelling a dialog or backing out of a
    // gizmo drag. Disabled; there's no keyboard shortcut for quitting.
    SetExitKey(KEY_NULL);
    SetTargetFPS(60);

    // The hardcoded window_width/window_height default (1600x900) can be
    // wider and/or taller than the actual screen (e.g. a 13" MacBook's
    // 1470x956-point default-scaled display) -- when it doesn't fit, macOS
    // shifts the window up/left to keep it on screen, which can tuck the
    // very top of the content area (where ImGui's main menu bar lives)
    // behind the OS menu bar/title bar instead of below it. Shrinking to
    // fit the monitor's work area and recentering avoids that; on screens
    // that already fit the default size this is a no-op. DPI-aware and
    // shared with the camera_lidar_* apps -- see raylib_widgets.
    raylib_widgets::fitWindowToScreen(/*marginW=*/100, /*marginH=*/100, /*centerVertically=*/true);

    rlImGuiSetup(true);
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard | ImGuiConfigFlags_NavEnableGamepad | ImGuiConfigFlags_DockingEnable;
    io.ConfigDockingWithShift = true;

    scan_renderer.init();

    app_state.camera.applyPerspectiveProjection(static_cast<int>(window_width), static_cast<int>(window_height));

    return true;
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
                      << "   <input_file>         Path to Mandeye JSON Session file (*.mjs)\n"
                      << "   -h, /h, --help, /?   Show this help and exit\n\n";

            return 0;
        }

        // search for available geoid models in the system and populate the menu
        geoids = GNSS::get_available_geoids();

        initGL(&argc, argv, winTitle, display, mouse);

        if (argc > 1)
        {
            for (int i = 1; i < argc; i++)
            {
                std::string ext = fs::path(argv[i]).extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

                if (ext == ".mjs" || ext == ".json")
                {
                    loadSessionFromPath(argv[i]);
                    break;
                }
            }
        }

        // Was glutMainLoop() -- which repeatedly invoked the registered
        // display/mouse/motion/wheel/keyboard callbacks. Those are called
        // directly here instead: mouse() on raylib button-state transitions
        // (mirroring glutMouseFunc's fire-on-transition semantics), motion()
        // every frame (mirroring glutMotionFunc -- motion() itself only acts
        // when app_state.mouse_buttons is set, so this is safe unconditionally), wheel()
        // when GetMouseWheelMove() is nonzero, and display() once per frame.
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

            // Drag & drop a session file (*.mjs/*.json) onto the window to load it. raylib's GLFW backend
            // surfaces OS drag & drop the same way on Windows, Linux and macOS, so no platform-specific code is
            // needed here. Only the first dropped path is used; loadSessionFromPath() reports unsupported drops
            // via a message box instead of silently ignoring them.
            if (IsFileDropped())
            {
                FilePathList dropped_files = LoadDroppedFiles();
                if (dropped_files.count > 0)
                {
                    loadSessionFromPath(dropped_files.paths[0]);
                }
                UnloadDroppedFiles(dropped_files);
            }

            BeginDrawing();
            display();
            EndDrawing();
        }

        rlImGuiShutdown();
        CloseWindow();
    } catch (const std::bad_alloc& e)
    {
        spdlog::error("System is out of memory : {}", e.what());
        mandeye::fd::OutOfMemMessage();
    } catch (const std::exception& e)
    {
        spdlog::error(e.what());
    } catch (...)
    {
        spdlog::error("Unknown fatal error occurred!");
    }

    return 0;
}