#include "rl_utils.h"

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"
#include "external/glad.h"

#include <imgui_internal.h>

#include <Core/transformations.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <HDMapping/Version.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <limits>

#ifdef _WIN32
#include <shellapi.h>
#include <windows.h>
#endif

///////////////////////////////////////////////////////////////////////////////////
// Formerly <Core/utils.hpp>'s extern globals -- defined here now (see
// rl_utils.h's top comment for why).
///////////////////////////////////////////////////////////////////////////////////

int viewer_decimate_point_cloud = 2;

int mouse_old_x, mouse_old_y;
int mouse_buttons = 0;
float mouse_sensitivity = 1.0;

bool is_ortho = false;
bool lock_z = false;
bool show_axes = true;
ImVec4 bg_color = ImVec4(0.65f, 0.65f, 0.65f, 1.00f);
int point_size = 1;

bool info_gui = false;
bool compass_ruler = true;

Eigen::Affine3f viewLocal;

Eigen::Vector3f rotation_center = Eigen::Vector3f::Zero();
float rotate_x = -35.264f, rotate_y = 135.0f;
float translate_x, translate_y = 0.0;
float translate_z = -50.0;

double camera_ortho_xy_view_zoom = 10;
double camera_ortho_xy_view_shift_x = 0.0;
double camera_ortho_xy_view_shift_y = 0.0;
double camera_mode_ortho_z_center_h = 0.0;

// Target camera state for smooth transitions
Eigen::Vector3f new_rotation_center = rotation_center;
float new_rotate_x = rotate_x;
float new_rotate_y = rotate_y;
float new_translate_x = translate_x;
float new_translate_y = translate_y;
float new_translate_z = translate_z;

bool cor_gui = false;

// Transition timing
bool camera_transition_active = false;

bool scroll_hint_enabled = true;
bool scroll_hint_active = false;
int scroll_hint_count = 0;
float scroll_hint_accu = 0.0f;
double scroll_hint_lastT = 0.0;

bool show_about = false;

bool glLineWidthSupport = true;

float m_ortho_projection[] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
float m_ortho_gizmo_view[] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

// ============================================================================
// Formerly core/src/utils.cpp -- local now (see the big comment at the top
// of this file for why). Everywhere the original was pure ImGui/Eigen/GLM
// (no gl*/glu*/glut* calls), it's copied verbatim. Everywhere it touched
// legacy GL, it's reimplemented with rlgl's rl*() legacy-emulation API
// (a software matrix stack + immediate-mode layer that mirrors gl*()'s
// call shape but works under a core-profile context), or with raylib/
// raymath equivalents (gluUnProject -> Vector3Unproject, glutBitmapCharacter
// -> DrawText). Function names/signatures/globals are unchanged so every
// call site elsewhere in this file (display(), mouse(), the panel
// functions, ...) needed no changes.
// ============================================================================

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
        if (dir > 0)
        {
            if (is_ortho)
            {
                camera_ortho_xy_view_zoom -= 0.1f * camera_ortho_xy_view_zoom;

                if (camera_ortho_xy_view_zoom < 0.1)
                {
                    camera_ortho_xy_view_zoom = 0.1;
                }
            }
            else
            {
                if (io.KeyShift)
                    translate_z += 5.0f;
                else
                    translate_z += 1.0f;
            }
        }
        else
        {
            if (is_ortho)
                camera_ortho_xy_view_zoom += 0.1 * camera_ortho_xy_view_zoom;
            else
            {
                if (io.KeyShift)
                    translate_z -= 5.0f;
                else
                    translate_z -= 1.0f;
            }
        }

        mouse_sensitivity = fabs(translate_z) / 100; // 1 for translate_z 50 (default zoom)
        camera_transition_active = false;

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

// Was glMatrixMode/glLoadIdentity/gluPerspective/glOrtho -- rewritten with
// rlgl's software matrix-stack API (RL_PROJECTION/RL_MODELVIEW), which
// works under raylib's core-profile context. gluPerspective(fovy, aspect,
// near, far) has no rl* equivalent, so it's expanded to the equivalent
// rlFrustum() call by hand (standard fovy -> frustum-bounds formula).
void reshape(int w, int h)
{
    rlViewport(0, 0, (int)w, (int)h);
    rlMatrixMode(RL_PROJECTION);
    rlLoadIdentity();
    if (!is_ortho)
    {
        const double fovy = 60.0;
        const double aspect = (double)w / (double)h;
        const double nearP = 0.01, farP = 10000.0;
        const double top = nearP * tan(fovy * 0.5 * M_PI / 180.0);
        const double bottom = -top;
        const double right = top * aspect;
        const double left = -right;
        rlFrustum(left, right, bottom, top, nearP, farP);
    }
    else
    {
        ImGuiIO& io = ImGui::GetIO();
        float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);

        rlOrtho(
            -camera_ortho_xy_view_zoom,
            camera_ortho_xy_view_zoom,
            -camera_ortho_xy_view_zoom / ratio,
            camera_ortho_xy_view_zoom / ratio,
            -100000,
            100000);
    }
    rlMatrixMode(RL_MODELVIEW);
    rlLoadIdentity();
}

// GL-free -- copied verbatim, minus the trailing glutPostRedisplay() (a
// no-op here: this app's main loop already redraws every frame).
void motion(int x, int y)
{
    ImGuiIO& io = ImGui::GetIO();
    io.MousePos = ImVec2((float)x, (float)y);

    if (!io.WantCaptureMouse)
    {
        float dx, dy;
        dx = (float)(x - mouse_old_x);
        dy = (float)(y - mouse_old_y);

        if (mouse_buttons & 1) // left button
        {
            rotate_x += dy * 0.2f;
            rotate_y += dx * 0.2f;
            breakCameraTransition();
        }

        if (mouse_buttons & 4) // right button
        {
            if (is_ortho)
            {
                float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);
                Eigen::Vector3d v(
                    dx * (camera_ortho_xy_view_zoom / (float)io.DisplaySize.x * 2),
                    dy * (camera_ortho_xy_view_zoom / (float)io.DisplaySize.y * 2 / ratio),
                    0);
                TaitBryanPose pose_tb;
                pose_tb.px = 0.0;
                pose_tb.py = 0.0;
                pose_tb.pz = 0.0;
                pose_tb.om = 0.0;
                pose_tb.fi = 0.0;
                pose_tb.ka = (rotate_x + rotate_y) * M_PI / 180.0;
                auto m = affine_matrix_from_pose_tait_bryan(pose_tb);
                Eigen::Vector3d v_t = m * v;
                camera_ortho_xy_view_shift_x += v_t.x();
                camera_ortho_xy_view_shift_y += v_t.y();
            }
            else
            {
                translate_x += dx * 0.1f * mouse_sensitivity;
                translate_y -= dy * 0.1f * mouse_sensitivity;
                breakCameraTransition();
            }
        }

        mouse_old_x = x;
        mouse_old_y = y;
    }
}

// GL-free -- copied verbatim.
static bool first_time = true;

void ShowMainDockSpace()
{
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
        ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoInputs;

    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::SetNextWindowViewport(viewport->ID);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);

    ImGui::Begin("MainDockSpace", nullptr, window_flags);

    ImGui::PopStyleVar(2);

    // This is the dockspace!
    ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
    ImGui::DockSpace(dockspace_id, ImVec2(0, 0), ImGuiDockNodeFlags_PassthruCentralNode | ImGuiDockNodeFlags_NoDockingInCentralNode);

    if (first_time)
    {
        first_time = false;

        auto dock_id_left = ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Left, 0.2f, nullptr, &dockspace_id);
        auto dock_id_bottom = ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Down, 0.2f, nullptr, &dockspace_id);

        ImGui::DockBuilderDockWindow("Console", dock_id_bottom);
        ImGui::DockBuilderFinish(dockspace_id);
    }

    ImGui::End();
}

// Was glBegin(GL_LINES)/glColor3f/glVertex3f/glEnd -- rl* rename.
void showAxes()
{
    if (show_axes || ImGui::GetIO().KeyCtrl) // rotation center axes
    {
        rlBegin(RL_LINES);
        rlColor3f(1.f, 1.f, 1.f);
        rlVertex3f(rotation_center.x(), rotation_center.y(), rotation_center.z());
        rlVertex3f(rotation_center.x() + 1.f, rotation_center.y(), rotation_center.z());
        rlVertex3f(rotation_center.x(), rotation_center.y(), rotation_center.z());
        rlVertex3f(rotation_center.x() - 1.f, rotation_center.y(), rotation_center.z());
        rlVertex3f(rotation_center.x(), rotation_center.y(), rotation_center.z());
        rlVertex3f(rotation_center.x(), rotation_center.y() - 1.f, rotation_center.z());
        rlVertex3f(rotation_center.x(), rotation_center.y(), rotation_center.z());
        rlVertex3f(rotation_center.x(), rotation_center.y() + 1.f, rotation_center.z());
        rlVertex3f(rotation_center.x(), rotation_center.y(), rotation_center.z());
        rlVertex3f(rotation_center.x(), rotation_center.y(), rotation_center.z() - 1.f);
        rlVertex3f(rotation_center.x(), rotation_center.y(), rotation_center.z());
        rlVertex3f(rotation_center.x(), rotation_center.y(), rotation_center.z() + 1.f);
        rlEnd();
    }

    if (show_axes || ImGui::GetIO().KeyCtrl) // origin axes
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

// GL-free -- copied verbatim.
void updateCameraTransition()
{
    if (!camera_transition_active)
        return;

    float t = 1.0f - powf(1.0f - std::min(ImGui::GetIO().DeltaTime * camera_transition_speed, 1.0f), 3.0f);

    bool doneXrc = fabs(new_rotation_center.x() - rotation_center.x()) < 0.01f;
    bool doneYrc = fabs(new_rotation_center.y() - rotation_center.y()) < 0.01f;
    bool doneZrc = fabs(new_rotation_center.z() - rotation_center.z()) < 0.01f;
    bool doneXr = fabs(new_rotate_x - rotate_x) < 0.01f;
    bool doneYr = fabs(new_rotate_y - rotate_y) < 0.01f;
    bool doneXt = fabs(new_translate_x - translate_x) < 0.01f;
    bool doneYt = fabs(new_translate_y - translate_y) < 0.01f;
    bool doneZt = fabs(new_translate_z - translate_z) < 0.01f;

    if (!doneXrc)
        rotation_center.x() += (new_rotation_center.x() - rotation_center.x()) * t;
    if (!doneYrc)
        rotation_center.y() += (new_rotation_center.y() - rotation_center.y()) * t;
    if (!doneZrc)
        rotation_center.z() += (new_rotation_center.z() - rotation_center.z()) * t;
    if (!doneXr)
        rotate_x += (new_rotate_x - rotate_x) * t;
    if (!doneYr)
        rotate_y += (new_rotate_y - rotate_y) * t;
    if (!doneXt)
        translate_x += (new_translate_x - translate_x) * t;
    if (!doneYt)
        translate_y += (new_translate_y - translate_y) * t;
    if (!doneZt)
        translate_z += (new_translate_z - translate_z) * t;

    camera_transition_active = !(doneXrc && doneYrc && doneZrc && doneXr && doneYr && doneXt && doneYt && doneZt);

    if (!camera_transition_active)
    {
        rotation_center = new_rotation_center;
        rotate_x = new_rotate_x;
        rotate_y = new_rotate_y;
        translate_x = new_translate_x;
        translate_y = new_translate_y;
        translate_z = new_translate_z;
    }
}

// GL-free -- copied verbatim.
void breakCameraTransition()
{
    if (camera_transition_active == false)
        return;
    rotation_center = new_rotation_center;
    camera_transition_active = false;
}

// GL-free -- copied verbatim.
void setCameraPreset(CameraPreset preset)
{
    bool triggered = false;

    switch (preset)
    {
    case CAMERA_FRONT:
        new_rotate_x = -90.0f;
        new_rotate_y = +90.0f;
        triggered = true;
        break;
    case CAMERA_BACK:
        new_rotate_x = -90.0f;
        new_rotate_y = -90.0f;
        triggered = true;
        break;
    case CAMERA_LEFT:
        new_rotate_x = -90.0f;
        new_rotate_y = 180.0f;
        triggered = true;
        break;
    case CAMERA_RIGHT:
        new_rotate_x = -90.0f;
        new_rotate_y = 0.0f;
        triggered = true;
        break;
    case CAMERA_TOP:
        new_rotate_x = 0.0f;
        new_rotate_y = 90.0f;
        triggered = true;
        break;
    case CAMERA_BOTTOM:
        new_rotate_x = 180.0f;
        new_rotate_y = -90.0f;
        triggered = true;
        break;
    case CAMERA_ISO:
        new_rotate_x = -35.264f;
        new_rotate_y = 135.0f;
        triggered = true;
        break;
    case CAMERA_RESET:
        new_rotation_center = Eigen::Vector3f::Zero();
        new_rotate_x = 0;
        new_rotate_y = 0;
        new_translate_x = 0;
        new_translate_y = 0;
        new_translate_z = -50.0f;
        mouse_sensitivity = fabs(translate_z) / 100;

        camera_ortho_xy_view_zoom = 10;
        camera_ortho_xy_view_shift_x = 0.0;
        camera_ortho_xy_view_shift_y = 0.0;
        camera_mode_ortho_z_center_h = 0.0;

        viewer_decimate_point_cloud = 1000;
        triggered = false;
        break;
    }

    if (triggered)
    {
        new_rotation_center = rotation_center;
        new_translate_x = translate_x;
        new_translate_y = translate_y;
        new_translate_z = translate_z;
    }

    camera_transition_active = true;
}

// GL-free -- copied verbatim.
void camMenu()
{
    if (ImGui::BeginMenu("Camera"))
    {
        if (ImGui::MenuItem("Front (yz view)", "key F"))
            setCameraPreset(CAMERA_FRONT);
        if (ImGui::MenuItem("Back", "key B"))
            setCameraPreset(CAMERA_BACK);
        if (ImGui::MenuItem("Left (xz view)", "key L"))
            setCameraPreset(CAMERA_LEFT);
        if (ImGui::MenuItem("Right", "key R"))
            setCameraPreset(CAMERA_RIGHT);
        if (ImGui::MenuItem("Top (xy view)", "key T"))
            setCameraPreset(CAMERA_TOP);
        if (ImGui::MenuItem("Bottom", "key U"))
            setCameraPreset(CAMERA_BOTTOM);
        if (ImGui::MenuItem("Isometric", "key I"))
            setCameraPreset(CAMERA_ISO);
        ImGui::Separator();
        if (ImGui::MenuItem("Reset", "key Z"))
            setCameraPreset(CAMERA_RESET);

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
            ImGui::Text("%.3f", rotate_x);
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%.3f", translate_x);
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%.3f", rotation_center.x());

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + centered * 0.5f);
            ImGui::Text("Y");

            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.3f", rotate_y);
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%.3f", translate_y);
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%.3f", rotation_center.y());

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + centered * 0.5f);
            ImGui::Text("Z");

            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%.3f", translate_z);
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%.3f", rotation_center.y());

            ImGui::EndTable();
        }
        ImGui::Text("Mouse sensitivity: %.4f", mouse_sensitivity);

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

// GL-free -- copied verbatim.
void view_kbd_shortcuts()
{
    ImGuiIO& io = ImGui::GetIO();

    if (io.WantCaptureKeyboard)
        return;

    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_RightArrow, true))
    {
        translate_x += 0.5f * mouse_sensitivity;
        breakCameraTransition();
    }
    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_LeftArrow, true))
    {
        translate_x -= 0.5f * mouse_sensitivity;
        breakCameraTransition();
    }

    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_UpArrow, true))
    {
        translate_y += 0.5f * mouse_sensitivity;
        breakCameraTransition();
    }
    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_DownArrow, true))
    {
        translate_y -= 0.5f * mouse_sensitivity;
        breakCameraTransition();
    }

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_RightArrow, true))
    {
        rotate_y -= 0.6;
        breakCameraTransition();
    }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_LeftArrow, true))
    {
        rotate_y += 0.6;
        breakCameraTransition();
    }

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_UpArrow, true))
    {
        rotate_x -= 0.6;
        breakCameraTransition();
    }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_DownArrow, true))
    {
        rotate_x += 0.6;
        breakCameraTransition();
    }

    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_R, false))
        cor_gui = true;

    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_Z, false) && !is_ortho)
        lock_z = !lock_z;

    if (io.KeyCtrl || io.KeyAlt || io.KeyShift)
        return;

    if (ImGui::IsKeyPressed(ImGuiKey_B))
        setCameraPreset(CAMERA_BACK);
    if (ImGui::IsKeyPressed(ImGuiKey_F))
        setCameraPreset(CAMERA_FRONT);
    if (ImGui::IsKeyPressed(ImGuiKey_I))
        setCameraPreset(CAMERA_ISO);
    if (ImGui::IsKeyPressed(ImGuiKey_L))
        setCameraPreset(CAMERA_LEFT);
    if (ImGui::IsKeyPressed(ImGuiKey_R))
        setCameraPreset(CAMERA_RIGHT);
    if (ImGui::IsKeyPressed(ImGuiKey_T))
        setCameraPreset(CAMERA_TOP);
    if (ImGui::IsKeyPressed(ImGuiKey_U))
        setCameraPreset(CAMERA_BOTTOM);
    if (ImGui::IsKeyPressed(ImGuiKey_Z))
        setCameraPreset(CAMERA_RESET);

    if (ImGui::IsKeyPressed(ImGuiKey_C, false))
        compass_ruler = !compass_ruler;
    if (ImGui::IsKeyPressed(ImGuiKey_O, false))
        is_ortho = !is_ortho;
    if (ImGui::IsKeyPressed(ImGuiKey_X, false))
        show_axes = !show_axes;

    if (ImGui::IsKeyPressed(ImGuiKey_1))
        point_size = 1;
    if (ImGui::IsKeyPressed(ImGuiKey_2))
        point_size = 2;
    if (ImGui::IsKeyPressed(ImGuiKey_3))
        point_size = 3;
    if (ImGui::IsKeyPressed(ImGuiKey_4))
        point_size = 4;
    if (ImGui::IsKeyPressed(ImGuiKey_5))
        point_size = 5;
    if (ImGui::IsKeyPressed(ImGuiKey_6))
        point_size = 6;
    if (ImGui::IsKeyPressed(ImGuiKey_7))
        point_size = 7;
    if (ImGui::IsKeyPressed(ImGuiKey_8))
        point_size = 8;
    if (ImGui::IsKeyPressed(ImGuiKey_9))
        point_size = 9;
}

// GL-free -- copied verbatim.
void cor_window()
{
    if (cor_gui)
    {
        ImGui::OpenPopup("Center of rotation");
        cor_gui = false;
    }

    if (ImGui::BeginPopupModal("Center of rotation", NULL, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::Text("Select new center of rotation [m]:");
        ImGui::PushItemWidth(ImGuiNumberWidth);
        ImGui::InputFloat("X", &new_rotation_center.x(), 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(xText);
        ImGui::SameLine();
        ImGui::InputFloat("Y", &new_rotation_center.y(), 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(yText);
        ImGui::SameLine();
        ImGui::InputFloat("Z", &new_rotation_center.z(), 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(zText);
        ImGui::PopItemWidth();

        ImGui::Separator();

        if (ImGui::Button("Set"))
        {
            new_rotate_x = rotate_x;
            new_rotate_y = rotate_y;
            new_translate_x = -new_rotation_center.x();
            new_translate_y = -new_rotation_center.y();
            new_translate_z = translate_z;

            camera_transition_active = true;

            ImGui::CloseCurrentPopup();
        }

        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
        {
            ImGui::CloseCurrentPopup();
        }

        ImGui::EndPopup();
    }
}

// GL-free -- copied verbatim.
void ImGuiHyperlink(const char* url, ImVec4 color)
{
    ImGui::PushStyleColor(ImGuiCol_Text, color);
    ImGui::TextUnformatted(url);
    ImGui::PopStyleColor();

    ImVec2 pos = ImGui::GetItemRectMin();
    ImVec2 size = ImGui::GetItemRectSize();

    if (ImGui::IsItemHovered())
        ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);

    if (ImGui::IsItemHovered())
    {
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        draw_list->AddLine(ImVec2(pos.x, pos.y + size.y), ImVec2(pos.x + size.x, pos.y + size.y), ImColor(color));
    }

    if (ImGui::IsItemClicked())
    {
#ifdef _WIN32
        ShellExecuteA(0, "open", url, 0, 0, SW_SHOWNORMAL);
#elif __APPLE__
        std::string cmd = std::string("open ") + url;
        system(cmd.c_str());
#else
        std::string cmd = std::string("xdg-open ") + url;
        system(cmd.c_str());
#endif
    }
}

// General shortcuts applicable to any app -- GL-free, copied verbatim.
static const std::vector<ShortcutEntry> shortcuts = { { "Normal keys", "A", "" },
                                                      { "", "Ctrl+A", "" },
                                                      { "", "B", "camera Back" },
                                                      { "", "Ctrl+B", "" },
                                                      { "", "C", "Compass/ruler" },
                                                      { "", "Ctrl+C", "" },
                                                      { "", "D", "" },
                                                      { "", "Ctrl+D", "" },
                                                      { "", "E", "" },
                                                      { "", "Ctrl+E", "" },
                                                      { "", "F", "camera Front" },
                                                      { "", "Ctrl+F", "" },
                                                      { "", "G", "" },
                                                      { "", "Ctrl+G", "" },
                                                      { "", "H", "" },
                                                      { "", "Ctrl+H", "" },
                                                      { "", "I", "camera Isometric" },
                                                      { "", "Ctrl+I", "" },
                                                      { "", "J", "" },
                                                      { "", "Ctrl+J", "" },
                                                      { "", "K", "" },
                                                      { "", "Ctrl+K", "" },
                                                      { "", "L", "camera Left" },
                                                      { "", "Ctrl+L", "" },
                                                      { "", "M", "" },
                                                      { "", "Ctrl+M", "" },
                                                      { "", "N", "" },
                                                      { "", "Ctrl+N", "" },
                                                      { "", "O", "Ortographic view" },
                                                      { "", "Ctrl+O", "Open/load session/data" },
                                                      { "", "P", "" },
                                                      { "", "Ctrl+P", "" },
                                                      { "", "Q", "" },
                                                      { "", "Ctrl+Q", "" },
                                                      { "", "R", "camera Right" },
                                                      { "", "Ctrl+R", "" },
                                                      { "", "Shift+R", "Rotation center" },
                                                      { "", "S", "" },
                                                      { "", "Ctrl+S", "" },
                                                      { "", "Ctrl+Shift+S", "" },
                                                      { "", "T", "camera Top" },
                                                      { "", "Ctrl+T", "" },
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

// GL-free -- copied verbatim.
void ShowShortcutsTable(const std::vector<ShortcutEntry> appShortcuts)
{
    if (ImGui::BeginTable(
            "ShortcutsTable", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY, ImVec2(-FLT_MIN, 200)))
    {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("Shortcut", ImGuiTableColumnFlags_WidthFixed, 120);
        ImGui::TableSetupColumn("Description");
        ImGui::TableHeadersRow();

        std::string lastType;

        for (size_t i = 0; i < shortcuts.size(); ++i)
        {
            const auto& s = shortcuts[i];

            if (!s.type.empty() && s.type != lastType)
            {
                lastType = s.type;
                ImGui::TableNextRow();

                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, IM_COL32(70, 70, 140, 255));

                ImGui::TableSetColumnIndex(0);
                ImGui::TextColored(ImVec4(0.8f, 0.8f, 1.0f, 1.0f), "%s", lastType.c_str());
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted("");
            }

            auto description = s.description;

            if (description.empty())
                description = appShortcuts[i].description;

            if (!description.empty())
            {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(s.shortcut.c_str());
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(description.c_str());
            }
        }

        ImGui::EndTable();
    }
}

// GL-free -- copied verbatim (glGetString(GL_RENDERER/...) is a plain
// string query, still valid under a core-profile context).
void info_window(const std::vector<std::string>& infoLines, const std::vector<ShortcutEntry>& appShortcuts)
{
    if (!info_gui)
        return;

    if (ImGui::Begin(
            "Info",
            &info_gui,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoCollapse))
    {
        bool firstLine = true;
        for (const auto& line : infoLines)
        {
            if (line.empty())
                ImGui::NewLine();
            else if (line.rfind("https://", 0) == 0)
                ImGuiHyperlink(line.c_str());
            else
                ImGui::Text(line.c_str());

            if (firstLine)
            {
                ImGui::SameLine(
                    ImGui::GetWindowWidth() - ImGui::CalcTextSize("ImGui").x - ImGui::GetStyle().ItemSpacing.x * 2 -
                    ImGui::GetStyle().FramePadding.x * 2);
                if (ImGui::Button("ImGui"))
                    show_about = true;
                if (ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    const GLubyte* renderer = glGetString(GL_RENDERER);
                    const GLubyte* version = glGetString(GL_VERSION);
                    const GLubyte* glslVersion = glGetString(GL_SHADING_LANGUAGE_VERSION);

                    ImGui::Text("Renderer: %s", renderer);
                    ImGui::Text("OpenGL version supported: %s", version);
                    ImGui::Text("GLSL version: %s", glslVersion);
                    ImGui::EndTooltip();
                }

                firstLine = false;
            }
        }

        ImGui::NewLine();
        ImGui::Text("Author: Janusz Bedkowski & contributors");
        ImGui::NewLine();
        ImGui::Text("Part of HDMapping software suite");
        ImGui::Text("Version: %s (%s)", HDMAPPING_VERSION_STRING, __DATE__);
        ImGui::Text("Project page: ");
        ImGui::SameLine();
        ImGuiHyperlink("https://github.com/MapsHD/HDMapping");

        ImGui::NewLine();
        ImGui::Separator();
        ImGui::NewLine();

        ShowShortcutsTable(appShortcuts);

        if (show_about)
            ImGui::ShowAboutWindow(&show_about);
    }

    ImGui::End();
}

// Was a dedicated 200x200 GL sub-viewport with its own glOrtho projection,
// rotation-only modelview (viewLocal's rotation), and GLUT bitmap-font text
// (glRasterPos + glutBitmapCharacter). Reimplemented as a pure 2D
// screen-space overlay instead: project each world axis direction through
// viewLocal's rotation to get eye-space X/Y (screen right/up), and draw
// with raylib's DrawLineEx/DrawText -- same bottom-left placement, same
// "nice number" ruler tied to zoom (translate_z), no sub-viewport or GLUT
// font needed.
void drawMiniCompassWithRuler()
{
    const float compassSize = 200.0f;
    const float originX = compassSize * 0.5f;
    const float originY = static_cast<float>(GetScreenHeight()) - compassSize * 0.5f;
    const float axisPixelLength = compassSize * 0.35f;

    struct Axis
    {
        Eigen::Vector3f dir;
        const char* label;
        Color color;
    };
    const Axis axes[3] = {
        {Eigen::Vector3f::UnitX(), "X (long.)", RED},
        {Eigen::Vector3f::UnitY(), "Y (lat.)", GREEN},
        {Eigen::Vector3f::UnitZ(), "Z (vert.)", BLUE},
    };

    for (const auto& axis : axes)
    {
        Eigen::Vector3f eyeDir = viewLocal.rotation() * axis.dir;
        Vector2 tip = {originX + eyeDir.x() * axisPixelLength, originY - eyeDir.y() * axisPixelLength};
        DrawLineEx(Vector2{originX, originY}, tip, 2.f, axis.color);
        DrawText(axis.label, (int)tip.x + 4, (int)tip.y - 6, 12, axis.color);
    }

    // Ruler: "nice" (1/2/5 x 10^n) length, mirroring the original's
    // 0.1 * fabs(translate_z) heuristic (translate_z is this app's
    // zoom/dolly distance).
    float rawUnit = std::max(0.001f, 0.1f * fabsf(translate_z));
    float base = powf(10.0f, floorf(log10f(rawUnit)));
    float normalized = rawUnit / base;
    float niceUnit = normalized < 2.0f ? 1.0f : (normalized < 5.0f ? 2.0f : 5.0f);
    float worldLength = niceUnit * base;

    char label[32];
    if (worldLength >= 1000.0f)
        snprintf(label, sizeof(label), "%.0f [km]", worldLength / 1000.0f);
    else if (worldLength >= 1.0f)
        snprintf(label, sizeof(label), "%.0f [m]", worldLength);
    else if (worldLength >= 0.01f)
        snprintf(label, sizeof(label), "%.0f [cm]", worldLength * 100.0f);
    else
        snprintf(label, sizeof(label), "<1 [cm]");

    float rulerY = originY + compassSize * 0.45f;
    Color rulerColor = ColorFromNormalized(Vector4{1.0f - bg_color.x, 1.0f - bg_color.y, 1.0f - bg_color.z, 1.0f});
    DrawLineEx(Vector2{originX - 40.f, rulerY}, Vector2{originX + 40.f, rulerY}, 2.f, rulerColor);
    DrawLineEx(Vector2{originX - 40.f, rulerY - 5.f}, Vector2{originX - 40.f, rulerY + 5.f}, 2.f, rulerColor);
    DrawLineEx(Vector2{originX + 40.f, rulerY - 5.f}, Vector2{originX + 40.f, rulerY + 5.f}, 2.f, rulerColor);
    DrawText(label, (int)originX - 20, (int)rulerY + 6, 14, rulerColor);
}

// GL-free -- copied verbatim.
float distanceToPlane(const RegistrationPlaneFeature::Plane& plane, const Eigen::Vector3d& p)
{
    return (plane.a * p.x() + plane.b * p.y() + plane.c * p.z() + plane.d);
}

// GL-free -- copied verbatim.
Eigen::Vector3d rayIntersection(const LaserBeam& laser_beam, const RegistrationPlaneFeature::Plane& plane)
{
    float TOLERANCE = 0.0001;
    Eigen::Vector3d out_point;
    out_point.x() = laser_beam.position.x();
    out_point.y() = laser_beam.position.y();
    out_point.z() = laser_beam.position.z();

    float a = plane.a * laser_beam.direction.x() + plane.b * laser_beam.direction.y() + plane.c * laser_beam.direction.z();

    if (a > -TOLERANCE && a < TOLERANCE)
    {
        return out_point;
    }

    float distance = distanceToPlane(plane, out_point);

    out_point.x() = laser_beam.position.x() - laser_beam.direction.x() * (distance / a);
    out_point.y() = laser_beam.position.y() - laser_beam.direction.y() * (distance / a);
    out_point.z() = laser_beam.position.z() - laser_beam.direction.z() * (distance / a);

    return out_point;
}

// Was gluUnProject(winX, winY, winZ, modelview, projection, viewport, ...)
// against glGetDoublev(GL_MODELVIEW/PROJECTION_MATRIX) -- rewritten with
// raymath's Vector3Unproject against rlgl's current matrix stack
// (rlGetMatrixModelview/Projection), following the same NDC-space
// conversion raylib's own GetScreenToWorldRayEx uses. The original's
// far point used winZ=-1000 (an out-of-range hack to get a point far along
// the ray, since gluUnProject doesn't clamp); using the actual far-plane
// NDC z=1 here is equally valid for the same purpose (only direction, not
// magnitude, of laser_beam.direction matters to callers).
LaserBeam GetLaserBeam(int x, int y)
{
    int width = GetScreenWidth();
    int height = GetScreenHeight();

    float ndcX = (2.0f * (float)x) / (float)width - 1.0f;
    float ndcY = 1.0f - (2.0f * (float)y) / (float)height;

    Matrix matView = rlGetMatrixModelview();
    Matrix matProj = rlGetMatrixProjection();

    Vector3 nearPoint = Vector3Unproject(Vector3{ndcX, ndcY, 0.0f}, matProj, matView);
    Vector3 farPoint = Vector3Unproject(Vector3{ndcX, ndcY, 1.0f}, matProj, matView);

    LaserBeam laser_beam;
    laser_beam.position = Eigen::Vector3d(nearPoint.x, nearPoint.y, nearPoint.z);
    laser_beam.direction = Eigen::Vector3d(farPoint.x - nearPoint.x, farPoint.y - nearPoint.y, farPoint.z - nearPoint.z);
    return laser_beam;
}

// GL-free -- copied verbatim.
double distance_point_to_line(const Eigen::Vector3d& point, const LaserBeam& line)
{
    Eigen::Vector3d AP = point - line.position;
    return (AP.cross(line.direction)).norm();
}

// GL-free -- copied verbatim.
void getClosestTrajectoryPoint(Session& session_, int x, int y, bool gcpPicking, int& picked_index)
{
    picked_index = -1;

    const auto laser_beam = GetLaserBeam(x, y);
    double min_distance = std::numeric_limits<double>::max();
    int index_i = -1;
    int index_j = -1;

    for (int i = 0; i < session_.point_clouds_container.point_clouds.size(); i++)
    {
        for (int j = 0; j < session_.point_clouds_container.point_clouds[i].local_trajectory.size(); j++)
        {
            const auto& p = session_.point_clouds_container.point_clouds[i].local_trajectory[j].m_pose.translation();
            Eigen::Vector3d vp = session_.point_clouds_container.point_clouds[i].m_pose * p;

            double dist = distance_point_to_line(vp, laser_beam);

            if (dist < min_distance)
            {
                min_distance = dist;
                index_i = i;
                index_j = j;

                new_rotation_center.x() = static_cast<float>(vp.x());
                new_rotation_center.y() = static_cast<float>(vp.y());
                new_rotation_center.z() = static_cast<float>(vp.z());

                if (gcpPicking)
                {
                    session_.ground_control_points.picking_mode_index_to_node_inner = index_i;
                    session_.ground_control_points.picking_mode_index_to_node_outer = index_j;
                }

                picked_index = index_i;
            }
        }
    }

    new_rotate_x = rotate_x;
    new_rotate_y = rotate_y;
    new_translate_x = -new_rotation_center.x();
    new_translate_y = -new_rotation_center.y();
    new_translate_z = translate_z;
    camera_transition_active = true;
}

// GL-free -- copied verbatim.
void setNewRotationCenter(int x, int y)
{
    const auto laser_beam = GetLaserBeam(x, y);

    RegistrationPlaneFeature::Plane pl;

    pl.a = 0;
    pl.b = 0;
    pl.c = 1;
    pl.d = 0;
    new_rotation_center = rayIntersection(laser_beam, pl).cast<float>();

    std::cout << "Setting new rotation center to:\n" << new_rotation_center << std::endl;

    new_rotate_x = rotate_x;
    new_rotate_y = rotate_y;
    new_translate_x = -new_rotation_center.x();
    new_translate_y = -new_rotation_center.y();
    new_translate_z = translate_z;

    camera_transition_active = true;
}

// GL-free -- copied verbatim.
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

// Was glOrtho + gluLookAt (folded into GL_PROJECTION, matching the
// original's call order -- gluLookAt ran before the GL_MODELVIEW switch
// below) -- rewritten as rlOrtho + rlMultMatrixf with the same lookAt
// matrix already computed via GLM for m_ortho_gizmo_view just above it.
void updateOrthoView()
{
    // still updating viewLocal for compass
    viewLocal.rotate(Eigen::AngleAxisf((rotate_x + rotate_y) * DEG_TO_RAD, Eigen::Vector3f::UnitZ()));

    ImGuiIO& io = ImGui::GetIO();
    float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);

    rlOrtho(
        -camera_ortho_xy_view_zoom,
        camera_ortho_xy_view_zoom,
        -camera_ortho_xy_view_zoom / ratio,
        camera_ortho_xy_view_zoom / ratio,
        -100000,
        100000);

    glm::mat4 proj = glm::orthoLH_ZO<float>(
        -camera_ortho_xy_view_zoom,
        camera_ortho_xy_view_zoom,
        -camera_ortho_xy_view_zoom / ratio,
        camera_ortho_xy_view_zoom / ratio,
        -100,
        100);

    std::copy(&proj[0][0], &proj[3][3], m_ortho_projection);

    Eigen::Vector3d v_eye_t(-camera_ortho_xy_view_shift_x, camera_ortho_xy_view_shift_y, camera_mode_ortho_z_center_h + 10);
    Eigen::Vector3d v_center_t(-camera_ortho_xy_view_shift_x, camera_ortho_xy_view_shift_y, camera_mode_ortho_z_center_h);
    Eigen::Vector3d v(0, 1, 0);

    TaitBryanPose pose_tb;
    pose_tb.px = 0.0;
    pose_tb.py = 0.0;
    pose_tb.pz = 0.0;
    pose_tb.om = 0.0;
    pose_tb.fi = 0.0;
    pose_tb.ka = -(rotate_x + rotate_y) * DEG_TO_RAD;
    auto m = affine_matrix_from_pose_tait_bryan(pose_tb);

    Eigen::Vector3d v_t = m * v;

    glm::mat4 lookat = glm::lookAt(
        glm::vec3(v_eye_t.x(), v_eye_t.y(), v_eye_t.z()),
        glm::vec3(v_center_t.x(), v_center_t.y(), v_center_t.z()),
        glm::vec3(v_t.x(), v_t.y(), v_t.z()));
    std::copy(&lookat[0][0], &lookat[3][3], m_ortho_gizmo_view);

    rlMultMatrixf(&lookat[0][0]);

    rlMatrixMode(RL_MODELVIEW);
    rlLoadIdentity();
}

// Restores rlgl's default 2D screen-space projection (matches what
// raylib's own EndMode3D() does), since this app drives the rlgl matrix
// stack manually (rlMatrixMode/rlFrustum/rlMultMatrixf in reshape()/
// display() above) instead of using raylib's BeginMode3D/EndMode3D
// wrapper. Must be called after all 3D drawing and before any 2D drawing
// (the mini-compass, ImGui) each frame.
void end3DMatrixStack()
{
    rlDrawRenderBatchActive();
    rlMatrixMode(RL_PROJECTION);
    rlLoadIdentity();
    // io.DisplaySize, not GetScreenWidth()/GetScreenHeight(): reshape()
    // sets the actual GL viewport from io.DisplaySize (see display()'s call
    // to it), and the two can differ under DPI scaling -- this has to
    // match the viewport currently in effect, or 2D screen-space math done
    // against it (e.g. renderLoopClosureLabels()'s world-to-pixel
    // projection) lands off by the mismatch.
    ImGuiIO& io = ImGui::GetIO();
    rlOrtho(0, io.DisplaySize.x, io.DisplaySize.y, 0, 0.0f, 1.0f);
    rlMatrixMode(RL_MODELVIEW);
    rlLoadIdentity();
    rlDisableDepthTest();
}
