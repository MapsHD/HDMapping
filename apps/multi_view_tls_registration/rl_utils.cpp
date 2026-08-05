#include "rl_utils.h"

#include "external/glad.h"
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include <imgui_internal.h>

#include <Core/transformations.h>
#include <RaylibWidgets/CompassRuler.h>

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
// NOGDI/NOUSER: windows.h's wingdi.h/winuser.h #define (or, for CloseWindow/
// ShowCursor, directly declare) identifiers that collide with raylib.h's
// own DrawText/CloseWindow/ShowCursor -- without these, windows.h wins and
// every call to raylib's DrawText() above silently becomes a call to the
// Win32 GDI DrawTextA() instead, which doesn't compile against raylib's
// arguments. NOUSER also strips SW_SHOWNORMAL (a windows.h macro), so
// ImGuiHyperlink's ShellExecuteA call below uses its literal value (1, a
// stable, decades-unchanged Win32 constant) instead.
//
// windows.h must come before shellapi.h -- shellapi.h depends on macros/
// types windows.h defines, and this file (unlike core/src/utils.cpp, which
// gets windows.h transitively via its precompiled header before this same
// ordering matters) has nothing else pulling windows.h in first.
#define NOGDI
#define NOUSER
// clang-format off
#include <windows.h>
#include <shellapi.h>
// clang-format on
#endif

///////////////////////////////////////////////////////////////////////////////////
// Formerly <Core/utils.hpp>'s extern globals -- now members of AppStateBase,
// defined in rl_utils.h (see its top comment for why).
///////////////////////////////////////////////////////////////////////////////////

bool cor_gui = false;

bool scroll_hint_enabled = true;
bool scroll_hint_active = false;
int scroll_hint_count = 0;
float scroll_hint_accu = 0.0f;
double scroll_hint_lastT = 0.0;

bool show_about = false;

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
            if (app_state.is_ortho)
            {
                app_state.camera_ortho_xy_view_zoom -= 0.1f * app_state.camera_ortho_xy_view_zoom;

                if (app_state.camera_ortho_xy_view_zoom < 0.1)
                {
                    app_state.camera_ortho_xy_view_zoom = 0.1;
                }
            }
            else
            {
                if (io.KeyShift)
                    app_state.translate_z += 5.0f;
                else
                    app_state.translate_z += 1.0f;
            }
        }
        else
        {
            if (app_state.is_ortho)
                app_state.camera_ortho_xy_view_zoom += 0.1 * app_state.camera_ortho_xy_view_zoom;
            else
            {
                if (io.KeyShift)
                    app_state.translate_z -= 5.0f;
                else
                    app_state.translate_z -= 1.0f;
            }
        }

        app_state.mouse_sensitivity = fabs(app_state.translate_z) / 100; // 1 for app_state.translate_z 50 (default zoom)
        app_state.camera_transition_active = false;

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
    // GetRenderWidth/Height(), not w/h: see display()'s matching comment --
    // w/h are logical points (window size), the GL viewport needs actual
    // framebuffer pixels.
    rlViewport(0, 0, GetRenderWidth(), GetRenderHeight());
    rlMatrixMode(RL_PROJECTION);
    rlLoadIdentity();
    if (!app_state.is_ortho)
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
            -app_state.camera_ortho_xy_view_zoom,
            app_state.camera_ortho_xy_view_zoom,
            -app_state.camera_ortho_xy_view_zoom / ratio,
            app_state.camera_ortho_xy_view_zoom / ratio,
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
        dx = (float)(x - app_state.mouse_old_x);
        dy = (float)(y - app_state.mouse_old_y);

        if (app_state.mouse_buttons & 1) // left button
        {
            app_state.rotate_x += dy * 0.2f;
            app_state.rotate_y += dx * 0.2f;
            breakCameraTransition();
        }

        if (app_state.mouse_buttons & 4) // right button
        {
            if (app_state.is_ortho)
            {
                float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);
                Eigen::Vector3d v(
                    dx * (app_state.camera_ortho_xy_view_zoom / (float)io.DisplaySize.x * 2),
                    dy * (app_state.camera_ortho_xy_view_zoom / (float)io.DisplaySize.y * 2 / ratio),
                    0);
                TaitBryanPose pose_tb;
                pose_tb.px = 0.0;
                pose_tb.py = 0.0;
                pose_tb.pz = 0.0;
                pose_tb.om = 0.0;
                pose_tb.fi = 0.0;
                pose_tb.ka = (app_state.rotate_x + app_state.rotate_y) * M_PI / 180.0;
                auto m = affine_matrix_from_pose_tait_bryan(pose_tb);
                Eigen::Vector3d v_t = m * v;
                app_state.camera_ortho_xy_view_shift_x += v_t.x();
                app_state.camera_ortho_xy_view_shift_y += v_t.y();
            }
            else
            {
                app_state.translate_x += dx * 0.1f * app_state.mouse_sensitivity;
                app_state.translate_y -= dy * 0.1f * app_state.mouse_sensitivity;
                breakCameraTransition();
            }
        }

        app_state.mouse_old_x = x;
        app_state.mouse_old_y = y;
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
    if (app_state.show_axes || ImGui::GetIO().KeyCtrl) // rotation center axes
    {
        rlBegin(RL_LINES);
        rlColor3f(1.f, 1.f, 1.f);
        rlVertex3f(app_state.rotation_center.x(), app_state.rotation_center.y(), app_state.rotation_center.z());
        rlVertex3f(app_state.rotation_center.x() + 1.f, app_state.rotation_center.y(), app_state.rotation_center.z());
        rlVertex3f(app_state.rotation_center.x(), app_state.rotation_center.y(), app_state.rotation_center.z());
        rlVertex3f(app_state.rotation_center.x() - 1.f, app_state.rotation_center.y(), app_state.rotation_center.z());
        rlVertex3f(app_state.rotation_center.x(), app_state.rotation_center.y(), app_state.rotation_center.z());
        rlVertex3f(app_state.rotation_center.x(), app_state.rotation_center.y() - 1.f, app_state.rotation_center.z());
        rlVertex3f(app_state.rotation_center.x(), app_state.rotation_center.y(), app_state.rotation_center.z());
        rlVertex3f(app_state.rotation_center.x(), app_state.rotation_center.y() + 1.f, app_state.rotation_center.z());
        rlVertex3f(app_state.rotation_center.x(), app_state.rotation_center.y(), app_state.rotation_center.z());
        rlVertex3f(app_state.rotation_center.x(), app_state.rotation_center.y(), app_state.rotation_center.z() - 1.f);
        rlVertex3f(app_state.rotation_center.x(), app_state.rotation_center.y(), app_state.rotation_center.z());
        rlVertex3f(app_state.rotation_center.x(), app_state.rotation_center.y(), app_state.rotation_center.z() + 1.f);
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

// GL-free -- copied verbatim.
void updateCameraTransition()
{
    if (!app_state.camera_transition_active)
        return;

    float t = 1.0f - powf(1.0f - std::min(ImGui::GetIO().DeltaTime * camera_transition_speed, 1.0f), 3.0f);

    bool doneXrc = fabs(app_state.new_rotation_center.x() - app_state.rotation_center.x()) < 0.01f;
    bool doneYrc = fabs(app_state.new_rotation_center.y() - app_state.rotation_center.y()) < 0.01f;
    bool doneZrc = fabs(app_state.new_rotation_center.z() - app_state.rotation_center.z()) < 0.01f;
    bool doneXr = fabs(app_state.new_rotate_x - app_state.rotate_x) < 0.01f;
    bool doneYr = fabs(app_state.new_rotate_y - app_state.rotate_y) < 0.01f;
    bool doneXt = fabs(app_state.new_translate_x - app_state.translate_x) < 0.01f;
    bool doneYt = fabs(app_state.new_translate_y - app_state.translate_y) < 0.01f;
    bool doneZt = fabs(app_state.new_translate_z - app_state.translate_z) < 0.01f;

    if (!doneXrc)
        app_state.rotation_center.x() += (app_state.new_rotation_center.x() - app_state.rotation_center.x()) * t;
    if (!doneYrc)
        app_state.rotation_center.y() += (app_state.new_rotation_center.y() - app_state.rotation_center.y()) * t;
    if (!doneZrc)
        app_state.rotation_center.z() += (app_state.new_rotation_center.z() - app_state.rotation_center.z()) * t;
    if (!doneXr)
        app_state.rotate_x += (app_state.new_rotate_x - app_state.rotate_x) * t;
    if (!doneYr)
        app_state.rotate_y += (app_state.new_rotate_y - app_state.rotate_y) * t;
    if (!doneXt)
        app_state.translate_x += (app_state.new_translate_x - app_state.translate_x) * t;
    if (!doneYt)
        app_state.translate_y += (app_state.new_translate_y - app_state.translate_y) * t;
    if (!doneZt)
        app_state.translate_z += (app_state.new_translate_z - app_state.translate_z) * t;

    app_state.camera_transition_active = !(doneXrc && doneYrc && doneZrc && doneXr && doneYr && doneXt && doneYt && doneZt);

    if (!app_state.camera_transition_active)
    {
        app_state.rotation_center = app_state.new_rotation_center;
        app_state.rotate_x = app_state.new_rotate_x;
        app_state.rotate_y = app_state.new_rotate_y;
        app_state.translate_x = app_state.new_translate_x;
        app_state.translate_y = app_state.new_translate_y;
        app_state.translate_z = app_state.new_translate_z;
    }
}

// GL-free -- copied verbatim.
void breakCameraTransition()
{
    if (app_state.camera_transition_active == false)
        return;
    app_state.rotation_center = app_state.new_rotation_center;
    app_state.camera_transition_active = false;
}

// GL-free -- copied verbatim.
void setCameraPreset(CameraPreset preset)
{
    bool triggered = false;

    switch (preset)
    {
    case CAMERA_FRONT:
        app_state.new_rotate_x = -90.0f;
        app_state.new_rotate_y = +90.0f;
        triggered = true;
        break;
    case CAMERA_BACK:
        app_state.new_rotate_x = -90.0f;
        app_state.new_rotate_y = -90.0f;
        triggered = true;
        break;
    case CAMERA_LEFT:
        app_state.new_rotate_x = -90.0f;
        app_state.new_rotate_y = 180.0f;
        triggered = true;
        break;
    case CAMERA_RIGHT:
        app_state.new_rotate_x = -90.0f;
        app_state.new_rotate_y = 0.0f;
        triggered = true;
        break;
    case CAMERA_TOP:
        app_state.new_rotate_x = 0.0f;
        app_state.new_rotate_y = 90.0f;
        triggered = true;
        break;
    case CAMERA_BOTTOM:
        app_state.new_rotate_x = 180.0f;
        app_state.new_rotate_y = -90.0f;
        triggered = true;
        break;
    case CAMERA_ISO:
        app_state.new_rotate_x = -35.264f;
        app_state.new_rotate_y = 135.0f;
        triggered = true;
        break;
    case CAMERA_RESET:
        app_state.new_rotation_center = Eigen::Vector3f::Zero();
        app_state.new_rotate_x = 0;
        app_state.new_rotate_y = 0;
        app_state.new_translate_x = 0;
        app_state.new_translate_y = 0;
        app_state.new_translate_z = -50.0f;
        app_state.mouse_sensitivity = fabs(app_state.translate_z) / 100;

        app_state.camera_ortho_xy_view_zoom = 10;
        app_state.camera_ortho_xy_view_shift_x = 0.0;
        app_state.camera_ortho_xy_view_shift_y = 0.0;
        app_state.camera_mode_ortho_z_center_h = 0.0;

        app_state.viewer_decimate_point_cloud = 1000;
        triggered = false;
        break;
    }

    if (triggered)
    {
        app_state.new_rotation_center = app_state.rotation_center;
        app_state.new_translate_x = app_state.translate_x;
        app_state.new_translate_y = app_state.translate_y;
        app_state.new_translate_z = app_state.translate_z;
    }

    app_state.camera_transition_active = true;
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
            ImGui::Text("%.3f", app_state.rotate_x);
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%.3f", app_state.translate_x);
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%.3f", app_state.rotation_center.x());

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + centered * 0.5f);
            ImGui::Text("Y");

            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.3f", app_state.rotate_y);
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%.3f", app_state.translate_y);
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%.3f", app_state.rotation_center.y());

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + centered * 0.5f);
            ImGui::Text("Z");

            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%.3f", app_state.translate_z);
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%.3f", app_state.rotation_center.y());

            ImGui::EndTable();
        }
        ImGui::Text("Mouse sensitivity: %.4f", app_state.mouse_sensitivity);

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
        app_state.translate_x += 0.5f * app_state.mouse_sensitivity;
        breakCameraTransition();
    }
    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_LeftArrow, true))
    {
        app_state.translate_x -= 0.5f * app_state.mouse_sensitivity;
        breakCameraTransition();
    }

    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_UpArrow, true))
    {
        app_state.translate_y += 0.5f * app_state.mouse_sensitivity;
        breakCameraTransition();
    }
    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_DownArrow, true))
    {
        app_state.translate_y -= 0.5f * app_state.mouse_sensitivity;
        breakCameraTransition();
    }

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_RightArrow, true))
    {
        app_state.rotate_y -= 0.6;
        breakCameraTransition();
    }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_LeftArrow, true))
    {
        app_state.rotate_y += 0.6;
        breakCameraTransition();
    }

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_UpArrow, true))
    {
        app_state.rotate_x -= 0.6;
        breakCameraTransition();
    }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_DownArrow, true))
    {
        app_state.rotate_x += 0.6;
        breakCameraTransition();
    }

    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_R, false))
        cor_gui = true;

    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_Z, false) && !app_state.is_ortho)
        app_state.lock_z = !app_state.lock_z;

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
        app_state.compass_ruler = !app_state.compass_ruler;
    if (ImGui::IsKeyPressed(ImGuiKey_O, false))
        app_state.is_ortho = !app_state.is_ortho;
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
        ImGui::InputFloat("X", &app_state.new_rotation_center.x(), 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(xText);
        ImGui::SameLine();
        ImGui::InputFloat("Y", &app_state.new_rotation_center.y(), 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(yText);
        ImGui::SameLine();
        ImGui::InputFloat("Z", &app_state.new_rotation_center.z(), 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(zText);
        ImGui::PopItemWidth();

        ImGui::Separator();

        if (ImGui::Button("Set"))
        {
            app_state.new_rotate_x = app_state.rotate_x;
            app_state.new_rotate_y = app_state.rotate_y;
            app_state.new_translate_x = -app_state.new_rotation_center.x();
            app_state.new_translate_y = -app_state.new_rotation_center.y();
            app_state.new_translate_z = app_state.translate_z;

            app_state.camera_transition_active = true;

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
        ShellExecuteA(0, "open", url, 0, 0, 1 /* SW_SHOWNORMAL, unavailable under NOUSER -- see this file's top comment */);
#elif __APPLE__
        std::string cmd = std::string("open ") + url;
        system(cmd.c_str());
#else
        std::string cmd = std::string("xdg-open ") + url;
        system(cmd.c_str());
#endif
    }
}

// ShortcutEntry/ShowShortcutsTable moved to raylib_widgets (shared with the
// camera_lidar_* apps) -- the generic shortcut-label scaffolding that used to
// live here was merged directly into gui.cpp's appShortcuts (see the comment
// there), removing the two-list indirection (and a pre-existing off-by-one:
// gui.cpp's list was missing a "Ctrl+J" entry, silently misaligning every
// entry after "J" against this list's descriptions).

// GL-free -- copied verbatim (glGetString(GL_RENDERER/...) is a plain
// string query, still valid under a core-profile context).
void info_window(const std::vector<std::string>& infoLines, const std::vector<ShortcutEntry>& appShortcuts)
{
    if (!app_state.info_gui)
        return;

    if (ImGui::Begin(
            "Info",
            &app_state.info_gui,
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

        raylib_widgets::ShowShortcutsTable(appShortcuts);

        if (show_about)
            ImGui::ShowAboutWindow(&show_about);
    }

    ImGui::End();
}

// Drawing itself now lives in raylib_widgets::drawCompassRuler (shared with
// the camera_lidar_* apps' identical overlay) -- this just adapts this app's
// own camera/background state (app_state.viewLocal's rotation matrix,
// app_state.translate_z zoom, app_state.bg_color) into that function's
// right/up/zoomDistance/rulerColor parameters. Row 0/1 of a world-to-eye
// rotation matrix R are exactly the world-space directions that map to
// eye-space +X/+Y (screen right/up): (R * dir).x() == dot(R.row(0), dir).
void drawMiniCompassWithRuler()
{
    const Eigen::Matrix3f& R = app_state.viewLocal.rotation();
    Vector3 right = { R(0, 0), R(0, 1), R(0, 2) };
    Vector3 up = { R(1, 0), R(1, 1), R(1, 2) };
    Color rulerColor =
        ColorFromNormalized(Vector4{ 1.0f - app_state.bg_color.x, 1.0f - app_state.bg_color.y, 1.0f - app_state.bg_color.z, 1.0f });
    raylib_widgets::drawCompassRuler(
        right, up, app_state.translate_z, rulerColor, raylib_widgets::CompassAxisLabels{ "X (long.)", "Y (lat.)", "Z (vert.)" });
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

    Matrix matView = app_state.frame_view_3d;
    Matrix matProj = app_state.frame_proj_3d;

    Vector3 nearPoint = Vector3Unproject(Vector3{ ndcX, ndcY, 0.0f }, matProj, matView);
    Vector3 farPoint = Vector3Unproject(Vector3{ ndcX, ndcY, 1.0f }, matProj, matView);

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

                app_state.new_rotation_center.x() = static_cast<float>(vp.x());
                app_state.new_rotation_center.y() = static_cast<float>(vp.y());
                app_state.new_rotation_center.z() = static_cast<float>(vp.z());

                if (gcpPicking)
                {
                    session_.ground_control_points.picking_mode_index_to_node_inner = index_i;
                    session_.ground_control_points.picking_mode_index_to_node_outer = index_j;
                }

                picked_index = index_i;
            }
        }
    }

    app_state.new_rotate_x = app_state.rotate_x;
    app_state.new_rotate_y = app_state.rotate_y;
    app_state.new_translate_x = -app_state.new_rotation_center.x();
    app_state.new_translate_y = -app_state.new_rotation_center.y();
    app_state.new_translate_z = app_state.translate_z;
    app_state.camera_transition_active = true;
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
    app_state.new_rotation_center = rayIntersection(laser_beam, pl).cast<float>();

    std::cout << "Setting new rotation center to:\n" << app_state.new_rotation_center << std::endl;

    app_state.new_rotate_x = app_state.rotate_x;
    app_state.new_rotate_y = app_state.rotate_y;
    app_state.new_translate_x = -app_state.new_rotation_center.x();
    app_state.new_translate_y = -app_state.new_rotation_center.y();
    app_state.new_translate_z = app_state.translate_z;

    app_state.camera_transition_active = true;
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
// matrix already computed via GLM for app_state.m_ortho_gizmo_view just above it.
void updateOrthoView()
{
    // still updating app_state.viewLocal for compass
    app_state.viewLocal.rotate(Eigen::AngleAxisf((app_state.rotate_x + app_state.rotate_y) * DEG_TO_RAD, Eigen::Vector3f::UnitZ()));

    ImGuiIO& io = ImGui::GetIO();
    float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);

    rlOrtho(
        -app_state.camera_ortho_xy_view_zoom,
        app_state.camera_ortho_xy_view_zoom,
        -app_state.camera_ortho_xy_view_zoom / ratio,
        app_state.camera_ortho_xy_view_zoom / ratio,
        -100000,
        100000);

    glm::mat4 proj = glm::orthoLH_ZO<float>(
        -app_state.camera_ortho_xy_view_zoom,
        app_state.camera_ortho_xy_view_zoom,
        -app_state.camera_ortho_xy_view_zoom / ratio,
        app_state.camera_ortho_xy_view_zoom / ratio,
        -100,
        100);

    std::copy(&proj[0][0], &proj[3][3], app_state.m_ortho_projection);

    Eigen::Vector3d v_eye_t(
        -app_state.camera_ortho_xy_view_shift_x, app_state.camera_ortho_xy_view_shift_y, app_state.camera_mode_ortho_z_center_h + 10);
    Eigen::Vector3d v_center_t(
        -app_state.camera_ortho_xy_view_shift_x, app_state.camera_ortho_xy_view_shift_y, app_state.camera_mode_ortho_z_center_h);
    Eigen::Vector3d v(0, 1, 0);

    TaitBryanPose pose_tb;
    pose_tb.px = 0.0;
    pose_tb.py = 0.0;
    pose_tb.pz = 0.0;
    pose_tb.om = 0.0;
    pose_tb.fi = 0.0;
    pose_tb.ka = -(app_state.rotate_x + app_state.rotate_y) * DEG_TO_RAD;
    auto m = affine_matrix_from_pose_tait_bryan(pose_tb);

    Eigen::Vector3d v_t = m * v;

    glm::mat4 lookat = glm::lookAt(
        glm::vec3(v_eye_t.x(), v_eye_t.y(), v_eye_t.z()),
        glm::vec3(v_center_t.x(), v_center_t.y(), v_center_t.z()),
        glm::vec3(v_t.x(), v_t.y(), v_t.z()));
    std::copy(&lookat[0][0], &lookat[3][3], app_state.m_ortho_gizmo_view);

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
