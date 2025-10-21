#include "utils.hpp"
//#include <GL/gl.h>
#include <GL/glew.h>
#include <GL/glut.h>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <HDMapping/Version.hpp>
#include <filesystem>

#ifdef _WIN32
//#include <windows.h>
#include <shellapi.h>
#endif



void drawMiniCompassWithRuler(
    const Eigen::Affine3f& viewLocal,
    float translate_z,
    const ImVec4& bg_color,
    ImVec2 compassSize)
{
    auto drawLabel = [](float x, float y, float z, const char* text, float r, float g, float b)
        {
            glColor3f(r, g, b);
            glRasterPos3f(x, y, z);
            for (const char* c = text; *c; ++c)
                glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *c);
        };

    ImGuiIO& io = ImGui::GetIO();

    // Save viewport
    GLint prevViewport[4];
    glGetIntegerv(GL_VIEWPORT, prevViewport);
    // Save matrixes
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    // Save line width
    GLfloat prevLineWidth;
    glGetFloatv(GL_LINE_WIDTH, &prevLineWidth);

    GLboolean depthTestEnabled = glIsEnabled(GL_DEPTH_TEST);

    glViewport(0, 0, compassSize.x, compassSize.y);
    glClear(GL_DEPTH_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);

    // Projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-1.5, 1.5, -1.5, 1.5, -10, 10);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Apply rotation
    Eigen::Affine3f onlyRot = Eigen::Affine3f::Identity();
    onlyRot.linear() = viewLocal.rotation();
    glMultMatrixf(onlyRot.matrix().data());

    float miniAxisLength = 1.0f;

    // Snap ruler length to a "nice" round number (1, 2, or 5 × 10^n)
    float rawUnit = 0.1f * translate_z; // adjust factor to taste
    float base = pow(10.0f, floor(log10(rawUnit)));
    float normalized = rawUnit / base;
    float niceUnit;
    if (normalized < 2.0f)      niceUnit = 1.0f;
    else if (normalized < 5.0f) niceUnit = 2.0f;
    else                        niceUnit = 5.0f;
    float worldLength = niceUnit * base;

    float scale = miniAxisLength / rawUnit; // convert scroll units to visual units
    float axisDrawLength = worldLength * scale; // final axis length in mini compass

    // Draw axes
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(axisDrawLength, 0, 0); // X
    glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, axisDrawLength, 0); // Y
    glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, axisDrawLength); // Z
    glEnd();

    // Draw axis labels
    drawLabel(axisDrawLength + 0.2f, 0, 0, "X (long.)", 1, 0, 0);
    drawLabel(0, axisDrawLength + 0.2f, 0, "Y (lat.)", 0, 1, 0);
    drawLabel(0, 0, axisDrawLength + 0.2f, "Z (vert.)", 0, 0, 1);

    // End scale label
    char label[24];
    if (worldLength >= 1000.0f) sprintf(label, "%.0f [km]", worldLength / 1000.0f);
    else if (worldLength >= 1.0f)    sprintf(label, "%.0f [m]", worldLength);
    else                             sprintf(label, "%.0f [cm]", worldLength * 100.0f);

    //drawLabel(axes[2].x() + 0.2f, axes[2].y() + 0.4f, axes[2].z() - 0.2f, label,
    //    colors[2][0], colors[2][1], colors[2][2]);
    drawLabel(axisDrawLength / 2, axisDrawLength / 2, axisDrawLength / 2, label, 1.0 - bg_color.x, 1.0 - bg_color.y, 1.0 - bg_color.z);

    // Restore line width
    glLineWidth(prevLineWidth);

    // Restore viewport
    if (depthTestEnabled)
        glEnable(GL_DEPTH_TEST);
    else
        glDisable(GL_DEPTH_TEST);

    // Restore matrices
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    // Restore viewport
    glViewport(prevViewport[0], prevViewport[1], prevViewport[2], prevViewport[3]);
}



// Target camera state for smooth transitions
float target_rotate_x = rotate_x;
float target_rotate_y = rotate_y;
float target_translate_x = translate_x;
float target_translate_y = translate_y;
float target_translate_z = translate_z;

// Transition timing
bool camera_transition_active = false;
const float camera_transition_speed = 1.0f; // higher = faster

void updateCameraTransition(float deltaTime)
{
    if (!camera_transition_active)
        return;

    // Smoothly approach the target using exponential interpolation
    //const float t = std::min(deltaTime * camera_transition_speed, 1.0f);

    // Ease out curve, slow down effect
    float t = 1.0f - pow(1.0f - std::min(deltaTime * camera_transition_speed, 1.0f), 3.0f);

    bool doneXr = fabs(target_rotate_x - rotate_x) < 0.01f;
    bool doneYr = fabs(target_rotate_y - rotate_y) < 0.01f;
    bool doneXt = fabs(target_translate_x - translate_x) < 0.01f;
    bool doneYt = fabs(target_translate_y - translate_y) < 0.01f;
    bool doneZt = fabs(target_translate_z - translate_z) < 0.01f;

    if (!doneXr) rotate_x += (target_rotate_x - rotate_x) * t;
    if (!doneYr) rotate_y += (target_rotate_y - rotate_y) * t;
    if (!doneXt) translate_x += (target_translate_x - translate_x) * t;
    if (!doneYt) translate_y += (target_translate_y - translate_y) * t;
    if (!doneZt) translate_z += (target_translate_z - translate_z) * t;

    camera_transition_active = !(doneXr && doneYr && doneZt);
}



void setCameraPreset(CameraPreset preset)
{
    target_translate_z = translate_z;

    switch (preset)
    {
    case CAMERA_FRONT:
        target_rotate_x = -90.0f;
        target_rotate_y = +90.0f;
        break;
    case CAMERA_BACK:
        target_rotate_x = -90.0f;
        target_rotate_y = -90.0f;
        break;
    case CAMERA_LEFT:
        target_rotate_x = -90.0f;
        target_rotate_y = 180.0f;
        break;
    case CAMERA_RIGHT:
        target_rotate_x = -90.0f;
        target_rotate_y = 0.0f;
        break;
    case CAMERA_TOP:
        target_rotate_x = 0.0f;
        target_rotate_y = 90.0f;
        break;
    case CAMERA_BOTTOM:
        target_rotate_x = 180.0f;
        target_rotate_y = -90.0f;
        break;
    case CAMERA_ISO:
        target_rotate_x = -35.264f;
        target_rotate_y = 135.0f;
        break;
    case CAMERA_RESET:
        target_rotate_x = 0;
        target_rotate_y = 0;
        target_translate_x = 0;
        target_translate_y = 0;
        target_translate_z = -50.0f;
#ifdef ENABLE_ORTHO_SETTINGS
        viewer_decimate_point_cloud = 1000;

        camera_ortho_xy_view_zoom = 10;
        camera_ortho_xy_view_shift_x = 0.0;
        camera_ortho_xy_view_shift_y = 0.0;
        camera_ortho_xy_view_rotation_angle_deg = 0;
        camera_mode_ortho_z_center_h = 0.0;
#endif
        break;
    }

    camera_transition_active = true;

    std::cout << "Camera: " << rotate_x << ", " << rotate_y << ", " << camera_transition_active << std::endl;
}



void ImGuiHyperlink(const char* url, ImVec4 color)
{
    ImGui::PushStyleColor(ImGuiCol_Text, color);
    ImGui::TextUnformatted(url);
    ImGui::PopStyleColor();

    // Get the item's rectangle
    ImVec2 pos = ImGui::GetItemRectMin();
    ImVec2 size = ImGui::GetItemRectSize();

    // Change cursor on hover
    if (ImGui::IsItemHovered())
        ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);

    // Draw underline on hover
    if (ImGui::IsItemHovered())
    {
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        draw_list->AddLine(
            ImVec2(pos.x, pos.y + size.y),
            ImVec2(pos.x + size.x, pos.y + size.y),
            ImColor(color)
        );
    }

    // Open URL on click
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



void info_window(const std::vector<std::string>& infoLines, bool* open)
{
    if (!open || !*open) return;

    if (ImGui::Begin("Info", open, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize))
    {
        for (const auto& line : infoLines)
        {
            if (line.empty())
                ImGui::NewLine();
            else if (line.rfind("https://", 0) == 0) // starts with "https://"
                ImGuiHyperlink(line.c_str());
            else
                ImGui::Text(line.c_str());
        }

        ImGui::NewLine();
        ImGui::Text("Author: Janusz Bedkowski & contributors");
        ImGui::NewLine();
        ImGui::Text("Part of HDMapping software suite");
        ImGui::Text("Version: %s (%s)", HDMAPPING_VERSION_STRING, __DATE__);
        ImGui::Text("Project page: ");
        ImGui::SameLine();
        ImGuiHyperlink("https://github.com/MapsHD/HDMapping");

        ImGui::End();
    }
}



std::string truncPath(const std::string& fullPath)
{
    namespace fs = std::filesystem;
    fs::path path(fullPath);

    auto parent = path.parent_path();
    auto filename = path.filename().string();

    std::string dir2;
    if (parent.has_parent_path())
        dir2 = parent.parent_path().string(); // second to last folder

    return "..\\" + dir2 + "\\..\\" + filename;
}