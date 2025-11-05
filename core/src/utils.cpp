#include "utils.hpp"
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

///////////////////////////////////////////////////////////////////////////////////

int viewer_decimate_point_cloud = 1000;

int mouse_old_x, mouse_old_y;
bool gui_mouse_down = false;
int mouse_buttons = 0;
float mouse_sensitivity = 1.0;

bool is_ortho = false;
bool show_axes = true;
ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
int point_size = 1;

bool info_gui = false;
bool compass_ruler = true;

Eigen::Vector3f rotation_center = Eigen::Vector3f::Zero();
float rotate_x = 0.0, rotate_y = 0.0;
float translate_x, translate_y = 0.0;
float translate_z = -50.0;

double camera_ortho_xy_view_zoom = 10;
double camera_ortho_xy_view_shift_x = 0.0;
double camera_ortho_xy_view_shift_y = 0.0;
double camera_ortho_xy_view_rotation_angle_deg = 0;
double camera_mode_ortho_z_center_h = 0.0;

// Target camera state for smooth transitions
Eigen::Vector3f new_rotation_center = rotation_center;
float new_rotate_x = rotate_x;
float new_rotate_y = rotate_y;
float new_translate_x = translate_x;
float new_translate_y = translate_y;
float new_translate_z = translate_z;

// Transition timing
bool camera_transition_active = false;

///////////////////////////////////////////////////////////////////////////////////

std::string truncPath(const std::string& fullPath)
{
    namespace fs = std::filesystem;
    fs::path path(fullPath);

    auto parent = path.parent_path().parent_path().filename().string(); // second to last folder
    auto filename = path.filename().string();

    return "..\\" + parent + "\\..\\" + filename;
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

        mouse_sensitivity = fabs(translate_z) / 100; //1 for translate_z 50 (default zoom)
        camera_transition_active = false;
    }
}

//SpecialKeys handlers needed because of ImGui version <1.89 bug in handling keys
void specialDown(int key, int x, int y)
{
    ImGuiIO& io = ImGui::GetIO();
    switch (key)
    {
    case GLUT_KEY_UP:    io.KeysDown[ImGuiKey_UpArrow] = true; break;
    case GLUT_KEY_DOWN:  io.KeysDown[ImGuiKey_DownArrow] = true; break;
    case GLUT_KEY_LEFT:  io.KeysDown[ImGuiKey_LeftArrow] = true; break;
    case GLUT_KEY_RIGHT: io.KeysDown[ImGuiKey_RightArrow] = true; break;
    case GLUT_KEY_PAGE_UP:   io.KeysDown[ImGuiKey_PageUp] = true; break;
    case GLUT_KEY_PAGE_DOWN: io.KeysDown[ImGuiKey_PageDown] = true; break;
    }

    int mods = glutGetModifiers();
    io.KeyCtrl = (mods & GLUT_ACTIVE_CTRL) != 0;
    io.KeyShift = (mods & GLUT_ACTIVE_SHIFT) != 0;
    io.KeyAlt = (mods & GLUT_ACTIVE_ALT) != 0;
}

void specialUp(int key, int x, int y)
{
    ImGuiIO& io = ImGui::GetIO();
    switch (key)
    {
    case GLUT_KEY_UP:    io.KeysDown[ImGuiKey_UpArrow] = false; break;
    case GLUT_KEY_DOWN:  io.KeysDown[ImGuiKey_DownArrow] = false; break;
    case GLUT_KEY_LEFT:  io.KeysDown[ImGuiKey_LeftArrow] = false; break;
    case GLUT_KEY_RIGHT: io.KeysDown[ImGuiKey_RightArrow] = false; break;
    case GLUT_KEY_PAGE_UP:   io.KeysDown[ImGuiKey_PageUp] = false; break;
    case GLUT_KEY_PAGE_DOWN: io.KeysDown[ImGuiKey_PageDown] = false; break;
    }

    int mods = glutGetModifiers();
    io.KeyCtrl = (mods & GLUT_ACTIVE_CTRL) != 0;
    io.KeyShift = (mods & GLUT_ACTIVE_SHIFT) != 0;
    io.KeyAlt = (mods & GLUT_ACTIVE_ALT) != 0;
}



void updateCameraTransition()
{
    if (!camera_transition_active)
        return;

    // Smoothly approach the target using exponential interpolation
    //const float t = std::min(deltaTime * camera_transition_speed, 1.0f);

    // Ease out curve, slow down effect
    float t = 1.0f - pow(1.0f - std::min(ImGui::GetIO().DeltaTime * camera_transition_speed, 1.0f), 3.0f);

    bool doneXrc = fabs(new_rotation_center.x() - rotation_center.x()) < 0.01f;
    bool doneYrc = fabs(new_rotation_center.y() - rotation_center.y()) < 0.01f;
    bool doneZrc = fabs(new_rotation_center.z() - rotation_center.z()) < 0.01f;
    bool doneXr = fabs(new_rotate_x - rotate_x) < 0.01f;
    bool doneYr = fabs(new_rotate_y - rotate_y) < 0.01f;
    bool doneXt = fabs(new_translate_x - translate_x) < 0.01f;
    bool doneYt = fabs(new_translate_y - translate_y) < 0.01f;
    bool doneZt = fabs(new_translate_z - translate_z) < 0.01f;

    if (!doneXrc) rotation_center.x() += (new_rotation_center.x() - rotation_center.x()) * t;
    if (!doneYrc) rotation_center.y() += (new_rotation_center.y() - rotation_center.y()) * t;
    if (!doneZrc) rotation_center.z() += (new_rotation_center.z() - rotation_center.z()) * t;
    if (!doneXr) rotate_x += (new_rotate_x - rotate_x) * t;
    if (!doneYr) rotate_y += (new_rotate_y - rotate_y) * t;
    if (!doneXt) translate_x += (new_translate_x - translate_x) * t;
    if (!doneYt) translate_y += (new_translate_y - translate_y) * t;
    if (!doneZt) translate_z += (new_translate_z - translate_z) * t;

    camera_transition_active = !(doneXrc && doneYrc && doneZrc && doneXr && doneYr && doneXt && doneYt && doneZt);

	//making sure exact coordinates are reached at the end of the transition
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

void breakCameraTransition()
{
    if (camera_transition_active == false)
        return;
    //commit new rotation center
	rotation_center = new_rotation_center;
	//reset transition
	camera_transition_active = false;
}

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
        mouse_sensitivity = fabs(translate_z) / 100; //1 for translate_z 50 (default zoom)

        camera_ortho_xy_view_zoom = 10;
        camera_ortho_xy_view_shift_x = 0.0;
        camera_ortho_xy_view_shift_y = 0.0;
        camera_ortho_xy_view_rotation_angle_deg = 0;
        camera_mode_ortho_z_center_h = 0.0;

        viewer_decimate_point_cloud = 1000;
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
}

void view_kbd_shortcuts()
{
    ImGuiIO& io = ImGui::GetIO();

    if (io.WantCaptureKeyboard) return;

    //translate camera
    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_RightArrow, true))
    {
        translate_x += 0.2f * mouse_sensitivity;
        breakCameraTransition();
    }
    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_LeftArrow, true))
    {
        translate_x -= 0.2f * mouse_sensitivity;
        breakCameraTransition();
    }

    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_UpArrow, true))
    {
        translate_y += 0.2f * mouse_sensitivity;
        breakCameraTransition();
    }
    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_DownArrow, true))
    {
        translate_y -= 0.2f * mouse_sensitivity;
        breakCameraTransition();
    }

    //rotate camera
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_RightArrow, true))
    {
        rotate_y -= mouse_sensitivity;
        breakCameraTransition();
    }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_LeftArrow, true))
    {
        rotate_y += mouse_sensitivity;
        breakCameraTransition();
    }

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_UpArrow, true))
    {
        rotate_x -= mouse_sensitivity;
        breakCameraTransition();
    }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_DownArrow, true))
    {
        rotate_x += mouse_sensitivity;
        breakCameraTransition();
    }

    //only checking for single key press (no modifiers) from this point
    if (io.KeyCtrl || io.KeyShift) return;


    if (ImGui::IsKeyPressed('B'))
        setCameraPreset(CAMERA_BACK);
    if (ImGui::IsKeyPressed('F'))
        setCameraPreset(CAMERA_FRONT);
    if (ImGui::IsKeyPressed('I'))
        setCameraPreset(CAMERA_ISO);
    if (ImGui::IsKeyPressed('L'))
        setCameraPreset(CAMERA_LEFT);
    if (ImGui::IsKeyPressed('R'))
        setCameraPreset(CAMERA_RIGHT);
    if (ImGui::IsKeyPressed('T'))
        setCameraPreset(CAMERA_TOP);
    if (ImGui::IsKeyPressed('U'))
        setCameraPreset(CAMERA_BOTTOM);
    if (ImGui::IsKeyPressed('Z'))
        setCameraPreset(CAMERA_RESET);

    if (ImGui::IsKeyPressed('C'))
        compass_ruler = !compass_ruler;
    if (ImGui::IsKeyPressed('O'))
        is_ortho = !is_ortho;
    if (ImGui::IsKeyPressed('X'))
        show_axes = !show_axes;


    if (ImGui::IsKeyPressed('1'))
		point_size = 1;
	if (ImGui::IsKeyPressed('2'))
		point_size = 2;
	if (ImGui::IsKeyPressed('3'))
		point_size = 3;
	if (ImGui::IsKeyPressed('4'))
		point_size = 4;
	if (ImGui::IsKeyPressed('5'))
		point_size = 5;
    if (ImGui::IsKeyPressed('6'))
		point_size = 6;
	if (ImGui::IsKeyPressed('7'))
		point_size = 7;
	if (ImGui::IsKeyPressed('8'))
		point_size = 8;
	if (ImGui::IsKeyPressed('9'))
		point_size = 9;

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
    if (worldLength >= 1000.0f)      sprintf(label, "%.0f [km]", worldLength / 1000.0f);
    else if (worldLength >= 1.0f)    sprintf(label, "%.0f [m]", worldLength);
    else if (worldLength >= 0.01f)   sprintf(label, "%.0f [cm]", worldLength * 100.0f);
    else                             sprintf(label, "<1 [cm]");

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



float distanceToPlane(const RegistrationPlaneFeature::Plane& plane, const Eigen::Vector3d& p)
{
    return (plane.a * p.x() + plane.b * p.y() + plane.c * p.z() + plane.d);
}

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

LaserBeam GetLaserBeam(int x, int y)
{
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posXnear, posYnear, posZnear;
    GLdouble posXfar, posYfar, posZfar;

    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);

    winX = (float)x;
    winY = (float)viewport[3] - (float)y;

    LaserBeam laser_beam;
    gluUnProject(winX, winY, 0, modelview, projection, viewport, &posXnear, &posYnear, &posZnear);
    gluUnProject(winX, winY, -1000, modelview, projection, viewport, &posXfar, &posYfar, &posZfar);

    laser_beam.position.x() = posXnear;
    laser_beam.position.y() = posYnear;
    laser_beam.position.z() = posZnear;

    laser_beam.direction.x() = posXfar - posXnear;
    laser_beam.direction.y() = posYfar - posYnear;
    laser_beam.direction.z() = posZfar - posZnear;

    return laser_beam;
}



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