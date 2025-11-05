#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <ImGuizmo.h>
#include <imgui_internal.h>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <utils.hpp>

#include <Eigen/Eigen>

#include <transformations.h>

#include "pfd_wrapper.hpp"

#include <filesystem>
#include "../lidar_odometry_step_1/lidar_odometry_utils.h"

#include <HDMapping/Version.hpp>

#include <mutex>

#ifdef _WIN32
    #include <windows.h>
    #include <shellapi.h>  // <-- Required for ShellExecuteA
    #include "../../resources/resourceV.h"
#endif

std::string winTitle = std::string("Single session viewer ") + HDMAPPING_VERSION_STRING;

std::vector<std::string> infoLines = {
    "This program is optional step in MANDEYE process",
    "",
    "It analyzes session created in step_1 for problems that need to be addressed further",
    "Next step will be to load session.json file with 'multi_view_tls_registration_step_2' program"
};

#define SAMPLE_PERIOD (1.0 / 200.0)
namespace fs = std::filesystem;

ImVec4 pc_neigbouring_color = ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
ImVec4 pc_color2 = ImVec4(0.0f, 0.0f, 1.0f, 1.0f);

float m_ortho_projection[] = {1, 0, 0, 0,
                              0, 1, 0, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1};

float m_ortho_gizmo_view[] = {1, 0, 0, 0,
                              0, 1, 0, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1};

int index_rendered_points_local = -1;
float offset_intensity = 0.0;
bool show_neighbouring_scans = false;

Session session;

void reshape(int w, int h)
{
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if (!is_ortho)
    {
        gluPerspective(60.0, (GLfloat)w / (GLfloat)h, 0.01, 10000.0);
    }
    else
    {
        ImGuiIO &io = ImGui::GetIO();
        float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);

        glOrtho(-camera_ortho_xy_view_zoom, camera_ortho_xy_view_zoom,
                -camera_ortho_xy_view_zoom / ratio,
                camera_ortho_xy_view_zoom / ratio, -100000, 100000);
        // glOrtho(-translate_z, translate_z, -translate_z * (float)h / float(w), translate_z * float(h) / float(w), -10000, 10000);
    }
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void motion(int x, int y)
{
    ImGuiIO &io = ImGui::GetIO();
    io.MousePos = ImVec2((float)x, (float)y);

    if (!io.WantCaptureMouse)
    {
        float dx, dy;
        dx = (float)(x - mouse_old_x);
        dy = (float)(y - mouse_old_y);

        if (is_ortho)
        {
            if (mouse_buttons & 1)
            {
                float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);
                Eigen::Vector3d v(dx * (camera_ortho_xy_view_zoom / (GLsizei)io.DisplaySize.x * 2),
                                  dy * (camera_ortho_xy_view_zoom / (GLsizei)io.DisplaySize.y * 2 / ratio), 0);
                TaitBryanPose pose_tb;
                pose_tb.px = 0.0;
                pose_tb.py = 0.0;
                pose_tb.pz = 0.0;
                pose_tb.om = 0.0;
                pose_tb.fi = 0.0;
                pose_tb.ka = camera_ortho_xy_view_rotation_angle_deg * M_PI / 180.0;
                auto m = affine_matrix_from_pose_tait_bryan(pose_tb);
                Eigen::Vector3d v_t = m * v;
                camera_ortho_xy_view_shift_x += v_t.x();
                camera_ortho_xy_view_shift_y += v_t.y();
            }
        }
        else
        {
            gui_mouse_down = mouse_buttons > 0;
            if (mouse_buttons & 1)
            {
                rotate_x += dy * 0.2f; // * mouse_sensitivity;
                rotate_y += dx * 0.2f; // * mouse_sensitivity;
                camera_transition_active = false;
            }
            if (mouse_buttons & 4)
            {
                translate_x += dx * 0.05f * mouse_sensitivity;
                translate_y -= dy * 0.05f * mouse_sensitivity;
                camera_transition_active = false;
            }
        }

        mouse_old_x = x;
        mouse_old_y = y;
    }
    glutPostRedisplay();
}

void openSession()
{
    info_gui = false;

    std::string session_file_name = "";
    session_file_name = mandeye::fd::OpenFileDialogOneFile("Open session file", mandeye::fd::Session_filter);
    std::cout << "Session file: '" << session_file_name << "'" << std::endl;

    if (session_file_name.size() > 0)
    {
        session.load(fs::path(session_file_name).string(), false, 0.0, 0.0, 0.0, false);
        index_rendered_points_local = 0;

        std::string newTitle = winTitle + " - " + truncPath(session_file_name);
        glutSetWindowTitle(newTitle.c_str());
    }
}

void display()
{
    ImGuiIO& io = ImGui::GetIO();

    view_kbd_shortcuts();

    if (io.KeyCtrl && ImGui::IsKeyPressed('O', false))
        openSession();

    if (session.point_clouds_container.point_clouds.size() > 0)
    {
        if (io.KeyCtrl && ImGui::IsKeyPressed('N'))
            show_neighbouring_scans = !show_neighbouring_scans;
        if (ImGui::IsKeyPressed(ImGuiKey_UpArrow, true))
            offset_intensity += 0.01;
        if (ImGui::IsKeyPressed(ImGuiKey_DownArrow, true))
            offset_intensity -= 0.01;
        if (offset_intensity < 0) {
            offset_intensity = 0;
        }
        if (offset_intensity > 1) {
            offset_intensity = 1;
        }

        if ((!io.KeyCtrl && !io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_RightArrow, true))
            || ImGui::IsKeyPressed(ImGuiKey_PageUp, true)
            || ImGui::IsKeyPressed('+', true)
            )
            index_rendered_points_local += 1;
        if ((!io.KeyCtrl && !io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_LeftArrow, true))
            || ImGui::IsKeyPressed(ImGuiKey_PageDown, true)
            || ImGui::IsKeyPressed('-', true)
            )
            index_rendered_points_local -= 1;

        if (index_rendered_points_local < 0)
            index_rendered_points_local = 0;
        if (index_rendered_points_local >= session.point_clouds_container.point_clouds.size())
            index_rendered_points_local = session.point_clouds_container.point_clouds.size() - 1;
    }

    updateCameraTransition();

    glViewport(0, 0, (GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);

    if (is_ortho)
    {

        glOrtho(-camera_ortho_xy_view_zoom, camera_ortho_xy_view_zoom,
                -camera_ortho_xy_view_zoom / ratio,
                camera_ortho_xy_view_zoom / ratio, -100000, 100000);

        glm::mat4 proj = glm::orthoLH_ZO<float>(-camera_ortho_xy_view_zoom, camera_ortho_xy_view_zoom,
                                                -camera_ortho_xy_view_zoom / ratio,
                                                camera_ortho_xy_view_zoom / ratio, -100, 100);

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
        pose_tb.ka = -camera_ortho_xy_view_rotation_angle_deg * M_PI / 180.0;
        auto m = affine_matrix_from_pose_tait_bryan(pose_tb);

        Eigen::Vector3d v_t = m * v;

        gluLookAt(v_eye_t.x(), v_eye_t.y(), v_eye_t.z(),
                  v_center_t.x(), v_center_t.y(), v_center_t.z(),
                  v_t.x(), v_t.y(), v_t.z());
        glm::mat4 lookat = glm::lookAt(glm::vec3(v_eye_t.x(), v_eye_t.y(), v_eye_t.z()),
                                       glm::vec3(v_center_t.x(), v_center_t.y(), v_center_t.z()),
                                       glm::vec3(v_t.x(), v_t.y(), v_t.z()));
        std::copy(&lookat[0][0], &lookat[3][3], m_ortho_gizmo_view);
    }

    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    Eigen::Affine3f viewLocal = Eigen::Affine3f::Identity();

    if (!is_ortho)
    {
        reshape((GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);

        Eigen::Affine3f viewTranslation = Eigen::Affine3f::Identity();
        viewTranslation.translate(rotation_center);
        viewLocal.translate(Eigen::Vector3f(translate_x, translate_y, translate_z));
        viewLocal.rotate(Eigen::AngleAxisf(M_PI * rotate_x / 180.f, Eigen::Vector3f::UnitX()));
        viewLocal.rotate(Eigen::AngleAxisf(M_PI * rotate_y / 180.f, Eigen::Vector3f::UnitZ()));

        Eigen::Affine3f viewTranslation2 = Eigen::Affine3f::Identity();
        viewTranslation2.translate(-rotation_center);

        Eigen::Affine3f result = viewTranslation * viewLocal * viewTranslation2;

        glLoadMatrixf(result.matrix().data());
        /*      glTranslatef(translate_x, translate_y, translate_z);
              glRotatef(rotate_x, 1.0, 0.0, 0.0);
              glRotatef(rotate_y, 0.0, 0.0, 1.0);*/
    }
    else
    {
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
    }

    if (ImGui::GetIO().KeyCtrl)
    {
        glBegin(GL_LINES);
        glColor3f(1.f, 1.f, 1.f);
        glVertex3fv(rotation_center.data());
        glVertex3f(rotation_center.x() + 1.f, rotation_center.y(), rotation_center.z());
        glVertex3fv(rotation_center.data());
        glVertex3f(rotation_center.x() - 1.f, rotation_center.y(), rotation_center.z());
        glVertex3fv(rotation_center.data());
        glVertex3f(rotation_center.x(), rotation_center.y() - 1.f, rotation_center.z());
        glVertex3fv(rotation_center.data());
        glVertex3f(rotation_center.x(), rotation_center.y() + 1.f, rotation_center.z());
        glVertex3fv(rotation_center.data());
        glVertex3f(rotation_center.x(), rotation_center.y(), rotation_center.z() - 1.f);
        glVertex3fv(rotation_center.data());
        glVertex3f(rotation_center.x(), rotation_center.y(), rotation_center.z() + 1.f);
        glEnd();
    }

    if (show_axes)
    {
        glBegin(GL_LINES);
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(1, 0.0f, 0.0f);

        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 1, 0.0f);

        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 1);
        glEnd();
    }

    if (index_rendered_points_local >= 0 && index_rendered_points_local < session.point_clouds_container.point_clouds[index_rendered_points_local].points_local.size())
    {
        double max_intensity = 0.0;
        for (int i = 0; i < session.point_clouds_container.point_clouds[index_rendered_points_local].intensities.size(); i++)
        {
            if (session.point_clouds_container.point_clouds[index_rendered_points_local].intensities[i] > max_intensity)
            {
                max_intensity = session.point_clouds_container.point_clouds[index_rendered_points_local].intensities[i];
            }
        }

        Eigen::Affine3d pose = session.point_clouds_container.point_clouds[index_rendered_points_local].m_pose;
        pose(0, 3) = 0.0;
        pose(1, 3) = 0.0;
        pose(2, 3) = 0.0;

        glPointSize(point_size);
        glBegin(GL_POINTS);
        for (int i = 0; i < session.point_clouds_container.point_clouds[index_rendered_points_local].points_local.size(); i++)
        {
            glColor3f(session.point_clouds_container.point_clouds[index_rendered_points_local].intensities[i] / max_intensity + offset_intensity, 0.0, 1.0 - session.point_clouds_container.point_clouds[index_rendered_points_local].intensities[i] / max_intensity + offset_intensity);

            Eigen::Vector3d p(session.point_clouds_container.point_clouds[index_rendered_points_local].points_local[i].x(),
                              session.point_clouds_container.point_clouds[index_rendered_points_local].points_local[i].y(),
                              session.point_clouds_container.point_clouds[index_rendered_points_local].points_local[i].z());
            p = pose * p;
            glVertex3f(p.x(), p.y(), p.z());
        }
        glEnd();

        if (show_neighbouring_scans){
            //pc_neigbouring_color

            glColor3f(pc_neigbouring_color.x, pc_neigbouring_color.y, pc_neigbouring_color.z);

            glPointSize(point_size);
            glBegin(GL_POINTS);
            for (int index = index_rendered_points_local - 20; index <= index_rendered_points_local + 20; index +=5){
                if (index != index_rendered_points_local && index >= 0 && index < session.point_clouds_container.point_clouds.size()){
                    Eigen::Affine3d pose = session.point_clouds_container.point_clouds[index].m_pose;
                    Eigen::Affine3d pose_offset = session.point_clouds_container.point_clouds[index_rendered_points_local].m_pose;

                    pose(0, 3) -= pose_offset(0, 3);
                    pose(1, 3) -= pose_offset(1, 3);
                    pose(2, 3) -= pose_offset(2, 3);

                    for (int i = 0; i < session.point_clouds_container.point_clouds[index].points_local.size(); i++)
                    {
                        Eigen::Vector3d p(session.point_clouds_container.point_clouds[index].points_local[i].x(),
                                            session.point_clouds_container.point_clouds[index].points_local[i].y(),
                                            session.point_clouds_container.point_clouds[index].points_local[i].z());
                        p = pose * p;
                        glVertex3f(p.x(), p.y(), p.z());
                    }
                }
            }
            glEnd();
        }
    }

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();

    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::Button("Open session"))
			openSession();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Select session to open for analyze (Ctrl+O)");

        ImGui::SameLine();
        ImGui::Dummy(ImVec2(20, 0));
        ImGui::SameLine();

        if (ImGui::BeginMenu("View"))
        {
            if (session.point_clouds_container.point_clouds.size() > 0)
            {
                ImGui::PushItemWidth(ImGuiNumberWidth);
                ImGui::InputFloat("Offset intensity", &offset_intensity, 0.01, 0.1, "%.2f");
                if (offset_intensity < 0) {
                    offset_intensity = 0;
                }
                if (offset_intensity > 1) {
                    offset_intensity = 1;
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("keyboard up/down arrows");

                auto tmp = point_size;
                ImGui::InputInt("Points size", &point_size);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("keyboard 1-9 keys");
                ImGui::PopItemWidth();
                if (point_size < 1)
                    point_size = 1;
                if (point_size > 10)
                    point_size = 10;

                if (tmp != point_size)
                {
                    for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
                    {
                        session.point_clouds_container.point_clouds[i].point_size = point_size;
                    }
                }

                ImGui::MenuItem("Show neighbouring scans", "Ctrl+N", &show_neighbouring_scans);
                ImGui::BeginDisabled(!show_neighbouring_scans);
                {
                    ImGui::ColorEdit3("Neigbouring color", (float*)&pc_neigbouring_color, ImGuiColorEditFlags_NoInputs);
                }
                ImGui::EndDisabled();

                ImGui::Separator();
            }

            ImGui::MenuItem("Show axes", "key X", &show_axes);

            ImGui::Separator();

            ImGui::MenuItem("Show compass/ruler", "key C", &compass_ruler);

            //ImGui::MenuItem("show_covs", nullptr, &show_covs);

            ImGui::Separator();

            ImGui::ColorEdit3("Background color", (float*)&clear_color, ImGuiColorEditFlags_NoInputs);

            ImGui::EndMenu();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Scene view relevant parameters");
 
        camMenu();

        ImGui::SameLine();
        ImGui::Dummy(ImVec2(20, 0));
        ImGui::SameLine();

        if (session.point_clouds_container.point_clouds.size() > 0)
        {
            int tempIndex = index_rendered_points_local;
            ImGui::Text("index_rendered_points_local: ");
            ImGui::SameLine();
            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::SliderInt("##irpls", &tempIndex, 0, static_cast<int>(session.point_clouds_container.point_clouds.size() - 1));
            ImGui::SameLine();
            ImGui::InputInt("##irpli", &tempIndex, 1, 10);
            ImGui::PopItemWidth();
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                ImGui::Text(session.point_clouds_container.point_clouds[index_rendered_points_local].file_name.c_str());
                double ts = session.point_clouds_container.point_clouds[index_rendered_points_local].timestamps[0] / 1e9;
                ImGui::Text((std::string("Timestamp: ") + std::to_string(ts)).c_str());
                ImGui::EndTooltip();
            }

            if ((tempIndex >= 0) && (tempIndex < session.point_clouds_container.point_clouds.size()))
            {
                index_rendered_points_local = tempIndex;
            }
        }

        ImGui::SameLine(ImGui::GetWindowWidth() - ImGui::CalcTextSize("Info").x - ImGui::GetStyle().ItemSpacing.x * 2 - ImGui::GetStyle().FramePadding.x * 2);

        ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4, 2));
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetStyleColorVec4(ImGuiCol_HeaderHovered));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImGui::GetStyleColorVec4(ImGuiCol_Header));
        if (ImGui::SmallButton("Info"))
        {
            info_gui = !info_gui;
        }
        ImGui::PopStyleVar(2);
        ImGui::PopStyleColor(3);


        ImGui::EndMainMenuBar();
    }

    info_window(infoLines, &info_gui);

    if (compass_ruler)
        drawMiniCompassWithRuler(viewLocal, fabs(translate_z), clear_color);

    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    glutSwapBuffers();
    glutPostRedisplay();
}

bool initGL(int *argc, char **argv)
{
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowSize(window_width, window_height);
    glutCreateWindow(winTitle.c_str());

    #ifdef _WIN32
        HWND hwnd = FindWindow(NULL, winTitle.c_str()); // The window title must match exactly
        HINSTANCE hInstance = GetModuleHandle(NULL);
        SendMessage(hwnd, WM_SETICON, ICON_BIG, (LPARAM)LoadIcon(hInstance, MAKEINTRESOURCE(IDI_ICON1)));
        SendMessage(hwnd, WM_SETICON, ICON_SMALL, (LPARAM)LoadIcon(hInstance, MAKEINTRESOURCE(IDI_ICON1)));
    #endif

    glutDisplayFunc(display);
    glutMotionFunc(motion);

    // default initialization
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glEnable(GL_DEPTH_TEST);

    // viewport
    glViewport(0, 0, window_width, window_height);

    // projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (GLfloat)window_width / (GLfloat)window_height, 0.01, 10000.0);
    glutReshapeFunc(reshape);
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls

    ImGui::StyleColorsDark();
    ImGui_ImplGLUT_Init();
    ImGui_ImplGLUT_InstallFuncs();
    ImGui_ImplOpenGL2_Init();
    return true;
}

void mouse(int glut_button, int state, int x, int y)
{
    ImGuiIO &io = ImGui::GetIO();
    io.MousePos = ImVec2((float)x, (float)y);

    int button = -1;
    if (glut_button == GLUT_LEFT_BUTTON)
        button = 0;
    if (glut_button == GLUT_RIGHT_BUTTON)
        button = 1;
    if (glut_button == GLUT_MIDDLE_BUTTON)
        button = 2;
    if (button != -1 && state == GLUT_DOWN)
        io.MouseDown[button] = true;
    if (button != -1 && state == GLUT_UP)
        io.MouseDown[button] = false;

    static int glutMajorVersion = glutGet(GLUT_VERSION) / 10000;
    if (state == GLUT_DOWN && (glut_button == 3 || glut_button == 4) &&
        glutMajorVersion < 3)
    {
        wheel(glut_button, glut_button == 3 ? 1 : -1, x, y);
    }

    if (!io.WantCaptureMouse)
    {
        if (glut_button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN && io.KeyCtrl)
        {
        }

        if (state == GLUT_DOWN)
        {
            mouse_buttons |= 1 << glut_button;
        }
        else if (state == GLUT_UP)
        {
            mouse_buttons = 0;
        }
        mouse_old_x = x;
        mouse_old_y = y;
    }
}

int main(int argc, char *argv[])
{
    initGL(&argc, argv);
    glutDisplayFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutMouseWheelFunc(wheel);
    glutSpecialFunc(specialDown);
    glutSpecialUpFunc(specialUp);
    glutMainLoop();

    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGLUT_Shutdown();

    ImGui::DestroyContext();
    return 0;
}