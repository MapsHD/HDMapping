#include <pch/pch.h>

// clang-format off
#include <GL/glew.h>
#include <GL/freeglut.h>
// clang-format on

#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <imgui_internal.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "utils.hpp"

#include <HDMapping/Version.hpp>

#ifdef _WIN32
#include <shellapi.h>
#include <windows.h>
#endif

///////////////////////////////////////////////////////////////////////////////////

int viewer_decimate_point_cloud = 1000;

int mouse_old_x, mouse_old_y;
int mouse_buttons = 0;
float mouse_sensitivity = 1.0;

bool is_ortho = false;
bool lock_z = false;
bool show_axes = true;
ImVec4 bg_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
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

// General shortcuts applicable to any app
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
        ImGuiIO& io = ImGui::GetIO();
        float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);

        glOrtho(
            -camera_ortho_xy_view_zoom,
            camera_ortho_xy_view_zoom,
            -camera_ortho_xy_view_zoom / ratio,
            camera_ortho_xy_view_zoom / ratio,
            -100000,
            100000);
        // glOrtho(-translate_z, translate_z, -translate_z * (float)h / float(w), translate_z * float(h) / float(w), -10000, 10000);
    }
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

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
                    dx * (camera_ortho_xy_view_zoom / (GLsizei)io.DisplaySize.x * 2),
                    dy * (camera_ortho_xy_view_zoom / (GLsizei)io.DisplaySize.y * 2 / ratio),
                    0);
                TaitBryanPose pose_tb;
                pose_tb.px = 0.0;
                pose_tb.py = 0.0;
                pose_tb.pz = 0.0;
                pose_tb.om = 0.0;
                pose_tb.fi = 0.0;
                pose_tb.ka = (rotate_x + rotate_y)*M_PI / 180.0;
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
    glutPostRedisplay();
}

ImGuiKey keyToImGuiKey(unsigned char key)
{
    if (key >= 'a' && key <= 'z')
        return ImGuiKey(ImGuiKey_A + (key - 'a'));
    if (key >= 'A' && key <= 'Z')
        return ImGuiKey(ImGuiKey_A + (key - 'A'));
    if (key >= '0' && key <= '9')
        return ImGuiKey(ImGuiKey_0 + (key - '0'));
    switch (key)
    {
    case 27:
        return ImGuiKey_Escape;
    case 13:
        return ImGuiKey_Enter;
    case 32:
        return ImGuiKey_Space;
    default:
        return ImGuiKey_None;
    }
}

void keyboardDown(unsigned char key, int x, int y)
{
    ImGuiIO& io = ImGui::GetIO();

    int mods = glutGetModifiers();
    // Update modifier keys using the new API
    io.AddKeyEvent(ImGuiMod_Ctrl, (mods & GLUT_ACTIVE_CTRL) != 0);
    io.AddKeyEvent(ImGuiMod_Shift, (mods & GLUT_ACTIVE_SHIFT) != 0);
    io.AddKeyEvent(ImGuiMod_Alt, (mods & GLUT_ACTIVE_ALT) != 0);

    // Handle Ctrl+letter (ASCII 1-26) (FreeGLUT “lost key” problem)
    if ((mods & GLUT_ACTIVE_CTRL) && (key >= 1 && key <= 26))
        key = 'A' + (key - 1);

    io.AddKeyEvent(keyToImGuiKey(key), true);

    // forward to ImGui GLUT backend
    ImGui_ImplGLUT_KeyboardFunc(key, x, y);

    // std::cout << "Down key: " << key << ", mod: " << mods << std::endl;
}

void keyboardUp(unsigned char key, int x, int y)
{
    ImGuiIO& io = ImGui::GetIO();

    int mods = glutGetModifiers();
    // Update modifier keys using the new API
    io.AddKeyEvent(ImGuiMod_Ctrl, (mods & GLUT_ACTIVE_CTRL) != 0);
    io.AddKeyEvent(ImGuiMod_Shift, (mods & GLUT_ACTIVE_SHIFT) != 0);
    io.AddKeyEvent(ImGuiMod_Alt, (mods & GLUT_ACTIVE_ALT) != 0);

    // Handle Ctrl+letter (ASCII 1-26) (FreeGLUT “lost key” problem)
    if ((mods & GLUT_ACTIVE_CTRL) && (key >= 1 && key <= 26))
        key = 'A' + (key - 1);

    io.AddKeyEvent(keyToImGuiKey(key), false);

    ImGui_ImplGLUT_KeyboardUpFunc(key, x, y);

    // std::cout << "Up key: " << key << ", mod: " << mods << std::endl;
}

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

        // ImGui::DockBuilderDockWindow("Settings", dock_id_left);
        ImGui::DockBuilderDockWindow("Console", dock_id_bottom);
        ImGui::DockBuilderFinish(dockspace_id);
    }

    ImGui::End();
}

bool initGL(int* argc, char** argv, const std::string& winTitle, void (*display)(), void (*mouse)(int, int, int, int))
{
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowSize(window_width, window_height);
    if (glutCreateWindow(winTitle.c_str()) <= 0)
        return false; // window creation failed

#ifdef _WIN32
    HWND hwnd = FindWindow(NULL, winTitle.c_str()); // The window title must match exactly
    if (!hwnd)
        return false; // couldn't find window handle
    HINSTANCE hInstance = GetModuleHandle(NULL);
    SendMessage(hwnd, WM_SETICON, ICON_BIG, (LPARAM)LoadIcon(hInstance, MAKEINTRESOURCE(IDI_ICON1)));
    SendMessage(hwnd, WM_SETICON, ICON_SMALL, (LPARAM)LoadIcon(hInstance, MAKEINTRESOURCE(IDI_ICON1)));
#endif

    while (glGetError() != GL_NO_ERROR)
    {
    } // Clear any existing GL errors from platform or driver init

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
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard // Enable Keyboard Controls
        | ImGuiConfigFlags_NavEnableGamepad | ImGuiConfigFlags_DockingEnable;
    io.ConfigDockingWithShift = true;
    io.MouseDrawCursor = false; // use OS cursor (for future ImGUI update to 1.93+)

    ImGui::StyleColorsDark();
    ImGui_ImplGLUT_Init();
    ImGui_ImplOpenGL2_Init();

    ImGui_ImplGLUT_InstallFuncs();

    glutDisplayFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutMouseWheelFunc(wheel);
    glutKeyboardFunc(keyboardDown);
    glutKeyboardUpFunc(keyboardUp);
    // glutSpecialFunc(specialDown);
    // glutSpecialUpFunc(specialUp);

    // check line width range support
    GLfloat range[2];
    glGetFloatv(GL_LINE_WIDTH_RANGE, range);
    if (range[0] == range[1])
    {
        std::cerr << "No line width support in this GPU/driver configuration, range: " << range[0] << " - " << range[1] << std::endl;
        glLineWidthSupport = false;
    }

    return (glGetError() == GL_NO_ERROR);
}

void draw_ellipse(const Eigen::Matrix3d& covar, const Eigen::Vector3d& mean, Eigen::Vector3f color, float nstd)
{
    Eigen::LLT<Eigen::Matrix<double, 3, 3>> cholSolver(covar);
    Eigen::Matrix3d transform = cholSolver.matrixL();

    const double pi = 3.141592;
    const double di = 0.02;
    const double dj = 0.04;
    const double du = di * 2 * pi;
    const double dv = dj * pi;
    glColor3f(color.x(), color.y(), color.z());

    for (double i = 0; i < 1.0; i += di) // horizonal
    {
        for (double j = 0; j < 1.0; j += dj) // vertical
        {
            double u = i * 2 * pi; // 0     to  2pi
            double v = (j - 0.5) * pi; //-pi/2 to pi/2

            const Eigen::Vector3d pp0(cos(v) * cos(u), cos(v) * sin(u), sin(v));
            const Eigen::Vector3d pp1(cos(v) * cos(u + du), cos(v) * sin(u + du), sin(v));
            const Eigen::Vector3d pp2(cos(v + dv) * cos(u + du), cos(v + dv) * sin(u + du), sin(v + dv));
            const Eigen::Vector3d pp3(cos(v + dv) * cos(u), cos(v + dv) * sin(u), sin(v + dv));
            Eigen::Vector3d tp0 = transform * (nstd * pp0) + mean;
            Eigen::Vector3d tp1 = transform * (nstd * pp1) + mean;
            Eigen::Vector3d tp2 = transform * (nstd * pp2) + mean;
            Eigen::Vector3d tp3 = transform * (nstd * pp3) + mean;

            glBegin(GL_LINE_LOOP);
            glVertex3dv(tp0.data());
            glVertex3dv(tp1.data());
            glVertex3dv(tp2.data());
            glVertex3dv(tp3.data());
            glEnd();
        }
    }
}

void showAxes()
{
    if (show_axes || ImGui::GetIO().KeyCtrl) // rotation center axes
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

    if (show_axes || ImGui::GetIO().KeyCtrl) // origin axes
    {
        glBegin(GL_LINES);
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(100, 0.0f, 0.0f);

        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 100, 0.0f);

        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 100);
        glEnd();
    }
}

void updateCameraTransition()
{
    if (!camera_transition_active)
        return;

    // Smoothly approach the target using exponential interpolation
    // const float t = std::min(deltaTime * camera_transition_speed, 1.0f);

    // Ease out curve, slow down effect
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

    // making sure exact coordinates are reached at the end of the transition
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
    // commit new rotation center
    rotation_center = new_rotation_center;
    // reset transition
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
        mouse_sensitivity = fabs(translate_z) / 100; // 1 for translate_z 50 (default zoom)

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

        // Reset accumulated scroll if user stops scrolling
        if (ImGui::GetTime() - scroll_hint_lastT > 1)
            scroll_hint_active = false;
    }
}

void view_kbd_shortcuts()
{
    ImGuiIO& io = ImGui::GetIO();

    if (io.WantCaptureKeyboard)
        return;

    // translate camera
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

    // rotate camera
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

    // only checking for single key press (no modifiers) from this point
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

void cor_window()
{
    // temporary location for new rotation center (minimal impact change in all apps)
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
        draw_list->AddLine(ImVec2(pos.x, pos.y + size.y), ImVec2(pos.x + size.x, pos.y + size.y), ImColor(color));
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

            // Insert a "fake" type row when type changes
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
                // Normal row
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
            else if (line.rfind("https://", 0) == 0) // starts with "https://"
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
                    // Query versions info
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

void drawMiniCompassWithRuler()
{
    const ImVec2 compassSize = ImVec2(200, 200);

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
    float rawUnit = 0.1f * fabs(translate_z); // adjust factor to taste
    float base = pow(10.0f, floor(log10(rawUnit)));
    float normalized = rawUnit / base;
    float niceUnit;
    if (normalized < 2.0f)
        niceUnit = 1.0f;
    else if (normalized < 5.0f)
        niceUnit = 2.0f;
    else
        niceUnit = 5.0f;
    float worldLength = niceUnit * base;

    float scale = miniAxisLength / rawUnit; // convert scroll units to visual units
    float axisDrawLength = worldLength * scale; // final axis length in mini compass

    // Draw axes
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(axisDrawLength, 0, 0); // X
    glColor3f(0, 1, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, axisDrawLength, 0); // Y
    glColor3f(0, 0, 1);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, axisDrawLength); // Z
    glEnd();

    // Draw axis labels
    drawLabel(axisDrawLength + 0.2f, 0, 0, "X (long.)", 1, 0, 0);
    drawLabel(0, axisDrawLength + 0.2f, 0, "Y (lat.)", 0, 1, 0);
    drawLabel(0, 0, axisDrawLength + 0.2f, "Z (vert.)", 0, 0, 1);

    // End scale label
    char label[24];
    if (worldLength >= 1000.0f)
        sprintf(label, "%.0f [km]", worldLength / 1000.0f);
    else if (worldLength >= 1.0f)
        sprintf(label, "%.0f [m]", worldLength);
    else if (worldLength >= 0.01f)
        sprintf(label, "%.0f [cm]", worldLength * 100.0f);
    else
        sprintf(label, "<1 [cm]");

    // drawLabel(axes[2].x() + 0.2f, axes[2].y() + 0.4f, axes[2].z() - 0.2f, label,
    //     colors[2][0], colors[2][1], colors[2][2]);
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

double distance_point_to_line(const Eigen::Vector3d& point, const LaserBeam& line)
{
    Eigen::Vector3d AP = point - line.position;

    double dist = (AP.cross(line.direction)).norm();
    return dist;
}

void getClosestTrajectoryPoint(Session& session, int x, int y, bool gcpPicking, int& picked_index)
{
    picked_index = -1;

    const auto laser_beam = GetLaserBeam(x, y);
    double min_distance = std::numeric_limits<double>::max();
    int index_i = -1;
    int index_j = -1;

    for (int i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
    {
        for (int j = 0; j < session.point_clouds_container.point_clouds[i].local_trajectory.size(); j++)
        {
            const auto& p = session.point_clouds_container.point_clouds[i].local_trajectory[j].m_pose.translation();
            Eigen::Vector3d vp = session.point_clouds_container.point_clouds[i].m_pose * p;

            double dist = distance_point_to_line(vp, laser_beam);

            if (dist < min_distance)
            {
                min_distance = dist;
                index_i = i;
                index_j = j;

                new_rotation_center.x() = static_cast<float>(vp.x());
                new_rotation_center.y() = static_cast<float>(vp.y());
                new_rotation_center.z() = static_cast<float>(vp.z());

#if WITH_GUI == 1
                if (gcpPicking)
                {
                    session.ground_control_points.picking_mode_index_to_node_inner = index_i;
                    session.ground_control_points.picking_mode_index_to_node_outer = index_j;
                }
#endif

                picked_index = index_i;

                // if (picking_mode_index_to_node_inner != -1 && picking_mode_index_to_node_outer != -1)
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

void getClosestTrajectoriesPoint(
    std::vector<Session>& sessions,
    int x,
    int y,
    const int first_session_index,
    const int second_session_index,
    const int number_visible_sessions,
    int& index_loop_closure_source,
    int& index_loop_closure_target,
    bool KeyShift)
{
    const auto laser_beam = GetLaserBeam(x, y);
    double min_distance = std::numeric_limits<double>::max();

    if (number_visible_sessions == 1)
    {
        // std::cout << "first_session_index " << first_session_index << std::endl;
        for (size_t i = 0; i < sessions[first_session_index].point_clouds_container.point_clouds.size(); i++)
        {
            for (size_t j = 0; j < sessions[first_session_index].point_clouds_container.point_clouds[i].local_trajectory.size(); j++)
            {
                const auto& p =
                    sessions[first_session_index].point_clouds_container.point_clouds[i].local_trajectory[j].m_pose.translation();
                Eigen::Vector3d vp = sessions[first_session_index].point_clouds_container.point_clouds[i].m_pose * p;

                double dist = distance_point_to_line(vp, laser_beam);

                if (dist < min_distance)
                {
                    min_distance = dist;

                    if (number_visible_sessions == 1)
                    {
                        if (!KeyShift) // io.KeyCtrl
                        {
                            new_rotation_center.x() = static_cast<float>(vp.x());
                            new_rotation_center.y() = static_cast<float>(vp.y());
                            new_rotation_center.z() = static_cast<float>(vp.z());
                            index_loop_closure_source = i;
                        }
                        else // io.KeyShift
                        {
                            index_loop_closure_target = i;
                        }
                    }
                }
            }
        }
    }
    else if (number_visible_sessions == 2)
    {
        int index = -1;
        if (!KeyShift) // io.KeyCtrl
        {
            index = first_session_index;
        }
        else // io.KeyShift
        {
            index = second_session_index;
        }
        // std::cout << "first_session_index " << first_session_index << std::endl;
        for (size_t i = 0; i < sessions[index].point_clouds_container.point_clouds.size(); i++)
        {
            for (size_t j = 0; j < sessions[index].point_clouds_container.point_clouds[i].local_trajectory.size(); j++)
            {
                const auto& p = sessions[index].point_clouds_container.point_clouds[i].local_trajectory[j].m_pose.translation();
                Eigen::Vector3d vp = sessions[index].point_clouds_container.point_clouds[i].m_pose * p;

                double dist = distance_point_to_line(vp, laser_beam);

                if (dist < min_distance)
                {
                    min_distance = dist;

                    if (!KeyShift) // io.KeyCtrl
                    {
                        index_loop_closure_source = i;
                        new_rotation_center.x() = static_cast<float>(vp.x());
                        new_rotation_center.y() = static_cast<float>(vp.y());
                        new_rotation_center.z() = static_cast<float>(vp.z());
                    }
                    else // io.KeyShift
                    {
                        index_loop_closure_target = i;
                    }
                }
            }
        }
    }
    else // (number_visible_sessions > 2)
    {
        // std::cout << "first_session_index " << first_session_index << std::endl;
        for (size_t s = 0; s < sessions.size(); s++)
        {
            if (sessions[s].visible)
            {
                for (size_t i = 0; i < sessions[s].point_clouds_container.point_clouds.size(); i++)
                {
                    for (size_t j = 0; j < sessions[s].point_clouds_container.point_clouds[i].local_trajectory.size(); j++)
                    {
                        const auto& p = sessions[s].point_clouds_container.point_clouds[i].local_trajectory[j].m_pose.translation();
                        Eigen::Vector3d vp = sessions[s].point_clouds_container.point_clouds[i].m_pose * p;

                        double dist = distance_point_to_line(vp, laser_beam);

                        if (dist < min_distance)
                        {
                            min_distance = dist;

                            new_rotation_center.x() = static_cast<float>(vp.x());
                            new_rotation_center.y() = static_cast<float>(vp.y());
                            new_rotation_center.z() = static_cast<float>(vp.z());
                        }
                    }
                }
            }
        }
    }

    new_translate_x = -new_rotation_center.x();
    new_translate_y = -new_rotation_center.y();
    new_translate_z = translate_z;
    new_rotate_x = rotate_x;
    new_rotate_y = rotate_y;
    camera_transition_active = true;
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



void updateOrthoView()
{
    // still updating viewLocal for compass
    viewLocal.rotate(Eigen::AngleAxisf((rotate_x + rotate_y) * DEG_TO_RAD, Eigen::Vector3f::UnitZ()));

    ImGuiIO& io = ImGui::GetIO();
    float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);

    

    glOrtho(
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

    gluLookAt(v_eye_t.x(), v_eye_t.y(), v_eye_t.z(), v_center_t.x(), v_center_t.y(), v_center_t.z(), v_t.x(), v_t.y(), v_t.z());
    glm::mat4 lookat = glm::lookAt(
        glm::vec3(v_eye_t.x(), v_eye_t.y(), v_eye_t.z()),
        glm::vec3(v_center_t.x(), v_center_t.y(), v_center_t.z()),
        glm::vec3(v_t.x(), v_t.y(), v_t.z()));
    std::copy(&lookat[0][0], &lookat[3][3], m_ortho_gizmo_view);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}