#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <imgui_internal.h>

#include <ImGuizmo.h>

// clang-format off
#include <GL/glew.h>
#include <GL/freeglut.h>
// clang-format on

#include <Eigen/Eigen>

#include <laz_wrapper.h>
#include <odo_with_gnss_fusion.h>
#include <project_settings.h>
#include <roi_exporter.h>
#include <single_trajectory_viewer.h>
#include <transformations.h>

#include <HDMapping/Version.hpp>

Eigen::Vector3d GLWidgetGetOGLPos(int x, int y, float picking_plane_height);

static bool show_demo_window = true;
static bool show_another_window = false;
// static ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
const uint32_t window_width = 800;
const uint32_t window_height = 600;
int mouse_old_x, mouse_old_y;
int mouse_buttons = 0;
bool gui_mouse_down{ false };
float mouse_sensitivity = 1.0;
bool show_axes = false;

float zoom = 1.0;
float camera_eye_height = 10;
//----
ProjectSettings project_setings;
OdoWithGnssFusion odo_with_gnss_fusion;
SingleTrajectoryViewer single_trajectory_viewer;
RoiExporter roi_exporter;
CommonData common_data;
LazWrapper laz_wrapper;

void reshape(int w, int h);

void my_display_code()
{
    // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear
    // ImGui!).
    if (show_demo_window)
        ImGui::ShowDemoWindow(&show_demo_window);

    // 2. Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
    {
        static float f = 0.0f;
        static int counter = 0;

        ImGui::Begin("Hello, world!"); // Create a window called "Hello, world!" and append into it.

        ImGui::Text("This is some useful text."); // Display some text (you can use a format strings too)
        ImGui::Checkbox("Demo Window", &show_demo_window); // Edit bools storing our window open/close state
        ImGui::Checkbox("Another Window", &show_another_window);

        ImGui::SliderFloat("float", &f, 0.0f, 1.0f); // Edit 1 float using a slider from 0.0f to 1.0f
        ImGui::ColorEdit3("clear color", (float*)&project_setings.clear_color); // Edit 3 floats representing a color

        if (ImGui::Button("Button")) // Buttons return true when clicked (most widgets return true when edited/activated)
            counter++;
        ImGui::SameLine();
        ImGui::Text("counter = %d", counter);

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::End();
    }

    // 3. Show another simple window.
    if (show_another_window)
    {
        ImGui::Begin("Another Window", &show_another_window); // Pass a pointer to our bool variable (the window will have a closing button
                                                              // that will clear the bool when clicked)
        ImGui::Text("Hello from another window!");
        if (ImGui::Button("Close Me"))
            show_another_window = false;
        ImGui::End();
    }
}

void display()
{
    ImGuiIO& io = ImGui::GetIO();
    reshape((GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
    glViewport(0, 0, (GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
    glClearColor(
        project_setings.clear_color.x * project_setings.clear_color.w,
        project_setings.clear_color.y * project_setings.clear_color.w,
        project_setings.clear_color.z * project_setings.clear_color.w,
        project_setings.clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    if (common_data.is_ortho)
    {
        Eigen::Vector3d v_eye_t(common_data.translate_x * mouse_sensitivity, common_data.translate_y * mouse_sensitivity, 10);
        Eigen::Vector3d v_center_t(common_data.translate_x * mouse_sensitivity, common_data.translate_y * mouse_sensitivity, 0);
        Eigen::Vector3d v_t(0, 1, 0);

        gluLookAt(v_eye_t.x(), v_eye_t.y(), v_eye_t.z(), v_center_t.x(), v_center_t.y(), v_center_t.z(), v_t.x(), v_t.y(), v_t.z());
    }
    else
    {
        Eigen::Vector3d v(-zoom, 0, 0);

        TaitBryanPose pose_tb;
        pose_tb.ka = common_data.rotate_y;
        // pose_tb.fi = common_data.rotate_y;
        // pose_tb.ka = common_data.rotate_y;
        Eigen::Affine3d m = affine_matrix_from_pose_tait_bryan(pose_tb);
        Eigen::Vector3d vt = m * v;

        Eigen::Vector3d v_eye_t(common_data.roi(0, 3) + vt.x(), common_data.roi(1, 3) + vt.y(), common_data.roi(2, 3) + camera_eye_height);

        Eigen::Vector3d v_center_t(common_data.roi(0, 3), common_data.roi(1, 3), common_data.roi(2, 3) + camera_eye_height);
        Eigen::Vector3d v_t(0, 0, 1);

        gluLookAt(v_eye_t.x(), v_eye_t.y(), v_eye_t.z(), v_center_t.x(), v_center_t.y(), v_center_t.z(), v_t.x(), v_t.y(), v_t.z());
        // glTranslatef(common_data.translate_x, common_data.translate_y, common_data.translate_z);
        // glRotatef(common_data.rotate_x, 1.0, 0.0, 0.0);
        // glRotatef(common_data.rotate_y, 0.0, 0.0, 1.0);

        // common_data.roi.
    }

    if (show_axes)
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

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();
    ImGui::NewFrame();

    // my_display_code();
    project_setings.imgui(odo_with_gnss_fusion, laz_wrapper.sectors, roi_exporter.rois_with_constraints, common_data);
    if (common_data.odo_with_gnss_fusion)
    {
        odo_with_gnss_fusion.imgui(common_data);
    }
    else if (common_data.single_trajectory_viewer)
    {
        single_trajectory_viewer.imgui(common_data);
    }
    else
    {
        if (common_data.roi_exorter)
            roi_exporter.imgui(common_data, project_setings, laz_wrapper.sectors);
        if (common_data.laz_wrapper)
            laz_wrapper.imgui(common_data, project_setings);
    }

    if (common_data.odo_with_gnss_fusion)
    {
        odo_with_gnss_fusion.render();
    }
    else if (common_data.single_trajectory_viewer)
    {
        single_trajectory_viewer.render();
    }
    else
    {
        project_setings.render(roi_exporter.rois_with_constraints);
        if (common_data.roi_exorter)
            roi_exporter.render(common_data);
        if (common_data.laz_wrapper)
            laz_wrapper.render(common_data);
    }

    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    glutSwapBuffers();
    glutPostRedisplay();
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
        gui_mouse_down = mouse_buttons > 0;
        if (mouse_buttons & 1 && !common_data.is_ortho)
        {
            common_data.rotate_x += dy * 0.001f * mouse_sensitivity;
            common_data.rotate_y += dx * 0.001f * mouse_sensitivity;
            camera_eye_height += dy * 0.1f * mouse_sensitivity;
        }
        else if (mouse_buttons & 4)
        {
            common_data.translate_z += dy * 1.0f * mouse_sensitivity;

            zoom += dy * 0.1f * mouse_sensitivity;
            if (zoom < 0.1)
            {
                zoom = 0.1;
            }
        }
        else if (mouse_buttons & 3)
        {
            common_data.translate_x += dx * 0.5f * mouse_sensitivity;
            common_data.translate_y -= dy * 0.5f * mouse_sensitivity;
        }
        mouse_old_x = x;
        mouse_old_y = y;
    }
    glutPostRedisplay();
}

void mouse(int glut_button, int state, int x, int y)
{
    ImGuiIO& io = ImGui::GetIO();
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

    if (!io.WantCaptureMouse)
    {
        if (state == GLUT_DOWN)
        {
            mouse_buttons |= 1 << glut_button;
            if (common_data.is_ortho && button == 0)
            {
                common_data.roi = GLWidgetGetOGLPos(x, y, 0.0);
            }
        }
        else if (state == GLUT_UP)
        {
            mouse_buttons = 0;
        }
        mouse_old_x = x;
        mouse_old_y = y;
    }
}

void reshape(int w, int h)
{
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if (!common_data.is_ortho)
    {
        gluPerspective(60.0, (GLfloat)w / (GLfloat)h, 0.01, 100000.0);
    }
    else
    {
        glOrtho(
            -common_data.translate_z,
            common_data.translate_z,
            -common_data.translate_z * (float)h / float(w),
            common_data.translate_z * float(h) / float(w),
            -100000,
            100000);
    }
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

bool initGL(int* argc, char** argv)
{
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowSize(window_width, window_height);
    glutCreateWindow("hd_mapper " HDMAPPING_VERSION_STRING);
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
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls

    ImGui::StyleColorsDark();
    ImGui_ImplGLUT_Init();
    ImGui_ImplGLUT_InstallFuncs();
    ImGui_ImplOpenGL2_Init();
    return true;
}

int main(int argc, char** argv)
{
    initGL(&argc, argv);
    glutDisplayFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutMainLoop();

    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGLUT_Shutdown();

    ImGui::DestroyContext();
    return 0;
}

float distanceToPlane(const Plane& plane, const Eigen::Vector3d& p)
{
    return (plane.a * p.x() + plane.b * p.y() + plane.c * p.z() + plane.d);
}

Eigen::Vector3d rayIntersection(const LaserBeam& laser_beam, const Plane& plane)
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

Eigen::Vector3d GLWidgetGetOGLPos(int x, int y, float picking_plane_height /*, const ObservationPicking& observation_picking*/)
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

    laser_beam.direction.normalize();

    Plane pl;

    pl.a = 0;
    pl.b = 0;
    pl.c = 1;
    pl.d = -picking_plane_height;

    Eigen::Vector3d pos = rayIntersection(laser_beam, pl);

    std::cout << "intersection: " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;

    return pos;
}