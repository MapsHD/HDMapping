
#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <ImGuizmo.h>
#include <imgui_internal.h>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <glm/glm/glm.hpp>
#include <glm/glm/gtc/matrix_transform.hpp>

#include <Eigen/Eigen>

#include <transformations.h>

#include <portable-file-dialogs.h>

#include <filesystem>
#include "../lidar_odometry_step_1/lidar_odometry_utils.h"

#include <HDMapping/Version.hpp>

#include <mutex>

#define SAMPLE_PERIOD (1.0 / 200.0)
namespace fs = std::filesystem;

const unsigned int window_width = 800;
const unsigned int window_height = 600;
double camera_ortho_xy_view_zoom = 10;
double camera_ortho_xy_view_shift_x = 0.0;
double camera_ortho_xy_view_shift_y = 0.0;
double camera_mode_ortho_z_center_h = 0.0;
double camera_ortho_xy_view_rotation_angle_deg = 0;
bool is_ortho = false;
bool show_axes = true;
ImVec4 clear_color = ImVec4(0.8f, 0.8f, 0.8f, 1.00f);
ImVec4 pc_color = ImVec4(1.0f, 0.0f, 0.0f, 1.00f);
ImVec4 pc_color2 = ImVec4(0.0f, 0.0f, 1.0f, 1.00f);

int point_size = 1;
Eigen::Vector3f rotation_center = Eigen::Vector3f::Zero();
float translate_x, translate_y = 0.0;
float translate_z = -50.0;
float rotate_x = 0.0, rotate_y = 0.0;
int mouse_old_x, mouse_old_y;
int mouse_buttons = 0;
bool gui_mouse_down{false};
float mouse_sensitivity = 1.0;

float m_ortho_projection[] = {1, 0, 0, 0,
                              0, 1, 0, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1};

float m_ortho_gizmo_view[] = {1, 0, 0, 0,
                              0, 1, 0, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1};

std::vector<std::string> laz_files;
std::vector<std::string> csv_files;
std::vector<std::string> sn_files;
std::string working_directory = "";
std::string imuSnToUse;
std::string working_directory_preview;
double filter_threshold_xy = 0.1;
bool fusionConventionNwu = true;
bool fusionConventionEnu = false;
bool fusionConventionNed = false;
int number_of_points_threshold = 20000;
bool is_init = true;
int index_rendered_points_local = -1;
std::vector<std::vector<Eigen::Vector3d>> all_points_local;
std::vector<std::vector<int>> all_lidar_ids;
std::vector<int> indexes_to_filename;
std::vector<std::string> all_file_names;
LidarOdometryParams params;
int threshold_initial_points = 10000;
std::vector<WorkerData> worker_data;

std::mutex renderPtrLock;
std::vector<Eigen::Affine3d> render_trajectory;
std::vector<std::pair<Eigen::Vector3d, float>> render_pointcloud;

#define THRESHOLD_NR_POSES 20

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
            }
            if (mouse_buttons & 4)
            {
                translate_x += dx * 0.05f * mouse_sensitivity;
                translate_y -= dy * 0.05f * mouse_sensitivity;
            }
        }

        mouse_old_x = x;
        mouse_old_y = y;
    }
    glutPostRedisplay();
}

void project_gui()
{
    return;
}

void display()
{
    ImGuiIO &io = ImGui::GetIO();
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

    if (!is_ortho)
    {
        reshape((GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);

        Eigen::Affine3f viewTranslation = Eigen::Affine3f::Identity();
        viewTranslation.translate(rotation_center);
        Eigen::Affine3f viewLocal = Eigen::Affine3f::Identity();
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

    /*if (show_axes)
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
    }*/

    {
        std::lock_guard<std::mutex> lock(renderPtrLock);

        glLineWidth(2);
        glColor3f(0.0, 1.0, 0.0);
        glBegin(GL_LINE_STRIP);
        for (const auto &t : render_trajectory)
        {
            glVertex3f(t.translation().x(), t.translation().y(), t.translation().z());
        }
        glEnd();
        glLineWidth(1);

        if (render_trajectory.size() > 0)
        {
            auto m = render_trajectory[render_trajectory.size() - 1];
            glBegin(GL_LINES);
            glColor3f(1.0f, 0.0f, 0.0f);
            glVertex3f(m(0, 3), m(1, 3), m(2, 3));
            glVertex3f(m(0, 3) + m(0, 0), m(1, 3) + m(1, 0), m(2, 3) + m(2, 0));

            glColor3f(0.0f, 1.0f, 0.0f);
            glVertex3f(m(0, 3), m(1, 3), m(2, 3));
            glVertex3f(m(0, 3) + m(0, 1), m(1, 3) + m(1, 1), m(2, 3) + m(2, 1));

            glColor3f(0.0f, 0.0f, 1.0f);
            glVertex3f(m(0, 3), m(1, 3), m(2, 3));
            glVertex3f(m(0, 3) + m(0, 2), m(1, 3) + m(1, 2), m(2, 3) + m(2, 2));
            glEnd();
        }

        glBegin(GL_POINTS);
        for (int i = 0; i < render_pointcloud.size(); i++)
        {
            glColor3f(render_pointcloud[i].second, 0.0, 1 - render_pointcloud[i].second);
            glVertex3f(render_pointcloud[i].first.x(), render_pointcloud[i].first.y(), render_pointcloud[i].first.z());
        }
        glEnd();
    }

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();

    // project_gui();

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
    glutCreateWindow("quick_start_demo " HDMAPPING_VERSION_STRING);
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

void wheel(int button, int dir, int x, int y);

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

void wheel(int button, int dir, int x, int y)
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
            translate_z -= 0.05f * translate_z;
        }
    }
    else
    {
        if (is_ortho)
        {
            camera_ortho_xy_view_zoom += 0.1 * camera_ortho_xy_view_zoom;
        }
        else
        {
            translate_z += 0.05f * translate_z;
        }
    }

    return;
}

bool compute_step_2_demo(std::vector<WorkerData> &worker_data, LidarOdometryParams &params, double &ts_failure)
{
    if (worker_data.size() != 0)
    {

        std::chrono::time_point<std::chrono::system_clock> start, end;
        start = std::chrono::system_clock::now();
        double acc_distance = 0.0;
        std::vector<Point3Di> points_global;

        Eigen::Affine3d m_last = params.m_g;
        auto tmp = worker_data[0].intermediate_trajectory;

        worker_data[0].intermediate_trajectory[0] = m_last;
        for (int k = 1; k < tmp.size(); k++)
        {
            Eigen::Affine3d m_update = tmp[k - 1].inverse() * tmp[k];
            m_last = m_last * m_update;
            worker_data[0].intermediate_trajectory[k] = m_last;
        }
        worker_data[0].intermediate_trajectory_motion_model = worker_data[0].intermediate_trajectory;

        auto pp = params.initial_points;
        for (int i = 0; i < pp.size(); i++)
        {
            pp[i].point = params.m_g * pp[i].point;
        }
        update_rgd(params.in_out_params_indoor, params.buckets_indoor, pp, params.m_g.translation());

        TaitBryanPose motion_model_correction;
        motion_model_correction.px = 0.0;
        motion_model_correction.py = 0.0;
        motion_model_correction.pz = 0.0;
        motion_model_correction.om = 0.0;
        motion_model_correction.fi = 0.0;
        motion_model_correction.ka = 0.0;

        for (int i = 0; i < worker_data.size(); i++)
        {
            Eigen::Vector3d mean_shift(0.0, 0.0, 0.0);
            if (i > 1 && params.use_motion_from_previous_step)
            {
                mean_shift = worker_data[i - 1].intermediate_trajectory[worker_data[i - 1].intermediate_trajectory.size() - 1].translation() -
                             worker_data[i - 2].intermediate_trajectory[worker_data[i - 2].intermediate_trajectory.size() - 1].translation();
                mean_shift /= (worker_data[i - 1].intermediate_trajectory.size());

                if (mean_shift.norm() > 1.0)
                {
                    std::cout << "!!!mean_shift.norm() > 1.0!!!" << std::endl;
                    mean_shift = Eigen::Vector3d(0.0, 0.0, 0.0);
                }

                std::vector<Eigen::Affine3d> new_trajectory;
                Eigen::Affine3d current_node = worker_data[i - 1].intermediate_trajectory[worker_data[i - 1].intermediate_trajectory.size() - 1];
                new_trajectory.push_back(current_node);

                for (int tr = 1; tr < worker_data[i].intermediate_trajectory.size(); tr++)
                {
                    auto update = worker_data[i].intermediate_trajectory[tr - 1].inverse() * worker_data[i].intermediate_trajectory[tr];
                    current_node = current_node * update;

                    new_trajectory.push_back(current_node);
                }

                for (int tr = 0; tr < new_trajectory.size(); tr++)
                {
                    new_trajectory[tr].translation() += mean_shift * tr;
                }

                worker_data[i].intermediate_trajectory = new_trajectory;
                worker_data[i].intermediate_trajectory_motion_model = new_trajectory;
            }

            bool add_pitch_roll_constraint = false;

            std::chrono::time_point<std::chrono::system_clock> start1, end1;
            start1 = std::chrono::system_clock::now();

            auto tmp_worker_data = worker_data[i].intermediate_trajectory;

            for (int iter = 0; iter < params.nr_iter; iter++)
            {
                double delta = 1000000.0;
                optimize_lidar_odometry(worker_data[i].intermediate_points, worker_data[i].intermediate_trajectory, worker_data[i].intermediate_trajectory_motion_model,
                                        params.in_out_params_indoor, params.buckets_indoor, params.in_out_params_outdoor,
                                        params.buckets_outdoor, /*params.useMultithread*/ false, 70.0, delta, 1.0, motion_model_correction,
                                        params.lidar_odometry_motion_model_x_1_sigma_m,
                                        params.lidar_odometry_motion_model_y_1_sigma_m,
                                        params.lidar_odometry_motion_model_z_1_sigma_m,
                                        params.lidar_odometry_motion_model_om_1_sigma_deg,
                                        params.lidar_odometry_motion_model_fi_1_sigma_deg,
                                        params.lidar_odometry_motion_model_ka_1_sigma_deg,
                                        params.lidar_odometry_motion_model_fix_origin_x_1_sigma_m,
                                        params.lidar_odometry_motion_model_fix_origin_y_1_sigma_m,
                                        params.lidar_odometry_motion_model_fix_origin_z_1_sigma_m,
                                        params.lidar_odometry_motion_model_fix_origin_om_1_sigma_deg,
                                        params.lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg,
                                        params.lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg);
            }

            end1 = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
            std::cout << "optimizing worker_data [" << i + 1 << "] of " << worker_data.size() << " acc_distance: " << acc_distance << " elapsed time: " << elapsed_seconds1.count() << std::endl;

            {
                std::lock_guard<std::mutex> lock(renderPtrLock);

                for (int k = 0; k < worker_data[i].intermediate_points.size(); k++)
                {
                    Point3Di p = worker_data[i].intermediate_points[k];
                    Point3Di p_local = p;
                    int index_pose = p.index_pose;
                    p.point = worker_data[i].intermediate_trajectory[index_pose] * p.point;
                    if (p_local.point.norm() > 1.0)
                    {
                        render_pointcloud.emplace_back(p.point, (((float)p.intensity) / 256.0) * 1.8);
                    }
                }
                for (int k = 0; k < worker_data[i].intermediate_trajectory.size(); k++)
                {
                    render_trajectory.push_back(worker_data[i].intermediate_trajectory[k]);
                }
            }

            // temp save
            if (i % 100 == 0)
            {
                std::vector<Point3Di> global_points;
                for (int k = 0; k < worker_data[i].intermediate_points.size(); k++)
                {
                    Point3Di p = worker_data[i].intermediate_points[k];
                    int index_pose = p.index_pose;
                    p.point = worker_data[i].intermediate_trajectory[index_pose] * p.point;
                    global_points.push_back(p);
                }
                std::string fn = params.working_directory_preview + "/temp_point_cloud_" + std::to_string(i) + ".laz";
                saveLaz(fn.c_str(), global_points);
            }
            auto acc_distance_tmp = acc_distance;
            acc_distance += ((worker_data[i].intermediate_trajectory[0].inverse()) *
                             worker_data[i].intermediate_trajectory[worker_data[i].intermediate_trajectory.size() - 1])
                                .translation()
                                .norm();

            if (!(acc_distance == acc_distance))
            {
                worker_data[i].intermediate_trajectory = tmp_worker_data;
                std::cout << "CHALLENGING DATA OCCURED!!!" << std::endl;
                acc_distance = acc_distance_tmp;
                std::cout << "please split data set into subsets" << std::endl;
                ts_failure = worker_data[i].intermediate_trajectory_timestamps[0].first;
                // std::cout << "calculations canceled for TIMESTAMP: " << (long long int)worker_data[i].intermediate_trajectory_timestamps[0].first << std::endl;
                return false;
            }

            // update
            for (int j = i + 1; j < worker_data.size(); j++)
            {
                Eigen::Affine3d m_last = worker_data[j - 1].intermediate_trajectory[worker_data[j - 1].intermediate_trajectory.size() - 1];
                auto tmp = worker_data[j].intermediate_trajectory;

                worker_data[j].intermediate_trajectory[0] = m_last;

                for (int k = 1; k < tmp.size(); k++)
                {
                    Eigen::Affine3d m_update = tmp[k - 1].inverse() * tmp[k];
                    m_last = m_last * m_update;
                    worker_data[j].intermediate_trajectory[k] = m_last;
                }
            }

            for (int j = 0; j < worker_data[i].intermediate_points.size(); j++)
            {
                Point3Di pp = worker_data[i].intermediate_points[j];
                pp.point = worker_data[i].intermediate_trajectory[worker_data[i].intermediate_points[j].index_pose] * pp.point;
                points_global.push_back(pp);
            }

            if (acc_distance > params.sliding_window_trajectory_length_threshold)
            {
                std::chrono::time_point<std::chrono::system_clock> startu, endu;
                startu = std::chrono::system_clock::now();

                if (params.reference_points.size() == 0)
                {
                    params.buckets_indoor.clear();
                    params.buckets_outdoor.clear();
                }

                std::vector<Point3Di> points_global_new;
                points_global_new.reserve(points_global.size() / 2 + 1);
                for (int k = points_global.size() / 2; k < points_global.size(); k++)
                {
                    points_global_new.emplace_back(points_global[k]);
                }

                acc_distance = 0;
                points_global = points_global_new;

                // decimate
                if (params.decimation > 0)
                {
                    decimate(points_global, params.decimation, params.decimation, params.decimation);
                }
                update_rgd(params.in_out_params_indoor, params.buckets_indoor, points_global, worker_data[i].intermediate_trajectory[0].translation());
                //
                endu = std::chrono::system_clock::now();

                std::chrono::duration<double> elapsed_secondsu = endu - startu;
                std::time_t end_timeu = std::chrono::system_clock::to_time_t(endu);

                std::cout << "finished computation at " << std::ctime(&end_timeu)
                          << "elapsed time update: " << elapsed_secondsu.count() << "s\n";
            }
            else
            {
                std::vector<Point3Di> pg;
                for (int j = 0; j < worker_data[i].intermediate_points.size(); j++)
                {
                    Point3Di pp = worker_data[i].intermediate_points[j];
                    pp.point = worker_data[i].intermediate_trajectory[worker_data[i].intermediate_points[j].index_pose] * pp.point;
                    pg.push_back(pp);
                }
                update_rgd(params.in_out_params_indoor, params.buckets_indoor, pg, worker_data[i].intermediate_trajectory[0].translation());
            }

            if (i > 1)
            {
                double translation = (worker_data[i - 1].intermediate_trajectory[0].translation() -
                                      worker_data[i - 2].intermediate_trajectory[0].translation())
                                         .norm();
                params.consecutive_distance += translation;
            }
        }

        for (int i = 0; i < worker_data.size(); i++)
        {
            worker_data[i].intermediate_trajectory_motion_model = worker_data[i].intermediate_trajectory;
        }

        end = std::chrono::system_clock::now();

        std::chrono::duration<double> elapsed_seconds = end - start;
        std::time_t end_time = std::chrono::system_clock::to_time_t(end);

        std::cout << "finished computation at " << std::ctime(&end_time)
                  << "elapsed time: " << elapsed_seconds.count() << "s\n";

        // estimate total lenght of trajectory
        double length_of_trajectory = 0;
        for (int i = 1; i < worker_data.size(); i++)
        {
            length_of_trajectory += (worker_data[i].intermediate_trajectory[0].translation() - worker_data[i - 1].intermediate_trajectory[0].translation()).norm();
        }
        std::cout << "length_of_trajectory: " << length_of_trajectory << " [m]" << std::endl;
    }

    return true;
}

int main(int argc, char *argv[])
{
    params.in_out_params_indoor.resolution_X = 0.3;
    params.in_out_params_indoor.resolution_Y = 0.3;
    params.in_out_params_indoor.resolution_Z = 0.3;
    params.in_out_params_indoor.bounding_box_extension = 20.0;

    params.in_out_params_outdoor.resolution_X = 0.3;
    params.in_out_params_outdoor.resolution_Y = 0.3;
    params.in_out_params_outdoor.resolution_Z = 0.3;
    params.in_out_params_outdoor.bounding_box_extension = 20.0;

    std::vector<std::string> input_file_names;

    input_file_names.push_back("imu0001.csv");
    input_file_names.push_back("imu0002.csv");
    input_file_names.push_back("imu0003.csv");

    input_file_names.push_back("lidar0001.sn");
    input_file_names.push_back("lidar0002.sn");
    input_file_names.push_back("lidar0003.sn");

    input_file_names.push_back("lidar0001.laz");
    input_file_names.push_back("lidar0002.laz");
    input_file_names.push_back("lidar0003.laz");

    std::sort(std::begin(input_file_names), std::end(input_file_names));

    // std::vector<std::string> csv_files;
    std::vector<std::string> laz_files;
    // std::vector<std::string> sn_files;

    std::for_each(std::begin(input_file_names), std::end(input_file_names), [&](const std::string &fileName)
                  {
                    if (fileName.ends_with(".laz") || fileName.ends_with(".las"))
                    {
                        laz_files.push_back(fileName);
                    }
                    if (fileName.ends_with(".csv"))
                    {
                        csv_files.push_back(fileName);
                    }
                    if (fileName.ends_with(".sn"))
                    {
                        sn_files.push_back(fileName);
                    } });

    if (input_file_names.size() > 0 && laz_files.size() == csv_files.size())
    {
        working_directory = "."; // fs::path(input_file_names[0]).parent_path().string();

        // std::cout << "0" << std::endl;

        // check if folder exists!
        if (!fs::exists(working_directory))
        {
            std::cout << "folder '" << working_directory << "' does not exist" << std::endl;

            std::string message_info = "folder '" + working_directory + "' does not exist --> PLEASE REMOVE e.g. POLISH LETTERS from path. !!!PROGRAM WILL SHUT DOWN AFTER THIS MESSAGE!!!";

            [[maybe_unused]]
            pfd::message message(
                "Information",
                message_info.c_str(),
                pfd::choice::ok, pfd::icon::error);
            message.result();
            exit(1);
        }

        const auto calibrationFile = (fs::path(working_directory) / "calibration.json").string();
        const auto preloadedCalibration = MLvxCalib::GetCalibrationFromFile(calibrationFile);
        imuSnToUse = MLvxCalib::GetImuSnToUse(calibrationFile);
        if (!preloadedCalibration.empty())
        {
            std::cout << "Loaded calibration for: \n";
            for (const auto &[sn, _] : preloadedCalibration)
            {
                std::cout << " -> " << sn << std::endl;
            }
        }
        else
        {
            std::cout << "There is no calibration.json file in folder (check comment in source code) file: " << __FILE__ << " line: " << __LINE__ << std::endl;
            std::cout << "IGNORE THIS MESSAGE IF YOU HAVE ONLY 1 LIDAR" << std::endl;

            // example file for 2x livox";
            /*
            {
                "calibration" : {
                    "47MDL9T0020193" : {
                        "identity" : "true"
                    },
                    "47MDL9S0020300" :
                        {
                            "order" : "ROW",
                            "inverted" : "TRUE",
                            "data" : [
                                0.999824, 0.00466397, -0.0181595, -0.00425984,
                                -0.0181478, -0.00254457, -0.999832, -0.151599,
                                -0.0047094, 0.999986, -0.00245948, -0.146408,
                                0, 0, 0, 1
                            ]
                        }
                },
                                "imuToUse" : "47MDL9T0020193"
            }*/
        }

        fs::path wdp = fs::path(input_file_names[0]).parent_path();
        wdp /= "preview";
        if (!fs::exists(wdp))
        {
            std::cout << "trying creating folder: '" << wdp << "'" << std::endl;
            fs::create_directory(wdp);
            std::cout << "folder created" << std::endl;
        }

        params.working_directory_preview = wdp.string();

        for (size_t i = 0; i < input_file_names.size(); i++)
        {
            std::cout << input_file_names[i] << std::endl;
        }
        std::cout << "loading imu" << std::endl;
        std::vector<std::tuple<std::pair<double, double>, FusionVector, FusionVector>> imu_data;

        for (size_t fileNo = 0; fileNo < csv_files.size(); fileNo++)
        {
            const std::string &imufn = csv_files.at(fileNo);
            const std::string snFn = (fileNo >= sn_files.size()) ? ("") : (sn_files.at(fileNo));
            const auto idToSn = MLvxCalib::GetIdToSnMapping(snFn);
            // GetId of Imu to use
            int imuNumberToUse = MLvxCalib::GetImuIdToUse(idToSn, imuSnToUse);
            std::cout << "imuNumberToUse  " << imuNumberToUse << " at '" << imufn << "'" << std::endl;
            auto imu = load_imu(imufn.c_str(), imuNumberToUse);
            std::cout << imufn << " with mapping " << snFn << std::endl;
            imu_data.insert(std::end(imu_data), std::begin(imu), std::end(imu));
        }

        std::cout << "loading points" << std::endl;
        std::vector<std::vector<Point3Di>> pointsPerFile;
        pointsPerFile.resize(laz_files.size());
        std::mutex mtx;
        std::cout << "start std::transform" << std::endl;

        std::transform(std::execution::par_unseq, std::begin(laz_files), std::end(laz_files), std::begin(pointsPerFile), [&](const std::string &fn)
                       {
                           // Load mapping from id to sn
                           fs::path fnSn(fn);
                           fnSn.replace_extension(".sn");

                           // GetId of Imu to use
                           const auto idToSn = MLvxCalib::GetIdToSnMapping(fnSn.string());
                           auto calibration = MLvxCalib::CombineIntoCalibration(idToSn, preloadedCalibration);
                           auto data = load_point_cloud(fn.c_str(), true, params.filter_threshold_xy_inner, params.filter_threshold_xy_outer, calibration);

                           std::sort(data.begin(), data.end(), [](const Point3Di &a, const Point3Di &b)
                                     { return a.timestamp < b.timestamp; });

                           if ((fn == laz_files.front()) && (params.save_calibration_validation))
                           {
                               fs::path calibrationValidtationFile = wdp / "calibrationValidation.asc";
                               std::ofstream testPointcloud{calibrationValidtationFile.c_str()};
                               int row_index = 0;
                               for (const auto &p : data)
                               {
                                   if (row_index++ >= params.calibration_validation_points)
                                   {
                                       break;
                                   }
                                   testPointcloud << p.point.x() << "\t" << p.point.y() << "\t" << p.point.z() << "\t" << p.intensity << "\t" << (int)p.lidarid << "\n";
                               }
                           }

                           std::unique_lock lck(mtx);
                           for (const auto &[id, calib] : calibration)
                           {
                               std::cout << " id : " << id << std::endl;
                               std::cout << calib.matrix() << std::endl;
                           }
                           return data;
                           // std::cout << fn << std::endl;
                           //
                       });
        std::cout << "std::transform finished" << std::endl;

        FusionAhrs ahrs;
        FusionAhrsInitialise(&ahrs);

        if (fusionConventionNwu)
        {
            ahrs.settings.convention = FusionConventionNwu;
        }
        if (fusionConventionEnu)
        {
            ahrs.settings.convention = FusionConventionEnu;
        }
        if (fusionConventionNed)
        {
            ahrs.settings.convention = FusionConventionNed;
        }

        std::map<double, std::pair<Eigen::Matrix4d, double>> trajectory;

        int counter = 1;
        for (const auto &[timestamp_pair, gyr, acc] : imu_data)
        {
            const FusionVector gyroscope = {static_cast<float>(gyr.axis.x * 180.0 / M_PI), static_cast<float>(gyr.axis.y * 180.0 / M_PI), static_cast<float>(gyr.axis.z * 180.0 / M_PI)};
            const FusionVector accelerometer = {acc.axis.x, acc.axis.y, acc.axis.z};

            // std::cout << "acc.axis.x: " << acc.axis.x << " acc.axis.y: " << acc.axis.y << " acc.axis.z: " << acc.axis.z << std::endl;

            FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

            FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);

            Eigen::Quaterniond d{quat.element.w, quat.element.x, quat.element.y, quat.element.z};
            Eigen::Affine3d t{Eigen::Matrix4d::Identity()};
            t.rotate(d);

            //
            // TaitBryanPose rot_y;
            // rot_y.px = rot_y.py = rot_y.pz = rot_y.px = rot_y.py = rot_y.pz;
            // rot_y.fi = -5 * M_PI / 180.0;
            // Eigen::Affine3d m_rot_y = affine_matrix_from_pose_tait_bryan(rot_y);
            // t = t * m_rot_y;
            //
            // std::map<double, Eigen::Matrix4d> trajectory;
            trajectory[timestamp_pair.first] = std::pair(t.matrix(), timestamp_pair.second);
            const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
            counter++;
            if (counter % 100 == 0)
            {
                printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f [%d of %d]\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw, counter++, imu_data.size());
            }
        }

        int number_of_points = 0;
        for (const auto &pp : pointsPerFile)
        {
            number_of_points += pp.size();
        }
        std::cout << "number of points: " << number_of_points << std::endl;
        std::cout << "start transforming points" << std::endl;

        int number_of_initial_points = 0;
        double timestamp_begin;
        for (const auto &pp : pointsPerFile)
        {
            // number_of_points += pp.size();
            for (const auto &p : pp)
            {
                number_of_initial_points++;
                params.initial_points.push_back(p);
                if (number_of_initial_points > threshold_initial_points)
                {
                    timestamp_begin = p.timestamp;
                    break;
                }
            }
            if (number_of_initial_points > threshold_initial_points)
            {
                break;
            }
        }

        std::cout << "timestamp_begin: " << timestamp_begin << std::endl;

        std::vector<std::pair<double, double>> timestamps;
        std::vector<Eigen::Affine3d> poses;
        for (const auto &t : trajectory)
        {
            if (t.first >= timestamp_begin)
            {
                timestamps.emplace_back(t.first, t.second.second);
                Eigen::Affine3d m;
                m.matrix() = t.second.first;
                poses.push_back(m);
            }
        }

        std::cout << "poses.size(): " << poses.size() << std::endl;

        if (poses.empty())
        {
            std::cerr << "Loading poses went wrong! Could not load poses!" << std::endl;
            return 1;
        }

        int thershold = THRESHOLD_NR_POSES;
        WorkerData wd;
        // std::vector<double> temp_ts;
        // temp_ts.reserve(1000000);

        // int last_point = 0;
        int index_begin = 0;

        const int n_iter = std::floor(poses.size() / thershold);
        worker_data.reserve(n_iter);
        for (int i = 0; i < n_iter; i++)
        {
            if (i % 50 == 0)
            {
                std::cout << "preparing data " << i + 1 << " of " << n_iter << std::endl;
            }
            WorkerData wd;
            wd.intermediate_trajectory.reserve(thershold);
            wd.intermediate_trajectory_motion_model.reserve(thershold);
            wd.intermediate_trajectory_timestamps.reserve(thershold);
            wd.imu_om_fi_ka.reserve(thershold);
            for (int ii = 0; ii < thershold; ii++)
            {
                int idx = i * thershold + ii;
                wd.intermediate_trajectory.emplace_back(poses[idx]);
                wd.intermediate_trajectory_motion_model.emplace_back(poses[idx]);
                wd.intermediate_trajectory_timestamps.emplace_back(timestamps[idx]);
                TaitBryanPose tb = pose_tait_bryan_from_affine_matrix(poses[idx]);
                // wd.imu_roll_pitch.emplace_back(tb.om, tb.fi);
                wd.imu_om_fi_ka.emplace_back(tb.om, tb.fi, tb.ka);
            }
            std::vector<Point3Di> points;
            bool found = false;
            auto lower = pointsPerFile[0].begin();
            for (int index = index_begin; index < pointsPerFile.size(); index++)
            {
                auto lower = std::lower_bound(
                    pointsPerFile[index].begin(), pointsPerFile[index].end(),
                    wd.intermediate_trajectory_timestamps[0].first,
                    [](const Point3Di &point, double timestamp)
                    {
                        return point.timestamp < timestamp;
                    });
                auto upper = std::lower_bound(
                    pointsPerFile[index].begin(), pointsPerFile[index].end(),
                    wd.intermediate_trajectory_timestamps[wd.intermediate_trajectory_timestamps.size() - 1].first,
                    [](const Point3Di &point, double timestamp)
                    {
                        return point.timestamp < timestamp;
                    });
                auto tmp = std::distance(lower, upper);
                points.reserve(points.size() + std::distance(lower, upper));
                if (lower != upper)
                {
                    points.insert(points.end(), std::make_move_iterator(lower), std::make_move_iterator(upper));
                    found = true;
                }
            }

            wd.original_points = points;
            for (unsigned long long int k = 0; k < wd.original_points.size(); k++)
            {
                Point3Di &p = wd.original_points[k];
                auto lower = std::lower_bound(wd.intermediate_trajectory_timestamps.begin(), wd.intermediate_trajectory_timestamps.end(), p.timestamp,
                                              [](std::pair<double, double> lhs, double rhs) -> bool
                                              { return lhs.first < rhs; });
                p.index_pose = std::distance(wd.intermediate_trajectory_timestamps.begin(), lower);
            }

            if (params.decimation > 0.0)
            {
                wd.intermediate_points = decimate(wd.original_points, params.decimation, params.decimation, params.decimation);
            }
            worker_data.push_back(wd);
        }

        params.m_g = worker_data[0].intermediate_trajectory[0];
        // step_1_done = true;
        std::cout << "step_1_done please click 'compute_all (step 2)' to continue calculations" << std::endl;
    }
    else
    {
        std::cout << "please select files correctly" << std::endl;
        std::cout << "input_file_names.size(): " << input_file_names.size() << std::endl;
        std::cout << "laz_files.size(): " << laz_files.size() << std::endl;
        std::cout << "csv_files.size(): " << csv_files.size() << std::endl;

        std::cout << "condition: input_file_names.size() > 0 && laz_files.size() == csv_files.size() NOT SATISFIED!!!" << std::endl;
    }

    initGL(&argc, argv);
    glutDisplayFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutMouseWheelFunc(wheel);

    //
    double ts_failure = 0.0;

    std::thread th([&]()
                   {
        compute_step_2_demo(worker_data, params, ts_failure);

        {
        std::lock_guard<std::mutex> lock(renderPtrLock);
        std::vector<Point3Di> global_points;

        for (int k = 0; k < render_pointcloud.size(); k++)
        {
            Point3Di p;
            p.point = render_pointcloud[k].first;
            p.intensity = render_pointcloud[k].second * 256;
            global_points.push_back(p);
        }
        
        saveLaz("out_demo_point_cloud.laz", global_points);
        std::cout << "file: out_demo_point_cloud.laz saved" << std::endl;
        } });

    glutMainLoop();

    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGLUT_Shutdown();

    ImGui::DestroyContext();
    return 0;
}

