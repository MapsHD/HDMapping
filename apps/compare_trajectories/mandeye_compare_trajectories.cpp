#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <ImGuizmo.h>
#include <imgui_internal.h>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <Eigen/Eigen>

#include <transformations.h>

#include <portable-file-dialogs.h>

#include <filesystem>
#include "../lidar_odometry_step_1/lidar_odometry_utils.h"

#include <HDMapping/Version.hpp>

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
int point_size = 1;
Eigen::Vector3f rotation_center = Eigen::Vector3f::Zero();
float translate_x, translate_y = 0.0;
float translate_z = -50.0;
float rotate_x = 0.0, rotate_y = 0.0;
int mouse_old_x, mouse_old_y;
int mouse_buttons = 0;
bool gui_mouse_down{false};
float mouse_sensitivity = 1.0;
bool move_source_trajectory_with_gizmo = false;
bool show_correspondences = false;

float m_ortho_projection[] = {1, 0, 0, 0,
                              0, 1, 0, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1};

float m_ortho_gizmo_view[] = {1, 0, 0, 0,
                              0, 1, 0, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1};

float m_gizmo[] = {1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1, 0,
                   0, 0, 0, 1};
namespace Data
{
    std::map<double, Eigen::Matrix4d> trajectory_gt;
    std::map<double, Eigen::Matrix4d> trajectory_est;
    Eigen::Matrix4d trajectory_offset = Eigen::Matrix4d::Identity();
}

// Function to split a CSV line into values
std::vector<double> parseCSVLine(const std::string &line)
{
    std::vector<double> values;
    std::stringstream ss(line);
    std::string token;

    while (std::getline(ss, token, ','))
    {
        values.push_back(std::stod(token)); // Convert string to double
    }
    return values;
}

std::vector<double> parseCSVLineSpace(const std::string &line)
{
    std::vector<double> values;
    std::stringstream ss(line);
    std::string token;

    while (std::getline(ss, token, ' '))
    {
        values.push_back(std::stod(token)); // Convert string to double
    }
    return values;
}

std::map<double, Eigen::Matrix4d> load_trajectory_from_CSV(const std::string &file_path)
{
    std::map<double, Eigen::Matrix4d> trajectory;
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        std::cerr << "Error: could not open file " << file_path << std::endl;
        return trajectory;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::vector<double> values = parseCSVLine(line);
        if (values.size() != 1 + 3 + 4)
        {
            std::cerr << "Error: invalid line in CSV file" << std::endl;
            continue;
        }

        double timestamp = values[0] / 1e9;
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose(0, 3) = values[1];
        pose(1, 3) = values[2];
        pose(2, 3) = values[3];
        pose.block<3, 3>(0, 0) = Eigen::Quaterniond(values[4], values[5], values[6], values[7]).toRotationMatrix();

        trajectory[timestamp] = pose;
    }
    return trajectory;
}

std::map<double, Eigen::Matrix4d> load_trajectory_from_CSV_LIDARROT(const std::string &file_path)
{
    std::map<double, Eigen::Matrix4d> trajectory;
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        std::cerr << "Error: could not open file " << file_path << std::endl;
        return trajectory;
    }

    std::string line;
    std::getline(file, line); // Skip header line

    while (std::getline(file, line))
    {
        std::vector<double> values = parseCSVLine(line);
        if (values.size() != 13)
        {
            std::cerr << "Error: Invalid line in CSV file. Expected 13 values but found " << values.size() << std::endl;
            continue;
        }
        double timestamp = values[0] / 1e9;
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose(0, 3) = values[1];
        pose(1, 3) = values[2];
        pose(2, 3) = values[3];
        double r00 = values[4], r01 = values[5], r02 = values[6];
        double r10 = values[7], r11 = values[8], r12 = values[9];
        double r20 = values[10], r21 = values[11], r22 = values[12];
        Eigen::Matrix3d rotationMatrix;
        rotationMatrix << r00, r01, r02,
            r10, r11, r12,
            r20, r21, r22;
        pose.block<3, 3>(0, 0) = rotationMatrix;

        trajectory[timestamp] = pose;
    }
    return trajectory;
}

std::map<double, Eigen::Matrix4d> load_trajectory_from_CSV_UNIXROT(const std::string &file_path)
{
    std::map<double, Eigen::Matrix4d> trajectory;
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        std::cerr << "Error: could not open file " << file_path << std::endl;
        return trajectory;
    }

    std::string line;
    std::getline(file, line); // Skip header line

    while (std::getline(file, line))
    {
        std::vector<double> values = parseCSVLine(line);
        if (values.size() != 13)
        {
            std::cerr << "Error: Invalid line in CSV file. Expected 13 values but found " << values.size() << std::endl;
            continue;
        }
        double timestamp = values[0] / 1e9;
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose(0, 3) = values[1];
        pose(1, 3) = values[2];
        pose(2, 3) = values[3];
        double r00 = values[4], r01 = values[5], r02 = values[6];
        double r10 = values[7], r11 = values[8], r12 = values[9];
        double r20 = values[10], r21 = values[11], r22 = values[12];
        Eigen::Matrix3d rotationMatrix;
        rotationMatrix << r00, r01, r02,
            r10, r11, r12,
            r20, r21, r22;
        pose.block<3, 3>(0, 0) = rotationMatrix;

        trajectory[timestamp] = pose;
    }
    return trajectory;
}

std::map<double, Eigen::Matrix4d> load_trajectory_from_CSV_LIDARUNIXROT(const std::string &file_path)
{
    std::map<double, Eigen::Matrix4d> trajectory;
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        std::cerr << "Error: could not open file " << file_path << std::endl;
        return trajectory;
    }

    std::string line;
    std::getline(file, line); // Skip header line

    while (std::getline(file, line))
    {
        std::vector<double> values = parseCSVLine(line);
        if (values.size() != 14)
        {
            std::cerr << "Error: Invalid line in CSV file. Expected 14 values but found " << values.size() << std::endl;
            continue;
        }
        double timestamplidar = values[0] / 1e9;
        double timestampunix = values[1] / 1e9;
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose(0, 3) = values[2];
        pose(1, 3) = values[3];
        pose(2, 3) = values[4];
        double r00 = values[5], r01 = values[6], r02 = values[7];
        double r10 = values[8], r11 = values[9], r12 = values[10];
        double r20 = values[11], r21 = values[12], r22 = values[13];
        Eigen::Matrix3d rotationMatrix;
        rotationMatrix << r00, r01, r02,
            r10, r11, r12,
            r20, r21, r22;
        pose.block<3, 3>(0, 0) = rotationMatrix;

        trajectory[timestamplidar] = pose;
    }
    return trajectory;
}

std::map<double, Eigen::Matrix4d> load_trajectory_from_CSV_LIDARQ(const std::string &file_path)
{
    std::map<double, Eigen::Matrix4d> trajectory;
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        std::cerr << "Error: could not open file " << file_path << std::endl;
        return trajectory;
    }

    std::string line;
    std::getline(file, line); // Skip header line

    while (std::getline(file, line))
    {
        std::vector<double> values = parseCSVLine(line);
        if (values.size() != 1 + 3 + 4)
        {
            std::cerr << "Error: Invalid line in CSV file. Expected 8 values but found " << values.size() << std::endl;
            continue;
        }
        double timestamp = values[0] / 1e9;
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose(0, 3) = values[1];
        pose(1, 3) = values[2];
        pose(2, 3) = values[3];
        pose.block<3, 3>(0, 0) = Eigen::Quaterniond(values[4], values[5], values[6], values[7]).toRotationMatrix();

        trajectory[timestamp] = pose;
    }
    return trajectory;
}

std::map<double, Eigen::Matrix4d> load_trajectory_from_CSV_UNIXQ(const std::string &file_path)
{
    std::map<double, Eigen::Matrix4d> trajectory;
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        std::cerr << "Error: could not open file " << file_path << std::endl;
        return trajectory;
    }

    std::string line;
    std::getline(file, line); // Skip header line

    while (std::getline(file, line))
    {
        std::vector<double> values = parseCSVLine(line);
        if (values.size() != 1 + 3 + 4)
        {
            std::cerr << "Error: Invalid line in CSV file. Expected 8 values but found " << values.size() << std::endl;
            continue;
        }
        double timestamp = values[0] / 1e9;
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose(0, 3) = values[1];
        pose(1, 3) = values[2];
        pose(2, 3) = values[3];
        pose.block<3, 3>(0, 0) = Eigen::Quaterniond(values[4], values[5], values[6], values[7]).toRotationMatrix();

        trajectory[timestamp] = pose;
    }
    return trajectory;
}

std::map<double, Eigen::Matrix4d> load_trajectory_from_CSV_LIDARUNIXQ(const std::string &file_path)
{
    std::map<double, Eigen::Matrix4d> trajectory;
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        std::cerr << "Error: could not open file " << file_path << std::endl;
        return trajectory;
    }

    std::string line;
    std::getline(file, line); // Skip header line

    while (std::getline(file, line))
    {
        std::vector<double> values = parseCSVLine(line);
        if (values.size() != 1 + 3 + 5)
        {
            std::cerr << "Error: Invalid line in CSV file. Expected 9 values but found " << values.size() << std::endl;
            continue;
        }
        double timestamplidar = values[0] / 1e9;
        double timestampunix = values[1] / 1e9;
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose(0, 3) = values[2];
        pose(1, 3) = values[3];
        pose(2, 3) = values[4];
        pose.block<3, 3>(0, 0) = Eigen::Quaterniond(values[5], values[6], values[7], values[8]).toRotationMatrix();

        trajectory[timestamplidar] = pose;
    }
    return trajectory;
}

std::map<double, Eigen::Matrix4d> load_trajectory_from_CSV_STEP1(const std::string &file_path)
{
    std::map<double, Eigen::Matrix4d> trajectory;
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        std::cerr << "Error: could not open file " << file_path << std::endl;
        return trajectory;
    }

    std::string line;
    std::getline(file, line); // Skip header line

    while (std::getline(file, line))
    {
        std::vector<double> values = parseCSVLineSpace(line);
        if (values.size() != 17)
        {
            std::cerr << "Error: Invalid line in CSV file. Expected 17 values but found " << values.size() << std::endl;
            continue;
        }
        double timestamp = values[0] / 1e9;
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose(0, 3) = values[1];
        pose(1, 3) = values[2];
        pose(2, 3) = values[3];
        double r00 = values[4], r01 = values[5], r02 = values[6];
        double r10 = values[7], r11 = values[8], r12 = values[9];
        double r20 = values[10], r21 = values[11], r22 = values[12];
        Eigen::Matrix3d rotationMatrix;
        rotationMatrix << r00, r01, r02,
            r10, r11, r12,
            r20, r21, r22;
        pose.block<3, 3>(0, 0) = rotationMatrix;

        trajectory[timestamp] = pose;
    }
    return trajectory;
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
    if (ImGui::Begin("main gui window"))
    {
        ImGui::ColorEdit3("clear color", (float *)&clear_color);

        ImGui::InputInt("point_size", &point_size);
        point_size = std::max(1, point_size);

        ImGui::Checkbox("show_axes", &show_axes);
        ImGui::Checkbox("is_ortho", &is_ortho);
        ImGui::Checkbox("show_correspondences", &show_correspondences);
        ImGui::Checkbox("move_source_trajectory_with_gizmo", &move_source_trajectory_with_gizmo);
        // if (ImGui::Button("Load target trajectory (e.g. ground truth trajectory))
        // {
        //     auto file_path = pfd::open_file("Select target trajectory file", fs::current_path().string(), {"CSV Files", "*.csv"}).result();
        //     if (!file_path.empty())
        //     {
        //         Data::trajectory_gt = load_trajectory_from_CSV(file_path[0]);
        //     }
        // }

        ImGui::Text("--------------------------------");
        if (ImGui::Button("(Step 1) Load target trajectory (timestampLidar,x,y,z,qx,qy,qz,qw(ground truth trajectory)"))
        {
            auto file_path = pfd::open_file("Select target trajectory file", fs::current_path().string(), {"CSV Files", "*.csv"}).result();
            if (!file_path.empty())
            {
                Data::trajectory_gt = load_trajectory_from_CSV_LIDARQ(file_path[0]);
            }
        }
        if (ImGui::Button("(Step 2) Load source trajectory (timestampLidar,x,y,z,qx,qy,qz,qw(estimated trajectory)"))
        {
            auto file_path = pfd::open_file("Select source trajectory file", fs::current_path().string(), {"CSV Files", "*.csv"}).result();
            if (!file_path.empty())
            {
                Data::trajectory_est = load_trajectory_from_CSV_LIDARQ(file_path[0]);
            }
        }
        if (ImGui::Button("(Step 3) Calculate ATE (Absolute Trajectory Error)"))
        {
            // std::cout << std::setprecision(20);
            // for (const auto &p : Data::trajectory_est)
            //{
            //     std::cout << p.first << std::endl;
            // }
            // std::cout << " ---------------------------" << std::endl;
            // for (const auto &p : Data::trajectory_gt)
            //{
            //     std::cout << p.first << std::endl;
            // }

            double ATE = 0.0;
            double max_ATE = -1000000.0;
            double min_ATE = 1000000.0;
            int count = 0;
            for (const auto &p : Data::trajectory_est)
            {
                // std::cout << p.second.matrix() << std::endl;

                auto it = getInterpolatedPose(Data::trajectory_gt, p.first);
                if (!it.isZero())
                {
                    auto tp = Data::trajectory_offset * p.second;
                    // std::cout << tp.matrix() << " " <<   std::endl;

                    // std::cout << "ATE " << ATE << std::endl;
                    double ate = (tp - it).norm();

                    if (ate > max_ATE){
                        max_ATE = ate;
                    }
                    if (ate < min_ATE)
                    {
                        min_ATE = ate;
                    }
                    ATE += ate;
                    count++;
                }
            }
            if (count > 0)
            {
                std::cout << "ATE: " << ATE / count << std::endl;
                std::cout << "min_ATE: " << min_ATE << std::endl;
                std::cout << "max_ATE: " << max_ATE << std::endl;
            }
            else
            {
                std::cout << "I cant calculate ATE --> 0 correspondences" << std::endl;
            }
        }
        ImGui::Text("--------------------------------");

        static bool other_function = false;
        ImGui::Checkbox("other_function", &other_function);

        if (other_function)
        {
            if (ImGui::Button("Load source trajectory (timestampLidar,x,y,z,r00,r01,r02,r10,r11,r12,r20,r21,r22(estimated trajectory)"))
            {
                auto file_path = pfd::open_file("Select source trajectory file", fs::current_path().string(), {"CSV Files", "*.csv"}).result();
                if (!file_path.empty())
                {
                    Data::trajectory_est = load_trajectory_from_CSV_LIDARROT(file_path[0]);
                }
            }
            if (ImGui::Button("Load source trajectory (timestampUnix,x,y,z,r00,r01,r02,r10,r11,r12,r20,r21,r22(estimated trajectory)"))
            {
                auto file_path = pfd::open_file("Select source trajectory file", fs::current_path().string(), {"CSV Files", "*.csv"}).result();
                if (!file_path.empty())
                {
                    Data::trajectory_est = load_trajectory_from_CSV_UNIXROT(file_path[0]);
                }
            }
            if (ImGui::Button("Load source trajectory (timestampLidar,timestampUnix,x,y,z,r00,r01,r02,r10,r11,r12,r20,r21,r22(estimated trajectory)"))
            {
                auto file_path = pfd::open_file("Select source trajectory file", fs::current_path().string(), {"CSV Files", "*.csv"}).result();
                if (!file_path.empty())
                {
                    Data::trajectory_est = load_trajectory_from_CSV_LIDARUNIXROT(file_path[0]);
                }
            }

            if (ImGui::Button("Load source trajectory (timestampUnix,x,y,z,qx,qy,qz,qw(estimated trajectory)"))
            {
                auto file_path = pfd::open_file("Select source trajectory file", fs::current_path().string(), {"CSV Files", "*.csv"}).result();
                if (!file_path.empty())
                {
                    Data::trajectory_est = load_trajectory_from_CSV_UNIXQ(file_path[0]);
                }
            }
            if (ImGui::Button("Load source trajectory (timestampLidar,timestampUnix,x,y,z,qx,qy,qz,qw(estimated trajectory)"))
            {
                auto file_path = pfd::open_file("Select source trajectory file", fs::current_path().string(), {"CSV Files", "*.csv"}).result();
                if (!file_path.empty())
                {
                    Data::trajectory_est = load_trajectory_from_CSV_LIDARUNIXQ(file_path[0]);
                }
            }
            if (ImGui::Button("Load source trajectory timestamp_nanoseconds,pose00,pose01,pose02,pose03,pose10,pose11,pose12,pose13,pose20,pose21,pose22,pose23,timestampUnix_nanoseconds,imuom,imufi,imuka ground truth trajectory"))
            {
                auto file_path = pfd::open_file("Select source trajectory file", fs::current_path().string(), {"CSV Files", "*.csv"}).result();
                if (!file_path.empty())
                {
                    Data::trajectory_est = load_trajectory_from_CSV_STEP1(file_path[0]);
                }
            }
            if (ImGui::Button("Load target trajectory (timestampLidar,x,y,z,r00,r01,r02,r10,r11,r12,r20,r21,r22(ground trajectory)"))
            {
                auto file_path = pfd::open_file("Select target trajectory file", fs::current_path().string(), {"CSV Files", "*.csv"}).result();
                if (!file_path.empty())
                {
                    Data::trajectory_gt = load_trajectory_from_CSV_LIDARROT(file_path[0]);
                }
            }
            if (ImGui::Button("Load target trajectory (timestampUnix,x,y,z,r00,r01,r02,r10,r11,r12,r20,r21,r22(ground truth trajectory)"))
            {
                auto file_path = pfd::open_file("Select target trajectory file", fs::current_path().string(), {"CSV Files", "*.csv"}).result();
                if (!file_path.empty())
                {
                    Data::trajectory_gt = load_trajectory_from_CSV_UNIXROT(file_path[0]);
                }
            }
            if (ImGui::Button("Load target trajectory (timestampLidar,timestampUnix,x,y,z,r00,r01,r02,r10,r11,r12,r20,r21,r22(ground truth trajectory)"))
            {
                auto file_path = pfd::open_file("Select target trajectory file", fs::current_path().string(), {"CSV Files", "*.csv"}).result();
                if (!file_path.empty())
                {
                    Data::trajectory_gt = load_trajectory_from_CSV_LIDARUNIXROT(file_path[0]);
                }
            }

            if (ImGui::Button("Load target trajectory (timestampUnix,x,y,z,qx,qy,qz,qw(ground truth trajectory)"))
            {
                auto file_path = pfd::open_file("Select target trajectory file", fs::current_path().string(), {"CSV Files", "*.csv"}).result();
                if (!file_path.empty())
                {
                    Data::trajectory_gt = load_trajectory_from_CSV_UNIXQ(file_path[0]);
                }
            }
            if (ImGui::Button("Load target trajectory (timestampLidar,timestampUnix,x,y,z,qx,qy,qz,qw(ground truth trajectory)"))
            {
                auto file_path = pfd::open_file("Select target trajectory file", fs::current_path().string(), {"CSV Files", "*.csv"}).result();
                if (!file_path.empty())
                {
                    Data::trajectory_gt = load_trajectory_from_CSV_LIDARUNIXQ(file_path[0]);
                }
            }
            if (ImGui::Button("Load target trajectory timestamp_nanoseconds,pose00,pose01,pose02,pose03,pose10,pose11,pose12,pose13,pose20,pose21,pose22,pose23,timestampUnix_nanoseconds,imuom,imufi,imuka ground truth trajectory"))
            {
                auto file_path = pfd::open_file("Select source trajectory file", fs::current_path().string(), {"CSV Files", "*.csv"}).result();
                if (!file_path.empty())
                {
                    Data::trajectory_gt = load_trajectory_from_CSV_STEP1(file_path[0]);
                }
            }
            if (ImGui::Button("Align source trajectory to target trajectory (1 iteration of optimization)"))
            {
                Eigen::MatrixXd AtPA(6, 6);
                AtPA.setZero();
                Eigen::MatrixXd AtPB(6, 1);
                AtPB.setZero();

                const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(Eigen::Affine3d(Data::trajectory_offset.matrix()));

                for (const auto &p : Data::trajectory_est)
                {
                    auto it = getInterpolatedPose(Data::trajectory_gt, p.first);
                    if (!it.isZero())
                    {
                        Eigen::Vector3d p_s = (Eigen::Affine3d(p.second.matrix())).translation();
                        Eigen::Vector3d p_t = (Eigen::Affine3d(it.matrix())).translation();

                        Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AtPA_;
                        point_to_point_source_to_target_tait_bryan_wc_AtPA_simplified(
                            AtPA_,
                            pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                            p_s.x(), p_s.y(), p_s.z(),
                            1, 0, 0, 0, 1, 0, 0, 0, 1);

                        Eigen::Matrix<double, 6, 1> AtPB_;
                        point_to_point_source_to_target_tait_bryan_wc_AtPB_simplified(
                            AtPB_,
                            pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                            p_s.x(), p_s.y(), p_s.z(),
                            1, 0, 0, 0, 1, 0, 0, 0, 1,
                            p_t.x(), p_t.y(), p_t.z());

                        AtPA.block<6, 6>(0, 0) += AtPA_;
                        AtPB.block<6, 1>(0, 0) -= AtPB_;
                    }
                }

                Eigen::SparseMatrix<double> AtPAc(6, 6);
                Eigen::SparseMatrix<double> AtPBc(6, 1);

                AtPAc = AtPA.sparseView();
                AtPBc = AtPB.sparseView();

                Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPAc);
                Eigen::SparseMatrix<double> x = solver.solve(AtPBc);
                std::vector<double> h_x;
                for (int k = 0; k < x.outerSize(); ++k)
                {
                    std::cout << "result pose updates" << std::endl;
                    for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
                    {
                        h_x.push_back(it.value());
                        std::cout << std::fixed << std::setprecision(6) << it.row() << " " << it.col() << " " << it.value() << std::endl;
                    }
                }

                if (h_x.size() == 6)
                {
                    int counter = 0;
                    TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(Eigen::Affine3d(Data::trajectory_offset.matrix()));

                    pose.px += h_x[counter++];
                    pose.py += h_x[counter++];
                    pose.pz += h_x[counter++];
                    pose.om += h_x[counter++];
                    pose.fi += h_x[counter++];
                    pose.ka += h_x[counter++];

                    auto m_pose_result = affine_matrix_from_pose_tait_bryan(pose);

                    Data::trajectory_offset = m_pose_result.matrix();
                    std::cout << "PairWiseICP::compute SUCCESS" << std::endl;
                    // return true;
                }
                else
                {
                    std::cout << "PairWiseICP::compute FAILED" << std::endl;
                }
            }
        }
        // if (ImGui::Button("Calculate RPE (Relative Pose Error)"))
        //{
        // }
        ImGui::End();
    }
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

    glPointSize(point_size);
    glBegin(GL_POINTS);
    glColor3f(1, 0, 0);
    for (const auto &p : Data::trajectory_gt)
    {
        glVertex3f(p.second(0, 3), p.second(1, 3), p.second(2, 3));
    }
    glColor3f(0, 1, 0);
    for (const auto &p : Data::trajectory_est)
    {
        auto tp = Data::trajectory_offset * p.second;
        glVertex3f(tp(0, 3), tp(1, 3), tp(2, 3));
    }
    glEnd();
    glPointSize(1);

    if (show_correspondences)
    {
        glBegin(GL_LINES);
        glColor3f(1, 1, 1);
        for (const auto &p : Data::trajectory_est)
        {
            auto it = getInterpolatedPose(Data::trajectory_gt, p.first);
            if (!it.isZero())
            {
                auto tp = Data::trajectory_offset * p.second;
                glVertex3f(tp(0, 3), tp(1, 3), tp(2, 3));
                glVertex3f(it(0, 3), it(1, 3), it(2, 3));
            }
        }
        glEnd();
    }

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();
	ImGui::NewFrame();

    ImGuizmo::BeginFrame();
    ImGuizmo::Enable(true);
    ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

    GLfloat projection[16];
    glGetFloatv(GL_PROJECTION_MATRIX, projection);

    GLfloat modelview[16];
    glGetFloatv(GL_MODELVIEW_MATRIX, modelview);

    if (move_source_trajectory_with_gizmo)
    {
        if (!is_ortho)
        {
            GLfloat projection[16];
            glGetFloatv(GL_PROJECTION_MATRIX, projection);

            GLfloat modelview[16];
            glGetFloatv(GL_MODELVIEW_MATRIX, modelview);

            ImGuizmo::Manipulate(&modelview[0], &projection[0], ImGuizmo::TRANSLATE | ImGuizmo::ROTATE_Z | ImGuizmo::ROTATE_X | ImGuizmo::ROTATE_Y, ImGuizmo::WORLD, m_gizmo, NULL);
        }
        else
        {
            ImGuizmo::Manipulate(m_ortho_gizmo_view, m_ortho_projection, ImGuizmo::TRANSLATE_X | ImGuizmo::TRANSLATE_Y | ImGuizmo::ROTATE_Z, ImGuizmo::WORLD, m_gizmo, NULL);
        }
        Data::trajectory_offset = Eigen::Map<Eigen::Matrix4f>(m_gizmo).cast<double>();
    }

    project_gui();

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
    glutCreateWindow("mandeye trajectory compare data viewer " HDMAPPING_VERSION_STRING);
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

int main(int argc, char *argv[])
{
    if (argc == 3)
    {
        Data::trajectory_gt = load_trajectory_from_CSV(argv[1]);
        Data::trajectory_est = load_trajectory_from_CSV(argv[2]);

        double start_gt = Data::trajectory_gt.begin()->first;
        double start_est = Data::trajectory_est.begin()->first;

        double dur_gt = Data::trajectory_gt.rbegin()->first - start_gt;
        double dur_est = Data::trajectory_est.rbegin()->first - start_est;

        std::cout << "GT duration: " << dur_gt << "s" << std::endl;
        std::cout << "EST duration: " << dur_est << "s" << std::endl;
        std::cout << "GT start: " << start_gt << "s" << std::endl;
        std::cout << "EST start: " << start_est << "s" << std::endl;

        // std::abort();
    }
    initGL(&argc, argv);
    glutDisplayFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutMouseWheelFunc(wheel);
    glutMainLoop();

    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGLUT_Shutdown();

    ImGui::DestroyContext();
    return 0;
}