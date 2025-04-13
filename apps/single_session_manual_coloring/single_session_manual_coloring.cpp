
#include <iostream>
#include <GL/freeglut.h>

#include "imgui.h"
#include "imgui_impl_glut.h"
#include "imgui_impl_opengl2.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <mutex>
#include <vector>

#include "color_las_loader.h"
#include "pfd_wrapper.hpp"
#include <execution>

#include <structures.h>
#include <transformations.h>
#include <observation_equations/codes/python-scripts/camera-metrics/equirectangular_camera_colinearity_tait_bryan_wc_jacobian.h>
#include <Eigen/Eigen>
#include <observation_equations/codes/common/include/cauchy.h>
#include <observation_equations/codes/python-scripts/camera-metrics/fisheye_camera_calibRT_tait_bryan_wc_jacobian.h>

#include <HDMapping/Version.hpp>

#include <glm.hpp>
#include <gtc/matrix_transform.hpp>

#include <session.h>
#include <filesystem>

#if 0

double fx = 2141.3412300023847;
double fy = 2141.3412300023847;
double cx = 1982.4503600047012;
double cy = 1472.7228631802407;
double k1 = -0.00042559601894193817;
double k2 = 0.003534402929232146;
double k3 = -0.0022518302398800826;
double k4 = 0.0001842010188374431;
double alpha = 0;

bool color = false;

GLuint tex1;

GLUquadric *sphere;

GLuint make_tex(const std::string &fn)
{
    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    // set the texture wrapping/filtering options (on the currently bound texture object)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // load and generate the texture
    int width, height, nrChannels;
    unsigned char *data = stbi_load(fn.c_str(), &width, &height, &nrChannels, 0);

    std::cout << "width: " << width << " height: " << height << " nrChannels: " << nrChannels << std::endl;

    if (data)
    {
        if (nrChannels == 1)
        {
            unsigned char *data3 = (unsigned char *)malloc(width * height * 3);

            int counter = 0;
            for (int wh = 0; wh < width * height; wh++)
            {
                data3[counter++] = data[wh];
                data3[counter++] = data[wh];
                data3[counter++] = data[wh];
            }
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data3);
            stbi_image_free(data3);
        }
        else if (nrChannels == 4)
        {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        }
        else
        {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
        }

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        // glGenerateMipmap(GL_TEXTURE_2D);
    }
    stbi_image_free(data);
    return tex;
}

float rot = 0;
float width = 34;
float height = 71;
const unsigned int window_width = 500;
const unsigned int window_height = 400;
int mouse_old_x, mouse_old_y;
int mouse_buttons = 0;
float rotate_x = 0.0, rotate_y = 0.0;
float translate_z = -90.0;
float translate_x, translate_y = 0.0;
bool gui_mouse_down{false};

void display();
void reshape(int w, int h);
void mouse(int glut_button, int state, int x, int y);
void motion(int x, int y);
bool initGL(int *argc, char **argv);

float imgui_co_size{1000.0f};
bool imgui_draw_co{true};

double CameraRotationZ = 0;
double CameraHeight = 0;

namespace SystemData
{
    std::vector<mandeye::PointRGB> points;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> clickedRay;
    int closestPointIndex{-1};
    std::vector<ImVec2> pointPickedImage;
    std::vector<Eigen::Vector3d> pointPickedPointCloud;

    unsigned char *imageData;
    int imageWidth, imageHeight, imageNrChannels;

    Eigen::Affine3d camera_pose = Eigen::Affine3d::Identity();

    int point_size = 1;
}

int main(int argc, char *argv[])
{
    TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(SystemData::camera_pose);
    // pose.om = M_PI * 0.5;
    // pose.fi = 0;
    // pose.ka = M_PI * 0.5;
    // pose.px = 0;
    // pose.py = 0;
    // pose.pz = 0; //-0.25;
    pose.om = M_PI * 0.5;
    pose.fi = 0;
    pose.ka = M_PI * 0.5;
    pose.px = 0.055;
    pose.py = 0.13;
    pose.pz = 0.085;

    SystemData::camera_pose = affine_matrix_from_pose_tait_bryan(pose);

    initGL(&argc, argv);
    glutDisplayFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutMainLoop();
}

void imagePicker(const std::string &name, ImTextureID tex1, std::vector<ImVec2> &point_picked, std::vector<ImVec2> point_pickedInPointcloud)
{
    ImGuiIO &io = ImGui::GetIO();
    static float zoom = 0.1f;
    const int Tex_width = 5000;
    const int Tex_height = 2500;

    float speed = io.KeyShift ? 10.f : 1.f;
    int transX = 0;
    int transY = 0;
    if (io.KeysDown[io.KeyMap[ImGuiKey_UpArrow]])
    {
        transY = -10 * speed;
    }
    if (io.KeysDown[io.KeyMap[ImGuiKey_DownArrow]])
    {
        transY = 10 * speed;
    }
    if (io.KeysDown[io.KeyMap[ImGuiKey_LeftArrow]])
    {
        transX = -10 * speed;
    }
    if (io.KeysDown[io.KeyMap[ImGuiKey_RightArrow]])
    {
        transX = 10 * speed;
    }
    if (io.KeysDown[io.KeyMap[ImGuiKey_PageUp]])
    {
        zoom *= 1.0f + 0.01f * speed;
    }

    if (io.KeysDown[io.KeyMap[ImGuiKey_PageDown]])
    {
        zoom /= 1.00f + 0.01f * speed;
    }

    ImVec2 uv_min = ImVec2(0.0f, 0.0f);                 // Top-left
    ImVec2 uv_max = ImVec2(1.0f, 1.0f);                 // Lower-right
    ImVec4 tint_col = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);   // No tint
    ImVec4 border_col = ImVec4(1.0f, 1.0f, 1.0f, 0.5f); // 50% opaque white

    ImGuiWindowFlags window_flags = ImGuiWindowFlags_HorizontalScrollbar | ImGuiWindowFlags_NoScrollWithMouse;

    ImGui::InputFloat("zoom", &zoom, 0.1f, 0.5f);
    float my_tex_w = Tex_width * zoom;
    float my_tex_h = Tex_height * zoom;
    const ImVec2 child_size{ImGui::GetWindowWidth() * 1.0f, ImGui::GetWindowHeight() * 0.5f};

    ImGui::Checkbox("color", &color);

    struct point_pair
    {
        ImVec2 p1;
        bool visible1;
        ImVec2 p2;
        bool visible2;
    };

    auto draw_zoom_pick_point = [my_tex_w, my_tex_h, tint_col, border_col](const ImTextureID &tex, std::vector<ImVec2> &point_picked)
    {
        ImVec2 img_start = ImGui::GetItemRectMin();
        ImGuiIO &io = ImGui::GetIO();
        ImGui::BeginTooltip();
        float region_sz = 32.0f;
        float region_x = io.MousePos.x - img_start.x - region_sz * 0.5f;
        float region_y = io.MousePos.y - img_start.y - region_sz * 0.5f;

        // add point
        if (io.MouseClicked[2] && io.KeyShift)
        {
            ImVec2 picked_point{(io.MousePos.x - img_start.x) / my_tex_w, (io.MousePos.y - img_start.y) / my_tex_h};
            point_picked.push_back(picked_point);
        }

        // remove last point
        if (io.MouseClicked[1] && io.KeyShift && point_picked.size() > 0)
        {
            point_picked.pop_back();
        }
        float local_zoom = 4.0f;
        if (region_x < 0.0f)
        {
            region_x = 0.0f;
        }
        else if (region_x > my_tex_w - region_sz)
        {
            region_x = my_tex_w - region_sz;
        }
        if (region_y < 0.0f)
        {
            region_y = 0.0f;
        }
        else if (region_y > my_tex_h - region_sz)
        {
            region_y = my_tex_h - region_sz;
        }
        ImVec2 uv0 = ImVec2((region_x) / my_tex_w, (region_y) / my_tex_h);
        ImVec2 uv1 = ImVec2((region_x + region_sz) / my_tex_w, (region_y + region_sz) / my_tex_h);
        ImGui::Image(tex, ImVec2(region_sz * local_zoom, region_sz * local_zoom), uv0, uv1, tint_col, border_col);
        ImVec2 img_start_loc = ImGui::GetItemRectMin();
        ImVec2 img_sz = {ImGui::GetItemRectMax().x - ImGui::GetItemRectMin().x, ImGui::GetItemRectMax().y - ImGui::GetItemRectMin().y};
        ImVec2 window_center = ImVec2(img_start_loc.x + img_sz.x * 0.5f, img_start_loc.y + img_sz.y * 0.5f);
        ImGui::GetForegroundDrawList()->AddLine({window_center.x - 10, window_center.y}, {window_center.x + 10, window_center.y}, IM_COL32(0, 255, 0, 200), 1);
        ImGui::GetForegroundDrawList()->AddLine({window_center.x, window_center.y - 10}, {window_center.x, window_center.y + 10}, IM_COL32(0, 255, 0, 200), 1);
        ImGui::EndTooltip();
    };

    ImGui::BeginChild((name + "_child1").c_str(), child_size, false, window_flags);
    {
        ImGui::Image(tex1, ImVec2(my_tex_w, my_tex_h), uv_min, uv_max, tint_col, border_col);
        const ImVec2 view_port_start = ImGui::GetWindowPos();
        const ImVec2 view_port_end{view_port_start.x + ImGui::GetWindowWidth(), view_port_start.y + ImGui::GetWindowHeight()};
        ImVec2 img_start = ImGui::GetItemRectMin();
        for (int i = 0; i < point_picked.size(); i++)
        {
            const auto &p = point_picked[i];

            ImVec2 center{img_start.x + p.x * my_tex_w, img_start.y + p.y * my_tex_h};

            if (center.x > view_port_start.x && center.x < view_port_end.x && center.y > view_port_start.y && center.y < view_port_end.y)
            {
                char data[16];
                snprintf(data, 16, "%d", i);

                ImGui::GetForegroundDrawList()->AddLine({center.x - 10, center.y}, {center.x + 10, center.y}, IM_COL32(255, 255, 0, 200), 1);
                ImGui::GetForegroundDrawList()->AddLine({center.x, center.y - 10}, {center.x, center.y + 10}, IM_COL32(255, 255, 0, 200), 1);
                ImGui::GetForegroundDrawList()->AddText({center.x, center.y + 10}, IM_COL32(255, 0, 0, 200), data);
                if (i < point_pickedInPointcloud.size())
                {
                    ImGui::GetForegroundDrawList()->AddLine({center.x, center.y}, point_pickedInPointcloud.at(i), IM_COL32(255, 0, 0, 200), 1);

                    ImGui::GetForegroundDrawList()->AddText({point_pickedInPointcloud.at(i)}, IM_COL32(255, 0, 0, 200), data);
                }
            }
        }
        if (ImGui::IsItemHovered())
        {
            draw_zoom_pick_point(tex1, point_picked);
        }
    }
    ImGui::SetScrollX(ImGui::GetScrollX() + transX);
    ImGui::SetScrollY(ImGui::GetScrollY() + transY);
    ImGui::EndChild();
}

ImVec2 UnprojectPoint(const Eigen::Vector3d &point)
{
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];

    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);

    GLdouble winX, winY, winZ;
    gluProject(point[0], point[1], point[2], modelview, projection, viewport, &winX, &winY, &winZ);
    return {static_cast<float>(winX), static_cast<float>(viewport[3] - winY)};
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> GetRay(int x, int y)
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

    Eigen::Vector3d position;
    Eigen::Vector3d direction;
    gluUnProject(winX, winY, 0, modelview, projection, viewport, &posXnear, &posYnear, &posZnear);
    gluUnProject(winX, winY, -1000, modelview, projection, viewport, &posXfar, &posYfar, &posZfar);

    position.x() = posXnear;
    position.y() = posYnear;
    position.z() = posZnear;

    direction.x() = posXfar - posXnear;
    direction.y() = posYfar - posYnear;
    direction.z() = posZfar - posZnear;

    direction.normalize();

    return {position, direction};
}

double GetDistanceToRay(const Eigen::Vector3d &qureyPoint, const std::pair<Eigen::Vector3d, Eigen::Vector3d> &ray)
{
    return ray.second.cross(qureyPoint - ray.first).norm();
}

std::vector<mandeye::PointRGB> ApplyColorToPointcloud(const std::vector<mandeye::PointRGB> &pointsRGB, const unsigned char *imageData, int imageWidth, int imageHeight, int nrChannels, const Eigen::Affine3d &transfom)
{
    std::vector<mandeye::PointRGB> newCloud(pointsRGB.size());
    std::transform(std::execution::par_unseq, pointsRGB.begin(), pointsRGB.end(), newCloud.begin(), [&](mandeye::PointRGB p)
                   {
            TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(transfom);
            double du, dv;
            equrectangular_camera_colinearity_tait_bryan_wc(du,dv, imageHeight, imageWidth,
                M_PI, pose.px, pose.py, pose.pz, pose.om, pose.fi, pose.ka,
                p.point.x(),
                p.point.y(),
                p.point.z());
            int u = std::round(du);
            int v = std::round(dv);
            if (u > 0 && v > 0 && u < imageWidth && v < imageHeight)
            {
                int index = (v * imageWidth + u) * nrChannels;
                unsigned char red = imageData[index];
                unsigned char green = imageData[index + 1];
                unsigned char blue = imageData[index + 2];
                p.rgb = { 1.f * red / 256.f,1.f * green / 256.f, 1.f * blue / 256.f, 1.f };
            }
        return p; });
    return newCloud;
}

std::vector<mandeye::PointRGB> ApplyColorToPointcloudFishEye(const std::vector<mandeye::PointRGB> &pointsRGB, const unsigned char *imageData, int imageWidth, int imageHeight, int nrChannels, const Eigen::Affine3d &transfom)
{
    std::vector<mandeye::PointRGB> newCloud(pointsRGB.size());
    std::transform(std::execution::par_unseq, pointsRGB.begin(), pointsRGB.end(), newCloud.begin(), [&](mandeye::PointRGB p)
                   {
            TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(transfom);
            double du, dv;
            //equrectangular_camera_colinearity_tait_bryan_wc(du,dv, imageHeight, imageWidth,
            //    M_PI, pose.px, pose.py, pose.pz, pose.om, pose.fi, pose.ka,
            //    p.point.x(),
            //    p.point.y(),
            //    p.point.z());

            projection_fisheye_camera_tait_bryan_wc(du, dv, fx, fy, cx, cy,
                                                    pose.px, pose.py, pose.pz, pose.om, pose.fi, pose.ka,
                                                    p.point.x(), p.point.y(), p.point.z(), k1, k2, k3, k4, alpha);

            int u = std::round(du);
            int v = std::round(dv);
            if (u > 0 && v > 0 && u < imageWidth && v < imageHeight)
            {
                int index = (v * imageWidth + u) * nrChannels;
                unsigned char red = imageData[index];
                unsigned char green = imageData[index + 1];
                unsigned char blue = imageData[index + 2];
                p.rgb = { 1.f * red / 256.f,1.f * green / 256.f, 1.f * blue / 256.f, 1.f };
            }
        return p; });
    return newCloud;
}

void ImGuiLoadSaveButtons()
{

    namespace SD = SystemData;
    if (ImGui::Button("Load Image"))
    {
        const auto input_file_names = mandeye::fd::OpenFileDialog("Choose Image", mandeye::fd::ImageFilter, false);
        if (input_file_names.size())
        {
            // std::cout << "1" << std::endl;
            tex1 = make_tex(input_file_names.front());
            // std::cout << "2" << std::endl;
            SD::imageData = stbi_load(input_file_names.front().c_str(), &SD::imageWidth, &SD::imageHeight, &SD::imageNrChannels, 0);
            // std::cout << "3" << std::endl;
        }
        // std::cout << "4" << std::endl;
        SystemData::points = ApplyColorToPointcloud(SystemData::points, SystemData::imageData, SystemData::imageWidth, SystemData::imageHeight, SystemData::imageNrChannels, SystemData::camera_pose);
        // std::cout << "5" << std::endl;
    }
    ImGui::SameLine();
    if (ImGui::Button("Load Poincloud"))
    {
        const auto input_file_names = mandeye::fd::OpenFileDialog("Choose Pointcloud", mandeye::fd::LazFilter, false);
        if (!input_file_names.empty())
        {
            auto points = mandeye::load(input_file_names.front());
            SystemData::points.resize(points.size());
            std::transform(points.begin(), points.end(), SystemData::points.begin(), [&](const mandeye::Point &p)
                           { return p; });
        }
        SystemData::points = ApplyColorToPointcloud(SystemData::points, SystemData::imageData, SystemData::imageWidth, SystemData::imageHeight, SystemData::imageNrChannels, SystemData::camera_pose);
    }
    ImGui::SameLine();
    if (ImGui::Button("Save Poincloud"))
    {
        const auto input_file_names = mandeye::fd::SaveFileDialog("Choose Pointcloud", mandeye::fd::LazFilter);
        if (!input_file_names.empty())
        {
            mandeye::saveLaz(input_file_names, SD::points);
        }
    }
    ImGui::SameLine();
}

void optimize()
{
    if (SystemData::pointPickedPointCloud.size() == SystemData::pointPickedImage.size() && SystemData::pointPickedPointCloud.size() >= 5)
    {
        std::vector<Eigen::Triplet<double>> tripletListA;
        std::vector<Eigen::Triplet<double>> tripletListP;
        std::vector<Eigen::Triplet<double>> tripletListB;

        TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(SystemData::camera_pose);
        for (int i = 0; i < SystemData::pointPickedImage.size(); i++)
        {

            Eigen::Matrix<double, 2, 1> delta;
            observation_equation_equrectangular_camera_colinearity_tait_bryan_wc(delta, SystemData::imageHeight, SystemData::imageWidth, M_PI,
                                                                                 pose.px, pose.py, pose.pz, pose.om, pose.fi, pose.ka,
                                                                                 SystemData::pointPickedPointCloud[i].x(),
                                                                                 SystemData::pointPickedPointCloud[i].y(),
                                                                                 SystemData::pointPickedPointCloud[i].z(),
                                                                                 SystemData::pointPickedImage[i].x * SystemData::imageWidth,
                                                                                 SystemData::pointPickedImage[i].y * SystemData::imageHeight);

            Eigen::Matrix<double, 2, 9, Eigen::RowMajor> jacobian;
            observation_equation_equrectangular_camera_colinearity_tait_bryan_wc_jacobian(jacobian, SystemData::imageHeight, SystemData::imageWidth, M_PI,
                                                                                          pose.px, pose.py, pose.pz, pose.om, pose.fi, pose.ka,
                                                                                          SystemData::pointPickedPointCloud[i].x(),
                                                                                          SystemData::pointPickedPointCloud[i].y(),
                                                                                          SystemData::pointPickedPointCloud[i].z(),
                                                                                          SystemData::pointPickedImage[i].x, SystemData::pointPickedImage[i].y);

            int ir = tripletListB.size();
            int ic_camera = 0;

            tripletListA.emplace_back(ir, ic_camera, -jacobian(0, 0));
            tripletListA.emplace_back(ir, ic_camera + 1, -jacobian(0, 1));
            tripletListA.emplace_back(ir, ic_camera + 2, -jacobian(0, 2));
            tripletListA.emplace_back(ir, ic_camera + 3, -jacobian(0, 3));
            tripletListA.emplace_back(ir, ic_camera + 4, -jacobian(0, 4));
            tripletListA.emplace_back(ir, ic_camera + 5, -jacobian(0, 5));
            tripletListA.emplace_back(ir + 1, ic_camera, -jacobian(1, 0));
            tripletListA.emplace_back(ir + 1, ic_camera + 1, -jacobian(1, 1));
            tripletListA.emplace_back(ir + 1, ic_camera + 2, -jacobian(1, 2));
            tripletListA.emplace_back(ir + 1, ic_camera + 3, -jacobian(1, 3));
            tripletListA.emplace_back(ir + 1, ic_camera + 4, -jacobian(1, 4));
            tripletListA.emplace_back(ir + 1, ic_camera + 5, -jacobian(1, 5));
            tripletListP.emplace_back(ir, ir, cauchy(delta(0, 0), 1));
            tripletListP.emplace_back(ir + 1, ir + 1, cauchy(delta(1, 0), 1));
            tripletListB.emplace_back(ir, 0, delta(0, 0));
            tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
        }

        Eigen::SparseMatrix<double> matA(tripletListB.size(), 6);
        Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
        Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

        matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
        matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
        matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

        Eigen::SparseMatrix<double> AtPA(6, 6);
        Eigen::SparseMatrix<double> AtPB(6, 1);

        {
            Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
            AtPA = (AtP)*matA;
            AtPB = (AtP)*matB;
        }

        tripletListA.clear();
        tripletListP.clear();
        tripletListB.clear();

        // std::cout << "AtPA.size: " << AtPA.size() << std::endl;
        // std::cout << "AtPB.size: " << AtPB.size() << std::endl;

        // std::cout << "start solving AtPA=AtPB" << std::endl;
        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);

        // std::cout << "x = solver.solve(AtPB)" << std::endl;
        Eigen::SparseMatrix<double> x = solver.solve(AtPB);

        std::vector<double> h_x;

        for (int k = 0; k < x.outerSize(); ++k)
        {
            for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            {
                h_x.push_back(it.value());
            }
        }

        if (h_x.size() == 6)
        {
            // for (size_t i = 0; i < h_x.size(); i++)
            //{
            //     std::cout << h_x[i] << std::endl;
            // }
            // std::cout << "AtPA=AtPB SOLVED" << std::endl;
            // std::cout << "update" << std::endl;

            int counter = 0;
            pose.px += h_x[counter++] * 0.1;
            pose.py += h_x[counter++] * 0.1;
            pose.pz += h_x[counter++] * 0.1;
            pose.om += h_x[counter++] * 0.1;
            pose.fi += h_x[counter++] * 0.1;
            pose.ka += h_x[counter++] * 0.1;

            SystemData::camera_pose = affine_matrix_from_pose_tait_bryan(pose);
        }
        else
        {
            std::cout << "AtPA=AtPB FAILED" << std::endl;
        }
    }
    else
    {
        std::cout << "Please mark at least 5 proper image to cloud correspondances" << std::endl;
    }
}

void optimize_fish_eye()
{
    if (SystemData::pointPickedPointCloud.size() == SystemData::pointPickedImage.size() && SystemData::pointPickedPointCloud.size() >= 5)
    {
        std::vector<Eigen::Triplet<double>> tripletListA;
        std::vector<Eigen::Triplet<double>> tripletListP;
        std::vector<Eigen::Triplet<double>> tripletListB;

        TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(SystemData::camera_pose);
        for (int i = 0; i < SystemData::pointPickedImage.size(); i++)
        {

            Eigen::Matrix<double, 2, 1> delta;
            // observation_equation_equrectangular_camera_colinearity_tait_bryan_wc(delta, SystemData::imageHeight, SystemData::imageWidth, M_PI,
            //                                                                      pose.px, pose.py, pose.pz, pose.om, pose.fi, pose.ka,
            //                                                                      SystemData::pointPickedPointCloud[i].x(),
            //                                                                      SystemData::pointPickedPointCloud[i].y(),
            //                                                                      SystemData::pointPickedPointCloud[i].z(),
            //                                                                      SystemData::pointPickedImage[i].x * SystemData::imageWidth,
            //                                                                      SystemData::pointPickedImage[i].y * SystemData::imageHeight);

            observation_equation_fisheye_camera_tait_bryan_wc(delta, fx, fy, cx, cy,
                                                              pose.px, pose.py, pose.pz, pose.om, pose.fi, pose.ka,
                                                              SystemData::pointPickedPointCloud[i].x(),
                                                              SystemData::pointPickedPointCloud[i].y(),
                                                              SystemData::pointPickedPointCloud[i].z(),
                                                              SystemData::pointPickedImage[i].x * SystemData::imageWidth,
                                                              SystemData::pointPickedImage[i].y * SystemData::imageHeight,
                                                              k1, k2, k3, k4, alpha);

            Eigen::Matrix<double, 2, 6> jacobian;
            // observation_equation_equrectangular_camera_colinearity_tait_bryan_wc_jacobian(jacobian, SystemData::imageHeight, SystemData::imageWidth, M_PI,
            //                                                                               pose.px, pose.py, pose.pz, pose.om, pose.fi, pose.ka,
            //                                                                               SystemData::pointPickedPointCloud[i].x(),
            //                                                                               SystemData::pointPickedPointCloud[i].y(),
            //                                                                               SystemData::pointPickedPointCloud[i].z(),
            //                                                                               SystemData::pointPickedImage[i].x, SystemData::pointPickedImage[i].y);

            observation_equation_fisheye_camera_tait_bryan_wc_jacobian(jacobian, fx, fy,
                                                                       pose.px, pose.py, pose.pz, pose.om, pose.fi, pose.ka,
                                                                       SystemData::pointPickedPointCloud[i].x(),
                                                                       SystemData::pointPickedPointCloud[i].y(),
                                                                       SystemData::pointPickedPointCloud[i].z(),
                                                                       k1, k2, k3, k4, alpha);

            int ir = tripletListB.size();
            int ic_camera = 0;

            tripletListA.emplace_back(ir, ic_camera, -jacobian(0, 0));
            tripletListA.emplace_back(ir, ic_camera + 1, -jacobian(0, 1));
            tripletListA.emplace_back(ir, ic_camera + 2, -jacobian(0, 2));
            tripletListA.emplace_back(ir, ic_camera + 3, -jacobian(0, 3));
            tripletListA.emplace_back(ir, ic_camera + 4, -jacobian(0, 4));
            tripletListA.emplace_back(ir, ic_camera + 5, -jacobian(0, 5));
            tripletListA.emplace_back(ir + 1, ic_camera, -jacobian(1, 0));
            tripletListA.emplace_back(ir + 1, ic_camera + 1, -jacobian(1, 1));
            tripletListA.emplace_back(ir + 1, ic_camera + 2, -jacobian(1, 2));
            tripletListA.emplace_back(ir + 1, ic_camera + 3, -jacobian(1, 3));
            tripletListA.emplace_back(ir + 1, ic_camera + 4, -jacobian(1, 4));
            tripletListA.emplace_back(ir + 1, ic_camera + 5, -jacobian(1, 5));

            tripletListP.emplace_back(ir, ir, cauchy(delta(0, 0), 1));
            tripletListP.emplace_back(ir + 1, ir + 1, cauchy(delta(1, 0), 1));

            tripletListB.emplace_back(ir, 0, delta(0, 0));
            tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
        }

        Eigen::SparseMatrix<double> matA(tripletListB.size(), 6);
        Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
        Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

        matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
        matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
        matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

        Eigen::SparseMatrix<double> AtPA(6, 6);
        Eigen::SparseMatrix<double> AtPB(6, 1);

        {
            Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
            AtPA = (AtP)*matA;
            AtPB = (AtP)*matB;
        }

        tripletListA.clear();
        tripletListP.clear();
        tripletListB.clear();

        // std::cout << "AtPA.size: " << AtPA.size() << std::endl;
        // std::cout << "AtPB.size: " << AtPB.size() << std::endl;

        // std::cout << "start solving AtPA=AtPB" << std::endl;
        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);

        // std::cout << "x = solver.solve(AtPB)" << std::endl;
        Eigen::SparseMatrix<double> x = solver.solve(AtPB);

        std::vector<double> h_x;

        for (int k = 0; k < x.outerSize(); ++k)
        {
            for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            {
                h_x.push_back(it.value());
            }
        }

        if (h_x.size() == 6)
        {
            // for (size_t i = 0; i < h_x.size(); i++)
            //{
            //     std::cout << h_x[i] << std::endl;
            // }
            // std::cout << "AtPA=AtPB SOLVED" << std::endl;
            // std::cout << "update" << std::endl;

            int counter = 0;
            pose.px += h_x[counter++] * 0.1;
            pose.py += h_x[counter++] * 0.1;
            pose.pz += h_x[counter++] * 0.1;
            pose.om += h_x[counter++] * 0.1;
            pose.fi += h_x[counter++] * 0.1;
            pose.ka += h_x[counter++] * 0.1;

            SystemData::camera_pose = affine_matrix_from_pose_tait_bryan(pose);
        }
        else
        {
            std::cout << "AtPA=AtPB FAILED" << std::endl;
        }
    }
    else
    {
        std::cout << "Please mark at least 5 proper image to cloud correspondances" << std::endl;
    }
}

void display()
{
    ImGuiIO &io = ImGui::GetIO();
    glViewport(0, 0, (GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);

    reshape((GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(translate_x, translate_y, translate_z);
    glRotatef(rotate_x, 1.0, 0.0, 0.0);
    glRotatef(rotate_y, 0.0, 0.0, 1.0);

    //////////
    // glColor3f(p.rgb.data());
    glPointSize(SystemData::point_size);
    glBegin(GL_POINTS);
    for (const auto &p : SystemData::points)
    {
        if (color)
        {
            glColor3fv(p.rgb.data());
        }
        else
        {
            glColor3f(p.intensity - 100, p.intensity - 100, p.intensity - 100);
            // p.intensity
        }

        glVertex3dv(p.point.data());
    }
    glEnd();
    //////////////////////////////////
    glPointSize(10);
    glBegin(GL_POINTS);
    for (const auto &p : SystemData::pointPickedPointCloud)
    {
        glColor3f(1.f, 0.f, 0.f);
        glVertex3dv(p.data());
    }
    glEnd();

    if (imgui_draw_co)
    {
        glLineWidth(5);
        glBegin(GL_LINES);
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(imgui_co_size, 0.0f, 0.0f);

        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, imgui_co_size, 0.0f);

        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, imgui_co_size);
        glEnd();
    }
    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();

    std::vector<ImVec2> picked3DPoints(SystemData::pointPickedPointCloud.size());
    std::transform(SystemData::pointPickedPointCloud.begin(), SystemData::pointPickedPointCloud.end(), picked3DPoints.begin(), UnprojectPoint);

    ImGui::Begin("Image");
    ImGuiLoadSaveButtons();
    if (ImGui::Button("apply color to PC (fishEye)"))
    {
        SystemData::points = ApplyColorToPointcloudFishEye(SystemData::points, SystemData::imageData, SystemData::imageWidth, SystemData::imageHeight, SystemData::imageNrChannels, SystemData::camera_pose);
    }

    if (ImGui::Button("Optimize"))
    {
        optimize();

        TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(SystemData::camera_pose);
        std::cout << "pose" << std::endl;
        std::cout << "px " << pose.px << std::endl;
        std::cout << "py " << pose.py << std::endl;
        std::cout << "pz " << pose.pz << std::endl;
        std::cout << "om " << pose.om << std::endl;
        std::cout << "fi " << pose.fi << std::endl;
        std::cout << "ka " << pose.ka << std::endl;
        SystemData::points = ApplyColorToPointcloud(SystemData::points, SystemData::imageData, SystemData::imageWidth, SystemData::imageHeight, SystemData::imageNrChannels, SystemData::camera_pose);
    }
    ImGui::SameLine();

    if (ImGui::Button("Optimize(fisheye)"))
    {
        optimize_fish_eye();

        TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(SystemData::camera_pose);
        std::cout << "pose" << std::endl;
        std::cout << "px " << pose.px << std::endl;
        std::cout << "py " << pose.py << std::endl;
        std::cout << "pz " << pose.pz << std::endl;
        std::cout << "om " << pose.om << std::endl;
        std::cout << "fi " << pose.fi << std::endl;
        std::cout << "ka " << pose.ka << std::endl;
        SystemData::points = ApplyColorToPointcloudFishEye(SystemData::points, SystemData::imageData, SystemData::imageWidth, SystemData::imageHeight, SystemData::imageNrChannels, SystemData::camera_pose);
    }
    ImGui::SameLine();

    if (ImGui::Button("Optimize x 100"))
    {
        for (int i = 0; i < 100; i++)
        {
            optimize();
            if (i % 10 == 0)
            {
                std::cout << "iteration: " << i << " of 100" << std::endl;
            }
        }
        TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(SystemData::camera_pose);
        std::cout << "pose" << std::endl;
        std::cout << "px " << pose.px << std::endl;
        std::cout << "py " << pose.py << std::endl;
        std::cout << "pz " << pose.pz << std::endl;
        std::cout << "om " << pose.om << std::endl;
        std::cout << "fi " << pose.fi << std::endl;
        std::cout << "ka " << pose.ka << std::endl;
        SystemData::points = ApplyColorToPointcloud(SystemData::points, SystemData::imageData, SystemData::imageWidth, SystemData::imageHeight, SystemData::imageNrChannels, SystemData::camera_pose);
    }
    ImGui::InputInt("point_size", &SystemData::point_size);

    if (SystemData::point_size < 1)
    {
        SystemData::point_size = 1;
    }

    imagePicker("ImagePicker", (ImTextureID)tex1, SystemData::pointPickedImage, picked3DPoints);

    ImGui::Text("!!! SELECT at least 5 image <--> point cloud pairs !!!");
    ImGui::Text("To pick image: press shift and middle mouse button (cursor on image)");
    ImGui::Text("To pick point in 3D: press shift and middle mouse button (cursor on point cloud)");
    ImGui::Text("page up: zoom in");
    ImGui::Text("page down: zoom out");
    ImGui::Text("arrows: move image");

    // 2D Points Picked
    ImGui::BeginChild("2D", ImVec2(300, 0), true);
    ImGui::Text("2D:");
    for (auto it = SystemData::pointPickedImage.begin(); it != SystemData::pointPickedImage.end(); it++)
    {
        auto index = std::distance(SystemData::pointPickedImage.begin(), it);
        const auto &p = *it;
        ImGui::Text("%d : %.1f,%.1f", index, p.x, p.y);
        ImGui::SameLine();
        const auto label = std::string("-##2s") + std::to_string(index);
        if (ImGui::Button(label.c_str()))
        {
            SystemData::pointPickedImage.erase(it);
            break;
        }
    }

    ImGui::EndChild();
    ImGui::SameLine();

    // 3D Points Picked
    ImGui::BeginChild("3D", ImVec2(300, 0), true);
    ImGui::Text("3D:");
    for (auto it = SystemData::pointPickedPointCloud.begin(); it != SystemData::pointPickedPointCloud.end(); it++)
    {
        const auto &p = *it;
        const auto index = std::distance(SystemData::pointPickedPointCloud.begin(), it);
        auto prev = it != SystemData::pointPickedPointCloud.begin() ? it - 1 : SystemData::pointPickedPointCloud.end();
        auto next = it + 1 != SystemData::pointPickedPointCloud.end() ? it + 1 : SystemData::pointPickedPointCloud.end();

        const auto label = std::string("-##2s") + std::to_string(index);

        if (ImGui::Button(label.c_str()))
        {
            SystemData::pointPickedPointCloud.erase(it);
            break;
        }
        const auto labelUp = std::string("U##2s") + std::to_string(index);
        const auto labelDn = std::string("D##2s") + std::to_string(index);
        if (prev != SystemData::pointPickedPointCloud.end())
        {
            ImGui::SameLine();
            if (ImGui::Button(labelUp.c_str()))
            {
                std::swap(*it, *prev);
                break;
            }
        }
        if (next != SystemData::pointPickedPointCloud.end())
        {
            ImGui::SameLine();
            if (ImGui::Button(labelDn.c_str()))
            {
                std::swap(*it, *next);
                break;
            }
        }
        ImGui::SameLine();
        ImGui::Text("%d: %.1f,%.1f,%.1f", index, p.x(), p.y(), p.z());
    }
    ImGui::EndChild();

    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
    glutSwapBuffers();
    glutPostRedisplay();
}

void mouse(int glut_button, int state, int x, int y)
{
    ImGui_ImplGLUT_MouseFunc(glut_button, state, x, y);
    ImGuiIO &io = ImGui::GetIO();
    int button = -1;
    if (glut_button == GLUT_LEFT_BUTTON)
        button = 0;
    if (glut_button == GLUT_RIGHT_BUTTON)
        button = 1;
    if (glut_button == GLUT_MIDDLE_BUTTON)
        button = 2;

    if (!io.WantCaptureMouse)
    {
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

        if (state == GLUT_DOWN)
        {
            if (glut_button == GLUT_MIDDLE_BUTTON && io.KeyShift)
            {
                SystemData::clickedRay = GetRay(x, y);

                std::mutex mtx;
                std::pair<double, int> distanceIndexPair{std::numeric_limits<double>::max(), -1};

                std::for_each(std::execution::par_unseq, SystemData::points.begin(), SystemData::points.end(), [&](const mandeye::PointRGB &p)
                              {
                    double D = GetDistanceToRay(p.point, SystemData::clickedRay);
                    std::lock_guard<std::mutex> guard(mtx);
                    if (D < distanceIndexPair.first)
                    {
                        // Assume that SystemData::point is an array-like type implementation, naked pointer arithmetic ahead:
                        const int index = &p - &SystemData::points.front();
                        assert(index >= 0);
                        assert(index < SystemData::points.size());
                        distanceIndexPair = { D, index };
                    } });

                if (distanceIndexPair.second > 0)
                {
                    const auto &[distance, index] = distanceIndexPair;
                    std::cout << "Closest point found, distance " << distance << std::endl;
                    SystemData::closestPointIndex = distanceIndexPair.second;
                    SystemData::pointPickedPointCloud.push_back(SystemData::points.at(index).point);
                }
            }
            if (glut_button == GLUT_RIGHT_BUTTON && io.KeyShift)
            {
                if (SystemData::pointPickedPointCloud.size() > 0)
                {
                    SystemData::pointPickedPointCloud.pop_back();
                }
            }
        }
    }
}

void motion(int x, int y)
{
    ImGui_ImplGLUT_MotionFunc(x, y);
    ImGuiIO &io = ImGui::GetIO();

    if (!io.WantCaptureMouse)
    {
        float dx, dy;
        dx = (float)(x - mouse_old_x);
        dy = (float)(y - mouse_old_y);
        gui_mouse_down = mouse_buttons > 0;
        if (mouse_buttons & 1)
        {
            rotate_x += dy * 0.2f;
            rotate_y += dx * 0.2f;
        }
        else if (mouse_buttons & 4)
        {
            translate_z += dy * 0.05f;
        }
        else if (mouse_buttons & 3)
        {
            translate_x += dx * 0.05f;
            translate_y -= dy * 0.05f;
        }
        mouse_old_x = x;
        mouse_old_y = y;
    }
    glutPostRedisplay();
}

void reshape(int w, int h)
{
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (GLfloat)w / (GLfloat)h, 0.01, 10000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

bool initGL(int *argc, char **argv)
{
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
    glutInitWindowSize(window_width, window_height);
    glutCreateWindow("MANDEYE with GoPro MAX manual coloring " HDMAPPING_VERSION_STRING);
    glutDisplayFunc(display);
    glutMotionFunc(motion);

    // default initialization
    glClearColor(1.0, 1.0, 1.0, 1.0);
    // glEnable(GL_DEPTH_TEST);

    glViewport(0, 0, window_width, window_height);

    // projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (GLfloat)window_width / (GLfloat)window_height, 0.01,
                   1000.0);
    glutReshapeFunc(reshape);
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;

    ImGui::StyleColorsDark();
    ImGui_ImplGLUT_Init();
    ImGui_ImplGLUT_InstallFuncs();
    ImGui_ImplOpenGL2_Init();

    return true;
}

#endif

#if 0
GLuint texture;
float cameraYaw = 0.0f;
float cameraPitch = -90.0f;
// Load image as OpenGL texture
GLuint loadTexture(const char *filename)
{
    int width, height, channels;
    unsigned char *data = stbi_load(filename, &width, &height, &channels, STBI_rgb_alpha);
    if (!data)
    {
        std::cerr << "Failed to load: " << filename << std::endl;
        exit(1);
    }

    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
    stbi_image_free(data);

    return tex;
}

// Draw sphere with correct UV mapping
void drawSphere(float radius, int slices, int stacks)
{
    for (int i = 0; i < stacks; ++i)
    {
        float lat0 = M_PI * (-0.5 + (float)(i) / stacks);
        float lat1 = M_PI * (-0.5 + (float)(i + 1) / stacks);
        float z0 = sin(lat0), zr0 = cos(lat0);
        float z1 = sin(lat1), zr1 = cos(lat1);

        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= slices; ++j)
        {
            float lng = 2 * M_PI * (float)(j) / slices;
            float x = cos(lng), y = sin(lng);

            float u = (float)(j) / slices;
            float v0 = (float)(i) / stacks;
            float v1 = (float)(i + 1) / stacks;

            glTexCoord2f(u, v0);
            glVertex3f(x * zr0 * radius, y * zr0 * radius, z0 * radius);
            glTexCoord2f(u, v1);
            glVertex3f(x * zr1 * radius, y * zr1 * radius, z1 * radius);
        }
        glEnd();
    }
}

// Render scene
void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    int width = 800;
    int height = 600;
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(90.0, (GLfloat)width / (GLfloat)height, 0.1, 100.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glRotatef(90, 0.0f, 0.0f, 1.0f);
    glRotatef(cameraPitch, 0.0f, 1.0f, 0.0f);
    glRotatef(cameraYaw, 0.0f, 0.0f, 1.0f);

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture);

    drawSphere(100.0f, 40, 40);
    glutSwapBuffers();
}

// Handle arrow keys for rotation
void keyboard(int key, int, int)
{
    if (key == GLUT_KEY_LEFT)
        cameraYaw -= 2.0f;
    if (key == GLUT_KEY_RIGHT)
        cameraYaw += 2.0f;
    if (key == GLUT_KEY_UP)
        cameraPitch -= 2.0f;
    if (key == GLUT_KEY_DOWN)
        cameraPitch += 2.0f;

    glutPostRedisplay();
}

// Initialize OpenGL
void init(const char *img)
{
    glEnable(GL_DEPTH_TEST);
    texture = loadTexture(img);
}

// Main function
int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <image_file>\n";
        return 1;
    }

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Equirectangular Viewer");

    init(argv[1]);

    glutDisplayFunc(display);
    glutSpecialFunc(keyboard);
    glutMainLoop();

    return 0;
}

#endif

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
ImVec4 pc_neigbouring_color = ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
ImVec4 pc_color2 = ImVec4(0.0f, 0.0f, 1.0f, 1.0f);

int point_size = 1;
Eigen::Vector3f rotation_center = Eigen::Vector3f::Zero();
float translate_x, translate_y = 0.0;
float translate_z = -20.0;
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

const std::vector<std::string>
    Session_filter = {"Session, json", "*.json"};
Session session;

namespace fs = std::filesystem;

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
    if (ImGui::Begin("single_session_manual_coloring"))
    {
        ImGui::ColorEdit3("clear color", (float *)&clear_color);

        if (ImGui::Button("load session"))
        {
            std::string input_file_name = "";
            input_file_name = mandeye::fd::OpenFileDialogOneFile("Load session file", Session_filter);
            std::cout << "Session file: '" << input_file_name << "'" << std::endl;

            if (input_file_name.size() > 0)
            {
                session.load(fs::path(input_file_name).string(), false, 0.0, 0.0, 0.0, false);
            }
        }
/*
        ImGui::InputInt("point_size", &point_size);
        if (point_size < 1)
        {
            point_size = 1;
        }

        

        if (session.point_clouds_container.point_clouds.size() > 0)
        {
            ImGui::InputFloat("offset_intensity", &offset_intensity, 0.01, 0.1);
            if (offset_intensity < 0)
            {
                offset_intensity = 0;
            }
            if (offset_intensity > 1)
            {
                offset_intensity = 1;
            }

            ImGui::Checkbox("show_neighbouring_scans", &show_neighbouring_scans);

            if (show_neighbouring_scans)
            {
                ImGui::ColorEdit3("pc_neigbouring_color", (float *)&pc_neigbouring_color);
            }

            ImGui::Text("----------- navigate with index_rendered_points_local ---------");

            ImGui::InputInt("index_rendered_points_local", &index_rendered_points_local, 1, 10);
            if (index_rendered_points_local < 0)
            {
                index_rendered_points_local = 0;
            }
            if (index_rendered_points_local >= session.point_clouds_container.point_clouds.size() - 1)
            {
                index_rendered_points_local = session.point_clouds_container.point_clouds.size() - 1;
            }

            ImGui::Text(session.point_clouds_container.point_clouds[index_rendered_points_local].file_name.c_str());

            double ts = session.point_clouds_container.point_clouds[index_rendered_points_local].timestamps[0] / 1e9;
            ImGui::Text((std::string("ts: ") + std::to_string(ts)).c_str());
        }
*/
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

    /*if (ImGui::GetIO().KeyCtrl)
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
    }*/

    #if 0
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

        if (show_neighbouring_scans)
        {
            // pc_neigbouring_color

            glColor3f(pc_neigbouring_color.x, pc_neigbouring_color.y, pc_neigbouring_color.z);

            glBegin(GL_POINTS);
            for (int index = index_rendered_points_local - 20; index <= index_rendered_points_local + 20; index += 5)
            {
                if (index != index_rendered_points_local)
                {
                    if (index >= 0 && index < session.point_clouds_container.point_clouds.size())
                    {
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
            }
            glEnd();
        }
    }
    #endif

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();

    project_gui();

    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    glutSwapBuffers();
    glutPostRedisplay();
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

bool initGL(int *argc, char **argv)
{
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowSize(window_width, window_height);
    glutCreateWindow("single_session_manual_coloring " HDMAPPING_VERSION_STRING);
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

int main(int argc, char *argv[]){
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