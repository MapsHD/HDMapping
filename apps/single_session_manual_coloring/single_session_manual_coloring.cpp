
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

#include <Eigen/Eigen>
#include <observation_equations/codes/common/include/cauchy.h>
#include <observation_equations/codes/python-scripts/camera-metrics/equirectangular_camera_colinearity_tait_bryan_wc_jacobian.h>
#include <observation_equations/codes/python-scripts/camera-metrics/fisheye_camera_calibRT_tait_bryan_wc_jacobian.h>
#include <structures.h>
#include <transformations.h>

#include <HDMapping/Version.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <session.h>

#include <observation_picking.h>

#include <execution>
#include <filesystem>
#include <iostream>

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

Eigen::Vector3f rotation_center = Eigen::Vector3f::Zero();
float translate_x, translate_y = 0.0;
float translate_z = -20.0;
float rotate_x = 0.0, rotate_y = 0.0;
int mouse_old_x, mouse_old_y;
int mouse_buttons = 0;
bool gui_mouse_down{ false };
float mouse_sensitivity = 1.0;

float m_ortho_projection[] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

float m_ortho_gizmo_view[] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

int decimation_step = 1000;
Session session;

std::vector<int> corresponding_images;
std::vector<int> offsets;
std::vector<std::string> images_file_names;

namespace fs = std::filesystem;

namespace SystemData
{
    std::vector<mandeye::PointRGB> points;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> clickedRay;
    int closestPointIndex{ -1 };
    std::vector<ImVec2> pointPickedImage;
    std::vector<Eigen::Vector3d> pointPickedPointCloud;

    unsigned char* imageData;
    int imageWidth, imageHeight, imageNrChannels;

    Eigen::Affine3d camera_pose = Eigen::Affine3d::Identity();

    int point_size = 1;
} // namespace SystemData

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

        if (is_ortho)
        {
            if (mouse_buttons & 1)
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

std::vector<mandeye::PointRGB> ApplyColorToPointcloud(
    const std::vector<mandeye::PointRGB>& pointsRGB,
    const unsigned char* imageData,
    int imageWidth,
    int imageHeight,
    int nrChannels,
    const Eigen::Affine3d& transfom)
{
    std::vector<mandeye::PointRGB> newCloud(pointsRGB.size());
    std::transform(
        std::execution::par_unseq,
        pointsRGB.begin(),
        pointsRGB.end(),
        newCloud.begin(),
        [&](mandeye::PointRGB p)
        {
            TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(transfom);
            double du, dv;
            equrectangular_camera_colinearity_tait_bryan_wc(
                du,
                dv,
                imageHeight,
                imageWidth,
                M_PI,
                pose.px,
                pose.py,
                pose.pz,
                pose.om,
                pose.fi,
                pose.ka,
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
                p.rgb = { 1.f * red / 256.f, 1.f * green / 256.f, 1.f * blue / 256.f, 1.f };
            }
            else
            {
                p.rgb = { 0.f, 0.f, 0.f, 1.f };
            }
            return p;
        });
    return newCloud;
}

void project_gui()
{
    if (ImGui::Begin("single_session_manual_coloring"))
    {
        ImGui::ColorEdit3("clear color", (float*)&clear_color);
        ImGui::Checkbox("show_axes", &show_axes);
        ImGui::InputInt("point cloud decimation_step", &decimation_step);
        if (decimation_step < 1)
        {
            decimation_step = 1;
        }

        ImGui::InputInt("point_size", &SystemData::point_size);

        if (SystemData::point_size < 1)
        {
            SystemData::point_size = 1;
        }

        if (ImGui::Button("load session"))
        {
            corresponding_images.clear();
            offsets.clear();

            std::string input_file_name = "";
            input_file_name = mandeye::fd::OpenFileDialogOneFile("Load session file", mandeye::fd::Session_filter);
            std::cout << "Session file: '" << input_file_name << "'" << std::endl;

            if (input_file_name.size() > 0)
            {
                session.load(fs::path(input_file_name).string(), false, 0.0, 0.0, 0.0, false);
            }

            for (int i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
            {
                corresponding_images.push_back(0);
                offsets.push_back(0);
            }
        }

        ImGui::SameLine();
        if (ImGui::Button("save colored pointcloud"))
        {
            const auto input_file_names = mandeye::fd::SaveFileDialog("Colored point cloud", mandeye::fd::LazFilter);
            if (!input_file_names.empty())
            {
                std::vector<mandeye::PointRGB> pointsRGB;

                for (const auto& p : session.point_clouds_container.point_clouds)
                {
                    if (p.visible)
                    {
                        for (int index = 0; index < p.points_local.size(); index++)
                        {
                            mandeye::PointRGB out;
                            out.point = p.m_pose * p.points_local[index];
                            out.intensity = p.intensities[index];
                            out.rgb[0] = p.colors[index].x();
                            out.rgb[1] = p.colors[index].y();
                            out.rgb[2] = p.colors[index].z();
                            out.rgb[3] = 1.0;
                            pointsRGB.push_back(out);
                        }
                    }
                }

                mandeye::saveLaz(input_file_names, pointsRGB);
            }
        }

        if (ImGui::Button("load equirectangular images"))
        {
            images_file_names.clear();
            std::vector<std::string> input_file_names;

            input_file_names = mandeye::fd::OpenFileDialog("Load all files", mandeye::fd::ImageFilter, true);

            std::cout << "input_images list begin" << std::endl;
            std::cout << "----------------------" << std::endl;
            for (const auto& fn : input_file_names)
            {
                std::cout << "'" << fn << "'" << std::endl;
                images_file_names.push_back(fn);
            }
            std::cout << "----------------------" << std::endl;
            std::cout << "input_images list end" << std::endl;
        }

        if (ImGui::Button("load camera to lidar extrinsic calibration (*.reg)"))
        {
            std::string input_file_name = "";
            input_file_name = mandeye::fd::OpenFileDialogOneFile("Load RESSO", mandeye::fd::Resso_filter);
            std::cout << "resso file: '" << input_file_name << "'" << std::endl;

            std::ifstream infile(input_file_name);
            if (!infile.good())
            {
                std::cout << "problem with file: '" << input_file_name << "'" << std::endl;
                return;
            }
            std::string line;
            std::getline(infile, line);
            std::istringstream iss(line);

            int num_scans;
            iss >> num_scans;

            std::cout << "number of scans: " << num_scans << std::endl;
            size_t sum_points_before_decimation = 0;
            size_t sum_points_after_decimation = 0;

            for (size_t i = 0; i < num_scans; i++)
            {
                std::getline(infile, line);
                std::istringstream iss(line);
                std::string point_cloud_file_name;
                iss >> point_cloud_file_name;

                double r11, r12, r13, r21, r22, r23, r31, r32, r33;
                double t14, t24, t34;

                std::getline(infile, line);
                std::istringstream iss1(line);
                iss1 >> r11 >> r12 >> r13 >> t14;

                std::getline(infile, line);
                std::istringstream iss2(line);
                iss2 >> r21 >> r22 >> r23 >> t24;

                std::getline(infile, line);
                std::istringstream iss3(line);
                iss3 >> r31 >> r32 >> r33 >> t34;

                std::getline(infile, line);

                SystemData::camera_pose = Eigen::Affine3d::Identity();
                SystemData::camera_pose(0, 0) = r11;
                SystemData::camera_pose(0, 1) = r12;
                SystemData::camera_pose(0, 2) = r13;
                SystemData::camera_pose(1, 0) = r21;
                SystemData::camera_pose(1, 1) = r22;
                SystemData::camera_pose(1, 2) = r23;
                SystemData::camera_pose(2, 0) = r31;
                SystemData::camera_pose(2, 1) = r32;
                SystemData::camera_pose(2, 2) = r33;
                SystemData::camera_pose(0, 3) = t14;
                SystemData::camera_pose(1, 3) = t24;
                SystemData::camera_pose(2, 3) = t34;
            }
            infile.close();
        }

        if (ImGui::Button("select all"))
        {
            for (auto& pc : session.point_clouds_container.point_clouds)
            {
                pc.visible = true;
            }
        }

        ImGui::SameLine();

        if (ImGui::Button("unselect all"))
        {
            for (auto& pc : session.point_clouds_container.point_clouds)
            {
                pc.visible = false;
            }
        }

        for (int i = 0; i < session.point_clouds_container.point_clouds.size();
             i++ /*auto &pc : session.point_clouds_container.point_clouds*/)
        {
            auto& pc = session.point_clouds_container.point_clouds[i];

            ImGui::Text("----------------------------");
            ImGui::Checkbox(pc.file_name.c_str(), &pc.visible);

            if (pc.visible)
            {
                if (session.point_clouds_container.point_clouds.size() == corresponding_images.size() && images_file_names.size() > 0)
                {
                    int prev = corresponding_images[i];

                    ImGui::InputInt((std::string("image[") + std::to_string(i) + std::string("]")).c_str(), &corresponding_images[i]);
                    if (corresponding_images[i] < 0)
                    {
                        corresponding_images[i] = 0;
                    }
                    if (corresponding_images[i] >= images_file_names.size() - 1)
                    {
                        corresponding_images[i] = images_file_names.size() - 1;
                    }

                    if (prev != corresponding_images[i])
                    {
                        namespace SD = SystemData;
                        SD::imageData = stbi_load(
                            images_file_names[corresponding_images[i]].c_str(), &SD::imageWidth, &SD::imageHeight, &SD::imageNrChannels, 0);
                        std::cout << "imageWidth: " << SD::imageWidth << std::endl;
                        std::cout << "imageHeight: " << SD::imageHeight << std::endl;
                        std::cout << "imageNrChannels: " << SD::imageNrChannels << std::endl;

                        ///////////////
                        // std::vector<mandeye::PointRGB> newCloud(session.point_clouds_container.point_clouds[i]..size());

                        session.point_clouds_container.point_clouds[i].colors.resize(
                            session.point_clouds_container.point_clouds[i].points_local.size());
                        for (auto& c : session.point_clouds_container.point_clouds[i].colors)
                        {
                            c.x() = c.y() = c.z() = 0.0;
                        }

                        Eigen::Affine3d transfom = SystemData::camera_pose; // * pc.local_trajectory[offsets[i]].m_pose.inverse();

                        std::vector<mandeye::PointRGB> pointsRGB;

                        for (int p = 0; p < session.point_clouds_container.point_clouds[i].points_local.size(); p++)
                        {
                            mandeye::PointRGB point;
                            point.point = session.point_clouds_container.point_clouds[i].points_local[p];
                            // point.point = pc.local_trajectory[0].m_pose.inverse() * point.point;
                            // point.point = pc.local_trajectory[offsets[i]].m_pose * point.point;
                            point.rgb = { 0.f, 0.f, 0.f, 1.f };
                            pointsRGB.push_back(point);
                        }

                        std::vector<mandeye::PointRGB> pc = ApplyColorToPointcloud(
                            pointsRGB, SD::imageData, SD::imageWidth, SD::imageHeight, SD::imageNrChannels, transfom);

                        for (int color_idx = 0; color_idx < pc.size(); color_idx++)
                        {
                            session.point_clouds_container.point_clouds[i].colors[color_idx].x() = pc[color_idx].rgb.x();
                            session.point_clouds_container.point_clouds[i].colors[color_idx].y() = pc[color_idx].rgb.y();
                            session.point_clouds_container.point_clouds[i].colors[color_idx].z() = pc[color_idx].rgb.z();
                        }

                        session.point_clouds_container.point_clouds[i].show_color = true;
                    }

                    // ImGui::InputInt((std::string("trajectory offset[") + std::to_string(i) + std::string("]")).c_str(), &offsets[i]);
                    // if (offsets[i] < 0)
                    //{
                    //    offsets[i] = 0;
                    //}
                    // if (offsets[i] >= pc.local_trajectory.size() - 1)
                    //{
                    //    offsets[i] = pc.local_trajectory.size() - 1;
                    //}

                    /*
                    if (ImGui::Button(("colorize with '" + images_file_names[corresponding_images[i]] + "'").c_str()))
                    {
                        //
                        // std::vector<Eigen::Vector3d> points_local;
                        // std::vector<Eigen::Vector3d> normal_vectors_local;
                        // std::vector<Eigen::Vector3d> colors;
                        //

                        namespace SD = SystemData;
                        SD::imageData = stbi_load(images_file_names[corresponding_images[i]].c_str(), &SD::imageWidth, &SD::imageHeight,
                    &SD::imageNrChannels, 0); std::cout << "imageWidth: " << SD::imageWidth << std::endl; std::cout << "imageHeight: " <<
                    SD::imageHeight << std::endl; std::cout << "imageNrChannels: " << SD::imageNrChannels << std::endl;

                        ///////////////
                        // std::vector<mandeye::PointRGB> newCloud(session.point_clouds_container.point_clouds[i]..size());

                        session.point_clouds_container.point_clouds[i].colors.resize(session.point_clouds_container.point_clouds[i].points_local.size());
                        for (auto &c : session.point_clouds_container.point_clouds[i].colors)
                        {
                            c.x() = c.y() = c.z() = 0.0;
                        }

                        Eigen::Affine3d transfom = SystemData::camera_pose;// * pc.local_trajectory[offsets[i]].m_pose.inverse();

                        std::vector<mandeye::PointRGB> pointsRGB;

                        for (int p = 0; p < session.point_clouds_container.point_clouds[i].points_local.size(); p++)
                        {
                            mandeye::PointRGB point;
                            point.point = session.point_clouds_container.point_clouds[i].points_local[p];
                            //point.point = pc.local_trajectory[0].m_pose.inverse() * point.point;
                            //point.point = pc.local_trajectory[offsets[i]].m_pose * point.point;
                            point.rgb = {0.f, 0.f, 0.f, 1.f};
                            pointsRGB.push_back(point);
                        }

                        std::vector<mandeye::PointRGB>
                            pc = ApplyColorToPointcloud(pointsRGB, SD::imageData, SD::imageWidth, SD::imageHeight, SD::imageNrChannels,
                    transfom);

                        for (int color_idx = 0; color_idx < pc.size(); color_idx++)
                        {
                            session.point_clouds_container.point_clouds[i].colors[color_idx].x() = pc[color_idx].rgb.x();
                            session.point_clouds_container.point_clouds[i].colors[color_idx].y() = pc[color_idx].rgb.y();
                            session.point_clouds_container.point_clouds[i].colors[color_idx].z() = pc[color_idx].rgb.z();
                        }

                        session.point_clouds_container.point_clouds[i].show_color = true;
                        // return newCloud;
                        ///////////////
                    }*/
                }
            }

            // ImGui::Checkbox(session.point_clouds_container.point_clouds[i].file_name.c_str(),
            // &session.point_clouds_container.point_clouds[i].visible);
#if 0 
        for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
        {
            ImGui::Separator();
            ImGui::Checkbox(session.point_clouds_container.point_clouds[i].file_name.c_str(), &session.point_clouds_container.point_clouds[i].visible);
            // ImGui::SameLine();
            ImGui::Text("--");
            ImGui::SameLine();
            ImGui::Checkbox((std::string("gizmo_") + std::to_string(i)).c_str(), &session.point_clouds_container.point_clouds[i].gizmo);
            ImGui::SameLine();
            ImGui::Checkbox((std::string("fixed_") + std::to_string(i)).c_str(), &session.point_clouds_container.point_clouds[i].fixed);
            ImGui::SameLine();
            ImGui::PushButtonRepeat(true);
            float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
            if (ImGui::ArrowButton(("[" + std::to_string(i) + "] ##left").c_str(), ImGuiDir_Left))
            {
                (session.point_clouds_container.point_clouds[i].point_size)--;
            }
            ImGui::SameLine(0.0f, spacing);
            if (ImGui::ArrowButton(("[" + std::to_string(i) + "] ##right").c_str(), ImGuiDir_Right))
            {
                (session.point_clouds_container.point_clouds[i].point_size)++;
            }
            ImGui::PopButtonRepeat();
            ImGui::SameLine();
            ImGui::Text("point size %d", session.point_clouds_container.point_clouds[i].point_size);
            if (session.point_clouds_container.point_clouds[i].point_size < 1)
            {
                session.point_clouds_container.point_clouds[i].point_size = 1;
            }

            ImGui::SameLine();
            if (ImGui::Button(std::string("#" + std::to_string(i) + " save scan(global reference frame)").c_str()))
            {
                const auto output_file_name = mandeye::fd::SaveFileDialog("Choose folder", {});
                std::cout << "Scan file to save: '" << output_file_name << "'" << std::endl;
                if (output_file_name.size() > 0)
                {
                    session.point_clouds_container.point_clouds[i].save_as_global(output_file_name);
                }
            }
            ImGui::SameLine();
            if (ImGui::Button(std::string("#" + std::to_string(i) + " shift points to center").c_str()))
            {
                session.point_clouds_container.point_clouds[i].shift_to_center();
            }
            if (session.point_clouds_container.point_clouds[i].gizmo)
            {
                for (size_t j = 0; j < session.point_clouds_container.point_clouds.size(); j++)
                {
                    if (i != j)
                    {
                        session.point_clouds_container.point_clouds[j].gizmo = false;
                    }
                }
                m_gizmo[0] = (float)session.point_clouds_container.point_clouds[i].m_pose(0, 0);
                m_gizmo[1] = (float)session.point_clouds_container.point_clouds[i].m_pose(1, 0);
                m_gizmo[2] = (float)session.point_clouds_container.point_clouds[i].m_pose(2, 0);
                m_gizmo[3] = (float)session.point_clouds_container.point_clouds[i].m_pose(3, 0);
                m_gizmo[4] = (float)session.point_clouds_container.point_clouds[i].m_pose(0, 1);
                m_gizmo[5] = (float)session.point_clouds_container.point_clouds[i].m_pose(1, 1);
                m_gizmo[6] = (float)session.point_clouds_container.point_clouds[i].m_pose(2, 1);
                m_gizmo[7] = (float)session.point_clouds_container.point_clouds[i].m_pose(3, 1);
                m_gizmo[8] = (float)session.point_clouds_container.point_clouds[i].m_pose(0, 2);
                m_gizmo[9] = (float)session.point_clouds_container.point_clouds[i].m_pose(1, 2);
                m_gizmo[10] = (float)session.point_clouds_container.point_clouds[i].m_pose(2, 2);
                m_gizmo[11] = (float)session.point_clouds_container.point_clouds[i].m_pose(3, 2);
                m_gizmo[12] = (float)session.point_clouds_container.point_clouds[i].m_pose(0, 3);
                m_gizmo[13] = (float)session.point_clouds_container.point_clouds[i].m_pose(1, 3);
                m_gizmo[14] = (float)session.point_clouds_container.point_clouds[i].m_pose(2, 3);
                m_gizmo[15] = (float)session.point_clouds_container.point_clouds[i].m_pose(3, 3);
            }

            if (session.point_clouds_container.point_clouds[i].visible)
            {
                ImGui::Text("--");
                ImGui::SameLine();
                ImGui::Checkbox(std::string(std::to_string(i) + ": show_color").c_str(), &session.point_clouds_container.point_clouds[i].show_color); //

                if (!session.point_clouds_container.point_clouds[i].show_color)
                {
                    ImGui::SameLine();
                    ImGui::ColorEdit3(std::string(std::to_string(i) + ": pc_color").c_str(), session.point_clouds_container.point_clouds[i].render_color);
                }

                ImGui::SameLine();
                if (ImGui::Button(std::string("#" + std::to_string(i) + "_ICP").c_str()))
                {
                    size_t index_target = i;
                    PointClouds pcs;
                    for (size_t k = 0; k < index_target; k++)
                    {
                        if (session.point_clouds_container.point_clouds[k].visible)
                        {
                            pcs.point_clouds.push_back(session.point_clouds_container.point_clouds[k]);
                        }
                    }

                    if (pcs.point_clouds.size() > 0)
                    {
                        for (size_t k = 0; k < pcs.point_clouds.size(); k++)
                        {
                            pcs.point_clouds[k].fixed = true;
                        }
                    }
                    pcs.point_clouds.push_back(session.point_clouds_container.point_clouds[index_target]);
                    pcs.point_clouds[pcs.point_clouds.size() - 1].fixed = false;

                    ICP icp;
                    icp.search_radious = 0.3; // ToDo move to params
                    for (auto &pc : pcs.point_clouds)
                    {
                        pc.rgd_params.resolution_X = icp.search_radious;
                        pc.rgd_params.resolution_Y = icp.search_radious;
                        pc.rgd_params.resolution_Z = icp.search_radious;

                        pc.build_rgd();
                        pc.cout_rgd();
                        pc.compute_normal_vectors(0.5);
                    }

                    icp.number_of_threads = std::thread::hardware_concurrency();

                    icp.number_of_iterations = 10;
                    icp.is_adaptive_robust_kernel = false;

                    icp.is_ballanced_horizontal_vs_vertical = false;
                    icp.is_fix_first_node = false;
                    icp.is_gauss_newton = true;
                    icp.is_levenberg_marguardt = false;
                    icp.is_cw = false;
                    icp.is_wc = true;
                    icp.is_tait_bryan_angles = true;
                    icp.is_quaternion = false;
                    icp.is_rodrigues = false;
                    std::cout << "optimization_point_to_point_source_to_target" << std::endl;

                    icp.optimization_point_to_point_source_to_target(pcs);

                    std::cout << "pose before: " << session.point_clouds_container.point_clouds[index_target].m_pose.matrix() << std::endl;

                    std::vector<Eigen::Affine3d> all_m_poses;
                    for (int j = 0; j < session.point_clouds_container.point_clouds.size(); j++)
                    {
                        all_m_poses.push_back(session.point_clouds_container.point_clouds[j].m_pose);
                    }

                    session.point_clouds_container.point_clouds[index_target].m_pose = pcs.point_clouds[pcs.point_clouds.size() - 1].m_pose;

                    std::cout << "pose after ICP: " << session.point_clouds_container.point_clouds[index_target].m_pose.matrix() << std::endl;

                    // like gizmo
                    if (!manipulate_only_marked_gizmo)
                    {
                        std::cout << "update all poses after current pose" << std::endl;

                        Eigen::Affine3d curr_m_pose = session.point_clouds_container.point_clouds[index_target].m_pose;
                        for (int j = index_target + 1; j < session.point_clouds_container.point_clouds.size(); j++)
                        {
                            curr_m_pose = curr_m_pose * (all_m_poses[j - 1].inverse() * all_m_poses[j]);
                            session.point_clouds_container.point_clouds[j].m_pose = curr_m_pose;
                        }
                    }
                }
            }
            ImGui::SameLine();
            if (ImGui::Button(std::string("#" + std::to_string(i) + " print frame to console").c_str()))
            {
                std::cout << session.point_clouds_container.point_clouds[i].m_pose.matrix() << std::endl;
            }

            ImGui::SameLine();
            ImGui::Checkbox(std::string("#" + std::to_string(i) + " fuse inclination from IMU").c_str(), &session.point_clouds_container.point_clouds[i].fuse_inclination_from_IMU);

        }
#endif
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
    ImGuiIO& io = ImGui::GetIO();
    glViewport(0, 0, (GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);

    if (is_ortho)
    {
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
        pose_tb.ka = -camera_ortho_xy_view_rotation_angle_deg * M_PI / 180.0;
        auto m = affine_matrix_from_pose_tait_bryan(pose_tb);

        Eigen::Vector3d v_t = m * v;

        gluLookAt(v_eye_t.x(), v_eye_t.y(), v_eye_t.z(), v_center_t.x(), v_center_t.y(), v_center_t.z(), v_t.x(), v_t.y(), v_t.z());
        glm::mat4 lookat = glm::lookAt(
            glm::vec3(v_eye_t.x(), v_eye_t.y(), v_eye_t.z()),
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

    if (show_axes)
    {
        glBegin(GL_LINES);
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(10, 0.0f, 0.0f);

        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 10, 0.0f);

        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 10);
        glEnd();
    }

    ObservationPicking observation_picking;
    observation_picking.point_size = SystemData::point_size;

    for (int k = 0; k < session.point_clouds_container.point_clouds.size(); k++)
    {
        session.point_clouds_container.point_clouds[k].point_size = SystemData::point_size;
    }

    session.point_clouds_container.render(
        observation_picking, decimation_step, false, false, false, false, false, false, false, false, false, false, false, false, 10000);
    // session.ground_control_points.render(session.point_clouds_container);

    /*session.point_clouds_container.render({}, {}, session.point_clouds_container.xz_intersection,
       session.point_clouds_container.yz_intersection, session.point_clouds_container.xy_intersection,
                                          session.point_clouds_container.xz_grid_10x10, session.point_clouds_container.xz_grid_1x1,
       session.point_clouds_container.xz_grid_01x01, session.point_clouds_container.yz_grid_10x10,
       session.point_clouds_container.yz_grid_1x1, session.point_clouds_container.yz_grid_01x01,
                                          session.point_clouds_container.xy_grid_10x10, session.point_clouds_container.xy_grid_1x1,
       session.point_clouds_container.xy_grid_01x01, session.point_clouds_container.intersection_width);*/

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

    */

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
    ImGui::NewFrame();

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

    static int glutMajorVersion = glutGet(GLUT_VERSION) / 10000;
    if (state == GLUT_DOWN && (glut_button == 3 || glut_button == 4) && glutMajorVersion < 3)
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

bool initGL(int* argc, char** argv)
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
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls

    ImGui::StyleColorsDark();
    ImGui_ImplGLUT_Init();
    ImGui_ImplGLUT_InstallFuncs();
    ImGui_ImplOpenGL2_Init();
    return true;
}

int main(int argc, char* argv[])
{
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