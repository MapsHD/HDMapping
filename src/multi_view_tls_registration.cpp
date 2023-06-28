#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>

#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <ImGuizmo.h>
#include <imgui_internal.h>

#include <glew.h>
#include <GL/freeglut.h>

#include <Eigen/Eigen>

#include <glm.hpp>
#include <gtc/matrix_transform.hpp>

#include <point_clouds.h>
#include <ndt.h>
#include <icp.h>
#include <registration_plane_feature.h>
#include <transformations.h>
#include <pose_graph_slam.h>
#include <pcl_wrapper.h>
#include <observation_picking.h>

#include <portable-file-dialogs.h>

#include <laszip/laszip_api.h>

#include <iostream>
#include <filesystem>
#include <fstream>

#include <manual_pose_graph_loop_closure.h>
namespace fs = std::filesystem;

static bool show_demo_window = true;
static bool show_another_window = false;
static ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
float rotate_x = 0.0, rotate_y = 0.0;
float translate_x, translate_y = 0.0;
float translate_z = -50.0;
const unsigned int window_width = 800;
const unsigned int window_height = 600;
int mouse_old_x, mouse_old_y;
int mouse_buttons = 0;
bool gui_mouse_down{false};
float mouse_sensitivity = 1.0;
bool show_axes = false;
bool is_ortho = false;
bool manual_pose_graph_loop_closure_mode = false;

bool is_ndt_gui = false;
bool is_icp_gui = false;
bool is_pose_graph_slam = false;
bool is_registration_plane_feature = false;
bool is_manual_analisys = false;
bool is_decimate = true;
double bucket_x = 0.1;
double bucket_y = 0.1;
double bucket_z = 0.1;
int viewer_decmiate_point_cloud = 1000;

double camera_ortho_xy_view_zoom = 10;
double camera_ortho_xy_view_shift_x = 0.0;
double camera_ortho_xy_view_shift_y = 0.0;
double camera_ortho_xy_view_rotation_angle_deg = 0;
double camera_mode_ortho_z_center_h = 0.0;

std::vector<GeoPoint> available_geo_points;

PointClouds point_clouds_container;
NDT ndt;
ICP icp;
PoseGraphSLAM pose_graph_slam;
RegistrationPlaneFeature registration_plane_feature;
ObservationPicking observation_picking;
std::vector<Eigen::Vector3d> picked_points;
std::string working_directory = "";
ManualPoseGraphLoopClosure manual_pose_graph_loop_closure;
int all_point_size = 1;
int index_loop_closure_source = 0;
int index_loop_closure_target = 0;

float m_gizmo[] = {1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1, 0,
                   0, 0, 0, 1};

float m_ortho_gizmo_view[] = {1, 0, 0, 0,
                              0, 1, 0, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1};

float m_ortho_projection[] = {1, 0, 0, 0,
                              0, 1, 0, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1};

bool manipulate_only_marked_gizmo = true;

void reshape(int w, int h);
void perform_experiment_on_windows();
void perform_experiment_on_linux();
Eigen::Vector3d GLWidgetGetOGLPos(int x, int y, const ObservationPicking &observation_picking);
bool exportLaz(const std::string &filename, const std::vector<Eigen::Vector3d> &pointcloud, const std::vector<unsigned short> &intensity);
double compute_rms(bool initial);

void adjustHeader(laszip_header *header, const Eigen::Affine3d &m_pose, const Eigen::Vector3d &offset_in)
{
    Eigen::Vector3d max{header->max_x, header->max_y, header->max_z};
    Eigen::Vector3d min{header->min_x, header->min_y, header->min_z};
    // Eigen::Vector3d offset{ header->x_offset, header->y_offset, header->z_offset };

    max -= offset_in;
    min -= offset_in;
    // offset -= offset_in;

    Eigen::Vector3d adj_max = m_pose * max + offset_in;
    Eigen::Vector3d adj_min = m_pose * min + offset_in;
    // Eigen::Vector3d adj_off = m_pose * offset + offset_in;

    header->max_x = adj_max.x();
    header->max_y = adj_max.y();
    header->max_z = adj_max.z();

    header->min_x = adj_min.x();
    header->min_y = adj_min.y();
    header->min_z = adj_min.z();

    // header->x_offset = adj_off.x();
    // header->y_offset = adj_off.y();
    // header->z_offset = adj_off.z();
}

void adjustPoint(laszip_F64 output_coordinates[3], laszip_F64 input_coordinates[3], const Eigen::Affine3d &m_pose, const Eigen::Vector3d &offset)
{
    Eigen::Vector3d i{input_coordinates[0], input_coordinates[1], input_coordinates[2]};
    i -= offset;
    Eigen::Vector3d o = m_pose * i;
    o += offset;
    output_coordinates[0] = o.x();
    output_coordinates[1] = o.y();
    output_coordinates[2] = o.z();
}

void save_processed_pc(const fs::path &file_path_in, const fs::path &file_path_put, const Eigen::Affine3d &m_pose, const Eigen::Vector3d &offset)
{
    std::cout << "processing: " << file_path_in << std::endl;

    laszip_POINTER laszip_reader;
    if (laszip_create(&laszip_reader))
    {
        fprintf(stderr, "DLL ERROR: creating laszip reader\n");
        std::abort();
    }

    laszip_POINTER laszip_writer;
    if (laszip_create(&laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: creating laszip reader\n");
        std::abort();
    }

    const std::string file_name_in = file_path_in.string();
    const std::string file_name_out = file_path_put.string();

    laszip_BOOL is_compressed = 0;
    if (laszip_open_reader(laszip_reader, file_name_in.c_str(), &is_compressed))
    {
        fprintf(stderr, "DLL ERROR: opening laszip reader for '%s'\n", file_name_in.c_str());
        std::abort();
    }
    std::cout << "compressed : " << is_compressed << std::endl;

    laszip_header *header;

    if (laszip_get_header_pointer(laszip_reader, &header))
    {
        fprintf(stderr, "DLL ERROR: getting header pointer from laszip reader\n");
        std::abort();
    }

    adjustHeader(header, m_pose, offset);

    if (laszip_set_header(laszip_writer, header))
    {
        fprintf(stderr, "DLL ERROR: setting header pointer from laszip reader\n");
        std::abort();
    }

    fprintf(stderr, "file '%s' contains %u points\n", file_name_in.c_str(), header->number_of_point_records);

    if (laszip_open_writer(laszip_writer, file_name_out.c_str(), is_compressed))
    {
        fprintf(stderr, "DLL ERROR: opening laszip writer for '%s'\n", file_name_out.c_str());
        return;
    }

    laszip_point *input_point;
    if (laszip_get_point_pointer(laszip_reader, &input_point))
    {
        fprintf(stderr, "DLL ERROR: getting point pointer from laszip reader\n");
        std::abort();
    }

    laszip_point *output_point;
    if (laszip_get_point_pointer(laszip_writer, &output_point))
    {
        fprintf(stderr, "DLL ERROR: getting point pointer from laszip reader\n");
        std::abort();
    }

    for (int i = 0; i < header->number_of_point_records; i++)
    {
        if (laszip_read_point(laszip_reader))
        {
            fprintf(stderr, "DLL ERROR: reading point %u\n", i);
            std::abort();
        }

        laszip_F64 input_coordinates[3];
        if (laszip_get_coordinates(laszip_reader, input_coordinates))
        {
            fprintf(stderr, "DLL ERROR: laszip_set_coordinates %u\n", i);
            std::abort();
        }

        laszip_F64 output_coordinates[3];
        adjustPoint(output_coordinates, input_coordinates, m_pose, offset);

        if (laszip_set_coordinates(laszip_writer, output_coordinates))
        {
            fprintf(stderr, "DLL ERROR: laszip_set_coordinates %u\n", i);
            std::abort();
        }
        output_point->intensity = input_point->intensity;
        output_point->classification = input_point->classification;
        output_point->num_extra_bytes = input_point->num_extra_bytes;
        memcpy(output_point->extra_bytes, input_point->extra_bytes, output_point->num_extra_bytes);

        if (laszip_write_point(laszip_writer))
        {
            fprintf(stderr, "DLL ERROR: writing point %I64d\n", i);
            return;
        }
    }

    // close the reader
    if (laszip_close_reader(laszip_reader))
    {
        fprintf(stderr, "DLL ERROR: closing laszip reader\n");
        return;
    }

    // destroy the reader

    if (laszip_destroy(laszip_reader))
    {
        fprintf(stderr, "DLL ERROR: destroying laszip reader\n");
        return;
    }

    laszip_I64 p_count{0};
    if (laszip_get_point_count(laszip_writer, &p_count))
    {
        fprintf(stderr, "DLL ERROR: getting point count\n");
        return;
    }

    fprintf(stderr, "successfully written %ld points\n", p_count);

    // close the writer

    if (laszip_close_writer(laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: closing laszip writer\n");
        return;
    }

    // destroy the writer

    // ToDo --> solve it
    if (laszip_destroy(laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: destroying laszip writer\n");
        return;
    }

    std::cout << "saving to " << file_path_put << std::endl;
}

void my_display_code()
{
    // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
    if (show_demo_window)
        ImGui::ShowDemoWindow(&show_demo_window);

    // 2. Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
    {
        static float f = 0.0f;
        static int counter = 0;

        ImGui::Begin("Hello, world!"); // Create a window called "Hello, world!" and append into it.

        ImGui::Text("This is some useful text.");          // Display some text (you can use a format strings too)
        ImGui::Checkbox("Demo Window", &show_demo_window); // Edit bools storing our window open/close state
        ImGui::Checkbox("Another Window", &show_another_window);

        ImGui::SliderFloat("float", &f, 0.0f, 1.0f);             // Edit 1 float using a slider from 0.0f to 1.0f
        ImGui::ColorEdit3("clear color", (float *)&clear_color); // Edit 3 floats representing a color

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
        ImGui::Begin("Another Window", &show_another_window); // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
        ImGui::Text("Hello from another window!");
        if (ImGui::Button("Close Me"))
            show_another_window = false;
        ImGui::End();
    }
}

void project_gui()
{
    ImGui::Begin("Project");

    if (ImGui::Button("reset view"))
    {
        rotate_x = 0.0;
        rotate_y = 0.0;
        translate_x = 0.0;
        translate_y = 0.0;
        translate_z = -50.0;
        viewer_decmiate_point_cloud = 1000;

        camera_ortho_xy_view_zoom = 10;
        camera_ortho_xy_view_shift_x = 0.0;
        camera_ortho_xy_view_shift_y = 0.0;
        camera_ortho_xy_view_rotation_angle_deg = 0;
        camera_mode_ortho_z_center_h = 0.0;
    }
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

    ImGui::Text("Offset x: %.10f y: %.10f z: %.10f", point_clouds_container.offset.x(), point_clouds_container.offset.y(), point_clouds_container.offset.z());
    ImGui::SameLine();
    if (ImGui::Button("print offset to console"))
    {
        std::cout << "offset:" << std::endl;
        std::cout << std::setprecision(10) << std::endl;
        std::cout << point_clouds_container.offset << std::endl;
    }

    ImGui::InputInt("viewer_decmiate_point_cloud", &viewer_decmiate_point_cloud);
    if (viewer_decmiate_point_cloud < 1)
    {
        viewer_decmiate_point_cloud = 1;
    }

    auto tmp = all_point_size;
    ImGui::InputInt("all points size", &all_point_size);
    if (all_point_size < 1)
        all_point_size = 1;

    if (tmp != all_point_size)
    {
        for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
        {
            point_clouds_container.point_clouds[i].point_size = all_point_size;
        }
    }

    ImGui::Checkbox("is_ortho", &is_ortho);
    if (is_ortho)
    {
        rotate_x = 0.0;
        rotate_y = 0.0;
    }
    ImGui::SameLine();
    ImGui::Checkbox("show_axes", &show_axes);
    ImGui::SameLine();

    ImGui::SliderFloat("mouse_sensitivity", &mouse_sensitivity, 0.01f, 10.0f);

    ImGui::Checkbox("decimate during load", &is_decimate);
    ImGui::InputDouble("bucket_x", &bucket_x);
    ImGui::InputDouble("bucket_y", &bucket_y);
    ImGui::InputDouble("bucket_z", &bucket_z);

    ImGui::Text("-----------------------------------------------------------------------------");
    // common_data
    // manual_pose_graph_loop_closure_mode

    if (ImGui::Button("load RESSO file (transformation_GroundTruth.reg)"))
    {
        static std::shared_ptr<pfd::open_file> open_file;
        std::string input_file_name = "";
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
        const auto t = [&]()
        {
            auto sel = pfd::open_file("Load RESSO file", "C:\\").result();
            for (int i = 0; i < sel.size(); i++)
            {
                input_file_name = sel[i];
                std::cout << "RESSO file: '" << input_file_name << "'" << std::endl;
            }
        };
        std::thread t1(t);
        t1.join();

        if (input_file_name.size() > 0)
        {

            working_directory = fs::path(input_file_name).parent_path().string();

            if (!point_clouds_container.load(working_directory.c_str(), input_file_name.c_str(), is_decimate, bucket_x, bucket_y, bucket_z))
            {
                std::cout << "check input files" << std::endl;
                return;
            }
            else
            {
                std::cout << "loaded: " << point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
            }
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("save RESSO file"))
    {
        std::shared_ptr<pfd::save_file> save_file;
        std::string output_file_name = "";
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
        const auto t = [&]()
        {
            auto sel = pfd::save_file("Save RESSO file", "C:\\").result();
            output_file_name = sel;
            std::cout << "RESSO file to save: '" << output_file_name << "'" << std::endl;
        };
        std::thread t1(t);
        t1.join();

        if (output_file_name.size() > 0)
        {
            point_clouds_container.save_poses(fs::path(output_file_name).string());
        }
    }

    ImGui::Text("RESSO dataset: https://3d.bk.tudelft.nl/liangliang/publications/2019/plade/resso.html");
    if (ImGui::Button("load ETH file (pairs.txt)"))
    {
        static std::shared_ptr<pfd::open_file> open_file;
        std::string input_file_name = "";
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
        const auto t = [&]()
        {
            auto sel = pfd::open_file("Load ETH file", "C:\\").result();
            for (int i = 0; i < sel.size(); i++)
            {
                input_file_name = sel[i];
                std::cout << "ETH file: '" << input_file_name << "'" << std::endl;
            }
        };
        std::thread t1(t);
        t1.join();

        if (input_file_name.size() > 0)
        {
            working_directory = fs::path(input_file_name).parent_path().string();

            if (!point_clouds_container.load_eth(working_directory.c_str(), input_file_name.c_str(), is_decimate, bucket_x, bucket_y, bucket_z))
            {
                std::cout << "check input files" << std::endl;
                return;
            }
            else
            {
                std::cout << "loaded: " << point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
            }
        }
    }
    ImGui::Text("ETH dataset: https://prs.igp.ethz.ch/research/completed_projects/automatic_registration_of_point_clouds.html");

    static bool calculate_offset = false;

    if (ImGui::Button("load AlignedPointCloud from WHU-TLS (select all *.las files in folder 2-AlignedPointCloud)"))
    {
        point_clouds_container.point_clouds.clear();
        static std::shared_ptr<pfd::open_file> open_file;
        std::vector<std::string> input_file_names;
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
        const auto t = [&]()
        {
            std::vector<std::string> filters;
            auto sel = pfd::open_file("Load las files", "C:\\", filters, true).result();
            for (int i = 0; i < sel.size(); i++)
            {
                input_file_names.push_back(sel[i]);
                // std::cout << "las file: '" << input_file_name << "'" << std::endl;
            }
        };
        std::thread t1(t);
        t1.join();

        if (input_file_names.size() > 0)
        {
            working_directory = fs::path(input_file_names[0]).parent_path().string();

            std::cout << "Las/Laz files:" << std::endl;
            for (size_t i = 0; i < input_file_names.size(); i++)
            {
                std::cout << input_file_names[i] << std::endl;
            }

            if (!point_clouds_container.load_whu_tls(input_file_names, is_decimate, bucket_x, bucket_y, bucket_z, calculate_offset))
            {
                std::cout << "check input files laz/las" << std::endl;
                // return;
            }
            else
            {
                std::cout << "loaded: " << point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
            }
        }
    }
    ImGui::SameLine();
    ImGui::Checkbox("calculate_offset for WHU-TLS", &calculate_offset);
    ImGui::Text("WHU-TLS dataset: http://3s.whu.edu.cn/ybs/en/benchmark.htm");

    if (ImGui::Button("load 3DTK files (select all *.txt files)"))
    {
        point_clouds_container.point_clouds.clear();
        static std::shared_ptr<pfd::open_file> open_file;
        std::vector<std::string> input_file_names;
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
        const auto t = [&]()
        {
            std::vector<std::string> filters;
            auto sel = pfd::open_file("Load txt files", "C:\\", filters, true).result();
            for (int i = 0; i < sel.size(); i++)
            {
                input_file_names.push_back(sel[i]);
                // std::cout << "las file: '" << input_file_name << "'" << std::endl;
            }
        };
        std::thread t1(t);
        t1.join();

        if (input_file_names.size() > 0)
        {
            working_directory = fs::path(input_file_names[0]).parent_path().string();

            std::cout << "txt files:" << std::endl;
            for (size_t i = 0; i < input_file_names.size(); i++)
            {
                std::cout << input_file_names[i] << std::endl;
            }

            if (!point_clouds_container.load_3DTK_tls(input_file_names, is_decimate, bucket_x, bucket_y, bucket_z))
            {
                std::cout << "check input files" << std::endl;
                return;
            }
            else
            {
                std::cout << "loaded: " << point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
            }
        }
    }
    ImGui::Text("3DTK dataset: http://kos.informatik.uni-osnabrueck.de/3Dscans/ 18: the campus of the Jacobs University Bremen");

    if (ImGui::Button("update initial poses from RESSO file"))
    {
        static std::shared_ptr<pfd::open_file> open_file;
        std::string input_file_name = "";
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
        const auto t = [&]()
        {
            auto sel = pfd::open_file("Load RESSO file", "C:\\").result();
            for (int i = 0; i < sel.size(); i++)
            {
                input_file_name = sel[i];
                std::cout << "RESSO file: '" << input_file_name << "'" << std::endl;
            }
        };
        std::thread t1(t);
        t1.join();

        if (input_file_name.size() > 0)
        {

            working_directory = fs::path(input_file_name).parent_path().string();

            if (!point_clouds_container.update_initial_poses_from_RESSO(working_directory.c_str(), input_file_name.c_str()))
            {
                std::cout << "check input files" << std::endl;
                return;
            }
            else
            {
                std::cout << "updated: " << point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
            }
        }
    }

    if (ImGui::Button("update poses from RESSO file"))
    {
        static std::shared_ptr<pfd::open_file> open_file;
        std::string input_file_name = "";
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
        const auto t = [&]()
        {
            auto sel = pfd::open_file("Load RESSO file", "C:\\").result();
            for (int i = 0; i < sel.size(); i++)
            {
                input_file_name = sel[i];
                std::cout << "RESSO file: '" << input_file_name << "'" << std::endl;
            }
        };
        std::thread t1(t);
        t1.join();

        if (input_file_name.size() > 0)
        {

            working_directory = fs::path(input_file_name).parent_path().string();

            if (!point_clouds_container.update_poses_from_RESSO(working_directory.c_str(), input_file_name.c_str()))
            {
                std::cout << "check input files" << std::endl;
                return;
            }
            else
            {
                std::cout << "updated: " << point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
            }
        }
    }

    ImGui::Text("-----------------------------------------------------------------------------");
    ImGui::Checkbox("Normal Distributions transform", &is_ndt_gui);
    ImGui::Checkbox("Iterative Closest Point", &is_icp_gui);
    ImGui::Checkbox("Plane Features", &is_registration_plane_feature);
    ImGui::Checkbox("Pose Graph SLAM", &is_pose_graph_slam);
    ImGui::Checkbox("Manual Analysis", &is_manual_analisys);
    ImGui::Checkbox("Manual Pose Graph Loop Closure Mode", &manual_pose_graph_loop_closure_mode);
    ImGui::ColorEdit3("background color", (float *)&clear_color);

    if (manual_pose_graph_loop_closure_mode)
    {
        manual_pose_graph_loop_closure.Gui(point_clouds_container, index_loop_closure_source, index_loop_closure_target, m_gizmo);

        /*if (manual_pose_graph_loop_closure.gizmo && manual_pose_graph_loop_closure.edges.size()> 0)
        {
            int index_src = manual_pose_graph_loop_closure.edges[manual_pose_graph_loop_closure.index_active_edge].index_from;
            int index_trg = manual_pose_graph_loop_closure.edges[manual_pose_graph_loop_closure.index_active_edge].index_to;

            Eigen::Affine3d m_from = point_clouds_container.point_clouds.at(index_src).m_pose;
            Eigen::Affine3d m_to = m_from * affine_matrix_from_pose_tait_bryan(manual_pose_graph_loop_closure.edges[manual_pose_graph_loop_closure.index_active_edge].relative_pose_tb);

            m_gizmo[0] = (float)m_to(0, 0);
            m_gizmo[1] = (float)m_to(1, 0);
            m_gizmo[2] = (float)m_to(2, 0);
            m_gizmo[3] = (float)m_to(3, 0);
            m_gizmo[4] = (float)m_to(0, 1);
            m_gizmo[5] = (float)m_to(1, 1);
            m_gizmo[6] = (float)m_to(2, 1);
            m_gizmo[7] = (float)m_to(3, 1);
            m_gizmo[8] = (float)m_to(0, 2);
            m_gizmo[9] = (float)m_to(1, 2);
            m_gizmo[10] = (float)m_to(2, 2);
            m_gizmo[11] = (float)m_to(3, 2);
            m_gizmo[12] = (float)m_to(0, 3);
            m_gizmo[13] = (float)m_to(1, 3);
            m_gizmo[14] = (float)m_to(2, 3);
            m_gizmo[15] = (float)m_to(3, 3);
        }*/
    }
    else
    {
        if (ImGui::Button("show all"))
        {
            point_clouds_container.show_all();
        }
        ImGui::SameLine();
        if (ImGui::Button("hide all"))
        {
            point_clouds_container.hide_all();
        }
        ImGui::SameLine();
        if (ImGui::Button("reset poses"))
        {
            for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
            {
                point_clouds_container.point_clouds[i].m_pose = point_clouds_container.point_clouds[i].m_initial_pose;
                point_clouds_container.point_clouds[i].pose = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);
                point_clouds_container.point_clouds[i].gui_translation[0] = (float)point_clouds_container.point_clouds[i].pose.px;
                point_clouds_container.point_clouds[i].gui_translation[1] = (float)point_clouds_container.point_clouds[i].pose.py;
                point_clouds_container.point_clouds[i].gui_translation[2] = (float)point_clouds_container.point_clouds[i].pose.pz;
                point_clouds_container.point_clouds[i].gui_rotation[0] = (float)rad2deg(point_clouds_container.point_clouds[i].pose.om);
                point_clouds_container.point_clouds[i].gui_rotation[1] = (float)rad2deg(point_clouds_container.point_clouds[i].pose.fi);
                point_clouds_container.point_clouds[i].gui_rotation[2] = (float)rad2deg(point_clouds_container.point_clouds[i].pose.ka);
            }
        }

        ImGui::Checkbox("show_with_initial_pose", &point_clouds_container.show_with_initial_pose);
        ImGui::SameLine();
        ImGui::Checkbox("manipulate_only_marked_gizmo (false: move also succesive nodes)", &manipulate_only_marked_gizmo);

        for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
        {
            ImGui::Separator();
            ImGui::Checkbox(point_clouds_container.point_clouds[i].file_name.c_str(), &point_clouds_container.point_clouds[i].visible);
            ImGui::SameLine();
            ImGui::Checkbox((std::string("gizmo_") + std::to_string(i)).c_str(), &point_clouds_container.point_clouds[i].gizmo);
            ImGui::SameLine();
            ImGui::Checkbox((std::string("fixed_") + std::to_string(i)).c_str(), &point_clouds_container.point_clouds[i].fixed);
            ImGui::SameLine();
            ImGui::PushButtonRepeat(true);
            float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
            if (ImGui::ArrowButton(("[" + std::to_string(i) + "] ##left").c_str(), ImGuiDir_Left))
            {
                (point_clouds_container.point_clouds[i].point_size)--;
            }
            ImGui::SameLine(0.0f, spacing);
            if (ImGui::ArrowButton(("[" + std::to_string(i) + "] ##right").c_str(), ImGuiDir_Right))
            {
                (point_clouds_container.point_clouds[i].point_size)++;
            }
            ImGui::PopButtonRepeat();
            ImGui::SameLine();
            ImGui::Text("point size %d", point_clouds_container.point_clouds[i].point_size);
            if (point_clouds_container.point_clouds[i].point_size < 1)
                point_clouds_container.point_clouds[i].point_size = 1;
            ImGui::SameLine();
            if (ImGui::Button(std::string("#" + std::to_string(i) + " save scan(global reference frame)").c_str()))
            {
                static std::shared_ptr<pfd::save_file> save_file;
                std::string output_file_name = "";
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
                const auto t = [&]()
                {
                    auto sel = pfd::save_file("Choose folder", "C:\\").result();
                    output_file_name = sel;
                    std::cout << "Scan file to save: '" << output_file_name << "'" << std::endl;
                };
                std::thread t1(t);
                t1.join();

                if (output_file_name.size() > 0)
                {
                    point_clouds_container.point_clouds[i].save_as_global(output_file_name);
                }
            }
            ImGui::SameLine();
            if (ImGui::Button(std::string("#" + std::to_string(i) + " shift points to center").c_str()))
            {
                point_clouds_container.point_clouds[i].shift_to_center();
            }
            if (point_clouds_container.point_clouds[i].gizmo)
            {
                for (size_t j = 0; j < point_clouds_container.point_clouds.size(); j++)
                {
                    if (i != j)
                    {
                        point_clouds_container.point_clouds[j].gizmo = false;
                    }
                }
                m_gizmo[0] = (float)point_clouds_container.point_clouds[i].m_pose(0, 0);
                m_gizmo[1] = (float)point_clouds_container.point_clouds[i].m_pose(1, 0);
                m_gizmo[2] = (float)point_clouds_container.point_clouds[i].m_pose(2, 0);
                m_gizmo[3] = (float)point_clouds_container.point_clouds[i].m_pose(3, 0);
                m_gizmo[4] = (float)point_clouds_container.point_clouds[i].m_pose(0, 1);
                m_gizmo[5] = (float)point_clouds_container.point_clouds[i].m_pose(1, 1);
                m_gizmo[6] = (float)point_clouds_container.point_clouds[i].m_pose(2, 1);
                m_gizmo[7] = (float)point_clouds_container.point_clouds[i].m_pose(3, 1);
                m_gizmo[8] = (float)point_clouds_container.point_clouds[i].m_pose(0, 2);
                m_gizmo[9] = (float)point_clouds_container.point_clouds[i].m_pose(1, 2);
                m_gizmo[10] = (float)point_clouds_container.point_clouds[i].m_pose(2, 2);
                m_gizmo[11] = (float)point_clouds_container.point_clouds[i].m_pose(3, 2);
                m_gizmo[12] = (float)point_clouds_container.point_clouds[i].m_pose(0, 3);
                m_gizmo[13] = (float)point_clouds_container.point_clouds[i].m_pose(1, 3);
                m_gizmo[14] = (float)point_clouds_container.point_clouds[i].m_pose(2, 3);
                m_gizmo[15] = (float)point_clouds_container.point_clouds[i].m_pose(3, 3);
            }

            if (point_clouds_container.point_clouds[i].visible)
            {
                ImGui::ColorEdit3(std::string(std::to_string(i) + ": pc_color").c_str(), point_clouds_container.point_clouds[i].render_color);
                ImGui::InputFloat3(std::string(std::to_string(i) + ": translation [m]").c_str(), point_clouds_container.point_clouds[i].gui_translation);
                ImGui::InputFloat3(std::string(std::to_string(i) + ": rotation [deg]").c_str(), point_clouds_container.point_clouds[i].gui_rotation);
                point_clouds_container.point_clouds[i].update_from_gui();
            }
            ImGui::SameLine();

            if (ImGui::Button(std::string("#" + std::to_string(i) + " print frame to console").c_str()))
            {
                std::cout << point_clouds_container.point_clouds[i].m_pose.matrix() << std::endl;
            }
            ImGui::SameLine();
            ImGui::Checkbox(std::string("#" + std::to_string(i) + " choose_geo").c_str(), &point_clouds_container.point_clouds[i].choosing_geo);

            if (point_clouds_container.point_clouds[i].choosing_geo)
            {
                for (int gp = 0; gp < point_clouds_container.point_clouds[i].available_geo_points.size(); gp++)
                {
                    ImGui::Checkbox(std::string("#" + std::to_string(i) + " " + std::to_string(gp) + "[" +
                                                point_clouds_container.point_clouds[i].available_geo_points[gp].name + "]")
                                        .c_str(),
                                    &point_clouds_container.point_clouds[i].available_geo_points[gp].choosen);
                }
            }
        }
        ImGui::Separator();
        int total_number_of_points = 0;
        for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
        {
            total_number_of_points += point_clouds_container.point_clouds[i].points_local.size();
        }
        std::string point_size_message = "total number of points: " + std::to_string(total_number_of_points);
        ImGui::Text(point_size_message.c_str());

        ImGui::Separator();
        ImGui::Separator();

        if (ImGui::Button("save all marked scans to laz (as one global scan)"))
        {
            std::shared_ptr<pfd::save_file> save_file;
            std::string output_file_name = "";
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
            const auto t = [&]()
            {
                auto sel = pfd::save_file("Save las or laz file", "C:\\").result();
                output_file_name = sel;
                std::cout << "las or laz file to save: '" << output_file_name << "'" << std::endl;
            };
            std::thread t1(t);
            t1.join();

            if (output_file_name.size() > 0)
            {
                std::vector<Eigen::Vector3d> pointcloud;
                std::vector<unsigned short> intensity;

                // point_clouds_container.render(observation_picking, viewer_decmiate_point_cloud);

                for (auto &p : point_clouds_container.point_clouds)
                {
                    if (p.visible)
                    {
                        for (int i = 0; i < p.points_local.size(); i++)
                        {
                            const auto &pp = p.points_local[i];
                            Eigen::Vector3d vp;
                            vp = p.m_pose * pp + point_clouds_container.offset;

                            pointcloud.push_back(vp);
                            if (i < p.intensities.size())
                            {
                                intensity.push_back(p.intensities[i]);
                            }
                            else
                            {
                                intensity.push_back(0);
                            }
                        }
                    }
                }
                if (!exportLaz(output_file_name, pointcloud, intensity))
                {
                    std::cout << "problem with saving file: " << output_file_name << std::endl;
                }
            }
        }

        if (ImGui::Button("save all marked scans to laz (as separate global scans)"))
        {
            for (auto &p : point_clouds_container.point_clouds)
            {
                if (p.visible)
                {

                    fs::path file_path_in = p.file_name;
                    // std::cout << filePath.stem() << std::endl;
                    // std::cout << filePath.extension() << std::endl;
                    // std::cout << filePath.root_name() << std::endl;
                    // std::cout << filePath.root_directory() << std::endl;
                    // std::cout << filePath.root_path() << std::endl;
                    // std::cout << filePath.relative_path() << std::endl;
                    // std::cout << filePath.parent_path() << std::endl;
                    // std::cout << filePath.filename() << std::endl;
                    fs::path file_path_put = file_path_in.parent_path();
                    file_path_put /= (file_path_in.stem().string() + "_processed" + file_path_in.extension().string());
                    std::cout << "file_in: " << file_path_in << std::endl;
                    std::cout << "file_out: " << file_path_put << std::endl;

                    std::cout << "start save_processed_pc" << std::endl;
                    save_processed_pc(file_path_in, file_path_put, p.m_pose, point_clouds_container.offset);
                    std::cout << "processed_pc finished" << std::endl;
                }
            }
        }

        ImGui::Separator();
        ImGui::Separator();

        if (point_clouds_container.point_clouds.size() > 0)
        {
            if (ImGui::Button("load georefence points"))
            {
                static std::shared_ptr<pfd::open_file> open_file;
                std::string input_file_name = "";
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
                const auto t = [&]()
                {
                    auto sel = pfd::open_file("Load geo-reference file", "C:\\").result();
                    for (int i = 0; i < sel.size(); i++)
                    {
                        input_file_name = sel[i];
                        std::cout << "geo-reference file: '" << input_file_name << "'" << std::endl;
                    }
                };
                std::thread t1(t);
                t1.join();

                if (input_file_name.size() > 0)
                {
                    std::ifstream f;
                    f.open(input_file_name.c_str());
                    if (f.good())
                    {
                        std::cout << "parsing file: " << input_file_name << std::endl;

                        std::string s;
                        getline(f, s);
                        while (!f.eof())
                        {
                            getline(f, s);

                            // underground_mining::Intersection intersection;
                            std::string name;
                            double x;
                            double y;
                            double z;

                            stringstream ss(s);
                            ss >> name;
                            ss >> x;
                            ss >> y;
                            ss >> z;

                            GeoPoint geopoint;
                            geopoint.choosen = false;
                            geopoint.coordinates.x() = x;
                            geopoint.coordinates.y() = y;
                            geopoint.coordinates.z() = z;
                            geopoint.name = name;

                            std::cout << "adding geo point: " << geopoint.name << " " << geopoint.coordinates.x() << " " << geopoint.coordinates.y() << " " << geopoint.coordinates.z() << std::endl;

                            available_geo_points.push_back(geopoint);
                        }
                        f.close();

                        auto geo = available_geo_points;
                        for (auto &g : geo)
                        {
                            g.coordinates -= point_clouds_container.offset;
                        }
                        for (auto &p : point_clouds_container.point_clouds)
                        {
                            p.available_geo_points = geo;
                        }
                    }
                }
            }
        }

        ImGui::Separator();
        ImGui::Separator();
        if (ImGui::Button("perform experiment on WIN"))
        {
            perform_experiment_on_windows();
        }
        if (ImGui::Button("perform experiment on LINUX"))
        {
            perform_experiment_on_linux();
        }
    }
    ImGui::End();
}

void ndt_gui()
{
    ImGui::Begin("Normal Distribution Transforms");

    ImGui::InputFloat3("bucket_size (x[m],y[m],z[m])", ndt.bucket_size);
    if (ndt.bucket_size[0] < 0.1)
        ndt.bucket_size[0] = 0.1f;
    if (ndt.bucket_size[1] < 0.1)
        ndt.bucket_size[1] = 0.1f;
    if (ndt.bucket_size[2] < 0.1)
        ndt.bucket_size[2] = 0.1f;

    ImGui::InputInt("number_of_threads", &ndt.number_of_threads);
    if (ndt.number_of_threads < 1)
        ndt.number_of_threads = 1;

    ImGui::InputInt("number_of_iterations", &ndt.number_of_iterations);
    if (ndt.number_of_iterations < 1)
        ndt.number_of_iterations = 1;

    ImGui::Checkbox("ndt fix_first_node (add I to first pose in Hessian)", &ndt.is_fix_first_node);

    ImGui::Checkbox("ndt Gauss-Newton", &ndt.is_gauss_newton);
    if (ndt.is_gauss_newton)
    {
        ndt.is_levenberg_marguardt = false;
    }

    ImGui::SameLine();
    ImGui::Checkbox("ndt Levenberg-Marguardt", &ndt.is_levenberg_marguardt);
    if (ndt.is_levenberg_marguardt)
    {
        ndt.is_gauss_newton = false;
    }

    ImGui::Checkbox("ndt poses expressed as camera<-world (cw)", &ndt.is_cw);
    if (ndt.is_cw)
    {
        ndt.is_wc = false;
    }
    ImGui::SameLine();
    ImGui::Checkbox("ndt poses expressed as camera->world (wc)", &ndt.is_wc);
    if (ndt.is_wc)
    {
        ndt.is_cw = false;
    }

    ImGui::Checkbox("ndt Tait-Bryan angles (om fi ka: RxRyRz)", &ndt.is_tait_bryan_angles);
    if (ndt.is_tait_bryan_angles)
    {
        ndt.is_quaternion = false;
        ndt.is_rodrigues = false;
    }

    ImGui::SameLine();
    ImGui::Checkbox("ndt Quaternion (q0 q1 q2 q3)", &ndt.is_quaternion);
    if (ndt.is_quaternion)
    {
        ndt.is_tait_bryan_angles = false;
        ndt.is_rodrigues = false;
    }

    ImGui::SameLine();
    ImGui::Checkbox("ndt Rodrigues (sx sy sz)", &ndt.is_rodrigues);
    if (ndt.is_rodrigues)
    {
        ndt.is_tait_bryan_angles = false;
        ndt.is_quaternion = false;
    }

    if (ImGui::Button("ndt_optimization"))
    {
        double rms_initial = 0.0;
        double rms_final = 0.0;
        double mui = 0.0;
        // ndt.optimize(point_clouds_container.point_clouds, rms_initial, rms_final, mui);
        // std::cout << "mui: " << mui << " rms_initial: " << rms_initial << " rms_final: " << rms_final << std::endl;
        ndt.optimize(point_clouds_container.point_clouds, false);
    }

    if (ImGui::Button("compute mean mahalanobis distance"))
    {
        double rms_initial = 0.0;
        double rms_final = 0.0;
        double mui = 0.0;
        // ndt.optimize(point_clouds_container.point_clouds, rms_initial, rms_final, mui);
        // std::cout << "mui: " << mui << " rms_initial: " << rms_initial << " rms_final: " << rms_final << std::endl;
        ndt.optimize(point_clouds_container.point_clouds, true);
    }

    ImGui::Text("--------------------------------------------------------------------------------------------------------");

    if (ImGui::Button("ndt_optimization(Lie-algebra left Jacobian)"))
    {
        // icp.optimize_source_to_target_lie_algebra_left_jacobian(point_clouds_container);
        ndt.optimize_lie_algebra_left_jacobian(point_clouds_container.point_clouds);
    }
    if (ImGui::Button("ndt_optimization(Lie-algebra right Jacobian)"))
    {
        // icp.optimize_source_to_target_lie_algebra_right_jacobian(point_clouds_container);
        ndt.optimize_lie_algebra_right_jacobian(point_clouds_container.point_clouds);
    }

    ImGui::Text("--------------------------------------------------------------------------------------------------------");

    ImGui::Checkbox("generelized", &ndt.is_generalized);

    if (ndt.is_generalized)
    {
        ImGui::InputDouble("sigma_r", &ndt.sigma_r, 0.01, 0.01);
        ImGui::InputDouble("sigma_polar_angle_rad", &ndt.sigma_polar_angle, 0.0001, 0.0001);
        ImGui::InputDouble("sigma_azimuthal_angle_rad", &ndt.sigma_azimuthal_angle, 0.0001, 0.0001);
        ImGui::InputInt("num_extended_points", &ndt.num_extended_points, 1, 1);
    }

    if (ImGui::Button("Set Zoller+Fröhlich TLS Imager 5006i errors"))
    {
        ndt.sigma_r = 0.0068;
        ndt.sigma_polar_angle = 0.007 / 180.0 * M_PI;
        ndt.sigma_azimuthal_angle = 0.007 / 180.0 * M_PI;
    }
    if (ImGui::Button("Set Zoller+Fröhlich TLS Imager 5010C errors"))
    {
        ndt.sigma_r = 0.01;
        ndt.sigma_polar_angle = 0.007 / 180.0 * M_PI;
        ndt.sigma_azimuthal_angle = 0.007 / 180.0 * M_PI;
    }
    if (ImGui::Button("Set Faro Focus3D errors"))
    {
        ndt.sigma_r = 0.001;
        ndt.sigma_polar_angle = 19.0 * (1.0 / 3600.0) / 180.0 * M_PI;
        ndt.sigma_azimuthal_angle = 19.0 * (1.0 / 3600.0) / 180.0 * M_PI;
    }
    if (ImGui::Button("Set Leica ScanStation C5 C10 errors"))
    {
        ndt.sigma_r = 0.006;
        ndt.sigma_polar_angle = 0.00006;
        ndt.sigma_azimuthal_angle = 0.00006;
    }
    if (ImGui::Button("Set Riegl VZ400 errors"))
    {
        ndt.sigma_r = 0.005;
        ndt.sigma_polar_angle = 0.0005 / 180.0 * M_PI + 0.0003;     // Laser Beam Dicvergence
        ndt.sigma_azimuthal_angle = 0.0005 / 180.0 * M_PI + 0.0003; // Laser Beam Dicvergence
    }
    if (ImGui::Button("Set Leica HDS6100 errors"))
    {
        ndt.sigma_r = 0.009;
        ndt.sigma_polar_angle = 0.000125;
        ndt.sigma_azimuthal_angle = 0.000125;
    }
    if (ImGui::Button("Set Leica P40 errors"))
    {
        ndt.sigma_r = 0.0012;
        ndt.sigma_polar_angle = 8.0 / 3600;
        ndt.sigma_azimuthal_angle = 8.0 / 3600;
    }

    ImGui::End();
}

void icp_gui()
{
    ImGui::Begin("Iterative Closest Point");

    ImGui::InputFloat("icp_search_radious", &icp.search_radious, 0.01f, 0.1f);
    if (icp.search_radious < 0.01f)
        icp.search_radious = 0.01f;
    if (icp.search_radious > 2.0f)
        icp.search_radious = 2.0f;

    ImGui::InputInt("icp_number_of_threads", &icp.number_of_threads);
    if (icp.number_of_threads < 1)
        icp.number_of_threads = 1;

    ImGui::InputInt("icp_number_of_iterations", &icp.number_of_iterations);
    if (icp.number_of_iterations < 1)
        icp.number_of_iterations = 1;

    ImGui::Checkbox("icp_adaptive_robust_kernel", &icp.is_adaptive_robust_kernel);
    ImGui::SameLine();
    ImGui::Checkbox("icp_fix_first_node (add I to first pose in Hessian)", &icp.is_fix_first_node);

    ImGui::Checkbox("icp Gauss-Newton", &icp.is_gauss_newton);
    if (icp.is_gauss_newton)
    {
        icp.is_levenberg_marguardt = false;
    }

    ImGui::SameLine();
    ImGui::Checkbox("icp Levenberg-Marguardt", &icp.is_levenberg_marguardt);
    if (icp.is_levenberg_marguardt)
    {
        icp.is_gauss_newton = false;
    }

    ImGui::Text("--------------------------------------------------------------------------------------------------------");
    ImGui::Checkbox("icp poses expressed as camera<-world (cw)", &icp.is_cw);
    if (icp.is_cw)
    {
        icp.is_wc = false;
    }
    ImGui::SameLine();
    ImGui::Checkbox("icp poses expressed as camera->world (wc)", &icp.is_wc);
    if (icp.is_wc)
    {
        icp.is_cw = false;
    }

    ImGui::Checkbox("icp Tait-Bryan angles (om fi ka: RxRyRz)", &icp.is_tait_bryan_angles);
    if (icp.is_tait_bryan_angles)
    {
        icp.is_quaternion = false;
        icp.is_rodrigues = false;
    }

    ImGui::SameLine();
    ImGui::Checkbox("icp Quaternion (q0 q1 q2 q3)", &icp.is_quaternion);
    if (icp.is_quaternion)
    {
        icp.is_tait_bryan_angles = false;
        icp.is_rodrigues = false;
    }

    ImGui::SameLine();
    ImGui::Checkbox("icp Rodrigues (sx sy sz)", &icp.is_rodrigues);
    if (icp.is_rodrigues)
    {
        icp.is_tait_bryan_angles = false;
        icp.is_quaternion = false;
    }
    if (ImGui::Button("optimization_point_to_point_source_to_target"))
    {
        icp.optimization_point_to_point_source_to_target(point_clouds_container);
    }
    ImGui::Text("--------------------------------------------------------------------------------------------------------");

    if (ImGui::Button("icp_optimization_source_to_target(Lie-algebra left Jacobian)"))
    {
        icp.optimize_source_to_target_lie_algebra_left_jacobian(point_clouds_container);
    }
    if (ImGui::Button("icp_optimization_source_to_target(Lie-algebra right Jacobian)"))
    {
        icp.optimize_source_to_target_lie_algebra_right_jacobian(point_clouds_container);
    }
    ImGui::Text("--------------------------------------------------------------------------------------------------------");

    if (ImGui::Button("compute rms(optimization_point_to_point_source_to_target)"))
    {
        double rms = 0.0;
        icp.optimization_point_to_point_source_to_target_compute_rms(point_clouds_container, rms);
        std::cout << "rms(optimization_point_to_point_source_to_target): " << rms << std::endl;
    }

    ImGui::End();
}

void registration_plane_feature_gui()
{
    ImGui::Begin("Registration Plane Feature");

    ImGui::InputFloat("plane_feature_search_radious", &registration_plane_feature.search_radious, 0.01, 2.0);
    if (registration_plane_feature.search_radious < 0.01)
        registration_plane_feature.search_radious = 0.01;
    if (registration_plane_feature.search_radious > 2.0)
        registration_plane_feature.search_radious = 2.0;

    ImGui::InputInt("plane_feature_number_of_threads", &registration_plane_feature.number_of_threads);
    if (registration_plane_feature.number_of_threads < 1)
        registration_plane_feature.number_of_threads = 1;

    ImGui::InputInt("plane_feature_number_of_iterations", &registration_plane_feature.number_of_iterations);
    if (registration_plane_feature.number_of_iterations < 1)
        registration_plane_feature.number_of_iterations = 1;

    ImGui::Checkbox("plane_feature_adaptive_robust_kernel", &registration_plane_feature.is_adaptive_robust_kernel);
    ImGui::SameLine();
    ImGui::Checkbox("fix_first_node (add I to first pose in Hessian)", &registration_plane_feature.is_fix_first_node);

    ImGui::Checkbox("Gauss-Newton", &registration_plane_feature.is_gauss_newton);
    if (registration_plane_feature.is_gauss_newton)
    {
        // registration_plane_feature.is_newton = false;
        registration_plane_feature.is_levenberg_marguardt = false;
    }

    ImGui::SameLine();
    ImGui::Checkbox("Levenberg-Marguardt", &registration_plane_feature.is_levenberg_marguardt);
    if (registration_plane_feature.is_levenberg_marguardt)
    {
        // registration_plane_feature.is_newton = false;
        registration_plane_feature.is_gauss_newton = false;
    }

    ImGui::Checkbox("poses expressed as camera<-world (cw)", &registration_plane_feature.is_cw);
    if (registration_plane_feature.is_cw)
    {
        registration_plane_feature.is_wc = false;
    }
    ImGui::SameLine();
    ImGui::Checkbox("poses expressed as camera->world (wc)", &registration_plane_feature.is_wc);
    if (registration_plane_feature.is_wc)
    {
        registration_plane_feature.is_cw = false;
    }

    ImGui::Checkbox("Tait-Bryan angles (om fi ka: RxRyRz)", &registration_plane_feature.is_tait_bryan_angles);
    if (registration_plane_feature.is_tait_bryan_angles)
    {
        registration_plane_feature.is_quaternion = false;
        registration_plane_feature.is_rodrigues = false;
    }

    ImGui::SameLine();
    ImGui::Checkbox("Quaternion (q0 q1 q2 q3)", &registration_plane_feature.is_quaternion);
    if (registration_plane_feature.is_quaternion)
    {
        registration_plane_feature.is_tait_bryan_angles = false;
        registration_plane_feature.is_rodrigues = false;
    }

    ImGui::SameLine();
    ImGui::Checkbox("Rodrigues (sx sy sz)", &registration_plane_feature.is_rodrigues);
    if (registration_plane_feature.is_rodrigues)
    {
        registration_plane_feature.is_tait_bryan_angles = false;
        registration_plane_feature.is_quaternion = false;
    }
    ImGui::Text("--------------------------------------------------------------------------------------------");
    if (ImGui::Button("optimize_point_to_projection_onto_plane_source_to_target"))
    {
        registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(point_clouds_container);
    }
    // ImGui::Text("--------------------------------------------------------------------------------------------------------");
    if (ImGui::Button("optimize_point_to_projection_onto_plane_source_to_target(Lie-algebra left Jacobian)"))
    {
        registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_left_jacobian(point_clouds_container);
    }
    if (ImGui::Button("optimize_point_to_projection_onto_plane_source_to_target(Lie-algebra right Jacobian)"))
    {
        registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_right_jacobian(point_clouds_container);
    }

    ImGui::Text("--------------------------------------------------------------------------------------------");

    if (ImGui::Button("optimize_point_to_plane_source_to_target (using dot product)"))
    {
        registration_plane_feature.optimize_point_to_plane_source_to_target(point_clouds_container);
    }
    if (ImGui::Button("optimize_distance_point_to_plane_source_to_target"))
    {
        registration_plane_feature.optimize_distance_point_to_plane_source_to_target(point_clouds_container);
    }
    if (ImGui::Button("optimize_plane_to_plane_source_to_target"))
    {
        registration_plane_feature.optimize_plane_to_plane_source_to_target(point_clouds_container);
    }

    ImGui::End();
}

void pose_graph_slam_gui()
{
    ImGui::Begin("Pose Graph SLAM");

    ImGui::InputFloat("pgslam search_radious", &pose_graph_slam.search_radious, 0.01f, 2.0f);
    if (pose_graph_slam.search_radious < 0.01f)
        pose_graph_slam.search_radious = 0.01f;

    ImGui::InputInt("pgslam number_of_threads", &pose_graph_slam.number_of_threads);
    if (pose_graph_slam.number_of_threads < 1)
        pose_graph_slam.number_of_threads = 1;

    ImGui::InputInt("pgslam number_of_iterations_pair_wise_matching", &pose_graph_slam.number_of_iterations_pair_wise_matching);
    if (pose_graph_slam.number_of_iterations_pair_wise_matching < 1)
        pose_graph_slam.number_of_iterations_pair_wise_matching = 1;

    ImGui::InputFloat("pgslam overlap_threshold", &pose_graph_slam.overlap_threshold, 0.1f, 0.8f);
    if (pose_graph_slam.overlap_threshold < 0.1f)
        pose_graph_slam.overlap_threshold = 0.1f;

    // ImGui::Checkbox("pgslam adaptive_robust_kernel", &pose_graph_slam.icp.is_adaptive_robust_kernel);

    //--
    ImGui::Checkbox("pgslam adaptive_robust_kernel", &pose_graph_slam.is_adaptive_robust_kernel);
    ImGui::SameLine();
    ImGui::Checkbox("pgslam fix_first_node (add I to first pose in Hessian)", &pose_graph_slam.is_fix_first_node);

    ImGui::Checkbox("pgslam Gauss-Newton", &pose_graph_slam.is_gauss_newton);
    if (pose_graph_slam.is_gauss_newton)
    {
        pose_graph_slam.is_levenberg_marguardt = false;
    }

    ImGui::SameLine();
    ImGui::Checkbox("pgslam Levenberg-Marguardt", &pose_graph_slam.is_levenberg_marguardt);
    if (pose_graph_slam.is_levenberg_marguardt)
    {
        pose_graph_slam.is_gauss_newton = false;
    }

    ImGui::Checkbox("pgslam poses expressed as camera<-world (cw)", &pose_graph_slam.is_cw);
    if (pose_graph_slam.is_cw)
    {
        pose_graph_slam.is_wc = false;
    }
    ImGui::SameLine();
    ImGui::Checkbox("pgslam poses expressed as camera->world (wc)", &pose_graph_slam.is_wc);
    if (pose_graph_slam.is_wc)
    {
        pose_graph_slam.is_cw = false;
    }

    ImGui::Checkbox("Tait-Bryan angles (om fi ka: RxRyRz)", &pose_graph_slam.is_tait_bryan_angles);
    if (pose_graph_slam.is_tait_bryan_angles)
    {
        pose_graph_slam.is_quaternion = false;
        pose_graph_slam.is_rodrigues = false;
    }

    ImGui::SameLine();
    ImGui::Checkbox("Quaternion (q0 q1 q2 q3)", &pose_graph_slam.is_quaternion);
    if (pose_graph_slam.is_quaternion)
    {
        pose_graph_slam.is_tait_bryan_angles = false;
        pose_graph_slam.is_rodrigues = false;
    }

    ImGui::SameLine();
    ImGui::Checkbox("Rodrigues (sx sy sz)", &pose_graph_slam.is_rodrigues);
    if (pose_graph_slam.is_rodrigues)
    {
        pose_graph_slam.is_tait_bryan_angles = false;
        pose_graph_slam.is_quaternion = false;
    }

    ImGui::Text("--------Method for pair wise matching (general)-------------------------------------------------");
    ImGui::Checkbox("pgslam ndt", &pose_graph_slam.is_ndt);
    if (pose_graph_slam.is_ndt)
    {
        pose_graph_slam.set_all_to_false();
        pose_graph_slam.is_ndt = true;
        pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    }

    ImGui::Checkbox("pgslam optimization_point_to_point_source_to_target", &pose_graph_slam.is_optimization_point_to_point_source_to_target);
    if (pose_graph_slam.is_optimization_point_to_point_source_to_target)
    {
        pose_graph_slam.set_all_to_false();
        pose_graph_slam.is_optimization_point_to_point_source_to_target = true;
        pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    }

    ImGui::Checkbox("pgslam optimize_point_to_projection_onto_plane_source_to_target", &pose_graph_slam.is_optimize_point_to_projection_onto_plane_source_to_target);
    if (pose_graph_slam.is_optimize_point_to_projection_onto_plane_source_to_target)
    {
        pose_graph_slam.set_all_to_false();
        pose_graph_slam.is_optimize_point_to_projection_onto_plane_source_to_target = true;
        pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    }

    ImGui::Checkbox("pgslam optimize_point_to_plane_source_to_target", &pose_graph_slam.is_optimize_point_to_plane_source_to_target);
    if (pose_graph_slam.is_optimize_point_to_plane_source_to_target)
    {
        pose_graph_slam.set_all_to_false();
        pose_graph_slam.is_optimize_point_to_plane_source_to_target = true;
        pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    }

    ImGui::Checkbox("pgslam optimize_distance_point_to_plane_source_to_target", &pose_graph_slam.is_optimize_distance_point_to_plane_source_to_target);
    if (pose_graph_slam.is_optimize_distance_point_to_plane_source_to_target)
    {
        pose_graph_slam.set_all_to_false();
        pose_graph_slam.is_optimize_distance_point_to_plane_source_to_target = true;
        pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    }

    ImGui::Checkbox("pgslam optimize_plane_to_plane_source_to_target", &pose_graph_slam.is_optimize_plane_to_plane_source_to_target);
    if (pose_graph_slam.is_optimize_plane_to_plane_source_to_target)
    {
        pose_graph_slam.set_all_to_false();
        pose_graph_slam.is_optimize_plane_to_plane_source_to_target = true;
        pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    }
    ImGui::Text("--------Method for pair wise matching (with PCL)------------------------------------------------");
#ifdef WITH_PCL
    ImGui::Checkbox("pgslam optimize with pcl (ndt based pair wise matching)", &pose_graph_slam.is_optimize_pcl_ndt);
    if (pose_graph_slam.is_optimize_pcl_ndt)
    {
        pose_graph_slam.set_all_to_false();
        pose_graph_slam.is_optimize_pcl_ndt = true;
        pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::pcl_ndt;
    }
    ImGui::Checkbox("pgslam optimize with pcl (icp based pair wise matching)", &pose_graph_slam.is_optimize_pcl_icp);
    if (pose_graph_slam.is_optimize_pcl_icp)
    {
        pose_graph_slam.set_all_to_false();
        pose_graph_slam.is_optimize_pcl_icp = true;
        pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::pcl_icp;
    }
#endif
    ImGui::Text("------------------------------------------------------------------------------------------------");
    ImGui::Text("--------Method for pair wise matching (with Lie-algebra)----------------------------------------");

    ImGui::Checkbox("pgslam optimize ndt(Lie-algebra left Jacobian)", &pose_graph_slam.is_ndt_lie_algebra_left_jacobian);
    if (pose_graph_slam.is_ndt_lie_algebra_left_jacobian)
    {
        pose_graph_slam.set_all_to_false();
        pose_graph_slam.is_ndt_lie_algebra_left_jacobian = true;
        pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    }

    ImGui::Checkbox("pgslam optimize ndt(Lie-algebra right Jacobian)", &pose_graph_slam.is_ndt_lie_algebra_right_jacobian);
    if (pose_graph_slam.is_ndt_lie_algebra_right_jacobian)
    {
        pose_graph_slam.set_all_to_false();
        pose_graph_slam.is_ndt_lie_algebra_right_jacobian = true;
        pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    }

    ImGui::Checkbox("pgslam optimize point to point source to target(Lie-algebra left Jacobian)", &pose_graph_slam.is_optimize_point_to_point_source_to_target_lie_algebra_left_jacobian);
    if (pose_graph_slam.is_optimize_point_to_point_source_to_target_lie_algebra_left_jacobian)
    {
        pose_graph_slam.set_all_to_false();
        pose_graph_slam.is_optimize_point_to_point_source_to_target_lie_algebra_left_jacobian = true;
        pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    }
    ImGui::Checkbox("pgslam optimize point to point source to target(Lie-algebra right Jacobian)", &pose_graph_slam.is_optimize_point_to_point_source_to_target_lie_algebra_right_jacobian);
    if (pose_graph_slam.is_optimize_point_to_point_source_to_target_lie_algebra_right_jacobian)
    {
        pose_graph_slam.set_all_to_false();
        pose_graph_slam.is_optimize_point_to_point_source_to_target_lie_algebra_right_jacobian = true;
        pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    }
    ImGui::Checkbox("pgslam optimize point to projection onto plane source to target(Lie-algebra left Jacobian)", &pose_graph_slam.is_optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_left_jacobian);
    if (pose_graph_slam.is_optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_left_jacobian)
    {
        pose_graph_slam.set_all_to_false();
        pose_graph_slam.is_optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_left_jacobian = true;
        pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    }
    ImGui::Checkbox("pgslam optimize point to projection onto plane source to target(Lie-algebra right Jacobian)", &pose_graph_slam.is_optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_right_jacobian);
    if (pose_graph_slam.is_optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_right_jacobian)
    {
        pose_graph_slam.set_all_to_false();
        pose_graph_slam.is_optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_right_jacobian = true;
        pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    }
    ImGui::Text("------------------------------------------------------------------------------------------------");

    if (ImGui::Button("optimize"))
    {
        pose_graph_slam.ndt_bucket_size[0] = ndt.bucket_size[0];
        pose_graph_slam.ndt_bucket_size[1] = ndt.bucket_size[1];
        pose_graph_slam.ndt_bucket_size[2] = ndt.bucket_size[2];
        // double rms_initial = 0.0;
        // double rms_final = 0.0;
        // double mui = 0.0;
        pose_graph_slam.optimize(point_clouds_container);
        // pose_graph_slam.optimize(point_clouds_container, rms_initial, rms_final, mui);
        // std::cout << "mean uncertainty impact: " << mui << " rms_initial: " << rms_initial << " rms_final: " << rms_final << std::endl;
    }
    ImGui::Text("----------with GTSAM----------------------------------------------------------------------------");
#if WITH_GTSAM
    if (ImGui::Button("optimize with GTSAM"))
    {
        pose_graph_slam.ndt_bucket_size[0] = ndt.bucket_size[0];
        pose_graph_slam.ndt_bucket_size[1] = ndt.bucket_size[1];
        pose_graph_slam.ndt_bucket_size[2] = ndt.bucket_size[2];
        double rms_initial = 0.0;
        double rms_final = 0.0;
        double mui = 0.0;
        pose_graph_slam.optimize_with_GTSAM(point_clouds_container);
        // std::cout << "mean uncertainty impact: " << mui << " rms_initial: " << rms_initial << " rms_final: " << rms_final << std::endl;
    }
#endif
    ImGui::Text("------------------------------------------------------------------------------------------------");
    ImGui::Text("----------with MANIF----------------------------------------------------------------------------");

#if WITH_MANIF
    if (ImGui::Button("optimize with manif (A small header-only library for Lie theory)"))
    {
        pose_graph_slam.optimize_with_manif(point_clouds_container);
        std::cout << "optimize with manif (A small header-only library for Lie theory) DONE" << std::endl;
    }
#endif
    ImGui::Text("------------------------------------------------------------------------------------------------");
    ImGui::End();
}

void export_result_to_folder(std::string output_folder_name)
{
    fs::path path(output_folder_name);
    std::string file_name_rms = "rms.csv";
    auto path_rms = path;
    path_rms /= file_name_rms;
    std::cout << "exporting to file: '" << path_rms.string() << "'" << std::endl;
    std::ofstream outfile_rms;
    outfile_rms.open(path_rms, std::ios_base::app);
    outfile_rms << "index_roi, rms_initial, rms_result" << std::endl;

    for (int i = 0; i < observation_picking.intersections.size(); i++)
    {
        std::string file_name_initial = "intersection_" + std::to_string(i) + "_initial.csv";
        std::string file_name_result = "intersection_" + std::to_string(i) + "_result.csv";

        auto path_initial = path;
        auto path_result = path;

        path_initial /= file_name_initial;
        path_result /= file_name_result;

        std::cout << "exporting to file: '" << path_initial.string() << "'" << std::endl;
        std::cout << "exporting to file: '" << path_result.string() << "'" << std::endl;

        std::ofstream outfile_initial;
        std::ofstream outfile_result;

        outfile_initial.open(path_initial, std::ios_base::app);
        outfile_result.open(path_result, std::ios_base::app);

        const auto &intersection = observation_picking.intersections[i];
        TaitBryanPose pose;
        pose.px = intersection.translation[0];
        pose.py = intersection.translation[1];
        pose.pz = intersection.translation[2];
        pose.om = intersection.rotation[0];
        pose.fi = intersection.rotation[1];
        pose.ka = intersection.rotation[2];
        Eigen::Affine3d m_pose_inv = affine_matrix_from_pose_tait_bryan(pose).inverse();

        double w = intersection.width_length_height[0] * 0.5;
        double l = intersection.width_length_height[1] * 0.5;
        double h = intersection.width_length_height[2] * 0.5;

        outfile_initial << "x;y;z;pc_index;is_initial;index_intersection;file" << std::endl;
        outfile_result << "x;y;z;pc_index;is_initial;index_intersection;file" << std::endl;

        for (int pc_index = 0; pc_index < point_clouds_container.point_clouds.size(); pc_index++)
        {
            const auto &pc = point_clouds_container.point_clouds[pc_index];
            for (const auto &p : pc.points_local)
            {
                Eigen::Vector3d vpi = pc.m_initial_pose * p;
                Eigen::Vector3d vpr = pc.m_pose * p;

                Eigen::Vector3d vpit = m_pose_inv * vpi;
                Eigen::Vector3d vprt = m_pose_inv * vpr;

                if (fabs(vpit.x()) < w)
                {
                    if (fabs(vpit.y()) < l)
                    {
                        if (fabs(vpit.z()) < h)
                        {
                            outfile_initial << vpit.x() << ";" << vpit.y() << ";" << vpit.z() << ";" << pc_index << ";1;" << i << ";" << pc.file_name << std::endl;
                        }
                    }
                }
                if (fabs(vprt.x()) < w)
                {
                    if (fabs(vprt.y()) < l)
                    {
                        if (fabs(vprt.z()) < h)
                        {
                            outfile_result << vprt.x() << ";" << vprt.y() << ";" << vprt.z() << ";" << pc_index << ";0;" << i << ";" << pc.file_name << std::endl;
                        }
                    }
                }
            }
        }
        outfile_initial.close();
        outfile_result.close();

        const auto &obs = observation_picking.observations[i];
        double rms_initial = 0.0;
        int sum = 0;
        double rms_result = 0.0;

        for (const auto &[key1, value1] : obs)
        {
            for (const auto &[key2, value2] : obs)
            {
                if (key1 != key2)
                {
                    Eigen::Vector3d p1, p2;
                    p1 = point_clouds_container.point_clouds[key1].m_initial_pose * value1;
                    p2 = point_clouds_container.point_clouds[key2].m_initial_pose * value2;
                    rms_initial += (p2.x() - p1.x()) * (p2.x() - p1.x());
                    rms_initial += (p2.y() - p1.y()) * (p2.y() - p1.y());

                    p1 = point_clouds_container.point_clouds[key1].m_pose * value1;
                    p2 = point_clouds_container.point_clouds[key2].m_pose * value2;
                    rms_result += (p2.x() - p1.x()) * (p2.x() - p1.x());
                    rms_result += (p2.y() - p1.y()) * (p2.y() - p1.y());

                    sum += 2;
                }
            }
        }
        std::cout << "sum: " << sum << std::endl;
        if (sum > 0)
        {
            rms_initial = sqrt(rms_initial / sum);
            rms_result = sqrt(rms_result / sum);
            outfile_rms << i << ";" << rms_initial << ";" << rms_result << std::endl;
        }
    }
    outfile_rms.close();

    std::string file_name_poses = "poses_RESSO.reg";
    auto path_poses = path;
    path_poses /= file_name_poses;
    std::cout << "saving poses to: " << path_poses << std::endl;
    point_clouds_container.save_poses(path_poses.string());
}

void export_result_to_folder(std::string output_folder_name, int method_id)
{
    fs::path path(output_folder_name);
    path /= std::to_string(method_id);
    create_directory(path);
    export_result_to_folder(path.string());
}

void observation_picking_gui()
{
    static std::string observations_file_name = "";

    ImGui::Begin("Observations");

    ImGui::Checkbox("observation picking mode", &observation_picking.is_observation_picking_mode);
    if (observation_picking.is_observation_picking_mode)
    {
        ImGui::Checkbox("grid 10 x 10 [m]", &observation_picking.grid10x10m);
        ImGui::Checkbox("grid 1 x 1 [m]", &observation_picking.grid1x1m);
        ImGui::Checkbox("grid 0.1 x 0.1 [m]", &observation_picking.grid01x01m);
        ImGui::Checkbox("grid 0.01 x 0.01 [m]", &observation_picking.grid001x001m);
        // ImGui::SliderFloat("picking_plane_height", &observation_picking.picking_plane_height, -20.0f, 20.0f);
        ImGui::InputFloat("picking_plane_height", &observation_picking.picking_plane_height);
        // ImGui::SliderFloat("picking_plane_threshold", &observation_picking.picking_plane_threshold, 0.01f, 200.0f);
        ImGui::InputFloat("picking_plane_threshold", &observation_picking.picking_plane_threshold);
        // ImGui::SliderFloat("picking_plane_max_xy", &observation_picking.max_xy, 10.0f, 1000.0f);
        ImGui::InputFloat("picking_plane_max_xy", &observation_picking.max_xy);
        // ImGui::SliderInt("point_size", &observation_picking.point_size, 1, 10);
        ImGui::InputInt("point_size", &observation_picking.point_size);

        if (ImGui::Button("accept_current_observation"))
        {
            std::vector<Eigen::Affine3d> m_poses;
            for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
            {
                m_poses.push_back(point_clouds_container.point_clouds[i].m_pose);
            }
            observation_picking.accept_current_observation(m_poses);
        }

        ImGui::SameLine();

        if (ImGui::Button("clear_current_observation"))
        {
            observation_picking.current_observation.clear();
        }

        if (ImGui::Button("reset view"))
        {
            rotate_x = 0.0;
            rotate_y = 0.0;
        }
    }

    ImGui::Text((std::string("number of observations: ") + std::to_string(observation_picking.observations.size())).c_str());

    if (ImGui::Button("load observations"))
    {
        static std::shared_ptr<pfd::open_file> open_file;
        std::string input_file_name = "";
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
        const auto t = [&]()
        {
            auto sel = pfd::open_file("Load observations", "C:\\").result();
            for (int i = 0; i < sel.size(); i++)
            {
                input_file_name = sel[i];
                std::cout << "json file: '" << input_file_name << "'" << std::endl;
            }
        };
        std::thread t1(t);
        t1.join();

        if (input_file_name.size() > 0)
        {
            observations_file_name = input_file_name;
            observation_picking.import_observations(input_file_name);

            for (const auto &obs : observation_picking.observations)
            {
                for (const auto &[key, value] : obs)
                {
                    if (point_clouds_container.show_with_initial_pose)
                    {
                        auto p = point_clouds_container.point_clouds[key].m_initial_pose * value;
                        observation_picking.add_intersection(p);
                    }
                    else
                    {
                        auto p = point_clouds_container.point_clouds[key].m_pose * value;
                        observation_picking.add_intersection(p);
                    }
                    break;
                }
            }
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("save observations"))
    {
        static std::shared_ptr<pfd::save_file> save_file;
        std::string output_file_name = "";
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
        const auto t = [&]()
        {
            auto sel = pfd::save_file("Save observations", "C:\\").result();
            output_file_name = sel;
            std::cout << "json file to save: '" << output_file_name << "'" << std::endl;
        };
        std::thread t1(t);
        t1.join();

        if (output_file_name.size() > 0)
        {
            observation_picking.export_observation(output_file_name);
        }
    }

    ImGui::Text((std::string("Loaded observations from file: '") + observations_file_name + std::string("'")).c_str());

    if (ImGui::Button("compute RMS(xy)"))
    {
        double rms = compute_rms(true);
        std::cout << "RMS (initial poses): " << rms << std::endl;
        rms = compute_rms(false);
        std::cout << "RMS (current poses): " << rms << std::endl;
    }

    ImGui::Text("------------------------------------------------");
    if (ImGui::Button("add intersection"))
    {
        observation_picking.add_intersection(Eigen::Vector3d(0.0, 0.0, 0.0));
    }

    ImGui::SameLine();

    /*if (ImGui::Button("add intersections from loaded observations")) {
        for (const auto& obs : observation_picking.observations) {
            for (const auto& [key, value] : obs) {
                if (point_clouds_container.show_with_initial_pose) {
                    auto p = point_clouds_container.point_clouds[key].m_initial_pose * value;
                    observation_picking.add_intersection(p);
                }
                else {
                    auto p = point_clouds_container.point_clouds[key].m_pose * value;
                    observation_picking.add_intersection(p);
                }
                break;
            }
        }
    }*/

    int index_intersetion_to_remove = -1;
    for (int i = 0; i < observation_picking.intersections.size(); i++)
    {
        ImGui::Text("--");
        ImGui::Text(std::string("intersection: '" + std::to_string(i) + "'").c_str());
        ImGui::ColorEdit3(std::string(std::to_string(i) + ": color").c_str(), observation_picking.intersections[i].color);
        ImGui::InputFloat3(std::string(std::to_string(i) + ": translation [m]").c_str(), observation_picking.intersections[i].translation);
        ImGui::InputFloat3(std::string(std::to_string(i) + ": rotation [deg]").c_str(), observation_picking.intersections[i].rotation);
        ImGui::InputFloat3(std::string(std::to_string(i) + ": width_length_height [m]").c_str(), observation_picking.intersections[i].width_length_height);
        if (ImGui::Button(std::string("remove: '" + std::to_string(i) + "'").c_str()))
        {
            index_intersetion_to_remove = i;
        }
    }
    if (index_intersetion_to_remove != -1)
    {
        std::vector<Intersection> intersections;
        for (int i = 0; i < observation_picking.intersections.size(); i++)
        {
            if (i != index_intersetion_to_remove)
            {
                intersections.push_back(observation_picking.intersections[i]);
            }
        }
        observation_picking.intersections = intersections;
    }
    ImGui::Text("------------------------------------------------");
    if (observation_picking.intersections.size() > 0)
    {
        if (ImGui::Button("export point clouds inside intersections, rms and poses (RESSO format) to folder"))
        {
            static std::shared_ptr<pfd::select_folder> selected_folder;
            std::string output_folder_name = "";
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)selected_folder);
            const auto t = [&]()
            {
                auto sel = pfd::select_folder("Choose folder", "C:\\").result();
                output_folder_name = sel;
                std::cout << "folder: '" << output_folder_name << "'" << std::endl;
            };
            std::thread t1(t);
            t1.join();

            if (output_folder_name.size() > 0)
            {
                export_result_to_folder(output_folder_name);
            }
        }
        ImGui::InputFloat("label_dist", &observation_picking.label_dist);
    }
    ImGui::Text("------------------------------------------------");

    ImGui::End();
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
        glTranslatef(translate_x, translate_y, translate_z);
        glRotatef(rotate_x, 1.0, 0.0, 0.0);
        glRotatef(rotate_y, 0.0, 0.0, 1.0);
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
        glVertex3f(100, 0.0f, 0.0f);

        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 100, 0.0f);

        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 100);
        glEnd();
    }

    if (manual_pose_graph_loop_closure_mode)
    {
        manual_pose_graph_loop_closure.Render(point_clouds_container, index_loop_closure_source, index_loop_closure_target);
    }
    else
    {
        for (const auto &g : available_geo_points)
        {
            glBegin(GL_LINES);
            glColor3f(1.0f, 0.0f, 0.0f);
            auto c = g.coordinates - point_clouds_container.offset;
            glVertex3f(c.x() - 0.5, c.y(), c.z());
            glVertex3f(c.x() + 0.5, c.y(), c.z());

            glVertex3f(c.x(), c.y() - 0.5, c.z());
            glVertex3f(c.x(), c.y() + 0.5, c.z());

            glVertex3f(c.x(), c.y(), c.z() - 0.5);
            glVertex3f(c.x(), c.y(), c.z() + 0.5);
            glEnd();
        }

        //
        for (const auto &pc : point_clouds_container.point_clouds)
        {
            for (const auto &gp : pc.available_geo_points)
            {
                if (gp.choosen)
                {
                    auto c = pc.m_pose * gp.coordinates;
                    glBegin(GL_LINES);
                    glColor3f(1.0f, 0.0f, 0.0f);
                    glVertex3f(c.x() - 0.5, c.y(), c.z());
                    glVertex3f(c.x() + 0.5, c.y(), c.z());

                    glVertex3f(c.x(), c.y() - 0.5, c.z());
                    glVertex3f(c.x(), c.y() + 0.5, c.z());

                    glVertex3f(c.x(), c.y(), c.z() - 0.5);
                    glVertex3f(c.x(), c.y(), c.z() + 0.5);
                    glEnd();

                    glBegin(GL_LINES);
                    glColor3f(0.0f, 1.0f, 0.0f);
                    glVertex3f(c.x(), c.y(), c.z());
                    glVertex3f(gp.coordinates.x(), gp.coordinates.y(), gp.coordinates.z());
                    glEnd();

                    glColor3f(0.0f, 0.0f, 0.0f);
                    glBegin(GL_LINES);
                    glVertex3f(c.x(), c.y(), c.z());
                    glVertex3f(c.x() + 10, c.y(), c.z());
                    glEnd();

                    glRasterPos3f(c.x() + 10, c.y(), c.z());
                    glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, (const unsigned char *)gp.name.c_str());
                }
            }
        }
    }

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();

    // my_display_code();
    if (is_ndt_gui)
        ndt_gui();
    if (is_icp_gui)
        icp_gui();
    if (is_pose_graph_slam)
        pose_graph_slam_gui();
    if (is_registration_plane_feature)
        registration_plane_feature_gui();
    if (is_manual_analisys)
        observation_picking_gui();
    //if (manual_pose_graph_loop_closure_mode)
    //{
    //    manual_pose_graph_loop_closure.Gui();
    //}
    project_gui();

    if (!manual_pose_graph_loop_closure_mode)
    {
        for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
        {
            if (point_clouds_container.point_clouds[i].gizmo)
            {
                std::vector<Eigen::Affine3d> all_m_poses;
                for (int j = 0; j < point_clouds_container.point_clouds.size(); j++)
                {
                    all_m_poses.push_back(point_clouds_container.point_clouds[j].m_pose);
                }

                ImGuiIO &io = ImGui::GetIO();
                // ImGuizmo -----------------------------------------------
                ImGuizmo::BeginFrame();
                ImGuizmo::Enable(true);
                ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

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

                point_clouds_container.point_clouds[i].m_pose(0, 0) = m_gizmo[0];
                point_clouds_container.point_clouds[i].m_pose(1, 0) = m_gizmo[1];
                point_clouds_container.point_clouds[i].m_pose(2, 0) = m_gizmo[2];
                point_clouds_container.point_clouds[i].m_pose(3, 0) = m_gizmo[3];
                point_clouds_container.point_clouds[i].m_pose(0, 1) = m_gizmo[4];
                point_clouds_container.point_clouds[i].m_pose(1, 1) = m_gizmo[5];
                point_clouds_container.point_clouds[i].m_pose(2, 1) = m_gizmo[6];
                point_clouds_container.point_clouds[i].m_pose(3, 1) = m_gizmo[7];
                point_clouds_container.point_clouds[i].m_pose(0, 2) = m_gizmo[8];
                point_clouds_container.point_clouds[i].m_pose(1, 2) = m_gizmo[9];
                point_clouds_container.point_clouds[i].m_pose(2, 2) = m_gizmo[10];
                point_clouds_container.point_clouds[i].m_pose(3, 2) = m_gizmo[11];
                point_clouds_container.point_clouds[i].m_pose(0, 3) = m_gizmo[12];
                point_clouds_container.point_clouds[i].m_pose(1, 3) = m_gizmo[13];
                point_clouds_container.point_clouds[i].m_pose(2, 3) = m_gizmo[14];
                point_clouds_container.point_clouds[i].m_pose(3, 3) = m_gizmo[15];
                point_clouds_container.point_clouds[i].pose = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);

                point_clouds_container.point_clouds[i].gui_translation[0] = (float)point_clouds_container.point_clouds[i].pose.px;
                point_clouds_container.point_clouds[i].gui_translation[1] = (float)point_clouds_container.point_clouds[i].pose.py;
                point_clouds_container.point_clouds[i].gui_translation[2] = (float)point_clouds_container.point_clouds[i].pose.pz;

                point_clouds_container.point_clouds[i].gui_rotation[0] = (float)(point_clouds_container.point_clouds[i].pose.om * 180.0 / M_PI);
                point_clouds_container.point_clouds[i].gui_rotation[1] = (float)(point_clouds_container.point_clouds[i].pose.fi * 180.0 / M_PI);
                point_clouds_container.point_clouds[i].gui_rotation[2] = (float)(point_clouds_container.point_clouds[i].pose.ka * 180.0 / M_PI);

                ImGui::End();

                if (!manipulate_only_marked_gizmo)
                {
                    Eigen::Affine3d curr_m_pose = point_clouds_container.point_clouds[i].m_pose;
                    for (int j = i + 1; j < point_clouds_container.point_clouds.size(); j++)
                    {
                        curr_m_pose = curr_m_pose * (all_m_poses[j - 1].inverse() * all_m_poses[j]);
                        point_clouds_container.point_clouds[j].m_pose = curr_m_pose;
                        point_clouds_container.point_clouds[j].pose = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[j].m_pose);

                        point_clouds_container.point_clouds[j].gui_translation[0] = (float)point_clouds_container.point_clouds[j].pose.px;
                        point_clouds_container.point_clouds[j].gui_translation[1] = (float)point_clouds_container.point_clouds[j].pose.py;
                        point_clouds_container.point_clouds[j].gui_translation[2] = (float)point_clouds_container.point_clouds[j].pose.pz;

                        point_clouds_container.point_clouds[j].gui_rotation[0] = (float)(point_clouds_container.point_clouds[j].pose.om * 180.0 / M_PI);
                        point_clouds_container.point_clouds[j].gui_rotation[1] = (float)(point_clouds_container.point_clouds[j].pose.fi * 180.0 / M_PI);
                        point_clouds_container.point_clouds[j].gui_rotation[2] = (float)(point_clouds_container.point_clouds[j].pose.ka * 180.0 / M_PI);
                    }
                }
            }
        }

        point_clouds_container.render(observation_picking, viewer_decmiate_point_cloud);
        observation_picking.render();

        glPushAttrib(GL_ALL_ATTRIB_BITS);
        glPointSize(5);
        for (const auto &obs : observation_picking.observations)
        {
            for (const auto &[key1, value1] : obs)
            {
                for (const auto &[key2, value2] : obs)
                {
                    if (key1 != key2)
                    {
                        Eigen::Vector3d p1, p2;
                        if (point_clouds_container.show_with_initial_pose)
                        {
                            p1 = point_clouds_container.point_clouds[key1].m_initial_pose * value1;
                            p2 = point_clouds_container.point_clouds[key2].m_initial_pose * value2;
                        }
                        else
                        {
                            p1 = point_clouds_container.point_clouds[key1].m_pose * value1;
                            p2 = point_clouds_container.point_clouds[key2].m_pose * value2;
                        }
                        glColor3f(0, 1, 0);
                        glBegin(GL_POINTS);
                        glVertex3f(p1.x(), p1.y(), p1.z());
                        glVertex3f(p2.x(), p2.y(), p2.z());
                        glEnd();
                        glColor3f(1, 0, 0);
                        glBegin(GL_LINES);
                        glVertex3f(p1.x(), p1.y(), p1.z());
                        glVertex3f(p2.x(), p2.y(), p2.z());
                        glEnd();
                    }
                }
            }
        }
        glPopAttrib();

        for (const auto &obs : observation_picking.observations)
        {
            Eigen::Vector3d mean(0, 0, 0);
            int counter = 0;
            for (const auto &[key1, value1] : obs)
            {
                mean += point_clouds_container.point_clouds[key1].m_initial_pose * value1;
                counter++;
            }
            if (counter > 0)
            {
                mean /= counter;

                glColor3f(1, 0, 0);
                glBegin(GL_LINE_STRIP);
                glVertex3f(mean.x() - 1, mean.y() - 1, mean.z());
                glVertex3f(mean.x() + 1, mean.y() - 1, mean.z());
                glVertex3f(mean.x() + 1, mean.y() + 1, mean.z());
                glVertex3f(mean.x() - 1, mean.y() + 1, mean.z());
                glVertex3f(mean.x() - 1, mean.y() - 1, mean.z());
                glEnd();
            }
        }

        glColor3f(1, 0, 1);
        glBegin(GL_POINTS);
        for (auto p : picked_points)
        {
            glVertex3f(p.x(), p.y(), p.z());
        }
        glEnd();
    }else{
        // ImGuizmo -----------------------------------------------
        if (manual_pose_graph_loop_closure.gizmo && manual_pose_graph_loop_closure.edges.size() > 0)
        {
            ImGuizmo::BeginFrame();
            ImGuizmo::Enable(true);
            ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

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

            Eigen::Affine3d m_g = Eigen::Affine3d::Identity();

            m_g(0, 0) = m_gizmo[0];
            m_g(1, 0) = m_gizmo[1];
            m_g(2, 0) = m_gizmo[2];
            m_g(3, 0) = m_gizmo[3];
            m_g(0, 1) = m_gizmo[4];
            m_g(1, 1) = m_gizmo[5];
            m_g(2, 1) = m_gizmo[6];
            m_g(3, 1) = m_gizmo[7];
            m_g(0, 2) = m_gizmo[8];
            m_g(1, 2) = m_gizmo[9];
            m_g(2, 2) = m_gizmo[10];
            m_g(3, 2) = m_gizmo[11];
            m_g(0, 3) = m_gizmo[12];
            m_g(1, 3) = m_gizmo[13];
            m_g(2, 3) = m_gizmo[14];
            m_g(3, 3) = m_gizmo[15];

            const int &index_src = manual_pose_graph_loop_closure.edges[manual_pose_graph_loop_closure.index_active_edge].index_from;
           
            const Eigen::Affine3d &m_src = point_clouds_container.point_clouds.at(index_src).m_pose;
            manual_pose_graph_loop_closure.edges[manual_pose_graph_loop_closure.index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_src.inverse() * m_g);

            ImGui::End();
        }
    }

    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    glutSwapBuffers();
    glutPostRedisplay();
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
                rotate_x += dy * 0.2f * mouse_sensitivity;
                rotate_y += dx * 0.2f * mouse_sensitivity;
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

    if (!io.WantCaptureMouse)
    {
        if (state == GLUT_DOWN)
        {
            mouse_buttons |= 1 << glut_button;

            //-
            if (observation_picking.is_observation_picking_mode)
            {
                Eigen::Vector3d p = GLWidgetGetOGLPos(x, y, observation_picking);
                int number_active_pcs = 0;
                int index_picked = -1;
                for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
                {
                    if (point_clouds_container.point_clouds[i].visible)
                    {
                        number_active_pcs++;
                        index_picked = i;
                    }
                }
                if (number_active_pcs == 1)
                {
                    observation_picking.add_picked_to_current_observation(index_picked, p);
                }
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

bool initGL(int *argc, char **argv)
{
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowSize(window_width, window_height);
    glutCreateWindow("multi_view_tls_registration v0.14");
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

int main(int argc, char *argv[])
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

float distanceToPlane(const RegistrationPlaneFeature::Plane &plane, const Eigen::Vector3d &p)
{
    return (plane.a * p.x() + plane.b * p.y() + plane.c * p.z() + plane.d);
}

Eigen::Vector3d rayIntersection(const LaserBeam &laser_beam, const RegistrationPlaneFeature::Plane &plane)
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

Eigen::Vector3d GLWidgetGetOGLPos(int x, int y, const ObservationPicking &observation_picking)
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

    RegistrationPlaneFeature::Plane pl;

    pl.a = 0;
    pl.b = 0;
    pl.c = 1;
    pl.d = -observation_picking.picking_plane_height;

    Eigen::Vector3d pos = rayIntersection(laser_beam, pl);

    std::cout << "intersection: " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;

    return pos;
}

void reset_poses()
{
    for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
    {
        point_clouds_container.point_clouds[i].m_pose = point_clouds_container.point_clouds[i].m_initial_pose;
        point_clouds_container.point_clouds[i].pose = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);
        point_clouds_container.point_clouds[i].gui_translation[0] = (float)point_clouds_container.point_clouds[i].pose.px;
        point_clouds_container.point_clouds[i].gui_translation[1] = (float)point_clouds_container.point_clouds[i].pose.py;
        point_clouds_container.point_clouds[i].gui_translation[2] = (float)point_clouds_container.point_clouds[i].pose.pz;
        point_clouds_container.point_clouds[i].gui_rotation[0] = (float)rad2deg(point_clouds_container.point_clouds[i].pose.om);
        point_clouds_container.point_clouds[i].gui_rotation[1] = (float)rad2deg(point_clouds_container.point_clouds[i].pose.fi);
        point_clouds_container.point_clouds[i].gui_rotation[2] = (float)rad2deg(point_clouds_container.point_clouds[i].pose.ka);
    }
}

double compute_rms(bool initial)
{
    double rms = 0.0;
    int sum = 0;
    for (const auto &obs : observation_picking.observations)
    {
        for (const auto &[key1, value1] : obs)
        {
            for (const auto &[key2, value2] : obs)
            {
                if (key1 != key2)
                {
                    Eigen::Vector3d p1, p2;
                    if (initial)
                    {
                        p1 = point_clouds_container.point_clouds[key1].m_initial_pose * value1;
                        p2 = point_clouds_container.point_clouds[key2].m_initial_pose * value2;
                    }
                    else
                    {
                        p1 = point_clouds_container.point_clouds[key1].m_pose * value1;
                        p2 = point_clouds_container.point_clouds[key2].m_pose * value2;
                    }
                    rms += (p2.x() - p1.x()) * (p2.x() - p1.x());
                    sum++;
                    rms += (p2.y() - p1.y()) * (p2.y() - p1.y());
                    sum++;
                }
            }
        }
    }
    if (sum == 0)
    {
        std::cout << "sum == 0" << std::endl;
        return -1;
    }
    else
    {
        rms = sqrt(rms / sum);
        return rms;
    }
}

template <typename T>
void append_to_result_file(std::string file_name, std::string method, const T &t, float rms, int id_method, std::chrono::milliseconds elapsed)
{
    std::ofstream outfile;
    outfile.open(file_name, std::ios_base::app);
    outfile << method << ";" << id_method << ";" << int(t.is_gauss_newton) << ";" << int(t.is_levenberg_marguardt) << ";" << int(t.is_wc) << ";" << int(t.is_cw)
            << ";" << int(t.is_tait_bryan_angles) << ";" << int(t.is_quaternion) << ";" << int(t.is_rodrigues) << ";" << int(t.is_lie_algebra_left_jacobian)
            << ";" << int(t.is_lie_algebra_right_jacobian) << ";" << rms << ";" << elapsed.count() << std::endl;
    outfile.close();
}

void create_header(std::string file_name)
{
    std::ofstream outfile;
    outfile.open(file_name);
    outfile << "method;id_method;gauss_newton;levenberg_marguardt;wc;cw;tait_bryan_angles;quaternion;rodrigues;Lie_algebra_left_jacobian;Lie_algebra_right_jacobian;rms;elapsed_time_miliseconds" << std::endl;
    outfile.close();
}

void add_initial_rms_to_file(std::string file_name, float rms)
{
    std::ofstream outfile;
    outfile.open(file_name, std::ios_base::app);
    // outfile << "method;is_gauss_newton;is_levenberg_marguardt;is_wc;is_cw;is_tait_bryan_angles;is_quaternion;is_rodrigues;is_Lie_algebra_left;Lie_algebra_right;rms;elapsed_time_miliseconds" << std::endl;
    outfile << "initial_rms;0;0;0;0;0;0;0;0;0;0;" << rms << ";0" << std::endl;
    outfile.close();
}

void perform_experiment_on_linux()
{
    fs::path path_result(working_directory);
    path_result /= "results_linux";
    create_directory(path_result);

    point_clouds_container.show_with_initial_pose = false;
    auto temp_data = point_clouds_container;
    // reset_poses();
    float rms = 0.0f;
    std::string result_file = working_directory + "/result_linux.csv";
    create_header(result_file);
    double initial_rms = compute_rms(false);
    std::cout << "initial rms: " << initial_rms << std::endl;
    add_initial_rms_to_file(result_file, initial_rms);

    float search_radious = 0.1f;
    int number_of_threads = 16;
    int number_of_iterations = 6;
    int id_method = 0;

    // pose graph slam
    //--94--
    pose_graph_slam.overlap_threshold = 0.3;
    pose_graph_slam.iterations = 6;

    pose_graph_slam.search_radious = search_radious;
    pose_graph_slam.number_of_threads = number_of_threads;
    pose_graph_slam.number_of_iterations_pair_wise_matching = number_of_iterations;

    //--
    pose_graph_slam.is_adaptive_robust_kernel = false;
    pose_graph_slam.is_fix_first_node = true;
    pose_graph_slam.is_gauss_newton = true;
    pose_graph_slam.is_levenberg_marguardt = false;
    pose_graph_slam.is_cw = false;
    pose_graph_slam.is_wc = true;
    pose_graph_slam.is_tait_bryan_angles = false;
    pose_graph_slam.is_quaternion = false;
    pose_graph_slam.is_rodrigues = true;

    point_clouds_container = temp_data;

    // pose_graph_slam.is_ndt = true;
    // pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    // pose_graph_slam.ndt_bucket_size[0] = ndt.bucket_size[0];
    // pose_graph_slam.ndt_bucket_size[1] = ndt.bucket_size[1];
    // pose_graph_slam.ndt_bucket_size[2] = ndt.bucket_size[2];
    // pose_graph_slam.optimize(point_clouds_container);
    // rms = compute_rms();
    // id_method = 94;
    // append_to_result_file(result_file, "pose_graph_slam (normal_distributions_transform)", pose_graph_slam, rms, id_method);

#ifdef WITH_PCL
    pose_graph_slam.set_all_to_false();
    pose_graph_slam.is_optimize_pcl_ndt = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::pcl_ndt;

    auto start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(point_clouds_container);
    auto end = std::chrono::system_clock::now();
    auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    rms = compute_rms(false);
    id_method = 94;
    append_to_result_file(result_file, "pose_graph_slam (pcl_ndt)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--95--
    pose_graph_slam.set_all_to_false();
    pose_graph_slam.is_optimize_pcl_icp = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::pcl_icp;

    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    rms = compute_rms(false);
    id_method = 95;
    append_to_result_file(result_file, "pose_graph_slam (pcl_icp)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;
#endif

#if WITH_GTSAM
    //--96--
    try
    {
        pose_graph_slam.ndt_bucket_size[0] = ndt.bucket_size[0];
        pose_graph_slam.ndt_bucket_size[1] = ndt.bucket_size[1];
        pose_graph_slam.ndt_bucket_size[2] = ndt.bucket_size[2];

        pose_graph_slam.set_all_to_false();
        pose_graph_slam.is_optimize_pcl_ndt = true;
        pose_graph_slam.is_lie_algebra_right_jacobian = true;
        pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::pcl_ndt;

        start = std::chrono::system_clock::now();
        pose_graph_slam.optimize_with_GTSAM(point_clouds_container);
        end = std::chrono::system_clock::now();
        elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        rms = compute_rms(false);
        id_method = 96;
        append_to_result_file(result_file, "pose_graph_slam (GTSAM pcl_ndt)", pose_graph_slam, rms, id_method, elapsed);
        export_result_to_folder(path_result.string(), id_method);
        point_clouds_container = temp_data;
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl;
        rms = compute_rms(false);
        append_to_result_file(result_file, "pose_graph_slam (GTSAM pcl_ndt)", pose_graph_slam, rms, id_method, elapsed);
        export_result_to_folder(path_result.string(), id_method);
        point_clouds_container = temp_data;
    }
    //--97--
    try
    {
        pose_graph_slam.set_all_to_false();
        pose_graph_slam.is_optimize_pcl_icp = true;
        pose_graph_slam.is_lie_algebra_right_jacobian = true;
        pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::pcl_icp;

        start = std::chrono::system_clock::now();
        pose_graph_slam.optimize_with_GTSAM(point_clouds_container);
        end = std::chrono::system_clock::now();
        elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        rms = compute_rms(false);
        id_method = 97;
        append_to_result_file(result_file, "pose_graph_slam (GTSAM pcl_icp)", pose_graph_slam, rms, id_method, elapsed);
        export_result_to_folder(path_result.string(), id_method);
        point_clouds_container = temp_data;
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl;
        rms = compute_rms(false);
        append_to_result_file(result_file, "pose_graph_slam (GTSAM pcl_icp)", pose_graph_slam, rms, id_method, elapsed);
        export_result_to_folder(path_result.string(), id_method);
        point_clouds_container = temp_data;
    }
#endif
#if WITH_MANIF
    //--98--
    pose_graph_slam.set_all_to_false();
    pose_graph_slam.is_optimize_pcl_ndt = true;
    pose_graph_slam.is_lie_algebra_right_jacobian = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::pcl_ndt;

    auto start = std::chrono::system_clock::now();
    pose_graph_slam.optimize_with_manif(point_clouds_container);
    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    rms = compute_rms(false);
    id_method = 98;
    append_to_result_file(result_file, "pose_graph_slam (manif pcl_ndt)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--99--
    pose_graph_slam.set_all_to_false();
    pose_graph_slam.is_optimize_pcl_icp = true;
    pose_graph_slam.is_lie_algebra_right_jacobian = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::pcl_icp;

    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize_with_manif(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    rms = compute_rms(false);
    id_method = 99;
    append_to_result_file(result_file, "pose_graph_slam (manif pcl_icp)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;
#endif
}

void perform_experiment_on_windows()
{
    point_clouds_container.show_with_initial_pose = false;
    auto temp_data = point_clouds_container;
    reset_poses();
    double rms = 0.0f;
    std::string result_file = working_directory + "/result_win.csv";
    float search_radious = 0.1f;
    int number_of_threads = 16;
    int number_of_iterations = 6;
    int id_method = 0;

    create_header(result_file);
    double initial_rms = compute_rms(false);
    std::cout << "initial rms: " << initial_rms << std::endl;
    add_initial_rms_to_file(result_file, initial_rms);

    // void export_result_to_folder(std::string output_folder_name, int method_id) {
    //     fs::path path(output_folder_name);
    //     path /= std::to_string(method_id);
    //     create_directory(path);
    //     export_result_to_folder(path.string());
    // }

    fs::path path_result(working_directory);
    path_result /= "results_win";
    create_directory(path_result);

    //--0--
    icp.is_adaptive_robust_kernel = false;
    icp.is_fix_first_node = false;
    icp.search_radious = search_radious;
    icp.number_of_threads = number_of_threads;
    icp.number_of_iterations = number_of_iterations;
    icp.is_adaptive_robust_kernel = false;

    icp.is_gauss_newton = true;
    icp.is_levenberg_marguardt = false;

    icp.is_wc = true;
    icp.is_cw = false;

    icp.is_tait_bryan_angles = true;
    icp.is_quaternion = false;
    icp.is_rodrigues = false;
    icp.is_lie_algebra_left_jacobian = false;
    icp.is_lie_algebra_right_jacobian = false;

    auto start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(point_clouds_container);
    auto end = std::chrono::system_clock::now();
    auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    std::cout << "final rms: " << rms << std::endl;

    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;
    id_method++;

    //--1--
    icp.is_tait_bryan_angles = false;
    icp.is_quaternion = true;
    icp.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 1;

    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;
    // id_method++;
    //--2--
    icp.is_tait_bryan_angles = false;
    icp.is_quaternion = false;
    icp.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 2;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--3--
    icp.is_wc = false;
    icp.is_cw = true;

    icp.is_tait_bryan_angles = true;
    icp.is_quaternion = false;
    icp.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 3;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--4--
    icp.is_tait_bryan_angles = false;
    icp.is_quaternion = true;
    icp.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 4;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--5--
    icp.is_tait_bryan_angles = false;
    icp.is_quaternion = false;
    icp.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 5;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--6--
    icp.is_gauss_newton = false;
    icp.is_levenberg_marguardt = true;

    icp.is_wc = true;
    icp.is_cw = false;

    icp.is_tait_bryan_angles = true;
    icp.is_quaternion = false;
    icp.is_rodrigues = false;
    icp.is_lie_algebra_left_jacobian = false;
    icp.is_lie_algebra_right_jacobian = false;

    start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 6;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--7--
    icp.is_tait_bryan_angles = false;
    icp.is_quaternion = true;
    icp.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 7;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--8--
    icp.is_tait_bryan_angles = false;
    icp.is_quaternion = false;
    icp.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 8;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--9--
    icp.is_wc = false;
    icp.is_cw = true;

    icp.is_tait_bryan_angles = true;
    icp.is_quaternion = false;
    icp.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 9;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--10--
    icp.is_wc = false;
    icp.is_cw = true;

    icp.is_tait_bryan_angles = false;
    icp.is_quaternion = true;
    icp.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 10;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--11--
    icp.is_wc = false;
    icp.is_cw = true;

    icp.is_tait_bryan_angles = false;
    icp.is_quaternion = false;
    icp.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 11;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--12--
    icp.is_gauss_newton = true;
    icp.is_levenberg_marguardt = false;

    icp.is_wc = true;
    icp.is_cw = false;

    icp.is_tait_bryan_angles = false;
    icp.is_quaternion = false;
    icp.is_rodrigues = true;

    icp.is_lie_algebra_left_jacobian = true;
    icp.is_lie_algebra_right_jacobian = false;

    start = std::chrono::system_clock::now();
    icp.optimize_source_to_target_lie_algebra_left_jacobian(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 12;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--13--
    icp.is_lie_algebra_left_jacobian = false;
    icp.is_lie_algebra_right_jacobian = true;
    start = std::chrono::system_clock::now();
    icp.optimize_source_to_target_lie_algebra_right_jacobian(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 13;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //---NDT---
    //--14--
    //
    ndt.is_fix_first_node = false;
    ndt.bucket_size[0] = 0.5;
    ndt.bucket_size[1] = 0.5;
    ndt.bucket_size[2] = 0.5;
    ndt.number_of_threads = number_of_threads;
    ndt.number_of_iterations = number_of_iterations;

    ndt.is_gauss_newton = true;
    ndt.is_levenberg_marguardt = false;

    ndt.is_wc = true;
    ndt.is_cw = false;

    ndt.is_tait_bryan_angles = true;
    ndt.is_quaternion = false;
    ndt.is_rodrigues = false;
    ndt.is_lie_algebra_left_jacobian = false;
    ndt.is_lie_algebra_right_jacobian = false;

    start = std::chrono::system_clock::now();
    ndt.optimize(point_clouds_container.point_clouds, true);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    std::cout << "final rms: " << rms << std::endl;

    id_method = 14;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--15--
    ndt.is_tait_bryan_angles = false;
    ndt.is_quaternion = true;
    ndt.is_rodrigues = false;
    start = std::chrono::system_clock::now();
    ndt.optimize(point_clouds_container.point_clouds, true);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);

    id_method = 15;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    point_clouds_container = temp_data;

    //--16--
    ndt.is_tait_bryan_angles = false;
    ndt.is_quaternion = false;
    ndt.is_rodrigues = true;
    start = std::chrono::system_clock::now();
    ndt.optimize(point_clouds_container.point_clouds, true);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);

    id_method = 16;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--17--
    ndt.is_wc = false;
    ndt.is_cw = true;

    ndt.is_tait_bryan_angles = true;
    ndt.is_quaternion = false;
    ndt.is_rodrigues = false;
    start = std::chrono::system_clock::now();
    ndt.optimize(point_clouds_container.point_clouds, true);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 17;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--18--
    ndt.is_tait_bryan_angles = false;
    ndt.is_quaternion = true;
    ndt.is_rodrigues = false;
    start = std::chrono::system_clock::now();
    ndt.optimize(point_clouds_container.point_clouds, true);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);

    id_method = 18;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--19--
    ndt.is_tait_bryan_angles = false;
    ndt.is_quaternion = false;
    ndt.is_rodrigues = true;
    start = std::chrono::system_clock::now();
    ndt.optimize(point_clouds_container.point_clouds, true);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);

    id_method = 19;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--20--
    ndt.is_gauss_newton = false;
    ndt.is_levenberg_marguardt = true;

    ndt.is_wc = true;
    ndt.is_cw = false;

    ndt.is_tait_bryan_angles = true;
    ndt.is_quaternion = false;
    ndt.is_rodrigues = false;
    start = std::chrono::system_clock::now();
    ndt.optimize(point_clouds_container.point_clouds, true);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);

    id_method = 20;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--21--
    ndt.is_tait_bryan_angles = false;
    ndt.is_quaternion = true;
    ndt.is_rodrigues = false;
    start = std::chrono::system_clock::now();
    ndt.optimize(point_clouds_container.point_clouds, true);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);

    id_method = 21;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--22--
    ndt.is_tait_bryan_angles = false;
    ndt.is_quaternion = false;
    ndt.is_rodrigues = true;
    start = std::chrono::system_clock::now();
    ndt.optimize(point_clouds_container.point_clouds, true);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);

    id_method = 22;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--23--
    ndt.is_wc = false;
    ndt.is_cw = true;

    ndt.is_tait_bryan_angles = true;
    ndt.is_quaternion = false;
    ndt.is_rodrigues = false;
    start = std::chrono::system_clock::now();
    ndt.optimize(point_clouds_container.point_clouds, true);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);

    id_method = 23;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--24--
    ndt.is_tait_bryan_angles = false;
    ndt.is_quaternion = true;
    ndt.is_rodrigues = false;
    start = std::chrono::system_clock::now();
    ndt.optimize(point_clouds_container.point_clouds, true);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);

    id_method = 24;

    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--25--
    ndt.is_tait_bryan_angles = false;
    ndt.is_quaternion = false;
    ndt.is_rodrigues = true;
    start = std::chrono::system_clock::now();
    ndt.optimize(point_clouds_container.point_clouds, true);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);

    id_method = 25;

    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--26--
    ndt.is_gauss_newton = true;
    ndt.is_levenberg_marguardt = false;

    ndt.is_wc = true;
    ndt.is_cw = false;

    ndt.is_tait_bryan_angles = false;
    ndt.is_quaternion = false;
    ndt.is_rodrigues = false;

    ndt.is_lie_algebra_left_jacobian = true;
    ndt.is_lie_algebra_right_jacobian = false;
    start = std::chrono::system_clock::now();
    ndt.optimize(point_clouds_container.point_clouds, true);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);

    id_method = 26;

    ndt.is_rodrigues = true;

    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--27--
    ndt.is_gauss_newton = false;
    ndt.is_levenberg_marguardt = true;
    ndt.is_rodrigues = false;
    start = std::chrono::system_clock::now();
    ndt.optimize(point_clouds_container.point_clouds, true);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);

    id_method = 27;
    ndt.is_rodrigues = true;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--28--
    ndt.is_lie_algebra_left_jacobian = false;
    ndt.is_lie_algebra_right_jacobian = true;
    ndt.is_rodrigues = false;

    ndt.is_gauss_newton = true;
    ndt.is_levenberg_marguardt = false;
    start = std::chrono::system_clock::now();
    ndt.optimize(point_clouds_container.point_clouds, true);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);

    id_method = 28;
    ndt.is_rodrigues = true;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--29--
    ndt.is_gauss_newton = false;
    ndt.is_levenberg_marguardt = true;
    ndt.is_rodrigues = false;
    start = std::chrono::system_clock::now();
    ndt.optimize(point_clouds_container.point_clouds, true);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);

    id_method = 29;
    ndt.is_rodrigues = true;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //----------------------------------------------------------------------------

    registration_plane_feature.search_radious = search_radious;
    registration_plane_feature.number_of_threads = number_of_threads;
    registration_plane_feature.number_of_iterations = number_of_iterations;
    registration_plane_feature.is_adaptive_robust_kernel = false;
    registration_plane_feature.is_fix_first_node = false;

    //--30--
    registration_plane_feature.is_gauss_newton = true;
    registration_plane_feature.is_levenberg_marguardt = false;

    registration_plane_feature.is_wc = true;
    registration_plane_feature.is_cw = false;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 30;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--31--
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 31;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--32--
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 32;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--33--
    registration_plane_feature.is_wc = false;
    registration_plane_feature.is_cw = true;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 33;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--34--
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 34;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--35--
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 35;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //------------------------------------------------------
    //--36--
    registration_plane_feature.is_gauss_newton = false;
    registration_plane_feature.is_levenberg_marguardt = true;

    registration_plane_feature.is_wc = true;
    registration_plane_feature.is_cw = false;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 36;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--37--
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 37;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--38--
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 38;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--39--
    registration_plane_feature.is_wc = false;
    registration_plane_feature.is_cw = true;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 39;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--40--
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 40;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--41--
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 41;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--42-- Lie
    registration_plane_feature.is_gauss_newton = true;
    registration_plane_feature.is_levenberg_marguardt = false;

    registration_plane_feature.is_wc = true;
    registration_plane_feature.is_cw = false;

    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    registration_plane_feature.is_lie_algebra_left_jacobian = true;
    registration_plane_feature.is_lie_algebra_right_jacobian = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_left_jacobian(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 42;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--43--
    registration_plane_feature.is_gauss_newton = false;
    registration_plane_feature.is_levenberg_marguardt = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_left_jacobian(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 43;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--44--
    registration_plane_feature.is_gauss_newton = true;
    registration_plane_feature.is_levenberg_marguardt = false;
    registration_plane_feature.is_lie_algebra_left_jacobian = false;
    registration_plane_feature.is_lie_algebra_right_jacobian = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_right_jacobian(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 44;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--45--
    registration_plane_feature.is_gauss_newton = false;
    registration_plane_feature.is_levenberg_marguardt = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_right_jacobian(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 45;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--46--using dot product
    registration_plane_feature.is_lie_algebra_left_jacobian = false;
    registration_plane_feature.is_lie_algebra_right_jacobian = false;

    registration_plane_feature.is_gauss_newton = true;
    registration_plane_feature.is_levenberg_marguardt = false;

    registration_plane_feature.is_wc = true;
    registration_plane_feature.is_cw = false;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 46;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--47
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 47;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--48
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 48;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--49
    registration_plane_feature.is_wc = false;
    registration_plane_feature.is_cw = true;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 49;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--50
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 50;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--51
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 51;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--52
    registration_plane_feature.is_gauss_newton = false;
    registration_plane_feature.is_levenberg_marguardt = true;

    registration_plane_feature.is_wc = true;
    registration_plane_feature.is_cw = false;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 52;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--53
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 53;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--54
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 54;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--55
    registration_plane_feature.is_wc = false;
    registration_plane_feature.is_cw = true;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 55;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--56
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 56;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--57
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 57;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--58 optimize_distance_point_to_plane_source_to_target
    registration_plane_feature.is_gauss_newton = true;
    registration_plane_feature.is_levenberg_marguardt = false;

    registration_plane_feature.is_wc = true;
    registration_plane_feature.is_cw = false;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 58;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--59
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 59;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--60
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 60;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--61
    registration_plane_feature.is_wc = false;
    registration_plane_feature.is_cw = true;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 61;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--62
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 62;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--63
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 63;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--64
    registration_plane_feature.is_gauss_newton = false;
    registration_plane_feature.is_levenberg_marguardt = true;

    registration_plane_feature.is_wc = true;
    registration_plane_feature.is_cw = false;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 64;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--65
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 65;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--66
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 66;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--67
    registration_plane_feature.is_wc = false;
    registration_plane_feature.is_cw = true;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 67;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--68
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 68;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--69
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 69;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--70 optimize_plane_to_plane_source_to_target
    registration_plane_feature.is_adaptive_robust_kernel = true;

    registration_plane_feature.is_gauss_newton = true;
    registration_plane_feature.is_levenberg_marguardt = false;

    registration_plane_feature.is_wc = true;
    registration_plane_feature.is_cw = false;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 70;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--71
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 71;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--72
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 72;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--73
    registration_plane_feature.is_wc = false;
    registration_plane_feature.is_cw = true;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 73;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--74
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 74;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--75
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 75;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--76
    registration_plane_feature.is_gauss_newton = false;
    registration_plane_feature.is_levenberg_marguardt = true;

    registration_plane_feature.is_wc = true;
    registration_plane_feature.is_cw = false;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 76;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--77
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 77;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--78
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 78;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--79
    registration_plane_feature.is_wc = false;
    registration_plane_feature.is_cw = true;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 79;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--80
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 80;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--81
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 81;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    // pose graph slam
    //--82--
    pose_graph_slam.overlap_threshold = 0.3;
    pose_graph_slam.iterations = 6;

    pose_graph_slam.search_radious = search_radious;
    pose_graph_slam.number_of_threads = number_of_threads;
    pose_graph_slam.number_of_iterations_pair_wise_matching = number_of_iterations;

    //--
    pose_graph_slam.is_adaptive_robust_kernel = false;
    pose_graph_slam.is_fix_first_node = true;
    pose_graph_slam.is_gauss_newton = true;
    pose_graph_slam.is_levenberg_marguardt = false;
    pose_graph_slam.is_cw = false;
    pose_graph_slam.is_wc = true;
    pose_graph_slam.is_tait_bryan_angles = true;
    pose_graph_slam.is_quaternion = false;
    pose_graph_slam.is_rodrigues = false;

    point_clouds_container = temp_data;
    pose_graph_slam.set_all_to_false();
    pose_graph_slam.is_ndt = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    pose_graph_slam.ndt_bucket_size[0] = ndt.bucket_size[0];
    pose_graph_slam.ndt_bucket_size[1] = ndt.bucket_size[1];
    pose_graph_slam.ndt_bucket_size[2] = ndt.bucket_size[2];
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 82;
    append_to_result_file(result_file, "pose_graph_slam (normal_distributions_transform)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);

    //--83
    point_clouds_container = temp_data;
    pose_graph_slam.set_all_to_false();
    pose_graph_slam.is_optimization_point_to_point_source_to_target = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 83;
    append_to_result_file(result_file, "pose_graph_slam (point_to_point)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--84
    point_clouds_container = temp_data;
    pose_graph_slam.set_all_to_false();
    pose_graph_slam.is_optimize_point_to_projection_onto_plane_source_to_target = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 84;
    append_to_result_file(result_file, "pose_graph_slam (point_to_projection_onto_plane)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);

    //--85
    point_clouds_container = temp_data;
    pose_graph_slam.set_all_to_false();
    pose_graph_slam.is_optimize_point_to_plane_source_to_target = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 85;
    append_to_result_file(result_file, "pose_graph_slam (point_to_plane_using_dot_product)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);

    //--86
    point_clouds_container = temp_data;
    pose_graph_slam.set_all_to_false();
    pose_graph_slam.is_optimize_distance_point_to_plane_source_to_target = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 86;
    append_to_result_file(result_file, "pose_graph_slam (distance_point_to_plane)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
    point_clouds_container = temp_data;

    //--87
    pose_graph_slam.is_adaptive_robust_kernel = true;

    pose_graph_slam.set_all_to_false();
    pose_graph_slam.is_optimize_plane_to_plane_source_to_target = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 87;
    append_to_result_file(result_file, "pose_graph_slam (plane_to_plane)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);

    //--88
    pose_graph_slam.set_all_to_false();
    point_clouds_container = temp_data;
    pose_graph_slam.is_adaptive_robust_kernel = false;
    pose_graph_slam.is_ndt_lie_algebra_left_jacobian = true;
    pose_graph_slam.is_lie_algebra_left_jacobian = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 88;
    append_to_result_file(result_file, "pose_graph_slam (ndt_lie_algebra_left_jacobian)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);

    //--89
    pose_graph_slam.set_all_to_false();
    point_clouds_container = temp_data;
    pose_graph_slam.is_ndt_lie_algebra_right_jacobian = true;
    pose_graph_slam.is_lie_algebra_right_jacobian = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 89;
    append_to_result_file(result_file, "pose_graph_slam (ndt_lie_algebra_right_jacobian)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);

    //--90
    pose_graph_slam.set_all_to_false();
    point_clouds_container = temp_data;
    pose_graph_slam.is_optimize_point_to_point_source_to_target_lie_algebra_left_jacobian = true;
    pose_graph_slam.is_lie_algebra_left_jacobian = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 90;
    append_to_result_file(result_file, "pose_graph_slam (point_to_point_lie_algebra_left_jacobian)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);

    //--91
    pose_graph_slam.set_all_to_false();
    point_clouds_container = temp_data;
    pose_graph_slam.is_optimize_point_to_point_source_to_target_lie_algebra_right_jacobian = true;
    pose_graph_slam.is_lie_algebra_right_jacobian = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 91;
    append_to_result_file(result_file, "pose_graph_slam (point_to_point_lie_algebra_right_jacobian)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);

    //--92
    pose_graph_slam.set_all_to_false();
    point_clouds_container = temp_data;
    pose_graph_slam.is_optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_left_jacobian = true;
    pose_graph_slam.is_lie_algebra_left_jacobian = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 92;
    append_to_result_file(result_file, "pose_graph_slam (point_to_projection_onto_plane_lie_algebra_left_jacobian)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);

    //--93
    pose_graph_slam.set_all_to_false();
    point_clouds_container = temp_data;
    pose_graph_slam.is_optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_right_jacobian = true;
    pose_graph_slam.is_lie_algebra_right_jacobian = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false);
    id_method = 93;
    append_to_result_file(result_file, "pose_graph_slam (point_to_projection_onto_plane_lie_algebra_right_jacobian)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method);
}

bool exportLaz(const std::string &filename,
               const std::vector<Eigen::Vector3d> &pointcloud,
               const std::vector<unsigned short> &intensity)
{

    constexpr float scale = 0.0001f; // one tenth of milimeter
    // find max
    Eigen::Vector3d max(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());
    Eigen::Vector3d min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    for (auto &p : pointcloud)
    {
        max.x() = std::max(max.x(), p.x());
        max.y() = std::max(max.y(), p.y());
        max.z() = std::max(max.z(), p.z());

        min.x() = std::min(min.x(), p.x());
        min.y() = std::min(min.y(), p.y());
        min.z() = std::min(min.z(), p.z());
    }

    // create the writer
    laszip_POINTER laszip_writer;
    if (laszip_create(&laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: creating laszip writer\n");
        return false;
    }

    // get a pointer to the header of the writer so we can populate it

    laszip_header *header;

    if (laszip_get_header_pointer(laszip_writer, &header))
    {
        fprintf(stderr, "DLL ERROR: getting header pointer from laszip writer\n");
        return false;
    }

    // populate the header

    header->file_source_ID = 4711;
    header->global_encoding = (1 << 0); // see LAS specification for details
    header->version_major = 1;
    header->version_minor = 2;
    //    header->file_creation_day = 120;
    //    header->file_creation_year = 2013;
    header->point_data_format = 1;
    header->point_data_record_length = 0;
    header->number_of_point_records = pointcloud.size();
    header->number_of_points_by_return[0] = pointcloud.size();
    header->number_of_points_by_return[1] = 0;
    header->point_data_record_length = 28;
    header->x_scale_factor = scale;
    header->y_scale_factor = scale;
    header->z_scale_factor = scale;

    header->max_x = max.x();
    header->min_x = min.x();
    header->max_y = max.y();
    header->min_y = min.y();
    header->max_z = max.z();
    header->min_z = min.z();

    // optional: use the bounding box and the scale factor to create a "good" offset
    // open the writer
    laszip_BOOL compress = (strstr(filename.c_str(), ".laz") != 0);

    if (laszip_open_writer(laszip_writer, filename.c_str(), compress))
    {
        fprintf(stderr, "DLL ERROR: opening laszip writer for '%s'\n", filename.c_str());
        return false;
    }

    fprintf(stderr, "writing file '%s' %scompressed\n", filename.c_str(), (compress ? "" : "un"));

    // get a pointer to the point of the writer that we will populate and write

    laszip_point *point;
    if (laszip_get_point_pointer(laszip_writer, &point))
    {
        fprintf(stderr, "DLL ERROR: getting point pointer from laszip writer\n");
        return false;
    }

    laszip_I64 p_count = 0;
    laszip_F64 coordinates[3];

    for (int i = 0; i < pointcloud.size(); i++)
    {
        point->intensity = intensity[i];

        const auto &p = pointcloud[i];
        p_count++;
        coordinates[0] = p.x();
        coordinates[1] = p.y();
        coordinates[2] = p.z();
        if (laszip_set_coordinates(laszip_writer, coordinates))
        {
            fprintf(stderr, "DLL ERROR: setting coordinates for point %I64d\n", p_count);
            return false;
        }

        // p.SetIntensity(pp.intensity);

        // if (i < intensity.size()) {
        //     point->intensity = intensity[i];
        // }
        // laszip_set_point

        if (laszip_write_point(laszip_writer))
        {
            fprintf(stderr, "DLL ERROR: writing point %I64d\n", p_count);
            return false;
        }
    }

    if (laszip_get_point_count(laszip_writer, &p_count))
    {
        fprintf(stderr, "DLL ERROR: getting point count\n");
        return false;
    }

    fprintf(stderr, "successfully written %I64d points\n", p_count);

    // close the writer

    if (laszip_close_writer(laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: closing laszip writer\n");
        return false;
    }

    // destroy the writer

    if (laszip_destroy(laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: destroying laszip writer\n");
        return false;
    }

    std::cout << "exportLaz DONE" << std::endl;
    return true;
}
