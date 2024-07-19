#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>

#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <ImGuizmo.h>
#include <imgui_internal.h>

#include <GL/glew.h>
#include <GL/freeglut.h>

#include <Eigen/Eigen>

#include <glm.hpp>
#include <gtc/matrix_transform.hpp>

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

#include <gnss.h>
#include <session.h>
#include <pfd_wrapper.hpp>

#include <HDMapping/Version.hpp>

namespace fs = std::filesystem;

static bool show_demo_window = true;
static bool show_another_window = false;
static ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
float rotate_x = 0.0, rotate_y = 0.0;
float translate_x, translate_y = 0.0;
Eigen::Vector3f rotation_center = Eigen::Vector3f::Zero();
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
NDT ndt;
ICP icp;
PoseGraphSLAM pose_graph_slam;
RegistrationPlaneFeature registration_plane_feature;
ObservationPicking observation_picking;
std::vector<Eigen::Vector3d> picked_points;
GNSS gnss;
int all_point_size = 1;
int index_loop_closure_source = 0;
int index_loop_closure_target = 0;
int index_begin = 0;
int index_end = 0;
bool save_subsession = false;

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

Session session;
bool simple_gui = true;
bool session_loaded = false;

void export_result_to_folder(std::string output_folder_name, ObservationPicking &observation_picking, Session &session);
void reshape(int w, int h);

// this functions performs experiment from paper
//@article
//{BEDKOWSKI2023113199,
//      title = {Benchmark of multi-view Terrestrial Laser Scanning Point Cloud data registration algorithms},
//      journal = {Measurement},
//      pages = {113199},
//      year = {2023},
//      issn = {0263-2241},
//      doi = {https://doi.org/10.1016/j.measurement.2023.113199},
//      url = {https://www.sciencedirect.com/science/article/pii/S0263224123007637},
//      author = {Janusz Będkowski},
//      keywords = {TLS, Point cloud, Open-source, Multi-view data registration, LiDAR data metrics, Robust loss function, Tait-bryan angles, Quaternions, Rodrigues’ formula, Lie algebra, Rotation matrix parameterization},
//      abstract = {This study addresses multi-view Terrestrial Laser Scanning Point Cloud data registration methods. Multiple rigid point cloud data registration is mandatory for aligning all scans into a common reference frame and it is still considered a challenge looking from a large-scale surveys point of view. The goal of this work is to support the development of cutting-edge registration methods in geoscience and mobile robotics domains. This work evaluates 3 data sets of total 20 scenes available in the literature. This paper provides a novel open-source framework for multi-view Terrestrial Laser Scanning Point Cloud data registration benchmarks. The goal was to verify experimentally which registration variant can improve the open-source data looking from the quantitative and qualitative points of view. In particular, the following scanners provided measurement data: Z+F TLS Imager 5006i, Z+F TLS Imager 5010C, Leica ScanStation C5, Leica ScanStation C10, Leica P40 and Riegl VZ-400. The benchmark shows an impact of the metric e.g. point to point, point to projection onto a plane, plane to plane etc..., rotation matrix parameterization (Tait-Bryan, quaternion, Rodrigues) and other implementation variations (e.g. multi-view Normal Distributions Transform, Pose Graph SLAM approach) onto the multi-view data registration accuracy and performance. An open-source project is created and it can be used for improving existing data sets reported in the literature, it is the added value of the presented research. The combination of metrics, rotation matrix parameterization and optimization algorithms creates hundreds of possible approaches. It is shown that chosen metric is a dominant factor in data registration. The rotation parameterization and other degrees of freedom of proposed variants are rather negligible compared with chosen metric. Most of the proposed approaches improve registered reference data provided by other researchers. Only for 2 from 20 scenes it was not possible to provide significant improvement. The largest improvements are evident for large-scale scenes. The project is available and maintained at https://github.com/MapsHD/HDMapping.}
// }
void perform_experiment_on_windows(Session &session, ObservationPicking &observation_picking, ICP &icp, NDT &ndt,
                                   RegistrationPlaneFeature &registration_plane_feature, PoseGraphSLAM &pose_graph_slam);
void perform_experiment_on_linux(Session &session, ObservationPicking &observation_picking, ICP &icp, NDT &ndt,
                                 RegistrationPlaneFeature &registration_plane_feature, PoseGraphSLAM &pose_graph_slam);

LaserBeam GetLaserBeam(int x, int y);
Eigen::Vector3d rayIntersection(const LaserBeam &laser_beam, const RegistrationPlaneFeature::Plane &plane);
Eigen::Vector3d GLWidgetGetOGLPos(int x, int y, const ObservationPicking &observation_picking);
bool exportLaz(const std::string &filename, const std::vector<Eigen::Vector3d> &pointcloud, const std::vector<unsigned short> &intensity, double offset_x,
               double offset_y,
               double offset_alt);
double compute_rms(bool initial, Session &session, ObservationPicking &observation_picking);
void reset_poses(Session &session);

void adjustHeader(laszip_header *header, const Eigen::Affine3d &m_pose, const Eigen::Vector3d &offset_in)
{
    Eigen::Vector3d max{header->max_x, header->max_y, header->max_z};
    Eigen::Vector3d min{header->min_x, header->min_y, header->min_z};

    max -= offset_in;
    min -= offset_in;

    Eigen::Vector3d adj_max = m_pose * max + offset_in;
    Eigen::Vector3d adj_min = m_pose * min + offset_in;

    header->max_x = adj_max.x();
    header->max_y = adj_max.y();
    header->max_z = adj_max.z();

    header->min_x = adj_min.x();
    header->min_y = adj_min.y();
    header->min_z = adj_min.z();
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

void save_processed_pc(const fs::path &file_path_in, const fs::path &file_path_put, const Eigen::Affine3d &m_pose, const Eigen::Vector3d &offset, bool override_compressed = false)
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

    if (override_compressed)
    {
        is_compressed = 0;
        std::cout << "compressed : " << is_compressed << std::endl;
    }

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
    static bool calculate_offset = false;

    ImGui::Begin("Single session processing");

    ImGui::Text("This program is second step in MANDEYE process.");
    ImGui::Text("To change centre of rotation press 'ctrl + middle mouse button'");
    ImGui::Text("It refines trajectory e.g with loop closure.");
    ImGui::Text("It refines trajectory with many approaches e.g. Iterative Closest Point, Normal Distributions Transform.");
    ImGui::Text("It exports session as rigid point cloud to single LAZ file).");
    ImGui::Text("LAZ files are the product of MANDEYE process. Open it with Cloud Compare.");

    ImGui::Checkbox("simple_gui", &simple_gui);
    ImGui::SameLine();
    ImGui::Checkbox("is ground truth", &session.is_ground_truth);

    const std::vector<std::string>
        Session_filter = {"Session, json", "*.json"};
    const std::vector<std::string> Resso_filter = {"Resso, reg", "*.reg"};
    const std::vector<std::string> LAS_LAZ_filter = {"LAS file (*.laz)", "*.laz", "LASzip file (*.las)", "*.las", "All files", "*"};

    if (!session_loaded)
    {
        if (ImGui::Button("load session (first step)"))
        {
            static std::shared_ptr<pfd::open_file> open_file;
            std::string input_file_name = "";
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
            const auto t = [&]()
            {
                auto sel = pfd::open_file("Load session", "C:\\", Session_filter).result();
                for (int i = 0; i < sel.size(); i++)
                {
                    input_file_name = sel[i];
                    std::cout << "Session file: '" << input_file_name << "'" << std::endl;
                }
            };
            std::thread t1(t);
            t1.join();

            if (input_file_name.size() > 0)
            {
                session.load(fs::path(input_file_name).string(), is_decimate, bucket_x, bucket_y, bucket_z, calculate_offset);
                session_loaded = true;
            }
        }
        ImGui::SameLine();
    }

    if (ImGui::Button("save session (last step)"))
    {
        std::shared_ptr<pfd::save_file> save_file;
        std::string output_file_name = "";
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
        const auto t = [&]()
        {
            auto sel = pfd::save_file("Save session", "C:\\", Session_filter).result();
            output_file_name = sel;
            std::cout << "Session file to save: '" << output_file_name << "'" << std::endl;
        };
        std::thread t_1(t);
        t_1.join();

        if (output_file_name.size() > 0)
        {
            if (!save_subsession)
            {
                session.save(fs::path(output_file_name).string(), session.point_clouds_container.poses_file_name, session.point_clouds_container.initial_poses_file_name, save_subsession);
                std::cout << "saving result to: " << session.point_clouds_container.poses_file_name << std::endl;
                session.point_clouds_container.save_poses(fs::path(session.point_clouds_container.poses_file_name).string(), save_subsession);
            }
            else
            {
                // std::string poses_file_name;
                // std::string initial_poses_file_name;
                std::shared_ptr<pfd::save_file> save_file1;
                std::string poses_file_name = "";
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file1);
                const auto t = [&]()
                {
                    auto sel = pfd::save_file("poses_file_name", "C:\\", Resso_filter).result();
                    poses_file_name = sel;
                    std::cout << "Resso file to save: '" << poses_file_name << "'" << std::endl;
                };
                std::thread t1(t);
                t1.join();

                std::shared_ptr<pfd::save_file> save_file2;
                std::string initial_poses_file_name = "";
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file2);
                const auto tt = [&]()
                {
                    auto sel = pfd::save_file("initial_poses_file_name", "C:\\", Resso_filter).result();
                    initial_poses_file_name = sel;
                    std::cout << "Resso file to save: '" << initial_poses_file_name << "'" << std::endl;
                };
                std::thread t2(tt);
                t2.join();

                if (poses_file_name.size() > 0 && initial_poses_file_name.size() > 0)
                {
                    session.save(fs::path(output_file_name).string(), poses_file_name, initial_poses_file_name, save_subsession);
                    std::cout << "saving poses to: " << poses_file_name << std::endl;
                    session.point_clouds_container.save_poses(fs::path(poses_file_name).string(), save_subsession);
                    std::cout << "saving initial poses to: " << initial_poses_file_name << std::endl;
                    session.point_clouds_container.save_poses(fs::path(initial_poses_file_name).string(), save_subsession);
                }
            }
        }
    }
    ImGui::SameLine();
    ImGui::Checkbox("save_subsession", &save_subsession);

    if (session_loaded)
    {
        ImGui::Text(std::string("input session file: '" + session.session_file_name + "'").c_str());
    }

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
    if (!simple_gui)
    {
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    }

    if (!simple_gui)
    {
        ImGui::Text("Offset x: %.10f y: %.10f z: %.10f", session.point_clouds_container.offset.x(), session.point_clouds_container.offset.y(), session.point_clouds_container.offset.z());
        ImGui::SameLine();
        if (ImGui::Button("print offset to console"))
        {
            std::cout << "offset:" << std::endl;
            std::cout << std::setprecision(10) << std::endl;
            std::cout << session.point_clouds_container.offset << std::endl;
        }
    }

    ImGui::InputInt("increase for better performance, decrease for rendering more points", &viewer_decmiate_point_cloud);
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
        for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
        {
            session.point_clouds_container.point_clouds[i].point_size = all_point_size;
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

    ImGui::InputFloat3("rotation center", rotation_center.data());
    if (!simple_gui)
    {
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

                session.working_directory = fs::path(input_file_name).parent_path().string();

                if (!session.point_clouds_container.load(session.working_directory.c_str(), input_file_name.c_str(), is_decimate, bucket_x, bucket_y, bucket_z))
                {
                    std::cout << "check input files" << std::endl;
                    return;
                }
                else
                {
                    std::cout << "loaded: " << session.point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
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
                session.point_clouds_container.save_poses(fs::path(output_file_name).string(), false);
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
                session.working_directory = fs::path(input_file_name).parent_path().string();

                if (!session.point_clouds_container.load_eth(session.working_directory.c_str(), input_file_name.c_str(), is_decimate, bucket_x, bucket_y, bucket_z))
                {
                    std::cout << "check input files" << std::endl;
                    return;
                }
                else
                {
                    std::cout << "loaded: " << session.point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
                }
            }
        }
        ImGui::Text("ETH dataset: https://prs.igp.ethz.ch/research/completed_projects/automatic_registration_of_point_clouds.html");

        if (ImGui::Button("load AlignedPointCloud from WHU-TLS (select all *.las or *.laz files in folder 2-AlignedPointCloud)"))
        {
            session.point_clouds_container.point_clouds.clear();
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
                session.working_directory = fs::path(input_file_names[0]).parent_path().string();

                std::cout << "Las/Laz files:" << std::endl;
                for (size_t i = 0; i < input_file_names.size(); i++)
                {
                    std::cout << input_file_names[i] << std::endl;
                }

                if (!session.point_clouds_container.load_whu_tls(input_file_names, is_decimate, bucket_x, bucket_y, bucket_z, calculate_offset))
                {
                    std::cout << "check input files laz/las" << std::endl;
                    // return;
                }
                else
                {
                    std::cout << "loaded: " << session.point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
                }
            }
        }
        ImGui::SameLine();
        ImGui::Checkbox("calculate_offset for WHU-TLS", &calculate_offset);
        ImGui::Text("WHU-TLS dataset: http://3s.whu.edu.cn/ybs/en/benchmark.htm");

        if (ImGui::Button("load 3DTK files (select all *.txt files)"))
        {
            session.point_clouds_container.point_clouds.clear();
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
                session.working_directory = fs::path(input_file_names[0]).parent_path().string();

                std::cout << "txt files:" << std::endl;
                for (size_t i = 0; i < input_file_names.size(); i++)
                {
                    std::cout << input_file_names[i] << std::endl;
                }

                if (!session.point_clouds_container.load_3DTK_tls(input_file_names, is_decimate, bucket_x, bucket_y, bucket_z))
                {
                    std::cout << "check input files" << std::endl;
                    return;
                }
                else
                {
                    std::cout << "loaded: " << session.point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
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

                session.working_directory = fs::path(input_file_name).parent_path().string();

                if (!session.point_clouds_container.update_initial_poses_from_RESSO(session.working_directory.c_str(), input_file_name.c_str()))
                {

                    std::cout << "check input files" << std::endl;
                    return;
                }
                else
                {
                    session.point_clouds_container.initial_poses_file_name = input_file_name;
                    std::cout << "updated: " << session.point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
                }
            }
        }
        ImGui::SameLine();
        ImGui::Text(session.point_clouds_container.initial_poses_file_name.c_str());

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
                session.working_directory = fs::path(input_file_name).parent_path().string();

                if (!session.point_clouds_container.update_poses_from_RESSO(session.working_directory.c_str(), input_file_name.c_str()))
                {
                    std::cout << "check input files" << std::endl;
                    return;
                }
                else
                {
                    std::cout << "updated: " << session.point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
                    session.point_clouds_container.poses_file_name = input_file_name;
                }
            }
        }
        ImGui::SameLine();

        if (ImGui::Button("update poses from RESSO file (inverse)"))
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
                session.working_directory = fs::path(input_file_name).parent_path().string();

                if (!session.point_clouds_container.update_poses_from_RESSO_inverse(session.working_directory.c_str(), input_file_name.c_str()))
                {
                    std::cout << "check input files" << std::endl;
                    return;
                }
                else
                {
                    std::cout << "updated: " << session.point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
                    session.point_clouds_container.poses_file_name = input_file_name;
                }
            }
        }
        ImGui::SameLine();
        ImGui::Text(session.point_clouds_container.poses_file_name.c_str());
    }

    if (ImGui::Button("generate random colors"))
    {
        for (auto &pc : session.point_clouds_container.point_clouds)
        {
            pc.show_color = false;
            pc.render_color[0] = float(rand() % 255) / 255.0f;
            pc.render_color[1] = float(rand() % 255) / 255.0f;
            pc.render_color[2] = float(rand() % 255) / 255.0f;
        }
    }
    ImGui::SameLine();

    if (ImGui::Button("generate random color for all"))
    {
        float color = float(rand() % 255) / 255.0f;
        for (auto &pc : session.point_clouds_container.point_clouds)
        {
            pc.show_color = false;
            pc.render_color[0] = color;
            pc.render_color[1] = color;
            pc.render_color[2] = color;
        }
    }

    if (ImGui::Button("Set initial pose to Identity and update other poses"))
    {
        if (session.point_clouds_container.point_clouds.size() > 0)
        {
            auto m_inv = session.point_clouds_container.point_clouds[0].m_pose.inverse();
            for (int i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
            {
                session.point_clouds_container.point_clouds[i].m_pose = m_inv * session.point_clouds_container.point_clouds[i].m_pose;
            }
        }
    }

    ImGui::Text("-----------------------------------------------------------------------------");
    if (!simple_gui)
    {
        ImGui::Checkbox("Normal Distributions transform", &is_ndt_gui);
        ImGui::Checkbox("Iterative Closest Point", &is_icp_gui);
        ImGui::Checkbox("Plane Features", &is_registration_plane_feature);
        ImGui::Checkbox("Pose Graph SLAM", &is_pose_graph_slam);
        ImGui::Checkbox("Manual Analysis", &is_manual_analisys);
    }
    if (session.point_clouds_container.point_clouds.size() > 0)
    {
        ImGui::Checkbox("Show ground control points gui", &session.ground_control_points.is_imgui);
        ImGui::Checkbox("Manual Pose Graph Loop Closure Mode", &manual_pose_graph_loop_closure_mode);
    }
    ImGui::ColorEdit3("background color", (float *)&clear_color);

    if (manual_pose_graph_loop_closure_mode)
    {
        session.manual_pose_graph_loop_closure.Gui(session.point_clouds_container, 
            index_loop_closure_source, 
            index_loop_closure_target, 
            m_gizmo, 
            gnss,
            session.ground_control_points);

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
        if (!simple_gui)
        {
            if (ImGui::Button("show all"))
            {
                session.point_clouds_container.show_all();
            }
            ImGui::SameLine();
            if (ImGui::Button("hide all"))
            {
                session.point_clouds_container.hide_all();
            }
            ImGui::SameLine();
            if (ImGui::Button("reset poses"))
            {
                reset_poses(session);
                /*for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
                {
                    point_clouds_container.point_clouds[i].m_pose = point_clouds_container.point_clouds[i].m_initial_pose;
                    point_clouds_container.point_clouds[i].pose = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);
                    point_clouds_container.point_clouds[i].gui_translation[0] = (float)point_clouds_container.point_clouds[i].pose.px;
                    point_clouds_container.point_clouds[i].gui_translation[1] = (float)point_clouds_container.point_clouds[i].pose.py;
                    point_clouds_container.point_clouds[i].gui_translation[2] = (float)point_clouds_container.point_clouds[i].pose.pz;
                    point_clouds_container.point_clouds[i].gui_rotation[0] = (float)rad2deg(point_clouds_container.point_clouds[i].pose.om);
                    point_clouds_container.point_clouds[i].gui_rotation[1] = (float)rad2deg(point_clouds_container.point_clouds[i].pose.fi);
                    point_clouds_container.point_clouds[i].gui_rotation[2] = (float)rad2deg(point_clouds_container.point_clouds[i].pose.ka);
                }*/
            }

            ImGui::Checkbox("show_with_initial_pose", &session.point_clouds_container.show_with_initial_pose);
            ImGui::SameLine();
            ImGui::Checkbox("manipulate_only_marked_gizmo (false: move also succesive nodes)", &manipulate_only_marked_gizmo);

            int idx_begin = index_begin;
            int idx_end = index_end;
            ImGui::InputInt("index_show_from", &index_begin);
            if (index_begin < 0)
            {
                index_begin = 0;
            }
            if (index_begin >= session.point_clouds_container.point_clouds.size() - 1)
            {
                index_begin = session.point_clouds_container.point_clouds.size() - 1;
            }
            ImGui::InputInt("index_show_to", &index_end);
            if (index_end < 0)
            {
                index_end = 0;
            }
            if (index_end >= session.point_clouds_container.point_clouds.size() - 1)
            {
                index_end = session.point_clouds_container.point_clouds.size() - 1;
            }
            if (ImGui::Button("Show selected"))
            {
                session.point_clouds_container.show_all_from_range(index_begin, index_end);
            }
            // if (ImGui::Button("mark all pcs from range <index_begin, index_end>"))
            //{
            //     session.point_clouds_container.hide_all();
            //     session.point_clouds_container.show_all_from_range(index_begin, index_end);
            // }

            for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
            {
                ImGui::Separator();
                ImGui::Checkbox(session.point_clouds_container.point_clouds[i].file_name.c_str(), &session.point_clouds_container.point_clouds[i].visible);
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
                ImGui::Checkbox(std::string("#" + std::to_string(i) + " choose_geo").c_str(), &session.point_clouds_container.point_clouds[i].choosing_geo);

                if (session.point_clouds_container.point_clouds[i].choosing_geo)
                {
                    for (int gp = 0; gp < session.point_clouds_container.point_clouds[i].available_geo_points.size(); gp++)
                    {
                        ImGui::Checkbox(std::string("#" + std::to_string(i) + " " + std::to_string(gp) + "[" +
                                                    session.point_clouds_container.point_clouds[i].available_geo_points[gp].name + "]")
                                            .c_str(),
                                        &session.point_clouds_container.point_clouds[i].available_geo_points[gp].choosen);
                    }
                }
            }
            ImGui::Separator();
            int total_number_of_points = 0;
            for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
            {
                total_number_of_points += session.point_clouds_container.point_clouds[i].points_local.size();
            }
            std::string point_size_message = "total number of points: " + std::to_string(total_number_of_points);
            ImGui::Text(point_size_message.c_str());

            ImGui::Separator();
            ImGui::Separator();
        }
        // gnss.offset_x, gnss.offset_y, gnss.offset_alt
        ImGui::Text("Set offsets x y z to export point cloud in global coordinate system (each local coordinate of the point += offset)");
        ImGui::InputDouble("offset_x", &gnss.offset_x);
        ImGui::InputDouble("offset_y", &gnss.offset_y);
        ImGui::InputDouble("offset_z", &gnss.offset_alt);

        if (ImGui::Button("save all marked scans to laz (as one global scan)"))
        {
            std::shared_ptr<pfd::save_file> save_file;
            std::string output_file_name = "";
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
            const auto t = [&]()
            {
                auto sel = pfd::save_file("Save las or laz file", "C:\\", LAS_LAZ_filter).result();
                output_file_name = sel;
                std::cout << "las or laz file to save: '" << output_file_name << "'" << std::endl;
            };
            std::thread t1(t);
            t1.join();

            if (output_file_name.size() > 0)
            {
                std::vector<Eigen::Vector3d> pointcloud;
                std::vector<unsigned short> intensity;

                for (auto &p : session.point_clouds_container.point_clouds)
                {
                    if (p.visible)
                    {
                        for (int i = 0; i < p.points_local.size(); i++)
                        {
                            const auto &pp = p.points_local[i];
                            Eigen::Vector3d vp;
                            vp = p.m_pose * pp + session.point_clouds_container.offset;

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
                if (!exportLaz(output_file_name, pointcloud, intensity, gnss.offset_x, gnss.offset_y, gnss.offset_alt))
                {
                    std::cout << "problem with saving file: " << output_file_name << std::endl;
                }
            }
        }
        ImGui::SameLine();
        ImGui::Text("To export in full resolution, close the program and open again, unmark 'simple_gui', unmark 'decimate during load'");
        if (!simple_gui)
        {

            if (ImGui::Button("save all marked scans to laz (as separate global scans)"))
            {
                for (auto &p : session.point_clouds_container.point_clouds)
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
                        save_processed_pc(file_path_in, file_path_put, p.m_pose, session.point_clouds_container.offset, false);
                        std::cout << "processed_pc finished" << std::endl;
                    }
                }
            }
            if (ImGui::Button("save all marked scans to las (as separate global scans)"))
            {
                for (auto &p : session.point_clouds_container.point_clouds)
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
                        file_path_put /= (file_path_in.stem().string() + "_processed.las" /*+ file_path_in.extension().string()*/);
                        std::cout << "file_in: " << file_path_in << std::endl;
                        std::cout << "file_out: " << file_path_put << std::endl;

                        std::cout << "start save_processed_pc" << std::endl;
                        save_processed_pc(file_path_in, file_path_put, p.m_pose, session.point_clouds_container.offset, true);
                        std::cout << "processed_pc finished" << std::endl;
                    }
                }
            }

            static bool is_trajectory_export_downsampling = false;
            ImGui::Checkbox("is_trajectory_export_downsampling", &is_trajectory_export_downsampling);

            static float not_curve_consecutive_distance_meters = 1.0f;
            static float curve_consecutive_distance_meters = 0.05f;

            if (is_trajectory_export_downsampling)
            {
                ImGui::InputFloat("curve_consecutive_distance_meters", &curve_consecutive_distance_meters);
                ImGui::InputFloat("not_curve_consecutive_distance_meters", &not_curve_consecutive_distance_meters);
            }

            if (ImGui::Button("save all marked trajectories to laz (as one global scan)"))
            {
                std::shared_ptr<pfd::save_file> save_file;
                std::string output_file_name = "";
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
                const auto t = [&]()
                {
                    auto sel = pfd::save_file("Save las or laz file", "C:\\", LAS_LAZ_filter).result();
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
                    float consecutive_distance = 0;

                    for (auto &p : session.point_clouds_container.point_clouds)
                    {
                        if (p.visible)
                        {
                            for (int i = 0; i < p.local_trajectory.size(); i++)
                            {
                                const auto &pp = p.local_trajectory[i].m_pose.translation();
                                Eigen::Vector3d vp;
                                vp = p.m_pose * pp + session.point_clouds_container.offset;

                                if (i > 0)
                                {
                                    double dist = (p.local_trajectory[i].m_pose.translation() - p.local_trajectory[i - 1].m_pose.translation()).norm();
                                    consecutive_distance += dist;
                                }

                                bool is_curve = false;

                                if (i > 100 && i < p.local_trajectory.size() - 100)
                                {
                                    Eigen::Vector3d position_prev = p.local_trajectory[i - 100].m_pose.translation();
                                    Eigen::Vector3d position_curr = p.local_trajectory[i].m_pose.translation();
                                    Eigen::Vector3d position_next = p.local_trajectory[i + 100].m_pose.translation();

                                    Eigen::Vector3d v1 = position_curr - position_prev;
                                    Eigen::Vector3d v2 = position_next - position_curr;

                                    if (v1.norm() > 0 && v2.norm() > 0)
                                    {
                                        double angle_deg = fabs(acos(v1.dot(v2) / (v1.norm() * v2.norm())) * 180.0 / M_PI);

                                        if (angle_deg > 10.0)
                                        {
                                            is_curve = true;
                                        }
                                    }
                                }
                                double tol = not_curve_consecutive_distance_meters;

                                if (is_curve)
                                {
                                    tol = curve_consecutive_distance_meters;
                                }

                                if (!is_trajectory_export_downsampling)
                                {
                                    pointcloud.push_back(vp);
                                    intensity.push_back(0);
                                }
                                else
                                {
                                    if (consecutive_distance >= tol)
                                    {
                                        consecutive_distance = 0;
                                        pointcloud.push_back(vp);
                                        intensity.push_back(0);
                                    }
                                }
                            }
                        }
                    }
                    if (!exportLaz(output_file_name, pointcloud, intensity, gnss.offset_x, gnss.offset_y, gnss.offset_alt))
                    {
                        std::cout << "problem with saving file: " << output_file_name << std::endl;
                    }
                }
            }

            if (ImGui::Button("save gnss data to laz file"))
            {
                std::shared_ptr<pfd::save_file> save_file;
                std::string output_file_name = "";
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
                const auto t = [&]()
                {
                    auto sel = pfd::save_file("Save las or laz file", "C:\\", LAS_LAZ_filter).result();
                    output_file_name = sel;
                    std::cout << "las or laz file to save: '" << output_file_name << "'" << std::endl;
                };
                std::thread t1(t);
                t1.join();

                if (output_file_name.size() > 0)
                {
                    gnss.save_to_laz(output_file_name);
                }
            }

            if (ImGui::Button("save scale board for all marked trajectories to laz (as one global scan - dec 0.1)"))
            {
                std::shared_ptr<pfd::save_file> save_file;
                std::string output_file_name = "";
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
                const auto t = [&]()
                {
                    auto sel = pfd::save_file("Save las or laz file", "C:\\", LAS_LAZ_filter).result();
                    output_file_name = sel;
                    std::cout << "las or laz file to save: '" << output_file_name << "'" << std::endl;
                };
                std::thread t1(t);
                t1.join();

                if (output_file_name.size() > 0)
                {
                    std::vector<Eigen::Vector3d> pointcloud;
                    std::vector<unsigned short> intensity;

                    float min_x = 1000000000.0;
                    float max_x = -1000000000.0;
                    float min_y = 1000000000.0;
                    float max_y = -1000000000.0;
                    float min_z = 1000000000.0;
                    float max_z = -1000000000.0;

                    for (auto &p : session.point_clouds_container.point_clouds)
                    {
                        if (p.visible)
                        {

                            for (int i = 0; i < p.local_trajectory.size(); i++)
                            {
                                const auto &pp = p.local_trajectory[i].m_pose.translation();
                                Eigen::Vector3d vp;
                                vp = p.m_pose * pp + session.point_clouds_container.offset;

                                // pointcloud.push_back(vp);
                                // intensity.push_back(0);
                                if (vp.x() < min_x)
                                {
                                    min_x = vp.x();
                                }
                                if (vp.x() > max_x)
                                {
                                    max_x = vp.x();
                                }
                                if (vp.y() < min_y)
                                {
                                    min_y = vp.y();
                                }
                                if (vp.y() > max_y)
                                {
                                    max_y = vp.y();
                                }
                                if (vp.z() < min_z)
                                {
                                    min_z = vp.z();
                                }
                                if (vp.z() > max_z)
                                {
                                    max_z = vp.z();
                                }
                            }
                        }
                    }

                    for (float x = min_x - 100.0; x <= max_x + 100.0; x += 0.1)
                    {
                        for (float y = min_y - 100.0; y <= max_y + 100.0; y += 0.001)
                        {
                            Eigen::Vector3d vp(x, y, (max_z + min_z) / 2.0);
                            pointcloud.push_back(vp);
                            intensity.push_back(0);
                        }
                    }

                    for (float y = min_y - 100.0; y <= max_y + 100.0; y += 0.1)
                    {
                        for (float x = min_x - 100.0; x <= max_x + 100.0; x += 0.001)
                        {
                            Eigen::Vector3d vp(x, y, (max_z + min_z) / 2.0);
                            pointcloud.push_back(vp);
                            intensity.push_back(0);
                        }
                    }

                    if (!exportLaz(output_file_name, pointcloud, intensity, gnss.offset_x, gnss.offset_y, gnss.offset_alt))
                    {
                        std::cout << "problem with saving file: " << output_file_name << std::endl;
                    }
                }
            }
            if (ImGui::Button("save scale board for all marked trajectories to laz (as one global scan - dec 1.0)"))
            {
                std::shared_ptr<pfd::save_file> save_file;
                std::string output_file_name = "";
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
                const auto t = [&]()
                {
                    auto sel = pfd::save_file("Save las or laz file", "C:\\", LAS_LAZ_filter).result();
                    output_file_name = sel;
                    std::cout << "las or laz file to save: '" << output_file_name << "'" << std::endl;
                };
                std::thread t1(t);
                t1.join();

                if (output_file_name.size() > 0)
                {
                    std::vector<Eigen::Vector3d> pointcloud;
                    std::vector<unsigned short> intensity;

                    float min_x = 1000000000.0;
                    float max_x = -1000000000.0;
                    float min_y = 1000000000.0;
                    float max_y = -1000000000.0;
                    float min_z = 1000000000.0;
                    float max_z = -1000000000.0;

                    for (auto &p : session.point_clouds_container.point_clouds)
                    {
                        if (p.visible)
                        {

                            for (int i = 0; i < p.local_trajectory.size(); i++)
                            {
                                const auto &pp = p.local_trajectory[i].m_pose.translation();
                                Eigen::Vector3d vp;
                                vp = p.m_pose * pp + session.point_clouds_container.offset;

                                // pointcloud.push_back(vp);
                                // intensity.push_back(0);
                                if (vp.x() < min_x)
                                {
                                    min_x = vp.x();
                                }
                                if (vp.x() > max_x)
                                {
                                    max_x = vp.x();
                                }
                                if (vp.y() < min_y)
                                {
                                    min_y = vp.y();
                                }
                                if (vp.y() > max_y)
                                {
                                    max_y = vp.y();
                                }
                                if (vp.z() < min_z)
                                {
                                    min_z = vp.z();
                                }
                                if (vp.z() > max_z)
                                {
                                    max_z = vp.z();
                                }
                            }
                        }
                    }

                    for (float x = min_x - 100.0; x <= max_x + 100.0; x += 1.0)
                    {
                        for (float y = min_y - 100.0; y <= max_y + 100.0; y += 0.001)
                        {
                            Eigen::Vector3d vp(x, y, (max_z + min_z) / 2.0);
                            pointcloud.push_back(vp);
                            intensity.push_back(0);
                        }
                    }

                    for (float y = min_y - 100.0; y <= max_y + 100.0; y += 1.0)
                    {
                        for (float x = min_x - 100.0; x <= max_x + 100.0; x += 0.001)
                        {
                            Eigen::Vector3d vp(x, y, (max_z + min_z) / 2.0);
                            pointcloud.push_back(vp);
                            intensity.push_back(0);
                        }
                    }

                    if (!exportLaz(output_file_name, pointcloud, intensity, gnss.offset_x, gnss.offset_y, gnss.offset_alt))
                    {
                        std::cout << "problem with saving file: " << output_file_name << std::endl;
                    }
                }
            }
            if (ImGui::Button("save scale board for all marked trajectories to laz (as one global scan - dec 10.0)"))
            {
                std::shared_ptr<pfd::save_file> save_file;
                std::string output_file_name = "";
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
                const auto t = [&]()
                {
                    auto sel = pfd::save_file("Save las or laz file", "C:\\", LAS_LAZ_filter).result();
                    output_file_name = sel;
                    std::cout << "las or laz file to save: '" << output_file_name << "'" << std::endl;
                };
                std::thread t1(t);
                t1.join();

                if (output_file_name.size() > 0)
                {
                    std::vector<Eigen::Vector3d> pointcloud;
                    std::vector<unsigned short> intensity;

                    float min_x = 1000000000.0;
                    float max_x = -1000000000.0;
                    float min_y = 1000000000.0;
                    float max_y = -1000000000.0;
                    float min_z = 1000000000.0;
                    float max_z = -1000000000.0;

                    for (auto &p : session.point_clouds_container.point_clouds)
                    {
                        if (p.visible)
                        {

                            for (int i = 0; i < p.local_trajectory.size(); i++)
                            {
                                const auto &pp = p.local_trajectory[i].m_pose.translation();
                                Eigen::Vector3d vp;
                                vp = p.m_pose * pp + session.point_clouds_container.offset;

                                // pointcloud.push_back(vp);
                                // intensity.push_back(0);
                                if (vp.x() < min_x)
                                {
                                    min_x = vp.x();
                                }
                                if (vp.x() > max_x)
                                {
                                    max_x = vp.x();
                                }
                                if (vp.y() < min_y)
                                {
                                    min_y = vp.y();
                                }
                                if (vp.y() > max_y)
                                {
                                    max_y = vp.y();
                                }
                                if (vp.z() < min_z)
                                {
                                    min_z = vp.z();
                                }
                                if (vp.z() > max_z)
                                {
                                    max_z = vp.z();
                                }
                            }
                        }
                    }

                    for (float x = min_x - 100.0; x <= max_x + 100.0; x += 10.0)
                    {
                        for (float y = min_y - 100.0; y <= max_y + 100.0; y += 0.001)
                        {
                            Eigen::Vector3d vp(x, y, (max_z + min_z) / 2.0);
                            pointcloud.push_back(vp);
                            intensity.push_back(0);
                        }
                    }

                    for (float y = min_y - 100.0; y <= max_y + 100.0; y += 10.0)
                    {
                        for (float x = min_x - 100.0; x <= max_x + 100.0; x += 0.001)
                        {
                            Eigen::Vector3d vp(x, y, (max_z + min_z) / 2.0);
                            pointcloud.push_back(vp);
                            intensity.push_back(0);
                        }
                    }

                    if (!exportLaz(output_file_name, pointcloud, intensity, gnss.offset_x, gnss.offset_y, gnss.offset_alt))
                    {
                        std::cout << "problem with saving file: " << output_file_name << std::endl;
                    }
                }
            }
            if (ImGui::Button("save scale board 10km x 10km to laz (10m)"))
            {
                std::shared_ptr<pfd::save_file> save_file;
                std::string output_file_name = "";
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
                const auto t = [&]()
                {
                    auto sel = pfd::save_file("Save las or laz file", "C:\\", LAS_LAZ_filter).result();
                    output_file_name = sel;
                    std::cout << "las or laz file to save: '" << output_file_name << "'" << std::endl;
                };
                std::thread t1(t);
                t1.join();

                if (output_file_name.size() > 0)
                {
                    std::vector<Eigen::Vector3d> pointcloud;
                    std::vector<unsigned short> intensity;

                    for (float x = -5000.0; x <= 5000.0; x += 10.0)
                    {
                        for (float y = -5000.0; y <= 5000.0; y += 0.2f)
                        {
                            Eigen::Vector3d vp(x, y, 0.0);
                            pointcloud.push_back(vp);
                            intensity.push_back(0);
                        }
                    }

                    for (float y = -5000.0; y <= 5000.0; y += 10.0)
                    {
                        for (float x = -5000.0; x <= 5000.0; x += 0.2f)
                        {
                            Eigen::Vector3d vp(x, y, 0.0);
                            pointcloud.push_back(vp);
                            intensity.push_back(0);
                        }
                    }

                    if (!exportLaz(output_file_name, pointcloud, intensity, gnss.offset_x, gnss.offset_y, gnss.offset_alt))
                    {
                        std::cout << "problem with saving file: " << output_file_name << std::endl;
                    }
                }
            }
            if (ImGui::Button("save scale board 10km x 10km to laz (100m)"))
            {
                std::shared_ptr<pfd::save_file> save_file;
                std::string output_file_name = "";
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
                const auto t = [&]()
                {
                    auto sel = pfd::save_file("Save las or laz file", "C:\\", LAS_LAZ_filter).result();
                    output_file_name = sel;
                    std::cout << "las or laz file to save: '" << output_file_name << "'" << std::endl;
                };
                std::thread t1(t);
                t1.join();

                if (output_file_name.size() > 0)
                {
                    std::vector<Eigen::Vector3d> pointcloud;
                    std::vector<unsigned short> intensity;

                    for (float x = -5000.0; x <= 5000.0; x += 100.0)
                    {
                        for (float y = -5000.0; y <= 5000.0; y += 0.2f)
                        {
                            Eigen::Vector3d vp(x, y, 0.0);
                            pointcloud.push_back(vp);
                            intensity.push_back(0);
                        }
                    }

                    for (float y = -5000.0; y <= 5000.0; y += 100.0)
                    {
                        for (float x = -5000.0; x <= 5000.0; x += 0.2f)
                        {
                            Eigen::Vector3d vp(x, y, 0.0);
                            pointcloud.push_back(vp);
                            intensity.push_back(0);
                        }
                    }

                    if (!exportLaz(output_file_name, pointcloud, intensity, gnss.offset_x, gnss.offset_y, gnss.offset_alt))
                    {
                        std::cout << "problem with saving file: " << output_file_name << std::endl;
                    }
                }
            }
            if (ImGui::Button("save scale board 10km x 10km to laz (1000m)"))
            {
                std::shared_ptr<pfd::save_file> save_file;
                std::string output_file_name = "";
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
                const auto t = [&]()
                {
                    auto sel = pfd::save_file("Save las or laz file", "C:\\", LAS_LAZ_filter).result();
                    output_file_name = sel;
                    std::cout << "las or laz file to save: '" << output_file_name << "'" << std::endl;
                };
                std::thread t1(t);
                t1.join();

                if (output_file_name.size() > 0)
                {
                    std::vector<Eigen::Vector3d> pointcloud;
                    std::vector<unsigned short> intensity;

                    for (float x = -5000.0; x <= 5000.0; x += 1000.0)
                    {
                        for (float y = -5000.0; y <= 5000.0; y += 0.2f)
                        {
                            Eigen::Vector3d vp(x, y, 0.0);
                            pointcloud.push_back(vp);
                            intensity.push_back(0);
                        }
                    }

                    for (float y = -5000.0; y <= 5000.0; y += 1000.0)
                    {
                        for (float x = -5000.0; x <= 5000.0; x += 0.2f)
                        {
                            Eigen::Vector3d vp(x, y, 0.0);
                            pointcloud.push_back(vp);
                            intensity.push_back(0);
                        }
                    }

                    if (!exportLaz(output_file_name, pointcloud, intensity, gnss.offset_x, gnss.offset_y, gnss.offset_alt))
                    {
                        std::cout << "problem with saving file: " << output_file_name << std::endl;
                    }
                }
            }
            if (ImGui::Button("save all marked trajectories to csv (timestampLidar,x,y,z,r00,r01,r02,r10,r11,r12,r20,r21,r22)"))
            {
                std::shared_ptr<pfd::save_file> save_file;
                std::string output_file_name = "";
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
                const auto t = [&]()
                {
                    auto sel = pfd::save_file("Save csv file", "C:\\", {"csv file", "*.csv"}).result();
                    output_file_name = sel;
                    std::cout << "csv file to save: '" << output_file_name << "'" << std::endl;
                };
                std::thread t1(t);
                t1.join();

                if (output_file_name.size() > 0)
                {
                    std::ofstream outfile(output_file_name);
                    if (outfile.good())
                    {
                        float consecutive_distance = 0;

                        for (auto &p : session.point_clouds_container.point_clouds)
                        {
                            if (p.visible)
                            {
                                for (int i = 0; i < p.local_trajectory.size(); i++)
                                {
                                    const auto &m = p.local_trajectory[i].m_pose;
                                    Eigen::Affine3d pose = p.m_pose * m;
                                    pose.translation() += session.point_clouds_container.offset;

                                    if (i > 0)
                                    {
                                        double dist = (p.local_trajectory[i].m_pose.translation() - p.local_trajectory[i - 1].m_pose.translation()).norm();
                                        consecutive_distance += dist;
                                    }

                                    bool is_curve = false;

                                    if (i > 100 && i < p.local_trajectory.size() - 100)
                                    {
                                        Eigen::Vector3d position_prev = p.local_trajectory[i - 100].m_pose.translation();
                                        Eigen::Vector3d position_curr = p.local_trajectory[i].m_pose.translation();
                                        Eigen::Vector3d position_next = p.local_trajectory[i + 100].m_pose.translation();

                                        Eigen::Vector3d v1 = position_curr - position_prev;
                                        Eigen::Vector3d v2 = position_next - position_curr;

                                        if (v1.norm() > 0 && v2.norm() > 0)
                                        {
                                            double angle_deg = fabs(acos(v1.dot(v2) / (v1.norm() * v2.norm())) * 180.0 / M_PI);

                                            if (angle_deg > 10.0)
                                            {
                                                is_curve = true;
                                            }
                                        }
                                    }
                                    double tol = not_curve_consecutive_distance_meters;

                                    if (is_curve)
                                    {
                                        tol = curve_consecutive_distance_meters;
                                    }

                                    if (!is_trajectory_export_downsampling)
                                    {
                                        outfile << std::setprecision(20);
                                        outfile << p.local_trajectory[i].timestamps.first << "," << pose(0, 3) << "," << pose(1, 3) << "," << pose(2, 3) << "," << pose(0, 0) << "," << pose(0, 1) << "," << pose(0, 2) << "," << pose(1, 0) << "," << pose(1, 1) << "," << pose(1, 2) << "," << pose(2, 0) << "," << pose(2, 1) << "," << pose(2, 2) << std::endl;
                                    }
                                    else
                                    {
                                        if (consecutive_distance >= tol)
                                        {
                                            consecutive_distance = 0;
                                            outfile << std::setprecision(20);
                                            outfile << p.local_trajectory[i].timestamps.first << "," << pose(0, 3) << "," << pose(1, 3) << "," << pose(2, 3) << "," << pose(0, 0) << "," << pose(0, 1) << "," << pose(0, 2) << "," << pose(1, 0) << "," << pose(1, 1) << "," << pose(1, 2) << "," << pose(2, 0) << "," << pose(2, 1) << "," << pose(2, 2) << std::endl;
                                        }
                                    }
                                }
                            }
                        }
                        outfile.close();
                    }
                }
            }
            if (ImGui::Button("save all marked trajectories to csv (timestampUnix,x,y,z,r00,r01,r02,r10,r11,r12,r20,r21,r22)"))
            {
                std::shared_ptr<pfd::save_file> save_file;
                std::string output_file_name = "";
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
                const auto t = [&]()
                {
                    auto sel = pfd::save_file("Save csv file", "C:\\", {"csv file", "*.csv"}).result();
                    output_file_name = sel;
                    std::cout << "csv file to save: '" << output_file_name << "'" << std::endl;
                };
                std::thread t1(t);
                t1.join();

                if (output_file_name.size() > 0)
                {
                    std::ofstream outfile(output_file_name);
                    if (outfile.good())
                    {
                        float consecutive_distance = 0;

                        for (auto &p : session.point_clouds_container.point_clouds)
                        {
                            if (p.visible)
                            {

                                for (int i = 0; i < p.local_trajectory.size(); i++)
                                {
                                    const auto &m = p.local_trajectory[i].m_pose;
                                    Eigen::Affine3d pose = p.m_pose * m;
                                    pose.translation() += session.point_clouds_container.offset;

                                    if (i > 0)
                                    {
                                        double dist = (p.local_trajectory[i].m_pose.translation() - p.local_trajectory[i - 1].m_pose.translation()).norm();
                                        consecutive_distance += dist;
                                    }

                                    bool is_curve = false;

                                    if (i > 100 && i < p.local_trajectory.size() - 100)
                                    {
                                        Eigen::Vector3d position_prev = p.local_trajectory[i - 100].m_pose.translation();
                                        Eigen::Vector3d position_curr = p.local_trajectory[i].m_pose.translation();
                                        Eigen::Vector3d position_next = p.local_trajectory[i + 100].m_pose.translation();

                                        Eigen::Vector3d v1 = position_curr - position_prev;
                                        Eigen::Vector3d v2 = position_next - position_curr;

                                        if (v1.norm() > 0 && v2.norm() > 0)
                                        {
                                            double angle_deg = fabs(acos(v1.dot(v2) / (v1.norm() * v2.norm())) * 180.0 / M_PI);

                                            if (angle_deg > 10.0)
                                            {
                                                is_curve = true;
                                            }
                                        }
                                    }
                                    double tol = not_curve_consecutive_distance_meters;

                                    if (is_curve)
                                    {
                                        tol = curve_consecutive_distance_meters;
                                    }

                                    if (!is_trajectory_export_downsampling)
                                    {
                                        outfile << std::setprecision(20);
                                        outfile << p.local_trajectory[i].timestamps.second << "," << pose(0, 3) << "," << pose(1, 3) << "," << pose(2, 3) << "," << pose(0, 0) << "," << pose(0, 1) << "," << pose(0, 2) << "," << pose(1, 0) << "," << pose(1, 1) << "," << pose(1, 2) << "," << pose(2, 0) << "," << pose(2, 1) << "," << pose(2, 2) << std::endl;
                                    }
                                    else
                                    {
                                        if (consecutive_distance >= tol)
                                        {
                                            consecutive_distance = 0;
                                            outfile << std::setprecision(20);
                                            outfile << p.local_trajectory[i].timestamps.second << "," << pose(0, 3) << "," << pose(1, 3) << "," << pose(2, 3) << "," << pose(0, 0) << "," << pose(0, 1) << "," << pose(0, 2) << "," << pose(1, 0) << "," << pose(1, 1) << "," << pose(1, 2) << "," << pose(2, 0) << "," << pose(2, 1) << "," << pose(2, 2) << std::endl;
                                        }
                                    }
                                }
                            }
                        }
                        outfile.close();
                    }
                }
            }
            if (ImGui::Button("save all marked trajectories to csv (timestampLidar,timestampUnix,x,y,z,r00,r01,r02,r10,r11,r12,r20,r21,r22)"))
            {
                std::shared_ptr<pfd::save_file> save_file;
                std::string output_file_name = "";
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
                const auto t = [&]()
                {
                    auto sel = pfd::save_file("Save csv file", "C:\\", {"csv file", "*.csv"}).result();
                    output_file_name = sel;
                    std::cout << "csv file to save: '" << output_file_name << "'" << std::endl;
                };
                std::thread t1(t);
                t1.join();

                if (output_file_name.size() > 0)
                {
                    std::ofstream outfile(output_file_name);
                    if (outfile.good())
                    {
                        float consecutive_distance = 0;

                        for (auto &p : session.point_clouds_container.point_clouds)
                        {
                            if (p.visible)
                            {

                                for (int i = 0; i < p.local_trajectory.size(); i++)
                                {
                                    const auto &m = p.local_trajectory[i].m_pose;
                                    Eigen::Affine3d pose = p.m_pose * m;
                                    pose.translation() += session.point_clouds_container.offset;

                                    if (i > 0)
                                    {
                                        double dist = (p.local_trajectory[i].m_pose.translation() - p.local_trajectory[i - 1].m_pose.translation()).norm();
                                        consecutive_distance += dist;
                                    }

                                    bool is_curve = false;

                                    if (i > 100 && i < p.local_trajectory.size() - 100)
                                    {
                                        Eigen::Vector3d position_prev = p.local_trajectory[i - 100].m_pose.translation();
                                        Eigen::Vector3d position_curr = p.local_trajectory[i].m_pose.translation();
                                        Eigen::Vector3d position_next = p.local_trajectory[i + 100].m_pose.translation();

                                        Eigen::Vector3d v1 = position_curr - position_prev;
                                        Eigen::Vector3d v2 = position_next - position_curr;

                                        if (v1.norm() > 0 && v2.norm() > 0)
                                        {
                                            double angle_deg = fabs(acos(v1.dot(v2) / (v1.norm() * v2.norm())) * 180.0 / M_PI);

                                            if (angle_deg > 10.0)
                                            {
                                                is_curve = true;
                                            }
                                        }
                                    }

                                    double tol = not_curve_consecutive_distance_meters;

                                    if (is_curve)
                                    {
                                        tol = curve_consecutive_distance_meters;
                                    }

                                    if (!is_trajectory_export_downsampling)
                                    {
                                        outfile << std::setprecision(20);
                                        outfile << p.local_trajectory[i].timestamps.first << "," << p.local_trajectory[i].timestamps.second << "," << pose(0, 3) << "," << pose(1, 3) << "," << pose(2, 3) << "," << pose(0, 0) << "," << pose(0, 1) << "," << pose(0, 2) << "," << pose(1, 0) << "," << pose(1, 1) << "," << pose(1, 2) << "," << pose(2, 0) << "," << pose(2, 1) << "," << pose(2, 2) << std::endl;
                                    }
                                    else
                                    {
                                        if (consecutive_distance >= tol)
                                        {
                                            consecutive_distance = 0;
                                            outfile << std::setprecision(20);
                                            outfile << p.local_trajectory[i].timestamps.first << "," << p.local_trajectory[i].timestamps.second << "," << pose(0, 3) << "," << pose(1, 3) << "," << pose(2, 3) << "," << pose(0, 0) << "," << pose(0, 1) << "," << pose(0, 2) << "," << pose(1, 0) << "," << pose(1, 1) << "," << pose(1, 2) << "," << pose(2, 0) << "," << pose(2, 1) << "," << pose(2, 2) << std::endl;
                                        }
                                    }
                                }
                            }
                        }
                        outfile.close();
                    }
                }
            }
            if (ImGui::Button("save all marked trajectories to csv (timestampLidar,x,y,z,qx,qy,qz,qw)"))
            {
                std::shared_ptr<pfd::save_file> save_file;
                std::string output_file_name = "";
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
                const auto t = [&]()
                {
                    auto sel = pfd::save_file("csv file", "C:\\").result();
                    output_file_name = sel;
                    std::cout << "csv file to save: '" << output_file_name << "'" << std::endl;
                };
                std::thread t1(t);
                t1.join();

                if (output_file_name.size() > 0)
                {
                    std::ofstream outfile(output_file_name);
                    if (outfile.good())
                    {
                        float consecutive_distance = 0;

                        for (auto &p : session.point_clouds_container.point_clouds)
                        {
                            if (p.visible)
                            {

                                for (int i = 0; i < p.local_trajectory.size(); i++)
                                {
                                    const auto &m = p.local_trajectory[i].m_pose;
                                    Eigen::Affine3d pose = p.m_pose * m;
                                    pose.translation() += session.point_clouds_container.offset;
                                    Eigen::Quaterniond q(pose.rotation());

                                    if (i > 0)
                                    {
                                        double dist = (p.local_trajectory[i].m_pose.translation() - p.local_trajectory[i - 1].m_pose.translation()).norm();
                                        consecutive_distance += dist;
                                    }

                                    bool is_curve = false;

                                    if (i > 100 && i < p.local_trajectory.size() - 100)
                                    {
                                        Eigen::Vector3d position_prev = p.local_trajectory[i - 100].m_pose.translation();
                                        Eigen::Vector3d position_curr = p.local_trajectory[i].m_pose.translation();
                                        Eigen::Vector3d position_next = p.local_trajectory[i + 100].m_pose.translation();

                                        Eigen::Vector3d v1 = position_curr - position_prev;
                                        Eigen::Vector3d v2 = position_next - position_curr;

                                        if (v1.norm() > 0 && v2.norm() > 0)
                                        {
                                            double angle_deg = fabs(acos(v1.dot(v2) / (v1.norm() * v2.norm())) * 180.0 / M_PI);

                                            if (angle_deg > 10.0)
                                            {
                                                is_curve = true;
                                            }
                                        }
                                    }
                                    double tol = not_curve_consecutive_distance_meters;

                                    if (is_curve)
                                    {
                                        tol = curve_consecutive_distance_meters;
                                    }

                                    if (!is_trajectory_export_downsampling)
                                    {
                                        outfile << std::setprecision(20);
                                        outfile << p.local_trajectory[i].timestamps.first << "," << pose(0, 3) << "," << pose(1, 3) << "," << pose(2, 3) << "," << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << std::endl;
                                    }
                                    else
                                    {
                                        if (consecutive_distance >= tol)
                                        {
                                            consecutive_distance = 0;
                                            outfile << std::setprecision(20);
                                            outfile << p.local_trajectory[i].timestamps.first << "," << pose(0, 3) << "," << pose(1, 3) << "," << pose(2, 3) << "," << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << std::endl;
                                        }
                                    }
                                }
                            }
                        }
                        outfile.close();
                    }
                }
            }
            if (ImGui::Button("save all marked trajectories to csv (timestampUnix,x,y,z,qx,qy,qz,qw)"))
            {
                std::shared_ptr<pfd::save_file> save_file;
                std::string output_file_name = "";
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
                const auto t = [&]()
                {
                    auto sel = pfd::save_file("Save csv file", "C:\\").result();
                    output_file_name = sel;
                    std::cout << "csv file to save: '" << output_file_name << "'" << std::endl;
                };
                std::thread t1(t);
                t1.join();

                if (output_file_name.size() > 0)
                {
                    std::ofstream outfile(output_file_name);
                    if (outfile.good())
                    {
                        float consecutive_distance = 0;

                        for (auto &p : session.point_clouds_container.point_clouds)
                        {
                            if (p.visible)
                            {

                                for (int i = 0; i < p.local_trajectory.size(); i++)
                                {
                                    const auto &m = p.local_trajectory[i].m_pose;
                                    Eigen::Affine3d pose = p.m_pose * m;
                                    pose.translation() += session.point_clouds_container.offset;
                                    Eigen::Quaterniond q(pose.rotation());

                                    if (i > 0)
                                    {
                                        double dist = (p.local_trajectory[i].m_pose.translation() - p.local_trajectory[i - 1].m_pose.translation()).norm();
                                        consecutive_distance += dist;
                                    }

                                    bool is_curve = false;

                                    if (i > 100 && i < p.local_trajectory.size() - 100)
                                    {
                                        Eigen::Vector3d position_prev = p.local_trajectory[i - 100].m_pose.translation();
                                        Eigen::Vector3d position_curr = p.local_trajectory[i].m_pose.translation();
                                        Eigen::Vector3d position_next = p.local_trajectory[i + 100].m_pose.translation();

                                        Eigen::Vector3d v1 = position_curr - position_prev;
                                        Eigen::Vector3d v2 = position_next - position_curr;

                                        if (v1.norm() > 0 && v2.norm() > 0)
                                        {
                                            double angle_deg = fabs(acos(v1.dot(v2) / (v1.norm() * v2.norm())) * 180.0 / M_PI);

                                            if (angle_deg > 10.0)
                                            {
                                                is_curve = true;
                                            }
                                        }
                                    }
                                    double tol = not_curve_consecutive_distance_meters;

                                    if (is_curve)
                                    {
                                        tol = curve_consecutive_distance_meters;
                                    }

                                    if (!is_trajectory_export_downsampling)
                                    {
                                        outfile << std::setprecision(20);
                                        outfile << p.local_trajectory[i].timestamps.second << "," << pose(0, 3) << "," << pose(1, 3) << "," << pose(2, 3) << "," << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << std::endl;
                                    }
                                    else
                                    {
                                        if (consecutive_distance >= tol)
                                        {
                                            consecutive_distance = 0;
                                            outfile << std::setprecision(20);
                                            outfile << p.local_trajectory[i].timestamps.second << "," << pose(0, 3) << "," << pose(1, 3) << "," << pose(2, 3) << "," << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << std::endl;
                                        }
                                    }
                                }
                            }
                        }
                        outfile.close();
                    }
                }
            }
            if (ImGui::Button("save all marked trajectories to csv (timestampLidar,timestampUnix,x,y,z,qx,qy,qz,qw)"))
            {
                std::shared_ptr<pfd::save_file> save_file;
                std::string output_file_name = "";
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
                const auto t = [&]()
                {
                    auto sel = pfd::save_file("Save csv file", "C:\\").result();
                    output_file_name = sel;
                    std::cout << "csv file to save: '" << output_file_name << "'" << std::endl;
                };
                std::thread t1(t);
                t1.join();

                if (output_file_name.size() > 0)
                {
                    std::ofstream outfile(output_file_name);
                    if (outfile.good())
                    {
                        float consecutive_distance = 0;
                        for (auto &p : session.point_clouds_container.point_clouds)
                        {
                            if (p.visible)
                            {

                                for (int i = 0; i < p.local_trajectory.size(); i++)
                                {
                                    const auto &m = p.local_trajectory[i].m_pose;
                                    Eigen::Affine3d pose = p.m_pose * m;
                                    pose.translation() += session.point_clouds_container.offset;
                                    Eigen::Quaterniond q(pose.rotation());

                                    if (i > 0)
                                    {
                                        double dist = (p.local_trajectory[i].m_pose.translation() - p.local_trajectory[i - 1].m_pose.translation()).norm();
                                        consecutive_distance += dist;
                                    }

                                    bool is_curve = false;

                                    if (i > 100 && i < p.local_trajectory.size() - 100)
                                    {
                                        Eigen::Vector3d position_prev = p.local_trajectory[i - 100].m_pose.translation();
                                        Eigen::Vector3d position_curr = p.local_trajectory[i].m_pose.translation();
                                        Eigen::Vector3d position_next = p.local_trajectory[i + 100].m_pose.translation();

                                        Eigen::Vector3d v1 = position_curr - position_prev;
                                        Eigen::Vector3d v2 = position_next - position_curr;

                                        if (v1.norm() > 0 && v2.norm() > 0)
                                        {
                                            double angle_deg = fabs(acos(v1.dot(v2) / (v1.norm() * v2.norm())) * 180.0 / M_PI);

                                            if (angle_deg > 10.0)
                                            {
                                                is_curve = true;
                                            }
                                        }
                                    }
                                    double tol = not_curve_consecutive_distance_meters;

                                    if (is_curve)
                                    {
                                        tol = curve_consecutive_distance_meters;
                                    }

                                    if (!is_trajectory_export_downsampling)
                                    {
                                        outfile << std::setprecision(20);
                                        outfile << p.local_trajectory[i].timestamps.first << "," << p.local_trajectory[i].timestamps.second << "," << pose(0, 3) << "," << pose(1, 3) << "," << pose(2, 3) << "," << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << std::endl;
                                    }
                                    else
                                    {
                                        if (consecutive_distance >= tol)
                                        {
                                            consecutive_distance = 0;
                                            outfile << std::setprecision(20);
                                            outfile << p.local_trajectory[i].timestamps.first << "," << p.local_trajectory[i].timestamps.second << "," << pose(0, 3) << "," << pose(1, 3) << "," << pose(2, 3) << "," << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << std::endl;
                                        }
                                    }
                                }
                            }
                        }
                        outfile.close();
                    }
                }
            }

        } // simple gui

        ImGui::Separator();
        ImGui::Separator();

        if (session.point_clouds_container.point_clouds.size() > 0)
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
                            g.coordinates -= session.point_clouds_container.offset;
                        }
                        for (auto &p : session.point_clouds_container.point_clouds)
                        {
                            p.available_geo_points = geo;
                        }
                    }
                }
            }

            if (ImGui::Button("load gnss files and convert from wgs84 to puwg92"))
            {
                static std::shared_ptr<pfd::open_file> open_file;
                std::vector<std::string> input_file_names;
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
                const auto t = [&]()
                {
                    std::vector<std::string> filters;
                    auto sel = pfd::open_file("Load gnss files", "C:\\", filters, true).result();
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
                    if (!gnss.load(input_file_names))
                    {
                        std::cout << "problem with loading gnss files" << std::endl;
                    }
                }
            }

            ImGui::InputDouble("WGS84ReferenceLatitude", &gnss.WGS84ReferenceLatitude);
            ImGui::InputDouble("WGS84ReferenceLongitude", &gnss.WGS84ReferenceLongitude);
            ImGui::InputDouble("OffsetAltitude", &gnss.offset_alt);

            ImGui::Checkbox("setWGS84ReferenceFromFirstPose", &gnss.setWGS84ReferenceFromFirstPose);

            if (ImGui::Button("load gnss files and convert from wgs84 to Cartesian using Mercator projection"))
            {
                static std::shared_ptr<pfd::open_file> open_file;
                std::vector<std::string> input_file_names;
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
                const auto t = [&]()
                {
                    std::vector<std::string> filters;
                    auto sel = pfd::open_file("Load gnss files", "C:\\", filters, true).result();
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
                    if (!gnss.load_mercator_projection(input_file_names))
                    {
                        std::cout << "problem with loading gnss files" << std::endl;
                    }
                }
            }
        }
        if (gnss.gnss_poses.size() > 0)
        {
            ImGui::Checkbox("show GNSS correspondences", &gnss.show_correspondences);
        }

        if (!simple_gui)
        {
            ImGui::Separator();
            ImGui::Separator();
            ImGui::Separator();
            ImGui::Separator();
            ImGui::Separator();
            ImGui::Separator();
            ImGui::Separator();
            ImGui::Separator();
            if (ImGui::Button("perform experiment on WIN"))
            {
                perform_experiment_on_windows(session, observation_picking, icp, ndt, registration_plane_feature, pose_graph_slam);
            }
            if (ImGui::Button("perform experiment on LINUX"))
            {
                perform_experiment_on_linux(session, observation_picking, icp, ndt, registration_plane_feature, pose_graph_slam);
            }
        }
    }
    ImGui::End();
}

void ndt_gui()
{
    static bool compute_mean_and_cov_for_bucket = false;
    ImGui::Begin("Normal Distribution Transforms");

    ImGui::InputFloat3("bucket_size (x[m],y[m],z[m])", ndt.bucket_size);
    if (ndt.bucket_size[0] < 0.01)
        ndt.bucket_size[0] = 0.01f;
    if (ndt.bucket_size[1] < 0.01)
        ndt.bucket_size[1] = 0.01f;
    if (ndt.bucket_size[2] < 0.01)
        ndt.bucket_size[2] = 0.01f;

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
        ndt.optimize(session.point_clouds_container.point_clouds, false, compute_mean_and_cov_for_bucket);
    }

    if (ImGui::Button("compute mean mahalanobis distance"))
    {
        double rms_initial = 0.0;
        double rms_final = 0.0;
        double mui = 0.0;
        // ndt.optimize(point_clouds_container.point_clouds, rms_initial, rms_final, mui);
        // std::cout << "mui: " << mui << " rms_initial: " << rms_initial << " rms_final: " << rms_final << std::endl;
        ndt.optimize(session.point_clouds_container.point_clouds, true, compute_mean_and_cov_for_bucket);
    }

    ImGui::Text("--------------------------------------------------------------------------------------------------------");

    if (ImGui::Button("ndt_optimization(Lie-algebra left Jacobian)"))
    {
        // icp.optimize_source_to_target_lie_algebra_left_jacobian(point_clouds_container);
        ndt.optimize_lie_algebra_left_jacobian(session.point_clouds_container.point_clouds, compute_mean_and_cov_for_bucket);
    }
    if (ImGui::Button("ndt_optimization(Lie-algebra right Jacobian)"))
    {
        // icp.optimize_source_to_target_lie_algebra_right_jacobian(point_clouds_container);
        ndt.optimize_lie_algebra_right_jacobian(session.point_clouds_container.point_clouds, compute_mean_and_cov_for_bucket);
    }

    ImGui::Text("--------------------------------------------------------------------------------------------------------");

    ImGui::Checkbox("generalized", &ndt.is_generalized);

    if (ndt.is_generalized)
    {
        ImGui::InputDouble("sigma_r", &ndt.sigma_r, 0.01, 0.01);
        ImGui::InputDouble("sigma_polar_angle_rad", &ndt.sigma_polar_angle, 0.0001, 0.0001);
        ImGui::InputDouble("sigma_azimuthal_angle_rad", &ndt.sigma_azimuthal_angle, 0.0001, 0.0001);
        ImGui::InputInt("num_extended_points", &ndt.num_extended_points, 1, 1);

        ImGui::Checkbox("compute_mean_and_cov_for_bucket", &compute_mean_and_cov_for_bucket);
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

    if (ImGui::Button("Set Zoller+Fröhlich TLS Imager 5016 errors"))
    {
        ndt.sigma_r = 0.00025;
        ndt.sigma_polar_angle = 0.004 / 180.0 * M_PI;
        ndt.sigma_azimuthal_angle = 0.004 / 180.0 * M_PI;
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
        icp.optimization_point_to_point_source_to_target(session.point_clouds_container);
    }
    ImGui::Text("--------------------------------------------------------------------------------------------------------");

    if (ImGui::Button("icp_optimization_source_to_target(Lie-algebra left Jacobian)"))
    {
        icp.optimize_source_to_target_lie_algebra_left_jacobian(session.point_clouds_container);
    }
    if (ImGui::Button("icp_optimization_source_to_target(Lie-algebra right Jacobian)"))
    {
        icp.optimize_source_to_target_lie_algebra_right_jacobian(session.point_clouds_container);
    }
    ImGui::Text("--------------------------------------------------------------------------------------------------------");

    if (ImGui::Button("compute rms(optimization_point_to_point_source_to_target)"))
    {
        double rms = 0.0;
        icp.optimization_point_to_point_source_to_target_compute_rms(session.point_clouds_container, rms);
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
        registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(session.point_clouds_container);
    }
    // ImGui::Text("--------------------------------------------------------------------------------------------------------");
    if (ImGui::Button("optimize_point_to_projection_onto_plane_source_to_target(Lie-algebra left Jacobian)"))
    {
        registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_left_jacobian(session.point_clouds_container);
    }
    if (ImGui::Button("optimize_point_to_projection_onto_plane_source_to_target(Lie-algebra right Jacobian)"))
    {
        registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_right_jacobian(session.point_clouds_container);
    }

    ImGui::Text("--------------------------------------------------------------------------------------------");

    if (ImGui::Button("optimize_point_to_plane_source_to_target (using dot product)"))
    {
        registration_plane_feature.optimize_point_to_plane_source_to_target(session.point_clouds_container);
    }
    if (ImGui::Button("optimize_distance_point_to_plane_source_to_target"))
    {
        registration_plane_feature.optimize_distance_point_to_plane_source_to_target(session.point_clouds_container);
    }
    if (ImGui::Button("optimize_plane_to_plane_source_to_target"))
    {
        registration_plane_feature.optimize_plane_to_plane_source_to_target(session.point_clouds_container);
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
        pose_graph_slam.optimize(session.point_clouds_container);
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
        pose_graph_slam.optimize_with_GTSAM(session.point_clouds_container);
        // std::cout << "mean uncertainty impact: " << mui << " rms_initial: " << rms_initial << " rms_final: " << rms_final << std::endl;
    }
#endif
    ImGui::Text("------------------------------------------------------------------------------------------------");
    ImGui::Text("----------with MANIF----------------------------------------------------------------------------");

#if WITH_MANIF
    if (ImGui::Button("optimize with manif (A small header-only library for Lie theory)"))
    {
        pose_graph_slam.optimize_with_manif(session.point_clouds_container);
        std::cout << "optimize with manif (A small header-only library for Lie theory) DONE" << std::endl;
    }
#endif
    ImGui::Text("------------------------------------------------------------------------------------------------");
    ImGui::End();
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
            for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
            {
                m_poses.push_back(session.point_clouds_container.point_clouds[i].m_pose);
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
                    if (session.point_clouds_container.show_with_initial_pose)
                    {
                        auto p = session.point_clouds_container.point_clouds[key].m_initial_pose * value;
                        observation_picking.add_intersection(p);
                    }
                    else
                    {
                        auto p = session.point_clouds_container.point_clouds[key].m_pose * value;
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
        double rms = compute_rms(true, session, observation_picking);
        std::cout << "RMS (initial poses): " << rms << std::endl;
        rms = compute_rms(false, session, observation_picking);
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
                export_result_to_folder(output_folder_name, observation_picking, session);
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

    if (show_axes || ImGui::GetIO().KeyCtrl)
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

    if (show_axes || ImGui::GetIO().KeyCtrl)
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
        session.manual_pose_graph_loop_closure.Render(session.point_clouds_container, index_loop_closure_source, index_loop_closure_target);
    }
    else
    {
        for (const auto &g : available_geo_points)
        {
            glBegin(GL_LINES);
            glColor3f(1.0f, 0.0f, 0.0f);
            auto c = g.coordinates - session.point_clouds_container.offset;
            glVertex3f(c.x() - 0.5, c.y(), c.z());
            glVertex3f(c.x() + 0.5, c.y(), c.z());

            glVertex3f(c.x(), c.y() - 0.5, c.z());
            glVertex3f(c.x(), c.y() + 0.5, c.z());

            glVertex3f(c.x(), c.y(), c.z() - 0.5);
            glVertex3f(c.x(), c.y(), c.z() + 0.5);
            glEnd();
        }

        //
        for (const auto &pc : session.point_clouds_container.point_clouds)
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

    gnss.render(session.point_clouds_container);
    session.ground_control_points.render(session.point_clouds_container);

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();

    // my_display_code();
    if (is_ndt_gui)
    {
        ndt_gui();
    }
    if (is_icp_gui)
    {
        icp_gui();
    }
    if (is_pose_graph_slam)
    {
        pose_graph_slam_gui();
    }
    if (is_registration_plane_feature)
    {
        registration_plane_feature_gui();
    }
    if (is_manual_analisys)
    {
        observation_picking_gui();
    }
    if (session.ground_control_points.is_imgui)
    {
        session.ground_control_points.imgui(session.point_clouds_container);
    }
    //if (manual_pose_graph_loop_closure_mode)
    //{
    //     manual_pose_graph_loop_closure.Gui();
    //}
    project_gui();

    if (!manual_pose_graph_loop_closure_mode)
    {
        for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
        {
            if (session.point_clouds_container.point_clouds[i].gizmo)
            {
                std::vector<Eigen::Affine3d> all_m_poses;
                for (int j = 0; j < session.point_clouds_container.point_clouds.size(); j++)
                {
                    all_m_poses.push_back(session.point_clouds_container.point_clouds[j].m_pose);
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

                session.point_clouds_container.point_clouds[i].m_pose(0, 0) = m_gizmo[0];
                session.point_clouds_container.point_clouds[i].m_pose(1, 0) = m_gizmo[1];
                session.point_clouds_container.point_clouds[i].m_pose(2, 0) = m_gizmo[2];
                session.point_clouds_container.point_clouds[i].m_pose(3, 0) = m_gizmo[3];
                session.point_clouds_container.point_clouds[i].m_pose(0, 1) = m_gizmo[4];
                session.point_clouds_container.point_clouds[i].m_pose(1, 1) = m_gizmo[5];
                session.point_clouds_container.point_clouds[i].m_pose(2, 1) = m_gizmo[6];
                session.point_clouds_container.point_clouds[i].m_pose(3, 1) = m_gizmo[7];
                session.point_clouds_container.point_clouds[i].m_pose(0, 2) = m_gizmo[8];
                session.point_clouds_container.point_clouds[i].m_pose(1, 2) = m_gizmo[9];
                session.point_clouds_container.point_clouds[i].m_pose(2, 2) = m_gizmo[10];
                session.point_clouds_container.point_clouds[i].m_pose(3, 2) = m_gizmo[11];
                session.point_clouds_container.point_clouds[i].m_pose(0, 3) = m_gizmo[12];
                session.point_clouds_container.point_clouds[i].m_pose(1, 3) = m_gizmo[13];
                session.point_clouds_container.point_clouds[i].m_pose(2, 3) = m_gizmo[14];
                session.point_clouds_container.point_clouds[i].m_pose(3, 3) = m_gizmo[15];
                session.point_clouds_container.point_clouds[i].pose = pose_tait_bryan_from_affine_matrix(session.point_clouds_container.point_clouds[i].m_pose);

                session.point_clouds_container.point_clouds[i].gui_translation[0] = (float)session.point_clouds_container.point_clouds[i].pose.px;
                session.point_clouds_container.point_clouds[i].gui_translation[1] = (float)session.point_clouds_container.point_clouds[i].pose.py;
                session.point_clouds_container.point_clouds[i].gui_translation[2] = (float)session.point_clouds_container.point_clouds[i].pose.pz;

                session.point_clouds_container.point_clouds[i].gui_rotation[0] = (float)(session.point_clouds_container.point_clouds[i].pose.om * 180.0 / M_PI);
                session.point_clouds_container.point_clouds[i].gui_rotation[1] = (float)(session.point_clouds_container.point_clouds[i].pose.fi * 180.0 / M_PI);
                session.point_clouds_container.point_clouds[i].gui_rotation[2] = (float)(session.point_clouds_container.point_clouds[i].pose.ka * 180.0 / M_PI);

                ImGui::End();

                if (!manipulate_only_marked_gizmo)
                {
                    Eigen::Affine3d curr_m_pose = session.point_clouds_container.point_clouds[i].m_pose;
                    for (int j = i + 1; j < session.point_clouds_container.point_clouds.size(); j++)
                    {
                        curr_m_pose = curr_m_pose * (all_m_poses[j - 1].inverse() * all_m_poses[j]);
                        session.point_clouds_container.point_clouds[j].m_pose = curr_m_pose;
                        session.point_clouds_container.point_clouds[j].pose = pose_tait_bryan_from_affine_matrix(session.point_clouds_container.point_clouds[j].m_pose);

                        session.point_clouds_container.point_clouds[j].gui_translation[0] = (float)session.point_clouds_container.point_clouds[j].pose.px;
                        session.point_clouds_container.point_clouds[j].gui_translation[1] = (float)session.point_clouds_container.point_clouds[j].pose.py;
                        session.point_clouds_container.point_clouds[j].gui_translation[2] = (float)session.point_clouds_container.point_clouds[j].pose.pz;

                        session.point_clouds_container.point_clouds[j].gui_rotation[0] = (float)(session.point_clouds_container.point_clouds[j].pose.om * 180.0 / M_PI);
                        session.point_clouds_container.point_clouds[j].gui_rotation[1] = (float)(session.point_clouds_container.point_clouds[j].pose.fi * 180.0 / M_PI);
                        session.point_clouds_container.point_clouds[j].gui_rotation[2] = (float)(session.point_clouds_container.point_clouds[j].pose.ka * 180.0 / M_PI);
                    }
                }
            }
        }

        session.point_clouds_container.render(observation_picking, viewer_decmiate_point_cloud);
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
                        if (session.point_clouds_container.show_with_initial_pose)
                        {
                            p1 = session.point_clouds_container.point_clouds[key1].m_initial_pose * value1;
                            p2 = session.point_clouds_container.point_clouds[key2].m_initial_pose * value2;
                        }
                        else
                        {
                            p1 = session.point_clouds_container.point_clouds[key1].m_pose * value1;
                            p2 = session.point_clouds_container.point_clouds[key2].m_pose * value2;
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
                mean += session.point_clouds_container.point_clouds[key1].m_initial_pose * value1;
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
    }
    else
    {
        // ImGuizmo -----------------------------------------------
        if (session.manual_pose_graph_loop_closure.gizmo && session.manual_pose_graph_loop_closure.edges.size() > 0)
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

            const int &index_src = session.manual_pose_graph_loop_closure.edges[session.manual_pose_graph_loop_closure.index_active_edge].index_from;

            const Eigen::Affine3d &m_src = session.point_clouds_container.point_clouds.at(index_src).m_pose;
            session.manual_pose_graph_loop_closure.edges[session.manual_pose_graph_loop_closure.index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_src.inverse() * m_g);

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

double distance_point_to_line(const Eigen::Vector3d &point, const LaserBeam &line)
{
    Eigen::Vector3d AP = point - line.position;

    double dist = (AP.cross(line.direction)).norm();
    return dist;
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

        if (glut_button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN && io.KeyCtrl)
        {
            if(session.ground_control_points.picking_mode){
                std::cout << "gcp picking" << std::endl;
                const auto laser_beam = GetLaserBeam(x, y);
                double min_distance = 10000000000;
                int index_i = -1;
                int index_j = -1;

                for (int i = 0; i < session.point_clouds_container.point_clouds.size(); i++){
                    for (int j = 0; j < session.point_clouds_container.point_clouds[i].local_trajectory.size(); j++){
                        const auto &p = session.point_clouds_container.point_clouds[i].local_trajectory[j].m_pose.translation();
                        Eigen::Vector3d vp = session.point_clouds_container.point_clouds[i].m_pose * p;

                        double dist = distance_point_to_line(vp, laser_beam);

                        if (dist < min_distance)
                        {
                            min_distance = dist;
                            index_i = i;
                            index_j = j;

                            rotation_center.x() = vp.x();
                            rotation_center.y() = vp.y();
                            rotation_center.z() = vp.z();

                            session.ground_control_points.picking_mode_index_to_node_inner = index_i;
                            session.ground_control_points.picking_mode_index_to_node_outer = index_j;

                            //if (picking_mode_index_to_node_inner != -1 && picking_mode_index_to_node_outer != -1)
                        }
                    }
                }

                //std::cout << "i: " << index_i << " j: " << index_j << std::endl;
                //rotation_center

                /*
                int PointPicking::pick_point(int x, int y, const std::vector<underground_mining::PointInsideROI>& points_global) {
    underground_mining::LaserBeam lb = GLWidgetGetOGLPos(x, y);

    double min_distance = 10000000000;
    int index = -1;
    for (size_t j = 0; j < points_global.size(); j++) {
        double dist = distance_point_to_line(points_global[j].coordinates_global, lb);
        if (dist < min_distance) {
            min_distance = dist;
            index = j;
            //if (dist < 0.0005) {
            //	return index;
            //}
        }
    }

    if (index != -1) {
        std::cout << "min_distance_to_line: " << min_distance << std::endl;
    }

    if (min_distance > 0.1) {
        return -1;
    }

    return index;
}
                */
            }else{
                const auto laser_beam = GetLaserBeam(x, y);

                RegistrationPlaneFeature::Plane pl;

                pl.a = 0;
                pl.b = 0;
                pl.c = 1;
                pl.d = 0;
                auto old_Totation_center = rotation_center;
                rotation_center = rayIntersection(laser_beam, pl).cast<float>();

                std::cout << "setting new rotation center to " << rotation_center << std::endl;

                rotate_x = 0.f;
                rotate_y = 0.f;
            }
        }

        if (state == GLUT_DOWN)
        {
            mouse_buttons |= 1 << glut_button;

            //-
            if (observation_picking.is_observation_picking_mode)
            {
                Eigen::Vector3d p = GLWidgetGetOGLPos(x, y, observation_picking);
                int number_active_pcs = 0;
                int index_picked = -1;
                for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
                {
                    if (session.point_clouds_container.point_clouds[i].visible)
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
    glutCreateWindow("multi_view_tls_registration_step_2 " HDMAPPING_VERSION_STRING);
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
    try {
        initGL(&argc, argv);
        glutDisplayFunc(display);
        glutMouseFunc(mouse);
        glutMotionFunc(motion);
        glutMouseWheelFunc(wheel);
        glutMainLoop();

        ImGui_ImplOpenGL2_Shutdown();
        ImGui_ImplGLUT_Shutdown();

        ImGui::DestroyContext();
    }
    catch (const std::bad_alloc e)
    {
        std::cerr << "System is out of memory : " << e.what() << std::endl;
        mandeye::fd::OutOfMemMessage();
    }
    catch (const std::exception e)
    {
        std::cout << e.what();
    }
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

Eigen::Vector3d GLWidgetGetOGLPos(int x, int y, const ObservationPicking &observation_picking)
{
    const auto laser_beam = GetLaserBeam(x, y);

    RegistrationPlaneFeature::Plane pl;

    pl.a = 0;
    pl.b = 0;
    pl.c = 1;
    pl.d = -observation_picking.picking_plane_height;

    Eigen::Vector3d pos = rayIntersection(laser_beam, pl);

    std::cout << "intersection: " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;

    return pos;
}