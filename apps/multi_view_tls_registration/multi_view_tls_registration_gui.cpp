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

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <ndt.h>
#include <icp.h>
#include <registration_plane_feature.h>
#include <transformations.h>
#include <pose_graph_slam.h>
#include <pcl_wrapper.h>
#include <observation_picking.h>

#include <utils.hpp>

#include <portable-file-dialogs.h>

#include <laszip/laszip_api.h>

#include <iostream>
#include <filesystem>
#include <fstream>

#include <manual_pose_graph_loop_closure.h>
#include "multi_view_tls_registration.h"

#include <gnss.h>
#include <session.h>
#include <pfd_wrapper.hpp>

#include <HDMapping/Version.hpp>

#include <export_laz.h>
#include "wgs84_do_puwg92.h"
#include "WGS84toCartesian.hpp"

#ifdef _WIN32
#include <windows.h>
#include "../../resources/resource2.h"
#endif

#ifdef _WIN32
bool consWin = true;
#endif
bool consImGui = false;

std::string winTitle = std::string("Step 2 (Multi view TSL registration) ") + HDMAPPING_VERSION_STRING;

std::vector<std::string> infoLines = {
    "This program is second step in MANDEYE process.",
    "",
    "It refines trajectory (e.g with loop closure)",
    "It refines trajectory with many approaches (e.g. Iterative Closest Point, Normal Distributions Transform)",
    "It exports session as rigid point cloud to single LAZ file.",
    "LAZ files are the product of MANDEYE process (open them with Cloud Compare)",
};

namespace fs = std::filesystem;

static bool show_demo_window = true;
static bool show_another_window = false;

bool block_z = false;

bool gnssWithOffset = false;

//radio button selectors
static int NDTnomSelection = 0;
static int NDTpeSelection = 0;
static int NDT3dSelection = 0;
static int ICPnomSelection = 0;
static int ICPpeSelection = 0;
static int ICP3dSelection = 0;
static int RPFnomSelection = 0;
static int RPFpeSelection = 0;
static int RPF3dSelection = 0;
static int PGSnomSelection = 0;
static int PGSpeSelection = 0;
static int PGS3dSelection = 0;
static int PGSpwmtSelection = 0;

std::string session_file_name = "";
int session_total_number_of_points = 0;
PointClouds::PointCloudDimensions session_dims;
//bool dynamicSubsampling = true;
//static double lastAdjustTime = 0.0;  // last time we changed subsampling
//const double cooldownSeconds = 1;  // wait between auto adjustments
//static float fps_avg = 60.0f;

bool is_pca_gui = false;
bool is_ndt_gui = true;
bool is_icp_gui = false;
bool is_rpf_gui = false;
bool is_pose_graph_slam = false;
bool is_manual_analisys = false;
bool is_loop_closure_gui = false;
bool is_lio_segments_gui = false;

bool new_loop_closure_index = false;

TLSRegistration tls_registration;
ObservationPicking observation_picking;
std::vector<Eigen::Vector3d> picked_points;

int index_loop_closure_source = 0;
int index_loop_closure_target = 0;
int index_begin = 0;
int index_end = 0;

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

bool manipulate_only_marked_gizmo = false;

Session session;
bool simple_gui = true;
bool session_loaded = false;

int num_edge_extended_before = 0;
int num_edge_extended_after = 0;

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
double compute_rms(bool initial, Session &session, ObservationPicking &observation_picking);
void reset_poses(Session &session);

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
    ImGui::Begin("Single session processing");

    ImGui::Checkbox("Simple_gui", &simple_gui);
    ImGui::SameLine();
    ImGui::Checkbox("Is ground truth", &session.is_ground_truth);

    ImGui::NewLine();

    if (ImGui::Button("Set initial pose to Identity and update other poses"))
    {
        initial_pose_to_identity(session);
    }

    if (!simple_gui)
    {
		ImGui::NewLine();
        
        ImGui::Text("Offset [m] x: %.10f y: %.10f z: %.10f", session.point_clouds_container.offset.x(), session.point_clouds_container.offset.y(), session.point_clouds_container.offset.z());
        ImGui::SameLine();
        if (ImGui::Button("Print offset to console"))
            std::cout << "Offset: " << std::setprecision(10) << session.point_clouds_container.offset << std::endl;

        ImGui::Checkbox("Decimate during load", &tls_registration.is_decimate);

		ImGui::Text("Bucket [m]:");
        ImGui::PushItemWidth(ImGuiNumberWidth);
        ImGui::InputDouble("X##b", &tls_registration.bucket_x, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(xText);
        ImGui::SameLine();
        ImGui::InputDouble("Y##b", &tls_registration.bucket_y, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(yText);
        ImGui::SameLine();
        ImGui::InputDouble("Z##b", &tls_registration.bucket_z, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(zText);
		ImGui::PopItemWidth();

        ImGui::Separator();
        // common_data
        // manual_pose_graph_loop_closure_mode

        if (ImGui::Button("Load RESSO file (transformation_GroundTruth.reg)"))
        {
            std::string input_file_name = "";
            input_file_name = mandeye::fd::OpenFileDialogOneFile("Load RESSO", mandeye::fd::Resso_filter);
            std::cout << "RESSO file: '" << input_file_name << "'" << std::endl;

            if (input_file_name.size() > 0)
            {

                session.working_directory = fs::path(input_file_name).parent_path().string();

                if (!session.point_clouds_container.load(session.working_directory.c_str(), input_file_name.c_str(), tls_registration.is_decimate, tls_registration.bucket_x, tls_registration.bucket_y, tls_registration.bucket_z))
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
        if (ImGui::Button("Save RESSO file"))
        {
            const auto output_file_name = mandeye::fd::SaveFileDialog("Save RESSO file", mandeye::fd::Resso_filter);
            std::cout << "RESSO file to save: '" << output_file_name << "'" << std::endl;
            if (output_file_name.size() > 0)
            {
                session.point_clouds_container.save_poses(fs::path(output_file_name).string(), false);
            }
        }
        ImGui::Text("RESSO dataset: ");
        ImGui::SameLine();
        ImGuiHyperlink("https://3d.bk.tudelft.nl/liangliang/publications/2019/plade/resso.html");

        if (ImGui::Button("Load ETH file (pairs.txt)"))
        {
            std::string input_file_name = "";
            input_file_name = mandeye::fd::OpenFileDialogOneFile("Load ETH file", {});
            if (input_file_name.size() > 0)
            {
                session.working_directory = fs::path(input_file_name).parent_path().string();

                if (!session.point_clouds_container.load_eth(session.working_directory.c_str(), input_file_name.c_str(), tls_registration.is_decimate, tls_registration.bucket_x, tls_registration.bucket_y, tls_registration.bucket_z))
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
        ImGui::Text("ETH dataset: ");
        ImGui::SameLine();
        ImGuiHyperlink("https://prs.igp.ethz.ch/research/completed_projects/automatic_registration_of_point_clouds.html");

        if (ImGui::Button("load AlignedPointCloud from WHU-TLS (select all *.las or *.laz files in folder 2-AlignedPointCloud)"))
        {
            session.point_clouds_container.point_clouds.clear();
            std::vector<std::string> input_file_names;
            input_file_names = mandeye::fd::OpenFileDialog("Load las files", {}, true);
            if (input_file_names.size() > 0)
            {
                session.working_directory = fs::path(input_file_names[0]).parent_path().string();

                std::cout << "Las/Laz files:" << std::endl;
                for (size_t i = 0; i < input_file_names.size(); i++)
                {
                    std::cout << input_file_names[i] << std::endl;
                }

                if (!session.point_clouds_container.load_whu_tls(input_file_names, tls_registration.is_decimate, tls_registration.bucket_x, tls_registration.bucket_y, tls_registration.bucket_z, tls_registration.calculate_offset))
                {
                    std::cout << "Check input files laz/las" << std::endl;
                    // return;
                }
                else
                {
                    std::cout << "Loaded: " << session.point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
                }
            }

            int number_of_point = 0;
            for (const auto &pc : session.point_clouds_container.point_clouds)
            {
                number_of_point += pc.points_local.size();
            }

            session.point_clouds_container.print_point_cloud_dimension();

            [[maybe_unused]]
            pfd::message message(
                "Information",
                "If you can not see point cloud --> 1. Change 'Points render subsampling', 2. Check console 'min max coordinates should be small numbers to see points in our local coordinate system'. 3. Set checkbox 'calculate_offset for WHU-TLS'. 4. Later on You can change offset directly in session json file.",
                pfd::choice::ok, pfd::icon::info);
            message.result();
        }
        ImGui::SameLine();
        ImGui::Checkbox("Calculate_offset for WHU-TLS", &tls_registration.calculate_offset);
        ImGui::Text("WHU-TLS dataset: ");
        ImGui::SameLine();
        ImGuiHyperlink("http://3s.whu.edu.cn/ybs/en/benchmark.htm");

        if (ImGui::Button("load 3DTK files (select all *.txt files)"))
        {
            session.point_clouds_container.point_clouds.clear();
            std::vector<std::string> input_file_names;
            input_file_names = mandeye::fd::OpenFileDialog("Load txt files", {}, true);

            if (input_file_names.size() > 0)
            {
                session.working_directory = fs::path(input_file_names[0]).parent_path().string();

                std::cout << "TXT files:" << std::endl;
                for (size_t i = 0; i < input_file_names.size(); i++)
                {
                    std::cout << input_file_names[i] << std::endl;
                }

                if (!session.point_clouds_container.load_3DTK_tls(input_file_names, tls_registration.is_decimate, tls_registration.bucket_x, tls_registration.bucket_y, tls_registration.bucket_z))
                {
                    std::cout << "Check input files" << std::endl;
                    return;
                }
                else
                {
                    std::cout << "Loaded: " << session.point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
                }
            }
        }
        ImGui::Text("3DTK dataset (18: the campus of the Jacobs University Bremen)");
        ImGui::SameLine();
        ImGuiHyperlink("http://kos.informatik.uni-osnabrueck.de/3Dscans/");

        if (ImGui::Button("Update initial poses from RESSO file"))
        {
            std::string input_file_name;
            input_file_name = mandeye::fd::OpenFileDialogOneFile("Load RESSO file", {});
            if (input_file_name.size() > 0)
            {

                session.working_directory = fs::path(input_file_name).parent_path().string();

                if (!session.point_clouds_container.update_initial_poses_from_RESSO(session.working_directory.c_str(), input_file_name.c_str()))
                {

                    std::cout << "Check input files" << std::endl;
                    return;
                }
                else
                {
                    session.point_clouds_container.initial_poses_file_name = input_file_name;
                    std::cout << "Updated: " << session.point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
                }
            }
        }
        ImGui::SameLine();
        ImGui::Text(session.point_clouds_container.initial_poses_file_name.c_str());

        if (ImGui::Button("Update poses from RESSO file"))
        {
            std::string input_file_name;
            input_file_name = mandeye::fd::OpenFileDialogOneFile("Load RESSO file", {});

            if (input_file_name.size() > 0)
            {
                session.working_directory = fs::path(input_file_name).parent_path().string();

                if (!session.point_clouds_container.update_poses_from_RESSO(session.working_directory.c_str(), input_file_name.c_str()))
                {
                    std::cout << "Check input files" << std::endl;
                    return;
                }
                else
                {
                    std::cout << "Updated: " << session.point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
                    session.point_clouds_container.poses_file_name = input_file_name;
                }
            }
        }
        ImGui::SameLine();

        if (ImGui::Button("Update poses from RESSO file (inverse)"))
        {
            std::string input_file_name;
            input_file_name = mandeye::fd::OpenFileDialogOneFile("Load RESSO file", {});

            if (input_file_name.size() > 0)
            {
                session.working_directory = fs::path(input_file_name).parent_path().string();

                if (!session.point_clouds_container.update_poses_from_RESSO_inverse(session.working_directory.c_str(), input_file_name.c_str()))
                {
                    std::cout << "Check input files" << std::endl;
                    return;
                }
                else
                {
                    std::cout << "Updated: " << session.point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
                    session.point_clouds_container.poses_file_name = input_file_name;
                }
            }
        }
        ImGui::SameLine();
        ImGui::Text(session.point_clouds_container.poses_file_name.c_str());
    }

    ImGui::Separator();

    if (!is_loop_closure_gui)
    {
        if (!simple_gui)
        {
            static double x_origin = 0.0;
            static double y_origin = 0.0;
            static double z_origin = 0.0;

            ImGui::Text("Origin [m]: ");
            ImGui::SameLine();
			ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputDouble("X##o", &x_origin, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(xText);
			ImGui::SameLine();
            ImGui::InputDouble("Y##o", &y_origin, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(yText);
            ImGui::SameLine();
            ImGui::InputDouble("Z##o", &z_origin, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(zText);
			ImGui::PopItemWidth();
            ImGui::SameLine();
            if (ImGui::Button("Set XYZ origin"))
            {
                if (session.point_clouds_container.point_clouds.size() != 0)
                {
                    std::vector<Eigen::Affine3d> all_m_poses2;
                    for (int j = 0; j < session.point_clouds_container.point_clouds.size(); j++)
                    {
                        all_m_poses2.push_back(session.point_clouds_container.point_clouds[j].m_pose);
                    }

                    session.point_clouds_container.point_clouds[0].m_pose(0, 3) = x_origin;
                    session.point_clouds_container.point_clouds[0].m_pose(1, 3) = y_origin;
                    session.point_clouds_container.point_clouds[0].m_pose(2, 3) = z_origin;

                    session.point_clouds_container.point_clouds[0].pose = pose_tait_bryan_from_affine_matrix(session.point_clouds_container.point_clouds[0].m_pose);

                    session.point_clouds_container.point_clouds[0].gui_translation[0] = (float)session.point_clouds_container.point_clouds[0].pose.px;
                    session.point_clouds_container.point_clouds[0].gui_translation[1] = (float)session.point_clouds_container.point_clouds[0].pose.py;
                    session.point_clouds_container.point_clouds[0].gui_translation[2] = (float)session.point_clouds_container.point_clouds[0].pose.pz;

                    session.point_clouds_container.point_clouds[0].gui_rotation[0] = (float)(session.point_clouds_container.point_clouds[0].pose.om * 180.0 / M_PI);
                    session.point_clouds_container.point_clouds[0].gui_rotation[1] = (float)(session.point_clouds_container.point_clouds[0].pose.fi * 180.0 / M_PI);
                    session.point_clouds_container.point_clouds[0].gui_rotation[2] = (float)(session.point_clouds_container.point_clouds[0].pose.ka * 180.0 / M_PI);

                    Eigen::Affine3d curr_m_pose2 = session.point_clouds_container.point_clouds[0].m_pose;
                    for (int j = 1; j < session.point_clouds_container.point_clouds.size(); j++)
                    {
                        curr_m_pose2 = curr_m_pose2 * (all_m_poses2[j - 1].inverse() * all_m_poses2[j]);

                        // std::cout << curr_m_pose2.matrix() << std::endl;
                        session.point_clouds_container.point_clouds[j]
                            .m_pose = curr_m_pose2;
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
            
            ImGui::Separator();
        }

        ImGui::Text("Set offsets to export point cloud in global coordinate system [m]:");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("each local coordinate of the point += offset");
        ImGui::PushItemWidth(ImGuiNumberWidth);
        ImGui::InputDouble("X##t", &session.point_clouds_container.offset.x(), 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(xText);
		ImGui::SameLine();
        ImGui::InputDouble("Y##t", &session.point_clouds_container.offset.y(), 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(yText);
        ImGui::SameLine();
        ImGui::InputDouble("Z##t", &session.point_clouds_container.offset.z(), 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(zText);
		ImGui::PopItemWidth();

        if (!simple_gui)
        {
            ImGui::NewLine();
            ImGui::NewLine();
            ImGui::NewLine();
            ImGui::Separator();
            ImGui::NewLine();
            ImGui::NewLine();
            ImGui::NewLine();
			ImGui::Text("Perform experiment on:");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Experiments to compare methods and approaches for multi-view TLS registration");
			ImGui::SameLine();
            if (ImGui::Button("WINDOWS"))
            {
                perform_experiment_on_windows(session, observation_picking, tls_registration.icp, tls_registration.ndt, tls_registration.registration_plane_feature, tls_registration.pose_graph_slam);
            }
            ImGui::SameLine();
            if (ImGui::Button("LINUX"))
            {
                perform_experiment_on_linux(session, observation_picking, tls_registration.icp, tls_registration.ndt, tls_registration.registration_plane_feature, tls_registration.pose_graph_slam);
            }
        }
    }
    ImGui::End();
}

ImVec4 orangeBorder(1.0f, 0.5f, 0.0f, 1.0f);

void ndt_gui()
{
    ImGui::InputFloat3("Bucket size (x,y,z) [m]", tls_registration.ndt.bucket_size);
    if (tls_registration.ndt.bucket_size[0] < 0.01)
        tls_registration.ndt.bucket_size[0] = 0.01f;
    if (tls_registration.ndt.bucket_size[1] < 0.01)
        tls_registration.ndt.bucket_size[1] = 0.01f;
    if (tls_registration.ndt.bucket_size[2] < 0.01)
        tls_registration.ndt.bucket_size[2] = 0.01f;

    ImGui::PushItemWidth(ImGuiNumberWidth);
    ImGui::InputInt("Number of threads", &tls_registration.ndt.number_of_threads);
    if (tls_registration.ndt.number_of_threads < 1)
        tls_registration.ndt.number_of_threads = 1;
    ImGui::SameLine();
    ImGui::InputInt("Number of iterations", &tls_registration.ndt.number_of_iterations);
    if (tls_registration.ndt.number_of_iterations < 1)
        tls_registration.ndt.number_of_iterations = 1;
    ImGui::PopItemWidth();

    ImGui::Checkbox("Fix first node (add I to first pose in Hessian)", &tls_registration.ndt.is_fix_first_node);

    ImGui::Text("Nonlinear optimization method:");
    ImGui::SameLine();
    ImGui::RadioButton("Gauss-Newton", &NDTnomSelection, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Levenberg-Marguardt", &NDTnomSelection, 1);

    tls_registration.ndt.is_gauss_newton = (NDTnomSelection == 0);
    tls_registration.ndt.is_levenberg_marguardt = (NDTnomSelection == 1);

    ImGui::Text("Poses expressed as:");
    ImGui::SameLine();
    ImGui::RadioButton("camera<-world (cw)", &NDTpeSelection, 0);
    ImGui::SameLine();
    ImGui::RadioButton("camera->world (wc)", &NDTpeSelection, 1);

    tls_registration.ndt.is_cw = (NDTpeSelection == 0);
    tls_registration.ndt.is_wc = (NDTpeSelection == 1);

    ImGui::Text("Parameterizations of 3D rotation:");
    ImGui::RadioButton("Tait-Bryan angles (om fi ka: RxRyRz)", &NDT3dSelection, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Quaternion (q0 q1 q2 q3)", &NDT3dSelection, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Rodrigues (sx sy sz)", &NDT3dSelection, 2);

    tls_registration.ndt.is_tait_bryan_angles = (NDT3dSelection == 0);
    tls_registration.ndt.is_quaternion = (NDT3dSelection == 1);
    tls_registration.ndt.is_rodrigues = (NDT3dSelection == 2);


    if (ImGui::Button("Optimization"))
    {
        double rms_initial = 0.0;
        double rms_final = 0.0;
        double mui = 0.0;
        // ndt.optimize(point_clouds_container.point_clouds, rms_initial, rms_final, mui);
        // std::cout << "mui: " << mui << " rms_initial: " << rms_initial << " rms_final: " << rms_final << std::endl;
        tls_registration.ndt.optimize(session.point_clouds_container.point_clouds, false, tls_registration.compute_mean_and_cov_for_bucket);
    }
        
    if (ImGui::Button("Compute mean Mahalanobis distance"))
    {
        double rms_initial = 0.0;
        double rms_final = 0.0;
        double mui = 0.0;

        tls_registration.ndt.optimize(session.point_clouds_container.point_clouds, true, tls_registration.compute_mean_and_cov_for_bucket);
    }
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::Text("Average of all Mahalanobis distances between transformed source points\nand their corresponding Gaussian cells, where:");
        ImGui::Text("Mahalanobis distance measures how far a point is from the mean of a multivariate Gaussian distribution,\ntaking into account the covariance (shape and orientation) of that distribution");
        ImGui::EndTooltip();
    }

    ImGui::Separator();

    ImGui::Text("NDT optimization Lie algebra:");
    ImGui::SameLine();
    if (ImGui::Button("left Jacobian"))
    {

        tls_registration.ndt.optimize_lie_algebra_left_jacobian(session.point_clouds_container.point_clouds, tls_registration.compute_mean_and_cov_for_bucket);
    }
    ImGui::SameLine();
    if (ImGui::Button("right Jacobian"))
    {

        tls_registration.ndt.optimize_lie_algebra_right_jacobian(session.point_clouds_container.point_clouds, tls_registration.compute_mean_and_cov_for_bucket);
    }

    ImGui::Separator();

    ImGui::Checkbox("Generalized", &tls_registration.ndt.is_generalized);
    ImGui::BeginDisabled(!tls_registration.ndt.is_generalized);
    {
        ImGui::PushItemWidth(ImGuiNumberWidth);
        ImGui::InputDouble("sigma_r", &tls_registration.ndt.sigma_r, 0.01, 0.01);
        ImGui::InputDouble("sigma_polar_angle_rad", &tls_registration.ndt.sigma_polar_angle, 0.0001, 0.0001);
        ImGui::InputDouble("sigma_azimuthal_angle_rad", &tls_registration.ndt.sigma_azimuthal_angle, 0.0001, 0.0001);
        ImGui::InputInt("num_extended_points", &tls_registration.ndt.num_extended_points, 1, 1);
        ImGui::PopItemWidth();

        ImGui::Checkbox("compute_mean_and_cov_for_bucket", &tls_registration.compute_mean_and_cov_for_bucket);
    }
    ImGui::EndDisabled();

    ImGui::Text("Set error presets:");
    ImGui::Text("Zoller+Fröhlich TLS Imager");
    ImGui::SameLine();
    if (ImGui::Button("5006i")) tls_registration.set_zoller_frohlich_tls_imager_5006i_errors();
    ImGui::SameLine();
    if (ImGui::Button("5010C")) tls_registration.set_zoller_frohlich_tls_imager_5010c_errors();
    ImGui::SameLine();
    if (ImGui::Button("5016")) tls_registration.set_zoller_frohlich_tls_imager_5016_errors();


    ImGui::Text("Leica");
    ImGui::SameLine();
    if (ImGui::Button("ScanStation C5 C10")) tls_registration.set_leica_scanstation_c5_c10_errors();
    ImGui::SameLine();
    if (ImGui::Button("Leica HDS6100")) tls_registration.set_leica_hds6100_errors();
    ImGui::SameLine();
    if (ImGui::Button("Leica P40")) tls_registration.set_leica_p40_errors();

    if (ImGui::Button("Faro Focus3D")) tls_registration.set_faro_focus3d_errors();
    ImGui::SameLine();
    if (ImGui::Button("Riegl VZ400")) tls_registration.set_riegl_vz400_errors();
    ImGui::SameLine();
    if (ImGui::Button("Livox mid360")) tls_registration.set_livox_mid360_errors();
}

void icp_gui()
{
    ImGui::PushItemWidth(ImGuiNumberWidth);
    ImGui::InputFloat("Search radius", &tls_registration.icp.search_radius, 0.01f, 0.1f);
    if (tls_registration.icp.search_radius < 0.01f)
        tls_registration.icp.search_radius = 0.01f;
    if (tls_registration.icp.search_radius > 2.0f)
        tls_registration.icp.search_radius = 2.0f;

    ImGui::InputInt("Number of threads", &tls_registration.icp.number_of_threads);
    if (tls_registration.icp.number_of_threads < 1)
        tls_registration.icp.number_of_threads = 1;
    ImGui::SameLine();
    ImGui::InputInt("Number of iterations", &tls_registration.icp.number_of_iterations);
    if (tls_registration.icp.number_of_iterations < 1)
        tls_registration.icp.number_of_iterations = 1;
    ImGui::PopItemWidth();

    ImGui::Checkbox("Adaptive robust kernel", &tls_registration.icp.is_adaptive_robust_kernel);
    ImGui::SameLine();
    ImGui::Checkbox("Fix first node (add I to first pose in Hessian)", &tls_registration.icp.is_fix_first_node);

    ImGui::Text("Nonlinear optimization method:");
    ImGui::SameLine();
    ImGui::RadioButton("Gauss-Newton", &ICPnomSelection, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Levenberg-Marguardt", &ICPnomSelection, 1);

    tls_registration.icp.is_gauss_newton = (ICPnomSelection == 0);
    tls_registration.icp.is_levenberg_marguardt = (ICPnomSelection == 1);

    ImGui::Text("Poses expressed as:");
    ImGui::SameLine();
    ImGui::RadioButton("camera<-world (cw)", &ICPpeSelection, 0);
    ImGui::SameLine();
    ImGui::RadioButton("camera->world (wc)", &ICPpeSelection, 1);

    tls_registration.icp.is_cw = (ICPpeSelection == 0);
    tls_registration.icp.is_wc = (ICPpeSelection == 1);

    ImGui::Text("Parameterizations of 3D rotation:");
    ImGui::RadioButton("Tait-Bryan angles (om fi ka: RxRyRz)", &ICP3dSelection, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Quaternion (q0 q1 q2 q3)", &ICP3dSelection, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Rodrigues (sx sy sz)", &ICP3dSelection, 2);

    tls_registration.icp.is_tait_bryan_angles = (ICP3dSelection == 0);
    tls_registration.icp.is_quaternion = (ICP3dSelection == 1);
    tls_registration.icp.is_rodrigues = (ICP3dSelection == 2);

    if (ImGui::Button("Optimization point to point source to target"))
    {
        tls_registration.icp.optimization_point_to_point_source_to_target(session.point_clouds_container);
    }
    ImGui::Separator();

    ImGui::Text("Optimization source to target Lie-algebra:");
    ImGui::SameLine();
    if (ImGui::Button("left Jacobian"))
    {
        tls_registration.icp.optimize_source_to_target_lie_algebra_left_jacobian(session.point_clouds_container);
    }
    ImGui::SameLine();
    if (ImGui::Button("right Jacobian"))
    {
        tls_registration.icp.optimize_source_to_target_lie_algebra_right_jacobian(session.point_clouds_container);
    }
    ImGui::Separator();

    if (ImGui::Button("Compute rms (optimization_point_to_point_source_to_target)"))
    {
        double rms = 0.0;
        tls_registration.icp.optimization_point_to_point_source_to_target_compute_rms(session.point_clouds_container, rms);
        std::cout << "rms (optimization_point_to_point_source_to_target): " << rms << std::endl;
    }
}

void rpf_gui()
{
    ImGui::PushItemWidth(ImGuiNumberWidth);
    ImGui::InputFloat("Search radius", &tls_registration.registration_plane_feature.search_radius, 0.01, 2.0);
    if (tls_registration.registration_plane_feature.search_radius < 0.01)
        tls_registration.registration_plane_feature.search_radius = 0.01;
    if (tls_registration.registration_plane_feature.search_radius > 2.0)
        tls_registration.registration_plane_feature.search_radius = 2.0;

    ImGui::InputInt("Number of threads", &tls_registration.registration_plane_feature.number_of_threads);
    if (tls_registration.registration_plane_feature.number_of_threads < 1)
        tls_registration.registration_plane_feature.number_of_threads = 1;
    ImGui::SameLine();
    ImGui::InputInt("Number of iterations", &tls_registration.registration_plane_feature.number_of_iterations);
    if (tls_registration.registration_plane_feature.number_of_iterations < 1)
        tls_registration.registration_plane_feature.number_of_iterations = 1;
    ImGui::PopItemWidth();

    ImGui::Checkbox("Adaptive robust kernel", &tls_registration.registration_plane_feature.is_adaptive_robust_kernel);
    ImGui::SameLine();
    ImGui::Checkbox("Fix first node (add I to first pose in Hessian)", &tls_registration.registration_plane_feature.is_fix_first_node);

    ImGui::Text("Nonlinear optimization method:");
    ImGui::SameLine();
    ImGui::RadioButton("Gauss-Newton", &RPFnomSelection, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Levenberg-Marguardt", &RPFnomSelection, 1);

    tls_registration.registration_plane_feature.is_gauss_newton = (RPFnomSelection == 0);
    tls_registration.registration_plane_feature.is_levenberg_marguardt = (RPFnomSelection == 1);

    ImGui::Text("Poses expressed as:");
    ImGui::SameLine();
    ImGui::RadioButton("camera<-world (cw)", &RPFpeSelection, 0);
    ImGui::SameLine();
    ImGui::RadioButton("camera->world (wc)", &RPFpeSelection, 1);

    tls_registration.registration_plane_feature.is_cw = (RPFpeSelection == 0);
    tls_registration.registration_plane_feature.is_wc = (RPFpeSelection == 1);

    ImGui::Text("Parameterizations of 3D rotation:");
    ImGui::RadioButton("Tait-Bryan angles (om fi ka: RxRyRz)", &RPF3dSelection, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Quaternion (q0 q1 q2 q3)", &RPF3dSelection, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Rodrigues (sx sy sz)", &RPF3dSelection, 2);

    tls_registration.registration_plane_feature.is_tait_bryan_angles = (RPF3dSelection == 0);
    tls_registration.registration_plane_feature.is_quaternion = (RPF3dSelection == 1);
    tls_registration.registration_plane_feature.is_rodrigues = (RPF3dSelection == 2);

    ImGui::Separator();
    ImGui::Text("Optimize point to projection onto plane source to target:");
    if (ImGui::Button("Basic Jacobian"))
        tls_registration.registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(session.point_clouds_container);
    ImGui::SameLine();
    if (ImGui::Button("Lie-algebra left Jacobian"))
        tls_registration.registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_left_jacobian(session.point_clouds_container);
    ImGui::SameLine();
    if (ImGui::Button("Lie-algebra right Jacobian"))
        tls_registration.registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_right_jacobian(session.point_clouds_container);

    ImGui::Separator();

    ImGui::Text("Optimize source to target:");
    if (ImGui::Button("point to plane (using dot product)"))
        tls_registration.registration_plane_feature.optimize_point_to_plane_source_to_target(session.point_clouds_container);
    ImGui::SameLine();
    if (ImGui::Button("distance point to plane"))
        tls_registration.registration_plane_feature.optimize_distance_point_to_plane_source_to_target(session.point_clouds_container);
    ImGui::SameLine();
    if (ImGui::Button("plane to plane"))
        tls_registration.registration_plane_feature.optimize_plane_to_plane_source_to_target(session.point_clouds_container);
}


void pca_gui()
{
    ImGui::Begin("Point cloud alignment", &is_pca_gui, ImGuiWindowFlags_MenuBar);

    if (ImGui::BeginMenuBar())
    {
        bool justPushed = false;

        if (is_ndt_gui) ImGui::PushStyleColor(ImGuiCol_Button, orangeBorder);
        if (ImGui::Button("Normal Distributions Transform"))
        {
            if (!is_ndt_gui)
            {
                is_ndt_gui = true;
                is_icp_gui = false;
                is_rpf_gui = false;
                justPushed = true;
            }
        }
        if (is_ndt_gui && !justPushed) ImGui::PopStyleColor();
        if (ImGui::IsItemHovered())
        {
            ImGui::BeginTooltip();
            ImGui::Text("Probabilistic alternative to ICP that models one cloud (the target)\nas a set of Gaussian distributions rather than raw points");
            ImGui::Text("Robust for rough initial poses but can converge to a local optimum\nif the initial misalignment is very large");
            ImGui::Text("Known for being faster and smoother in optimization because\nit replaces discrete point-point correspondences with continuous probability density functions.");
            ImGui::EndTooltip();
        }

        ImGui::SameLine();

        if (is_icp_gui) ImGui::PushStyleColor(ImGuiCol_Button, orangeBorder);
        if (ImGui::Button("Iterative Closest Point"))
        {
            if (!is_icp_gui)
            {
                is_ndt_gui = false;
                is_icp_gui = true;
                is_rpf_gui = false;
                justPushed = true;
            }
        }
        if (is_icp_gui && !justPushed) ImGui::PopStyleColor();
        if (ImGui::IsItemHovered())
        {
            ImGui::BeginTooltip();
            ImGui::Text("Geometric registration algorithm that aligns two point clouds\nby minimizing the Euclidean distances between corresponding points");
            ImGui::Text("Very precise at local refinement, especially point-to-plane ICP,\nbut it struggles if the starting alignment is too far off");
            ImGui::EndTooltip();
        }

        ImGui::SameLine();

        if (is_rpf_gui) ImGui::PushStyleColor(ImGuiCol_Button, orangeBorder);
        if (ImGui::Button("Registration Plane Feature"))
        {
            if (!is_rpf_gui)
            {
                is_ndt_gui = false;
                is_icp_gui = false;
                is_rpf_gui = true;
                justPushed = true;
            }
        }
        if (is_rpf_gui && !justPushed) ImGui::PopStyleColor();
        if (ImGui::IsItemHovered())
        {
            ImGui::BeginTooltip();
            ImGui::Text("Feature based registration technique that uses detected\nplanar surfaces in the environment (walls, floors, ceilings, etc.) as constraints for alignment");
            ImGui::Text("Can be much more robust to noise and partial overlap,\nrequire far fewer correspondences (just a few planes can define a full 6 DOF pose),\nhandle low texture regions better than ICP");
            ImGui::EndTooltip();
        }

        ImGui::EndMenuBar();
    }

    if (is_ndt_gui)
        ndt_gui();
    if (is_icp_gui)
        icp_gui();
    if (is_rpf_gui)
        rpf_gui();

    ImGui::End();
}

void pose_graph_slam_gui()
{
    ImGui::Begin("Pose Graph SLAM", &is_pose_graph_slam);

    ImGui::PushItemWidth(ImGuiNumberWidth);
    ImGui::InputFloat("Search radius", &tls_registration.pose_graph_slam.search_radius, 0.01f, 2.0f);
    if (tls_registration.pose_graph_slam.search_radius < 0.01f)
        tls_registration.pose_graph_slam.search_radius = 0.01f;

    ImGui::InputInt("Number of threads", &tls_registration.pose_graph_slam.number_of_threads);
    if (tls_registration.pose_graph_slam.number_of_threads < 1)
        tls_registration.pose_graph_slam.number_of_threads = 1;

    ImGui::InputInt("Number of iterations (pair wise matching)", &tls_registration.pose_graph_slam.number_of_iterations_pair_wise_matching);
    if (tls_registration.pose_graph_slam.number_of_iterations_pair_wise_matching < 1)
        tls_registration.pose_graph_slam.number_of_iterations_pair_wise_matching = 1;

    ImGui::InputFloat("Overlap threshold", &tls_registration.pose_graph_slam.overlap_threshold, 0.1f, 0.8f);
    if (tls_registration.pose_graph_slam.overlap_threshold < 0.1f)
        tls_registration.pose_graph_slam.overlap_threshold = 0.1f;
    ImGui::PopItemWidth();

    // ImGui::Checkbox("pgslam adaptive_robust_kernel", &pose_graph_slam.icp.is_adaptive_robust_kernel);

    //--
    ImGui::Checkbox("Adaptive robust kernel", &tls_registration.pose_graph_slam.is_adaptive_robust_kernel);
    ImGui::SameLine();
    ImGui::Checkbox("Fix first node (add I to first pose in Hessian)", &tls_registration.pose_graph_slam.is_fix_first_node);

     ImGui::Text("Nonlinear optimization method:");
    ImGui::SameLine();
    ImGui::RadioButton("Gauss-Newton", &PGSnomSelection, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Levenberg-Marguardt", &PGSnomSelection, 1);

    tls_registration.pose_graph_slam.is_gauss_newton = (PGSnomSelection == 0);
    tls_registration.pose_graph_slam.is_levenberg_marguardt = (PGSnomSelection == 1);

    ImGui::Text("Poses expressed as:");
    ImGui::SameLine();
    ImGui::RadioButton("camera<-world (cw)", &PGSpeSelection, 0);
    ImGui::SameLine();
    ImGui::RadioButton("camera->world (wc)", &PGSpeSelection, 1);

    tls_registration.pose_graph_slam.is_cw = (PGSpeSelection == 0);
    tls_registration.pose_graph_slam.is_wc = (PGSpeSelection == 1);

    ImGui::Text("Parameterizations of 3D rotation:");
    ImGui::RadioButton("Tait-Bryan angles (om fi ka: RxRyRz)", &PGS3dSelection, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Quaternion (q0 q1 q2 q3)", &PGS3dSelection, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Rodrigues (sx sy sz)", &PGS3dSelection, 2);

    tls_registration.pose_graph_slam.is_tait_bryan_angles = (PGS3dSelection == 0);
    tls_registration.pose_graph_slam.is_quaternion = (PGS3dSelection == 1);
    tls_registration.pose_graph_slam.is_rodrigues = (PGS3dSelection == 2);

    ImGui::Separator();

    ImGui::Text("Method for pair wise matching (general):");
    ImGui::RadioButton("NDT", &PGSpwmtSelection, 0);
    ImGui::RadioButton("Optimization_point_to_point_source_to_target", &PGSpwmtSelection, 1);
    ImGui::RadioButton("Optimize_point_to_projection_onto_plane_source_to_target", &PGSpwmtSelection, 2);
    ImGui::RadioButton("Optimize_point_to_plane_source_to_target", &PGSpwmtSelection, 3);
    ImGui::RadioButton("Optimize_distance_point_to_plane_source_to_target", &PGSpwmtSelection, 4);
    ImGui::RadioButton("Optimize_plane_to_plane_source_to_target", &PGSpwmtSelection, 5);

    ImGui::Separator();

    ImGui::Text("Method for pair wise matching (with Lie-algebra):");
    ImGui::RadioButton("Optimize NDT (Lie-algebra left Jacobian)", &PGSpwmtSelection, 6);
    ImGui::RadioButton("Optimize NDT (Lie-algebra right Jacobian)", &PGSpwmtSelection, 7);
    ImGui::RadioButton("Optimize point to point source to target (Lie-algebra left Jacobian)", &PGSpwmtSelection, 8);
    ImGui::RadioButton("Optimize point to point source to target (Lie-algebra right Jacobian)", &PGSpwmtSelection, 9);
    ImGui::RadioButton("Optimize point to projection onto plane source to target (Lie-algebra left Jacobian)", &PGSpwmtSelection, 10);
    ImGui::RadioButton("Optimize point to projection onto plane source to target (Lie-algebra right Jacobian)", &PGSpwmtSelection, 11);

#ifdef WITH_PCL
    ImGui::Separator();
    ImGui::Text("Method for pair wise matching (with PCL):");
    ImGui::RadioButton("Optimize with PCL (NDT based pair wise matching)", &PGSpwmtSelection, 12);
    ImGui::RadioButton("Optimize with PCL (ICP based pair wise matching)", &PGSpwmtSelection, 13);
#endif

    //tls_registration.pose_graph_slam.set_all_to_false();
    tls_registration.pose_graph_slam.is_ndt = (PGSpwmtSelection == 0);
    tls_registration.pose_graph_slam.is_optimization_point_to_point_source_to_target = (PGSpwmtSelection == 1);
    tls_registration.pose_graph_slam.is_optimize_point_to_projection_onto_plane_source_to_target = (PGSpwmtSelection == 2);
    tls_registration.pose_graph_slam.is_optimize_point_to_plane_source_to_target = (PGSpwmtSelection == 3);
    tls_registration.pose_graph_slam.is_optimize_distance_point_to_plane_source_to_target = (PGSpwmtSelection == 4);
    tls_registration.pose_graph_slam.is_optimize_plane_to_plane_source_to_target = (PGSpwmtSelection == 5);

    tls_registration.pose_graph_slam.is_ndt_lie_algebra_left_jacobian = (PGSpwmtSelection == 6);
    tls_registration.pose_graph_slam.is_ndt_lie_algebra_right_jacobian = (PGSpwmtSelection == 7);
    tls_registration.pose_graph_slam.is_optimize_point_to_point_source_to_target_lie_algebra_left_jacobian = (PGSpwmtSelection == 8);
    tls_registration.pose_graph_slam.is_optimize_point_to_point_source_to_target_lie_algebra_right_jacobian = (PGSpwmtSelection == 9);
    tls_registration.pose_graph_slam.is_optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_left_jacobian = (PGSpwmtSelection == 10);
    tls_registration.pose_graph_slam.is_optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_right_jacobian = (PGSpwmtSelection == 11);

    tls_registration.pose_graph_slam.is_optimize_pcl_ndt = (PGSpwmtSelection == 12);
    tls_registration.pose_graph_slam.is_optimize_pcl_icp = (PGSpwmtSelection == 13);

    if (PGSpwmtSelection >= 0 && PGSpwmtSelection <= 11)
        tls_registration.pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    if (PGSpwmtSelection == 6)
        tls_registration.pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::pcl_ndt;
    if (PGSpwmtSelection == 7)
        tls_registration.pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::pcl_icp;
    
    ImGui::Separator();
    if (ImGui::Button("Optimize"))
    {
        tls_registration.pose_graph_slam.ndt_bucket_size[0] = tls_registration.ndt.bucket_size[0];
        tls_registration.pose_graph_slam.ndt_bucket_size[1] = tls_registration.ndt.bucket_size[1];
        tls_registration.pose_graph_slam.ndt_bucket_size[2] = tls_registration.ndt.bucket_size[2];
        // double rms_initial = 0.0;
        // double rms_final = 0.0;
        // double mui = 0.0;
        tls_registration.pose_graph_slam.optimize(session.point_clouds_container);
        // pose_graph_slam.optimize(point_clouds_container, rms_initial, rms_final, mui);
        // std::cout << "mean uncertainty impact: " << mui << " rms_initial: " << rms_initial << " rms_final: " << rms_final << std::endl;
    }

#if WITH_GTSAM
    if (ImGui::Button("Optimize with GTSAM"))
    {
        ImGui::Separator();
        ImGui::Text("With GTSAM:");
        tls_registration.pose_graph_slam.ndt_bucket_size[0] = tls_registration.ndt.bucket_size[0];
        tls_registration.pose_graph_slam.ndt_bucket_size[1] = tls_registration.ndt.bucket_size[1];
        tls_registration.pose_graph_slam.ndt_bucket_size[2] = tls_registration.ndt.bucket_size[2];
        double rms_initial = 0.0;
        double rms_final = 0.0;
        double mui = 0.0;
        tls_registration.pose_graph_slam.optimize_with_GTSAM(session.point_clouds_container);
        // std::cout << "mean uncertainty impact: " << mui << " rms_initial: " << rms_initial << " rms_final: " << rms_final << std::endl;
    }
#endif

#if WITH_MANIF
    ImGui::Separator();
    ImGui::Text("With MANIF:");
    if (ImGui::Button("Optimize with manif (a small header-only library for Lie theory)"))
    {
        tls_registration.pose_graph_slam.optimize_with_manif(session.point_clouds_container);
        std::cout << "optimize with manif (a small header-only library for Lie theory) DONE" << std::endl;
    }
#endif
    ImGui::End();
}

void observation_picking_gui()
{
    static std::string observations_file_name = "";

    ImGui::Begin("Observations", &is_manual_analisys);

    ImGui::Checkbox("Observation picking mode", &observation_picking.is_observation_picking_mode);
    ImGui::BeginDisabled(!observation_picking.is_observation_picking_mode);
    {
        ImGui::Text("Grid [m]:");
        ImGui::Checkbox("10x10", &observation_picking.grid10x10m);
        ImGui::SameLine();
        ImGui::Checkbox("1x1", &observation_picking.grid1x1m);
        ImGui::SameLine();
        ImGui::Checkbox("0.1x0.1", &observation_picking.grid01x01m);
        ImGui::SameLine();
        ImGui::Checkbox("0.01x0.01", &observation_picking.grid001x001m);

        ImGui::Text("Picking plane:");
        ImGui::PushItemWidth(ImGuiNumberWidth);
        // ImGui::SliderFloat("picking_plane_height", &observation_picking.picking_plane_height, -20.0f, 20.0f);
        ImGui::InputFloat("Height [m]", &observation_picking.picking_plane_height);
        // ImGui::SliderFloat("picking_plane_threshold", &observation_picking.picking_plane_threshold, 0.01f, 200.0f);
        ImGui::InputFloat("Threshold [m]", &observation_picking.picking_plane_threshold);
        // ImGui::SliderFloat("picking_plane_max_xy", &observation_picking.max_xy, 10.0f, 1000.0f);
        ImGui::InputFloat("Grid size [m]", &observation_picking.max_xy);
        // ImGui::SliderInt("point_size", &observation_picking.point_size, 1, 10);
        ImGui::InputInt("Point size", &observation_picking.point_size);
        ImGui::PopItemWidth();
        if (observation_picking.point_size < 1)
            observation_picking.point_size = 1;
        if (observation_picking.point_size > 20)
            observation_picking.point_size = 20;
        ImGui::PopItemWidth();

        if (ImGui::Button("Accept current observation"))
        {
            std::vector<Eigen::Affine3d> m_poses;
            for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
                m_poses.push_back(session.point_clouds_container.point_clouds[i].m_pose);
            observation_picking.accept_current_observation(m_poses);
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear current observation"))
            observation_picking.current_observation.clear();

        if (ImGui::Button("Reset view"))
        {
			new_rotation_center = rotation_center;
            new_rotate_x = 0.0;
            new_rotate_y = 0.0;
            new_translate_x = translate_x;
            new_translate_y = translate_y;
            new_translate_z = translate_z;
            camera_transition_active = true;
        }
    }
    ImGui::EndDisabled();

    ImGui::Text((std::string("Number of observations: ") + std::to_string(observation_picking.observations.size())).c_str());

    if (ImGui::Button("Load observations"))
    {
        std::string input_file_name = "";
        input_file_name = mandeye::fd::OpenFileDialogOneFile("Load observations", {});

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
    if (ImGui::Button("Save observations"))
    {
        const auto output_file_name = mandeye::fd::SaveFileDialog("Save observations", {}, ".json");
        std::cout << "json file to save: '" << output_file_name << "'" << std::endl;

        if (output_file_name.size() > 0)
            observation_picking.export_observation(output_file_name);
    }

    ImGui::Text((std::string("Loaded observations from file: '") + observations_file_name + std::string("'")).c_str());

    if (ImGui::Button("Compute RMS (xy)"))
    {
        double rms = compute_rms(true, session, observation_picking);
        std::cout << "RMS (initial poses): " << rms << std::endl;
        rms = compute_rms(false, session, observation_picking);
        std::cout << "RMS (current poses): " << rms << std::endl;
    }

    ImGui::Separator();
    if (ImGui::Button("Add intersection"))
        observation_picking.add_intersection(Eigen::Vector3d(0.0, 0.0, 0.0));

    int index_intersection_to_remove = -1;
    for (int i = 0; i < observation_picking.intersections.size(); i++)
    {
        ImGui::Separator();
        ImGui::SetWindowFontScale(1.25f);
        ImGui::Text("Intersection %zu", i);
        ImGui::SetWindowFontScale(1.0f);
        ImGui::SameLine();
        ImGui::ColorEdit3(std::string("Color##" + std::to_string(i)).c_str(), observation_picking.intersections[i].color, ImGuiColorEditFlags_NoInputs);
		ImGui::SameLine();
        if (ImGui::Button(std::string("Remove##" + std::to_string(i)).c_str()))
            index_intersection_to_remove = i;

        ImGui::InputFloat3(std::string("Translation [m]##" + std::to_string(i)).c_str(), observation_picking.intersections[i].translation);
        ImGui::InputFloat3(std::string("Rotation [deg]##" + std::to_string(i)).c_str(), observation_picking.intersections[i].rotation);
        ImGui::InputFloat3(std::string("Width length height [m]##" + std::to_string(i)).c_str(), observation_picking.intersections[i].width_length_height);
    }

    if (index_intersection_to_remove != -1)
    {
        std::vector<Intersection> intersections;
        for (int i = 0; i < observation_picking.intersections.size(); i++)
        {
            if (i != index_intersection_to_remove)
                intersections.push_back(observation_picking.intersections[i]);
        }
        observation_picking.intersections = intersections;
    }

    ImGui::Separator();

    ImGui::BeginDisabled(observation_picking.intersections.size() <= 0);
    {
        if (ImGui::Button("Export point clouds inside intersections, RMS and poses"))
        {
            std::string output_folder_name = "";
            output_folder_name = mandeye::fd::SelectFolder("Choose folder");
            std::cout << "folder: '" << output_folder_name << "'" << std::endl;

            if (output_folder_name.size() > 0)
                export_result_to_folder(output_folder_name, observation_picking, session);
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("in RESSO format to folder");
        ImGui::SetNextItemWidth(ImGuiNumberWidth);
        ImGui::InputFloat("Label distance [m]", &observation_picking.label_dist);
    }
    ImGui::EndDisabled();

    ImGui::End();
}

void loop_closure_gui()
{
    ImGui::Begin("Manual Pose Graph Loop Closure", &is_loop_closure_gui);

    ImGui::Text("Num edge extended:");

    ImGui::Text("before: ");
    ImGui::SameLine();
    ImGui::PushItemWidth(ImGuiNumberWidth);
    ImGui::SliderInt("##fs", &num_edge_extended_before, 0, session.point_clouds_container.point_clouds.size() - 1);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("min 0; max %zu", session.point_clouds_container.point_clouds.size() - 1);
    ImGui::SameLine();
    ImGui::InputInt("##fi", &num_edge_extended_before, 1, 5);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("min 0; max %zu", session.point_clouds_container.point_clouds.size() - 1);
    if (num_edge_extended_before < 0)
        num_edge_extended_before = 0;
    if (num_edge_extended_before >= session.point_clouds_container.point_clouds.size() - 1)
        num_edge_extended_before = session.point_clouds_container.point_clouds.size() - 1;

    ImGui::Text(" after: ");
    ImGui::SameLine();

    ImGui::SliderInt("##ts", &num_edge_extended_after, index_loop_closure_target, static_cast<int>(session.point_clouds_container.point_clouds.size() - 1));
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("min 0; max %zu", session.point_clouds_container.point_clouds.size() - 1);
    ImGui::SameLine();
    ImGui::InputInt("##ti", &num_edge_extended_after, 1, 5);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("min 0; max %zu", session.point_clouds_container.point_clouds.size() - 1);
    if (num_edge_extended_after < 0)
        num_edge_extended_after = 0;
    if (num_edge_extended_after >= session.point_clouds_container.point_clouds.size() - 1)
        num_edge_extended_after = session.point_clouds_container.point_clouds.size() - 1;
    ImGui::PopItemWidth();

	int prev_index_active_edge = session.pose_graph_loop_closure.index_active_edge;
    session.pose_graph_loop_closure.Gui(session.point_clouds_container,
        index_loop_closure_source,
        index_loop_closure_target,
        m_gizmo,
        tls_registration.gnss,
        session.ground_control_points,
        session.control_points,
        num_edge_extended_before,
        num_edge_extended_after);

    new_loop_closure_index = (prev_index_active_edge != session.pose_graph_loop_closure.index_active_edge);

    ImGui::End();
}

void lio_segments_gui()
{
    ImGui::Begin("LIO segments editor", &is_lio_segments_gui);

    ImGui::Text("index from: ");
    ImGui::SameLine();
    ImGui::PushItemWidth(ImGuiNumberWidth);
    ImGui::SliderInt("##fs", &index_begin, 0, index_end);
    ImGui::SameLine();
    ImGui::InputInt("##fi", &index_begin, 1, 5);
    if (index_begin < 0)
        index_begin = 0;
    if (index_begin >= index_end)
        index_begin = index_end;

    ImGui::SameLine();
    ImGui::Text(" to: ");
    ImGui::SameLine();

    ImGui::SliderInt("##ts", &index_end, index_begin, static_cast<int>(session.point_clouds_container.point_clouds.size() - 1));
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("max %zu", session.point_clouds_container.point_clouds.size() - 1);
    ImGui::SameLine();
    ImGui::InputInt("##ti", &index_end, 1, 5);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("max %zu", session.point_clouds_container.point_clouds.size() - 1);    
    if (index_end < index_begin)
        index_end = index_begin;
    if (index_end >= session.point_clouds_container.point_clouds.size() - 1)
        index_end = session.point_clouds_container.point_clouds.size() - 1;
    ImGui::PopItemWidth();

    ImGui::Text("Selection: ");
    ImGui::SameLine();
    if (ImGui::Button("show"))
        session.point_clouds_container.show_all_from_range(index_begin, index_end);
    ImGui::SameLine();
    if (ImGui::Button("shift -"))
    {
        int step = index_end - index_begin;
        index_begin -= step;
        index_end -= step;

        if (index_begin < 0)
            index_begin = 0;
        if (index_end < 0)
            index_end = 0;

        rotation_center.x() = session.point_clouds_container.point_clouds[index_begin].m_pose(0, 3);
        rotation_center.y() = session.point_clouds_container.point_clouds[index_begin].m_pose(1, 3);
        rotation_center.z() = session.point_clouds_container.point_clouds[index_begin].m_pose(2, 3);
        session.point_clouds_container.show_all_from_range(index_begin, index_end);
    }
    ImGui::SameLine();
    if (ImGui::Button("shift +"))
    {
        int step = index_end - index_begin;
        index_begin += step;
        index_end += step;

        if (index_begin > session.point_clouds_container.point_clouds.size() - 1)
            index_begin = session.point_clouds_container.point_clouds.size() - 1;
        if (index_end > session.point_clouds_container.point_clouds.size() - 1)
            index_end = session.point_clouds_container.point_clouds.size() - 1;

        rotation_center.x() = session.point_clouds_container.point_clouds[index_begin].m_pose(0, 3);
        rotation_center.y() = session.point_clouds_container.point_clouds[index_begin].m_pose(1, 3);
        rotation_center.z() = session.point_clouds_container.point_clouds[index_begin].m_pose(2, 3);
        session.point_clouds_container.show_all_from_range(index_begin, index_end);
    }
    ImGui::SameLine();
    if (ImGui::Button("Show all"))
        session.point_clouds_container.show_all();
    ImGui::SameLine();
    if (ImGui::Button("Hide all"))
        session.point_clouds_container.hide_all();
    ImGui::SameLine();
    if (ImGui::Button("Reset poses"))
        reset_poses(session);

    ImGui::Checkbox("Show with initial pose", &session.point_clouds_container.show_with_initial_pose);
    ImGui::SameLine();
    ImGui::Checkbox("Manipulate only marked gizmo", &manipulate_only_marked_gizmo);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("false: move also succesive nodes");

    ImGui::Text("Fuse IMU inclination: ");
    ImGui::SameLine();

    static double angle_diff = 5.0;

    if (ImGui::Button("Set those that sattisfy acceptable angle"))
    {
        for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
        {
            double om = session.point_clouds_container.point_clouds[i].local_trajectory[0].imu_om_fi_ka.x() * 180.0 / M_PI;
            double fi = session.point_clouds_container.point_clouds[i].local_trajectory[0].imu_om_fi_ka.y() * 180.0 / M_PI;

            std::cout << "om: " << om << " fi " << fi << std::endl;
            if (fabs(om) > angle_diff || fabs(fi) > angle_diff)
            {
            }
            else
                session.point_clouds_container.point_clouds[i].fuse_inclination_from_IMU = true;
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("unset all"))
    {
        for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
            session.point_clouds_container.point_clouds[i].fuse_inclination_from_IMU = false;
    }

    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGuiNumberWidth);
    ImGui::InputDouble("acceptable angle [deg]", &angle_diff);

    ImGui::Separator();
    //ImGui::Text("motion model");

    //session.pose_graph_loop_closure.edges.

    //ImGui::InputDouble("motion_model_w_px_1_sigma_m", &session.pose_graph_loop_closure.motion_model_w_px_1_sigma_m);
    //ImGui::InputDouble("motion_model_w_py_1_sigma_m", &session.pose_graph_loop_closure.motion_model_w_py_1_sigma_m);
    //ImGui::InputDouble("motion_model_w_pz_1_sigma_m", &session.pose_graph_loop_closure.motion_model_w_pz_1_sigma_m);
    //ImGui::InputDouble("motion_model_w_om_1_sigma_deg", &session.pose_graph_loop_closure.motion_model_w_om_1_sigma_deg);
    //ImGui::InputDouble("motion_model_w_fi_1_sigma_deg", &session.pose_graph_loop_closure.motion_model_w_fi_1_sigma_deg);
    //ImGui::InputDouble("motion_model_w_ka_1_sigma_deg", &session.pose_graph_loop_closure.motion_model_w_ka_1_sigma_deg);

    //ImGui::Separator();

    ImGui::BeginChild("LIO segments", ImVec2(0, 0), true);
    {
        for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
        {
            if (i > 0)
                ImGui::Separator();
            ImGui::SetWindowFontScale(1.25f);
            ImGui::Checkbox(std::filesystem::path(session.point_clouds_container.point_clouds[i].file_name).filename().string().c_str(), &session.point_clouds_container.point_clouds[i].visible);
            ImGui::SetWindowFontScale(1.0f);
            ImGui::SameLine();
            ImGui::Checkbox(("gizmo##" + std::to_string(i)).c_str(), &session.point_clouds_container.point_clouds[i].gizmo);

    #if 0
            ImGui::SameLine();
            ImGui::Checkbox(("fixed##" + std::to_string(i)).c_str(), &session.point_clouds_container.point_clouds[i].fixed);
            ImGui::SameLine();
            ImGui::PushButtonRepeat(true);
            float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
            if (ImGui::ArrowButton(("left##" + std::to_string(i)).c_str(), ImGuiDir_Left))
            {
                (session.point_clouds_container.point_clouds[i].point_size)--;
            }
            ImGui::SameLine(0.0f, spacing);
            if (ImGui::ArrowButton(("right##" + std::to_string(i)).c_str(), ImGuiDir_Right))
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
    #endif
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
                ImGui::SameLine();
                ImGui::ColorEdit3(("pc_color##" + std::to_string(i)).c_str(), session.point_clouds_container.point_clouds[i].render_color, ImGuiColorEditFlags_NoInputs);

    #if 0
                ImGui::SameLine();
                if (ImGui::Button(std::string(("ICP##" + std::to_string(i)).c_str())
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
                    for (auto& pc : pcs.point_clouds)
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
    #endif

                ImGui::SameLine();
                ImGui::Checkbox(("fuse IMU inclination##" + std::to_string(i)).c_str(), &session.point_clouds_container.point_clouds[i].fuse_inclination_from_IMU);

                ImGui::Text("fixed: ");

                ImGui::SameLine();
                ImGui::Checkbox(("X##" + std::to_string(i)).c_str(), &session.point_clouds_container.point_clouds[i].fixed_x);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(xText);

                ImGui::SameLine();
                ImGui::Checkbox(("Y##" + std::to_string(i)).c_str(), &session.point_clouds_container.point_clouds[i].fixed_y);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(yText);

                ImGui::SameLine();
                ImGui::Checkbox(("Z##" + std::to_string(i)).c_str(), &session.point_clouds_container.point_clouds[i].fixed_z);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(zText);

                ImGui::SameLine();
                ImGui::Checkbox(("om##" + std::to_string(i)).c_str(), &session.point_clouds_container.point_clouds[i].fixed_om);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(omText);

                ImGui::SameLine();
                ImGui::Checkbox(("fi##" + std::to_string(i)).c_str(), &session.point_clouds_container.point_clouds[i].fixed_fi);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(fiText);

                ImGui::SameLine();
                ImGui::Checkbox(("ka##" + std::to_string(i)).c_str(), &session.point_clouds_container.point_clouds[i].fixed_ka);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(kaText);

            }
    #if 0
            ImGui::SameLine();
            if (ImGui::Button(std::string("#" + std::to_string(i) + " print frame to console").c_str()))
            {
                std::cout << session.point_clouds_container.point_clouds[i].m_pose.matrix() << std::endl;
            }
    #endif
        }

    }
    ImGui::EndChild();

    ImGui::End();
}

void openSession()
{
    session_file_name = mandeye::fd::OpenFileDialogOneFile("Open session", mandeye::fd::Session_filter);
    std::cout << "Session file: '" << session_file_name << "'" << std::endl;

    if (session_file_name.size() > 0)
    {
        if (session.load(fs::path(session_file_name).string(), tls_registration.is_decimate, tls_registration.bucket_x, tls_registration.bucket_y, tls_registration.bucket_z, tls_registration.calculate_offset))
        {
            session_loaded = true;
            index_begin = 0;
            index_end = session.point_clouds_container.point_clouds.size() - 1;

            std::string newTitle = winTitle + " - " + truncPath(session_file_name);
            glutSetWindowTitle(newTitle.c_str());

            for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
            {
                session_total_number_of_points += session.point_clouds_container.point_clouds[i].points_local.size();
            }

            session_dims = session.point_clouds_container.compute_point_cloud_dimension();
        }
    }
}

void saveSession()
{
    const auto output_file_name = mandeye::fd::SaveFileDialog("Save session", mandeye::fd::Session_filter, ".json", session_file_name);
    std::cout << "Session file to save: '" << output_file_name << "'" << std::endl;

    if (output_file_name.size() > 0)
    {
        if (session.point_clouds_container.initial_poses_file_name.empty())
        {
            std::cout << "Please assign initial_poses_file_name to session" << std::endl;
            std::cout << "Session is not saved" << std::endl;

            [[maybe_unused]] pfd::message message(
                "Please assign initial_poses_file_name to session",
                "Session is not saved. Please assign initial_poses_file_name to session. "
                "Follow guidlines available here : "
                "https://github.com/MapsHD/HDMapping/tree/main/doc/, "
                "You can do this using button 'update initial poses from RESSO file'",
                pfd::choice::ok, pfd::icon::error);
            message.result();

            std::shared_ptr<pfd::save_file> save_file2;
            std::string initial_poses_file_name = "";
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file2);
            const auto tt = [&]()
                {
                    auto sel = pfd::save_file("initial_poses_file_name", "C:\\", mandeye::fd::Resso_filter).result();
                    initial_poses_file_name = sel;
                    std::cout << "Resso file to save: '" << initial_poses_file_name << "'" << std::endl;
                };
            std::thread t2(tt);
            t2.join();

            if (initial_poses_file_name.size() > 0)
            {
                std::cout << "saving initial poses to: " << initial_poses_file_name << std::endl;
                session.point_clouds_container.save_poses(fs::path(initial_poses_file_name).string(), false);
            }
        }

        if (session.point_clouds_container.poses_file_name.empty())
        {
            std::cout << "Please assign poses_file_name to session" << std::endl;
            std::cout << "Session is not saved" << std::endl;

            [[maybe_unused]] pfd::message message(
                "Please assign poses_file_name to session",
                "Session is not saved. Please assign poses_file_name to session. "
                "Follow guidlines available here : "
                "https://github.com/MapsHD/HDMapping/tree/main/doc/,"
                "You can do this using button 'update poses from RESSO file'",
                pfd::choice::ok, pfd::icon::error);
            message.result();

            const auto poses_file_name = mandeye::fd::SaveFileDialog("poses_file_name", mandeye::fd::Resso_filter);
            std::cout << "Resso file to save: '" << poses_file_name << "'" << std::endl;
            if (poses_file_name.size() > 0)
            {
                std::cout << "saving poses to: " << poses_file_name << std::endl;
                session.point_clouds_container.save_poses(fs::path(poses_file_name).string(), false);
            }
        }

        session.save(fs::path(output_file_name).string(), session.point_clouds_container.poses_file_name, session.point_clouds_container.initial_poses_file_name, false);
        std::cout << "saving result to: " << session.point_clouds_container.poses_file_name << std::endl;
        session.point_clouds_container.save_poses(fs::path(session.point_clouds_container.poses_file_name).string(), false);
    }
}

void saveSubsession()
{
    //creating filename proposal based on current selection
    std::filesystem::path path(session_file_name);
    // Extract parts
    auto dir = path.parent_path();
    auto stem = path.stem().string();
    auto ext = path.extension().string();
    std::string indexpart = " " + std::to_string(index_begin) + "-" + std::to_string(index_end);

    // Build new name
    std::string indexed_file_name = (dir / (stem + indexpart + ext)).string();

    const auto output_file_name = mandeye::fd::SaveFileDialog("Save subsession", mandeye::fd::Session_filter, ".json", indexed_file_name);
    std::cout << "Subsession file to save: '" << output_file_name << "'" << std::endl;

    if (output_file_name.size() > 0)
    {
        const auto initial_poses_file_name = (dir / ("lio_initial_poses" + indexpart + ".reg")).string();
        const auto poses_file_name = (dir / ("poses" + indexpart + ".reg")).string();

        session.save(fs::path(output_file_name).string(), poses_file_name, initial_poses_file_name, true);
        std::cout << "Saving initial poses to: " << initial_poses_file_name << std::endl;
        session.point_clouds_container.save_poses(fs::path(initial_poses_file_name).string(), true);
        std::cout << "Saving poses to: " << poses_file_name << std::endl;
        session.point_clouds_container.save_poses(fs::path(poses_file_name).string(), true);
    }
}

void display()
{
    ImGuiIO &io = ImGui::GetIO();

    view_kbd_shortcuts();

    if (io.KeyCtrl && ImGui::IsKeyPressed('A'))
		is_pca_gui = !is_pca_gui;
    if (io.KeyCtrl && ImGui::IsKeyPressed('C'))
    {
        if (session.control_points.is_imgui)
			session.control_points.is_imgui = false;
        else if (!session.ground_control_points.is_imgui)
            session.control_points.is_imgui = true;
    }
    if (io.KeyCtrl && ImGui::IsKeyPressed('E'))
    {
        if (is_lio_segments_gui)
            is_lio_segments_gui = false;
        else if (!is_loop_closure_gui)
            is_lio_segments_gui = true;
    }

    if (io.KeyCtrl && ImGui::IsKeyPressed('G'))
    {
        if (session.ground_control_points.is_imgui)
            session.ground_control_points.is_imgui = false;
        else if (!session.control_points.is_imgui)
            session.ground_control_points.is_imgui = true;
    }

    if (io.KeyCtrl && ImGui::IsKeyPressed('L'))
    {
        if (is_loop_closure_gui)
            is_loop_closure_gui = false;
        else if (!is_lio_segments_gui)
            is_loop_closure_gui = true;
    }

    if (io.KeyCtrl && ImGui::IsKeyPressed('O', false))
        openSession();
    if (io.KeyCtrl && ImGui::IsKeyPressed('P'))
        is_pose_graph_slam = !is_pose_graph_slam;

    if (io.KeyCtrl && ImGui::IsKeyPressed('R')) //random colors
    {
        for (auto& pc : session.point_clouds_container.point_clouds)
        {
            pc.render_color[0] = float(rand() % 255) / 255.0f;
            pc.render_color[1] = float(rand() % 255) / 255.0f;
            pc.render_color[2] = float(rand() % 255) / 255.0f;
            pc.show_color = false;
        }
    }

    if (io.KeyCtrl && ImGui::IsKeyPressed('S', false))
        saveSession();
    if (io.KeyCtrl && io.KeyShift && ImGui::IsKeyPressed('S', false))
        saveSubsession();

    updateCameraTransition();

    glViewport(0, 0, (GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);

    Eigen::Affine3f viewLocal = Eigen::Affine3f::Identity();

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

        // janusz
        if (is_loop_closure_gui)
        {
            if (new_loop_closure_index)
            {
                //if (index_loop_closure_source < session.point_clouds_container.point_clouds.size())
                //{
                //    new_rotation_center.x() = session.point_clouds_container.point_clouds[index_loop_closure_source].m_pose.translation().x();
                //    new_rotation_center.y() = session.point_clouds_container.point_clouds[index_loop_closure_source].m_pose.translation().y();
                //    new_rotation_center.z() = session.point_clouds_container.point_clouds[index_loop_closure_source].m_pose.translation().z();
                //
                //    new_translate_x = -new_rotation_center.x();
                //    new_translate_y = -new_rotation_center.y();
                //    camera_transition_active = true;
                //}

                if (session.pose_graph_loop_closure.manipulate_active_edge) {
                    if (session.pose_graph_loop_closure.edges.size() > 0)
                    {
                        if (session.pose_graph_loop_closure.index_active_edge < session.pose_graph_loop_closure.edges.size())
                        {
                            new_rotation_center.x() = session.point_clouds_container.point_clouds[session.pose_graph_loop_closure.edges[session.pose_graph_loop_closure.index_active_edge].index_from].m_pose.translation().x();
                            new_rotation_center.y() = session.point_clouds_container.point_clouds[session.pose_graph_loop_closure.edges[session.pose_graph_loop_closure.index_active_edge].index_from].m_pose.translation().y();
                            new_rotation_center.z() = session.point_clouds_container.point_clouds[session.pose_graph_loop_closure.edges[session.pose_graph_loop_closure.index_active_edge].index_from].m_pose.translation().z();
                        }
                    }

                    new_rotate_x = rotate_x;
                    new_rotate_y = rotate_y;
                    new_translate_x = -new_rotation_center.x();
                    new_translate_y = -new_rotation_center.y();
                    new_translate_z = translate_z;
                    camera_transition_active = true;
                }

				new_loop_closure_index = false;
            }
        }

        viewTranslation.translate(rotation_center);

        viewLocal.translate(Eigen::Vector3f(translate_x, translate_y, translate_z));

        if (!block_z)
        {
            viewLocal.rotate(Eigen::AngleAxisf(M_PI * rotate_x / 180.f, Eigen::Vector3f::UnitX()));
            viewLocal.rotate(Eigen::AngleAxisf(M_PI * rotate_y / 180.f, Eigen::Vector3f::UnitZ()));
        }
        else
        {
            viewLocal.rotate(Eigen::AngleAxisf(-90.0 * M_PI / 180.f, Eigen::Vector3f::UnitX()));
            viewLocal.rotate(Eigen::AngleAxisf(M_PI * rotate_y / 180.f, Eigen::Vector3f::UnitZ()));
        }

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

    if (session.control_points.is_imgui)
        session.control_points.render(session.point_clouds_container);
    else
    {
        if (is_loop_closure_gui)
            session.pose_graph_loop_closure.Render(session.point_clouds_container, index_loop_closure_source, index_loop_closure_target,
                                                   num_edge_extended_before, num_edge_extended_after);

        tls_registration.gnss.render(session.point_clouds_container);
        session.ground_control_points.render(session.point_clouds_container);
    }

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();

    if (ImGui::BeginMainMenuBar())
    {
        if (!session_loaded)
        {
            if (ImGui::Button("Load session"))
				openSession();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Select session to open (Ctrl+O)");

            ImGui::SameLine();
            ImGui::Dummy(ImVec2(20, 0));
            ImGui::SameLine();
        }
        else
        {
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("Save session", "Ctrl+S"))
                    saveSession();
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("last step in linear workflow");

                ImGui::BeginDisabled(!((index_begin > 0) || (index_end < static_cast<int>(session.point_clouds_container.point_clouds.size() - 1))));
                {
                    if (ImGui::MenuItem("Save subsession", "Ctrl+Shift+S"))
                        saveSubsession();
                }
                ImGui::EndDisabled();

                ImGui::Separator();

                if (ImGui::BeginMenu("Save all marked scans"))
                {
                    if (ImGui::MenuItem("Local scan"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                        std::cout << "laz file to save: '" << output_file_name << "'" << std::endl;

                        if (output_file_name.size() > 0)
                            save_all_to_las(session, output_file_name, true);
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("As one local scan transformed via inverse pose of first scan");

                    if (ImGui::MenuItem("Global scan"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                        std::cout << "laz file to save: '" << output_file_name << "'" << std::endl;

                        if (output_file_name.size() > 0)
                            save_all_to_las(session, output_file_name, false);
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("To export in full resolution, close the program and open again, unmark 'simple_gui', unmark 'decimate during load'");

                    if (ImGui::MenuItem("Separate global scans (laz)"))
                    {
                        std::string output_folder_name_separately = "";
                        output_folder_name_separately = mandeye::fd::SelectFolder("Choose folder");
                        save_separately_to_las(session, output_folder_name_separately, ".laz");
                    }

                    if (ImGui::MenuItem("Separate global scans (las)"))
                    {
                        std::string output_folder_name_separately = "";
                        output_folder_name_separately = mandeye::fd::SelectFolder("Choose folder");
                        save_separately_to_las(session, output_folder_name_separately, ".las");
                    }

                    ImGui::EndMenu();
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Save all marked scans as las/laz files");

                if (ImGui::BeginMenu("Save all marked trajectories"))
                {
                    ImGui::MenuItem("is_trajectory_export_downsampling", nullptr, &tls_registration.is_trajectory_export_downsampling);
                    ImGui::PushItemWidth(ImGuiNumberWidth);
                    ImGui::InputFloat("curve_consecutive_distance [m]", &tls_registration.curve_consecutive_distance_meters);
                    ImGui::InputFloat("not_curve_consecutive_distance [m]", &tls_registration.not_curve_consecutive_distance_meters);
                    ImGui::PopItemWidth();

                    if (ImGui::MenuItem("Save all as las/laz files"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                        std::cout << "laz file to save: '" << output_file_name << "'" << std::endl;
                        if (output_file_name.size() > 0)
                            save_trajectories_to_laz(session, output_file_name, tls_registration.curve_consecutive_distance_meters, tls_registration.not_curve_consecutive_distance_meters, tls_registration.is_trajectory_export_downsampling);
                    }

                    ImGui::Separator();

                    ImGui::Text("(x,y,z,r00,r01,r02,r10,r11,r12,r20,r21,r22)");
                    if (ImGui::MenuItem("Save all as csv (timestamp Lidar)"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::Csv_filter, ".csv");
                        std::cout << "csv file to save: '" << output_file_name << "'" << std::endl;

                        if (output_file_name.size() > 0)
                            save_trajectories(session, output_file_name, tls_registration.curve_consecutive_distance_meters, tls_registration.not_curve_consecutive_distance_meters, tls_registration.is_trajectory_export_downsampling, true, false, false, false);
                    }
                    if (ImGui::MenuItem("Save all as csv (timestamp Unix)"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::Csv_filter, ".csv");
                        std::cout << "csv file to save: '" << output_file_name << "'" << std::endl;

                        if (output_file_name.size() > 0)
                            save_trajectories(session, output_file_name, tls_registration.curve_consecutive_distance_meters, tls_registration.not_curve_consecutive_distance_meters, tls_registration.is_trajectory_export_downsampling, false, true, false, false);
                    }
                    if (ImGui::MenuItem("Save all as csv (timestamp Lidar, Unix)"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::Csv_filter, ".csv");
                        std::cout << "csv file to save: '" << output_file_name << "'" << std::endl;

                        if (output_file_name.size() > 0)
                            save_trajectories(session, output_file_name, tls_registration.curve_consecutive_distance_meters, tls_registration.not_curve_consecutive_distance_meters, tls_registration.is_trajectory_export_downsampling, true, true, false, false);
                    }

                    ImGui::Separator();
                    ImGui::Text("(x,y,z,qx,qy,qz,qw)");

                    if (ImGui::MenuItem("Save all as csv (timestamp Lidar)"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::Csv_filter, ".csv");
                        std::cout << "csv file to save: '" << output_file_name << "'" << std::endl;

                        if (output_file_name.size() > 0)
                            save_trajectories(session, output_file_name, tls_registration.curve_consecutive_distance_meters, tls_registration.not_curve_consecutive_distance_meters, tls_registration.is_trajectory_export_downsampling, true, false, true, false);
                    }
                    if (ImGui::MenuItem("Save all as csv (timestamp Unix)"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::Csv_filter, ".csv");
                        std::cout << "csv file to save: '" << output_file_name << "'" << std::endl;

                        if (output_file_name.size() > 0)
                            save_trajectories(session, output_file_name, tls_registration.curve_consecutive_distance_meters, tls_registration.not_curve_consecutive_distance_meters, tls_registration.is_trajectory_export_downsampling, false, true, true, false);
                    }
                    if (ImGui::MenuItem("Save all as csv (timestamp Lidar, Unix)"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::Csv_filter, ".csv");
                        std::cout << "csv file to save: '" << output_file_name << "'" << std::endl;

                        if (output_file_name.size() > 0)
                            save_trajectories(session, output_file_name, tls_registration.curve_consecutive_distance_meters, tls_registration.not_curve_consecutive_distance_meters, tls_registration.is_trajectory_export_downsampling, true, true, true, false);
                    }

                    ImGui::Separator();

                    if (ImGui::MenuItem("Save all as dxf as polyline"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog("Ouput file name", mandeye::fd::Dxf_filter, ".dxf");
                        std::cout << "dxf file to save: '" << output_file_name << "'" << std::endl;

                        if (output_file_name.size() > 0)
                            save_trajectories(session, output_file_name, tls_registration.curve_consecutive_distance_meters, tls_registration.not_curve_consecutive_distance_meters, tls_registration.is_trajectory_export_downsampling, false, false, false, true);
                    }

                    ImGui::EndMenu();
                }

                if (ImGui::BeginMenu("Save scale board"))
                {
                    ImGui::Text("For all marked trajectories as one global scan to laz");

                    if (ImGui::MenuItem("> dec 0.1"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                        std::cout << "laz file to save: '" << output_file_name << "'" << std::endl;

                        if (output_file_name.size() > 0)
                            save_scale_board_to_laz(session, output_file_name, 0.1);
                    }

                    if (ImGui::MenuItem("> dec 1.0"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                        std::cout << "laz file to save: '" << output_file_name << "'" << std::endl;

                        if (output_file_name.size() > 0)
                            save_scale_board_to_laz(session, output_file_name, 1.0);
                    }

                    if (ImGui::MenuItem("> dec 10.0"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                        std::cout << "laz file to save: '" << output_file_name << "'" << std::endl;

                        if (output_file_name.size() > 0)
                            save_scale_board_to_laz(session, output_file_name, 10.0);
                    }

                    ImGui::Separator();
                    ImGui::Text("10km x 10km to laz");

                    if (ImGui::MenuItem("> 10m"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                        std::cout << "laz file to save: '" << output_file_name << "'" << std::endl;

                        if (output_file_name.size() > 0)
                            save_scale_board_to_laz(session, output_file_name, 10.0, 10000.0);
                    }

                    if (ImGui::MenuItem("> 100m"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                        std::cout << "laz file to save: '" << output_file_name << "'" << std::endl;

                        if (output_file_name.size() > 0)
                            save_scale_board_to_laz(session, output_file_name, 100.0, 10000.0);
                    }

                    if (ImGui::MenuItem("> 1000m"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                        std::cout << "laz file to save: '" << output_file_name << "'" << std::endl;

                        if (output_file_name.size() > 0)
                            save_scale_board_to_laz(session, output_file_name, 1000.0, 10000.0);
                    }

                    ImGui::EndMenu();
                }

                if (ImGui::BeginMenu("GNSS"))
                {
                    ImGui::MenuItem("Load with offset", nullptr, &gnssWithOffset);
                    
                    if (ImGui::MenuItem("Load GNSS files and convert WGS84 to PUWG92"))
                    {
                        std::vector<std::string> input_file_names;
                        input_file_names = mandeye::fd::OpenFileDialog("Load gnss files", { "GNSS", "*.gnss" }, true);

                        if (input_file_names.size() > 0)
                        {
                            if (!tls_registration.gnss.load(input_file_names, gnssWithOffset))
                                std::cout << "problem with loading gnss files" << std::endl;
                        }
                    }

                    ImGui::MenuItem("Set WGS84 reference from 1st pose", nullptr, &tls_registration.gnss.setWGS84ReferenceFromFirstPose);

                    ImGui::Text("Load & convert WGS84 to Cartesian by Mercator projection");

                    if (ImGui::MenuItem("Load GNSS"))
                    {
                        std::vector<std::string> input_file_names;
                        input_file_names = mandeye::fd::OpenFileDialog("Load gnss files", { "GNSS", "*.gnss" }, true);

                        if (input_file_names.size() > 0)
                        {
                            if (!tls_registration.gnss.load_mercator_projection(input_file_names))
                                std::cout << "problem with loading gnss files" << std::endl;
                        }
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Load structured GNSS dataset and decode it into coordinates");

                    if (ImGui::MenuItem("Load NMEA"))
                    {
                        std::vector<std::string> input_file_names;
                        input_file_names = mandeye::fd::OpenFileDialog("Load nmea files", { "NMEA", "*.nmea" }, true);

                        if (input_file_names.size() > 0)
                        {
                            if (!tls_registration.gnss.load_nmea_mercator_projection(input_file_names))
                                std::cout << "problem with loading gnss files" << std::endl;
                        }
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Load raw GNSS serial output and decode it into coordinates");

                    ImGui::Separator();

                    if (ImGui::MenuItem("Save GNSS data to las/laz file"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog("Save las or laz file", mandeye::fd::LAS_LAZ_filter, ".laz");
                        std::cout << "las or laz file to save: '" << output_file_name << "'" << std::endl;

                        if (output_file_name.size() > 0)
                            tls_registration.gnss.save_to_laz(output_file_name, session.point_clouds_container.offset.x(), session.point_clouds_container.offset.y(), session.point_clouds_container.offset.z());
                    }

                    if (ImGui::MenuItem("Save metascan points in PUWG92"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                        std::vector<Eigen::Vector3d> pointcloud;
                        std::vector<unsigned short> intensity;
                        std::vector<double> timestamps;

                        for (auto& p : session.point_clouds_container.point_clouds)
                        {
                            if (p.visible)
                            {
                                for (int i = 0; i < p.points_local.size(); i++)
                                {
                                    const auto& pp = p.points_local[i];
                                    Eigen::Vector3d vp;
                                    vp = p.m_pose * pp;

                                    pointcloud.push_back(vp);
                                    if (i < p.intensities.size())
                                        intensity.push_back(p.intensities[i]);
                                    else
                                        intensity.push_back(0);
                                    if (i < p.timestamps.size())
                                        timestamps.push_back(p.timestamps[i]);
                                }
                            }
                        }

                        const auto lat = tls_registration.gnss.WGS84ReferenceLatitude;
                        const auto lon = tls_registration.gnss.WGS84ReferenceLongitude;
                        const auto alt = tls_registration.gnss.gnss_poses[0].alt;

                        double Xpuwg92 = 0.0;
                        double Ypuwg92 = 0.0;
                        wgs84_do_puwg92(lat, lon, &Xpuwg92, &Ypuwg92);
                        Eigen::Vector3d offset(Ypuwg92, Xpuwg92, alt);
                        exportLaz(output_file_name, pointcloud, intensity, timestamps, offset.x(), offset.y(), offset.z());
                    }

                    ImGui::EndMenu();
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("GNSS (GPS, etc.) related open/save commands");

                ImGui::EndMenu();
            }
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                ImGui::Text("Loaded session:");
                ImGui::Text(std::string(session.session_file_name).c_str());
                ImGui::Separator();
                ImGui::Text("Total number of points: %zu", session_total_number_of_points);

                if (ImGui::BeginTable("Dimensions", 4))
                {
                    ImGui::TableSetupColumn("Coord [m]");
                    ImGui::TableSetupColumn("min");
                    ImGui::TableSetupColumn("max");
                    ImGui::TableSetupColumn("size");
                    ImGui::TableHeadersRow();

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);

                    
                    std::string text = "X";
                    float centered = ImGui::GetColumnWidth() - ImGui::CalcTextSize(text.c_str()).x;
                    // Set cursor so text is centered
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + centered * 0.5f);

                    ImGui::Text("X");

                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%.3f", session_dims.x_min);
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%.3f", session_dims.x_max);
                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("%.3f", session_dims.length);

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + centered * 0.5f);
                    ImGui::Text("Y");

                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%.3f", session_dims.y_min);
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%.3f", session_dims.y_max);
                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("%.3f", session_dims.width);

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + centered * 0.5f);
                    ImGui::Text("Z");

                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%.3f", session_dims.z_min);
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%.3f", session_dims.z_max);
                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("%.3f", session_dims.height);

                    ImGui::EndTable();
                }

                ImGui::EndTooltip();
            }
            

            if (ImGui::BeginMenu("Tools"))
            {
                ImGui::MenuItem("Point Cloud Alignment", "Ctrl+A", &is_pca_gui);
                if (ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    ImGui::Text("Point cloud alignment (registration) algorithms:");
                    ImGui::Text("(aligning two 3D point sets from LiDAR scans, by estimating\nthe relative pose(translation and rotation) between them)");
                    ImGui::Text("- Normal Distributions Transform");
                    ImGui::Text("- Iterative Closest Point");
                    ImGui::Text("- Registration Plane Feature");
                    ImGui::EndTooltip();
                }
				ImGui::Separator();
                ImGui::MenuItem("Pose Graph SLAM", "Ctrl+P", &is_pose_graph_slam);
                ImGui::MenuItem("Manual Analysis", nullptr, &is_manual_analisys);


                ImGui::Separator();
                
                if (ImGui::MenuItem("Control Points", "Ctrl+C", &session.control_points.is_imgui, !session.ground_control_points.is_imgui))
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Known reference points used to align or verify the scan");

                if (ImGui::MenuItem("Ground Control Points", "Ctrl+G", &session.ground_control_points.is_imgui, !session.control_points.is_imgui))
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Accurate real-world points used to georeference the scan");

                ImGui::MenuItem("Manual Loop Closure", "Ctrl+L", &is_loop_closure_gui, !is_lio_segments_gui);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Manually connect overlapping scan sections");
                
                ImGui::Separator();
                ImGui::MenuItem("LIO segments editor", "Ctrl+E", &is_lio_segments_gui, !is_loop_closure_gui);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Manually adjust or review Lidar Inertial Odometry trajectory segments");

                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Intersections")) {
                ImGui::SetNextItemWidth(ImGuiNumberWidth);
                ImGui::InputDouble("Intersection width [m]", &session.point_clouds_container.intersection_width, 0.0, 0.0, "%.2f");
                if (session.point_clouds_container.intersection_width < 0.001)
                    session.point_clouds_container.intersection_width = 0.001;

                ImGui::Separator();

                ImGui::MenuItem("xz_intersection", nullptr, &session.point_clouds_container.xz_intersection);
                ImGui::MenuItem("10m grid##xz", nullptr, &session.point_clouds_container.xz_grid_10x10);
                ImGui::MenuItem("1m grid##xz", nullptr, &session.point_clouds_container.xz_grid_1x1);
                ImGui::MenuItem("0.1m grid##xz", nullptr, &session.point_clouds_container.xz_grid_01x01);
                
                if (ImGui::MenuItem("Export xz intersection", nullptr, false, session.point_clouds_container.xz_intersection))
                {
                    const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                    std::cout << "laz file to save: '" << output_file_name << "'" << std::endl;

                    if (output_file_name.size() > 0)
                    {
                        save_intersection(session, output_file_name,
                            session.point_clouds_container.xz_intersection, session.point_clouds_container.yz_intersection, session.point_clouds_container.xy_intersection,
                            session.point_clouds_container.intersection_width);
                    }
                }

                ImGui::Separator();

                ImGui::MenuItem("yz_intersection", nullptr, &session.point_clouds_container.yz_intersection);
                ImGui::MenuItem("10m grid##yz", nullptr, &session.point_clouds_container.yz_grid_10x10);
                ImGui::MenuItem("1m grid##yz", nullptr, &session.point_clouds_container.yz_grid_1x1);
                ImGui::MenuItem("0.1m grid##yz", nullptr, &session.point_clouds_container.yz_grid_01x01);
                
                if (ImGui::MenuItem("Export yz intersection", nullptr, false, session.point_clouds_container.yz_intersection))
                {
                    const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                    std::cout << "laz file to save: '" << output_file_name << "'" << std::endl;

                    if (output_file_name.size() > 0)
                    {
                        save_intersection(session, output_file_name,
                            session.point_clouds_container.xz_intersection, session.point_clouds_container.yz_intersection, session.point_clouds_container.xy_intersection,
                            session.point_clouds_container.intersection_width);
                    }
                }

                ImGui::Separator();

                ImGui::MenuItem("xy_intersection", nullptr, &session.point_clouds_container.xy_intersection);
                ImGui::MenuItem("10m grid##xy", nullptr, &session.point_clouds_container.xy_grid_10x10);
                ImGui::MenuItem("1m grid##xy", nullptr, &session.point_clouds_container.xy_grid_1x1);
                ImGui::MenuItem("0.1m grid##xy", nullptr, &session.point_clouds_container.xy_grid_01x01);

                if (ImGui::MenuItem("Export xy intersection", nullptr, false, session.point_clouds_container.xy_intersection))
                {
                    const auto output_file_name = mandeye::fd::SaveFileDialog(out_fn.c_str(), mandeye::fd::LAS_LAZ_filter, ".laz");
                    std::cout << "laz file to save: '" << output_file_name << "'" << std::endl;

                    if (output_file_name.size() > 0)
                    {
                        save_intersection(session, output_file_name,
                            session.point_clouds_container.xz_intersection, session.point_clouds_container.yz_intersection, session.point_clouds_container.xy_intersection,
                            session.point_clouds_container.intersection_width);
                    }
                }

                ImGui::EndMenu();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Intersection menu");
        }

        if (ImGui::BeginMenu("View"))
        {
            ImGui::BeginDisabled(!session_loaded);
            {
                ImGui::PushItemWidth(ImGuiNumberWidth);

                auto tmp = point_size;
                ImGui::InputInt("points size", &point_size);
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

                ImGui::BeginDisabled(tls_registration.gnss.gnss_poses.size() <= 0);
                {
                    ImGui::MenuItem("Show GNSS correspondences", nullptr, &tls_registration.gnss.show_correspondences);
                }
                ImGui::EndDisabled();

                ImGui::Separator();
            }
            ImGui::EndDisabled();

            ImGui::MenuItem("Orthographic", nullptr, &is_ortho);
            if (is_ortho)
            {
				new_rotation_center = rotation_center;
                new_rotate_x = 0.0;
                new_rotate_y = 0.0;
                new_translate_x = translate_x;
                new_translate_y = translate_y;
                new_translate_z = translate_z;
                camera_transition_active = true;
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Switch between perspective view (3D) and orthographic view (2D/flat)");
            
            ImGui::MenuItem("Show axes", "Ctrl+X", &show_axes);
            ImGui::MenuItem("Block Z", nullptr, &block_z);

            ImGui::Separator();

            ImGui::MenuItem("Show compass ruler", "Ctrl+C", &compass_ruler);

            ImGui::Separator();

            ImGui::ColorEdit4("Background color", (float*)&clear_color, ImGuiColorEditFlags_NoInputs);

            ImGui::BeginDisabled(!session_loaded);
            {

                float color[3];
                if (session_loaded)
                {
                    color[0] = session.point_clouds_container.point_clouds[0].render_color[0];
                    color[1] = session.point_clouds_container.point_clouds[0].render_color[1];
                    color[2] = session.point_clouds_container.point_clouds[0].render_color[2];
                }

                if (ImGui::ColorEdit3("Cloud color", (float*)&color, ImGuiColorEditFlags_NoInputs))
                {
                    for (auto& pc : session.point_clouds_container.point_clouds)
                    {
                        pc.render_color[0] = color[0];
                        pc.render_color[1] = color[1];
                        pc.render_color[2] = color[2];
                        pc.show_color = false;
                    }
                }

                if (ImGui::MenuItem("Random segments colors", "Ctrl+R"))
                {
                    for (auto& pc : session.point_clouds_container.point_clouds)
                    {
                        pc.render_color[0] = float(rand() % 255) / 255.0f;
                        pc.render_color[1] = float(rand() % 255) / 255.0f;
                        pc.render_color[2] = float(rand() % 255) / 255.0f;
                        pc.show_color = false;
                    }
                }
            }
            ImGui::EndDisabled();

            ImGui::Separator();

            if (ImGui::BeginMenu("Console"))
            {
#ifdef _WIN32

                if (ImGui::MenuItem("Use Windows console", nullptr, &consWin))
                {
                    if (consWin)
                    {
                        AllocConsole();
                        freopen("CONOUT$", "w", stdout);
                        freopen("CONOUT$", "w", stderr);
                        freopen("CONIN$", "r", stdin);
                    }
                    else
                        FreeConsole();
                }
                if (ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    ImGui::Text("!!! If not used.. !!!");
                    ImGui::Text("- old console output is lost");
                    ImGui::Text("- new console output can only be seen in subwindow");
                    ImGui::Text("- app might run faster");
                    ImGui::EndTooltip();
                }
#endif
                //ImGui::MenuItem("Subwindow", nullptr, &consImGui);
                //if (ImGui::IsItemHovered())
                //    ImGui::SetTooltip("Show/hide console output as GUI window");

                ImGui::EndMenu();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Control console output");

            ImGui::EndMenu();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Scene view relevant parameters");

        camMenu();

        ImGui::BeginDisabled(session.point_clouds_container.point_clouds.size() <= 0);
        {
            ImGui::SameLine();
            ImGui::Dummy(ImVec2(20, 0));
            ImGui::SameLine();

            ImGui::SetNextItemWidth(ImGuiNumberWidth);
            ImGui::InputInt("Points render subsampling", &viewer_decimate_point_cloud, 10, 100);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("increase for better performance, decrease for rendering more points");
            ImGui::SameLine();

            //fps_avg = fps_avg * 0.7f + ImGui::GetIO().Framerate * 0.3f;  // exponential smoothing

            //double now = ImGui::GetTime();  // ImGui’s built-in timer (in seconds)

            //ImGui::Checkbox("dynamic", &dynamicSubsampling);
            //if (ImGui::IsItemHovered())
            //    ImGui::SetTooltip("automatically control subsampling vs FPS: increase bellow 10, decrease above 60");
            //if (dynamicSubsampling && (fps_avg < 15) && (now - lastAdjustTime > cooldownSeconds))
            //{
            //    viewer_decimate_point_cloud += 1;
            //    lastAdjustTime = now;
            //}
            //ImGui::SameLine();
            //ImGui::Text("(avg %.1f)", fps_avg);

            if (viewer_decimate_point_cloud < 1)
            {
                viewer_decimate_point_cloud = 1;
            }
            ImGui::SameLine();
            ImGui::Text("(%.1f FPS)", ImGui::GetIO().Framerate);
        }
        ImGui::EndDisabled();

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

    project_gui();

    // my_display_code(); 

    if (is_pca_gui)
        pca_gui();

    if (is_pose_graph_slam)
        pose_graph_slam_gui();

    if (is_manual_analisys)
        observation_picking_gui();

    if (is_loop_closure_gui)
        loop_closure_gui();

    if (is_lio_segments_gui)
        lio_segments_gui();

    int prev_index_pose = session.control_points.index_pose;
    if (session.control_points.is_imgui)
        session.control_points.imgui(session.point_clouds_container, rotation_center);

    if (prev_index_pose != session.control_points.index_pose)
    {
		session.control_points.index_picked_point = -1; // reset picked point when pose changes

        new_rotation_center.x() = session.point_clouds_container.point_clouds[session.control_points.index_pose].m_pose.translation().x();
        new_rotation_center.y() = session.point_clouds_container.point_clouds[session.control_points.index_pose].m_pose.translation().y();
        new_rotation_center.z() = session.point_clouds_container.point_clouds[session.control_points.index_pose].m_pose.translation().z();

        new_rotate_x = rotate_x;
        new_rotate_y = rotate_y;
        if (session.control_points.track_pose_with_camera)
        {
            new_translate_x = -new_rotation_center.x();
            new_translate_y = -new_rotation_center.y();
        }
        else
        {
            new_translate_x = translate_x;
			new_translate_y = translate_y;
        }
        new_translate_z = translate_z;
        camera_transition_active = true;
    }

    if (session.ground_control_points.is_imgui)
        session.ground_control_points.imgui(session.point_clouds_container);

    info_window(infoLines, &info_gui);

    if (compass_ruler)
        drawMiniCompassWithRuler(viewLocal, fabs(translate_z), clear_color);

    if (!session.control_points.is_imgui)
    {
        if (!is_loop_closure_gui)
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

            session.point_clouds_container.render(observation_picking, viewer_decimate_point_cloud, session.point_clouds_container.xz_intersection, session.point_clouds_container.yz_intersection,
                                                  session.point_clouds_container.xy_intersection,
                                                  session.point_clouds_container.xz_grid_10x10, session.point_clouds_container.xz_grid_1x1, session.point_clouds_container.xz_grid_01x01,
                                                  session.point_clouds_container.yz_grid_10x10, session.point_clouds_container.yz_grid_1x1, session.point_clouds_container.yz_grid_01x01,
                                                  session.point_clouds_container.xy_grid_10x10, session.point_clouds_container.xy_grid_1x1, session.point_clouds_container.xy_grid_01x01,
                                                  session.point_clouds_container.intersection_width, session_dims);

            // std::cout << "session.point_clouds_container.xy_grid_10x10 " << (int)session.point_clouds_container.xy_grid_10x10 << std::endl;

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
            if (session.pose_graph_loop_closure.gizmo && session.pose_graph_loop_closure.edges.size() > 0)
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

                const int &index_src = session.pose_graph_loop_closure.edges[session.pose_graph_loop_closure.index_active_edge].index_from;

                const Eigen::Affine3d &m_src = session.point_clouds_container.point_clouds.at(index_src).m_pose;
                session.pose_graph_loop_closure.edges[session.pose_graph_loop_closure.index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_src.inverse() * m_g);

                ImGui::End();
            }
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
            if (mouse_buttons & 1) //left button
            {
                rotate_x += dy * 0.2f; // * mouse_sensitivity;
                rotate_y += dx * 0.2f; // * mouse_sensitivity;
                breakCameraTransition();
            }
            if (mouse_buttons & 4) //right button
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

double distance_point_to_line(const Eigen::Vector3d &point, const LaserBeam &line)
{
    Eigen::Vector3d AP = point - line.position;

    double dist = (AP.cross(line.direction)).norm();
    return dist;
}

void getClosestTrajectoryPoint(Session& session, int x, int y, bool gcpPicking)
{
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

                new_rotation_center.x() = vp.x();
                new_rotation_center.y() = vp.y();
                new_rotation_center.z() = vp.z();

                if (gcpPicking)
                {
                    session.ground_control_points.picking_mode_index_to_node_inner = index_i;
                    session.ground_control_points.picking_mode_index_to_node_outer = index_j;
                }

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

        if ((glut_button == GLUT_MIDDLE_BUTTON || glut_button == GLUT_LEFT_BUTTON) && state == GLUT_DOWN && io.KeyCtrl)
        {
            if (session.ground_control_points.is_imgui)
            {
                std::cout << "GCP picking" << std::endl;
 
				getClosestTrajectoryPoint(session, x, y, true);
            }
            else if (session.control_points.is_imgui)
            {
                std::cout << "Control point picking" << std::endl;
                const auto laser_beam = GetLaserBeam(x, y);
                double min_distance = std::numeric_limits<double>::max();

                session.control_points.index_picked_point = -1;

                int i = session.control_points.index_pose;
                if (session.control_points.index_pose >= 0 && session.control_points.index_pose < session.point_clouds_container.point_clouds.size())
                {
                    for (int j = 0; j < session.point_clouds_container.point_clouds[i].points_local.size(); j++)
                    {
                        const auto &p = session.point_clouds_container.point_clouds[i].points_local[j];
                        Eigen::Vector3d vp = session.point_clouds_container.point_clouds[i].m_pose * p;

                        double dist = distance_point_to_line(vp, laser_beam);

                        if (dist < min_distance && dist < 0.1)
                        {
                            min_distance = dist;

                            new_rotation_center.x() = vp.x();
                            new_rotation_center.y() = vp.y();
                            new_rotation_center.z() = vp.z();

                            session.control_points.index_picked_point = j;
                        }
                    }

                    new_rotate_x = rotate_x;
                    new_rotate_y = rotate_y;
                    new_translate_x = -new_rotation_center.x();
                    new_translate_y = -new_rotation_center.y();
                    new_translate_z = translate_z;
                    camera_transition_active = true;
                }
            }
            else
            {
                if (glut_button == GLUT_MIDDLE_BUTTON)
                    if (session_loaded)
                        getClosestTrajectoryPoint(session, x, y, false);
                    else
                        setNewRotationCenter(x, y);
            }
        }

        if (glut_button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN && io.KeyCtrl)
        {
            if (session_loaded)
                getClosestTrajectoryPoint(session, x, y, false);
            else
                setNewRotationCenter(x, y);
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

int main(int argc, char *argv[])
{
    try
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