#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

// #include <export_laz.h>

#include <cmath>
#include <filesystem>

#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <ImGuizmo.h>
#include <imgui_internal.h>

// #define GLEW_STATIC
// #include <GL/glew.h>
#include <GL/freeglut.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <nlohmann/json.hpp>

#include <Eigen/Eigen>

#include <session.h>

#include <portable-file-dialogs.h>
#include <pfd_wrapper.hpp>

#include <utils.hpp>

#include <icp.h>

#include <python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_cw_jacobian.h>
#include <python-scripts/point-to-feature-metrics/point_to_line_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian.h>

#include <registration_plane_feature.h>

#include <m_estimators.h>

#include <HDMapping/Version.hpp>

#include <ndt.h>

#include <pair_wise_iterative_closest_point.h>
#include <observation_picking.h>

#include <export_laz.h>

#ifdef _WIN32
#include <windows.h>
#include "resource.h"
#endif

std::string winTitle = std::string("Step 3 (Multi session registration) ") + HDMAPPING_VERSION_STRING;

std::vector<std::string> infoLines = {
    "This program is third/final step in MANDEYE process",
    "",
    "First step: create project by adding sessions (result of 'multi_view_tls_registration_step_2' program)",
    "Last step: save project",
    "To produce map use 'multi_view_tls_registration_step_2' export functionality"};

// App specific shortcuts (Type and Shortcut are just for easy reference)
static const std::vector<ShortcutEntry> appShortcuts = {
    {"Normal keys", "A", ""},
    {"", "Ctrl+A", "Add session(s)"},
    {"", "B", ""},
    {"", "Ctrl+B", ""},
    {"", "C", ""},
    {"", "Ctrl+C", ""},
    {"", "D", ""},
    {"", "Ctrl+D", ""},
    {"", "E", ""},
    {"", "Ctrl+E", ""},
    {"", "F", ""},
    {"", "Ctrl+F", ""},
    {"", "G", ""},
    {"", "Ctrl+G", ""},
    {"", "H", ""},
    {"", "Ctrl+H", ""},
    {"", "I", ""},
    {"", "Ctrl+I", ""},
    {"", "J", ""},
    {"", "Ctrl+K", ""},
    {"", "K", ""},
    {"", "Ctrl+K", ""},
    {"", "L", ""},
    {"", "Ctrl+L", "Load sessions"},
    {"", "M", ""},
    {"", "Ctrl+M", ""},
    {"", "N", ""},
    {"", "Ctrl+N", ""},
    {"", "O", ""},
    {"", "Ctrl+O", "Open project"},
    {"", "P", ""},
    {"", "Ctrl+P", ""},
    {"", "Q", ""},
    {"", "Ctrl+Q", ""},
    {"", "R", ""},
    {"", "Ctrl+R", "Remove session(s)"},
    {"", "Shift+R", ""},
    {"", "S", ""},
    {"", "Ctrl+S", "Save project"},
    {"", "Ctrl+Shift+S", ""},
    {"", "T", ""},
    {"", "Ctrl+T", ""},
    {"", "U", ""},
    {"", "Ctrl+U", ""},
    {"", "V", ""},
    {"", "Ctrl+V", ""},
    {"", "W", ""},
    {"", "Ctrl+W", ""},
    {"", "X", ""},
    {"", "Ctrl+X", ""},
    {"", "Y", ""},
    {"", "Ctrl+Y", ""},
    {"", "Z", ""},
    {"", "Ctrl+Z", ""},
    {"", "Shift+Z", ""},
    {"", "1-9", ""},
    {"Special keys", "Up arrow", ""},
    {"", "Shift + up arrow", ""},
    {"", "Ctrl + up arrow", ""},
    {"", "Down arrow", ""},
    {"", "Shift + down arrow", ""},
    {"", "Ctrl + down arrow", ""},
    {"", "Left arrow", ""},
    {"", "Shift + left arrow", ""},
    {"", "Ctrl + left arrow", ""},
    {"", "Right arrow", ""},
    {"", "Shift + right arrow", ""},
    {"", "Ctrl + right arrow", ""},
    {"", "Pg down", ""},
    {"", "Pg up", ""},
    {"", "- key", ""},
    {"", "+ key", ""},
    {"Mouse related", "Left click + drag", ""},
    {"", "Right click + drag", "n"},
    {"", "Scroll", ""},
    {"", "Shift + scroll", ""},
    {"", "Shift + drag", ""},
    {"", "Ctrl + left click", ""},
    {"", "Ctrl + right click", ""},
    {"", "Ctrl + middle click", ""}};

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

bool is_decimate = true;
double bucket_x = 0.1;
double bucket_y = 0.1;
double bucket_z = 0.1;
bool calculate_offset = false;
ObservationPicking observation_picking;
int index_loop_closure_source = -1;
int index_loop_closure_target = -1;
int first_session_index = -1;
int second_session_index = -1;
double search_radius = 0.3;
bool loaded_sessions = false;
bool optimized = false;
bool gizmo_all_sessions = false;
bool is_ndt_gui = false;
bool is_loop_closure_gui = false;
bool remove_gui = false;
NDT ndt;

int number_visible_sessions = 0;
int index_gt = -1;
int old_index_gt = -1;
int index_gizmo = -1;
int old_index_gizmo = -1;

double time_stamp_offset = 0.0;

struct ProjectSettings
{
    std::vector<std::string> session_file_names;
};

struct Edge
{
    TaitBryanPose relative_pose_tb;
    TaitBryanPose relative_pose_tb_weights;
    int index_session_from;
    int index_session_to;
    int index_from;
    int index_to;
    bool is_fixed_px = false;
    bool is_fixed_py = false;
    bool is_fixed_pz = false;
    bool is_fixed_om = false;
    bool is_fixed_fi = false;
    bool is_fixed_ka = false;
};

std::vector<Edge> edges;
int index_active_edge = -1;
bool manipulate_active_edge = false;
bool edge_gizmo = false;

ProjectSettings project_settings;
std::vector<Session> sessions;

namespace fs = std::filesystem;

///////////////////////////////////////////////////////////////////////////////////

void ndt_gui()
{
    static bool compute_mean_and_cov_for_bucket = false;
    if (ImGui::Begin("Normal Distributions Transform", &is_ndt_gui, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::InputFloat3("Bucket size [m] (x, y,z)", ndt.bucket_size);
        if (ndt.bucket_size[0] < 0.01)
            ndt.bucket_size[0] = 0.01f;
        if (ndt.bucket_size[1] < 0.01)
            ndt.bucket_size[1] = 0.01f;
        if (ndt.bucket_size[2] < 0.01)
            ndt.bucket_size[2] = 0.01f;

        ImGui::PushItemWidth(ImGuiNumberWidth);
        ImGui::InputInt("Number of threads", &ndt.number_of_threads);
        if (ndt.number_of_threads < 1)
            ndt.number_of_threads = 1;
        ImGui::SameLine();
        ImGui::InputInt("Number of iterations", &ndt.number_of_iterations);
        if (ndt.number_of_iterations < 1)
            ndt.number_of_iterations = 1;
        ImGui::PopItemWidth();

        if (ImGui::Button("NDT optimization"))
        {
            for (auto &s : sessions)
            {
                s.is_gizmo = false;
            }

            double rms_initial = 0.0;
            double rms_final = 0.0;
            double mui = 0.0;
            // ndt.optimize(point_clouds_container.point_clouds, rms_initial, rms_final, mui);
            // std::cout << "mui: " << mui << " rms_initial: " << rms_initial << " rms_final: " << rms_final << std::endl;
            ndt.optimize(sessions, false, compute_mean_and_cov_for_bucket);
        }
        ImGui::End();
    }

#if 0
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
        ndt.sigma_polar_angle = 0.007 * DEG_TO_RAD;
        ndt.sigma_azimuthal_angle = 0.007 * DEG_TO_RAD;
    }

    if (ImGui::Button("Set Zoller+Fröhlich TLS Imager 5010C errors"))
    {
        ndt.sigma_r = 0.01;
        ndt.sigma_polar_angle = 0.007 * DEG_TO_RAD;
        ndt.sigma_azimuthal_angle = 0.007 * DEG_TO_RAD;
    }

    if (ImGui::Button("Set Zoller+Fröhlich TLS Imager 5016 errors"))
    {
        ndt.sigma_r = 0.00025;
        ndt.sigma_polar_angle = 0.004 * DEG_TO_RAD;
        ndt.sigma_azimuthal_angle = 0.004 * DEG_TO_RAD;
    }
    if (ImGui::Button("Set Faro Focus3D errors"))
    {
        ndt.sigma_r = 0.001;
        ndt.sigma_polar_angle = 19.0 * (1.0 / 3600.0) * DEG_TO_RAD;
        ndt.sigma_azimuthal_angle = 19.0 * (1.0 / 3600.0) * DEG_TO_RAD;
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
        ndt.sigma_polar_angle = 0.0005 * DEG_TO_RAD + 0.0003;     // Laser Beam Dicvergence
        ndt.sigma_azimuthal_angle = 0.0005 * DEG_TO_RAD + 0.0003; // Laser Beam Dicvergence
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
#endif
}

void loop_closure_gui()
{
    if (ImGui::Begin("Manual Pose Graph Loop Closure Mode", &is_loop_closure_gui, ImGuiWindowFlags_AlwaysAutoResize))
    {
        if (!manipulate_active_edge)
        {
            ImGui::InputInt("index_loop_closure_source", &index_loop_closure_source);
            if (index_loop_closure_source < 0)
                index_loop_closure_source = 0;
            if (index_loop_closure_source >= sessions[first_session_index].point_clouds_container.point_clouds.size() - 1)
                index_loop_closure_source = sessions[first_session_index].point_clouds_container.point_clouds.size() - 1;
            ImGui::InputInt("index_loop_closure_target", &index_loop_closure_target);
            if (index_loop_closure_target < 0)
                index_loop_closure_target = 0;
            if (index_loop_closure_target >= sessions[second_session_index].point_clouds_container.point_clouds.size() - 1)
                index_loop_closure_target = sessions[second_session_index].point_clouds_container.point_clouds.size() - 1;
        }
        if (ImGui::Button("Add edge"))
        {
            Edge edge;
            edge.index_from = index_loop_closure_source;
            edge.index_to = index_loop_closure_target;
            edge.index_session_from = first_session_index;
            edge.index_session_to = second_session_index;

            edge.relative_pose_tb = pose_tait_bryan_from_affine_matrix(
                sessions[first_session_index].point_clouds_container.point_clouds[index_loop_closure_source].m_pose.inverse() *
                sessions[second_session_index].point_clouds_container.point_clouds[index_loop_closure_target].m_pose);

            edge.relative_pose_tb_weights.px = 1000000.0;
            edge.relative_pose_tb_weights.py = 1000000.0;
            edge.relative_pose_tb_weights.pz = 1000000.0;
            edge.relative_pose_tb_weights.om = 1000000.0;
            edge.relative_pose_tb_weights.fi = 1000000.0;
            edge.relative_pose_tb_weights.ka = 1000000.0;

            edges.push_back(edge);

            index_active_edge = edges.size() - 1;
        }

        std::string number_active_edges = "number_edges: " + std::to_string(edges.size());
        ImGui::Text(number_active_edges.c_str());
        if (edges.size() > 0)
        {
            ImGui::Checkbox("manipulate_active_edge", &manipulate_active_edge);
            if (manipulate_active_edge)
            {
                int remove_edge_index = -1;
                if (ImGui::Button("remove active edge"))
                {
                    edge_gizmo = false;
                    remove_edge_index = index_active_edge;
                }

                int prev_index_active_edge = index_active_edge;

                if (!edge_gizmo)
                {
                    bool is_gizmo = false;

                    for (const auto &s : sessions)
                    {
                        if (s.is_gizmo)
                            is_gizmo = true;
                    }

                    if (!is_gizmo)
                    {
                        ImGui::InputInt("index_active_edge", &index_active_edge);

                        if (index_active_edge < 0)
                            index_active_edge = 0;
                        if (index_active_edge >= (int)edges.size())
                            index_active_edge = (int)edges.size() - 1;
                    }
                }

                std::string txt = "index_session_from: " + std::to_string(edges[index_active_edge].index_session_from);
                ImGui::Text(txt.c_str());
                txt = "index_session_to: " + std::to_string(edges[index_active_edge].index_session_to);
                ImGui::Text(txt.c_str());
                txt = "index_from: " + std::to_string(edges[index_active_edge].index_from);
                ImGui::Text(txt.c_str());
                txt = "index_to: " + std::to_string(edges[index_active_edge].index_to);
                ImGui::Text(txt.c_str());

                if (remove_edge_index != -1)
                {
                    std::vector<Edge> new_edges;
                    for (size_t i = 0; i < edges.size(); i++)
                    {
                        if (remove_edge_index != i)
                            new_edges.push_back(edges[i]);
                    }
                    edges = new_edges;

                    index_active_edge = remove_edge_index - 1;
                    manipulate_active_edge = false;
                }

                bool prev_gizmo = edge_gizmo;
                ImGui::Checkbox("gizmo", &edge_gizmo);

                if (prev_gizmo != edge_gizmo)
                {
                    auto m_to = sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from].m_pose *
                                affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

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
                }
                if (!edge_gizmo)
                {
                    if (ImGui::Button("ICP"))
                    {
                        std::cout << "Iterative Closest Point" << std::endl;
                        if (sessions[edges[index_active_edge].index_session_from].is_ground_truth &&
                            sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                        {
                            std::cout << "Two sessions are ground truth!!! ICP is disabled" << std::endl;
                        }
                        else
                        {
                            bool is_with_ground_truth = false;
                            if (sessions[edges[index_active_edge].index_session_from].is_ground_truth ||
                                sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                            {
                                is_with_ground_truth = true;
                            }

                            if (is_with_ground_truth)
                            {
                                int index_session_from = -1;
                                int index_session_to = -1;
                                int index_from = -1;
                                int index_to = -1;

                                if (sessions[edges[index_active_edge].index_session_from].is_ground_truth)
                                {
                                    index_session_from = edges[index_active_edge].index_session_from;
                                    index_session_to = edges[index_active_edge].index_session_to;
                                    index_from = edges[index_active_edge].index_from;
                                    index_to = edges[index_active_edge].index_to;
                                }
                                else
                                {
                                    index_session_from = edges[index_active_edge].index_session_to;
                                    index_session_to = edges[index_active_edge].index_session_from;
                                    index_from = edges[index_active_edge].index_to;
                                    index_to = edges[index_active_edge].index_from;
                                }

                                double x_min = 1000000000000.0;
                                double y_min = 1000000000000.0;
                                double z_min = 1000000000000.0;
                                double x_max = -1000000000000.0;
                                double y_max = -1000000000000.0;
                                double z_max = -1000000000000.0;

                                auto &points_to = sessions[index_session_to].point_clouds_container.point_clouds[index_to];

                                for (const auto &p : points_to.points_local)
                                {
                                    auto pg = points_to.m_pose * p;
                                    if (pg.x() < x_min)
                                        x_min = pg.x();
                                    if (pg.y() < y_min)
                                        y_min = pg.y();
                                    if (pg.z() < z_min)
                                        z_min = pg.z();

                                    if (pg.x() > x_max)
                                        x_max = pg.x();
                                    if (pg.y() > y_max)
                                        y_max = pg.y();
                                    if (pg.z() > z_max)
                                        z_max = pg.z();
                                }
                                auto &points_from = sessions[index_session_from].point_clouds_container.point_clouds[index_from];
                                std::vector<Eigen::Vector3d> ground_truth;
                                for (const auto &p : points_from.points_local)
                                {
                                    auto pg = points_from.m_pose * p;
                                    if (pg.x() > x_min && pg.x() < x_max)
                                    {
                                        if (pg.y() > y_min && pg.y() < y_max)
                                        {
                                            if (pg.z() > z_min && pg.z() < z_max)
                                                ground_truth.push_back(p);
                                        }
                                    }
                                }

                                int number_of_iterations = 10;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                std::vector<Eigen::Vector3d> source = sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds[edges[index_active_edge].index_to].points_local;
                                std::vector<Eigen::Vector3d> target = ground_truth; // sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from].points_local;

                                if (icp.compute(source, target, search_radius, number_of_iterations, m_pose))
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);

                                /*PointClouds pcs;
                                pcs.point_clouds.push_back(sessions[index_session_from].point_clouds_container.point_clouds[index_from]);
                                pcs.point_clouds.push_back(sessions[index_session_to].point_clouds_container.point_clouds[index_to]);
                                pcs.point_clouds[0].points_local = ground_truth;
                                pcs.point_clouds[0].m_pose = Eigen::Affine3d::Identity();
                                pcs.point_clouds[1].m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);


                                ICP icp;
                                icp.search_radius = (float)search_radius;

                                for (auto &pc : pcs.point_clouds)
                                {
                                    pc.rgd_params.resolution_X = icp.search_radius;
                                    pc.rgd_params.resolution_Y = icp.search_radius;
                                    pc.rgd_params.resolution_Z = icp.search_radius;
                                    pc.build_rgd();
                                    pc.cout_rgd();
                                    pc.compute_normal_vectors(0.5);
                                }

                                icp.number_of_threads = std::thread::hardware_concurrency();
                                icp.number_of_iterations = 10;
                                icp.is_adaptive_robust_kernel = false;

                                icp.is_ballanced_horizontal_vs_vertical = false;
                                icp.is_fix_first_node = true;
                                icp.is_gauss_newton = true;
                                icp.is_levenberg_marguardt = false;
                                icp.is_cw = false;
                                icp.is_wc = true;
                                icp.is_tait_bryan_angles = true;
                                icp.is_quaternion = false;
                                icp.is_rodrigues = false;
                                std::cout << "optimization_point_to_point_source_to_target" << std::endl;

                                icp.optimization_point_to_point_source_to_target(pcs);

                                edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(pcs.point_clouds[0].m_pose.inverse() * pcs.point_clouds[1].m_pose);
                                */
                            }
                            else
                            {
                                /*PointClouds pcs;
                                pcs.point_clouds.push_back(sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from]);
                                pcs.point_clouds.push_back(sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds[edges[index_active_edge].index_to]);
                                pcs.point_clouds[0].m_pose = Eigen::Affine3d::Identity();
                                pcs.point_clouds[1].m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);
                                ICP icp;
                                icp.search_radius = (float)search_radius;

                                for (auto &pc : pcs.point_clouds)
                                {
                                    pc.rgd_params.resolution_X = icp.search_radius;
                                    pc.rgd_params.resolution_Y = icp.search_radius;
                                    pc.rgd_params.resolution_Z = icp.search_radius;
                                    pc.build_rgd();
                                    pc.cout_rgd();
                                    pc.compute_normal_vectors(0.5);
                                }

                                icp.number_of_threads = std::thread::hardware_concurrency();
                                icp.number_of_iterations = 10;
                                icp.is_adaptive_robust_kernel = false;

                                icp.is_ballanced_horizontal_vs_vertical = false;
                                icp.is_fix_first_node = true;
                                icp.is_gauss_newton = true;
                                icp.is_levenberg_marguardt = false;
                                icp.is_cw = false;
                                icp.is_wc = true;
                                icp.is_tait_bryan_angles = true;
                                icp.is_quaternion = false;
                                icp.is_rodrigues = false;
                                std::cout << "optimization_point_to_point_source_to_target" << std::endl;

                                icp.optimization_point_to_point_source_to_target(pcs);

                                edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(pcs.point_clouds[0].m_pose.inverse() * pcs.point_clouds[1].m_pose);
                                */
                                int number_of_iterations = 10;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                std::vector<Eigen::Vector3d> source = sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds[edges[index_active_edge].index_to].points_local;
                                std::vector<Eigen::Vector3d> target = sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from].points_local;

                                if (icp.compute(source, target, search_radius, number_of_iterations, m_pose))
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                            }
                        }
                    }
                    ImGui::SameLine();
                    ImGui::InputDouble("search_radius", &search_radius);
                    if (search_radius < 0.01)
                        search_radius = 0.01;

                    /*if (ImGui::Button("ICP"))
                    {
                        std::cout << "Iterative Closest Point" << std::endl;
                        if (sessions[edges[index_active_edge].index_session_from].is_ground_truth &&
                            sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                        {
                            std::cout << "Two sessions are ground truth!!! ICP is disabled" << std::endl;
                        }
                        else
                        {
                            bool is_with_ground_truth = false;
                            if (sessions[edges[index_active_edge].index_session_from].is_ground_truth ||
                                sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                            {
                                is_with_ground_truth = true;
                            }

                            if (is_with_ground_truth)
                            {
                                int index_session_from = -1;
                                int index_session_to = -1;
                                int index_from = -1;
                                int index_to = -1;

                                if (sessions[edges[index_active_edge].index_session_from].is_ground_truth)
                                {
                                    index_session_from = edges[index_active_edge].index_session_from;
                                    index_session_to = edges[index_active_edge].index_session_to;
                                    index_from = edges[index_active_edge].index_from;
                                    index_to = edges[index_active_edge].index_to;
                                }
                                else
                                {
                                    index_session_from = edges[index_active_edge].index_session_to;
                                    index_session_to = edges[index_active_edge].index_session_from;
                                    index_from = edges[index_active_edge].index_to;
                                    index_to = edges[index_active_edge].index_from;
                                }

                                double x_min = 1000000000000.0;
                                double y_min = 1000000000000.0;
                                double z_min = 1000000000000.0;
                                double x_max = -1000000000000.0;
                                double y_max = -1000000000000.0;
                                double z_max = -1000000000000.0;

                                auto &points_to = sessions[index_session_to].point_clouds_container.point_clouds[index_to];

                                for (const auto &p : points_to.points_local)
                                {
                                    auto pg = points_to.m_pose * p;
                                    if (pg.x() < x_min)
                                    {
                                        x_min = pg.x();
                                    }
                                    if (pg.y() < y_min)
                                    {
                                        y_min = pg.y();
                                    }
                                    if (pg.z() < z_min)
                                    {
                                        z_min = pg.z();
                                    }
                                    if (pg.x() > x_max)
                                    {
                                        x_max = pg.x();
                                    }
                                    if (pg.y() > y_max)
                                    {
                                        y_max = pg.y();
                                    }
                                    if (pg.z() > z_max)
                                    {
                                        z_max = pg.z();
                                    }
                                }
                                auto &points_from = sessions[index_session_from].point_clouds_container.point_clouds[index_from];
                                std::vector<Eigen::Vector3d> ground_truth;
                                for (const auto &p : points_from.points_local)
                                {
                                    auto pg = points_from.m_pose * p;
                                    if (pg.x() > x_min && pg.x() < x_max)
                                    {
                                        if (pg.y() > y_min && pg.y() < y_max)
                                        {
                                            if (pg.z() > z_min && pg.z() < z_max)
                                            {
                                                ground_truth.push_back(p);
                                            }
                                        }
                                    }
                                }

                                PointClouds pcs;
                                pcs.point_clouds.push_back(sessions[index_session_from].point_clouds_container.point_clouds[index_from]);
                                pcs.point_clouds.push_back(sessions[index_session_to].point_clouds_container.point_clouds[index_to]);
                                pcs.point_clouds[0].points_local = ground_truth;
                                pcs.point_clouds[0].m_pose = Eigen::Affine3d::Identity();
                                pcs.point_clouds[1].m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);
                                ICP icp;
                                icp.search_radius = (float)search_radius;

                                for (auto &pc : pcs.point_clouds)
                                {
                                    pc.rgd_params.resolution_X = icp.search_radius;
                                    pc.rgd_params.resolution_Y = icp.search_radius;
                                    pc.rgd_params.resolution_Z = icp.search_radius;
                                    pc.build_rgd();
                                    pc.cout_rgd();
                                    pc.compute_normal_vectors(0.5);
                                }

                                icp.number_of_threads = std::thread::hardware_concurrency();
                                icp.number_of_iterations = 10;
                                icp.is_adaptive_robust_kernel = false;

                                icp.is_ballanced_horizontal_vs_vertical = false;
                                icp.is_fix_first_node = true;
                                icp.is_gauss_newton = true;
                                icp.is_levenberg_marguardt = false;
                                icp.is_cw = false;
                                icp.is_wc = true;
                                icp.is_tait_bryan_angles = true;
                                icp.is_quaternion = false;
                                icp.is_rodrigues = false;
                                std::cout << "optimization_point_to_point_source_to_target" << std::endl;

                                icp.optimization_point_to_point_source_to_target(pcs);

                                edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(pcs.point_clouds[0].m_pose.inverse() * pcs.point_clouds[1].m_pose);
                            }
                            else
                            {
                                PointClouds pcs;
                                pcs.point_clouds.push_back(sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from]);
                                pcs.point_clouds.push_back(sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds[edges[index_active_edge].index_to]);
                                pcs.point_clouds[0].m_pose = Eigen::Affine3d::Identity();
                                pcs.point_clouds[1].m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);
                                ICP icp;
                                icp.search_radius = (float)search_radius;

                                for (auto &pc : pcs.point_clouds)
                                {
                                    pc.rgd_params.resolution_X = icp.search_radius;
                                    pc.rgd_params.resolution_Y = icp.search_radius;
                                    pc.rgd_params.resolution_Z = icp.search_radius;
                                    pc.build_rgd();
                                    pc.cout_rgd();
                                    pc.compute_normal_vectors(0.5);
                                }

                                icp.number_of_threads = std::thread::hardware_concurrency();
                                icp.number_of_iterations = 10;
                                icp.is_adaptive_robust_kernel = false;

                                icp.is_ballanced_horizontal_vs_vertical = false;
                                icp.is_fix_first_node = true;
                                icp.is_gauss_newton = true;
                                icp.is_levenberg_marguardt = false;
                                icp.is_cw = false;
                                icp.is_wc = true;
                                icp.is_tait_bryan_angles = true;
                                icp.is_quaternion = false;
                                icp.is_rodrigues = false;
                                std::cout << "optimization_point_to_point_source_to_target" << std::endl;

                                icp.optimization_point_to_point_source_to_target(pcs);

                                edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(pcs.point_clouds[0].m_pose.inverse() * pcs.point_clouds[1].m_pose);
                            }
                        }
                    }
                    ImGui::SameLine();
                    ImGui::InputDouble("search_radius", &search_radius);
                    if (search_radius < 0.01)
                    {
                        search_radius = 0.01;
                    }*/

                    /////////////////////////////////
                    if (ImGui::Button("ICP [search radius 2m]"))
                    {
                        float sr = 2.0;
                        std::cout << "Iterative Closest Point" << std::endl;
                        if (sessions[edges[index_active_edge].index_session_from].is_ground_truth &&
                            sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                        {
                            std::cout << "Two sessions are ground truth!!! ICP is disabled" << std::endl;
                        }
                        else
                        {
                            bool is_with_ground_truth = false;
                            if (sessions[edges[index_active_edge].index_session_from].is_ground_truth ||
                                sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                            {
                                is_with_ground_truth = true;
                            }

                            if (is_with_ground_truth)
                            {
                                int index_session_from = -1;
                                int index_session_to = -1;
                                int index_from = -1;
                                int index_to = -1;

                                if (sessions[edges[index_active_edge].index_session_from].is_ground_truth)
                                {
                                    index_session_from = edges[index_active_edge].index_session_from;
                                    index_session_to = edges[index_active_edge].index_session_to;
                                    index_from = edges[index_active_edge].index_from;
                                    index_to = edges[index_active_edge].index_to;
                                }
                                else
                                {
                                    index_session_from = edges[index_active_edge].index_session_to;
                                    index_session_to = edges[index_active_edge].index_session_from;
                                    index_from = edges[index_active_edge].index_to;
                                    index_to = edges[index_active_edge].index_from;
                                }

                                double x_min = 1000000000000.0;
                                double y_min = 1000000000000.0;
                                double z_min = 1000000000000.0;
                                double x_max = -1000000000000.0;
                                double y_max = -1000000000000.0;
                                double z_max = -1000000000000.0;

                                auto &points_to = sessions[index_session_to].point_clouds_container.point_clouds[index_to];

                                for (const auto &p : points_to.points_local)
                                {
                                    auto pg = points_to.m_pose * p;
                                    if (pg.x() < x_min)
                                        x_min = pg.x();
                                    if (pg.y() < y_min)
                                        y_min = pg.y();
                                    if (pg.z() < z_min)
                                        z_min = pg.z();

                                    if (pg.x() > x_max)
                                        x_max = pg.x();
                                    if (pg.y() > y_max)
                                        y_max = pg.y();
                                    if (pg.z() > z_max)
                                        z_max = pg.z();
                                }
                                auto &points_from = sessions[index_session_from].point_clouds_container.point_clouds[index_from];
                                std::vector<Eigen::Vector3d> ground_truth;
                                for (const auto &p : points_from.points_local)
                                {
                                    auto pg = points_from.m_pose * p;
                                    if (pg.x() > x_min && pg.x() < x_max)
                                    {
                                        if (pg.y() > y_min && pg.y() < y_max)
                                        {
                                            if (pg.z() > z_min && pg.z() < z_max)
                                                ground_truth.push_back(p);
                                        }
                                    }
                                }

                                int number_of_iterations = 30;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                std::vector<Eigen::Vector3d> source = sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds[edges[index_active_edge].index_to].points_local;
                                std::vector<Eigen::Vector3d> target = ground_truth; // sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from].points_local;

                                if (icp.compute(source, target, sr, number_of_iterations, m_pose))
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                            }
                            else
                            {

                                int number_of_iterations = 30;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                std::vector<Eigen::Vector3d> source = sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds[edges[index_active_edge].index_to].points_local;
                                std::vector<Eigen::Vector3d> target = sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from].points_local;

                                if (icp.compute(source, target, sr, number_of_iterations, m_pose))
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                            }
                        }
                    }

                    ImGui::SameLine();
                    if (ImGui::Button("ICP [search radius 1m]"))
                    {
                        float sr = 1.0;
                        std::cout << "Iterative Closest Point" << std::endl;
                        if (sessions[edges[index_active_edge].index_session_from].is_ground_truth &&
                            sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                        {
                            std::cout << "Two sessions are ground truth!!! ICP is disabled" << std::endl;
                        }
                        else
                        {
                            bool is_with_ground_truth = false;
                            if (sessions[edges[index_active_edge].index_session_from].is_ground_truth ||
                                sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                            {
                                is_with_ground_truth = true;
                            }

                            if (is_with_ground_truth)
                            {
                                int index_session_from = -1;
                                int index_session_to = -1;
                                int index_from = -1;
                                int index_to = -1;

                                if (sessions[edges[index_active_edge].index_session_from].is_ground_truth)
                                {
                                    index_session_from = edges[index_active_edge].index_session_from;
                                    index_session_to = edges[index_active_edge].index_session_to;
                                    index_from = edges[index_active_edge].index_from;
                                    index_to = edges[index_active_edge].index_to;
                                }
                                else
                                {
                                    index_session_from = edges[index_active_edge].index_session_to;
                                    index_session_to = edges[index_active_edge].index_session_from;
                                    index_from = edges[index_active_edge].index_to;
                                    index_to = edges[index_active_edge].index_from;
                                }

                                double x_min = 1000000000000.0;
                                double y_min = 1000000000000.0;
                                double z_min = 1000000000000.0;
                                double x_max = -1000000000000.0;
                                double y_max = -1000000000000.0;
                                double z_max = -1000000000000.0;

                                auto &points_to = sessions[index_session_to].point_clouds_container.point_clouds[index_to];

                                for (const auto &p : points_to.points_local)
                                {
                                    auto pg = points_to.m_pose * p;
                                    if (pg.x() < x_min)
                                        x_min = pg.x();
                                    if (pg.y() < y_min)
                                        y_min = pg.y();
                                    if (pg.z() < z_min)
                                        z_min = pg.z();

                                    if (pg.x() > x_max)
                                        x_max = pg.x();
                                    if (pg.y() > y_max)
                                        y_max = pg.y();
                                    if (pg.z() > z_max)
                                        z_max = pg.z();
                                }
                                auto &points_from = sessions[index_session_from].point_clouds_container.point_clouds[index_from];
                                std::vector<Eigen::Vector3d> ground_truth;
                                for (const auto &p : points_from.points_local)
                                {
                                    auto pg = points_from.m_pose * p;
                                    if (pg.x() > x_min && pg.x() < x_max)
                                    {
                                        if (pg.y() > y_min && pg.y() < y_max)
                                        {
                                            if (pg.z() > z_min && pg.z() < z_max)
                                                ground_truth.push_back(p);
                                        }
                                    }
                                }

                                int number_of_iterations = 30;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                std::vector<Eigen::Vector3d> source = sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds[edges[index_active_edge].index_to].points_local;
                                std::vector<Eigen::Vector3d> target = ground_truth; // sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from].points_local;

                                if (icp.compute(source, target, sr, number_of_iterations, m_pose))
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                            }
                            else
                            {

                                int number_of_iterations = 30;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                std::vector<Eigen::Vector3d> source = sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds[edges[index_active_edge].index_to].points_local;
                                std::vector<Eigen::Vector3d> target = sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from].points_local;

                                if (icp.compute(source, target, sr, number_of_iterations, m_pose))
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                            }
                        }
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("ICP [search radius 0.5m]"))
                    {
                        float sr = 0.5;
                        std::cout << "Iterative Closest Point" << std::endl;
                        if (sessions[edges[index_active_edge].index_session_from].is_ground_truth &&
                            sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                        {
                            std::cout << "Two sessions are ground truth!!! ICP is disabled" << std::endl;
                        }
                        else
                        {
                            bool is_with_ground_truth = false;
                            if (sessions[edges[index_active_edge].index_session_from].is_ground_truth ||
                                sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                            {
                                is_with_ground_truth = true;
                            }

                            if (is_with_ground_truth)
                            {
                                int index_session_from = -1;
                                int index_session_to = -1;
                                int index_from = -1;
                                int index_to = -1;

                                if (sessions[edges[index_active_edge].index_session_from].is_ground_truth)
                                {
                                    index_session_from = edges[index_active_edge].index_session_from;
                                    index_session_to = edges[index_active_edge].index_session_to;
                                    index_from = edges[index_active_edge].index_from;
                                    index_to = edges[index_active_edge].index_to;
                                }
                                else
                                {
                                    index_session_from = edges[index_active_edge].index_session_to;
                                    index_session_to = edges[index_active_edge].index_session_from;
                                    index_from = edges[index_active_edge].index_to;
                                    index_to = edges[index_active_edge].index_from;
                                }

                                double x_min = 1000000000000.0;
                                double y_min = 1000000000000.0;
                                double z_min = 1000000000000.0;
                                double x_max = -1000000000000.0;
                                double y_max = -1000000000000.0;
                                double z_max = -1000000000000.0;

                                auto &points_to = sessions[index_session_to].point_clouds_container.point_clouds[index_to];

                                for (const auto &p : points_to.points_local)
                                {
                                    auto pg = points_to.m_pose * p;
                                    if (pg.x() < x_min)
                                        x_min = pg.x();
                                    if (pg.y() < y_min)
                                        y_min = pg.y();
                                    if (pg.z() < z_min)
                                        z_min = pg.z();

                                    if (pg.x() > x_max)
                                        x_max = pg.x();
                                    if (pg.y() > y_max)
                                        y_max = pg.y();
                                    if (pg.z() > z_max)
                                        z_max = pg.z();
                                }
                                auto &points_from = sessions[index_session_from].point_clouds_container.point_clouds[index_from];
                                std::vector<Eigen::Vector3d> ground_truth;
                                for (const auto &p : points_from.points_local)
                                {
                                    auto pg = points_from.m_pose * p;
                                    if (pg.x() > x_min && pg.x() < x_max)
                                    {
                                        if (pg.y() > y_min && pg.y() < y_max)
                                        {
                                            if (pg.z() > z_min && pg.z() < z_max)
                                                ground_truth.push_back(p);
                                        }
                                    }
                                }

                                int number_of_iterations = 30;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                std::vector<Eigen::Vector3d> source = sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds[edges[index_active_edge].index_to].points_local;
                                std::vector<Eigen::Vector3d> target = ground_truth; // sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from].points_local;

                                if (icp.compute(source, target, sr, number_of_iterations, m_pose))
                                {
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                                }
                            }
                            else
                            {

                                int number_of_iterations = 30;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                std::vector<Eigen::Vector3d> source = sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds[edges[index_active_edge].index_to].points_local;
                                std::vector<Eigen::Vector3d> target = sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from].points_local;

                                if (icp.compute(source, target, sr, number_of_iterations, m_pose))
                                {
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                                }
                            }
                        }
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("ICP [search radius 0.25m]"))
                    {
                        float sr = 0.25;
                        std::cout << "Iterative Closest Point" << std::endl;
                        if (sessions[edges[index_active_edge].index_session_from].is_ground_truth &&
                            sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                        {
                            std::cout << "Two sessions are ground truth!!! ICP is disabled" << std::endl;
                        }
                        else
                        {
                            bool is_with_ground_truth = false;
                            if (sessions[edges[index_active_edge].index_session_from].is_ground_truth ||
                                sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                            {
                                is_with_ground_truth = true;
                            }

                            if (is_with_ground_truth)
                            {
                                int index_session_from = -1;
                                int index_session_to = -1;
                                int index_from = -1;
                                int index_to = -1;

                                if (sessions[edges[index_active_edge].index_session_from].is_ground_truth)
                                {
                                    index_session_from = edges[index_active_edge].index_session_from;
                                    index_session_to = edges[index_active_edge].index_session_to;
                                    index_from = edges[index_active_edge].index_from;
                                    index_to = edges[index_active_edge].index_to;
                                }
                                else
                                {
                                    index_session_from = edges[index_active_edge].index_session_to;
                                    index_session_to = edges[index_active_edge].index_session_from;
                                    index_from = edges[index_active_edge].index_to;
                                    index_to = edges[index_active_edge].index_from;
                                }

                                double x_min = 1000000000000.0;
                                double y_min = 1000000000000.0;
                                double z_min = 1000000000000.0;
                                double x_max = -1000000000000.0;
                                double y_max = -1000000000000.0;
                                double z_max = -1000000000000.0;

                                auto &points_to = sessions[index_session_to].point_clouds_container.point_clouds[index_to];

                                for (const auto &p : points_to.points_local)
                                {
                                    auto pg = points_to.m_pose * p;
                                    if (pg.x() < x_min)
                                        x_min = pg.x();
                                    if (pg.y() < y_min)
                                        y_min = pg.y();
                                    if (pg.z() < z_min)
                                        z_min = pg.z();

                                    if (pg.x() > x_max)
                                        x_max = pg.x();
                                    if (pg.y() > y_max)
                                        y_max = pg.y();
                                    if (pg.z() > z_max)
                                        z_max = pg.z();
                                }
                                auto &points_from = sessions[index_session_from].point_clouds_container.point_clouds[index_from];
                                std::vector<Eigen::Vector3d> ground_truth;
                                for (const auto &p : points_from.points_local)
                                {
                                    auto pg = points_from.m_pose * p;
                                    if (pg.x() > x_min && pg.x() < x_max)
                                    {
                                        if (pg.y() > y_min && pg.y() < y_max)
                                        {
                                            if (pg.z() > z_min && pg.z() < z_max)
                                                ground_truth.push_back(p);
                                        }
                                    }
                                }

                                int number_of_iterations = 30;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                std::vector<Eigen::Vector3d> source = sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds[edges[index_active_edge].index_to].points_local;
                                std::vector<Eigen::Vector3d> target = ground_truth; // sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from].points_local;

                                if (icp.compute(source, target, sr, number_of_iterations, m_pose))
                                {
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                                }
                            }
                            else
                            {

                                int number_of_iterations = 30;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                std::vector<Eigen::Vector3d> source = sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds[edges[index_active_edge].index_to].points_local;
                                std::vector<Eigen::Vector3d> target = sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from].points_local;

                                if (icp.compute(source, target, sr, number_of_iterations, m_pose))
                                {
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                                }
                            }
                        }
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("ICP [search radius 0.1m]"))
                    {
                        float sr = 0.1;
                        std::cout << "Iterative Closest Point" << std::endl;
                        if (sessions[edges[index_active_edge].index_session_from].is_ground_truth &&
                            sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                        {
                            std::cout << "Two sessions are ground truth!!! ICP is disabled" << std::endl;
                        }
                        else
                        {
                            bool is_with_ground_truth = false;
                            if (sessions[edges[index_active_edge].index_session_from].is_ground_truth ||
                                sessions[edges[index_active_edge].index_session_to].is_ground_truth)
                            {
                                is_with_ground_truth = true;
                            }

                            if (is_with_ground_truth)
                            {
                                int index_session_from = -1;
                                int index_session_to = -1;
                                int index_from = -1;
                                int index_to = -1;

                                if (sessions[edges[index_active_edge].index_session_from].is_ground_truth)
                                {
                                    index_session_from = edges[index_active_edge].index_session_from;
                                    index_session_to = edges[index_active_edge].index_session_to;
                                    index_from = edges[index_active_edge].index_from;
                                    index_to = edges[index_active_edge].index_to;
                                }
                                else
                                {
                                    index_session_from = edges[index_active_edge].index_session_to;
                                    index_session_to = edges[index_active_edge].index_session_from;
                                    index_from = edges[index_active_edge].index_to;
                                    index_to = edges[index_active_edge].index_from;
                                }

                                double x_min = 1000000000000.0;
                                double y_min = 1000000000000.0;
                                double z_min = 1000000000000.0;
                                double x_max = -1000000000000.0;
                                double y_max = -1000000000000.0;
                                double z_max = -1000000000000.0;

                                auto &points_to = sessions[index_session_to].point_clouds_container.point_clouds[index_to];

                                for (const auto &p : points_to.points_local)
                                {
                                    auto pg = points_to.m_pose * p;
                                    if (pg.x() < x_min)
                                        x_min = pg.x();
                                    if (pg.y() < y_min)
                                        y_min = pg.y();
                                    if (pg.z() < z_min)
                                        z_min = pg.z();

                                    if (pg.x() > x_max)
                                        x_max = pg.x();
                                    if (pg.y() > y_max)
                                        y_max = pg.y();
                                    if (pg.z() > z_max)
                                        z_max = pg.z();
                                }
                                auto &points_from = sessions[index_session_from].point_clouds_container.point_clouds[index_from];
                                std::vector<Eigen::Vector3d> ground_truth;
                                for (const auto &p : points_from.points_local)
                                {
                                    auto pg = points_from.m_pose * p;
                                    if (pg.x() > x_min && pg.x() < x_max)
                                    {
                                        if (pg.y() > y_min && pg.y() < y_max)
                                        {
                                            if (pg.z() > z_min && pg.z() < z_max)
                                                ground_truth.push_back(p);
                                        }
                                    }
                                }

                                int number_of_iterations = 30;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                std::vector<Eigen::Vector3d> source = sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds[edges[index_active_edge].index_to].points_local;
                                std::vector<Eigen::Vector3d> target = ground_truth; // sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from].points_local;

                                if (icp.compute(source, target, sr, number_of_iterations, m_pose))
                                {
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                                }
                            }
                            else
                            {

                                int number_of_iterations = 30;
                                PairWiseICP icp;
                                auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                                std::vector<Eigen::Vector3d> source = sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds[edges[index_active_edge].index_to].points_local;
                                std::vector<Eigen::Vector3d> target = sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from].points_local;

                                if (icp.compute(source, target, sr, number_of_iterations, m_pose))
                                {
                                    edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                                }
                            }
                        }
                    }
#if 0
                    if (ImGui::Button("Save src"))
                    {
                        const auto output_file_name = mandeye::fd::SaveFileDialog("Output file name", mandeye::fd::LAS_LAZ_filter, ".laz");
                        std::cout << "laz file to save: '" << output_file_name << "'" << std::endl;

                        if (output_file_name.size() > 0)
                        {
                            std::vector<Eigen::Vector3d> source = sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds[edges[index_active_edge].index_to].points_local;
                            std::vector<Eigen::Vector3d> pointcloud;
                            std::vector<unsigned short> intensity;
                            std::vector<double> timestamps;

                            for (size_t i = 0; i < source.size(); i++)
                            {
                                pointcloud.push_back(source[i]);
                                intensity.push_back(0);
                                timestamps.push_back(0.0);
                            }

                            exportLaz(
                                output_file_name[0],
                                pointcloud,
                                intensity,
                                timestamps);
                        }

                        /*int number_of_iterations = 30;
                        PairWiseICP icp;
                        auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                        std::vector<Eigen::Vector3d> source = sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds[edges[index_active_edge].index_to].points_local;
                        std::vector<Eigen::Vector3d> target = sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds[edges[index_active_edge].index_from].points_local;

                        if (icp.compute(source, target, sr, number_of_iterations, m_pose))
                        {
                            edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
                        }*/
                        //save
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Save trg (transfromed only by rotation)"))
                    {
                    }
#endif
                    //////////////////////////////////
                }
            }
        }

        ImGui::End();
    }
}

void save_trajectories_to_laz(const Session &session, std::string output_file_name, float curve_consecutive_distance_meters, float not_curve_consecutive_distance_meters, bool is_trajectory_export_downsampling)
{
    std::vector<Eigen::Vector3d> pointcloud;
    std::vector<unsigned short> intensity;
    std::vector<double> timestamps;

    float consecutive_distance = 0;
    for (auto &p : session.point_clouds_container.point_clouds)
    {
        if (p.visible)
        {
            for (size_t i = 0; i < p.local_trajectory.size(); i++)
            {
                const auto &pp = p.local_trajectory[i].m_pose.translation();
                Eigen::Vector3d vp;
                vp = p.m_pose * pp; // + session.point_clouds_container.offset;

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
                        double angle_deg = fabs(acos(v1.dot(v2) / (v1.norm() * v2.norm())) * RAD_TO_DEG);

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
                    timestamps.push_back(p.local_trajectory[i].timestamps.first);
                }
                else
                {
                    if (consecutive_distance >= tol)
                    {
                        consecutive_distance = 0;
                        pointcloud.push_back(vp);
                        intensity.push_back(0);
                        timestamps.push_back(p.local_trajectory[i].timestamps.first);
                    }
                }
            }
        }
    }
    // if (!exportLaz(output_file_name, pointcloud, intensity, gnss.offset_x, gnss.offset_y, gnss.offset_alt))
    if (!exportLaz(output_file_name, pointcloud, intensity, timestamps, session.point_clouds_container.offset.x(), session.point_clouds_container.offset.y(), session.point_clouds_container.offset.z()))
    {
        std::cout << "problem with saving file: " << output_file_name << std::endl;
    }
}

void createDXFPolyline(const std::string &filename, const std::vector<Eigen::Vector3d> &points)
{
    std::ofstream dxfFile(filename);
    dxfFile << std::setprecision(20);
    if (!dxfFile.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    // DXF header
    dxfFile << "0\nSECTION\n2\nHEADER\n0\nENDSEC\n";
    dxfFile << "0\nSECTION\n2\nTABLES\n0\nENDSEC\n";

    // Start the ENTITIES section
    dxfFile << "0\nSECTION\n2\nENTITIES\n";

    // Start the POLYLINE entity
    dxfFile << "0\nPOLYLINE\n";
    dxfFile << "8\n0\n";  // Layer 0
    dxfFile << "66\n1\n"; // Indicates the presence of vertices
    dxfFile << "70\n8\n"; // 1 = Open polyline

    // Write the VERTEX entities
    for (const auto &point : points)
    {
        dxfFile << "0\nVERTEX\n";
        dxfFile << "8\n0\n"; // Layer 0
        dxfFile << "10\n"
                << point.x() << "\n"; // X coordinate
        dxfFile << "20\n"
                << point.y() << "\n"; // Y coordinate
        dxfFile << "30\n"
                << point.z() << "\n"; // Z coordinate
    }

    // End the POLYLINE
    dxfFile << "0\nSEQEND\n";

    // End the ENTITIES section
    dxfFile << "0\nENDSEC\n";

    // End the DXF file
    dxfFile << "0\nEOF\n";

    dxfFile.close();
    std::cout << "DXF file created: " << filename << std::endl;
}

void save_trajectories(
    Session &session, std::string output_file_name, float curve_consecutive_distance_meters,
    float not_curve_consecutive_distance_meters, bool is_trajectory_export_downsampling,
    bool write_lidar_timestamp, bool write_unix_timestamp, bool use_quaternions,
    bool save_to_dxf)
{
    std::ofstream outfile;
    if (!save_to_dxf)
    {
        outfile.open(output_file_name);
    }
    if (save_to_dxf || outfile.good())
    {
        float consecutive_distance = 0;
        std::vector<Eigen::Vector3d> polylinePoints;
        for (auto &p : session.point_clouds_container.point_clouds)
        {
            if (p.visible)
            {
                for (size_t i = 0; i < p.local_trajectory.size(); i++)
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
                            double angle_deg = fabs(acos(v1.dot(v2) / (v1.norm() * v2.norm())) * RAD_TO_DEG);

                            if (angle_deg > 10.0)
                                is_curve = true;
                        }
                    }
                    double tol = not_curve_consecutive_distance_meters;

                    if (is_curve)
                        tol = curve_consecutive_distance_meters;

                    if (!is_trajectory_export_downsampling || (is_trajectory_export_downsampling && consecutive_distance >= tol))
                    {
                        if (is_trajectory_export_downsampling)
                            consecutive_distance = 0;
                        if (save_to_dxf)
                            polylinePoints.push_back(pose.translation());
                        else
                        {
                            outfile << std::setprecision(20);

                            if (write_lidar_timestamp)
                                outfile << p.local_trajectory[i].timestamps.first << ",";
                            if (write_unix_timestamp)
                                outfile << p.local_trajectory[i].timestamps.second << ",";

                            outfile << pose(0, 3) << "," << pose(1, 3) << "," << pose(2, 3) << ",";
                            if (use_quaternions)
                            {
                                Eigen::Quaterniond q(pose.rotation());
                                outfile << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << std::endl;
                            }
                            else
                                outfile << pose(0, 0) << "," << pose(0, 1) << "," << pose(0, 2) << "," << pose(1, 0) << "," << pose(1, 1) << "," << pose(1, 2) << "," << pose(2, 0) << "," << pose(2, 1) << "," << pose(2, 2) << std::endl;
                        }
                    }
                }
            }
        }
        if (!save_to_dxf)
            outfile.close();
        else
            createDXFPolyline(output_file_name, polylinePoints);
    }
}

bool save_project_settings(const std::string &file_name, const ProjectSettings &_project_settings)
{
    std::cout << "saving file: '" << file_name << "'" << std::endl;

    nlohmann::json jj;

    nlohmann::json jsession_file_names;
    for (const auto &pc : _project_settings.session_file_names)
    {
        nlohmann::json jfn{
            {"session_file_name", pc}};
        jsession_file_names.push_back(jfn);
    }
    jj["session_file_names"] = jsession_file_names;

    nlohmann::json jloop_closure_edges;
    for (const auto &edge : edges)
    {
        nlohmann::json jloop_closure_edge{
            {"px", edge.relative_pose_tb.px},
            {"py", edge.relative_pose_tb.py},
            {"pz", edge.relative_pose_tb.pz},
            {"om", edge.relative_pose_tb.om},
            {"fi", edge.relative_pose_tb.fi},
            {"ka", edge.relative_pose_tb.ka},
            {"w_px", edge.relative_pose_tb_weights.px},
            {"w_py", edge.relative_pose_tb_weights.py},
            {"w_pz", edge.relative_pose_tb_weights.pz},
            {"w_om", edge.relative_pose_tb_weights.om},
            {"w_fi", edge.relative_pose_tb_weights.fi},
            {"w_ka", edge.relative_pose_tb_weights.ka},
            {"index_from", edge.index_from},
            {"index_to", edge.index_to},
            {"is_fixed_px", edge.is_fixed_px},
            {"is_fixed_py", edge.is_fixed_py},
            {"is_fixed_pz", edge.is_fixed_pz},
            {"is_fixed_om", edge.is_fixed_om},
            {"is_fixed_fi", edge.is_fixed_fi},
            {"is_fixed_ka", edge.is_fixed_ka},
            {"index_session_from", edge.index_session_from},
            {"index_session_to", edge.index_session_to},
        };
        jloop_closure_edges.push_back(jloop_closure_edge);
    }
    jj["loop_closure_edges"] = jloop_closure_edges;

    std::ofstream fs(file_name);
    if (!fs.good())
        return false;
    fs << jj.dump(2);
    fs.close();

    return true;
}

void update_timestamp_offset()
{
    std::cout << "update_timestamp" << std::endl;
    time_stamp_offset = 0.0;

    for (const auto &s : sessions)
    {
        if (!s.point_clouds_container.point_clouds.empty() &&
            !s.point_clouds_container.point_clouds[0].local_trajectory.empty())
        {
            double ts = s.point_clouds_container.point_clouds[0].local_trajectory[0].timestamps.first;
            if (ts > time_stamp_offset)
                time_stamp_offset = ts;
        }
    }

    std::cout << "new time_stamp_offset = " << time_stamp_offset << std::endl;
}

bool optimize(std::vector<Session> &sessions)
{
    for (auto &session : sessions)
    {
        // std::cout << session.point_clouds_container.point_clouds.size() << std::endl;
        for (auto &pc : session.point_clouds_container.point_clouds)
            pc.m_pose_temp = pc.m_pose;
    }

    std::vector<int> sums;
    sums.push_back(0);
    int sum = 0;
    std::vector<bool> vfixed_x;
    std::vector<bool> vfixed_y;
    std::vector<bool> vfixed_z;
    std::vector<bool> vfixed_om;
    std::vector<bool> vfixed_fi;
    std::vector<bool> vfixed_ka;

    for (size_t i = 0; i < sessions.size(); i++)
    {
        sum += sessions[i].point_clouds_container.point_clouds.size();
        sums.push_back(sum);

        for (size_t j = 0; j < sessions[i].point_clouds_container.point_clouds.size(); j++)
        {
            vfixed_x.push_back(sessions[i].point_clouds_container.point_clouds[j].fixed_x);
            vfixed_y.push_back(sessions[i].point_clouds_container.point_clouds[j].fixed_y);
            vfixed_z.push_back(sessions[i].point_clouds_container.point_clouds[j].fixed_z);

            vfixed_om.push_back(sessions[i].point_clouds_container.point_clouds[j].fixed_om);
            vfixed_fi.push_back(sessions[i].point_clouds_container.point_clouds[j].fixed_fi);
            vfixed_ka.push_back(sessions[i].point_clouds_container.point_clouds[j].fixed_ka);
        }
    }

    ///////////////////////////////////////////////////////////////////
    // graph slam
    bool is_ok = true;
    std::vector<Eigen::Affine3d> m_poses;
    std::vector<Eigen::Affine3d> poses_motion_model;
    std::vector<int> index_trajectory;

    for (size_t j = 0; j < sessions.size(); j++)
    {
        for (size_t i = 0; i < sessions[j].point_clouds_container.point_clouds.size(); i++)
        {
            m_poses.push_back(sessions[j].point_clouds_container.point_clouds[i].m_pose);
            poses_motion_model.push_back(sessions[j].point_clouds_container.point_clouds[i].m_initial_pose);
            // poses_motion_model.push_back(sessions[j].point_clouds_container.point_clouds[i].m_pose);
            index_trajectory.push_back(j);
        }
    }

    std::vector<TaitBryanPose> poses;

    bool is_wc = true;
    bool is_cw = false;
    int iterations = 1;
    bool is_fix_first_node = true;
    std::vector<int> indexes_ground_truth;
    for (size_t j = 0; j < sessions.size(); j++)
    {
        for (size_t i = 0; i < sessions[j].point_clouds_container.point_clouds.size(); i++)
        {
            if (is_wc)
            {
                poses.push_back(pose_tait_bryan_from_affine_matrix(sessions[j].point_clouds_container.point_clouds[i].m_pose));
            }
            else if (is_cw)
            {
                poses.push_back(pose_tait_bryan_from_affine_matrix(sessions[j].point_clouds_container.point_clouds[i].m_pose.inverse()));
            }
            if (sessions[j].is_ground_truth)
            {
                indexes_ground_truth.push_back(poses.size() - 1);
            }
        }
    }

    std::vector<Edge> all_edges;

    // motion model edges;
    double angle = 0.1 * DEG_TO_RAD;
    double wangle = 1.0 / (angle * angle);

    for (size_t i = 1; i < poses_motion_model.size(); i++)
    {
        if (index_trajectory[i - 1] == index_trajectory[i])
        {
            Eigen::Affine3d m_rel = poses_motion_model[i - 1].inverse() * poses_motion_model[i];
            Edge edge;
            edge.index_from = i - 1;
            edge.index_to = i;
            edge.index_session_from = -1;
            edge.index_session_to = -1;
            edge.relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_rel);
            edge.relative_pose_tb_weights.om = wangle;
            edge.relative_pose_tb_weights.fi = wangle;
            edge.relative_pose_tb_weights.ka = wangle;
            edge.relative_pose_tb_weights.px = 1000000.0;
            edge.relative_pose_tb_weights.py = 1000000.0;
            edge.relative_pose_tb_weights.pz = 1000000.0;
            all_edges.push_back(edge);
        }
    }

    for (size_t i = 0; i < sessions.size(); i++)
    {
        for (size_t j = 0; j < sessions[i].pose_graph_loop_closure.edges.size(); j++)
        {
            Edge edge;
            edge.index_from = sessions[i].pose_graph_loop_closure.edges[j].index_from + sums[i];
            edge.index_to = sessions[i].pose_graph_loop_closure.edges[j].index_to + sums[i];
            edge.relative_pose_tb = sessions[i].pose_graph_loop_closure.edges[j].relative_pose_tb;
            edge.relative_pose_tb_weights = sessions[i].pose_graph_loop_closure.edges[j].relative_pose_tb_weights;
            // edge.is_fixed_fi = ToDo
            all_edges.push_back(edge);
        }
    }

    for (size_t i = 0; i < edges.size(); i++)
    {
        Edge edge;
        edge.index_from = edges[i].index_from + sums[edges[i].index_session_from];
        edge.index_to = edges[i].index_to + sums[edges[i].index_session_to];
        edge.relative_pose_tb = edges[i].relative_pose_tb;
        edge.relative_pose_tb_weights = edges[i].relative_pose_tb_weights;
        // edge.is_fixed_fi = ToDo
        all_edges.push_back(edge);
    }

    for (int iter = 0; iter < iterations; iter++)
    {
        // std::cout << "iteration " << iter + 1 << " of " << iterations << std::endl;
        std::vector<Eigen::Triplet<double>> tripletListA;
        std::vector<Eigen::Triplet<double>> tripletListP;
        std::vector<Eigen::Triplet<double>> tripletListB;

        for (size_t i = 0; i < all_edges.size(); i++)
        {
            Eigen::Matrix<double, 6, 1> delta;
            Eigen::Matrix<double, 6, 12, Eigen::RowMajor> jacobian;
            // auto relative_pose = pose_tait_bryan_from_affine_matrix(all_edges[i].relative_pose_tb);
            if (is_wc)
            {
                relative_pose_obs_eq_tait_bryan_wc_case1(
                    delta,
                    poses[all_edges[i].index_from].px,
                    poses[all_edges[i].index_from].py,
                    poses[all_edges[i].index_from].pz,
                    normalize_angle(poses[all_edges[i].index_from].om),
                    normalize_angle(poses[all_edges[i].index_from].fi),
                    normalize_angle(poses[all_edges[i].index_from].ka),
                    poses[all_edges[i].index_to].px,
                    poses[all_edges[i].index_to].py,
                    poses[all_edges[i].index_to].pz,
                    normalize_angle(poses[all_edges[i].index_to].om),
                    normalize_angle(poses[all_edges[i].index_to].fi),
                    normalize_angle(poses[all_edges[i].index_to].ka),
                    all_edges[i].relative_pose_tb.px,
                    all_edges[i].relative_pose_tb.py,
                    all_edges[i].relative_pose_tb.pz,
                    normalize_angle(all_edges[i].relative_pose_tb.om),
                    normalize_angle(all_edges[i].relative_pose_tb.fi),
                    normalize_angle(all_edges[i].relative_pose_tb.ka));
                relative_pose_obs_eq_tait_bryan_wc_case1_jacobian(jacobian,
                                                                  poses[all_edges[i].index_from].px,
                                                                  poses[all_edges[i].index_from].py,
                                                                  poses[all_edges[i].index_from].pz,
                                                                  normalize_angle(poses[all_edges[i].index_from].om),
                                                                  normalize_angle(poses[all_edges[i].index_from].fi),
                                                                  normalize_angle(poses[all_edges[i].index_from].ka),
                                                                  poses[all_edges[i].index_to].px,
                                                                  poses[all_edges[i].index_to].py,
                                                                  poses[all_edges[i].index_to].pz,
                                                                  normalize_angle(poses[all_edges[i].index_to].om),
                                                                  normalize_angle(poses[all_edges[i].index_to].fi),
                                                                  normalize_angle(poses[all_edges[i].index_to].ka));
            }
            else if (is_cw)
            {
                relative_pose_obs_eq_tait_bryan_cw_case1(
                    delta,
                    poses[all_edges[i].index_from].px,
                    poses[all_edges[i].index_from].py,
                    poses[all_edges[i].index_from].pz,
                    normalize_angle(poses[all_edges[i].index_from].om),
                    normalize_angle(poses[all_edges[i].index_from].fi),
                    normalize_angle(poses[all_edges[i].index_from].ka),
                    poses[all_edges[i].index_to].px,
                    poses[all_edges[i].index_to].py,
                    poses[all_edges[i].index_to].pz,
                    normalize_angle(poses[all_edges[i].index_to].om),
                    normalize_angle(poses[all_edges[i].index_to].fi),
                    normalize_angle(poses[all_edges[i].index_to].ka),
                    all_edges[i].relative_pose_tb.px,
                    all_edges[i].relative_pose_tb.py,
                    all_edges[i].relative_pose_tb.pz,
                    normalize_angle(all_edges[i].relative_pose_tb.om),
                    normalize_angle(all_edges[i].relative_pose_tb.fi),
                    normalize_angle(all_edges[i].relative_pose_tb.ka));
                relative_pose_obs_eq_tait_bryan_cw_case1_jacobian(jacobian,
                                                                  poses[all_edges[i].index_from].px,
                                                                  poses[all_edges[i].index_from].py,
                                                                  poses[all_edges[i].index_from].pz,
                                                                  normalize_angle(poses[all_edges[i].index_from].om),
                                                                  normalize_angle(poses[all_edges[i].index_from].fi),
                                                                  normalize_angle(poses[all_edges[i].index_from].ka),
                                                                  poses[all_edges[i].index_to].px,
                                                                  poses[all_edges[i].index_to].py,
                                                                  poses[all_edges[i].index_to].pz,
                                                                  normalize_angle(poses[all_edges[i].index_to].om),
                                                                  normalize_angle(poses[all_edges[i].index_to].fi),
                                                                  normalize_angle(poses[all_edges[i].index_to].ka));
            }

            int ir = tripletListB.size();

            int ic_1 = all_edges[i].index_from * 6;
            int ic_2 = all_edges[i].index_to * 6;

            for (size_t row = 0; row < 6; row++)
            {
                tripletListA.emplace_back(ir + row, ic_1, -jacobian(row, 0));
                tripletListA.emplace_back(ir + row, ic_1 + 1, -jacobian(row, 1));
                tripletListA.emplace_back(ir + row, ic_1 + 2, -jacobian(row, 2));
                tripletListA.emplace_back(ir + row, ic_1 + 3, -jacobian(row, 3));
                tripletListA.emplace_back(ir + row, ic_1 + 4, -jacobian(row, 4));
                tripletListA.emplace_back(ir + row, ic_1 + 5, -jacobian(row, 5));

                tripletListA.emplace_back(ir + row, ic_2, -jacobian(row, 6));
                tripletListA.emplace_back(ir + row, ic_2 + 1, -jacobian(row, 7));
                tripletListA.emplace_back(ir + row, ic_2 + 2, -jacobian(row, 8));
                tripletListA.emplace_back(ir + row, ic_2 + 3, -jacobian(row, 9));
                tripletListA.emplace_back(ir + row, ic_2 + 4, -jacobian(row, 10));
                tripletListA.emplace_back(ir + row, ic_2 + 5, -jacobian(row, 11));
            }

            tripletListB.emplace_back(ir, 0, delta(0, 0));
            tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
            tripletListB.emplace_back(ir + 2, 0, delta(2, 0));
            tripletListB.emplace_back(ir + 3, 0, normalize_angle(delta(3, 0)));
            tripletListB.emplace_back(ir + 4, 0, normalize_angle(delta(4, 0)));
            tripletListB.emplace_back(ir + 5, 0, normalize_angle(delta(5, 0)));

            // std::cout << "delta(0, 0): " << delta(0, 0) << std::endl;
            // std::cout << "delta(1, 0): " << delta(0, 0) << std::endl;
            // std::cout << "delta(2, 0): " << delta(0, 0) << std::endl;
            // std::cout << "normalize_angle(delta(3, 0)): " << normalize_angle(delta(3, 0)) << std::endl;
            // std::cout << "normalize_angle(delta(4, 0)): " << normalize_angle(delta(4, 0)) << std::endl;
            // std::cout << "normalize_angle(delta(5, 0)): " << normalize_angle(delta(5, 0)) << std::endl;

            // for (int r = 0; r < 6; r++) {
            //     for (int c = 0; c < 6; c++) {
            //         tripletListP.emplace_back(ir + r, ir + c, edges[i].information_matrix.coeffRef(r, c));
            //    }
            // }

            // tripletListP.emplace_back(ir    , ir    , get_cauchy_w(delta(0, 0), 10));
            // tripletListP.emplace_back(ir + 1, ir + 1, get_cauchy_w(delta(1, 0), 10));
            // tripletListP.emplace_back(ir + 2, ir + 2, get_cauchy_w(delta(2, 0), 10));
            // tripletListP.emplace_back(ir + 3, ir + 3, get_cauchy_w(delta(3, 0), 10));
            // tripletListP.emplace_back(ir + 4, ir + 4, get_cauchy_w(delta(4, 0), 10));
            // tripletListP.emplace_back(ir + 5, ir + 5, get_cauchy_w(delta(5, 0), 10));

            // if (sessions[all_edges[i].index_session_from].is_ground_truth || sessions[all_edges[i].index_session_to].is_ground_truth)
            //{
            //     tripletListP.emplace_back(ir, ir, all_edges[i].relative_pose_tb_weights.px);
            //     tripletListP.emplace_back(ir + 1, ir + 1, all_edges[i].relative_pose_tb_weights.py);
            //     tripletListP.emplace_back(ir + 2, ir + 2, all_edges[i].relative_pose_tb_weights.pz);
            //     tripletListP.emplace_back(ir + 3, ir + 3, all_edges[i].relative_pose_tb_weights.om);
            //     tripletListP.emplace_back(ir + 4, ir + 4, all_edges[i].relative_pose_tb_weights.fi);
            //     tripletListP.emplace_back(ir + 5, ir + 5, all_edges[i].relative_pose_tb_weights.ka);
            // }
            // else
            //{

            tripletListP.emplace_back(ir, ir, all_edges[i].relative_pose_tb_weights.px);
            tripletListP.emplace_back(ir + 1, ir + 1, all_edges[i].relative_pose_tb_weights.py);
            tripletListP.emplace_back(ir + 2, ir + 2, all_edges[i].relative_pose_tb_weights.pz);
            tripletListP.emplace_back(ir + 3, ir + 3, all_edges[i].relative_pose_tb_weights.om);
            tripletListP.emplace_back(ir + 4, ir + 4, all_edges[i].relative_pose_tb_weights.fi);
            tripletListP.emplace_back(ir + 5, ir + 5, all_edges[i].relative_pose_tb_weights.ka);

            // tripletListP.emplace_back(ir, ir, all_edges[i].relative_pose_tb_weights.px * get_cauchy_w(delta(0, 0), 1));
            // tripletListP.emplace_back(ir + 1, ir + 1, all_edges[i].relative_pose_tb_weights.py * get_cauchy_w(delta(1, 0), 1));
            // tripletListP.emplace_back(ir + 2, ir + 2, all_edges[i].relative_pose_tb_weights.pz * get_cauchy_w(delta(2, 0), 1));
            // tripletListP.emplace_back(ir + 3, ir + 3, all_edges[i].relative_pose_tb_weights.om * get_cauchy_w(delta(3, 0), 1));
            // tripletListP.emplace_back(ir + 4, ir + 4, all_edges[i].relative_pose_tb_weights.fi * get_cauchy_w(delta(4, 0), 1));
            // tripletListP.emplace_back(ir + 5, ir + 5, all_edges[i].relative_pose_tb_weights.ka * get_cauchy_w(delta(5, 0), 1));
            // }
        }
        if (is_fix_first_node)
        {
            int ir = tripletListB.size();
            tripletListA.emplace_back(ir, 0, 1);
            tripletListA.emplace_back(ir + 1, 1, 1);
            tripletListA.emplace_back(ir + 2, 2, 1);
            tripletListA.emplace_back(ir + 3, 3, 1);
            tripletListA.emplace_back(ir + 4, 4, 1);
            tripletListA.emplace_back(ir + 5, 5, 1);

            tripletListP.emplace_back(ir, ir, 0.0001);
            tripletListP.emplace_back(ir + 1, ir + 1, 0.0001);
            tripletListP.emplace_back(ir + 2, ir + 2, 0.0001);
            tripletListP.emplace_back(ir + 3, ir + 3, 0.0001);
            tripletListP.emplace_back(ir + 4, ir + 4, 0.0001);
            tripletListP.emplace_back(ir + 5, ir + 5, 0.0001);

            tripletListB.emplace_back(ir, 0, 0);
            tripletListB.emplace_back(ir + 1, 0, 0);
            tripletListB.emplace_back(ir + 2, 0, 0);
            tripletListB.emplace_back(ir + 3, 0, 0);
            tripletListB.emplace_back(ir + 4, 0, 0);
            tripletListB.emplace_back(ir + 5, 0, 0);

            for (size_t i = 1; i < index_trajectory.size(); i++)
            {
                if (index_trajectory[i - 1] != index_trajectory[i])
                {
                    ir = tripletListB.size();
                    tripletListA.emplace_back(ir, i * 6 + 0, 1);
                    tripletListA.emplace_back(ir + 1, i * 6 + 1, 1);
                    tripletListA.emplace_back(ir + 2, i * 6 + 2, 1);
                    tripletListA.emplace_back(ir + 3, i * 6 + 3, 1);
                    tripletListA.emplace_back(ir + 4, i * 6 + 4, 1);
                    tripletListA.emplace_back(ir + 5, i * 6 + 5, 1);

                    tripletListP.emplace_back(ir, ir, 0.0001);
                    tripletListP.emplace_back(ir + 1, ir + 1, 0.0001);
                    tripletListP.emplace_back(ir + 2, ir + 2, 0.0001);
                    tripletListP.emplace_back(ir + 3, ir + 3, 0.0001);
                    tripletListP.emplace_back(ir + 4, ir + 4, 0.0001);
                    tripletListP.emplace_back(ir + 5, ir + 5, 0.0001);

                    tripletListB.emplace_back(ir, 0, 0);
                    tripletListB.emplace_back(ir + 1, 0, 0);
                    tripletListB.emplace_back(ir + 2, 0, 0);
                    tripletListB.emplace_back(ir + 3, 0, 0);
                    tripletListB.emplace_back(ir + 4, 0, 0);
                    tripletListB.emplace_back(ir + 5, 0, 0);
                }
            }
        }

        /*double angle = 1.0 * DEG_TO_RAD;
        double wangle = 1.0 / (angle * angle);

        for (size_t index = 0; index < indexes_ground_truth.size(); index++)
        {
            int ir = tripletListB.size();
            int ic = indexes_ground_truth[index] * 6;
            tripletListA.emplace_back(ir, ic + 0, 1);
            tripletListA.emplace_back(ir + 1, ic + 1, 1);
            tripletListA.emplace_back(ir + 2, ic + 2, 1);
            tripletListA.emplace_back(ir + 3, ic + 3, 1);
            tripletListA.emplace_back(ir + 4, ic + 4, 1);
            tripletListA.emplace_back(ir + 5, ic + 5, 1);

            tripletListP.emplace_back(ir    , ir    , 10000.0);
            tripletListP.emplace_back(ir + 1, ir + 1, 10000.0);
            tripletListP.emplace_back(ir + 2, ir + 2, 10000.0);
            tripletListP.emplace_back(ir + 3, ir + 3, wangle);
            tripletListP.emplace_back(ir + 4, ir + 4, wangle);
            tripletListP.emplace_back(ir + 5, ir + 5, wangle);

            tripletListB.emplace_back(ir, 0, 0);
            tripletListB.emplace_back(ir + 1, 0, 0);
            tripletListB.emplace_back(ir + 2, 0, 0);
            tripletListB.emplace_back(ir + 3, 0, 0);
            tripletListB.emplace_back(ir + 4, 0, 0);
            tripletListB.emplace_back(ir + 5, 0, 0);
        }*/

        // gnss
        // for (const auto &pc : point_clouds_container.point_clouds)
        /*for (size_t index_pose = 0; index_pose < point_clouds_container.point_clouds.size(); index_pose++)
        {
            const auto &pc = point_clouds_container.point_clouds[index_pose];
            for (size_t i = 0; i < gnss.gnss_poses.size(); i++)
            {
                double time_stamp = gnss.gnss_poses[i].timestamp;

                auto it = std::lower_bound(pc.local_trajectory.begin(), pc.local_trajectory.end(),
                                           time_stamp, [](const PointCloud::LocalTrajectoryNode &lhs, const double &time) -> bool
                                           { return lhs.timestamp < time; });

                int index = it - pc.local_trajectory.begin();

                if (index > 0 && index < pc.local_trajectory.size())
                {

                    if (fabs(time_stamp - pc.local_trajectory[index].timestamp) < 10e12)
                    {

                        Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
                        TaitBryanPose pose_s;
                        pose_s = pose_tait_bryan_from_affine_matrix(m_poses[index_pose]);
                        Eigen::Vector3d p_s = pc.local_trajectory[index].m_pose.translation();
                        point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian, pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                                               p_s.x(), p_s.y(), p_s.z());

                        double delta_x;
                        double delta_y;
                        double delta_z;
                        Eigen::Vector3d p_t(gnss.gnss_poses[i].x - gnss.offset_x, gnss.gnss_poses[i].y - gnss.offset_y, gnss.gnss_poses[i].alt - gnss.offset_alt);
                        point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z,
                                                                      pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                                      p_s.x(), p_s.y(), p_s.z(), p_t.x(), p_t.y(), p_t.z());

                        std::cout << " delta_x " << delta_x << " delta_y " << delta_y << " delta_z " << delta_z << std::endl;

                        int ir = tripletListB.size();
                        int ic = index_pose * 6;
                        for (int row = 0; row < 3; row++)
                        {
                            for (int col = 0; col < 6; col++)
                            {
                                if (jacobian(row, col) != 0.0)
                                {
                                    tripletListA.emplace_back(ir + row, ic + col, -jacobian(row, col));
                                }
                            }
                        }
                        tripletListP.emplace_back(ir, ir, get_cauchy_w(delta_x, 1));
                        tripletListP.emplace_back(ir + 1, ir + 1, get_cauchy_w(delta_y, 1));
                        tripletListP.emplace_back(ir + 2, ir + 2, get_cauchy_w(delta_z, 1));

                        tripletListB.emplace_back(ir, 0, delta_x);
                        tripletListB.emplace_back(ir + 1, 0, delta_y);
                        tripletListB.emplace_back(ir + 2, 0, delta_z);

                        // jacobian3x6 = get_point_to_point_jacobian_tait_bryan(pose_convention, point_clouds_container.point_clouds[i].m_pose, p_s, p_t);

                        // auto m = pc.m_pose * pc.local_trajectory[index].m_pose;
                        // glVertex3f(m(0, 3), m(1, 3), m(2, 3));
                        // glVertex3f(gnss_poses[i].x - offset_x, gnss_poses[i].y - offset_y, gnss_poses[i].alt - offset_alt);
                    }
                }
            }
        }*/
        //

        for (size_t j = 0; j < sessions.size(); j++)
        {
            for (size_t jj = 0; jj < sessions[j].ground_control_points.gpcs.size(); jj++)
            {
                Eigen::Vector3d p_s = sessions[j].point_clouds_container.point_clouds[sessions[j].ground_control_points.gpcs[jj].index_to_node_inner].local_trajectory[sessions[j].ground_control_points.gpcs[jj].index_to_node_outer].m_pose.translation();
                Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
                TaitBryanPose pose_s;
                pose_s = pose_tait_bryan_from_affine_matrix(sessions[j].point_clouds_container.point_clouds[sessions[j].ground_control_points.gpcs[jj].index_to_node_inner].m_pose);
                point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian, pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                                       p_s.x(), p_s.y(), p_s.z());

                double delta_x;
                double delta_y;
                double delta_z;
                Eigen::Vector3d p_t(sessions[j].ground_control_points.gpcs[jj].x, sessions[j].ground_control_points.gpcs[jj].y, sessions[j].ground_control_points.gpcs[jj].z + sessions[j].ground_control_points.gpcs[jj].lidar_height_above_ground);
                point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z,
                                                              pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                              p_s.x(), p_s.y(), p_s.z(), p_t.x(), p_t.y(), p_t.z());

                int ir = tripletListB.size();
                int ic = sessions[j].ground_control_points.gpcs[jj].index_to_node_inner * 6 + sums[j] * 6;

                for (int row = 0; row < 3; row++)
                {
                    for (int col = 0; col < 6; col++)
                    {
                        if (jacobian(row, col) != 0.0)
                        {
                            tripletListA.emplace_back(ir + row, ic + col, -jacobian(row, col));
                        }
                    }
                }
                tripletListP.emplace_back(ir + 0, ir + 0, (1.0 / (sessions[j].ground_control_points.gpcs[jj].sigma_x * sessions[j].ground_control_points.gpcs[jj].sigma_x)) * get_cauchy_w(delta_x, 1));
                tripletListP.emplace_back(ir + 1, ir + 1, (1.0 / (sessions[j].ground_control_points.gpcs[jj].sigma_y * sessions[j].ground_control_points.gpcs[jj].sigma_y)) * get_cauchy_w(delta_y, 1));
                tripletListP.emplace_back(ir + 2, ir + 2, (1.0 / (sessions[j].ground_control_points.gpcs[jj].sigma_z * sessions[j].ground_control_points.gpcs[jj].sigma_z)) * get_cauchy_w(delta_z, 1));

                tripletListB.emplace_back(ir, 0, delta_x);
                tripletListB.emplace_back(ir + 1, 0, delta_y);
                tripletListB.emplace_back(ir + 2, 0, delta_z);

                std::cout << "gcp: delta_x " << delta_x << " delta_y " << delta_y << " delta_z " << delta_z << std::endl;
            }
        }

        ///////////////////////////////////////////////////////
        for (size_t i = 0; i < sessions.size(); i++)
        {
            for (size_t index_pose = 0; index_pose < sessions[i].point_clouds_container.point_clouds.size(); index_pose++)
            {
                if (sessions[i].point_clouds_container.point_clouds[index_pose].fixed_x)
                {
                    int ir = tripletListB.size();
                    int ic = (index_pose + sums[i]) * 6;
                    tripletListA.emplace_back(ir + 0, ic, 1000000000000);
                    tripletListP.emplace_back(ir, ir, 1);
                    tripletListB.emplace_back(ir, 0, 0);
                }

                if (sessions[i].point_clouds_container.point_clouds[index_pose].fixed_y)
                {
                    int ir = tripletListB.size();
                    int ic = (index_pose + sums[i]) * 6 + 1;
                    tripletListA.emplace_back(ir + 0, ic, 1000000000000);
                    tripletListP.emplace_back(ir, ir, 1);
                    tripletListB.emplace_back(ir, 0, 0);
                }

                if (sessions[i].point_clouds_container.point_clouds[index_pose].fixed_z)
                {
                    int ir = tripletListB.size();
                    int ic = (index_pose + sums[i]) * 6 + 2;
                    tripletListA.emplace_back(ir + 0, ic, 1000000000000);
                    tripletListP.emplace_back(ir, ir, 1);
                    tripletListB.emplace_back(ir, 0, 0);
                }

                if (sessions[i].point_clouds_container.point_clouds[index_pose].fixed_om)
                {
                    int ir = tripletListB.size();
                    int ic = (index_pose + sums[i]) * 6 + 3;
                    tripletListA.emplace_back(ir + 0, ic, 1000000000000);
                    tripletListP.emplace_back(ir, ir, 1);
                    tripletListB.emplace_back(ir, 0, 0);
                }

                if (sessions[i].point_clouds_container.point_clouds[index_pose].fixed_fi)
                {
                    int ir = tripletListB.size();
                    int ic = (index_pose + sums[i]) * 6 + 4;
                    tripletListA.emplace_back(ir + 0, ic, 1000000000000);
                    tripletListP.emplace_back(ir, ir, 1);
                    tripletListB.emplace_back(ir, 0, 0);
                }

                if (sessions[i].point_clouds_container.point_clouds[index_pose].fixed_ka)
                {
                    int ir = tripletListB.size();
                    int ic = (index_pose + sums[i]) * 6 + 5;
                    tripletListA.emplace_back(ir + 0, ic, 1000000000000);
                    tripletListP.emplace_back(ir, ir, 1);
                    tripletListB.emplace_back(ir, 0, 0);
                }

                // std::cout << "add manual_pose_graph_loop_closure.edge" << std::endl;
                // Edge edge;
                // edge.index_from = sessions[i].pose_graph_loop_closure.edges[j].index_from + sums[i];
                // edge.index_to = sessions[i].pose_graph_loop_closure.edges[j].index_to + sums[i];
                // edge.relative_pose_tb = sessions[i].pose_graph_loop_closure.edges[j].relative_pose_tb;
                // edge.relative_pose_tb_weights = sessions[i].pose_graph_loop_closure.edges[j].relative_pose_tb_weights;
                // edge.is_fixed_fi = ToDo
                // all_edges.push_back(edge);
            }
        }

        //////////////////////////////////////////////////////
        // for (size_t i = 0; i < gcps.gpcs.size(); i++)
        //{

        Eigen::SparseMatrix<double> matA(tripletListB.size(), poses.size() * 6);
        Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
        Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

        matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
        matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
        matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

        Eigen::SparseMatrix<double> AtPA(poses.size() * 6, poses.size() * 6);
        Eigen::SparseMatrix<double> AtPB(poses.size() * 6, 1);

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

        // std::cout << "h_x.size(): " << h_x.size() << std::endl;

        // std::cout << "AtPA=AtPB SOLVED" << std::endl;
        //  std::cout << "updates:" << std::endl;
        //  for (size_t i = 0; i < h_x.size(); i += 6)
        //{
        //      std::cout << h_x[i] << "," << h_x[i + 1] << "," << h_x[i + 2] << "," << h_x[i + 3] << "," << h_x[i + 4] << "," << h_x[i + 5] << std::endl;
        //  }

        if (h_x.size() == 6 * poses.size())
        {
            int counter = 0;

            for (size_t i = 0; i < poses.size(); i++)
            {

                TaitBryanPose pose = poses[i];
                pose.px += h_x[counter++] * 0.1;
                pose.py += h_x[counter++] * 0.1;
                pose.pz += h_x[counter++] * 0.1;
                pose.om += h_x[counter++] * 0.1;
                pose.fi += h_x[counter++] * 0.1;
                pose.ka += h_x[counter++] * 0.1;

                if (!vfixed_x[i])
                    poses[i].px = pose.px;
                if (!vfixed_y[i])
                    poses[i].py = pose.py;
                if (!vfixed_z[i])
                    poses[i].pz = pose.pz;
                if (!vfixed_om[i])
                    poses[i].om = pose.om;
                if (!vfixed_fi[i])
                    poses[i].fi = pose.fi;
                if (!vfixed_ka[i])
                    poses[i].ka = pose.ka;

                // if (i == 0 && is_fix_first_node)
                //     poses[i] = pose;
            }
            // std::cout << "optimizing with tait bryan finished" << std::endl;
        }
        else
        {
            std::cout << "optimizing with tait bryan FAILED" << std::endl;
            std::cout << "h_x.size(): " << h_x.size() << " should be: " << 6 * poses.size() << std::endl;
            is_ok = false;
            break;
        }

        if (is_ok)
        {
            for (size_t i = 0; i < m_poses.size(); i++)
            {
                if (is_wc)
                    m_poses[i] = affine_matrix_from_pose_tait_bryan(poses[i]);
                else if (is_cw)
                    m_poses[i] = affine_matrix_from_pose_tait_bryan(poses[i]).inverse();
            }
        }

        if (is_ok)
        {
            int index = 0;
            for (size_t i = 0; i < m_poses.size(); i++)
            {
                if (i > 0)
                {
                    if (index_trajectory[i - 1] != index_trajectory[i])
                        index = 0;
                }
                if (!sessions[index_trajectory[i]].is_ground_truth)
                {
                    sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].m_pose = m_poses[i];
                    sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].pose = pose_tait_bryan_from_affine_matrix(sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].m_pose);
                    sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].gui_translation[0] = sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].pose.px;
                    sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].gui_translation[1] = sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].pose.py;
                    sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].gui_translation[2] = sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].pose.pz;
                    sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].gui_rotation[0] = rad2deg(sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].pose.om);
                    sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].gui_rotation[1] = rad2deg(sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].pose.fi);
                    sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].gui_rotation[2] = rad2deg(sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].pose.ka);
                }
                index++;
            }
        }
    }
    return true;
}

bool revert(std::vector<Session> &sessions)
{
    for (auto &session : sessions)
    {
        for (auto &pc : session.point_clouds_container.point_clouds)
            pc.m_pose = pc.m_pose_temp;
    }
    return true;
}

bool save_results(std::vector<Session> &sessions)
{
    for (auto &session : sessions)
    {
        if (!session.is_ground_truth)
        {
            std::cout << "saving result to: " << session.point_clouds_container.poses_file_name << std::endl;
            session.point_clouds_container.save_poses(fs::path(session.point_clouds_container.poses_file_name).string(), false);
        }
    }
    return true;
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

bool load_project_settings(const std::string &file_name, ProjectSettings &_project_settings)
{
    std::cout << "Opening project file: '" << file_name << "'\n";

    try
    {
        std::ifstream fs(file_name);
        if (!fs.good())
            return false;
        nlohmann::json data = nlohmann::json::parse(fs);
        fs.close();

        _project_settings.session_file_names.clear();

        std::cout << "Contained sessions:\n";

        for (const auto &fn_json : data["session_file_names"])
        {
            std::string fn = fn_json["session_file_name"];
            _project_settings.session_file_names.push_back(fn);
            std::cout << "'" << fn << "'";
            if (!fs::exists(fn))
                std::cout << "  (WARNING: session file does not exist! Please manually adapt path)";
            std::cout << "\n";
        }

        edges.clear();
        for (const auto &edge_json : data["loop_closure_edges"])
        {
            Edge edge;
            edge.index_from = edge_json["index_from"];
            edge.index_to = edge_json["index_to"];
            edge.is_fixed_fi = edge_json["is_fixed_fi"];
            edge.is_fixed_ka = edge_json["is_fixed_ka"];
            edge.is_fixed_om = edge_json["is_fixed_om"];
            edge.is_fixed_px = edge_json["is_fixed_px"];
            edge.is_fixed_py = edge_json["is_fixed_py"];
            edge.is_fixed_pz = edge_json["is_fixed_pz"];
            edge.relative_pose_tb.fi = edge_json["fi"];
            edge.relative_pose_tb.ka = edge_json["ka"];
            edge.relative_pose_tb.om = edge_json["om"];
            edge.relative_pose_tb.px = edge_json["px"];
            edge.relative_pose_tb.py = edge_json["py"];
            edge.relative_pose_tb.pz = edge_json["pz"];
            edge.relative_pose_tb_weights.fi = edge_json["w_fi"];
            edge.relative_pose_tb_weights.ka = edge_json["w_ka"];
            edge.relative_pose_tb_weights.om = edge_json["w_om"];
            edge.relative_pose_tb_weights.px = edge_json["w_px"];
            edge.relative_pose_tb_weights.py = edge_json["w_py"];
            edge.relative_pose_tb_weights.pz = edge_json["w_pz"];
            edge.index_session_from = edge_json["index_session_from"];
            edge.index_session_to = edge_json["index_session_to"];
            edges.push_back(edge);
        }

        std::cout << "Found " << edges.size() << "edges\nOpening done\n";

        return true;
    }
    catch (std::exception &e)
    {
        std::cout << "can't load project settings: " << e.what() << std::endl;
        return false;
    }

    return true;
}

void openProject()
{
    std::string input_file_name = "";
    input_file_name = mandeye::fd::OpenFileDialogOneFile("Open project", mandeye::fd::Project_filter);

    if (input_file_name.size() > 0)
    {
        load_project_settings(fs::path(input_file_name).string(), project_settings);

        std::string newTitle = winTitle + " - " + truncPath(input_file_name);
        glutSetWindowTitle(newTitle.c_str());

        loaded_sessions = false;
        time_stamp_offset = 0.0;
    }
}

void saveProject()
{
    std::string output_file_name = "";
    output_file_name = mandeye::fd::SaveFileDialog("Save project file", mandeye::fd::Project_filter, ".mjp", "project");

    if (output_file_name.size() > 0)
        if (save_project_settings(fs::path(output_file_name).string(), project_settings))
        {
            std::string newTitle = winTitle + " - " + truncPath(output_file_name);
            glutSetWindowTitle(newTitle.c_str());
        }
}

void addSession()
{
    auto input_file_names = mandeye::fd::OpenFileDialog("Add session(s)", mandeye::fd::Session_filter, true);

    if (input_file_names.size() > 0)
    {
        for (const auto &input_file_name : input_file_names)
        {
            std::cout << "Adding session file: '" << input_file_name << "'" << std::endl;
            project_settings.session_file_names.push_back(input_file_name);
        }

        loaded_sessions = false;
        time_stamp_offset = 0.0;
    }
}

void loadSessions()
{
    sessions.clear();
    for (const auto &ps : project_settings.session_file_names)
    {
        Session session;
        session.load(fs::path(ps).string(), is_decimate, bucket_x, bucket_y, bucket_z, calculate_offset);
        sessions.push_back(session);
        if (session.is_ground_truth)
			index_gt = sessions.size() - 1;
    }
    loaded_sessions = true;

    // reorder
    std::vector<Session> sessions_reorder;
    std::vector<std::string> session_file_names_reordered;

    std::map<int, int> map_reorder;
    // project_settings.session_file_names.
    int new_index = 0;
    for (size_t i = 0; i < sessions.size(); i++)
    {
        if (sessions[i].is_ground_truth)
        {
            sessions_reorder.push_back(sessions[i]);
            session_file_names_reordered.push_back(project_settings.session_file_names[i]);
            map_reorder[i] = new_index++;
        }
    }
    for (size_t i = 0; i < sessions.size(); i++)
    {
        if (!sessions[i].is_ground_truth)
        {
            sessions_reorder.push_back(sessions[i]);
            session_file_names_reordered.push_back(project_settings.session_file_names[i]);
            map_reorder[i] = new_index++;
        }
    }
    sessions = sessions_reorder;
    project_settings.session_file_names = session_file_names_reordered;

    for (auto &e : edges)
    {
        e.index_session_from = map_reorder[e.index_session_from];
        e.index_session_to = map_reorder[e.index_session_to];
    }

    std::cout << "sessions reordered, ground truth should be in front" << std::endl;
    for (const auto &s : sessions)
    {
        std::cout << "session: '" << s.session_file_name << "' ground truth [" << int(s.is_ground_truth) << "]" << std::endl;
    }

    // update time_stamp_offset
    std::cout << "update time_stamp_offset" << std::endl;
    for (const auto &s : sessions)
    {
        if (s.point_clouds_container.point_clouds.size() > 0)
        {
            if (s.point_clouds_container.point_clouds[0].local_trajectory.size() > 0)
            {
                if (s.point_clouds_container.point_clouds[0].local_trajectory[0].timestamps.first > time_stamp_offset)
                {
                    time_stamp_offset = s.point_clouds_container.point_clouds[0].local_trajectory[0].timestamps.first;
                }
            }
        }
    }
}

void project_gui()
{
    if (ImGui::Begin("Settings"))
    {
        ImGui::Checkbox("Downsample during load", &is_decimate);
        ImGui::SameLine();
        ImGui::Text("Bucket [m]:");
        ImGui::PushItemWidth(ImGuiNumberWidth);
        ImGui::InputDouble("X##b", &bucket_x, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(xText);
        ImGui::SameLine();
        ImGui::InputDouble("Y##b", &bucket_y, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(yText);
        ImGui::SameLine();
        ImGui::InputDouble("Z##b", &bucket_z, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(zText);
        ImGui::PopItemWidth();

        ImGui::NewLine();

        ImGui::Separator();

        ImGui::Text("Benchmark settings:");

        ImGui::PushItemWidth(ImGuiNumberWidth * 2);

        static double fast_plus = 100000000.0;
        static double fast_plus_plus = 1000000000.0;

        ImGui::InputDouble("Increment", &fast_plus);
        ImGui::InputDouble("Fast increment", &fast_plus_plus);
        ImGui::InputDouble("Timestamp offset", &time_stamp_offset, fast_plus, fast_plus_plus);
        ImGui::PopItemWidth();
        ImGui::SameLine();
        if (ImGui::Button("Set to origin"))
        {
            for (auto &session : sessions)
            {
                int index_point_clouds = -1;
                int index_local_trajectory = -1;
                bool found = false;
                for (size_t a = 0; a < session.point_clouds_container.point_clouds.size(); a++)
                {
                    for (size_t b = 0; b < session.point_clouds_container.point_clouds[a].local_trajectory.size(); b++)
                    {
                        if (session.point_clouds_container.point_clouds[a].local_trajectory[b].timestamps.first > time_stamp_offset)
                        {
                            if (!found)
                            {
                                found = true;
                                index_point_clouds = a;
                                index_local_trajectory = b;
                                break;
                            }
                        }
                    }
                }

                if (index_point_clouds != -1 && index_local_trajectory != -1)
                {
                    auto m1 = session.point_clouds_container.point_clouds[index_point_clouds].m_pose;
                    auto m2 = session.point_clouds_container.point_clouds[index_point_clouds].local_trajectory[index_local_trajectory].m_pose;

                    auto inv = (m1 * m2).inverse();

                    for (size_t index = 0; index < session.point_clouds_container.point_clouds.size(); index++)
                        session.point_clouds_container.point_clouds[index].m_pose = inv * session.point_clouds_container.point_clouds[index].m_pose;
                }
            }
        }

        if (project_settings.session_file_names.size() > 0)
        {
            ImGui::Separator();

            ImGui::Text("Sessions:");

            for (size_t i = 0; i < project_settings.session_file_names.size(); i++)
            {
                ImGui::Text(truncPath(project_settings.session_file_names[i]).c_str());
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(project_settings.session_file_names[i].c_str());

                if (project_settings.session_file_names.size() == sessions.size())
                {
                    ImGui::BeginDisabled(is_loop_closure_gui);
                    {
                        ImGui::SameLine();
                        ImGui::Checkbox(("Visible##" + std::to_string(i)).c_str(), &sessions[i].visible);
                    }
				    ImGui::EndDisabled();

                    ImGui::SameLine();
                    if (ImGui::RadioButton(("Ground truth##" + std::to_string(i)).c_str(), &index_gt, i))
                        if (old_index_gt == i)
                            index_gt = -1; // unselect
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Select session as unmovable reference");

                    ImGui::BeginDisabled(!sessions[i].visible);
                    {
                        ImGui::BeginDisabled(sessions[i].is_ground_truth);
                        {
                            ImGui::SameLine();
                            if (ImGui::RadioButton(("Gizmo##" + std::to_string(i)).c_str(), &index_gizmo, i))
                                if (old_index_gizmo == i)
                                    index_gizmo = -1; // unselect
                        }
                        ImGui::EndDisabled();

                        ImGui::SameLine();
                        ImGui::Checkbox(("Show RGB##" + std::to_string(i)).c_str(), &sessions[i].show_rgb);

                        if (!sessions[i].show_rgb)
                        {
                            ImGui::SameLine();
                            ImGui::ColorEdit3(("Color##" + std::to_string(i)).c_str(), (float *)&sessions[i].render_color, ImGuiColorEditFlags_NoInputs);
                            for (auto &pc : sessions[i].point_clouds_container.point_clouds)
                            {
                                pc.render_color[0] = sessions[i].render_color[0];
                                pc.render_color[1] = sessions[i].render_color[1];
                                pc.render_color[2] = sessions[i].render_color[2];
                                pc.show_color = sessions[i].show_rgb;
                            }
                        }
                        else
                        {
                            for (auto &pc : sessions[i].point_clouds_container.point_clouds)
                                pc.show_color = sessions[i].show_rgb;
                        }
                    }
                    ImGui::EndDisabled();

                    //
                    if (sessions[i].point_clouds_container.point_clouds.size() > 0)
                    {
                        if (sessions[i].point_clouds_container.point_clouds[0].local_trajectory.size() > 0)
                        {
                            if (sessions[i].point_clouds_container.point_clouds[sessions[i].point_clouds_container.point_clouds.size() - 1].local_trajectory.size() > 0)
                            {
                                ImGui::SameLine();

                                int index_last = sessions[i].point_clouds_container.point_clouds.size() - 1;
                                int index_last2 = sessions[i].point_clouds_container.point_clouds[index_last].local_trajectory.size() - 1;

                                ImGui::Text("Timestamp range: <%.0f, %.0f>",
                                            sessions[i].point_clouds_container.point_clouds[0].local_trajectory[0].timestamps.first,
                                            sessions[i].point_clouds_container.point_clouds[index_last].local_trajectory[index_last2].timestamps.first);
                            }
                        }
                    }
                }
            }

            if (project_settings.session_file_names.size() == sessions.size())
            {
                if ((old_index_gt != index_gt) || (old_index_gizmo != index_gizmo))
                {
                    for (size_t i = 0; i < sessions.size(); i++)
                    {
					    sessions[i].is_ground_truth = (i == index_gt);
                        sessions[i].is_gizmo = (i == index_gizmo);
                    }

                    old_index_gt = index_gt;
                    old_index_gizmo = index_gizmo;
                }

                if (index_gizmo != -1 && index_gizmo < sessions.size())
                {
                    // sessions[index_gizmo].is_gizmo = true;
                    m_gizmo[0] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(0, 0);
                    m_gizmo[1] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(1, 0);
                    m_gizmo[2] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(2, 0);
                    m_gizmo[3] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(3, 0);
                    m_gizmo[4] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(0, 1);
                    m_gizmo[5] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(1, 1);
                    m_gizmo[6] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(2, 1);
                    m_gizmo[7] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(3, 1);
                    m_gizmo[8] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(0, 2);
                    m_gizmo[9] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(1, 2);
                    m_gizmo[10] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(2, 2);
                    m_gizmo[11] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(3, 2);
                    m_gizmo[12] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(0, 3);
                    m_gizmo[13] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(1, 3);
                    m_gizmo[14] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(2, 3);
                    m_gizmo[15] = (float)sessions[index_gizmo].point_clouds_container.point_clouds[0].m_pose(3, 3);
                }
            }

            ImGui::BeginDisabled((project_settings.session_file_names.size() < 2) || (index_gizmo == -1));
            {
                ImGui::Checkbox("Gizmo all sessions", &gizmo_all_sessions);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Gizmo will move all sessions except ground truth one");
            }
            ImGui::EndDisabled();

            ImGui::Separator();

            ImGui::NewLine();

            if (project_settings.session_file_names.size() == sessions.size())
            {
                number_visible_sessions = 0;

                bool first_session_index_found = false;
                for (size_t index = 0; index < sessions.size(); index++)
                {
                    if (sessions[index].visible)
                    {
                        number_visible_sessions++;
                        if (!first_session_index_found)
                        {
                            first_session_index = index;
                            second_session_index = index;
                            first_session_index_found = true;
                        }
                        else
                        {
                            second_session_index = index;
                        }
                    }
                }

                if (!is_loop_closure_gui)
                {
                    static int nr_iter = 100;
                    ImGui::SetNextItemWidth(ImGuiNumberWidth);
                    ImGui::InputInt("Number of iterations", &nr_iter);
                    if (nr_iter < 1)
                        nr_iter = 1;

                    std::string bn = "Optimize (number of iterations: " + std::to_string(nr_iter) + ")";

                    if (ImGui::Button(bn.c_str()))
                    {
                        for (int i = 0; i < nr_iter; i++)
                        {
                            std::cout << "Iteration [" << i + 1 << "] of: " << nr_iter << std::endl;
                            optimize(sessions);
                        }
                        optimized = true;
                    }

                    // if (optimized)
                    //{
                    ImGui::SameLine();
                    if (ImGui::Button("Revert"))
                        revert(sessions);
                    ImGui::SameLine();
                    if (ImGui::Button("Save results"))
                        save_results(sessions);
                    //}
                }

                // if (!is_loop_closure_gui && prev_is_loop_closure_gui)
                //{
                //     exit(1);
                // }
            }
        }
    }

    ImGui::End();
}

void display()
{
    ImGuiIO &io = ImGui::GetIO();
    glViewport(0, 0, (GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);

    glClearColor(bg_color.x * bg_color.w, bg_color.y * bg_color.w, bg_color.z * bg_color.w, bg_color.w);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);

    updateCameraTransition();

    viewLocal = Eigen::Affine3f::Identity();

    if (!is_ortho)
    {
        reshape((GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
        glTranslatef(translate_x, translate_y, translate_z);

        // janusz
        if (is_loop_closure_gui)
        {
            // sessions[first_session_index].point_clouds_container.point_clouds.at(index_loop_closure_source).render(false, observation_picking, viewer_decmiate_point_cloud, false, false, false, false, false, false, false, false, false, false, false, false, 100000);
            // sessions[second_session_index].point_clouds_container.point_clouds.at(index_loop_closure_target).render(false, observation_picking, viewer_decmiate_point_cloud, false, false, false, false, false, false, false, false, false, false, false, false, 100000);

            if (first_session_index < sessions[first_session_index].point_clouds_container.point_clouds.size())
            {
                rotation_center.x() = sessions[first_session_index].point_clouds_container.point_clouds[index_loop_closure_source].m_pose.translation().x();
                rotation_center.y() = sessions[first_session_index].point_clouds_container.point_clouds[index_loop_closure_source].m_pose.translation().y();
                rotation_center.z() = sessions[first_session_index].point_clouds_container.point_clouds[index_loop_closure_source].m_pose.translation().z();
            }

            if (manipulate_active_edge)
            {
                if (edges.size() > 0)
                {
                    int index_src = edges[index_active_edge].index_from;
                    Eigen::Affine3d m_src = sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds.at(index_src).m_pose;

                    rotation_center.x() = m_src(0, 3);
                    rotation_center.y() = m_src(1, 3);
                    rotation_center.z() = m_src(2, 3);
                }
            }

            /*if (session.pose_graph_loop_closure.manipulate_active_edge)
            {
                if (session.pose_graph_loop_closure.edges.size() > 0)
                {
                    if (session.pose_graph_loop_closure.index_active_edge < session.pose_graph_loop_closure.edges.size())
                    {
                        rotation_center.x() = session.point_clouds_container.point_clouds[session.pose_graph_loop_closure.edges[session.pose_graph_loop_closure.index_active_edge].index_from].m_pose(0, 3);
                        rotation_center.y() = session.point_clouds_container.point_clouds[session.pose_graph_loop_closure.edges[session.pose_graph_loop_closure.index_active_edge].index_from].m_pose(1, 3);
                        rotation_center.z() = session.point_clouds_container.point_clouds[session.pose_graph_loop_closure.edges[session.pose_graph_loop_closure.index_active_edge].index_from].m_pose(2, 3);
                    }
                }
            }*/
        }

        viewLocal.translate(rotation_center);

        viewLocal.translate(Eigen::Vector3f(translate_x, translate_y, translate_z));
        if (!lock_z)
            viewLocal.rotate(Eigen::AngleAxisf(rotate_x * DEG_TO_RAD, Eigen::Vector3f::UnitX()));
        else
            viewLocal.rotate(Eigen::AngleAxisf(-90.0 * DEG_TO_RAD, Eigen::Vector3f::UnitX()));
        viewLocal.rotate(Eigen::AngleAxisf(rotate_y * DEG_TO_RAD, Eigen::Vector3f::UnitZ()));

        viewLocal.translate(-rotation_center);

        glLoadMatrixf(viewLocal.matrix().data());
    }
    else
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
        pose_tb.ka = -camera_ortho_xy_view_rotation_angle_deg * DEG_TO_RAD;
        auto m = affine_matrix_from_pose_tait_bryan(pose_tb);

        Eigen::Vector3d v_t = m * v;

        gluLookAt(v_eye_t.x(), v_eye_t.y(), v_eye_t.z(),
                  v_center_t.x(), v_center_t.y(), v_center_t.z(),
                  v_t.x(), v_t.y(), v_t.z());
        glm::mat4 lookat = glm::lookAt(glm::vec3(v_eye_t.x(), v_eye_t.y(), v_eye_t.z()),
                                       glm::vec3(v_center_t.x(), v_center_t.y(), v_center_t.z()),
                                       glm::vec3(v_t.x(), v_t.y(), v_t.z()));
        std::copy(&lookat[0][0], &lookat[3][3], m_ortho_gizmo_view);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
    }

    showAxes();

    if (is_loop_closure_gui)
    {
        if (manipulate_active_edge)
        {
            if (edges.size() > 0)
            {
                int index_src = edges[index_active_edge].index_from;
                int index_trg = edges[index_active_edge].index_to;

                Eigen::Affine3d m_src = sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds.at(index_src).m_pose;
                Eigen::Affine3d m_trg = m_src * affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds.at(index_src).render(m_src, viewer_decimate_point_cloud);
                sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds.at(index_trg).render(m_trg, viewer_decimate_point_cloud);
            }
        }
        else
        {
            ObservationPicking observation_picking;
            sessions[first_session_index].point_clouds_container.point_clouds.at(index_loop_closure_source).render(false, observation_picking, viewer_decimate_point_cloud, false, false, false, 100000, false);
            sessions[second_session_index].point_clouds_container.point_clouds.at(index_loop_closure_target).render(false, observation_picking, viewer_decimate_point_cloud, false, false, false, 100000, false);
        }

        // sessions[first_session_index].point_clouds_container.render();

        glBegin(GL_LINE_STRIP);
        for (auto &pc : sessions[first_session_index].point_clouds_container.point_clouds)
        {
            glColor3f(pc.render_color[0], pc.render_color[1], pc.render_color[2]);
            glVertex3f(pc.m_pose(0, 3), pc.m_pose(1, 3), pc.m_pose(2, 3));
        }
        glEnd();

        int i = 0;
        for (auto &pc : sessions[first_session_index].point_clouds_container.point_clouds)
        {
            glColor3f(pc.render_color[0], pc.render_color[1], pc.render_color[2]);
            glRasterPos3f(pc.m_pose(0, 3), pc.m_pose(1, 3), pc.m_pose(2, 3) + 0.1);
            glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, (const unsigned char *)std::to_string(i).c_str());
            i++;
        }

        glBegin(GL_LINE_STRIP);
        for (auto &pc : sessions[second_session_index].point_clouds_container.point_clouds)
        {
            glColor3f(pc.render_color[0], pc.render_color[1], pc.render_color[2]);
            glVertex3f(pc.m_pose(0, 3), pc.m_pose(1, 3), pc.m_pose(2, 3));
        }
        glEnd();

        i = 0;
        for (auto &pc : sessions[second_session_index].point_clouds_container.point_clouds)
        {
            glColor3f(pc.render_color[0], pc.render_color[1], pc.render_color[2]);
            glRasterPos3f(pc.m_pose(0, 3), pc.m_pose(1, 3), pc.m_pose(2, 3) + 0.1);
            glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, (const unsigned char *)std::to_string(i).c_str());
            i++;
        }

        for (size_t i = 0; i < sessions.size(); i++)
        {
            for (size_t j = 0; j < sessions[i].pose_graph_loop_closure.edges.size(); j++)
            {
                int index_src = sessions[i].pose_graph_loop_closure.edges[j].index_from;
                int index_trg = sessions[i].pose_graph_loop_closure.edges[j].index_to;

                glColor3f(0.0f, 0.0f, 1.0f);
                glBegin(GL_LINES);
                auto v1 = sessions[i].point_clouds_container.point_clouds.at(index_src).m_pose.translation();
                auto v2 = sessions[i].point_clouds_container.point_clouds.at(index_trg).m_pose.translation();
                glVertex3f(v1.x(), v1.y(), v1.z());
                glVertex3f(v2.x(), v2.y(), v2.z());

                glVertex3f((v1.x() + v2.x()) * 0.5, (v1.y() + v2.y()) * 0.5, (v1.z() + v2.z()) * 0.5);
                glVertex3f((v1.x() + v2.x()) * 0.5, (v1.y() + v2.y()) * 0.5, (v1.z() + v2.z()) * 0.5 + 10);
                glEnd();

                glRasterPos3f((v1.x() + v2.x()) * 0.5, (v1.y() + v2.y()) * 0.5, (v1.z() + v2.z()) * 0.5 + 10 + 0.1);
                glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, (const unsigned char *)std::to_string(j).c_str());
            }
        }

        for (size_t i = 0; i < edges.size(); i++)
        {
            int index_src = edges[i].index_from;
            int index_trg = edges[i].index_to;

            int index_session_from = edges[i].index_session_from;
            int index_session_to = edges[i].index_session_to;

            if (sessions[index_session_from].is_ground_truth || sessions[index_session_to].is_ground_truth)
                glColor3f(0.0f, 1.0f, 1.0f);
            else
                glColor3f(1.0f, 1.0f, 0.0f);

            glBegin(GL_LINES);
            auto v1 = sessions[index_session_from].point_clouds_container.point_clouds.at(index_src).m_pose.translation();
            auto v2 = sessions[index_session_to].point_clouds_container.point_clouds.at(index_trg).m_pose.translation();
            glVertex3f(v1.x(), v1.y(), v1.z());
            glVertex3f(v2.x(), v2.y(), v2.z());

            glVertex3f((v1.x() + v2.x()) * 0.5, (v1.y() + v2.y()) * 0.5, (v1.z() + v2.z()) * 0.5);
            glVertex3f((v1.x() + v2.x()) * 0.5, (v1.y() + v2.y()) * 0.5, (v1.z() + v2.z()) * 0.5 + 10);
            glEnd();

            glRasterPos3f((v1.x() + v2.x()) * 0.5, (v1.y() + v2.y()) * 0.5, (v1.z() + v2.z()) * 0.5 + 10 + 0.1);
            glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, (const unsigned char *)std::to_string(i).c_str());
        }
    }
    else
    {
        for (auto &session : sessions)
        {
            if (session.visible)
            {
                session.point_clouds_container.render(observation_picking, viewer_decimate_point_cloud, false, false, false, false, false, false, false, false, false, false, false, false, 10000);
                session.ground_control_points.render(session.point_clouds_container);

                ////
                int index_point_clouds = -1;
                int index_local_trajectory = -1;
                bool found = false;
                for (size_t a = 0; a < session.point_clouds_container.point_clouds.size(); a++)
                {
                    for (size_t b = 0; b < session.point_clouds_container.point_clouds[a].local_trajectory.size(); b++)
                    {
                        if (session.point_clouds_container.point_clouds[a].local_trajectory[b].timestamps.first > time_stamp_offset)
                        {
                            if (!found)
                            {
                                found = true;
                                index_point_clouds = a;
                                index_local_trajectory = b;
                                break;
                            }
                        }
                    }
                }

                if (index_point_clouds != -1 && index_local_trajectory != -1)
                {
                    if (index_local_trajectory < session.point_clouds_container.point_clouds[index_point_clouds].local_trajectory.size())
                    {
                        glColor3f(session.point_clouds_container.point_clouds[index_point_clouds].render_color[0], session.point_clouds_container.point_clouds[index_point_clouds].render_color[1], session.point_clouds_container.point_clouds[index_point_clouds].render_color[2]);
                        glBegin(GL_LINES);

                        auto m1 = session.point_clouds_container.point_clouds[index_point_clouds].m_pose;
                        auto m2 = session.point_clouds_container.point_clouds[index_point_clouds].local_trajectory[index_local_trajectory].m_pose;

                        auto v1 = (m1 * m2).translation();

                        glVertex3f(v1.x() - 1.0, v1.y(), v1.z());
                        glVertex3f(v1.x() + 1.0, v1.y(), v1.z());

                        glVertex3f(v1.x(), v1.y() - 1.0, v1.z());
                        glVertex3f(v1.x(), v1.y() + 1.0, v1.z());

                        glVertex3f(v1.x(), v1.y(), v1.z() - 1.0);
                        glVertex3f(v1.x(), v1.y(), v1.z() + 1.0);

                        glEnd();
                    }
                }
            }
        }
    }

    /*if (is_loop_closure_gui)
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
    }*/

    // gnss.render(session.point_clouds_container);

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();
    ImGui::NewFrame();

    ShowMainDockSpace();

    if (!is_loop_closure_gui)
    {
        Eigen::Affine3d prev_pose_manipulated = Eigen::Affine3d::Identity();
        Eigen::Affine3d prev_pose_after_gismo = Eigen::Affine3d::Identity();

        for (size_t i = 0; i < sessions.size(); i++)
        {
            // guizmo_all_sessions;
            if (sessions[i].is_gizmo && !sessions[i].is_ground_truth)
            {
                if (sessions[i].point_clouds_container.point_clouds.size() > 0)
                {
                    prev_pose_manipulated = sessions[i].point_clouds_container.point_clouds[0].m_pose;
                    std::vector<Eigen::Affine3d> all_m_poses;
                    for (size_t j = 0; j < sessions[i].point_clouds_container.point_clouds.size(); j++)
                        all_m_poses.push_back(sessions[i].point_clouds_container.point_clouds[j].m_pose);

                    // if (all_m_poses.size() > 1)
                    //{
                    ImGuiIO &io = ImGui::GetIO();
                    // ImGuizmo -----------------------------------------------
                    ImGuizmo::BeginFrame();
                    ImGuizmo::Enable(true);
                    ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

                    // std::cout << "3" << std::endl;
                    if (!is_ortho)
                    {
                        GLfloat projection[16];
                        glGetFloatv(GL_PROJECTION_MATRIX, projection);

                        GLfloat modelview[16];
                        glGetFloatv(GL_MODELVIEW_MATRIX, modelview);

                        ImGuizmo::Manipulate(modelview, projection, ImGuizmo::TRANSLATE | ImGuizmo::ROTATE_Z | ImGuizmo::ROTATE_X | ImGuizmo::ROTATE_Y, ImGuizmo::WORLD, m_gizmo, NULL);
                    }
                    else
                        ImGuizmo::Manipulate(m_ortho_gizmo_view, m_ortho_projection, ImGuizmo::TRANSLATE_X | ImGuizmo::TRANSLATE_Y | ImGuizmo::ROTATE_Z, ImGuizmo::WORLD, m_gizmo, NULL);

                    sessions[i].point_clouds_container.point_clouds[0].m_pose =
                        Eigen::Map<const Eigen::Matrix4f>(m_gizmo).cast<double>();
                    prev_pose_after_gismo = sessions[i].point_clouds_container.point_clouds[0].m_pose;
                    sessions[i].point_clouds_container.point_clouds[0].pose = pose_tait_bryan_from_affine_matrix(sessions[i].point_clouds_container.point_clouds[0].m_pose);

                    sessions[i].point_clouds_container.point_clouds[0].gui_translation[0] = (float)sessions[i].point_clouds_container.point_clouds[0].pose.px;
                    sessions[i].point_clouds_container.point_clouds[0].gui_translation[1] = (float)sessions[i].point_clouds_container.point_clouds[0].pose.py;
                    sessions[i].point_clouds_container.point_clouds[0].gui_translation[2] = (float)sessions[i].point_clouds_container.point_clouds[0].pose.pz;

                    sessions[i].point_clouds_container.point_clouds[0].gui_rotation[0] = (float)(sessions[i].point_clouds_container.point_clouds[0].pose.om * RAD_TO_DEG);
                    sessions[i].point_clouds_container.point_clouds[0].gui_rotation[1] = (float)(sessions[i].point_clouds_container.point_clouds[0].pose.fi * RAD_TO_DEG);
                    sessions[i].point_clouds_container.point_clouds[0].gui_rotation[2] = (float)(sessions[i].point_clouds_container.point_clouds[0].pose.ka * RAD_TO_DEG);

                    Eigen::Affine3d curr_m_pose = sessions[i].point_clouds_container.point_clouds[0].m_pose;
                    for (size_t j = 1; j < sessions[i].point_clouds_container.point_clouds.size(); j++)
                    {
                        curr_m_pose = curr_m_pose * (all_m_poses[j - 1].inverse() * all_m_poses[j]);
                        sessions[i].point_clouds_container.point_clouds[j].m_pose = curr_m_pose;
                        sessions[i].point_clouds_container.point_clouds[j].pose = pose_tait_bryan_from_affine_matrix(sessions[i].point_clouds_container.point_clouds[j].m_pose);

                        sessions[i].point_clouds_container.point_clouds[j].gui_translation[0] = (float)sessions[i].point_clouds_container.point_clouds[j].pose.px;
                        sessions[i].point_clouds_container.point_clouds[j].gui_translation[1] = (float)sessions[i].point_clouds_container.point_clouds[j].pose.py;
                        sessions[i].point_clouds_container.point_clouds[j].gui_translation[2] = (float)sessions[i].point_clouds_container.point_clouds[j].pose.pz;

                        sessions[i].point_clouds_container.point_clouds[j].gui_rotation[0] = (float)(sessions[i].point_clouds_container.point_clouds[j].pose.om * RAD_TO_DEG);
                        sessions[i].point_clouds_container.point_clouds[j].gui_rotation[1] = (float)(sessions[i].point_clouds_container.point_clouds[j].pose.fi * RAD_TO_DEG);
                        sessions[i].point_clouds_container.point_clouds[j].gui_rotation[2] = (float)(sessions[i].point_clouds_container.point_clouds[j].pose.ka * RAD_TO_DEG);
                    }
                    //}
                }
            }
        }
        if (gizmo_all_sessions)
        {
            for (size_t i = 0; i < sessions.size(); i++)
            {
                // guizmo_all_sessions;
                if (!sessions[i].is_gizmo && !sessions[i].is_ground_truth)
                {
                    std::vector<Eigen::Affine3d> all_m_poses;
                    for (size_t j = 0; j < sessions[i].point_clouds_container.point_clouds.size(); j++)
                        all_m_poses.push_back(sessions[i].point_clouds_container.point_clouds[j].m_pose);

                    Eigen::Affine3d m_rel_org = prev_pose_manipulated.inverse() * sessions[i].point_clouds_container.point_clouds[0].m_pose;

                    Eigen::Affine3d m_new = prev_pose_after_gismo * m_rel_org;

                    sessions[i].point_clouds_container.point_clouds[0].m_pose = m_new;
                    sessions[i].point_clouds_container.point_clouds[0].pose = pose_tait_bryan_from_affine_matrix(sessions[i].point_clouds_container.point_clouds[0].m_pose);

                    sessions[i].point_clouds_container.point_clouds[i].gui_translation[0] = (float)sessions[i].point_clouds_container.point_clouds[0].pose.px;
                    sessions[i].point_clouds_container.point_clouds[i].gui_translation[1] = (float)sessions[i].point_clouds_container.point_clouds[0].pose.py;
                    sessions[i].point_clouds_container.point_clouds[i].gui_translation[2] = (float)sessions[i].point_clouds_container.point_clouds[0].pose.pz;

                    sessions[i].point_clouds_container.point_clouds[i].gui_rotation[0] = (float)(sessions[i].point_clouds_container.point_clouds[0].pose.om * RAD_TO_DEG);
                    sessions[i].point_clouds_container.point_clouds[i].gui_rotation[1] = (float)(sessions[i].point_clouds_container.point_clouds[0].pose.fi * RAD_TO_DEG);
                    sessions[i].point_clouds_container.point_clouds[i].gui_rotation[2] = (float)(sessions[i].point_clouds_container.point_clouds[0].pose.ka * RAD_TO_DEG);

                    Eigen::Affine3d curr_m_pose = sessions[i].point_clouds_container.point_clouds[0].m_pose;
                    for (size_t j = 1; j < sessions[i].point_clouds_container.point_clouds.size(); j++)
                    {
                        curr_m_pose = curr_m_pose * (all_m_poses[j - 1].inverse() * all_m_poses[j]);
                        sessions[i].point_clouds_container.point_clouds[j].m_pose = curr_m_pose;
                        sessions[i].point_clouds_container.point_clouds[j].pose = pose_tait_bryan_from_affine_matrix(sessions[i].point_clouds_container.point_clouds[j].m_pose);

                        sessions[i].point_clouds_container.point_clouds[j].gui_translation[0] = (float)sessions[i].point_clouds_container.point_clouds[j].pose.px;
                        sessions[i].point_clouds_container.point_clouds[j].gui_translation[1] = (float)sessions[i].point_clouds_container.point_clouds[j].pose.py;
                        sessions[i].point_clouds_container.point_clouds[j].gui_translation[2] = (float)sessions[i].point_clouds_container.point_clouds[j].pose.pz;

                        sessions[i].point_clouds_container.point_clouds[j].gui_rotation[0] = (float)(sessions[i].point_clouds_container.point_clouds[j].pose.om * RAD_TO_DEG);
                        sessions[i].point_clouds_container.point_clouds[j].gui_rotation[1] = (float)(sessions[i].point_clouds_container.point_clouds[j].pose.fi * RAD_TO_DEG);
                        sessions[i].point_clouds_container.point_clouds[j].gui_rotation[2] = (float)(sessions[i].point_clouds_container.point_clouds[j].pose.ka * RAD_TO_DEG);
                    }
                }
            }
        }
    }
    else
    {
        // ImGuizmo -----------------------------------------------
        if (edge_gizmo && edges.size() > 0)
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
                ImGuizmo::Manipulate(m_ortho_gizmo_view, m_ortho_projection, ImGuizmo::TRANSLATE_X | ImGuizmo::TRANSLATE_Y | ImGuizmo::ROTATE_Z, ImGuizmo::WORLD, m_gizmo, NULL);

            Eigen::Affine3d m_g = Eigen::Affine3d::Identity();

            m_g.matrix() = Eigen::Map<const Eigen::Matrix4f>(m_gizmo).cast<double>();

            const int &index_src = edges[index_active_edge].index_from;

            const Eigen::Affine3d &m_src = sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds.at(index_src).m_pose;
            edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_src.inverse() * m_g);
        }
    }

    /*if (!is_loop_closure_gui)
{
    for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
    {
        if (session.point_clouds_container.point_clouds[i].gizmo)
        {
            std::vector<Eigen::Affine3d> all_m_poses;
            for (size_t j = 0; j < session.point_clouds_container.point_clouds.size(); j++)
                all_m_poses.push_back(session.point_clouds_container.point_clouds[j].m_pose);

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
                ImGuizmo::Manipulate(m_ortho_gizmo_view, m_ortho_projection, ImGuizmo::TRANSLATE_X | ImGuizmo::TRANSLATE_Y | ImGuizmo::ROTATE_Z, ImGuizmo::WORLD, m_gizmo, NULL);

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

            session.point_clouds_container.point_clouds[i].gui_rotation[0] = (float)(session.point_clouds_container.point_clouds[i].pose.om * RAD_TO_DEG);
            session.point_clouds_container.point_clouds[i].gui_rotation[1] = (float)(session.point_clouds_container.point_clouds[i].pose.fi * RAD_TO_DEG);
            session.point_clouds_container.point_clouds[i].gui_rotation[2] = (float)(session.point_clouds_container.point_clouds[i].pose.ka * RAD_TO_DEG);

            if (!manipulate_only_marked_gizmo)
            {
                Eigen::Affine3d curr_m_pose = session.point_clouds_container.point_clouds[i].m_pose;
                for (size_t j = i + 1; j < session.point_clouds_container.point_clouds.size(); j++)
                {
                    curr_m_pose = curr_m_pose * (all_m_poses[j - 1].inverse() * all_m_poses[j]);
                    session.point_clouds_container.point_clouds[j].m_pose = curr_m_pose;
                    session.point_clouds_container.point_clouds[j].pose = pose_tait_bryan_from_affine_matrix(session.point_clouds_container.point_clouds[j].m_pose);

                    session.point_clouds_container.point_clouds[j].gui_translation[0] = (float)session.point_clouds_container.point_clouds[j].pose.px;
                    session.point_clouds_container.point_clouds[j].gui_translation[1] = (float)session.point_clouds_container.point_clouds[j].pose.py;
                    session.point_clouds_container.point_clouds[j].gui_translation[2] = (float)session.point_clouds_container.point_clouds[j].pose.pz;

                    session.point_clouds_container.point_clouds[j].gui_rotation[0] = (float)(session.point_clouds_container.point_clouds[j].pose.om * RAD_TO_DEG);
                    session.point_clouds_container.point_clouds[j].gui_rotation[1] = (float)(session.point_clouds_container.point_clouds[j].pose.fi * RAD_TO_DEG);
                    session.point_clouds_container.point_clouds[j].gui_rotation[2] = (float)(session.point_clouds_container.point_clouds[j].pose.ka * RAD_TO_DEG);
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
            glEnd();F
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
            ImGuizmo::Manipulate(m_ortho_gizmo_view, m_ortho_projection, ImGuizmo::TRANSLATE_X | ImGuizmo::TRANSLATE_Y | ImGuizmo::ROTATE_Z, ImGuizmo::WORLD, m_gizmo, NULL);

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
    }
}*/

    view_kbd_shortcuts();

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_A, false))
    {
        addSession();

        // workaround
        io.AddKeyEvent(ImGuiKey_A, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
    }
    if ((project_settings.session_file_names.size() > 0) && !loaded_sessions)
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_L, false))
        {
            loadSessions();

            // workaround
            io.AddKeyEvent(ImGuiKey_L, false);
            io.AddKeyEvent(ImGuiMod_Ctrl, false);
        }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_O, false))
    {
        openProject();

        // workaround
        io.AddKeyEvent(ImGuiKey_O, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
    }

    if (project_settings.session_file_names.size() > 0)
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_R, false))
        {
            remove_gui = true;

            // workaround
            io.AddKeyEvent(ImGuiKey_R, false);
            io.AddKeyEvent(ImGuiMod_Ctrl, false);
        }

    if (sessions.size() > 0)
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S, false))
        {
            saveProject();

            // workaround
            io.AddKeyEvent(ImGuiKey_S, false);
            io.AddKeyEvent(ImGuiMod_Ctrl, false);
        }

    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Open project", "Ctrl+O"))
                openProject();
            if (ImGui::MenuItem("Save project", "Ctrl+S", nullptr, project_settings.session_file_names.size() > 0))
                saveProject();

            ImGui::Separator();

            if (ImGui::MenuItem("Add session(s)", "Ctrl+A"))
                addSession();
            if (ImGui::MenuItem("Remove session(s)", "Ctrl+R", nullptr, project_settings.session_file_names.size() > 0))
                remove_gui = true;

            if (ImGui::MenuItem("Load sessions", "Ctrl+L", nullptr, (project_settings.session_file_names.size() > 0) && !loaded_sessions))
                loadSessions();

            ImGui::Separator();

            if (ImGui::BeginMenu("Save all marked trajectories", sessions.size() > 0))
            {
                if (ImGui::MenuItem("Save all as las/laz files"))
                {
                    for (size_t i = 0; i < project_settings.session_file_names.size(); ++i)
                    {
                        const auto &session_path = project_settings.session_file_names[i];

                        if (i >= sessions.size())
                        {
                            std::cerr << "No loaded session for: " << session_path << std::endl;
                            continue;
                        }

                        Session &session = sessions[i];

                        std::filesystem::path dir = std::filesystem::path(session_path).parent_path();
                        std::string folder_name = dir.filename().string();
                        std::string laz_path = (dir / (folder_name + "_trajectory_laz.laz")).string();

                        std::cout << "Saving trajectory to LAZ: " << laz_path << std::endl;

                        save_trajectories_to_laz(session,
                                                 laz_path,
                                                 0.0f,
                                                 0.0f,
                                                 false);
                    }

                    std::cout << "Finished saving all trajectories to .laz files." << std::endl;
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("As one global scan");

                ImGui::Separator();

                ImGui::Text("(x,y,z,r00,r01,r02,r10,r11,r12,r20,r21,r22)");
                if (ImGui::MenuItem("Save all as csv (timestamp Lidar)##1"))
                {
                    for (size_t i = 0; i < project_settings.session_file_names.size(); ++i)
                    {
                        const auto &session_path = project_settings.session_file_names[i];

                        if (i >= sessions.size())
                        {
                            std::cerr << "No loaded session for: " << session_path << std::endl;
                            continue;
                        }

                        Session &session = sessions[i];
                        std::filesystem::path dir = std::filesystem::path(session_path).parent_path();
                        std::string folder_name = dir.filename().string();
                        std::string csv_path = (dir / (folder_name + "_trajectory_timestampLidar_r.csv")).string();

                        std::cout << "Saving trajectory to CSV: " << csv_path << std::endl;

                        try
                        {
                            std::ofstream outfile(csv_path);
                            if (!outfile.is_open())
                            {
                                std::cerr << "Failed to create file: " << csv_path << std::endl;
                                continue;
                            }

                            outfile << "timestampLidar,x,y,z,"
                                    << "r00,r01,r02,"
                                    << "r10,r11,r12,"
                                    << "r20,r21,r22\n";

                            for (const auto &pc : session.point_clouds_container.point_clouds)
                            {
                                if (!pc.visible)
                                    continue;

                                for (const auto &traj : pc.local_trajectory)
                                {
                                    Eigen::Affine3d pose = pc.m_pose * traj.m_pose;
                                    Eigen::Vector3d pos = pose.translation();
                                    Eigen::Matrix3d rot = pose.rotation();

                                    outfile << std::fixed << std::setprecision(0)
                                            << traj.timestamps.first << ","
                                            << std::setprecision(10)
                                            << pos.x() << "," << pos.y() << "," << pos.z() << ","
                                            << rot(0, 0) << "," << rot(0, 1) << "," << rot(0, 2) << ","
                                            << rot(1, 0) << "," << rot(1, 1) << "," << rot(1, 2) << ","
                                            << rot(2, 0) << "," << rot(2, 1) << "," << rot(2, 2) << "\n";
                                }
                            }

                            outfile.close();
                            std::cout << "Saved: " << csv_path << std::endl;
                        }
                        catch (const std::exception &e)
                        {
                            std::cerr << "Error creating " << csv_path << ": " << e.what() << std::endl;
                        }
                    }

                    std::cout << "Finished saving all trajectories to CSV files." << std::endl;
                }
                if (ImGui::MenuItem("Save all as csv (timestamp Unix)##1"))
                {
                    for (size_t i = 0; i < project_settings.session_file_names.size(); ++i)
                    {
                        const auto &session_path = project_settings.session_file_names[i];
                        if (i >= sessions.size())
                        {
                            std::cerr << "No loaded session for: " << session_path << std::endl;
                            continue;
                        }

                        Session &session = sessions[i];
                        std::filesystem::path dir = std::filesystem::path(session_path).parent_path();
                        std::string folder_name = dir.filename().string();
                        std::string csv_path = (dir / (folder_name + "_trajectory_timestampUnix_r.csv")).string();

                        try
                        {
                            std::ofstream outfile(csv_path);
                            if (!outfile.is_open())
                            {
                                std::cerr << "Failed to create file: " << csv_path << std::endl;
                                continue;
                            }

                            outfile << "timestampUnix,x,y,z," << "r00,r01,r02,r10,r11,r12,r20,r21,r22\n";

                            for (const auto &pc : session.point_clouds_container.point_clouds)
                            {
                                if (!pc.visible)
                                    continue;
                                for (const auto &traj : pc.local_trajectory)
                                {
                                    Eigen::Affine3d pose = pc.m_pose * traj.m_pose;
                                    Eigen::Vector3d pos = pose.translation();
                                    Eigen::Matrix3d rot = pose.rotation();
                                    outfile << std::fixed << std::setprecision(0)
                                            << traj.timestamps.second << "," // Unix timestamp
                                            << std::setprecision(10)
                                            << pos.x() << "," << pos.y() << "," << pos.z() << ","
                                            << rot(0, 0) << "," << rot(0, 1) << "," << rot(0, 2) << ","
                                            << rot(1, 0) << "," << rot(1, 1) << "," << rot(1, 2) << ","
                                            << rot(2, 0) << "," << rot(2, 1) << "," << rot(2, 2) << "\n";
                                }
                            }

                            outfile.close();
                            std::cout << "Saved: " << csv_path << std::endl;
                        }
                        catch (const std::exception &e)
                        {
                            std::cerr << "Error creating " << csv_path << ": " << e.what() << std::endl;
                        }
                    }
                }
                if (ImGui::MenuItem("Save all as csv (timestamp Lidar, Unix)##1"))
                {
                    for (size_t i = 0; i < project_settings.session_file_names.size(); ++i)
                    {
                        const auto &session_path = project_settings.session_file_names[i];
                        if (i >= sessions.size())
                        {
                            std::cerr << "No loaded session for: " << session_path << std::endl;
                            continue;
                        }

                        Session &session = sessions[i];
                        std::filesystem::path dir = std::filesystem::path(session_path).parent_path();
                        std::string folder_name = dir.filename().string();
                        std::string csv_path = (dir / (folder_name + "_trajectory_timestampLidarUnix_r.csv")).string();

                        try
                        {
                            std::ofstream outfile(csv_path);
                            if (!outfile.is_open())
                            {
                                std::cerr << "Failed to create file: " << csv_path << std::endl;
                                continue;
                            }

                            outfile << "timestampLidar,timestampUnix,x,y,z,"
                                    << "r00,r01,r02,r10,r11,r12,r20,r21,r22\n";

                            for (const auto &pc : session.point_clouds_container.point_clouds)
                            {
                                if (!pc.visible)
                                    continue;
                                for (const auto &traj : pc.local_trajectory)
                                {
                                    Eigen::Affine3d pose = pc.m_pose * traj.m_pose;
                                    Eigen::Vector3d pos = pose.translation();
                                    Eigen::Matrix3d rot = pose.rotation();
                                    outfile << std::fixed << std::setprecision(0)
                                            << traj.timestamps.first << ","  // Lidar timestamp
                                            << traj.timestamps.second << "," // Unix timestamp
                                            << std::setprecision(10)
                                            << pos.x() << "," << pos.y() << "," << pos.z() << ","
                                            << rot(0, 0) << "," << rot(0, 1) << "," << rot(0, 2) << ","
                                            << rot(1, 0) << "," << rot(1, 1) << "," << rot(1, 2) << ","
                                            << rot(2, 0) << "," << rot(2, 1) << "," << rot(2, 2) << "\n";
                                }
                            }

                            outfile.close();
                            std::cout << "Saved: " << csv_path << std::endl;
                        }
                        catch (const std::exception &e)
                        {
                            std::cerr << "Error creating " << csv_path << ": " << e.what() << std::endl;
                        }
                    }
                }

                ImGui::Separator();
                ImGui::Text("(x,y,z,qx,qy,qz,qw)");

                if (ImGui::MenuItem("Save all as csv (timestamp Lidar)##2"))
                {
                    for (size_t i = 0; i < project_settings.session_file_names.size(); ++i)
                    {
                        const auto &session_path = project_settings.session_file_names[i];

                        if (i >= sessions.size())
                        {
                            std::cerr << "No loaded session for: " << session_path << std::endl;
                            continue;
                        }

                        Session &session = sessions[i];
                        std::filesystem::path dir = std::filesystem::path(session_path).parent_path();
                        std::string folder_name = dir.filename().string();
                        std::string csv_path = (dir / (folder_name + "_trajectory_timestampLidar_q.csv")).string();

                        std::cout << "Saving trajectory to CSV: " << csv_path << std::endl;

                        try
                        {
                            std::ofstream outfile(csv_path);
                            if (!outfile.is_open())
                            {
                                std::cerr << "Failed to create file: " << csv_path << std::endl;
                                continue;
                            }

                            outfile << "timestampLidar,x,y,z,qx,qy,qz,qw\n";

                            for (const auto &pc : session.point_clouds_container.point_clouds)
                            {
                                if (!pc.visible)
                                    continue;

                                for (const auto &traj : pc.local_trajectory)
                                {
                                    Eigen::Affine3d pose = pc.m_pose * traj.m_pose;
                                    Eigen::Vector3d pos = pose.translation();
                                    Eigen::Quaterniond q(pose.rotation());

                                    outfile << std::fixed << std::setprecision(0)
                                            << traj.timestamps.first << "," // Lidar timestamp
                                            << std::setprecision(10)
                                            << pos.x() << "," << pos.y() << "," << pos.z() << ","
                                            << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "\n";
                                }
                            }

                            outfile.close();
                            std::cout << "Saved: " << csv_path << std::endl;
                        }
                        catch (const std::exception &e)
                        {
                            std::cerr << "Error creating " << csv_path << ": " << e.what() << std::endl;
                        }
                    }

                    std::cout << "Finished saving all trajectories to CSV files." << std::endl;
                }
                if (ImGui::MenuItem("Save all as csv (timestamp Unix)##2"))
                {
                    for (size_t i = 0; i < project_settings.session_file_names.size(); ++i)
                    {
                        const auto &session_path = project_settings.session_file_names[i];

                        if (i >= sessions.size())
                        {
                            std::cerr << "No loaded session for: " << session_path << std::endl;
                            continue;
                        }

                        Session &session = sessions[i];
                        std::filesystem::path dir = std::filesystem::path(session_path).parent_path();
                        std::string folder_name = dir.filename().string();
                        std::string csv_path = (dir / (folder_name + "_trajectory_timestampUnix_q.csv")).string();

                        std::cout << "Saving trajectory to CSV: " << csv_path << std::endl;

                        try
                        {
                            std::ofstream outfile(csv_path);
                            if (!outfile.is_open())
                            {
                                std::cerr << "Failed to create file: " << csv_path << std::endl;
                                continue;
                            }

                            outfile << "timestampUnix,x,y,z,qx,qy,qz,qw\n";

                            for (const auto &pc : session.point_clouds_container.point_clouds)
                            {
                                if (!pc.visible)
                                    continue;

                                for (const auto &traj : pc.local_trajectory)
                                {
                                    Eigen::Affine3d pose = pc.m_pose * traj.m_pose;
                                    Eigen::Vector3d pos = pose.translation();
                                    Eigen::Quaterniond q(pose.rotation());

                                    outfile << std::fixed << std::setprecision(0)
                                            << traj.timestamps.second << "," // Unix timestamp
                                            << std::setprecision(10)
                                            << pos.x() << "," << pos.y() << "," << pos.z() << ","
                                            << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "\n";
                                }
                            }

                            outfile.close();
                            std::cout << "Saved: " << csv_path << std::endl;
                        }
                        catch (const std::exception &e)
                        {
                            std::cerr << "Error creating " << csv_path << ": " << e.what() << std::endl;
                        }
                    }

                    std::cout << "Finished saving all trajectories to CSV files." << std::endl;
                }
                if (ImGui::MenuItem("Save all as csv (timestamp Lidar, Unix)##2"))
                {
                    for (size_t i = 0; i < project_settings.session_file_names.size(); ++i)
                    {
                        const auto &session_path = project_settings.session_file_names[i];

                        if (i >= sessions.size())
                        {
                            std::cerr << "No loaded session for: " << session_path << std::endl;
                            continue;
                        }

                        Session &session = sessions[i];
                        std::filesystem::path dir = std::filesystem::path(session_path).parent_path();
                        std::string folder_name = dir.filename().string();
                        std::string csv_path = (dir / (folder_name + "_trajectory_timestampLidarUnix_q.csv")).string();

                        std::cout << "Saving trajectory to CSV: " << csv_path << std::endl;

                        try
                        {
                            std::ofstream outfile(csv_path);
                            if (!outfile.is_open())
                            {
                                std::cerr << "Failed to create file: " << csv_path << std::endl;
                                continue;
                            }

                            outfile << "timestampLidar,timestampUnix,x,y,z,qx,qy,qz,qw\n";

                            for (const auto &pc : session.point_clouds_container.point_clouds)
                            {
                                if (!pc.visible)
                                    continue;

                                for (const auto &traj : pc.local_trajectory)
                                {
                                    Eigen::Affine3d pose = pc.m_pose * traj.m_pose;
                                    Eigen::Vector3d pos = pose.translation();
                                    Eigen::Quaterniond q(pose.rotation());

                                    outfile << std::fixed << std::setprecision(0)
                                            << traj.timestamps.first << ","  // Lidar timestamp
                                            << traj.timestamps.second << "," // Unix timestamp
                                            << std::setprecision(10)
                                            << pos.x() << "," << pos.y() << "," << pos.z() << ","
                                            << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "\n";
                                }
                            }

                            outfile.close();
                            std::cout << "Saved: " << csv_path << std::endl;
                        }
                        catch (const std::exception &e)
                        {
                            std::cerr << "Error creating " << csv_path << ": " << e.what() << std::endl;
                        }
                    }

                    std::cout << "Finished saving all trajectories to CSV files." << std::endl;
                }
                if (ImGui::MenuItem("Save all as TUM TXT"))
                {
                    for (size_t i = 0; i < project_settings.session_file_names.size(); ++i)
                    {
                        const auto &session_path = project_settings.session_file_names[i];
                        if (i >= sessions.size())
                        {
                            std::cerr << "No loaded session for: " << session_path << std::endl;
                            continue;
                        }
                        Session &session = sessions[i];
                        std::filesystem::path dir = std::filesystem::path(session_path).parent_path();
                        std::string folder_name = dir.filename().string();
                        std::string txt_path = (dir / (folder_name + "_trajectory_tum.txt")).string();

                        std::cout << "Saving trajectory to TUM TXT: " << txt_path << std::endl;
                        try
                        {
                            std::ofstream outfile(txt_path);
                            if (!outfile.is_open())
                            {
                                std::cerr << "Failed to create file: " << txt_path << std::endl;
                                continue;
                            }

                            for (const auto &pc : session.point_clouds_container.point_clouds)
                            {
                                if (!pc.visible)
                                    continue;
                                for (const auto &traj : pc.local_trajectory)
                                {
                                    Eigen::Affine3d pose = pc.m_pose * traj.m_pose;
                                    Eigen::Vector3d pos = pose.translation();
                                    Eigen::Quaterniond q(pose.rotation());

                                    double t_s = static_cast<double>(traj.timestamps.first) / 1e9;

                                    outfile << std::fixed
                                            << std::setprecision(9) << t_s << " "
                                            << std::setprecision(10) << pos.x() << " " << pos.y() << " " << pos.z() << " "
                                            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
                                }
                            }

                            outfile.close();
                            std::cout << "Saved: " << txt_path << std::endl;
                        }
                        catch (const std::exception &e)
                        {
                            std::cerr << "Error creating " << txt_path << ": " << e.what() << std::endl;
                        }
                    }
                    std::cout << "Finished saving all trajectories to TUM TXT files." << std::endl;
                }

                ImGui::EndMenu();
            }

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Tools"))
        {
            ImGui::MenuItem("Normal Distributions Transform", nullptr, &is_ndt_gui, !is_loop_closure_gui && (sessions.size() > 0));
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                ImGui::Text("Point cloud alignment (registration) algorithm");
                ImGui::Text("Probabilistic alternative to ICP that models one cloud (the target)\nas a set of Gaussian distributions rather than raw points");
                ImGui::Text("Robust for rough initial poses but can converge to a local optimum\nif the initial misalignment is very large");
                ImGui::Text("Known for being faster and smoother in optimization because\nit replaces discrete point-point correspondences with continuous probability density functions.");
                ImGui::EndTooltip();
            }

            // bool prev_is_loop_closure_gui
            ImGui::MenuItem("Manual Loop Closure", "Ctrl+L", &is_loop_closure_gui, (number_visible_sessions == 1 || number_visible_sessions == 2));
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Manually connect overlapping scan sections");

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View"))
        {
            ImGui::BeginDisabled(!(sessions.size() > 0));
            {
                auto tmp = point_size;
                ImGui::SetNextItemWidth(ImGuiNumberWidth);
                ImGui::InputInt("Points size", &point_size);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("keyboard 1-9 keys");
                if (point_size < 1)
                    point_size = 1;
                else if (point_size > 10)
                    point_size = 10;

                if (tmp != point_size)
                    for (auto &session : sessions)
                        for (auto &point_cloud : session.point_clouds_container.point_clouds)
                            point_cloud.point_size = point_size;

                ImGui::Separator();
            }
            ImGui::EndDisabled();

            ImGui::MenuItem("Orthographic", "key O", &is_ortho);
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

            ImGui::MenuItem("Show axes", "key X", &show_axes);
            ImGui::MenuItem("Show compass/ruler", "key C", &compass_ruler);

            ImGui::MenuItem("Lock Z", "Shift + Z", &lock_z, !is_ortho);

            // ImGui::MenuItem("show_covs", nullptr, &show_covs);

            ImGui::Separator();

            ImGui::Text("Colors:");

            ImGui::ColorEdit3("Background", (float *)&bg_color, ImGuiColorEditFlags_NoInputs);

            ImGui::EndMenu();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Scene view relevant parameters");

        camMenu();

        ImGui::BeginDisabled(sessions.size() <= 0);
        {
            ImGui::SameLine();
            ImGui::Dummy(ImVec2(20, 0));
            ImGui::SameLine();

            ImGui::SetNextItemWidth(ImGuiNumberWidth);
            ImGui::InputInt("Points render subsampling", &viewer_decimate_point_cloud, 10, 100);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("increase for better performance, decrease for rendering more points");
            ImGui::SameLine();

            if (viewer_decimate_point_cloud < 1)
                viewer_decimate_point_cloud = 1;

            ImGui::SameLine();
            ImGui::Text("(%.1f FPS)", ImGui::GetIO().Framerate);
        }
        ImGui::EndDisabled();

        ImGui::SameLine();
        ImGui::Dummy(ImVec2(20, 0));
        ImGui::SameLine();

        ImGui::SameLine(ImGui::GetWindowWidth() - ImGui::CalcTextSize("Info").x - ImGui::GetStyle().ItemSpacing.x * 2 - ImGui::GetStyle().FramePadding.x * 2);

        ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4, 2));
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetStyleColorVec4(ImGuiCol_HeaderHovered));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImGui::GetStyleColorVec4(ImGuiCol_Header));
        if (ImGui::SmallButton("Info"))
            info_gui = !info_gui;

        ImGui::PopStyleVar(2);
        ImGui::PopStyleColor(3);

        ImGui::EndMainMenuBar();
    }

    if (remove_gui)
    {
        ImGui::OpenPopup("Remove session(s)");
        remove_gui = false;
    }

    if (ImGui::BeginPopupModal("Remove session(s)", NULL, ImGuiWindowFlags_AlwaysAutoResize))
    {
        static std::vector<bool> session_marked_for_removal;
        if (session_marked_for_removal.size() != project_settings.session_file_names.size())
            session_marked_for_removal.resize(project_settings.session_file_names.size(), false);

        ImGui::Text("Select session(s) to remove:");
        ImGui::Separator();

        for (size_t i = 0; i < project_settings.session_file_names.size(); i++)
        {
            bool checked = session_marked_for_removal[i];
            if (ImGui::Checkbox(project_settings.session_file_names[i].c_str(), &checked))
                session_marked_for_removal[i] = checked;
        }

        ImGui::Separator();

        if (ImGui::Button("Remove"))
        {
            for (size_t i = project_settings.session_file_names.size() - 1; i >= 0; i--)
            {
                if (session_marked_for_removal[i])
                {
                    std::cout << "Removing session: " << project_settings.session_file_names[i] << std::endl;
                    project_settings.session_file_names.erase(project_settings.session_file_names.begin() + i);
                    if ((sessions.size() > 0) && i < sessions.size())
                        sessions.erase(sessions.begin() + i);
                }
            }

            session_marked_for_removal.clear();

            if (!sessions.empty())
                update_timestamp_offset();
            else
            {
                loaded_sessions = false;
                time_stamp_offset = 0.0;
            }

            ImGui::CloseCurrentPopup();
        }

        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
        {
            session_marked_for_removal.clear();
            ImGui::CloseCurrentPopup();
        }

        ImGui::EndPopup();
    }

    if (is_ndt_gui)
        ndt_gui();

    if (is_loop_closure_gui)
        loop_closure_gui();

    cor_window();

    info_window(infoLines, appShortcuts);

    if (compass_ruler)
        drawMiniCompassWithRuler();

    // my_display_code();
    /*if (is_ndt_gui)
        ndt_gui();
    if (is_icp_gui)
        icp_gui();
    if (is_pose_graph_slam)
        pose_graph_slam_gui();
    if (is_registration_plane_feature)
        registration_plane_feature_gui();
    if (is_manual_analisys)
        observation_picking_gui();*/
    // if (is_loop_closure_gui)
    //     manual_pose_graph_loop_closure.Gui();

    project_gui();

    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    glutSwapBuffers();
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

    static int glutMajorVersion = glutGet(GLUT_VERSION) / 10000;
    if (state == GLUT_DOWN && (glut_button == 3 || glut_button == 4) && glutMajorVersion < 3)
        wheel(glut_button, glut_button == 3 ? 1 : -1, x, y);

    if (!io.WantCaptureMouse)
    {
        if ((glut_button == GLUT_MIDDLE_BUTTON || glut_button == GLUT_RIGHT_BUTTON) && state == GLUT_DOWN && (io.KeyCtrl || io.KeyShift) && !manipulate_active_edge)
        {
            //if (s_loop_closure_gui)
            if ((sessions.size() > 0) && (number_visible_sessions > 0))
            {
                getClosestTrajectoriesPoint(sessions, x, y, first_session_index, second_session_index, number_visible_sessions, index_loop_closure_source, index_loop_closure_target, io.KeyShift);
            }
            else
            {
                setNewRotationCenter(x, y);
            }
        }

        if (state == GLUT_DOWN)
        {
            mouse_buttons |= 1 << glut_button;

            /*if (observation_picking.is_observation_picking_mode)
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
            }*/
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
    try
    {
        initGL(&argc, argv, winTitle, display, mouse);

        glutMainLoop();

        ImGui_ImplOpenGL2_Shutdown();
        ImGui_ImplGLUT_Shutdown();
        ImGui::DestroyContext();
    }
    catch (const std::bad_alloc &e)
    {
        std::cerr << "System is out of memory : " << e.what() << std::endl;
        mandeye::fd::OutOfMemMessage();
    }
    catch (const std::exception &e)
    {
        std::cout << e.what();
    }
    catch (...)
    {
        std::cerr << "Unknown fatal error occurred." << std::endl;
    }

    return 0;
}