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
#include <glm.hpp>
#include <gtc/matrix_transform.hpp>

#include <Eigen/Eigen>

#include <session.h>

#include <portable-file-dialogs.h>

#include <icp.h>

#include <python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_cw_jacobian.h>

#include <registration_plane_feature.h>

#include <m_estimators.h>

double camera_ortho_xy_view_zoom = 10;
double camera_ortho_xy_view_shift_x = 0.0;
double camera_ortho_xy_view_shift_y = 0.0;
double camera_ortho_xy_view_rotation_angle_deg = 0;
double camera_mode_ortho_z_center_h = 0.0;

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

static ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
Eigen::Vector3f rotation_center = Eigen::Vector3f::Zero();
float rotate_x = 0.0, rotate_y = 0.0;
float translate_x, translate_y = 0.0;
float translate_z = -50.0;
const unsigned int window_width = 800;
const unsigned int window_height = 600;
int mouse_old_x, mouse_old_y;
int mouse_buttons = 0;
bool show_axes = false;
bool is_ortho = false;
int viewer_decmiate_point_cloud = 1000;
bool gui_mouse_down{false};
float mouse_sensitivity = 1.0;
bool is_decimate = true;
double bucket_x = 0.1;
double bucket_y = 0.1;
double bucket_z = 0.1;
int all_point_size = 1;
bool calculate_offset = false;
bool manual_pose_graph_loop_closure_mode = false;
ObservationPicking observation_picking;
int index_loop_closure_source = -1;
int index_loop_closure_target = -1;
int first_session_index = -1;
int second_session_index = -1;
double search_radious = 0.3;
bool loaded_sessions = false;
bool optimized = false;
bool gizmo_all_sessions = false;

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

// this funciton performs pose graph slam calculations
bool optimize(std::vector<Session> &sessions);

// this function revert results to previous one
bool revert(std::vector<Session> &sessions);

// this function saves result (poses) to files
bool save_results(std::vector<Session> &sessions);

LaserBeam GetLaserBeam(int x, int y);
Eigen::Vector3d rayIntersection(const LaserBeam &laser_beam, const RegistrationPlaneFeature::Plane &plane);

bool load_project_settings(const std::string &file_name, ProjectSettings &_project_settings)
{
    std::cout << "loading file: '" << file_name << "'" << std::endl;
    std::vector<std::string> session_file_names;

    try
    {
        std::ifstream fs(file_name);
        if (!fs.good())
            return false;
        nlohmann::json data = nlohmann::json::parse(fs);
        fs.close();

        for (const auto &fn_json : data["session_file_names"])
        {
            std::string fn = fn_json["session_file_name"];
            session_file_names.push_back(fn);
        }

        std::cout << "------session file names-----" << std::endl;
        for (const auto &fn : session_file_names)
        {
            std::cout << "'" << fn << "'" << std::endl;
        }

        _project_settings.session_file_names = session_file_names;

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
        return true;
    }
    catch (std::exception &e)
    {
        std::cout << "cant load project settings: " << e.what() << std::endl;
        return false;
    }

    return true;
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

void project_gui()
{
    const std::vector<std::string> Session_filter = {"Session, json", "*.json"};
    const std::vector<std::string> Project_filter = {"Project, json", "*.json"};

    if (ImGui::Begin("multi_session_registration_step_3"))
    {
        ImGui::Text("This program is third step in MANDEYE process.");
        ImGui::Text("To change centre of rotation press 'ctrl + middle mouse button'");
        ImGui::Text("It refines sessions with loop closure.");
        ImGui::Text("First step: create project by adding sessions: result of 'multi_view_tls_registration_step_2' program.");
        ImGui::Text("Last step: save project.");
        ImGui::Text("To produce map use 'multi_view_tls_registration_step_2' export functionality.");

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::ColorEdit3("clear color", (float *)&clear_color); // Edit 3 floats representing a color

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
            for (auto &session : sessions)
            {
                for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
                {
                    session.point_clouds_container.point_clouds[i].point_size = all_point_size;
                }
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

        ImGui::SliderFloat("mouse_sensitivity", &mouse_sensitivity, 0.01f, 25.0f);
        ImGui::InputFloat3("rotation center", rotation_center.data());
        ImGui::Checkbox("decimate during load", &is_decimate);
        ImGui::InputDouble("bucket_x", &bucket_x);
        ImGui::InputDouble("bucket_y", &bucket_y);
        ImGui::InputDouble("bucket_z", &bucket_z);

        ImGui::Text("---------------------------------------------");
        ImGui::Text("-------PROJECT SETTINGS BEGIN----------------");
        if (ImGui::Button("add session to project"))
        {
            static std::shared_ptr<pfd::open_file> open_file;
            std::string input_file_name = "";
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
            const auto t = [&]()
            {
                auto sel = pfd::open_file("Load RESSO file", "C:\\", Session_filter).result();
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
                project_settings.session_file_names.push_back(input_file_name);
            }
        }

        if (ImGui::Button("load project"))
        {
            static std::shared_ptr<pfd::open_file> open_file;
            std::string input_file_name = "";
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
            const auto t = [&]()
            {
                auto sel = pfd::open_file("Load project", "C:\\", Project_filter).result();
                for (int i = 0; i < sel.size(); i++)
                {
                    input_file_name = sel[i];
                    std::cout << "Project file: '" << input_file_name << "'" << std::endl;
                }
            };
            std::thread t1(t);
            t1.join();

            if (input_file_name.size() > 0)
            {
                load_project_settings(fs::path(input_file_name).string(), project_settings);
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("save project"))
        {
            std::shared_ptr<pfd::save_file> save_file;
            std::string output_file_name = "";
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
            const auto t = [&]()
            {
                auto sel = pfd::save_file("Save project file", "C:\\", Project_filter).result();
                output_file_name = sel;
                std::cout << "Project file to save: '" << output_file_name << "'" << std::endl;
            };
            std::thread t1(t);
            t1.join();

            if (output_file_name.size() > 0)
            {
                save_project_settings(fs::path(output_file_name).string(), project_settings);
            }
        }

        ImGui::Text("-----------session_file_names begin----------");
        ImGui::Checkbox("gizmo_all_sessions", &gizmo_all_sessions);

        int index_gizmo = -1;

        for (int i = 0; i < project_settings.session_file_names.size(); i++)
        {
            ImGui::Text(project_settings.session_file_names[i].c_str());

            if (project_settings.session_file_names.size() == sessions.size())
            {
                if (sessions[i].is_ground_truth)
                {
                    ImGui::SameLine();
                    ImGui::Text(" [ground_truth] ");
                }
                ImGui::SameLine();
                ImGui::Checkbox(("visible[" + std::to_string(i) + "]").c_str(), &sessions[i].visible);
                if (!sessions[i].is_ground_truth)
                {
                    ImGui::SameLine();
                    ImGui::Checkbox(("gizmo[" + std::to_string(i) + "]").c_str(), &sessions[i].is_gizmo);
                }

                if (sessions[i].is_gizmo)
                {
                    for (int ii = 0; ii < sessions.size(); ii++)
                    {
                        sessions[ii].is_gizmo = false;
                    }
                    index_gizmo = i;
                }
                ImGui::SameLine();
                ImGui::Checkbox(("show_rgb[" + std::to_string(i) + "]").c_str(), &sessions[i].show_rgb);

                if (!sessions[i].show_rgb)
                {
                    // ImGui::SameLine();
                    ImGui::ColorEdit3(("color[" + std::to_string(i) + "]").c_str(), (float *)&sessions[i].render_color);
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
                    {
                        pc.show_color = sessions[i].show_rgb;
                    }
                }
            }
        }
        if (project_settings.session_file_names.size() == sessions.size())
        {
            for (int i = 0; i < sessions.size(); i++)
            {
                sessions[i].is_gizmo = false;
            }
            if (index_gizmo != -1)
            {
                sessions[index_gizmo].is_gizmo = true;
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

        ImGui::Text("-----------session_file_names end------------");

        ImGui::Text("-------PROJECT SETTINGS END------------------");

        if (project_settings.session_file_names.size() > 0)
        {
            if (!loaded_sessions)
            {
                if (ImGui::Button("load sessions"))
                {
                    sessions.clear();
                    for (const auto &ps : project_settings.session_file_names)
                    {
                        Session session;
                        session.load(fs::path(ps).string(), is_decimate, bucket_x, bucket_y, bucket_z, calculate_offset);
                        sessions.push_back(session);
                    }
                    loaded_sessions = true;

                    // reorder
                    std::vector<Session> sessions_reorder;
                    std::vector<std::string> session_file_names_reordered;

                    std::map<int, int> map_reorder;
                    // project_settings.session_file_names.
                    int new_index = 0;
                    for (int i = 0; i < sessions.size(); i++)
                    {
                        if (sessions[i].is_ground_truth)
                        {
                            sessions_reorder.push_back(sessions[i]);
                            session_file_names_reordered.push_back(project_settings.session_file_names[i]);
                            map_reorder[i] = new_index++;
                        }
                    }
                    for (int i = 0; i < sessions.size(); i++)
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
                }
            }

            if (project_settings.session_file_names.size() == sessions.size())
            {
                int number_visible_sessions = 0;

                bool first_session_index_found = false;
                for (int index = 0; index < sessions.size(); index++)
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

                if (!manual_pose_graph_loop_closure_mode)
                {
                    if (ImGui::Button("Optimize"))
                    {
                        for (int i = 0; i < 10; i++)
                        {
                            optimize(sessions);
                        }
                        optimized = true;
                    }
                    if (optimized)
                    {
                        ImGui::SameLine();
                        if (ImGui::Button("Revert"))
                        {
                            revert(sessions);
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("Save results"))
                        {
                            save_results(sessions);
                        }
                    }
                }

                if (number_visible_sessions == 1 || number_visible_sessions == 2)
                {
                    ImGui::Checkbox("Manual Pose Graph Loop Closure Mode", &manual_pose_graph_loop_closure_mode);
                    if (manual_pose_graph_loop_closure_mode)
                    {
                        if (!manipulate_active_edge)
                        {
                            ImGui::InputInt("index_loop_closure_source", &index_loop_closure_source);
                            if (index_loop_closure_source < 0)
                            {
                                index_loop_closure_source = 0;
                            }
                            if (index_loop_closure_source >= sessions[first_session_index].point_clouds_container.point_clouds.size() - 1)
                            {
                                index_loop_closure_source = sessions[first_session_index].point_clouds_container.point_clouds.size() - 1;
                            }
                            ImGui::InputInt("index_loop_closure_target", &index_loop_closure_target);
                            if (index_loop_closure_target < 0)
                            {
                                index_loop_closure_target = 0;
                            }
                            if (index_loop_closure_target >= sessions[second_session_index].point_clouds_container.point_clouds.size() - 1)
                            {
                                index_loop_closure_target = sessions[second_session_index].point_clouds_container.point_clouds.size() - 1;
                            }
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
                                    
                                    for(const auto &s:sessions){
                                        if(s.is_gizmo){
                                            is_gizmo = true;
                                        }
                                    }

                                    if (!is_gizmo){
                                        ImGui::InputInt("index_active_edge", &index_active_edge);

                                        if (index_active_edge < 0)
                                        {
                                            index_active_edge = 0;
                                        }
                                        if (index_active_edge >= (int)edges.size())
                                        {
                                            index_active_edge = (int)edges.size() - 1;
                                        }
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
                                    for (int i = 0; i < edges.size(); i++)
                                    {
                                        if (remove_edge_index != i)
                                        {
                                            new_edges.push_back(edges[i]);
                                        }
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
                                                icp.search_radious = (float)search_radious;

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
                                                icp.search_radious = (float)search_radious;

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
                                    ImGui::InputDouble("search_radious", &search_radious);
                                    if (search_radious < 0.01)
                                    {
                                        search_radious = 0.01;
                                    }
                                }
                            }
                        }
                    }
                }

                // if (!manual_pose_graph_loop_closure_mode)
                //{

                //}
            }
        }

        ImGui::End();
    }
}

void reshape(int w, int h);

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
        // reshape((GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
        // glTranslatef(translate_x, translate_y, translate_z);
        // glRotatef(rotate_x, 1.0, 0.0, 0.0);
        // glRotatef(rotate_y, 0.0, 0.0, 1.0);
    }
    else
    {
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
    }

    // if (show_axes)
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

    // if (show_axes || ImGui::GetIO().KeyCtrl)
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
        if (manipulate_active_edge)
        {
            if (edges.size() > 0)
            {
                int index_src = edges[index_active_edge].index_from;
                int index_trg = edges[index_active_edge].index_to;

                Eigen::Affine3d m_src = sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds.at(index_src).m_pose;
                Eigen::Affine3d m_trg = m_src * affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds.at(index_src).render(m_src, 1);
                sessions[edges[index_active_edge].index_session_to].point_clouds_container.point_clouds.at(index_trg).render(m_trg, 1);
            }
        }
        else
        {
            ObservationPicking observation_picking;
            sessions[first_session_index].point_clouds_container.point_clouds.at(index_loop_closure_source).render(false, observation_picking, 1);
            sessions[second_session_index].point_clouds_container.point_clouds.at(index_loop_closure_target).render(false, observation_picking, 1);
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
            glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_10, (const unsigned char *)std::to_string(i).c_str());
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
            glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_10, (const unsigned char *)std::to_string(i).c_str());
            i++;
        }

        for (int i = 0; i < sessions.size(); i++)
        {
            for (int j = 0; j < sessions[i].manual_pose_graph_loop_closure.edges.size(); j++)
            {
                int index_src = sessions[i].manual_pose_graph_loop_closure.edges[j].index_from;
                int index_trg = sessions[i].manual_pose_graph_loop_closure.edges[j].index_to;

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

        for (int i = 0; i < edges.size(); i++)
        {
            int index_src = edges[i].index_from;
            int index_trg = edges[i].index_to;

            int index_session_from = edges[i].index_session_from;
            int index_session_to = edges[i].index_session_to;

            if (sessions[index_session_from].is_ground_truth || sessions[index_session_to].is_ground_truth)
            {
                glColor3f(0.0f, 1.0f, 1.0f);
            }
            else
            {
                glColor3f(1.0f, 1.0f, 0.0f);
            }

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
                session.point_clouds_container.render(observation_picking, viewer_decmiate_point_cloud);
            }
        }
    }

    /*if (manual_pose_graph_loop_closure_mode)
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
    // if (manual_pose_graph_loop_closure_mode)
    //{
    //     manual_pose_graph_loop_closure.Gui();
    // }
    project_gui();

    if (!manual_pose_graph_loop_closure_mode)
    {
        Eigen::Affine3d prev_pose_manipulated = Eigen::Affine3d::Identity();
        Eigen::Affine3d prev_pose_after_gismo = Eigen::Affine3d::Identity();
        for (size_t i = 0; i < sessions.size(); i++)
        {
            // gizmo_all_sessions;
            if (sessions[i].is_gizmo && !sessions[i].is_ground_truth)
            {
                prev_pose_manipulated = sessions[i].point_clouds_container.point_clouds[0].m_pose;
                std::vector<Eigen::Affine3d> all_m_poses;
                for (int j = 0; j < sessions[i].point_clouds_container.point_clouds.size(); j++)
                {
                    all_m_poses.push_back(sessions[i].point_clouds_container.point_clouds[j].m_pose);
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

                sessions[i].point_clouds_container.point_clouds[0].m_pose(0, 0) = m_gizmo[0];
                sessions[i].point_clouds_container.point_clouds[0].m_pose(1, 0) = m_gizmo[1];
                sessions[i].point_clouds_container.point_clouds[0].m_pose(2, 0) = m_gizmo[2];
                sessions[i].point_clouds_container.point_clouds[0].m_pose(3, 0) = m_gizmo[3];
                sessions[i].point_clouds_container.point_clouds[0].m_pose(0, 1) = m_gizmo[4];
                sessions[i].point_clouds_container.point_clouds[0].m_pose(1, 1) = m_gizmo[5];
                sessions[i].point_clouds_container.point_clouds[0].m_pose(2, 1) = m_gizmo[6];
                sessions[i].point_clouds_container.point_clouds[0].m_pose(3, 1) = m_gizmo[7];
                sessions[i].point_clouds_container.point_clouds[0].m_pose(0, 2) = m_gizmo[8];
                sessions[i].point_clouds_container.point_clouds[0].m_pose(1, 2) = m_gizmo[9];
                sessions[i].point_clouds_container.point_clouds[0].m_pose(2, 2) = m_gizmo[10];
                sessions[i].point_clouds_container.point_clouds[0].m_pose(3, 2) = m_gizmo[11];
                sessions[i].point_clouds_container.point_clouds[0].m_pose(0, 3) = m_gizmo[12];
                sessions[i].point_clouds_container.point_clouds[0].m_pose(1, 3) = m_gizmo[13];
                sessions[i].point_clouds_container.point_clouds[0].m_pose(2, 3) = m_gizmo[14];
                sessions[i].point_clouds_container.point_clouds[0].m_pose(3, 3) = m_gizmo[15];
                prev_pose_after_gismo = sessions[i].point_clouds_container.point_clouds[0].m_pose;
                sessions[i].point_clouds_container.point_clouds[0].pose = pose_tait_bryan_from_affine_matrix(sessions[i].point_clouds_container.point_clouds[0].m_pose);

                sessions[i].point_clouds_container.point_clouds[i].gui_translation[0] = (float)sessions[i].point_clouds_container.point_clouds[0].pose.px;
                sessions[i].point_clouds_container.point_clouds[i].gui_translation[1] = (float)sessions[i].point_clouds_container.point_clouds[0].pose.py;
                sessions[i].point_clouds_container.point_clouds[i].gui_translation[2] = (float)sessions[i].point_clouds_container.point_clouds[0].pose.pz;

                sessions[i].point_clouds_container.point_clouds[i].gui_rotation[0] = (float)(sessions[i].point_clouds_container.point_clouds[0].pose.om * 180.0 / M_PI);
                sessions[i].point_clouds_container.point_clouds[i].gui_rotation[1] = (float)(sessions[i].point_clouds_container.point_clouds[0].pose.fi * 180.0 / M_PI);
                sessions[i].point_clouds_container.point_clouds[i].gui_rotation[2] = (float)(sessions[i].point_clouds_container.point_clouds[0].pose.ka * 180.0 / M_PI);

                ImGui::End();

                Eigen::Affine3d curr_m_pose = sessions[i].point_clouds_container.point_clouds[0].m_pose;
                for (int j = 1; j < sessions[i].point_clouds_container.point_clouds.size(); j++)
                {
                    curr_m_pose = curr_m_pose * (all_m_poses[j - 1].inverse() * all_m_poses[j]);
                    sessions[i].point_clouds_container.point_clouds[j].m_pose = curr_m_pose;
                    sessions[i].point_clouds_container.point_clouds[j].pose = pose_tait_bryan_from_affine_matrix(sessions[i].point_clouds_container.point_clouds[j].m_pose);

                    sessions[i].point_clouds_container.point_clouds[j].gui_translation[0] = (float)sessions[i].point_clouds_container.point_clouds[j].pose.px;
                    sessions[i].point_clouds_container.point_clouds[j].gui_translation[1] = (float)sessions[i].point_clouds_container.point_clouds[j].pose.py;
                    sessions[i].point_clouds_container.point_clouds[j].gui_translation[2] = (float)sessions[i].point_clouds_container.point_clouds[j].pose.pz;

                    sessions[i].point_clouds_container.point_clouds[j].gui_rotation[0] = (float)(sessions[i].point_clouds_container.point_clouds[j].pose.om * 180.0 / M_PI);
                    sessions[i].point_clouds_container.point_clouds[j].gui_rotation[1] = (float)(sessions[i].point_clouds_container.point_clouds[j].pose.fi * 180.0 / M_PI);
                    sessions[i].point_clouds_container.point_clouds[j].gui_rotation[2] = (float)(sessions[i].point_clouds_container.point_clouds[j].pose.ka * 180.0 / M_PI);
                }
            }
        }
        if (gizmo_all_sessions)
        {
            for (size_t i = 0; i < sessions.size(); i++)
            {
                // gizmo_all_sessions;
                if (!sessions[i].is_gizmo && !sessions[i].is_ground_truth)
                {
                    std::vector<Eigen::Affine3d> all_m_poses;
                    for (int j = 0; j < sessions[i].point_clouds_container.point_clouds.size(); j++)
                    {
                        all_m_poses.push_back(sessions[i].point_clouds_container.point_clouds[j].m_pose);
                    }

                    Eigen::Affine3d m_rel_org = prev_pose_manipulated.inverse() * sessions[i].point_clouds_container.point_clouds[0].m_pose;

                    Eigen::Affine3d m_new = prev_pose_after_gismo * m_rel_org;

                    sessions[i].point_clouds_container.point_clouds[0].m_pose = m_new;
                    sessions[i].point_clouds_container.point_clouds[0].pose = pose_tait_bryan_from_affine_matrix(sessions[i].point_clouds_container.point_clouds[0].m_pose);

                    sessions[i].point_clouds_container.point_clouds[i].gui_translation[0] = (float)sessions[i].point_clouds_container.point_clouds[0].pose.px;
                    sessions[i].point_clouds_container.point_clouds[i].gui_translation[1] = (float)sessions[i].point_clouds_container.point_clouds[0].pose.py;
                    sessions[i].point_clouds_container.point_clouds[i].gui_translation[2] = (float)sessions[i].point_clouds_container.point_clouds[0].pose.pz;

                    sessions[i].point_clouds_container.point_clouds[i].gui_rotation[0] = (float)(sessions[i].point_clouds_container.point_clouds[0].pose.om * 180.0 / M_PI);
                    sessions[i].point_clouds_container.point_clouds[i].gui_rotation[1] = (float)(sessions[i].point_clouds_container.point_clouds[0].pose.fi * 180.0 / M_PI);
                    sessions[i].point_clouds_container.point_clouds[i].gui_rotation[2] = (float)(sessions[i].point_clouds_container.point_clouds[0].pose.ka * 180.0 / M_PI);

                    Eigen::Affine3d curr_m_pose = sessions[i].point_clouds_container.point_clouds[0].m_pose;
                    for (int j = 1; j < sessions[i].point_clouds_container.point_clouds.size(); j++)
                    {
                        curr_m_pose = curr_m_pose * (all_m_poses[j - 1].inverse() * all_m_poses[j]);
                        sessions[i].point_clouds_container.point_clouds[j].m_pose = curr_m_pose;
                        sessions[i].point_clouds_container.point_clouds[j].pose = pose_tait_bryan_from_affine_matrix(sessions[i].point_clouds_container.point_clouds[j].m_pose);

                        sessions[i].point_clouds_container.point_clouds[j].gui_translation[0] = (float)sessions[i].point_clouds_container.point_clouds[j].pose.px;
                        sessions[i].point_clouds_container.point_clouds[j].gui_translation[1] = (float)sessions[i].point_clouds_container.point_clouds[j].pose.py;
                        sessions[i].point_clouds_container.point_clouds[j].gui_translation[2] = (float)sessions[i].point_clouds_container.point_clouds[j].pose.pz;

                        sessions[i].point_clouds_container.point_clouds[j].gui_rotation[0] = (float)(sessions[i].point_clouds_container.point_clouds[j].pose.om * 180.0 / M_PI);
                        sessions[i].point_clouds_container.point_clouds[j].gui_rotation[1] = (float)(sessions[i].point_clouds_container.point_clouds[j].pose.fi * 180.0 / M_PI);
                        sessions[i].point_clouds_container.point_clouds[j].gui_rotation[2] = (float)(sessions[i].point_clouds_container.point_clouds[j].pose.ka * 180.0 / M_PI);
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

            const int &index_src = edges[index_active_edge].index_from;

            const Eigen::Affine3d &m_src = sessions[edges[index_active_edge].index_session_from].point_clouds_container.point_clouds.at(index_src).m_pose;
            edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_src.inverse() * m_g);

            ImGui::End();
        }
    }
    /*if (!manual_pose_graph_loop_closure_mode)
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
    }*/

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

        if (state == GLUT_DOWN)
        {
            mouse_buttons |= 1 << glut_button;

            //-
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
    glutCreateWindow("multi_session_registration_step_3 v0.45");
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

bool optimize(std::vector<Session> &sessions)
{
    for (auto &session : sessions)
    {
        // std::cout << session.point_clouds_container.point_clouds.size() << std::endl;
        for (auto &pc : session.point_clouds_container.point_clouds)
        {
            pc.m_pose_temp = pc.m_pose;
        }
    }

    std::vector<int> sums;
    sums.push_back(0);
    int sum = 0;

    for (int i = 0; i < sessions.size(); i++)
    {
        sum += sessions[i].point_clouds_container.point_clouds.size();
        sums.push_back(sum);
    }

    std::cout << "Compute Pose Graph SLAM" << std::endl;

    ///////////////////////////////////////////////////////////////////
    // graph slam
    bool is_ok = true;
    std::vector<Eigen::Affine3d> m_poses;
    std::vector<Eigen::Affine3d> poses_motion_model;
    std::vector<int> index_trajectory;

    for (int j = 0; j < sessions.size(); j++)
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
    int iterations = 10;
    bool is_fix_first_node = true;

    // if (gnss.gnss_poses.size() > 0)
    //{
    //     is_fix_first_node = false;
    // }

    std::vector<int> indexes_ground_truth;
    for (int j = 0; j < sessions.size(); j++)
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
    double angle = 0.1 / 180.0 * M_PI;
    double wangle = 1.0 / (angle * angle);

    for (int i = 1; i < poses_motion_model.size(); i++)
    {
        if (index_trajectory[i - 1] == index_trajectory[i])
        {
            Eigen::Affine3d m_rel = poses_motion_model[i - 1].inverse() * poses_motion_model[i];
            Edge edge;
            edge.index_from = i - 1;
            edge.index_to = i;
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

    for (int i = 0; i < sessions.size(); i++)
    {
        for (int j = 0; j < sessions[i].manual_pose_graph_loop_closure.edges.size(); j++)
        {
            // std::cout << "add manual_pose_graph_loop_closure.edge" << std::endl;
            Edge edge;
            edge.index_from = sessions[i].manual_pose_graph_loop_closure.edges[j].index_from + sums[i];
            edge.index_to = sessions[i].manual_pose_graph_loop_closure.edges[j].index_to + sums[i];
            edge.relative_pose_tb = sessions[i].manual_pose_graph_loop_closure.edges[j].relative_pose_tb;
            edge.relative_pose_tb_weights = sessions[i].manual_pose_graph_loop_closure.edges[j].relative_pose_tb_weights;
            // edge.is_fixed_fi = ToDo
            all_edges.push_back(edge);
        }
    }

    for (int i = 0; i < edges.size(); i++)
    {
        // std::cout << "add edges" << std::endl;
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
        std::cout << "iteration " << iter + 1 << " of " << iterations << std::endl;
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
            tripletListP.emplace_back(ir, ir, all_edges[i].relative_pose_tb_weights.px * get_cauchy_w(delta(0, 0), 1));
            tripletListP.emplace_back(ir + 1, ir + 1, all_edges[i].relative_pose_tb_weights.py * get_cauchy_w(delta(1, 0), 1));
            tripletListP.emplace_back(ir + 2, ir + 2, all_edges[i].relative_pose_tb_weights.pz * get_cauchy_w(delta(2, 0), 1));
            tripletListP.emplace_back(ir + 3, ir + 3, all_edges[i].relative_pose_tb_weights.om * get_cauchy_w(delta(3, 0), 1));
            tripletListP.emplace_back(ir + 4, ir + 4, all_edges[i].relative_pose_tb_weights.fi * get_cauchy_w(delta(4, 0), 1));
            tripletListP.emplace_back(ir + 5, ir + 5, all_edges[i].relative_pose_tb_weights.ka * get_cauchy_w(delta(5, 0), 1));
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

            for (int i = 1; i < index_trajectory.size(); i++)
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

        /*double angle = 1.0 / 180.0 * M_PI;
        double wangle = 1.0 / (angle * angle);

        for (int index = 0; index < indexes_ground_truth.size(); index++)
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
        /*for (int index_pose = 0; index_pose < point_clouds_container.point_clouds.size(); index_pose++)
        {
            const auto &pc = point_clouds_container.point_clouds[index_pose];
            for (int i = 0; i < gnss.gnss_poses.size(); i++)
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

        std::cout << "AtPA.size: " << AtPA.size() << std::endl;
        std::cout << "AtPB.size: " << AtPB.size() << std::endl;

        std::cout << "start solving AtPA=AtPB" << std::endl;
        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);

        std::cout << "x = solver.solve(AtPB)" << std::endl;
        Eigen::SparseMatrix<double> x = solver.solve(AtPB);

        std::vector<double> h_x;

        for (int k = 0; k < x.outerSize(); ++k)
        {
            for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            {
                h_x.push_back(it.value());
            }
        }

        std::cout << "h_x.size(): " << h_x.size() << std::endl;

        std::cout << "AtPA=AtPB SOLVED" << std::endl;
        // std::cout << "updates:" << std::endl;
        // for (size_t i = 0; i < h_x.size(); i += 6)
        //{
        //     std::cout << h_x[i] << "," << h_x[i + 1] << "," << h_x[i + 2] << "," << h_x[i + 3] << "," << h_x[i + 4] << "," << h_x[i + 5] << std::endl;
        // }

        if (h_x.size() == 6 * poses.size())
        {
            int counter = 0;

            for (size_t i = 0; i < poses.size(); i++)
            {

                TaitBryanPose pose = poses[i];
                poses[i].px += h_x[counter++] * 0.1;
                poses[i].py += h_x[counter++] * 0.1;
                poses[i].pz += h_x[counter++] * 0.1;
                poses[i].om += h_x[counter++] * 0.1;
                poses[i].fi += h_x[counter++] * 0.1;
                poses[i].ka += h_x[counter++] * 0.1;

                if (i == 0 && is_fix_first_node)
                {
                    poses[i] = pose;
                }
            }
            std::cout << "optimizing with tait bryan finished" << std::endl;
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
                {
                    m_poses[i] = affine_matrix_from_pose_tait_bryan(poses[i]);
                }
                else if (is_cw)
                {
                    m_poses[i] = affine_matrix_from_pose_tait_bryan(poses[i]).inverse();
                }
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
                    {
                        index = 0;
                    }
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
        {
            pc.m_pose = pc.m_pose_temp;
        }
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