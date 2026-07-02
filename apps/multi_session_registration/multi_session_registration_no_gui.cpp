#include <cmath>
#include <filesystem>

#include <GL/freeglut.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <nlohmann/json.hpp>

#include <Eigen/Eigen>

#include <Core/export_laz.h>
#include <Core/icp.h>
#include <Core/ndt.h>
#include <Core/observation_picking.h>
#include <Core/pair_wise_iterative_closest_point.h>
#include <Core/pfd_wrapper.hpp>
#include <Core/registration_plane_feature.h>
#include <Core/session.h>

#include <portable-file-dialogs.h>

#include <HDMapping/Version.hpp>

#ifdef _WIN32
#include "resource.h"
#include <windows.h>

#endif

#include "multi_session_factor_graph.h"

std::string winTitle = std::string("Step 3 (Multi session registration) ") + HDMAPPING_VERSION_STRING;

std::vector<std::string> infoLines = {
    "This program is third/final step in MANDEYE process",
    "",
    "First step: create project by adding sessions (result of 'multi_view_tls_registration_step_2' program)",
    "Last step: save project",
    "To produce map use 'multi_view_tls_registration_step_2' export functionality"
};

float m_gizmo[] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

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

bool update_rotation_center = false;

bool is_settings_gui = true;

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

std::vector<Edge> edges;
int index_active_edge = -1;
bool manipulate_active_edge = false;
bool edge_gizmo = false;

ProjectSettings project_settings;
std::vector<Session> sessions;

int viewer_reduce_rendered_trajectory = 1;
namespace fs = std::filesystem;

int num_edge_extended_before = 0;
int num_edge_extended_after = 0;

int gui_point_size = 2;

TaitBryanPose motion_model_weights = { 0.01, 0.01, 0.01, 0.1, 0.1, 0.1 };


// void save_trajectories_to_laz(
//     const Session& session,
//     const std::string& output_file_name,
//     float curve_consecutive_distance_meters,
//     float not_curve_consecutive_distance_meters,
//     bool is_trajectory_export_downsampling)
// {
//     std::vector<Eigen::Vector3d> pointcloud;
//     std::vector<unsigned short> intensity;
//     std::vector<double> timestamps;

//     float consecutive_distance = 0;
//     for (auto& p : session.point_clouds_container.point_clouds)
//     {
//         if (p.visible)
//         {
//             for (size_t i = 0; i < p.local_trajectory.size(); i++)
//             {
//                 const auto& pp = p.local_trajectory[i].m_pose.translation();
//                 Eigen::Vector3d vp;
//                 vp = p.m_pose * pp; // + session.point_clouds_container.offset;

//                 if (i > 0)
//                 {
//                     double dist = (p.local_trajectory[i].m_pose.translation() - p.local_trajectory[i - 1].m_pose.translation()).norm();
//                     consecutive_distance += dist;
//                 }

//                 bool is_curve = false;

//                 if (i > 100 && i < p.local_trajectory.size() - 100)
//                 {
//                     Eigen::Vector3d position_prev = p.local_trajectory[i - 100].m_pose.translation();
//                     Eigen::Vector3d position_curr = p.local_trajectory[i].m_pose.translation();
//                     Eigen::Vector3d position_next = p.local_trajectory[i + 100].m_pose.translation();

//                     Eigen::Vector3d v1 = position_curr - position_prev;
//                     Eigen::Vector3d v2 = position_next - position_curr;

//                     if (v1.norm() > 0 && v2.norm() > 0)
//                     {
//                         double angle_deg = fabs(acos(v1.dot(v2) / (v1.norm() * v2.norm())) * RAD_TO_DEG);

//                         if (angle_deg > 10.0)
//                         {
//                             is_curve = true;
//                         }
//                     }
//                 }
//                 double tol = not_curve_consecutive_distance_meters;

//                 if (is_curve)
//                 {
//                     tol = curve_consecutive_distance_meters;
//                 }

//                 if (!is_trajectory_export_downsampling)
//                 {
//                     pointcloud.push_back(vp);
//                     intensity.push_back(0);
//                     timestamps.push_back(p.local_trajectory[i].timestamps.first);
//                 }
//                 else
//                 {
//                     if (consecutive_distance >= tol)
//                     {
//                         consecutive_distance = 0;
//                         pointcloud.push_back(vp);
//                         intensity.push_back(0);
//                         timestamps.push_back(p.local_trajectory[i].timestamps.first);
//                     }
//                 }
//             }
//         }
//     }
//     // if (!exportLaz(output_file_name, pointcloud, intensity, gnss.offset_x, gnss.offset_y, gnss.offset_alt))
//     if (!exportLaz(
//             output_file_name,
//             pointcloud,
//             intensity,
//             timestamps,
//             session.point_clouds_container.offset.x(),
//             session.point_clouds_container.offset.y(),
//             session.point_clouds_container.offset.z()))
//     {
//         std::cout << "problem with saving file: " << output_file_name << std::endl;
//     }
// }

void createDXFPolyline(const std::string& filename, const std::vector<Eigen::Vector3d>& points)
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
    dxfFile << "8\n0\n"; // Layer 0
    dxfFile << "66\n1\n"; // Indicates the presence of vertices
    dxfFile << "70\n8\n"; // 1 = Open polyline

    // Write the VERTEX entities
    for (const auto& point : points)
    {
        dxfFile << "0\nVERTEX\n";
        dxfFile << "8\n0\n"; // Layer 0
        dxfFile << "10\n" << point.x() << "\n"; // X coordinate
        dxfFile << "20\n" << point.y() << "\n"; // Y coordinate
        dxfFile << "30\n" << point.z() << "\n"; // Z coordinate
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
    Session& session,
    const std::string& output_file_name,
    float curve_consecutive_distance_meters,
    float not_curve_consecutive_distance_meters,
    bool is_trajectory_export_downsampling,
    bool write_lidar_timestamp,
    bool write_unix_timestamp,
    bool use_quaternions,
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
        for (auto& p : session.point_clouds_container.point_clouds)
        {
            if (p.visible)
            {
                for (size_t i = 0; i < p.local_trajectory.size(); i++)
                {
                    const auto& m = p.local_trajectory[i].m_pose;
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
                                outfile << pose(0, 0) << "," << pose(0, 1) << "," << pose(0, 2) << "," << pose(1, 0) << "," << pose(1, 1)
                                        << "," << pose(1, 2) << "," << pose(2, 0) << "," << pose(2, 1) << "," << pose(2, 2) << std::endl;
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

bool save_project_settings(const std::string& file_name, const ProjectSettings& _project_settings)
{
    std::cout << "saving file: '" << file_name << "'" << std::endl;

    nlohmann::json jj;

    nlohmann::json jsession_file_names;
    for (const auto& pc : _project_settings.session_file_names)
    {
        nlohmann::json jfn{ { "session_file_name", pc } };
        jsession_file_names.push_back(jfn);
    }
    jj["session_file_names"] = jsession_file_names;

    nlohmann::json jloop_closure_edges;
    for (const auto& edge : edges)
    {
        nlohmann::json jloop_closure_edge{
            { "px", edge.relative_pose_tb.px },
            { "py", edge.relative_pose_tb.py },
            { "pz", edge.relative_pose_tb.pz },
            { "om", edge.relative_pose_tb.om },
            { "fi", edge.relative_pose_tb.fi },
            { "ka", edge.relative_pose_tb.ka },
            { "w_px", edge.relative_pose_tb_weights.px },
            { "w_py", edge.relative_pose_tb_weights.py },
            { "w_pz", edge.relative_pose_tb_weights.pz },
            { "w_om", edge.relative_pose_tb_weights.om },
            { "w_fi", edge.relative_pose_tb_weights.fi },
            { "w_ka", edge.relative_pose_tb_weights.ka },
            { "index_from", edge.index_from },
            { "index_to", edge.index_to },
            { "is_fixed_px", edge.is_fixed_px },
            { "is_fixed_py", edge.is_fixed_py },
            { "is_fixed_pz", edge.is_fixed_pz },
            { "is_fixed_om", edge.is_fixed_om },
            { "is_fixed_fi", edge.is_fixed_fi },
            { "is_fixed_ka", edge.is_fixed_ka },
            { "index_session_from", edge.index_session_from },
            { "index_session_to", edge.index_session_to },
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
    time_stamp_offset = std::numeric_limits<double>::max();

    for (const auto& s : sessions)
    {
        if (!s.point_clouds_container.point_clouds.empty() && !s.point_clouds_container.point_clouds[0].local_trajectory.empty())
        {
            double ts = s.point_clouds_container.point_clouds[0].local_trajectory[0].timestamps.first;
            if (ts < time_stamp_offset)
                time_stamp_offset = ts;
        }
    }

    std::cout << "new time_stamp_offset = " << time_stamp_offset << std::endl;
}

bool revert(std::vector<Session>& sessions)
{
    for (auto& session : sessions)
    {
        for (auto& pc : session.point_clouds_container.point_clouds)
            pc.m_pose = pc.m_pose_temp;
    }
    return true;
}

bool revert_to_initial(std::vector<Session>& sessions)
{
    for (auto& session : sessions)
    {
        for (auto& pc : session.point_clouds_container.point_clouds)
            pc.m_pose = pc.m_initial_pose;
    }
    return true;
}

bool save_results(std::vector<Session>& sessions)
{
    for (auto& session : sessions)
    {
        if (!session.is_ground_truth)
        {
            std::cout << "saving result to: " << session.point_clouds_container.poses_file_name << std::endl;
            session.point_clouds_container.save_poses(fs::path(session.point_clouds_container.poses_file_name).string(), false);
        }
    }
    return true;
}



bool loadProject(const std::string& file_name, ProjectSettings& _project_settings)
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

        for (const auto& fn_json : data["session_file_names"])
        {
            const std::string fn = fn_json["session_file_name"];
            _project_settings.session_file_names.push_back(fn);
            std::cout << "'" << fn << "'";
            if (!fs::exists(fn))
                std::cout << "  (WARNING: session file does not exist! Please manually adapt path)";
            std::cout << "\n";
        }

        edges.clear();
        for (const auto& edge_json : data["loop_closure_edges"])
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
    } catch (std::exception& e)
    {
        std::cout << "can't load project settings: " << e.what() << std::endl;
        return false;
    }

    loaded_sessions = false;
    time_stamp_offset = 0.0;

    return true;
}

void openProject()
{
    std::string input_file_name = "";
    input_file_name = mandeye::fd::OpenFileDialogOneFile("Open project", mandeye::fd::Project_filter);

    if (input_file_name.size() > 0)
    {
        loadProject(fs::path(input_file_name).string(), project_settings);
    }
}


void addSession()
{
    auto input_file_names = mandeye::fd::OpenFileDialog("Add session(s)", mandeye::fd::Session_filter, true);

    if (input_file_names.size() > 0)
    {
        for (const auto& input_file_name : input_file_names)
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
    for (const auto& ps : project_settings.session_file_names)
    {
        Session session;
        session.load(fs::path(ps).string(), is_decimate, bucket_x, bucket_y, bucket_z, calculate_offset);

        // making sure irelevant session specific settings that could affect rendering are off
        session.point_clouds_container.xz_intersection = false;
        session.point_clouds_container.yz_intersection = false;
        session.point_clouds_container.xy_intersection = false;
        session.point_clouds_container.xz_grid_10x10 = false;
        session.point_clouds_container.xz_grid_1x1 = false;
        session.point_clouds_container.xz_grid_01x01 = false;
        session.point_clouds_container.yz_grid_10x10 = false;
        session.point_clouds_container.yz_grid_1x1 = false;
        session.point_clouds_container.yz_grid_01x01 = false;
        session.point_clouds_container.xy_grid_10x10 = false;
        session.point_clouds_container.xy_grid_1x1 = false;
        session.point_clouds_container.xy_grid_01x01 = false;

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

    for (auto& e : edges)
    {
        e.index_session_from = map_reorder[e.index_session_from];
        e.index_session_to = map_reorder[e.index_session_to];
    }

    std::cout << "sessions reordered, ground truth should be in front" << std::endl;
    for (const auto& s : sessions)
    {
        std::cout << "session: '" << s.session_file_name << "' ground truth [" << int(s.is_ground_truth) << "]" << std::endl;
    }

    // update time_stamp_offset
    std::cout << "update time_stamp_offset" << std::endl;
    for (const auto& s : sessions)
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

void save_all_sessions_as_tum_cli()
{
    for (size_t i = 0; i < project_settings.session_file_names.size(); ++i)
    {
        if (i >= sessions.size())
            continue;


        Session& session = sessions[i];


        std::filesystem::path session_path =
            std::filesystem::path(project_settings.session_file_names[i]);


        std::filesystem::path parent =
            session_path.parent_path()
                        .parent_path()
                        .parent_path();


        std::filesystem::path tum_dir =
            parent / "tum";


        std::filesystem::create_directories(tum_dir);


        std::string name =
            session_path.parent_path().filename().string();


        std::filesystem::path out =
            tum_dir / (name + "_trajectory_tum.txt");


        std::cout << "Saving: " << out << std::endl;


        std::ofstream file(out);

        if (!file.is_open())
        {
            std::cerr << "Cannot create " << out << std::endl;
            continue;
        }


        for (const auto& pc :
             session.point_clouds_container.point_clouds)
        {
            if (!pc.visible)
                continue;


            for (const auto& traj :
                 pc.local_trajectory)
            {
                Eigen::Affine3d pose =
                    pc.m_pose * traj.m_pose;


                Eigen::Vector3d p =
                    pose.translation();


                Eigen::Quaterniond q(
                    pose.rotation()
                );


                double t =
                    double(traj.timestamps.first) / 1e9;


                file
                    << std::fixed
                    << std::setprecision(9)
                    << t << " "
                    << std::setprecision(10)
                    << p.x() << " "
                    << p.y() << " "
                    << p.z() << " "
                    << q.x() << " "
                    << q.y() << " "
                    << q.z() << " "
                    << q.w()
                    << "\n";
            }
        }


        file.close();

        std::cout << "DONE: " << out << std::endl;
    }
}


void set_sessions_to_origin()
{
    bool is_first_gt = false;

    if (!sessions.empty() && sessions[0].is_ground_truth)
        is_first_gt = true;


    Eigen::Affine3d m_gt = Eigen::Affine3d::Identity();


    if (!sessions.empty())
    {
        int ipc = -1;
        int ilt = -1;

        for (size_t a = 0; a < sessions[0].point_clouds_container.point_clouds.size(); a++)
        {
            for (size_t b = 0; b < sessions[0].point_clouds_container.point_clouds[a].local_trajectory.size(); b++)
            {
                if (sessions[0].point_clouds_container.point_clouds[a].local_trajectory[b].timestamps.first > time_stamp_offset)
                {
                    ipc = a;
                    ilt = b;
                    break;
                }
            }

            if (ipc != -1)
                break;
        }


        if (ipc != -1)
        {
            m_gt =
                sessions[0].point_clouds_container.point_clouds[ipc].m_pose *
                sessions[0].point_clouds_container.point_clouds[ipc].local_trajectory[ilt].m_pose;
        }
    }


    for (auto& session : sessions)
    {
        if (is_first_gt && session.is_ground_truth)
            continue;


        int ipc = -1;
        int ilt = -1;


        for (size_t a = 0; a < session.point_clouds_container.point_clouds.size(); a++)
        {
            for (size_t b = 0; b < session.point_clouds_container.point_clouds[a].local_trajectory.size(); b++)
            {
                if (session.point_clouds_container.point_clouds[a].local_trajectory[b].timestamps.first > time_stamp_offset)
                {
                    ipc = a;
                    ilt = b;
                    break;
                }
            }

            if (ipc != -1)
                break;
        }


        if (ipc != -1)
        {
            auto m =
                session.point_clouds_container.point_clouds[ipc].m_pose *
                session.point_clouds_container.point_clouds[ipc].local_trajectory[ilt].m_pose;


            auto inv = m.inverse();


            for (auto& pc : session.point_clouds_container.point_clouds)
                pc.m_pose = inv * pc.m_pose;


            for (auto& pc : session.point_clouds_container.point_clouds)
                pc.m_pose = m_gt * pc.m_pose;
        }
    }
}

int run_multi_session_registration(
    const std::vector<std::string>& session_files)
{
    try
    {
        // czyścimy poprzednie
        project_settings.session_file_names.clear();
        sessions.clear();

        // odpowiednik addSession()
        for (const auto& input_file_name : session_files)
        {
            std::cout 
                << "Adding session file: '"
                << input_file_name
                << "'\n";

            project_settings.session_file_names.push_back(input_file_name);
        }

        loaded_sessions = false;
        time_stamp_offset = 0.0;


        // odpowiednik przycisku Load Sessions
        loadSessions();


        if (!loaded_sessions)
        {
            throw std::runtime_error(
                "Sessions loading failed");
        }


        std::cout
            << "Loaded "
            << sessions.size()
            << " sessions\n";

        std::cout << "=== Applying set_sessions_to_origin ===" << std::endl;

        set_sessions_to_origin();

        std::cout << "=== Saving results BEFORE set_to_origin ===" << std::endl;

        for (auto& session : sessions)
        {
            if (!session.is_ground_truth)
            {
                std::cout 
                    << "Saving poses to: "
                    << session.point_clouds_container.poses_file_name
                    << std::endl;
            }
        }

        save_results(sessions);

        save_all_sessions_as_tum_cli();

        std::cout << "Export finished\n";

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "ERROR: "
            << e.what()
            << std::endl;

        return -1;
    }
}


int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        std::cout
            << "Usage: "
            << argv[0]
            << " session1.mjs session2.mjs ...\n";

        return 1;
    }


    std::vector<std::string> session_files;


    for(int i = 1; i < argc; i++)
    {
        session_files.push_back(argv[i]);
    }


    return run_multi_session_registration(session_files);
}