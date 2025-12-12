#include <session.h>
#include <nlohmann/json.hpp>
#include <filesystem>
#include "../../shared/include/HDMapping/Version.hpp"

namespace fs = std::filesystem;

bool Session::load(const std::string &file_name, bool is_decimate, double bucket_x, double bucket_y, double bucket_z, bool calculate_offset)
{
    this->session_file_name = file_name;
    std::cout << "Loading file: '" << file_name << "'" << std::endl;
    // point_clouds_container.point_clouds.clear();

    std::string folder_name;
    std::string out_folder_name;
    std::string poses_file_name;
    std::string initial_poses_file_name;
    std::string out_poses_file_name;

    std::vector<PoseGraphLoopClosure::Edge> loop_closure_edges;
    std::vector<std::string> laz_file_names;
    std::vector<bool> vfuse_inclination_from_IMU;
    std::vector<bool> vfixed_x;
    std::vector<bool> vfixed_y;
    std::vector<bool> vfixed_z;
    std::vector<bool> vfixed_om;
    std::vector<bool> vfixed_fi;
    std::vector<bool> vfixed_ka;

    // Get a loaded file directory
    std::string directory = fs::path(file_name).parent_path().string();

    // Local pathUpdater lambda (keep directories unchanged, update files to be in session directory)
    auto getNewPath = [&](const std::string &path) -> std::string
    {
        fs::path p(path);
        if (is_directory(p))
            return p.string();
        return (fs::path(directory) / p.filename()).string();
    };

    try
    {
        std::ifstream fs(file_name);
        if (!fs.good())
            return false;
        nlohmann::json data = nlohmann::json::parse(fs);
        fs.close();

        const auto &project_settings_json = data["Session Settings"];
        point_clouds_container.offset.x() = project_settings_json.value("offset_x", 0.0);
        point_clouds_container.offset.y() = project_settings_json.value("offset_y", 0.0);
        point_clouds_container.offset.z() = project_settings_json.value("offset_z", 0.0);
        point_clouds_container.offset_to_apply.x() = project_settings_json.value("offset_to_apply_x", 0.0);
        point_clouds_container.offset_to_apply.y() = project_settings_json.value("offset_to_apply_y", 0.0);
        point_clouds_container.offset_to_apply.z() = project_settings_json.value("offset_to_apply_z", 0.0);
        folder_name = getNewPath(project_settings_json.value("folder_name", ""));
        out_folder_name = getNewPath(project_settings_json.value("out_folder_name", ""));
        poses_file_name = getNewPath(project_settings_json.value("poses_file_name", ""));
        initial_poses_file_name = getNewPath(project_settings_json.value("initial_poses_file_name", ""));
        std::cout << "!!" << initial_poses_file_name << std::endl;
        out_poses_file_name = getNewPath(project_settings_json.value("out_poses_file_name", ""));

        is_ground_truth = project_settings_json.value("ground_truth", false);

        for (const auto &edge_json : data["loop_closure_edges"])
        {
            PoseGraphLoopClosure::Edge edge;
            edge.index_from = edge_json["index_from"];
            edge.index_to = edge_json["index_to"];
            edge.is_fixed_fi = edge_json.value("is_fixed_fi", false);
            edge.is_fixed_ka = edge_json.value("is_fixed_ka", false);
            edge.is_fixed_om = edge_json.value("is_fixed_om", false);
            edge.is_fixed_px = edge_json.value("is_fixed_px", false);
            edge.is_fixed_py = edge_json.value("is_fixed_py", false);
            edge.is_fixed_pz = edge_json.value("is_fixed_pz", false);
            edge.relative_pose_tb.fi = edge_json.value("fi", 0.0);
            edge.relative_pose_tb.ka = edge_json.value("ka", 0.0);
            edge.relative_pose_tb.om = edge_json.value("om", 0.0);
            edge.relative_pose_tb.px = edge_json.value("px", 0.0);
            edge.relative_pose_tb.py = edge_json.value("py", 0.0);
            edge.relative_pose_tb.pz = edge_json.value("pz", 0.0);
            edge.relative_pose_tb_weights.fi = edge_json.value("w_fi", 0.0);
            edge.relative_pose_tb_weights.ka = edge_json.value("w_ka", 0.0);
            edge.relative_pose_tb_weights.om = edge_json.value("w_om", 0.0);
            edge.relative_pose_tb_weights.px = edge_json.value("w_px", 0.0);
            edge.relative_pose_tb_weights.py = edge_json.value("w_py", 0.0);
            edge.relative_pose_tb_weights.pz = edge_json.value("w_pz", 0.0);
            loop_closure_edges.push_back(edge);
        }

        for (const auto &fn_json : data["laz_file_names"])
        {
            const std::string fn = getNewPath(fn_json["file_name"]);
            laz_file_names.push_back(fn);

            vfixed_x.push_back(fn_json.value("fixed_x", false));
            vfixed_y.push_back(fn_json.value("fixed_y", false));
            vfixed_z.push_back(fn_json.value("fixed_z", false));
            vfixed_om.push_back(fn_json.value("fixed_om", false));
            vfixed_fi.push_back(fn_json.value("fixed_fi", false));
            vfixed_ka.push_back(fn_json.value("fixed_ka", false));
            vfuse_inclination_from_IMU.push_back(fn_json.value("fuse_inclination_from_IMU", false));
        }

        std::cout << "Loaded: "
                  << "offset_x: " << point_clouds_container.offset.x()
                  << ", offset_y: " << point_clouds_container.offset.y()
                  << ", offset_z: " << point_clouds_container.offset.z()
                  << " [m]\n";
        std::cout << "Folder_name: '" << folder_name << "'\n";
        std::cout << "Out_folder_name: '" << out_folder_name << "'\n";
        std::cout << "Initial_poses_file_name: '" << initial_poses_file_name << "'\n";
        std::cout << "Poses_file_name: '" << poses_file_name << "'\n";
        std::cout << "Out_poses_file_name: '" << out_poses_file_name << "'\n";

        if (!loop_closure_edges.empty())
        {
            std::cout << "Loop closure edges:" << std::endl;

            for (const auto &edge : loop_closure_edges)
            {
                std::cout << "<<<<<<<<<<<<<<<<<<<" << std::endl;
                std::cout << "index_from: " << edge.index_from
                          << ", index_to: " << edge.index_to << std::endl;
                std::cout << "is_fixed fi: " << edge.is_fixed_fi
                          << ", ka: " << edge.is_fixed_ka
                          << ", om: " << edge.is_fixed_om << std::endl;
                std::cout << "is_fixed px: " << edge.is_fixed_px
                          << ", py: " << edge.is_fixed_py
                          << ", pz: " << edge.is_fixed_pz << std::endl;
                std::cout << "relative_pose_tb fi: " << edge.relative_pose_tb.fi
                          << ", ka: " << edge.relative_pose_tb.ka
                          << ", om: " << edge.relative_pose_tb.om << std::endl;
                std::cout << "relative_pose_tb px: " << edge.relative_pose_tb.px
                          << ", py: " << edge.relative_pose_tb.py
                          << ", pz: " << edge.relative_pose_tb.pz << std::endl;
                std::cout << "relative_pose_tb_weights fi: " << edge.relative_pose_tb_weights.fi
                          << ", ka: " << edge.relative_pose_tb_weights.ka
                          << ", om: " << edge.relative_pose_tb_weights.om << std::endl;
                std::cout << "relative_pose_tb_weights px: " << edge.relative_pose_tb_weights.px
                          << ", py: " << edge.relative_pose_tb_weights.py
                          << ", pz: " << edge.relative_pose_tb_weights.pz << std::endl;
            }
        }

        // std::cout << "------laz file names-----" << std::endl;
        // for (const auto &fn : laz_file_names)
        //{
        //     std::cout << "'" << fn << "'" << std::endl;
        // }
#if WITH_GUI == 1
        for (const auto &gcp_json : data["ground_control_points"])
        {
            GroundControlPoint gcp;
            std::string name = gcp_json.value("name", "");
            strcpy(gcp.name, name.c_str());
            gcp.x = gcp_json.value("x", 0.0);
            gcp.y = gcp_json.value("y", 0.0);
            gcp.z = gcp_json.value("z", 0.0);
            gcp.sigma_x = gcp_json.value("sigma_x", 0.0);
            gcp.sigma_y = gcp_json.value("sigma_y", 0.0);
            gcp.sigma_z = gcp_json.value("sigma_z", 0.0);
            gcp.lidar_height_above_ground = gcp_json.value("lidar_height_above_ground", 0.0);
            gcp.index_to_node_inner = gcp_json["index_to_node_inner"];
            gcp.index_to_node_outer = gcp_json["index_to_node_outer"];
            ground_control_points.gpcs.push_back(gcp);

            std::cout << "adding gcp[" << name << "]" << std::endl;
        };

        for (const auto &cp_json : data["control_points"])
        {
            ControlPoint cp;
            std::string name = cp_json.value("name", "");
            strcpy(cp.name, name.c_str());
            cp.x_source_local = cp_json.value("x_source_local", 0.0);
            cp.y_source_local = cp_json.value("y_source_local", 0.0);
            cp.z_source_local = cp_json.value("z_source_local", 0.0);
            cp.x_target_global = cp_json.value("x_target_global", 0.0);
            cp.y_target_global = cp_json.value("y_target_global", 0.0);
            cp.z_target_global = cp_json.value("z_target_global", 0.0);
            cp.sigma_x = cp_json.value("sigma_x", 0.0);
            cp.sigma_y = cp_json.value("sigma_y", 0.0);
            cp.sigma_z = cp_json.value("sigma_z", 0.0);
            cp.index_to_pose = cp_json["index_to_pose"];
            cp.is_z_0 = cp_json.value("is_z_0", false);
            control_points.cps.push_back(cp);

            std::cout << "adding cp[" << name << "]" << std::endl;
        };
#endif

        std::cout << "-----------------------------" << std::endl;

        // loading all data
        point_clouds_container.load_whu_tls(laz_file_names, is_decimate, bucket_x, bucket_y, bucket_z, calculate_offset, this->load_cache_mode);

        if (laz_file_names.size() > 0)
        {
            working_directory = fs::path(laz_file_names[0]).parent_path().string();

            point_clouds_container.folder_name = folder_name;
            point_clouds_container.out_folder_name = out_folder_name;
            point_clouds_container.out_poses_file_name = out_poses_file_name;

            if (!point_clouds_container.update_initial_poses_from_RESSO(working_directory.c_str(), initial_poses_file_name.c_str()))
            {
                std::cout << __FILE__ << " " << __LINE__ << std::endl;
                std::cout << "check input files" << std::endl;
                return false;
            }
            else
            {
                point_clouds_container.initial_poses_file_name = initial_poses_file_name;
                std::cout << "updated: " << point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
            }

            if (!point_clouds_container.update_poses_from_RESSO(working_directory.c_str(), poses_file_name.c_str()))
            {
                std::cout << __FILE__ << " " << __LINE__ << std::endl;
                std::cout << "check input files" << std::endl;
                return false;
            }
            else
            {
                std::cout << "updated: " << point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
                point_clouds_container.poses_file_name = poses_file_name;
            }
        }

        pose_graph_loop_closure.edges = loop_closure_edges;

        // change color
        render_color[0] = float(rand() % 255) / 255.0;
        render_color[1] = float(rand() % 255) / 255.0;
        render_color[2] = float(rand() % 255) / 255.0;

        int index = 0;
        for (auto &pc : point_clouds_container.point_clouds)
        {
            pc.render_color[0] = render_color[0];
            pc.render_color[1] = render_color[1];
            pc.render_color[2] = render_color[2];
            pc.fixed_x = vfixed_x[index];
            pc.fixed_y = vfixed_y[index];
            pc.fixed_z = vfixed_z[index];
            pc.fixed_om = vfixed_om[index];
            pc.fixed_fi = vfixed_fi[index];
            pc.fixed_ka = vfixed_ka[index];
            pc.fuse_inclination_from_IMU = vfuse_inclination_from_IMU[index];
            index++;
        }

        // sanity check;

        std::cout << std::setprecision(10);
        for (int i = 0; i < point_clouds_container.point_clouds.size(); i++)
            //auto &pc : point_clouds_container.point_clouds)
        {
            for (int j = 1; j < point_clouds_container.point_clouds[i].local_trajectory.size(); j++)
            {
                Eigen::Affine3d m = point_clouds_container.point_clouds[i].local_trajectory[j - 1].m_pose.inverse() * point_clouds_container.point_clouds[i].local_trajectory[j].m_pose;

                TaitBryanPose tb_pose = pose_tait_bryan_from_affine_matrix(m);

                TaitBryanPose tb_prev;
                tb_prev.om = point_clouds_container.point_clouds[i].local_trajectory[j - 1].imu_om_fi_ka.x();
                tb_prev.fi = point_clouds_container.point_clouds[i].local_trajectory[j - 1].imu_om_fi_ka.y();
                tb_prev.ka = point_clouds_container.point_clouds[i].local_trajectory[j - 1].imu_om_fi_ka.z();
                tb_prev.px = 0.0;
                tb_prev.py = 0.0;
                tb_prev.pz = 0.0;

                Eigen::Affine3d m_prev = affine_matrix_from_pose_tait_bryan(tb_prev);

                TaitBryanPose tb_curr;
                tb_curr.om = point_clouds_container.point_clouds[i].local_trajectory[j].imu_om_fi_ka.x();
                tb_curr.fi = point_clouds_container.point_clouds[i].local_trajectory[j].imu_om_fi_ka.y();
                tb_curr.ka = point_clouds_container.point_clouds[i].local_trajectory[j].imu_om_fi_ka.z();
                tb_curr.px = 0.0;
                tb_curr.py = 0.0;
                tb_curr.pz = 0.0;

                Eigen::Affine3d m_curr = affine_matrix_from_pose_tait_bryan(tb_curr);

                Eigen::Affine3d mm = m_prev.inverse() * m_curr;

                TaitBryanPose tb_pose_mm = pose_tait_bryan_from_affine_matrix(mm);

                Eigen::Vector3d diff (fabs(tb_pose_mm.om - tb_pose.om), fabs(tb_pose_mm.fi - tb_pose.fi), fabs(tb_pose_mm.ka - tb_pose.ka));

                point_clouds_container.point_clouds[i].local_trajectory[j].imu_diff_angle_om_fi_ka_deg = diff;

                // TaitBryanPose tb_pose2;
                //tb_pose2.om = point_clouds_container.point_clouds[i].local_trajectory[j-1].imu_om_fi_ka.x()

                //std::cout << tb_pose.om << " " << tb_pose.fi << " " << tb_pose.ka << " "
                //          << point_clouds_container.point_clouds[i].local_trajectory[j].imu_om_fi_ka.x() << " " <<
                //    point_clouds_container.point_clouds[i].local_trajectory[j].imu_om_fi_ka.y() << " " 
                //    << point_clouds_container.point_clouds[i].local_trajectory[j].imu_om_fi_ka.z() <<  std::endl;
            }
            //struct LocalTrajectoryNode{
            //    std::pair<double, double> timestamps;
            //    Eigen::Affine3d m_pose;
            //    Eigen::Vector3d imu_om_fi_ka;
            //    Eigen::Vector3d imu_diff_angle_om_fi_ka_deg;
	        //};
            //pc.local_trajectory
        }

        return true;
    }
    catch (const std::exception &e)
    {
        std::cout << "can't load session: " << e.what() << std::endl;
        return false;
    }
}

bool Session::save(const std::string &file_name, const std::string &poses_file_name, const std::string &initial_poses_file_name, bool is_subsession)
{
    std::cout << "saving file: '" << file_name << "'" << std::endl;

    nlohmann::json jj;
    nlohmann::json j;

    j["offset_x"] = point_clouds_container.offset.x();
    j["offset_y"] = point_clouds_container.offset.y();
    j["offset_z"] = point_clouds_container.offset.z();
    j["offset_to_apply_x"] = point_clouds_container.offset_to_apply.x();
    j["offset_to_apply_y"] = point_clouds_container.offset_to_apply.y();
    j["offset_to_apply_z"] = point_clouds_container.offset_to_apply.z();
    j["folder_name"] = point_clouds_container.folder_name;
    j["out_folder_name"] = point_clouds_container.out_folder_name;
    j["poses_file_name"] = poses_file_name;                 // point_clouds_container.poses_file_name;
    j["initial_poses_file_name"] = initial_poses_file_name; // point_clouds_container.initial_poses_file_name;
    j["out_poses_file_name"] = point_clouds_container.out_poses_file_name;
    j["ground_truth"] = is_ground_truth;
    j["exporting_software_version"] = HDMAPPING_VERSION_STRING;
    jj["Session Settings"] = j;

    nlohmann::json jloop_closure_edges;
    if (!is_subsession)
    {
        for (const auto &edge : pose_graph_loop_closure.edges)
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
                {"is_fixed_ka", edge.is_fixed_ka}};
            jloop_closure_edges.push_back(jloop_closure_edge);
        }
    }
    jj["loop_closure_edges"] = jloop_closure_edges;

    nlohmann::json jlaz_file_names;
    for (const auto &pc : point_clouds_container.point_clouds)
    {
        if (is_subsession)
        {
            if (pc.visible)
            {
                nlohmann::json jfn{
                    {"file_name", pc.file_name},
                    {"fuse_inclination_from_IMU", pc.fuse_inclination_from_IMU},
                    {"fixed_x", pc.fixed_x},
                    {"fixed_y", pc.fixed_y},
                    {"fixed_z", pc.fixed_z},
                    {"fixed_om", pc.fixed_om},
                    {"fixed_fi", pc.fixed_fi},
                    {"fixed_ka", pc.fixed_ka}};
                jlaz_file_names.push_back(jfn);
            }
        }
        else
        {
            nlohmann::json jfn{
                {"file_name", pc.file_name},
                {"fuse_inclination_from_IMU", pc.fuse_inclination_from_IMU},
                {"fixed_x", pc.fixed_x},
                {"fixed_y", pc.fixed_y},
                {"fixed_z", pc.fixed_z},
                {"fixed_om", pc.fixed_om},
                {"fixed_fi", pc.fixed_fi},
                {"fixed_ka", pc.fixed_ka}};
            jlaz_file_names.push_back(jfn);
        }
    }
    jj["laz_file_names"] = jlaz_file_names;
#if WITH_GUI == 1
    nlohmann::json jgcps;
    for (const auto &gcp : ground_control_points.gpcs)
    {
        nlohmann::json jgcp{
            {"name", gcp.name},
            {"x", gcp.x},
            {"y", gcp.y},
            {"z", gcp.z},
            {"sigma_x", gcp.sigma_x},
            {"sigma_y", gcp.sigma_y},
            {"sigma_z", gcp.sigma_z},
            {"lidar_height_above_ground", gcp.lidar_height_above_ground},
            {"index_to_node_inner", gcp.index_to_node_inner},
            {"index_to_node_outer", gcp.index_to_node_outer}};
        jgcps.push_back(jgcp);
    }
    jj["ground_control_points"] = jgcps;
#endif

#if WITH_GUI == 1
    nlohmann::json jcps;
    for (const auto &cp : control_points.cps)
    {
        nlohmann::json jcp{
            {"name", cp.name},
            {"x_source_local", cp.x_source_local},
            {"y_source_local", cp.y_source_local},
            {"z_source_local", cp.z_source_local},
            {"x_target_global", cp.x_target_global},
            {"y_target_global", cp.y_target_global},
            {"z_target_global", cp.z_target_global},
            {"sigma_x", cp.sigma_x},
            {"sigma_y", cp.sigma_y},
            {"sigma_z", cp.sigma_z},
            {"index_to_pose", cp.index_to_pose},
            {"is_z_0", cp.is_z_0}};
        jcps.push_back(jcp);
    }
    jj["control_points"] = jcps;
#endif

    std::ofstream fs(file_name);
    if (!fs.good())
        return false;
    fs << jj.dump(2);
    fs.close();
    return true;
}

void Session::fill_session_from_worker_data(
    const std::vector<WorkerData> &worker_data, bool save_selected,
    bool filter_on_export, bool apply_pose, double threshould_output_filter)
{
    this->point_clouds_container.point_clouds.clear(); // clear whatever was there
    for (int i = 0; i < worker_data.size(); i++)
    {
        WorkerData wd;
        std::vector<Point3Di> intermediate_points;
        if (!load_vector_data(worker_data[i].intermediate_points_cache_file_name.string(), intermediate_points))
        {
            std::cout << "problem with load_vector_data '" << worker_data[i].intermediate_points_cache_file_name.string() << "'" << std::endl;
        }

        if (!save_selected || worker_data[i].show)
        {
            PointCloud pc;
            for (const auto &p : intermediate_points)
            {
                if (!filter_on_export || (p.point.norm() > threshould_output_filter))
                {
                    Eigen::Vector3d pt = p.point;
                    if (apply_pose)
                    {
                        pt = worker_data[i].intermediate_trajectory[p.index_pose] * p.point;
                        pt = worker_data[i].intermediate_trajectory[0].inverse() * pt;
                    }
                    pc.points_local.push_back(pt);
                    pc.intensities.push_back(p.intensity);
                    pc.timestamps.push_back(p.timestamp);
                }
            }
            // TODO: check if this is correct pose to be applied
            // pc.m_pose = worker_data[i].intermediate_trajectory[0].inverse();
            pc.m_pose = worker_data[i].intermediate_trajectory[0];
            this->point_clouds_container.point_clouds.push_back(pc);
        }
    }
}
