#include <session.h>
#include <nlohmann/json.hpp>
#include <filesystem>
namespace fs = std::filesystem;

std::string pathUpdater(std::string path, std::string newPath)
{
    fs::path p(path);
    fs::path newpath(newPath);
    if (is_directory(p))
    {
        return p.string();
    }

    // get filename
    fs::path filename = p.filename();
    return (newpath / filename).string();
}

bool Session::load(const std::string &file_name, bool is_decimate, double bucket_x, double bucket_y, double bucket_z, bool calculate_offset)
{
    this->session_file_name = file_name;
    std::cout
        << "loading file: '" << file_name << "'" << std::endl;
    // point_clouds_container.point_clouds.clear();

    std::string folder_name;
    std::string out_folder_name;
    std::string poses_file_name;
    std::string initial_poses_file_name;
    std::string out_poses_file_name;

    std::vector<PoseGraphLoopClosure::Edge> loop_closure_edges;
    std::vector<std::string> laz_file_names;

    // Get a loaded file directory
    std::string directory = fs::path(file_name).parent_path().string();

    try
    {
        std::ifstream fs(file_name);
        if (!fs.good())
            return false;
        nlohmann::json data = nlohmann::json::parse(fs);
        fs.close();

        auto project_settings_json = data["Session Settings"];
        point_clouds_container.offset.x() = project_settings_json["offset_x"];
        point_clouds_container.offset.y() = project_settings_json["offset_y"];
        point_clouds_container.offset.z() = project_settings_json["offset_z"];
        folder_name = pathUpdater(project_settings_json["folder_name"], directory);
        out_folder_name = pathUpdater(project_settings_json["out_folder_name"], directory);
        poses_file_name = pathUpdater(project_settings_json["poses_file_name"], directory);
        initial_poses_file_name = pathUpdater(project_settings_json["initial_poses_file_name"], directory);
        std::cout << "!!" << initial_poses_file_name << std::endl;
        out_poses_file_name = pathUpdater(project_settings_json["out_poses_file_name"], directory);
        if (project_settings_json.contains("ground_truth"))
        {
            is_ground_truth = project_settings_json["ground_truth"];
        }
        else
        {
            is_ground_truth = false;
        }

        for (const auto &edge_json : data["loop_closure_edges"])
        {
            PoseGraphLoopClosure::Edge edge;
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
            loop_closure_edges.push_back(edge);
        }

        for (const auto &fn_json : data["laz_file_names"])
        {
            std::string fn = pathUpdater(fn_json["file_name"], directory);
            laz_file_names.push_back(fn);
        }

        std::cout << "loaded from json: " << file_name << std::endl;
        std::cout << "offset_x: " << point_clouds_container.offset.x() << std::endl;
        std::cout << "offset_y: " << point_clouds_container.offset.y() << std::endl;
        std::cout << "offset_z: " << point_clouds_container.offset.z() << std::endl;
        std::cout << "folder_name: '" << folder_name << "'" << std::endl;
        std::cout << "out_folder_name: '" << out_folder_name << "'" << std::endl;
        std::cout << "poses_file_name: '" << poses_file_name << "'" << std::endl;
        std::cout << "initial_poses_file_name: '" << initial_poses_file_name << "'" << std::endl;
        std::cout << "out_poses_file_name: '" << out_poses_file_name << "'" << std::endl;

        std::cout << "------loop closure edges-----" << std::endl;

        for (const auto &edge : loop_closure_edges)
        {
            std::cout << "<<<<<<<<<<<<<<<<<<<" << std::endl;
            std::cout << "index_from: " << edge.index_from << std::endl;
            std::cout << "index_to: " << edge.index_to << std::endl;
            std::cout << "is_fixed_fi: " << edge.is_fixed_fi << std::endl;
            std::cout << "is_fixed_ka: " << edge.is_fixed_ka << std::endl;
            std::cout << "is_fixed_om: " << edge.is_fixed_om << std::endl;
            std::cout << "is_fixed_px: " << edge.is_fixed_px << std::endl;
            std::cout << "is_fixed_py: " << edge.is_fixed_py << std::endl;
            std::cout << "is_fixed_pz: " << edge.is_fixed_pz << std::endl;
            std::cout << "relative_pose_tb.fi: " << edge.relative_pose_tb.fi << std::endl;
            std::cout << "relative_pose_tb.ka: " << edge.relative_pose_tb.ka << std::endl;
            std::cout << "relative_pose_tb.om: " << edge.relative_pose_tb.om << std::endl;
            std::cout << "relative_pose_tb.px: " << edge.relative_pose_tb.px << std::endl;
            std::cout << "relative_pose_tb.py: " << edge.relative_pose_tb.py << std::endl;
            std::cout << "relative_pose_tb.pz: " << edge.relative_pose_tb.pz << std::endl;
            std::cout << "relative_pose_tb_weights.fi: " << edge.relative_pose_tb_weights.fi << std::endl;
            std::cout << "relative_pose_tb_weights.ka: " << edge.relative_pose_tb_weights.ka << std::endl;
            std::cout << "relative_pose_tb_weights.om: " << edge.relative_pose_tb_weights.om << std::endl;
            std::cout << "relative_pose_tb_weights.px: " << edge.relative_pose_tb_weights.px << std::endl;
            std::cout << "relative_pose_tb_weights.py: " << edge.relative_pose_tb_weights.py << std::endl;
            std::cout << "relative_pose_tb_weights.pz: " << edge.relative_pose_tb_weights.pz << std::endl;
        }

        std::cout << "------laz file names-----" << std::endl;
        for (const auto &fn : laz_file_names)
        {
            std::cout << "'" << fn << "'" << std::endl;
        }
#if WITH_GUI == 1
        for (const auto &gcp_json : data["ground_control_points"])
        {
            GroundControlPoint gcp;
            // gcp.name = gcp_json["name"];
            std::string name = gcp_json["name"];
            strcpy(gcp.name, name.c_str());
            gcp.x = gcp_json["x"];
            gcp.y = gcp_json["y"];
            gcp.z = gcp_json["z"];
            gcp.sigma_x = gcp_json["sigma_x"];
            gcp.sigma_y = gcp_json["sigma_y"];
            gcp.sigma_z = gcp_json["sigma_z"];
            gcp.lidar_height_above_ground = gcp_json["lidar_height_above_ground"];
            gcp.index_to_node_inner = gcp_json["index_to_node_inner"];
            gcp.index_to_node_outer = gcp_json["index_to_node_outer"];
            ground_control_points.gpcs.push_back(gcp);

            std::cout << "adding gcp[" << name << "]" << std::endl;
        };
#endif

#if WITH_GUI == 1
        for (const auto &cp_json : data["control_points"])
        {
            ControlPoint cp;
            // cp.name = cp_json["name"];
            std::string name = cp_json["name"];
            strcpy(cp.name, name.c_str());
            cp.x_source_local = cp_json["x_source_local"];
            cp.y_source_local = cp_json["y_source_local"];
            cp.z_source_local = cp_json["z_source_local"];
            cp.x_target_global = cp_json["x_target_global"];
            cp.y_target_global = cp_json["y_target_global"];
            cp.z_target_global = cp_json["z_target_global"];
            cp.sigma_x = cp_json["sigma_x"];
            cp.sigma_y = cp_json["sigma_y"];
            cp.sigma_z = cp_json["sigma_z"];
            cp.index_to_pose = cp_json["index_to_pose"];
            cp.is_z_0 = cp_json["is_z_0"];
            control_points.cps.push_back(cp);

            std::cout << "adding cp[" << name << "]" << std::endl;
        };
#endif

        // loading all data
        point_clouds_container.load_whu_tls(laz_file_names, is_decimate, bucket_x, bucket_y, bucket_z, calculate_offset);

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

        for (auto &pc : point_clouds_container.point_clouds)
        {
            pc.render_color[0] = render_color[0];
            pc.render_color[1] = render_color[1];
            pc.render_color[2] = render_color[2];
        }

        return true;
    }
    catch (std::exception &e)
    {
        std::cout << "cant load session: " << e.what() << std::endl;
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
    j["folder_name"] = point_clouds_container.folder_name;
    j["out_folder_name"] = point_clouds_container.out_folder_name;
    j["poses_file_name"] = poses_file_name;                 // point_clouds_container.poses_file_name;
    j["initial_poses_file_name"] = initial_poses_file_name; // point_clouds_container.initial_poses_file_name;
    j["out_poses_file_name"] = point_clouds_container.out_poses_file_name;
    j["ground_truth"] = is_ground_truth;
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
                    {"file_name", pc.file_name}};
                jlaz_file_names.push_back(jfn);
            }
        }
        else
        {
            nlohmann::json jfn{
                {"file_name", pc.file_name}};
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
