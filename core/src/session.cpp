#include <session.h>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;

bool Session::load(const std::string &file_name, bool is_decimate, double bucket_x, double bucket_y, double bucket_z, bool calculate_offset)
{
    std::cout << "loading file: '" << file_name << "'" << std::endl;
    // point_clouds_container.point_clouds.clear();

    double offset_x;
    double offset_y;
    double offset_z;
    std::string folder_name;
    std::string out_folder_name;
    std::string poses_file_name;
    std::string initial_poses_file_name;
    std::string out_poses_file_name;

    std::vector<ManualPoseGraphLoopClosure::Edge> loop_closure_edges;
    std::vector<std::string> laz_file_names;

    try
    {
        std::ifstream fs(file_name);
        if (!fs.good())
            return false;
        nlohmann::json data = nlohmann::json::parse(fs);
        fs.close();

        auto project_settings_json = data["Session Settings"];
        offset_x = project_settings_json["offset_x"];
        offset_y = project_settings_json["offset_y"];
        offset_z = project_settings_json["offset_z"];
        folder_name = project_settings_json["folder_name"];
        out_folder_name = project_settings_json["out_folder_name"];
        poses_file_name = project_settings_json["poses_file_name"];
        initial_poses_file_name = project_settings_json["initial_poses_file_name"];
        out_poses_file_name = project_settings_json["out_poses_file_name"];

        for (const auto &edge_json : data["loop_closure_edges"])
        {
            ManualPoseGraphLoopClosure::Edge edge;
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
            std::string fn = fn_json["file_name"];
            laz_file_names.push_back(fn);
        }

        std::cout << "loaded from json: " << file_name << std::endl;
        std::cout << "offset_x: " << offset_x << std::endl;
        std::cout << "offset_y: " << offset_y << std::endl;
        std::cout << "offset_z: " << offset_z << std::endl;
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

        // loading all data
        point_clouds_container.load_whu_tls(laz_file_names, is_decimate, bucket_x, bucket_y, bucket_z, calculate_offset);

        if (laz_file_names.size() > 0)
        {
            working_directory = fs::path(laz_file_names[0]).parent_path().string();

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

		manual_pose_graph_loop_closure.edges = loop_closure_edges;

		return true;
	}
    catch (std::exception &e)
    {
        std::cout << "cant load session: " << e.what() << std::endl;
        return false;
    }
}

#if 0
bool ShaftProject::load(const std::string& file_name, underground_mining::CommonData& common_data, bool only_sessions)
{
	//common_data.geo_point_clouds.clear();
	//common_data.rois.clear();
	//common_data.sessions.clear();
	//common_data.cross_geo.clear();
	//common_data.observations.clear();

	common_data.picked_points_geo.clear();
	common_data.solver_observations.clear();
	common_data.segment_geo_observations.clear();
	common_data.cross_geo.clear();
	common_data.rois.clear();
	common_data.geo_point_clouds.clear();
	common_data.sessions.clear();

	try {

		std::ifstream fs(file_name);
		if (!fs.good())return false;
		nlohmann::json data = nlohmann::json::parse(fs);
		fs.close();

		auto project_settings_json = data["Project Settings"];

		if (!only_sessions) {
			common_data.shift_x = project_settings_json["shift_x"];
			common_data.shift_y = project_settings_json["shift_y"];
			common_data.background_color.x = project_settings_json["background_color_x"];
			common_data.background_color.y = project_settings_json["background_color_y"];
			common_data.background_color.z = project_settings_json["background_color_z"];
			common_data.background_color.w = project_settings_json["background_color_w"];

			common_data.grid_color.x = project_settings_json["grid_color_x"];
			common_data.grid_color.y = project_settings_json["grid_color_y"];
			common_data.grid_color.z = project_settings_json["grid_color_z"];
			common_data.grid_color.w = project_settings_json["grid_color_w"];

			common_data.camera_ortho_xy_view_shift_x = project_settings_json["camera_ortho_xy_view_shift_x"];
			common_data.camera_ortho_xy_view_shift_y = project_settings_json["camera_ortho_xy_view_shift_y"];
			common_data.camera_ortho_xy_view_zoom = project_settings_json["camera_ortho_xy_view_zoom"];
			common_data.camera_ortho_xy_view_rotation_angle_deg = project_settings_json["camera_ortho_xy_view_rotation_angle_deg"];

			common_data.camera_ortho_z_view_shift_bottom_up = project_settings_json["camera_ortho_z_view_shift_bottom_up"];
			common_data.camera_ortho_z_view_zoom = project_settings_json["camera_ortho_z_view_zoom"];
			common_data.camera_ortho_z_view_rotation_angle_deg = project_settings_json["camera_ortho_z_view_rotation_angle_deg"];
			common_data.camera_mode_ortho_z_center_h = project_settings_json["camera_mode_ortho_z_center_h"];
			common_data.camera_ortho_z_view_shift_x = project_settings_json["camera_ortho_z_view_shift_x"];
			common_data.camera_ortho_z_view_shift_y = project_settings_json["camera_ortho_z_view_shift_y"];
			common_data.main_roi.translation.z() = common_data.camera_mode_ortho_z_center_h;


			common_data.main_roi.color.x = project_settings_json["main_roi_color_x"];
			common_data.main_roi.color.y = project_settings_json["main_roi_color_y"];
			common_data.main_roi.color.z = project_settings_json["main_roi_color_z"];
			common_data.main_roi.color.w = project_settings_json["main_roi_color_w"];
			common_data.main_roi.bounding_box_width = project_settings_json["main_roi_bounding_box_width"];
			common_data.main_roi.bounding_box_length = project_settings_json["main_roi_bounding_box_length"];
			common_data.main_roi.bounding_box_height = project_settings_json["main_roi_bounding_box_height"];
			common_data.main_roi.rotation_angle_deg = project_settings_json["main_roi_rotation_angle_deg"];
			common_data.main_roi.translation.x() = project_settings_json["main_roi_translation_x"];
			common_data.main_roi.translation.y() = project_settings_json["main_roi_translation_y"];
			common_data.main_roi.translation.z() = project_settings_json["main_roi_translation_z"];
			//common_data.main_roi.point_size = project_settings_json["main_roi_point_size"];
			common_data.main_roi.active = project_settings_json["main_roi_active"];
			common_data.main_roi.manipulate_with_mouse = project_settings_json["main_roi_manipulate_with_mouse"];
			common_data.main_roi.line_width = project_settings_json["main_roi_line_width"];

			for (const auto& gpc_json : project_settings_json["georeference_point_clouds"]) {
				underground_mining::GeoPointCloud gpc;
				gpc.centroid.x() = gpc_json["centroid"][0];
				gpc.centroid.y() = gpc_json["centroid"][1];
				gpc.centroid.z() = gpc_json["centroid"][2];
				gpc.color.x = gpc_json["colors"][0];
				gpc.color.y = gpc_json["colors"][1];
				gpc.color.z = gpc_json["colors"][2];
				gpc.color.w = gpc_json["colors"][3];
				gpc.decimation = gpc_json["decimation"];
				gpc.file_name = gpc_json["geo_pc_file"];
				gpc.visible = gpc_json["visible"];
				gpc.point_size = gpc_json["point_size"];

				auto tmp_gpc = gpc;
				GeorefercePointClouds gpcs;
				if (gpcs.load(gpc.file_name, gpc, common_data)) {
					gpc.color = tmp_gpc.color;
					gpc.decimation = tmp_gpc.decimation;
					gpc.visible = tmp_gpc.visible;

					std::cout << "file: " << gpc.file_name << " loaded" << std::endl;
					std::cout << "number of loaded points: " << gpc.points.size() << std::endl;

					common_data.geo_point_clouds.push_back(gpc);
				}
				else {
					std::cout << "problem with opening file: " << gpc.file_name << std::endl;
				}
			}

			for (const auto& roi_json : project_settings_json["rois"]) {
				underground_mining::ROI roi;
				roi.color.x = roi_json["colors"][0];
				roi.color.y = roi_json["colors"][1];
				roi.color.z = roi_json["colors"][2];
				roi.color.w = roi_json["colors"][3];
				roi.bounding_box_width = roi_json["bounding_box_width"];
				roi.bounding_box_length = roi_json["bounding_box_length"];
				roi.bounding_box_height = roi_json["bounding_box_height"];
				roi.rotation_angle_deg = roi_json["rotation_angle_deg"];
				roi.translation.x() = roi_json["translation"][0];
				roi.translation.y() = roi_json["translation"][1];
				roi.translation.z() = roi_json["translation"][2];
				//roi.point_size = roi_json["point_size"];
				roi.line_width = roi_json["line_width"];
				roi.active = roi_json["active"];
				roi.manipulate_with_mouse = roi_json["manipulate_with_mouse"];
				common_data.rois.push_back(roi);
			}
		}

		for (const auto& sessions_json : project_settings_json["sessions"]) {
			underground_mining::Session session;
			session.trajectory_file_name = sessions_json["trajectory_file_name"];
			session.color.x = sessions_json["color_x"];
			session.color.y = sessions_json["color_y"];
			session.color.z = sessions_json["color_z"];
			session.color.w = sessions_json["color_w"];
			session.line_width = sessions_json["line_width"];

			if (common_data.load_trajectories_from_json) {
				session.trajectory.clear();

				for (const auto& trajectorynodes_json : sessions_json["trajectory"]) {
					underground_mining::TrajectoryNode trajectory_node;
					trajectory_node.is_fixed_px = trajectorynodes_json["is_fixed_px"];
					trajectory_node.is_fixed_py = trajectorynodes_json["is_fixed_py"];
					trajectory_node.is_fixed_pz = trajectorynodes_json["is_fixed_pz"];
					trajectory_node.is_fixed_om = trajectorynodes_json["is_fixed_om"];
					trajectory_node.is_fixed_fi = trajectorynodes_json["is_fixed_fi"];
					trajectory_node.is_fixed_ka = trajectorynodes_json["is_fixed_ka"];
					trajectory_node.m_pose(0, 0) = trajectorynodes_json["m_pose"][0];
					trajectory_node.m_pose(0, 1) = trajectorynodes_json["m_pose"][1];
					trajectory_node.m_pose(0, 2) = trajectorynodes_json["m_pose"][2];
					trajectory_node.m_pose(0, 3) = trajectorynodes_json["m_pose"][3];

					trajectory_node.m_pose(1, 0) = trajectorynodes_json["m_pose"][4];
					trajectory_node.m_pose(1, 1) = trajectorynodes_json["m_pose"][5];
					trajectory_node.m_pose(1, 2) = trajectorynodes_json["m_pose"][6];
					trajectory_node.m_pose(1, 3) = trajectorynodes_json["m_pose"][7];

					trajectory_node.m_pose(2, 0) = trajectorynodes_json["m_pose"][8];
					trajectory_node.m_pose(2, 1) = trajectorynodes_json["m_pose"][9];
					trajectory_node.m_pose(2, 2) = trajectorynodes_json["m_pose"][10];
					trajectory_node.m_pose(2, 3) = trajectorynodes_json["m_pose"][11];

					trajectory_node.m_pose(3, 0) = trajectorynodes_json["m_pose"][12];
					trajectory_node.m_pose(3, 1) = trajectorynodes_json["m_pose"][13];
					trajectory_node.m_pose(3, 2) = trajectorynodes_json["m_pose"][14];
					trajectory_node.m_pose(3, 3) = trajectorynodes_json["m_pose"][15];

					trajectory_node.timestamp = trajectorynodes_json["timestamp"];
					trajectory_node.w_tx = trajectorynodes_json["w_tx"];
					trajectory_node.w_ty = trajectorynodes_json["w_ty"];
					trajectory_node.w_tz = trajectorynodes_json["w_tz"];
					trajectory_node.w_om = trajectorynodes_json["w_om"];
					trajectory_node.w_fi = trajectorynodes_json["w_fi"];
					trajectory_node.w_ka = trajectorynodes_json["w_ka"];

					session.trajectory.push_back(trajectory_node);
				}

				session.trajectory_motion_model.clear();

				for (const auto& trajectorynodes_json : sessions_json["trajectory_motion_model"]) {
					underground_mining::TrajectoryNode trajectory_node;
					trajectory_node.is_fixed_px = trajectorynodes_json["is_fixed_px"];
					trajectory_node.is_fixed_py = trajectorynodes_json["is_fixed_py"];
					trajectory_node.is_fixed_pz = trajectorynodes_json["is_fixed_pz"];
					trajectory_node.is_fixed_om = trajectorynodes_json["is_fixed_om"];
					trajectory_node.is_fixed_fi = trajectorynodes_json["is_fixed_fi"];
					trajectory_node.is_fixed_ka = trajectorynodes_json["is_fixed_ka"];
					trajectory_node.m_pose(0, 0) = trajectorynodes_json["m_pose"][0];
					trajectory_node.m_pose(0, 1) = trajectorynodes_json["m_pose"][1];
					trajectory_node.m_pose(0, 2) = trajectorynodes_json["m_pose"][2];
					trajectory_node.m_pose(0, 3) = trajectorynodes_json["m_pose"][3];

					trajectory_node.m_pose(1, 0) = trajectorynodes_json["m_pose"][4];
					trajectory_node.m_pose(1, 1) = trajectorynodes_json["m_pose"][5];
					trajectory_node.m_pose(1, 2) = trajectorynodes_json["m_pose"][6];
					trajectory_node.m_pose(1, 3) = trajectorynodes_json["m_pose"][7];

					trajectory_node.m_pose(2, 0) = trajectorynodes_json["m_pose"][8];
					trajectory_node.m_pose(2, 1) = trajectorynodes_json["m_pose"][9];
					trajectory_node.m_pose(2, 2) = trajectorynodes_json["m_pose"][10];
					trajectory_node.m_pose(2, 3) = trajectorynodes_json["m_pose"][11];

					trajectory_node.m_pose(3, 0) = trajectorynodes_json["m_pose"][12];
					trajectory_node.m_pose(3, 1) = trajectorynodes_json["m_pose"][13];
					trajectory_node.m_pose(3, 2) = trajectorynodes_json["m_pose"][14];
					trajectory_node.m_pose(3, 3) = trajectorynodes_json["m_pose"][15];

					trajectory_node.timestamp = trajectorynodes_json["timestamp"];
					trajectory_node.w_tx = trajectorynodes_json["w_tx"];
					trajectory_node.w_ty = trajectorynodes_json["w_ty"];
					trajectory_node.w_tz = trajectorynodes_json["w_tz"];
					trajectory_node.w_om = trajectorynodes_json["w_om"];
					trajectory_node.w_fi = trajectorynodes_json["w_fi"];
					trajectory_node.w_ka = trajectorynodes_json["w_ka"];

					session.trajectory_motion_model.push_back(trajectory_node);
				}
			}

			//session.m_initial_pose(0, 0) = sessions_json["m_initial_pose"][0];
			//session.m_initial_pose(0, 1) = sessions_json["m_initial_pose"][1];
			//session.m_initial_pose(0, 2) = sessions_json["m_initial_pose"][2];
			//session.m_initial_pose(0, 3) = sessions_json["m_initial_pose"][3];

			//session.m_initial_pose(1, 0) = sessions_json["m_initial_pose"][4];
			//session.m_initial_pose(1, 1) = sessions_json["m_initial_pose"][5];
			//session.m_initial_pose(1, 2) = sessions_json["m_initial_pose"][6];
			//session.m_initial_pose(1, 3) = sessions_json["m_initial_pose"][7];

			//session.m_initial_pose(2, 0) = sessions_json["m_initial_pose"][8];
			//session.m_initial_pose(2, 1) = sessions_json["m_initial_pose"][9];
			//session.m_initial_pose(2, 2) = sessions_json["m_initial_pose"][10];
			//session.m_initial_pose(2, 3) = sessions_json["m_initial_pose"][11];

			//session.m_initial_pose(3, 0) = sessions_json["m_initial_pose"][12];
			//session.m_initial_pose(3, 1) = sessions_json["m_initial_pose"][13];
			//session.m_initial_pose(3, 2) = sessions_json["m_initial_pose"][14];
			//session.m_initial_pose(3, 3) = sessions_json["m_initial_pose"][15];

			session.line_decimation = sessions_json["line_decimation"];


			for (const auto& lidarstream_json : sessions_json["lidarstreams"]) {
				underground_mining::LidarStream lidar_stream;
				lidar_stream.chunks_file_name = lidarstream_json["chunks_file_name"];
				lidar_stream.initial_calibration_file_name = lidarstream_json["initial_calibration_file_name"];
				lidar_stream.points_size = lidarstream_json["points_size"];
				lidar_stream.points_decimation = lidarstream_json["points_decimation"];
				lidar_stream.color.x = lidarstream_json["color_x"];
				lidar_stream.color.y = lidarstream_json["color_y"];
				lidar_stream.color.z = lidarstream_json["color_z"];
				lidar_stream.color.w = lidarstream_json["color_w"];

				if (common_data.load_calibrations_from_json) {
					for (const auto& calibration_json : lidarstream_json["calibrations"]) {
						underground_mining::CalibrationNode cn;

						cn.timestamp = calibration_json["timestamp"];
						cn.scale = calibration_json["scale"];
						cn.m_pose(0, 0) = calibration_json["m_pose"][0];
						cn.m_pose(0, 1) = calibration_json["m_pose"][1];
						cn.m_pose(0, 2) = calibration_json["m_pose"][2];
						cn.m_pose(0, 3) = calibration_json["m_pose"][3];

						cn.m_pose(1, 0) = calibration_json["m_pose"][4];
						cn.m_pose(1, 1) = calibration_json["m_pose"][5];
						cn.m_pose(1, 2) = calibration_json["m_pose"][6];
						cn.m_pose(1, 3) = calibration_json["m_pose"][7];

						cn.m_pose(2, 0) = calibration_json["m_pose"][8];
						cn.m_pose(2, 1) = calibration_json["m_pose"][9];
						cn.m_pose(2, 2) = calibration_json["m_pose"][10];
						cn.m_pose(2, 3) = calibration_json["m_pose"][11];

						cn.m_pose(3, 0) = calibration_json["m_pose"][12];
						cn.m_pose(3, 1) = calibration_json["m_pose"][13];
						cn.m_pose(3, 2) = calibration_json["m_pose"][14];
						cn.m_pose(3, 3) = calibration_json["m_pose"][15];

						cn.uncertainty_sigma_x_m = calibration_json["uncertainty_sigma_x_m"];
						cn.uncertainty_sigma_y_m = calibration_json["uncertainty_sigma_y_m"];
						cn.uncertainty_sigma_z_m = calibration_json["uncertainty_sigma_z_m"];
						cn.uncertainty_sigma_om_deg = calibration_json["uncertainty_sigma_om_deg"];
						cn.uncertainty_sigma_fi_deg = calibration_json["uncertainty_sigma_fi_deg"];
						cn.uncertainty_sigma_ka_deg = calibration_json["uncertainty_sigma_ka_deg"];

						lidar_stream.calibrations.push_back(cn);
					}
				}

				session.lidar_streams.push_back(lidar_stream);
			}


			//for (const auto& sessions_json : project_settings_json["sessions"]) {
			for (const auto& calibration_observations_json : sessions_json["calibration_observations"]) {
				underground_mining::Observation obs;
				obs.information_matrix(0, 0) = calibration_observations_json["information_matrix"][0];
				obs.information_matrix(0, 1) = calibration_observations_json["information_matrix"][1];
				obs.information_matrix(0, 2) = calibration_observations_json["information_matrix"][2];
				obs.information_matrix(1, 0) = calibration_observations_json["information_matrix"][3];
				obs.information_matrix(1, 1) = calibration_observations_json["information_matrix"][4];
				obs.information_matrix(1, 2) = calibration_observations_json["information_matrix"][5];
				obs.information_matrix(2, 0) = calibration_observations_json["information_matrix"][6];
				obs.information_matrix(2, 1) = calibration_observations_json["information_matrix"][7];
				obs.information_matrix(2, 2) = calibration_observations_json["information_matrix"][8];

				obs.isX = calibration_observations_json["isX"];
				obs.isY = calibration_observations_json["isY"];
				obs.isZ = calibration_observations_json["isZ"];

				obs.line_color.x = calibration_observations_json["line_color"][0];
				obs.line_color.y = calibration_observations_json["line_color"][1];
				obs.line_color.z = calibration_observations_json["line_color"][2];
				obs.line_color.w = calibration_observations_json["line_color"][3];

				obs.line_width = calibration_observations_json["line_width"];
				obs.point_size = calibration_observations_json["point_size"];
				obs.visible = calibration_observations_json["visible"];

				for (const auto& picked_points_json : calibration_observations_json["picked_points"]) {
					underground_mining::PickedPoint pp;
					pp.chunks_file_name = picked_points_json["chunks_file_name"];
					pp.color.x = picked_points_json["color"][0];
					pp.color.y = picked_points_json["color"][1];
					pp.color.z = picked_points_json["color"][2];
					pp.color.w = picked_points_json["color"][3];

					pp.coordinates.x() = picked_points_json["coordinates"][0];
					pp.coordinates.y() = picked_points_json["coordinates"][1];
					pp.coordinates.z() = picked_points_json["coordinates"][2];

					pp.is_geo_point = picked_points_json["is_geo_point"];
					pp.timestamp = picked_points_json["timestamp"];
					pp.trajectory_file_name = picked_points_json["trajectory_file_name"];
					obs.picked_points.push_back(pp);
				}

				session.calibration_observations.push_back(obs);
			}

			std::vector<underground_mining::GeoPoseSegment> geo_segments;
			for (const auto& geo_segments_json : sessions_json["geo_segments"]) {
				underground_mining::GeoPoseSegment geo_segment;
				geo_segment.mean_pose_translation.x() = geo_segments_json["mean_pose_translation"][0];
				geo_segment.mean_pose_translation.y() = geo_segments_json["mean_pose_translation"][1];
				geo_segment.mean_pose_translation.z() = geo_segments_json["mean_pose_translation"][2];
				geo_segment.rms = geo_segments_json["rms"];
				geo_segment.visible = geo_segments_json["visible"];
			
				for (const auto& geo_pose_json : geo_segments_json["geo_poses"]) {
					underground_mining::GeoPose geo_pose;
					geo_pose.is_x = geo_pose_json["is_x"];
					geo_pose.is_y = geo_pose_json["is_y"];
					geo_pose.is_z = geo_pose_json["is_z"];
					geo_pose.is_om = geo_pose_json["is_om"];
					geo_pose.is_fi = geo_pose_json["is_fi"];
					geo_pose.is_ka = geo_pose_json["is_ka"];
					geo_pose.w_x = geo_pose_json["w_x"];
					geo_pose.w_y = geo_pose_json["w_y"];
					geo_pose.w_z = geo_pose_json["w_z"];
					geo_pose.w_om = geo_pose_json["w_om"];
					geo_pose.w_fi = geo_pose_json["w_fi"];
					geo_pose.w_ka = geo_pose_json["w_ka"];
					geo_pose.timestamp = geo_pose_json["timestamp"];
					geo_pose.m_pose(0, 0) = geo_pose_json["m_pose"][0];
					geo_pose.m_pose(0, 1) = geo_pose_json["m_pose"][1];
					geo_pose.m_pose(0, 2) = geo_pose_json["m_pose"][2];
					geo_pose.m_pose(0, 3) = geo_pose_json["m_pose"][3];

					geo_pose.m_pose(1, 0) = geo_pose_json["m_pose"][4];
					geo_pose.m_pose(1, 1) = geo_pose_json["m_pose"][5];
					geo_pose.m_pose(1, 2) = geo_pose_json["m_pose"][6];
					geo_pose.m_pose(1, 3) = geo_pose_json["m_pose"][7];

					geo_pose.m_pose(2, 0) = geo_pose_json["m_pose"][8];
					geo_pose.m_pose(2, 1) = geo_pose_json["m_pose"][9];
					geo_pose.m_pose(2, 2) = geo_pose_json["m_pose"][10];
					geo_pose.m_pose(2, 3) = geo_pose_json["m_pose"][11];

					geo_pose.m_pose(3, 0) = geo_pose_json["m_pose"][12];
					geo_pose.m_pose(3, 1) = geo_pose_json["m_pose"][13];
					geo_pose.m_pose(3, 2) = geo_pose_json["m_pose"][14];
					geo_pose.m_pose(3, 3) = geo_pose_json["m_pose"][15];

					geo_segment.geo_poses.push_back(geo_pose);
				}


				geo_segments.push_back(geo_segment);

				nlohmann::json json_geo_segments;
				for (const auto& obs : session.geo_segments) {
					nlohmann::json jgs{
						{"mean_pose_translation", {obs.mean_pose_translation.x(), obs.mean_pose_translation.y(), obs.mean_pose_translation.z()}},
					};

					nlohmann::json jgeo_poses;
					for (const auto& geo_pose : obs.geo_poses) {
						nlohmann::json jgeo_pose{
							{"w_x", geo_pose.w_x},
							{"w_y", geo_pose.w_y},
							{"w_z", geo_pose.w_z},
							{"w_om", geo_pose.w_om},
							{"w_fi", geo_pose.w_fi},
							{"w_ka", geo_pose.w_ka},
							{"is_x", geo_pose.is_x},
							{"is_y", geo_pose.is_y},
							{"is_z", geo_pose.is_z},
							{"is_om", geo_pose.is_om},
							{"is_fi", geo_pose.is_fi},
							{"is_ka", geo_pose.is_ka},
							{"m_pose", {geo_pose.m_pose(0,0),  geo_pose.m_pose(0,1), geo_pose.m_pose(0,2), geo_pose.m_pose(0,3),
										geo_pose.m_pose(1,0),  geo_pose.m_pose(1,1), geo_pose.m_pose(1,2), geo_pose.m_pose(1,3),
										geo_pose.m_pose(2,0),  geo_pose.m_pose(2,1), geo_pose.m_pose(2,2), geo_pose.m_pose(2,3),
										geo_pose.m_pose(3,0),  geo_pose.m_pose(3,1), geo_pose.m_pose(3,2), geo_pose.m_pose(3,3)}},
							{"timestamp", geo_pose.timestamp}
						};
						jgeo_poses.push_back(jgeo_pose);
					}
					jgs["geo_poses"] = jgeo_poses;
					json_geo_segments.push_back(jgs);
				}
				jsession["geo_segments"] = json_geo_segments;

			}
			session.geo_segments = geo_segments;
			common_data.sessions.push_back(session);
		}
		
		for (const auto& solver_observations_json : project_settings_json["solver_observations"]) {
			underground_mining::Observation obs;
			obs.information_matrix(0, 0) = solver_observations_json["information_matrix"][0];
			obs.information_matrix(0, 1) = solver_observations_json["information_matrix"][1];
			obs.information_matrix(0, 2) = solver_observations_json["information_matrix"][2];
			obs.information_matrix(1, 0) = solver_observations_json["information_matrix"][3];
			obs.information_matrix(1, 1) = solver_observations_json["information_matrix"][4];
			obs.information_matrix(1, 2) = solver_observations_json["information_matrix"][5];
			obs.information_matrix(2, 0) = solver_observations_json["information_matrix"][6];
			obs.information_matrix(2, 1) = solver_observations_json["information_matrix"][7];
			obs.information_matrix(2, 2) = solver_observations_json["information_matrix"][8];

			obs.isX = solver_observations_json["isX"];
			obs.isY = solver_observations_json["isY"];
			obs.isZ = solver_observations_json["isZ"];

			obs.line_color.x = solver_observations_json["line_color"][0];
			obs.line_color.y = solver_observations_json["line_color"][1];
			obs.line_color.z = solver_observations_json["line_color"][2];
			obs.line_color.w = solver_observations_json["line_color"][3];

			obs.line_width = solver_observations_json["line_width"];
			obs.point_size = solver_observations_json["point_size"];
			obs.visible = solver_observations_json["visible"];

			for (const auto& picked_points_json : solver_observations_json["picked_points"]) {
				underground_mining::PickedPoint pp;
				pp.chunks_file_name = picked_points_json["chunks_file_name"];
				pp.color.x = picked_points_json["color"][0];
				pp.color.y = picked_points_json["color"][1];
				pp.color.z = picked_points_json["color"][2];
				pp.color.w = picked_points_json["color"][3];

				pp.coordinates.x() = picked_points_json["coordinates"][0];
				pp.coordinates.y() = picked_points_json["coordinates"][1];
				pp.coordinates.z() = picked_points_json["coordinates"][2];

				pp.is_geo_point = picked_points_json["is_geo_point"];
				pp.timestamp = picked_points_json["timestamp"];
				pp.trajectory_file_name = picked_points_json["trajectory_file_name"];
				obs.picked_points.push_back(pp);
			}
			std::cout << "solver_observation loaded" << std::endl;
			common_data.solver_observations.push_back(obs);
		}

		//std::vector<underground_mining::GeoPoseSegment> geo_segments;

		std::vector<underground_mining::CrossGeo> cross_geo;
		for (const auto& cross_geo_json : project_settings_json["cross_geo"]) {

			underground_mining::CrossGeo geo_segment;
			geo_segment.color.x = cross_geo_json["color"][0];
			geo_segment.color.y = cross_geo_json["color"][1];
			geo_segment.color.z = cross_geo_json["color"][2];
			geo_segment.color.w = cross_geo_json["color"][3];
			geo_segment.id = cross_geo_json["id"];
			geo_segment.visible = cross_geo_json["visible"];

			for (const auto& points_geo_json : cross_geo_json["points_geo"]) {
				Eigen::Vector3d p;
				p.x() = points_geo_json["x"];
				p.y() = points_geo_json["y"];
				p.z() = points_geo_json["z"];
				geo_segment.points_geo.push_back(p);
			}
			cross_geo.push_back(geo_segment);
		}
		common_data.cross_geo = cross_geo;


		for (const auto& observation_json : project_settings_json["observations"]) {
			//underground_mining::Session session;
			//session.trajectory_file_name = sessions_json["trajectory_file_name"];
			//session.color.x = sessions_json["color_x"];
			//session.color.y = sessions_json["color_y"];
			//session.color.z = sessions_json["color_z"];
			//session.color.w = sessions_json["color_w"];
			underground_mining::Observation observation;
			observation.geo_point.x() = observation_json["geo_point"][0];
			observation.geo_point.y() = observation_json["geo_point"][1];
			observation.geo_point.z() = observation_json["geo_point"][2];
			observation.geo_point_information_matrix(0, 0) = observation_json["geo_point_information_matrix"][0];
			observation.geo_point_information_matrix(0, 1) = observation_json["geo_point_information_matrix"][1];
			observation.geo_point_information_matrix(0, 2) = observation_json["geo_point_information_matrix"][2];
			observation.geo_point_information_matrix(1, 0) = observation_json["geo_point_information_matrix"][3];
			observation.geo_point_information_matrix(1, 1) = observation_json["geo_point_information_matrix"][4];
			observation.geo_point_information_matrix(1, 2) = observation_json["geo_point_information_matrix"][5];
			observation.geo_point_information_matrix(2, 0) = observation_json["geo_point_information_matrix"][6];
			observation.geo_point_information_matrix(2, 1) = observation_json["geo_point_information_matrix"][7];
			observation.geo_point_information_matrix(2, 2) = observation_json["geo_point_information_matrix"][8];
			observation.is_geo_point = observation_json["is_geo_point"];
			observation.line_color.x = observation_json["line_color"][0];
			observation.line_color.y = observation_json["line_color"][1];
			observation.line_color.z = observation_json["line_color"][2];
			observation.line_color.w = observation_json["line_color"][3];
			observation.line_width = observation_json["line_width"];
			observation.point_size = observation_json["point_size"];
			observation.visible = observation_json["visible"];
			observation.isX = observation_json["isX"];
			observation.isY = observation_json["isY"];
			observation.isZ = observation_json["isZ"];


			for (const auto& lidar_observation_json : observation_json["lidar_observations"]) {

				underground_mining::LidarObservation lidar_observation;
				lidar_observation.chunks_file_name = lidar_observation_json["chunks_file_name"];
				lidar_observation.coordinates_local.x() = lidar_observation_json["coordinates_local"][0];
				lidar_observation.coordinates_local.y() = lidar_observation_json["coordinates_local"][1];
				lidar_observation.coordinates_local.z() = lidar_observation_json["coordinates_local"][2];
				lidar_observation.information_matrix(0, 0) = lidar_observation_json["information_matrix"][0];
				lidar_observation.information_matrix(0, 1) = lidar_observation_json["information_matrix"][1];
				lidar_observation.information_matrix(0, 2) = lidar_observation_json["information_matrix"][2];
				lidar_observation.information_matrix(1, 0) = lidar_observation_json["information_matrix"][3];
				lidar_observation.information_matrix(1, 1) = lidar_observation_json["information_matrix"][4];
				lidar_observation.information_matrix(1, 2) = lidar_observation_json["information_matrix"][5];
				lidar_observation.information_matrix(2, 0) = lidar_observation_json["information_matrix"][6];
				lidar_observation.information_matrix(2, 1) = lidar_observation_json["information_matrix"][7];
				lidar_observation.information_matrix(2, 1) = lidar_observation_json["information_matrix"][8];
				lidar_observation.timestamp = lidar_observation_json["timestamp"];
				lidar_observation.trajectory_file_name = lidar_observation_json["trajectory_file_name"];

				observation.lidar_observations.push_back(lidar_observation);
			}
			common_data.observations.push_back(observation);
		}

		for (auto& session : common_data.sessions) {
			for (auto& lidar : session.lidar_streams) {
				if (lidar.calibrations.size() == 0) {
					std::cout << "initializing calibration from file: '" << lidar.initial_calibration_file_name << "' and taking first timestamp from trajectory: '" <<
						session.trajectory_file_name << "'" << std::endl;


					lci_state_chest trajectory;
					if (trajectory.read_from_binary_file(session.trajectory_file_name)) {
						underground_mining::CalibrationNode calibration;

						if (trajectory.data.size() > 0) {
							calibration.timestamp = trajectory.data[0].timestamp;
						}
						else {
							std::cout << "PROBLEM with trajectory --> size == 0: '" << session.trajectory_file_name << "' so setting timestamp to 0.0" << std::endl;
						}


						std::vector<underground_mining::Calibration> calib;
						if (load_calibration(lidar.initial_calibration_file_name, calib)) {
							calibration.scale = calib[0].scale;
							calibration.m_pose = affine_matrix_from_pose_tait_bryan(calib[0].pose.p.x, calib[0].pose.p.y, calib[0].pose.p.z,
								calib[0].pose.o.x_angle_rad, calib[0].pose.o.y_angle_rad, calib[0].pose.o.z_angle_rad);
							lidar.calibrations.push_back(calibration);
						}
						else {
							std::cout << "unknown format calibration file: " << lidar.initial_calibration_file_name << std::endl;
						}
					}
					else {
						std::cout << "PROBLEM with opening file: '" << session.trajectory_file_name << "' so setting timestamp to 0.0" << std::endl;
					}

				}
				else {
					std::cout << "calibrations loaded from json" << std::endl;
					for (auto& c : lidar.calibrations) {
						std::cout << "----" << std::endl;
						std::cout << "timestamp: " << c.timestamp << " scale: " << c.scale << std::endl;
						std::cout << c.m_pose.matrix() << std::endl;
					}
				}
			}
		}


	}
	catch (std::exception& e) {
		std::cout << "cant load project: " << e.what() << std::endl;
		return false;
	}

	std::cout << "load json done" << std::endl;
	return true;
}
#endif

bool Session::save(const std::string &file_name)
{
    std::cout << "saving file: '" << file_name << "'" << std::endl;

    nlohmann::json jj;
    nlohmann::json j;

    j["offset_x"] = point_clouds_container.offset.x();
    j["offset_y"] = point_clouds_container.offset.y();
    j["offset_z"] = point_clouds_container.offset.z();
    j["folder_name"] = point_clouds_container.folder_name;
    j["out_folder_name"] = point_clouds_container.out_folder_name;
    j["poses_file_name"] = point_clouds_container.poses_file_name;
    j["initial_poses_file_name"] = point_clouds_container.initial_poses_file_name;
    j["out_poses_file_name"] = point_clouds_container.out_poses_file_name;

    jj["Session Settings"] = j;

    nlohmann::json jloop_closure_edges;
    for (const auto &edge : manual_pose_graph_loop_closure.edges)
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
    jj["loop_closure_edges"] = jloop_closure_edges;

    nlohmann::json jlaz_file_names;
    for (const auto &pc : point_clouds_container.point_clouds)
    {
        nlohmann::json jfn{
            {"file_name", pc.file_name}};
        jlaz_file_names.push_back(jfn);
    }
    jj["laz_file_names"] = jlaz_file_names;

    std::ofstream fs(file_name);
    if (!fs.good())
        return false;
    fs << jj.dump(2);
    fs.close();
    return true;
}
