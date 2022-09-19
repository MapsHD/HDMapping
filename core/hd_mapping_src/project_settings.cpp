#include <project_settings.h>

#include <nlohmann/json.hpp>

#include <portable-file-dialogs.h>

#include <iostream>
#include <fstream>
#include <thread>

#include <GL/freeglut.h>

#include <transformations.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian.h>
#include <m_estimators.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>

void ProjectSettings::imgui(OdoWithGnssFusion& odo_with_gnss_fusion, std::vector<LAZSector>& sectors, std::vector<ROIwithConstraints>& rois_with_constraints,
	CommonData &common_data )
{
	ImGui::Begin("project settings");
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

	ImGui::Checkbox("regions of interes", &common_data.roi_exorter);
	if (common_data.roi_exorter) {
		common_data.odo_with_gnss_fusion = false;
		common_data.single_trajectory_viewer = false;
	}
	
	ImGui::Checkbox("laz manager", &common_data.laz_wrapper);
	if (common_data.laz_wrapper) {
		common_data.odo_with_gnss_fusion = false;
		common_data.single_trajectory_viewer = false;
	}

	ImGui::Checkbox("single trajectory viewer", &common_data.single_trajectory_viewer);
	if (common_data.single_trajectory_viewer) {
		common_data.odo_with_gnss_fusion = false;
	}
	
	ImGui::Checkbox("odo with gnss fusion", &common_data.odo_with_gnss_fusion);


	if (common_data.odo_with_gnss_fusion) {
		common_data.roi_exorter = false;
		common_data.laz_wrapper = false;
		common_data.single_trajectory_viewer = false;
	}
	if (common_data.single_trajectory_viewer) {
		common_data.roi_exorter = false;
		common_data.laz_wrapper = false;
		common_data.odo_with_gnss_fusion = false;
	}


	if (ImGui::Button("load project")) {
		static std::shared_ptr<pfd::open_file> open_file;
		std::string input_file_name = "";
		ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
		const auto t = [&]() {
			auto sel = pfd::open_file("Load project settings", "C:\\").result();
			for (int i = 0; i < sel.size(); i++)
			{
				input_file_name = sel[i];
				std::cout << "project settings file: '" << input_file_name << "'" << std::endl;
			}
		};
		std::thread t1(t);
		t1.join();

		if (input_file_name.size() > 0) {
			load(std::filesystem::path(input_file_name).string(), sectors, rois_with_constraints, common_data);
			odo_with_gnss_fusion.update_shift(common_data.shift_x, common_data.shift_y, common_data);
		}
	}
	ImGui::SameLine();
	if (ImGui::Button("save project")) {
		static std::shared_ptr<pfd::save_file> save_file;
		std::string output_file_name = "";
		ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
		const auto t = [&]() {
			auto sel = pfd::save_file("Choose folder", "C:\\").result();
			output_file_name = sel;
			std::cout << "project settings file to save: '" << output_file_name << "'" << std::endl;
		};
		std::thread t1(t);
		t1.join();

		if (output_file_name.size() > 0) {
			save(std::filesystem::path(output_file_name).string(), sectors, rois_with_constraints, common_data);
		}
	}

	ImGui::ColorEdit3("clear color", (float*)&clear_color);

	float old_shift_x = common_data.shift_x;
	float old_shift_y = common_data.shift_y;

	ImGui::InputFloat("shift_x", &common_data.shift_x, 1.0f, 100.0f, "%.3f", 0);
	ImGui::InputFloat("shift_y", &common_data.shift_y, 1.0f, 100.0f, "%.3f", 0);

	if (old_shift_x != common_data.shift_x || old_shift_y != common_data.shift_y) {
		odo_with_gnss_fusion.update_shift(common_data.shift_x, common_data.shift_y, common_data);
	}

	/*if (ImGui::Button("set project main folder")) {
		static std::shared_ptr<pfd::select_folder> folder;
		std::string folder_name = "";
		ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)folder);
		const auto t = [&]() {
			auto sel = pfd::select_folder("Choose folder", "C:\\").result();
			folder_name = sel;
			std::cout << "project_main_folder: '" << folder_name << "'" << std::endl;
		};
		std::thread t1(t);
		t1.join();

		if (folder_name.size() > 0) {
			project_main_folder = folder_name;
		}
	}
	if (project_main_folder.size() > 0) {
		ImGui::SameLine();
		ImGui::Text(std::string("project_main_folder: " + project_main_folder).c_str() );
	}*/

	if (ImGui::Button("add trajectory")) {
		static std::shared_ptr<pfd::open_file> open_file;
		std::string input_file_name = "";
		ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
		const auto t = [&]() {
			auto sel = pfd::open_file("Choose trajectory", "C:\\").result();
			for (int i = 0; i < sel.size(); i++)
			{
				input_file_name = sel[i];
				std::cout << "trajectory file: '" << input_file_name << "'" << std::endl;
			}
		};
		std::thread t1(t);
		t1.join();

		static std::shared_ptr<pfd::open_file> open_file_mm;
		std::string input_file_name_mm = "";
		ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file_mm);
		const auto tt = [&]() {
			auto sel = pfd::open_file("Choose motion model trajectory", "C:\\").result();
			for (int i = 0; i < sel.size(); i++)
			{
				input_file_name_mm = sel[i];
				std::cout << "motion model file: '" << input_file_name_mm << "'" << std::endl;
			}
		};
		std::thread t2(tt);
		t2.join();

		if (input_file_name.size() > 0) {
			//create motion model trajectory

			add_trajectory(input_file_name, input_file_name_mm, float(rand()%255)/256.0, float(rand() % 255) / 256.0, float(rand() % 255) / 256.0);
		}
	}

	for (size_t i = 0; i < trajectories.size(); i++) {
		ImGui::Text(std::string(trajectories[i].trajectory_file).c_str());
		ImGui::SameLine();
		ImGui::Checkbox(("[" + std::to_string(i) + "] visible").c_str(), &trajectories[i].visible);
		ImGui::SameLine();
				
		ImGui::PushButtonRepeat(true);
		float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
		if (ImGui::ArrowButton(("[" + std::to_string(i) + "] ##left").c_str(), ImGuiDir_Left)) { (trajectories[i].line_width)--; }
		ImGui::SameLine(0.0f, spacing);
		if (ImGui::ArrowButton(("[" + std::to_string(i) + "] ##right").c_str(), ImGuiDir_Right)) { (trajectories[i].line_width)++; }
		ImGui::PopButtonRepeat();
		ImGui::SameLine();
		ImGui::Text("line width %d", trajectories[i].line_width);
		if (trajectories[i].line_width < 1) trajectories[i].line_width = 1;

		ImVec4 color;
		color.x = trajectories[i].color_x;
		color.y = trajectories[i].color_y;
		color.z = trajectories[i].color_z;
		color.w = 1;
		ImGui::ColorEdit3(("[" +  std::to_string(i) + "]:trajectory color").c_str(), (float*)&color);
		trajectories[i].color_x = color.x;
		trajectories[i].color_y = color.y;
		trajectories[i].color_z = color.z;

		//ImGui::SameLine();
		

		
	}
	ImGui::Text(("total_length: " + std::to_string(int(total_length)) + "[m]").c_str());
	
	if (ImGui::Button("pose graph slam")) {
		pose_graph_slam(rois_with_constraints);
	}

	ImGui::End();
}

bool ProjectSettings::save(const std::string& file_name, std::vector<LAZSector>& sectors,
	const std::vector<ROIwithConstraints>& rois_with_constraints, const CommonData& common_data)
{
	nlohmann::json jj;
	nlohmann::json j;

	j["shift_x"] = common_data.shift_x;
	j["shift_y"] = common_data.shift_y;

	j["clear_color_x"] = clear_color.x;
	j["clear_color_y"] = clear_color.y;
	j["clear_color_z"] = clear_color.z;
	j["clear_color_w"] = clear_color.w;

	//j["project_main_folder"] = project_main_folder;
	
	nlohmann::json json_trajectories;
	for (const auto& t : trajectories) {

		//auto new_path = std::filesystem::path(t.trajectory_file).parent_path();
		//auto file_name = std::filesystem::path(t.trajectory_file).stem();

		//new_path /= file_name;
		//new_path += "_motion_model.csv";

		nlohmann::json trajectory{
					{"trajectory_file", t.trajectory_file}, {"trajectory_motion_model_file", t.motion_model_file_name}, {"colors",{t.color_x, t.color_y, t.color_z} } };
		json_trajectories.push_back(trajectory);
	}
	
	j["Trajectories"] = json_trajectories;
	

	nlohmann::json json_laz_sectors;
	for (const auto& s : sectors) {
		nlohmann::json json_sector{ {"laz_file", s.file_name}, {"local bounding box", {s.min_x, s.max_x, s.min_y, s.max_y} } };
		json_laz_sectors.push_back(json_sector);
	}
	j["LAZ sectors"] = json_laz_sectors;

	nlohmann::json json_rois_with_constraints;
	for (const auto& rwc : rois_with_constraints) {
		nlohmann::json json_rwc;
		for (const auto& c : rwc.constraints) {
			nlohmann::json json_roi{
				{"trajectory_name", c.trajectory_file_name},
				{"time_stamp", c.time_stamp},
				{"m_pose", {c.m_pose(0,0), c.m_pose(0,1), c.m_pose(0,2), c.m_pose(0,3), 
							c.m_pose(1,0), c.m_pose(1,1), c.m_pose(1,2), c.m_pose(1,3),
							c.m_pose(2,0), c.m_pose(2,1), c.m_pose(2,2), c.m_pose(2,3)} }
			};
			json_rwc.push_back(json_roi);
		}
		json_rois_with_constraints.push_back(json_rwc);
	}
	j["ROIs with constraints"] = json_rois_with_constraints;

	jj["Project Settings"] = j;

	std::ofstream fs(file_name);
	if (!fs.good())return false;
	fs << jj.dump(2);
	fs.close();
	
	return true;
}

bool ProjectSettings::load(const std::string& file_name, std::vector<LAZSector>& sectors, 
	std::vector<ROIwithConstraints>& rois_with_constraints, CommonData& common_data) {
	sectors.clear();
	rois_with_constraints.clear();
	trajectories.clear();

	std::ifstream fs(file_name);
	if (!fs.good())return false;
	nlohmann::json data = nlohmann::json::parse(fs);
	fs.close();
	
	auto project_settings_json = data["Project Settings"];
	
	clear_color.x = project_settings_json["clear_color_x"];
	clear_color.y = project_settings_json["clear_color_y"];
	clear_color.z = project_settings_json["clear_color_z"];
	clear_color.w = project_settings_json["clear_color_w"];
	common_data.shift_x = project_settings_json["shift_x"];
	common_data.shift_y = project_settings_json["shift_y"];
	//project_main_folder = project_settings_json["project_main_folder"];
	
	for (const auto& t_json : project_settings_json["Trajectories"])
	{
		add_trajectory(t_json["trajectory_file"], t_json["trajectory_motion_model_file"], t_json["colors"][0], t_json["colors"][1], t_json["colors"][2]);
	}

	for (const auto& s_json : project_settings_json["LAZ sectors"])
	{
		LAZSector s;
		s.file_name = s_json["laz_file"];
		s.min_x = s_json["local bounding box"][0];
		s.max_x = s_json["local bounding box"][1];
		s.min_y = s_json["local bounding box"][2];
		s.max_y = s_json["local bounding box"][3];
		s.visible = false;
		sectors.push_back(s);
	}

	for (const auto& s_json : project_settings_json["ROIs with constraints"]) {
		ROIwithConstraints roic;
		for (const auto& r_json : s_json) {
			ConstraintToGeoreference c;
			c.trajectory_file_name = r_json["trajectory_name"];
			c.time_stamp = r_json["time_stamp"];
			c.m_pose = Eigen::Affine3d::Identity();
			c.m_pose(0, 0) = r_json["m_pose"][0];
			c.m_pose(0, 1) = r_json["m_pose"][1];
			c.m_pose(0, 2) = r_json["m_pose"][2];
			c.m_pose(0, 3) = r_json["m_pose"][3];

			c.m_pose(1, 0) = r_json["m_pose"][4];
			c.m_pose(1, 1) = r_json["m_pose"][5];
			c.m_pose(1, 2) = r_json["m_pose"][6];
			c.m_pose(1, 3) = r_json["m_pose"][7];

			c.m_pose(2, 0) = r_json["m_pose"][8];
			c.m_pose(2, 1) = r_json["m_pose"][9];
			c.m_pose(2, 2) = r_json["m_pose"][10];
			c.m_pose(2, 3) = r_json["m_pose"][11];
			roic.constraints.push_back(c);
		}
		rois_with_constraints.push_back(roic);
	}

	//total_length
	total_length = 0.0;
	for (size_t i = 0; i < trajectories.size(); i++) {
		for (size_t j = 1; j < trajectories[i].fused_trajectory.size(); j++) {
			total_length += (trajectories[i].fused_trajectory[j].m_pose.translation() - trajectories[i].fused_trajectory[j - 1].m_pose.translation()).norm();
		}
	}

	return true;
}

bool ProjectSettings::add_trajectory(const std::string& file_name, const std::string& file_motion_model_name, float color_x, float color_y, float color_z)
{
	OdoWithGnssFusion trajectory;
	trajectory.color_x = color_x;
	trajectory.color_y = color_y;
	trajectory.color_z = color_z;
	trajectory.trajectory_file = file_name;
	trajectory.motion_model_file_name = file_motion_model_name;

	trajectory.fused_trajectory = trajectory.load_trajectory(trajectory.trajectory_file);
	trajectory.fused_trajectory_motion_model = trajectory.load_trajectory(trajectory.motion_model_file_name);

	trajectories.push_back(trajectory);

	return true;
}

void ProjectSettings::render(const std::vector<ROIwithConstraints>& rois_with_constraints)
{
	for (const auto& trj : trajectories) {
		if (trj.visible) {
			glColor3f(trj.color_x, trj.color_y, trj.color_z);
			glLineWidth(trj.line_width);
			glBegin(GL_LINE_STRIP);
			for (const auto& node : trj.fused_trajectory) {
				glVertex3f(node.m_pose(0, 3), node.m_pose(1, 3), node.m_pose(2, 3));
			}
			glEnd();
		}
		glLineWidth(1);
	}

	for (const auto &rwcs: rois_with_constraints) {
		for (const auto& c : rwcs.constraints) {
			for (const auto& trj : trajectories) {
				if (c.trajectory_file_name == trj.trajectory_file) {
					auto it = std::lower_bound(trj.fused_trajectory.begin(), trj.fused_trajectory.end(),
						c.time_stamp, [](Node lhs, double time) -> bool { return lhs.timestamp < time; });

					int index = it - trj.fused_trajectory.begin();
					
					if (fabs(trj.fused_trajectory[index].timestamp - c.time_stamp) < 0.1) {
						glColor3f(0.1, 0.1, 0.1);
						glBegin(GL_LINES);
						glVertex3f(trj.fused_trajectory[index].m_pose.translation().x(), 
							trj.fused_trajectory[index].m_pose.translation().y(),
							trj.fused_trajectory[index].m_pose.translation().z());

						glVertex3f(c.m_pose.translation().x(), c.m_pose.translation().y(), c.m_pose.translation().z());
						glEnd();
					}
				}
			}
		}
	}
}

void ProjectSettings::pose_graph_slam(std::vector<ROIwithConstraints>& rois_with_constraints)
{
	/*for (auto& trj : trajectories) {
		for (auto& node : trj.fused_trajectory) {
			node.m_pose(2, 3) += 10;
			std::cout << node.index_to_gnss << " ";
		}
	}*/

	std::vector<Eigen::Affine3d> constraints;

	for (const auto& rwcs : rois_with_constraints) {
		for (const auto& c : rwcs.constraints) {
			for (auto& trj : trajectories) {
				if (c.trajectory_file_name == trj.trajectory_file) {
					auto it = std::lower_bound(trj.fused_trajectory.begin(), trj.fused_trajectory.end(),
						c.time_stamp, [](Node lhs, double time) -> bool { return lhs.timestamp < time; });

					int index = it - trj.fused_trajectory.begin();

					if (fabs(trj.fused_trajectory[index].timestamp - c.time_stamp) < 0.1) {
						Eigen::Affine3d constraint = c.m_pose;
						trj.fused_trajectory[index].index_to_gnss = constraints.size();
						constraints.push_back(constraint);
					}
				}
			}
		}
	}

	for (auto& trj : trajectories) {
		std::vector<BetweenNode> bn = find_between_nodes(trj.fused_trajectory);
		fuse_with_georeference(bn, constraints, trj.fused_trajectory, trj.fused_trajectory_motion_model);
		//for (auto& b : bn) {
		//	std::cout << b.node_outer.index_to_gnss << std::endl;
		//}
	}
	/*for (auto& trj : trajectories) {
		for (auto& node : trj.fused_trajectory) {
			node.m_pose(2, 3) += 10;
			std::cout << node.index_to_gnss << " ";
			if(node.index_to_gnss != -1)
			return;
		}
	}*/
}


std::vector<BetweenNode> ProjectSettings::find_between_nodes(std::vector<Node> &fused_trajectory) {
	std::vector<BetweenNode> between_nodes;
	double dist_along = 0.0f;
	//double dist_along_gnss = 0.0f;

	BetweenNode node_outer;
	node_outer.node_outer.index_to_lidar_odometry_odo = 0;
	node_outer.node_outer.m_pose = fused_trajectory[0].m_pose;
	node_outer.node_outer.timestamp = fused_trajectory[0].timestamp;

	Node node_inner;
	
	for (size_t i = 1; i < fused_trajectory.size(); i++) {
		node_inner.index_to_lidar_odometry_odo = i;
		node_inner.index_to_gnss = -1;
		node_inner.m_pose = fused_trajectory[i].m_pose;
		node_inner.timestamp = fused_trajectory[i].timestamp;

		double dist_increment = (fused_trajectory[i].m_pose.translation() - fused_trajectory[i - 1].m_pose.translation()).norm();
		dist_along += dist_increment;
		//dist_along_gnss += dist_increment;

		//std::cout << "dist_increment " << dist_increment << std::endl;
#if 0
		if (dist_along_gnss > 10) {
			auto it = std::lower_bound(gnss_trajectory_shifted.begin(), gnss_trajectory_shifted.end(), fused_trajectory[i],
				[](Node lhs, Node rhs) -> bool { return lhs.timestamp < rhs.timestamp; });
			if (fabs(it->timestamp - fused_trajectory[i].timestamp) < 0.01) {
				int index_to_gnss = it - gnss_trajectory_shifted.begin();
				int res_index = index_to_gnss;
				double dist_min = 1000000.0;
				for (int index = index_to_gnss - 100; index < index_to_gnss + 100; index++) {
					if (index >= 0 && index < gnss_trajectory_shifted.size()) {
						double distance = sqrt((fused_trajectory[i].m_pose(0, 3) - gnss_trajectory_shifted[index].m_pose(0, 3)) *
							(fused_trajectory[i].m_pose(0, 3) - gnss_trajectory_shifted[index].m_pose(0, 3)) +
							(fused_trajectory[i].m_pose(1, 3) - gnss_trajectory_shifted[index].m_pose(1, 3)) *
							(fused_trajectory[i].m_pose(1, 3) - gnss_trajectory_shifted[index].m_pose(1, 3)));

						if (distance < dist_min) {
							dist_min = distance;
							res_index = index;
						}
					}
				}
				node_inner.index_to_gnss = res_index;
				dist_along_gnss = 0.0;
				//node_inner.index_to_gnss = it - gnss_trajectory_shifted.begin();
				//dist_along_gnss = 0.0;
			}
		}
#endif
		node_outer.nodes_between.push_back(node_inner);

		if ((dist_along > 100 || i == fused_trajectory.size() - 1) || fused_trajectory[i].index_to_gnss != -1) {
			for (auto& n : node_outer.nodes_between) {
				n.m_pose = node_outer.node_outer.m_pose.inverse() * n.m_pose;
			}
			node_outer.color_x = float(rand() % 255) / 256.0;
			node_outer.color_y = float(rand() % 255) / 256.0;
			node_outer.color_z = float(rand() % 255) / 256.0;

			between_nodes.push_back(node_outer);

			node_outer.nodes_between.clear();
			node_outer.node_outer.index_to_lidar_odometry_odo = i;
			node_outer.node_outer.index_to_gnss = fused_trajectory[i].index_to_gnss;
			node_outer.node_outer.m_pose = fused_trajectory[i].m_pose;
			node_outer.node_outer.timestamp = fused_trajectory[i].timestamp;

			dist_along = 0.0;
		}
	}

	return between_nodes;
}

void ProjectSettings::fuse_with_georeference(std::vector<BetweenNode>& between_nodes, const std::vector<Eigen::Affine3d>& constraints, std::vector<Node>& fused_trajectory, const std::vector<Node>& motion_model) {
	std::vector<std::pair<int, int>> odo_edges;
	for (size_t i = 1; i < between_nodes.size(); i++) {
		odo_edges.emplace_back(i - 1, i);
	}

	std::vector<Eigen::Triplet<double>> tripletListA;
	std::vector<Eigen::Triplet<double>> tripletListP;
	std::vector<Eigen::Triplet<double>> tripletListB;

	std::vector<TaitBryanPose> poses;
	std::vector<TaitBryanPose> poses_desired;

	for (size_t i = 0; i < between_nodes.size(); i++) {
		poses.push_back(pose_tait_bryan_from_affine_matrix(between_nodes[i].node_outer.m_pose));

		poses_desired.push_back(pose_tait_bryan_from_affine_matrix(motion_model[between_nodes[i].node_outer.index_to_lidar_odometry_odo].m_pose));
	
	}

	//motiuon_model


	poses_desired = poses;

	for (size_t i = 0; i < odo_edges.size(); i++) {
		Eigen::Matrix<double, 6, 1> relative_pose_measurement_odo;
		relative_pose_tait_bryan_wc_case1(relative_pose_measurement_odo,
			poses_desired[odo_edges[i].first].px,
			poses_desired[odo_edges[i].first].py,
			poses_desired[odo_edges[i].first].pz,
			poses_desired[odo_edges[i].first].om,
			poses_desired[odo_edges[i].first].fi,
			poses_desired[odo_edges[i].first].ka,
			poses_desired[odo_edges[i].second].px,
			poses_desired[odo_edges[i].second].py,
			poses_desired[odo_edges[i].second].pz,
			poses_desired[odo_edges[i].second].om,
			poses_desired[odo_edges[i].second].fi,
			poses_desired[odo_edges[i].second].ka);

		Eigen::Matrix<double, 6, 1> delta;
		relative_pose_obs_eq_tait_bryan_wc_case1(
			delta,
			poses[odo_edges[i].first].px,
			poses[odo_edges[i].first].py,
			poses[odo_edges[i].first].pz,
			poses[odo_edges[i].first].om,
			poses[odo_edges[i].first].fi,
			poses[odo_edges[i].first].ka,
			poses[odo_edges[i].second].px,
			poses[odo_edges[i].second].py,
			poses[odo_edges[i].second].pz,
			poses[odo_edges[i].second].om,
			poses[odo_edges[i].second].fi,
			poses[odo_edges[i].second].ka,
			relative_pose_measurement_odo(0, 0),
			relative_pose_measurement_odo(1, 0),
			relative_pose_measurement_odo(2, 0),
			normalize_angle(relative_pose_measurement_odo(3, 0)),
			normalize_angle(relative_pose_measurement_odo(4, 0)),
			normalize_angle(relative_pose_measurement_odo(5, 0)));

		Eigen::Matrix<double, 6, 12, Eigen::RowMajor> jacobian;
		relative_pose_obs_eq_tait_bryan_wc_case1_jacobian(jacobian,
			poses[odo_edges[i].first].px,
			poses[odo_edges[i].first].py,
			poses[odo_edges[i].first].pz,
			poses[odo_edges[i].first].om,
			poses[odo_edges[i].first].fi,
			poses[odo_edges[i].first].ka,
			poses[odo_edges[i].second].px,
			poses[odo_edges[i].second].py,
			poses[odo_edges[i].second].pz,
			poses[odo_edges[i].second].om,
			poses[odo_edges[i].second].fi,
			poses[odo_edges[i].second].ka);

		int ir = tripletListB.size();

		int ic_1 = odo_edges[i].first * 6;
		int ic_2 = odo_edges[i].second * 6;

		for (size_t row = 0; row < 6; row++) {
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

		tripletListP.emplace_back(ir, ir, 10000);
		tripletListP.emplace_back(ir + 1, ir + 1, 10000);
		tripletListP.emplace_back(ir + 2, ir + 2, 10000);
		tripletListP.emplace_back(ir + 3, ir + 3, 10000);
		tripletListP.emplace_back(ir + 4, ir + 4, 10000);
		tripletListP.emplace_back(ir + 5, ir + 5, 10000);
	}

	//gnss correspondences

	for (int i = 0; i < between_nodes.size(); i++) {
		if (between_nodes[i].node_outer.index_to_gnss != -1) {
			TaitBryanPose pose_source = pose_tait_bryan_from_affine_matrix(between_nodes[i].node_outer.m_pose);

			const Eigen::Affine3d& m_geo = constraints[between_nodes[i].node_outer.index_to_gnss];

			Eigen::Vector3d p_s(
				//between_nodes[i].nodes_between[j].m_pose.translation().x(),
				//between_nodes[i].nodes_between[j].m_pose.translation().y(),
				//between_nodes[i].nodes_between[j].m_pose.translation().z());
				0,
				0,
				0);
			Eigen::Vector3d p_t(
				m_geo.translation().x(),
				m_geo.translation().y(),
				m_geo.translation().z());

			double delta_x;
			double delta_y;
			double delta_z;
			point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z,
				pose_source.px, pose_source.py, pose_source.pz, pose_source.om, pose_source.fi, pose_source.ka,
				p_s.x(), p_s.y(), p_s.z(), p_t.x(), p_t.y(), p_t.z());

			Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
			point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian,
				pose_source.px, pose_source.py, pose_source.pz, pose_source.om, pose_source.fi, pose_source.ka, p_s.x(), p_s.y(), p_s.z());

			int ir = tripletListB.size();
			int ic = i * 6;
			for (int row = 0; row < 3; row++) {
				for (int col = 0; col < 6; col++) {
					if (jacobian(row, col) != 0.0) {
						tripletListA.emplace_back(ir + row, ic + col, -jacobian(row, col));
					}
				}
			}

			tripletListP.emplace_back(ir, ir, get_cauchy_w(delta_x, 1));
			tripletListP.emplace_back(ir + 1, ir + 1, get_cauchy_w(delta_y, 1));
			tripletListP.emplace_back(ir + 2, ir + 2, get_cauchy_w(delta_z, 1));

			//tripletListP.emplace_back(ir    ,     ir, 1);
			//tripletListP.emplace_back(ir + 1, ir + 1, 1);
			//tripletListP.emplace_back(ir + 2, ir + 2, 1);

			tripletListB.emplace_back(ir    , 0, delta_x);
			tripletListB.emplace_back(ir + 1, 0, delta_y);
			tripletListB.emplace_back(ir + 2, 0, delta_z);


		}
	}
	
	Eigen::SparseMatrix<double> matA(tripletListB.size(), between_nodes.size() * 6);
	Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
	Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

	matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
	matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
	matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

	Eigen::SparseMatrix<double> AtPA(between_nodes.size() * 6, between_nodes.size() * 6);
	Eigen::SparseMatrix<double> AtPB(between_nodes.size() * 6, 1);

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
	std::cout << "result: row, col, value" << std::endl;
	for (int k = 0; k < x.outerSize(); ++k) {
		for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it) {
			//std::cout << it.row() << " " << it.col() << " " << it.value() << std::endl;
			h_x.push_back(it.value());
		}
	}

	if (h_x.size() == 6 * between_nodes.size()) {
		int counter = 0;

		for (size_t i = 0; i < between_nodes.size(); i++) {
			TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(between_nodes[i].node_outer.m_pose);
			pose.px += h_x[counter++];
			pose.py += h_x[counter++];
			pose.pz += h_x[counter++];
			pose.om += h_x[counter++];
			pose.fi += h_x[counter++];
			pose.ka += h_x[counter++];

			between_nodes[i].node_outer.m_pose = affine_matrix_from_pose_tait_bryan(pose);
		}
		std::cout << "optimizing with tait bryan finished" << std::endl;

		for (size_t i = 0; i < between_nodes.size(); i++) {
			fused_trajectory[between_nodes[i].node_outer.index_to_lidar_odometry_odo].m_pose = between_nodes[i].node_outer.m_pose;
		}
		for (size_t i = 0; i < between_nodes.size(); i++) {
			for (size_t j = 0; j < between_nodes[i].nodes_between.size() - 1; j++) {
				fused_trajectory[between_nodes[i].nodes_between[j].index_to_lidar_odometry_odo].m_pose =
					between_nodes[i].node_outer.m_pose * between_nodes[i].nodes_between[j].m_pose;
			}
		}

		fused_trajectory[fused_trajectory.size() - 1].m_pose =
			between_nodes[between_nodes.size() - 1].node_outer.m_pose *
			between_nodes[between_nodes.size() - 1].nodes_between[between_nodes[between_nodes.size() - 1].nodes_between.size() - 1].m_pose;
	}
	else {
		std::cout << "optimizing with tait bryan FAILED" << std::endl;
	}
}