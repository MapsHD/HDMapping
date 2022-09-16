#ifndef _PROJECT_SETTINGS_H_
#define _PROJECT_SETTINGS_H_

#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <ImGuizmo.h>
#include <imgui_internal.h>

#include <string>

#include <odo_with_gnss_fusion.h>
#include <structures.h>


class LazWrapper;

class ProjectSettings {
public:
	ProjectSettings() { ; };
	~ProjectSettings() { ; };

	void imgui(OdoWithGnssFusion& odo_with_gnss_fusion, std::vector<LAZSector>& sectors, std::vector<ROIwithConstraints>& rois_with_constraints, CommonData& common_data);
	bool save(const std::string& file_name, std::vector<LAZSector>& sectors, const std::vector<ROIwithConstraints>& rois_with_constraints);
	bool load(const std::string& file_name, std::vector<LAZSector>& sectors, std::vector<ROIwithConstraints>& rois_with_constraints);
	bool add_trajectory(const std::string& file_name, const std::string& file_motion_model_name, float color_x, float color_y, float color_z);
	void render(const std::vector<ROIwithConstraints>& rois_with_constraints);
	void pose_graph_slam(std::vector<ROIwithConstraints>& rois_with_constraints);
	std::vector<BetweenNode> find_between_nodes(std::vector<Node>& fused_trajectory);
	void fuse_with_georeference(std::vector<BetweenNode>& bn, const std::vector<Eigen::Affine3d>& constraints, std::vector<Node>& fused_trajectory, const std::vector<Node>& motion_model);

	ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
	float shift_x = 0.0;
	float shift_y = 0.0;
	//std::string project_main_folder;

	std::vector<OdoWithGnssFusion> trajectories;
	double total_length = 0;
};

#endif