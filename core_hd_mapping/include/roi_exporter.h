#ifndef _ROI_EXPORTER_H_
#define _ROI_EXPORTER_H_

#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <ImGuizmo.h>
#include <imgui_internal.h>

#include <structures.h>
#include <project_settings.h>

class RoiExporter {
public:
	RoiExporter() { ; };
	~RoiExporter() { ; };

	void imgui(CommonData& common_data, const ProjectSettings& project_setings, std::vector<LAZSector>& sectors);
	void render(const CommonData& common_data);
	PointCloudWithPose get_pc_from_laz(CommonData& common_data, std::vector<LAZSector>& sectors, double shift_x, double shift_y);
	void export_to_RESSO_format(std::vector<PointCloudWithPose>& roi_point_clouds);
	void import_from_RESSO_format_as_constraints_to_georeference();

	std::vector<PointCloudWithPose> roi_point_clouds;
	int num_threads = 16;
	int decimation = 100;

	std::vector<ROIwithConstraints> rois_with_constraints;
	Eigen::Affine3d m_fake;
};

#endif