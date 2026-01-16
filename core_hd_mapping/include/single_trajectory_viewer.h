#pragma once

#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <imgui_internal.h>

#include <ImGuizmo.h>

#include <odo_with_gnss_fusion.h>
#include <structures.h>

class SingleTrajectoryViewer
{
public:
    SingleTrajectoryViewer() = default;
    ~SingleTrajectoryViewer() = default;

    void imgui(const CommonData& common_data);
    void render();

    OdoWithGnssFusion trajectory_container;
    double current_time_stamp = 0.0f;
    std::vector<ChunkFile> chunk_files;
    std::string working_directory;
    std::vector<Point> points;
    std::vector<PointCloudWithPose> point_clouds;
    std::string trajectory_filename;

    std::vector<PointCloudWithPose> get_point_cloud_for_roi(Eigen::Vector3d roi, float roi_size);

    bool load_fused_trajectory(const std::string& file_name);
    bool load_fused_trajectory();
    std::vector<Point> load_points_and_transform_to_global(double ts, Eigen::Vector3d roi, float roi_size, int ext);
};
