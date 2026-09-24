#pragma once

#include <Core/point_clouds.h>

#include <string>
#include <vector>

struct ControlPoint
{
    char name[64];
    double x_source_local;
    double y_source_local;
    double z_source_local;
    double x_target_global;
    double y_target_global;
    double z_target_global;
    double sigma_x;
    double sigma_y;
    double sigma_z;
    int index_to_pose;
    bool is_z_0 = false;
};

class ControlPoints
{
public:
    ControlPoints() = default;
    ~ControlPoints() = default;

    bool is_imgui = false;
    std::vector<ControlPoint> cps;

    // Data members stay outside WITH_GUI so the class layout is identical in GUI and non-GUI builds.
    bool picking_mode = false;
    bool draw_uncertainty = false;
    int index_picked_point = -1;
    bool track_pose_with_camera = true;

    int index_pose = 0;

#if WITH_GUI == 1
    void imgui(PointClouds& point_clouds_container, const Eigen::Vector3f& rotation_center);
    void render(const PointClouds& point_clouds_container, bool show_pc);
    void draw_ellipse(const Eigen::Matrix3d& covar, const Eigen::Vector3d& mean, const Eigen::Vector3f& color, float nstd = 1);
#endif
};
