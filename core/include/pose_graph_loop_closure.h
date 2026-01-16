#pragma once

#include <control_points.h>
#include <gnss.h>
#include <ground_control_points.h>
#include <point_clouds.h>

class PoseGraphLoopClosure
{
public:
    enum EdgeType
    {
        motion_model_1,
        motion_model_2,
        loop_closure
    };

    struct Edge
    {
        TaitBryanPose relative_pose_tb;
        TaitBryanPose relative_pose_tb_weights;
        int index_from;
        int index_to;
        bool is_fixed_px = false;
        bool is_fixed_py = false;
        bool is_fixed_pz = false;
        bool is_fixed_om = false;
        bool is_fixed_fi = false;
        bool is_fixed_ka = false;
        EdgeType type;
    };

    std::vector<Edge> edges;
    std::vector<Eigen::Affine3d> poses_motion_model;

    // edge.relative_pose_tb_weights.om = 10000.0;
    // edge.relative_pose_tb_weights.fi = 10000.0;
    // edge.relative_pose_tb_weights.ka = 10000.0;
    // edge.relative_pose_tb_weights.px = 100.0;
    // edge.relative_pose_tb_weights.py = 10000.0;
    // edge.relative_pose_tb_weights.pz = 10000.0;

    double motion_model_w_px_1_sigma_m = 0.1;
    double motion_model_w_py_1_sigma_m = 0.01;
    double motion_model_w_pz_1_sigma_m = 0.01;

    double motion_model_w_om_1_sigma_deg = 1.0 / 100.0 * 180.0 / M_PI; // 0.01;
    double motion_model_w_fi_1_sigma_deg = 1.0 / 100.0 * 180.0 / M_PI;
    double motion_model_w_ka_1_sigma_deg = 1.0 / 100.0 * 180.0 / M_PI;

    PoseGraphLoopClosure() {};
    ~PoseGraphLoopClosure() {};

    void add_edge(PointClouds& point_clouds_container, int index_loop_closure_source, int index_loop_closure_target);
    void set_initial_poses_as_motion_model(PointClouds& point_clouds_container);
    void set_current_poses_as_motion_model(PointClouds& point_clouds_container);
    void graph_slam(PointClouds& point_clouds_container, GNSS& gnss, GroundControlPoints& gcps, ControlPoints& cps);
    void FuseTrajectoryWithGNSS(PointClouds& point_clouds_container, GNSS& gnss);
    void run_icp(
        PointClouds& point_clouds_container,
        int index_active_edge,
        float search_radius,
        int number_of_iterations,
        int num_edge_extended_before,
        int num_edge_extended_after);
};
