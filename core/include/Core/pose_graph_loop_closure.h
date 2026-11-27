#pragma once

#include <Core/control_points.h>
#include <Core/gnss.h>
#include <Core/ground_control_points.h>
#include <Core/pair_wise_iterative_closest_point.h>
#include <Core/point_clouds.h>
#include <Core/tum.h>

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

    bool render_source_as_red_target_as_blue = true;
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

    // When false, graph_slam() does not add any GNSS <-> LiDAR-trajectory
    // observations to the optimisation (loaded GNSS poses are still drawn,
    // they just don't pull on the pose graph). When true (default) every
    // GNSS pose contributes, as before.
    bool use_gnss_correspondences = true;

    PoseGraphLoopClosure() {};
    ~PoseGraphLoopClosure() {};

    void add_edge(PointClouds& point_clouds_container, int index_loop_closure_source, int index_loop_closure_target);
    void set_initial_poses_as_motion_model(PointClouds& point_clouds_container);
    void set_current_poses_as_motion_model(PointClouds& point_clouds_container);
    void graph_slam(PointClouds& point_clouds_container, GNSS& gnss, GroundControlPoints& gcps, ControlPoints& cps);
    void FuseTrajectoryWithGNSS(PointClouds& point_clouds_container, GNSS& gnss);
    void FuseTrajectoryWithTUM(PointClouds& point_clouds_container, TUM& tum);
    //! Refines one edge's relative pose with point-to-point ICP between its two scans (each extended by its
    //! neighbouring scans).
    //! @return false when ICP fails; the edge is then left unchanged
    bool run_icp(
        PointClouds& point_clouds_container,
        int index_active_edge,
        float search_radius,
        int number_of_iterations,
        int num_edge_extended_before,
        int num_edge_extended_after);
    //! Standard deviations of one edge's relative pose (px, py, pz in metres; om, fi, ka in radians) from the ICP
    //! normal matrix at the current relative pose, on the same clouds as run_icp().
    //! @param point_to_plane false: point-to-point residuals (as run_icp() minimises); true: point-to-plane residuals
    //! @return the standard deviations (infinity for every parameter when the geometry does not constrain the pose) and
    //! the residual statistics with the covariance
    std::pair<TaitBryanPose, IcpResidualStatistics> compute_icp_uncertainty(
        const PointClouds& point_clouds_container,
        int index_active_edge,
        double search_radius,
        int num_edge_extended_before,
        int num_edge_extended_after,
        bool point_to_plane);
    //! Monte Carlo ICP ("ICP spread") of one edge's relative pose: PairWiseICP::compute_icp_montecarlo() on the same
    //! clouds as run_icp(). The edge is left unchanged.
    IcpSpread compute_icp_montecarlo(
        const PointClouds& point_clouds_container,
        int index_active_edge,
        double search_radius,
        int number_of_iterations,
        int num_edge_extended_before,
        int num_edge_extended_after,
        int number_of_runs,
        double start_offset_m,
        double start_offset_rad) const;
    //! Search radii for coarse-to-fine ICP: `start_radius`, then those of the per-edge ICP buttons' steps (2.0, 1.0,
    //! 0.5, 0.25, 0.1 m) below it.
    static std::vector<float> icp_search_radii(float start_radius);
};
