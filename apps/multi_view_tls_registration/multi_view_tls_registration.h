
#pragma once

#include <filesystem>
#include <gnss.h>
#include <icp.h>
#include <ndt.h>
#include <pose_graph_slam.h>
#include <regex>
#include <registration_plane_feature.h>
#include <session.h>
#include <transformations.h>

namespace fs = std::filesystem;

struct TLSRegistration
{
    // NDT
    bool use_ndt = false;
    NDT ndt;
    bool compute_only_mahalanobis_distance = false;
    bool compute_mean_and_cov_for_bucket = false;
    bool use_lie_algebra_left_jacobian_ndt = false;
    bool use_lie_algebra_right_jacobian_ndt = false;
    void set_zoller_frohlich_tls_imager_5006i_errors()
    {
        ndt.sigma_r = 0.0068;
        ndt.sigma_polar_angle = 0.007 / 180.0 * M_PI;
        ndt.sigma_azimuthal_angle = 0.007 / 180.0 * M_PI;
    }
    void set_zoller_frohlich_tls_imager_5010c_errors()
    {
        ndt.sigma_r = 0.01;
        ndt.sigma_polar_angle = 0.007 / 180.0 * M_PI;
        ndt.sigma_azimuthal_angle = 0.007 / 180.0 * M_PI;
    }
    void set_zoller_frohlich_tls_imager_5016_errors()
    {
        ndt.sigma_r = 0.00025;
        ndt.sigma_polar_angle = 0.004 / 180.0 * M_PI;
        ndt.sigma_azimuthal_angle = 0.004 / 180.0 * M_PI;
    }
    void set_faro_focus3d_errors()
    {
        ndt.sigma_r = 0.001;
        ndt.sigma_polar_angle = 19.0 * (1.0 / 3600.0) / 180.0 * M_PI;
        ndt.sigma_azimuthal_angle = 19.0 * (1.0 / 3600.0) / 180.0 * M_PI;
    }
    void set_leica_scanstation_c5_c10_errors()
    {
        ndt.sigma_r = 0.006;
        ndt.sigma_polar_angle = 0.00006;
        ndt.sigma_azimuthal_angle = 0.00006;
    }
    void set_riegl_vz400_errors()
    {
        ndt.sigma_r = 0.005;
        ndt.sigma_polar_angle = 0.0005 / 180.0 * M_PI + 0.0003; // Laser Beam Dicvergence
        ndt.sigma_azimuthal_angle = 0.0005 / 180.0 * M_PI + 0.0003; // Laser Beam Dicvergence
    }
    void set_leica_hds6100_errors()
    {
        ndt.sigma_r = 0.009;
        ndt.sigma_polar_angle = 0.000125;
        ndt.sigma_azimuthal_angle = 0.000125;
    }
    void set_leica_p40_errors()
    {
        ndt.sigma_r = 0.0012;
        ndt.sigma_polar_angle = 8.0 / 3600;
        ndt.sigma_azimuthal_angle = 8.0 / 3600;
    }

    void set_livox_mid360_errors()
    {
        ndt.sigma_r = 0.02;
        ndt.sigma_polar_angle = 0.15 / 180.0 * M_PI;
        ndt.sigma_azimuthal_angle = 0.15 / 180.0 * M_PI;
    }

    // ICP
    bool use_icp = false;
    ICP icp;
    bool point_to_point_source_to_target = true;
    bool use_lie_algebra_left_jacobian_icp = false;
    bool use_lie_algebra_right_jacobian_icp = false;
    bool point_to_point_source_to_target_compute_rms = false;

    // Plane features
    bool use_plane_features = false;
    RegistrationPlaneFeature registration_plane_feature;
    bool point_to_projection_onto_plane_source_to_target = true;
    bool use_lie_algebra_left_jacobian_plane_features = false;
    bool use_lie_algebra_right_jacobian_plane_features = false;
    bool point_to_plane_source_to_target_dot_product = false;
    bool point_to_plane_source_to_target = false;
    bool plane_to_plane_source_to_target = false;

    // PGSLAM
    bool use_pgslam = false;
    PoseGraphSLAM pose_graph_slam;

    // GNSS
    GNSS gnss;

    // Loading
    bool calculate_offset; // Whether to calculate offset to point cloud on loading
    bool is_decimate = true; // Whether to decimate point clouds on loading
    double bucket_x = 0.1; // Bucket size for decimation in x dimension
    double bucket_y = 0.1; // Bucket size for decimation in y dimension
    double bucket_z = 0.1; // Bucket size for decimation in z dimension
    std::string resso_upd_init = ""; // Path to RESSO initial poses
    std::string resso_upd = ""; // Path to RESSO poses
    std::string resso_upd_inv = ""; // Path to RESSO inverse poses

    // Registration
    bool initial_pose_to_identity = true; // Whether to set first pose as identity and recalculate the rest relatively to it

    // Export
    bool save_las = false; // Save resulting point cloud as las
    bool save_laz = false; // Save resulting point cloud as laz
    bool save_as_separate_las = false; // Whether to save all scans as separate global scans in las format
    bool save_as_separate_laz = false; // Whether to save all scans as separate global scans in laz format
    bool save_trajectories_laz = false; // Save laz with all trajectories
    bool save_gnss_laz = false; // Save laz with GNSS data
    bool save_scale_board_laz = false; // Save laz with scale board
    float scale_board_dec = 0.1; // Decimation for scale board laz export
    float scale_board_side_len = -1.0; // Range covered in x and y dimensions
    bool save_initial_poses = false; // Path where initial poses are saved
    bool save_poses = false; // Path where poses are saved
    bool is_trajectory_export_downsampling = false; // Whether to downsample trajectory on export
    float curve_consecutive_distance_meters = 1.0f; // Meters after which trajectory point is saved (if downsampling and curve is detected)
    float not_curve_consecutive_distance_meters =
        0.05f; // Meters after which trajectory point is saved (if downsampling and no curve detected)
    bool save_trajectories_csv = false; // Save trajectories as csv
    bool save_trajectories_dxf = false; // Save trajectories as dxf
    bool write_lidar_timestamp = true; // Whether lidar timestamp is wrriten to csv trajectory output
    bool write_unix_timestamp = false; // Whether unix timestamp is written to csv trajectory
    bool use_quaternions = true; // Whether quaternions are used when writing csv trajectory
};

bool has_extension(const std::string file_path, const std::string extension);

void initial_pose_to_identity(Session& session);

void save_intersection(
    const Session& session,
    std::string output_las_name,
    bool xz_intersection,
    bool yz_intersection,
    bool xy_intersection,
    double intersection_width);

void save_separately_to_las(const Session& session, fs::path outwd, std::string extension = ".las");

void save_trajectories_to_laz(
    const Session& session,
    std::string output_file_name,
    float curve_consecutive_distance_meters,
    float not_curve_consecutive_distance_meters,
    bool is_trajectory_export_downsampling);

void save_scale_board_to_laz(const Session& session, std::string output_file_name, float dec, float side_len = -1.0);

void createDXFPolyline(const std::string& filename, const std::vector<Eigen::Vector3d>& points);

// void load_available_geo_points(Session& session, std::string input_file_name);

void save_trajectories(
    Session& session,
    std::string output_file_name,
    float curve_consecutive_distance_meters,
    float not_curve_consecutive_distance_meters,
    bool is_trajectory_export_downsampling,
    bool write_lidar_timestamp = true,
    bool write_unix_timestamp = false,
    bool use_quaternions = true,
    bool save_to_dxf = false);

void run_multi_view_tls_registration(std::string input_file_name, TLSRegistration& tls_registration, std::string output_dir = "");
