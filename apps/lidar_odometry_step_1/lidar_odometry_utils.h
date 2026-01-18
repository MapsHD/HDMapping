#pragma once

#include <chrono>
#include <execution>
#include <filesystem>
#include <iostream>
#include <map>
#include <vector>

#include <Eigen/Dense>
#include <Fusion.h>
#include <common/include/cauchy.h>
#include <laszip/laszip_api.h>
#include <ndt.h>
#include <nlohmann/json.hpp>
#include <structures.h>
#include <transformations.h>

#include <python-scripts/constraints/constraint_fixed_parameter_jacobian.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>
#include <python-scripts/constraints/smoothness_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-feature-metrics/point_to_line_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian_simplified.h>

#if WITH_GUI == 1
#include <imgui.h>
#endif

namespace fs = std::filesystem;

using NDTBucketMapType = std::unordered_map<uint64_t, NDT::Bucket>;
using NDTBucketMapType2 = std::unordered_map<uint64_t, NDT::Bucket2>;

// Helper function for getting software version from CMake macros
inline std::string get_software_version()
{
#ifdef HDMAPPING_VERSION_MAJOR
    return std::to_string(HDMAPPING_VERSION_MAJOR) + "." + std::to_string(HDMAPPING_VERSION_MINOR) + "." +
        std::to_string(HDMAPPING_VERSION_PATCH);
#else
    return "0.84.0"; // fallback if CMake macros not available
#endif
}

struct LidarOdometryParams
{
    // version information - automatically generated from CMake build system
    std::string software_version = get_software_version();
    std::string config_version = "1.0";
    std::string build_date = __DATE__;

    // performance
    bool useMultithread = true;
    double real_time_threshold_seconds = 10.0; // for realtime use: threshold_nr_poses * 0.005, where 0.005 is related with IMU frequency

    // filter points
    double filter_threshold_xy_inner = 0.3; // filtering points during load
    double filter_threshold_xy_outer = 70.0; // filtering points during load
    double decimation = 0.01; // filtering points during load
    double threshould_output_filter = 0.5; // for export --> all points xyz.norm() < threshould_output_filter will be removed
    int min_counter_concatenated_trajectory_nodes = 10; // for export

    // Madgwick filter
    bool fusionConventionNwu = true;
    bool fusionConventionEnu = false;
    bool fusionConventionNed = false;
    double ahrs_gain = 0.5;

    // lidar odometry control
    bool use_motion_from_previous_step = true;
    int nr_iter = 100;
    NDT::GridParameters in_out_params_indoor;
    NDT::GridParameters in_out_params_outdoor;
    double sliding_window_trajectory_length_threshold = 5.0;
    double max_distance_lidar = 70.0; // I am not processing data above dist distance in lidar odometry
    int threshold_initial_points = 10000;
    int threshold_nr_poses = 20;

    // lidar odometry debug info
    bool save_calibration_validation = false;
    int calibration_validation_points = 1000000;

    // consistency
    int num_constistency_iter = 10;
    bool use_mutliple_gaussian = false;

    // motion_model uncertainty
    double lidar_odometry_motion_model_x_1_sigma_m = 0.0005;
    double lidar_odometry_motion_model_y_1_sigma_m = 0.0005;
    double lidar_odometry_motion_model_z_1_sigma_m = 0.0005;

    double lidar_odometry_motion_model_om_1_sigma_deg = 0.01;
    double lidar_odometry_motion_model_fi_1_sigma_deg = 0.01;
    double lidar_odometry_motion_model_ka_1_sigma_deg = 0.01;

    // motion_model first trajectory node prior uncertainty
    double lidar_odometry_motion_model_fix_origin_x_1_sigma_m = 0.000001;
    double lidar_odometry_motion_model_fix_origin_y_1_sigma_m = 0.000001;
    double lidar_odometry_motion_model_fix_origin_z_1_sigma_m = 0.000001;
    double lidar_odometry_motion_model_fix_origin_om_1_sigma_deg = 0.000001;
    double lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg = 0.000001;
    double lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg = 0.000001;

    // motion model correction (experimental --> not recommended yet)
    TaitBryanPose motion_model_correction;

    // robust lidar odometry control (experimental --> not recommended yet)
    bool use_robust_and_accurate_lidar_odometry = false;
    double distance_bucket = 0.2;
    double polar_angle_deg = 10.0;
    double azimutal_angle_deg = 10.0;
    int robust_and_accurate_lidar_odometry_iterations = 20;
    double distance_bucket_rigid_sf = 0.5;
    double polar_angle_deg_rigid_sf = 10.0;
    double azimutal_angle_deg_rigid_sf = 10.0;
    int robust_and_accurate_lidar_odometry_rigid_sf_iterations = 30;
    double max_distance_lidar_rigid_sf = 70.0;
    double rgd_sf_sigma_x_m = 0.001;
    double rgd_sf_sigma_y_m = 0.001;
    double rgd_sf_sigma_z_m = 0.001;
    double rgd_sf_sigma_om_deg = 0.01;
    double rgd_sf_sigma_fi_deg = 0.01;
    double rgd_sf_sigma_ka_deg = 0.01;

    // paths
    std::string current_output_dir = "";
    std::string working_directory_preview = "";
    std::string working_directory_cache = "";

    // other
    Eigen::Affine3d m_g = Eigen::Affine3d::Identity();
    std::vector<Point3Di> initial_points;
    double consecutive_distance = 0.0;
    std::vector<Point3Di> reference_points;
    NDTBucketMapType reference_buckets;
    double total_length_of_calculated_trajectory = 0.0;
    NDTBucketMapType buckets_indoor;
    NDTBucketMapType buckets_outdoor;
#if WITH_GUI == 1
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
#endif

    bool use_removie_imu_bias_from_first_stationary_scan = false;

    // ablation study
    bool ablation_study_use_planarity = false;
    bool ablation_study_use_norm = false;
    bool ablation_study_use_hierarchical_rgd = true;
    bool ablation_study_use_view_point_and_normal_vectors = true;
    bool save_index_pose = false;
};

uint64_t get_index(const int16_t x, const int16_t y, const int16_t z);
uint64_t get_rgd_index(const Eigen::Vector3d p, const Eigen::Vector3d b);

// this function finds interpolated pose between two poses according to query_time
Eigen::Matrix4d getInterpolatedPose(const std::map<double, Eigen::Matrix4d>& trajectory, double query_time);

// this function reduces number of points by preserving only first point for each bucket {bucket_x, bucket_y, bucket_z}
std::vector<Point3Di> decimate(const std::vector<Point3Di>& points, double bucket_x, double bucket_y, double bucket_z);

// this function updates each bucket (mean value, covariance) in regular grid decomposition
void update_rgd(
    const NDT::GridParameters& rgd_params,
    NDTBucketMapType& buckets,
    const std::vector<Point3Di>& points_global,
    const Eigen::Vector3d& viewport = Eigen::Vector3d(0, 0, 0));

void update_rgd_spherical_coordinates(
    const NDT::GridParameters& rgd_params,
    NDTBucketMapType& buckets,
    const std::vector<Point3Di>& points_global,
    const std::vector<Eigen::Vector3d>& points_global_spherical);

//! This function load inertial measurement unit data.
//! This function expects a file with the following format:
//! timestamp angular_velocity_x angular_velocity_y angular_velocity_z linear_acceleration_x linear_acceleration_y linear_acceleration_z
//! imu_id
//! @note imu_id is an optional column, if not present, it is assumed that all data comes from the same IMU.
//! @param imu_file - path to file with IMU data
//! @param imuToUse - id number of IMU to use, the same index as in pointcloud return by @ref load_point_cloud
//! @return vector of tuples (std::pair<timestamp, timestampUnix>, angular_velocity, linear_acceleration)
std::vector<std::tuple<std::pair<double, double>, FusionVector, FusionVector>> load_imu(const std::string& imu_file, int imuToUse);

//! This function load point cloud from LAS/LAZ file.
//! Optionally it can apply extrinsic calibration to each point.
//! The calibration is stored in a map, where key is laser scanner id.
//! The id of the laser scanner is stored in LAS/LAZ file as `user_data` field.
//! @param lazFile - path to file with point cloud
//! @param ommit_points_with_timestamp_equals_zero - if true, points with timestamp == 0 will be omited
//! @param filter_threshold_xy - threshold for filtering points in xy plane
//! @param calibrations - map of calibrations for each scanner key is scanner id.
//! @return vector of points of @ref Point3Di type
std::vector<Point3Di> load_point_cloud(
    const std::string& lazFile,
    bool ommit_points_with_timestamp_equals_zero,
    double filter_threshold_xy_inner,
    double filter_threshold_xy_outer,
    const std::unordered_map<int, Eigen::Affine3d>& calibrations);

bool save_poses(const std::string& file_name, const std::vector<Eigen::Affine3d>& m_poses, const std::vector<std::string>& filenames);

fs::path get_next_result_path(const std::string& working_directory);

// this function performs main LiDAR odometry calculations
void optimize_lidar_odometry(
    std::vector<Point3Di>& intermediate_points,
    std::vector<Eigen::Affine3d>& intermediate_trajectory,
    std::vector<Eigen::Affine3d>& intermediate_trajectory_motion_model,
    NDT::GridParameters& rgd_params_indoor,
    NDTBucketMapType& buckets_indoor,
    NDT::GridParameters& rgd_params_outdoor,
    NDTBucketMapType& buckets_outdoor,
    bool useMultithread,
    double max_distance,
    double& delta,
    double lm_factor,
    TaitBryanPose motion_model_correction,
    double lidar_odometry_motion_model_x_1_sigma_m,
    double lidar_odometry_motion_model_y_1_sigma_m,
    double lidar_odometry_motion_model_z_1_sigma_m,
    double lidar_odometry_motion_model_om_1_sigma_deg,
    double lidar_odometry_motion_model_fi_1_sigma_deg,
    double lidar_odometry_motion_model_ka_1_sigma_deg,
    double lidar_odometry_motion_model_fix_origin_x_1_sigma_m,
    double lidar_odometry_motion_model_fix_origin_y_1_sigma_m,
    double lidar_odometry_motion_model_fix_origin_z_1_sigma_m,
    double lidar_odometry_motion_model_fix_origin_om_1_sigma_deg,
    double lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg,
    double lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg,
    bool ablation_study_use_planarity,
    bool ablation_study_use_norm,
    bool ablation_study_use_hierarchical_rgd,
    bool ablation_study_use_view_point_and_normal_vectors);

void optimize_sf(
    std::vector<Point3Di>& intermediate_points,
    std::vector<Eigen::Affine3d>& intermediate_trajectory,
    std::vector<Eigen::Affine3d>& intermediate_trajectory_motion_model,
    NDT::GridParameters& rgd_params,
    NDTBucketMapType& buckets,
    bool useMultithread);

void optimize_sf2(
    std::vector<Point3Di>& intermediate_points,
    std::vector<Point3Di>& intermediate_points_sf,
    std::vector<Eigen::Affine3d>& intermediate_trajectory,
    const std::vector<Eigen::Affine3d>& intermediate_trajectory_motion_model,
    NDT::GridParameters& rgd_params,
    bool useMultithread,
    double wx,
    double wy,
    double wz,
    double wom,
    double wfi,
    double wka);

void optimize_icp(
    std::vector<Point3Di>& intermediate_points,
    std::vector<Eigen::Affine3d>& intermediate_trajectory,
    std::vector<Eigen::Affine3d>& intermediate_trajectory_motion_model,
    NDT::GridParameters& rgd_params,
    /*NDTBucketMapType &buckets*/ const std::vector<Point3Di>& points_global,
    bool useMultithread /*,
bool add_pitch_roll_constraint, const std::vector<std::pair<double, double>> &imu_roll_pitch*/
);

// this function registers initial point cloud to geoferenced point cloud
void align_to_reference(
    NDT::GridParameters& rgd_params, std::vector<Point3Di>& initial_points, Eigen::Affine3d& m_g, NDTBucketMapType& buckets);

// this function apply correction to pitch and roll
// void fix_ptch_roll(std::vector<WorkerData> &worker_data);

bool compute_step_2(
    std::vector<WorkerData>& worker_data,
    LidarOdometryParams& params,
    double& ts_failure,
    std::atomic<float>& loProgress,
    const std::atomic<bool>& pause,
    bool debugMsg);
void compute_step_2_fast_forward_motion(std::vector<WorkerData>& worker_data, LidarOdometryParams& params);

// for reconstructing worker data from step 1 output
bool loadLaz(
    const std::string& filename,
    std::vector<Point3Di>& points_out,
    const std::vector<int>& index_poses_i,
    const std::vector<Eigen::Affine3d>& intermediate_trajectory,
    const Eigen::Affine3d& inverse_pose);
bool load_poses(const fs::path& poses_file, std::vector<Eigen::Affine3d>& out_poses);
bool load_trajectory_csv(
    const std::string& filename,
    const Eigen::Affine3d& m_pose,
    std::vector<std::pair<double, double>>& intermediate_trajectory_timestamps,
    std::vector<Eigen::Affine3d>& intermediate_trajectory,
    std::vector<Eigen::Vector3d>& imu_om_fi_ka);
bool load_point_sizes(const std::filesystem::path& path, std::vector<int>& vector);
bool load_index_poses(const std::filesystem::path& path, std::vector<std::vector<int>>& index_poses_out);
bool load_worker_data_from_results(const fs::path& session_file, std::vector<WorkerData>& worker_data_out);

//! This namespace contains functions for loading calibration file (.json/.mjc and .sn).
//!
//! Calibration file is a json file with the following format:
//!{```json
//!    "calibration": {
//!      "47MDL9T0020193": {
//!        "identity" : "true"
//!      },
//!      "47MDL9S0020300":
//!          {
//!            "order" : "ROW",
//!            "inverted" : "TRUE",
//!            "data":[
//!             0.999824, 0.00466397, -0.0181595, -0.00425984,
//!             -0.0181478, -0.00254457, -0.999832, -0.151599,
//!              -0.0047094,0.999986, -0.00245948, -0.146408,
//!              0, 0, 0, 1
//!            ]
//!          }
//!    },
//!                    "imuToUse": "47MDL9T0020193"
//!}```
//! Json object `calibration` contains a map of calibration for each sensor.
//! The key of the map is serial number of the sensor.
//! The value is a json object with the following fields:
//! - `identity` - if true, the calibration is identity matrix
//!  - `order` - order of the matrix, can be `ROW` or `COLUMN`
//!  - `inverted` - if true, the calibration matrix is inverted
//!  - `data` - calibration matrix in given order
//! Json object `imuToUse` contains serial number of the sensor to use for IMU.
//! The JSON file contains mapping from sensor id to serial number to calibration.
//! The sensor id is the id of the point in LAS/LAZ file.
//!
//! The MANDEYE_CONTROLLER saves the sensor id in `user_data` field of LAZ file,
//! and also saves the serial number of the sensors in .sn file.
//! The .sn file is a text file with the following format:
//! ```text
//! 0 47MDL9T0020193
//! 1 47MDL9S0020300
//! ```
//! The first column is sensor id, the second column is serial number.
//! It is a mapping from lidarid (used in LAZ file) to serial number.
//! Those two files allows to apply calibration to each point in LAZ file.
namespace MLvxCalib
{
    //! Parse the calibration file and return a map from sensor id to serial number.
    //! Sensor id is the id of the point in laz file.
    //! Serial number is the serial number of the Livox.
    //! @param filename calibration file
    //! @return map of serial number, where key is sensor id.
    std::unordered_map<int, std::string> GetIdToSnMapping(const std::string& filename);

    //! Parse the calibration file and return a map from serial number to calibration.
    //! @param filename calibration file
    //! @return map of extrinsic calibration, where key is serial number of the lidar.
    std::unordered_map<std::string, Eigen::Affine3d> GetCalibrationFromFile(const std::string& filename);

    //! Parse the calibration file and return a serial number of the Livox to use for IMU.
    //! @param filename calibration file
    //! @return serial number of the Livox to use for IMU
    std::string GetImuSnToUse(const std::string& filename);

    //! Combine the id to serial number mapping and the calibration into a single map.
    //! The single map is from sensor id to calibration.
    //! @param idToSn mapping from serial number to Id number in pointcloud or IMU CSV
    //! @param calibration map of extrinsic calibration, where key is serial number of the lidar.
    //! @return map from sensor id to extrinsic calibration
    std::unordered_map<int, Eigen::Affine3d> CombineIntoCalibration(
        const std::unordered_map<int, std::string>& idToSn, const std::unordered_map<std::string, Eigen::Affine3d>& calibration);
    //! Get the id of the IMU to use.
    //! @param idToSn mapping from serial number to Id number in pointcloud or IMU CSV
    //! @param snToUse serial number of the Livox to use for IMU
    //! @return id of the IMU to use
    int GetImuIdToUse(const std::unordered_map<int, std::string>& idToSn, const std::string& snToUse);
} // namespace MLvxCalib

void Consistency(std::vector<WorkerData>& worker_data, const LidarOdometryParams& params);
void Consistency2(std::vector<WorkerData>& worker_data, const LidarOdometryParams& params);
