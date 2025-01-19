#ifndef _LIDAR_ODOMETRY_UTILS_H_
#define _LIDAR_ODOMETRY_UTILS_H_

#include <portable-file-dialogs.h>

#include <laszip/laszip_api.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <Fusion.h>
#include <map>
#include <execution>

#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <ImGuizmo.h>
#include <imgui_internal.h>

#include <GL/glew.h>
#include <GL/freeglut.h>

#include <structures.h>
#include <ndt.h>

#include <transformations.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>
#include <chrono>
#include <python-scripts/constraints/smoothness_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian_simplified.h>
#include <python-scripts/constraints/constraint_fixed_parameter_jacobian.h>
#include <common/include/cauchy.h>
#include <python-scripts/point-to-feature-metrics/point_to_line_tait_bryan_wc_jacobian.h>
#include <pfd_wrapper.hpp>

struct WorkerData
{
    std::vector<Point3Di> intermediate_points;
    std::vector<Point3Di> original_points;
    std::vector<Eigen::Affine3d> intermediate_trajectory;
    std::vector<Eigen::Affine3d> intermediate_trajectory_motion_model;
    std::vector<std::pair<double, double>> intermediate_trajectory_timestamps;
    std::vector<Eigen::Vector3d> imu_om_fi_ka;
    bool show = false;
};

using NDTBucketMapType = std::unordered_map<uint64_t, NDT::Bucket>;
using NDTBucketMapType2 = std::unordered_map<uint64_t, NDT::Bucket2>;

struct LidarOdometryParams
{
    double filter_threshold_xy = 0.3;
    Eigen::Affine3d m_g = Eigen::Affine3d::Identity();
    std::vector<Point3Di> initial_points;
    NDT::GridParameters in_out_params;
    NDTBucketMapType buckets;
    bool use_motion_from_previous_step = true;
    double consecutive_distance = 0.0;
    int nr_iter = 100;
    bool useMultithread = true;
    std::vector<Point3Di> reference_points;
    double decimation = 0.1;
    NDTBucketMapType reference_buckets;
    std::string working_directory_preview = "";
    double sliding_window_trajectory_length_threshold = 50.0;
    bool save_calibration_validation = true;
    int calibration_validation_points = 1000000;

    //rgd_sf
    bool use_robust_and_accurate_lidar_odometry = false;
    double distance_bucket = 0.5;
    double polar_angle_deg = 5.0;
    double azimutal_angle_deg = 5.0;
    int robust_and_accurate_lidar_odometry_iterations = 20;
    double max_distance_lidar = 30.0;
};

unsigned long long int get_index(const int16_t x, const int16_t y, const int16_t z);
unsigned long long int get_rgd_index(const Eigen::Vector3d p, const Eigen::Vector3d b);

// this function finds interpolated pose between two poses according to query_time
Eigen::Matrix4d getInterpolatedPose(const std::map<double, Eigen::Matrix4d> &trajectory, double query_time);

// this function reduces number of points by preserving only first point for each bucket {bucket_x, bucket_y, bucket_z}
std::vector<Point3Di> decimate(const std::vector<Point3Di> &points, double bucket_x, double bucket_y, double bucket_z);

// this function updates each bucket (mean value, covariance) in regular grid decomposition
void update_rgd(NDT::GridParameters &rgd_params, NDTBucketMapType &buckets,
                std::vector<Point3Di> &points_global, Eigen::Vector3d viewport = Eigen::Vector3d(0, 0, 0));

void update_rgd_spherical_coordinates(NDT::GridParameters &rgd_params, NDTBucketMapType &buckets,
                                      std::vector<Point3Di> &points_global, std::vector<Eigen::Vector3d> &points_global_spherical, Eigen::Vector3d viewport);

//! This function load inertial measurement unit data.
//! This function expects a file with the following format:
//! timestamp angular_velocity_x angular_velocity_y angular_velocity_z linear_acceleration_x linear_acceleration_y linear_acceleration_z imu_id
//! @note imu_id is an optional column, if not present, it is assumed that all data comes from the same IMU.
//! @param imu_file - path to file with IMU data
//! @param imuToUse - id number of IMU to use, the same index as in pointcloud return by @ref load_point_cloud
//! @return vector of tuples (std::pair<timestamp, timestampUnix>, angular_velocity, linear_acceleration)
std::vector<std::tuple<std::pair<double, double>, FusionVector, FusionVector>> load_imu(const std::string &imu_file, int imuToUse);

//! This function load point cloud from LAS/LAZ file.
//! Optionally it can apply extrinsic calibration to each point.
//! The calibration is stored in a map, where key is laser scanner id.
//! The id of the laser scanner is stored in LAS/LAZ file as `user_data` field.
//! @param lazFile - path to file with point cloud
//! @param ommit_points_with_timestamp_equals_zero - if true, points with timestamp == 0 will be omited
//! @param filter_threshold_xy - threshold for filtering points in xy plane
//! @param calibrations - map of calibrations for each scanner key is scanner id.
//! @return vector of points of @ref Point3Di type
std::vector<Point3Di> load_point_cloud(const std::string &lazFile, bool ommit_points_with_timestamp_equals_zero, double filter_threshold_xy,
                                       const std::unordered_map<int, Eigen::Affine3d> &calibrations);

bool saveLaz(const std::string &filename, const WorkerData &data, double threshould_output_filter);
bool saveLaz(const std::string &filename, const std::vector<Point3Di> &points_global);
bool save_poses(const std::string file_name, std::vector<Eigen::Affine3d> m_poses, std::vector<std::string> filenames);

// this function draws ellipse for each bucket
void draw_ellipse(const Eigen::Matrix3d &covar, const Eigen::Vector3d &mean, Eigen::Vector3f color, float nstd = 3);

// this function performs main LiDAR odometry calculations
void optimize(std::vector<Point3Di> &intermediate_points, std::vector<Eigen::Affine3d> &intermediate_trajectory,
              std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model,
              NDT::GridParameters &rgd_params, NDTBucketMapType &buckets, bool useMultithread /*,
               bool add_pitch_roll_constraint, const std::vector<std::pair<double, double>> &imu_roll_pitch*/
);

void optimize_sf(std::vector<Point3Di> &intermediate_points, std::vector<Eigen::Affine3d> &intermediate_trajectory,
              std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model,
              NDT::GridParameters &rgd_params, NDTBucketMapType &buckets, bool useMultithread );

void optimize_sf2(std::vector<Point3Di> &intermediate_points, std::vector<Point3Di> &intermediate_points_sf, std::vector<Eigen::Affine3d> &intermediate_trajectory,
                  const std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model,
                  NDT::GridParameters &rgd_params, bool useMultithread, double wx, double wy, double wz, double wom, double wfi, double wka);

void optimize_icp(std::vector<Point3Di> &intermediate_points, std::vector<Eigen::Affine3d> &intermediate_trajectory,
                  std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model,
                  NDT::GridParameters &rgd_params, /*NDTBucketMapType &buckets*/ std::vector<Point3Di> points_global, bool useMultithread /*,
                   bool add_pitch_roll_constraint, const std::vector<std::pair<double, double>> &imu_roll_pitch*/
);

// this function registers initial point cloud to geoferenced point cloud
void align_to_reference(NDT::GridParameters &rgd_params, std::vector<Point3Di> &initial_points, Eigen::Affine3d &m_g, NDTBucketMapType &buckets);

// this function apply correction to pitch and roll
// void fix_ptch_roll(std::vector<WorkerData> &worker_data);

bool compute_step_2(std::vector<WorkerData> &worker_data, LidarOdometryParams &params, double &ts_failure);
void compute_step_2_fast_forward_motion(std::vector<WorkerData> &worker_data, LidarOdometryParams &params);

//! This namespace contains functions for loading calibration file (.json and .sn).
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
    //! Sensor id is the id is id of the point in laz file.
    //! Serial number is the serial number of the Livox.
    //! @param filename calibration file
    //! @return map of serial number, where key is sensor id.
    std::unordered_map<int, std::string> GetIdToSnMapping(const std::string &filename);

    //! Parse the calibration file and return a map from serial number to calibration.
    //! @param filename calibration file
    //! @return map of extrinsic calibration, where key is serial number of the lidar.
    std::unordered_map<std::string, Eigen::Affine3d> GetCalibrationFromFile(const std::string &filename);

    //! Parse the calibration file and return a serial number of the Livox to use for IMU.
    //! @param filename calibration file
    //! @return serial number of the Livox to use for IMU
    std::string GetImuSnToUse(const std::string &filename);

    //! Combine the id to serial number mapping and the calibration into a single map.
    //! The single map is from sensor id to calibration.
    //! @param idToSn mapping from serial number to Id number in pointcloud or IMU CSV
    //! @param calibration map of extrinsic calibration, where key is serial number of the lidar.
    //! @return map from sensor id to extrinsic calibration
    std::unordered_map<int, Eigen::Affine3d> CombineIntoCalibration(const std::unordered_map<int, std::string> &idToSn,
                                                                    const std::unordered_map<std::string, Eigen::Affine3d> &calibration);
    //! Get the id of the IMU to use.
    //! @param idToSn mapping from serial number to Id number in pointcloud or IMU CSV
    //! @param snToUse serial number of the Livox to use for IMU
    //! @return id of the IMU to use
    int GetImuIdToUse(const std::unordered_map<int, std::string> &idToSn, const std::string &snToUse);
}

void Consistency(std::vector<WorkerData> &worker_data, LidarOdometryParams &params);
void Consistency2(std::vector<WorkerData> &worker_data, LidarOdometryParams &params);
#endif
