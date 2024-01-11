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

#include <glew.h>
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

struct WorkerData
{
    std::vector<Point3Di> intermediate_points;
    std::vector<Point3Di> original_points;
    std::vector<Eigen::Affine3d> intermediate_trajectory;
    std::vector<Eigen::Affine3d> intermediate_trajectory_motion_model;
    std::vector<double> intermediate_trajectory_timestamps;
    std::vector<std::pair<double, double>> imu_roll_pitch;
    bool show = false;
};

using NDTBucketMapType = std::unordered_map<uint64_t, NDT::Bucket>;

struct LidarOdometryParams
{
    double filter_threshold_xy = 0.5;
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

//this function load inertial measurement unit data
std::vector<std::tuple<double, FusionVector, FusionVector>> load_imu(const std::string &imu_file);

//this function load point cloud from LAS/LAZ file
std::vector<Point3Di> load_point_cloud(const std::string &lazFile, bool ommit_points_with_timestamp_equals_zero, double filter_threshold_xy);


bool saveLaz(const std::string &filename, const WorkerData &data);
bool saveLaz(const std::string &filename, const std::vector<Point3Di> &points_global);
bool save_poses(const std::string file_name, std::vector<Eigen::Affine3d> m_poses, std::vector<std::string> filenames);


// this function draws ellipse for each bucket
void draw_ellipse(const Eigen::Matrix3d &covar, const Eigen::Vector3d &mean, Eigen::Vector3f color, float nstd = 3);

// this function performs main LiDAR odometry calculations
void optimize(std::vector<Point3Di> &intermediate_points, std::vector<Eigen::Affine3d> &intermediate_trajectory,
              std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model,
              NDT::GridParameters &rgd_params, NDTBucketMapType &buckets, bool useMultithread,
              bool add_pitch_roll_constraint, const std::vector<std::pair<double, double>> &imu_roll_pitch);

// this function registers initial point cloud to geoferenced point cloud
void align_to_reference(NDT::GridParameters &rgd_params, std::vector<Point3Di> &initial_points, Eigen::Affine3d &m_g, NDTBucketMapType &buckets);

// this function apply correction to pitch and roll
void fix_ptch_roll(std::vector<WorkerData> &worker_data);

void compute_step_2(std::vector<WorkerData> &worker_data, LidarOdometryParams& params);
void compute_step_2_fast_forward_motion(std::vector<WorkerData> &worker_data, LidarOdometryParams &params);
#endif