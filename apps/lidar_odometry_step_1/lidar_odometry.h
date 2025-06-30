#ifndef _LIDAR_ODOMETRY_H_
#define _LIDAR_ODOMETRY_H_

#include "lidar_odometry_utils.h"
#include <session.h>
#include <HDMapping/Version.hpp>
#include <laszip/laszip_api.h>
#include <export_laz.h>
#include <nlohmann/json.hpp>

//#define SAMPLE_PERIOD (1.0 / 200.0)

using Trajectory = std::map<double, std::pair<Eigen::Matrix4d, double>>;
using Imu = std::vector<std::tuple<std::pair<double, double>, FusionVector, FusionVector>>;

bool load_data(std::vector<std::string>& input_file_names, LidarOdometryParams& params, std::vector<std::vector<Point3Di>>& pointsPerFile, Imu& imu_data);
void calculate_trajectory(
    Trajectory& trajectory, Imu& imu_data, bool fusionConventionNwu, bool fusionConventionEnu, bool fusionConventionNed, double ahrs_gain);
bool compute_step_1(
    std::vector<std::vector<Point3Di>>& pointsPerFile, LidarOdometryParams& params, 
    Trajectory& trajectory, std::vector<WorkerData>& worker_data);
void run_consistency(std::vector<WorkerData>& worker_data, LidarOdometryParams& params);
void filter_reference_buckets(LidarOdometryParams& params);
void load_reference_point_clouds(std::vector<std::string> input_file_names, LidarOdometryParams& params);
void save_result(std::vector<WorkerData> &worker_data, LidarOdometryParams &params, fs::path outwd, double elapsed_seconds);
void save_trajectory_to_ascii(std::vector<WorkerData>& worker_data, std::string output_file_name);
void save_all_to_las(
    std::vector<WorkerData>& worker_data, LidarOdometryParams& params, std::string output_file_name, Session& session,
    bool export_selected = false, bool filter_on_export = true, bool apply_pose = true, bool add_to_pc_container = false);
std::string save_results_automatic(
    LidarOdometryParams &params, std::vector<WorkerData> &worker_data, std::string working_directory, double elapsed_seconds);
std::vector<WorkerData> run_lidar_odometry(std::string input_dir, LidarOdometryParams& params);
#endif