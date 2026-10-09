#pragma once

#include <iomanip>
#include <sstream>

#include "lidar_odometry_utils.h"
#include "toml_io.h"
#include <HDMapping/HDMAPPING_ConfigureInfo.hpp>
#include <laszip/laszip_api.h>
#include <nlohmann/json.hpp>

#include <Core/export_laz.h>
#include <Core/session.h>

using Trajectory = std::map<double, std::tuple<Eigen::Matrix4d, double, RawIMUData>>;
using Imu = std::vector<std::tuple<std::pair<double, double>, Eigen::Vector3f, Eigen::Vector3f>>;

bool load_data(
    std::vector<std::string>& input_file_names,
    LidarOdometryParams& params,
    std::vector<std::vector<Point3Di>>& pointsPerFile,
    Imu& imu_data,
    bool debugMsg);
void calculate_trajectory(Trajectory& trajectory, Imu& imu_data, LidarOdometryParams& params, bool debugMsg);
// pointsPerFile is read, and with lazy_load_raw_clouds loaded and freed file by file
bool compute_step_1(
    std::vector<std::vector<Point3Di>>& pointsPerFile,
    LidarOdometryParams& params,
    Trajectory& trajectory,
    std::vector<WorkerData>& worker_data,
    const std::atomic<bool>& pause);
// after a successful step 1: free the raw clouds (and the lazy loader) — nothing after step 1 reads them
void release_raw_clouds(std::vector<std::vector<Point3Di>>& pointsPerFile, LidarOdometryParams& params);
// remove params.working_directory_cache; call only once nothing reads worker_data[*].*_cache_file_name any more
void remove_cache(const LidarOdometryParams& params);
void run_consistency(std::vector<WorkerData>& worker_data, const LidarOdometryParams& params);
void filter_reference_buckets(LidarOdometryParams& params);
void load_reference_point_clouds(const std::vector<std::string>& input_file_names, LidarOdometryParams& params);
void save_result(std::vector<WorkerData>& worker_data, LidarOdometryParams& params, fs::path outwd, double elapsed_seconds);
void save_parameters_toml(const LidarOdometryParams& params, const fs::path& outwd, double elapsed_seconds);
void save_processing_results_json(const LidarOdometryParams& params, const fs::path& outwd, double elapsed_seconds);
void save_trajectory_to_ascii(std::vector<WorkerData>& worker_data, std::string output_file_name);
std::string save_results_automatic(
    LidarOdometryParams& params, std::vector<WorkerData>& worker_data, const std::string& working_directory, double elapsed_seconds);
std::vector<WorkerData> run_lidar_odometry(const std::string& input_dir, LidarOdometryParams& params);
// bool SaveParametersToTomlFile(const std::string &filepath, const LidarOdometryParams &params);
// bool LoadParametersFromTomlFile(const std::string &filepath, LidarOdometryParams &params);
