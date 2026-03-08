#pragma once

#include <map>
#include <toml++/toml.hpp>
#include <variant>

#include "lidar_odometry_utils.h"

class TomlIO
{
public:
    TomlIO()
    {
        ;
    };
    ~TomlIO()
    {
        ;
    };

    using ParamValue = std::
        variant<bool LidarOdometryParams::*, double LidarOdometryParams::*, int LidarOdometryParams::*, std::string LidarOdometryParams::*>;

    const std::map<std::string, ParamValue> POINTERS = {
        // version information
        { "software_version", &LidarOdometryParams::software_version },
        { "config_version", &LidarOdometryParams::config_version },
        { "build_date", &LidarOdometryParams::build_date },

        { "useMultithread", &LidarOdometryParams::useMultithread },
        { "real_time_threshold_seconds", &LidarOdometryParams::real_time_threshold_seconds },
        { "filter_threshold_xy_inner", &LidarOdometryParams::filter_threshold_xy_inner },
        { "filter_threshold_xy_outer", &LidarOdometryParams::filter_threshold_xy_outer },
        { "decimation", &LidarOdometryParams::decimation },
        { "threshould_output_filter", &LidarOdometryParams::threshould_output_filter },
        { "min_counter_concatenated_trajectory_nodes", &LidarOdometryParams::min_counter_concatenated_trajectory_nodes },
        { "fusionConventionNwu", &LidarOdometryParams::fusionConventionNwu },
        { "fusionConventionEnu", &LidarOdometryParams::fusionConventionEnu },
        { "fusionConventionNed", &LidarOdometryParams::fusionConventionNed },
        { "vqf_tauAcc", &LidarOdometryParams::vqf_tauAcc },
        { "vqf_motionBiasEstEnabled", &LidarOdometryParams::vqf_motionBiasEstEnabled },
        { "vqf_restBiasEstEnabled", &LidarOdometryParams::vqf_restBiasEstEnabled },
        { "vqf_biasSigmaInit", &LidarOdometryParams::vqf_biasSigmaInit },
        { "vqf_biasForgettingTime", &LidarOdometryParams::vqf_biasForgettingTime },
        { "vqf_biasClip", &LidarOdometryParams::vqf_biasClip },
        { "vqf_biasSigmaMotion", &LidarOdometryParams::vqf_biasSigmaMotion },
        { "vqf_biasVerticalForgettingFactor", &LidarOdometryParams::vqf_biasVerticalForgettingFactor },
        { "vqf_biasSigmaRest", &LidarOdometryParams::vqf_biasSigmaRest },
        { "vqf_restMinT", &LidarOdometryParams::vqf_restMinT },
        { "vqf_restFilterTau", &LidarOdometryParams::vqf_restFilterTau },
        { "vqf_restThGyr", &LidarOdometryParams::vqf_restThGyr },
        { "vqf_restThAcc", &LidarOdometryParams::vqf_restThAcc },
        { "vqf_useMagnetometer", &LidarOdometryParams::vqf_useMagnetometer },
        { "vqf_tauMag", &LidarOdometryParams::vqf_tauMag },
        { "vqf_magDistRejectionEnabled", &LidarOdometryParams::vqf_magDistRejectionEnabled },
        { "vqf_magCurrentTau", &LidarOdometryParams::vqf_magCurrentTau },
        { "vqf_magRefTau", &LidarOdometryParams::vqf_magRefTau },
        { "vqf_magNormTh", &LidarOdometryParams::vqf_magNormTh },
        { "vqf_magDipTh", &LidarOdometryParams::vqf_magDipTh },
        { "vqf_magNewTime", &LidarOdometryParams::vqf_magNewTime },
        { "vqf_magNewFirstTime", &LidarOdometryParams::vqf_magNewFirstTime },
        { "vqf_magNewMinGyr", &LidarOdometryParams::vqf_magNewMinGyr },
        { "vqf_magMinUndisturbedTime", &LidarOdometryParams::vqf_magMinUndisturbedTime },
        { "vqf_magMaxRejectionTime", &LidarOdometryParams::vqf_magMaxRejectionTime },
        { "vqf_magRejectionFactor", &LidarOdometryParams::vqf_magRejectionFactor },
        { "use_motion_from_previous_step", &LidarOdometryParams::use_motion_from_previous_step },
        { "nr_iter", &LidarOdometryParams::nr_iter },
        { "sliding_window_trajectory_length_threshold", &LidarOdometryParams::sliding_window_trajectory_length_threshold },
        { "max_distance_lidar", &LidarOdometryParams::max_distance_lidar },
        { "threshold_initial_points", &LidarOdometryParams::threshold_initial_points },
        { "threshold_nr_poses", &LidarOdometryParams::threshold_nr_poses },
        { "convergence_delta_threshold", &LidarOdometryParams::convergence_delta_threshold },
        { "save_calibration_validation", &LidarOdometryParams::save_calibration_validation },
        { "calibration_validation_points", &LidarOdometryParams::calibration_validation_points },
        { "num_constistency_iter", &LidarOdometryParams::num_constistency_iter },
        { "use_mutliple_gaussian", &LidarOdometryParams::use_mutliple_gaussian },
        { "lidar_odometry_motion_model_x_1_sigma_m", &LidarOdometryParams::lidar_odometry_motion_model_x_1_sigma_m },
        { "lidar_odometry_motion_model_y_1_sigma_m", &LidarOdometryParams::lidar_odometry_motion_model_y_1_sigma_m },
        { "lidar_odometry_motion_model_z_1_sigma_m", &LidarOdometryParams::lidar_odometry_motion_model_z_1_sigma_m },
        { "lidar_odometry_motion_model_om_1_sigma_deg", &LidarOdometryParams::lidar_odometry_motion_model_om_1_sigma_deg },
        { "lidar_odometry_motion_model_fi_1_sigma_deg", &LidarOdometryParams::lidar_odometry_motion_model_fi_1_sigma_deg },
        { "lidar_odometry_motion_model_ka_1_sigma_deg", &LidarOdometryParams::lidar_odometry_motion_model_ka_1_sigma_deg },
        { "lidar_odometry_motion_model_fix_origin_x_1_sigma_m", &LidarOdometryParams::lidar_odometry_motion_model_fix_origin_x_1_sigma_m },
        { "lidar_odometry_motion_model_fix_origin_y_1_sigma_m", &LidarOdometryParams::lidar_odometry_motion_model_fix_origin_y_1_sigma_m },
        { "lidar_odometry_motion_model_fix_origin_z_1_sigma_m", &LidarOdometryParams::lidar_odometry_motion_model_fix_origin_z_1_sigma_m },
        { "lidar_odometry_motion_model_fix_origin_om_1_sigma_deg",
          &LidarOdometryParams::lidar_odometry_motion_model_fix_origin_om_1_sigma_deg },
        { "lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg",
          &LidarOdometryParams::lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg },
        { "lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg",
          &LidarOdometryParams::lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg },
        { "use_robust_and_accurate_lidar_odometry", &LidarOdometryParams::use_robust_and_accurate_lidar_odometry },
        { "distance_bucket", &LidarOdometryParams::distance_bucket },
        { "polar_angle_deg", &LidarOdometryParams::polar_angle_deg },
        { "azimutal_angle_deg", &LidarOdometryParams::azimutal_angle_deg },
        { "robust_and_accurate_lidar_odometry_iterations", &LidarOdometryParams::robust_and_accurate_lidar_odometry_iterations },
        { "distance_bucket_rigid_sf", &LidarOdometryParams::distance_bucket_rigid_sf },
        { "polar_angle_deg_rigid_sf", &LidarOdometryParams::polar_angle_deg_rigid_sf },
        { "azimutal_angle_deg_rigid_sf", &LidarOdometryParams::azimutal_angle_deg_rigid_sf },
        { "robust_and_accurate_lidar_odometry_rigid_sf_iterations",
          &LidarOdometryParams::robust_and_accurate_lidar_odometry_rigid_sf_iterations },
        { "rgd_sf_sigma_x_m", &LidarOdometryParams::rgd_sf_sigma_x_m },
        { "rgd_sf_sigma_y_m", &LidarOdometryParams::rgd_sf_sigma_y_m },
        { "rgd_sf_sigma_z_m", &LidarOdometryParams::rgd_sf_sigma_z_m },
        { "rgd_sf_sigma_om_deg", &LidarOdometryParams::rgd_sf_sigma_om_deg },
        { "rgd_sf_sigma_fi_deg", &LidarOdometryParams::rgd_sf_sigma_fi_deg },
        { "rgd_sf_sigma_ka_deg", &LidarOdometryParams::rgd_sf_sigma_ka_deg },
        { "max_distance_lidar_rigid_sf", &LidarOdometryParams::max_distance_lidar_rigid_sf },
        { "current_output_dir", &LidarOdometryParams::current_output_dir },
        { "working_directory_preview", &LidarOdometryParams::working_directory_preview },
        { "use_imu_preintegration", &LidarOdometryParams::use_imu_preintegration },
        { "imu_preintegration_method", &LidarOdometryParams::imu_preintegration_method }
    };

    // Special handling for TaitBryanPose members
    const std::map<std::string, double TaitBryanPose::*> MOTION_MODEL_CORRECTION_POINTERS = {
        { "motion_model_correction_om", &TaitBryanPose::om },
        { "motion_model_correction_fi", &TaitBryanPose::fi },
        { "motion_model_correction_ka", &TaitBryanPose::ka }
    };

    std::map<std::string, std::vector<std::string>> CATEGORIES = {
        { "version_info", { "software_version", "config_version", "build_date" } },
        { "performance", { "useMultithread", "real_time_threshold_seconds" } },
        { "filter_points",
          { "filter_threshold_xy_inner",
            "filter_threshold_xy_outer",
            "decimation",
            "threshould_output_filter",
            "min_counter_concatenated_trajectory_nodes" } },
        { "ahrs_vqf", {
            "fusionConventionNwu", "fusionConventionEnu", "fusionConventionNed",
            "vqf_tauAcc",
            "vqf_motionBiasEstEnabled", "vqf_restBiasEstEnabled",
            "vqf_biasSigmaInit", "vqf_biasForgettingTime", "vqf_biasClip",
            "vqf_biasSigmaMotion", "vqf_biasVerticalForgettingFactor", "vqf_biasSigmaRest",
            "vqf_restMinT", "vqf_restFilterTau", "vqf_restThGyr", "vqf_restThAcc",
            "vqf_useMagnetometer", "vqf_tauMag", "vqf_magDistRejectionEnabled",
            "vqf_magCurrentTau", "vqf_magRefTau", "vqf_magNormTh", "vqf_magDipTh",
            "vqf_magNewTime", "vqf_magNewFirstTime", "vqf_magNewMinGyr",
            "vqf_magMinUndisturbedTime", "vqf_magMaxRejectionTime", "vqf_magRejectionFactor"
        } },
        { "lidar_odometry_control",
          { "use_motion_from_previous_step",
            "nr_iter",
            "sliding_window_trajectory_length_threshold",
            "max_distance_lidar",
            "threshold_initial_points",
            "threshold_nr_poses",
            "convergence_delta_threshold" } },
        { "lidar_odometry_debug_info", { "save_calibration_validation", "calibration_validation_points" } },
        { "consistency", { "num_constistency_iter", "use_mutliple_gaussian" } },
        { "motion_model_uncertainty",
          { "lidar_odometry_motion_model_x_1_sigma_m",
            "lidar_odometry_motion_model_y_1_sigma_m",
            "lidar_odometry_motion_model_z_1_sigma_m",
            "lidar_odometry_motion_model_om_1_sigma_deg",
            "lidar_odometry_motion_model_fi_1_sigma_deg",
            "lidar_odometry_motion_model_ka_1_sigma_deg" } },
        { "motion_model_correction", { "motion_model_correction_om", "motion_model_correction_fi", "motion_model_correction_ka" } },
        { "motion_model_first_node_prior_uncertainty",
          { "lidar_odometry_motion_model_fix_origin_x_1_sigma_m",
            "lidar_odometry_motion_model_fix_origin_y_1_sigma_m",
            "lidar_odometry_motion_model_fix_origin_z_1_sigma_m",
            "lidar_odometry_motion_model_fix_origin_om_1_sigma_deg",
            "lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg",
            "lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg" } },
        { "robust_lidar_odometry",
          { "use_robust_and_accurate_lidar_odometry",
            "distance_bucket",
            "polar_angle_deg",
            "azimutal_angle_deg",
            "robust_and_accurate_lidar_odometry_iterations",
            "distance_bucket_rigid_sf",
            "polar_angle_deg_rigid_sf",
            "azimutal_angle_deg_rigid_sf",
            "robust_and_accurate_lidar_odometry_rigid_sf_iterations",
            "max_distance_lidar_rigid_sf",
            "rgd_sf_sigma_x_m",
            "rgd_sf_sigma_y_m",
            "rgd_sf_sigma_z_m",
            "rgd_sf_sigma_om_deg",
            "rgd_sf_sigma_fi_deg",
            "rgd_sf_sigma_ka_deg" } },
        { "imu_preintegration", { "use_imu_preintegration", "imu_preintegration_method" } },
        { "paths", { "current_output_dir", "working_directory_preview" } },
        { "misc", { "clear_color" } }
    };

    bool SaveParametersToTomlFile(const std::string& filepath, const LidarOdometryParams& params);

    // Version validation and handling functions
    struct VersionInfo
    {
        std::string software_version;
        std::string config_version;
        std::string build_date;
        bool found = false;
    };

    VersionInfo CheckConfigVersion(const std::string& filepath);
    bool IsVersionCompatible(const std::string& file_version, const std::string& current_version);
    void HandleMissingVersion(LidarOdometryParams& params);

    template<typename T>
    void set_if_exists(NDT::GridParameters& grid, const toml::table* tbl, const std::string& key, T NDT::GridParameters::* member);
    void read_grid_params(NDT::GridParameters& grid, const toml::table* grid_tbl);
    bool LoadParametersFromTomlFile(const std::string& filepath, LidarOdometryParams& params);
};
