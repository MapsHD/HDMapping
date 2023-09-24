#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <Fusion.h>
#include <map>
#include <execution>

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
#include <string.h>
namespace mandeye
{


    struct SlamConfig
    {
        bool fusionConventionNwu{ true };
        bool fusionConventionEnu{ false };
        bool fusionConventionNed{ false };
        bool use_motion_from_previous_step{ true };
        bool quiet{ false };
        double sample_peroid{ 1.0 / 200.0 }; //!< Imu Sample peroid
        double decimation{ 0.1 }; //!< Decimation, 0.0 to disable
        std::string tempSave{};
        int threshold_initial_points{ 100000 };
        unsigned int nr_iter{ 100 };
        bool useMultithread{ true };
        double sliding_window_trajectory_length_threshold = 50.0;
    };

    using InputImuData = std::vector<std::tuple<double, FusionVector, FusionVector>>;
    using InputPointCloudData = std::vector<Point3Di>;

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


    void optimizeTrajectory(const InputImuData& imu, const InputPointCloudData& initialPoints, const SlamConfig& config);

    using NDTBucketMapType = std::unordered_map<uint64_t, NDT::Bucket>;

    Eigen::Matrix4d getInterpolatedPose(const std::map<double, Eigen::Matrix4d>& trajectory, double query_time);

    std::vector<Point3Di> decimate(const std::vector<Point3Di>& points, double bucket_x, double bucket_y, double bucket_z);

    void optimize(std::vector<Point3Di>& intermediate_points, std::vector<Eigen::Affine3d>& intermediate_trajectory,
        std::vector<Eigen::Affine3d>& intermediate_trajectory_motion_model,
        NDT::GridParameters& rgd_params, NDTBucketMapType& buckets, bool multithread,
        bool add_pitch_roll_constraint, const std::vector<std::pair<double, double>>& imu_roll_pitch);

    void update_rgd(NDT::GridParameters& rgd_params, NDTBucketMapType& buckets, std::vector<Point3Di>& points_global);

    void align_to_reference(NDT::GridParameters& rgd_params, std::vector<Point3Di>& initial_points, Eigen::Affine3d& m_g, NDTBucketMapType& reference_buckets);

    inline unsigned long long int get_index(const int16_t x, const int16_t y, const int16_t z)
    {
        return ((static_cast<unsigned long long int>(x) << 32) & (0x0000FFFF00000000ull)) |
            ((static_cast<unsigned long long int>(y) << 16) & (0x00000000FFFF0000ull)) |
            ((static_cast<unsigned long long int>(z) << 0) & (0x000000000000FFFFull));
    }

    inline unsigned long long int get_rgd_index(const Eigen::Vector3d& p, const Eigen::Vector3d& b)
    {
        const int16_t x = static_cast<int16_t>(p.x() / b.x());
        const int16_t y = static_cast<int16_t>(p.y() / b.y());
        const int16_t z = static_cast<int16_t>(p.z() / b.z());
        return get_index(x, y, z);
    }

    
}
