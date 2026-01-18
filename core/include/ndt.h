#pragma once

#include <Eigen/Eigen>
#include <point_cloud.h>
#include <session.h>
#include <thread>

class NDT
{
public:
    struct GridParameters
    {
        double bounding_box_min_X;
        double bounding_box_min_Y;
        double bounding_box_min_Z;
        double bounding_box_max_X;
        double bounding_box_max_Y;
        double bounding_box_max_Z;
        double bounding_box_extension;
        int number_of_buckets_X;
        int number_of_buckets_Y;
        int number_of_buckets_Z;
        uint64_t number_of_buckets;
        double resolution_X;
        double resolution_Y;
        double resolution_Z;
    };

    struct PointBucketIndexPair
    {
        int index_of_point;
        uint64_t index_of_bucket;
        int index_pose;
    };

    struct Bucket
    {
        uint64_t index_begin;
        uint64_t index_end;
        uint64_t number_of_points;
        Eigen::Vector3d mean;
        Eigen::Matrix3d cov;
        Eigen::Vector3d normal_vector;
    };

    struct BucketCoef
    {
        Eigen::Vector3d mean;
        Eigen::Matrix3d cov;
        Eigen::Vector3d normal_vector;
        std::vector<uint64_t> point_indexes;
        bool valid = false;
    };

    struct Bucket2
    {
        int classification = 0; // 1 - ceiling, 2 - floor
        uint64_t index_begin_inclusive;
        uint64_t index_end_exclusive;
        // uint64_t number_of_points;
        std::unordered_map<uint64_t, BucketCoef> buckets;
    };

    struct Job
    {
        uint64_t index_begin_inclusive;
        uint64_t index_end_exclusive;
    };

    enum PoseConvention
    {
        cw,
        wc
    };
    enum OptimizationAlgorithm
    {
        gauss_newton,
        levenberg_marguardt
    };
    enum RotationMatrixParametrization
    {
        tait_bryan_xyz,
        rodrigues,
        quaternion,
        lie_algebra_left_jacobian,
        lie_algebra_right_jacobian
    };

    NDT()
    {
        bucket_size[0] = 0.3;
        bucket_size[1] = 0.3;
        bucket_size[2] = 0.3;
        bucket_size_external[0] = 5.0;
        bucket_size_external[1] = 5.0;
        bucket_size_external[2] = 5.0;
        number_of_threads = std::thread::hardware_concurrency();
        number_of_iterations = 30;
    }

    ~NDT() = default;

    void grid_calculate_params(const std::vector<Point3D>& point_cloud_global, GridParameters& in_out_params);
    void grid_calculate_params(const std::vector<Point3Di>& point_cloud_global, GridParameters& in_out_params);
    void grid_calculate_params(
        GridParameters& in_out_params, double min_x, double max_x, double min_y, double max_y, double min_z, double max_z);

    void build_rgd(
        std::vector<Point3D>& points,
        std::vector<PointBucketIndexPair>& index_pair,
        std::vector<Bucket>& buckets,
        GridParameters& rgd_params,
        int num_threads = std::thread::hardware_concurrency());
    void build_rgd(
        std::vector<Point3Di>& points,
        std::vector<PointBucketIndexPair>& index_pair,
        std::vector<Bucket>& buckets,
        GridParameters& rgd_params,
        int num_threads = std::thread::hardware_concurrency());
    std::vector<Job> get_jobs(uint64_t size, int num_threads = std::thread::hardware_concurrency());
    void reindex(
        std::vector<Point3D>& points, std::vector<NDT::PointBucketIndexPair>& index_pair, NDT::GridParameters& rgd_params, int num_threads);
    void reindex(
        std::vector<Point3Di>& points,
        std::vector<NDT::PointBucketIndexPair>& index_pair,
        NDT::GridParameters& rgd_params,
        int num_threads);

    bool optimize(std::vector<PointCloud>& point_clouds, bool compute_only_mahalanobis_distance, bool compute_mean_and_cov_for_bucket);
    bool optimize(std::vector<Session>& sessions, bool compute_only_mahalanobis_distance, bool compute_mean_and_cov_for_bucket);

    std::vector<Eigen::SparseMatrix<double>> compute_covariance_matrices_and_rms(std::vector<PointCloud>& point_clouds, double& rms);

    bool optimize(
        std::vector<PointCloud>& point_clouds, double& rms_initial, double& rms_final, double& mui, bool compute_mean_and_cov_for_bucket);

    // std::vector<Session> sessions;

    bool optimize_lie_algebra_left_jacobian(std::vector<PointCloud>& point_clouds, bool compute_mean_and_cov_for_bucket);
    bool optimize_lie_algebra_right_jacobian(std::vector<PointCloud>& point_clouds, bool compute_mean_and_cov_for_bucket);
    // std::vector<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>> compute_covariance_matrices7x7(PointClouds& point_clouds_container);

    bool compute_cov_mean(
        std::vector<Point3D>& points,
        std::vector<PointBucketIndexPair>& index_pair,
        std::vector<Bucket>& buckets,
        GridParameters& rgd_params,
        double min_x,
        double max_x,
        double min_y,
        double max_y,
        double min_z,
        double max_z,
        int num_threads = std::thread::hardware_concurrency());

    bool compute_cov_mean(
        std::vector<Point3Di>& points,
        std::vector<PointBucketIndexPair>& index_pair,
        std::vector<Bucket>& buckets,
        GridParameters& rgd_params,
        int num_threads = std::thread::hardware_concurrency());

    bool compute_cov_mean(
        std::vector<Point3Di>& points,
        std::vector<PointBucketIndexPair>& index_pair,
        std::map<uint64_t, NDT::Bucket>& buckets,
        GridParameters& rgd_params,
        int num_threads = std::thread::hardware_concurrency());

    void build_rgd(
        std::vector<Point3Di>& points,
        std::vector<PointBucketIndexPair>& index_pair,
        std::map<uint64_t, NDT::Bucket>& buckets,
        GridParameters& rgd_params,
        int num_threads = std::thread::hardware_concurrency());

    float bucket_size[3];
    float bucket_size_external[3];
    int number_of_threads;
    int number_of_iterations;

    bool is_fix_first_node = false;
    bool is_gauss_newton = true;
    bool is_levenberg_marguardt = false;
    bool is_wc = true;
    bool is_cw = false;
    bool is_tait_bryan_angles = true;
    bool is_quaternion = false;
    bool is_rodrigues = false;
    bool is_lie_algebra_left_jacobian = false;
    bool is_lie_algebra_right_jacobian = false;

    bool is_generalized = false;
    double sigma_r = 0.01;
    double sigma_polar_angle = 0.0001;
    double sigma_azimuthal_angle = 0.0001;
    int num_extended_points = 10;
};
