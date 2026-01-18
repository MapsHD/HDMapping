#pragma once

#include <point_clouds.h>
#include <thread>

class ICP
{
public:
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
        quaternion
    };

    struct Job
    {
        uint64_t index_begin_inclusive;
        uint64_t index_end_exclusive;
    };
    std::vector<Job> get_jobs(uint64_t size, int num_threads = 8);

    ICP()
    {
        search_radius = 0.1;
        number_of_threads = std::thread::hardware_concurrency();
        number_of_iterations = 6;
        is_adaptive_robust_kernel = false;
        is_ballanced_horizontal_vs_vertical = true;
        barron_c = 1.0;
    };
    ~ICP()
    {
        ;
    };

    bool optimize_source_to_target_wc(PointClouds& point_clouds_container, bool fix_first_node);
    bool compute_uncertainty(PointClouds& point_clouds_container);

    bool optimize_source_to_target_lie_algebra_left_jacobian(PointClouds& point_clouds_container, bool fix_first_node);
    bool optimize_source_to_target_lie_algebra_right_jacobian(PointClouds& point_clouds_container, bool fix_first_node);
    //-
    bool optimization_point_to_point_source_to_target(PointClouds& point_clouds_container);
    bool optimization_point_to_point_source_to_target_compute_rms(PointClouds& point_clouds_container, double& rms);
    bool optimization_point_to_point_source_to_target(
        PointClouds& point_clouds_container,
        PoseConvention pose_convention,
        OptimizationAlgorithm optimization_algorithm,
        RotationMatrixParametrization rotation_matrix_parametrization,
        double& out_rms,
        bool compute_only_rms);
    std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> compute_covariance_matrices_tait_bryan_point_to_point_source_to_target(
        PointClouds& point_clouds_container, PoseConvention pose_convention);
    std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> compute_covariance_matrices_rodrigues_point_to_point_source_to_target(
        PointClouds& point_clouds_container, PoseConvention pose_convention);
    std::vector<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>> compute_covariance_matrices_quaternion_point_to_point_source_to_target(
        PointClouds& point_clouds_container, PoseConvention pose_convention);

    bool optimize_source_to_target_lie_algebra_left_jacobian(PointClouds& point_clouds_container);
    std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>
    compute_covariance_matrices_point_to_point_source_to_target_source_to_target_lie_algebra_left_jacobian(
        PointClouds& point_clouds_container);
    bool optimize_source_to_target_lie_algebra_right_jacobian(PointClouds& point_clouds_container);
    std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>
    compute_covariance_matrices_point_to_point_source_to_target_source_to_target_lie_algebra_right_jacobian(
        PointClouds& point_clouds_container);

    float search_radius;
    int number_of_threads;
    int number_of_iterations;
    bool is_adaptive_robust_kernel;
    double barron_c = 1.0;
    bool is_ballanced_horizontal_vs_vertical;
    bool is_fix_first_node = false;
    bool is_gauss_newton = true;
    bool is_levenberg_marguardt = false;
    bool is_cw = false;
    bool is_wc = true;
    bool is_tait_bryan_angles = true;
    bool is_quaternion = false;
    bool is_rodrigues = false;
    bool is_lie_algebra_left_jacobian = false;
    bool is_lie_algebra_right_jacobian = false;
};
