#include <icp.h>
#include <iostream>
#include <fstream>
#include <thread>

#include <transformations.h>
#include <m_estimators.h>

#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_cw_jacobian.h>

#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_rodrigues_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_rodrigues_cw_jacobian.h>

#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_quaternion_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_quaternion_cw_jacobian.h>
#include <python-scripts/constraints/quaternion_constraint_jacobian.h>

Eigen::Matrix<double, 3, 1> get_delta_point_to_point_tait_bryan(ICP::PoseConvention pose_convention,
                                                                Eigen::Affine3d m_pose_wc, Eigen::Vector3d p_s, Eigen::Vector3d p_t)
{
    Eigen::Matrix<double, 3, 1> delta;

    TaitBryanPose pose_s;

    if (pose_convention == ICP::PoseConvention::wc)
    {
        pose_s = pose_tait_bryan_from_affine_matrix(m_pose_wc);
    }
    if (pose_convention == ICP::PoseConvention::cw)
    {
        pose_s = pose_tait_bryan_from_affine_matrix(m_pose_wc.inverse());
    }

    if (pose_convention == ICP::PoseConvention::wc)
    {
        double delta_x;
        double delta_y;
        double delta_z;
        point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z,
                                                      pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka, p_s.x(), p_s.y(), p_s.z(), p_t.x(), p_t.y(), p_t.z());

        delta(0, 0) = delta_x;
        delta(1, 0) = delta_y;
        delta(2, 0) = delta_z;
    }
    if (pose_convention == ICP::PoseConvention::cw)
    {
        double delta_x;
        double delta_y;
        double delta_z;
        point_to_point_source_to_target_tait_bryan_cw(delta_x, delta_y, delta_z,
                                                      pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka, p_s.x(), p_s.y(), p_s.z(), p_t.x(), p_t.y(), p_t.z());

        delta(0, 0) = delta_x;
        delta(1, 0) = delta_y;
        delta(2, 0) = delta_z;
    }
    return delta;
}

Eigen::Matrix<double, 3, 1> get_delta_point_to_point_rodrigues(ICP::PoseConvention pose_convention,
                                                               Eigen::Affine3d m_pose_wc, Eigen::Vector3d p_s, Eigen::Vector3d p_t)
{
    Eigen::Matrix<double, 3, 1> delta;

    RodriguesPose pose_s;

    if (pose_convention == ICP::PoseConvention::wc)
    {
        pose_s = pose_rodrigues_from_affine_matrix(m_pose_wc);
    }
    if (pose_convention == ICP::PoseConvention::cw)
    {
        pose_s = pose_rodrigues_from_affine_matrix(m_pose_wc.inverse());
    }

    if (pose_convention == ICP::PoseConvention::wc)
    {
        double delta_x;
        double delta_y;
        double delta_z;
        point_to_point_source_to_target_rodrigues_wc(delta_x, delta_y, delta_z,
                                                     pose_s.px, pose_s.py, pose_s.pz, pose_s.sx, pose_s.sy, pose_s.sz, p_s.x(), p_s.y(), p_s.z(), p_t.x(), p_t.y(), p_t.z());

        delta(0, 0) = delta_x;
        delta(1, 0) = delta_y;
        delta(2, 0) = delta_z;
    }
    if (pose_convention == ICP::PoseConvention::cw)
    {
        double delta_x;
        double delta_y;
        double delta_z;
        point_to_point_source_to_target_rodrigues_cw(delta_x, delta_y, delta_z,
                                                     pose_s.px, pose_s.py, pose_s.pz, pose_s.sx, pose_s.sy, pose_s.sz, p_s.x(), p_s.y(), p_s.z(), p_t.x(), p_t.y(), p_t.z());

        delta(0, 0) = delta_x;
        delta(1, 0) = delta_y;
        delta(2, 0) = delta_z;
    }
    return delta;
}

Eigen::Matrix<double, 3, 1> get_delta_point_to_point_quaternion(ICP::PoseConvention pose_convention,
                                                                Eigen::Affine3d m_pose_wc, Eigen::Vector3d p_s, Eigen::Vector3d p_t)
{
    Eigen::Matrix<double, 3, 1> delta;

    QuaternionPose pose_s;

    if (pose_convention == ICP::PoseConvention::wc)
    {
        pose_s = pose_quaternion_from_affine_matrix(m_pose_wc);
    }
    if (pose_convention == ICP::PoseConvention::cw)
    {
        pose_s = pose_quaternion_from_affine_matrix(m_pose_wc.inverse());
    }

    if (pose_convention == ICP::PoseConvention::wc)
    {
        double delta_x;
        double delta_y;
        double delta_z;
        point_to_point_source_to_target_quaternion_wc(delta_x, delta_y, delta_z,
                                                      pose_s.px, pose_s.py, pose_s.pz, pose_s.q0, pose_s.q1, pose_s.q2, pose_s.q3, p_s.x(), p_s.y(), p_s.z(), p_t.x(), p_t.y(), p_t.z());

        delta(0, 0) = delta_x;
        delta(1, 0) = delta_y;
        delta(2, 0) = delta_z;
    }
    if (pose_convention == ICP::PoseConvention::cw)
    {
        double delta_x;
        double delta_y;
        double delta_z;
        point_to_point_source_to_target_quaternion_cw(delta_x, delta_y, delta_z,
                                                      pose_s.px, pose_s.py, pose_s.pz, pose_s.q0, pose_s.q1, pose_s.q2, pose_s.q3, p_s.x(), p_s.y(), p_s.z(), p_t.x(), p_t.y(), p_t.z());

        delta(0, 0) = delta_x;
        delta(1, 0) = delta_y;
        delta(2, 0) = delta_z;
    }
    return delta;
}

Eigen::Matrix<double, 3, 6, Eigen::RowMajor> get_point_to_point_jacobian_tait_bryan(ICP::PoseConvention pose_convention,
                                                                                    Eigen::Affine3d m_pose_wc, Eigen::Vector3d p_s, Eigen::Vector3d p_t)
{

    Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;

    TaitBryanPose pose_s;

    if (pose_convention == ICP::PoseConvention::wc)
    {
        pose_s = pose_tait_bryan_from_affine_matrix(m_pose_wc);
    }
    if (pose_convention == ICP::PoseConvention::cw)
    {
        pose_s = pose_tait_bryan_from_affine_matrix(m_pose_wc.inverse());
    }

    if (pose_convention == ICP::PoseConvention::wc)
    {
        point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian, pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka, p_s.x(), p_s.y(), p_s.z());
    }
    if (pose_convention == ICP::PoseConvention::cw)
    {
        point_to_point_source_to_target_tait_bryan_cw_jacobian(jacobian, pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka, p_s.x(), p_s.y(), p_s.z());
    }
    return jacobian;
}

Eigen::Matrix<double, 3, 6, Eigen::RowMajor> get_point_to_point_jacobian_rodrigues(ICP::PoseConvention pose_convention,
                                                                                   Eigen::Affine3d m_pose_wc, Eigen::Vector3d p_s, Eigen::Vector3d p_t)
{

    Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;

    RodriguesPose pose_s;

    if (pose_convention == ICP::PoseConvention::wc)
    {
        pose_s = pose_rodrigues_from_affine_matrix(m_pose_wc);
    }
    if (pose_convention == ICP::PoseConvention::cw)
    {
        pose_s = pose_rodrigues_from_affine_matrix(m_pose_wc.inverse());
    }

    if (pose_convention == ICP::PoseConvention::wc)
    {
        point_to_point_source_to_target_rodrigues_wc_jacobian(jacobian, pose_s.px, pose_s.py, pose_s.pz, pose_s.sx, pose_s.sy, pose_s.sz, p_s.x(), p_s.y(), p_s.z());
    }
    if (pose_convention == ICP::PoseConvention::cw)
    {
        point_to_point_source_to_target_rodrigues_cw_jacobian(jacobian, pose_s.px, pose_s.py, pose_s.pz, pose_s.sx, pose_s.sy, pose_s.sz, p_s.x(), p_s.y(), p_s.z());
    }
    return jacobian;
}

Eigen::Matrix<double, 3, 7, Eigen::RowMajor> get_point_to_point_jacobian_quaternion(ICP::PoseConvention pose_convention,
                                                                                    Eigen::Affine3d m_pose_wc, Eigen::Vector3d p_s, Eigen::Vector3d p_t)
{

    Eigen::Matrix<double, 3, 7, Eigen::RowMajor> jacobian;

    QuaternionPose pose_s;

    if (pose_convention == ICP::PoseConvention::wc)
    {
        pose_s = pose_quaternion_from_affine_matrix(m_pose_wc);
    }
    if (pose_convention == ICP::PoseConvention::cw)
    {
        pose_s = pose_quaternion_from_affine_matrix(m_pose_wc.inverse());
    }

    if (pose_convention == ICP::PoseConvention::wc)
    {
        point_to_point_source_to_target_quaternion_wc_jacobian(jacobian, pose_s.px, pose_s.py, pose_s.pz, pose_s.q0, pose_s.q1, pose_s.q2, pose_s.q3, p_s.x(), p_s.y(), p_s.z());
    }
    if (pose_convention == ICP::PoseConvention::cw)
    {
        point_to_point_source_to_target_quaternion_cw_jacobian(jacobian, pose_s.px, pose_s.py, pose_s.pz, pose_s.q0, pose_s.q1, pose_s.q2, pose_s.q3, p_s.x(), p_s.y(), p_s.z());
    }
    return jacobian;
}

//-
bool ICP::optimization_point_to_point_source_to_target(PointClouds &point_clouds_container)
{
    if (is_rodrigues || is_quaternion)
    {
        for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
        {
            if (!point_clouds_container.point_clouds[i].fixed)
            {
                TaitBryanPose pose;
                pose.px = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
                pose.py = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
                pose.pz = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
                pose.om = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
                pose.fi = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
                pose.ka = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
                Eigen::Affine3d m = affine_matrix_from_pose_tait_bryan(pose);
                point_clouds_container.point_clouds[i].m_pose = point_clouds_container.point_clouds[i].m_pose * m;
            }
        }
    }

    OptimizationAlgorithm optimization_algorithm;
    if (is_gauss_newton)
    {
        optimization_algorithm = OptimizationAlgorithm::gauss_newton;
    }
    if (is_levenberg_marguardt)
    {
        optimization_algorithm = OptimizationAlgorithm::levenberg_marguardt;
    }

    PoseConvention pose_convention;
    if (is_wc)
    {
        pose_convention = PoseConvention::wc;
    }
    if (is_cw)
    {
        pose_convention = PoseConvention::cw;
    }

    RotationMatrixParametrization rotation_matrix_parametrization;
    if (is_tait_bryan_angles)
    {
        rotation_matrix_parametrization = RotationMatrixParametrization::tait_bryan_xyz;
    }
    if (is_rodrigues)
    {
        rotation_matrix_parametrization = RotationMatrixParametrization::rodrigues;
    }
    if (is_quaternion)
    {
        rotation_matrix_parametrization = RotationMatrixParametrization::quaternion;
    }

    std::cout << "===========================================================" << std::endl;
    std::cout << "optimization_point_to_point_source_to_target" << std::endl;
    if (pose_convention == PoseConvention::wc)
    {
        std::cout << "pose_convention == PoseConvention::wc" << std::endl;
    }
    if (pose_convention == PoseConvention::cw)
    {
        std::cout << "pose_convention == PoseConvention::cw" << std::endl;
    }
    if (optimization_algorithm == OptimizationAlgorithm::gauss_newton)
    {
        std::cout << "optimization_algorithm == OptimizationAlgorithm::gauss_newton" << std::endl;
    }
    if (optimization_algorithm == OptimizationAlgorithm::levenberg_marguardt)
    {
        std::cout << "optimization_algorithm == OptimizationAlgorithm::levenberg_marguardt" << std::endl;
    }
    if (rotation_matrix_parametrization == RotationMatrixParametrization::tait_bryan_xyz)
    {
        std::cout << "rotation_matrix_parametrization == RotationMatrixParametrization::tait_bryan_xyz" << std::endl;
    }
    if (rotation_matrix_parametrization == RotationMatrixParametrization::rodrigues)
    {
        std::cout << "rotation_matrix_parametrization == RotationMatrixParametrization::rodrigues" << std::endl;
    }
    if (rotation_matrix_parametrization == RotationMatrixParametrization::quaternion)
    {
        std::cout << "rotation_matrix_parametrization == RotationMatrixParametrization::quaternion" << std::endl;
    }

    /*std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covariance_matrices_before6x6;
    std::vector<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>> covariance_matrices_before7x7;
    std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covariance_matrices_after6x6;
    std::vector<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>> covariance_matrices_after7x7;

    if (rotation_matrix_parametrization == RotationMatrixParametrization::tait_bryan_xyz) {
        covariance_matrices_before6x6 = compute_covariance_matrices_tait_bryan_point_to_point_source_to_target(point_clouds_container, pose_convention);
    }
    if (rotation_matrix_parametrization == RotationMatrixParametrization::rodrigues) {
        covariance_matrices_before6x6 = compute_covariance_matrices_rodrigues_point_to_point_source_to_target(point_clouds_container, pose_convention);
    }
    if (rotation_matrix_parametrization == RotationMatrixParametrization::quaternion) {
        covariance_matrices_before7x7 = compute_covariance_matrices_quaternion_point_to_point_source_to_target(point_clouds_container, pose_convention);
    }*/

    //----------------------------------------------------------------------------------------------------
    double rms = 0.0;
    optimization_point_to_point_source_to_target(point_clouds_container, pose_convention, optimization_algorithm, rotation_matrix_parametrization, rms, false);
    //----------------------------------------------------------------------------------------------------

    /*if (rotation_matrix_parametrization == RotationMatrixParametrization::tait_bryan_xyz) {
        covariance_matrices_after6x6 = compute_covariance_matrices_tait_bryan_point_to_point_source_to_target(point_clouds_container, pose_convention);
    }
    if (rotation_matrix_parametrization == RotationMatrixParametrization::rodrigues) {
        covariance_matrices_after6x6 = compute_covariance_matrices_rodrigues_point_to_point_source_to_target(point_clouds_container, pose_convention);
    }
    if (rotation_matrix_parametrization == RotationMatrixParametrization::quaternion) {
        covariance_matrices_after7x7 = compute_covariance_matrices_quaternion_point_to_point_source_to_target(point_clouds_container, pose_convention);
    }

    for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++) {
        std::cout << "-----------------------------------------------" << std::endl;
        std::cout << "covariance matrices for: " << point_clouds_container.point_clouds[i].file_name << std::endl;

        std::cout << "covariance matrix before: " << std::endl;
        if (rotation_matrix_parametrization == RotationMatrixParametrization::tait_bryan_xyz ||
            rotation_matrix_parametrization == RotationMatrixParametrization::rodrigues) {
            std::cout << covariance_matrices_before6x6[i] << std::endl;
        }
        if (rotation_matrix_parametrization == RotationMatrixParametrization::quaternion) {
            std::cout << covariance_matrices_before7x7[i] << std::endl;
        }

        std::cout << "covariance matrix after: " << std::endl;
        if (rotation_matrix_parametrization == RotationMatrixParametrization::tait_bryan_xyz ||
            rotation_matrix_parametrization == RotationMatrixParametrization::rodrigues) {
            std::cout << covariance_matrices_after6x6[i] << std::endl;
        }
        if (rotation_matrix_parametrization == RotationMatrixParametrization::quaternion) {
            std::cout << covariance_matrices_after7x7[i] << std::endl;
        }
    }

    double mui;
    if (rotation_matrix_parametrization == RotationMatrixParametrization::tait_bryan_xyz ||
        rotation_matrix_parametrization == RotationMatrixParametrization::rodrigues) {
        mui = get_mean_uncertainty_xyz_impact6x6(covariance_matrices_before6x6, covariance_matrices_after6x6);
    }
    if (rotation_matrix_parametrization == RotationMatrixParametrization::quaternion) {
        mui = get_mean_uncertainty_xyz_impact7x7(covariance_matrices_before7x7, covariance_matrices_after7x7);
    }

    std::cout << "optimization_point_to_point_source_to_target" << std::endl;
    if (pose_convention == PoseConvention::wc) {
        std::cout << "pose_convention == PoseConvention::wc" << std::endl;
    }
    if (pose_convention == PoseConvention::cw) {
        std::cout << "pose_convention == PoseConvention::cw" << std::endl;
    }
    if (optimization_algorithm == OptimizationAlgorithm::gauss_newton) {
        std::cout << "optimization_algorithm == OptimizationAlgorithm::gauss_newton" << std::endl;
    }
    if (optimization_algorithm == OptimizationAlgorithm::levenberg_marguardt) {
        std::cout << "optimization_algorithm == OptimizationAlgorithm::levenberg_marguardt" << std::endl;
    }
    if (rotation_matrix_parametrization == RotationMatrixParametrization::tait_bryan_xyz) {
        std::cout << "rotation_matrix_parametrization == RotationMatrixParametrization::tait_bryan_xyz" << std::endl;
    }
    if (rotation_matrix_parametrization == RotationMatrixParametrization::rodrigues) {
        std::cout << "rotation_matrix_parametrization == RotationMatrixParametrization::rodrigues" << std::endl;
    }
    if (rotation_matrix_parametrization == RotationMatrixParametrization::quaternion) {
        std::cout << "rotation_matrix_parametrization == RotationMatrixParametrization::quaternion" << std::endl;
    }
    std::cout << "mean uncertainty_xyz impact: " << mui << std::endl;
    */
    return true;
}

bool ICP::optimization_point_to_point_source_to_target_compute_rms(PointClouds &point_clouds_container, double &rms)
{
    OptimizationAlgorithm optimization_algorithm;
    if (is_gauss_newton)
    {
        optimization_algorithm = OptimizationAlgorithm::gauss_newton;
    }
    if (is_levenberg_marguardt)
    {
        optimization_algorithm = OptimizationAlgorithm::levenberg_marguardt;
    }

    PoseConvention pose_convention;
    if (is_wc)
    {
        pose_convention = PoseConvention::wc;
    }
    if (is_cw)
    {
        pose_convention = PoseConvention::cw;
    }

    RotationMatrixParametrization rotation_matrix_parametrization;
    if (is_tait_bryan_angles)
    {
        rotation_matrix_parametrization = RotationMatrixParametrization::tait_bryan_xyz;
    }
    if (is_rodrigues)
    {
        rotation_matrix_parametrization = RotationMatrixParametrization::rodrigues;
    }
    if (is_quaternion)
    {
        rotation_matrix_parametrization = RotationMatrixParametrization::quaternion;
    }

    std::cout << "===========================================================" << std::endl;
    std::cout << "optimization_point_to_point_source_to_target" << std::endl;
    if (pose_convention == PoseConvention::wc)
    {
        std::cout << "pose_convention == PoseConvention::wc" << std::endl;
    }
    if (pose_convention == PoseConvention::cw)
    {
        std::cout << "pose_convention == PoseConvention::cw" << std::endl;
    }
    if (optimization_algorithm == OptimizationAlgorithm::gauss_newton)
    {
        std::cout << "optimization_algorithm == OptimizationAlgorithm::gauss_newton" << std::endl;
    }
    if (optimization_algorithm == OptimizationAlgorithm::levenberg_marguardt)
    {
        std::cout << "optimization_algorithm == OptimizationAlgorithm::levenberg_marguardt" << std::endl;
    }
    if (rotation_matrix_parametrization == RotationMatrixParametrization::tait_bryan_xyz)
    {
        std::cout << "rotation_matrix_parametrization == RotationMatrixParametrization::tait_bryan_xyz" << std::endl;
    }
    if (rotation_matrix_parametrization == RotationMatrixParametrization::rodrigues)
    {
        std::cout << "rotation_matrix_parametrization == RotationMatrixParametrization::rodrigues" << std::endl;
    }
    if (rotation_matrix_parametrization == RotationMatrixParametrization::quaternion)
    {
        std::cout << "rotation_matrix_parametrization == RotationMatrixParametrization::quaternion" << std::endl;
    }

    //----------------------------------------------------------------------------------------------------
    optimization_point_to_point_source_to_target(point_clouds_container, pose_convention, optimization_algorithm, rotation_matrix_parametrization, rms, true);
    //----------------------------------------------------------------------------------------------------

    return true;
}

void alpha_point_to_point_job(ICP::Job *job, std::vector<double> *alphas, float barron_c, std::vector<std::vector<std::pair<int, int>>> *all_nns,
                              std::vector<PointCloud> *point_clouds, std::vector<int> *j_indexes, Eigen::Affine3d m_pose_s_wc, float scale_factor_x, float scale_factor_y, float scale_factor_z,
                              std::vector<double> *sums_x, std::vector<double> *sums_y, std::vector<double> *sums_z, int index_source, ICP::PoseConvention pose_convention,
                              ICP::RotationMatrixParametrization rotation_matrix_parametrization)
{

    for (size_t ii = job->index_begin_inclusive; ii < job->index_end_exclusive; ii++)
    {
        double &alpha = (*alphas)[ii];
        double Z_tilde = get_approximate_partition_function(-10, 10, alpha, barron_c, 100);
        double sum_x = 0;
        double sum_y = 0;
        double sum_z = 0;

        for (size_t ni = 0; ni < (*all_nns).size(); ni++)
        {
            for (size_t nj = 0; nj < (*all_nns)[ni].size(); nj++)
            {
                Eigen::Vector3d p_s((*point_clouds)[index_source].points_local[(*all_nns)[ni][nj].first]);
                Eigen::Vector3d p_t((*point_clouds)[(*j_indexes)[ni]].m_pose * (*point_clouds)[(*j_indexes)[ni]].points_local[(*all_nns)[ni][nj].second]);

                Eigen::Matrix<double, 3, 1> delta;
                if (rotation_matrix_parametrization == ICP::RotationMatrixParametrization::tait_bryan_xyz)
                {
                    delta = get_delta_point_to_point_tait_bryan(pose_convention, m_pose_s_wc, p_s, p_t);
                }
                if (rotation_matrix_parametrization == ICP::RotationMatrixParametrization::rodrigues)
                {
                    delta = get_delta_point_to_point_rodrigues(pose_convention, m_pose_s_wc, p_s, p_t);
                }
                if (rotation_matrix_parametrization == ICP::RotationMatrixParametrization::quaternion)
                {
                    delta = get_delta_point_to_point_quaternion(pose_convention, m_pose_s_wc, p_s, p_t);
                }

                if (!(delta(0, 0) == delta(0, 0)))
                {
                    continue;
                }
                if (!(delta(1, 0) == delta(1, 0)))
                {
                    continue;
                }
                if (!(delta(2, 0) == delta(2, 0)))
                {
                    continue;
                }

                delta(0, 0) *= scale_factor_x;
                delta(1, 0) *= scale_factor_y;
                delta(2, 0) *= scale_factor_z;

                sum_x += get_truncated_robust_kernel(delta(0, 0), alpha, barron_c, Z_tilde);
                sum_y += get_truncated_robust_kernel(delta(1, 0), alpha, barron_c, Z_tilde);
                sum_z += get_truncated_robust_kernel(delta(2, 0), alpha, barron_c, Z_tilde);
            }
        }
        (*sums_x)[ii] = sum_x;
        (*sums_y)[ii] = sum_y;
        (*sums_z)[ii] = sum_z;
    }
}

bool ICP::optimization_point_to_point_source_to_target(PointClouds &point_clouds_container,
                                                       PoseConvention pose_convention, OptimizationAlgorithm optimization_algorithm, RotationMatrixParametrization rotation_matrix_parametrization, double &out_rms, bool compute_only_rms)
{
    bool precompute_rgd = false;

    int number_of_unknowns_per_pose;
    if (rotation_matrix_parametrization == RotationMatrixParametrization::tait_bryan_xyz || rotation_matrix_parametrization == RotationMatrixParametrization::rodrigues)
    {
        number_of_unknowns_per_pose = 6;
    }
    if (rotation_matrix_parametrization == RotationMatrixParametrization::quaternion)
    {
        number_of_unknowns_per_pose = 7;
    }

    if (precompute_rgd)
    {
        int counter_computed_rgd = 1;
        for (auto &pc : point_clouds_container.point_clouds)
        {
            std::cout << "computing rgd [" << counter_computed_rgd++ << "] of " << point_clouds_container.point_clouds.size() << std::endl;
            pc.build_rgd();
            pc.cout_rgd();
            pc.points_type.resize(pc.points_local.size());
            // pc.compute_normal_vectors(0.5);
        }
    }
    else
    {
        // int counter_computed_nv = 1;
        for (auto &pc : point_clouds_container.point_clouds)
        {
            pc.points_type.resize(pc.points_local.size());
            // std::cout << "computing nv [" << counter_computed_nv++ << "] of " << point_clouds_container.point_clouds.size() << std::endl;
            // pc.compute_normal_vectors(0.5);
        }
    }

    double lm_lambda = 0.0001;
    double previous_rms = std::numeric_limits<double>::max();
    int number_of_lm_iterations = 0;

    std::vector<Eigen::Affine3d> m_poses_tmp;
    if (optimization_algorithm == OptimizationAlgorithm::levenberg_marguardt)
    {
        m_poses_tmp.clear();
        for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
        {
            m_poses_tmp.push_back(point_clouds_container.point_clouds[i].m_pose);
        }
    }

    for (int iter = 0; iter < number_of_iterations; iter++)
    {
        if (!compute_only_rms)
        {
            std::cout << "Iteration: " << iter + 1 << " of " << number_of_iterations << std::endl;
        }

        std::vector<Eigen::Triplet<double>> tripletListA;
        std::vector<Eigen::Triplet<double>> tripletListP;
        std::vector<Eigen::Triplet<double>> tripletListB;

        double rms = 0.0;
        double rms_geo = 0.0;
        double rms_geo_sum = 0.0;
        double sum_obs = 0.0;

        for (int i = 0; i < point_clouds_container.point_clouds.size(); i++)
        {
            // barron
            double min_sum_x = std::numeric_limits<double>::max();
            double min_sum_y = std::numeric_limits<double>::max();
            double min_sum_z = std::numeric_limits<double>::max();

            double barron_alpha_x = -10.;
            double barron_alpha_y = -10.;
            double barron_alpha_z = -10.;

            std::vector<std::vector<std::pair<int, int>>> all_nns;
            std::vector<int> j_indexes;

            float scale_factor_x = 10;
            float scale_factor_y = 10;
            float scale_factor_z = 10;

            if (is_adaptive_robust_kernel)
            {
                for (int j = 0; j < point_clouds_container.point_clouds.size(); j++)
                {
                    if (i != j)
                    {
                        if (!precompute_rgd)
                        {
                            point_clouds_container.point_clouds[j].build_rgd();
                            point_clouds_container.point_clouds[j].cout_rgd();
                        }

                        std::vector<std::pair<int, int>> nns = point_clouds_container.point_clouds[i].nns(point_clouds_container.point_clouds[j], search_radious);

                        if (!precompute_rgd)
                        {
                            point_clouds_container.point_clouds[j].buckets.clear();
                            point_clouds_container.point_clouds[j].index_pairs.clear();
                        }

                        all_nns.push_back(nns);
                        j_indexes.push_back(j);
                    }
                }

                std::vector<double> alphas;
                std::vector<double> sums_x;
                std::vector<double> sums_y;
                std::vector<double> sums_z;

                for (double alpha = -10; alpha <= 2; alpha += 0.1)
                {
                    alphas.push_back(alpha);
                    sums_x.push_back(0);
                    sums_y.push_back(0);
                    sums_z.push_back(0);
                }
                std::vector<Job> jobs = get_jobs(alphas.size(), this->number_of_threads);
                std::vector<std::thread> threads;

                for (size_t k = 0; k < jobs.size(); k++)
                {
                    threads.push_back(std::thread(alpha_point_to_point_job, &jobs[k], &alphas, barron_c, &all_nns, &point_clouds_container.point_clouds, &j_indexes,
                                                  point_clouds_container.point_clouds[i].m_pose, scale_factor_x, scale_factor_y, scale_factor_z, &sums_x, &sums_y, &sums_z, i, pose_convention, rotation_matrix_parametrization));
                }

                for (size_t j = 0; j < threads.size(); j++)
                {
                    threads[j].join();
                }

                for (size_t s = 0; s < sums_x.size(); s++)
                {
                    if (sums_x[s] < min_sum_x)
                    {
                        min_sum_x = sums_x[s];
                        barron_alpha_x = alphas[s];
                    }
                    if (sums_y[s] < min_sum_y)
                    {
                        min_sum_y = sums_y[s];
                        barron_alpha_y = alphas[s];
                    }
                    if (sums_z[s] < min_sum_z)
                    {
                        min_sum_z = sums_z[s];
                        barron_alpha_z = alphas[s];
                    }
                }
                std::cout << "barron_alpha_x: " << barron_alpha_x << " barron_alpha_y: " << barron_alpha_y << " barron_alpha_z: " << barron_alpha_z << std::endl;
            }

            for (int j = 0; j < point_clouds_container.point_clouds.size(); j++)
            {
                if (i != j)
                {
                    std::cout << "computing i[" << i + 1 << "] of " << point_clouds_container.point_clouds.size() << std::endl;
                    std::cout << "computing j[" << j + 1 << "] of " << point_clouds_container.point_clouds.size() << std::endl;

                    if (!precompute_rgd)
                    {
                        point_clouds_container.point_clouds[j].build_rgd();
                        point_clouds_container.point_clouds[j].cout_rgd();
                        // point_clouds_container.point_clouds[j].compute_normal_vectors(0.5);
                    }

                    std::cout << "computing nns start" << std::endl;
                    std::vector<std::pair<int, int>> nns = point_clouds_container.point_clouds[i].nns(point_clouds_container.point_clouds[j], search_radious);
                    std::cout << "computing nns finished nns.size(): " << nns.size() << std::endl;

                    if (!precompute_rgd)
                    {
                        point_clouds_container.point_clouds[j].buckets.clear();
                        point_clouds_container.point_clouds[j].index_pairs.clear();
                    }

                    for (size_t k = 0; k < nns.size(); k++)
                    {
                        Eigen::Vector3d p_s(point_clouds_container.point_clouds[i].points_local[nns[k].first]);
                        Eigen::Vector3d p_t(point_clouds_container.point_clouds[j].m_pose * point_clouds_container.point_clouds[j].points_local[nns[k].second]);

                        Eigen::Matrix<double, 3, 1> delta;
                        if (rotation_matrix_parametrization == RotationMatrixParametrization::tait_bryan_xyz)
                        {
                            delta = get_delta_point_to_point_tait_bryan(pose_convention, point_clouds_container.point_clouds[i].m_pose, p_s, p_t);
                        }
                        if (rotation_matrix_parametrization == RotationMatrixParametrization::rodrigues)
                        {
                            delta = get_delta_point_to_point_rodrigues(pose_convention, point_clouds_container.point_clouds[i].m_pose, p_s, p_t);
                        }
                        if (rotation_matrix_parametrization == RotationMatrixParametrization::quaternion)
                        {
                            delta = get_delta_point_to_point_quaternion(pose_convention, point_clouds_container.point_clouds[i].m_pose, p_s, p_t);
                        }

                        if (!(delta(0, 0) == delta(0, 0)))
                        {
                            continue;
                        }
                        if (!(delta(1, 0) == delta(1, 0)))
                        {
                            continue;
                        }
                        if (!(delta(2, 0) == delta(2, 0)))
                        {
                            continue;
                        }

                        if (!compute_only_rms)
                        {
                            Eigen::Matrix<double, 3, 6> jacobian3x6;
                            Eigen::Matrix<double, 3, 7> jacobian3x7;
                            if (rotation_matrix_parametrization == RotationMatrixParametrization::tait_bryan_xyz)
                            {
                                jacobian3x6 = get_point_to_point_jacobian_tait_bryan(pose_convention, point_clouds_container.point_clouds[i].m_pose, p_s, p_t);
                            }
                            if (rotation_matrix_parametrization == RotationMatrixParametrization::rodrigues)
                            {
                                jacobian3x6 = get_point_to_point_jacobian_rodrigues(pose_convention, point_clouds_container.point_clouds[i].m_pose, p_s, p_t);
                            }
                            if (rotation_matrix_parametrization == RotationMatrixParametrization::quaternion)
                            {
                                jacobian3x7 = get_point_to_point_jacobian_quaternion(pose_convention, point_clouds_container.point_clouds[i].m_pose, p_s, p_t);
                            }

                            int ir = tripletListB.size();

                            if (rotation_matrix_parametrization == RotationMatrixParametrization::tait_bryan_xyz || rotation_matrix_parametrization == RotationMatrixParametrization::rodrigues)
                            {
                                int ic = i * number_of_unknowns_per_pose;
                                for (int row = 0; row < 3; row++)
                                {
                                    for (int col = 0; col < number_of_unknowns_per_pose; col++)
                                    {
                                        if (jacobian3x6(row, col) != 0.0)
                                        {
                                            tripletListA.emplace_back(ir + row, ic + col, -jacobian3x6(row, col));
                                        }
                                    }
                                }
                            }
                            if (rotation_matrix_parametrization == RotationMatrixParametrization::quaternion)
                            {
                                int ic = i * number_of_unknowns_per_pose;
                                for (int row = 0; row < 3; row++)
                                {
                                    for (int col = 0; col < number_of_unknowns_per_pose; col++)
                                    {
                                        if (jacobian3x7(row, col) != 0.0)
                                        {
                                            tripletListA.emplace_back(ir + row, ic + col, -jacobian3x7(row, col));
                                        }
                                    }
                                }
                            }

                            if (is_adaptive_robust_kernel)
                            {
                                tripletListP.emplace_back(ir, ir, get_barron_w(delta(0, 0) * scale_factor_x, barron_alpha_x, barron_c));
                                tripletListP.emplace_back(ir + 1, ir + 1, get_barron_w(delta(1, 0) * scale_factor_y, barron_alpha_y, barron_c));
                                tripletListP.emplace_back(ir + 2, ir + 2, get_barron_w(delta(2, 0) * scale_factor_z, barron_alpha_z, barron_c));
                            }
                            else
                            {
                                tripletListP.emplace_back(ir, ir, 1);
                                tripletListP.emplace_back(ir + 1, ir + 1, 1);
                                tripletListP.emplace_back(ir + 2, ir + 2, 1);
                            }

                            tripletListB.emplace_back(ir, 0, delta(0, 0));
                            tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
                            tripletListB.emplace_back(ir + 2, 0, delta(2, 0));
                        }


                        rms += delta(0, 0) * delta(0, 0) + delta(1, 0) * delta(1, 0) + delta(2, 0) * delta(2, 0);
                        sum_obs += 3.0;

                        //if(sqrt(delta(0, 0) * delta(0, 0) + delta(1, 0) * delta(1, 0) + delta(2, 0) * delta(2, 0)) > 0.1){
                        //    std::cout << sqrt(delta(0, 0) * delta(0, 0) + delta(1, 0) * delta(1, 0) + delta(2, 0) * delta(2, 0)) << std::endl;
                        //}
                    }

                    double curr_rms = sqrt(rms / sum_obs);
                    std::cout << "curr_rms: " << curr_rms << std::endl;
                    std::cout << "number of observations: " << sum_obs << std::endl;
                }
            }

            // if (!precompute_rgd)
            //{
            //     point_clouds_container.point_clouds[i].buckets.clear();
            //     point_clouds_container.point_clouds[i].index_pairs.clear();
            // }
        }
        rms = sqrt(rms / sum_obs);
        out_rms = rms;
        if (compute_only_rms)
        {
            return true;
        }

        for (int i = 0; i < point_clouds_container.point_clouds.size(); i++)
        {
            for (int gp = 0; gp < point_clouds_container.point_clouds[i].available_geo_points.size(); gp++)
            {
                if (point_clouds_container.point_clouds[i].available_geo_points[gp].choosen)
                {
                    std::cout << "X adding geo point " << point_clouds_container.point_clouds[i].available_geo_points[gp].name << std::endl;

                    Eigen::Vector3d p_s = point_clouds_container.point_clouds[i].available_geo_points[gp].coordinates;
                    Eigen::Vector3d p_t = point_clouds_container.point_clouds[i].available_geo_points[gp].coordinates;

                    Eigen::Matrix<double, 3, 1> delta;
                    if (rotation_matrix_parametrization == RotationMatrixParametrization::tait_bryan_xyz)
                    {
                        delta = get_delta_point_to_point_tait_bryan(pose_convention, point_clouds_container.point_clouds[i].m_pose, p_s, p_t);
                    }
                    if (rotation_matrix_parametrization == RotationMatrixParametrization::rodrigues)
                    {
                        delta = get_delta_point_to_point_rodrigues(pose_convention, point_clouds_container.point_clouds[i].m_pose, p_s, p_t);
                    }
                    if (rotation_matrix_parametrization == RotationMatrixParametrization::quaternion)
                    {
                        delta = get_delta_point_to_point_quaternion(pose_convention, point_clouds_container.point_clouds[i].m_pose, p_s, p_t);
                    }

                    if (!(delta(0, 0) == delta(0, 0)))
                    {
                        continue;
                    }
                    if (!(delta(1, 0) == delta(1, 0)))
                    {
                        continue;
                    }
                    if (!(delta(2, 0) == delta(2, 0)))
                    {
                        continue;
                    }

                    Eigen::Matrix<double, 3, 6> jacobian3x6;
                    Eigen::Matrix<double, 3, 7> jacobian3x7;
                    if (rotation_matrix_parametrization == RotationMatrixParametrization::tait_bryan_xyz)
                    {
                        jacobian3x6 = get_point_to_point_jacobian_tait_bryan(pose_convention, point_clouds_container.point_clouds[i].m_pose, p_s, p_t);
                    }
                    if (rotation_matrix_parametrization == RotationMatrixParametrization::rodrigues)
                    {
                        jacobian3x6 = get_point_to_point_jacobian_rodrigues(pose_convention, point_clouds_container.point_clouds[i].m_pose, p_s, p_t);
                    }
                    if (rotation_matrix_parametrization == RotationMatrixParametrization::quaternion)
                    {
                        jacobian3x7 = get_point_to_point_jacobian_quaternion(pose_convention, point_clouds_container.point_clouds[i].m_pose, p_s, p_t);
                    }

                    int ir = tripletListB.size();

                    if (rotation_matrix_parametrization == RotationMatrixParametrization::tait_bryan_xyz || rotation_matrix_parametrization == RotationMatrixParametrization::rodrigues)
                    {
                        int ic = i * number_of_unknowns_per_pose;
                        for (int row = 0; row < 3; row++)
                        {
                            for (int col = 0; col < number_of_unknowns_per_pose; col++)
                            {
                                if (jacobian3x6(row, col) != 0.0)
                                {
                                    tripletListA.emplace_back(ir + row, ic + col, -jacobian3x6(row, col));
                                }
                            }
                        }
                    }
                    if (rotation_matrix_parametrization == RotationMatrixParametrization::quaternion)
                    {
                        int ic = i * number_of_unknowns_per_pose;
                        for (int row = 0; row < 3; row++)
                        {
                            for (int col = 0; col < number_of_unknowns_per_pose; col++)
                            {
                                if (jacobian3x7(row, col) != 0.0)
                                {
                                    tripletListA.emplace_back(ir + row, ic + col, -jacobian3x7(row, col));
                                }
                            }
                        }
                    }

                    tripletListP.emplace_back(ir, ir, point_clouds_container.point_clouds[i].available_geo_points[gp].w_x * get_cauchy_w(delta(0, 0), 1));
                    tripletListP.emplace_back(ir + 1, ir + 1, point_clouds_container.point_clouds[i].available_geo_points[gp].w_y * get_cauchy_w(delta(1, 0), 1));
                    tripletListP.emplace_back(ir + 2, ir + 2, point_clouds_container.point_clouds[i].available_geo_points[gp].w_z * get_cauchy_w(delta(2, 0), 1));

                    std::cout << "delta(0, 0) " << delta(0, 0) << " get_cauchy_w(delta(0, 0), 1): " << get_cauchy_w(delta(0, 0), 1) << std::endl;
                    std::cout << "delta(1, 0) " << delta(1, 0) << " get_cauchy_w(delta(1, 0), 1): " << get_cauchy_w(delta(1, 0), 1) << std::endl;
                    std::cout << "delta(2, 0) " << delta(2, 0) << " get_cauchy_w(delta(2, 0), 1): " << get_cauchy_w(delta(2, 0), 1) << std::endl;

                    tripletListB.emplace_back(ir, 0, delta(0, 0));
                    tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
                    tripletListB.emplace_back(ir + 2, 0, delta(2, 0));

                    rms_geo += delta(0, 0) * delta(0, 0);
                    rms_geo += delta(1, 0) * delta(1, 0);
                    rms_geo += delta(2, 0) * delta(2, 0);
                    rms_geo_sum += 3.0;
                }
            }
        }

        if (rms_geo_sum > 0)
        {
            std::cout << "rms geo: " << sqrt(rms_geo / rms_geo_sum) << std::endl;
        }

        std::cout << "previous_rms: " << previous_rms << " rms: " << rms << std::endl;
        if (optimization_algorithm == OptimizationAlgorithm::levenberg_marguardt)
        {

            if (rms < previous_rms)
            {
                if (lm_lambda < 1000000)
                {
                    lm_lambda *= 10.0;
                }
                previous_rms = rms;
                std::cout << " lm_lambda: " << lm_lambda << std::endl;
            }
            else
            {
                lm_lambda /= 10.0;
                number_of_lm_iterations++;
                iter--;
                std::cout << " lm_lambda: " << lm_lambda << std::endl;
                for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
                {
                    point_clouds_container.point_clouds[i].m_pose = m_poses_tmp[i];
                }
                previous_rms = std::numeric_limits<double>::max();
                continue;
            }
        }
        else
        {
            previous_rms = rms;
        }

        if (is_fix_first_node)
        {
            int ir = tripletListB.size();
            tripletListA.emplace_back(ir, 0, 1);
            tripletListA.emplace_back(ir + 1, 1, 1);
            tripletListA.emplace_back(ir + 2, 2, 1);
            tripletListA.emplace_back(ir + 3, 3, 1);
            tripletListA.emplace_back(ir + 4, 4, 1);
            tripletListA.emplace_back(ir + 5, 5, 1);
            if (rotation_matrix_parametrization == RotationMatrixParametrization::quaternion)
            {
                tripletListA.emplace_back(ir + 6, 6, 1);
            }

            tripletListP.emplace_back(ir, ir, 1000000);
            tripletListP.emplace_back(ir + 1, ir + 1, 1000000);
            tripletListP.emplace_back(ir + 2, ir + 2, 1000000);
            tripletListP.emplace_back(ir + 3, ir + 3, 1000000);
            tripletListP.emplace_back(ir + 4, ir + 4, 1000000);
            tripletListP.emplace_back(ir + 5, ir + 5, 1000000);
            if (rotation_matrix_parametrization == RotationMatrixParametrization::quaternion)
            {
                tripletListP.emplace_back(ir + 6, ir + 6, 1000000);
            }

            tripletListB.emplace_back(ir, 0, 0);
            tripletListB.emplace_back(ir + 1, 0, 0);
            tripletListB.emplace_back(ir + 2, 0, 0);
            tripletListB.emplace_back(ir + 3, 0, 0);
            tripletListB.emplace_back(ir + 4, 0, 0);
            tripletListB.emplace_back(ir + 5, 0, 0);
            if (rotation_matrix_parametrization == RotationMatrixParametrization::quaternion)
            {
                tripletListB.emplace_back(ir + 6, 0, 0);
            }
        }
        if (rotation_matrix_parametrization == RotationMatrixParametrization::quaternion)
        {
            for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
            {
                int ic = i * 7;
                int ir = tripletListB.size();
                QuaternionPose pose;
                if (pose_convention == PoseConvention::wc)
                {
                    pose = pose_quaternion_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);
                }
                if (pose_convention == PoseConvention::cw)
                {
                    pose = pose_quaternion_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose.inverse());
                }
                double delta;
                quaternion_constraint(delta, pose.q0, pose.q1, pose.q2, pose.q3);

                Eigen::Matrix<double, 1, 4> jacobian;
                quaternion_constraint_jacobian(jacobian, pose.q0, pose.q1, pose.q2, pose.q3);

                tripletListA.emplace_back(ir, ic + 3, -jacobian(0, 0));
                tripletListA.emplace_back(ir, ic + 4, -jacobian(0, 1));
                tripletListA.emplace_back(ir, ic + 5, -jacobian(0, 2));
                tripletListA.emplace_back(ir, ic + 6, -jacobian(0, 3));

                tripletListP.emplace_back(ir, ir, 1000000.0);

                tripletListB.emplace_back(ir, 0, delta);
            }
        }

        Eigen::SparseMatrix<double> matA(tripletListB.size(), point_clouds_container.point_clouds.size() * number_of_unknowns_per_pose);
        Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
        Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

        matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
        matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
        matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

        Eigen::SparseMatrix<double> AtPA(point_clouds_container.point_clouds.size() * number_of_unknowns_per_pose, point_clouds_container.point_clouds.size() * number_of_unknowns_per_pose);
        Eigen::SparseMatrix<double> AtPB(point_clouds_container.point_clouds.size() * number_of_unknowns_per_pose, 1);

        Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
        AtPA = AtP * matA;
        AtPB = AtP * matB;

        if (optimization_algorithm == OptimizationAlgorithm::levenberg_marguardt)
        {
            Eigen::SparseMatrix<double> LM(point_clouds_container.point_clouds.size() * number_of_unknowns_per_pose, point_clouds_container.point_clouds.size() * number_of_unknowns_per_pose);
            LM.setIdentity();
            LM *= lm_lambda;
            AtPA += LM;
        }

        tripletListA.clear();
        tripletListP.clear();
        tripletListB.clear();

        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);
        Eigen::SparseMatrix<double> x = solver.solve(AtPB);

        std::vector<double> h_x;
        // std::cout << "Solution: " << std::endl;
        for (int k = 0; k < x.outerSize(); ++k)
        {
            for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            {
                h_x.push_back(it.value());
                // std::cout << "col: " << it.col() << " row: " << it.row() << " value: " << it.value() << std::endl;
            }
        }

        if (h_x.size() == point_clouds_container.point_clouds.size() * number_of_unknowns_per_pose)
        {
            int counter = 0;
            std::cout << "Solution" << std::endl;
            if (rotation_matrix_parametrization == RotationMatrixParametrization::tait_bryan_xyz)
            {
                std::cout << "x,y,z,om,fi,ka" << std::endl;
                for (size_t i = 0; i < h_x.size(); i += number_of_unknowns_per_pose)
                {
                    std::cout << h_x[i] << "," << h_x[i + 1] << "," << h_x[i + 2] << "," << h_x[i + 3] << "," << h_x[i + 4] << "," << h_x[i + 5] << std::endl;
                }
            }
            if (rotation_matrix_parametrization == RotationMatrixParametrization::rodrigues)
            {
                std::cout << "x,y,z,sx,sy,sz" << std::endl;
                for (size_t i = 0; i < h_x.size(); i += number_of_unknowns_per_pose)
                {
                    std::cout << h_x[i] << "," << h_x[i + 1] << "," << h_x[i + 2] << "," << h_x[i + 3] << "," << h_x[i + 4] << "," << h_x[i + 5] << std::endl;
                }
            }
            if (rotation_matrix_parametrization == RotationMatrixParametrization::quaternion)
            {
                std::cout << "x,y,z,q0,q1,q2,q3" << std::endl;
                for (size_t i = 0; i < h_x.size(); i += number_of_unknowns_per_pose)
                {
                    std::cout << h_x[i] << "," << h_x[i + 1] << "," << h_x[i + 2] << "," << h_x[i + 3] << "," << h_x[i + 4] << "," << h_x[i + 5] << "," << h_x[i + 6] << std::endl;
                }
            }

            for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
            {

                if (rotation_matrix_parametrization == RotationMatrixParametrization::tait_bryan_xyz)
                {
                    TaitBryanPose pose;
                    if (pose_convention == PoseConvention::wc)
                    {
                        pose = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);
                    }
                    if (pose_convention == PoseConvention::cw)
                    {
                        pose = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose.inverse());
                    }

                    pose.px += h_x[counter++] * 0.5;
                    pose.py += h_x[counter++] * 0.5;
                    pose.pz += h_x[counter++] * 0.5;
                    pose.om += h_x[counter++] * 0.5;
                    pose.fi += h_x[counter++] * 0.5;
                    pose.ka += h_x[counter++] * 0.5;

                    if (i == 0 && is_fix_first_node)
                    {
                        continue;
                    }
                    if (point_clouds_container.point_clouds[i].fixed){
                        std::cout << "point cloud " << i << " is fixed, continue" << std::endl;
                        continue;
                    }

                    if (pose_convention == PoseConvention::wc)
                    {
                        point_clouds_container.point_clouds[i].m_pose = affine_matrix_from_pose_tait_bryan(pose);
                    }
                    if (pose_convention == PoseConvention::cw)
                    {
                        point_clouds_container.point_clouds[i].m_pose = affine_matrix_from_pose_tait_bryan(pose).inverse();
                    }
                }
                if (rotation_matrix_parametrization == RotationMatrixParametrization::rodrigues)
                {
                    RodriguesPose pose;
                    if (pose_convention == PoseConvention::wc)
                    {
                        pose = pose_rodrigues_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);
                    }
                    if (pose_convention == PoseConvention::cw)
                    {
                        pose = pose_rodrigues_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose.inverse());
                    }

                    pose.px += h_x[counter++] * 0.5;
                    pose.py += h_x[counter++] * 0.5;
                    pose.pz += h_x[counter++] * 0.5;
                    pose.sx += h_x[counter++] * 0.5;
                    pose.sy += h_x[counter++] * 0.5;
                    pose.sz += h_x[counter++] * 0.5;

                    if (i == 0 && is_fix_first_node)
                    {
                        continue;
                    }
                    if (point_clouds_container.point_clouds[i].fixed)
                    {
                        std::cout << "PC: " << point_clouds_container.point_clouds[i].file_name << " is fixed" << std::endl;
                        continue;
                    }

                    if (pose_convention == PoseConvention::wc)
                    {
                        point_clouds_container.point_clouds[i].m_pose = affine_matrix_from_pose_rodrigues(pose);
                    }
                    if (pose_convention == PoseConvention::cw)
                    {
                        point_clouds_container.point_clouds[i].m_pose = affine_matrix_from_pose_rodrigues(pose).inverse();
                    }
                }
                if (rotation_matrix_parametrization == RotationMatrixParametrization::quaternion)
                {
                    QuaternionPose pose;
                    if (pose_convention == PoseConvention::wc)
                    {
                        pose = pose_quaternion_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);
                    }
                    if (pose_convention == PoseConvention::cw)
                    {
                        pose = pose_quaternion_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose.inverse());
                    }

                    pose.px += h_x[counter++] * 0.5;
                    pose.py += h_x[counter++] * 0.5;
                    pose.pz += h_x[counter++] * 0.5;
                    pose.q0 += h_x[counter++] * 0.5;
                    pose.q1 += h_x[counter++] * 0.5;
                    pose.q2 += h_x[counter++] * 0.5;
                    pose.q3 += h_x[counter++] * 0.5;

                    if (i == 0 && is_fix_first_node)
                    {
                        continue;
                    }
                    if (point_clouds_container.point_clouds[i].fixed)
                    {
                        std::cout << "PC: " << point_clouds_container.point_clouds[i].file_name << " is fixed" << std::endl;
                        continue;
                    }

                    if (pose_convention == PoseConvention::wc)
                    {
                        point_clouds_container.point_clouds[i].m_pose = affine_matrix_from_pose_quaternion(pose);
                    }
                    if (pose_convention == PoseConvention::cw)
                    {
                        point_clouds_container.point_clouds[i].m_pose = affine_matrix_from_pose_quaternion(pose).inverse();
                    }
                }

                if (!point_clouds_container.point_clouds[i].fixed)
                {
                    point_clouds_container.point_clouds[i].pose = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);
                    point_clouds_container.point_clouds[i].gui_translation[0] = (float)point_clouds_container.point_clouds[i].pose.px;
                    point_clouds_container.point_clouds[i].gui_translation[1] = (float)point_clouds_container.point_clouds[i].pose.py;
                    point_clouds_container.point_clouds[i].gui_translation[2] = (float)point_clouds_container.point_clouds[i].pose.pz;
                    point_clouds_container.point_clouds[i].gui_rotation[0] = (float)rad2deg(point_clouds_container.point_clouds[i].pose.om);
                    point_clouds_container.point_clouds[i].gui_rotation[1] = (float)rad2deg(point_clouds_container.point_clouds[i].pose.fi);
                    point_clouds_container.point_clouds[i].gui_rotation[2] = (float)rad2deg(point_clouds_container.point_clouds[i].pose.ka);
                }
                else
                {
                    std::cout << "PC: " << point_clouds_container.point_clouds[i].file_name << " is fixed (check it!!!)" << std::endl;
                }
            }

            if (optimization_algorithm == OptimizationAlgorithm::levenberg_marguardt)
            {
                m_poses_tmp.clear();
                for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
                {
                    m_poses_tmp.push_back(point_clouds_container.point_clouds[i].m_pose);
                }
            }
        }
        else
        {
            std::cout << "AtPA=AtPB FAILED" << std::endl;
            std::cout << "number of computed unknowns: " << h_x.size() << " should be: " << point_clouds_container.point_clouds.size() * number_of_unknowns_per_pose << std::endl;

            // clean
            for (auto &pc : point_clouds_container.point_clouds)
            {
                pc.clean();
            }
            return false;
        }
    }

    // clean
    for (auto &pc : point_clouds_container.point_clouds)
    {
        pc.clean();
    }
    return true;
}

std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> ICP::compute_covariance_matrices_tait_bryan_point_to_point_source_to_target(
    PointClouds &point_clouds_container, PoseConvention pose_convention)
{
    for (auto &pc : point_clouds_container.point_clouds)
    {
        pc.build_rgd();
        pc.cout_rgd();
        pc.compute_normal_vectors(0.5);
    }

    std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covariance_matrices;
    std::vector<Eigen::Triplet<double>> tripletListA;

    double ssr = 0.0;
    int num_obs = 0;
    for (int i = 0; i < point_clouds_container.point_clouds.size(); i++)
    {
        for (int j = 0; j < point_clouds_container.point_clouds.size(); j++)
        {
            if (i != j)
            {
                std::vector<std::pair<int, int>> nns = point_clouds_container.point_clouds[i].nns(point_clouds_container.point_clouds[j], search_radious);

                for (size_t k = 0; k < nns.size(); k++)
                {
                    Eigen::Vector3d p_s(point_clouds_container.point_clouds[i].points_local[nns[k].first]);
                    Eigen::Vector3d p_t(point_clouds_container.point_clouds[j].m_pose * point_clouds_container.point_clouds[j].points_local[nns[k].second]);

                    Eigen::Matrix<double, 3, 1> delta = get_delta_point_to_point_tait_bryan(pose_convention, point_clouds_container.point_clouds[i].m_pose, p_s, p_t);

                    if (!(delta(0, 0) == delta(0, 0)))
                    {
                        continue;
                    }
                    if (!(delta(1, 0) == delta(1, 0)))
                    {
                        continue;
                    }
                    if (!(delta(2, 0) == delta(2, 0)))
                    {
                        continue;
                    }

                    Eigen::Matrix<double, 3, 6> jacobian = get_point_to_point_jacobian_tait_bryan(pose_convention, point_clouds_container.point_clouds[i].m_pose, p_s, p_t);

                    int ir = num_obs;
                    int ic = i * 6;
                    for (int row = 0; row < 3; row++)
                    {
                        for (int col = 0; col < 6; col++)
                        {
                            if (jacobian(row, col) != 0.0)
                            {
                                tripletListA.emplace_back(ir + row, ic + col, -jacobian(row, col));
                            }
                        }
                    }

                    ssr += delta(0, 0) * delta(0, 0);
                    ssr += delta(1, 0) * delta(1, 0);
                    ssr += delta(2, 0) * delta(2, 0);
                    num_obs += 3;
                }
            }
        }
    }
    double sq = ssr / ((double)num_obs - point_clouds_container.point_clouds.size() * 6);

    Eigen::SparseMatrix<double> matA(num_obs, point_clouds_container.point_clouds.size() * 6);
    matA.setFromTriplets(tripletListA.begin(), tripletListA.end());

    Eigen::SparseMatrix<double> AtA(point_clouds_container.point_clouds.size() * 6, point_clouds_container.point_clouds.size() * 6);
    AtA = matA.transpose() * matA;
    tripletListA.clear();
    AtA = 0.5 * AtA;

    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtA);
    Eigen::SparseMatrix<double> I(point_clouds_container.point_clouds.size() * 6, point_clouds_container.point_clouds.size() * 6);
    I.setIdentity();

    Eigen::SparseMatrix<double> AtAinv(point_clouds_container.point_clouds.size() * 6, point_clouds_container.point_clouds.size() * 6);
    AtAinv = solver.solve(I);

    AtAinv = AtAinv * sq;

    for (int i = 0; i < point_clouds_container.point_clouds.size(); i++)
    {
        Eigen::Matrix<double, 6, 6, Eigen::RowMajor> cm;
        for (int r = 0; r < 6; r++)
        {
            for (int c = 0; c < 6; c++)
            {
                cm.coeffRef(r, c) = AtAinv.coeff(i * 6 + r, i * 6 + c);
            }
        }
        covariance_matrices.push_back(cm);
    }

    // clean
    for (auto &pc : point_clouds_container.point_clouds)
    {
        pc.clean();
    }
    return covariance_matrices;
}

std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> ICP::compute_covariance_matrices_rodrigues_point_to_point_source_to_target(
    PointClouds &point_clouds_container, PoseConvention pose_convention)
{
    for (auto &pc : point_clouds_container.point_clouds)
    {
        pc.build_rgd();
        pc.cout_rgd();
        pc.compute_normal_vectors(0.5);
    }

    std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covariance_matrices;
    std::vector<Eigen::Triplet<double>> tripletListA;

    double ssr = 0.0;
    int num_obs = 0;
    for (int i = 0; i < point_clouds_container.point_clouds.size(); i++)
    {
        for (int j = 0; j < point_clouds_container.point_clouds.size(); j++)
        {
            if (i != j)
            {
                std::vector<std::pair<int, int>> nns = point_clouds_container.point_clouds[i].nns(point_clouds_container.point_clouds[j], search_radious);

                for (size_t k = 0; k < nns.size(); k++)
                {
                    Eigen::Vector3d p_s(point_clouds_container.point_clouds[i].points_local[nns[k].first]);
                    Eigen::Vector3d p_t(point_clouds_container.point_clouds[j].m_pose * point_clouds_container.point_clouds[j].points_local[nns[k].second]);

                    Eigen::Matrix<double, 3, 1> delta = get_delta_point_to_point_rodrigues(pose_convention, point_clouds_container.point_clouds[i].m_pose, p_s, p_t);

                    if (!(delta(0, 0) == delta(0, 0)))
                    {
                        continue;
                    }
                    if (!(delta(1, 0) == delta(1, 0)))
                    {
                        continue;
                    }
                    if (!(delta(2, 0) == delta(2, 0)))
                    {
                        continue;
                    }

                    Eigen::Matrix<double, 3, 6> jacobian = get_point_to_point_jacobian_rodrigues(pose_convention, point_clouds_container.point_clouds[i].m_pose, p_s, p_t);

                    int ir = num_obs;
                    int ic = i * 6;
                    for (int row = 0; row < 3; row++)
                    {
                        for (int col = 0; col < 6; col++)
                        {
                            if (jacobian(row, col) != 0.0)
                            {
                                tripletListA.emplace_back(ir + row, ic + col, -jacobian(row, col));
                            }
                        }
                    }

                    ssr += delta(0, 0) * delta(0, 0);
                    ssr += delta(1, 0) * delta(1, 0);
                    ssr += delta(2, 0) * delta(2, 0);
                    num_obs += 3;
                }
            }
        }
    }
    double sq = ssr / ((double)num_obs - point_clouds_container.point_clouds.size() * 6);

    Eigen::SparseMatrix<double> matA(num_obs, point_clouds_container.point_clouds.size() * 6);
    matA.setFromTriplets(tripletListA.begin(), tripletListA.end());

    Eigen::SparseMatrix<double> AtA(point_clouds_container.point_clouds.size() * 6, point_clouds_container.point_clouds.size() * 6);
    AtA = matA.transpose() * matA;
    tripletListA.clear();
    AtA = 0.5 * AtA;

    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtA);
    Eigen::SparseMatrix<double> I(point_clouds_container.point_clouds.size() * 6, point_clouds_container.point_clouds.size() * 6);
    I.setIdentity();

    Eigen::SparseMatrix<double> AtAinv(point_clouds_container.point_clouds.size() * 6, point_clouds_container.point_clouds.size() * 6);
    AtAinv = solver.solve(I);

    AtAinv = AtAinv * sq;

    for (int i = 0; i < point_clouds_container.point_clouds.size(); i++)
    {
        Eigen::Matrix<double, 6, 6, Eigen::RowMajor> cm;
        for (int r = 0; r < 6; r++)
        {
            for (int c = 0; c < 6; c++)
            {
                cm.coeffRef(r, c) = AtAinv.coeff(i * 6 + r, i * 6 + c);
            }
        }
        covariance_matrices.push_back(cm);
    }

    // clean
    for (auto &pc : point_clouds_container.point_clouds)
    {
        pc.clean();
    }
    return covariance_matrices;
}

std::vector<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>> ICP::compute_covariance_matrices_quaternion_point_to_point_source_to_target(
    PointClouds &point_clouds_container, PoseConvention pose_convention)
{
    for (auto &pc : point_clouds_container.point_clouds)
    {
        pc.build_rgd();
        pc.cout_rgd();
        pc.compute_normal_vectors(0.5);
    }

    std::vector<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>> covariance_matrices;

    std::vector<Eigen::Triplet<double>> tripletListA;

    double ssr = 0.0;
    int num_obs = 0;
    for (int i = 0; i < point_clouds_container.point_clouds.size(); i++)
    {
        for (int j = 0; j < point_clouds_container.point_clouds.size(); j++)
        {
            if (i != j)
            {
                std::vector<std::pair<int, int>> nns = point_clouds_container.point_clouds[i].nns(point_clouds_container.point_clouds[j], search_radious);

                for (size_t k = 0; k < nns.size(); k++)
                {
                    Eigen::Vector3d p_s(point_clouds_container.point_clouds[i].points_local[nns[k].first]);
                    Eigen::Vector3d p_t(point_clouds_container.point_clouds[j].m_pose * point_clouds_container.point_clouds[j].points_local[nns[k].second]);

                    Eigen::Matrix<double, 3, 1> delta = get_delta_point_to_point_quaternion(pose_convention, point_clouds_container.point_clouds[i].m_pose, p_s, p_t);

                    if (!(delta(0, 0) == delta(0, 0)))
                    {
                        continue;
                    }
                    if (!(delta(1, 0) == delta(1, 0)))
                    {
                        continue;
                    }
                    if (!(delta(2, 0) == delta(2, 0)))
                    {
                        continue;
                    }

                    Eigen::Matrix<double, 3, 7> jacobian = get_point_to_point_jacobian_quaternion(pose_convention, point_clouds_container.point_clouds[i].m_pose, p_s, p_t);

                    int ir = num_obs;
                    int ic = i * 7;
                    for (int row = 0; row < 3; row++)
                    {
                        for (int col = 0; col < 7; col++)
                        {
                            if (jacobian(row, col) != 0.0)
                            {
                                tripletListA.emplace_back(ir + row, ic + col, -jacobian(row, col));
                            }
                        }
                    }

                    ssr += delta(0, 0) * delta(0, 0);
                    ssr += delta(1, 0) * delta(1, 0);
                    ssr += delta(2, 0) * delta(2, 0);
                    num_obs += 3;
                }
            }
        }
    }
    double sq = ssr / ((double)num_obs - point_clouds_container.point_clouds.size() * 7);

    Eigen::SparseMatrix<double> matA(num_obs, point_clouds_container.point_clouds.size() * 7);
    matA.setFromTriplets(tripletListA.begin(), tripletListA.end());

    Eigen::SparseMatrix<double> AtA(point_clouds_container.point_clouds.size() * 7, point_clouds_container.point_clouds.size() * 7);
    AtA = matA.transpose() * matA;
    tripletListA.clear();
    AtA = 0.5 * AtA;

    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtA);
    Eigen::SparseMatrix<double> I(point_clouds_container.point_clouds.size() * 7, point_clouds_container.point_clouds.size() * 7);
    I.setIdentity();

    Eigen::SparseMatrix<double> AtAinv(point_clouds_container.point_clouds.size() * 7, point_clouds_container.point_clouds.size() * 7);
    AtAinv = solver.solve(I);

    AtAinv = AtAinv * sq;

    for (int i = 0; i < point_clouds_container.point_clouds.size(); i++)
    {
        Eigen::Matrix<double, 7, 7, Eigen::RowMajor> cm;
        for (int r = 0; r < 7; r++)
        {
            for (int c = 0; c < 7; c++)
            {
                cm.coeffRef(r, c) = AtAinv.coeff(i * 7 + r, i * 7 + c);
            }
        }
        covariance_matrices.push_back(cm);
    }

    // clean
    for (auto &pc : point_clouds_container.point_clouds)
    {
        pc.clean();
    }

    return covariance_matrices;
}

bool ICP::optimize_source_to_target_lie_algebra_left_jacobian(PointClouds &point_clouds_container)
{
    // std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covariance_matrices_before6x6;
    // std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covariance_matrices_after6x6;

    // covariance_matrices_before6x6 = compute_covariance_matrices_point_to_point_source_to_target_source_to_target_lie_algebra_left_jacobian(point_clouds_container);

    optimize_source_to_target_lie_algebra_left_jacobian(point_clouds_container, is_fix_first_node);

    // covariance_matrices_after6x6 = compute_covariance_matrices_point_to_point_source_to_target_source_to_target_lie_algebra_left_jacobian(point_clouds_container);

    // double mui = get_mean_uncertainty_xyz_impact6x6(covariance_matrices_before6x6, covariance_matrices_after6x6);
    // std::cout << "mean uncertainty_xyz impact: " << mui << std::endl;
    return true;
}

std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> ICP::compute_covariance_matrices_point_to_point_source_to_target_source_to_target_lie_algebra_left_jacobian(PointClouds &point_clouds_container)
{
    for (auto &pc : point_clouds_container.point_clouds)
    {
        pc.build_rgd();
        pc.cout_rgd();
        pc.compute_normal_vectors(0.5);
    }

    std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covariance_matrices;
    std::vector<Eigen::Triplet<double>> tripletListA;

    double ssr = 0.0;
    int num_obs = 0;
    for (int i = 0; i < point_clouds_container.point_clouds.size(); i++)
    {
        for (int j = 0; j < point_clouds_container.point_clouds.size(); j++)
        {
            if (i != j)
            {
                std::vector<std::pair<int, int>> nns = point_clouds_container.point_clouds[i].nns(point_clouds_container.point_clouds[j], search_radious);
                TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);

                for (size_t k = 0; k < nns.size(); k++)
                {
                    Eigen::Vector3d p_s(point_clouds_container.point_clouds[i].points_local[nns[k].first]);
                    Eigen::Vector3d p_t(point_clouds_container.point_clouds[j].m_pose * point_clouds_container.point_clouds[j].points_local[nns[k].second]);

                    Eigen::Matrix3d R = point_clouds_container.point_clouds[i].m_pose.rotation();
                    Eigen::Vector3d Rp = R * p_s;
                    Eigen::Matrix3d Rpx;
                    Rpx(0, 0) = 0;
                    Rpx(0, 1) = -Rp.z();
                    Rpx(0, 2) = Rp.y();
                    Rpx(1, 0) = Rp.z();
                    Rpx(1, 1) = 0;
                    Rpx(1, 2) = -Rp.x();
                    Rpx(2, 0) = -Rp.y();
                    Rpx(2, 1) = Rp.x();
                    Rpx(2, 2) = 0;

                    int ir = num_obs;
                    int ic = i * 6;

                    tripletListA.emplace_back(ir, ic + 0, 1);
                    // tripletListA.emplace_back(ir, ic + 1, 0);
                    // tripletListA.emplace_back(ir, ic + 2, 0);
                    tripletListA.emplace_back(ir, ic + 3, -Rpx(0, 0));
                    tripletListA.emplace_back(ir, ic + 4, -Rpx(0, 1));
                    tripletListA.emplace_back(ir, ic + 5, -Rpx(0, 2));

                    // tripletListA.emplace_back(ir + 1, ic + 0, 0);
                    tripletListA.emplace_back(ir + 1, ic + 1, 1);
                    // tripletListA.emplace_back(ir + 1, ic + 2, 0);
                    tripletListA.emplace_back(ir + 1, ic + 3, -Rpx(1, 0));
                    tripletListA.emplace_back(ir + 1, ic + 4, -Rpx(1, 1));
                    tripletListA.emplace_back(ir + 1, ic + 5, -Rpx(1, 2));

                    // tripletListA.emplace_back(ir + 2, ic + 0, 0);
                    // tripletListA.emplace_back(ir + 2, ic + 1, 0);
                    tripletListA.emplace_back(ir + 2, ic + 2, 1);
                    tripletListA.emplace_back(ir + 2, ic + 3, -Rpx(2, 0));
                    tripletListA.emplace_back(ir + 2, ic + 4, -Rpx(2, 1));
                    tripletListA.emplace_back(ir + 2, ic + 5, -Rpx(2, 2));

                    Eigen::Vector3d target = p_t;
                    Eigen::Vector3d source = point_clouds_container.point_clouds[i].m_pose * p_s;

                    double delta_x = target.x() - source.x();
                    double delta_y = target.y() - source.y();
                    double delta_z = target.z() - source.z();

                    ssr += delta_x * delta_x;
                    ssr += delta_y * delta_y;
                    ssr += delta_z * delta_z;
                    num_obs += 3;
                }
            }
        }
    }
    double sq = ssr / ((double)num_obs - point_clouds_container.point_clouds.size() * 6);

    Eigen::SparseMatrix<double> matA(num_obs, point_clouds_container.point_clouds.size() * 6);
    matA.setFromTriplets(tripletListA.begin(), tripletListA.end());

    Eigen::SparseMatrix<double> AtA(point_clouds_container.point_clouds.size() * 6, point_clouds_container.point_clouds.size() * 6);
    AtA = matA.transpose() * matA;
    tripletListA.clear();
    AtA = 0.5 * AtA;

    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtA);
    Eigen::SparseMatrix<double> I(point_clouds_container.point_clouds.size() * 6, point_clouds_container.point_clouds.size() * 6);
    I.setIdentity();

    Eigen::SparseMatrix<double> AtAinv(point_clouds_container.point_clouds.size() * 6, point_clouds_container.point_clouds.size() * 6);
    AtAinv = solver.solve(I);
    AtAinv = AtAinv * sq;

    for (int i = 0; i < point_clouds_container.point_clouds.size(); i++)
    {
        Eigen::Matrix<double, 6, 6, Eigen::RowMajor> cm;
        for (int r = 0; r < 6; r++)
        {
            for (int c = 0; c < 6; c++)
            {
                cm.coeffRef(r, c) = AtAinv.coeff(i * 6 + r, i * 6 + c);
            }
        }
        covariance_matrices.push_back(cm);
    }

    // clean
    for (auto &pc : point_clouds_container.point_clouds)
    {
        pc.clean();
    }
    return covariance_matrices;
}

bool ICP::optimize_source_to_target_lie_algebra_right_jacobian(PointClouds &point_clouds_container)
{
    // std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covariance_matrices_before6x6;
    // std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covariance_matrices_after6x6;

    // covariance_matrices_before6x6 = compute_covariance_matrices_point_to_point_source_to_target_source_to_target_lie_algebra_right_jacobian(point_clouds_container);

    optimize_source_to_target_lie_algebra_right_jacobian(point_clouds_container, is_fix_first_node);

    // covariance_matrices_after6x6 = compute_covariance_matrices_point_to_point_source_to_target_source_to_target_lie_algebra_right_jacobian(point_clouds_container);

    // double mui = get_mean_uncertainty_xyz_impact6x6(covariance_matrices_before6x6, covariance_matrices_after6x6);
    // std::cout << "mean uncertainty_xyz impact: " << mui << std::endl;
    return true;
}

std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> ICP::compute_covariance_matrices_point_to_point_source_to_target_source_to_target_lie_algebra_right_jacobian(PointClouds &point_clouds_container)
{
    for (auto &pc : point_clouds_container.point_clouds)
    {
        pc.build_rgd();
        pc.cout_rgd();
        pc.compute_normal_vectors(0.5);
    }

    std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covariance_matrices;
    std::vector<Eigen::Triplet<double>> tripletListA;

    double ssr = 0.0;
    int num_obs = 0;
    for (int i = 0; i < point_clouds_container.point_clouds.size(); i++)
    {
        for (int j = 0; j < point_clouds_container.point_clouds.size(); j++)
        {
            if (i != j)
            {
                std::vector<std::pair<int, int>> nns = point_clouds_container.point_clouds[i].nns(point_clouds_container.point_clouds[j], search_radious);
                TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);

                for (size_t k = 0; k < nns.size(); k++)
                {
                    Eigen::Vector3d p_s(point_clouds_container.point_clouds[i].points_local[nns[k].first]);
                    Eigen::Vector3d p_t(point_clouds_container.point_clouds[j].m_pose * point_clouds_container.point_clouds[j].points_local[nns[k].second]);

                    Eigen::Matrix3d px;
                    px(0, 0) = 0;
                    px(0, 1) = -p_s.z();
                    px(0, 2) = p_s.y();
                    px(1, 0) = p_s.z();
                    px(1, 1) = 0;
                    px(1, 2) = -p_s.x();
                    px(2, 0) = -p_s.y();
                    px(2, 1) = p_s.x();
                    px(2, 2) = 0;
                    Eigen::Matrix3d R = point_clouds_container.point_clouds[i].m_pose.rotation();
                    Eigen::Matrix3d Rpx = R * px;

                    int ir = num_obs;
                    int ic = i * 6;

                    tripletListA.emplace_back(ir, ic + 0, R(0, 0));
                    tripletListA.emplace_back(ir, ic + 1, R(0, 1));
                    tripletListA.emplace_back(ir, ic + 2, R(0, 2));
                    tripletListA.emplace_back(ir, ic + 3, -Rpx(0, 0));
                    tripletListA.emplace_back(ir, ic + 4, -Rpx(0, 1));
                    tripletListA.emplace_back(ir, ic + 5, -Rpx(0, 2));

                    tripletListA.emplace_back(ir + 1, ic + 0, R(1, 0));
                    tripletListA.emplace_back(ir + 1, ic + 1, R(1, 1));
                    tripletListA.emplace_back(ir + 1, ic + 2, R(1, 2));
                    tripletListA.emplace_back(ir + 1, ic + 3, -Rpx(1, 0));
                    tripletListA.emplace_back(ir + 1, ic + 4, -Rpx(1, 1));
                    tripletListA.emplace_back(ir + 1, ic + 5, -Rpx(1, 2));

                    tripletListA.emplace_back(ir + 2, ic + 0, R(2, 0));
                    tripletListA.emplace_back(ir + 2, ic + 1, R(2, 1));
                    tripletListA.emplace_back(ir + 2, ic + 2, R(2, 2));
                    tripletListA.emplace_back(ir + 2, ic + 3, -Rpx(2, 0));
                    tripletListA.emplace_back(ir + 2, ic + 4, -Rpx(2, 1));
                    tripletListA.emplace_back(ir + 2, ic + 5, -Rpx(2, 2));

                    Eigen::Vector3d target = p_t;
                    Eigen::Vector3d source = point_clouds_container.point_clouds[i].m_pose * p_s;

                    double delta_x = target.x() - source.x();
                    double delta_y = target.y() - source.y();
                    double delta_z = target.z() - source.z();

                    ssr += delta_x * delta_x;
                    ssr += delta_y * delta_y;
                    ssr += delta_z * delta_z;
                    num_obs += 3;
                }
            }
        }
    }
    double sq = ssr / ((double)num_obs - point_clouds_container.point_clouds.size() * 6);

    Eigen::SparseMatrix<double> matA(num_obs, point_clouds_container.point_clouds.size() * 6);
    matA.setFromTriplets(tripletListA.begin(), tripletListA.end());

    Eigen::SparseMatrix<double> AtA(point_clouds_container.point_clouds.size() * 6, point_clouds_container.point_clouds.size() * 6);
    AtA = matA.transpose() * matA;
    tripletListA.clear();
    AtA = 0.5 * AtA;

    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtA);
    Eigen::SparseMatrix<double> I(point_clouds_container.point_clouds.size() * 6, point_clouds_container.point_clouds.size() * 6);
    I.setIdentity();

    Eigen::SparseMatrix<double> AtAinv(point_clouds_container.point_clouds.size() * 6, point_clouds_container.point_clouds.size() * 6);
    AtAinv = solver.solve(I);
    AtAinv = AtAinv * sq;

    for (int i = 0; i < point_clouds_container.point_clouds.size(); i++)
    {
        Eigen::Matrix<double, 6, 6, Eigen::RowMajor> cm;
        for (int r = 0; r < 6; r++)
        {
            for (int c = 0; c < 6; c++)
            {
                cm.coeffRef(r, c) = AtAinv.coeff(i * 6 + r, i * 6 + c);
            }
        }
        covariance_matrices.push_back(cm);
    }

    // clean
    for (auto &pc : point_clouds_container.point_clouds)
    {
        pc.clean();
    }
    return covariance_matrices;
}