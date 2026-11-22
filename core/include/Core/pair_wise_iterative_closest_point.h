#pragma once

#include <Core/structures.h>

#include <Eigen/Eigen>
#include <vector>

#include <limits>

//! Residuals behind an ICP uncertainty estimate.
struct IcpResidualStatistics
{
    double sum_squared_residuals = 0.0; //!< [m^2]
    int number_of_correspondences = 0;
    int number_of_observations = 0; //!< 3 per correspondence for point-to-point, 1 for point-to-plane
    //! Standard deviation of unit weight, sqrt(sum_squared_residuals / (number_of_observations - 6)) [m]; NaN when
    //! there are not more observations than the 6 pose parameters.
    double sigma0 = std::numeric_limits<double>::quiet_NaN();
};

class PairWiseICP
{
public:
    PairWiseICP() = default;
    ~PairWiseICP() = default;

    bool compute(
        const std::vector<Eigen::Vector3d>& source,
        const std::vector<Eigen::Vector3d>& target,
        double search_radious,
        int number_of_iterations,
        Eigen::Affine3d& m_pose_result);
    //! Pose standard deviations (px, py, pz in metres; om, fi, ka in radians) of an aligned source from point-to-point
    //! residuals to the nearest target point within search_radious. Covariance = sigma0^2 * (J^T J)^-1,
    //! sigma0^2 = sum r^2 / (3n - 6). If statistics is given, the residual statistics are written there.
    TaitBryanPose compute_icp_uncertainty(
        const std::vector<Eigen::Vector3d>& source,
        const std::vector<Eigen::Vector3d>& target,
        const Eigen::Affine3d& m_pose_result,
        double search_radious,
        IcpResidualStatistics* statistics = nullptr);

    //! Pose standard deviations (px, py, pz in metres; om, fi, ka in radians) of an aligned source from point-to-plane
    //! residuals: each source point is matched to the plane fitted to the target points within search_radious
    //! (non-planar neighbourhoods are skipped). Covariance = sigma0^2 * (J^T J)^-1, sigma0^2 = sum r^2 / (n - 6).
    //! Returns infinity for every parameter when there are too few correspondences or the geometry is degenerate.
    //! If statistics is given, the residual statistics are written there.
    TaitBryanPose compute_icp_uncertainty_point_to_plane(
        const std::vector<Eigen::Vector3d>& source,
        const std::vector<Eigen::Vector3d>& target,
        const Eigen::Affine3d& m_pose_result,
        double search_radious,
        IcpResidualStatistics* statistics = nullptr);
};
