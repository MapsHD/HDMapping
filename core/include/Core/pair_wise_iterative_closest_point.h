#pragma once

#include <Core/structures.h>

#include <Eigen/Eigen>
#include <functional>
#include <vector>

#include <limits>
#include <utility>

//! Residuals and pose covariance behind an ICP uncertainty estimate.
struct IcpResidualStatistics
{
    double sum_squared_residuals = 0.0; //!< [m^2]
    int number_of_correspondences = 0;
    int number_of_observations = 0; //!< 3 per correspondence for point-to-point, 1 for point-to-plane
    //! Standard deviation of unit weight, sqrt(sum_squared_residuals / (number_of_observations - 6)) [m]; NaN when
    //! there are not more observations than the 6 pose parameters.
    double sigma0 = std::numeric_limits<double>::quiet_NaN();
    //! Pose covariance sigma0^2 * (J^T J)^-1, rows and columns px, py, pz, om, fi, ka ([m^2], [m*rad], [rad^2]); NaN
    //! when it is not computed (too few correspondences, degenerate geometry).
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Constant(std::numeric_limits<double>::quiet_NaN());
};

//! Spread of an ICP result over runs started from randomly perturbed poses (Monte Carlo). Parameters are in the order
//! px, py, pz [m], om, fi, ka [rad].
struct IcpSpread
{
    int number_of_runs = 0;
    int number_of_converged_runs = 0; //!< runs where PairWiseICP::compute() succeeded
    //! Sample standard deviations of the starting offsets from the reference pose.
    Eigen::Matrix<double, 6, 1> start_sigma = Eigen::Matrix<double, 6, 1>::Constant(std::numeric_limits<double>::quiet_NaN());
    //! Mean converged pose minus the reference one.
    Eigen::Matrix<double, 6, 1> mean_offset = Eigen::Matrix<double, 6, 1>::Constant(std::numeric_limits<double>::quiet_NaN());
    //! Sample covariance of the converged poses; NaN with fewer than 2 converged runs.
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Constant(std::numeric_limits<double>::quiet_NaN());
};

class PairWiseICP
{
public:
    PairWiseICP() = default;
    ~PairWiseICP() = default;

    //! false: compute() and compute_icp_montecarlo() print only failures, not every iteration.
    bool verbose = true;

    bool compute(
        const std::vector<Eigen::Vector3d>& source,
        const std::vector<Eigen::Vector3d>& target,
        double search_radious,
        int number_of_iterations,
        Eigen::Affine3d& m_pose_result);
    //! Pose standard deviations (px, py, pz in metres; om, fi, ka in radians) of an aligned source from point-to-point
    //! residuals to the nearest target point within search_radious. Covariance = sigma0^2 * (J^T J)^-1,
    //! sigma0^2 = sum r^2 / (3n - 6).
    //! @return the standard deviations and the residual statistics with the covariance
    std::pair<TaitBryanPose, IcpResidualStatistics> compute_icp_uncertainty(
        const std::vector<Eigen::Vector3d>& source,
        const std::vector<Eigen::Vector3d>& target,
        const Eigen::Affine3d& m_pose_result,
        double search_radious);

    //! Pose standard deviations (px, py, pz in metres; om, fi, ka in radians) of an aligned source from point-to-plane
    //! residuals: each source point is matched to the plane fitted to the target points within search_radious
    //! (non-planar neighbourhoods are skipped). Covariance = sigma0^2 * (J^T J)^-1, sigma0^2 = sum r^2 / (n - 6).
    //! Returns infinity for every parameter when there are too few correspondences or the geometry is degenerate.
    //! @return the standard deviations and the residual statistics with the covariance
    std::pair<TaitBryanPose, IcpResidualStatistics> compute_icp_uncertainty_point_to_plane(
        const std::vector<Eigen::Vector3d>& source,
        const std::vector<Eigen::Vector3d>& target,
        const Eigen::Affine3d& m_pose_result,
        double search_radious);

    //! Monte Carlo ICP: runs compute() number_of_runs times from m_pose plus a random Tait-Bryan offset, uniform in
    //! [-start_offset_m, start_offset_m] on px, py, pz and [-start_offset_rad, start_offset_rad] on om, fi, ka, and
    //! returns the spread of the results around m_pose. The offsets come from a fixed seed, so the result is
    //! repeatable. Takes time.
    //! @param on_run_done if given, called after each run with the number of runs done
    IcpSpread compute_icp_montecarlo(
        const std::vector<Eigen::Vector3d>& source,
        const std::vector<Eigen::Vector3d>& target,
        const Eigen::Affine3d& m_pose,
        double search_radious,
        int number_of_iterations,
        int number_of_runs,
        double start_offset_m,
        double start_offset_rad,
        const std::function<void(int run)>& on_run_done = {});
};
