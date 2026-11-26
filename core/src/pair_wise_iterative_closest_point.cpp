#include <pch/pch.h>

#include <Core/hash_utils.h>
#include <Core/pair_wise_iterative_closest_point.h>
#include <Core/structures.h>
#include <Core/transformations.h>

#include <tbb/parallel_for_each.h>

#include <cmath>
#include <limits>
#include <random>

#include <python-scripts/point-to-feature-metrics/point_to_plane_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian_simplified.h>

bool PairWiseICP::compute(
    const std::vector<Eigen::Vector3d>& source,
    const std::vector<Eigen::Vector3d>& target,
    double search_radious,
    int number_of_iterations,
    Eigen::Affine3d& m_pose_result)
{
    if (verbose)
        std::cout << "PairWiseICP::compute" << std::endl;
    bool multithread = true;

    // The target and the search radius don't change between iterations, so its bucket index is built once.
    std::vector<std::pair<uint64_t, uint32_t>> indexes;

    for (int i = 0; i < target.size(); i++)
    {
        uint64_t index = get_rgd_index_3d(target[i], { search_radious, search_radious, search_radious });
        indexes.emplace_back(index, i);
    }

    // Ties broken by point index: std::sort leaves the order within a bucket to the standard library, and the
    // search below skips each bucket's last point, so without this the result differs between platforms.
    std::sort(
        indexes.begin(),
        indexes.end(),
        [](const std::pair<uint64_t, uint32_t>& a, const std::pair<uint64_t, uint32_t>& b)
        {
            return a.first != b.first ? a.first < b.first : a.second < b.second;
        });

    std::unordered_map<uint64_t, std::pair<uint32_t, uint32_t>> buckets;

    for (uint32_t i = 0; i < indexes.size(); i++)
    {
        uint64_t index_of_bucket = indexes[i].first;
        if (buckets.contains(index_of_bucket))
        {
            buckets[index_of_bucket].second = i;
        }
        else
        {
            buckets[index_of_bucket].first = i;
            buckets[index_of_bucket].second = i;
        }
    }

    // Target points in bucket order, so a bucket's points are read from contiguous memory.
    std::vector<Eigen::Vector3d> target_sorted(indexes.size());
    for (size_t i = 0; i < indexes.size(); i++)
    {
        target_sorted[i] = target[indexes[i].second];
    }
    const double search_radious_squared = search_radious * search_radious;

    // Normal equations are summed per block of source points, then the blocks in order: no lock per point, and
    // the same sum on every run and machine.
    struct NormalEquations
    {
        Eigen::Matrix<double, 6, 6> AtPA = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> AtPB = Eigen::Matrix<double, 6, 1>::Zero();
        int counter_nn = 0;
    };
    constexpr size_t number_of_blocks = 256;
    const size_t block_size = (source.size() + number_of_blocks - 1) / number_of_blocks;

    for (int iter = 0; iter < number_of_iterations; iter++)
    {
        if (verbose)
            std::cout << "iteration: " << iter + 1 << " of: " << number_of_iterations << std::endl;
        Eigen::MatrixXd AtPA(6, 6);
        AtPA.setZero();
        Eigen::MatrixXd AtPB(6, 1);
        AtPB.setZero();
        Eigen::Vector3d b(search_radious, search_radious, search_radious);

        const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose_result);
        std::vector<NormalEquations> block_sums(number_of_blocks);

        const auto hessian_fun = [&](const Eigen::Vector3d& source_i, NormalEquations& sum)
        {
            if (source_i.norm() < 0.1)
            {
                return;
            }

            Eigen::Vector3d source_point_global = m_pose_result * source_i;

            double min_dist_squared = 1000000000.0;
            Eigen::Vector3d target_i_nn;
            bool nn_found = false;
            for (double x = -search_radious; x <= search_radious; x += search_radious)
            {
                for (double y = -search_radious; y <= search_radious; y += search_radious)
                {
                    for (double z = -search_radious; z <= search_radious; z += search_radious)
                    {
                        Eigen::Vector3d source_point_global_ext = source_point_global + Eigen::Vector3d(x, y, z);
                        uint64_t index_of_bucket =
                            get_rgd_index_3d(source_point_global_ext, { search_radious, search_radious, search_radious });

                        const auto bucket = buckets.find(index_of_bucket);
                        if (bucket != buckets.end())
                        {
                            for (int index = bucket->second.first; index <= bucket->second.second; index++)
                            {
                                const Eigen::Vector3d& target_i = target_sorted[index];
                                const double d_squared = (source_point_global - target_i).squaredNorm();

                                if (d_squared < search_radious_squared)
                                {
                                    if (d_squared < min_dist_squared)
                                    {
                                        min_dist_squared = d_squared;
                                        nn_found = true;
                                        target_i_nn = target_i;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            if (nn_found)
            {
                const Eigen::Vector3d& p_s = source_i;

                Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AtPA_;
                point_to_point_source_to_target_tait_bryan_wc_AtPA_simplified(
                    AtPA_,
                    pose_s.px,
                    pose_s.py,
                    pose_s.pz,
                    pose_s.om,
                    pose_s.fi,
                    pose_s.ka,
                    p_s.x(),
                    p_s.y(),
                    p_s.z(),
                    1,
                    0,
                    0,
                    0,
                    1,
                    0,
                    0,
                    0,
                    1);

                Eigen::Matrix<double, 6, 1> AtPB_;
                point_to_point_source_to_target_tait_bryan_wc_AtPB_simplified(
                    AtPB_,
                    pose_s.px,
                    pose_s.py,
                    pose_s.pz,
                    pose_s.om,
                    pose_s.fi,
                    pose_s.ka,
                    p_s.x(),
                    p_s.y(),
                    p_s.z(),
                    1,
                    0,
                    0,
                    0,
                    1,
                    0,
                    0,
                    0,
                    1,
                    target_i_nn.x(),
                    target_i_nn.y(),
                    target_i_nn.z());

                sum.AtPA += AtPA_;
                sum.AtPB -= AtPB_;
                sum.counter_nn++;
            }
        };

        const auto block_fun = [&](NormalEquations& sum)
        {
            const size_t block = &sum - block_sums.data();
            const size_t begin = std::min(source.size(), block * block_size);
            const size_t end = std::min(source.size(), begin + block_size);
            for (size_t i = begin; i < end; i++)
            {
                hessian_fun(source[i], sum);
            }
        };

        if (multithread)
        {
            // TBB rather than std::execution, which AppleClang's libc++ doesn't run in parallel.
            tbb::parallel_for_each(block_sums.begin(), block_sums.end(), block_fun);
        }
        else
        {
            std::for_each(block_sums.begin(), block_sums.end(), block_fun);
        }

        int counter_nn = 0;
        for (const auto& sum : block_sums)
        {
            AtPA += sum.AtPA;
            AtPB += sum.AtPB;
            counter_nn += sum.counter_nn;
        }

        if (AtPB(0, 0) == 0.0 && AtPB(1, 0) == 0.0 && AtPB(2, 0) == 0.0 && AtPB(3, 0) == 0.0 && AtPB(4, 0) == 0.0 && AtPB(5, 0) == 0.0)
        {
            std::cout
                << "PairWiseICP::compute FAILED (Relative pose is not changed/solved/optimized --> please consider removing this edge)"
                << std::endl;
            return false;
        }

        if (counter_nn < 100)
        {
            std::cout << "not suficient number of observations" << std::endl;
            std::cout
                << "PairWiseICP::compute FAILED (Relative pose is not changed/solved/optimized --> please consider removing this edge)"
                << std::endl;
            return false;
        }

        Eigen::SparseMatrix<double> AtPAc(6, 6);
        Eigen::SparseMatrix<double> AtPBc(6, 1);

        AtPAc = AtPA.sparseView();
        AtPBc = AtPB.sparseView();

        Eigen::SparseMatrix<double> AtPA_I(6, 6);
        AtPA_I.setIdentity();
        AtPA += (AtPA_I * 100);

        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPAc);
        Eigen::SparseMatrix<double> x = solver.solve(AtPBc);
        std::vector<double> h_x;
        for (int k = 0; k < x.outerSize(); ++k)
        {
            if (verbose)
                std::cout << "result pose updates" << std::endl;
            for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            {
                h_x.push_back(it.value());
                if (verbose)
                    std::cout << it.row() << " " << it.col() << " " << it.value() << std::endl;
            }
        }

        if (h_x.size() == 6)
        {
            int counter = 0;
            TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(m_pose_result);

            pose.px += h_x[counter++];
            pose.py += h_x[counter++];
            pose.pz += h_x[counter++];
            pose.om += h_x[counter++];
            pose.fi += h_x[counter++];
            pose.ka += h_x[counter++];

            m_pose_result = affine_matrix_from_pose_tait_bryan(pose);
            if (verbose)
                std::cout << "PairWiseICP::compute SUCCESS" << std::endl;
            // return true;
        }
        else
        {
            std::cout
                << "PairWiseICP::compute FAILED (Relative pose is not changed/solved/optimized --> please consider removing this edge)"
                << std::endl;
            return false;
        }
    }

    return true;
}

namespace
{
    TaitBryanPose infinite_uncertainty()
    {
        constexpr double inf = std::numeric_limits<double>::infinity();
        TaitBryanPose sigma;
        sigma.px = sigma.py = sigma.pz = sigma.om = sigma.fi = sigma.ka = inf;
        return sigma;
    }

    //! Bucket grid over a target cloud with the cell size equal to the search radius, so every neighbour of a query
    //! point lies in the 27 cells around it.
    class TargetGrid
    {
    public:
        TargetGrid(const std::vector<Eigen::Vector3d>& target, double search_radious)
            : radius(search_radious)
            , radius_squared(search_radious * search_radious)
            , cell(search_radious, search_radious, search_radious)
        {
            std::vector<std::pair<uint64_t, uint32_t>> indexes;
            indexes.reserve(target.size());
            for (uint32_t i = 0; i < target.size(); i++)
            {
                indexes.emplace_back(get_rgd_index_3d(target[i], cell), i);
            }
            // Ties broken by point index so the order within a bucket is the same on every platform.
            std::sort(indexes.begin(), indexes.end());

            sorted.resize(indexes.size());
            for (uint32_t i = 0; i < indexes.size(); i++)
            {
                sorted[i] = target[indexes[i].second];
                const auto [bucket, inserted] = buckets.try_emplace(indexes[i].first, i, i);
                bucket->second.second = i;
            }
        }

        //! Calls f(target_point, squared_distance) for every target point closer to query than the search radius.
        template<typename F>
        void for_each_neighbour(const Eigen::Vector3d& query, F&& f) const
        {
            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        const Eigen::Vector3d shifted = query + radius * Eigen::Vector3d(dx, dy, dz);
                        const auto bucket = buckets.find(get_rgd_index_3d(shifted, cell));
                        if (bucket == buckets.end())
                        {
                            continue;
                        }
                        for (uint32_t index = bucket->second.first; index <= bucket->second.second; index++)
                        {
                            const double d_squared = (query - sorted[index]).squaredNorm();
                            if (d_squared < radius_squared)
                            {
                                f(sorted[index], d_squared);
                            }
                        }
                    }
                }
            }
        }

    private:
        double radius;
        double radius_squared;
        Eigen::Vector3d cell;
        std::vector<Eigen::Vector3d> sorted;
        std::unordered_map<uint64_t, std::pair<uint32_t, uint32_t>> buckets;
    };

    //! Writes the residual statistics (sigma0 = sqrt(sum of squared residuals / (number_of_observations - 6))) and
    //! resets the covariance to NaN.
    void fill_residual_statistics(
        IcpResidualStatistics& residual_statistics, double sum_squared_residuals, int number_of_correspondences, int number_of_observations)
    {
        residual_statistics.sum_squared_residuals = sum_squared_residuals;
        residual_statistics.number_of_correspondences = number_of_correspondences;
        residual_statistics.number_of_observations = number_of_observations;
        residual_statistics.sigma0 = number_of_observations > 6 ? std::sqrt(sum_squared_residuals / (number_of_observations - 6))
                                                                : std::numeric_limits<double>::quiet_NaN();
        // Set by compute_icp_uncertainty_from_hessian().
        residual_statistics.covariance.setConstant(std::numeric_limits<double>::quiet_NaN());
    }

    TaitBryanPose compute_icp_uncertainty_from_hessian(
        const Eigen::Matrix<double, 6, 6>& AtPA,
        int number_of_observations,
        double sum_squared_residuals,
        IcpResidualStatistics& statistics)
    {
        TaitBryanPose sigma = infinite_uncertainty();

        // The pose has 6 unknowns.
        const int redundancy = number_of_observations - 6;
        if (redundancy <= 0)
        {
            return sigma;
        }

        // AtPA is symmetric positive semi-definite; the eigen decomposition exposes unconstrained directions.
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen_solver(AtPA);
        if (eigen_solver.info() != Eigen::Success)
        {
            return sigma;
        }
        const Eigen::Matrix<double, 6, 1>& eigenvalues = eigen_solver.eigenvalues(); // ascending
        if (eigenvalues(0) <= 1e-12 * eigenvalues(5))
        {
            std::cout << "compute_icp_uncertainty_from_hessian: degenerate geometry, pose not fully constrained" << std::endl;
            return sigma;
        }

        const double sigma0_squared = sum_squared_residuals / redundancy;
        const Eigen::Matrix<double, 6, 6>& V = eigen_solver.eigenvectors();
        const Eigen::Matrix<double, 6, 6> covariance = sigma0_squared * V * eigenvalues.cwiseInverse().asDiagonal() * V.transpose();
        statistics.covariance = covariance;

        sigma.px = std::sqrt(covariance(0, 0));
        sigma.py = std::sqrt(covariance(1, 1));
        sigma.pz = std::sqrt(covariance(2, 2));
        sigma.om = std::sqrt(covariance(3, 3));
        sigma.fi = std::sqrt(covariance(4, 4));
        sigma.ka = std::sqrt(covariance(5, 5));
        return sigma;
    }
} // namespace

std::pair<TaitBryanPose, IcpResidualStatistics> PairWiseICP::compute_icp_uncertainty(
    const std::vector<Eigen::Vector3d>& source,
    const std::vector<Eigen::Vector3d>& target,
    const Eigen::Affine3d& m_pose_result,
    double search_radious)
{
    bool multithread = true;
    IcpResidualStatistics icp_residual_statistics;
    TaitBryanPose icp_uncertainty;
    icp_uncertainty.px = 0;
    icp_uncertainty.py = 0;
    icp_uncertainty.pz = 0;
    icp_uncertainty.om = 0;
    icp_uncertainty.fi = 0;
    icp_uncertainty.ka = 0;

    // The target and the search radius don't change between iterations, so its bucket index is built once.
    std::vector<std::pair<uint64_t, uint32_t>> indexes;

    for (int i = 0; i < target.size(); i++)
    {
        uint64_t index = get_rgd_index_3d(target[i], { search_radious, search_radious, search_radious });
        indexes.emplace_back(index, i);
    }

    // Ties broken by point index: std::sort leaves the order within a bucket to the standard library, and the
    // search below skips each bucket's last point, so without this the result differs between platforms.
    std::sort(
        indexes.begin(),
        indexes.end(),
        [](const std::pair<uint64_t, uint32_t>& a, const std::pair<uint64_t, uint32_t>& b)
        {
            return a.first != b.first ? a.first < b.first : a.second < b.second;
        });

    std::unordered_map<uint64_t, std::pair<uint32_t, uint32_t>> buckets;

    for (uint32_t i = 0; i < indexes.size(); i++)
    {
        uint64_t index_of_bucket = indexes[i].first;
        if (buckets.contains(index_of_bucket))
        {
            buckets[index_of_bucket].second = i;
        }
        else
        {
            buckets[index_of_bucket].first = i;
            buckets[index_of_bucket].second = i;
        }
    }

    // Target points in bucket order, so a bucket's points are read from contiguous memory.
    std::vector<Eigen::Vector3d> target_sorted(indexes.size());
    for (size_t i = 0; i < indexes.size(); i++)
    {
        target_sorted[i] = target[indexes[i].second];
    }
    const double search_radious_squared = search_radious * search_radious;

    // Normal equations are summed per block of source points, then the blocks in order: no lock per point, and
    // the same sum on every run and machine.
    struct NormalEquations
    {
        Eigen::Matrix<double, 6, 6> AtPA = Eigen::Matrix<double, 6, 6>::Zero();
        // Eigen::Matrix<double, 6, 1> AtPB = Eigen::Matrix<double, 6, 1>::Zero();
        double sum_squared_residuals = 0.0;
        int counter_nn = 0;
    };
    constexpr size_t number_of_blocks = 256;
    const size_t block_size = (source.size() + number_of_blocks - 1) / number_of_blocks;

    // std::cout << "iteration: " << iter + 1 << " of: " << number_of_iterations << std::endl;
    Eigen::MatrixXd AtPA(6, 6);
    AtPA.setZero();
    // Eigen::MatrixXd AtPB(6, 1);
    // AtPB.setZero();
    // Eigen::Vector3d b(search_radious, search_radious, search_radious);

    const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose_result);

    std::vector<NormalEquations> block_sums(number_of_blocks);

    const auto hessian_fun = [&](const Eigen::Vector3d& source_i, NormalEquations& sum)
    {
        if (source_i.norm() < 0.1)
        {
            return;
        }

        Eigen::Vector3d source_point_global = m_pose_result * source_i;

        double min_dist_squared = 1000000000.0;
        Eigen::Vector3d target_i_nn;
        bool nn_found = false;
        for (double x = -search_radious; x <= search_radious; x += search_radious)
        {
            for (double y = -search_radious; y <= search_radious; y += search_radious)
            {
                for (double z = -search_radious; z <= search_radious; z += search_radious)
                {
                    Eigen::Vector3d source_point_global_ext = source_point_global + Eigen::Vector3d(x, y, z);
                    uint64_t index_of_bucket =
                        get_rgd_index_3d(source_point_global_ext, { search_radious, search_radious, search_radious });

                    const auto bucket = buckets.find(index_of_bucket);
                    if (bucket != buckets.end())
                    {
                        for (int index = bucket->second.first; index <= bucket->second.second; index++)
                        {
                            const Eigen::Vector3d& target_i = target_sorted[index];
                            const double d_squared = (source_point_global - target_i).squaredNorm();

                            if (d_squared < search_radious_squared)
                            {
                                if (d_squared < min_dist_squared)
                                {
                                    // std::cout << "sqrt_d_squared: " << sqrt(d_squared) << std::endl;
                                    min_dist_squared = d_squared;
                                    nn_found = true;
                                    target_i_nn = target_i;
                                }
                                /**/
                            }
                        }
                    }
                }
            }
        }

        if (nn_found)
        {
            const Eigen::Vector3d& p_s = source_i;

            Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AtPA_;
            point_to_point_source_to_target_tait_bryan_wc_AtPA_simplified(
                AtPA_,
                pose_s.px,
                pose_s.py,
                pose_s.pz,
                pose_s.om,
                pose_s.fi,
                pose_s.ka,
                p_s.x(),
                p_s.y(),
                p_s.z(),
                1,
                0,
                0,
                0,
                1,
                0,
                0,
                0,
                1);

            sum.AtPA += AtPA_;
            sum.sum_squared_residuals += min_dist_squared;
            sum.counter_nn++;
        }
    };

    const auto block_fun = [&](NormalEquations& sum)
    {
        const size_t block = &sum - block_sums.data();
        const size_t begin = std::min(source.size(), block * block_size);
        const size_t end = std::min(source.size(), begin + block_size);
        for (size_t i = begin; i < end; i++)
        {
            hessian_fun(source[i], sum);
        }
    };

    if (multithread)
    {
        // TBB rather than std::execution, which AppleClang's libc++ doesn't run in parallel.
        tbb::parallel_for_each(block_sums.begin(), block_sums.end(), block_fun);
    }
    else
    {
        std::for_each(block_sums.begin(), block_sums.end(), block_fun);
    }

    int counter_nn = 0;
    double sum_squared_residuals = 0.0;
    for (const auto& sum : block_sums)
    {
        AtPA += sum.AtPA;
        // AtPB += sum.AtPB;
        sum_squared_residuals += sum.sum_squared_residuals;
        counter_nn += sum.counter_nn;
    }
    // Each point-to-point correspondence gives 3 observations (x, y, z).
    fill_residual_statistics(icp_residual_statistics, sum_squared_residuals, counter_nn, 3 * counter_nn);

    if (counter_nn < 100)
    {
        std::cout << "not suficient number of observations" << std::endl;
        std::cout << "PairWiseICP::compute FAILED (Relative pose is not changed/solved/optimized --> please consider removing this edge)"
                  << std::endl;
        return { icp_uncertainty, icp_residual_statistics };
    }

    // Each point-to-point correspondence gives 3 observations (x, y, z).
    icp_uncertainty = compute_icp_uncertainty_from_hessian(AtPA, 3 * counter_nn, sum_squared_residuals, icp_residual_statistics);

    return { icp_uncertainty, icp_residual_statistics };
}

std::pair<TaitBryanPose, IcpResidualStatistics> PairWiseICP::compute_icp_uncertainty_point_to_plane(
    const std::vector<Eigen::Vector3d>& source,
    const std::vector<Eigen::Vector3d>& target,
    const Eigen::Affine3d& m_pose_result,
    double search_radious)
{
    IcpResidualStatistics residual_statistics;
    // A target neighbourhood is used as a plane only if it has enough points and is flat: smallest eigenvalue of
    // its covariance at most this fraction of the middle one.
    constexpr int min_neighbours = 5;
    constexpr double planarity_threshold = 0.1;
    constexpr int min_correspondences = 100;

    const TargetGrid grid(target, search_radious);
    const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose_result);

    // Normal equations are summed per block of source points, then the blocks in order: no lock per point, and
    // the same sum on every run and machine.
    struct NormalEquations
    {
        Eigen::Matrix<double, 6, 6> AtPA = Eigen::Matrix<double, 6, 6>::Zero();
        double sum_squared_residuals = 0.0;
        int counter = 0;
    };
    constexpr size_t number_of_blocks = 256;
    const size_t block_size = (source.size() + number_of_blocks - 1) / number_of_blocks;
    std::vector<NormalEquations> block_sums(number_of_blocks);

    const auto add_observation = [&](const Eigen::Vector3d& source_i, NormalEquations& sum)
    {
        if (source_i.norm() < 0.1)
        {
            return;
        }
        const Eigen::Vector3d source_point_global = m_pose_result * source_i;

        // Plane of the target neighbourhood from running sums (no per-point allocation). The sums are taken relative
        // to the query point: with large global coordinates (e.g. UTM) E[pp^T] - mean*mean^T would lose all precision.
        Eigen::Vector3d sum_p = Eigen::Vector3d::Zero();
        Eigen::Matrix3d sum_ppt = Eigen::Matrix3d::Zero();
        int k = 0;
        grid.for_each_neighbour(
            source_point_global,
            [&](const Eigen::Vector3d& target_i, double)
            {
                const Eigen::Vector3d p = target_i - source_point_global;
                sum_p += p;
                sum_ppt += p * p.transpose();
                k++;
            });
        if (k < min_neighbours)
        {
            return;
        }
        const Eigen::Vector3d mean_relative = sum_p / k;
        const Eigen::Matrix3d covariance = sum_ppt / k - mean_relative * mean_relative.transpose();
        const Eigen::Vector3d mean = source_point_global + mean_relative;
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
        if (eigen_solver.info() != Eigen::Success)
        {
            return;
        }
        const Eigen::Vector3d& eigenvalues = eigen_solver.eigenvalues(); // ascending
        if (eigenvalues(0) > planarity_threshold * eigenvalues(1))
        {
            return;
        }
        const Eigen::Vector3d normal = eigen_solver.eigenvectors().col(0);

        const double residual = -normal.dot(mean_relative);

        Eigen::Matrix<double, 1, 6> j;
        point_to_plane_tait_bryan_wc_jacobian(
            j,
            pose_s.px,
            pose_s.py,
            pose_s.pz,
            pose_s.om,
            pose_s.fi,
            pose_s.ka,
            source_i.x(),
            source_i.y(),
            source_i.z(),
            mean.x(),
            mean.y(),
            mean.z(),
            normal.x(),
            normal.y(),
            normal.z());

        sum.AtPA += j.transpose() * j;
        sum.sum_squared_residuals += residual * residual;
        sum.counter++;
    };

    tbb::parallel_for_each(
        block_sums.begin(),
        block_sums.end(),
        [&](NormalEquations& sum)
        {
            const size_t block = &sum - block_sums.data();
            const size_t begin = std::min(source.size(), block * block_size);
            const size_t end = std::min(source.size(), begin + block_size);
            for (size_t i = begin; i < end; i++)
            {
                add_observation(source[i], sum);
            }
        });

    Eigen::Matrix<double, 6, 6> AtPA = Eigen::Matrix<double, 6, 6>::Zero();
    double sum_squared_residuals = 0.0;
    int counter = 0;
    for (const auto& sum : block_sums)
    {
        AtPA += sum.AtPA;
        sum_squared_residuals += sum.sum_squared_residuals;
        counter += sum.counter;
    }
    // Each point-to-plane correspondence gives 1 observation (distance along the normal).
    fill_residual_statistics(residual_statistics, sum_squared_residuals, counter, counter);

    if (counter < min_correspondences)
    {
        std::cout << "PairWiseICP::compute_icp_uncertainty_point_to_plane: not enough planar correspondences (" << counter << ")"
                  << std::endl;
        return { infinite_uncertainty(), residual_statistics };
    }

    // Each point-to-plane correspondence gives 1 observation (distance along the normal).
    const TaitBryanPose sigma = compute_icp_uncertainty_from_hessian(AtPA, counter, sum_squared_residuals, residual_statistics);
    return { sigma, residual_statistics };
}

IcpSpread PairWiseICP::compute_icp_montecarlo(
    const std::vector<Eigen::Vector3d>& source,
    const std::vector<Eigen::Vector3d>& target,
    const Eigen::Affine3d& m_pose,
    double search_radious,
    int number_of_iterations,
    int number_of_runs,
    double start_offset_m,
    double start_offset_rad,
    const std::function<void(int run)>& on_run_done)
{
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Matrix6d = Eigen::Matrix<double, 6, 6>;

    const auto to_vector = [](const TaitBryanPose& p)
    {
        return (Vector6d() << p.px, p.py, p.pz, p.om, p.fi, p.ka).finished();
    };
    const Vector6d reference = to_vector(pose_tait_bryan_from_affine_matrix(m_pose));

    // std::mt19937's output is fixed by the standard (unlike std::uniform_real_distribution's), so every platform
    // draws the same offsets.
    std::mt19937 generator(0);
    const auto uniform = [&generator](double half_width)
    {
        return half_width * (2.0 * static_cast<double>(generator()) / static_cast<double>(std::mt19937::max()) - 1.0);
    };

    std::vector<Vector6d> start_offsets;
    std::vector<Vector6d> result_offsets;
    for (int run = 0; run < number_of_runs; run++)
    {
        Vector6d start_offset;
        for (int i = 0; i < 6; i++)
        {
            start_offset(i) = uniform(i < 3 ? start_offset_m : start_offset_rad);
        }
        start_offsets.push_back(start_offset);

        const Vector6d start = reference + start_offset;
        const TaitBryanPose start_pose{ start(0), start(1), start(2), start(3), start(4), start(5) };
        Eigen::Affine3d m_pose_run = affine_matrix_from_pose_tait_bryan(start_pose);
        if (compute(source, target, search_radious, number_of_iterations, m_pose_run))
        {
            Vector6d offset = to_vector(pose_tait_bryan_from_affine_matrix(m_pose_run)) - reference;
            for (int i = 3; i < 6; i++)
            {
                offset(i) = std::remainder(offset(i), 2.0 * M_PI); // angle difference in [-pi, pi]
            }
            result_offsets.push_back(offset);
        }
        if (on_run_done)
            on_run_done(run + 1);
    }

    const auto mean_of = [](const std::vector<Vector6d>& samples)
    {
        Vector6d sum = Vector6d::Zero();
        for (const auto& s : samples)
        {
            sum += s;
        }
        return Vector6d(sum / static_cast<double>(samples.size()));
    };
    const auto covariance_of = [&mean_of](const std::vector<Vector6d>& samples)
    {
        const Vector6d mean = mean_of(samples);
        Matrix6d sum = Matrix6d::Zero();
        for (const auto& s : samples)
        {
            sum += (s - mean) * (s - mean).transpose();
        }
        return Matrix6d(sum / static_cast<double>(samples.size() - 1));
    };

    IcpSpread spread;
    spread.number_of_runs = number_of_runs;
    spread.number_of_converged_runs = static_cast<int>(result_offsets.size());
    if (start_offsets.size() >= 2)
    {
        spread.start_sigma = covariance_of(start_offsets).diagonal().cwiseSqrt();
    }
    if (!result_offsets.empty())
    {
        spread.mean_offset = mean_of(result_offsets);
    }
    if (result_offsets.size() >= 2)
    {
        spread.covariance = covariance_of(result_offsets);
    }
    if (verbose)
        std::cout << "ICP Monte Carlo: " << spread.number_of_converged_runs << " of " << number_of_runs << " runs converged" << std::endl;
    return spread;
}
