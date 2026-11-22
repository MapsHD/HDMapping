#include <pch/pch.h>

#include <Core/hash_utils.h>
#include <Core/pair_wise_iterative_closest_point.h>
#include <Core/structures.h>
#include <Core/transformations.h>

#include <tbb/parallel_for_each.h>

#include <cmath>
#include <limits>

#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian_simplified.h>

bool PairWiseICP::compute(
    const std::vector<Eigen::Vector3d>& source,
    const std::vector<Eigen::Vector3d>& target,
    double search_radious,
    int number_of_iterations,
    Eigen::Affine3d& m_pose_result)
{
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
            std::cout << "result pose updates" << std::endl;
            for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            {
                h_x.push_back(it.value());
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
    //! Standard deviations of the pose parameters (px, py, pz in metres; om, fi, ka in radians) from the point-to-point
    //! normal matrix AtPA = J^T J (unit weights). Covariance = sigma0^2 * AtPA^-1, where the variance factor
    //! sigma0^2 = sum of squared residuals / (3 * number_of_correspondences - 6). Returns infinity for every parameter
    //! when the problem is degenerate (e.g. a corridor or a plane leaves a direction unconstrained).
    TaitBryanPose compute_icp_uncertainty_from_hessian(
        const Eigen::Matrix<double, 6, 6>& AtPA, int number_of_correspondences, double sum_squared_residuals)
    {
        constexpr double inf = std::numeric_limits<double>::infinity();
        TaitBryanPose sigma;
        sigma.px = sigma.py = sigma.pz = sigma.om = sigma.fi = sigma.ka = inf;

        // Each correspondence gives 3 observations (x, y, z); the pose has 6 unknowns.
        const int redundancy = 3 * number_of_correspondences - 6;
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
        const Eigen::Matrix<double, 6, 6> covariance =
            sigma0_squared * V * eigenvalues.cwiseInverse().asDiagonal() * V.transpose();

        sigma.px = std::sqrt(covariance(0, 0));
        sigma.py = std::sqrt(covariance(1, 1));
        sigma.pz = std::sqrt(covariance(2, 2));
        sigma.om = std::sqrt(covariance(3, 3));
        sigma.fi = std::sqrt(covariance(4, 4));
        sigma.ka = std::sqrt(covariance(5, 5));
        return sigma;
    }
} // namespace

TaitBryanPose PairWiseICP::compute_icp_uncertainty(
    const std::vector<Eigen::Vector3d>& source,
    const std::vector<Eigen::Vector3d>& target,
    const Eigen::Affine3d& m_pose_result,
    double search_radious)
{
    bool multithread = true;
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
        //Eigen::Matrix<double, 6, 1> AtPB = Eigen::Matrix<double, 6, 1>::Zero();
        double sum_squared_residuals = 0.0;
        int counter_nn = 0;
    };
    constexpr size_t number_of_blocks = 256;
    const size_t block_size = (source.size() + number_of_blocks - 1) / number_of_blocks;

    
    //std::cout << "iteration: " << iter + 1 << " of: " << number_of_iterations << std::endl;
    Eigen::MatrixXd AtPA(6, 6);
    AtPA.setZero();
    //Eigen::MatrixXd AtPB(6, 1);
    //AtPB.setZero();
    //Eigen::Vector3d b(search_radious, search_radious, search_radious);

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
        //AtPB += sum.AtPB;
        sum_squared_residuals += sum.sum_squared_residuals;
        counter_nn += sum.counter_nn;
    }

    if (counter_nn < 100)
    {
        std::cout << "not suficient number of observations" << std::endl;
        std::cout
            << "PairWiseICP::compute FAILED (Relative pose is not changed/solved/optimized --> please consider removing this edge)"
            << std::endl;
        return icp_uncertainty;
    }

    /*
    Added compute_icp_uncertainty_from_hessian to pair_wise_iterative_closest_point.cpp. 

    How it works:

    Covariance: it computes covariance = σ₀² · (AᵀPA)⁻¹ and returns the square root of each diagonal entry as a TaitBryanPose. px/py/pz are in metres and om/fi/ka in radians.
    Scale factor: σ₀² = sum of squared residuals / (3·n − 6). Each match gives 3 observations and the pose has 6 unknowns.
    Matrix inverse: it inverts through an eigen decomposition, which shows when the geometry leaves a direction unconstrained (a plane, a corridor). If the smallest eigenvalue is below 1e-12 of the largest, it prints a warning and returns infinity for every parameter.
    Other changes:

    Extra parameter: the Hessian only gives the shape of the uncertainty, not its size, so the function needs the residuals too. The call is now compute_icp_uncertainty_from_hessian(AtPA, counter_nn, sum_squared_residuals).
    Residual sum: NormalEquations gets a sum_squared_residuals field. Each match adds its min_dist_squared, and the block sums are added up alongside AtPA.
    Includes: added <cmath> and <limits>.
    Things to check:

    Wasted work: hessian_fun still calls point_to_point_source_to_target_tait_bryan_wc_AtPB_simplified for every point and never uses the result. You can delete that call.
    Over-confident result: neighbouring LiDAR points are correlated, so this formal covariance tends to make the uncertainty look smaller than it really is. It's fine for comparing edges with each other, but don't read it as an absolute error.
    */
    icp_uncertainty = compute_icp_uncertainty_from_hessian(AtPA, counter_nn, sum_squared_residuals);

    return icp_uncertainty;
}