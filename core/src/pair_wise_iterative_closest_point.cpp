#include <pair_wise_iterative_closest_point.h>
#include <hash_utils.h>
#include <execution>
#include <structures.h>
#include <transformations.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian_simplified.h>

bool PairWiseICP::compute(const std::vector<Eigen::Vector3d> &source, const std::vector<Eigen::Vector3d> &target, double search_radious, int number_of_iterations, Eigen::Affine3d &m_pose_result)
{
    std::cout << "PairWiseICP::compute" << std::endl;
    bool multithread = true;

    std::vector<std::pair<unsigned long long int, unsigned int>> indexes;

    for (int i = 0; i < target.size(); i++)
    {
        unsigned long long int index = get_rgd_index(target[i], {search_radious, search_radious, search_radious});
        indexes.emplace_back(index, i);
    }

    std::sort(indexes.begin(), indexes.end(),
              [](const std::pair<unsigned long long int, unsigned int> &a, const std::pair<unsigned long long int, unsigned int> &b)
              { return a.first < b.first; });

    std::unordered_map<unsigned long long int, std::pair<unsigned int, unsigned int>> buckets;

    for (unsigned int i = 0; i < indexes.size(); i++)
    {
        unsigned long long int index_of_bucket = indexes[i].first;
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

    for (int iter = 0; iter < number_of_iterations; iter++)
    {
        std::cout << "iteration: " << iter + 1 << " of: " << number_of_iterations << std::endl;
        Eigen::MatrixXd AtPA(6, 6);
        AtPA.setZero();
        Eigen::MatrixXd AtPB(6, 1);
        AtPB.setZero();
        Eigen::Vector3d b(search_radious, search_radious, search_radious);

        std::mutex mutex;

        const auto hessian_fun = [&](const Eigen::Vector3d &source_i)
        {
            if (source_i.norm() < 0.1)
            {
                return;
            }

            Eigen::Vector3d source_point_global = m_pose_result * source_i;

            double min_dist = 1000000000.0;
            Eigen::Vector3d target_i;
            Eigen::Vector3d target_i_nn;
            bool nn_found = false;
            for (double x = -search_radious; x <= search_radious; x += search_radious)
            {
                for (double y = -search_radious; y <= search_radious; y += search_radious)
                {
                    for (double z = -search_radious; z <= search_radious; z += search_radious)
                    {
                        Eigen::Vector3d source_point_global_ext = source_point_global + Eigen::Vector3d(x, y, z);
                        unsigned long long int index_of_bucket = get_rgd_index(source_point_global_ext, {search_radious, search_radious, search_radious});

                        if (buckets.contains(index_of_bucket))
                        {
                            for (int index = buckets[index_of_bucket].first; index < buckets[index_of_bucket].second; index++)
                            {
                                int index_element_target = indexes[index].second;
                                target_i = target[index_element_target];
                                double d = (source_point_global - target_i).norm();

                                if (d < search_radious)
                                {
                                    if (d < min_dist)
                                    {
                                        min_dist = d;
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
                const Eigen::Vector3d &p_s = source_i;
                const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose_result);

                Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AtPA_;
                point_to_point_source_to_target_tait_bryan_wc_AtPA_simplified(
                    AtPA_,
                    pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                    p_s.x(), p_s.y(), p_s.z(),
                    1, 0, 0, 0, 1, 0, 0, 0, 1);

                Eigen::Matrix<double, 6, 1> AtPB_;
                point_to_point_source_to_target_tait_bryan_wc_AtPB_simplified(
                    AtPB_,
                    pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                    p_s.x(), p_s.y(), p_s.z(),
                    1, 0, 0, 0, 1, 0, 0, 0, 1,
                    target_i_nn.x(), target_i_nn.y(), target_i_nn.z());

                std::unique_lock lck(mutex);
                AtPA.block<6, 6>(0, 0) += AtPA_;
                AtPB.block<6, 1>(0, 0) -= AtPB_;
            }
        };

        if (multithread)
        {
            std::for_each(std::execution::par_unseq, std::begin(source), std::end(source), hessian_fun);
        }
        else
        {
            std::for_each(std::begin(source), std::end(source), hessian_fun);
        }

        Eigen::SparseMatrix<double> AtPAc(6, 6);
        Eigen::SparseMatrix<double> AtPBc(6, 1);

        AtPAc = AtPA.sparseView();
        AtPBc = AtPB.sparseView();

        // Eigen::SparseMatrix<double> AtPA_I(6, 6);
        // AtPA_I.setIdentity();
        // AtPA += AtPA_I;

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
            std::cout << "PairWiseICP::compute FAILED" << std::endl;
            return false;
        }
    }

    return true;
}
