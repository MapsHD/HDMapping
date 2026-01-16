#include <pch/pch.h>

#include <hash_utils.h>
#include <pair_wise_iterative_closest_point.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian_simplified.h>
#include <structures.h>
#include <transformations.h>

inline void point_to_point_source_to_target_tait_bryan_wc_AtPA_simplified_4(
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor>& AtPA,
    const double& tx,
    const double& ty,
    const double& tz,
    const double& om,
    const double& fi,
    const double& ka,
    const double& x_s,
    const double& y_s,
    const double& z_s)
{
    double sin_om = sin(om);
    double cos_om = cos(om);
    double sin_fi = sin(fi);
    double cos_fi = cos(fi);
    double sin_ka = sin(ka);
    double cos_ka = cos(ka);
    double x0 = cos_fi * z_s;
    double x1 = sin_ka * y_s;
    double x2 = cos_ka * x_s;
    double x3 = sin_fi * x1 - sin_fi * x2 + x0;
    double x4 = cos_ka * y_s + sin_ka * x_s;
    double x5 = -cos_fi * x4;
    double x6 = cos_ka * sin_om;
    double x7 = cos_om * sin_ka;
    double x8 = sin_fi * x7 + x6;
    double x9 = sin_ka * sin_om;
    double x10 = cos_ka * cos_om;
    double x11 = sin_fi * x10 - x9;
    double x12 = cos_om * x0 - x11 * x_s + x8 * y_s;
    double x13 = -x12;
    double x14 = -cos_fi * x1 + cos_fi * x2 + sin_fi * z_s;
    double x15 = sin_om * x14;
    double x16 = -sin_fi * x9 + x10;
    double x17 = sin_fi * x6 + x7;
    double x18 = x16 * x_s - x17 * y_s;
    double x19 = -sin_om * x0 + x16 * y_s + x17 * x_s;
    double x20 = -cos_om * x14;
    double x21 = x11 * y_s + x8 * x_s;
    double x22 = x14 * (-cos_om * x19 - sin_om * x12);
    double x23 = -x12 * x18 + x19 * x21;
    double x24 = pow(x14, 2);
    double x25 = -cos_fi * x3 * x4 - cos_om * x14 * x21 + sin_om * x14 * x18;
    AtPA.coeffRef(0, 0) = 1;
    AtPA.coeffRef(0, 1) = 0;
    AtPA.coeffRef(0, 2) = 0;
    AtPA.coeffRef(0, 3) = 0;
    AtPA.coeffRef(0, 4) = x3;
    AtPA.coeffRef(0, 5) = x5;
    AtPA.coeffRef(1, 0) = 0;
    AtPA.coeffRef(1, 1) = 1;
    AtPA.coeffRef(1, 2) = 0;
    AtPA.coeffRef(1, 3) = x13;
    AtPA.coeffRef(1, 4) = x15;
    AtPA.coeffRef(1, 5) = x18;
    AtPA.coeffRef(2, 0) = 0;
    AtPA.coeffRef(2, 1) = 0;
    AtPA.coeffRef(2, 2) = 1;
    AtPA.coeffRef(2, 3) = x19;
    AtPA.coeffRef(2, 4) = x20;
    AtPA.coeffRef(2, 5) = x21;
    AtPA.coeffRef(3, 0) = 0;
    AtPA.coeffRef(3, 1) = x13;
    AtPA.coeffRef(3, 2) = x19;
    AtPA.coeffRef(3, 3) = pow(x12, 2) + pow(x19, 2);
    AtPA.coeffRef(3, 4) = x22;
    AtPA.coeffRef(3, 5) = x23;
    AtPA.coeffRef(4, 0) = x3;
    AtPA.coeffRef(4, 1) = x15;
    AtPA.coeffRef(4, 2) = x20;
    AtPA.coeffRef(4, 3) = x22;
    AtPA.coeffRef(4, 4) = pow(cos_om, 2) * x24 + pow(sin_om, 2) * x24 + pow(x3, 2);
    AtPA.coeffRef(4, 5) = x25;
    AtPA.coeffRef(5, 0) = x5;
    AtPA.coeffRef(5, 1) = x18;
    AtPA.coeffRef(5, 2) = x21;
    AtPA.coeffRef(5, 3) = x23;
    AtPA.coeffRef(5, 4) = x25;
    AtPA.coeffRef(5, 5) = pow(cos_fi, 2) * pow(x4, 2) + pow(x18, 2) + pow(x21, 2);
}
inline void point_to_point_source_to_target_tait_bryan_wc_AtPB_simplified_4(
    Eigen::Matrix<double, 6, 1>& AtPB,
    const double& tx,
    const double& ty,
    const double& tz,
    const double& om,
    const double& fi,
    const double& ka,
    const double& x_s,
    const double& y_s,
    const double& z_s,
    const double& x_t,
    const double& y_t,
    const double& z_t)
{
    double sin_om = sin(om);
    double cos_om = cos(om);
    double sin_fi = sin(fi);
    double cos_fi = cos(fi);
    double sin_ka = sin(ka);
    double cos_ka = cos(ka);
    double x0 = cos_ka * x_s;
    double x1 = sin_ka * y_s;
    double x2 = cos_fi * x0 - cos_fi * x1 + sin_fi * z_s;
    double x3 = tx + x2 - x_t;
    double x4 = cos_om * sin_ka;
    double x5 = cos_ka * sin_om;
    double x6 = sin_fi * x5 + x4;
    double x7 = cos_ka * cos_om;
    double x8 = sin_ka * sin_om;
    double x9 = -sin_fi * x8 + x7;
    double x10 = cos_fi * z_s;
    double x11 = -sin_om * x10 + x6 * x_s + x9 * y_s;
    double x12 = ty + x11 - y_t;
    double x13 = sin_fi * x4 + x5;
    double x14 = sin_fi * x7 - x8;
    double x15 = cos_om * x10 + x13 * y_s - x14 * x_s;
    double x16 = tz + x15 - z_t;
    AtPB.coeffRef(0) = x3;
    AtPB.coeffRef(1) = x12;
    AtPB.coeffRef(2) = x16;
    AtPB.coeffRef(3) = x11 * x16 - x12 * x15;
    AtPB.coeffRef(4) = -cos_om * x16 * x2 + sin_om * x12 * x2 + x3 * (-sin_fi * x0 + sin_fi * x1 + x10);
    AtPB.coeffRef(5) = -cos_fi * x3 * (cos_ka * y_s + sin_ka * x_s) + x12 * (-x6 * y_s + x9 * x_s) + x16 * (x13 * x_s + x14 * y_s);
}

bool PairWiseICP::compute(
    const std::vector<Eigen::Vector3d>& source,
    const std::vector<Eigen::Vector3d>& target,
    double search_radious,
    int number_of_iterations,
    Eigen::Affine3d& m_pose_result)
{
    std::cout << "PairWiseICP::compute" << std::endl;
    bool multithread = true;

    for (int iter = 0; iter < number_of_iterations; iter++)
    {
        std::vector<std::pair<unsigned long long int, unsigned int>> indexes;

        for (int i = 0; i < target.size(); i++)
        {
            unsigned long long int index = get_rgd_index(target[i], { search_radious, search_radious, search_radious });
            indexes.emplace_back(index, i);
        }

        std::sort(
            indexes.begin(),
            indexes.end(),
            [](const std::pair<unsigned long long int, unsigned int>& a, const std::pair<unsigned long long int, unsigned int>& b)
            {
                return a.first < b.first;
            });

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

        std::cout << "iteration: " << iter + 1 << " of: " << number_of_iterations << std::endl;
        Eigen::MatrixXd AtPA(6, 6);
        AtPA.setZero();
        Eigen::MatrixXd AtPB(6, 1);
        AtPB.setZero();
        Eigen::Vector3d b(search_radious, search_radious, search_radious);

        std::mutex mutex;
        int counter_nn = 0;

        const auto hessian_fun = [&](const Eigen::Vector3d& source_i)
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
                        unsigned long long int index_of_bucket =
                            get_rgd_index(source_point_global_ext, { search_radious, search_radious, search_radious });

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
                const Eigen::Vector3d& p_s = source_i;
                const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose_result);

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

                std::unique_lock lck(mutex);
                AtPA.block<6, 6>(0, 0) += AtPA_;
                AtPB.block<6, 1>(0, 0) -= AtPB_;
                counter_nn++;
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

bool PairWiseICP::compute_fast(
    const std::vector<Eigen::Vector3d>& source,
    const std::vector<Eigen::Vector3d>& target,
    double search_radious,
    int number_of_iterations,
    Eigen::Affine3d& m_pose_result,
    int dec)
{
    // std::cout << "PairWiseICP::compute" << std::endl;
    bool multithread = true;

    std::vector<std::pair<unsigned long long int, unsigned int>> indexes;

    for (int i = 0; i < target.size(); i++)
    {
        unsigned long long int index = get_rgd_index(target[i], { search_radious, search_radious, search_radious });
        indexes.emplace_back(index, i);
    }

    std::sort(
        indexes.begin(),
        indexes.end(),
        [](const std::pair<unsigned long long int, unsigned int>& a, const std::pair<unsigned long long int, unsigned int>& b)
        {
            return a.first < b.first;
        });

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
        // std::cout << "iteration: " << iter + 1 << " of: " << number_of_iterations << std::endl;
        Eigen::MatrixXd AtPA(6, 6);
        AtPA.setZero();
        Eigen::MatrixXd AtPB(6, 1);
        AtPB.setZero();
        Eigen::Vector3d b(search_radious, search_radious, search_radious);

        std::mutex mutex;

        const auto hessian_fun = [&](const Eigen::Vector3d& source_i)
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
                        unsigned long long int index_of_bucket =
                            get_rgd_index(source_point_global_ext, { search_radious, search_radious, search_radious });

                        if (buckets.contains(index_of_bucket))
                        {
                            for (int index = buckets[index_of_bucket].first; index < buckets[index_of_bucket].second; index += dec)
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
                const Eigen::Vector3d& p_s = source_i;
                const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose_result);

                Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AtPA_;
                point_to_point_source_to_target_tait_bryan_wc_AtPA_simplified_4(
                    AtPA_, pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka, p_s.x(), p_s.y(), p_s.z());

                Eigen::Matrix<double, 6, 1> AtPB_;
                point_to_point_source_to_target_tait_bryan_wc_AtPB_simplified_4(
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
                    target_i_nn.x(),
                    target_i_nn.y(),
                    target_i_nn.z());

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
            // std::cout << "result pose updates" << std::endl;
            for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            {
                h_x.push_back(it.value());
                // std::cout << it.row() << " " << it.col() << " " << it.value() << std::endl;
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
            // std::cout << "PairWiseICP::compute SUCCESS" << std::endl;
            //  return true;
        }
        else
        {
            // std::cout << "PairWiseICP::compute FAILED" << std::endl;
            return false;
        }
    }

    return true;
}
