#include <pch/pch.h>

#include <icp.h>

#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_tait_bryan_wc_jacobian.h>

#include <m_estimators.h>
#include <transformations.h>

std::vector<ICP::Job> ICP::get_jobs(long long unsigned int size, int num_threads)
{
    int hc = size / num_threads;
    if (hc < 1)
        hc = 1;

    std::vector<Job> jobs;
    for (long long unsigned int i = 0; i < size; i += hc)
    {
        long long unsigned int sequence_length = hc;
        if (i + hc >= size)
        {
            sequence_length = size - i;
        }
        if (sequence_length == 0)
            break;

        Job j;
        j.index_begin_inclusive = i;
        j.index_end_exclusive = i + sequence_length;
        jobs.push_back(j);
    }
    return jobs;
}

void alpha_point_to_point_job(
    ICP::Job* job,
    std::vector<double>* alphas,
    float barron_c,
    std::vector<std::vector<std::pair<int, int>>>* all_nns,
    std::vector<PointCloud>* point_clouds,
    std::vector<int>* j_indexes,
    TaitBryanPose pose_s,
    float scale_factor_x,
    float scale_factor_y,
    float scale_factor_z,
    std::vector<double>* sums_x,
    std::vector<double>* sums_y,
    std::vector<double>* sums_z,
    int index_source)
{
    for (size_t ii = job->index_begin_inclusive; ii < job->index_end_exclusive; ii++)
    {
        double& alpha = (*alphas)[ii];
        double Z_tilde = get_approximate_partition_function(-10, 10, alpha, barron_c, 100);
        double sum_x = 0;
        double sum_y = 0;
        double sum_z = 0;

        for (size_t ni = 0; ni < (*all_nns).size(); ni++)
        {
            for (size_t nj = 0; nj < (*all_nns)[ni].size(); nj++)
            {
                // Eigen::Vector3d p_s((*point_clouds)[index_source].points_local[(*all_nns)[ni][nj].first]);
                Eigen::Vector3d p_s(
                    (*point_clouds)[index_source].m_pose * (*point_clouds)[index_source].points_local[(*all_nns)[ni][nj].first]);
                Eigen::Vector3d p_t(
                    (*point_clouds)[(*j_indexes)[ni]].m_pose * (*point_clouds)[(*j_indexes)[ni]].points_local[(*all_nns)[ni][nj].second]);

                double delta_x = p_t.x() - p_s.x();
                double delta_y = p_t.y() - p_s.y();
                double delta_z = p_t.z() - p_s.z();
                // point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z, pose_s.px, pose_s.py, pose_s.pz, pose_s.om,
                // pose_s.fi, pose_s.ka, p_s.x(), p_s.y(), p_s.z(), p_t.x(), p_t.y(), p_t.z());

                if (!(delta_x == delta_x))
                {
                    continue;
                }
                if (!(delta_y == delta_y))
                {
                    continue;
                }
                if (!(delta_z == delta_z))
                {
                    continue;
                }

                delta_x *= scale_factor_x;
                delta_y *= scale_factor_y;
                delta_z *= scale_factor_z;

                sum_x += get_truncated_robust_kernel(delta_x, alpha, barron_c, Z_tilde);
                sum_y += get_truncated_robust_kernel(delta_y, alpha, barron_c, Z_tilde);
                sum_z += get_truncated_robust_kernel(delta_z, alpha, barron_c, Z_tilde);
            }
        }
        (*sums_x)[ii] = sum_x;
        (*sums_y)[ii] = sum_y;
        (*sums_z)[ii] = sum_z;
    }
}

bool ICP::optimize_source_to_target_wc(PointClouds& point_clouds_container, bool fix_first_node)
{
    for (auto& pc : point_clouds_container.point_clouds)
    {
        pc.build_rgd();
        pc.cout_rgd();
        pc.compute_normal_vectors(0.5);
    }

    for (size_t iter = 0; iter < number_of_iterations; iter++)
    {
        std::cout << "ICP iteration: " << iter + 1 << " of " << number_of_iterations << std::endl;

        std::vector<Eigen::Triplet<double>> tripletListA;
        std::vector<Eigen::Triplet<double>> tripletListP;
        std::vector<Eigen::Triplet<double>> tripletListB;

        for (int i = 0; i < point_clouds_container.point_clouds.size(); i++)
        {
            // barron
            double min_sum_x = std::numeric_limits<double>::max();
            double min_sum_y = std::numeric_limits<double>::max();
            double min_sum_z = std::numeric_limits<double>::max();

            double barron_alpha_x = -10.;
            double barron_alpha_y = -10.;
            double barron_alpha_z = -10.;

            TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);
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
                        std::vector<std::pair<int, int>> nns =
                            point_clouds_container.point_clouds[i].nns(point_clouds_container.point_clouds[j], search_radius);
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
                    threads.push_back(
                        std::thread(
                            alpha_point_to_point_job,
                            &jobs[k],
                            &alphas,
                            barron_c,
                            &all_nns,
                            &point_clouds_container.point_clouds,
                            &j_indexes,
                            pose_s,
                            scale_factor_x,
                            scale_factor_y,
                            scale_factor_z,
                            &sums_x,
                            &sums_y,
                            &sums_z,
                            i));
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
                std::cout << "barron_alpha_x: " << barron_alpha_x << " barron_alpha_y: " << barron_alpha_y
                          << " barron_alpha_z: " << barron_alpha_z << std::endl;
            }

            for (int j = 0; j < point_clouds_container.point_clouds.size(); j++)
            {
                if (i != j)
                {
                    std::vector<std::pair<int, int>> nns =
                        point_clouds_container.point_clouds[i].nns(point_clouds_container.point_clouds[j], search_radius);
                    TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);

                    for (size_t k = 0; k < nns.size(); k++)
                    {
                        Eigen::Vector3d p_s(point_clouds_container.point_clouds[i].points_local[nns[k].first]);
                        Eigen::Vector3d p_t(
                            point_clouds_container.point_clouds[j].m_pose *
                            point_clouds_container.point_clouds[j].points_local[nns[k].second]);

                        double delta_x;
                        double delta_y;
                        double delta_z;
                        point_to_point_source_to_target_tait_bryan_wc(
                            delta_x,
                            delta_y,
                            delta_z,
                            pose_s.px,
                            pose_s.py,
                            pose_s.pz,
                            pose_s.om,
                            pose_s.fi,
                            pose_s.ka,
                            p_s.x(),
                            p_s.y(),
                            p_s.z(),
                            p_t.x(),
                            p_t.y(),
                            p_t.z());

                        Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
                        point_to_point_source_to_target_tait_bryan_wc_jacobian(
                            jacobian, pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka, p_s.x(), p_s.y(), p_s.z());

                        int ir = tripletListB.size();
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

                        float factor_ballanced_horizontal_vs_vertical = 1;
                        if (is_ballanced_horizontal_vs_vertical)
                        {
                            if (point_clouds_container.point_clouds[i].points_type[nns[k].first] == 0)
                            {
                                // number_points_horisontal
                                factor_ballanced_horizontal_vs_vertical =
                                    1.0 / float(point_clouds_container.point_clouds[i].number_points_horizontal); //
                            }
                            else
                            {
                                factor_ballanced_horizontal_vs_vertical =
                                    1.0 / float(point_clouds_container.point_clouds[i].number_points_vertical); //
                            }
                        }

                        if (is_adaptive_robust_kernel)
                        {
                            tripletListP.emplace_back(
                                ir,
                                ir,
                                get_barron_w(delta_x * scale_factor_x, barron_alpha_x, barron_c) * factor_ballanced_horizontal_vs_vertical);
                            tripletListP.emplace_back(
                                ir + 1,
                                ir + 1,
                                get_barron_w(delta_y * scale_factor_y, barron_alpha_y, barron_c) * factor_ballanced_horizontal_vs_vertical);
                            tripletListP.emplace_back(
                                ir + 2,
                                ir + 2,
                                get_barron_w(delta_z * scale_factor_z, barron_alpha_z, barron_c) * factor_ballanced_horizontal_vs_vertical);
                        }
                        else
                        {
                            tripletListP.emplace_back(ir, ir, 1 * factor_ballanced_horizontal_vs_vertical);
                            tripletListP.emplace_back(ir + 1, ir + 1, 1 * factor_ballanced_horizontal_vs_vertical);
                            tripletListP.emplace_back(ir + 2, ir + 2, 1 * factor_ballanced_horizontal_vs_vertical);
                            // exit(0);
                        }

                        tripletListB.emplace_back(ir, 0, delta_x);
                        tripletListB.emplace_back(ir + 1, 0, delta_y);
                        tripletListB.emplace_back(ir + 2, 0, delta_z);
                    }
                }
            }
        }
        if (fix_first_node)
        {
            int ir = tripletListB.size();
            tripletListA.emplace_back(ir, 0, 1);
            tripletListA.emplace_back(ir + 1, 1, 1);
            tripletListA.emplace_back(ir + 2, 2, 1);
            tripletListA.emplace_back(ir + 3, 3, 1);
            tripletListA.emplace_back(ir + 4, 4, 1);
            tripletListA.emplace_back(ir + 5, 5, 1);

            tripletListP.emplace_back(ir, ir, 1000000);
            tripletListP.emplace_back(ir + 1, ir + 1, 1000000);
            tripletListP.emplace_back(ir + 2, ir + 2, 1000000);
            tripletListP.emplace_back(ir + 3, ir + 3, 1000000);
            tripletListP.emplace_back(ir + 4, ir + 4, 1000000);
            tripletListP.emplace_back(ir + 5, ir + 5, 1000000);

            tripletListB.emplace_back(ir, 0, 0);
            tripletListB.emplace_back(ir + 1, 0, 0);
            tripletListB.emplace_back(ir + 2, 0, 0);
            tripletListB.emplace_back(ir + 3, 0, 0);
            tripletListB.emplace_back(ir + 4, 0, 0);
            tripletListB.emplace_back(ir + 5, 0, 0);
        }

        Eigen::SparseMatrix<double> matA(tripletListB.size(), point_clouds_container.point_clouds.size() * 6);
        Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
        Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

        matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
        matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
        matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

        Eigen::SparseMatrix<double> AtPA(point_clouds_container.point_clouds.size() * 6, point_clouds_container.point_clouds.size() * 6);
        Eigen::SparseMatrix<double> AtPB(point_clouds_container.point_clouds.size() * 6, 1);

        Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
        AtPA = AtP * matA;
        AtPB = AtP * matB;

        tripletListA.clear();
        tripletListP.clear();
        tripletListB.clear();

        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);
        Eigen::SparseMatrix<double> x = solver.solve(AtPB);

        std::vector<double> h_x;

        for (int k = 0; k < x.outerSize(); ++k)
        {
            for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            {
                h_x.push_back(it.value());
            }
        }

        if (h_x.size() == point_clouds_container.point_clouds.size() * 6)
        {
            std::cout << "ICP solution" << std::endl;
            std::cout << "x,y,z,om,fi,ka" << std::endl;
            for (size_t i = 0; i < h_x.size(); i += 6)
            {
                std::cout << h_x[i] << "," << h_x[i + 1] << "," << h_x[i + 2] << "," << h_x[i + 3] << "," << h_x[i + 4] << "," << h_x[i + 5]
                          << std::endl;
            }

            int counter = 0;
            for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
            {
                TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);
                pose.px += h_x[counter++] * 0.5;
                pose.py += h_x[counter++] * 0.5;
                pose.pz += h_x[counter++] * 0.5;
                pose.om += h_x[counter++] * 0.5;
                pose.fi += h_x[counter++] * 0.5;
                pose.ka += h_x[counter++] * 0.5;

                if (i == 0 && fix_first_node)
                {
                    continue;
                }

                /*if (!point_clouds_container.point_clouds[i].fixed) {
                    point_clouds_container.point_clouds[i].m_pose = affine_matrix_from_pose_tait_bryan(pose);
                    point_clouds_container.point_clouds[i].pose =
                pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);
                    point_clouds_container.point_clouds[i].gui_translation[0] = point_clouds_container.point_clouds[i].pose.px;
                    point_clouds_container.point_clouds[i].gui_translation[1] = point_clouds_container.point_clouds[i].pose.py;
                    point_clouds_container.point_clouds[i].gui_translation[2] = point_clouds_container.point_clouds[i].pose.pz;
                    point_clouds_container.point_clouds[i].gui_rotation[0] = rad2deg(point_clouds_container.point_clouds[i].pose.om);
                    point_clouds_container.point_clouds[i].gui_rotation[1] = rad2deg(point_clouds_container.point_clouds[i].pose.fi);
                    point_clouds_container.point_clouds[i].gui_rotation[2] = rad2deg(point_clouds_container.point_clouds[i].pose.ka);
                }
                else {
                    std::cout << "point cloud: " << point_clouds_container.point_clouds[i].file_name << " is fixed" << std::endl;
                }*/

                auto pose_res = pose;
                auto pose_src = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);

                if (!point_clouds_container.point_clouds[i].fixed_x)
                {
                    pose_src.px = pose_res.px;
                }
                if (!point_clouds_container.point_clouds[i].fixed_y)
                {
                    pose_src.py = pose_res.py;
                }
                if (!point_clouds_container.point_clouds[i].fixed_z)
                {
                    pose_src.pz = pose_res.pz;
                }
                if (!point_clouds_container.point_clouds[i].fixed_om)
                {
                    pose_src.om = pose_res.om;
                }
                if (!point_clouds_container.point_clouds[i].fixed_fi)
                {
                    pose_src.fi = pose_res.fi;
                }
                if (!point_clouds_container.point_clouds[i].fixed_ka)
                {
                    pose_src.ka = pose_res.ka;
                }

                point_clouds_container.point_clouds[i].m_pose = affine_matrix_from_pose_tait_bryan(pose_src);
                point_clouds_container.point_clouds[i].pose = pose_src;
                point_clouds_container.point_clouds[i].gui_translation[0] = pose_src.px;
                point_clouds_container.point_clouds[i].gui_translation[1] = pose_src.py;
                point_clouds_container.point_clouds[i].gui_translation[2] = pose_src.pz;
                point_clouds_container.point_clouds[i].gui_rotation[0] = rad2deg(pose_src.om);
                point_clouds_container.point_clouds[i].gui_rotation[1] = rad2deg(pose_src.fi);
                point_clouds_container.point_clouds[i].gui_rotation[2] = rad2deg(pose_src.ka);
            }
        }
        else
        {
            std::cout << "AtPA=AtPB FAILED" << std::endl;
            return false;
        }
    }

    // clean
    for (auto& pc : point_clouds_container.point_clouds)
    {
        pc.clean();
    }

    return true;
}

bool ICP::optimize_source_to_target_lie_algebra_left_jacobian(PointClouds& point_clouds_container, bool fix_first_node)
{
    for (auto& pc : point_clouds_container.point_clouds)
    {
        pc.build_rgd();
        pc.cout_rgd();
        pc.compute_normal_vectors(0.5);
    }

    for (size_t iter = 0; iter < number_of_iterations; iter++)
    {
        std::cout << "ICP iteration: " << iter + 1 << " of " << number_of_iterations << std::endl;

        std::vector<Eigen::Triplet<double>> tripletListA;
        std::vector<Eigen::Triplet<double>> tripletListP;
        std::vector<Eigen::Triplet<double>> tripletListB;

        for (int i = 0; i < point_clouds_container.point_clouds.size(); i++)
        {
            // barron
            double min_sum_x = std::numeric_limits<double>::max();
            double min_sum_y = std::numeric_limits<double>::max();
            double min_sum_z = std::numeric_limits<double>::max();

            double barron_alpha_x = -10.;
            double barron_alpha_y = -10.;
            double barron_alpha_z = -10.;

            TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);
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
                        std::vector<std::pair<int, int>> nns =
                            point_clouds_container.point_clouds[i].nns(point_clouds_container.point_clouds[j], search_radius);
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
                    threads.push_back(
                        std::thread(
                            alpha_point_to_point_job,
                            &jobs[k],
                            &alphas,
                            barron_c,
                            &all_nns,
                            &point_clouds_container.point_clouds,
                            &j_indexes,
                            pose_s,
                            scale_factor_x,
                            scale_factor_y,
                            scale_factor_z,
                            &sums_x,
                            &sums_y,
                            &sums_z,
                            i));
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
                std::cout << "barron_alpha_x: " << barron_alpha_x << " barron_alpha_y: " << barron_alpha_y
                          << " barron_alpha_z: " << barron_alpha_z << std::endl;
            }

            for (int j = 0; j < point_clouds_container.point_clouds.size(); j++)
            {
                if (i != j)
                {
                    std::vector<std::pair<int, int>> nns =
                        point_clouds_container.point_clouds[i].nns(point_clouds_container.point_clouds[j], search_radius);
                    // TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);

                    for (size_t k = 0; k < nns.size(); k++)
                    {
                        Eigen::Vector3d p_s(point_clouds_container.point_clouds[i].points_local[nns[k].first]);
                        Eigen::Vector3d p_t(
                            point_clouds_container.point_clouds[j].m_pose *
                            point_clouds_container.point_clouds[j].points_local[nns[k].second]);

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

                        int ir = tripletListB.size();
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

                        float factor_ballanced_horizontal_vs_vertical = 1;
                        if (is_ballanced_horizontal_vs_vertical)
                        {
                            if (point_clouds_container.point_clouds[i].points_type[nns[k].first] == 0)
                            {
                                // number_points_horisontal
                                factor_ballanced_horizontal_vs_vertical =
                                    1.0 / float(point_clouds_container.point_clouds[i].number_points_horizontal); //
                            }
                            else
                            {
                                factor_ballanced_horizontal_vs_vertical =
                                    1.0 / float(point_clouds_container.point_clouds[i].number_points_vertical); //
                            }
                        }

                        Eigen::Vector3d target = p_t;
                        Eigen::Vector3d source = point_clouds_container.point_clouds[i].m_pose * p_s;

                        double delta_x = target.x() - source.x();
                        double delta_y = target.y() - source.y();
                        double delta_z = target.z() - source.z();

                        if (is_adaptive_robust_kernel)
                        {
                            tripletListP.emplace_back(
                                ir,
                                ir,
                                get_barron_w(delta_x * scale_factor_x, barron_alpha_x, barron_c) * factor_ballanced_horizontal_vs_vertical);
                            tripletListP.emplace_back(
                                ir + 1,
                                ir + 1,
                                get_barron_w(delta_y * scale_factor_y, barron_alpha_y, barron_c) * factor_ballanced_horizontal_vs_vertical);
                            tripletListP.emplace_back(
                                ir + 2,
                                ir + 2,
                                get_barron_w(delta_z * scale_factor_z, barron_alpha_z, barron_c) * factor_ballanced_horizontal_vs_vertical);
                        }
                        else
                        {
                            tripletListP.emplace_back(ir, ir, 1 * factor_ballanced_horizontal_vs_vertical);
                            tripletListP.emplace_back(ir + 1, ir + 1, 1 * factor_ballanced_horizontal_vs_vertical);
                            tripletListP.emplace_back(ir + 2, ir + 2, 1 * factor_ballanced_horizontal_vs_vertical);
                        }
                        tripletListB.emplace_back(ir, 0, delta_x);
                        tripletListB.emplace_back(ir + 1, 0, delta_y);
                        tripletListB.emplace_back(ir + 2, 0, delta_z);
                    }
                }
            }
        }
        if (fix_first_node)
        {
            int ir = tripletListB.size();
            tripletListA.emplace_back(ir, 0, 1);
            tripletListA.emplace_back(ir + 1, 1, 1);
            tripletListA.emplace_back(ir + 2, 2, 1);
            tripletListA.emplace_back(ir + 3, 3, 1);
            tripletListA.emplace_back(ir + 4, 4, 1);
            tripletListA.emplace_back(ir + 5, 5, 1);

            tripletListP.emplace_back(ir, ir, 1000000);
            tripletListP.emplace_back(ir + 1, ir + 1, 1000000);
            tripletListP.emplace_back(ir + 2, ir + 2, 1000000);
            tripletListP.emplace_back(ir + 3, ir + 3, 1000000);
            tripletListP.emplace_back(ir + 4, ir + 4, 1000000);
            tripletListP.emplace_back(ir + 5, ir + 5, 1000000);

            tripletListB.emplace_back(ir, 0, 0);
            tripletListB.emplace_back(ir + 1, 0, 0);
            tripletListB.emplace_back(ir + 2, 0, 0);
            tripletListB.emplace_back(ir + 3, 0, 0);
            tripletListB.emplace_back(ir + 4, 0, 0);
            tripletListB.emplace_back(ir + 5, 0, 0);
        }

        Eigen::SparseMatrix<double> matA(tripletListB.size(), point_clouds_container.point_clouds.size() * 6);
        Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
        Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

        matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
        matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
        matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

        Eigen::SparseMatrix<double> AtPA(point_clouds_container.point_clouds.size() * 6, point_clouds_container.point_clouds.size() * 6);
        Eigen::SparseMatrix<double> AtPB(point_clouds_container.point_clouds.size() * 6, 1);

        Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
        AtPA = AtP * matA;
        AtPB = AtP * matB;

        tripletListA.clear();
        tripletListP.clear();
        tripletListB.clear();

        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);
        Eigen::SparseMatrix<double> x = solver.solve(AtPB);

        std::vector<double> h_x;

        for (int k = 0; k < x.outerSize(); ++k)
        {
            for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            {
                h_x.push_back(it.value());
            }
        }

        if (h_x.size() == point_clouds_container.point_clouds.size() * 6)
        {
            std::cout << "ICP solution" << std::endl;
            std::cout << "x,y,z,s_x,s_y,s_z" << std::endl;
            for (size_t i = 0; i < h_x.size(); i += 6)
            {
                std::cout << h_x[i] << "," << h_x[i + 1] << "," << h_x[i + 2] << "," << h_x[i + 3] << "," << h_x[i + 4] << "," << h_x[i + 5]
                          << std::endl;
            }

            int counter = 0;
            for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
            {
                RodriguesPose pose_update;
                pose_update.px = h_x[counter++] * 0.5;
                pose_update.py = h_x[counter++] * 0.5;
                pose_update.pz = h_x[counter++] * 0.5;
                pose_update.sx = h_x[counter++] * 0.5;
                pose_update.sy = h_x[counter++] * 0.5;
                pose_update.sz = h_x[counter++] * 0.5;

                if (i == 0 && fix_first_node)
                {
                    continue;
                }

                /*if (!point_clouds_container.point_clouds[i].fixed) {
                    point_clouds_container.point_clouds[i].m_pose = affine_matrix_from_pose_rodrigues(pose_update) *
                point_clouds_container.point_clouds[i].m_pose;

                    point_clouds_container.point_clouds[i].pose =
                pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);
                    point_clouds_container.point_clouds[i].gui_translation[0] = point_clouds_container.point_clouds[i].pose.px;
                    point_clouds_container.point_clouds[i].gui_translation[1] = point_clouds_container.point_clouds[i].pose.py;
                    point_clouds_container.point_clouds[i].gui_translation[2] = point_clouds_container.point_clouds[i].pose.pz;
                    point_clouds_container.point_clouds[i].gui_rotation[0] = rad2deg(point_clouds_container.point_clouds[i].pose.om);
                    point_clouds_container.point_clouds[i].gui_rotation[1] = rad2deg(point_clouds_container.point_clouds[i].pose.fi);
                    point_clouds_container.point_clouds[i].gui_rotation[2] = rad2deg(point_clouds_container.point_clouds[i].pose.ka);
                }*/

                auto pose_res = pose_tait_bryan_from_affine_matrix(
                    affine_matrix_from_pose_rodrigues(pose_update) * point_clouds_container.point_clouds[i].m_pose);
                auto pose_src = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);

                if (!point_clouds_container.point_clouds[i].fixed_x)
                {
                    pose_src.px = pose_res.px;
                }
                if (!point_clouds_container.point_clouds[i].fixed_y)
                {
                    pose_src.py = pose_res.py;
                }
                if (!point_clouds_container.point_clouds[i].fixed_z)
                {
                    pose_src.pz = pose_res.pz;
                }
                if (!point_clouds_container.point_clouds[i].fixed_om)
                {
                    pose_src.om = pose_res.om;
                }
                if (!point_clouds_container.point_clouds[i].fixed_fi)
                {
                    pose_src.fi = pose_res.fi;
                }
                if (!point_clouds_container.point_clouds[i].fixed_ka)
                {
                    pose_src.ka = pose_res.ka;
                }

                point_clouds_container.point_clouds[i].pose = pose_src;
                point_clouds_container.point_clouds[i].gui_translation[0] = pose_src.px;
                point_clouds_container.point_clouds[i].gui_translation[1] = pose_src.py;
                point_clouds_container.point_clouds[i].gui_translation[2] = pose_src.pz;
                point_clouds_container.point_clouds[i].gui_rotation[0] = rad2deg(pose_src.om);
                point_clouds_container.point_clouds[i].gui_rotation[1] = rad2deg(pose_src.fi);
                point_clouds_container.point_clouds[i].gui_rotation[2] = rad2deg(pose_src.ka);
            }
        }
        else
        {
            std::cout << "AtPA=AtPB FAILED" << std::endl;
            return false;
        }
    }

    // clean
    for (auto& pc : point_clouds_container.point_clouds)
    {
        pc.clean();
    }
    return true;
}

bool ICP::optimize_source_to_target_lie_algebra_right_jacobian(PointClouds& point_clouds_container, bool fix_first_node)
{
    for (auto& pc : point_clouds_container.point_clouds)
    {
        pc.build_rgd();
        pc.cout_rgd();
        pc.compute_normal_vectors(0.5);
    }

    for (size_t iter = 0; iter < number_of_iterations; iter++)
    {
        std::cout << "ICP iteration: " << iter + 1 << " of " << number_of_iterations << std::endl;

        std::vector<Eigen::Triplet<double>> tripletListA;
        std::vector<Eigen::Triplet<double>> tripletListP;
        std::vector<Eigen::Triplet<double>> tripletListB;

        for (int i = 0; i < point_clouds_container.point_clouds.size(); i++)
        {
            // barron
            double min_sum_x = std::numeric_limits<double>::max();
            double min_sum_y = std::numeric_limits<double>::max();
            double min_sum_z = std::numeric_limits<double>::max();

            double barron_alpha_x = -10.;
            double barron_alpha_y = -10.;
            double barron_alpha_z = -10.;

            TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);
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
                        std::vector<std::pair<int, int>> nns =
                            point_clouds_container.point_clouds[i].nns(point_clouds_container.point_clouds[j], search_radius);
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
                    threads.push_back(
                        std::thread(
                            alpha_point_to_point_job,
                            &jobs[k],
                            &alphas,
                            barron_c,
                            &all_nns,
                            &point_clouds_container.point_clouds,
                            &j_indexes,
                            pose_s,
                            scale_factor_x,
                            scale_factor_y,
                            scale_factor_z,
                            &sums_x,
                            &sums_y,
                            &sums_z,
                            i));
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
                std::cout << "barron_alpha_x: " << barron_alpha_x << " barron_alpha_y: " << barron_alpha_y
                          << " barron_alpha_z: " << barron_alpha_z << std::endl;
            }

            for (int j = 0; j < point_clouds_container.point_clouds.size(); j++)
            {
                if (i != j)
                {
                    std::vector<std::pair<int, int>> nns =
                        point_clouds_container.point_clouds[i].nns(point_clouds_container.point_clouds[j], search_radius);

                    for (size_t k = 0; k < nns.size(); k++)
                    {
                        Eigen::Vector3d p_s(point_clouds_container.point_clouds[i].points_local[nns[k].first]);
                        Eigen::Vector3d p_t(
                            point_clouds_container.point_clouds[j].m_pose *
                            point_clouds_container.point_clouds[j].points_local[nns[k].second]);

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

                        int ir = tripletListB.size();
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

                        float factor_ballanced_horizontal_vs_vertical = 1;
                        if (is_ballanced_horizontal_vs_vertical)
                        {
                            if (point_clouds_container.point_clouds[i].points_type[nns[k].first] == 0)
                            {
                                // number_points_horisontal
                                factor_ballanced_horizontal_vs_vertical =
                                    1.0 / float(point_clouds_container.point_clouds[i].number_points_horizontal); //
                            }
                            else
                            {
                                factor_ballanced_horizontal_vs_vertical =
                                    1.0 / float(point_clouds_container.point_clouds[i].number_points_vertical); //
                            }
                        }

                        Eigen::Vector3d target = p_t;
                        Eigen::Vector3d source = point_clouds_container.point_clouds[i].m_pose * p_s;

                        double delta_x = target.x() - source.x();
                        double delta_y = target.y() - source.y();
                        double delta_z = target.z() - source.z();

                        if (is_adaptive_robust_kernel)
                        {
                            tripletListP.emplace_back(
                                ir,
                                ir,
                                get_barron_w(delta_x * scale_factor_x, barron_alpha_x, barron_c) * factor_ballanced_horizontal_vs_vertical);
                            tripletListP.emplace_back(
                                ir + 1,
                                ir + 1,
                                get_barron_w(delta_y * scale_factor_y, barron_alpha_y, barron_c) * factor_ballanced_horizontal_vs_vertical);
                            tripletListP.emplace_back(
                                ir + 2,
                                ir + 2,
                                get_barron_w(delta_z * scale_factor_z, barron_alpha_z, barron_c) * factor_ballanced_horizontal_vs_vertical);
                        }
                        else
                        {
                            tripletListP.emplace_back(ir, ir, 1 * factor_ballanced_horizontal_vs_vertical);
                            tripletListP.emplace_back(ir + 1, ir + 1, 1 * factor_ballanced_horizontal_vs_vertical);
                            tripletListP.emplace_back(ir + 2, ir + 2, 1 * factor_ballanced_horizontal_vs_vertical);
                        }
                        tripletListB.emplace_back(ir, 0, delta_x);
                        tripletListB.emplace_back(ir + 1, 0, delta_y);
                        tripletListB.emplace_back(ir + 2, 0, delta_z);
                    }
                }
            }
        }
        if (fix_first_node)
        {
            int ir = tripletListB.size();
            tripletListA.emplace_back(ir, 0, 1);
            tripletListA.emplace_back(ir + 1, 1, 1);
            tripletListA.emplace_back(ir + 2, 2, 1);
            tripletListA.emplace_back(ir + 3, 3, 1);
            tripletListA.emplace_back(ir + 4, 4, 1);
            tripletListA.emplace_back(ir + 5, 5, 1);

            tripletListP.emplace_back(ir, ir, 1000000);
            tripletListP.emplace_back(ir + 1, ir + 1, 1000000);
            tripletListP.emplace_back(ir + 2, ir + 2, 1000000);
            tripletListP.emplace_back(ir + 3, ir + 3, 1000000);
            tripletListP.emplace_back(ir + 4, ir + 4, 1000000);
            tripletListP.emplace_back(ir + 5, ir + 5, 1000000);

            tripletListB.emplace_back(ir, 0, 0);
            tripletListB.emplace_back(ir + 1, 0, 0);
            tripletListB.emplace_back(ir + 2, 0, 0);
            tripletListB.emplace_back(ir + 3, 0, 0);
            tripletListB.emplace_back(ir + 4, 0, 0);
            tripletListB.emplace_back(ir + 5, 0, 0);
        }

        Eigen::SparseMatrix<double> matA(tripletListB.size(), point_clouds_container.point_clouds.size() * 6);
        Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
        Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

        matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
        matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
        matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

        Eigen::SparseMatrix<double> AtPA(point_clouds_container.point_clouds.size() * 6, point_clouds_container.point_clouds.size() * 6);
        Eigen::SparseMatrix<double> AtPB(point_clouds_container.point_clouds.size() * 6, 1);

        Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
        AtPA = AtP * matA;
        AtPB = AtP * matB;

        tripletListA.clear();
        tripletListP.clear();
        tripletListB.clear();

        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);
        Eigen::SparseMatrix<double> x = solver.solve(AtPB);

        std::vector<double> h_x;

        for (int k = 0; k < x.outerSize(); ++k)
        {
            for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            {
                h_x.push_back(it.value());
            }
        }

        if (h_x.size() == point_clouds_container.point_clouds.size() * 6)
        {
            std::cout << "ICP solution" << std::endl;
            std::cout << "x,y,z,s_x,s_y,s_z" << std::endl;
            for (size_t i = 0; i < h_x.size(); i += 6)
            {
                std::cout << h_x[i] << "," << h_x[i + 1] << "," << h_x[i + 2] << "," << h_x[i + 3] << "," << h_x[i + 4] << "," << h_x[i + 5]
                          << std::endl;
            }

            int counter = 0;
            for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
            {
                RodriguesPose pose_update;
                pose_update.px = h_x[counter++] * 0.5;
                pose_update.py = h_x[counter++] * 0.5;
                pose_update.pz = h_x[counter++] * 0.5;
                pose_update.sx = h_x[counter++] * 0.5;
                pose_update.sy = h_x[counter++] * 0.5;
                pose_update.sz = h_x[counter++] * 0.5;

                if (i == 0 && fix_first_node)
                {
                    continue;
                }

                /*if (!point_clouds_container.point_clouds[i].fixed) {
                    point_clouds_container.point_clouds[i].m_pose = point_clouds_container.point_clouds[i].m_pose *
                affine_matrix_from_pose_rodrigues(pose_update);

                    point_clouds_container.point_clouds[i].pose =
                pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);
                    point_clouds_container.point_clouds[i].gui_translation[0] = point_clouds_container.point_clouds[i].pose.px;
                    point_clouds_container.point_clouds[i].gui_translation[1] = point_clouds_container.point_clouds[i].pose.py;
                    point_clouds_container.point_clouds[i].gui_translation[2] = point_clouds_container.point_clouds[i].pose.pz;
                    point_clouds_container.point_clouds[i].gui_rotation[0] = rad2deg(point_clouds_container.point_clouds[i].pose.om);
                    point_clouds_container.point_clouds[i].gui_rotation[1] = rad2deg(point_clouds_container.point_clouds[i].pose.fi);
                    point_clouds_container.point_clouds[i].gui_rotation[2] = rad2deg(point_clouds_container.point_clouds[i].pose.ka);
                }*/
                auto pose_res = pose_tait_bryan_from_affine_matrix(
                    point_clouds_container.point_clouds[i].m_pose * affine_matrix_from_pose_rodrigues(pose_update));
                auto pose_src = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);

                if (!point_clouds_container.point_clouds[i].fixed_x)
                {
                    pose_src.px = pose_res.px;
                }
                if (!point_clouds_container.point_clouds[i].fixed_y)
                {
                    pose_src.py = pose_res.py;
                }
                if (!point_clouds_container.point_clouds[i].fixed_z)
                {
                    pose_src.pz = pose_res.pz;
                }
                if (!point_clouds_container.point_clouds[i].fixed_om)
                {
                    pose_src.om = pose_res.om;
                }
                if (!point_clouds_container.point_clouds[i].fixed_fi)
                {
                    pose_src.fi = pose_res.fi;
                }
                if (!point_clouds_container.point_clouds[i].fixed_ka)
                {
                    pose_src.ka = pose_res.ka;
                }

                point_clouds_container.point_clouds[i].pose = pose_src;
                point_clouds_container.point_clouds[i].gui_translation[0] = pose_src.px;
                point_clouds_container.point_clouds[i].gui_translation[1] = pose_src.py;
                point_clouds_container.point_clouds[i].gui_translation[2] = pose_src.pz;
                point_clouds_container.point_clouds[i].gui_rotation[0] = rad2deg(pose_src.om);
                point_clouds_container.point_clouds[i].gui_rotation[1] = rad2deg(pose_src.fi);
                point_clouds_container.point_clouds[i].gui_rotation[2] = rad2deg(pose_src.ka);
            }
        }
        else
        {
            std::cout << "AtPA=AtPB FAILED" << std::endl;
            return false;
        }
    }

    // clean
    for (auto& pc : point_clouds_container.point_clouds)
    {
        pc.clean();
    }
    return true;
}

bool ICP::compute_uncertainty(PointClouds& point_clouds_container)
{
    std::cout << "compute_uncertainty" << std::endl;

    for (auto& pc : point_clouds_container.point_clouds)
    {
        pc.build_rgd();
        pc.cout_rgd();
        pc.compute_normal_vectors(0.5);
    }

    std::vector<Eigen::Triplet<double>> tripletListA;

    double ssr = 0.0;
    int num_obs = 0;
    for (int i = 0; i < point_clouds_container.point_clouds.size(); i++)
    {
        for (int j = 0; j < point_clouds_container.point_clouds.size(); j++)
        {
            if (i != j)
            {
                std::vector<std::pair<int, int>> nns =
                    point_clouds_container.point_clouds[i].nns(point_clouds_container.point_clouds[j], search_radius);
                TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);

                for (size_t k = 0; k < nns.size(); k++)
                {
                    Eigen::Vector3d p_s(point_clouds_container.point_clouds[i].points_local[nns[k].first]);
                    Eigen::Vector3d p_t(
                        point_clouds_container.point_clouds[j].m_pose * point_clouds_container.point_clouds[j].points_local[nns[k].second]);

                    double delta_x;
                    double delta_y;
                    double delta_z;
                    point_to_point_source_to_target_tait_bryan_wc(
                        delta_x,
                        delta_y,
                        delta_z,
                        pose_s.px,
                        pose_s.py,
                        pose_s.pz,
                        pose_s.om,
                        pose_s.fi,
                        pose_s.ka,
                        p_s.x(),
                        p_s.y(),
                        p_s.z(),
                        p_t.x(),
                        p_t.y(),
                        p_t.z());

                    Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
                    point_to_point_source_to_target_tait_bryan_wc_jacobian(
                        jacobian, pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka, p_s.x(), p_s.y(), p_s.z());

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

                    ssr += delta_x * delta_x;
                    ssr += delta_y * delta_y;
                    ssr += delta_z * delta_z;
                    num_obs += 3;
                }
            }
        }
    }
    double sq = ssr / (num_obs - point_clouds_container.point_clouds.size() * 6);

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
        for (int r = 0; r < 6; r++)
        {
            for (int c = 0; c < 6; c++)
            {
                point_clouds_container.point_clouds[i].covariance_matrix_tait_bryan(r, c) = AtAinv.coeff(i * 6 + r, i * 6 + c);
            }
        }
        point_clouds_container.point_clouds[i].information_matrix_tait_bryan =
            point_clouds_container.point_clouds[i].covariance_matrix_tait_bryan.inverse();
    }

    // clean
    for (auto& pc : point_clouds_container.point_clouds)
    {
        pc.clean();
    }
    return true;
}
