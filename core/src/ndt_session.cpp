#include <pch/pch.h>

// Split out of ndt.cpp: Session has a WITH_GUI-dependent layout, so this must be built per GUI flavour, not in core_math.
#include <Core/ndt.h>
#include <Core/session.h>
#include <Core/transformations.h>

#include <python-scripts/constraints/quaternion_constraint_jacobian.h>

bool NDT::optimize(std::vector<Session>& sessions, bool compute_only_mahalanobis_distance, bool compute_mean_and_cov_for_bucket)
{
    std::cout << "optimize sessions" << std::endl;

    auto start = std::chrono::system_clock::now();

    Session tmp_session;

    if (sessions.size() > 1)
    {
        if (sessions[0].is_ground_truth)
        {
            tmp_session = sessions[0];
            PointClouds point_clouds_container;

            std::vector<PointCloud> point_clouds;

            std::vector<Eigen::Vector3d> points_local;

            for (const auto& pc : sessions[0].point_clouds_container.point_clouds)
            {
                for (const auto& p : pc.points_local)
                {
                    Eigen::Vector3d pg = pc.m_pose * p;
                    points_local.push_back(pg);
                }
            }

            PointCloud pc;
            pc.points_local = points_local;
            pc.m_initial_pose = Eigen::Affine3d::Identity();
            pc.m_pose = Eigen::Affine3d::Identity();

            point_clouds.push_back(pc);
            point_clouds_container.point_clouds = point_clouds;

            sessions[0].point_clouds_container = point_clouds_container;
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
    else if (is_rodrigues)
    {
        rotation_matrix_parametrization = RotationMatrixParametrization::rodrigues;
    }
    else if (is_quaternion)
    {
        rotation_matrix_parametrization = RotationMatrixParametrization::quaternion;
    }
    else if (is_lie_algebra_left_jacobian)
    {
        rotation_matrix_parametrization = RotationMatrixParametrization::lie_algebra_left_jacobian;
    }
    else if (is_lie_algebra_right_jacobian)
    {
        rotation_matrix_parametrization = RotationMatrixParametrization::lie_algebra_right_jacobian;
    }

    if (is_rodrigues || is_quaternion || is_lie_algebra_left_jacobian || is_lie_algebra_right_jacobian)
    {
        for (auto& s : sessions)
        {
            for (auto& pc : s.point_clouds_container.point_clouds)
            {
                TaitBryanPose pose;
                pose.px = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
                pose.py = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
                pose.pz = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
                pose.om = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
                pose.fi = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
                pose.ka = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
                Eigen::Affine3d m = affine_matrix_from_pose_tait_bryan(pose);
                pc.m_pose = pc.m_pose * m;
            }
        }
    }

    int number_of_unknowns;
    if (is_tait_bryan_angles || is_rodrigues || is_lie_algebra_left_jacobian || is_lie_algebra_right_jacobian)
    {
        number_of_unknowns = 6;
    }
    if (is_quaternion)
    {
        number_of_unknowns = 7;
    }

    double lm_lambda = 0.0001;
    double previous_rms = std::numeric_limits<double>::max();
    int number_of_lm_iterations = 0;

    std::vector<Eigen::Affine3d> m_poses_tmp;
    if (is_levenberg_marguardt)
    {
        m_poses_tmp.clear();
        // for (size_t i = 0; i < point_clouds.size(); i++)
        //{
        //	m_poses_tmp.push_back(point_clouds[i].m_pose);
        // }
        for (auto& s : sessions)
        {
            for (auto& pc : s.point_clouds_container.point_clouds)
            {
                m_poses_tmp.push_back(pc.m_pose);
            }
        }
    }

    for (int iter = 0; iter < number_of_iterations; iter++)
    {
        std::cout << "building points_global_external begin" << std::endl;
        std::vector<Point3D> points_global_external;
        size_t num_total_points = 0;

        // for (int i = 0; i < point_clouds.size(); i++)
        //{
        //	num_total_points += point_clouds[i].points_local.size();
        // }

        int num_point_clouds = 0;
        for (auto& s : sessions)
        {
            for (auto& pc : s.point_clouds_container.point_clouds)
            {
                num_total_points += pc.points_local.size();
                num_point_clouds++;
            }
        }

        points_global_external.reserve(num_total_points);
        Eigen::Vector3d vt;
        Point3D p;

        int index_pose = 0;

        for (auto& s : sessions)
        {
            for (auto& pc : s.point_clouds_container.point_clouds)
            {
                // num_total_points += pc.points_local.size();
                std::cout << "processing point_cloud [" << index_pose + 1 << "] of " << num_point_clouds << std::endl;

                for (int j = 0; j < pc.points_local.size(); j++)
                {
                    vt = pc.m_pose * pc.points_local[j];
                    p.x = vt.x();
                    p.y = vt.y();
                    p.z = vt.z();
                    p.index_pose = index_pose;
                    points_global_external.emplace_back(p);
                }
                index_pose++;
            }
        }

        std::cout << "building points_global_external end" << std::endl;

        std::vector<PointBucketIndexPair> index_pair_external;
        std::vector<Bucket> buckets_external;

        GridParameters rgd_params_external;
        rgd_params_external.resolution_X = this->bucket_size_external[0];
        rgd_params_external.resolution_Y = this->bucket_size_external[1];
        rgd_params_external.resolution_Z = this->bucket_size_external[2];

        int bbext = this->bucket_size_external[0];
        if (this->bucket_size_external[1] > bbext)
            bbext = this->bucket_size_external[1];
        if (this->bucket_size_external[2] > bbext)
            bbext = this->bucket_size_external[2];
        rgd_params_external.bounding_box_extension = bbext;

        std::cout << "building external grid begin" << std::endl;
        grid_calculate_params(points_global_external, rgd_params_external);
        int num_threads = 1;
        if (buckets_external.size() > this->number_of_threads)
        {
            num_threads = this->number_of_threads;
        }
        build_rgd(points_global_external, index_pair_external, buckets_external, rgd_params_external, num_threads);
        std::vector<Bucket> buckets_external_reduced;

        for (const auto& b : buckets_external)
        {
            if (b.number_of_points > 1000)
            {
                buckets_external_reduced.push_back(b);
            }
        }
        buckets_external = buckets_external_reduced;
        buckets_external_reduced.clear();

        std::sort(
            buckets_external.begin(),
            buckets_external.end(),
            [](const Bucket& a, const Bucket& b)
            {
                return (a.number_of_points > b.number_of_points);
            });

        std::cout << "building external grid end" << std::endl;
        std::cout << "number active buckets external: " << buckets_external.size() << std::endl;

        bool init = false;
        Eigen::SparseMatrix<double> AtPA_ndt(num_point_clouds * number_of_unknowns, num_point_clouds * number_of_unknowns);
        Eigen::SparseMatrix<double> AtPB_ndt(num_point_clouds * number_of_unknowns, 1);
        double rms = 0.0;
        int sum = 0;
        double md = 0.0;
        double md_sum = 0.0;

        for (int bi = 0; bi < buckets_external.size(); bi++)
        {
            if (compute_only_mahalanobis_distance)
            {
                std::cout << "bucket [" << bi + 1 << "] of " << buckets_external.size() << std::endl;
            }
            else
            {
                std::cout << "bucket [" << bi + 1 << "] of " << buckets_external.size() << " | iteration [" << iter + 1 << "] of "
                          << number_of_iterations << " | number of points: " << buckets_external[bi].number_of_points << std::endl;
            }
            std::vector<Point3D> points_global;

            for (size_t index = buckets_external[bi].index_begin; index < buckets_external[bi].index_end; index++)
            {
                points_global.push_back(points_global_external[index_pair_external[index].index_of_point]);
            }

            GridParameters rgd_params;
            rgd_params.resolution_X = this->bucket_size[0];
            rgd_params.resolution_Y = this->bucket_size[1];
            rgd_params.resolution_Z = this->bucket_size[2];
            rgd_params.bounding_box_extension = 1.0;

            std::vector<PointBucketIndexPair> index_pair;
            std::vector<Bucket> buckets;

            std::cout << "building rgd begin" << std::endl;
            grid_calculate_params(points_global, rgd_params);
            build_rgd(points_global, index_pair, buckets, rgd_params, this->number_of_threads);
            std::cout << "building rgd end" << std::endl;

            std::vector<Job> jobs = get_jobs(buckets.size(), this->number_of_threads);

            std::vector<std::thread> threads;

            std::vector<Eigen::SparseMatrix<double>> AtPAtmp(jobs.size());
            std::vector<Eigen::SparseMatrix<double>> AtPBtmp(jobs.size());
            std::vector<double> sumrmss(jobs.size());
            std::vector<int> sums(jobs.size());

            std::vector<double> md_out(jobs.size());
            std::vector<double> md_count_out(jobs.size());
            // double *md_out, double *md_count_out

            for (size_t i = 0; i < jobs.size(); i++)
            {
                AtPAtmp[i] = Eigen::SparseMatrix<double>(num_point_clouds * number_of_unknowns, num_point_clouds * number_of_unknowns);
                AtPBtmp[i] = Eigen::SparseMatrix<double>(num_point_clouds * number_of_unknowns, 1);
                sumrmss[i] = 0;
                sums[i] = 0;
                md_out[i] = 0.0;
                md_count_out[i] = 0.0;
            }

            std::vector<Eigen::Affine3d> mposes;
            std::vector<Eigen::Affine3d> mposes_inv;

            for (auto& s : sessions)
            {
                for (auto& pc : s.point_clouds_container.point_clouds)
                {
                    mposes.push_back(pc.m_pose);
                    mposes_inv.push_back(pc.m_pose.inverse());
                }
            }

            std::cout << "computing AtPA AtPB start" << std::endl;
            for (size_t k = 0; k < jobs.size(); k++)
            {
                threads.push_back(std::thread(
                    ndt_job,
                    k,
                    &jobs[k],
                    &buckets,
                    &(AtPAtmp[k]),
                    &(AtPBtmp[k]),
                    &index_pair,
                    &points_global,
                    &mposes,
                    &mposes_inv,
                    num_point_clouds,
                    pose_convention,
                    rotation_matrix_parametrization,
                    number_of_unknowns,
                    &(sumrmss[k]),
                    &(sums[k]),
                    is_generalized,
                    sigma_r,
                    sigma_polar_angle,
                    sigma_azimuthal_angle,
                    num_extended_points,
                    &(md_out[k]),
                    &(md_count_out[k]),
                    false,
                    compute_mean_and_cov_for_bucket));
            }

            for (size_t j = 0; j < threads.size(); j++)
            {
                threads[j].join();
            }
            std::cout << "computing AtPA AtPB finished" << std::endl;

            for (size_t k = 0; k < jobs.size(); k++)
            {
                rms += sumrmss[k];
                sum += sums[k];
                md += md_out[k];
                md_sum += md_count_out[k];
            }

            for (size_t k = 0; k < jobs.size(); k++)
            {
                if (!init)
                {
                    if (AtPBtmp[k].size() > 0)
                    {
                        AtPA_ndt = AtPAtmp[k];
                        AtPB_ndt = AtPBtmp[k];
                        init = true;
                    }
                }
                else
                {
                    if (AtPBtmp[k].size() > 0)
                    {
                        AtPA_ndt += AtPAtmp[k];
                        AtPB_ndt += AtPBtmp[k];
                    }
                }
            }
        }
        std::cout << "cleaning start" << std::endl;
        points_global_external.clear();
        index_pair_external.clear();
        buckets_external.clear();
        std::cout << "cleaning finished" << std::endl;

        rms /= sum;
        std::cout << "rms " << rms << std::endl;

        md /= md_sum;
        std::cout << "mean mahalanobis distance: " << md << std::endl;

        if (compute_only_mahalanobis_distance)
        {
            return true;
        }

        //////////////////////////////////////////////////////////////////

        if (is_fix_first_node)
        {
            Eigen::SparseMatrix<double> I(num_point_clouds * number_of_unknowns, num_point_clouds * number_of_unknowns);
            for (int ii = 0; ii < number_of_unknowns; ii++)
            {
                I.coeffRef(ii, ii) = 1000000;
            }
            AtPA_ndt += I;
        }

        std::cout << "previous_rms: " << previous_rms << " rms: " << rms << std::endl;
        if (is_levenberg_marguardt)
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
                int index = 0;
                for (auto& s : sessions)
                {
                    for (auto& pc : s.point_clouds_container.point_clouds)
                    {
                        pc.m_pose = m_poses_tmp[index++];
                    }
                }

                previous_rms = std::numeric_limits<double>::max();
                continue;
            }
        }
        else
        {
            previous_rms = rms;
        }

        if (is_quaternion)
        {
            std::vector<Eigen::Triplet<double>> tripletListA;
            std::vector<Eigen::Triplet<double>> tripletListP;
            std::vector<Eigen::Triplet<double>> tripletListB;

            int index = 0;
            for (auto& s : sessions)
            {
                for (auto& pc : s.point_clouds_container.point_clouds)
                {
                    // pc.m_pose = m_poses_tmp[index++];
                    int ic = index * 7;
                    int ir = 0;
                    QuaternionPose pose;
                    if (is_wc)
                    {
                        pose = pose_quaternion_from_affine_matrix(pc.m_pose);
                    }
                    else
                    {
                        pose = pose_quaternion_from_affine_matrix(pc.m_pose.inverse());
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

                    index++;
                }
            }

            Eigen::SparseMatrix<double> matA(tripletListB.size(), num_point_clouds * 7);
            Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
            Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

            matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
            matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
            matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

            Eigen::SparseMatrix<double> AtPA(num_point_clouds * 7, num_point_clouds * 7);
            Eigen::SparseMatrix<double> AtPB(num_point_clouds * 7, 1);

            Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
            AtPA = AtP * matA;
            AtPB = AtP * matB;

            AtPA_ndt += AtPA;
            AtPB_ndt += AtPB;
        }

        if (is_levenberg_marguardt)
        {
            Eigen::SparseMatrix<double> LM(num_point_clouds * number_of_unknowns, num_point_clouds * number_of_unknowns);
            LM.setIdentity();
            LM *= lm_lambda;
            AtPA_ndt += LM;
        }

        std::cout << "start solving AtPA=AtPB" << std::endl;
        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA_ndt);

        std::cout << "x = solver.solve(AtPB)" << std::endl;
        Eigen::SparseMatrix<double> x = solver.solve(AtPB_ndt);

        std::vector<double> h_x;
        std::cout << "redult: row,col,value" << std::endl;
        for (int k = 0; k < x.outerSize(); ++k)
        {
            for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            {
                if (it.value() == it.value())
                {
                    h_x.push_back(it.value());
                    std::cout << it.row() << "," << it.col() << "," << it.value() << std::endl;
                }
            }
        }

        if (h_x.size() == num_point_clouds * number_of_unknowns)
        {
            std::cout << "AtPA=AtPB SOLVED" << std::endl;
            int counter = 0;

            for (auto& s : sessions)
            {
                for (auto& pc : s.point_clouds_container.point_clouds)
                {
                    Eigen::Affine3d m_pose;

                    if (is_wc)
                    {
                        m_pose = pc.m_pose;
                    }
                    else
                    {
                        m_pose = pc.m_pose.inverse();
                    }

                    if (is_tait_bryan_angles)
                    {
                        TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(m_pose);
                        pose.px += h_x[counter++];
                        pose.py += h_x[counter++];
                        pose.pz += h_x[counter++];
                        pose.om += h_x[counter++];
                        pose.fi += h_x[counter++];
                        pose.ka += h_x[counter++];
                        m_pose = affine_matrix_from_pose_tait_bryan(pose);
                    }
                    else if (is_rodrigues)
                    {
                        RodriguesPose pose = pose_rodrigues_from_affine_matrix(m_pose);
                        pose.px += h_x[counter++];
                        pose.py += h_x[counter++];
                        pose.pz += h_x[counter++];
                        pose.sx += h_x[counter++];
                        pose.sy += h_x[counter++];
                        pose.sz += h_x[counter++];
                        m_pose = affine_matrix_from_pose_rodrigues(pose);
                    }
                    else if (is_quaternion)
                    {
                        QuaternionPose pose = pose_quaternion_from_affine_matrix(m_pose);

                        QuaternionPose poseq;
                        poseq.px = h_x[counter++];
                        poseq.py = h_x[counter++];
                        poseq.pz = h_x[counter++];
                        poseq.q0 = h_x[counter++];
                        poseq.q1 = h_x[counter++];
                        poseq.q2 = h_x[counter++];
                        poseq.q3 = h_x[counter++];

                        if (fabs(poseq.px) < this->bucket_size[0] && fabs(poseq.py) < this->bucket_size[0] &&
                            fabs(poseq.pz) < this->bucket_size[0] && fabs(poseq.q0) < 10 && fabs(poseq.q1) < 10 && fabs(poseq.q2) < 10 &&
                            fabs(poseq.q3) < 10)
                        {
                            pose.px += poseq.px;
                            pose.py += poseq.py;
                            pose.pz += poseq.pz;
                            pose.q0 += poseq.q0;
                            pose.q1 += poseq.q1;
                            pose.q2 += poseq.q2;
                            pose.q3 += poseq.q3;
                            m_pose = affine_matrix_from_pose_quaternion(pose);
                        }
                    }
                    else if (is_lie_algebra_left_jacobian)
                    {
                        RodriguesPose pose_update;
                        pose_update.px = h_x[counter++];
                        pose_update.py = h_x[counter++];
                        pose_update.pz = h_x[counter++];
                        pose_update.sx = h_x[counter++];
                        pose_update.sy = h_x[counter++];
                        pose_update.sz = h_x[counter++];
                        m_pose = affine_matrix_from_pose_rodrigues(pose_update) * m_pose;
                    }
                    else if (is_lie_algebra_right_jacobian)
                    {
                        RodriguesPose pose_update;
                        pose_update.px = h_x[counter++];
                        pose_update.py = h_x[counter++];
                        pose_update.pz = h_x[counter++];
                        pose_update.sx = h_x[counter++];
                        pose_update.sy = h_x[counter++];
                        pose_update.sz = h_x[counter++];
                        m_pose = m_pose * affine_matrix_from_pose_rodrigues(pose_update);
                    }

                    if (is_wc)
                    {
                    }
                    else
                    {
                        m_pose = m_pose.inverse();
                    }

                    auto pose_res = pose_tait_bryan_from_affine_matrix(m_pose);
                    auto pose_src = pose_tait_bryan_from_affine_matrix(pc.m_pose);

                    if (!pc.fixed_x)
                    {
                        pose_src.px = pose_res.px;
                    }
                    if (!pc.fixed_y)
                    {
                        pose_src.py = pose_res.py;
                    }
                    if (!pc.fixed_z)
                    {
                        pose_src.pz = pose_res.pz;
                    }
                    if (!pc.fixed_om)
                    {
                        pose_src.om = pose_res.om;
                    }
                    if (!pc.fixed_fi)
                    {
                        pose_src.fi = pose_res.fi;
                    }
                    if (!pc.fixed_ka)
                    {
                        pose_src.ka = pose_res.ka;
                    }

                    pc.pose = pose_src;
                    pc.gui_translation[0] = pose_src.px;
                    pc.gui_translation[1] = pose_src.py;
                    pc.gui_translation[2] = pose_src.pz;
                    pc.gui_rotation[0] = rad2deg(pose_src.om);
                    pc.gui_rotation[1] = rad2deg(pose_src.fi);
                    pc.gui_rotation[2] = rad2deg(pose_src.ka);

                    /*
                    if (is_wc)
                    {
                            // if (!s.is_ground_truth)
                            //{
                            pc.m_pose = m_pose; // ToDo check if !pc.fixed needed
                                                                    //}
                    }
                    else
                    {
                            // if (!s.is_ground_truth)
                            //{
                            pc.m_pose = m_pose.inverse(); // ToDo check if !pc.fixed needed
                                                                                      //}
                    }

                    if (!pc.fixed)
                    {
                            pc.pose = pose_tait_bryan_from_affine_matrix(pc.m_pose);
                            pc.gui_translation[0] = pc.pose.px;
                            pc.gui_translation[1] = pc.pose.py;
                            pc.gui_translation[2] = pc.pose.pz;
                            pc.gui_rotation[0] = rad2deg(pc.pose.om);
                            pc.gui_rotation[1] = rad2deg(pc.pose.fi);
                            pc.gui_rotation[2] = rad2deg(pc.pose.ka);
                    }*/
                }
            }
            if (is_levenberg_marguardt)
            {
                m_poses_tmp.clear();
                for (auto& s : sessions)
                {
                    for (auto& pc : s.point_clouds_container.point_clouds)
                    {
                        m_poses_tmp.push_back(pc.m_pose);
                    }
                }
            }

            std::cout << "iteration: " << iter + 1 << " of " << number_of_iterations << std::endl;
        }
        else
        {
            std::cout << "AtPA=AtPB FAILED" << std::endl;
            break;
        }
    }

    //////////

    if (sessions.size() > 1)
    {
        if (sessions[0].is_ground_truth)
        {
            Eigen::Affine3d pose_inv0 = sessions[0].point_clouds_container.point_clouds[0].m_pose.inverse();

            sessions[0].point_clouds_container = tmp_session.point_clouds_container;

            for (int i = 1; i < sessions.size(); i++)
            {
                for (int j = 0; j < sessions[i].point_clouds_container.point_clouds.size(); j++)
                {
                    sessions[i].point_clouds_container.point_clouds[j].m_pose =
                        sessions[i].point_clouds_container.point_clouds[j].m_pose * pose_inv0;
                    sessions[i].point_clouds_container.point_clouds[j].pose =
                        pose_tait_bryan_from_affine_matrix(sessions[i].point_clouds_container.point_clouds[j].m_pose);
                    sessions[i].point_clouds_container.point_clouds[j].gui_translation[0] =
                        sessions[i].point_clouds_container.point_clouds[j].pose.px;
                    sessions[i].point_clouds_container.point_clouds[j].gui_translation[1] =
                        sessions[i].point_clouds_container.point_clouds[j].pose.py;
                    sessions[i].point_clouds_container.point_clouds[j].gui_translation[2] =
                        sessions[i].point_clouds_container.point_clouds[j].pose.pz;
                    sessions[i].point_clouds_container.point_clouds[j].gui_rotation[0] =
                        rad2deg(sessions[i].point_clouds_container.point_clouds[j].pose.om);
                    sessions[i].point_clouds_container.point_clouds[j].gui_rotation[1] =
                        rad2deg(sessions[i].point_clouds_container.point_clouds[j].pose.fi);
                    sessions[i].point_clouds_container.point_clouds[j].gui_rotation[2] =
                        rad2deg(sessions[i].point_clouds_container.point_clouds[j].pose.ka);
                }
            }
        }
    }
    //////////

    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout << "ndt execution time [ms]: " << elapsed.count() << std::endl;

    return true;
}
