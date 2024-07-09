#include "lidar_odometry_utils.h"

#include <mutex>

void optimize(std::vector<Point3Di> &intermediate_points, std::vector<Eigen::Affine3d> &intermediate_trajectory,
              std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model,
              NDT::GridParameters &rgd_params, NDTBucketMapType &buckets, bool multithread,
              bool add_pitch_roll_constraint, const std::vector<std::pair<double, double>> &imu_roll_pitch)
{
    std::vector<Eigen::Triplet<double>> tripletListA;
    std::vector<Eigen::Triplet<double>> tripletListP;
    std::vector<Eigen::Triplet<double>> tripletListB;

    Eigen::MatrixXd AtPAndt(intermediate_trajectory.size() * 6, intermediate_trajectory.size() * 6);
    AtPAndt.setZero();
    Eigen::MatrixXd AtPBndt(intermediate_trajectory.size() * 6, 1);
    AtPBndt.setZero();
    Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

    std::vector<std::mutex> mutexes(intermediate_trajectory.size());

    const auto hessian_fun = [&](const Point3Di &intermediate_points_i)
    {
        if (intermediate_points_i.point.norm() < 1.0)
        {
            return;
        }

        Eigen::Vector3d point_global = intermediate_trajectory[intermediate_points_i.index_pose] * intermediate_points_i.point;
        auto index_of_bucket = get_rgd_index(point_global, b);

        auto bucket_it = buckets.find(index_of_bucket);
        // no bucket found
        if (bucket_it == buckets.end())
        {
            return;
        }
        auto &this_bucket = bucket_it->second;

        // if(buckets[index_of_bucket].number_of_points >= 5){
        const Eigen::Matrix3d &infm = this_bucket.cov.inverse();
        const double threshold = 10000.0;

        if ((infm.array() > threshold).any())
        {
            return;
        }
        if ((infm.array() < -threshold).any())
        {
            return;
        }

        // check nv
        Eigen::Vector3d &nv = this_bucket.normal_vector;
        Eigen::Vector3d viewport = intermediate_trajectory[intermediate_points_i.index_pose].translation();
        if (nv.dot(viewport - this_bucket.mean) < 0)
        {
            return;
        }

        const Eigen::Affine3d &m_pose = intermediate_trajectory[intermediate_points_i.index_pose];
        const Eigen::Vector3d &p_s = intermediate_points_i.point;
        const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose);
        //

        Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AtPA;
        point_to_point_source_to_target_tait_bryan_wc_AtPA_simplified(
            AtPA,
            pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
            p_s.x(), p_s.y(), p_s.z(),
            infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2));

        Eigen::Matrix<double, 6, 1> AtPB;
        point_to_point_source_to_target_tait_bryan_wc_AtPB_simplified(
            AtPB,
            pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
            p_s.x(), p_s.y(), p_s.z(),
            infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2),
            this_bucket.mean.x(), this_bucket.mean.y(), this_bucket.mean.z());

        int c = intermediate_points_i.index_pose * 6;

        std::mutex &m = mutexes[intermediate_points_i.index_pose];
        std::unique_lock lck(m);
        AtPAndt.block<6, 6>(c, c) += AtPA;
        AtPBndt.block<6, 1>(c, 0) -= AtPB;
    };

    if (multithread)
    {
        std::for_each(std::execution::par_unseq, std::begin(intermediate_points), std::end(intermediate_points), hessian_fun);
    }
    else
    {
        std::for_each(std::begin(intermediate_points), std::end(intermediate_points), hessian_fun);
    }
    std::vector<std::pair<int, int>> odo_edges;
    for (size_t i = 1; i < intermediate_trajectory.size(); i++)
    {
        odo_edges.emplace_back(i - 1, i);
    }

    std::vector<TaitBryanPose> poses;
    std::vector<TaitBryanPose> poses_desired;

    for (size_t i = 0; i < intermediate_trajectory.size(); i++)
    {
        poses.push_back(pose_tait_bryan_from_affine_matrix(intermediate_trajectory[i]));
    }
    for (size_t i = 0; i < intermediate_trajectory_motion_model.size(); i++)
    {
        poses_desired.push_back(pose_tait_bryan_from_affine_matrix(intermediate_trajectory_motion_model[i]));
    }

    /*for (size_t i = 0; i < odo_edges.size(); i++)
    {
        Eigen::Matrix<double, 6, 1> relative_pose_measurement_odo;
        relative_pose_tait_bryan_wc_case1(relative_pose_measurement_odo,
                                          poses_desired[odo_edges[i].first].px,
                                          poses_desired[odo_edges[i].first].py,
                                          poses_desired[odo_edges[i].first].pz,
                                          poses_desired[odo_edges[i].first].om,
                                          poses_desired[odo_edges[i].first].fi,
                                          poses_desired[odo_edges[i].first].ka,
                                          poses_desired[odo_edges[i].second].px,
                                          poses_desired[odo_edges[i].second].py,
                                          poses_desired[odo_edges[i].second].pz,
                                          poses_desired[odo_edges[i].second].om,
                                          poses_desired[odo_edges[i].second].fi,
                                          poses_desired[odo_edges[i].second].ka);

        Eigen::Matrix<double, 6, 1> delta;
        relative_pose_obs_eq_tait_bryan_wc_case1(
            delta,
            poses[odo_edges[i].first].px,
            poses[odo_edges[i].first].py,
            poses[odo_edges[i].first].pz,
            poses[odo_edges[i].first].om,
            poses[odo_edges[i].first].fi,
            poses[odo_edges[i].first].ka,
            poses[odo_edges[i].second].px,
            poses[odo_edges[i].second].py,
            poses[odo_edges[i].second].pz,
            poses[odo_edges[i].second].om,
            poses[odo_edges[i].second].fi,
            poses[odo_edges[i].second].ka,
            relative_pose_measurement_odo(0, 0),
            relative_pose_measurement_odo(1, 0),
            relative_pose_measurement_odo(2, 0),
            relative_pose_measurement_odo(3, 0),
            relative_pose_measurement_odo(4, 0),
            relative_pose_measurement_odo(5, 0));

        Eigen::Matrix<double, 6, 12, Eigen::RowMajor> jacobian;
        relative_pose_obs_eq_tait_bryan_wc_case1_jacobian(jacobian,
                                                          poses[odo_edges[i].first].px,
                                                          poses[odo_edges[i].first].py,
                                                          poses[odo_edges[i].first].pz,
                                                          poses[odo_edges[i].first].om,
                                                          poses[odo_edges[i].first].fi,
                                                          poses[odo_edges[i].first].ka,
                                                          poses[odo_edges[i].second].px,
                                                          poses[odo_edges[i].second].py,
                                                          poses[odo_edges[i].second].pz,
                                                          poses[odo_edges[i].second].om,
                                                          poses[odo_edges[i].second].fi,
                                                          poses[odo_edges[i].second].ka);

        int ir = tripletListB.size();

        int ic_1 = odo_edges[i].first * 6;
        int ic_2 = odo_edges[i].second * 6;

        for (size_t row = 0; row < 6; row++)
        {
            tripletListA.emplace_back(ir + row, ic_1, -jacobian(row, 0));
            tripletListA.emplace_back(ir + row, ic_1 + 1, -jacobian(row, 1));
            tripletListA.emplace_back(ir + row, ic_1 + 2, -jacobian(row, 2));
            tripletListA.emplace_back(ir + row, ic_1 + 3, -jacobian(row, 3));
            tripletListA.emplace_back(ir + row, ic_1 + 4, -jacobian(row, 4));
            tripletListA.emplace_back(ir + row, ic_1 + 5, -jacobian(row, 5));

            tripletListA.emplace_back(ir + row, ic_2, -jacobian(row, 6));
            tripletListA.emplace_back(ir + row, ic_2 + 1, -jacobian(row, 7));
            tripletListA.emplace_back(ir + row, ic_2 + 2, -jacobian(row, 8));
            tripletListA.emplace_back(ir + row, ic_2 + 3, -jacobian(row, 9));
            tripletListA.emplace_back(ir + row, ic_2 + 4, -jacobian(row, 10));
            tripletListA.emplace_back(ir + row, ic_2 + 5, -jacobian(row, 11));
        }

        tripletListB.emplace_back(ir, 0, delta(0, 0));
        tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
        tripletListB.emplace_back(ir + 2, 0, delta(2, 0));
        tripletListB.emplace_back(ir + 3, 0, delta(3, 0));
        tripletListB.emplace_back(ir + 4, 0, delta(4, 0));
        tripletListB.emplace_back(ir + 5, 0, delta(5, 0));

        tripletListP.emplace_back(ir, ir, 1000000);
        tripletListP.emplace_back(ir + 1, ir + 1, 1000000);
        tripletListP.emplace_back(ir + 2, ir + 2, 1000000);
        tripletListP.emplace_back(ir + 3, ir + 3, 100000000);
        tripletListP.emplace_back(ir + 4, ir + 4, 100000000);
        tripletListP.emplace_back(ir + 5, ir + 5, 1000000);
    }*/
    for (size_t i = 0; i < odo_edges.size(); i++)
    {
        Eigen::Matrix<double, 6, 1> relative_pose_measurement_odo;
        relative_pose_tait_bryan_wc_case1_simplified_1(relative_pose_measurement_odo,
                                                       poses_desired[odo_edges[i].first].px,
                                                       poses_desired[odo_edges[i].first].py,
                                                       poses_desired[odo_edges[i].first].pz,
                                                       poses_desired[odo_edges[i].first].om,
                                                       poses_desired[odo_edges[i].first].fi,
                                                       poses_desired[odo_edges[i].first].ka,
                                                       poses_desired[odo_edges[i].second].px,
                                                       poses_desired[odo_edges[i].second].py,
                                                       poses_desired[odo_edges[i].second].pz,
                                                       poses_desired[odo_edges[i].second].om,
                                                       poses_desired[odo_edges[i].second].fi,
                                                       poses_desired[odo_edges[i].second].ka);

        Eigen::Matrix<double, 12, 12> AtPAodo;
        relative_pose_obs_eq_tait_bryan_wc_case1_AtPA_simplified(AtPAodo,
                                                                 poses[odo_edges[i].first].px,
                                                                 poses[odo_edges[i].first].py,
                                                                 poses[odo_edges[i].first].pz,
                                                                 poses[odo_edges[i].first].om,
                                                                 poses[odo_edges[i].first].fi,
                                                                 poses[odo_edges[i].first].ka,
                                                                 poses[odo_edges[i].second].px,
                                                                 poses[odo_edges[i].second].py,
                                                                 poses[odo_edges[i].second].pz,
                                                                 poses[odo_edges[i].second].om,
                                                                 poses[odo_edges[i].second].fi,
                                                                 poses[odo_edges[i].second].ka,
                                                                 1000000,
                                                                 1000000,
                                                                 1000000,
                                                                 100000000,
                                                                 100000000,
                                                                 // 100000000, underground mining
                                                                 // 1000000, underground mining
                                                                 1000000);
        Eigen::Matrix<double, 12, 1> AtPBodo;
        relative_pose_obs_eq_tait_bryan_wc_case1_AtPB_simplified(AtPBodo,
                                                                 poses[odo_edges[i].first].px,
                                                                 poses[odo_edges[i].first].py,
                                                                 poses[odo_edges[i].first].pz,
                                                                 poses[odo_edges[i].first].om,
                                                                 poses[odo_edges[i].first].fi,
                                                                 poses[odo_edges[i].first].ka,
                                                                 poses[odo_edges[i].second].px,
                                                                 poses[odo_edges[i].second].py,
                                                                 poses[odo_edges[i].second].pz,
                                                                 poses[odo_edges[i].second].om,
                                                                 poses[odo_edges[i].second].fi,
                                                                 poses[odo_edges[i].second].ka,
                                                                 relative_pose_measurement_odo(0, 0),
                                                                 relative_pose_measurement_odo(1, 0),
                                                                 relative_pose_measurement_odo(2, 0),
                                                                 relative_pose_measurement_odo(3, 0),
                                                                 relative_pose_measurement_odo(4, 0),
                                                                 relative_pose_measurement_odo(5, 0),
                                                                 1000000,
                                                                 1000000,
                                                                 1000000,
                                                                 100000000,
                                                                 100000000,
                                                                 // 100000000, underground mining
                                                                 // 1000000, underground mining
                                                                 1000000);
        int ic_1 = odo_edges[i].first * 6;
        int ic_2 = odo_edges[i].second * 6;

        for (int row = 0; row < 6; row++)
        {
            for (int col = 0; col < 6; col++)
            {
                AtPAndt(ic_1 + row, ic_1 + col) += AtPAodo(row, col);
                AtPAndt(ic_1 + row, ic_2 + col) += AtPAodo(row, col + 6);
                AtPAndt(ic_2 + row, ic_1 + col) += AtPAodo(row + 6, col);
                AtPAndt(ic_2 + row, ic_2 + col) += AtPAodo(row + 6, col + 6);
            }
        }

        for (int row = 0; row < 6; row++)
        {
            AtPBndt(ic_1 + row, 0) -= AtPBodo(row, 0);
            AtPBndt(ic_2 + row, 0) -= AtPBodo(row + 6, 0);
        }
    }

    // smoothness
    /*for (size_t i = 1; i < poses.size() - 1; i++)
    {
        Eigen::Matrix<double, 6, 1> delta;
        smoothness_obs_eq_tait_bryan_wc(delta,
                                        poses[i - 1].px,
                                        poses[i - 1].py,
                                        poses[i - 1].pz,
                                        poses[i - 1].om,
                                        poses[i - 1].fi,
                                        poses[i - 1].ka,
                                        poses[i].px,
                                        poses[i].py,
                                        poses[i].pz,
                                        poses[i].om,
                                        poses[i].fi,
                                        poses[i].ka,
                                        poses[i + 1].px,
                                        poses[i + 1].py,
                                        poses[i + 1].pz,
                                        poses[i + 1].om,
                                        poses[i + 1].fi,
                                        poses[i + 1].ka);

        Eigen::Matrix<double, 6, 18, Eigen::RowMajor> jacobian;
        smoothness_obs_eq_tait_bryan_wc_jacobian(jacobian,
                                                 poses[i - 1].px,
                                                 poses[i - 1].py,
                                                 poses[i - 1].pz,
                                                 poses[i - 1].om,
                                                 poses[i - 1].fi,
                                                 poses[i - 1].ka,
                                                 poses[i].px,
                                                 poses[i].py,
                                                 poses[i].pz,
                                                 poses[i].om,
                                                 poses[i].fi,
                                                 poses[i].ka,
                                                 poses[i + 1].px,
                                                 poses[i + 1].py,
                                                 poses[i + 1].pz,
                                                 poses[i + 1].om,
                                                 poses[i + 1].fi,
                                                 poses[i + 1].ka);

        int ir = tripletListB.size();

        int ic_1 = (i - 1) * 6;
        int ic_2 = i * 6;
        int ic_3 = (i + 1) * 6;

        for (size_t row = 0; row < 6; row++)
        {
            tripletListA.emplace_back(ir + row, ic_1, -jacobian(row, 0));
            tripletListA.emplace_back(ir + row, ic_1 + 1, -jacobian(row, 1));
            tripletListA.emplace_back(ir + row, ic_1 + 2, -jacobian(row, 2));
            tripletListA.emplace_back(ir + row, ic_1 + 3, -jacobian(row, 3));
            tripletListA.emplace_back(ir + row, ic_1 + 4, -jacobian(row, 4));
            tripletListA.emplace_back(ir + row, ic_1 + 5, -jacobian(row, 5));

            tripletListA.emplace_back(ir + row, ic_2, -jacobian(row, 6));
            tripletListA.emplace_back(ir + row, ic_2 + 1, -jacobian(row, 7));
            tripletListA.emplace_back(ir + row, ic_2 + 2, -jacobian(row, 8));
            tripletListA.emplace_back(ir + row, ic_2 + 3, -jacobian(row, 9));
            tripletListA.emplace_back(ir + row, ic_2 + 4, -jacobian(row, 10));
            tripletListA.emplace_back(ir + row, ic_2 + 5, -jacobian(row, 11));

            tripletListA.emplace_back(ir + row, ic_3, -jacobian(row, 12));
            tripletListA.emplace_back(ir + row, ic_3 + 1, -jacobian(row, 13));
            tripletListA.emplace_back(ir + row, ic_3 + 2, -jacobian(row, 14));
            tripletListA.emplace_back(ir + row, ic_3 + 3, -jacobian(row, 15));
            tripletListA.emplace_back(ir + row, ic_3 + 4, -jacobian(row, 16));
            tripletListA.emplace_back(ir + row, ic_3 + 5, -jacobian(row, 17));
        }
        tripletListB.emplace_back(ir, 0, delta(0, 0));
        tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
        tripletListB.emplace_back(ir + 2, 0, delta(2, 0));
        tripletListB.emplace_back(ir + 3, 0, delta(3, 0));
        tripletListB.emplace_back(ir + 4, 0, delta(4, 0));
        tripletListB.emplace_back(ir + 5, 0, delta(5, 0));

        tripletListP.emplace_back(ir, ir, 10000);
        tripletListP.emplace_back(ir + 1, ir + 1, 10000);
        tripletListP.emplace_back(ir + 2, ir + 2, 10000);
        tripletListP.emplace_back(ir + 3, ir + 3, 10000);
        tripletListP.emplace_back(ir + 4, ir + 4, 10000);
        tripletListP.emplace_back(ir + 5, ir + 5, 10000);
    }*/

    // maintain angles
    if (add_pitch_roll_constraint)
    {
        for (int i = 0; i < imu_roll_pitch.size(); i++)
        {
            TaitBryanPose current_pose = poses[i];
            TaitBryanPose desired_pose = current_pose;
            desired_pose.om = imu_roll_pitch[i].first;
            desired_pose.fi = imu_roll_pitch[i].second;

            Eigen::Affine3d desired_mpose = affine_matrix_from_pose_tait_bryan(desired_pose);
            Eigen::Vector3d vx(desired_mpose(0, 0), desired_mpose(1, 0), desired_mpose(2, 0));
            Eigen::Vector3d vy(desired_mpose(0, 1), desired_mpose(1, 1), desired_mpose(2, 1));
            Eigen::Vector3d point_on_target_line(desired_mpose(0, 3), desired_mpose(1, 3), desired_mpose(2, 3));

            Eigen::Vector3d point_source_local(0, 0, 1);

            Eigen::Matrix<double, 2, 1> delta;
            point_to_line_tait_bryan_wc(delta,
                                        current_pose.px, current_pose.py, current_pose.pz, current_pose.om, current_pose.fi, current_pose.ka,
                                        point_source_local.x(), point_source_local.y(), point_source_local.z(),
                                        point_on_target_line.x(), point_on_target_line.y(), point_on_target_line.z(),
                                        vx.x(), vx.y(), vx.z(), vy.x(), vy.y(), vy.z());

            Eigen::Matrix<double, 2, 6> delta_jacobian;
            point_to_line_tait_bryan_wc_jacobian(delta_jacobian,
                                                 current_pose.px, current_pose.py, current_pose.pz, current_pose.om, current_pose.fi, current_pose.ka,
                                                 point_source_local.x(), point_source_local.y(), point_source_local.z(),
                                                 point_on_target_line.x(), point_on_target_line.y(), point_on_target_line.z(),
                                                 vx.x(), vx.y(), vx.z(), vy.x(), vy.y(), vy.z());

            int ir = tripletListB.size();

            for (int ii = 0; ii < 2; ii++)
            {
                for (int jj = 0; jj < 6; jj++)
                {
                    int ic = i * 6;
                    if (delta_jacobian(ii, jj) != 0.0)
                    {
                        tripletListA.emplace_back(ir + ii, ic + jj, -delta_jacobian(ii, jj));
                    }
                }
            }
            // tripletListP.emplace_back(ir, ir, cauchy(delta(0, 0), 1));
            // tripletListP.emplace_back(ir + 1, ir + 1, cauchy(delta(1, 0), 1));
            tripletListP.emplace_back(ir, ir, 1);
            tripletListP.emplace_back(ir + 1, ir + 1, 1);

            tripletListB.emplace_back(ir, 0, delta(0, 0));
            tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
        }
    }

    // underground mining
    /*double angle = 1.0 / 180.0 * M_PI;
    double w_angle = 1.0 / (angle * angle);

    double angle01 = 0.1 / 180.0 * M_PI;
    double w_angle01 = 1.0 / (angle01 * angle01);

    for (int ic = 0; ic < intermediate_trajectory.size(); ic ++){
        int ir = tripletListB.size();
        tripletListA.emplace_back(ir    , ic * 6 + 0, 1);
        tripletListA.emplace_back(ir + 1, ic * 6 + 1, 1);
        tripletListA.emplace_back(ir + 2, ic * 6 + 2, 1);
        tripletListA.emplace_back(ir + 3, ic * 6 + 3, 1);
        tripletListA.emplace_back(ir + 4, ic * 6 + 4, 1);
        tripletListA.emplace_back(ir + 5, ic * 6 + 5, 1);

        tripletListP.emplace_back(ir, ir, 0);
        tripletListP.emplace_back(ir + 1, ir + 1, 0);
        tripletListP.emplace_back(ir + 2, ir + 2, 0);
        tripletListP.emplace_back(ir + 3, ir + 3, w_angle01);
        tripletListP.emplace_back(ir + 4, ir + 4, w_angle01);
        tripletListP.emplace_back(ir + 5, ir + 5, 0);

        tripletListB.emplace_back(ir, 0, 0);
        tripletListB.emplace_back(ir + 1, 0, 0);
        tripletListB.emplace_back(ir + 2, 0, 0);
        tripletListB.emplace_back(ir + 3, 0, 0);
        tripletListB.emplace_back(ir + 4, 0, 0);
        tripletListB.emplace_back(ir + 5, 0, 0);
    }*/

    Eigen::SparseMatrix<double> matA(tripletListB.size(), intermediate_trajectory.size() * 6);
    Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
    Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

    matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
    matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
    matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

    Eigen::SparseMatrix<double> AtPA(intermediate_trajectory.size() * 6, intermediate_trajectory.size() * 6);
    Eigen::SparseMatrix<double> AtPB(intermediate_trajectory.size() * 6, 1);

    {
        Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
        AtPA = (AtP)*matA;
        AtPB = (AtP)*matB;
    }

    tripletListA.clear();
    tripletListP.clear();
    tripletListB.clear();

    AtPA += AtPAndt.sparseView();
    AtPB += AtPBndt.sparseView();
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

    if (h_x.size() == 6 * intermediate_trajectory.size())
    {
        int counter = 0;

        for (size_t i = 0; i < intermediate_trajectory.size(); i++)
        {
            TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(intermediate_trajectory[i]);
            auto prev_pose = pose;
            pose.px += h_x[counter++];
            pose.py += h_x[counter++];
            pose.pz += h_x[counter++];
            pose.om += h_x[counter++];
            pose.fi += h_x[counter++];
            pose.ka += h_x[counter++];

            Eigen::Vector3d p1(prev_pose.px, prev_pose.py, prev_pose.pz);
            Eigen::Vector3d p2(pose.px, pose.py, pose.pz);

            if((p1 - p2).norm() < 1.0){
                intermediate_trajectory[i] = affine_matrix_from_pose_tait_bryan(pose);
            }
        }
    }
    return;
}

void align_to_reference(NDT::GridParameters &rgd_params, std::vector<Point3Di> &initial_points, Eigen::Affine3d &m_g, NDTBucketMapType &reference_buckets)
{
    Eigen::SparseMatrix<double> AtPAndt(6, 6);
    Eigen::SparseMatrix<double> AtPBndt(6, 1);

    Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

    for (int i = 0; i < initial_points.size(); i += 1)
    {
        // if (initial_points[i].point.norm() < 1.0)
        //{
        //     continue;
        // }

        Eigen::Vector3d point_global = m_g * initial_points[i].point;
        auto index_of_bucket = get_rgd_index(point_global, b);

        if (!reference_buckets.contains(index_of_bucket))
        {
            continue;
        }

        // if(buckets[index_of_bucket].number_of_points >= 5){
        Eigen::Matrix3d infm = reference_buckets[index_of_bucket].cov.inverse();

        constexpr double threshold = 10000.0;

        if ((infm.array() > threshold).any())
        {
            continue;
        }
        if ((infm.array() < -threshold).any())
        {
            continue;
        }

        const Eigen::Affine3d &m_pose = m_g;
        const Eigen::Vector3d &p_s = initial_points[i].point;
        const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose);
        //
        Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AtPA;
        point_to_point_source_to_target_tait_bryan_wc_AtPA_simplified(
            AtPA,
            pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
            p_s.x(), p_s.y(), p_s.z(),
            infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2));

        Eigen::Matrix<double, 6, 1> AtPB;
        point_to_point_source_to_target_tait_bryan_wc_AtPB_simplified(
            AtPB,
            pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
            p_s.x(), p_s.y(), p_s.z(),
            infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2),
            reference_buckets[index_of_bucket].mean.x(), reference_buckets[index_of_bucket].mean.y(), reference_buckets[index_of_bucket].mean.z());

        int c = 0;

        for (int row = 0; row < 6; row++)
        {
            for (int col = 0; col < 6; col++)
            {
                AtPAndt.coeffRef(c + row, c + col) += AtPA(row, col);
            }
        }

        for (int row = 0; row < 6; row++)
        {
            AtPBndt.coeffRef(c + row, 0) -= AtPB(row, 0);
        }
        //}
    }

    AtPAndt.coeffRef(0, 0) += 10000.0;
    AtPAndt.coeffRef(1, 1) += 10000.0;
    AtPAndt.coeffRef(2, 2) += 10000.0;
    AtPAndt.coeffRef(3, 3) += 10000.0;
    AtPAndt.coeffRef(4, 4) += 10000.0;
    AtPAndt.coeffRef(5, 5) += 10000.0;

    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPAndt);
    Eigen::SparseMatrix<double> x = solver.solve(AtPBndt);
    std::vector<double> h_x;
    for (int k = 0; k < x.outerSize(); ++k)
    {
        for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
        {
            h_x.push_back(it.value());
        }
    }

    if (h_x.size() == 6)
    {
        int counter = 0;
        TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(m_g);
        pose.px += h_x[counter++];
        pose.py += h_x[counter++];
        pose.pz += h_x[counter++];
        pose.om += h_x[counter++];
        pose.fi += h_x[counter++];
        pose.ka += h_x[counter++];
        m_g = affine_matrix_from_pose_tait_bryan(pose);
    }
    else
    {
        std::cout << "align_to_reference FAILED" << std::endl;
    }
}

void fix_ptch_roll(std::vector<WorkerData> &worker_data)
{
    std::vector<Eigen::Triplet<double>> tripletListA;
    std::vector<Eigen::Triplet<double>> tripletListP;
    std::vector<Eigen::Triplet<double>> tripletListB;

    std::vector<TaitBryanPose> poses;

    for (size_t i = 0; i < worker_data.size(); i++)
    {
        poses.push_back(pose_tait_bryan_from_affine_matrix(worker_data[i].intermediate_trajectory[0]));
    }

    for (size_t i = 1; i < poses.size(); i++)
    {
        TaitBryanPose pose_prev = pose_tait_bryan_from_affine_matrix(worker_data[i - 1].intermediate_trajectory_motion_model[0]);
        pose_prev.om = worker_data[i - 1].imu_roll_pitch[0].first;
        pose_prev.fi = worker_data[i - 1].imu_roll_pitch[0].second;
        Eigen::Affine3d mrot_prev = affine_matrix_from_pose_tait_bryan(pose_prev);
        mrot_prev(0, 3) = 0;
        mrot_prev(1, 3) = 0;
        mrot_prev(2, 3) = 0;

        TaitBryanPose pose_curr = pose_tait_bryan_from_affine_matrix(worker_data[i].intermediate_trajectory_motion_model[0]);
        pose_curr.om = worker_data[i].imu_roll_pitch[0].first;
        pose_curr.fi = worker_data[i].imu_roll_pitch[0].second;
        Eigen::Affine3d mrot_curr = affine_matrix_from_pose_tait_bryan(pose_curr);
        mrot_curr(0, 3) = 0;
        mrot_curr(1, 3) = 0;
        mrot_curr(2, 3) = 0;

        auto m_rot_rel = mrot_prev.inverse() * mrot_curr;
        auto tb_rot_rel = pose_tait_bryan_from_affine_matrix(m_rot_rel);

        Eigen::Vector3d relative_translation = (worker_data[i - 1].intermediate_trajectory_motion_model[0].inverse() *
                                                worker_data[i].intermediate_trajectory_motion_model[0])
                                                   .translation();

        auto m = worker_data[i - 1].intermediate_trajectory_motion_model[0];
        m(0, 3) = 0;
        m(1, 3) = 0;
        m(2, 3) = 0;

        relative_translation = (mrot_prev.inverse() * m) * relative_translation;

        Eigen::Matrix<double, 6, 1> delta;
        relative_pose_obs_eq_tait_bryan_wc_case1(
            delta,
            poses[i - 1].px,
            poses[i - 1].py,
            poses[i - 1].pz,
            poses[i - 1].om,
            poses[i - 1].fi,
            poses[i - 1].ka,
            poses[i].px,
            poses[i].py,
            poses[i].pz,
            poses[i].om,
            poses[i].fi,
            poses[i].ka,
            relative_translation.x(),
            relative_translation.y(),
            relative_translation.z(),
            tb_rot_rel.om,
            tb_rot_rel.fi,
            tb_rot_rel.ka);

        Eigen::Matrix<double, 6, 12, Eigen::RowMajor> jacobian;
        relative_pose_obs_eq_tait_bryan_wc_case1_jacobian(jacobian,
                                                          poses[i - 1].px,
                                                          poses[i - 1].py,
                                                          poses[i - 1].pz,
                                                          poses[i - 1].om,
                                                          poses[i - 1].fi,
                                                          poses[i - 1].ka,
                                                          poses[i].px,
                                                          poses[i].py,
                                                          poses[i].pz,
                                                          poses[i].om,
                                                          poses[i].fi,
                                                          poses[i].ka);

        int ir = tripletListB.size();

        int ic_1 = (i - 1) * 6;
        int ic_2 = i * 6;

        for (size_t row = 0; row < 6; row++)
        {
            tripletListA.emplace_back(ir + row, ic_1, -jacobian(row, 0));
            tripletListA.emplace_back(ir + row, ic_1 + 1, -jacobian(row, 1));
            tripletListA.emplace_back(ir + row, ic_1 + 2, -jacobian(row, 2));
            tripletListA.emplace_back(ir + row, ic_1 + 3, -jacobian(row, 3));
            tripletListA.emplace_back(ir + row, ic_1 + 4, -jacobian(row, 4));
            tripletListA.emplace_back(ir + row, ic_1 + 5, -jacobian(row, 5));

            tripletListA.emplace_back(ir + row, ic_2, -jacobian(row, 6));
            tripletListA.emplace_back(ir + row, ic_2 + 1, -jacobian(row, 7));
            tripletListA.emplace_back(ir + row, ic_2 + 2, -jacobian(row, 8));
            tripletListA.emplace_back(ir + row, ic_2 + 3, -jacobian(row, 9));
            tripletListA.emplace_back(ir + row, ic_2 + 4, -jacobian(row, 10));
            tripletListA.emplace_back(ir + row, ic_2 + 5, -jacobian(row, 11));
        }

        tripletListB.emplace_back(ir, 0, delta(0, 0));
        tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
        tripletListB.emplace_back(ir + 2, 0, delta(2, 0));
        tripletListB.emplace_back(ir + 3, 0, delta(3, 0));
        tripletListB.emplace_back(ir + 4, 0, delta(4, 0));
        tripletListB.emplace_back(ir + 5, 0, delta(5, 0));

        tripletListP.emplace_back(ir, ir, 1);
        tripletListP.emplace_back(ir + 1, ir + 1, 1);
        tripletListP.emplace_back(ir + 2, ir + 2, 1);
        tripletListP.emplace_back(ir + 3, ir + 3, 1);
        tripletListP.emplace_back(ir + 4, ir + 4, 1);
        tripletListP.emplace_back(ir + 5, ir + 5, 1);
    }

    double rms = 0.0;
    for (size_t i = 0; i < worker_data.size(); i++)
    {
        TaitBryanPose current_pose = pose_tait_bryan_from_affine_matrix(worker_data[i].intermediate_trajectory[0]);
        TaitBryanPose desired_pose = current_pose;
        desired_pose.om = worker_data[i].imu_roll_pitch[0].first;
        desired_pose.fi = worker_data[i].imu_roll_pitch[0].second;

        Eigen::Affine3d desired_mpose = affine_matrix_from_pose_tait_bryan(desired_pose);
        Eigen::Vector3d vx(desired_mpose(0, 0), desired_mpose(1, 0), desired_mpose(2, 0));
        Eigen::Vector3d vy(desired_mpose(0, 1), desired_mpose(1, 1), desired_mpose(2, 1));
        Eigen::Vector3d point_on_target_line(desired_mpose(0, 3), desired_mpose(1, 3), desired_mpose(2, 3));

        Eigen::Vector3d point_source_local(0, 0, 1);

        Eigen::Matrix<double, 2, 1> delta;
        point_to_line_tait_bryan_wc(delta,
                                    current_pose.px, current_pose.py, current_pose.pz, current_pose.om, current_pose.fi, current_pose.ka,
                                    point_source_local.x(), point_source_local.y(), point_source_local.z(),
                                    point_on_target_line.x(), point_on_target_line.y(), point_on_target_line.z(),
                                    vx.x(), vx.y(), vx.z(), vy.x(), vy.y(), vy.z());

        Eigen::Matrix<double, 2, 6> delta_jacobian;
        point_to_line_tait_bryan_wc_jacobian(delta_jacobian,
                                             current_pose.px, current_pose.py, current_pose.pz, current_pose.om, current_pose.fi, current_pose.ka,
                                             point_source_local.x(), point_source_local.y(), point_source_local.z(),
                                             point_on_target_line.x(), point_on_target_line.y(), point_on_target_line.z(),
                                             vx.x(), vx.y(), vx.z(), vy.x(), vy.y(), vy.z());

        int ir = tripletListB.size();

        for (int ii = 0; ii < 2; ii++)
        {
            for (int jj = 0; jj < 6; jj++)
            {
                int ic = i * 6;
                if (delta_jacobian(ii, jj) != 0.0)
                {
                    tripletListA.emplace_back(ir + ii, ic + jj, -delta_jacobian(ii, jj));
                }
            }
        }
        // tripletListP.emplace_back(ir, ir, cauchy(delta(0, 0), 1));
        // tripletListP.emplace_back(ir + 1, ir + 1, cauchy(delta(1, 0), 1));
        tripletListP.emplace_back(ir, ir, 1);
        tripletListP.emplace_back(ir + 1, ir + 1, 1);

        tripletListB.emplace_back(ir, 0, delta(0, 0));
        tripletListB.emplace_back(ir + 1, 0, delta(1, 0));

        rms += sqrt(delta(0, 0) * delta(0, 0) + delta(1, 0) * delta(1, 0));
    }
    std::cout << "rms: " << rms << std::endl;

    int ir = tripletListB.size();
    tripletListA.emplace_back(ir, 0, 1);
    tripletListA.emplace_back(ir + 1, 1, 1);
    tripletListA.emplace_back(ir + 2, 2, 1);
    tripletListA.emplace_back(ir + 3, 3, 1);
    tripletListA.emplace_back(ir + 4, 4, 1);
    tripletListA.emplace_back(ir + 5, 5, 1);

    tripletListP.emplace_back(ir, ir, 10000000000000);
    tripletListP.emplace_back(ir + 1, ir + 1, 10000000000000);
    tripletListP.emplace_back(ir + 2, ir + 2, 10000000000000);
    tripletListP.emplace_back(ir + 3, ir + 3, 10000000000000);
    tripletListP.emplace_back(ir + 4, ir + 4, 10000000000000);
    tripletListP.emplace_back(ir + 5, ir + 5, 10000000000000);

    tripletListB.emplace_back(ir, 0, 0);
    tripletListB.emplace_back(ir + 1, 0, 0);
    tripletListB.emplace_back(ir + 2, 0, 0);
    tripletListB.emplace_back(ir + 3, 0, 0);
    tripletListB.emplace_back(ir + 4, 0, 0);
    tripletListB.emplace_back(ir + 5, 0, 0);

    Eigen::SparseMatrix<double> matA(tripletListB.size(), poses.size() * 6);
    Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
    Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

    matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
    matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
    matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

    Eigen::SparseMatrix<double> AtPA(poses.size() * 6, poses.size() * 6);
    Eigen::SparseMatrix<double> AtPB(poses.size() * 6, 1);

    {
        Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
        AtPA = (AtP)*matA;
        AtPB = (AtP)*matB;
    }

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

    if (h_x.size() == 6 * poses.size())
    {
        std::vector<Eigen::Affine3d> results;

        int counter = 0;

        for (size_t i = 0; i < poses.size(); i++)
        {

            poses[i].px += h_x[counter++];
            poses[i].py += h_x[counter++];
            poses[i].pz += h_x[counter++];
            poses[i].om += h_x[counter++];
            poses[i].fi += h_x[counter++];
            poses[i].ka += h_x[counter++];

            // worker_data[i].intermediate_trajectory[0] = affine_matrix_from_pose_tait_bryan(poses[i]);
            results.push_back(affine_matrix_from_pose_tait_bryan(poses[i]));
        }

        for (int i = 0; i < worker_data.size(); i++)
        {
            Eigen::Affine3d m_last = results[i]; // worker_data[i].intermediate_trajectory[0];

            std::vector<Eigen::Affine3d> local_result;
            local_result.push_back(m_last);
            for (int j = 1; j < worker_data[i].intermediate_trajectory.size(); j++)
            {
                m_last = m_last * (worker_data[i].intermediate_trajectory[j - 1].inverse() * worker_data[i].intermediate_trajectory[j]);
                local_result.push_back(m_last);
            }

            for (int j = 0; j < worker_data[i].intermediate_trajectory.size(); j++)
            {
                worker_data[i].intermediate_trajectory[j] = local_result[j];
            }
        }
    }
}

bool compute_step_2(std::vector<WorkerData> &worker_data, LidarOdometryParams &params, double &ts_failure)
{
    // std::vector<Eigen::Affine3d> mm_poses;
    // for (int i = 0; i < worker_data.size(); i++)
    //{
    //     mm_poses.push_back(worker_data[i].intermediate_trajectory_motion_model[0]);
    // }

    if (worker_data.size() != 0)
    {

        std::chrono::time_point<std::chrono::system_clock> start, end;
        start = std::chrono::system_clock::now();
        double acc_distance = 0.0;
        std::vector<Point3Di> points_global;

        Eigen::Affine3d m_last = params.m_g;
        auto tmp = worker_data[0].intermediate_trajectory;

        worker_data[0].intermediate_trajectory[0] = m_last;
        for (int k = 1; k < tmp.size(); k++)
        {
            Eigen::Affine3d m_update = tmp[k - 1].inverse() * tmp[k];
            m_last = m_last * m_update;
            worker_data[0].intermediate_trajectory[k] = m_last;
        }
        worker_data[0].intermediate_trajectory_motion_model = worker_data[0].intermediate_trajectory;

        auto pp = params.initial_points;
        for (int i = 0; i < pp.size(); i++)
        {
            pp[i].point = params.m_g * pp[i].point;
        }
        update_rgd(params.in_out_params, params.buckets, pp, params.m_g.translation());

        for (int i = 0; i < worker_data.size(); i++)
        {
            Eigen::Vector3d mean_shift(0.0, 0.0, 0.0);
            if (i > 1 && params.use_motion_from_previous_step)
            {
                mean_shift = worker_data[i - 1].intermediate_trajectory[0].translation() - worker_data[i - 2].intermediate_trajectory[worker_data[i - 2].intermediate_trajectory.size() - 1].translation();
                mean_shift /= ((worker_data[i - 2].intermediate_trajectory.size()) - 2);

                if (mean_shift.norm() > 1.0)
                {
                    std::cout << "!!!mean_shift.norm() > 1.0!!!" << std::endl;
                    mean_shift = Eigen::Vector3d(0.0, 0.0, 0.0);
                }

                std::vector<Eigen::Affine3d> new_trajectory;
                Eigen::Affine3d current_node = worker_data[i].intermediate_trajectory[0];
                new_trajectory.push_back(current_node);

                for (int tr = 1; tr < worker_data[i].intermediate_trajectory.size(); tr++)
                {
                    current_node.linear() = worker_data[i].intermediate_trajectory[tr].linear();
                    current_node.translation() += mean_shift;
                    new_trajectory.push_back(current_node);
                }

                worker_data[i].intermediate_trajectory = new_trajectory;
                ////////////////////////////////////////////////////////////////////////
                std::vector<Eigen::Affine3d> new_trajectory_motion_model;
                Eigen::Affine3d current_node_motion_model = worker_data[i].intermediate_trajectory_motion_model[0];
                new_trajectory_motion_model.push_back(current_node_motion_model);

                Eigen::Vector3d mean_shift_t = worker_data[i].intermediate_trajectory_motion_model[0].linear() * ((worker_data[i].intermediate_trajectory[0].linear()).inverse() * mean_shift);

                for (int tr = 1; tr < worker_data[i].intermediate_trajectory_motion_model.size(); tr++)
                {
                    current_node_motion_model.linear() = worker_data[i].intermediate_trajectory_motion_model[tr].linear();
                    current_node_motion_model.translation() += mean_shift_t;
                    new_trajectory_motion_model.push_back(current_node_motion_model);
                }

                worker_data[i].intermediate_trajectory_motion_model = new_trajectory_motion_model;
            }

            bool add_pitch_roll_constraint = false;
            // TaitBryanPose pose;
            // pose = pose_tait_bryan_from_affine_matrix(worker_data[i].intermediate_trajectory[0]);

            // double residual1;
            // double residual2;
            // residual_constraint_fixed_optimization_parameter(residual1, normalize_angle(worker_data[i].imu_roll_pitch[0].first), normalize_angle(pose.om));
            // residual_constraint_fixed_optimization_parameter(residual2, normalize_angle(worker_data[i].imu_roll_pitch[0].second), normalize_angle(pose.fi));

            // if (fabs(worker_data[i].imu_roll_pitch[0].first) < 30.0 / 180.0 * M_PI && fabs(worker_data[i].imu_roll_pitch[0].second) < 30.0 / 180.0 * M_PI)
            //{
            //     if (params.consecutive_distance > 10.0)
            //     {
            //        add_pitch_roll_constraint = true;
            //        params.consecutive_distance = 0.0;
            //    }
            //}

            // if (add_pitch_roll_constraint)
            //{
            //     std::cout << "residual_imu_roll_deg before: " << residual1 / M_PI * 180.0 << std::endl;
            //     std::cout << "residual_imu_pitch_deg before: " << residual2 / M_PI * 180.0 << std::endl;
            // }

            std::chrono::time_point<std::chrono::system_clock> start1, end1;
            start1 = std::chrono::system_clock::now();

            auto tmp_worker_data = worker_data[i].intermediate_trajectory;

            for (int iter = 0; iter < params.nr_iter; iter++)
            {
                optimize(worker_data[i].intermediate_points, worker_data[i].intermediate_trajectory, worker_data[i].intermediate_trajectory_motion_model,
                         params.in_out_params, params.buckets, params.useMultithread, add_pitch_roll_constraint, worker_data[i].imu_roll_pitch);
            }
            end1 = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
            std::cout << "optimizing worker_data [" << i + 1 << "] of " << worker_data.size() << " acc_distance: " << acc_distance << " elapsed time: " << elapsed_seconds1.count() << std::endl;

            // if (add_pitch_roll_constraint)
            //{
            //     pose = pose_tait_bryan_from_affine_matrix(worker_data[i].intermediate_trajectory[0]);

            //    residual_constraint_fixed_optimization_parameter(residual1, normalize_angle(worker_data[i].imu_roll_pitch[0].first), normalize_angle(pose.om));
            //    residual_constraint_fixed_optimization_parameter(residual2, normalize_angle(worker_data[i].imu_roll_pitch[0].second), normalize_angle(pose.fi));

            //    std::cout << "residual_imu_roll_deg after: " << residual1 / M_PI * 180.0 << std::endl;
            //    std::cout << "residual_imu_pitch_deg after: " << residual2 / M_PI * 180.0 << std::endl;
            //}

            // align to reference
            if (params.reference_points.size() > 0)
            {
                std::cout << "align to reference" << std::endl;
                Eigen::Affine3d m_first = worker_data[i].intermediate_trajectory[0];
                Eigen::Affine3d m_first_inv = m_first.inverse();

                // create rigid scan
                std::vector<Point3Di> local_points;
                for (int k = 0; k < worker_data[i].intermediate_points.size(); k++)
                {
                    Point3Di p = worker_data[i].intermediate_points[k];
                    int index_pose = p.index_pose;
                    p.point = worker_data[i].intermediate_trajectory[index_pose] * p.point;
                    p.point = m_first_inv * p.point;
                    local_points.push_back(p);
                }
                // std::cout << "before " << m_first.matrix() << std::endl;
                if (params.decimation > 0)
                {
                    local_points = decimate(local_points, params.decimation, params.decimation, params.decimation);
                }
                for (int iter = 0; iter < params.nr_iter; iter++)
                {
                    align_to_reference(params.in_out_params, local_points, m_first, params.reference_buckets);
                }

                auto tmp = worker_data[i].intermediate_trajectory;

                worker_data[i].intermediate_trajectory[0] = m_first;

                for (int k = 1; k < tmp.size(); k++)
                {
                    Eigen::Affine3d m_update = tmp[k - 1].inverse() * tmp[k];
                    m_first = m_first * m_update;
                    worker_data[i].intermediate_trajectory[k] = m_first;
                }
                worker_data[i].intermediate_trajectory_motion_model = worker_data[i].intermediate_trajectory;
            }

            // temp save
            if (i % 100 == 0)
            {
                std::vector<Point3Di> global_points;
                for (int k = 0; k < worker_data[i].intermediate_points.size(); k++)
                {
                    Point3Di p = worker_data[i].intermediate_points[k];
                    int index_pose = p.index_pose;
                    p.point = worker_data[i].intermediate_trajectory[index_pose] * p.point;
                    global_points.push_back(p);
                }
                std::string fn = params.working_directory_preview + "/temp_point_cloud_" + std::to_string(i) + ".laz";
                saveLaz(fn.c_str(), global_points);
            }
            auto acc_distance_tmp = acc_distance;
            acc_distance += ((worker_data[i].intermediate_trajectory[0].inverse()) *
                             worker_data[i].intermediate_trajectory[worker_data[i].intermediate_trajectory.size() - 1])
                                .translation()
                                .norm();

            if (!(acc_distance == acc_distance))
            {
                worker_data[i].intermediate_trajectory = tmp_worker_data;
                std::cout << "CHALLENGING DATA OCCURED!!!" << std::endl;
                acc_distance = acc_distance_tmp;
                std::cout << "please split data set into subsets" << std::endl;
                ts_failure = worker_data[i].intermediate_trajectory_timestamps[0].first;
                // std::cout << "calculations canceled for TIMESTAMP: " << (long long int)worker_data[i].intermediate_trajectory_timestamps[0].first << std::endl;
                return false;
            }

            // update

            for (int j = i + 1; j < worker_data.size(); j++)
            {
                Eigen::Affine3d m_last = worker_data[j - 1].intermediate_trajectory[worker_data[j - 1].intermediate_trajectory.size() - 1];
                auto tmp = worker_data[j].intermediate_trajectory;

                worker_data[j].intermediate_trajectory[0] = m_last;

                for (int k = 1; k < tmp.size(); k++)
                {
                    Eigen::Affine3d m_update = tmp[k - 1].inverse() * tmp[k];
                    m_last = m_last * m_update;
                    worker_data[j].intermediate_trajectory[k] = m_last;
                }
            }

            for (int j = 0; j < worker_data[i].intermediate_points.size(); j++)
            {
                Point3Di pp = worker_data[i].intermediate_points[j];
                pp.point = worker_data[i].intermediate_trajectory[worker_data[i].intermediate_points[j].index_pose] * pp.point;
                points_global.push_back(pp);
            }

            // if(reference_points.size() == 0){
            if (acc_distance > params.sliding_window_trajectory_length_threshold)
            {
                std::chrono::time_point<std::chrono::system_clock> startu, endu;
                startu = std::chrono::system_clock::now();

                if (params.reference_points.size() == 0)
                {
                    params.buckets.clear();
                }

                std::vector<Point3Di> points_global_new;
                points_global_new.reserve(points_global.size() / 2 + 1);
                for (int k = points_global.size() / 2; k < points_global.size(); k++)
                {
                    points_global_new.emplace_back(points_global[k]);
                }

                acc_distance = 0;
                points_global = points_global_new;

                // decimate
                if (params.decimation > 0)
                {
                    decimate(points_global, params.decimation, params.decimation, params.decimation);
                }
                update_rgd(params.in_out_params, params.buckets, points_global, worker_data[i].intermediate_trajectory[0].translation());
                //
                endu = std::chrono::system_clock::now();

                std::chrono::duration<double> elapsed_secondsu = endu - startu;
                std::time_t end_timeu = std::chrono::system_clock::to_time_t(endu);

                std::cout << "finished computation at " << std::ctime(&end_timeu)
                          << "elapsed time update: " << elapsed_secondsu.count() << "s\n";
            }
            else
            {
                std::vector<Point3Di> pg;
                for (int j = 0; j < worker_data[i].intermediate_points.size(); j++)
                {
                    Point3Di pp = worker_data[i].intermediate_points[j];
                    pp.point = worker_data[i].intermediate_trajectory[worker_data[i].intermediate_points[j].index_pose] * pp.point;
                    pg.push_back(pp);
                }
                update_rgd(params.in_out_params, params.buckets, pg, worker_data[i].intermediate_trajectory[0].translation());
            }

            if (i > 1)
            {
                double translation = (worker_data[i - 1].intermediate_trajectory[0].translation() -
                                      worker_data[i - 2].intermediate_trajectory[0].translation())
                                         .norm();
                params.consecutive_distance += translation;
            }
        }

        for (int i = 0; i < worker_data.size(); i++)
        {
            worker_data[i].intermediate_trajectory_motion_model = worker_data[i].intermediate_trajectory;
        }

        end = std::chrono::system_clock::now();

        std::chrono::duration<double> elapsed_seconds = end - start;
        std::time_t end_time = std::chrono::system_clock::to_time_t(end);

        std::cout << "finished computation at " << std::ctime(&end_time)
                  << "elapsed time: " << elapsed_seconds.count() << "s\n";

        // estimate total lenght of trajectory
        double length_of_trajectory = 0;
        for (int i = 1; i < worker_data.size(); i++)
        {
            length_of_trajectory += (worker_data[i].intermediate_trajectory[0].translation() - worker_data[i - 1].intermediate_trajectory[0].translation()).norm();
        }
        std::cout << "length_of_trajectory: " << length_of_trajectory << " [m]" << std::endl;
    }

    return true;
}

void compute_step_2_fast_forward_motion(std::vector<WorkerData> &worker_data, LidarOdometryParams &params)
{
}

#if 0
void Consistency(std::vector<WorkerData> &worker_data, LidarOdometryParams &params)
{
    std::vector<Eigen::Affine3d> trajectory;
    std::vector<Eigen::Affine3d> trajectory_motion_model;
    std::vector<std::pair<double, double>> timestamps;
    std::vector<Point3Di> all_points_local;

    std::vector<std::pair<int, int>> indexes;
    bool multithread = true;

    std::cout << "preparing data START" << std::endl;
    for (int i = 0; i < worker_data.size(); i++)
    {
        for (int j = 0; j < worker_data[i].intermediate_trajectory.size(); j++)
        {
            trajectory.push_back(worker_data[i].intermediate_trajectory[j]);
            trajectory_motion_model.push_back(worker_data[i].intermediate_trajectory_motion_model[j]);
            timestamps.push_back(worker_data[i].intermediate_trajectory_timestamps[j]);
            indexes.emplace_back(i, j);
        }
    }
    std::cout << "preparing data FINISHED" << std::endl;

    for (int i = 0; i < worker_data.size(); i++)
    {
        for (int j = 0; j < worker_data[i].intermediate_points.size(); j++)
        {
            all_points_local.push_back(worker_data[i].intermediate_points[j]);
        }
    }

    std::vector<Eigen::Triplet<double>> tripletListA;
    std::vector<Eigen::Triplet<double>> tripletListP;
    std::vector<Eigen::Triplet<double>> tripletListB;

    Eigen::MatrixXd AtPAndt(trajectory.size() * 6, trajectory.size() * 6);
    AtPAndt.setZero();
    Eigen::MatrixXd AtPBndt(trajectory.size() * 6, 1);
    AtPBndt.setZero();
    Eigen::Vector3d b(params.in_out_params.resolution_X, params.in_out_params.resolution_Y, params.in_out_params.resolution_Z);

    std::vector<std::mutex> mutexes(trajectory.size());

    std::cout << "NDT observations START" << std::endl;
    for (size_t index_point_begin = 0; index_point_begin < all_points_local.size(); index_point_begin += 2000000)
    {
        NDTBucketMapType buckets;
        std::vector<Point3Di> points_local;
        std::vector<Point3Di> points_global;

        for (int index = index_point_begin; index < index_point_begin + 2000000; index++)
        {
            if (index < all_points_local.size())
            {
                auto lower = std::lower_bound(timestamps.begin(), timestamps.end(), all_points_local[index].timestamp,
                                              [](std::pair<double, double> lhs, double rhs) -> bool
                                              { return lhs.first < rhs; });
                int index_pose = std::distance(timestamps.begin(), lower);
                if (index_pose >= 0 && index_pose < trajectory.size())
                {
                    auto pl = all_points_local[index];
                    pl.index_pose = index_pose;

                    points_local.push_back(pl);
                    pl.point = trajectory[pl.index_pose] * pl.point;
                    points_global.push_back(pl);
                }
            }
        }

        if (points_global.size() > 10000)
        {
            update_rgd(params.in_out_params, buckets, points_global, trajectory[points_global[0].index_pose].translation());
            const auto hessian_fun = [&](const Point3Di &intermediate_points_i)
            {
                if (intermediate_points_i.point.norm() < 1.0)
                {
                    return;
                }

                Eigen::Vector3d point_global = trajectory[intermediate_points_i.index_pose] * intermediate_points_i.point;
                auto index_of_bucket = get_rgd_index(point_global, b);

                auto bucket_it = buckets.find(index_of_bucket);
                // no bucket found
                if (bucket_it == buckets.end())
                {
                    return;
                }
                auto &this_bucket = bucket_it->second;

                const Eigen::Matrix3d &infm = this_bucket.cov.inverse();
                const double threshold = 10000.0;

                if ((infm.array() > threshold).any())
                {
                    return;
                }
                if ((infm.array() < -threshold).any())
                {
                    return;
                }

                // check nv
                Eigen::Vector3d &nv = this_bucket.normal_vector;
                Eigen::Vector3d viewport = trajectory[intermediate_points_i.index_pose].translation();
                if (nv.dot(viewport - this_bucket.mean) < 0)
                {
                    return;
                }

                const Eigen::Affine3d &m_pose = trajectory[intermediate_points_i.index_pose];
                const Eigen::Vector3d &p_s = intermediate_points_i.point;
                const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose);
                //

                Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AtPA;
                point_to_point_source_to_target_tait_bryan_wc_AtPA_simplified(
                    AtPA,
                    pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                    p_s.x(), p_s.y(), p_s.z(),
                    infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2));

                Eigen::Matrix<double, 6, 1> AtPB;
                point_to_point_source_to_target_tait_bryan_wc_AtPB_simplified(
                    AtPB,
                    pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                    p_s.x(), p_s.y(), p_s.z(),
                    infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2),
                    this_bucket.mean.x(), this_bucket.mean.y(), this_bucket.mean.z());

                int c = intermediate_points_i.index_pose * 6;

                std::mutex &m = mutexes[intermediate_points_i.index_pose];
                std::unique_lock lck(m);
                AtPAndt.block<6, 6>(c, c) += AtPA;
                AtPBndt.block<6, 1>(c, 0) -= AtPB;
            };

            if (multithread)
            {
                std::for_each(std::execution::par_unseq, std::begin(points_local), std::end(points_local), hessian_fun);
            }
            else
            {
                std::for_each(std::begin(points_local), std::end(points_local), hessian_fun);
            }
        }
    }
    std::cout << "NDT observations FINISHED" << std::endl;

    std::vector<std::pair<int, int>> odo_edges;
    for (size_t i = 1; i < trajectory.size(); i++)
    {
        odo_edges.emplace_back(i - 1, i);
    }

    std::vector<TaitBryanPose> poses;
    std::vector<TaitBryanPose> poses_desired;

    for (size_t i = 0; i < trajectory.size(); i++)
    {
        poses.push_back(pose_tait_bryan_from_affine_matrix(trajectory[i]));
    }
    for (size_t i = 0; i < trajectory_motion_model.size(); i++)
    {
        poses_desired.push_back(pose_tait_bryan_from_affine_matrix(trajectory_motion_model[i]));
    }

    double angle = 1.0 / 180.0 * M_PI;
    double wangle = 1.0 / (angle * angle);

    for (size_t i = 0; i < odo_edges.size(); i++)
    {
        Eigen::Matrix<double, 6, 1> relative_pose_measurement_odo;
        relative_pose_tait_bryan_wc_case1_simplified_1(relative_pose_measurement_odo,
                                                       poses_desired[odo_edges[i].first].px,
                                                       poses_desired[odo_edges[i].first].py,
                                                       poses_desired[odo_edges[i].first].pz,
                                                       poses_desired[odo_edges[i].first].om,
                                                       poses_desired[odo_edges[i].first].fi,
                                                       poses_desired[odo_edges[i].first].ka,
                                                       poses_desired[odo_edges[i].second].px,
                                                       poses_desired[odo_edges[i].second].py,
                                                       poses_desired[odo_edges[i].second].pz,
                                                       poses_desired[odo_edges[i].second].om,
                                                       poses_desired[odo_edges[i].second].fi,
                                                       poses_desired[odo_edges[i].second].ka);

        Eigen::Matrix<double, 12, 12> AtPAodo;
        relative_pose_obs_eq_tait_bryan_wc_case1_AtPA_simplified(AtPAodo,
                                                                 poses[odo_edges[i].first].px,
                                                                 poses[odo_edges[i].first].py,
                                                                 poses[odo_edges[i].first].pz,
                                                                 poses[odo_edges[i].first].om,
                                                                 poses[odo_edges[i].first].fi,
                                                                 poses[odo_edges[i].first].ka,
                                                                 poses[odo_edges[i].second].px,
                                                                 poses[odo_edges[i].second].py,
                                                                 poses[odo_edges[i].second].pz,
                                                                 poses[odo_edges[i].second].om,
                                                                 poses[odo_edges[i].second].fi,
                                                                 poses[odo_edges[i].second].ka,
                                                                 10000,
                                                                 10000,
                                                                 10000,
                                                                 wangle,
                                                                 wangle,
                                                                 wangle);
        Eigen::Matrix<double, 12, 1> AtPBodo;
        relative_pose_obs_eq_tait_bryan_wc_case1_AtPB_simplified(AtPBodo,
                                                                 poses[odo_edges[i].first].px,
                                                                 poses[odo_edges[i].first].py,
                                                                 poses[odo_edges[i].first].pz,
                                                                 poses[odo_edges[i].first].om,
                                                                 poses[odo_edges[i].first].fi,
                                                                 poses[odo_edges[i].first].ka,
                                                                 poses[odo_edges[i].second].px,
                                                                 poses[odo_edges[i].second].py,
                                                                 poses[odo_edges[i].second].pz,
                                                                 poses[odo_edges[i].second].om,
                                                                 poses[odo_edges[i].second].fi,
                                                                 poses[odo_edges[i].second].ka,
                                                                 relative_pose_measurement_odo(0, 0),
                                                                 relative_pose_measurement_odo(1, 0),
                                                                 relative_pose_measurement_odo(2, 0),
                                                                 relative_pose_measurement_odo(3, 0),
                                                                 relative_pose_measurement_odo(4, 0),
                                                                 relative_pose_measurement_odo(5, 0),
                                                                 10000,
                                                                 10000,
                                                                 10000,
                                                                 wangle,
                                                                 wangle,
                                                                 wangle);
        int ic_1 = odo_edges[i].first * 6;
        int ic_2 = odo_edges[i].second * 6;

        for (int row = 0; row < 6; row++)
        {
            for (int col = 0; col < 6; col++)
            {
                AtPAndt(ic_1 + row, ic_1 + col) += AtPAodo(row, col);
                AtPAndt(ic_1 + row, ic_2 + col) += AtPAodo(row, col + 6);
                AtPAndt(ic_2 + row, ic_1 + col) += AtPAodo(row + 6, col);
                AtPAndt(ic_2 + row, ic_2 + col) += AtPAodo(row + 6, col + 6);
            }
        }

        for (int row = 0; row < 6; row++)
        {
            AtPBndt(ic_1 + row, 0) -= AtPBodo(row, 0);
            AtPBndt(ic_2 + row, 0) -= AtPBodo(row + 6, 0);
        }
    }

    // smoothness

    for (size_t i = 1; i < poses.size() - 1; i++)
    {
        Eigen::Matrix<double, 6, 1> delta;
        smoothness_obs_eq_tait_bryan_wc(delta,
                                        poses[i - 1].px,
                                        poses[i - 1].py,
                                        poses[i - 1].pz,
                                        poses[i - 1].om,
                                        poses[i - 1].fi,
                                        poses[i - 1].ka,
                                        poses[i].px,
                                        poses[i].py,
                                        poses[i].pz,
                                        poses[i].om,
                                        poses[i].fi,
                                        poses[i].ka,
                                        poses[i + 1].px,
                                        poses[i + 1].py,
                                        poses[i + 1].pz,
                                        poses[i + 1].om,
                                        poses[i + 1].fi,
                                        poses[i + 1].ka);

        Eigen::Matrix<double, 6, 18, Eigen::RowMajor> jacobian;
        smoothness_obs_eq_tait_bryan_wc_jacobian(jacobian,
                                                 poses[i - 1].px,
                                                 poses[i - 1].py,
                                                 poses[i - 1].pz,
                                                 poses[i - 1].om,
                                                 poses[i - 1].fi,
                                                 poses[i - 1].ka,
                                                 poses[i].px,
                                                 poses[i].py,
                                                 poses[i].pz,
                                                 poses[i].om,
                                                 poses[i].fi,
                                                 poses[i].ka,
                                                 poses[i + 1].px,
                                                 poses[i + 1].py,
                                                 poses[i + 1].pz,
                                                 poses[i + 1].om,
                                                 poses[i + 1].fi,
                                                 poses[i + 1].ka);

        int ir = tripletListB.size();

        int ic_1 = (i - 1) * 6;
        int ic_2 = i * 6;
        int ic_3 = (i + 1) * 6;

        for (size_t row = 0; row < 6; row++)
        {
            tripletListA.emplace_back(ir + row, ic_1, -jacobian(row, 0));
            tripletListA.emplace_back(ir + row, ic_1 + 1, -jacobian(row, 1));
            tripletListA.emplace_back(ir + row, ic_1 + 2, -jacobian(row, 2));
            tripletListA.emplace_back(ir + row, ic_1 + 3, -jacobian(row, 3));
            tripletListA.emplace_back(ir + row, ic_1 + 4, -jacobian(row, 4));
            tripletListA.emplace_back(ir + row, ic_1 + 5, -jacobian(row, 5));

            tripletListA.emplace_back(ir + row, ic_2, -jacobian(row, 6));
            tripletListA.emplace_back(ir + row, ic_2 + 1, -jacobian(row, 7));
            tripletListA.emplace_back(ir + row, ic_2 + 2, -jacobian(row, 8));
            tripletListA.emplace_back(ir + row, ic_2 + 3, -jacobian(row, 9));
            tripletListA.emplace_back(ir + row, ic_2 + 4, -jacobian(row, 10));
            tripletListA.emplace_back(ir + row, ic_2 + 5, -jacobian(row, 11));

            tripletListA.emplace_back(ir + row, ic_3, -jacobian(row, 12));
            tripletListA.emplace_back(ir + row, ic_3 + 1, -jacobian(row, 13));
            tripletListA.emplace_back(ir + row, ic_3 + 2, -jacobian(row, 14));
            tripletListA.emplace_back(ir + row, ic_3 + 3, -jacobian(row, 15));
            tripletListA.emplace_back(ir + row, ic_3 + 4, -jacobian(row, 16));
            tripletListA.emplace_back(ir + row, ic_3 + 5, -jacobian(row, 17));
        }
        tripletListB.emplace_back(ir, 0, delta(0, 0));
        tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
        tripletListB.emplace_back(ir + 2, 0, delta(2, 0));
        tripletListB.emplace_back(ir + 3, 0, delta(3, 0));
        tripletListB.emplace_back(ir + 4, 0, delta(4, 0));
        tripletListB.emplace_back(ir + 5, 0, delta(5, 0));

        tripletListP.emplace_back(ir, ir, 1000000);
        tripletListP.emplace_back(ir + 1, ir + 1, 1000000);
        tripletListP.emplace_back(ir + 2, ir + 2, 1000000);
        tripletListP.emplace_back(ir + 3, ir + 3, wangle);
        tripletListP.emplace_back(ir + 4, ir + 4, wangle);
        tripletListP.emplace_back(ir + 5, ir + 5, wangle);
    }

    Eigen::SparseMatrix<double> matA(tripletListB.size(), trajectory.size() * 6);
    Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
    Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

    matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
    matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
    matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

    Eigen::SparseMatrix<double> AtPA(trajectory.size() * 6, trajectory.size() * 6);
    Eigen::SparseMatrix<double> AtPB(trajectory.size() * 6, 1);

    {
        Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
        AtPA = (AtP)*matA;
        AtPB = (AtP)*matB;
    }

    tripletListA.clear();
    tripletListP.clear();
    tripletListB.clear();

    AtPA += AtPAndt.sparseView();
    AtPB += AtPBndt.sparseView();
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

    if (h_x.size() == 6 * trajectory.size())
    {
        int counter = 0;

        for (size_t i = 0; i < trajectory.size(); i++)
        {
            TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(trajectory[i]);
            pose.px += h_x[counter++];
            pose.py += h_x[counter++];
            pose.pz += h_x[counter++];
            pose.om += h_x[counter++];
            pose.fi += h_x[counter++];
            pose.ka += h_x[counter++];
            trajectory[i] = affine_matrix_from_pose_tait_bryan(pose);
        }
    }

    for (int i = 0; i < indexes.size(); i++)
    {
        worker_data[indexes[i].first].intermediate_trajectory[indexes[i].second] = trajectory[i];
        // worker_data[indexes[i].first].intermediate_trajectory_motion_model[indexes[i].second] = trajectory[i];
    }
    //
}
#endif

void Consistency(std::vector<WorkerData> &worker_data, LidarOdometryParams &params)
{
    std::vector<Eigen::Affine3d> trajectory;
    std::vector<Eigen::Affine3d> trajectory_motion_model;
    std::vector<std::pair<double, double>> timestamps;
    std::vector<Point3Di> all_points_local;

    std::vector<std::pair<int, int>> indexes;
    bool multithread = true;

    std::cout << "preparing data START" << std::endl;
    for (int i = 0; i < worker_data.size(); i++)
    {
        for (int j = 0; j < worker_data[i].intermediate_trajectory.size(); j++)
        {
            trajectory.push_back(worker_data[i].intermediate_trajectory[j]);
            trajectory_motion_model.push_back(worker_data[i].intermediate_trajectory_motion_model[j]);
            timestamps.push_back(worker_data[i].intermediate_trajectory_timestamps[j]);
            indexes.emplace_back(i, j);
        }
    }
    std::cout << "preparing data FINISHED" << std::endl;

    for (int i = 0; i < worker_data.size(); i++)
    {
        for (int j = 0; j < worker_data[i].intermediate_points.size(); j++)
        {
            all_points_local.push_back(worker_data[i].intermediate_points[j]);
        }
    }

    std::vector<Eigen::Triplet<double>> tripletListA;
    std::vector<Eigen::Triplet<double>> tripletListP;
    std::vector<Eigen::Triplet<double>> tripletListB;

    // Eigen::MatrixXd AtPAndt(trajectory.size() * 6, trajectory.size() * 6);
    // AtPAndt.setZero();
    // Eigen::MatrixXd AtPBndt(trajectory.size() * 6, 1);
    // AtPBndt.setZero();
    Eigen::Vector3d b(params.in_out_params.resolution_X, params.in_out_params.resolution_Y, params.in_out_params.resolution_Z);
    // std::vector<std::mutex> mutexes(trajectory.size());

    std::vector<std::mutex> my_mutex(1);

    std::vector<std::pair<int, int>> odo_edges;
    for (size_t i = 1; i < trajectory.size(); i++)
    {
        odo_edges.emplace_back(i - 1, i);
    }

    std::vector<TaitBryanPose> poses;
    std::vector<TaitBryanPose> poses_desired;

    for (size_t i = 0; i < trajectory.size(); i++)
    {
        poses.push_back(pose_tait_bryan_from_affine_matrix(trajectory[i]));
    }
    for (size_t i = 0; i < trajectory_motion_model.size(); i++)
    {
        poses_desired.push_back(pose_tait_bryan_from_affine_matrix(trajectory_motion_model[i]));
    }

    double angle = 0.01 / 180.0 * M_PI;
    double wangle = 1.0 / (angle * angle);

    /*for (size_t i = 0; i < odo_edges.size(); i++)
    {
        Eigen::Matrix<double, 6, 1> relative_pose_measurement_odo;
        relative_pose_tait_bryan_wc_case1_simplified_1(relative_pose_measurement_odo,
                                                       poses_desired[odo_edges[i].first].px,
                                                       poses_desired[odo_edges[i].first].py,
                                                       poses_desired[odo_edges[i].first].pz,
                                                       poses_desired[odo_edges[i].first].om,
                                                       poses_desired[odo_edges[i].first].fi,
                                                       poses_desired[odo_edges[i].first].ka,
                                                       poses_desired[odo_edges[i].second].px,
                                                       poses_desired[odo_edges[i].second].py,
                                                       poses_desired[odo_edges[i].second].pz,
                                                       poses_desired[odo_edges[i].second].om,
                                                       poses_desired[odo_edges[i].second].fi,
                                                       poses_desired[odo_edges[i].second].ka);

        Eigen::Matrix<double, 12, 12> AtPAodo;
        relative_pose_obs_eq_tait_bryan_wc_case1_AtPA_simplified(AtPAodo,
                                                                 poses[odo_edges[i].first].px,
                                                                 poses[odo_edges[i].first].py,
                                                                 poses[odo_edges[i].first].pz,
                                                                 poses[odo_edges[i].first].om,
                                                                 poses[odo_edges[i].first].fi,
                                                                 poses[odo_edges[i].first].ka,
                                                                 poses[odo_edges[i].second].px,
                                                                 poses[odo_edges[i].second].py,
                                                                 poses[odo_edges[i].second].pz,
                                                                 poses[odo_edges[i].second].om,
                                                                 poses[odo_edges[i].second].fi,
                                                                 poses[odo_edges[i].second].ka,
                                                                 10000,
                                                                 10000,
                                                                 10000,
                                                                 wangle,
                                                                 wangle,
                                                                 wangle);
        Eigen::Matrix<double, 12, 1> AtPBodo;
        relative_pose_obs_eq_tait_bryan_wc_case1_AtPB_simplified(AtPBodo,
                                                                 poses[odo_edges[i].first].px,
                                                                 poses[odo_edges[i].first].py,
                                                                 poses[odo_edges[i].first].pz,
                                                                 poses[odo_edges[i].first].om,
                                                                 poses[odo_edges[i].first].fi,
                                                                 poses[odo_edges[i].first].ka,
                                                                 poses[odo_edges[i].second].px,
                                                                 poses[odo_edges[i].second].py,
                                                                 poses[odo_edges[i].second].pz,
                                                                 poses[odo_edges[i].second].om,
                                                                 poses[odo_edges[i].second].fi,
                                                                 poses[odo_edges[i].second].ka,
                                                                 relative_pose_measurement_odo(0, 0),
                                                                 relative_pose_measurement_odo(1, 0),
                                                                 relative_pose_measurement_odo(2, 0),
                                                                 relative_pose_measurement_odo(3, 0),
                                                                 relative_pose_measurement_odo(4, 0),
                                                                 relative_pose_measurement_odo(5, 0),
                                                                 10000,
                                                                 10000,
                                                                 10000,
                                                                 wangle,
                                                                 wangle,
                                                                 wangle);
        int ic_1 = odo_edges[i].first * 6;
        int ic_2 = odo_edges[i].second * 6;

        for (int row = 0; row < 6; row++)
        {
            for (int col = 0; col < 6; col++)
            {
                AtPAndt(ic_1 + row, ic_1 + col) += AtPAodo(row, col);
                AtPAndt(ic_1 + row, ic_2 + col) += AtPAodo(row, col + 6);
                AtPAndt(ic_2 + row, ic_1 + col) += AtPAodo(row + 6, col);
                AtPAndt(ic_2 + row, ic_2 + col) += AtPAodo(row + 6, col + 6);
            }
        }

        for (int row = 0; row < 6; row++)
        {
            AtPBndt(ic_1 + row, 0) -= AtPBodo(row, 0);
            AtPBndt(ic_2 + row, 0) -= AtPBodo(row + 6, 0);
        }
    }*/

    // smoothness

    for (size_t i = 1; i < poses.size() - 1; i++)
    {
        Eigen::Matrix<double, 6, 1> delta;
        smoothness_obs_eq_tait_bryan_wc(delta,
                                        poses[i - 1].px,
                                        poses[i - 1].py,
                                        poses[i - 1].pz,
                                        poses[i - 1].om,
                                        poses[i - 1].fi,
                                        poses[i - 1].ka,
                                        poses[i].px,
                                        poses[i].py,
                                        poses[i].pz,
                                        poses[i].om,
                                        poses[i].fi,
                                        poses[i].ka,
                                        poses[i + 1].px,
                                        poses[i + 1].py,
                                        poses[i + 1].pz,
                                        poses[i + 1].om,
                                        poses[i + 1].fi,
                                        poses[i + 1].ka);

        Eigen::Matrix<double, 6, 18, Eigen::RowMajor> jacobian;
        smoothness_obs_eq_tait_bryan_wc_jacobian(jacobian,
                                                 poses[i - 1].px,
                                                 poses[i - 1].py,
                                                 poses[i - 1].pz,
                                                 poses[i - 1].om,
                                                 poses[i - 1].fi,
                                                 poses[i - 1].ka,
                                                 poses[i].px,
                                                 poses[i].py,
                                                 poses[i].pz,
                                                 poses[i].om,
                                                 poses[i].fi,
                                                 poses[i].ka,
                                                 poses[i + 1].px,
                                                 poses[i + 1].py,
                                                 poses[i + 1].pz,
                                                 poses[i + 1].om,
                                                 poses[i + 1].fi,
                                                 poses[i + 1].ka);

        int ir = tripletListB.size();

        int ic_1 = (i - 1) * 6;
        int ic_2 = i * 6;
        int ic_3 = (i + 1) * 6;

        for (size_t row = 0; row < 6; row++)
        {
            tripletListA.emplace_back(ir + row, ic_1, -jacobian(row, 0));
            tripletListA.emplace_back(ir + row, ic_1 + 1, -jacobian(row, 1));
            tripletListA.emplace_back(ir + row, ic_1 + 2, -jacobian(row, 2));
            tripletListA.emplace_back(ir + row, ic_1 + 3, -jacobian(row, 3));
            tripletListA.emplace_back(ir + row, ic_1 + 4, -jacobian(row, 4));
            tripletListA.emplace_back(ir + row, ic_1 + 5, -jacobian(row, 5));

            tripletListA.emplace_back(ir + row, ic_2, -jacobian(row, 6));
            tripletListA.emplace_back(ir + row, ic_2 + 1, -jacobian(row, 7));
            tripletListA.emplace_back(ir + row, ic_2 + 2, -jacobian(row, 8));
            tripletListA.emplace_back(ir + row, ic_2 + 3, -jacobian(row, 9));
            tripletListA.emplace_back(ir + row, ic_2 + 4, -jacobian(row, 10));
            tripletListA.emplace_back(ir + row, ic_2 + 5, -jacobian(row, 11));

            tripletListA.emplace_back(ir + row, ic_3, -jacobian(row, 12));
            tripletListA.emplace_back(ir + row, ic_3 + 1, -jacobian(row, 13));
            tripletListA.emplace_back(ir + row, ic_3 + 2, -jacobian(row, 14));
            tripletListA.emplace_back(ir + row, ic_3 + 3, -jacobian(row, 15));
            tripletListA.emplace_back(ir + row, ic_3 + 4, -jacobian(row, 16));
            tripletListA.emplace_back(ir + row, ic_3 + 5, -jacobian(row, 17));
        }
        tripletListB.emplace_back(ir, 0, delta(0, 0));
        tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
        tripletListB.emplace_back(ir + 2, 0, delta(2, 0));
        tripletListB.emplace_back(ir + 3, 0, delta(3, 0));
        tripletListB.emplace_back(ir + 4, 0, delta(4, 0));
        tripletListB.emplace_back(ir + 5, 0, delta(5, 0));

        tripletListP.emplace_back(ir, ir, 100000000);
        tripletListP.emplace_back(ir + 1, ir + 1, 100000000);
        tripletListP.emplace_back(ir + 2, ir + 2, 100000000);
        tripletListP.emplace_back(ir + 3, ir + 3, wangle);
        tripletListP.emplace_back(ir + 4, ir + 4, wangle);
        tripletListP.emplace_back(ir + 5, ir + 5, wangle);
    }

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

    Eigen::SparseMatrix<double> matA(tripletListB.size(), trajectory.size() * 6);
    Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
    Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

    matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
    matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
    matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

    Eigen::SparseMatrix<double> AtPA(trajectory.size() * 6, trajectory.size() * 6);
    Eigen::SparseMatrix<double> AtPB(trajectory.size() * 6, 1);

    {
        Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
        AtPA = (AtP)*matA;
        AtPB = (AtP)*matB;
    }

    tripletListA.clear();
    tripletListP.clear();
    tripletListB.clear();

    // ndt

    std::cout << "NDT observations START" << std::endl;
    for (size_t index_point_begin = 0; index_point_begin < all_points_local.size(); index_point_begin += 10000000)
    {
        NDTBucketMapType buckets;
        std::vector<Point3Di> points_local;
        std::vector<Point3Di> points_global;

        for (int index = index_point_begin; index < index_point_begin + 10000000; index++)
        {
            if (index < all_points_local.size())
            {
                auto lower = std::lower_bound(timestamps.begin(), timestamps.end(), all_points_local[index].timestamp,
                                              [](std::pair<double, double> lhs, double rhs) -> bool
                                              { return lhs.first < rhs; });
                int index_pose = std::distance(timestamps.begin(), lower);
                if (index_pose >= 0 && index_pose < trajectory.size())
                {
                    auto pl = all_points_local[index];
                    pl.index_pose = index_pose;

                    points_local.push_back(pl);
                    pl.point = trajectory[pl.index_pose] * pl.point;
                    points_global.push_back(pl);
                }
            }
        }

        if (points_global.size() > 10000)
        {
            update_rgd(params.in_out_params, buckets, points_global, trajectory[points_global[0].index_pose].translation());
            const auto hessian_fun = [&](const Point3Di &intermediate_points_i)
            {
                if (intermediate_points_i.point.norm() < 1.0)
                {
                    return;
                }

                Eigen::Vector3d point_global = trajectory[intermediate_points_i.index_pose] * intermediate_points_i.point;
                auto index_of_bucket = get_rgd_index(point_global, b);

                auto bucket_it = buckets.find(index_of_bucket);
                // no bucket found
                if (bucket_it == buckets.end())
                {
                    return;
                }
                auto &this_bucket = bucket_it->second;

                const Eigen::Matrix3d &infm = this_bucket.cov.inverse();
                const double threshold = 10000.0;

                if ((infm.array() > threshold).any())
                {
                    return;
                }
                if ((infm.array() < -threshold).any())
                {
                    return;
                }

                // check nv
                Eigen::Vector3d &nv = this_bucket.normal_vector;
                Eigen::Vector3d viewport = trajectory[intermediate_points_i.index_pose].translation();
                if (nv.dot(viewport - this_bucket.mean) < 0)
                {
                    return;
                }

                const Eigen::Affine3d &m_pose = trajectory[intermediate_points_i.index_pose];
                const Eigen::Vector3d &p_s = intermediate_points_i.point;
                const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose);
                double delta_x;
                double delta_y;
                double delta_z;

                //

                // Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AtPA;
                // point_to_point_source_to_target_tait_bryan_wc_AtPA_simplified(
                //     AtPA,
                //     pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                //     p_s.x(), p_s.y(), p_s.z(),
                //     infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2));

                // Eigen::Matrix<double, 6, 1> AtPB;
                // point_to_point_source_to_target_tait_bryan_wc_AtPB_simplified(
                //     AtPB,
                //     pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                //     p_s.x(), p_s.y(), p_s.z(),
                //     infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2),
                //     this_bucket.mean.x(), this_bucket.mean.y(), this_bucket.mean.z());

                Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
                // TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix((*mposes)[p.index_pose]);

                point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z,
                                                              pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                              p_s.x(), p_s.y(), p_s.z(), this_bucket.mean.x(), this_bucket.mean.y(), this_bucket.mean.z());

                point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian,
                                                                       pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                                       p_s.x(), p_s.y(), p_s.z());

                int c = intermediate_points_i.index_pose * 6;

                std::mutex &m = my_mutex[0]; // mutexes[intermediate_points_i.index_pose];
                std::unique_lock lck(m);
                int ir = tripletListB.size();

                for (int row = 0; row < 3; row++)
                {
                    for (int col = 0; col < 6; col++)
                    {
                        if (jacobian(row, col) != 0.0)
                        {
                            tripletListA.emplace_back(ir + row, c + col, -jacobian(row, col));
                        }
                    }
                }
                tripletListB.emplace_back(ir, 0, delta_x);
                tripletListB.emplace_back(ir + 1, 0, delta_y);
                tripletListB.emplace_back(ir + 2, 0, delta_z);

                tripletListP.emplace_back(ir, ir, infm(0, 0));
                tripletListP.emplace_back(ir, ir + 1, infm(0, 1));
                tripletListP.emplace_back(ir, ir + 2, infm(0, 2));
                tripletListP.emplace_back(ir + 1, ir, infm(1, 0));
                tripletListP.emplace_back(ir + 1, ir + 1, infm(1, 1));
                tripletListP.emplace_back(ir + 1, ir + 2, infm(1, 2));
                tripletListP.emplace_back(ir + 2, ir, infm(2, 0));
                tripletListP.emplace_back(ir + 2, ir + 1, infm(2, 1));
                tripletListP.emplace_back(ir + 2, ir + 2, infm(2, 2));
            };

            if (multithread)
            {
                std::for_each(std::execution::par_unseq, std::begin(points_local), std::end(points_local), hessian_fun);
            }
            else
            {
                std::for_each(std::begin(points_local), std::end(points_local), hessian_fun);
            }

            Eigen::SparseMatrix<double> matAndt(tripletListB.size(), trajectory.size() * 6);
            Eigen::SparseMatrix<double> matPndt(tripletListB.size(), tripletListB.size());
            Eigen::SparseMatrix<double> matBndt(tripletListB.size(), 1);

            matAndt.setFromTriplets(tripletListA.begin(), tripletListA.end());
            matPndt.setFromTriplets(tripletListP.begin(), tripletListP.end());
            matBndt.setFromTriplets(tripletListB.begin(), tripletListB.end());

            Eigen::SparseMatrix<double> AtPAndt(trajectory.size() * 6, trajectory.size() * 6);
            Eigen::SparseMatrix<double> AtPBndt(trajectory.size() * 6, 1);

            {
                Eigen::SparseMatrix<double> AtPndt = matAndt.transpose() * matPndt;
                AtPAndt = (AtPndt)*matAndt;
                AtPBndt = (AtPndt)*matBndt;
            }

            tripletListA.clear();
            tripletListP.clear();
            tripletListB.clear();
            AtPA += AtPAndt;
            AtPB += AtPBndt;
        }
    }
    std::cout << "NDT observations FINISHED" << std::endl;

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

    if (h_x.size() == 6 * trajectory.size())
    {
        int counter = 0;

        for (size_t i = 0; i < trajectory.size(); i++)
        {
            TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(trajectory[i]);
            pose.px += h_x[counter++];
            pose.py += h_x[counter++];
            pose.pz += h_x[counter++];
            pose.om += h_x[counter++];
            pose.fi += h_x[counter++];
            pose.ka += h_x[counter++];
            trajectory[i] = affine_matrix_from_pose_tait_bryan(pose);
        }
    }

    for (int i = 0; i < indexes.size(); i++)
    {
        worker_data[indexes[i].first].intermediate_trajectory[indexes[i].second] = trajectory[i];
    }
}