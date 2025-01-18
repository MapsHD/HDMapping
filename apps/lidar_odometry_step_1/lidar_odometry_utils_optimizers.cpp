#include "lidar_odometry_utils.h"
#include <hash_utils.h>
#include <mutex>

// extern std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> global_tmp;

std::vector<std::pair<int, int>> nns(std::vector<Point3Di> points_global, const std::vector<int> &indexes_for_nn)
{
    Eigen::Vector3d search_radious = {0.1, 0.1, 0.1};

    std::vector<std::pair<int, int>> nn;

    std::vector<std::pair<unsigned long long int, unsigned int>> indexes;

    for (int i = 0; i < points_global.size(); i++)
    {
        unsigned long long int index = get_rgd_index(points_global[i].point, search_radious);
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

    for (size_t i = 0; i < indexes_for_nn.size(); i++)
    {
        int index_element_source = indexes_for_nn[i];
        auto source = points_global[index_element_source];

        std::vector<double> min_distances;
        std::vector<int> indexes_target;
        for (int j = 0; j < 30; j++)
        {
            min_distances.push_back(1000.0);
            indexes_target.push_back(-1);
        }

        // if (source.index_point >= 0)
        //{
        for (double x = -search_radious.x(); x <= search_radious.x(); x += search_radious.x())
        {
            for (double y = -search_radious.y(); y <= search_radious.y(); y += search_radious.y())
            {
                for (double z = -search_radious.z(); z <= search_radious.z(); z += search_radious.z())
                {
                    Eigen::Vector3d position_global = source.point + Eigen::Vector3d(x, y, z);
                    unsigned long long int index_of_bucket = get_rgd_index(position_global, search_radious);

                    if (buckets.contains(index_of_bucket))
                    {
                        for (int index = buckets[index_of_bucket].first; index < buckets[index_of_bucket].second; index++)
                        {
                            int index_element_target = indexes[index].second;
                            auto target = points_global[index_element_target];

                            if (source.index_point != target.index_point)
                            {
                                double dist = (target.point - source.point).norm();
                                if (dist < search_radious.norm())
                                {
                                    if (dist < min_distances[target.index_pose + 1])
                                    {
                                        min_distances[target.index_pose + 1] = dist;
                                        indexes_target[target.index_pose + 1] = index_element_target;
                                    }
                                }
                            }

                            /*if ((source - target).norm() < params.radious)
                            {
                                mean += target;
                                number_of_points_nn++;
                                batch_of_points.push_back(target);
                            }*/
                        }
                    }
                }
            }
        }

        // std::cout << "------------" << std::endl;
        for (size_t y = 0; y < min_distances.size(); y++)
        {
            // std::cout << min_distances[y] << " " << indexes_target[y] << " " << index_element_source << std::endl;

            if (indexes_target[y] != -1)
            {
                nn.emplace_back(index_element_source, indexes_target[y]);
            }
        }
    }
    //}

    return nn;
}

void optimize_icp(std::vector<Point3Di> &intermediate_points, std::vector<Eigen::Affine3d> &intermediate_trajectory,
                  std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model,
                  NDT::GridParameters &rgd_params, /*NDTBucketMapType &buckets*/ std::vector<Point3Di> points_global, bool useMultithread /*,
                   bool add_pitch_roll_constraint, const std::vector<std::pair<double, double>> &imu_roll_pitch*/
)
{

    std::vector<Point3Di> all_points_global = points_global;

    for (int i = 0; i < all_points_global.size(); i++)
    {
        all_points_global[i].index_point = i;
        all_points_global[i].index_pose = -1;
    }

    int size = all_points_global.size();

    std::vector<int> indexes_for_nn;

    for (int i = 0; i < intermediate_points.size(); i++)
    {
        Point3Di p = intermediate_points[i];
        p.point = intermediate_trajectory[p.index_pose] * p.point;
        p.index_point = i + size;
        all_points_global.push_back(p);

        indexes_for_nn.push_back(p.index_point);
        // p.
    }

    // all_points_global_target[i].index_pose is never 0!!!! ToDo check it why

    std::vector<std::pair<int, int>> nn = nns(all_points_global, indexes_for_nn);

    // std::cout << "nn.size(): " << nn.size() << std::endl;
    // return;

    std::vector<Eigen::Triplet<double>> tripletListA;
    std::vector<Eigen::Triplet<double>> tripletListP;
    std::vector<Eigen::Triplet<double>> tripletListB;

    Eigen::MatrixXd AtPAndt(intermediate_trajectory.size() * 6, intermediate_trajectory.size() * 6);
    AtPAndt.setZero();
    Eigen::MatrixXd AtPBndt(intermediate_trajectory.size() * 6, 1);
    AtPBndt.setZero();
    // Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

    Eigen::Matrix3d infm;
    infm.setZero();
    infm(0, 0) = 10000.0;
    infm(1, 1) = 10000.0;
    infm(2, 2) = 10000.0;

    for (int i = 0; i < nn.size(); i++)
    {
        auto intermediate_points_i = all_points_global[nn[i].first];
        const Eigen::Affine3d &m_pose = intermediate_trajectory[intermediate_points_i.index_pose];
        const Eigen::Vector3d &p_s = intermediate_points[intermediate_points_i.index_point - size].point; // intermediate_points_i.point;
        const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose);

        Eigen::Vector3d target = all_points_global[nn[i].second].point;

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
            target.x(), target.y(), target.z());

        int c = intermediate_points_i.index_pose * 6;

        // std::mutex &m = mutexes[intermediate_points_i.index_pose];
        // std::unique_lock lck(m);
        AtPAndt.block<6, 6>(c, c) += AtPA;
        AtPBndt.block<6, 1>(c, 0) -= AtPB;
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

    // maintain angles
    /*if (add_pitch_roll_constraint)
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

            if ((p1 - p2).norm() < 1.0)
            {
                intermediate_trajectory[i] = affine_matrix_from_pose_tait_bryan(pose);
            }
        }
    }
}

void optimize_sf(std::vector<Point3Di> &intermediate_points, std::vector<Eigen::Affine3d> &intermediate_trajectory,
                 std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model,
                 NDT::GridParameters &rgd_params, NDTBucketMapType &buckets_, bool multithread)
{
    // std::cout << "optimize_sf" << std::endl;

    auto int_tr = intermediate_trajectory;
    auto int_tr_tmp = intermediate_trajectory;
    auto int_tr_mm = intermediate_trajectory_motion_model;

    // NDT::GridParameters rgd_params;
    //  rgd_params.resolution_X = 0.3; // distance bucket
    //  rgd_params.resolution_Y = 0.3; // polar angle deg
    //  rgd_params.resolution_Z = 0.3; // azimutal angle deg
    // rgd_params.resolution_X = 0.5; // distance bucket
    // rgd_params.resolution_Y = 5.0; // polar angle deg
    // rgd_params.resolution_Z = 5.0; // azimutal angle deg

    std::vector<Point3Di> point_cloud_global;
    std::vector<Point3Di> points_local;

    std::vector<Eigen::Vector3d> point_cloud_global_sc;
    // std::vector<Point3Di> points_local_sc;

    for (int i = 0; i < intermediate_points.size(); i++)
    {
        double r_l = intermediate_points[i].point.norm();
        if (r_l > 0.5 && intermediate_points[i].index_pose != -1 && r_l < 30)
        {
            double polar_angle_deg_l = atan2(intermediate_points[i].point.y(), intermediate_points[i].point.x()) / M_PI * 180.0;
            double azimutal_angle_deg_l = acos(intermediate_points[i].point.z() / r_l) / M_PI * 180.0;

            Eigen::Vector3d pp = intermediate_points[i].point;
            // pps.x() = r;
            // pps.y() = polar_angle_deg;
            // pps.z() = azimutal_angle_deg;
            // point_cloud_spherical_coordinates.push_back(pps);

            Eigen::Affine3d pose = intermediate_trajectory[intermediate_points[i].index_pose];

            pp = pose * pp;

            Point3Di pg = intermediate_points[i];
            pg.point = pp;

            point_cloud_global.push_back(pg);
            points_local.push_back(intermediate_points[i]);

            ///////////////////////////////////////////////////////
            Point3Di p_sl = intermediate_points[i];
            p_sl.point.x() = r_l;
            p_sl.point.y() = polar_angle_deg_l;
            p_sl.point.z() = azimutal_angle_deg_l;

            // points_local_sc.push_back(p_sl);
            //
            double r_g = pg.point.norm();
            double polar_angle_deg_g = atan2(pg.point.y(), pg.point.x()) / M_PI * 180.0;
            double azimutal_angle_deg_g = acos(pg.point.z() / r_g) / M_PI * 180.0;

            Eigen::Vector3d p_sg = intermediate_points[i].point;
            p_sg.x() = r_g;
            p_sg.y() = polar_angle_deg_g;
            p_sg.z() = azimutal_angle_deg_g;

            point_cloud_global_sc.push_back(p_sg);
        }
    }

    NDTBucketMapType buckets;
    update_rgd_spherical_coordinates(rgd_params, buckets, point_cloud_global, point_cloud_global_sc, {0, 0, 0});
    // update_rgd(rgd_params, buckets, point_cloud_global, {0, 0, 0});
    // std::cout << "buckets.size(): " << buckets.size() << std::endl;

    std::vector<Eigen::Triplet<double>> tripletListA;
    std::vector<Eigen::Triplet<double>> tripletListP;
    std::vector<Eigen::Triplet<double>> tripletListB;
    std::vector<std::mutex> my_mutex(1);

    Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

    const auto hessian_fun = [&](const Point3Di &intermediate_points_i)
    {
        int ir = tripletListB.size();
        double delta_x;
        double delta_y;
        double delta_z;

        Eigen::Affine3d m_pose = intermediate_trajectory[intermediate_points_i.index_pose];
        Eigen::Vector3d point_local(intermediate_points_i.point.x(), intermediate_points_i.point.y(), intermediate_points_i.point.z());
        Eigen::Vector3d point_global = m_pose * point_local;

        ///////////////
        double r = point_global.norm();
        double polar_angle_deg = atan2(point_global.y(), point_global.x()) / M_PI * 180.0;
        double azimutal_angle_deg = acos(point_global.z() / r) / M_PI * 180.0;
        ///////////////

        auto index_of_bucket = get_rgd_index({r, polar_angle_deg, azimutal_angle_deg}, b);
        // auto index_of_bucket = get_rgd_index(point_global, b);

        auto bucket_it = buckets.find(index_of_bucket);
        // no bucket found
        if (bucket_it == buckets.end())
        {
            return;
        }
        auto &this_bucket = bucket_it->second;

        Eigen::Vector3d mean(this_bucket.mean.x(), this_bucket.mean.y(), this_bucket.mean.z());

        Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
        TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose);

        point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z,
                                                      pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                      point_local.x(), point_local.y(), point_local.z(), mean.x(), mean.y(), mean.z());

        point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian,
                                                               pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                               point_local.x(), point_local.y(), point_local.z());
        std::mutex &m = my_mutex[0]; // mutexes[intermediate_points_i.index_pose];
        std::unique_lock lck(m);

        int c = intermediate_points_i.index_pose * 6;
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

        Eigen::Matrix3d infm = this_bucket.cov.inverse();

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

    if (points_local.size() > 100)
    {
        // std::cout << "start adding lidar observations" << std::endl;
        if (multithread)
        {
            std::for_each(std::execution::par_unseq, std::begin(points_local), std::end(points_local), hessian_fun);
        }
        else
        {
            std::for_each(std::begin(points_local), std::end(points_local), hessian_fun);
        }
        // std::cout << "adding lidar observations finished" << std::endl;
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
    for (size_t i = 0; i < intermediate_trajectory.size(); i++)
    {
        poses_desired.push_back(pose_tait_bryan_from_affine_matrix(intermediate_trajectory[i]));
    }

    for (size_t i = 0; i < odo_edges.size(); i++)
    {
        /*Eigen::Matrix<double, 6, 1> relative_pose_measurement_odo;
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
                                          poses_desired[odo_edges[i].second].ka);*/

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
            0, 0, 0, 0, 0, 0);
        // relative_pose_measurement_odo(0, 0),
        // relative_pose_measurement_odo(1, 0),
        // relative_pose_measurement_odo(2, 0),
        // relative_pose_measurement_odo(3, 0),
        // relative_pose_measurement_odo(4, 0),
        // relative_pose_measurement_odo(5, 0));

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
        tripletListP.emplace_back(ir + 3, ir + 3, 1000000);
        tripletListP.emplace_back(ir + 4, ir + 4, 1000000);
        tripletListP.emplace_back(ir + 5, ir + 5, 1000000);
    }

    int ic = 0;
    int ir = tripletListB.size();
    tripletListA.emplace_back(ir, ic * 6 + 0, 1);
    tripletListA.emplace_back(ir + 1, ic * 6 + 1, 1);
    tripletListA.emplace_back(ir + 2, ic * 6 + 2, 1);
    tripletListA.emplace_back(ir + 3, ic * 6 + 3, 1);
    tripletListA.emplace_back(ir + 4, ic * 6 + 4, 1);
    tripletListA.emplace_back(ir + 5, ic * 6 + 5, 1);

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

    // AtPA += AtPAndt.sparseView();
    // AtPB += AtPBndt.sparseView();

    // Eigen::SparseMatrix<double> AtPA_I(intrinsics.size() * 6, intrinsics.size() * 6);
    // AtPA_I.setIdentity();
    // AtPA_I *= 1;
    // AtPA += AtPA_I;

    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>>
        solver(AtPA);
    // std::cout << "start solving" << std::endl;
    Eigen::SparseMatrix<double> x = solver.solve(AtPB);
    // std::cout << "start finished" << std::endl;
    std::vector<double> h_x;
    for (int k = 0; k < x.outerSize(); ++k)
    {
        for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
        {
            // std::cout << it.value() << " ";
            h_x.push_back(it.value());
        }
        // std::cout << std::endl;
    }

    if (h_x.size() == 6 * int_tr.size())
    {
        int counter = 0;

        for (size_t i = 0; i < int_tr.size(); i++)
        {
            TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(int_tr[i]);
            // auto prev_pose = pose;
            pose.px += h_x[counter++];
            pose.py += h_x[counter++];
            pose.pz += h_x[counter++];
            pose.om += h_x[counter++];
            pose.fi += h_x[counter++];
            pose.ka += h_x[counter++];

            int_tr[i] = affine_matrix_from_pose_tait_bryan(pose);
            // all_data[index_rendered_points_local].poses[i] = intermediate_trajectory[i];
        }

        intermediate_trajectory = int_tr;
        intermediate_trajectory_motion_model = int_tr;

        // auto int_tr = intermediate_trajectory;
        // auto int_tr_mm = intermediate_trajectory_motion_model;

        /*Eigen::Affine3d m = int_tr_tmp[0];

        std::vector<Eigen::Affine3d> out;
        out.push_back(m);

        for (int i = 1; i < int_tr.size(); i++)
        {
            auto update = int_tr[i - 1] * int_tr[i];
            m = m * update;
            out.push_back(m);
        }

        intermediate_trajectory = out;
        intermediate_trajectory_motion_model = out;*/
    }
    else
    {
        std::cout << "optimization failed" << std::endl;
    }
}

void optimize_sf2(std::vector<Point3Di> &intermediate_points, std::vector<Point3Di> &intermediate_points_sf, std::vector<Eigen::Affine3d> &intermediate_trajectory,
                  std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model,
                  NDT::GridParameters &rgd_params, bool useMultithread)
{
    std::vector<Point3Di> point_cloud_global;
    std::vector<Eigen::Vector3d> point_cloud_global_sc;
    std::vector<int> indexes;

    for (int i = 0; i < intermediate_points.size(); i++)
    {
        Point3Di pg = intermediate_points[i];
        pg.point = intermediate_trajectory[intermediate_points[i].index_pose] * pg.point;
        point_cloud_global.push_back(pg);
        double r_g = pg.point.norm();
        point_cloud_global_sc.emplace_back(r_g, atan2(pg.point.y(), pg.point.x()) / M_PI * 180.0, acos(pg.point.z() / r_g) / M_PI * 180.0);
        indexes.push_back(i);
    }

    NDTBucketMapType buckets;
    update_rgd_spherical_coordinates(rgd_params, buckets, point_cloud_global, point_cloud_global_sc, {0, 0, 0});

    std::vector<std::mutex> mutexes(indexes.size());

    Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

    Eigen::MatrixXd AtPAndt(intermediate_trajectory.size() * 6, intermediate_trajectory.size() * 6);
    AtPAndt.setZero();
    Eigen::MatrixXd AtPBndt(intermediate_trajectory.size() * 6, 1);
    AtPBndt.setZero();

    const auto hessian_fun = [&](const int &indexes_i)
    {
        auto index_of_bucket = get_rgd_index(point_cloud_global_sc[indexes_i], b);
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

        const Eigen::Affine3d &m_pose = intermediate_trajectory[intermediate_points[indexes_i].index_pose]; // intermediate_trajectory[intermediate_points_i.index_pose];
        const Eigen::Vector3d &p_s = intermediate_points[indexes_i].point;
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

        int c = intermediate_points[indexes_i].index_pose * 6;

        std::mutex &m = mutexes[intermediate_points[indexes_i].index_pose];
        std::unique_lock lck(m);
        AtPAndt.block<6, 6>(c, c) += AtPA;
        AtPBndt.block<6, 1>(c, 0) -= AtPB;
    };

    if (useMultithread)
    {
        std::for_each(std::execution::par_unseq, std::begin(indexes), std::end(indexes), hessian_fun);
    }
    else
    {
        std::for_each(std::begin(indexes), std::end(indexes), hessian_fun);
    }

    ///
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

    std::vector<std::pair<int, int>> odo_edges;
    for (size_t i = 1; i < intermediate_trajectory.size(); i++)
    {
        odo_edges.emplace_back(i - 1, i);
    }

    for (size_t i = 0; i < odo_edges.size(); i++)
    {
        /*Eigen::Matrix<double, 6, 1> relative_pose_measurement_odo;
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
                                                       poses_desired[odo_edges[i].second].ka);*/

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
                                                                 1000000,
                                                                 1000000,
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
                                                                 // relative_pose_measurement_odo(0, 0),
                                                                 // relative_pose_measurement_odo(1, 0),
                                                                 // relative_pose_measurement_odo(2, 0),
                                                                 // relative_pose_measurement_odo(3, 0),
                                                                 // relative_pose_measurement_odo(4, 0),
                                                                 // relative_pose_measurement_odo(5, 0),
                                                                 0, 0, 0, 0, 0, 0,
                                                                 1000000,
                                                                 1000000,
                                                                 1000000,
                                                                 1000000,
                                                                 1000000,
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

    std::vector<Eigen::Triplet<double>> tripletListA;
    std::vector<Eigen::Triplet<double>> tripletListP;
    std::vector<Eigen::Triplet<double>> tripletListB;

    int ic = 0;
    int ir = tripletListB.size();
    tripletListA.emplace_back(ir, ic * 6 + 0, 1);
    tripletListA.emplace_back(ir + 1, ic * 6 + 1, 1);
    tripletListA.emplace_back(ir + 2, ic * 6 + 2, 1);
    tripletListA.emplace_back(ir + 3, ic * 6 + 3, 1);
    tripletListA.emplace_back(ir + 4, ic * 6 + 4, 1);
    tripletListA.emplace_back(ir + 5, ic * 6 + 5, 1);

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

    Eigen::SparseMatrix<double> AtPA_I(intermediate_trajectory.size() * 6, intermediate_trajectory.size() * 6);
    AtPA_I.setIdentity();
    AtPA += AtPA_I;

    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>>
        solver(AtPA);
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

            // Eigen::Vector3d p1(prev_pose.px, prev_pose.py, prev_pose.pz);
            // Eigen::Vector3d p2(pose.px, pose.py, pose.pz);

            // if ((p1 - p2).norm() < 1.0)
            //{
            intermediate_trajectory[i] = affine_matrix_from_pose_tait_bryan(pose);
            intermediate_trajectory_motion_model[i] = intermediate_trajectory[i];
            //}
        }
    }
    ///
}

void optimize(std::vector<Point3Di> &intermediate_points, std::vector<Eigen::Affine3d> &intermediate_trajectory,
              std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model,
              NDT::GridParameters &rgd_params, NDTBucketMapType &buckets, bool multithread /*,
               bool add_pitch_roll_constraint, const std::vector<std::pair<double, double>> &imu_roll_pitch*/
)
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
        if (intermediate_points_i.point.norm() < 0.1)
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

        // planarity
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(this_bucket.cov, Eigen::ComputeEigenvectors);
        auto eigen_values = eigen_solver.eigenvalues();
        auto eigen_vectors = eigen_solver.eigenvectors();
        double ev1 = eigen_values.x();
        double ev2 = eigen_values.y();
        double ev3 = eigen_values.z();
        double sum_ev = ev1 + ev2 + ev3;
        auto planarity = 1 - ((3 * ev1 / sum_ev) * (3 * ev2 / sum_ev) * (3 * ev3 / sum_ev));

        double norm = p_s.norm();

        double w = planarity * norm;
        if (w > 10.0)
        {
            // std::cout << w << " " << planarity << " " << norm << "x " << p_s.x() << "y " << p_s.y() << "z " << p_s.z() << std::endl;
            w = 10.0;
        }

        AtPA *= w;
        AtPB *= w;

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
                                                                 //  100000000, underground mining
                                                                 //  1000000, underground mining
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
                                                                 //  100000000, underground mining
                                                                 //  1000000, underground mining
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
    /*if (add_pitch_roll_constraint)
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
    }*/

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

    // exit(1);
    int ic = 0;
    int ir = tripletListB.size();
    tripletListA.emplace_back(ir, ic * 6 + 0, 1);
    tripletListA.emplace_back(ir + 1, ic * 6 + 1, 1);
    tripletListA.emplace_back(ir + 2, ic * 6 + 2, 1);
    tripletListA.emplace_back(ir + 3, ic * 6 + 3, 1);
    tripletListA.emplace_back(ir + 4, ic * 6 + 4, 1);
    tripletListA.emplace_back(ir + 5, ic * 6 + 5, 1);

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

    Eigen::SparseMatrix<double> AtPA_I(intermediate_trajectory.size() * 6, intermediate_trajectory.size() * 6);
    AtPA_I.setIdentity();
    AtPA += AtPA_I;

    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>>
        solver(AtPA);
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

            if ((p1 - p2).norm() < 1.0)
            {
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

#if 0
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
#endif

bool compute_step_2(std::vector<WorkerData> &worker_data, LidarOdometryParams &params, double &ts_failure)
{
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
            // std::cout << "jojo" << std::endl;
            Eigen::Vector3d mean_shift(0.0, 0.0, 0.0);
            if (i > 1 && params.use_motion_from_previous_step)
            {
                // mean_shift = worker_data[i - 1].intermediate_trajectory[0].translation() - worker_data[i - 2].intermediate_trajectory[worker_data[i - 2].intermediate_trajectory.size() - 1].translation();
                // mean_shift /= ((worker_data[i - 2].intermediate_trajectory.size()) - 2);

                mean_shift = worker_data[i - 1].intermediate_trajectory[worker_data[i - 1].intermediate_trajectory.size() - 1].translation() -
                             worker_data[i - 2].intermediate_trajectory[worker_data[i - 2].intermediate_trajectory.size() - 1].translation();
                mean_shift /= (worker_data[i - 1].intermediate_trajectory.size());

                if (mean_shift.norm() > 1.0)
                {
                    std::cout << "!!!mean_shift.norm() > 1.0!!!" << std::endl;
                    mean_shift = Eigen::Vector3d(0.0, 0.0, 0.0);
                }

                std::vector<Eigen::Affine3d> new_trajectory;
                Eigen::Affine3d current_node = worker_data[i - 1].intermediate_trajectory[worker_data[i - 1].intermediate_trajectory.size() - 1];
                new_trajectory.push_back(current_node);

                for (int tr = 1; tr < worker_data[i].intermediate_trajectory.size(); tr++)
                {
                    auto update = worker_data[i].intermediate_trajectory[tr - 1].inverse() * worker_data[i].intermediate_trajectory[tr];
                    current_node = current_node * update;
                    // current_node.linear() = //worker_data[i].intermediate_trajectory[tr].linear();
                    // current_node.translation() += mean_shift;
                    new_trajectory.push_back(current_node);
                }

                for (int tr = 0; tr < new_trajectory.size(); tr++)
                {
                    new_trajectory[tr].translation() += mean_shift * tr;
                }

                worker_data[i].intermediate_trajectory = new_trajectory;
                worker_data[i].intermediate_trajectory_motion_model = new_trajectory;
                ////////////////////////////////////////////////////////////////////////
                // std::vector<Eigen::Affine3d> new_trajectory_motion_model;
                // Eigen::Affine3d current_node_motion_model = worker_data[i].intermediate_trajectory_motion_model[0];
                // new_trajectory_motion_model.push_back(current_node_motion_model);

                // Eigen::Vector3d mean_shift_t = worker_data[i].intermediate_trajectory_motion_model[0].linear() * ((worker_data[i].intermediate_trajectory[0].linear()).inverse() * mean_shift);

                // for (int tr = 1; tr < worker_data[i].intermediate_trajectory_motion_model.size(); tr++)
                //{
                //     current_node_motion_model.linear() = worker_data[i].intermediate_trajectory_motion_model[tr].linear();
                //     current_node_motion_model.translation() += mean_shift_t;
                //     new_trajectory_motion_model.push_back(current_node_motion_model);
                // }

                // worker_data[i].intermediate_trajectory_motion_model = new_trajectory_motion_model;
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

            // std::string fn1 = "input_" + std::to_string(i) + ".txt";
            // ofstream file1;
            // file1.open(fn1);
            // for (int k = 0; k < worker_data[i].intermediate_trajectory.size(); k++){
            //     file1 << worker_data[i].intermediate_trajectory[k](0, 3) << " " << worker_data[i].intermediate_trajectory[k](1, 3) << " " << worker_data[i].intermediate_trajectory[k](2, 3) << " 0" << std::endl;
            // }
            // file1.close();
            if (params.use_robust_and_accurate_lidar_odometry)
            {
                auto tr = worker_data[i].intermediate_trajectory;
                auto trmm = worker_data[i].intermediate_trajectory_motion_model;

                auto firstm = tr[0];

                for (auto &t : tr)
                {
                    t.translation() -= firstm.translation();
                }
                for (auto &t : trmm)
                {
                    t.translation() -= firstm.translation();
                }

                NDT::GridParameters rgd_params_sc;
                
                rgd_params_sc.resolution_X = params.distance_bucket;
                rgd_params_sc.resolution_Y = params.polar_angle_deg;
                rgd_params_sc.resolution_Z = params.azimutal_angle_deg;

                // for (int iter = 0; iter < 10; iter++)
                //{
                //     optimize_sf(worker_data[i].intermediate_points, tr, trmm,
                //                 rgd_params_sc, params.buckets, /*params.useMultithread*/ false);
                // }
                std::vector<Point3Di> points_local_sf;
                std::vector<Point3Di> points_local;

                ///
                for (int ii = 0; ii < worker_data[i].intermediate_points.size(); ii++)
                {
                    double r_l = worker_data[i].intermediate_points[ii].point.norm();
                    if (r_l > 0.5 && worker_data[i].intermediate_points[ii].index_pose != -1 && r_l < params.max_distance_lidar)
                    {
                        double polar_angle_deg_l = atan2(worker_data[i].intermediate_points[ii].point.y(), worker_data[i].intermediate_points[ii].point.x()) / M_PI * 180.0;
                        double azimutal_angle_deg_l = acos(worker_data[i].intermediate_points[ii].point.z() / r_l) / M_PI * 180.0;
                       
                        points_local.push_back(worker_data[i].intermediate_points[ii]);

                        ///////////////////////////////////////////////////////
                        Point3Di p_sl = worker_data[i].intermediate_points[ii];
                        p_sl.point.x() = r_l;
                        p_sl.point.y() = polar_angle_deg_l;
                        p_sl.point.z() = azimutal_angle_deg_l;

                        points_local_sf.push_back(p_sl);
                    }
                }
                ///
                std::cout << "optimize_sf2" << std::endl;
                for (int iter = 0; iter < params.robust_and_accurate_lidar_odometry_iterations; iter++)
                {
                    optimize_sf2(points_local, points_local_sf, tr, trmm, rgd_params_sc, params.useMultithread);
                }

                for (auto &t : tr)
                {
                    t.translation() += firstm.translation();
                }

                for (auto &t : trmm)
                {
                    t.translation() += firstm.translation();
                }

                worker_data[i].intermediate_trajectory = tr;
                worker_data[i].intermediate_trajectory_motion_model = tr;
            }

            for (int iter = 0; iter < params.nr_iter; iter++)
            {
                optimize(worker_data[i].intermediate_points, worker_data[i].intermediate_trajectory, worker_data[i].intermediate_trajectory_motion_model,
                         params.in_out_params, params.buckets, params.useMultithread /*, add_pitch_roll_constraint, worker_data[i].imu_roll_pitch*/);
            }

            // std::string fn2 = "output_" + std::to_string(i) + ".txt";
            // ofstream file2;
            // file2.open(fn2);
            // for (int k = 0; k < worker_data[i].intermediate_trajectory.size(); k++)
            //{
            //     file2 << worker_data[i].intermediate_trajectory[k](0, 3) << " " << worker_data[i].intermediate_trajectory[k](1, 3) << " " << worker_data[i].intermediate_trajectory[k](2, 3) << " 1" << std::endl;
            // }
            // file2.close();

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
            /*if (params.reference_points.size() > 0)
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
            }*/

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
    // std::vector<TaitBryanPose> poses_desired;

    for (size_t i = 0; i < trajectory.size(); i++)
    {
        poses.push_back(pose_tait_bryan_from_affine_matrix(trajectory[i]));
    }
    // for (size_t i = 0; i < trajectory_motion_model.size(); i++)
    //{
    //     poses_desired.push_back(pose_tait_bryan_from_affine_matrix(trajectory_motion_model[i]));
    // }

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
    for (size_t index_point_begin = 0; index_point_begin < all_points_local.size(); index_point_begin += 1000000)
    {
        NDTBucketMapType buckets;
        std::vector<Point3Di> points_local;
        std::vector<Point3Di> points_global;

        for (int index = index_point_begin; index < index_point_begin + 1000000; index++)
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

void Consistency2(std::vector<WorkerData> &worker_data, LidarOdometryParams &params)
{
    // global_tmp.clear();

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
    // std::vector<TaitBryanPose> poses_desired;

    for (size_t i = 0; i < trajectory.size(); i++)
    {
        poses.push_back(pose_tait_bryan_from_affine_matrix(trajectory[i]));
    }
    // for (size_t i = 0; i < trajectory_motion_model.size(); i++)
    //{
    //     poses_desired.push_back(pose_tait_bryan_from_affine_matrix(trajectory_motion_model[i]));
    // }

    double angle = 0.01 / 180.0 * M_PI;
    double wangle = 1.0 / (angle * angle);

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

    // reindex data
    std::cout << "reindex data START" << std::endl;

    std::vector<Point3Di> points_local;
    std::vector<Point3Di> points_global;

    points_local.reserve(all_points_local.size());
    points_global.reserve(all_points_local.size());

    for (int index = 0; index < all_points_local.size(); index++)
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

                points_local.emplace_back(pl);
                pl.point = trajectory[pl.index_pose] * pl.point;
                points_global.emplace_back(pl);
            }
        }
    }

    int counter_index = 0;
    int counter = 0;
    for (int i = 0; i < points_global.size(); i++)
    {
        points_global[i].index_point = counter_index;

        if (counter > 1000000)
        {
            counter = 0;
            counter_index++;
        }
        counter++;
    }

    std::cout << "reindex data FINISHED" << std::endl;

    std::cout << "build regular grid decomposition START" << std::endl;

    // update_rgd2(params.in_out_params, buckets, points_global, trajectory[points_global[0].index_pose].translation());

    std::vector<std::pair<unsigned long long int, unsigned int>> index_pairs;
    for (int i = 0; i < points_global.size(); i++)
    {
        auto index_of_bucket = get_rgd_index(points_global[i].point, b);
        index_pairs.emplace_back(index_of_bucket, i);
    }
    std::sort(index_pairs.begin(), index_pairs.end(),
              [](const std::pair<unsigned long long int, unsigned int> &a, const std::pair<unsigned long long int, unsigned int> &b)
              { return a.first < b.first; });

    // for (int i = 0; i < index_pairs.size(); i++)
    //{
    //     std::cout << index_pairs[i].first << " " << index_pairs[i].second << std::endl;
    // }

    NDTBucketMapType2 buckets;

    for (unsigned int i = 0; i < index_pairs.size(); i++)
    {
        unsigned long long int index_of_bucket = index_pairs[i].first;
        if (buckets.contains(index_of_bucket))
        {
            buckets[index_of_bucket].index_end_exclusive = i;
        }
        else
        {
            buckets[index_of_bucket].index_begin_inclusive = i;
            buckets[index_of_bucket].index_end_exclusive = i;
        }
    }

    // for (const auto &b : buckets){
    //     std::cout << "---------" << std::endl;
    //     std::cout << b.second.index_end_exclusive - b.second.index_begin_inclusive << std::endl;
    //     for (int i = b.second.index_begin_inclusive; i < b.second.index_end_exclusive; i++){
    //         std::cout << index_pairs[i].first << " " << index_pairs[i].second << " " << points_global[index_pairs[i].second].index_point << std::endl;
    //     }
    // }

    for (auto &b : buckets)
    {
        for (int i = b.second.index_begin_inclusive; i < b.second.index_end_exclusive; i++)
        {
            long long unsigned int index_point_segment = points_global[index_pairs[i].second].index_point;
            // std::vector<unsigned int> point_indexes;

            if (b.second.buckets.contains(index_point_segment))
            {
                b.second.buckets[index_point_segment].point_indexes.push_back(index_pairs[i].second);
            }
            else
            {
                NDT::BucketCoef bc;
                bc.point_indexes.push_back(index_pairs[i].second);
                b.second.buckets[index_point_segment] = bc;
            }
        }

        /*std::cout << "-----------------" << std::endl;
        for(const auto &bb:b.second.buckets){
            std::cout << "===============" << std::endl;
            for (const auto &p : bb.second.point_indexes){
                std::cout << points_global[p].point << std::endl;
                std::cout << "--" << std::endl;
            }
        }*/

        for (auto &bb : b.second.buckets)
        {
            if (bb.second.point_indexes.size() >= 5)
            {
                bb.second.valid = true;

                bb.second.mean = Eigen::Vector3d(0, 0, 0);
                for (const auto &p : bb.second.point_indexes)
                {
                    bb.second.mean += points_global[p].point;
                }
                bb.second.mean /= bb.second.point_indexes.size();

                bb.second.cov.setZero();
                for (const auto &p : bb.second.point_indexes)
                {
                    bb.second.cov(0, 0) += (bb.second.mean.x() - points_global[p].point.x()) * (bb.second.mean.x() - points_global[p].point.x());
                    bb.second.cov(0, 1) += (bb.second.mean.x() - points_global[p].point.x()) * (bb.second.mean.y() - points_global[p].point.y());
                    bb.second.cov(0, 2) += (bb.second.mean.x() - points_global[p].point.x()) * (bb.second.mean.z() - points_global[p].point.z());
                    bb.second.cov(1, 0) += (bb.second.mean.y() - points_global[p].point.y()) * (bb.second.mean.x() - points_global[p].point.x());
                    bb.second.cov(1, 1) += (bb.second.mean.y() - points_global[p].point.y()) * (bb.second.mean.y() - points_global[p].point.y());
                    bb.second.cov(1, 2) += (bb.second.mean.y() - points_global[p].point.y()) * (bb.second.mean.z() - points_global[p].point.z());
                    bb.second.cov(2, 0) += (bb.second.mean.z() - points_global[p].point.z()) * (bb.second.mean.x() - points_global[p].point.x());
                    bb.second.cov(2, 1) += (bb.second.mean.z() - points_global[p].point.z()) * (bb.second.mean.y() - points_global[p].point.y());
                    bb.second.cov(2, 2) += (bb.second.mean.z() - points_global[p].point.z()) * (bb.second.mean.z() - points_global[p].point.z());
                }
                bb.second.cov /= bb.second.point_indexes.size();

                //
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(bb.second.cov, Eigen::ComputeEigenvectors);

                Eigen::Matrix3d eigenVectorsPCA = eigen_solver.eigenvectors();
                Eigen::Vector3d nv = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
                nv.normalize();

                //  flip towards viewport
                const auto &m = trajectory[points_global[bb.second.point_indexes[0]].index_pose];

                if (nv.dot(m.translation() - bb.second.mean) < 0.0)
                {
                    nv *= -1.0;
                }
                bb.second.normal_vector = nv;

                bb.second.point_indexes.clear();
                // std::cout << bb.second.mean.x() << " " << bb.second.mean.y() << " " << bb.second.mean.z() << " " << bb.second.normal_vector.x() << " " << bb.second.normal_vector.y() << " " << bb.second.normal_vector.z() << " " << std::endl;
                // global_tmp.emplace_back(bb.second.mean, bb.second.normal_vector);
                /*const auto &p = (*pp)[(*index_pair_internal)[b.index_begin].index_of_point];
                const auto &m = (*mposes)[p.index_pose];
                //  flip towards viewport
                if (nv.dot(m.translation() - mean) < 0.0)
                {
                    nv *= -1.0;
                }
                // this_bucket.normal_vector = nv;
                (*buckets)[ii].normal_vector = nv;*/
                //
            }
            else
            {
                bb.second.valid = false;
            }
        }
    }
    std::cout << "build regular grid decomposition FINISHED" << std::endl;

    std::cout << "ndt observations start START" << std::endl;

    counter = 0;

    for (int i = 0; i < points_global.size(); i++)
    {
        if (i % 10000 == 0)
        {
            std::cout << "computing " << i << " of " << points_global.size() << std::endl;
        }
        if (points_local[i].point.norm() < 1.0)
        {
            continue;
        }

        auto index_of_bucket = get_rgd_index(points_global[i].point, b);
        auto bucket_it = buckets.find(index_of_bucket);
        // no bucket found
        if (bucket_it == buckets.end())
        {
            continue;
        }
        for (auto &bb : bucket_it->second.buckets)
        {
            if (bb.second.valid)
            {
                auto &this_bucket = bb.second;

                const Eigen::Matrix3d &infm = this_bucket.cov.inverse();
                const double threshold = 10000.0;

                if ((infm.array() > threshold).any())
                {
                    continue;
                }
                if ((infm.array() < -threshold).any())
                {
                    continue;
                }

                // check nv
                Eigen::Vector3d &nv = this_bucket.normal_vector;
                Eigen::Vector3d viewport = trajectory[points_global[i].index_pose].translation();
                if (nv.dot(viewport - this_bucket.mean) < 0)
                {
                    // std::cout << "nv!";
                    continue;
                }

                const Eigen::Affine3d &m_pose = trajectory[points_global[i].index_pose];
                const Eigen::Vector3d &p_s = points_local[i].point;
                const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose);
                double delta_x;
                double delta_y;
                double delta_z;

                Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
                // TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix((*mposes)[p.index_pose]);

                point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z,
                                                              pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                              p_s.x(), p_s.y(), p_s.z(), this_bucket.mean.x(), this_bucket.mean.y(), this_bucket.mean.z());

                point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian,
                                                                       pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                                       p_s.x(), p_s.y(), p_s.z());

                int c = points_global[i].index_pose * 6;

                std::mutex &m = my_mutex[0];
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

                counter++;
                if (counter > 10000000)
                {
                    counter = 0;
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
        }
    }

    if (counter > 0)
    {
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

    std::cout << "ndt observations start FINISHED" << std::endl;

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