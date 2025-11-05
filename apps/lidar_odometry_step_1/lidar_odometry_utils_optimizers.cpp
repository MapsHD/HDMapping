#include "lidar_odometry_utils.h"
#include <hash_utils.h>
#include <mutex>

#include <export_laz.h>
// extern std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> global_tmp;
namespace
{
    //! Structure holds block of Hessian from observations to be sumed
    struct Blocks
    {
        Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AtPAndtBlocksToSum;
        Eigen::Matrix<double, 6, 1> AtPBndtBlocksToSum;
        int c = 0;
    };

    //! Sumation of blocks
    void SumBlocks(const std::vector<Blocks> &blocks, Eigen::MatrixXd &AtPAndt, Eigen::MatrixXd &AtPBndt)
    {
        for (auto &block : blocks)
        {
            const int c = block.c;
            AtPAndt.block<6, 6>(c, c) += block.AtPAndtBlocksToSum;
            AtPBndt.block<6, 1>(c, 0) -= block.AtPBndtBlocksToSum;
        }
    };
}
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
                        }
                    }
                }
            }
        }

        // std::cout << "------------" << std::endl;
        for (size_t y = 0; y < min_distances.size(); y++)
        {
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

    std::vector<std::pair<int, int>> nn = nns(all_points_global, indexes_for_nn);

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

    auto int_tr = intermediate_trajectory;
    auto int_tr_tmp = intermediate_trajectory;
    auto int_tr_mm = intermediate_trajectory_motion_model;

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
    update_rgd_spherical_coordinates(rgd_params, buckets, point_cloud_global, point_cloud_global_sc);

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

    Eigen::SparseMatrix<double> x = solver.solve(AtPB);

    std::vector<double> h_x;
    for (int k = 0; k < x.outerSize(); ++k)
    {
        for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
        {

            h_x.push_back(it.value());
        }
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
        }

        intermediate_trajectory = int_tr;
        intermediate_trajectory_motion_model = int_tr;
    }
    else
    {
        std::cout << "optimization failed" << std::endl;
    }
}

void optimize_sf2(std::vector<Point3Di> &intermediate_points, std::vector<Point3Di> &intermediate_points_sf, std::vector<Eigen::Affine3d> &intermediate_trajectory,
                  const std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model,
                  NDT::GridParameters &rgd_params, bool useMultithread, double wx,
                  double wy,
                  double wz,
                  double wom,
                  double wfi,
                  double wka)
{

    auto trj = intermediate_trajectory;
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
    update_rgd_spherical_coordinates(rgd_params, buckets, point_cloud_global, point_cloud_global_sc);

    Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

    const auto hessian_fun = [&](const int &indexes_i) -> Blocks
    {
        int c = intermediate_points[indexes_i].index_pose * 6;
        auto index_of_bucket = get_rgd_index(point_cloud_global_sc[indexes_i], b);
        auto bucket_it = buckets.find(index_of_bucket);
        // no bucket found
        if (bucket_it == buckets.end())
        {
            return {Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero(), Eigen::Matrix<double, 6, 1>::Zero(), c};
        }
        auto &this_bucket = bucket_it->second;

        const Eigen::Matrix3d &infm = this_bucket.cov.inverse();
        const double threshold = 100000.0;

        if ((infm.array() > threshold).any())
        {

            return {Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero(), Eigen::Matrix<double, 6, 1>::Zero(), c};
        }
        if ((infm.array() < -threshold).any())
        {
            return {Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero(), Eigen::Matrix<double, 6, 1>::Zero(), c};
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

        // planarity
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(this_bucket.cov, Eigen::ComputeEigenvectors);
        auto eigen_values = eigen_solver.eigenvalues();
        auto eigen_vectors = eigen_solver.eigenvectors();
        double ev1 = eigen_values.x();
        double ev2 = eigen_values.y();
        double ev3 = eigen_values.z();
        double sum_ev = ev1 + ev2 + ev3;
        auto planarity = 1 - ((3 * ev1 / sum_ev) * (3 * ev2 / sum_ev) * (3 * ev3 / sum_ev));

        double w = planarity; // * (ref1 * ref2) / (a * b);

        // Wisdom ahead (mpelka) :
        // The order of execution in summing matters, so using concurrent execution, we will get slightly different output.
        // We should compute every contibution to Hessian in parallel, then add those ouputs sequentially.
        return Blocks{AtPA * w, AtPB * w, c};
    };

    Eigen::MatrixXd AtPAndt(intermediate_trajectory.size() * 6, intermediate_trajectory.size() * 6);
    AtPAndt.setZero();
    Eigen::MatrixXd AtPBndt(intermediate_trajectory.size() * 6, 1);
    AtPBndt.setZero();

    std::vector<Blocks> AtPAndtBlocksToSum(intermediate_points.size());

    if (useMultithread)
    {
        std::transform(std::execution::par_unseq, std::begin(indexes), std::end(indexes), std::begin(AtPAndtBlocksToSum), hessian_fun);
    }
    else
    {
        std::transform(std::begin(indexes), std::end(indexes), std::begin(AtPAndtBlocksToSum), hessian_fun);
    }

    SumBlocks(AtPAndtBlocksToSum, AtPAndt, AtPBndt);

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
                                                                 wx,
                                                                 wy,
                                                                 wz,
                                                                 wom,
                                                                 wfi,
                                                                 wka);
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
                                                                 0, 0, 0, 0, 0, 0,
                                                                 wx,
                                                                 wy,
                                                                 wz,
                                                                 wom,
                                                                 wfi,
                                                                 wka);
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
            pose.px += h_x[counter++] * 0.5;
            pose.py += h_x[counter++] * 0.5;
            pose.pz += h_x[counter++] * 0.5;
            pose.om += h_x[counter++] * 0.5;
            pose.fi += h_x[counter++] * 0.5;
            pose.ka += h_x[counter++] * 0.5;

            intermediate_trajectory[i] = affine_matrix_from_pose_tait_bryan(pose);
        }

        //

        Eigen::Affine3d first = trj[0];
        std::vector<Eigen::Affine3d> out;
        out.push_back(first);

        for (int i = 1; i < intermediate_trajectory.size(); i++)
        {
            auto update = intermediate_trajectory[i - 1].inverse() * intermediate_trajectory[i];

            first = first * update;
            out.push_back(first);
        }

        intermediate_trajectory = out;
        //
    }
    ///
}

void optimize_rigid_sf(
    const std::vector<Point3Di> &intermediate_points,
    std::vector<Eigen::Affine3d> &intermediate_trajectory,
    std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model,
    const NDTBucketMapType &buckets,
    double distance_bucket,
    double polar_angle_deg,
    double azimutal_angle_deg,
    int robust_and_accurate_lidar_odometry_sf_rigid_iterations,
    double max_distance_lidar_rigid_sf,
    bool useMultithread,
    double rgd_sf_sigma_x_m,
    double rgd_sf_sigma_y_m,
    double rgd_sf_sigma_z_m,
    double rgd_sf_sigma_om_deg,
    double rgd_sf_sigma_fi_deg,
    double rgd_sf_sigma_ka_deg)
{

    auto shift = intermediate_trajectory[0].translation();

    auto _intermediate_trajectory = intermediate_trajectory;
    auto _intermediate_trajectory_motion_model = intermediate_trajectory_motion_model;
    auto _buckets = buckets;

    NDT::GridParameters rgd_params_sc;

    rgd_params_sc.resolution_X = distance_bucket;
    rgd_params_sc.resolution_Y = polar_angle_deg;
    rgd_params_sc.resolution_Z = azimutal_angle_deg;

    for (auto &t : _intermediate_trajectory)
    {
        t.translation() -= shift;
    }

    for (auto &t : _intermediate_trajectory_motion_model)
    {
        t.translation() -= shift;
    }

    for (auto &b : _buckets)
    {
        b.second.mean -= shift;
    }

    std::vector<Eigen::Vector3d> points_rgd;
    std::vector<Point3Di> point_cloud_global;
    std::vector<Eigen::Vector3d> point_cloud_global_sc;

    auto minv = _intermediate_trajectory[0].inverse();

    for (const auto &b : _buckets)
    {
        auto pinv = minv * b.second.mean;

        if (pinv.norm() < max_distance_lidar_rigid_sf)
        {
            points_rgd.push_back(b.second.mean);
        }
    }

    for (int i = 0; i < points_rgd.size(); i++)
    {
        double r_l = points_rgd[i].norm();
        double polar_angle_deg_l = atan2(points_rgd[i].y(), points_rgd[i].x()) / M_PI * 180.0;
        double azimutal_angle_deg_l = acos(points_rgd[i].z() / r_l) / M_PI * 180.0;

        point_cloud_global_sc.emplace_back(r_l, polar_angle_deg_l, azimutal_angle_deg_l);

        Point3Di p;
        p.point = points_rgd[i];
        point_cloud_global.push_back(p);
    }

    NDTBucketMapType buckets_sf;
    update_rgd_spherical_coordinates(rgd_params_sc, buckets_sf, point_cloud_global, point_cloud_global_sc);

    std::vector<Eigen::Vector3d> points_local;
    std::vector<int> indexes;

    auto first_pose_inverse = _intermediate_trajectory[0].inverse();
    auto first_pose = _intermediate_trajectory[0];
    for (int i = 0; i < intermediate_points.size(); i++)
    {
        Eigen::Vector3d p = _intermediate_trajectory[intermediate_points[i].index_pose] * intermediate_points[i].point;
        p = first_pose_inverse * p;
        points_local.push_back(p);
        indexes.push_back(i);
    }

    // ICP spherical coordinates

    for (int iter = 0; iter < robust_and_accurate_lidar_odometry_sf_rigid_iterations; iter++)
    {
        [[maybe_unused]] std::atomic<int> number_of_observations = 0;

        const auto hessian_fun = [&](const int &indexes_i) -> Blocks
        {
            int c = 0;
            const auto &point_local = points_local[indexes_i];
            auto point_global = first_pose * point_local;

            double r_l = point_global.norm();
            double polar_angle_deg_l = atan2(point_global.y(), point_global.x()) / M_PI * 180.0;
            double azimutal_angle_deg_l = acos(point_global.z() / r_l) / M_PI * 180.0;

            auto index_of_bucket = get_rgd_index({r_l, polar_angle_deg_l, azimutal_angle_deg_l}, {rgd_params_sc.resolution_X, rgd_params_sc.resolution_Y, rgd_params_sc.resolution_Z});

            auto bucket_it = buckets_sf.find(index_of_bucket);
            // no bucket found
            if (bucket_it == buckets_sf.end())
            {
                return {Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero(), Eigen::Matrix<double, 6, 1>::Zero(), c};
            }
            auto &this_bucket = bucket_it->second;

            const Eigen::Matrix3d &infm = this_bucket.cov.inverse();
            const double threshold = 100000.0;

            if ((infm.array() > threshold).any())
            {
                // std::cout << "infm.array() " << infm.array() << std::endl;
                return {Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero(), Eigen::Matrix<double, 6, 1>::Zero(), c};
            }
            if ((infm.array() < -threshold).any())
            {
                // std::cout << "infm.array() " << infm.array() << std::endl;
                return {Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero(), Eigen::Matrix<double, 6, 1>::Zero(), c};
            }

            const Eigen::Affine3d &m_pose = first_pose;
            const Eigen::Vector3d &p_s = points_local[indexes_i];
            const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose);

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

            // int c = 0;

            // planarity
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(this_bucket.cov, Eigen::ComputeEigenvectors);
            auto eigen_values = eigen_solver.eigenvalues();
            auto eigen_vectors = eigen_solver.eigenvectors();
            double ev1 = eigen_values.x();
            double ev2 = eigen_values.y();
            double ev3 = eigen_values.z();
            double sum_ev = ev1 + ev2 + ev3;
            auto planarity = 1 - ((3 * ev1 / sum_ev) * (3 * ev2 / sum_ev) * (3 * ev3 / sum_ev));

            double w = planarity; // * (ref1 * ref2) / (a * b);
            AtPA *= w;
            AtPB *= w;

            number_of_observations++;
            return {AtPA, AtPB, c};
        };

        std::vector<Blocks> blocks(indexes.size());

        if (useMultithread) // ToDo fix for this case
        {
            std::transform(std::execution::par_unseq, std::begin(indexes), std::end(indexes), std::begin(blocks), hessian_fun);
        }
        else
        {
            std::transform(std::begin(indexes), std::end(indexes), std::begin(blocks), hessian_fun);
        }
        Eigen::MatrixXd AtPAndt(6, 6);
        AtPAndt.setZero();
        Eigen::MatrixXd AtPBndt(6, 1);
        AtPBndt.setZero();

        SumBlocks(blocks, AtPAndt, AtPBndt);

        // std::cout << "number_of_observations " << number_of_observations << std::endl;

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

        double wx = 1.0 / (rgd_sf_sigma_x_m * rgd_sf_sigma_x_m);
        double wy = 1.0 / (rgd_sf_sigma_y_m * rgd_sf_sigma_y_m);
        double wz = 1.0 / (rgd_sf_sigma_z_m * rgd_sf_sigma_z_m);

        double a_om = rgd_sf_sigma_om_deg / 180.0 * M_PI;
        double w_om = 1.0 / (a_om * a_om);

        double a_fi = rgd_sf_sigma_fi_deg / 180.0 * M_PI;
        double w_fi = 1.0 / (a_fi * a_fi);

        double a_ka = rgd_sf_sigma_ka_deg / 180.0 * M_PI;
        double w_ka = 1.0 / (a_ka * a_ka);

        tripletListP.emplace_back(ir, ir, wx);
        tripletListP.emplace_back(ir + 1, ir + 1, wy);
        tripletListP.emplace_back(ir + 2, ir + 2, wz);
        tripletListP.emplace_back(ir + 3, ir + 3, w_om);
        tripletListP.emplace_back(ir + 4, ir + 4, w_fi);
        tripletListP.emplace_back(ir + 5, ir + 5, w_ka);

        tripletListB.emplace_back(ir, 0, 0);
        tripletListB.emplace_back(ir + 1, 0, 0);
        tripletListB.emplace_back(ir + 2, 0, 0);
        tripletListB.emplace_back(ir + 3, 0, 0);
        tripletListB.emplace_back(ir + 4, 0, 0);
        tripletListB.emplace_back(ir + 5, 0, 0);

        Eigen::SparseMatrix<double> matA(tripletListB.size(), 6);
        Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
        Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

        matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
        matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
        matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

        Eigen::SparseMatrix<double> AtPA(6, 6);
        Eigen::SparseMatrix<double> AtPB(6, 1);

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
                // std::cout << "it.value() " << it.value() << std::endl;
                h_x.push_back(it.value());
            }
        }

        if (h_x.size() == 6)
        {
            TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(first_pose);

            pose.px += h_x[0];
            pose.py += h_x[1];
            pose.pz += h_x[2];
            pose.om += h_x[3];
            pose.fi += h_x[4];
            pose.ka += h_x[5];

            first_pose = affine_matrix_from_pose_tait_bryan(pose);
        }
    }

    std::vector<Eigen::Affine3d> trj; // = _intermediate_trajectory;
    trj.push_back(first_pose);

    for (int i = 1; i < _intermediate_trajectory.size(); i++)
    {
        auto update = _intermediate_trajectory[i - 1].inverse() * _intermediate_trajectory[i];
        first_pose = first_pose * update;
        trj.push_back(first_pose);
    }
    _intermediate_trajectory = trj;

    ///
    for (int i = 0; i < _intermediate_trajectory.size(); i++)
    {
        _intermediate_trajectory[i].translation() += shift;
    }

    intermediate_trajectory = _intermediate_trajectory;
    intermediate_trajectory_motion_model = _intermediate_trajectory;
}

void optimize_lidar_odometry(std::vector<Point3Di> &intermediate_points,
                             std::vector<Eigen::Affine3d> &intermediate_trajectory,
                             std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model,
                             NDT::GridParameters &rgd_params_indoor, NDTBucketMapType &buckets_indoor,
                             NDT::GridParameters &rgd_params_outdoor, NDTBucketMapType &buckets_outdoor, bool multithread,
                             double max_distance, double &delta,
                             double lm_factor,
                             TaitBryanPose motion_model_correction,
                             double lidar_odometry_motion_model_x_1_sigma_m,
                             double lidar_odometry_motion_model_y_1_sigma_m,
                             double lidar_odometry_motion_model_z_1_sigma_m,
                             double lidar_odometry_motion_model_om_1_sigma_deg,
                             double lidar_odometry_motion_model_fi_1_sigma_deg,
                             double lidar_odometry_motion_model_ka_1_sigma_deg,
                             double lidar_odometry_motion_model_fix_origin_x_1_sigma_m,
                             double lidar_odometry_motion_model_fix_origin_y_1_sigma_m,
                             double lidar_odometry_motion_model_fix_origin_z_1_sigma_m,
                             double lidar_odometry_motion_model_fix_origin_om_1_sigma_deg,
                             double lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg,
                             double lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg)
{

    double sigma_motion_model_om = lidar_odometry_motion_model_om_1_sigma_deg * M_PI / 180.0;
    double sigma_motion_model_fi = lidar_odometry_motion_model_fi_1_sigma_deg * M_PI / 180.0;
    double sigma_motion_model_ka = lidar_odometry_motion_model_ka_1_sigma_deg * M_PI / 180.0;

    double w_motion_model_om = 1.0 / (sigma_motion_model_om * sigma_motion_model_om);
    double w_motion_model_fi = 1.0 / (sigma_motion_model_fi * sigma_motion_model_fi);
    double w_motion_model_ka = 1.0 / (sigma_motion_model_ka * sigma_motion_model_ka);

    double w_motion_model_x = 1.0 / (lidar_odometry_motion_model_x_1_sigma_m * lidar_odometry_motion_model_x_1_sigma_m);
    double w_motion_model_y = 1.0 / (lidar_odometry_motion_model_y_1_sigma_m * lidar_odometry_motion_model_y_1_sigma_m);
    double w_motion_model_z = 1.0 / (lidar_odometry_motion_model_z_1_sigma_m * lidar_odometry_motion_model_z_1_sigma_m);



    std::vector<Eigen::Triplet<double>> tripletListA;
    std::vector<Eigen::Triplet<double>> tripletListP;
    std::vector<Eigen::Triplet<double>> tripletListB;

    Eigen::MatrixXd AtPAndt(intermediate_trajectory.size() * 6, intermediate_trajectory.size() * 6);
    AtPAndt.setZero();
    Eigen::MatrixXd AtPBndt(intermediate_trajectory.size() * 6, 1);
    AtPBndt.setZero();
    Eigen::Vector3d b_indoor(rgd_params_indoor.resolution_X, rgd_params_indoor.resolution_Y, rgd_params_indoor.resolution_Z);

    std::vector<std::mutex> mutexes(intermediate_trajectory.size());

    const auto hessian_fun_indoor = [&](const Point3Di &intermediate_points_i)
    {
        if (intermediate_points_i.point.norm() < 0.1 || intermediate_points_i.point.norm() > max_distance) // ToDo
        {
            return;
        }

        Eigen::Vector3d point_global = intermediate_trajectory[intermediate_points_i.index_pose] * intermediate_points_i.point;
        auto index_of_bucket = get_rgd_index(point_global, b_indoor);

        auto bucket_it = buckets_indoor.find(index_of_bucket);
        // no bucket found
        if (bucket_it == buckets_indoor.end())
        {
            return;
        }
        auto &this_bucket = bucket_it->second;

        // if(buckets[index_of_bucket].number_of_points >= 5){
        const Eigen::Matrix3d &infm = this_bucket.cov.inverse();
        const double threshold = 100000.0;

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
        std::for_each(std::execution::par_unseq, std::begin(intermediate_points), std::end(intermediate_points), hessian_fun_indoor);
    }
    else
    {
        std::for_each(std::begin(intermediate_points), std::end(intermediate_points), hessian_fun_indoor);
    }

    Eigen::Vector3d b_outdoor(rgd_params_outdoor.resolution_X, rgd_params_outdoor.resolution_Y, rgd_params_outdoor.resolution_Z);

    const auto hessian_fun_outdoor = [&](const Point3Di &intermediate_points_i)
    {
        if (intermediate_points_i.point.norm() < 0.1 || intermediate_points_i.point.norm() > max_distance) // ToDo
        {
            return;
        }

        Eigen::Vector3d point_global = intermediate_trajectory[intermediate_points_i.index_pose] * intermediate_points_i.point;
        auto index_of_bucket = get_rgd_index(point_global, b_outdoor);

        auto bucket_it = buckets_outdoor.find(index_of_bucket);
        // no bucket found
        if (bucket_it == buckets_outdoor.end())
        {
            return;
        }
        auto &this_bucket = bucket_it->second;

        // if(buckets[index_of_bucket].number_of_points >= 5){
        const Eigen::Matrix3d &infm = this_bucket.cov.inverse();
        const double threshold = 100000.0;

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

        if (norm < 5.0)
        {
            return;
        }

        AtPA *= planarity;
        AtPB *= planarity;

        std::mutex &m = mutexes[intermediate_points_i.index_pose];
        std::unique_lock lck(m);
        AtPAndt.block<6, 6>(c, c) += AtPA;
        AtPBndt.block<6, 1>(c, 0) -= AtPB;
    };

    if (multithread)
    {
        std::for_each(std::execution::par_unseq, std::begin(intermediate_points), std::end(intermediate_points), hessian_fun_outdoor);
    }
    else
    {
        std::for_each(std::begin(intermediate_points), std::end(intermediate_points), hessian_fun_outdoor);
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

        // relative_pose_measurement_odo(2, 0) += 0.01;

        // Eigen::Vector3d relative(relative_pose_measurement_odo(0, 0), relative_pose_measurement_odo(1, 0), relative_pose_measurement_odo(2, 0));

        // relative_pose_measurement_odo(2, 0) += relative.norm() * sin(poses_desired[odo_edges[i].first].fi);
        //TaitBryanPose relative_pose_measurement_odo
            // relative_pose_measurement_odo(2, 0) += 0.01;
        relative_pose_measurement_odo(3, 0) += (motion_model_correction.om / 180.0) * M_PI;
        relative_pose_measurement_odo(4, 0) += (motion_model_correction.fi / 180.0) * M_PI;
        relative_pose_measurement_odo(5, 0) += (motion_model_correction.ka / 180.0) * M_PI;

        Eigen::Matrix<double, 12, 12>
            AtPAodo;
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
                                                                 // 1000000,
                                                                 //  10000000000,
                                                                 //  10000000000,
                                                                 w_motion_model_x,
                                                                 w_motion_model_y,
                                                                 w_motion_model_z,
                                                                 w_motion_model_om * cauchy(relative_pose_measurement_odo(3, 0), 1),  // 100000000, //
                                                                 w_motion_model_fi * cauchy(relative_pose_measurement_odo(4, 0), 1),  // 100000000, //
                                                                 w_motion_model_ka * cauchy(relative_pose_measurement_odo(5, 0), 1)); // 100000000);
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
                                                                 // 1000000,
                                                                 //  10000000000,
                                                                 //  10000000000,
                                                                 w_motion_model_x,
                                                                 w_motion_model_y,
                                                                 w_motion_model_z,
                                                                 w_motion_model_om * cauchy(relative_pose_measurement_odo(3, 0), 1), // 100000000, //
                                                                 w_motion_model_fi * cauchy(relative_pose_measurement_odo(4, 0), 1), // 100000000, //
                                                                 w_motion_model_ka * cauchy(relative_pose_measurement_odo(5, 0), 1));

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

    int ic = 0;
    int ir = tripletListB.size();
    tripletListA.emplace_back(ir, ic * 6 + 0, 1);
    tripletListA.emplace_back(ir + 1, ic * 6 + 1, 1);
    tripletListA.emplace_back(ir + 2, ic * 6 + 2, 1);
    tripletListA.emplace_back(ir + 3, ic * 6 + 3, 1);
    tripletListA.emplace_back(ir + 4, ic * 6 + 4, 1);
    tripletListA.emplace_back(ir + 5, ic * 6 + 5, 1);

    tripletListP.emplace_back(ir, ir, 1.0 / (lidar_odometry_motion_model_fix_origin_x_1_sigma_m * lidar_odometry_motion_model_fix_origin_x_1_sigma_m));
    tripletListP.emplace_back(ir + 1, ir + 1, 1.0 / (lidar_odometry_motion_model_fix_origin_y_1_sigma_m * lidar_odometry_motion_model_fix_origin_y_1_sigma_m));
    tripletListP.emplace_back(ir + 2, ir + 2, 1.0 / (lidar_odometry_motion_model_fix_origin_z_1_sigma_m * lidar_odometry_motion_model_fix_origin_z_1_sigma_m));
    tripletListP.emplace_back(ir + 3, ir + 3, 1.0 / (lidar_odometry_motion_model_fix_origin_om_1_sigma_deg * lidar_odometry_motion_model_fix_origin_om_1_sigma_deg)); 
    tripletListP.emplace_back(ir + 4, ir + 4, 1.0 / (lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg * lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg)); 
    tripletListP.emplace_back(ir + 5, ir + 5, 1.0 / (lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg * lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg));

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
    AtPA_I *= lm_factor;
    AtPA += (AtPA_I);

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

    delta = 1000000.0;
    if (h_x.size() == 6 * intermediate_trajectory.size())
    {
        int counter = 0;

        for (size_t i = 0; i < intermediate_trajectory.size(); i++)
        {
            TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(intermediate_trajectory[i]);
            TaitBryanPose prev_pose = pose_tait_bryan_from_affine_matrix(intermediate_trajectory_motion_model[i]);
            pose.px += h_x[counter++];
            pose.py += h_x[counter++];
            pose.pz += h_x[counter++];
            pose.om += h_x[counter++];
            pose.fi += h_x[counter++];
            pose.ka += h_x[counter++];

            Eigen::Vector3d p1(prev_pose.px, prev_pose.py, prev_pose.pz);
            Eigen::Vector3d p2(pose.px, pose.py, pose.pz);

            // std::cout << "(p1 - p2).norm() " << (p1 - p2).norm() << std::endl;

            if ((p1 - p2).norm() < 1.0)
            {
                intermediate_trajectory[i] = affine_matrix_from_pose_tait_bryan(pose);
            }
            else
            {
                std::cout << "big jump on trajectory: " << (p1 - p2).norm() << std::endl;
            }
        }
        delta = 0.0;
        for (int i = 0; i < h_x.size(); i++)
        {
            delta += sqrt(h_x[i] * h_x[i]);
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

        Eigen::Vector3d point_global = m_g * initial_points[i].point;
        auto index_of_bucket = get_rgd_index(point_global, b);

        if (!reference_buckets.contains(index_of_bucket))
        {
            continue;
        }

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

//extern nglobals::icpProgress.store((float)i / globals::registeredFrames.size());
bool compute_step_2(std::vector<WorkerData> &worker_data, LidarOdometryParams &params, double &ts_failure, std::atomic<float> &loProgress, const std::atomic<bool> &pause, bool debugMsg)
{
    //exit(1);
    bool debug = false;
    bool debug2 = true;

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
        update_rgd(params.in_out_params_indoor, params.buckets_indoor, pp, params.m_g.translation());
        update_rgd(params.in_out_params_outdoor, params.buckets_outdoor, pp, params.m_g.translation());

        for (int i = 0; i < worker_data.size(); i++)
        {
            std::vector<Point3Di> intermediate_points;
            // = worker_data[i].load_points(worker_data[i].intermediate_points_cache_file_name);
            load_vector_data(worker_data[i].intermediate_points_cache_file_name.string(), intermediate_points);



            if (pause)
            {
                while (pause)
                {
                    std::cout << "pause" << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }

            // auto tmp_trj =
            Eigen::Vector3d mean_shift(0.0, 0.0, 0.0);
            if (i > 1 && params.use_motion_from_previous_step)
            {
                // mean_shift = worker_data[i - 1].intermediate_trajectory[0].translation() - worker_data[i - 2].intermediate_trajectory[worker_data[i - 2].intermediate_trajectory.size() - 1].translation();
                // mean_shift /= ((worker_data[i - 2].intermediate_trajectory.size()) - 2);

                mean_shift = worker_data[i - 1].intermediate_trajectory[worker_data[i - 1].intermediate_trajectory.size() - 1].translation() -
                             worker_data[i - 2].intermediate_trajectory[worker_data[i - 2].intermediate_trajectory.size() - 1].translation();
                // mean_shift = worker_data[i - 1].intermediate_trajectory[0].translation() -
                //               worker_data[i - 2].intermediate_trajectory[0].translation();

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
            }

            bool add_pitch_roll_constraint = false;

            std::chrono::time_point<std::chrono::system_clock> start1, end1;
            start1 = std::chrono::system_clock::now();

            auto tmp_worker_data = worker_data[i].intermediate_trajectory;

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

                std::vector<Point3Di> points_local_sf;
                std::vector<Point3Di> points_local;

                ///
                for (int ii = 0; ii < intermediate_points.size(); ii++)
                {
                    double r_l = intermediate_points[ii].point.norm();

                    // std::cout << worker_data[i].intermediate_points[ii].index_pose << " ";
                    if (r_l > 0.5 && intermediate_points[ii].index_pose != -1 && r_l < params.max_distance_lidar_rigid_sf)
                    {
                        double polar_angle_deg_l = atan2(intermediate_points[ii].point.y(), intermediate_points[ii].point.x()) / M_PI * 180.0;
                        double azimutal_angle_deg_l = acos(intermediate_points[ii].point.z() / r_l) / M_PI * 180.0;

                        points_local.push_back(intermediate_points[ii]);

                        ///////////////////////////////////////////////////////
                        Point3Di p_sl = intermediate_points[ii];
                        p_sl.point.x() = r_l;
                        p_sl.point.y() = polar_angle_deg_l;
                        p_sl.point.z() = azimutal_angle_deg_l;

                        points_local_sf.push_back(p_sl);
                    }
                }
                ///
                std::cout << "optimize_sf2" << std::endl;

                std::vector<Eigen::Vector3d> pointcloud;
                std::vector<unsigned short> intensity;
                std::vector<double> timestamps;

                if (debug)
                {
                    static int index_fn = 0;

                    for (int ii = 0; ii < points_local.size(); ii++)
                    {
                        Eigen::Vector3d pg = points_local[ii].point;
                        pg = tr[points_local[ii].index_pose] * pg;
                        pointcloud.push_back(pg);
                        intensity.push_back(points_local[ii].intensity);
                        timestamps.push_back(0);
                    }
                }

                double wx = 1000000;
                double wy = 1000000;
                double wz = 1000000;
                double angle_sigma_rad = 0.1 / 180.0 * M_PI;
                double wom = 1.0 / (angle_sigma_rad * angle_sigma_rad);
                double wfi = 1.0 / (angle_sigma_rad * angle_sigma_rad);
                double wka = 1.0 / (angle_sigma_rad * angle_sigma_rad);

                for (int iter = 0; iter < params.robust_and_accurate_lidar_odometry_iterations; iter++)
                {
                    optimize_sf2(points_local, points_local_sf, tr, trmm, rgd_params_sc, params.useMultithread, wx, wy, wz, wom, wfi, wka);
                }

                if (debug)
                {
                    static int index_fn = 0;

                    for (int i = 0; i < points_local.size(); i++)
                    {
                        Eigen::Vector3d pg = points_local[i].point;
                        pg = tr[points_local[i].index_pose] * pg;
                        pointcloud.push_back(pg);
                        intensity.push_back(points_local[i].intensity);
                        timestamps.push_back(1);
                    }

                    std::string output_file_name = "optimize_sf2_" + std::to_string(index_fn++) + ".laz";

                    if (!exportLaz(output_file_name, pointcloud, intensity, timestamps, 0, 0, 0))
                    {
                        std::cout << "problem with saving file: " << output_file_name << std::endl;
                    }
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

                optimize_rigid_sf(intermediate_points,
                                  worker_data[i].intermediate_trajectory,
                                  worker_data[i].intermediate_trajectory_motion_model,
                                  params.buckets_indoor,
                                  params.distance_bucket_rigid_sf,
                                  params.polar_angle_deg_rigid_sf,
                                  params.azimutal_angle_deg_rigid_sf,
                                  params.robust_and_accurate_lidar_odometry_rigid_sf_iterations,
                                  params.max_distance_lidar_rigid_sf,
                                  params.useMultithread,
                                  params.rgd_sf_sigma_x_m,
                                  params.rgd_sf_sigma_y_m,
                                  params.rgd_sf_sigma_z_m,
                                  params.rgd_sf_sigma_om_deg,
                                  params.rgd_sf_sigma_fi_deg,
                                  params.rgd_sf_sigma_ka_deg);
            }

            worker_data[i].intermediate_trajectory_motion_model = worker_data[i].intermediate_trajectory;

            double delta = 100000.0;
            double lm_factor = 1.0;

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();

            for (int iter = 0; iter < params.nr_iter; iter++)
            {

                delta = 100000.0;
                optimize_lidar_odometry(intermediate_points, worker_data[i].intermediate_trajectory, worker_data[i].intermediate_trajectory_motion_model,
                                        params.in_out_params_indoor, params.buckets_indoor,
                                        params.in_out_params_outdoor, params.buckets_outdoor,
                                        params.useMultithread, params.max_distance_lidar, delta, /*add_pitch_roll_constraint, worker_data[i].imu_roll_pitch,*/
                                        lm_factor,
                                        params.motion_model_correction,
                                        params.lidar_odometry_motion_model_x_1_sigma_m,
                                        params.lidar_odometry_motion_model_y_1_sigma_m,
                                        params.lidar_odometry_motion_model_z_1_sigma_m,
                                        params.lidar_odometry_motion_model_om_1_sigma_deg,
                                        params.lidar_odometry_motion_model_fi_1_sigma_deg,
                                        params.lidar_odometry_motion_model_ka_1_sigma_deg,
                                        params.lidar_odometry_motion_model_fix_origin_x_1_sigma_m,
                                        params.lidar_odometry_motion_model_fix_origin_y_1_sigma_m,
                                        params.lidar_odometry_motion_model_fix_origin_z_1_sigma_m,
                                        params.lidar_odometry_motion_model_fix_origin_om_1_sigma_deg,
                                        params.lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg,
                                        params.lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg);
                if (delta < 1e-12)
                {
                    std::cout << "finished at iteration " << iter + 1 << "/" << params.nr_iter;
                    break;
                }

                if (iter % 10 == 0 && iter > 0)
                {
                    if (debugMsg)
                    {
                        std::cout << "\nlm_factor " << lm_factor << ", delta " << std::setprecision(10) << delta << "\n";
                    }

                    lm_factor *= 10.0;
                }

                end = std::chrono::system_clock::now();
                std::chrono::duration<double> elapsed_seconds = end - start;

                if (elapsed_seconds.count() > params.real_time_threshold_seconds){
                    break;
                }
            }

            end1 = std::chrono::system_clock::now();

            std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
            std::cout << " optimizing worker_data " << i + 1 << "/" << worker_data.size()
                << " with acc_distance " << fixed << std::setprecision(2) << acc_distance << "[m] in "
                << fixed << std::setprecision(2) << elapsed_seconds1.count()
                << "[s], delta ";
            if (delta > 1e-12)
            {
                std::cout << std::setprecision(10) << delta << "!!!";
            }
            else
            {
                std::cout << "< 1e-12";
            }

            std::cout << "\n";

            loProgress.store((float)(i + 1) / worker_data.size());

            // temp save
            if (i % 100 == 0)
            {
                std::vector<Eigen::Vector3d> global_pointcloud;
                std::vector<unsigned short> intensity; 
                std::vector<double> timestamps;
                points_to_vector(
                    intermediate_points, worker_data[i].intermediate_trajectory,
                    0, nullptr, global_pointcloud, intensity, timestamps, false
                );
                std::string fn = params.working_directory_preview + "/temp_point_cloud_" + std::to_string(i) + ".laz";
                exportLaz(fn.c_str(), global_pointcloud, intensity, timestamps);
            }
            auto acc_distance_tmp = acc_distance;
            acc_distance += ((worker_data[i].intermediate_trajectory[0].inverse()) *
                             worker_data[i].intermediate_trajectory[worker_data[i].intermediate_trajectory.size() - 1])
                                .translation()
                                .norm();

			if (!(acc_distance == acc_distance)) //NaN check
            {
                worker_data[i].intermediate_trajectory = tmp_worker_data;
                std::cout << "CHALLENGING DATA OCCURED!!!" << std::endl;
                acc_distance = acc_distance_tmp;
                std::cout << "please split data set into subsets" << std::endl;
                ts_failure = worker_data[i].intermediate_trajectory_timestamps[0].first;
                std::cout << "calculations canceled for TIMESTAMP: " << (long long int)worker_data[i].intermediate_trajectory_timestamps[0].first << std::endl;
                return false;
            }

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

            for (int j = 0; j < intermediate_points.size(); j++)
            {
                Point3Di pp = intermediate_points[j];
                pp.point = worker_data[i].intermediate_trajectory[intermediate_points[j].index_pose] * pp.point;
                points_global.push_back(pp);
            }

            // if(reference_points.size() == 0){
            if (acc_distance > params.sliding_window_trajectory_length_threshold)
            {
                std::chrono::time_point<std::chrono::system_clock> startu, endu;
                startu = std::chrono::system_clock::now();

                if (params.reference_points.size() == 0)
                {
                    params.buckets_indoor.clear();
                    params.buckets_outdoor.clear();
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
                update_rgd(params.in_out_params_indoor, params.buckets_indoor, points_global, worker_data[i].intermediate_trajectory[0].translation());
                update_rgd(params.in_out_params_outdoor, params.buckets_outdoor, points_global, worker_data[i].intermediate_trajectory[0].translation());
                //
                endu = std::chrono::system_clock::now();

                std::chrono::duration<double> elapsed_secondsu = endu - startu;
                std::time_t end_timeu = std::chrono::system_clock::to_time_t(endu);

                //std::cout << "finished computation at " << std::ctime(&end_timeu)
                //          << "elapsed time update: " << std::setprecision(0) << elapsed_secondsu.count() << "s\n";
            }
            else
            {

                std::vector<Point3Di> pg;
                for (int j = 0; j < intermediate_points.size(); j++)
                {
                    Point3Di pp = intermediate_points[j];
                    pp.point = worker_data[i].intermediate_trajectory[intermediate_points[j].index_pose] * pp.point;
                    pg.push_back(pp);
                }
                update_rgd(params.in_out_params_indoor, params.buckets_indoor, pg, worker_data[i].intermediate_trajectory[0].translation());
                update_rgd(params.in_out_params_outdoor, params.buckets_outdoor, pg, worker_data[i].intermediate_trajectory[0].translation());
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

        std::tm local_tm = *std::localtime(&end_time);
        std::cout << "finished computation at " << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S")
            << ", elapsed time: " << std::setprecision(2) << elapsed_seconds.count() << "s\n";

        params.total_length_of_calculated_trajectory = 0;
        for (int i = 1; i < worker_data.size(); i++)
        {
            params.total_length_of_calculated_trajectory += (worker_data[i].intermediate_trajectory[0].translation() - worker_data[i - 1].intermediate_trajectory[0].translation()).norm();
        }
        std::cout << "total_length_of_calculated_trajectory: " << params.total_length_of_calculated_trajectory << " [m]" << std::endl;
    }

    return true;
}

void compute_step_2_fast_forward_motion(std::vector<WorkerData> &worker_data, LidarOdometryParams &params)
{
}

void Consistency(std::vector<WorkerData> &worker_data, LidarOdometryParams &params)
{
    std::vector<Eigen::Affine3d> trajectory;
    std::vector<Eigen::Affine3d> trajectory_motion_model;
    std::vector<std::pair<double, double>> timestamps;
    std::vector<Point3Di> all_points_local;

    std::vector<std::pair<int, int>> indexes;
    bool multithread = false;

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
        std::vector<Point3Di> intermediate_points;

        if (!load_vector_data(worker_data[i].intermediate_points_cache_file_name.string(), intermediate_points))
        {
            std::cout << "problem with load_vector_data file '" << worker_data[i].intermediate_points_cache_file_name.string() << "'" << std::endl;
        }

        for (int j = 0; j < intermediate_points.size(); j++)
        {
            all_points_local.push_back(intermediate_points[j]);
        }
    }

    std::vector<Eigen::Triplet<double>> tripletListA;
    std::vector<Eigen::Triplet<double>> tripletListP;
    std::vector<Eigen::Triplet<double>> tripletListB;

    Eigen::Vector3d b(params.in_out_params_indoor.resolution_X, params.in_out_params_indoor.resolution_Y, params.in_out_params_indoor.resolution_Z);

    std::vector<std::mutex> my_mutex(1);

    std::vector<std::pair<int, int>> odo_edges;
    for (size_t i = 1; i < trajectory.size(); i++)
    {
        odo_edges.emplace_back(i - 1, i);
    }

    std::vector<TaitBryanPose> poses;

    for (size_t i = 0; i < trajectory.size(); i++)
    {
        poses.push_back(pose_tait_bryan_from_affine_matrix(trajectory[i]));
    }

    double angle = 0.1 / 180.0 * M_PI;
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

        const auto ir = tripletListB.size();

        const auto ic_1 = (i - 1) * 6;
        const auto ic_2 = i * 6;
        const auto ic_3 = (i + 1) * 6;

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
                int index_pose = std::distance(timestamps.begin(), lower) - 1;
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
            update_rgd(params.in_out_params_indoor, buckets, points_global, trajectory[points_global[0].index_pose].translation());

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
                const double threshold = 100000.0;

                if ((infm.array() > threshold).any())
                {
                    return;
                }
                if ((infm.array() < -threshold).any())
                {
                    return;
                }

                const Eigen::Affine3d &m_pose = trajectory[intermediate_points_i.index_pose];
                const Eigen::Vector3d &p_s = intermediate_points_i.point;
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
        std::vector<Point3Di> intermediate_points;
        if (!load_vector_data(worker_data[i].intermediate_points_cache_file_name.string(), intermediate_points))
        {
            std::cout << "problem with load_vector_data for file '" << worker_data[i].intermediate_points_cache_file_name.string() << "'" << std::endl;
        }

        for (int j = 0; j < intermediate_points.size(); j++)
        {
            all_points_local.push_back(intermediate_points[j]);
        }
    }

    std::vector<Eigen::Triplet<double>> tripletListA;
    std::vector<Eigen::Triplet<double>> tripletListP;
    std::vector<Eigen::Triplet<double>> tripletListB;

    Eigen::Vector3d b(params.in_out_params_indoor.resolution_X, params.in_out_params_indoor.resolution_Y, params.in_out_params_indoor.resolution_Z);

    std::vector<std::mutex> my_mutex(1);

    std::vector<std::pair<int, int>> odo_edges;
    for (size_t i = 1; i < trajectory.size(); i++)
    {
        odo_edges.emplace_back(i - 1, i);
    }

    std::vector<TaitBryanPose> poses;

    for (size_t i = 0; i < trajectory.size(); i++)
    {
        poses.push_back(pose_tait_bryan_from_affine_matrix(trajectory[i]));
    }

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
            int index_pose = std::distance(timestamps.begin(), lower) - 1;
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

    for (auto &b : buckets)
    {
        for (int i = b.second.index_begin_inclusive; i < b.second.index_end_exclusive; i++)
        {
            long long unsigned int index_point_segment = points_global[index_pairs[i].second].index_point;

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
                const double threshold = 100000.0;

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