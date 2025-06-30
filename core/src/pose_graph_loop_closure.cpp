#include <icp.h>
#include <m_estimators.h>
#include <control_points.h>
#include <transformations.h>
#include <common/include/cauchy.h>
#include <ground_control_points.h>
#include <pose_graph_loop_closure.h>
#include <pair_wise_iterative_closest_point.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_cw_jacobian.h>
#include <python-scripts/point-to-feature-metrics/point_to_line_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian.h>

void PoseGraphLoopClosure::add_edge(PointClouds &point_clouds_container, 
    int index_loop_closure_source, int index_loop_closure_target)
{
    Edge edge;
    edge.index_from = index_loop_closure_source;
    edge.index_to = index_loop_closure_target;
    //

    edge.relative_pose_tb = pose_tait_bryan_from_affine_matrix(
        point_clouds_container.point_clouds[index_loop_closure_source].m_pose.inverse() *
        point_clouds_container.point_clouds[index_loop_closure_target].m_pose);

    edge.relative_pose_tb_weights.px = 1000000.0;
    edge.relative_pose_tb_weights.py = 1000000.0;
    edge.relative_pose_tb_weights.pz = 1000000.0;
    edge.relative_pose_tb_weights.om = 1000000.0;
    edge.relative_pose_tb_weights.fi = 1000000.0;
    edge.relative_pose_tb_weights.ka = 1000000.0;

    edges.push_back(edge);
}

void PoseGraphLoopClosure::set_initial_poses_as_motion_model(PointClouds &point_clouds_container)
{
    poses_motion_model.clear();
    for (auto &pc : point_clouds_container.point_clouds)
    {
        poses_motion_model.push_back(pc.m_initial_pose);
    }
    std::cout << "Set initial poses as motion model DONE" << std::endl;
}

void PoseGraphLoopClosure::set_current_poses_as_motion_model(PointClouds &point_clouds_container)
{
    poses_motion_model.clear();
    for (auto &pc : point_clouds_container.point_clouds)
    {
        poses_motion_model.push_back(pc.m_pose);
    }
    std::cout << "Set current result as motion model DONE" << std::endl;
}

void PoseGraphLoopClosure::graph_slam(PointClouds &point_clouds_container, GNSS &gnss, GroundControlPoints &gcps, ControlPoints &cps)
{
    std::cout << "Compute Pose Graph SLAM" << std::endl;

    ///////////////////////////////////////////////////////////////////
    // graph slam
    bool is_ok = true;
    std::vector<Eigen::Affine3d> m_poses;
    for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
    {
        m_poses.push_back(point_clouds_container.point_clouds[i].m_pose);
    }

    std::vector<TaitBryanPose> poses;

    bool is_wc = true;
    bool is_cw = false;
    int iterations = 10;
    bool is_fix_first_node = true;
    if (gnss.gnss_poses.size() > 0)
    {
        is_fix_first_node = false;
    }

    for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
    {
        if (is_wc)
        {
            poses.push_back(pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose));
        }
        else if (is_cw)
        {
            poses.push_back(pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose.inverse()));
        }
    }

    std::vector<Edge> all_edges;

    // motion model;
    for (int i = 1; i < poses_motion_model.size(); i++)
    {
        Eigen::Affine3d m_rel = poses_motion_model[i - 1].inverse() * poses_motion_model[i];
        Edge edge;
        edge.index_from = i - 1;
        edge.index_to = i;
        edge.relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_rel);
        edge.relative_pose_tb_weights.om = 10000.0;
        edge.relative_pose_tb_weights.fi = 10000.0;
        edge.relative_pose_tb_weights.ka = 10000.0;
        edge.relative_pose_tb_weights.px = 100.0;
        edge.relative_pose_tb_weights.py = 10000.0;
        edge.relative_pose_tb_weights.pz = 10000.0;
        all_edges.push_back(edge);
    }

    for (int i = 0; i < edges.size(); i++)
    {
        all_edges.push_back(edges[i]);
    }

    for (int iter = 0; iter < iterations; iter++)
    {
        std::cout << "iteration " << iter + 1 << " of " << iterations << std::endl;
        std::vector<Eigen::Triplet<double>> tripletListA;
        std::vector<Eigen::Triplet<double>> tripletListP;
        std::vector<Eigen::Triplet<double>> tripletListB;

        for (size_t i = 0; i < all_edges.size(); i++)
        {
            Eigen::Matrix<double, 6, 1> delta;
            Eigen::Matrix<double, 6, 12, Eigen::RowMajor> jacobian;
            // auto relative_pose = pose_tait_bryan_from_affine_matrix(all_edges[i].relative_pose_tb);
            if (is_wc)
            {
                relative_pose_obs_eq_tait_bryan_wc_case1(
                    delta,
                    poses[all_edges[i].index_from].px,
                    poses[all_edges[i].index_from].py,
                    poses[all_edges[i].index_from].pz,
                    normalize_angle(poses[all_edges[i].index_from].om),
                    normalize_angle(poses[all_edges[i].index_from].fi),
                    normalize_angle(poses[all_edges[i].index_from].ka),
                    poses[all_edges[i].index_to].px,
                    poses[all_edges[i].index_to].py,
                    poses[all_edges[i].index_to].pz,
                    normalize_angle(poses[all_edges[i].index_to].om),
                    normalize_angle(poses[all_edges[i].index_to].fi),
                    normalize_angle(poses[all_edges[i].index_to].ka),
                    all_edges[i].relative_pose_tb.px,
                    all_edges[i].relative_pose_tb.py,
                    all_edges[i].relative_pose_tb.pz,
                    normalize_angle(all_edges[i].relative_pose_tb.om),
                    normalize_angle(all_edges[i].relative_pose_tb.fi),
                    normalize_angle(all_edges[i].relative_pose_tb.ka));
                relative_pose_obs_eq_tait_bryan_wc_case1_jacobian(jacobian,
                                                                  poses[all_edges[i].index_from].px,
                                                                  poses[all_edges[i].index_from].py,
                                                                  poses[all_edges[i].index_from].pz,
                                                                  normalize_angle(poses[all_edges[i].index_from].om),
                                                                  normalize_angle(poses[all_edges[i].index_from].fi),
                                                                  normalize_angle(poses[all_edges[i].index_from].ka),
                                                                  poses[all_edges[i].index_to].px,
                                                                  poses[all_edges[i].index_to].py,
                                                                  poses[all_edges[i].index_to].pz,
                                                                  normalize_angle(poses[all_edges[i].index_to].om),
                                                                  normalize_angle(poses[all_edges[i].index_to].fi),
                                                                  normalize_angle(poses[all_edges[i].index_to].ka));
            }
            else if (is_cw)
            {
                relative_pose_obs_eq_tait_bryan_cw_case1(
                    delta,
                    poses[all_edges[i].index_from].px,
                    poses[all_edges[i].index_from].py,
                    poses[all_edges[i].index_from].pz,
                    normalize_angle(poses[all_edges[i].index_from].om),
                    normalize_angle(poses[all_edges[i].index_from].fi),
                    normalize_angle(poses[all_edges[i].index_from].ka),
                    poses[all_edges[i].index_to].px,
                    poses[all_edges[i].index_to].py,
                    poses[all_edges[i].index_to].pz,
                    normalize_angle(poses[all_edges[i].index_to].om),
                    normalize_angle(poses[all_edges[i].index_to].fi),
                    normalize_angle(poses[all_edges[i].index_to].ka),
                    all_edges[i].relative_pose_tb.px,
                    all_edges[i].relative_pose_tb.py,
                    all_edges[i].relative_pose_tb.pz,
                    normalize_angle(all_edges[i].relative_pose_tb.om),
                    normalize_angle(all_edges[i].relative_pose_tb.fi),
                    normalize_angle(all_edges[i].relative_pose_tb.ka));
                relative_pose_obs_eq_tait_bryan_cw_case1_jacobian(jacobian,
                                                                  poses[all_edges[i].index_from].px,
                                                                  poses[all_edges[i].index_from].py,
                                                                  poses[all_edges[i].index_from].pz,
                                                                  normalize_angle(poses[all_edges[i].index_from].om),
                                                                  normalize_angle(poses[all_edges[i].index_from].fi),
                                                                  normalize_angle(poses[all_edges[i].index_from].ka),
                                                                  poses[all_edges[i].index_to].px,
                                                                  poses[all_edges[i].index_to].py,
                                                                  poses[all_edges[i].index_to].pz,
                                                                  normalize_angle(poses[all_edges[i].index_to].om),
                                                                  normalize_angle(poses[all_edges[i].index_to].fi),
                                                                  normalize_angle(poses[all_edges[i].index_to].ka));
            }

            int ir = tripletListB.size();

            int ic_1 = all_edges[i].index_from * 6;
            int ic_2 = all_edges[i].index_to * 6;

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
            tripletListB.emplace_back(ir + 3, 0, normalize_angle(delta(3, 0)));
            tripletListB.emplace_back(ir + 4, 0, normalize_angle(delta(4, 0)));
            tripletListB.emplace_back(ir + 5, 0, normalize_angle(delta(5, 0)));

            tripletListP.emplace_back(ir, ir, all_edges[i].relative_pose_tb_weights.px * cauchy((delta(0, 0)), 1));
            tripletListP.emplace_back(ir + 1, ir + 1, all_edges[i].relative_pose_tb_weights.py * cauchy((delta(1, 0)), 1));
            tripletListP.emplace_back(ir + 2, ir + 2, all_edges[i].relative_pose_tb_weights.pz * cauchy((delta(2, 0)), 1));
            tripletListP.emplace_back(ir + 3, ir + 3, all_edges[i].relative_pose_tb_weights.om * cauchy(normalize_angle(delta(3, 0)), 1));
            tripletListP.emplace_back(ir + 4, ir + 4, all_edges[i].relative_pose_tb_weights.fi * cauchy(normalize_angle(delta(4, 0)), 1));
            tripletListP.emplace_back(ir + 5, ir + 5, all_edges[i].relative_pose_tb_weights.ka * cauchy(normalize_angle(delta(5, 0)), 1));
            
        }

        if (gcps.gpcs.size() == 0)
        {
            if (is_fix_first_node)
            {
                int ir = tripletListB.size();
                tripletListA.emplace_back(ir, 0, 1);
                tripletListA.emplace_back(ir + 1, 1, 1);
                tripletListA.emplace_back(ir + 2, 2, 1);
                tripletListA.emplace_back(ir + 3, 3, 1);
                tripletListA.emplace_back(ir + 4, 4, 1);
                tripletListA.emplace_back(ir + 5, 5, 1);

                tripletListP.emplace_back(ir, ir, 1);
                tripletListP.emplace_back(ir + 1, ir + 1, 1);
                tripletListP.emplace_back(ir + 2, ir + 2, 1);
                tripletListP.emplace_back(ir + 3, ir + 3, 1);
                tripletListP.emplace_back(ir + 4, ir + 4, 1);
                tripletListP.emplace_back(ir + 5, ir + 5, 1);

                tripletListB.emplace_back(ir, 0, 0);
                tripletListB.emplace_back(ir + 1, 0, 0);
                tripletListB.emplace_back(ir + 2, 0, 0);
                tripletListB.emplace_back(ir + 3, 0, 0);
                tripletListB.emplace_back(ir + 4, 0, 0);
                tripletListB.emplace_back(ir + 5, 0, 0);
            }
        }

        // gnss
        // for (const auto &pc : point_clouds_container.point_clouds)
        for (int index_pose = 0; index_pose < point_clouds_container.point_clouds.size(); index_pose++)
        {
            const auto &pc = point_clouds_container.point_clouds[index_pose];
            for (int i = 0; i < gnss.gnss_poses.size(); i++)
            {
                double time_stamp = gnss.gnss_poses[i].timestamp;

                auto it = std::lower_bound(pc.local_trajectory.begin(), pc.local_trajectory.end(),
                                           time_stamp, [](const PointCloud::LocalTrajectoryNode &lhs, const double &time) -> bool
                                           { return lhs.timestamps.first < time; });

                int index = it - pc.local_trajectory.begin();

                if (index > 0 && index < pc.local_trajectory.size())
                {

                    if (fabs(time_stamp - pc.local_trajectory[index].timestamps.first) < 10e12)
                    {

                        Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
                        TaitBryanPose pose_s;
                        pose_s = pose_tait_bryan_from_affine_matrix(m_poses[index_pose]);
                        Eigen::Vector3d p_s = pc.local_trajectory[index].m_pose.translation();
                        point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian, pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                                               p_s.x(), p_s.y(), p_s.z());

                        double delta_x;
                        double delta_y;
                        double delta_z;
                        // Eigen::Vector3d p_t(gnss.gnss_poses[i].x - gnss.offset_x, gnss.gnss_poses[i].y - gnss.offset_y, gnss.gnss_poses[i].alt - gnss.offset_alt);
                        Eigen::Vector3d p_t(gnss.gnss_poses[i].x - point_clouds_container.offset.x(), gnss.gnss_poses[i].y - point_clouds_container.offset.y(), gnss.gnss_poses[i].alt - point_clouds_container.offset.z());

                        point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z,
                                                                      pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                                      p_s.x(), p_s.y(), p_s.z(), p_t.x(), p_t.y(), p_t.z());

                        // std::cout << " delta_x " << delta_x << " delta_y " << delta_y << " delta_z " << delta_z << std::endl;

                        int ir = tripletListB.size();
                        int ic = index_pose * 6;
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
                        tripletListP.emplace_back(ir, ir, get_cauchy_w(delta_x, 1) * 0.01);
                        tripletListP.emplace_back(ir + 1, ir + 1, get_cauchy_w(delta_y, 1) * 0.01);
                        tripletListP.emplace_back(ir + 2, ir + 2, get_cauchy_w(delta_z, 1) * 0.01);

                        tripletListB.emplace_back(ir, 0, delta_x);
                        tripletListB.emplace_back(ir + 1, 0, delta_y);
                        tripletListB.emplace_back(ir + 2, 0, delta_z);

                    }
                }
            }
        }
        //

        // GCPs
        for (int i = 0; i < gcps.gpcs.size(); i++)
        {
            Eigen::Vector3d p_s = point_clouds_container.point_clouds[gcps.gpcs[i].index_to_node_inner].local_trajectory[gcps.gpcs[i].index_to_node_outer].m_pose.translation();

            Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
            TaitBryanPose pose_s;
            pose_s = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[gcps.gpcs[i].index_to_node_inner].m_pose);

            point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian, pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                                   p_s.x(), p_s.y(), p_s.z());

            double delta_x;
            double delta_y;
            double delta_z;
            Eigen::Vector3d p_t(gcps.gpcs[i].x, gcps.gpcs[i].y, gcps.gpcs[i].z + gcps.gpcs[i].lidar_height_above_ground);
            point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z,
                                                          pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                          p_s.x(), p_s.y(), p_s.z(), p_t.x(), p_t.y(), p_t.z());

            int ir = tripletListB.size();
            int ic = gcps.gpcs[i].index_to_node_inner * 6;

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
            tripletListP.emplace_back(ir + 0, ir + 0, (1.0 / (gcps.gpcs[i].sigma_x * gcps.gpcs[i].sigma_x)) * get_cauchy_w(delta_x, 1));
            tripletListP.emplace_back(ir + 1, ir + 1, (1.0 / (gcps.gpcs[i].sigma_y * gcps.gpcs[i].sigma_y)) * get_cauchy_w(delta_y, 1));
            tripletListP.emplace_back(ir + 2, ir + 2, (1.0 / (gcps.gpcs[i].sigma_z * gcps.gpcs[i].sigma_z)) * get_cauchy_w(delta_z, 1));

            tripletListB.emplace_back(ir, 0, delta_x);
            tripletListB.emplace_back(ir + 1, 0, delta_y);
            tripletListB.emplace_back(ir + 2, 0, delta_z);

            std::cout << "gcp: delta_x " << delta_x << " delta_y " << delta_y << " delta_z " << delta_z << std::endl;
        }

        // CPs
        for (int i = 0; i < cps.cps.size(); i++)
        {
            Eigen::Vector3d p_s(cps.cps[i].x_source_local,
                                cps.cps[i].y_source_local, cps.cps[i].z_source_local);

            Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
            TaitBryanPose pose_s;
            pose_s = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[cps.cps[i].index_to_pose].m_pose);

            point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian, pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                                   p_s.x(), p_s.y(), p_s.z());

            double delta_x;
            double delta_y;
            double delta_z;
            Eigen::Vector3d p_t(cps.cps[i].x_target_global,
                                cps.cps[i].y_target_global, cps.cps[i].z_target_global);
            point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z,
                                                          pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                          p_s.x(), p_s.y(), p_s.z(), p_t.x(), p_t.y(), p_t.z());

            int ir = tripletListB.size();
            int ic = cps.cps[i].index_to_pose * 6;

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
            tripletListP.emplace_back(ir + 0, ir + 0, (1.0 / (cps.cps[i].sigma_x * cps.cps[i].sigma_x)) * get_cauchy_w(delta_x, 1));
            tripletListP.emplace_back(ir + 1, ir + 1, (1.0 / (cps.cps[i].sigma_y * cps.cps[i].sigma_y)) * get_cauchy_w(delta_y, 1));
            tripletListP.emplace_back(ir + 2, ir + 2, (1.0 / (cps.cps[i].sigma_z * cps.cps[i].sigma_z)) * get_cauchy_w(delta_z, 1));

            tripletListB.emplace_back(ir, 0, delta_x);
            tripletListB.emplace_back(ir + 1, 0, delta_y);
            tripletListB.emplace_back(ir + 2, 0, delta_z);

            std::cout << "cp: delta_x " << delta_x << " delta_y " << delta_y << " delta_z " << delta_z << std::endl;
        }

        // fuse_inclination_from_imu
        // if (fuse_inclination_from_imu)
        //{
        //
        double error_imu = 0;
        double error_imu_sum = 0;

        for (int index_pose = 0; index_pose < point_clouds_container.point_clouds.size(); index_pose++)
        {

            const auto &pc = point_clouds_container.point_clouds[index_pose];
            if (!pc.fuse_inclination_from_IMU)
            {
                continue;
            }
            if (pc.local_trajectory.size() == 0)
            {
                continue;
            }
        }

        if (error_imu_sum > 0)
        {
            std::cout << "------------------------------" << std::endl;
            std::cout << "error imu: " << error_imu / error_imu_sum << std::endl;
        }

        Eigen::SparseMatrix<double>
            matA(tripletListB.size(), point_clouds_container.point_clouds.size() * 6);
        Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
        Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

        matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
        matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
        matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

        Eigen::SparseMatrix<double> AtPA(point_clouds_container.point_clouds.size() * 6, point_clouds_container.point_clouds.size() * 6);
        Eigen::SparseMatrix<double> AtPB(point_clouds_container.point_clouds.size() * 6, 1);

        {
            Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
            AtPA = (AtP)*matA;
            AtPB = (AtP)*matB;
        }

        tripletListA.clear();
        tripletListP.clear();
        tripletListB.clear();

        std::cout << "AtPA.size: " << AtPA.size() << std::endl;
        std::cout << "AtPB.size: " << AtPB.size() << std::endl;

        std::cout << "start solving AtPA=AtPB" << std::endl;
        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);

        std::cout << "x = solver.solve(AtPB)" << std::endl;
        Eigen::SparseMatrix<double> x = solver.solve(AtPB);

        std::vector<double> h_x;

        for (int k = 0; k < x.outerSize(); ++k)
        {
            for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            {
                h_x.push_back(it.value());
            }
        }

        std::cout << "h_x.size(): " << h_x.size() << std::endl;

        std::cout << "AtPA=AtPB SOLVED" << std::endl;

        if (h_x.size() == 6 * point_clouds_container.point_clouds.size())
        {
            int counter = 0;

            for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
            {
                TaitBryanPose pose = poses[i];
                poses[i].px += h_x[counter++];
                poses[i].py += h_x[counter++];
                poses[i].pz += h_x[counter++];
                poses[i].om += h_x[counter++];
                poses[i].fi += h_x[counter++];
                poses[i].ka += h_x[counter++];
            }
            std::cout << "optimizing with tait bryan finished" << std::endl;
        }
        else
        {
            std::cout << "optimizing with tait bryan FAILED" << std::endl;
            std::cout << "h_x.size(): " << h_x.size() << " should be: " << 6 * point_clouds_container.point_clouds.size() << std::endl;
            is_ok = false;
            break;
        }

        if (is_ok)
        {
            for (size_t i = 0; i < m_poses.size(); i++)
            {
                if (is_wc)
                {
                    m_poses[i] = affine_matrix_from_pose_tait_bryan(poses[i]);
                }
                else if (is_cw)
                {
                    m_poses[i] = affine_matrix_from_pose_tait_bryan(poses[i]).inverse();
                }
            }
        }

        if (is_ok)
        {
            for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
            {
                point_clouds_container.point_clouds[i].m_pose = m_poses[i];
                point_clouds_container.point_clouds[i].pose = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);
                point_clouds_container.point_clouds[i].gui_translation[0] = point_clouds_container.point_clouds[i].pose.px;
                point_clouds_container.point_clouds[i].gui_translation[1] = point_clouds_container.point_clouds[i].pose.py;
                point_clouds_container.point_clouds[i].gui_translation[2] = point_clouds_container.point_clouds[i].pose.pz;
                point_clouds_container.point_clouds[i].gui_rotation[0] = rad2deg(point_clouds_container.point_clouds[i].pose.om);
                point_clouds_container.point_clouds[i].gui_rotation[1] = rad2deg(point_clouds_container.point_clouds[i].pose.fi);
                point_clouds_container.point_clouds[i].gui_rotation[2] = rad2deg(point_clouds_container.point_clouds[i].pose.ka);
            }
        }
    }

    ///////////////////////////////////////////////////////////////////
    // GCP report
    std::cout << "----------------------------" << std::endl;
    std::cout << "Ground control points report" << std::endl;
    for (int i = 0; i < gcps.gpcs.size(); i++)
    {
        std::cout << "--" << std::endl;

        Eigen::Vector3d p_s = point_clouds_container.point_clouds[gcps.gpcs[i].index_to_node_inner].local_trajectory[gcps.gpcs[i].index_to_node_outer].m_pose.translation();
        TaitBryanPose pose_s;
        pose_s = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[gcps.gpcs[i].index_to_node_inner].m_pose);

        double delta_x;
        double delta_y;
        double delta_z;
        Eigen::Vector3d p_t(gcps.gpcs[i].x, gcps.gpcs[i].y, gcps.gpcs[i].z + gcps.gpcs[i].lidar_height_above_ground);
        point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z,
                                                      pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                      p_s.x(), p_s.y(), p_s.z(), p_t.x(), p_t.y(), p_t.z());

        std::cout << "GCP[" << i << "] name: '" << gcps.gpcs[i].name << "'" << std::endl;
        std::cout << "lidar_height_above_ground: " << gcps.gpcs[i].lidar_height_above_ground << " " << std::endl;

        std::cout << "delta_x: " << delta_x << " [m], delta_y: " << delta_y << " [m], delta_z: " << delta_z << " [m]" << std::endl;
        std::cout << "GCP->                              x:" << gcps.gpcs[i].x << " [m], y:" << gcps.gpcs[i].y << " [m], z:" << gcps.gpcs[i].z << " [m]" << std::endl;

        Eigen::Vector3d pp = point_clouds_container.point_clouds[gcps.gpcs[i].index_to_node_inner].m_pose * p_s;
        std::cout << "TrajectoryNode (center of LiDAR)-> x:" << pp.x() << " [m], y:" << pp.y() << " [m], z:" << pp.z() << " [m]" << std::endl;
        std::cout << "--" << std::endl;
    }
    std::cout << "Compute Pose Graph SLAM FINISHED" << std::endl;
}

void PoseGraphLoopClosure::FuseTrajectoryWithGNSS(PointClouds &point_clouds_container, GNSS &gnss)
{
    for (int iter = 0; iter < 30; iter++)
    {

        std::vector<Eigen::Triplet<double>> tripletListA;
        std::vector<Eigen::Triplet<double>> tripletListP;
        std::vector<Eigen::Triplet<double>> tripletListB;
        Eigen::Affine3d m_pose = Eigen::Affine3d::Identity();
        bool is_ok = true;

        for (int index_pose = 0; index_pose < point_clouds_container.point_clouds.size(); index_pose++)
        {
            const auto &pc = point_clouds_container.point_clouds[index_pose];

            double time_stamp = pc.timestamps[0];

            auto it = std::lower_bound(gnss.gnss_poses.begin(), gnss.gnss_poses.end(),
                                       time_stamp, [](const GNSS::GlobalPose &lhs, const double &time) -> bool
                                       { return lhs.timestamp < time; });

            int index = it - gnss.gnss_poses.begin() - 1;

            if (index > 0 && index < gnss.gnss_poses.size())
            {
                if (fabs(time_stamp - gnss.gnss_poses[index].timestamp) < 10e12)
                {
                    Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
                    TaitBryanPose pose_s;
                    pose_s = pose_tait_bryan_from_affine_matrix(m_pose);
                    Eigen::Vector3d p_s = pc.m_pose.translation();
                    point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian, pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                                           p_s.x(), p_s.y(), p_s.z());

                    double delta_x;
                    double delta_y;
                    double delta_z;
                    Eigen::Vector3d p_t(gnss.gnss_poses[index].x - point_clouds_container.offset.x(), gnss.gnss_poses[index].y - point_clouds_container.offset.y(), gnss.gnss_poses[index].alt - point_clouds_container.offset.z());

                    point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z,
                                                                  pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                                  p_s.x(), p_s.y(), p_s.z(), p_t.x(), p_t.y(), p_t.z());

                    // std::cout << " delta_x " << delta_x << " delta_y " << delta_y << " delta_z " << delta_z << std::endl;

                    int ir = tripletListB.size();
                    int ic = 0;
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
                  
                    tripletListP.emplace_back(ir, ir, 1);
                    tripletListP.emplace_back(ir + 1, ir + 1, 1);
                    tripletListP.emplace_back(ir + 2, ir + 2, 1);

                    tripletListB.emplace_back(ir, 0, delta_x);
                    tripletListB.emplace_back(ir + 1, 0, delta_y);
                    tripletListB.emplace_back(ir + 2, 0, delta_z);
                }
            }
        }

        Eigen::SparseMatrix<double>
            matA(tripletListB.size(), 6);
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

        std::cout << "AtPA.size: " << AtPA.size() << std::endl;
        std::cout << "AtPB.size: " << AtPB.size() << std::endl;

        std::cout << "start solving AtPA=AtPB" << std::endl;
        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);

        std::cout << "x = solver.solve(AtPB)" << std::endl;
        Eigen::SparseMatrix<double> x = solver.solve(AtPB);

        std::vector<double> h_x;

        for (int k = 0; k < x.outerSize(); ++k)
        {
            for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            {
                std::cout << it.value() << std::endl;
                h_x.push_back(it.value());
            }
        }

        std::cout << "h_x.size(): " << h_x.size() << std::endl;

        std::cout << "AtPA=AtPB SOLVED" << std::endl;
        TaitBryanPose pose;

        if (h_x.size() == 6)
        {
            int counter = 0;
            pose.px = h_x[counter++];
            pose.py = h_x[counter++];
            pose.pz = h_x[counter++];
            pose.om = h_x[counter++];
            pose.fi = h_x[counter++];
            pose.ka = h_x[counter++];
            is_ok = true;
            std::cout << "optimizing with tait bryan finished" << std::endl;
        }
        else
        {
            std::cout << "optimizing with tait bryan FAILED" << std::endl;
            std::cout << "h_x.size(): " << h_x.size() << " should be: " << 6 * point_clouds_container.point_clouds.size() << std::endl;
            is_ok = false;
        }

        if (is_ok)
        {
            m_pose = affine_matrix_from_pose_tait_bryan(pose);

            for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
            {
                point_clouds_container.point_clouds[i].m_pose = m_pose * point_clouds_container.point_clouds[i].m_pose;
                point_clouds_container.point_clouds[i].pose = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);
                point_clouds_container.point_clouds[i].gui_translation[0] = point_clouds_container.point_clouds[i].pose.px;
                point_clouds_container.point_clouds[i].gui_translation[1] = point_clouds_container.point_clouds[i].pose.py;
                point_clouds_container.point_clouds[i].gui_translation[2] = point_clouds_container.point_clouds[i].pose.pz;
                point_clouds_container.point_clouds[i].gui_rotation[0] = rad2deg(point_clouds_container.point_clouds[i].pose.om);
                point_clouds_container.point_clouds[i].gui_rotation[1] = rad2deg(point_clouds_container.point_clouds[i].pose.fi);
                point_clouds_container.point_clouds[i].gui_rotation[2] = rad2deg(point_clouds_container.point_clouds[i].pose.ka);
            }
        }
    }
}

void PoseGraphLoopClosure::run_icp(PointClouds &point_clouds_container, int index_active_edge, float search_radius, int number_of_iterations, int num_edge_extended_before, int num_edge_extended_after)
{
    PairWiseICP icp;
    auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);
    //std::vector<Eigen::Vector3d> source = point_clouds_container.point_clouds[edges[index_active_edge].index_to].points_local;
    //std::vector<Eigen::Vector3d> target = point_clouds_container.point_clouds[edges[index_active_edge].index_from].points_local;

    ////////////////////////////////
    std::vector<Eigen::Vector3d> source;
    auto &e = edges[index_active_edge];
    for (int i = -num_edge_extended_before; i <= num_edge_extended_after; i++)
    {
        int index_src = e.index_to + i;
        if (index_src >= 0 && index_src < point_clouds_container.point_clouds.size())
        {
            Eigen::Affine3d m_src = point_clouds_container.point_clouds.at(index_src).m_pose;
            for (int k = 0; k < point_clouds_container.point_clouds[index_src].points_local.size(); k++)
            {
                Eigen::Vector3d p_g = m_src * point_clouds_container.point_clouds[index_src].points_local[k];
                source.push_back(p_g);
            }
            // point_clouds_container.point_clouds.at(index_src).render(m_src, 1);
        }
    }
    std::vector<Eigen::Vector3d> target;

    for (int i = -num_edge_extended_before; i <= num_edge_extended_after; i++)
    {
        int index_trg = e.index_from + i;
        if (index_trg >= 0 && index_trg < point_clouds_container.point_clouds.size())
        {
            Eigen::Affine3d m_trg = point_clouds_container.point_clouds.at(index_trg).m_pose;
            for (int k = 0; k < point_clouds_container.point_clouds[index_trg].points_local.size(); k++)
            {
                Eigen::Vector3d p_g = m_trg * point_clouds_container.point_clouds[index_trg].points_local[k];
                target.push_back(p_g);
            }
            // point_clouds_container.point_clouds.at(index_src).render(m_src, 1);
        }
    }

    Eigen::Affine3d m_src_inv = point_clouds_container.point_clouds[e.index_to].m_pose.inverse();

    for (auto &p : source)
    {
        p = m_src_inv * p;
    }

    Eigen::Affine3d m_trg_inv = point_clouds_container.point_clouds[e.index_from].m_pose.inverse();

    for (auto &p : target)
    {
        p = m_trg_inv * p; 
    }
    ////////////////////////////////

    if (icp.compute(source, target, search_radius, number_of_iterations, m_pose))
    {
        edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
    }
}