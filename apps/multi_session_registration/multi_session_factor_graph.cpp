#include "multi_session_factor_graph.h"
#include <utils.hpp>

#include <m_estimators.h>

#include <python-scripts/constraints/relative_pose_tait_bryan_cw_jacobian.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-feature-metrics/point_to_line_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-feature-metrics/point_to_plane_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian.h>

bool optimize(std::vector<Session>& sessions, const std::vector<Edge>& edges)
{
    for (auto& session : sessions)
    {
        // std::cout << session.point_clouds_container.point_clouds.size() << std::endl;
        for (auto& pc : session.point_clouds_container.point_clouds)
            pc.m_pose_temp = pc.m_pose;
    }

    std::vector<int> sums;
    sums.push_back(0);
    int sum = 0;
    std::vector<bool> vfixed_x;
    std::vector<bool> vfixed_y;
    std::vector<bool> vfixed_z;
    std::vector<bool> vfixed_om;
    std::vector<bool> vfixed_fi;
    std::vector<bool> vfixed_ka;

    for (size_t i = 0; i < sessions.size(); i++)
    {
        sum += sessions[i].point_clouds_container.point_clouds.size();
        sums.push_back(sum);

        for (size_t j = 0; j < sessions[i].point_clouds_container.point_clouds.size(); j++)
        {
            vfixed_x.push_back(sessions[i].point_clouds_container.point_clouds[j].fixed_x);
            vfixed_y.push_back(sessions[i].point_clouds_container.point_clouds[j].fixed_y);
            vfixed_z.push_back(sessions[i].point_clouds_container.point_clouds[j].fixed_z);

            vfixed_om.push_back(sessions[i].point_clouds_container.point_clouds[j].fixed_om);
            vfixed_fi.push_back(sessions[i].point_clouds_container.point_clouds[j].fixed_fi);
            vfixed_ka.push_back(sessions[i].point_clouds_container.point_clouds[j].fixed_ka);
        }
    }

    ///////////////////////////////////////////////////////////////////
    // graph slam
    bool is_ok = true;
    std::vector<Eigen::Affine3d> m_poses;
    std::vector<Eigen::Affine3d> poses_motion_model;
    std::vector<int> index_trajectory;

    for (size_t j = 0; j < sessions.size(); j++)
    {
        for (size_t i = 0; i < sessions[j].point_clouds_container.point_clouds.size(); i++)
        {
            m_poses.push_back(sessions[j].point_clouds_container.point_clouds[i].m_pose);
            poses_motion_model.push_back(sessions[j].point_clouds_container.point_clouds[i].m_initial_pose);
            // poses_motion_model.push_back(sessions[j].point_clouds_container.point_clouds[i].m_pose);
            index_trajectory.push_back(j);
        }
    }

    std::vector<TaitBryanPose> poses;

    bool is_wc = true;
    bool is_cw = false;
    int iterations = 1;
    bool is_fix_first_node = true;
    // std::vector<int> indexes_ground_truth;
    for (size_t j = 0; j < sessions.size(); j++)
    {
        for (size_t i = 0; i < sessions[j].point_clouds_container.point_clouds.size(); i++)
        {
            if (is_wc)
            {
                poses.push_back(pose_tait_bryan_from_affine_matrix(sessions[j].point_clouds_container.point_clouds[i].m_pose));
            }
            else if (is_cw)
            {
                poses.push_back(pose_tait_bryan_from_affine_matrix(sessions[j].point_clouds_container.point_clouds[i].m_pose.inverse()));
            }
            // if (sessions[j].is_ground_truth)
            //{
            //     indexes_ground_truth.push_back(poses.size() - 1);
            // }
        }
    }

    std::vector<Edge> all_edges;

    // motion model edges;
    double angle = 0.1 * DEG_TO_RAD;
    double wangle = 1.0 / (angle * angle);

    for (size_t i = 1; i < poses_motion_model.size(); i++)
    {
        if (index_trajectory[i - 1] == index_trajectory[i])
        {
            Eigen::Affine3d m_rel = poses_motion_model[i - 1].inverse() * poses_motion_model[i];
            Edge edge;
            edge.index_from = i - 1;
            edge.index_to = i;
            edge.index_session_from = -1;
            edge.index_session_to = -1;
            edge.relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_rel);
            edge.relative_pose_tb_weights.om = wangle;
            edge.relative_pose_tb_weights.fi = wangle;
            edge.relative_pose_tb_weights.ka = wangle;
            edge.relative_pose_tb_weights.px = 1000000.0;
            edge.relative_pose_tb_weights.py = 1000000.0;
            edge.relative_pose_tb_weights.pz = 1000000.0;
            all_edges.push_back(edge);
        }
    }

    for (size_t i = 0; i < sessions.size(); i++)
    {
        for (size_t j = 0; j < sessions[i].pose_graph_loop_closure.edges.size(); j++)
        {
            Edge edge;
            edge.index_from = sessions[i].pose_graph_loop_closure.edges[j].index_from + sums[i];
            edge.index_to = sessions[i].pose_graph_loop_closure.edges[j].index_to + sums[i];
            edge.relative_pose_tb = sessions[i].pose_graph_loop_closure.edges[j].relative_pose_tb;
            edge.relative_pose_tb_weights = sessions[i].pose_graph_loop_closure.edges[j].relative_pose_tb_weights;
            // edge.is_fixed_fi = ToDo
            all_edges.push_back(edge);
        }
    }

    for (size_t i = 0; i < edges.size(); i++)
    {
        Edge edge;
        edge.index_from = edges[i].index_from + sums[edges[i].index_session_from];
        edge.index_to = edges[i].index_to + sums[edges[i].index_session_to];
        edge.relative_pose_tb = edges[i].relative_pose_tb;
        edge.relative_pose_tb_weights = edges[i].relative_pose_tb_weights;
        // edge.is_fixed_fi = ToDo
        all_edges.push_back(edge);
    }

    for (int iter = 0; iter < iterations; iter++)
    {
        // std::cout << "iteration " << iter + 1 << " of " << iterations << std::endl;
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
                relative_pose_obs_eq_tait_bryan_wc_case1_jacobian(
                    jacobian,
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
                relative_pose_obs_eq_tait_bryan_cw_case1_jacobian(
                    jacobian,
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

            // std::cout << "delta(0, 0): " << delta(0, 0) << std::endl;
            // std::cout << "delta(1, 0): " << delta(0, 0) << std::endl;
            // std::cout << "delta(2, 0): " << delta(0, 0) << std::endl;
            // std::cout << "normalize_angle(delta(3, 0)): " << normalize_angle(delta(3, 0)) << std::endl;
            // std::cout << "normalize_angle(delta(4, 0)): " << normalize_angle(delta(4, 0)) << std::endl;
            // std::cout << "normalize_angle(delta(5, 0)): " << normalize_angle(delta(5, 0)) << std::endl;

            // for (int r = 0; r < 6; r++) {
            //     for (int c = 0; c < 6; c++) {
            //         tripletListP.emplace_back(ir + r, ir + c, edges[i].information_matrix.coeffRef(r, c));
            //    }
            // }

            // tripletListP.emplace_back(ir    , ir    , get_cauchy_w(delta(0, 0), 10));
            // tripletListP.emplace_back(ir + 1, ir + 1, get_cauchy_w(delta(1, 0), 10));
            // tripletListP.emplace_back(ir + 2, ir + 2, get_cauchy_w(delta(2, 0), 10));
            // tripletListP.emplace_back(ir + 3, ir + 3, get_cauchy_w(delta(3, 0), 10));
            // tripletListP.emplace_back(ir + 4, ir + 4, get_cauchy_w(delta(4, 0), 10));
            // tripletListP.emplace_back(ir + 5, ir + 5, get_cauchy_w(delta(5, 0), 10));

            // if (sessions[all_edges[i].index_session_from].is_ground_truth || sessions[all_edges[i].index_session_to].is_ground_truth)
            //{
            //     tripletListP.emplace_back(ir, ir, all_edges[i].relative_pose_tb_weights.px);
            //     tripletListP.emplace_back(ir + 1, ir + 1, all_edges[i].relative_pose_tb_weights.py);
            //     tripletListP.emplace_back(ir + 2, ir + 2, all_edges[i].relative_pose_tb_weights.pz);
            //     tripletListP.emplace_back(ir + 3, ir + 3, all_edges[i].relative_pose_tb_weights.om);
            //     tripletListP.emplace_back(ir + 4, ir + 4, all_edges[i].relative_pose_tb_weights.fi);
            //     tripletListP.emplace_back(ir + 5, ir + 5, all_edges[i].relative_pose_tb_weights.ka);
            // }
            // else
            //{

            tripletListP.emplace_back(ir, ir, all_edges[i].relative_pose_tb_weights.px);
            tripletListP.emplace_back(ir + 1, ir + 1, all_edges[i].relative_pose_tb_weights.py);
            tripletListP.emplace_back(ir + 2, ir + 2, all_edges[i].relative_pose_tb_weights.pz);
            tripletListP.emplace_back(ir + 3, ir + 3, all_edges[i].relative_pose_tb_weights.om);
            tripletListP.emplace_back(ir + 4, ir + 4, all_edges[i].relative_pose_tb_weights.fi);
            tripletListP.emplace_back(ir + 5, ir + 5, all_edges[i].relative_pose_tb_weights.ka);

            // tripletListP.emplace_back(ir, ir, all_edges[i].relative_pose_tb_weights.px * get_cauchy_w(delta(0, 0), 1));
            // tripletListP.emplace_back(ir + 1, ir + 1, all_edges[i].relative_pose_tb_weights.py * get_cauchy_w(delta(1, 0), 1));
            // tripletListP.emplace_back(ir + 2, ir + 2, all_edges[i].relative_pose_tb_weights.pz * get_cauchy_w(delta(2, 0), 1));
            // tripletListP.emplace_back(ir + 3, ir + 3, all_edges[i].relative_pose_tb_weights.om * get_cauchy_w(delta(3, 0), 1));
            // tripletListP.emplace_back(ir + 4, ir + 4, all_edges[i].relative_pose_tb_weights.fi * get_cauchy_w(delta(4, 0), 1));
            // tripletListP.emplace_back(ir + 5, ir + 5, all_edges[i].relative_pose_tb_weights.ka * get_cauchy_w(delta(5, 0), 1));
            // }
        }
        if (is_fix_first_node)
        {
            int ir = tripletListB.size();
            tripletListA.emplace_back(ir, 0, 1);
            tripletListA.emplace_back(ir + 1, 1, 1);
            tripletListA.emplace_back(ir + 2, 2, 1);
            tripletListA.emplace_back(ir + 3, 3, 1);
            tripletListA.emplace_back(ir + 4, 4, 1);
            tripletListA.emplace_back(ir + 5, 5, 1);

            tripletListP.emplace_back(ir, ir, 0.0001);
            tripletListP.emplace_back(ir + 1, ir + 1, 0.0001);
            tripletListP.emplace_back(ir + 2, ir + 2, 0.0001);
            tripletListP.emplace_back(ir + 3, ir + 3, 0.0001);
            tripletListP.emplace_back(ir + 4, ir + 4, 0.0001);
            tripletListP.emplace_back(ir + 5, ir + 5, 0.0001);

            tripletListB.emplace_back(ir, 0, 0);
            tripletListB.emplace_back(ir + 1, 0, 0);
            tripletListB.emplace_back(ir + 2, 0, 0);
            tripletListB.emplace_back(ir + 3, 0, 0);
            tripletListB.emplace_back(ir + 4, 0, 0);
            tripletListB.emplace_back(ir + 5, 0, 0);

            for (size_t i = 1; i < index_trajectory.size(); i++)
            {
                if (index_trajectory[i - 1] != index_trajectory[i])
                {
                    ir = tripletListB.size();
                    tripletListA.emplace_back(ir, i * 6 + 0, 1);
                    tripletListA.emplace_back(ir + 1, i * 6 + 1, 1);
                    tripletListA.emplace_back(ir + 2, i * 6 + 2, 1);
                    tripletListA.emplace_back(ir + 3, i * 6 + 3, 1);
                    tripletListA.emplace_back(ir + 4, i * 6 + 4, 1);
                    tripletListA.emplace_back(ir + 5, i * 6 + 5, 1);

                    tripletListP.emplace_back(ir, ir, 0.0001);
                    tripletListP.emplace_back(ir + 1, ir + 1, 0.0001);
                    tripletListP.emplace_back(ir + 2, ir + 2, 0.0001);
                    tripletListP.emplace_back(ir + 3, ir + 3, 0.0001);
                    tripletListP.emplace_back(ir + 4, ir + 4, 0.0001);
                    tripletListP.emplace_back(ir + 5, ir + 5, 0.0001);

                    tripletListB.emplace_back(ir, 0, 0);
                    tripletListB.emplace_back(ir + 1, 0, 0);
                    tripletListB.emplace_back(ir + 2, 0, 0);
                    tripletListB.emplace_back(ir + 3, 0, 0);
                    tripletListB.emplace_back(ir + 4, 0, 0);
                    tripletListB.emplace_back(ir + 5, 0, 0);
                }
            }
        }

        /*double angle = 1.0 * DEG_TO_RAD;
        double wangle = 1.0 / (angle * angle);

        for (size_t index = 0; index < indexes_ground_truth.size(); index++)
        {
            int ir = tripletListB.size();
            int ic = indexes_ground_truth[index] * 6;
            tripletListA.emplace_back(ir, ic + 0, 1);
            tripletListA.emplace_back(ir + 1, ic + 1, 1);
            tripletListA.emplace_back(ir + 2, ic + 2, 1);
            tripletListA.emplace_back(ir + 3, ic + 3, 1);
            tripletListA.emplace_back(ir + 4, ic + 4, 1);
            tripletListA.emplace_back(ir + 5, ic + 5, 1);

            tripletListP.emplace_back(ir    , ir    , 10000.0);
            tripletListP.emplace_back(ir + 1, ir + 1, 10000.0);
            tripletListP.emplace_back(ir + 2, ir + 2, 10000.0);
            tripletListP.emplace_back(ir + 3, ir + 3, wangle);
            tripletListP.emplace_back(ir + 4, ir + 4, wangle);
            tripletListP.emplace_back(ir + 5, ir + 5, wangle);

            tripletListB.emplace_back(ir, 0, 0);
            tripletListB.emplace_back(ir + 1, 0, 0);
            tripletListB.emplace_back(ir + 2, 0, 0);
            tripletListB.emplace_back(ir + 3, 0, 0);
            tripletListB.emplace_back(ir + 4, 0, 0);
            tripletListB.emplace_back(ir + 5, 0, 0);
        }*/

        // gnss
        // for (const auto &pc : point_clouds_container.point_clouds)
        /*for (size_t index_pose = 0; index_pose < point_clouds_container.point_clouds.size(); index_pose++)
        {
            const auto &pc = point_clouds_container.point_clouds[index_pose];
            for (size_t i = 0; i < gnss.gnss_poses.size(); i++)
            {
                double time_stamp = gnss.gnss_poses[i].timestamp;

                auto it = std::lower_bound(pc.local_trajectory.begin(), pc.local_trajectory.end(),
                                           time_stamp, [](const PointCloud::LocalTrajectoryNode &lhs, const double &time) -> bool
                                           { return lhs.timestamp < time; });

                int index = it - pc.local_trajectory.begin();

                if (index > 0 && index < pc.local_trajectory.size())
                {

                    if (fabs(time_stamp - pc.local_trajectory[index].timestamp) < 10e12)
                    {

                        Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
                        TaitBryanPose pose_s;
                        pose_s = pose_tait_bryan_from_affine_matrix(m_poses[index_pose]);
                        Eigen::Vector3d p_s = pc.local_trajectory[index].m_pose.translation();
                        point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian, pose_s.px, pose_s.py, pose_s.pz, pose_s.om,
        pose_s.fi, pose_s.ka, p_s.x(), p_s.y(), p_s.z());

                        double delta_x;
                        double delta_y;
                        double delta_z;
                        Eigen::Vector3d p_t(gnss.gnss_poses[i].x - gnss.offset_x, gnss.gnss_poses[i].y - gnss.offset_y,
        gnss.gnss_poses[i].alt - gnss.offset_alt); point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z, pose_s.px,
        pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka, p_s.x(), p_s.y(), p_s.z(), p_t.x(), p_t.y(), p_t.z());

                        std::cout << " delta_x " << delta_x << " delta_y " << delta_y << " delta_z " << delta_z << std::endl;

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
                        tripletListP.emplace_back(ir, ir, get_cauchy_w(delta_x, 1));
                        tripletListP.emplace_back(ir + 1, ir + 1, get_cauchy_w(delta_y, 1));
                        tripletListP.emplace_back(ir + 2, ir + 2, get_cauchy_w(delta_z, 1));

                        tripletListB.emplace_back(ir, 0, delta_x);
                        tripletListB.emplace_back(ir + 1, 0, delta_y);
                        tripletListB.emplace_back(ir + 2, 0, delta_z);

                        // jacobian3x6 = get_point_to_point_jacobian_tait_bryan(pose_convention,
        point_clouds_container.point_clouds[i].m_pose, p_s, p_t);

                        // auto m = pc.m_pose * pc.local_trajectory[index].m_pose;
                        // glVertex3f(m(0, 3), m(1, 3), m(2, 3));
                        // glVertex3f(gnss_poses[i].x - offset_x, gnss_poses[i].y - offset_y, gnss_poses[i].alt - offset_alt);
                    }
                }
            }
        }*/
        //

        for (size_t j = 0; j < sessions.size(); j++)
        {
            for (size_t jj = 0; jj < sessions[j].ground_control_points.gpcs.size(); jj++)
            {
                Eigen::Vector3d p_s =
                    sessions[j]
                        .point_clouds_container.point_clouds[sessions[j].ground_control_points.gpcs[jj].index_to_node_inner]
                        .local_trajectory[sessions[j].ground_control_points.gpcs[jj].index_to_node_outer]
                        .m_pose.translation();
                Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
                TaitBryanPose pose_s;
                pose_s = pose_tait_bryan_from_affine_matrix(
                    sessions[j].point_clouds_container.point_clouds[sessions[j].ground_control_points.gpcs[jj].index_to_node_inner].m_pose);
                point_to_point_source_to_target_tait_bryan_wc_jacobian(
                    jacobian, pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka, p_s.x(), p_s.y(), p_s.z());

                double delta_x;
                double delta_y;
                double delta_z;
                Eigen::Vector3d p_t(
                    sessions[j].ground_control_points.gpcs[jj].x,
                    sessions[j].ground_control_points.gpcs[jj].y,
                    sessions[j].ground_control_points.gpcs[jj].z + sessions[j].ground_control_points.gpcs[jj].lidar_height_above_ground);
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

                int ir = tripletListB.size();
                int ic = sessions[j].ground_control_points.gpcs[jj].index_to_node_inner * 6 + sums[j] * 6;

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
                tripletListP.emplace_back(
                    ir + 0,
                    ir + 0,
                    (1.0 / (sessions[j].ground_control_points.gpcs[jj].sigma_x * sessions[j].ground_control_points.gpcs[jj].sigma_x)) *
                        get_cauchy_w(delta_x, 1));
                tripletListP.emplace_back(
                    ir + 1,
                    ir + 1,
                    (1.0 / (sessions[j].ground_control_points.gpcs[jj].sigma_y * sessions[j].ground_control_points.gpcs[jj].sigma_y)) *
                        get_cauchy_w(delta_y, 1));
                tripletListP.emplace_back(
                    ir + 2,
                    ir + 2,
                    (1.0 / (sessions[j].ground_control_points.gpcs[jj].sigma_z * sessions[j].ground_control_points.gpcs[jj].sigma_z)) *
                        get_cauchy_w(delta_z, 1));

                tripletListB.emplace_back(ir, 0, delta_x);
                tripletListB.emplace_back(ir + 1, 0, delta_y);
                tripletListB.emplace_back(ir + 2, 0, delta_z);

                std::cout << "gcp: delta_x " << delta_x << " delta_y " << delta_y << " delta_z " << delta_z << std::endl;
            }
        }

        ///////////////////////////////////////////////////////
        for (size_t i = 0; i < sessions.size(); i++)
        {
            for (size_t index_pose = 0; index_pose < sessions[i].point_clouds_container.point_clouds.size(); index_pose++)
            {
                if (sessions[i].point_clouds_container.point_clouds[index_pose].fixed_x || sessions[i].is_ground_truth)
                {
                    int ir = tripletListB.size();
                    int ic = (index_pose + sums[i]) * 6;
                    tripletListA.emplace_back(ir + 0, ic, 1000000000000);
                    tripletListP.emplace_back(ir, ir, 1);
                    tripletListB.emplace_back(ir, 0, 0);
                }

                if (sessions[i].point_clouds_container.point_clouds[index_pose].fixed_y || sessions[i].is_ground_truth)
                {
                    int ir = tripletListB.size();
                    int ic = (index_pose + sums[i]) * 6 + 1;
                    tripletListA.emplace_back(ir + 0, ic, 1000000000000);
                    tripletListP.emplace_back(ir, ir, 1);
                    tripletListB.emplace_back(ir, 0, 0);
                }

                if (sessions[i].point_clouds_container.point_clouds[index_pose].fixed_z || sessions[i].is_ground_truth)
                {
                    int ir = tripletListB.size();
                    int ic = (index_pose + sums[i]) * 6 + 2;
                    tripletListA.emplace_back(ir + 0, ic, 1000000000000);
                    tripletListP.emplace_back(ir, ir, 1);
                    tripletListB.emplace_back(ir, 0, 0);
                }

                if (sessions[i].point_clouds_container.point_clouds[index_pose].fixed_om || sessions[i].is_ground_truth)
                {
                    int ir = tripletListB.size();
                    int ic = (index_pose + sums[i]) * 6 + 3;
                    tripletListA.emplace_back(ir + 0, ic, 1000000000000);
                    tripletListP.emplace_back(ir, ir, 1);
                    tripletListB.emplace_back(ir, 0, 0);
                }

                if (sessions[i].point_clouds_container.point_clouds[index_pose].fixed_fi || sessions[i].is_ground_truth)
                {
                    int ir = tripletListB.size();
                    int ic = (index_pose + sums[i]) * 6 + 4;
                    tripletListA.emplace_back(ir + 0, ic, 1000000000000);
                    tripletListP.emplace_back(ir, ir, 1);
                    tripletListB.emplace_back(ir, 0, 0);
                }

                if (sessions[i].point_clouds_container.point_clouds[index_pose].fixed_ka || sessions[i].is_ground_truth)
                {
                    int ir = tripletListB.size();
                    int ic = (index_pose + sums[i]) * 6 + 5;
                    tripletListA.emplace_back(ir + 0, ic, 1000000000000);
                    tripletListP.emplace_back(ir, ir, 1);
                    tripletListB.emplace_back(ir, 0, 0);
                }

                // std::cout << "add manual_pose_graph_loop_closure.edge" << std::endl;
                // Edge edge;
                // edge.index_from = sessions[i].pose_graph_loop_closure.edges[j].index_from + sums[i];
                // edge.index_to = sessions[i].pose_graph_loop_closure.edges[j].index_to + sums[i];
                // edge.relative_pose_tb = sessions[i].pose_graph_loop_closure.edges[j].relative_pose_tb;
                // edge.relative_pose_tb_weights = sessions[i].pose_graph_loop_closure.edges[j].relative_pose_tb_weights;
                // edge.is_fixed_fi = ToDo
                // all_edges.push_back(edge);
            }
        }

        // fuse_inclination_from_imu

        double error_imu = 0;
        double error_imu_sum = 0;

        for (size_t j = 0; j < sessions.size(); j++)
        {
            for (size_t index_pose = 0; index_pose < sessions[j].point_clouds_container.point_clouds.size(); index_pose++)
            {
                const auto& pc = sessions[j].point_clouds_container.point_clouds[index_pose];
                if (!pc.fuse_inclination_from_IMU)
                {
                    continue;
                }
                if (pc.local_trajectory.size() == 0)
                {
                    continue;
                }

                TaitBryanPose target_pose;
                target_pose.om = pc.local_trajectory[0].imu_om_fi_ka.x();
                target_pose.fi = pc.local_trajectory[0].imu_om_fi_ka.y();
                target_pose.ka = pc.local_trajectory[0].imu_om_fi_ka.z();
                target_pose.px = pc.m_pose(0, 3);
                target_pose.py = pc.m_pose(1, 3);
                target_pose.pz = pc.m_pose(2, 3);

                Eigen::Affine3d target_mpose = affine_matrix_from_pose_tait_bryan(target_pose);

                double xtg = target_mpose(0, 3);
                double ytg = target_mpose(1, 3);
                double ztg = target_mpose(2, 3);
                double vzx = target_mpose(0, 2);
                double vzy = target_mpose(1, 2);
                double vzz = target_mpose(2, 2);

                TaitBryanPose current_pose = pose_tait_bryan_from_affine_matrix(pc.m_pose);

                Eigen::Matrix<double, 1, 1> delta;
                point_to_plane_tait_bryan_wc(
                    delta,
                    current_pose.px,
                    current_pose.py,
                    current_pose.pz,
                    current_pose.om,
                    current_pose.fi,
                    current_pose.ka,
                    1,
                    0,
                    0, // add 0,1,0
                    xtg,
                    ytg,
                    ztg,
                    vzx,
                    vzy,
                    vzz);

                Eigen::Matrix<double, 1, 6> delta_jacobian;
                point_to_plane_tait_bryan_wc_jacobian(
                    delta_jacobian,
                    current_pose.px,
                    current_pose.py,
                    current_pose.pz,
                    current_pose.om,
                    current_pose.fi,
                    current_pose.ka,
                    1,
                    0,
                    0,
                    xtg,
                    ytg,
                    ztg,
                    vzx,
                    vzy,
                    vzz);

                int ir = tripletListB.size();
                int ic = (index_pose + sums[j]) * 6;
                tripletListA.emplace_back(ir + 0, ic + 3, -delta_jacobian(0, 3));
                tripletListA.emplace_back(ir + 0, ic + 4, -delta_jacobian(0, 4));

                tripletListP.emplace_back(ir, ir, /*get_cauchy_w(delta(0, 0), 1) * 10000*/ 1);

                tripletListB.emplace_back(ir, 0, delta(0, 0));

                ///////////////////////////////
                point_to_plane_tait_bryan_wc(
                    delta,
                    current_pose.px,
                    current_pose.py,
                    current_pose.pz,
                    current_pose.om,
                    current_pose.fi,
                    current_pose.ka,
                    0,
                    1,
                    0,
                    xtg,
                    ytg,
                    ztg,
                    vzx,
                    vzy,
                    vzz);

                point_to_plane_tait_bryan_wc_jacobian(
                    delta_jacobian,
                    current_pose.px,
                    current_pose.py,
                    current_pose.pz,
                    current_pose.om,
                    current_pose.fi,
                    current_pose.ka,
                    0,
                    1,
                    0,
                    xtg,
                    ytg,
                    ztg,
                    vzx,
                    vzy,
                    vzz);

                ir = tripletListB.size();
                // ic = index_pose * 6;
                ic = (index_pose + sums[j]) * 6;

                tripletListA.emplace_back(ir + 0, ic + 3, -delta_jacobian(0, 3));
                tripletListA.emplace_back(ir + 0, ic + 4, -delta_jacobian(0, 4));

                tripletListP.emplace_back(ir, ir, /*get_cauchy_w(delta(0, 0), 1) * 10000*/ 1);

                tripletListB.emplace_back(ir, 0, delta(0, 0));
            }
        }

        // fuse control points
        for (size_t j = 0; j < sessions.size(); j++)
        {
            // CPs
            auto& cps = sessions[j].control_points;
            auto& point_clouds_container = sessions[j].point_clouds_container;

            for (int i = 0; i < cps.cps.size(); i++)
            {
                if (!cps.cps[i].is_z_0)
                {
                    Eigen::Vector3d p_s(cps.cps[i].x_source_local, cps.cps[i].y_source_local, cps.cps[i].z_source_local);

                    Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
                    TaitBryanPose pose_s;
                    pose_s = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[cps.cps[i].index_to_pose].m_pose);

                    point_to_point_source_to_target_tait_bryan_wc_jacobian(
                        jacobian, pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka, p_s.x(), p_s.y(), p_s.z());

                    double delta_x;
                    double delta_y;
                    double delta_z;
                    Eigen::Vector3d p_t(cps.cps[i].x_target_global, cps.cps[i].y_target_global, cps.cps[i].z_target_global);
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

                    int ir = tripletListB.size();

                    //(index_pose + sums[j]) * 6;
                    int ic = (cps.cps[i].index_to_pose + sums[j]) * 6;

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

                    std::cout << "cp [not z == 0]: delta_x " << delta_x << " delta_y " << delta_y << " delta_z " << delta_z << std::endl;
                }
                else
                {
                    Eigen::Vector3d p_s(cps.cps[i].x_source_local, cps.cps[i].y_source_local, cps.cps[i].z_source_local);

                    Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
                    TaitBryanPose pose_s;
                    pose_s = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[cps.cps[i].index_to_pose].m_pose);

                    point_to_point_source_to_target_tait_bryan_wc_jacobian(
                        jacobian, pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka, p_s.x(), p_s.y(), p_s.z());

                    double delta_x;
                    double delta_y;
                    double delta_z;
                    Eigen::Vector3d p_t(cps.cps[i].x_target_global, cps.cps[i].y_target_global, 0.0 /*cps.cps[i].z_target_global*/);
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

                    int ir = tripletListB.size();
                    // int ic = cps.cps[i].index_to_pose * 6;
                    int ic = (cps.cps[i].index_to_pose + sums[j]) * 6;

                    for (int row = 2; row < 3; row++)
                    {
                        for (int col = 0; col < 6; col++)
                        {
                            if (jacobian(row, col) != 0.0)
                            {
                                tripletListA.emplace_back(ir, ic + col, -jacobian(row, col));
                            }
                        }
                    }
                    // tripletListP.emplace_back(ir + 0, ir + 0, (1.0 / (cps.cps[i].sigma_x * cps.cps[i].sigma_x)) * get_cauchy_w(delta_x,
                    // 1)); tripletListP.emplace_back(ir + 1, ir + 1, (1.0 / (cps.cps[i].sigma_y * cps.cps[i].sigma_y)) *
                    // get_cauchy_w(delta_y, 1));
                    tripletListP.emplace_back(ir, ir, (1.0 / (cps.cps[i].sigma_z * cps.cps[i].sigma_z)));

                    // tripletListB.emplace_back(ir, 0, delta_x);
                    // tripletListB.emplace_back(ir + 1, 0, delta_y);
                    tripletListB.emplace_back(ir, 0, delta_z);

                    std::cout << "cp [not z == 0]: delta_z " << delta_z << std::endl;
                }
            }
        }

        //////////////////////////////////////////////////////
        // for (size_t i = 0; i < gcps.gpcs.size(); i++)
        //{

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

        // std::cout << "AtPA.size: " << AtPA.size() << std::endl;
        // std::cout << "AtPB.size: " << AtPB.size() << std::endl;

        // std::cout << "start solving AtPA=AtPB" << std::endl;
        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);

        // std::cout << "x = solver.solve(AtPB)" << std::endl;
        Eigen::SparseMatrix<double> x = solver.solve(AtPB);

        std::vector<double> h_x;

        for (int k = 0; k < x.outerSize(); ++k)
        {
            for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            {
                h_x.push_back(it.value());
            }
        }

        // std::cout << "h_x.size(): " << h_x.size() << std::endl;

        // std::cout << "AtPA=AtPB SOLVED" << std::endl;
        //  std::cout << "updates:" << std::endl;
        //  for (size_t i = 0; i < h_x.size(); i += 6)
        //{
        //      std::cout << h_x[i] << "," << h_x[i + 1] << "," << h_x[i + 2] << "," << h_x[i + 3] << "," << h_x[i + 4] << "," << h_x[i + 5]
        //      << std::endl;
        //  }

        if (h_x.size() == 6 * poses.size())
        {
            int counter = 0;

            for (size_t i = 0; i < poses.size(); i++)
            {
                TaitBryanPose pose = poses[i];
                pose.px += h_x[counter++] * 0.1;
                pose.py += h_x[counter++] * 0.1;
                pose.pz += h_x[counter++] * 0.1;
                pose.om += h_x[counter++] * 0.1;
                pose.fi += h_x[counter++] * 0.1;
                pose.ka += h_x[counter++] * 0.1;

                /*if (!vfixed_x[i])
                    poses[i].px = pose.px;
                if (!vfixed_y[i])
                    poses[i].py = pose.py;
                if (!vfixed_z[i])
                    poses[i].pz = pose.pz;
                if (!vfixed_om[i])
                    poses[i].om = pose.om;
                if (!vfixed_fi[i])
                    poses[i].fi = pose.fi;
                if (!vfixed_ka[i])
                    poses[i].ka = pose.ka;*/

                poses[i].px = pose.px;
                poses[i].py = pose.py;
                poses[i].pz = pose.pz;
                poses[i].om = pose.om;
                poses[i].fi = pose.fi;
                poses[i].ka = pose.ka;

                // if (i == 0 && is_fix_first_node)
                //     poses[i] = pose;
            }
            // std::cout << "optimizing with tait bryan finished" << std::endl;
        }
        else
        {
            std::cout << "optimizing with tait bryan FAILED" << std::endl;
            std::cout << "h_x.size(): " << h_x.size() << " should be: " << 6 * poses.size() << std::endl;
            is_ok = false;
            break;
        }

        if (is_ok)
        {
            for (size_t i = 0; i < m_poses.size(); i++)
            {
                if (is_wc)
                    m_poses[i] = affine_matrix_from_pose_tait_bryan(poses[i]);
                else if (is_cw)
                    m_poses[i] = affine_matrix_from_pose_tait_bryan(poses[i]).inverse();
            }
        }

        if (is_ok)
        {
            int index = 0;
            for (size_t i = 0; i < m_poses.size(); i++)
            {
                if (i > 0)
                {
                    if (index_trajectory[i - 1] != index_trajectory[i])
                        index = 0;
                }
                // if (!sessions[index_trajectory[i]].is_ground_truth)
                //{
                sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].m_pose = m_poses[i];
                sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].pose =
                    pose_tait_bryan_from_affine_matrix(sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].m_pose);
                sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].gui_translation[0] =
                    sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].pose.px;
                sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].gui_translation[1] =
                    sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].pose.py;
                sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].gui_translation[2] =
                    sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].pose.pz;
                sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].gui_rotation[0] =
                    rad2deg(sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].pose.om);
                sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].gui_rotation[1] =
                    rad2deg(sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].pose.fi);
                sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].gui_rotation[2] =
                    rad2deg(sessions[index_trajectory[i]].point_clouds_container.point_clouds[index].pose.ka);
                //}
                index++;
            }
        }
    }
    return true;
}