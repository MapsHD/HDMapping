#include "multi_session_edges.h"

#include <Core/hash_utils.h>
#include <Core/pair_wise_iterative_closest_point.h>
#include <Core/transformations.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <set>
#include <tuple>
#include <unordered_map>

namespace
{
    // Points of scan `index` of `session` and of its neighbouring scans, in the frame of scan `index`.
    std::vector<Eigen::Vector3d> scan_points(const Session& session, int index, int num_before, int num_after)
    {
        const auto& point_clouds = session.point_clouds_container.point_clouds;
        const Eigen::Affine3d m_inv = point_clouds[index].m_pose.inverse();

        std::vector<Eigen::Vector3d> points;
        for (int i = -num_before; i <= num_after; i++)
        {
            const int k = index + i;
            if (k < 0 || k >= static_cast<int>(point_clouds.size()))
                continue;

            const Eigen::Affine3d m = m_inv * point_clouds[k].m_pose;
            for (const auto& p : point_clouds[k].points_local)
                points.push_back(m * p);
        }
        return points;
    }

    // (session, scan) pair of an edge in a fixed order, so that both directions give the same key.
    std::tuple<int, int, int, int> pair_key(int session_a, int scan_a, int session_b, int scan_b)
    {
        if (std::make_pair(session_a, scan_a) <= std::make_pair(session_b, scan_b))
            return { session_a, scan_a, session_b, scan_b };
        return { session_b, scan_b, session_a, scan_a };
    }
} // namespace

void edge_point_clouds(
    const std::vector<Session>& sessions,
    const Edge& edge,
    int num_edge_extended_before,
    int num_edge_extended_after,
    std::vector<Eigen::Vector3d>& source,
    std::vector<Eigen::Vector3d>& target)
{
    source = scan_points(sessions[edge.index_session_to], edge.index_to, num_edge_extended_before, num_edge_extended_after);
    target = scan_points(sessions[edge.index_session_from], edge.index_from, num_edge_extended_before, num_edge_extended_after);
}

EdgeQuality edge_quality(
    const std::vector<Eigen::Vector3d>& source,
    const std::vector<Eigen::Vector3d>& target,
    const Eigen::Affine3d& m_pose,
    double search_radius)
{
    const Eigen::Vector3d bucket(search_radius, search_radius, search_radius);
    std::unordered_map<uint64_t, std::vector<int>> buckets;
    for (int i = 0; i < static_cast<int>(target.size()); i++)
        buckets[get_rgd_index_3d(target[i], bucket)].push_back(i);

    EdgeQuality quality;
    double sum_squared = 0.0;
    for (const auto& source_point : source)
    {
        // as PairWiseICP: points at the sensor origin are not registered
        if (source_point.norm() < 0.1)
            continue;
        quality.source_points++;

        const Eigen::Vector3d p = m_pose * source_point;
        double best = search_radius * search_radius;
        bool found = false;
        for (double x = -search_radius; x <= search_radius; x += search_radius)
        {
            for (double y = -search_radius; y <= search_radius; y += search_radius)
            {
                for (double z = -search_radius; z <= search_radius; z += search_radius)
                {
                    const auto it = buckets.find(get_rgd_index_3d(p + Eigen::Vector3d(x, y, z), bucket));
                    if (it == buckets.end())
                        continue;
                    for (int index : it->second)
                    {
                        const double d = (p - target[index]).squaredNorm();
                        if (d < best)
                        {
                            best = d;
                            found = true;
                        }
                    }
                }
            }
        }
        if (found)
        {
            quality.matched++;
            sum_squared += best;
        }
    }

    if (quality.source_points > 0)
        quality.fitness = static_cast<double>(quality.matched) / quality.source_points;
    if (quality.matched > 0)
        quality.rms = std::sqrt(sum_squared / quality.matched);
    return quality;
}

bool icp_edge(
    const std::vector<Session>& sessions,
    Edge& edge,
    const std::vector<double>& search_radii,
    int number_of_iterations,
    int num_edge_extended_before,
    int num_edge_extended_after,
    EdgeQuality* quality)
{
    std::vector<Eigen::Vector3d> source;
    std::vector<Eigen::Vector3d> target;
    edge_point_clouds(sessions, edge, num_edge_extended_before, num_edge_extended_after, source, target);

    Eigen::Affine3d m_pose = affine_matrix_from_pose_tait_bryan(edge.relative_pose_tb);
    for (double search_radius : search_radii)
    {
        PairWiseICP icp;
        if (!icp.compute(source, target, search_radius, number_of_iterations, m_pose))
            return false;
    }

    edge.relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
    if (quality != nullptr && !search_radii.empty())
        *quality = edge_quality(source, target, m_pose, search_radii.back());
    return true;
}

void icp_all_edges(
    std::vector<Session>& sessions,
    std::vector<Edge>& edges,
    float search_radius,
    int num_edge_extended_before,
    int num_edge_extended_after)
{
    std::cout << "icp_all_edges" << std::endl;

    const int number_of_iterations = 10;
    for (auto& edge : edges)
        icp_edge(sessions, edge, { search_radius }, number_of_iterations, num_edge_extended_before, num_edge_extended_after);
}

std::vector<Edge> propose_edges(
    const std::vector<Session>& sessions, const std::vector<Edge>& existing, const EdgeProposalSettings& settings)
{
    std::set<std::tuple<int, int, int, int>> joined;
    for (const auto& e : existing)
        joined.insert(pair_key(e.index_session_from, e.index_from, e.index_session_to, e.index_to));

    std::vector<Edge> proposed;
    for (int session_a = 0; session_a < static_cast<int>(sessions.size()); session_a++)
    {
        if (Session::is_point_cloud_file(sessions[session_a].session_file_name))
            continue;
        const auto& clouds_a = sessions[session_a].point_clouds_container.point_clouds;

        for (int scan_a = 0; scan_a < static_cast<int>(clouds_a.size()); scan_a++)
        {
            const Eigen::Vector3d position_a = clouds_a[scan_a].m_pose.translation();

            for (int session_b = 0; session_b < static_cast<int>(sessions.size()); session_b++)
            {
                if (Session::is_point_cloud_file(sessions[session_b].session_file_name))
                    continue;
                if (sessions[session_a].is_ground_truth && sessions[session_b].is_ground_truth)
                    continue;
                const auto& clouds_b = sessions[session_b].point_clouds_container.point_clouds;

                int nearest = -1;
                double nearest_distance = settings.radius;
                for (int scan_b = 0; scan_b < static_cast<int>(clouds_b.size()); scan_b++)
                {
                    if (session_a == session_b && std::abs(scan_a - scan_b) < settings.min_scan_gap)
                        continue;
                    const Eigen::Vector3d d = clouds_b[scan_b].m_pose.translation() - position_a;
                    const double distance = std::hypot(d.x(), d.y());
                    if (distance <= nearest_distance && (nearest < 0 || distance < nearest_distance))
                    {
                        nearest = scan_b;
                        nearest_distance = distance;
                    }
                }
                if (nearest < 0)
                    continue;

                const auto key = pair_key(session_a, scan_a, session_b, nearest);
                if (!joined.insert(key).second)
                    continue;

                Edge edge;
                edge.index_session_from = std::get<0>(key);
                edge.index_from = std::get<1>(key);
                edge.index_session_to = std::get<2>(key);
                edge.index_to = std::get<3>(key);
                const auto& from = sessions[edge.index_session_from].point_clouds_container.point_clouds[edge.index_from];
                const auto& to = sessions[edge.index_session_to].point_clouds_container.point_clouds[edge.index_to];
                edge.relative_pose_tb = pose_tait_bryan_from_affine_matrix(from.m_pose.inverse() * to.m_pose);
                edge.relative_pose_tb_weights = settings.weights;
                proposed.push_back(edge);
            }
        }
    }
    return proposed;
}
