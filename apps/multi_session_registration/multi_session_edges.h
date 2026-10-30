#pragma once

#include "multi_session_factor_graph.h"

#include <Core/session.h>

#include <Eigen/Eigen>

#include <vector>

// Loop-closure edges of step 3 without GUI state: shared by the GUI and the headless CLI.
// An edge's relative pose is inv(m_pose of scan index_from) * m_pose of scan index_to: it maps the points of
// the TO scan (source) into the frame of the FROM scan (target).

// How well an edge's two scans agree at a relative pose.
struct EdgeQuality
{
    double fitness = 0.0; // share of source points with a target point within the search radius
    double rms = 0.0; // RMS distance of those matches [m]
    int matched = 0;
    int source_points = 0;
};

// The edge's source and target points, each in the frame of the edge's own scan: the TO scan's points (with
// num_edge_extended_before / _after neighbouring scans of that session) and the FROM scan's points (likewise).
void edge_point_clouds(
    const std::vector<Session>& sessions,
    const Edge& edge,
    int num_edge_extended_before,
    int num_edge_extended_after,
    std::vector<Eigen::Vector3d>& source,
    std::vector<Eigen::Vector3d>& target);

// Fitness and RMS of source mapped by m_pose against target, nearest neighbour within search_radius.
EdgeQuality edge_quality(
    const std::vector<Eigen::Vector3d>& source,
    const std::vector<Eigen::Vector3d>& target,
    const Eigen::Affine3d& m_pose,
    double search_radius);

// Registers the edge with PairWiseICP at each radius of search_radii in turn, starting from its relative pose.
// The edge is updated only when every radius succeeds; quality (optional) is measured at the last radius.
bool icp_edge(
    const std::vector<Session>& sessions,
    Edge& edge,
    const std::vector<double>& search_radii,
    int number_of_iterations,
    int num_edge_extended_before,
    int num_edge_extended_after,
    EdgeQuality* quality = nullptr);

// The GUI's "ICP all edges": every edge at one search radius, 10 iterations; an edge whose ICP fails is kept as is.
void icp_all_edges(
    std::vector<Session>& sessions,
    std::vector<Edge>& edges,
    float search_radius,
    int num_edge_extended_before,
    int num_edge_extended_after);

struct EdgeProposalSettings
{
    double radius = 3.0; // sensor positions of the two scans at most this far apart in plan view [m]
    int min_scan_gap = 12; // within one session, only scans at least this many scans apart (revisits)
    TaitBryanPose weights = { 1000000.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0 }; // as a manual edge
};

// Candidate edges at the current poses: for every scan and every session, the scan of that session whose sensor
// is nearest in plan view within settings.radius. Pairs already joined by an edge of `existing`, pairs of two
// ground-truth sessions and sessions loaded from a single point cloud file are skipped. The relative pose is the
// current one: register the edges (icp_edge) before using them.
std::vector<Edge> propose_edges(
    const std::vector<Session>& sessions, const std::vector<Edge>& existing, const EdgeProposalSettings& settings);
