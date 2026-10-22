#pragma once

// Step 3's session-rendering helpers, on top of the shared raylib replacement for <Core/utils.hpp>
// (<Core/raylib_utils.hpp>: camera globals, input loop, line strips, labels, picking rays).

#include <Core/raylib_render.hpp>
#include <Core/raylib_utils.hpp>
#include <Core/session.h>

#include <string>
#include <vector>

//! Draws the active Intersections-menu grid overlays (10m/1m/0.1m, per xz/yz/xy plane).
void drawIntersectionGrids(const std::vector<Session>& sessions);
void getClosestTrajectoriesPoint(
    std::vector<Session>& sessions,
    int x,
    int y,
    const int first_session_index,
    const int second_session_index,
    const int number_visible_sessions,
    int& index_loop_closure_source,
    int& index_loop_closure_target,
    bool KeyShift,
    double& time_stamp_offset);
//! Ctrl + middle/right click outside loop closure picking (like Step 2): moves the rotation center to the
//! trajectory point of a visible session closest to the mouse ray, or to the z = 0 plane if there is none.
void setNewRotationCenterToClosestTrajectoryPoint(const std::vector<Session>& sessions, int x, int y);
//! Per-session GPU buffers (ScanRenderer). syncSessionRenderers() runs once per frame; it re-uploads after
//! invalidateSessionRenderers() or when the number of sessions changes, else only scans whose pose changed.
void syncSessionRenderers(const std::vector<Session>& sessions);
void invalidateSessionRenderers();
//! Releases the per-session GPU buffers; call before shutdownGL(), while the GL context exists.
void releaseSessionRenderers();
//! Replaces PointClouds::render(): session.visible is checked by the caller.
void renderSession(const Session& session, const ObservationPicking& observation_picking, int decimate, int reduce_trajectory);
//! Replaces PointCloud::render(show_with_initial_pose, ...) for scan `index` of session `session_index`.
void renderScan(
    int session_index,
    int index,
    bool show_with_initial_pose,
    const ObservationPicking& observation_picking,
    int decimate,
    int reduce_trajectory,
    bool xz_intersection,
    bool yz_intersection,
    bool xy_intersection,
    double intersection_width,
    bool visible_imu_diff);
//! Replaces PointCloud::render(pose, ...): points drawn at `pose`, trajectory at the scan's m_pose.
//! useSceneColorMode: shade points the same way the live render does (View > Points color's
//! color_mode, with the current scene's elevation/distance bounds) instead of flat `color` -- for
//! a gizmo-drag preview that should otherwise look identical to the real render. Off (the default)
//! keeps existing callers' flat highlight color (e.g. the loop-closure edge preview's red/blue).
//! xzIntersection/yzIntersection/xyIntersection/intersectionWidth: same cross-section slicing
//! draw() applies (View > Intersections); off (the default) draws every point, for callers that
//! want an unsliced preview regardless of the current global setting (e.g. the edge highlight).
void renderScanAtPose(
    int session_index,
    int index,
    const Eigen::Affine3d& pose,
    int decimate,
    int reduce_trajectory,
    const float color[3],
    bool useSceneColorMode = false,
    bool xzIntersection = false,
    bool yzIntersection = false,
    bool xyIntersection = false,
    double intersectionWidth = 0.1);
//! Replace GroundControlPoints::render() / ControlPoints::render(pcs, false), which use legacy GL.
void renderGroundControlPoints(const GroundControlPoints& ground_control_points, const PointClouds& point_clouds_container);
void renderControlPoints(const ControlPoints& control_points, const PointClouds& point_clouds_container);

//! View > Points color: ScanRenderer's shader color modes (default: session color shaded by intensity).
void pointsColorMenu();
//! After loading: red, blue, orange, ... for the first 8 sessions, then a random vivid hue (seeded by index).
void assignDistinctSessionColors(std::vector<Session>& sessions);
