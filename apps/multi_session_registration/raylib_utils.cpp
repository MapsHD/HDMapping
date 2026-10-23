#include "raylib_utils.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <random>

namespace
{
    std::vector<std::unique_ptr<ScanRenderer>> renderers;
    bool renderers_valid = false;
    const Session* renderers_base = nullptr;
    PointClouds::PointCloudDimensions scene_dims{ 0, 0, 0, 0, 0, 1, 1, 1, 1 };
    ScanColorMode color_mode = ScanColorMode::FlatIntensity;

    Vector3 toVec3(const Eigen::Vector3d& v)
    {
        return Vector3{ static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z()) };
    }

    ScanRenderer* rendererOf(int session_index)
    {
        return session_index >= 0 && session_index < static_cast<int>(renderers.size()) ? renderers[session_index].get() : nullptr;
    }

    // Ported from PointClouds::draw_grids() (core/src/point_clouds.cpp, legacy immediate-mode GL,
    // shared with the GLUT apps so it can't be changed) -- rl*() rename, one helper per cutting
    // plane instead of one copy-pasted block per grid density. Same port as step 2's
    // drawGridXZ/YZ/XY (apps/multi_view_tls_registration/multi_view_tls_registration_gui.cpp);
    // duplicated here rather than shared because this file is itself a from-scratch per-app
    // raylib port (see the header comment), not a shared library.
    void drawGridXZ(float step, Color color, const PointClouds::PointCloudDimensions& dims)
    {
        float x_min = std::floor(dims.x_min / step) * step;
        float x_max = std::ceil(dims.x_max / step) * step;
        float z_min = std::floor(dims.z_min / step) * step;
        float z_max = std::ceil(dims.z_max / step) * step;

        rlBegin(RL_LINES);
        rlColor3f(color.r / 255.f, color.g / 255.f, color.b / 255.f);
        for (float x = x_min; x <= x_max; x += step)
        {
            rlVertex3f(x, 0.0f, z_min);
            rlVertex3f(x, 0.0f, z_max);
        }
        for (float z = z_min; z <= z_max; z += step)
        {
            rlVertex3f(x_min, 0.0f, z);
            rlVertex3f(x_max, 0.0f, z);
        }
        rlEnd();
    }

    void drawGridYZ(float step, Color color, const PointClouds::PointCloudDimensions& dims)
    {
        float y_min = std::floor(dims.y_min / step) * step;
        float y_max = std::ceil(dims.y_max / step) * step;
        float z_min = std::floor(dims.z_min / step) * step;
        float z_max = std::ceil(dims.z_max / step) * step;

        rlBegin(RL_LINES);
        rlColor3f(color.r / 255.f, color.g / 255.f, color.b / 255.f);
        for (float y = y_min; y <= y_max; y += step)
        {
            rlVertex3f(0.0f, y, z_min);
            rlVertex3f(0.0f, y, z_max);
        }
        for (float z = z_min; z <= z_max; z += step)
        {
            rlVertex3f(0.0f, y_min, z);
            rlVertex3f(0.0f, y_max, z);
        }
        rlEnd();
    }

    void drawGridXY(float step, Color color, const PointClouds::PointCloudDimensions& dims)
    {
        float x_min = std::floor(dims.x_min / step) * step;
        float x_max = std::ceil(dims.x_max / step) * step;
        float y_min = std::floor(dims.y_min / step) * step;
        float y_max = std::ceil(dims.y_max / step) * step;

        rlBegin(RL_LINES);
        rlColor3f(color.r / 255.f, color.g / 255.f, color.b / 255.f);
        for (float x = x_min; x <= x_max; x += step)
        {
            rlVertex3f(x, y_min, 0.0f);
            rlVertex3f(x, y_max, 0.0f);
        }
        for (float y = y_min; y <= y_max; y += step)
        {
            rlVertex3f(x_min, y, 0.0f);
            rlVertex3f(x_max, y, 0.0f);
        }
        rlEnd();
    }
} // namespace

// Draws whichever of the 9 grid-density/plane checkboxes (Intersections menu) are on, spanning
// the whole scene's bounding box. Step 3 has no single active session, so -- same pattern as the
// xz/yz/xy_intersection toggles -- the Intersections menu pushes these flags to every session
// identically; sessions[0] is read here only as the (representative) place they live.
void drawIntersectionGrids(const std::vector<Session>& sessions)
{
    if (sessions.empty())
        return;

    const auto& pcc = sessions[0].point_clouds_container;
    const Color light = ColorFromNormalized(Vector4{ 0.7f, 0.7f, 0.7f, 1.0f });
    const Color dark = ColorFromNormalized(Vector4{ 0.3f, 0.3f, 0.3f, 1.0f });

    if (pcc.xz_grid_10x10)
        drawGridXZ(10.0f, light, scene_dims);
    if (pcc.xz_grid_1x1)
        drawGridXZ(1.0f, dark, scene_dims);
    if (pcc.xz_grid_01x01)
        drawGridXZ(0.1f, dark, scene_dims);

    if (pcc.yz_grid_10x10)
        drawGridYZ(10.0f, light, scene_dims);
    if (pcc.yz_grid_1x1)
        drawGridYZ(1.0f, dark, scene_dims);
    if (pcc.yz_grid_01x01)
        drawGridYZ(0.1f, dark, scene_dims);

    if (pcc.xy_grid_10x10)
        drawGridXY(10.0f, light, scene_dims);
    if (pcc.xy_grid_1x1)
        drawGridXY(1.0f, dark, scene_dims);
    if (pcc.xy_grid_01x01)
        drawGridXY(0.1f, dark, scene_dims);
}

// Same behavior as utils.cpp: Ctrl picks the loop closure source (and time_stamp_offset), Shift the target;
// with more than two visible sessions all visible ones are searched. Moves the rotation center to the pick.
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
    double& time_stamp_offset)
{
    const auto laser_beam = GetLaserBeam(x, y);
    double min_distance = std::numeric_limits<double>::max();
    Vector3 center = camera.eulerGoal.rotationCenter;

    auto visit = [&](int s, bool source_target)
    {
        if (s < 0 || s >= static_cast<int>(sessions.size()))
            return;
        const auto& pcs = sessions[s].point_clouds_container.point_clouds;
        for (size_t i = 0; i < pcs.size(); i++)
            for (size_t j = 0; j < pcs[i].local_trajectory.size(); j++)
            {
                Eigen::Vector3d vp = pcs[i].m_pose * pcs[i].local_trajectory[j].m_pose.translation();
                double dist = distance_point_to_line(vp, laser_beam);
                if (dist >= min_distance)
                    continue;
                min_distance = dist;
                if (source_target && KeyShift)
                {
                    index_loop_closure_target = static_cast<int>(i);
                    continue;
                }
                if (source_target)
                    index_loop_closure_source = static_cast<int>(i);
                center = toVec3(vp);
                time_stamp_offset = pcs[i].local_trajectory[j].timestamps.first;
            }
    };

    if (number_visible_sessions == 1)
        visit(first_session_index, true);
    else if (number_visible_sessions == 2)
        visit(KeyShift ? second_session_index : first_session_index, true);
    else
        for (size_t s = 0; s < sessions.size(); s++)
            if (sessions[s].visible)
                visit(static_cast<int>(s), false);

    camera.moveEulerRotationCenterTo(center);
}

void setNewRotationCenterToClosestTrajectoryPoint(const std::vector<Session>& sessions, int x, int y)
{
    const auto laser_beam = GetLaserBeam(x, y);
    double min_distance = std::numeric_limits<double>::max();
    Eigen::Vector3d center;

    for (const auto& session : sessions)
    {
        if (!session.visible)
            continue;
        for (const auto& pc : session.point_clouds_container.point_clouds)
            for (const auto& node : pc.local_trajectory)
            {
                Eigen::Vector3d vp = pc.m_pose * node.m_pose.translation();
                double dist = distance_point_to_line(vp, laser_beam);
                if (dist < min_distance)
                {
                    min_distance = dist;
                    center = vp;
                }
            }
    }

    if (min_distance == std::numeric_limits<double>::max())
        setNewRotationCenter(x, y);
    else
        setNewRotationCenter(center);
}

void syncSessionRenderers(const std::vector<Session>& sessions)
{
    if (renderers_valid && renderers.size() == sessions.size() && renderers_base == sessions.data())
    {
        for (size_t i = 0; i < sessions.size(); i++)
            renderers[i]->syncPoses(sessions[i].point_clouds_container.point_clouds);
        return;
    }

    renderers.clear();
    bool first = true;
    for (const auto& s : sessions)
    {
        auto renderer = std::make_unique<ScanRenderer>();
        renderer->init();
        renderer->rebuildAll(s.point_clouds_container.point_clouds);
        renderers.push_back(std::move(renderer));

        if (s.point_clouds_container.point_clouds.empty())
            continue;
        const auto d = s.point_clouds_container.compute_point_cloud_dimension();
        if (first)
            scene_dims = d;
        scene_dims.x_min = std::min(scene_dims.x_min, d.x_min);
        scene_dims.x_max = std::max(scene_dims.x_max, d.x_max);
        scene_dims.y_min = std::min(scene_dims.y_min, d.y_min);
        scene_dims.y_max = std::max(scene_dims.y_max, d.y_max);
        scene_dims.z_min = std::min(scene_dims.z_min, d.z_min);
        scene_dims.z_max = std::max(scene_dims.z_max, d.z_max);
        first = false;
    }
    scene_dims.length = scene_dims.x_max - scene_dims.x_min;
    scene_dims.width = scene_dims.y_max - scene_dims.y_min;
    scene_dims.height = scene_dims.z_max - scene_dims.z_min;
    renderers_base = sessions.data();
    renderers_valid = true;
}

void invalidateSessionRenderers()
{
    renderers_valid = false;
}

void releaseSessionRenderers()
{
    renderers.clear();
    renderers_valid = false;
}

void renderSession(const Session& session, const ObservationPicking&, int decimate, int reduce_trajectory)
{
    ScanRenderer* r = rendererOf(static_cast<int>(&session - renderers_base));
    const auto& pcc = session.point_clouds_container;
    if (!r || pcc.point_clouds.empty())
        return;

    const Vector3 rc = camera.euler.rotationCenter;
    r->draw(
        pcc.point_clouds,
        static_cast<float>(pcc.point_clouds[0].point_size),
        color_mode,
        static_cast<float>(scene_dims.z_min),
        static_cast<float>(scene_dims.z_max),
        Eigen::Vector3d(rc.x, rc.y, rc.z),
        static_cast<float>(std::max({ scene_dims.length, scene_dims.width, scene_dims.height, 1.0 })),
        decimate,
        pcc.xz_intersection,
        pcc.yz_intersection,
        pcc.xy_intersection,
        static_cast<float>(pcc.intersection_width),
        pcc.show_with_initial_pose);
    r->drawTrajectories(
        pcc.point_clouds,
        reduce_trajectory,
        pcc.show_imu_to_lio_diff,
        pcc.xz_intersection,
        pcc.yz_intersection,
        pcc.xy_intersection,
        pcc.show_with_initial_pose,
        pcc.imu_to_lio_diff_scale);
}

void renderScan(
    int session_index,
    int index,
    bool show_with_initial_pose,
    const ObservationPicking&,
    int decimate,
    int reduce_trajectory,
    bool xz_intersection,
    bool yz_intersection,
    bool xy_intersection,
    double intersection_width,
    bool)
{
    ScanRenderer* r = rendererOf(session_index);
    if (!r || index < 0 || index >= static_cast<int>(renderers_base[session_index].point_clouds_container.point_clouds.size()))
        return;
    const PointCloud& pc = renderers_base[session_index].point_clouds_container.point_clouds[index];
    renderScanAtPose(
        session_index,
        index,
        show_with_initial_pose ? pc.m_initial_pose : pc.m_pose,
        decimate,
        reduce_trajectory,
        pc.render_color,
        /*useSceneColorMode=*/false,
        xz_intersection,
        yz_intersection,
        xy_intersection,
        intersection_width);
}

void renderScanAtPose(
    int session_index,
    int index,
    const Eigen::Affine3d& pose,
    int,
    int reduce_trajectory,
    const float color[3],
    bool useSceneColorMode,
    bool xzIntersection,
    bool yzIntersection,
    bool xyIntersection,
    double intersectionWidth)
{
    ScanRenderer* r = rendererOf(session_index);
    if (!r || index < 0 || index >= static_cast<int>(renderers_base[session_index].point_clouds_container.point_clouds.size()))
        return;
    const PointCloud& pc = renderers_base[session_index].point_clouds_container.point_clouds[index];
    if (!pc.visible)
        return;

    const Vector3 rc = camera.euler.rotationCenter;
    r->drawCachedWithTransform(
        static_cast<size_t>(index),
        pose * pc.m_pose.inverse(),
        ColorFromNormalized(Vector4{ color[0], color[1], color[2], 1.f }),
        static_cast<float>(pc.point_size),
        useSceneColorMode ? color_mode : ScanColorMode::Flat,
        static_cast<float>(scene_dims.z_min),
        static_cast<float>(scene_dims.z_max),
        Eigen::Vector3d(rc.x, rc.y, rc.z),
        static_cast<float>(std::max({ scene_dims.length, scene_dims.width, scene_dims.height, 1.0 })),
        xzIntersection,
        yzIntersection,
        xyIntersection,
        static_cast<float>(intersectionWidth));

    const size_t stride = std::max(1, reduce_trajectory);
    rlBegin(RL_LINES);
    rlColor3f(color[0], color[1], color[2]);
    for (size_t i = stride; i < pc.local_trajectory.size(); i += stride)
        for (size_t k : { i - stride, i })
        {
            Eigen::Vector3d p = (pc.m_pose * pc.local_trajectory[k].m_pose).translation();
            rlVertex3f(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
        }
    rlEnd();
}

void renderGroundControlPoints(const GroundControlPoints& ground_control_points, const PointClouds& point_clouds_container)
{
    const Color mark{ 179, 77, 128, 255 };
    const Color connector{ 0, 77, 153, 255 };
    for (const auto& gcp : ground_control_points.gpcs)
    {
        if (gcp.index_to_node_inner < 0 || gcp.index_to_node_inner >= static_cast<int>(point_clouds_container.point_clouds.size()))
            continue;
        const auto& pc = point_clouds_container.point_clouds[gcp.index_to_node_inner];
        if (gcp.index_to_node_outer < 0 || gcp.index_to_node_outer >= static_cast<int>(pc.local_trajectory.size()))
            continue;

        Vector3 c = toVec3(pc.m_pose * pc.local_trajectory[gcp.index_to_node_outer].m_pose.translation());
        float h = static_cast<float>(gcp.lidar_height_above_ground);
        Vector3 g{ static_cast<float>(gcp.x), static_cast<float>(gcp.y), static_cast<float>(gcp.z) };
        DrawLine3D(Vector3{ g.x - 0.05f, g.y, g.z }, Vector3{ g.x + 0.05f, g.y, g.z }, mark);
        DrawLine3D(Vector3{ g.x, g.y - 0.05f, g.z }, Vector3{ g.x, g.y + 0.05f, g.z }, mark);
        DrawLine3D(g, Vector3{ g.x, g.y, g.z + h }, mark);
        DrawLine3D(c, Vector3{ g.x, g.y, g.z + h }, connector);
        addLabel(Vector3{ g.x, g.y, g.z + h + 0.1f }, gcp.name, WHITE);
    }
}

void renderControlPoints(const ControlPoints& control_points, const PointClouds& point_clouds_container)
{
    const Color mark{ 179, 77, 128, 255 };
    const Color connector{ 0, 77, 153, 255 };
    const auto& pcs = point_clouds_container.point_clouds;
    for (const auto& cp : control_points.cps)
    {
        if (cp.index_to_pose < 0 || cp.index_to_pose >= static_cast<int>(pcs.size()))
            continue;
        Vector3 c = toVec3(pcs[cp.index_to_pose].m_pose * Eigen::Vector3d(cp.x_source_local, cp.y_source_local, cp.z_source_local));
        Vector3 g{ static_cast<float>(cp.x_target_global), static_cast<float>(cp.y_target_global), static_cast<float>(cp.z_target_global) };
        DrawLine3D(Vector3{ g.x - 0.05f, g.y, g.z }, Vector3{ g.x + 0.05f, g.y, g.z }, mark);
        DrawLine3D(Vector3{ g.x, g.y - 0.05f, g.z }, Vector3{ g.x, g.y + 0.05f, g.z }, mark);
        DrawLine3D(c, g, connector);
        addLabel(Vector3{ g.x, g.y, g.z + 0.1f }, cp.name, WHITE);
    }
}

void pointsColorMenu()
{
    if (!ImGui::BeginMenu("Points color"))
        return;
    const std::tuple<const char*, ScanColorMode, const char*> items[] = {
        { "> Session color", ScanColorMode::FlatIntensity, "Each session in its own color (Settings window), shaded by intensity" },
        { "> By intensity (gradient)", ScanColorMode::Intensity, "Per-point jet colormap from LAS/LAZ intensity" },
        { "> By height (gradient)", ScanColorMode::Elevation, "Per-point jet colormap from world Z, over all sessions" },
        { "> By distance (gradient)", ScanColorMode::Distance, "Per-point jet colormap from distance to the rotation center" },
    };
    for (const auto& [label, mode, tooltip] : items)
    {
        if (ImGui::MenuItem(label, nullptr, color_mode == mode))
            color_mode = mode;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", tooltip);
    }
    ImGui::EndMenu();
}

void assignDistinctSessionColors(std::vector<Session>& sessions)
{
    static const float palette[][3] = {
        { 0.90f, 0.10f, 0.10f }, // red
        { 0.15f, 0.45f, 0.95f }, // blue
        { 1.00f, 0.55f, 0.05f }, // orange
        { 0.15f, 0.75f, 0.20f }, // green
        { 0.85f, 0.20f, 0.85f }, // magenta
        { 0.10f, 0.85f, 0.85f }, // cyan
        { 0.95f, 0.90f, 0.10f }, // yellow
        { 0.55f, 0.30f, 0.95f }, // violet
    };
    const size_t n = sizeof(palette) / sizeof(palette[0]);

    for (size_t i = 0; i < sessions.size(); i++)
    {
        float c[3];
        if (i < n)
            std::copy(palette[i], palette[i] + 3, c);
        else
        {
            std::mt19937 rng(static_cast<unsigned>(i));
            Color rgb = ColorFromHSV(std::uniform_real_distribution<float>(0.f, 360.f)(rng), 0.85f, 0.95f);
            c[0] = rgb.r / 255.f;
            c[1] = rgb.g / 255.f;
            c[2] = rgb.b / 255.f;
        }
        std::copy(c, c + 3, sessions[i].render_color);
        for (auto& pc : sessions[i].point_clouds_container.point_clouds)
        {
            std::copy(c, c + 3, pc.render_color);
            std::copy(c, c + 3, pc.traj_color);
        }
    }
}
