#include <Core/raylib_render.hpp>
#include <Core/transformations.h>

#include "external/glad.h"
#include "raymath.h"
#include "rlgl.h"

#include "raylib_render_shaders.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

namespace
{
    // Points/poses are used in HDMapping's native Z-up world frame directly (no
    // Y-up remap) -- see the header comment for why.
    inline Vector3 toVec3(const Eigen::Vector3d& p)
    {
        return Vector3{ static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()) };
    }
} // namespace

// ============================================================================
// ScanRenderer
// ============================================================================
namespace
{
    using raylib_render_shaders::kPointFS;
    using raylib_render_shaders::kPointVS;

    // Bytes per vertex in the uploaded buffer (xyz position + intensity).
    constexpr int kVertexStride = 4 * sizeof(float);
} // namespace

ScanRenderer::~ScanRenderer()
{
    shutdown();
}

void ScanRenderer::init()
{
    shader_ = LoadShaderFromMemory(kPointVS, kPointFS.c_str());
    shaderValid_ = shader_.id > 0;
    if (shaderValid_)
    {
        locMVP_ = rlGetLocationUniform(shader_.id, "mvp");
        locPointSize_ = rlGetLocationUniform(shader_.id, "pointSize");
        locColor_ = rlGetLocationUniform(shader_.id, "pointColor");
        locColorMode_ = rlGetLocationUniform(shader_.id, "colorMode");
        locElevMin_ = rlGetLocationUniform(shader_.id, "elevMin");
        locElevMax_ = rlGetLocationUniform(shader_.id, "elevMax");
        locDistCenter_ = rlGetLocationUniform(shader_.id, "distCenter");
        locDistMax_ = rlGetLocationUniform(shader_.id, "distMax");
    }
    else
    {
        TraceLog(LOG_ERROR, "ScanRenderer: point shader failed to compile");
    }

    // Required for gl_PointSize (set from the vertex shader) to take effect
    // under a core-profile context.
    glEnable(GL_PROGRAM_POINT_SIZE);
}

void ScanRenderer::shutdown()
{
    for (auto& c : clouds_)
    {
        unload(c);
    }
    clouds_.clear();

    for (auto& t : trajClouds_)
    {
        unloadTraj(t);
    }
    trajClouds_.clear();

    if (shaderValid_)
    {
        UnloadShader(shader_);
        shaderValid_ = false;
    }
}

void ScanRenderer::unload(CloudGPU& cloud)
{
    if (cloud.vao)
    {
        rlUnloadVertexArray(cloud.vao);
        cloud.vao = 0;
    }
    if (cloud.vbo)
    {
        rlUnloadVertexBuffer(cloud.vbo);
        cloud.vbo = 0;
    }
    cloud.count = 0;
}

void ScanRenderer::syncCount(const std::vector<PointCloud>& pointClouds)
{
    if (clouds_.size() > pointClouds.size())
    {
        for (size_t i = pointClouds.size(); i < clouds_.size(); ++i)
        {
            unload(clouds_[i]);
        }
    }
    clouds_.resize(pointClouds.size());
}

void ScanRenderer::rebuild(size_t index, const PointCloud& pc)
{
    if (index >= clouds_.size())
    {
        return;
    }

    CloudGPU& gpu = clouds_[index];
    unload(gpu);

    if (pc.points_local.empty() || !shaderValid_)
    {
        // Still record the pose so syncPoses() doesn't retry this scan every
        // frame -- there's nothing to rebuild until it has points anyway.
        gpu.lastPose = pc.m_pose;
        gpu.hasPose = true;
        return;
    }

    // Every point in points_local is uploaded -- no stride/skip here, so
    // loading a session always renders it at full point density.
    // Session::load's own voxel-bucket decimation, controlled by the
    // Settings panel's "Downsample during load" checkbox, is unrelated and
    // untouched -- it's shared unchanged with the legacy GLUT app.
    const bool hasIntensity = pc.intensities.size() == pc.points_local.size();
    float minIntensity = std::numeric_limits<float>::max();
    float maxIntensity = -std::numeric_limits<float>::max();
    if (hasIntensity)
    {
        for (unsigned short raw : pc.intensities)
        {
            float v = static_cast<float>(raw);
            minIntensity = std::min(minIntensity, v);
            maxIntensity = std::max(maxIntensity, v);
        }
    }
    const float intensityRange = (hasIntensity && maxIntensity > minIntensity) ? (maxIntensity - minIntensity) : 1.0f;

    std::vector<float> data;
    data.reserve(pc.points_local.size() * 4);
    for (size_t i = 0; i < pc.points_local.size(); ++i)
    {
        Eigen::Vector3d world = pc.m_pose * pc.points_local[i];
        Vector3 rp = toVec3(world);
        data.push_back(rp.x);
        data.push_back(rp.y);
        data.push_back(rp.z);
        data.push_back(hasIntensity ? (static_cast<float>(pc.intensities[i]) - minIntensity) / intensityRange : 0.0f);
    }

    if (data.empty())
    {
        return;
    }

    gpu.vao = rlLoadVertexArray();
    rlEnableVertexArray(gpu.vao);
    gpu.vbo = rlLoadVertexBuffer(data.data(), static_cast<int>(data.size() * sizeof(float)), false);
    rlSetVertexAttribute(0, 3, RL_FLOAT, false, kVertexStride, 0);
    rlEnableVertexAttribute(0);
    rlSetVertexAttribute(1, 1, RL_FLOAT, false, kVertexStride, 3 * sizeof(float));
    rlEnableVertexAttribute(1);
    rlDisableVertexArray();

    gpu.count = static_cast<int>(data.size() / 4);
    gpu.lastPose = pc.m_pose;
    gpu.hasPose = true;
}

void ScanRenderer::rebuildAll(const std::vector<PointCloud>& pointClouds)
{
    syncCount(pointClouds);
    for (size_t i = 0; i < pointClouds.size(); ++i)
    {
        rebuild(i, pointClouds[i]);
    }
}

void ScanRenderer::syncPoses(const std::vector<PointCloud>& pointClouds)
{
    syncCount(pointClouds);
    for (size_t i = 0; i < pointClouds.size(); ++i)
    {
        const CloudGPU& gpu = clouds_[i];
        if (!gpu.hasPose || !gpu.lastPose.isApprox(pointClouds[i].m_pose, 1e-9))
        {
            rebuild(i, pointClouds[i]);
        }
    }
}

void ScanRenderer::draw(
    const std::vector<PointCloud>& pointClouds,
    float pointSize,
    ScanColorMode colorMode,
    float elevationMin,
    float elevationMax,
    const Eigen::Vector3d& distanceCenter,
    float distanceMax,
    int decimateStride) const
{
    lastDrawCallCount_ = 0;
    lastVertexCount_ = 0;

    if (!shaderValid_)
    {
        return;
    }

    // Flush raylib's own pending immediate-mode batch (grid/lines) before
    // issuing raw draw calls, so draw order stays correct.
    rlDrawRenderBatchActive();

    Matrix mvp = MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());

    rlEnableShader(shader_.id);
    rlSetUniformMatrix(locMVP_, mvp);
    rlSetUniform(locPointSize_, &pointSize, RL_SHADER_UNIFORM_FLOAT, 1);
    rlSetUniform(locElevMin_, &elevationMin, RL_SHADER_UNIFORM_FLOAT, 1);
    rlSetUniform(locElevMax_, &elevationMax, RL_SHADER_UNIFORM_FLOAT, 1);
    float distCenterF[3] = { static_cast<float>(distanceCenter.x()),
                             static_cast<float>(distanceCenter.y()),
                             static_cast<float>(distanceCenter.z()) };
    rlSetUniform(locDistCenter_, distCenterF, RL_SHADER_UNIFORM_VEC3, 1);
    rlSetUniform(locDistMax_, &distanceMax, RL_SHADER_UNIFORM_FLOAT, 1);

    // decimateStride > 1 skips points by widening the vertex attribute
    // stride the GPU fetches from (e.g. stride=10 reads every 10th vertex),
    // rather than by re-uploading a thinned-out buffer -- so it's a pure
    // per-draw-call setting with no CPU cost and no ScanRenderer::rebuild()
    // needed when the user changes it. Mirrors the legacy GLUT app's
    // "Points render downsampling" (viewer_decimate_point_cloud) stride
    // skip, ported here as a GPU-side fetch stride instead of a CPU-side
    // point-array stride since points are already resident on the GPU.
    const int stride = std::max(1, decimateStride);
    const int byteStride = kVertexStride * stride;

    for (size_t i = 0; i < clouds_.size() && i < pointClouds.size(); ++i)
    {
        const PointCloud& pc = pointClouds[i];
        const CloudGPU& gpu = clouds_[i];
        if (!pc.visible || gpu.count == 0)
        {
            continue;
        }

        float color[4];
        int colorModeInt;
        if (gpu.hasMarkColor)
        {
            color[0] = gpu.markColor.r / 255.f;
            color[1] = gpu.markColor.g / 255.f;
            color[2] = gpu.markColor.b / 255.f;
            color[3] = gpu.markColor.a / 255.f;
            colorModeInt = 0;
        }
        else
        {
            color[0] = pc.render_color[0];
            color[1] = pc.render_color[1];
            color[2] = pc.render_color[2];
            color[3] = 1.0f;
            // Matches the shader's colorMode branches (0=flat, 1=intensity,
            // 2=elevation, 3=distance) exactly, since ScanColorMode's
            // enumerator order was chosen to match.
            colorModeInt = static_cast<int>(colorMode);
        }
        rlSetUniform(locColor_, color, RL_SHADER_UNIFORM_VEC4, 1);
        rlSetUniform(locColorMode_, &colorModeInt, RL_SHADER_UNIFORM_INT, 1);

        rlEnableVertexArray(gpu.vao);
        // Re-specifies both attributes' stride against the VAO's cached VBO
        // binding every draw (cheap: a handful of GL calls per visible
        // scan), since the VAO otherwise keeps whatever stride rebuild()
        // baked in and there's no persistent "decimated" VAO to switch to.
        rlEnableVertexBuffer(gpu.vbo);
        rlSetVertexAttribute(0, 3, RL_FLOAT, false, byteStride, 0);
        rlEnableVertexAttribute(0);
        rlSetVertexAttribute(1, 1, RL_FLOAT, false, byteStride, 3 * sizeof(float));
        rlEnableVertexAttribute(1);

        const int drawCount = (gpu.count + stride - 1) / stride;
        glDrawArrays(GL_POINTS, 0, drawCount);
        ++lastDrawCallCount_;
        lastVertexCount_ += drawCount;
        rlDisableVertexArray();
    }

    rlDisableShader();
}

void ScanRenderer::setMarkColor(size_t index, Color color)
{
    if (index < clouds_.size())
    {
        clouds_[index].hasMarkColor = true;
        clouds_[index].markColor = color;
    }
}

void ScanRenderer::clearMarkColor(size_t index)
{
    if (index < clouds_.size())
    {
        clouds_[index].hasMarkColor = false;
    }
}

void ScanRenderer::clearMarks()
{
    for (auto& c : clouds_)
    {
        c.hasMarkColor = false;
    }
}

namespace
{
    // Converts an Eigen::Affine3d (in HDMapping's native Z-up world frame, used
    // as-is -- see the header comment) to the equivalent raylib Matrix. No
    // coordinate remap needed (unlike an earlier version of this function),
    // since points/poses are no longer converted to a separate Y-up space.
    Matrix toRaylibMatrix(const Eigen::Affine3d& t)
    {
        const Eigen::Matrix3d& r = t.linear();
        const Eigen::Vector3d& tr = t.translation();

        Matrix m{};
        m.m0 = static_cast<float>(r(0, 0));
        m.m4 = static_cast<float>(r(0, 1));
        m.m8 = static_cast<float>(r(0, 2));
        m.m12 = static_cast<float>(tr.x());
        m.m1 = static_cast<float>(r(1, 0));
        m.m5 = static_cast<float>(r(1, 1));
        m.m9 = static_cast<float>(r(1, 2));
        m.m13 = static_cast<float>(tr.y());
        m.m2 = static_cast<float>(r(2, 0));
        m.m6 = static_cast<float>(r(2, 1));
        m.m10 = static_cast<float>(r(2, 2));
        m.m14 = static_cast<float>(tr.z());
        m.m3 = 0.f;
        m.m7 = 0.f;
        m.m11 = 0.f;
        m.m15 = 1.f;
        return m;
    }
} // namespace

void ScanRenderer::drawCachedWithTransform(
    size_t index, const Eigen::Affine3d& extraTransform, Color color, float pointSize, bool useIntensityColor) const
{
    if (!shaderValid_ || index >= clouds_.size())
    {
        return;
    }

    const CloudGPU& gpu = clouds_[index];
    if (gpu.count == 0)
    {
        return;
    }

    rlDrawRenderBatchActive();

    Matrix mvpBase = MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());
    // MatrixMultiply(A, B) composes as "apply A, then B" (see draw()'s own
    // modelview-then-projection use above) -- so this applies the delta
    // transform to the cached points first, then the normal view/projection.
    Matrix mvp = MatrixMultiply(toRaylibMatrix(extraTransform), mvpBase);

    float colorF[4] = { color.r / 255.f, color.g / 255.f, color.b / 255.f, color.a / 255.f };
    int colorMode = useIntensityColor ? 1 : 0;

    rlEnableShader(shader_.id);
    rlSetUniformMatrix(locMVP_, mvp);
    rlSetUniform(locPointSize_, &pointSize, RL_SHADER_UNIFORM_FLOAT, 1);
    rlSetUniform(locColor_, colorF, RL_SHADER_UNIFORM_VEC4, 1);
    rlSetUniform(locColorMode_, &colorMode, RL_SHADER_UNIFORM_INT, 1);

    rlEnableVertexArray(gpu.vao);
    // Explicitly re-specified (not just inherited from the VAO's last
    // binding) since draw() rebinds this same VAO's attributes with a
    // decimation-widened stride every frame -- this draw always wants every
    // cached point, independent of the current "sparse drawing" setting.
    rlEnableVertexBuffer(gpu.vbo);
    rlSetVertexAttribute(0, 3, RL_FLOAT, false, kVertexStride, 0);
    rlEnableVertexAttribute(0);
    rlSetVertexAttribute(1, 1, RL_FLOAT, false, kVertexStride, 3 * sizeof(float));
    rlEnableVertexAttribute(1);

    glDrawArrays(GL_POINTS, 0, gpu.count);
    rlDisableVertexArray();
    rlDisableShader();
}

namespace
{
    constexpr double RAD_TO_DEG = 180.0 / M_PI;

    // Ported from the "fuse_inclination_from_IMU" / "fixed_om && fixed_fi" /
    // "show_IMU" / "show_pose" quad-and-cross markers in
    // core/src/point_cloud.cpp's PointCloud::render().
    void drawSquareOutline(const Eigen::Affine3d& m, double half, Color color)
    {
        Eigen::Vector3d a1 = m * Eigen::Vector3d(-half, -half, 0);
        Eigen::Vector3d a2 = m * Eigen::Vector3d(half, -half, 0);
        Eigen::Vector3d a3 = m * Eigen::Vector3d(half, half, 0);
        Eigen::Vector3d a4 = m * Eigen::Vector3d(-half, half, 0);

        Vector3 p1 = toVec3(a1);
        Vector3 p2 = toVec3(a2);
        Vector3 p3 = toVec3(a3);
        Vector3 p4 = toVec3(a4);

        DrawLine3D(p1, p2, color);
        DrawLine3D(p2, p3, color);
        DrawLine3D(p3, p4, color);
        DrawLine3D(p4, p1, color);
    }

    void drawOrientationCross(const Eigen::Affine3d& m)
    {
        Vector3 origin = toVec3(m.translation());
        DrawLine3D(origin, toVec3(m.translation() + m.linear().col(0)), RED);
        DrawLine3D(origin, toVec3(m.translation() + m.linear().col(1)), GREEN);
        DrawLine3D(origin, toVec3(m.translation() + m.linear().col(2)), BLUE);
    }

    // The IMU-fused orientation (used by both the fuse-inclination quad marker
    // and show_IMU) mirrors the local pose's translation with orientation taken
    // from the first local_trajectory node's raw IMU om/fi/ka instead of the
    // LIO-optimized pose.
    Eigen::Affine3d imuOrientationAtPose(const PointCloud& pc)
    {
        TaitBryanPose tb;
        tb.px = pc.m_pose(0, 3);
        tb.py = pc.m_pose(1, 3);
        tb.pz = pc.m_pose(2, 3);
        tb.om = pc.local_trajectory[0].imu_om_fi_ka.x();
        tb.fi = pc.local_trajectory[0].imu_om_fi_ka.y();
        tb.ka = pc.local_trajectory[0].imu_om_fi_ka.z();
        return affine_matrix_from_pose_tait_bryan(tb);
    }
} // namespace

void ScanRenderer::unloadTraj(TrajGPU& traj) const
{
    if (traj.vao)
    {
        rlUnloadVertexArray(traj.vao);
        traj.vao = 0;
    }
    if (traj.vbo)
    {
        rlUnloadVertexBuffer(traj.vbo);
        traj.vbo = 0;
    }
    traj.vertexCount = 0;
}

void ScanRenderer::rebuildTrajectoryGPU(TrajGPU& traj, const PointCloud& pc, int stride) const
{
    unloadTraj(traj);

    traj.lastPose = pc.m_pose;
    traj.hasPose = true;
    traj.builtStride = stride;
    traj.builtPointCount = pc.local_trajectory.size();

    if (pc.local_trajectory.empty())
    {
        return;
    }

    // Drawn as points (GL_POINTS, sized via gl_PointSize in the shared point
    // shader) rather than a line/ribbon/cylinder between them -- simplest
    // possible trajectory geometry, one vertex per decimated pose, no
    // connecting geometry to regenerate or to zigzag with sensor jitter.
    std::vector<float> data;
    data.reserve((pc.local_trajectory.size() / stride + 1) * 3);
    for (size_t i = 0; i < pc.local_trajectory.size(); i += stride)
    {
        Vector3 p = toVec3((pc.m_pose * pc.local_trajectory[i].m_pose).translation());
        data.push_back(p.x);
        data.push_back(p.y);
        data.push_back(p.z);
    }

    if (data.empty())
    {
        return;
    }

    traj.vao = rlLoadVertexArray();
    rlEnableVertexArray(traj.vao);
    traj.vbo = rlLoadVertexBuffer(data.data(), static_cast<int>(data.size() * sizeof(float)), false);
    rlSetVertexAttribute(0, 3, RL_FLOAT, false, 3 * sizeof(float), 0);
    rlEnableVertexAttribute(0);
    rlDisableVertexArray();

    traj.vertexCount = static_cast<int>(data.size() / 3);
}

void ScanRenderer::drawTrajectories(const std::vector<PointCloud>& pointClouds, int reduceRenderedTrajectory, bool visibleImuDiff) const
{
    int stride = reduceRenderedTrajectory < 1 ? 1 : reduceRenderedTrajectory;

    if (trajClouds_.size() > pointClouds.size())
    {
        for (size_t i = pointClouds.size(); i < trajClouds_.size(); ++i)
        {
            unloadTraj(trajClouds_[i]);
        }
    }
    trajClouds_.resize(pointClouds.size());

    if (shaderValid_)
    {
        rlDrawRenderBatchActive();
        Matrix mvp = MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());
        rlEnableShader(shader_.id);
        rlSetUniformMatrix(locMVP_, mvp);
        int colorModeFlat = 0;
        rlSetUniform(locColorMode_, &colorModeFlat, RL_SHADER_UNIFORM_INT, 1);
    }

    for (size_t idx = 0; idx < pointClouds.size(); ++idx)
    {
        const PointCloud& pc = pointClouds[idx];
        if (!pc.visible || pc.local_trajectory.empty())
        {
            continue;
        }

        if (visibleImuDiff)
        {
            for (size_t i = 1; i < pc.local_trajectory.size(); ++i)
            {
                Eigen::Affine3d m = pc.m_pose * pc.local_trajectory[i].m_pose;
                Vector3 origin = toVec3(m.translation());
                const auto& diff = pc.local_trajectory[i].imu_diff_angle_om_fi_ka_deg;
                DrawLine3D(origin, toVec3(m.translation() + Eigen::Vector3d(diff.x() * 10, 0, 0)), RED);
                DrawLine3D(origin, toVec3(m.translation() + Eigen::Vector3d(0, diff.y() * 10, 0)), GREEN);
                DrawLine3D(origin, toVec3(m.translation() + Eigen::Vector3d(0, 0, diff.z() * 10)), BLUE);
            }
        }

        if (shaderValid_ && pc.line_width > 0)
        {
            TrajGPU& traj = trajClouds_[idx];
            bool stale = !traj.hasPose || !traj.lastPose.isApprox(pc.m_pose, 1e-9) || traj.builtStride != stride ||
                traj.builtPointCount != pc.local_trajectory.size();
            if (stale)
            {
                rebuildTrajectoryGPU(traj, pc, stride);
            }

            if (traj.vertexCount > 0)
            {
                float colorF[4] = { pc.traj_color[0], pc.traj_color[1], pc.traj_color[2], 1.0f };
                rlSetUniform(locColor_, colorF, RL_SHADER_UNIFORM_VEC4, 1);
                float pointSize = static_cast<float>(pc.line_width);
                rlSetUniform(locPointSize_, &pointSize, RL_SHADER_UNIFORM_FLOAT, 1);
                rlEnableVertexArray(traj.vao);
                glDrawArrays(GL_POINTS, 0, traj.vertexCount);
                rlDisableVertexArray();
            }
        }

        if (pc.fuse_inclination_from_IMU)
        {
            drawSquareOutline(pc.m_pose, 0.2, GREEN);
            drawSquareOutline(imuOrientationAtPose(pc), 0.2, RED);
        }

        if (pc.fixed_om && pc.fixed_fi)
        {
            for (double x = 0.4; x <= 1.0; x += 0.1)
            {
                drawSquareOutline(pc.m_pose, x, RED);
            }
        }

        if (pc.show_IMU)
        {
            drawOrientationCross(imuOrientationAtPose(pc));
        }

        if (pc.show_pose)
        {
            drawOrientationCross(pc.m_pose);
        }
    }

    if (shaderValid_)
    {
        rlDisableShader();
    }
}

// Mini compass + ruler: ported to gui.cpp's own drawMiniCompassWithRuler()
// (it needs this app's own viewLocal/translate_z globals, not a Camera3D),
// so no longer lives here.
