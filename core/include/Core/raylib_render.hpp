#pragma once

// GPU (rlgl-based) point cloud rendering, shared across any raylib-based
// HDMapping GUI (currently just multi_view_tls_registration, step2 --
// raylib-based since it was ported off GLUT/legacy-GL). This is the
// raylib-app equivalent of the role core/src/utils.cpp plays for the
// remaining GLUT apps: shared GL-context-specific rendering glue that
// individual tools' GUI code builds on top of.
//
// Core's own PointCloud::render()/PointClouds::render() are legacy
// immediate-mode OpenGL (glBegin/glVertex/...), unavailable under raylib's
// OpenGL 3.3 core-profile context, so ScanRenderer below is new code rather
// than a port of those methods -- it reads the same public PointCloud data
// (points_local, m_pose, render_color, visible, ...) instead.
//
// Coordinates are used as-is (no Y-up/Z-up remap): the one consumer of this
// header drives its own Z-up camera (rlMatrixMode/rlFrustum/rlMultMatrixf
// against Eigen-computed matrices, matching the original GLUT app's
// convention) rather than raylib's native Y-up Camera3D, so points are
// uploaded and drawn in HDMapping's native Z-up world frame directly.
//
// Only built when raylib is fetched (see cmake/raylib.cmake and
// core/CMakeLists.txt's core_raylib target) -- this header is not usable
// from the GLUT apps.

#include "raylib.h"

#include <Core/point_clouds.h>
#include <Core/session.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

// ============================================================================
// ScanRenderer -- GPU (rlgl-based) rendering of Core::PointCloud scans.
// ============================================================================
// Per-point GPU color modes for ScanRenderer::draw() -- jet-colormap
// gradients computed in the shader from data already on the GPU (per-point
// intensity, or world-space position for Elevation/Distance), rather than
// per-scan flat colors.
enum class ScanColorMode
{
    Flat, // pc.render_color, uniform per scan (the original app's only mode)
    Intensity, // jet colormap by normalized LAS/LAZ intensity
    Elevation, // jet colormap by world-space Z, normalized over [elevationMin, elevationMax]
    Distance, // jet colormap by distance from distanceCenter, normalized over [0, distanceMax]
};

class ScanRenderer
{
public:
    ScanRenderer() = default;
    ~ScanRenderer();

    ScanRenderer(const ScanRenderer&) = delete;
    ScanRenderer& operator=(const ScanRenderer&) = delete;

    // Must be called once, after the raylib window/GL context exists.
    void init();
    void shutdown();

    // Rebuilds the world-space GPU point+intensity buffer for a single scan
    // from its current points_local/m_pose/intensities. Call whenever a
    // scan's pose or point data changes.
    void rebuild(size_t index, const PointCloud& pc);

    // Keeps the internal per-scan GPU buffer list in sync with
    // point_clouds_container.point_clouds (adds/removes entries, does not
    // rebuild existing ones).
    void syncCount(const std::vector<PointCloud>& pointClouds);

    // Rebuilds every scan's GPU buffer (e.g. after a session load).
    void rebuildAll(const std::vector<PointCloud>& pointClouds);

    // Rebuilds only the GPU buffers whose scan's m_pose has changed since
    // the last call (a plain 4x4 comparison per scan -- cheap even for
    // hundreds of scans). Safety net against pose-mutating call sites that
    // forget to call rebuild()/rebuildAll(): call this once per frame,
    // right before draw().
    void syncPoses(const std::vector<PointCloud>& pointClouds);

    // colorMode switches every scan from its flat render_color to a
    // per-point jet-colormap gradient (see ScanColorMode). Intensity is
    // normalized per scan at rebuild() time (raw intensity range varies by
    // sensor); Elevation/Distance are normalized using the elevationMin/Max
    // and distanceCenter/distanceMax parameters, which the caller computes
    // once per frame from whatever it considers the current scene bounds
    // (e.g. session_dims.z_min/z_max, rotation_center) -- ScanRenderer has
    // no notion of session bounds itself. A scan with a mark color set (see
    // setMarkColor()) always draws flat in that color instead, regardless
    // of colorMode, so it stays visible as a highlight either way.
    // decimateStride > 1 draws only every Nth point of each scan (a GPU-side
    // vertex fetch stride, not a re-upload -- see the .cpp), for interactive
    // framerates while navigating very large sessions; 1 draws every point
    // (rebuild()'s default). Uses whatever rlgl projection/modelview
    // matrices are currently active (BeginMode3D or a manually-driven
    // rlMatrixMode/rlMultMatrixf stack, either works).
    void draw(
        const std::vector<PointCloud>& pointClouds,
        float pointSize,
        ScanColorMode colorMode,
        float elevationMin = 0.f,
        float elevationMax = 1.f,
        const Eigen::Vector3d& distanceCenter = Eigen::Vector3d::Zero(),
        float distanceMax = 1.f,
        int decimateStride = 1) const;

    // Number of glDrawArrays calls draw() issued the last time it ran (one
    // per visible scan) -- raylib/rlgl don't expose a draw-call counter for
    // custom, non-batched GL calls like these (rlgl's own internal
    // drawCounter only tracks its immediate-mode batch renderer), so this
    // app-level count is the closest equivalent for a "draw calls" stat.
    int lastDrawCallCount() const
    {
        return lastDrawCallCount_;
    }

    // Total points actually submitted to the GPU across all draw() calls
    // last frame (i.e. sum of each visible scan's decimated drawCount) --
    // the vertex-count counterpart to lastDrawCallCount() above.
    int lastVertexCount() const
    {
        return lastVertexCount_;
    }

    // Overrides a scan's draw() color with a flat mark color -- e.g. to
    // highlight the loop-closure source/target scans in red/blue -- drawn
    // from the same cached full-resolution GPU buffer as a normal scan
    // (just a different uniform), rather than needing a separate ad-hoc
    // redraw pass. Persists across frames until cleared; callers that only
    // want a highlight for the current frame (e.g. because the highlighted
    // set can change every frame) should call clearMarks() first. Silently
    // ignored if index is out of range (e.g. before syncCount() has caught
    // up with a just-added scan).
    void setMarkColor(size_t index, Color color);
    void clearMarkColor(size_t index);
    void clearMarks();

    // Ported from PointCloud::render()'s trajectory section (core/src/point_cloud.cpp):
    // draws each visible scan's local_trajectory (posed by m_pose) as points
    // (GL_POINTS, sized via pc.line_width -- skipped entirely if
    // pc.line_width <= 0) in its traj_color, decimated by
    // reduceRenderedTrajectory (mirrors "viewer_reduce_rendered_trajectory").
    // Positions are cached per scan (see TrajGPU below) and only rebuilt
    // when a scan's pose, trajectory point count or the stride changes --
    // line_width only affects the draw call's point-size uniform, not the
    // cached geometry, so changing it alone needs no rebuild.
    // Also draws the fuse-inclination-from-IMU quad markers, the fixed-om/fi
    // rings, and the show_IMU/show_pose orientation crosses. If
    // visibleImuDiff is set, also draws the IMU-vs-LIO angular-difference
    // debug lines. Intersection-slab gating (xz/yz/xy) is not implemented so
    // these always draw when the relevant per-scan flag is set.
    void drawTrajectories(const std::vector<PointCloud>& pointClouds, int reduceRenderedTrajectory, bool visibleImuDiff) const;

    // Draws a single already-cached scan (see rebuild()) straight from its
    // persistent, full-resolution GPU buffer -- no CPU re-transform or
    // re-upload -- displaced by an extra transform on top of the pose it
    // was cached at. extraTransform is the delta from the scan's current
    // m_pose to the desired preview pose (i.e. previewPose * m_pose.inverse());
    // it's folded into the MVP matrix for this draw call rather than applied
    // to the points themselves. For overlays that need to preview a scan at
    // a pose other than its stored one without paying for a CPU retransform
    // every frame (e.g. a loop-closure edge's in-progress relative pose). If
    // useIntensityColor is set, points use the same jet colormap draw()'s
    // colorByIntensity uses; otherwise every point draws flat in color
    // (color.a is used as alpha either way). Does nothing if index is out
    // of range or the scan has no cached buffer yet.
    void drawCachedWithTransform(
        size_t index, const Eigen::Affine3d& extraTransform, Color color, float pointSize, bool useIntensityColor) const;

    // A caller-owned GPU buffer for an arbitrary world-space point set drawn
    // via drawPoints() below -- for overlays that aren't part of any
    // PointCloud (e.g. an externally loaded TUM trajectory) but still want
    // trajectory-style point dots (GL_POINTS, sized via the same pointSize
    // shader uniform drawTrajectories() uses) instead of a thin rlgl line.
    // Default-constructed as empty/unallocated; the caller uploadPoints()s
    // into it once (or whenever its source data actually changes) and
    // drawPoints()s it every frame, rather than rebuilding a VAO/VBO every
    // frame. Caller must unloadPoints() it before destruction (e.g. in its
    // owner's own shutdown/destructor) to avoid leaking the VAO/VBO.
    struct PointsGPU
    {
        unsigned int vao = 0;
        unsigned int vbo = 0;
        int vertexCount = 0;
    };

    // (Re)uploads positions into gpu, replacing whatever it held before.
    // Call only when positions actually changed -- e.g. once after loading a
    // new TUM trajectory, not unconditionally every frame.
    void uploadPoints(PointsGPU& gpu, const std::vector<Eigen::Vector3d>& positions) const;

    // Draws a previously uploaded PointsGPU as GL_POINTS, flat-colored. Safe
    // to call every frame -- issues one glDrawArrays against the existing
    // buffer, no allocation. Does nothing if gpu is empty or the shader
    // failed to load.
    void drawPoints(const PointsGPU& gpu, Color color, float pointSize) const;

    // Releases gpu's VAO/VBO, resetting it back to empty/unallocated.
    void unloadPoints(PointsGPU& gpu) const;

private:
    struct CloudGPU
    {
        unsigned int vao = 0;
        unsigned int vbo = 0;
        int count = 0;
        Eigen::Affine3d lastPose = Eigen::Affine3d::Identity();
        bool hasPose = false;
        bool hasMarkColor = false;
        Color markColor = WHITE;
    };

    void unload(CloudGPU& cloud);

    // GPU cache for one scan's trajectory point positions (drawTrajectories())
    // -- a position-only VBO drawn with GL_POINTS, rebuilt only when stale.
    struct TrajGPU
    {
        unsigned int vao = 0;
        unsigned int vbo = 0;
        int vertexCount = 0;
        Eigen::Affine3d lastPose = Eigen::Affine3d::Identity();
        bool hasPose = false;
        int builtStride = -1;
        size_t builtPointCount = 0;
    };

    // Both take the cache entry by reference rather than an index so they
    // can be called from the const drawTrajectories() (lazy rebuild-if-stale
    // on an otherwise-const draw path, same rationale as lastDrawCallCount_/
    // lastVertexCount_ below being mutable) without needing non-const
    // access to *this itself.
    void unloadTraj(TrajGPU& traj) const;
    void rebuildTrajectoryGPU(TrajGPU& traj, const PointCloud& pc, int stride) const;

    mutable std::vector<TrajGPU> trajClouds_;

    std::vector<CloudGPU> clouds_;
    Shader shader_{};
    bool shaderValid_ = false;
    int locMVP_ = -1;
    int locPointSize_ = -1;
    int locColor_ = -1;
    int locColorMode_ = -1;
    int locElevMin_ = -1;
    int locElevMax_ = -1;
    int locDistCenter_ = -1;
    int locDistMax_ = -1;
    mutable int lastDrawCallCount_ = 0;
    mutable int lastVertexCount_ = 0;
};
