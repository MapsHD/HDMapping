#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace calib
{

    // Which projection projectPoint() applies. Selected by a "model" key in the
    // calibration JSON.
    //
    // Not every consumer honours this yet. apps/camera_lidar_calibration neither
    // writes the key (saveCalibration) nor reads it (loadCalibration /
    // loadIntrinsics -- and its OpenCV YAML input cannot express one at all), so
    // it always operates as Pinhole: opening an Equirectangular calibration there
    // would silently mis-project it, and rebuildImageTexture() would additionally
    // run initUndistortRectifyMap over a panorama. Its GLSL projection
    // (RendererShaders.h) and Renderer::drawCameraFrustum are pinhole-only too.
    // Likewise solveExtrinsicsFromCorrespondences, whose only caller is that app.
    //
    // The solver drop-in is ready when that app is picked up: the vendored
    // observation_equation_equrectangular_camera_colinearity_tait_bryan_wc[_jacobian]
    // take the same (tx,ty,tz,om,fi,ka,px,py,pz) order and 9-column layout as the
    // perspective ones, so the kCameraLidarAxisOffset pre-rotation, the
    // fixTranslation column slicing and the LM loop all carry over unchanged.
    // Only three things differ: (fx,fy,cx,cy) becomes (rows,cols,pi), the
    // jacobian is Eigen::Matrix<double, 2, 9, Eigen::RowMajor> rather than
    // column-major, and it takes two extra trailing u_kp, v_kp arguments.
    enum class CameraModel
    {
        Pinhole, // fx/fy/cx/cy + the rational distortion coefficients below
        Equirectangular // 360 panorama; width/height are the intrinsics, k*/p* unused
    };

    struct Intrinsics
    {
        CameraModel model = CameraModel::Pinhole;
        float fx = 800.f, fy = 800.f;
        float cx = 640.f, cy = 360.f;
        // OpenCV rational distortion model:
        // radial = (1 + k1 r² + k2 r⁴ + k3 r⁶) / (1 + k4 r² + k5 r⁴ + k6 r⁶)
        float k1 = 0.f, k2 = 0.f, k3 = 0.f;
        float k4 = 0.f, k5 = 0.f, k6 = 0.f;
        // tangential
        float p1 = 0.f, p2 = 0.f;
        // Image dimensions in pixels. Read only by CameraModel::Equirectangular,
        // where they play the role fx/fy/cx/cy play for a pinhole camera and so
        // *must* be set -- from the calibration file or from the loaded image --
        // before projectPoint() is called.
        int width = 0, height = 0;
    };

    // Minimum distance (degrees) fi is kept away from the om/fi/ka
    // parameterization's gimbal-lock points (fi = +/-90 deg), where om and
    // ka become individually non-unique (only om+ka, or om-ka, is
    // determined) and CameraCalibrationSolver's normal equations go
    // rank-deficient in that 2x2 block. Used by UI code that edits fi
    // interactively (see apps/camera_lidar_calibration/UI.cpp's
    // avoidGimbalLock) so a manual drag can't land exactly on the
    // singularity. Extrinsics' own default (below) no longer needs this --
    // see kCameraLidarAxisOffset -- but it's kept as a cheap safety net for
    // whatever fi a user or a loaded file lands on.
    constexpr float kGimbalLockEpsilonDeg = 0.1f;

    // Nudges fi_deg off the nearest gimbal-lock point (+/-90 deg) if it's
    // within kGimbalLockEpsilonDeg of one, in place. A no-op otherwise.
    // Safe to call unconditionally every frame after any edit to fi (manual
    // slider drag, typed value, or loaded from a file) -- idempotent.
    void avoidGimbalLock(float& fi_deg);

    // Fixed rotation baked into Extrinsics' om/fi/ka (see below): the
    // "camera axes vs LiDAR axes" alignment -- camera X=right, Y=down,
    // Z=forward matched to LiDAR X=forward, Y=left, Z=up. This is a
    // constant coordinate-convention twist that has nothing to do with the
    // actual calibration being solved for, so it's factored out as a fixed
    // offset rather than folded into om/fi/ka: om=fi=ka=0 is then already
    // the correct nominal alignment (Extrinsics' literal default), and
    // om/fi/ka become exactly "how far off nominal the real mount is" --
    // normally a few degrees at most, so nowhere near the om/fi/ka
    // parameterization's gimbal-lock points (fi=+/-90 deg) in practice,
    // unlike the old scheme where fi had to carry this entire 90-degree
    // twist directly and sat right on top of the singularity by default.
    inline const Eigen::Matrix3f kCameraLidarAxisOffset =
        (Eigen::Matrix3f() << 0.f, 0.f, 1.f, -1.f, 0.f, 0.f, 0.f, -1.f, 0.f).finished();

    struct Extrinsics
    {
        // Camera position in LiDAR/world frame
        float tx = 0.f, ty = 0.f, tz = 0.f;
        // Camera orientation in LiDAR/world frame, as a SMALL deviation from
        // the fixed kCameraLidarAxisOffset alignment: R_wc =
        // kCameraLidarAxisOffset * Rx(om) * Ry(fi) * Rz(ka). om/fi/ka are
        // degrees, Tait-Bryan, matching CameraCalibrationSolver's own
        // parameterization (om/fi/ka feed the vendored observation
        // equations directly there too -- see CameraCalibrationSolver.cpp
        // for how the offset is threaded through the solve without
        // modifying those equations).
        // Default: om=fi=ka=0, i.e. exactly the nominal alignment -- a
        // real calibration only needs to move these by however far the
        // actual camera mount deviates from nominal, typically a few
        // degrees, so "0,0,0" is already a good initial guess, not just a
        // mathematically convenient one.
        float om = 0.f, fi = 0.f, ka = 0.f;
    };

    // Rectangular region of interest, in full-resolution image pixels.
    // When enabled, only pixels inside [x, x+w) x [y, y+h) are considered valid
    // (e.g. for coloring a point cloud); everything outside is ignored.
    struct Roi
    {
        bool enabled = false;
        int x = 0, y = 0, w = 0, h = 0;
    };

    // R = kCameraLidarAxisOffset * Rx * Ry * Rz  (Tait-Bryan om/fi/ka,
    // degrees → rotation matrix). Matches Extrinsics' own om/fi/ka
    // convention above -- om=fi=ka=0 returns kCameraLidarAxisOffset exactly.
    Eigen::Matrix3f omFiKaToMat3(float om_deg, float fi_deg, float ka_deg);

    // Inverse of omFiKaToMat3: decomposes kCameraLidarAxisOffset^T * R
    // assuming that equals Rx(om)*Ry(fi)*Rz(ka), for reading a rotation
    // matrix (e.g. from a saved calibration file) back into Extrinsics'
    // om/fi/ka fields. Calibration files store the rotation as a plain
    // matrix (convention-independent, portable to any external tool, and
    // knows nothing about kCameraLidarAxisOffset), while the app's own
    // UI/solver work in om/fi/ka, so this conversion is needed at the file
    // -I/O boundary either way. Result is passed through avoidGimbalLock.
    void omFiKaFromMat3(const Eigen::Matrix3f& R, float& om_deg, float& fi_deg, float& ka_deg);

    // Intrinsics describing the same camera after its images are resampled by
    // `s` (e.g. 0.5 for half-size images): scales fx/fy/cx/cy for Pinhole and
    // width/height for Equirectangular, so a downscaled image projects with the
    // same geometry. Everything else (distortion, model) is carried over.
    Intrinsics scaleIntrinsics(const Intrinsics& K, float s);

    // Project a point from LiDAR frame to image pixel (u, v).
    // R_wc = camera orientation in world, t = camera position in world.
    //
    // Pinhole: depth = z component in camera frame (positive = in front), and
    // the function returns false for points behind the camera.
    // Equirectangular: depth = range from the camera, and only a point
    // essentially at the camera itself fails -- a full-sphere camera has no
    // frustum and no "behind". u comes back wrapped into [0, width); v spans
    // [0, height] *inclusive*, the south pole landing exactly on height.
    //
    // In both cases the caller owns rounding to integer pixels (which can itself
    // land on width at the equirectangular seam), bounds checking and any ROI
    // test.
    bool projectPoint(
        float px,
        float py,
        float pz,
        const Intrinsics& K,
        const Eigen::Matrix3f& R_wc,
        const Eigen::Vector3f& t,
        float& u,
        float& v,
        float& depth);

} // namespace calib
