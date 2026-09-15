#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <string>

namespace calib
{

    //! Which projection @ref projectPoint applies. Selected by a "model" key
    //! in the calibration JSON; absent, it is Pinhole.
    //! @note apps/camera_lidar_calibration supports Pinhole and Mei only -- it
    //!       has no Equirectangular solver, and its GLSL projection
    //!       (RendererShaders.h) and Renderer::drawCameraFrustum assume a
    //!       frustum a 360 panorama doesn't have.
    enum class CameraModel
    {
        Pinhole, // fx/fy/cx/cy + the rational distortion coefficients below
        Equirectangular, // 360 panorama; width/height are the intrinsics, k*/p* unused
        Mei // Insta 360
    };

    struct Intrinsics
    {
        CameraModel model = CameraModel::Pinhole;
        float fx = 800.f, fy = 800.f;
        float cx = 640.f, cy = 360.f;
        //! OpenCV rational distortion model (CameraModel::Pinhole):
        //! radial = (1 + k1 r² + k2 r⁴ + k3 r⁶) / (1 + k4 r² + k5 r⁴ + k6 r⁶)
        //! @note CameraModel::Mei reuses k1/k2/k3 and p1/p2 for its own
        //!       (non-rational) polynomial -- see @ref MeiCamera -- and leaves
        //!       k4/k5/k6 unused.
        float k1 = 0.f, k2 = 0.f, k3 = 0.f;
        float k4 = 0.f, k5 = 0.f, k6 = 0.f;
        //! Tangential distortion.
        float p1 = 0.f, p2 = 0.f;
        //! Unified-sphere mirror parameter, CameraModel::Mei only.
        //! @see MeiCamera
        float xi = 0.f;
        //! Read only by CameraModel::Equirectangular, where they play the role
        //! fx/fy/cx/cy play for a pinhole camera and so *must* be set before
        //! @ref projectPoint is called.
        int width = 0, height = 0;
    };

    //! Name of a camera model, as written to the calibration JSON's "model" key.
    //! @param m model to name
    //! @return one of "pinhole", "equirectangular", "mei"
    const char* modelToString(CameraModel m);

    //! Camera model named by a calibration JSON's "model" key.
    //! @param s model name, as written by @ref modelToString
    //! @return the named model, or CameraModel::Pinhole for anything
    //!         unrecognized (including an absent key)
    CameraModel modelFromString(const std::string& s);

    //! Minimum distance (degrees) fi is kept away from the om/fi/ka
    //! parameterization's gimbal-lock points (fi = +/-90 deg), where om and ka
    //! become individually non-unique (only om+ka, or om-ka, is determined)
    //! and CameraCalibrationSolver's normal equations go rank-deficient in
    //! that 2x2 block. Used by UI code that edits fi interactively so a manual
    //! drag can't land exactly on the singularity.
    //! @note @ref Extrinsics' own default no longer needs this -- see
    //!       kCameraLidarAxisOffset -- but it is kept as a cheap safety net
    //!       for whatever fi a user or a loaded file lands on.
    constexpr float kGimbalLockEpsilonDeg = 0.1f;

    //! Nudges fi_deg off the nearest gimbal-lock point (+/-90 deg) if it is
    //! within @ref kGimbalLockEpsilonDeg of one. A no-op otherwise.
    //! @param fi_deg angle to adjust, in place
    //! @note Idempotent, so it is safe to call unconditionally every frame
    //!       after any edit to fi (slider drag, typed value, or file load).
    void avoidGimbalLock(float& fi_deg);

    //! Fixed rotation baked into @ref Extrinsics' om/fi/ka: the "camera axes
    //! vs LiDAR axes" alignment -- camera X=right, Y=down, Z=forward matched
    //! to LiDAR X=forward, Y=left, Z=up.
    //! @note This constant coordinate-convention twist has nothing to do with
    //!       the calibration being solved for, so it is factored out rather
    //!       than folded into om/fi/ka. om=fi=ka=0 is then already the correct
    //!       nominal alignment, and om/fi/ka become exactly "how far off
    //!       nominal the real mount is" -- a few degrees at most, so nowhere
    //!       near fi=+/-90 deg in practice, unlike the old scheme where fi
    //!       carried the whole 90-degree twist and sat on the singularity.
    inline const Eigen::Matrix3f kCameraLidarAxisOffset =
        (Eigen::Matrix3f() << 0.f, 0.f, 1.f, -1.f, 0.f, 0.f, 0.f, -1.f, 0.f).finished();

    struct Extrinsics
    {
        //! Camera position in the LiDAR/world frame.
        float tx = 0.f, ty = 0.f, tz = 0.f;
        //! Camera orientation in the LiDAR/world frame, as a SMALL deviation
        //! from the fixed kCameraLidarAxisOffset alignment:
        //! R_wc = kCameraLidarAxisOffset * Rx(om) * Ry(fi) * Rz(ka). Degrees,
        //! Tait-Bryan, matching CameraCalibrationSolver's parameterization.
        //! @note Default om=fi=ka=0 is exactly the nominal alignment, so a real
        //!       calibration only moves these by however far the mount deviates
        //!       from nominal -- typically a few degrees. "0,0,0" is therefore
        //!       a good initial guess, not just a convenient one.
        float om = 0.f, fi = 0.f, ka = 0.f;
    };

    //! Rectangular region of interest, in full-resolution image pixels.
    //! When enabled, only pixels inside [x, x+w) x [y, y+h) are considered
    //! valid (e.g. for coloring a point cloud); everything outside is ignored.
    struct Roi
    {
        bool enabled = false;
        int x = 0, y = 0, w = 0, h = 0;
    };

    //! R = kCameraLidarAxisOffset * Rx * Ry * Rz (Tait-Bryan om/fi/ka).
    //! Matches @ref Extrinsics' own om/fi/ka convention.
    //! @param om_deg,fi_deg,ka_deg Tait-Bryan angles in degrees
    //! @return the rotation matrix; om=fi=ka=0 returns kCameraLidarAxisOffset
    //!         exactly
    Eigen::Matrix3f omFiKaToMat3(float om_deg, float fi_deg, float ka_deg);

    //! Inverse of @ref omFiKaToMat3: decomposes kCameraLidarAxisOffset^T * R
    //! assuming that equals Rx(om)*Ry(fi)*Rz(ka), for reading a rotation
    //! matrix (e.g. from a saved calibration file) back into @ref Extrinsics'
    //! om/fi/ka fields.
    //! @param R rotation matrix to decompose
    //! @param om_deg,fi_deg,ka_deg receive the Tait-Bryan angles, in degrees,
    //!        passed through @ref avoidGimbalLock
    //! @note Calibration files store the rotation as a plain matrix
    //!       (convention-independent, portable, and knowing nothing about
    //!       kCameraLidarAxisOffset) while the UI and solver work in om/fi/ka,
    //!       so this conversion is needed at the file-I/O boundary either way.
    void omFiKaFromMat3(const Eigen::Matrix3f& R, float& om_deg, float& fi_deg, float& ka_deg);

    //! The same camera after its images are resampled, so a downscaled image
    //! projects with the same geometry. Distortion terms are dimensionless and
    //! carry over unchanged.
    //! @param K intrinsics at the original resolution
    //! @param s resample factor (0.5 = half size)
    //! @return intrinsics valid for the resampled image
    Intrinsics scaleIntrinsics(const Intrinsics& K, float s);

    //! The same rectangle on a resampled image, so a ROI -- given in
    //! full-resolution pixels, see @ref Roi -- can be tested against a
    //! downscaled copy.
    //! @param r rectangle in full-resolution pixels
    //! @param s resample factor (0.5 = half size)
    //! @return the scaled rectangle
    //! @note An unset (w/h == 0) ROI comes back unchanged, and a set one never
    //!       collapses to empty, which callers would read as "no ROI".
    Roi scaleRoi(const Roi& r, float s);

    //! Project a point from the LiDAR frame to an image pixel, applying
    //! whichever model K.model selects.
    //! @param px,py,pz point in the LiDAR frame
    //! @param K camera intrinsics; K.model picks the projection
    //! @param R_wc camera orientation in world
    //! @param t camera position in world
    //! @param u,v receive the image pixel
    //! @param depth receives the camera-frame z for Pinhole, range from the
    //!        camera for Equirectangular and Mei
    //! @return false when the point does not project: behind the camera for
    //!         Pinhole, at the camera itself for Equirectangular, and either
    //!         of those or past the fold-back angle (where the projection
    //!         stops being injective) for Mei
    //! @note Equirectangular wraps u into [0, width); v spans [0, height]
    //!       *inclusive*, the south pole landing exactly on height.
    //! @note The caller owns rounding to integer pixels (which can land on
    //!       width at the equirectangular seam), bounds checking and any ROI
    //!       test.
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
