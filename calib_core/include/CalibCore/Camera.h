#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace calib
{

    struct Intrinsics
    {
        float fx = 800.f, fy = 800.f;
        float cx = 640.f, cy = 360.f;
        // OpenCV rational distortion model:
        // radial = (1 + k1 r² + k2 r⁴ + k3 r⁶) / (1 + k4 r² + k5 r⁴ + k6 r⁶)
        float k1 = 0.f, k2 = 0.f, k3 = 0.f;
        float k4 = 0.f, k5 = 0.f, k6 = 0.f;
        // tangential
        float p1 = 0.f, p2 = 0.f;
    };

    struct Extrinsics
    {
        // Camera position in LiDAR/world frame
        float tx = 0.f, ty = 0.f, tz = 0.f;
        // Camera orientation in LiDAR/world frame — Tait-Bryan om/fi/ka,
        // degrees: R_wc = Rx(om) * Ry(fi) * Rz(ka). This is the SAME
        // parameterization (and rotation order) as the vendored camera
        // observation equations CameraCalibrationSolver reuses directly, so
        // no conversion is needed between "what got picked/solved" and
        // "what's stored here" -- no rx/ry/rz<->om/fi/ka round-trip.
        // Default: standard camera (X=right, Y=down, Z=forward) aligned
        // with LiDAR (X=forward) -- the same physical orientation the old
        // ZYX-Euler default (rx=-90,ry=0,rz=-90) represented (within 0.1°),
        // just written in this parameterization. fi is nudged to 89.9° (not
        // exactly 90°) deliberately: at fi=90° exactly this parameterization
        // hits gimbal lock -- om and ka become individually non-unique
        // (only om+ka is determined) -- which made CameraCalibrationSolver's
        // very first solve of a fresh session start right on top of a rank
        // -deficient normal-equations block (confirmed in practice: the
        // solver visibly struggled). 0.1° off is enough to make the (om,ka)
        // block well-conditioned from the start while being visually
        // identical to "aligned".
        float om = -90.f, fi = 89.9f, ka = 0.f;
    };

    // Rectangular region of interest, in full-resolution image pixels.
    // When enabled, only pixels inside [x, x+w) x [y, y+h) are considered valid
    // (e.g. for coloring a point cloud); everything outside is ignored.
    struct Roi
    {
        bool enabled = false;
        int x = 0, y = 0, w = 0, h = 0;
    };

    // R = Rx * Ry * Rz  (Tait-Bryan om/fi/ka, degrees → rotation matrix).
    // Matches Extrinsics' own om/fi/ka convention above.
    Eigen::Matrix3f omFiKaToMat3(float om_deg, float fi_deg, float ka_deg);

    // Project a point from LiDAR frame to image pixel (u, v).
    // R_wc = camera orientation in world, t = camera position in world.
    // depth = z component in camera frame (positive = in front).
    // Returns false if depth <= 0 (behind camera).
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