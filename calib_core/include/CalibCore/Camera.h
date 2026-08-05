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
        // Camera orientation in LiDAR/world frame — ZYX Euler, degrees.
        // Default: standard camera (X=right, Y=down, Z=forward) aligned with LiDAR (X=forward).
        float rx = -90.f, ry = 0.f, rz = -90.f;
    };

    // Rectangular region of interest, in full-resolution image pixels.
    // When enabled, only pixels inside [x, x+w) x [y, y+h) are considered valid
    // (e.g. for coloring a point cloud); everything outside is ignored.
    struct Roi
    {
        bool enabled = false;
        int x = 0, y = 0, w = 0, h = 0;
    };

    // R = Rz * Ry * Rx  (ZYX Euler, degrees → rotation matrix)
    Eigen::Matrix3f eulerZYXtoMat3(float rx_deg, float ry_deg, float rz_deg);

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