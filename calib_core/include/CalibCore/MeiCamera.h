#pragma once

#include <Eigen/Core>

#include <string>

// Camera intrinsics for the Mei/unified-sphere fisheye model used by the
// Insta360 rig (distortion_model: insta360_mei_v2 in camera_info.yaml).
//
// IMPORTANT: camera_info.yaml's `distortion` array is ordered
// (k1, k2, k3, p1, p2) -- NOT OpenCV's usual pinhole order (k1, k2, p1, p2,
// k3). The two are easy to mix up (both are five numbers in a row) and doing
// so produces a plausible-looking but badly wrong reprojection with no crash.
struct MeiCamera
{
    std::string frameId;
    std::string distortionModel;
    int width = 0, height = 0;

    double fx = 0, fy = 0, cx = 0, cy = 0;
    double xi = 0;
    double k1 = 0, k2 = 0, k3 = 0, p1 = 0, p2 = 0;

    bool loaded = false;

    // Camera-frame 3D point (any positive scale) -> pixel coordinates.
    Eigen::Vector2d Project(const Eigen::Vector3d& P) const;
};

// Loads intrinsics from a camera_info.yaml in this rig's format: a flat
// top-level mapping of scalars plus a `distortion` flow sequence. On any
// failure (missing file, missing field, unexpected distortion element count)
// prints to stderr and returns loaded=false -- a malformed file should
// degrade the app to "no reprojection available", not crash it.
MeiCamera LoadMeiCamera(const std::string& path);
