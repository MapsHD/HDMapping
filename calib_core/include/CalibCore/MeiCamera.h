#pragma once

#include <opencv2/core.hpp>

#include <string>

// Camera intrinsics for the Mei/unified-sphere fisheye model used by the
// Insta360 rig this data comes from (distortion_model: insta360_mei_v2 in
// camera_info.yaml). Field meanings and the forward projection formula below
// are taken from /home/michal/code/insta3360-to-images
// (include/insta360/calibration.hpp, src/equirect.cpp), which cross-checked
// them against an independent reverse-engineering effort and validated them
// visually against Insta360's own equirect export — not re-derived from
// scratch here.
//
// IMPORTANT: camera_info.yaml's `distortion` array is ordered
// (k1, k2, k3, p1, p2) — NOT OpenCV's usual pinhole order (k1, k2, p1, p2,
// k3). The two conventions are trivially easy to mix up (both are just five
// numbers in a row) and doing so produces a plausible-looking but badly wrong
// reprojection with no crash — see calibration.hpp's own comment on this.
struct MeiCamera {
    std::string frameId;
    std::string distortionModel;
    int width = 0, height = 0;

    double fx = 0, fy = 0, cx = 0, cy = 0;
    double xi = 0;
    double k1 = 0, k2 = 0, k3 = 0, p1 = 0, p2 = 0;

    bool loaded = false;

    // Forward: camera-frame 3D point (any positive scale, need not be unit
    // length) -> pixel coordinates.
    cv::Point2d Project(cv::Point3d P) const;

    // Inverse: pixel -> unit-length ray direction in camera frame. Closed
    // form for the unit-sphere/xi step, iterative (Newton) for the
    // radial/tangential part, matching equirect.cpp's forward formula.
    // If thetaDeg is given, receives the angle from the optical axis (0 =
    // dead center, useful as a "how far into the fisheye edge is this point"
    // sanity check).
    cv::Point3d Unproject(cv::Point2d uv, double* thetaDeg = nullptr) const;
};

// Loads intrinsics from a camera_info.yaml written in this rig's format (see
// data/camera_info.yaml for a sample). On any failure (missing file, missing
// field, distortion array with an unexpected element count) prints a message
// to stderr and returns a default-constructed MeiCamera with loaded=false —
// a missing/malformed file should degrade the app to "no reprojection
// available", not crash it.
MeiCamera LoadMeiCamera(const std::string& path);
