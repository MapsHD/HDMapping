#pragma once

#include <Eigen/Core>

#include <string>

//! Camera intrinsics for the Mei/unified-sphere fisheye model used by the
//! Insta360 rig (distortion_model: insta360_mei_v2 in camera_info.yaml).
//! @warning camera_info.yaml's `distortion` array is ordered
//!          (k1, k2, k3, p1, p2) -- NOT OpenCV's usual pinhole order
//!          (k1, k2, p1, p2, k3). The two are easy to mix up, both being five
//!          numbers in a row, and doing so produces a plausible-looking but
//!          badly wrong reprojection with no crash.
struct MeiCamera
{
    //! Frame this calibration belongs to, from the yaml's `frame_id`.
    std::string frameId;
    //! The yaml's `distortion_model`; only "insta360_mei_v2" is supported.
    std::string distortionModel;
    //! Image dimensions the calibration was taken at, in pixels.
    int width = 0, height = 0;

    //! Focal lengths and principal point, applied after the unit-sphere step.
    double fx = 0, fy = 0, cx = 0, cy = 0;
    //! Unified-sphere mirror parameter.
    double xi = 0;
    //! Radial (k*) and tangential (p*) distortion, in the yaml's own order.
    double k1 = 0, k2 = 0, k3 = 0, p1 = 0, p2 = 0;

    //! False when @ref LoadMeiCamera failed; every other field is then unset.
    bool loaded = false;

    //! Project a camera-frame point to pixel coordinates.
    //! @param P point in the camera frame, of any positive scale (it need not
    //!        be unit length)
    //! @return the pixel it projects to, unclamped and unwrapped
    //! @note No domain guard: past the fold-back angle the projection stops
    //!       being injective and far-off-axis directions land back inside the
    //!       image. calib::projectPoint applies that guard for its callers.
    Eigen::Vector2d Project(const Eigen::Vector3d& P) const;
};

//! Load intrinsics from a camera_info.yaml in this rig's format: a flat
//! top-level mapping of scalars plus a `distortion` flow sequence.
//! @param path file to read
//! @return the intrinsics, or a default-constructed MeiCamera with
//!         @ref MeiCamera::loaded false on any failure (missing file, missing
//!         field, unexpected distortion element count)
//! @note Failures print to stderr rather than throwing -- a malformed file
//!       should degrade the app to "no reprojection available", not crash it.
MeiCamera LoadMeiCamera(const std::string& path);
