#pragma once
#include "Camera.h"
#include <Eigen/Core>
#include <string>
#include <vector>

namespace calib
{

    // A single manually-picked correspondence: a 3D point in the LiDAR/world
    // frame paired with the pixel it should project to in the camera image.
    // Pixel coordinates are expected in whatever frame the displayed image
    // itself is in -- the undistorted/ideal-pinhole frame for
    // CameraModel::Pinhole (picked from the rectified image display), or
    // the raw (distorted) frame for CameraModel::Mei, whose image is never
    // rectified (see AppState::rebuildImageTexture in
    // apps/camera_lidar_calibration) -- matching whichever solver below is
    // used for that model.
    struct PointPixelCorrespondence
    {
        Eigen::Vector3d p;
        double u = 0.0, v = 0.0;
    };

    // Solves for the extrinsics (camera position + orientation) that best
    // explain the given LiDAR-point <-> image-pixel correspondences via
    // damped Gauss-Newton (Levenberg-Marquardt) on the reused observation
    // equations. Intrinsics (fx, fy, cx, cy) are held fixed at their
    // current values in K. extrinsicsInOut is used as the initial guess and
    // is overwritten with the solved result. Pixel coordinates in
    // `correspondences` must be in the undistorted/ideal-pinhole frame --
    // i.e. picked from the rectified image display (calib::Intrinsics's
    // distortion terms are ignored here).
    //
    // Pinhole only -- the reused observation equations are a pure
    // rectilinear perspective projection with no distortion and no unified-
    // sphere term, so this cannot be used for CameraModel::Mei (or
    // CameraModel::Equirectangular, not wired into any app yet). See
    // solveExtrinsicsMeiCeres() below for Mei.
    //
    // fixTranslation=true blocks tx/ty/tz from being solved for -- they
    // stay pinned at extrinsicsInOut's initial values and only orientation
    // (3-DOF) is optimized. Useful when the camera position relative to the
    // LiDAR is already known precisely (e.g. measured by hand) and only
    // orientation needs refining from the picked pairs.
    //
    // Returns false (leaving extrinsicsInOut unchanged) if there are fewer
    // than 3 correspondences, or fewer than the number of free parameters
    // (3 with fixTranslation, else 6), or the normal-equations system is
    // singular.
    bool solveExtrinsicsFromCorrespondences(
        const std::vector<PointPixelCorrespondence>& correspondences,
        const Intrinsics& K,
        Extrinsics& extrinsicsInOut,
        double* outRmsPixels = nullptr,
        bool fixTranslation = false);

    // CameraModel::Mei counterpart to solveExtrinsicsFromCorrespondences()
    // above: no vendored analytic Jacobian exists for the unified-sphere
    // model (unlike Pinhole's), so this minimizes reprojection error with
    // Ceres' automatic differentiation instead of a hand-derived one,
    // reusing K's fx/fy/cx/cy/xi/k1/k2/k3/p1/p2 fixed and solving the same
    // (tx,ty,tz,om,fi,ka) Extrinsics this file's Pinhole solver does, with
    // the same fixTranslation meaning.
    //
    // `errorMessage` is set on failure (degenerate input, Ceres failing to
    // converge, or this build not having Ceres at all -- see below) and
    // left untouched on success. It is a required, non-defaulted parameter
    // -- hence its position ahead of the optional ones -- so that a failure
    // reason is never silently dropped.
    //
    // Only available when calib_core is built with -DCALIB_ENABLE_CERES=ON
    // (see calib_core/CMakeLists.txt) -- OFF by default, since HDMapping
    // otherwise depends on nothing but Eigen for its own optimization (see
    // the project README). Built without it, this always returns false and
    // sets `errorMessage` to say so, rather than requiring callers to
    // `#ifdef` around calling it at all.
    bool solveExtrinsicsMeiCeres(
        const std::vector<PointPixelCorrespondence>& correspondences,
        const Intrinsics& K,
        Extrinsics& extrinsicsInOut,
        std::string& errorMessage,
        double* outRmsPixels = nullptr,
        bool fixTranslation = false);

} // namespace calib
