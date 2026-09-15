#pragma once
#include "Camera.h"
#include <Eigen/Core>
#include <string>
#include <vector>

namespace calib
{

    // A single manually-picked correspondence: a 3D point in the LiDAR/world
    // frame paired with the pixel it should project to in the camera image.
    // Pixel coordinates are expected in whatever frame the displayed image is
    // in: undistorted/ideal-pinhole for CameraModel::Pinhole (picked from the
    // rectified display), raw/distorted for CameraModel::Mei, whose image is
    // never rectified.
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
    // Pinhole only: the reused observation equations are a pure rectilinear
    // perspective projection, with no distortion and no unified-sphere term.
    // See solveExtrinsicsMeiCeres() below for Mei.
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

    // CameraModel::Mei counterpart to solveExtrinsicsFromCorrespondences().
    // No vendored analytic Jacobian exists for the unified-sphere model, so
    // this minimizes reprojection error with Ceres' automatic differentiation,
    // holding K fixed and solving the same (tx,ty,tz,om,fi,ka) Extrinsics with
    // the same fixTranslation meaning.
    //
    // `errorMessage` is set on failure and left untouched on success. It is
    // required rather than defaulted -- hence its position ahead of the
    // optional parameters -- so a failure reason is never silently dropped.
    //
    // Needs -DCALIB_ENABLE_CERES=ON (OFF by default). Built without it, this
    // returns false and says so in `errorMessage`, so callers never need an
    // #ifdef of their own.
    bool solveExtrinsicsMeiCeres(
        const std::vector<PointPixelCorrespondence>& correspondences,
        const Intrinsics& K,
        Extrinsics& extrinsicsInOut,
        std::string& errorMessage,
        double* outRmsPixels = nullptr,
        bool fixTranslation = false);

} // namespace calib
