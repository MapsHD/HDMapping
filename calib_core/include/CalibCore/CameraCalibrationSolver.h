#pragma once
#include "Camera.h"
#include <Eigen/Core>
#include <vector>

namespace calib
{

    // A single manually-picked correspondence: a 3D point in the LiDAR/world
    // frame paired with the pixel it should project to in the camera image.
    // Pixel coordinates are expected in the *undistorted* (ideal pinhole)
    // frame -- i.e. picked from the rectified image display, see
    // solveExtrinsicsFromCorrespondences() below.
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

} // namespace calib
