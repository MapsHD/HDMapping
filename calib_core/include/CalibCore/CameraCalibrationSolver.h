#pragma once
#include "Camera.h"
#include <Eigen/Core>
#include <string>
#include <vector>

namespace calib
{

    //! A single manually-picked correspondence: a 3D point in the LiDAR/world
    //! frame paired with the pixel it should project to in the camera image.
    //! @note Pixel coordinates are expected in whatever frame the displayed
    //!       image is in: undistorted/ideal-pinhole for CameraModel::Pinhole
    //!       (picked from the rectified display), raw/distorted for
    //!       CameraModel::Mei, whose image is never rectified.
    struct PointPixelCorrespondence
    {
        //! Point in the LiDAR/world frame.
        Eigen::Vector3d p;
        //! Pixel it should project to.
        double u = 0.0, v = 0.0;
    };

    //! Solve for the extrinsics (camera position + orientation) that best
    //! explain the given LiDAR-point <-> image-pixel correspondences, via
    //! damped Gauss-Newton (Levenberg-Marquardt) on the reused observation
    //! equations.
    //! @param correspondences picked pairs; pixel coordinates must be in the
    //!        undistorted/ideal-pinhole frame, i.e. picked from the rectified
    //!        image display (@ref Intrinsics' distortion terms are ignored)
    //! @param K intrinsics, held fixed at fx/fy/cx/cy
    //! @param extrinsicsInOut initial guess in, solved result out
    //! @param outRmsPixels optionally receives the RMS reprojection error
    //! @param fixTranslation pin tx/ty/tz at their initial values and optimize
    //!        orientation only (3-DOF) -- useful when the camera position
    //!        relative to the LiDAR is already known precisely
    //! @return false, leaving extrinsicsInOut unchanged, for fewer than 3
    //!         correspondences, fewer correspondences than free parameters
    //!         (3 with fixTranslation, else 6), or a singular system
    //! @note Pinhole only: the reused observation equations are a pure
    //!       rectilinear perspective projection, with no distortion and no
    //!       unified-sphere term. @see solveExtrinsicsMeiCeres
    bool solveExtrinsicsFromCorrespondences(
        const std::vector<PointPixelCorrespondence>& correspondences,
        const Intrinsics& K,
        Extrinsics& extrinsicsInOut,
        double* outRmsPixels = nullptr,
        bool fixTranslation = false);

    //! CameraModel::Mei counterpart to @ref solveExtrinsicsFromCorrespondences.
    //! No vendored analytic Jacobian exists for the unified-sphere model, so
    //! this minimizes reprojection error with Ceres' automatic
    //! differentiation, solving the same (tx,ty,tz,om,fi,ka) Extrinsics.
    //! @param correspondences picked pairs; pixel coordinates are in the raw
    //!        (distorted) frame, since a Mei image is never rectified
    //! @param K intrinsics, held fixed
    //! @param extrinsicsInOut initial guess in, solved result out
    //! @param errorMessage set on failure, left untouched on success. Required
    //!        rather than defaulted -- hence its position ahead of the
    //!        optional parameters -- so a failure reason is never silently
    //!        dropped
    //! @param outRmsPixels optionally receives the RMS reprojection error
    //! @param fixTranslation as in @ref solveExtrinsicsFromCorrespondences
    //! @return false on failure, with the reason in errorMessage
    //! @note Needs -DCALIB_ENABLE_CERES=ON (OFF by default). Built without it
    //!       this always returns false and says so, so callers never need an
    //!       \#ifdef of their own.
    bool solveExtrinsicsMeiCeres(
        const std::vector<PointPixelCorrespondence>& correspondences,
        const Intrinsics& K,
        Extrinsics& extrinsicsInOut,
        std::string& errorMessage,
        double* outRmsPixels = nullptr,
        bool fixTranslation = false);

} // namespace calib
