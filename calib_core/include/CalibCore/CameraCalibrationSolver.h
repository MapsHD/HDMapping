#pragma once
#include "Camera.h"
#include <Eigen/Core>
#include <array>
#include <string>
#include <vector>

namespace calib
{

    //! A single manually-picked correspondence: a 3D point in the LiDAR/world
    //! frame paired with the pixel it should project to in the camera image.
    //! @note Pixel coordinates are expected in whatever frame the displayed
    //!       image is in: undistorted/ideal-pinhole for CameraModel::Pinhole
    //!       (picked from the rectified display), raw/distorted for
    //!       CameraModel::Mei and CameraModel::Fisheye, whose images are never
    //!       rectified.
    struct PointPixelCorrespondence
    {
        //! Point in the LiDAR/world frame.
        Eigen::Vector3d p;
        //! Pixel it should project to.
        double u = 0.0, v = 0.0;
    };

    /**
     * @brief Soft constraint (prior) on one extrinsic parameter: an extra
     *        observation equation (value - target) / sigma alongside the
     *        reprojection residuals, which are in pixels. A parameter sigma
     *        away from its target costs as much as one pixel of reprojection
     *        error, so it can still move when the pairs demand it, unlike a
     *        lock. Meant for directions the picked pairs constrain poorly,
     *        such as the camera's height (tz) when every point lies near the
     *        horizon.
     */
    struct ParameterPrior
    {
        //! Add the observation equation for this parameter.
        bool enabled = false;
        //! Value the parameter is pulled towards: metres for tx/ty/tz,
        //! degrees for om/fi/ka, in the same frame as Extrinsics.
        double target = 0.0;
        //! Allowed deviation, metres for tx/ty/tz, degrees for om/fi/ka.
        //! Must be > 0 when enabled.
        double sigma = 0.01;
    };

    /**
     * @brief One ParameterPrior per Extrinsics parameter, indexed as
     *        kPriorTx..kPriorKa. All disabled by default.
     */
    using ExtrinsicsPriors = std::array<ParameterPrior, 6>;
    enum ExtrinsicsPriorIndex
    {
        kPriorTx = 0,
        kPriorTy,
        kPriorTz,
        kPriorOm,
        kPriorFi,
        kPriorKa
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
    //! @param fixRotation pin om/fi/ka at their initial values and optimize
    //!        position only (3-DOF) -- useful when the camera orientation is
    //!        already known precisely
    //! @param priors soft constraints pulling individual parameters towards
    //!        a target (see ParameterPrior); ignored for a locked parameter.
    //!        outRmsPixels reports the reprojection error only, without them
    //! @return false, leaving extrinsicsInOut unchanged, for fewer than 3
    //!         correspondences, fewer correspondences than free parameters
    //!         (3 with fixTranslation or fixRotation, else 6), both fix flags
    //!         set (nothing left to solve), an enabled prior with sigma <= 0,
    //!         or a singular system
    //! @note Pinhole only: the reused observation equations are a pure
    //!       rectilinear perspective projection, with no distortion and no
    //!       unified-sphere term. @see solveExtrinsicsCeres
    bool solveExtrinsicsFromCorrespondences(
        const std::vector<PointPixelCorrespondence>& correspondences,
        const Intrinsics& K,
        Extrinsics& extrinsicsInOut,
        double* outRmsPixels = nullptr,
        bool fixTranslation = false,
        bool fixRotation = false,
        const ExtrinsicsPriors& priors = {});

    //! CameraModel::Mei and CameraModel::Fisheye counterpart to
    //! @ref solveExtrinsicsFromCorrespondences. No vendored analytic Jacobian
    //! exists for either model, so this minimizes reprojection error with
    //! Ceres' automatic differentiation, solving the same (tx,ty,tz,om,fi,ka)
    //! Extrinsics.
    //! @param correspondences picked pairs; pixel coordinates are in the raw
    //!        (distorted) frame, since neither model's image is rectified
    //! @param K intrinsics, held fixed; K.model must be Mei or Fisheye
    //! @param extrinsicsInOut initial guess in, solved result out
    //! @param errorMessage set on failure, left untouched on success. Required
    //!        rather than defaulted -- hence its position ahead of the
    //!        optional parameters -- so a failure reason is never silently
    //!        dropped
    //! @param outRmsPixels optionally receives the RMS reprojection error
    //! @param fixTranslation as in @ref solveExtrinsicsFromCorrespondences
    //! @param fixRotation as in @ref solveExtrinsicsFromCorrespondences
    //! @param priors as in @ref solveExtrinsicsFromCorrespondences
    //! @return false on failure, with the reason in errorMessage -- including
    //!         any other K.model, both fix flags set, or an enabled prior with
    //!         sigma <= 0
    //! @note Needs -DCALIB_ENABLE_CERES=ON (OFF by default). Built without it
    //!       this always returns false and says so, so callers never need an
    //!       \#ifdef of their own.
    bool solveExtrinsicsCeres(
        const std::vector<PointPixelCorrespondence>& correspondences,
        const Intrinsics& K,
        Extrinsics& extrinsicsInOut,
        std::string& errorMessage,
        double* outRmsPixels = nullptr,
        bool fixTranslation = false,
        bool fixRotation = false,
        const ExtrinsicsPriors& priors = {});

    /**
     * @brief Which CameraModel::Mei intrinsics @ref solveExtrinsicsAndIntrinsicsCeres
     *        refines along with the extrinsics; the rest stay as given.
     */
    struct MeiIntrinsicsRefinement
    {
        bool focal = false; //!< fx, fy
        bool principalPoint = false; //!< cx, cy
        bool xi = false; //!< unified-sphere parameter, kept >= 0
        bool distortion = false; //!< k1, k2, k3, p1, p2

        //! @return how many intrinsics are refined
        int count() const
        {
            return (focal ? 2 : 0) + (principalPoint ? 2 : 0) + (xi ? 1 : 0) + (distortion ? 5 : 0);
        }
    };

    /**
     * @brief @ref solveExtrinsicsCeres that also refines the CameraModel::Mei
     *        intrinsics `refine` selects.
     * @param correspondences picked pairs, as in solveExtrinsicsCeres; each gives
     *        2 equations, so at least half as many as there are free parameters
     * @param intrinsicsInOut intrinsics in, refined ones replaced on success;
     *        model Mei or Fisheye, and Mei when refining any
     * @param extrinsicsInOut initial guess in, solved result out
     * @param refine intrinsics to refine; none behaves as solveExtrinsicsCeres
     * @param errorMessage set on failure, as in solveExtrinsicsCeres
     * @param outRmsPixels optionally receives the RMS reprojection error
     * @param fixTranslation as in @ref solveExtrinsicsFromCorrespondences
     * @param fixRotation as in @ref solveExtrinsicsFromCorrespondences; with
     *        fixTranslation too, only the intrinsics are solved
     * @param priors as in @ref solveExtrinsicsFromCorrespondences
     * @return false, leaving both in/out parameters unchanged, on the failures
     *         solveExtrinsicsCeres has, or when refining a model other than Mei
     * @note xi trades off against fx/fy, and a few pairs barely pin the
     *       distortion: refine only what the pairs support.
     */
    bool solveExtrinsicsAndIntrinsicsCeres(
        const std::vector<PointPixelCorrespondence>& correspondences,
        Intrinsics& intrinsicsInOut,
        Extrinsics& extrinsicsInOut,
        const MeiIntrinsicsRefinement& refine,
        std::string& errorMessage,
        double* outRmsPixels = nullptr,
        bool fixTranslation = false,
        bool fixRotation = false,
        const ExtrinsicsPriors& priors = {});

    /**
     * @brief Checks that every enabled prior has a positive sigma.
     * @param priors priors to check
     * @return true when the priors can be passed to a solver
     */
    bool extrinsicsPriorsValid(const ExtrinsicsPriors& priors);

} // namespace calib
