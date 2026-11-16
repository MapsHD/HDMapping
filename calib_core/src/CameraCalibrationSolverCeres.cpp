#include <CalibCore/CameraCalibrationSolver.h>

// Always compiled; the #ifdef below picks between the real Ceres
// implementation and a stub that explains why it isn't available, so callers
// check solveExtrinsicsAndIntrinsicsCeres's return value rather than an #ifdef.
#ifdef CALIB_ENABLE_CERES

#include <ceres/ceres.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace calib
{
    namespace
    {
        // Ceres::Jet-compatible equivalent of Camera.cpp's omFiKaToMat3:
        // R = kCameraLidarAxisOffset * Rx(om)*Ry(fi)*Rz(ka), om/fi/ka in
        // RADIANS (Extrinsics stores degrees; solve() converts). The Rx*Ry*Rz
        // part mirrors Core/transformations.h's
        // affine_matrix_from_pose_tait_bryan. kCameraLidarAxisOffset's entries
        // are only {0, +-1}, so it is applied by permuting/negating Rdelta's
        // rows rather than a general 3x3 product: offset =
        // [[0,0,1],[-1,0,0],[0,-1,0]], so row 0 of R is row 2 of Rdelta,
        // row 1 is -(row 0), row 2 is -(row 1).
        template<typename T>
        void rotationMatrix(const T& om, const T& fi, const T& ka, T R[3][3])
        {
            const T sx = sin(om), cx = cos(om);
            const T sy = sin(fi), cy = cos(fi);
            const T sz = sin(ka), cz = cos(ka);

            T Rdelta[3][3];
            Rdelta[0][0] = cy * cz;
            Rdelta[1][0] = cz * sx * sy + cx * sz;
            Rdelta[2][0] = -cx * cz * sy + sx * sz;
            Rdelta[0][1] = -cy * sz;
            Rdelta[1][1] = cx * cz - sx * sy * sz;
            Rdelta[2][1] = cz * sx + cx * sy * sz;
            Rdelta[0][2] = sy;
            Rdelta[1][2] = -cy * sx;
            Rdelta[2][2] = cx * cy;

            for (int c = 0; c < 3; ++c)
            {
                R[0][c] = Rdelta[2][c];
                R[1][c] = -Rdelta[0][c];
                R[2][c] = -Rdelta[1][c];
            }
        }

        //! p_cam = R_wc^T * (p_world - C), from the two extrinsic parameter blocks.
        template<typename T>
        void lidarToCamera(const Eigen::Vector3d& p, const T* tx_ty_tz, const T* om_fi_ka, T pc[3])
        {
            T R[3][3];
            rotationMatrix(om_fi_ka[0], om_fi_ka[1], om_fi_ka[2], R);
            const T d[3] = { T(p.x()) - tx_ty_tz[0], T(p.y()) - tx_ty_tz[1], T(p.z()) - tx_ty_tz[2] };
            pc[0] = R[0][0] * d[0] + R[1][0] * d[1] + R[2][0] * d[2];
            pc[1] = R[0][1] * d[0] + R[1][1] * d[1] + R[2][1] * d[2];
            pc[2] = R[0][2] * d[0] + R[1][2] * d[1] + R[2][2] * d[2];
        }

        //! Templated calib::projectPoint Mei branch, for Ceres autodiff; the
        //! intrinsics are Jets too, so they can be refined.
        template<typename T>
        void projectMei(
            const T pc[3],
            const T& fx,
            const T& fy,
            const T& cx,
            const T& cy,
            const T& xi,
            const T& k1,
            const T& k2,
            const T& k3,
            const T& p1,
            const T& p2,
            T& u,
            T& v)
        {
            const T n = sqrt(pc[0] * pc[0] + pc[1] * pc[1] + pc[2] * pc[2]);
            const T Xx = pc[0] / n, Xy = pc[1] / n, Xz = pc[2] / n;
            const T denom = Xz + xi;
            const T x = Xx / denom, y = Xy / denom;
            const T r2 = x * x + y * y;
            const T radial = T(1.0) + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
            const T xd = x * radial + T(2.0) * p1 * x * y + p2 * (r2 + T(2.0) * x * x);
            const T yd = y * radial + p1 * (r2 + T(2.0) * y * y) + T(2.0) * p2 * x * y;
            u = fx * xd + cx;
            v = fy * yd + cy;
        }

        // Templated equivalent of calib::projectPoint's Fisheye branch, for
        // Ceres autodiff.
        template<typename T>
        void projectFisheye(
            const T pc[3], double fx, double fy, double cx, double cy, double k1, double k2, double k3, double k4, T& u, T& v)
        {
            const T r2 = pc[0] * pc[0] + pc[1] * pc[1];
            // theta_d / r, the factor that scales (x, y) onto the image plane.
            // sqrt's derivative is infinite at r = 0, so on the optical axis
            // use its limit 1/z instead.
            T s;
            if (r2 > T(1e-18))
            {
                const T r = sqrt(r2);
                const T theta = atan2(r, pc[2]);
                const T t2 = theta * theta;
                s = theta * (T(1.0) + t2 * (T(k1) + t2 * (T(k2) + t2 * (T(k3) + t2 * T(k4))))) / r;
            }
            else
            {
                s = T(1.0) / pc[2];
            }
            u = T(fx) * pc[0] * s + T(cx);
            v = T(fy) * pc[1] * s + T(cy);
        }

        // Reprojection residuals for one correspondence: predicted (u, v)
        // minus the picked pixel, like the Pinhole solver's observation
        // equation but autodiff'd, no vendored Mei or fisheye Jacobian
        // existing.

        //! Mei, with the intrinsics as parameter blocks (fx fy | cx cy | xi |
        //! k1 k2 k3 p1 p2), held constant unless refined.
        struct MeiReprojectionResidual
        {
            MeiReprojectionResidual(const Eigen::Vector3d& p, double u_kp, double v_kp)
                : p_(p)
                , u_kp_(u_kp)
                , v_kp_(v_kp)
            {
            }

            template<typename T>
            bool operator()(
                const T* const tx_ty_tz,
                const T* const om_fi_ka,
                const T* const focal,
                const T* const centre,
                const T* const xi,
                const T* const dist,
                T* residual) const
            {
                T pc[3];
                lidarToCamera(p_, tx_ty_tz, om_fi_ka, pc);
                T u, v;
                projectMei(pc, focal[0], focal[1], centre[0], centre[1], xi[0], dist[0], dist[1], dist[2], dist[3], dist[4], u, v);
                residual[0] = u - T(u_kp_);
                residual[1] = v - T(v_kp_);
                return true;
            }

            const Eigen::Vector3d p_;
            const double u_kp_, v_kp_;
        };

        //! Fisheye, intrinsics fixed.
        struct FisheyeReprojectionResidual
        {
            FisheyeReprojectionResidual(const Eigen::Vector3d& p, double u_kp, double v_kp, const Intrinsics& K)
                : p_(p)
                , u_kp_(u_kp)
                , v_kp_(v_kp)
                , K_(K)
            {
            }

            template<typename T>
            bool operator()(const T* const tx_ty_tz, const T* const om_fi_ka, T* residual) const
            {
                T pc[3];
                lidarToCamera(p_, tx_ty_tz, om_fi_ka, pc);
                T u, v;
                projectFisheye(pc, K_.fx, K_.fy, K_.cx, K_.cy, K_.k1, K_.k2, K_.k3, K_.k4, u, v);
                residual[0] = u - T(u_kp_);
                residual[1] = v - T(v_kp_);
                return true;
            }

            const Eigen::Vector3d p_;
            const double u_kp_, v_kp_;
            const Intrinsics K_;
        };

        // Prior observation equation on one component of a 3-parameter block:
        // (value - target) / sigma, in the block's own units (metres, or
        // radians for om/fi/ka).
        struct PriorResidual
        {
            PriorResidual(int index, double target, double invSigma)
                : index_(index)
                , target_(target)
                , invSigma_(invSigma)
            {
            }

            template<typename T>
            bool operator()(const T* const block, T* residual) const
            {
                residual[0] = (block[index_] - T(target_)) * T(invSigma_);
                return true;
            }

            const int index_;
            const double target_, invSigma_;
        };
    } // namespace

    bool solveExtrinsicsAndIntrinsicsCeres(
        const std::vector<PointPixelCorrespondence>& correspondences,
        Intrinsics& intrinsicsInOut,
        Extrinsics& extrinsicsInOut,
        const MeiIntrinsicsRefinement& refine,
        std::string& errorMessage,
        double* outRmsPixels,
        bool fixTranslation,
        bool fixRotation,
        const ExtrinsicsPriors& priors)
    {
        const Intrinsics& K = intrinsicsInOut;
        if (K.model != CameraModel::Mei && K.model != CameraModel::Fisheye)
        {
            errorMessage = std::string("The Ceres solver handles the mei and fisheye models only, not ") + modelToString(K.model);
            return false;
        }
        if (refine.count() > 0 && K.model != CameraModel::Mei)
        {
            errorMessage = std::string("Refining intrinsics is supported for the mei model only, not ") + modelToString(K.model);
            return false;
        }
        if (fixTranslation && fixRotation && refine.count() == 0)
        {
            errorMessage = "Translation and rotation are both locked -- nothing to solve";
            return false;
        }
        if (!extrinsicsPriorsValid(priors))
        {
            errorMessage = "Every enabled prior needs a sigma > 0";
            return false;
        }

        // Each correspondence gives 2 equations.
        const int nParams = (fixTranslation ? 0 : 3) + (fixRotation ? 0 : 3) + refine.count();
        const int nNeeded = std::max(3, (nParams + 1) / 2);
        if (static_cast<int>(correspondences.size()) < nNeeded)
        {
            errorMessage =
                "Need at least " + std::to_string(nNeeded) + " correspondences for " + std::to_string(nParams) + " free parameters";
            return false;
        }

        const double d2r = M_PI / 180.0;
        double txyz[3] = { extrinsicsInOut.tx, extrinsicsInOut.ty, extrinsicsInOut.tz };
        double omfika[3] = { extrinsicsInOut.om * d2r, extrinsicsInOut.fi * d2r, extrinsicsInOut.ka * d2r };
        double focal[2] = { K.fx, K.fy };
        double centre[2] = { K.cx, K.cy };
        double xi[1] = { K.xi };
        double dist[5] = { K.k1, K.k2, K.k3, K.p1, K.p2 };

        ceres::Problem problem;
        std::vector<ceres::ResidualBlockId> reprojectionBlocks;
        for (const auto& c : correspondences)
        {
            if (K.model == CameraModel::Mei)
            {
                auto* cost = new ceres::AutoDiffCostFunction<MeiReprojectionResidual, 2, 3, 3, 2, 2, 1, 5>(
                    new MeiReprojectionResidual(c.p, c.u, c.v));
                reprojectionBlocks.push_back(problem.AddResidualBlock(cost, nullptr, txyz, omfika, focal, centre, xi, dist));
            }
            else
            {
                auto* cost = new ceres::AutoDiffCostFunction<FisheyeReprojectionResidual, 2, 3, 3>(
                    new FisheyeReprojectionResidual(c.p, c.u, c.v, K));
                reprojectionBlocks.push_back(problem.AddResidualBlock(cost, nullptr, txyz, omfika));
            }
        }
        if (K.model == CameraModel::Mei)
        {
            if (!refine.focal)
                problem.SetParameterBlockConstant(focal);
            if (!refine.principalPoint)
                problem.SetParameterBlockConstant(centre);
            if (refine.xi)
                problem.SetParameterLowerBound(xi, 0, 0.0);
            else
                problem.SetParameterBlockConstant(xi);
            if (!refine.distortion)
                problem.SetParameterBlockConstant(dist);
        }
        // A locked parameter's prior is dropped: it cannot move anyway.
        for (int i = 0; i < 6; ++i)
        {
            const ParameterPrior& prior = priors[i];
            const bool isTranslation = i < 3;
            if (!prior.enabled || (isTranslation && fixTranslation) || (!isTranslation && fixRotation))
                continue;
            const double scale = isTranslation ? 1.0 : d2r;
            auto* cost = new ceres::AutoDiffCostFunction<PriorResidual, 1, 3>(
                new PriorResidual(i % 3, prior.target * scale, 1.0 / (prior.sigma * scale)));
            problem.AddResidualBlock(cost, nullptr, isTranslation ? txyz : omfika);
        }
        if (fixTranslation)
            problem.SetParameterBlockConstant(txyz);
        if (fixRotation)
            problem.SetParameterBlockConstant(omfika);

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 100;
        options.logging_type = ceres::SILENT;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        if (!summary.IsSolutionUsable())
        {
            errorMessage = "Ceres solve failed: " + summary.BriefReport();
            return false;
        }

        extrinsicsInOut.tx = static_cast<float>(txyz[0]);
        extrinsicsInOut.ty = static_cast<float>(txyz[1]);
        extrinsicsInOut.tz = static_cast<float>(txyz[2]);
        extrinsicsInOut.om = static_cast<float>(omfika[0] / d2r);
        extrinsicsInOut.fi = static_cast<float>(omfika[1] / d2r);
        extrinsicsInOut.ka = static_cast<float>(omfika[2] / d2r);
        if (refine.count() > 0)
        {
            intrinsicsInOut.fx = static_cast<float>(focal[0]);
            intrinsicsInOut.fy = static_cast<float>(focal[1]);
            intrinsicsInOut.cx = static_cast<float>(centre[0]);
            intrinsicsInOut.cy = static_cast<float>(centre[1]);
            intrinsicsInOut.xi = static_cast<float>(xi[0]);
            intrinsicsInOut.k1 = static_cast<float>(dist[0]);
            intrinsicsInOut.k2 = static_cast<float>(dist[1]);
            intrinsicsInOut.k3 = static_cast<float>(dist[2]);
            intrinsicsInOut.p1 = static_cast<float>(dist[3]);
            intrinsicsInOut.p2 = static_cast<float>(dist[4]);
        }

        // Reprojection error only, without the priors' share of the cost:
        // evaluate just the reprojection blocks. Their cost is
        // 0.5*sum(residual^2) over every SCALAR residual (2 per
        // correspondence), so the Pinhole solver's rms formula
        // sqrt(sum(du^2+dv^2) / (2*N)) simplifies to sqrt(cost/N).
        if (outRmsPixels)
        {
            ceres::Problem::EvaluateOptions evalOptions;
            evalOptions.residual_blocks = reprojectionBlocks;
            double reprojectionCost = 0.0;
            problem.Evaluate(evalOptions, &reprojectionCost, nullptr, nullptr, nullptr);
            *outRmsPixels = std::sqrt(reprojectionCost / static_cast<double>(correspondences.size()));
        }

        return true;
    }
} // namespace calib

#else // !CALIB_ENABLE_CERES

#include <iostream>

namespace calib
{
    bool solveExtrinsicsAndIntrinsicsCeres(
        const std::vector<PointPixelCorrespondence>&,
        Intrinsics&,
        Extrinsics&,
        const MeiIntrinsicsRefinement&,
        std::string& errorMessage,
        double*,
        bool,
        bool,
        const ExtrinsicsPriors&)
    {
        errorMessage = "Mei/fisheye extrinsics solving needs calib_core built with -DCALIB_ENABLE_CERES=ON (see calib_core/CMakeLists.txt)";
        std::cerr << errorMessage << std::endl;
        return false;
    }
} // namespace calib

#endif

namespace calib
{
    bool solveExtrinsicsCeres(
        const std::vector<PointPixelCorrespondence>& correspondences,
        const Intrinsics& K,
        Extrinsics& extrinsicsInOut,
        std::string& errorMessage,
        double* outRmsPixels,
        bool fixTranslation,
        bool fixRotation,
        const ExtrinsicsPriors& priors)
    {
        Intrinsics fixedK = K;
        return solveExtrinsicsAndIntrinsicsCeres(
            correspondences, fixedK, extrinsicsInOut, {}, errorMessage, outRmsPixels, fixTranslation, fixRotation, priors);
    }
} // namespace calib