#include <CalibCore/CameraCalibrationSolver.h>

// Always compiled (see calib_core/CMakeLists.txt); the #ifdef below picks
// between a real Ceres-based implementation and a stub that just explains
// why it isn't available, so callers (apps/camera_lidar_calibration's
// AppState::solvePairs) never need an #ifdef of their own around calling
// solveExtrinsicsMeiCeres -- only its return value.
#ifdef CALIB_ENABLE_CERES

#include <ceres/ceres.h>

#include <cmath>

namespace calib
{
    namespace
    {
        // Templated (Ceres::Jet-compatible) equivalent of Camera.cpp's
        // omFiKaToMat3: R = kCameraLidarAxisOffset * Rx(om)*Ry(fi)*Rz(ka),
        // om/fi/ka in RADIANS (Extrinsics' own fields are degrees -- solve()
        // below converts at the boundary). The Rx*Ry*Rz part mirrors
        // Core/transformations.h's affine_matrix_from_pose_tait_bryan
        // row-for-row rather than re-deriving it; kCameraLidarAxisOffset
        // (Camera.h) is applied by permuting/negating Rdelta's rows
        // directly instead of a general 3x3*3x3 product, since its own
        // entries are just {0, +-1}: offset = [[0,0,1],[-1,0,0],[0,-1,0]],
        // so row 0 of R is row 2 of Rdelta, row 1 is -(row 0), row 2 is
        // -(row 1).
        template <typename T>
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

        // Templated equivalent of MeiCamera::Project (CalibCore/
        // MeiCamera.h) for Ceres autodiff -- same formula, not re-derived;
        // intrinsics are plain doubles (fixed, not solved for), only pc is
        // the Jet-typed autodiff variable.
        template <typename T>
        void projectMei(
            const T pc[3],
            double fx,
            double fy,
            double cx,
            double cy,
            double xi,
            double k1,
            double k2,
            double k3,
            double p1,
            double p2,
            T& u,
            T& v)
        {
            const T n = sqrt(pc[0] * pc[0] + pc[1] * pc[1] + pc[2] * pc[2]);
            const T Xx = pc[0] / n, Xy = pc[1] / n, Xz = pc[2] / n;
            const T denom = Xz + T(xi);
            const T x = Xx / denom, y = Xy / denom;
            const T r2 = x * x + y * y;
            const T radial = T(1.0) + T(k1) * r2 + T(k2) * r2 * r2 + T(k3) * r2 * r2 * r2;
            const T xd = x * radial + T(2.0 * p1) * x * y + T(p2) * (r2 + T(2.0) * x * x);
            const T yd = y * radial + T(p1) * (r2 + T(2.0) * y * y) + T(2.0 * p2) * x * y;
            u = T(fx) * xd + T(cx);
            v = T(fy) * yd + T(cy);
        }

        // Reprojection residual for one correspondence: predicted (u, v)
        // minus the picked pixel, exactly like the Pinhole solver's reused
        // observation_equation_perspective_camera_tait_bryan_wc, just
        // autodiff'd instead of symbolically pre-differentiated (no
        // vendored Mei Jacobian exists to reuse -- see CameraCalibrationSolver.h).
        struct MeiReprojectionResidual
        {
            MeiReprojectionResidual(const Eigen::Vector3d& p, double u_kp, double v_kp, const Intrinsics& K)
                : p_(p), u_kp_(u_kp), v_kp_(v_kp), K_(K)
            {
            }

            template <typename T>
            bool operator()(const T* const tx_ty_tz, const T* const om_fi_ka, T* residual) const
            {
                T R[3][3];
                rotationMatrix(om_fi_ka[0], om_fi_ka[1], om_fi_ka[2], R);

                const T d[3] = { T(p_.x()) - tx_ty_tz[0], T(p_.y()) - tx_ty_tz[1], T(p_.z()) - tx_ty_tz[2] };
                // p_cam = R_wc^T * (p_world - C)
                const T pc[3] = {
                    R[0][0] * d[0] + R[1][0] * d[1] + R[2][0] * d[2],
                    R[0][1] * d[0] + R[1][1] * d[1] + R[2][1] * d[2],
                    R[0][2] * d[0] + R[1][2] * d[1] + R[2][2] * d[2],
                };

                T u, v;
                projectMei(pc, K_.fx, K_.fy, K_.cx, K_.cy, K_.xi, K_.k1, K_.k2, K_.k3, K_.p1, K_.p2, u, v);
                residual[0] = u - T(u_kp_);
                residual[1] = v - T(v_kp_);
                return true;
            }

            const Eigen::Vector3d p_;
            const double u_kp_, v_kp_;
            const Intrinsics K_;
        };
    } // namespace

    bool solveExtrinsicsMeiCeres(
        const std::vector<PointPixelCorrespondence>& correspondences,
        const Intrinsics& K,
        Extrinsics& extrinsicsInOut,
        std::string& errorMessage,
        double* outRmsPixels,
        bool fixTranslation)
    {
        const int nParams = fixTranslation ? 3 : 6;
        if (static_cast<int>(correspondences.size()) < 3 || static_cast<int>(correspondences.size()) * 2 < nParams)
        {
            errorMessage = "Need at least 3 correspondences";
            return false;
        }

        const double d2r = M_PI / 180.0;
        double txyz[3] = { extrinsicsInOut.tx, extrinsicsInOut.ty, extrinsicsInOut.tz };
        double omfika[3] = { extrinsicsInOut.om * d2r, extrinsicsInOut.fi * d2r, extrinsicsInOut.ka * d2r };

        ceres::Problem problem;
        for (const auto& c : correspondences)
        {
            auto* cost =
                new ceres::AutoDiffCostFunction<MeiReprojectionResidual, 2, 3, 3>(new MeiReprojectionResidual(c.p, c.u, c.v, K));
            problem.AddResidualBlock(cost, nullptr, txyz, omfika);
        }
        if (fixTranslation)
            problem.SetParameterBlockConstant(txyz);

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

        // Ceres' final_cost is 0.5*sum(residual_i^2) over every SCALAR
        // residual (2 per correspondence: du, dv), so sum(du^2+dv^2) =
        // 2*final_cost -- matching the Pinhole solver's own rms formula
        // (sqrt(sum(du^2+dv^2) / (2*N))) then simplifies to sqrt(final_cost/N).
        if (outRmsPixels)
            *outRmsPixels = std::sqrt(summary.final_cost / static_cast<double>(correspondences.size()));

        return true;
    }
} // namespace calib

#else // !CALIB_ENABLE_CERES

namespace calib
{
    bool solveExtrinsicsMeiCeres(
        const std::vector<PointPixelCorrespondence>&,
        const Intrinsics&,
        Extrinsics&,
        std::string& errorMessage,
        double*,
        bool)
    {
        errorMessage = "Mei extrinsics solving needs calib_core built with -DCALIB_ENABLE_CERES=ON (see calib_core/CMakeLists.txt)";
        return false;
    }
} // namespace calib

#endif