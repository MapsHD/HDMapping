#include <CalibCore/CameraCalibrationSolver.h>

// Reuses (does not duplicate) the analytic camera observation equations
// vendored at 3rdparty/observation_equations -- a pure-pinhole (no lens
// distortion) perspective camera model parameterized as:
//   t_wc = (tx,ty,tz)            camera position in world
//   R_wc = Rx(om)*Ry(fi)*Rz(ka)  camera orientation in world (radians)
// This mirrors the #include-the-generated-header convention core/src/
// control_points.cpp already uses for its own (point-to-point) vendored
// jacobian, rather than copying the math into this file. Provides, in the
// global namespace (the vendored header does not namespace itself):
//   projection_perspective_camera_tait_bryan_wc(...)
//   observation_equation_perspective_camera_tait_bryan_wc(...)          -- delta = target - projected
//   observation_equation_perspective_camera_tait_bryan_wc_jacobian(...) -- d(delta)/d[tx,ty,tz,om,fi,ka,px,py,pz], 2x9
#include <python-scripts/camera-metrics/perspective_camera_tait_bryan_wc_jacobian.h>

#include <Eigen/Dense>
#include <cmath>

namespace calib
{
    namespace
    {
        double sumSquaredResiduals(
            const std::vector<PointPixelCorrespondence>& correspondences,
            double fx,
            double fy,
            double cx,
            double cy,
            double tx,
            double ty,
            double tz,
            double om,
            double fi,
            double ka)
        {
            double sumSq = 0.0;
            for (const auto& c : correspondences)
            {
                Eigen::Matrix<double, 2, 1> delta;
                observation_equation_perspective_camera_tait_bryan_wc(
                    delta, fx, fy, cx, cy, tx, ty, tz, om, fi, ka, c.p.x(), c.p.y(), c.p.z(), c.u, c.v);
                sumSq += delta.squaredNorm();
            }
            return sumSq;
        }
    } // namespace

    bool solveExtrinsicsFromCorrespondences(
        const std::vector<PointPixelCorrespondence>& correspondences,
        const Intrinsics& K,
        Extrinsics& extrinsicsInOut,
        double* outRmsPixels,
        bool fixTranslation)
    {
        // Free parameters are always a contiguous slice of the full
        // [tx,ty,tz,om,fi,ka] (columns 0-5 of the 2x9 jacobian): all 6 of
        // them normally, or just [om,fi,ka] (columns 3-5) when translation
        // is pinned -- tx/ty/tz then never appear in the normal equations
        // at all, so they cannot move.
        const int nParams = fixTranslation ? 3 : 6;
        const int colOffset = fixTranslation ? 3 : 0;
        if (static_cast<int>(correspondences.size()) < 3 || static_cast<int>(correspondences.size()) * 2 < nParams)
            return false;

        // calib::Extrinsics' om/fi/ka are a SMALL deviation from the fixed
        // kCameraLidarAxisOffset alignment (R_wc = kCameraLidarAxisOffset *
        // Rx(om)Ry(fi)Rz(ka) -- see Camera.h), but the vendored equations
        // below assume om/fi/ka parameterize the FULL rotation on their
        // own. Reconciling the two without touching the vendored equations
        // or the LM loop at all: since
        //   p_cam = R_wc^T (p_world - C)
        //         = Rx(om)Ry(fi)Rz(ka)^T * (kCameraLidarAxisOffset^T*p_world - kCameraLidarAxisOffset^T*C)
        // pre-rotating every point by kCameraLidarAxisOffset^T once up
        // front, and solving for C' = kCameraLidarAxisOffset^T*C instead of
        // the real camera position C, makes the vendored equations' own
        // (px,py,pz) and (tx,ty,tz) exactly this p_world' and C' -- so
        // om/fi/ka fed to/from them are directly Extrinsics' om/fi/ka,
        // unchanged, and the rest of this function is untouched.
        const Eigen::Matrix3d offsetT = kCameraLidarAxisOffset.transpose().cast<double>();
        std::vector<PointPixelCorrespondence> rotated;
        rotated.reserve(correspondences.size());
        for (const auto& c : correspondences)
        {
            PointPixelCorrespondence rc;
            rc.p = offsetT * c.p;
            rc.u = c.u;
            rc.v = c.v;
            rotated.push_back(rc);
        }

        const double d2r = M_PI / 180.0;
        Eigen::Vector3d Cprime = offsetT * Eigen::Vector3d(extrinsicsInOut.tx, extrinsicsInOut.ty, extrinsicsInOut.tz);
        double tx = Cprime.x(), ty = Cprime.y(), tz = Cprime.z();
        double om = extrinsicsInOut.om * d2r, fi = extrinsicsInOut.fi * d2r, ka = extrinsicsInOut.ka * d2r;

        const double fx = K.fx, fy = K.fy, cx = K.cx, cy = K.cy;

        double cost = sumSquaredResiduals(rotated, fx, fy, cx, cy, tx, ty, tz, om, fi, ka);
        bool solvedOnce = false;

        // Levenberg-Marquardt: plain (undamped) Gauss-Newton can take huge,
        // divergent steps when the initial guess is only moderately off
        // (verified empirically -- a ~1.6m/several-degree initial error was
        // enough to blow the plain-GN version up completely), because the
        // normal-equations system is only well-conditioned near the true
        // solution. Damping the diagonal and only accepting steps that
        // actually reduce the residual keeps convergence robust without
        // changing the underlying (reused, unmodified) observation
        // equations at all.
        double lambda = 1e-3;
        constexpr int kMaxIterations = 50;
        constexpr int kMaxLmTries = 16;

        for (int iter = 0; iter < kMaxIterations; ++iter)
        {
            Eigen::MatrixXd AtA = Eigen::MatrixXd::Zero(nParams, nParams);
            Eigen::VectorXd AtB = Eigen::VectorXd::Zero(nParams);

            for (const auto& c : rotated)
            {
                Eigen::Matrix<double, 2, 1> delta;
                observation_equation_perspective_camera_tait_bryan_wc(
                    delta, fx, fy, cx, cy, tx, ty, tz, om, fi, ka, c.p.x(), c.p.y(), c.p.z(), c.u, c.v);

                Eigen::Matrix<double, 2, 9> jFull;
                observation_equation_perspective_camera_tait_bryan_wc_jacobian(
                    jFull, fx, fy, cx, cy, tx, ty, tz, om, fi, ka, c.p.x(), c.p.y(), c.p.z());

                // Only the free-parameter columns -- the 3D points
                // (columns 6-8) are fixed observations, not solved for, and
                // translation (columns 0-2) is skipped entirely when
                // fixTranslation is set.
                Eigen::MatrixXd A = -jFull.block(0, colOffset, 2, nParams);

                AtA += A.transpose() * A;
                AtB += A.transpose() * delta;
            }

            bool improved = false;
            for (int lmTry = 0; lmTry < kMaxLmTries; ++lmTry)
            {
                Eigen::MatrixXd damped = AtA;
                for (int d = 0; d < nParams; ++d)
                    damped(d, d) += lambda * (AtA(d, d) > 0.0 ? AtA(d, d) : 1.0);

                Eigen::FullPivLU<Eigen::MatrixXd> lu(damped);
                if (!lu.isInvertible())
                {
                    lambda *= 10.0;
                    continue;
                }

                Eigen::VectorXd x = lu.solve(AtB);
                double tx2 = tx, ty2 = ty, tz2 = tz, om2 = om, fi2 = fi, ka2 = ka;
                if (fixTranslation)
                {
                    om2 += x(0);
                    fi2 += x(1);
                    ka2 += x(2);
                }
                else
                {
                    tx2 += x(0);
                    ty2 += x(1);
                    tz2 += x(2);
                    om2 += x(3);
                    fi2 += x(4);
                    ka2 += x(5);
                }
                double newCost = sumSquaredResiduals(rotated, fx, fy, cx, cy, tx2, ty2, tz2, om2, fi2, ka2);

                if (newCost < cost)
                {
                    tx = tx2;
                    ty = ty2;
                    tz = tz2;
                    om = om2;
                    fi = fi2;
                    ka = ka2;
                    cost = newCost;
                    lambda = std::max(lambda * 0.3, 1e-12);
                    solvedOnce = true;
                    improved = true;

                    if (x.norm() < 1e-9)
                        iter = kMaxIterations; // converged -- stop the outer loop too
                    break;
                }

                lambda *= 10.0;
            }

            if (!improved)
                break; // damping maxed out without an improving step -- stuck or converged
        }

        if (!solvedOnce)
            return false;

        // Un-rotate the solved C' back to the real camera position C =
        // kCameraLidarAxisOffset * C' -- see the comment above the
        // pre-rotation this reverses. om/fi/ka need no such conversion:
        // they're Extrinsics' own deviation-from-offset angles already.
        Eigen::Vector3d C = kCameraLidarAxisOffset.cast<double>() * Eigen::Vector3d(tx, ty, tz);
        extrinsicsInOut.tx = static_cast<float>(C.x());
        extrinsicsInOut.ty = static_cast<float>(C.y());
        extrinsicsInOut.tz = static_cast<float>(C.z());
        extrinsicsInOut.om = static_cast<float>(om / d2r);
        extrinsicsInOut.fi = static_cast<float>(fi / d2r);
        extrinsicsInOut.ka = static_cast<float>(ka / d2r);

        if (outRmsPixels)
            *outRmsPixels = std::sqrt(cost / (2.0 * static_cast<double>(correspondences.size())));

        return true;
    }

} // namespace calib
