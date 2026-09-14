// Solver tests, split out of test_camera.cpp (which owns doctest's
// DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN / main()) -- this file just registers
// more TEST_CASEs into the same executable/registry, same multi-TU doctest
// setup shared/tests uses.
#include <doctest.h>

#include <CalibCore/CameraCalibrationSolver.h>

#include <cmath>

using namespace calib;

namespace
{
    Intrinsics meiIntrinsics()
    {
        Intrinsics K;
        K.model = CameraModel::Mei;
        K.fx = 300.f; K.fy = 300.f;
        K.cx = 320.f; K.cy = 240.f;
        K.xi = 1.2f;
        K.k1 = -0.15f; K.k2 = 0.02f; K.k3 = -0.001f;
        K.p1 = 0.001f; K.p2 = -0.0005f;
        return K;
    }
} // namespace

#ifndef CALIB_ENABLE_CERES

TEST_CASE("solveExtrinsicsMeiCeres: stub explains the build flag when Ceres is disabled")
{
    std::vector<PointPixelCorrespondence> corr(3); // content doesn't matter -- fails before using it
    Extrinsics E;
    std::string err;
    CHECK_FALSE(solveExtrinsicsMeiCeres(corr, meiIntrinsics(), E, err));
    CHECK(err.find("CALIB_ENABLE_CERES") != std::string::npos);
}

#else // CALIB_ENABLE_CERES

namespace
{
    // Ground truth this test solves for, expressed the same way
    // AppState::saveCalibration/loadCalibration do: camera position + a
    // small om/fi/ka deviation from kCameraLidarAxisOffset.
    Extrinsics groundTruthExtrinsics()
    {
        Extrinsics E;
        E.tx = 1.5f; E.ty = -0.3f; E.tz = 0.8f;
        E.om = 4.f; E.fi = -6.f; E.ka = 2.f;
        return E;
    }

    // A handful of LiDAR-frame points spread across the field of view,
    // roughly in front of groundTruthExtrinsics()'s camera.
    const Eigen::Vector3f kLidarPoints[] = {
        { 3.f, 0.f, 0.f },   { 4.f, 1.5f, 0.5f },  { 5.f, -1.f, -0.5f }, { 3.5f, 0.8f, -0.8f },
        { 6.f, -1.8f, 1.f }, { 4.5f, 0.3f, 1.2f }, { 3.f, -0.6f, 0.4f },
    };
} // namespace

TEST_CASE("solveExtrinsicsMeiCeres: recovers known extrinsics from synthetic correspondences")
{
    const Intrinsics K = meiIntrinsics();
    const Extrinsics truth = groundTruthExtrinsics();
    const Eigen::Matrix3f R_wc = omFiKaToMat3(truth.om, truth.fi, truth.ka);
    const Eigen::Vector3f C(truth.tx, truth.ty, truth.tz);

    std::vector<PointPixelCorrespondence> corr;
    for (const auto& p : kLidarPoints)
    {
        float u, v, depth;
        REQUIRE(projectPoint(p.x(), p.y(), p.z(), K, R_wc, C, u, v, depth));
        PointPixelCorrespondence c;
        c.p = p.cast<double>();
        c.u = u;
        c.v = v;
        corr.push_back(c);
    }

    // Perturbed initial guess -- a solver that just echoed its input back
    // unchanged (e.g. Ceres silently failing to run and IsSolutionUsable()
    // being CHECK_FALSE'd elsewhere) would not pass this.
    Extrinsics guess = truth;
    guess.tx += 0.3f; guess.ty -= 0.2f; guess.tz += 0.15f;
    guess.om += 2.f; guess.fi -= 1.5f; guess.ka += 1.f;

    double rms = -1.0;
    std::string err;
    REQUIRE(solveExtrinsicsMeiCeres(corr, K, guess, err, &rms, false));

    CHECK(rms < 0.5); // px -- points are noise-free, should fit almost exactly
    CHECK(guess.tx == doctest::Approx(truth.tx).epsilon(1e-3));
    CHECK(guess.ty == doctest::Approx(truth.ty).epsilon(1e-3));
    CHECK(guess.tz == doctest::Approx(truth.tz).epsilon(1e-3));
    CHECK(guess.om == doctest::Approx(truth.om).epsilon(1e-2));
    CHECK(guess.fi == doctest::Approx(truth.fi).epsilon(1e-2));
    CHECK(guess.ka == doctest::Approx(truth.ka).epsilon(1e-2));
}

TEST_CASE("solveExtrinsicsMeiCeres: fixTranslation leaves tx/ty/tz untouched")
{
    const Intrinsics K = meiIntrinsics();
    const Extrinsics truth = groundTruthExtrinsics();
    const Eigen::Matrix3f R_wc = omFiKaToMat3(truth.om, truth.fi, truth.ka);
    const Eigen::Vector3f C(truth.tx, truth.ty, truth.tz);

    std::vector<PointPixelCorrespondence> corr;
    for (const auto& p : kLidarPoints)
    {
        float u, v, depth;
        REQUIRE(projectPoint(p.x(), p.y(), p.z(), K, R_wc, C, u, v, depth));
        PointPixelCorrespondence c;
        c.p = p.cast<double>();
        c.u = u;
        c.v = v;
        corr.push_back(c);
    }

    Extrinsics guess = truth;
    guess.om += 3.f; guess.fi -= 2.f; guess.ka += 1.5f;
    const float lockedTx = guess.tx, lockedTy = guess.ty, lockedTz = guess.tz;

    double rms = -1.0;
    std::string err;
    REQUIRE(solveExtrinsicsMeiCeres(corr, K, guess, err, &rms, /*fixTranslation=*/true));

    CHECK(guess.tx == doctest::Approx(lockedTx));
    CHECK(guess.ty == doctest::Approx(lockedTy));
    CHECK(guess.tz == doctest::Approx(lockedTz));
    CHECK(guess.om == doctest::Approx(truth.om).epsilon(1e-2));
    CHECK(guess.fi == doctest::Approx(truth.fi).epsilon(1e-2));
    CHECK(guess.ka == doctest::Approx(truth.ka).epsilon(1e-2));
}

#endif // CALIB_ENABLE_CERES