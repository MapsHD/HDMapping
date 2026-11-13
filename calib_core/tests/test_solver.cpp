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

    Intrinsics pinholeIntrinsics()
    {
        Intrinsics K; // Pinhole, no distortion
        K.fx = 500.f; K.fy = 500.f;
        K.cx = 320.f; K.cy = 240.f;
        return K;
    }

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
    // roughly in front of groundTruthExtrinsics()'s camera. The last one is
    // ~100 deg off-axis, behind a pinhole camera.
    const Eigen::Vector3f kLidarPoints[] = {
        { 3.f, 0.f, 0.f },   { 4.f, 1.5f, 0.5f },  { 5.f, -1.f, -0.5f }, { 3.5f, 0.8f, -0.8f },
        { 6.f, -1.8f, 1.f }, { 4.5f, 0.3f, 1.2f }, { 3.f, -0.6f, 0.4f }, { 1.f, 3.f, 0.5f },
    };

    //! @param dropUnprojectable skip points K cannot see (the off-axis one,
    //!        for Pinhole) instead of failing the test on them
    std::vector<PointPixelCorrespondence> syntheticCorrespondences(
        const Intrinsics& K, const Extrinsics& truth, bool dropUnprojectable = false)
    {
        const Eigen::Matrix3f R_wc = omFiKaToMat3(truth.om, truth.fi, truth.ka);
        const Eigen::Vector3f C(truth.tx, truth.ty, truth.tz);

        std::vector<PointPixelCorrespondence> corr;
        for (const auto& p : kLidarPoints)
        {
            float u, v, depth;
            const bool projected = projectPoint(p.x(), p.y(), p.z(), K, R_wc, C, u, v, depth);
            if (!projected && dropUnprojectable)
                continue;
            REQUIRE(projected);
            PointPixelCorrespondence c;
            c.p = p.cast<double>();
            c.u = u;
            c.v = v;
            corr.push_back(c);
        }
        return corr;
    }

    //! Truth with every one of the six parameters nudged off it.
    Extrinsics perturbed(const Extrinsics& truth)
    {
        Extrinsics E = truth;
        E.tx += 0.3f; E.ty -= 0.2f; E.tz += 0.15f;
        E.om += 2.f; E.fi -= 1.5f; E.ka += 1.f;
        return E;
    }
} // namespace

TEST_CASE("solveExtrinsicsFromCorrespondences: locks keep their parameters and solve the rest")
{
    const Intrinsics K = pinholeIntrinsics();
    const Extrinsics truth = groundTruthExtrinsics();
    const auto corr = syntheticCorrespondences(K, truth, /*dropUnprojectable=*/true);
    REQUIRE(corr.size() >= 6);

    SUBCASE("no lock recovers all six")
    {
        Extrinsics E = perturbed(truth);
        double rms = -1.0;
        REQUIRE(solveExtrinsicsFromCorrespondences(corr, K, E, &rms));
        CHECK(rms < 0.5);
        CHECK(E.tx == doctest::Approx(truth.tx).epsilon(1e-3));
        CHECK(E.ty == doctest::Approx(truth.ty).epsilon(1e-3));
        CHECK(E.tz == doctest::Approx(truth.tz).epsilon(1e-3));
        CHECK(E.om == doctest::Approx(truth.om).epsilon(1e-2));
        CHECK(E.fi == doctest::Approx(truth.fi).epsilon(1e-2));
        CHECK(E.ka == doctest::Approx(truth.ka).epsilon(1e-2));
    }
    SUBCASE("fixTranslation")
    {
        Extrinsics E = truth;
        E.om += 3.f; E.fi -= 2.f; E.ka += 1.5f;
        REQUIRE(solveExtrinsicsFromCorrespondences(corr, K, E, nullptr, /*fixTranslation=*/true));
        CHECK(E.tx == doctest::Approx(truth.tx));
        CHECK(E.ty == doctest::Approx(truth.ty));
        CHECK(E.tz == doctest::Approx(truth.tz));
        CHECK(E.om == doctest::Approx(truth.om).epsilon(1e-2));
        CHECK(E.fi == doctest::Approx(truth.fi).epsilon(1e-2));
        CHECK(E.ka == doctest::Approx(truth.ka).epsilon(1e-2));
    }
    SUBCASE("fixRotation")
    {
        Extrinsics E = truth;
        E.tx += 0.3f; E.ty -= 0.2f; E.tz += 0.15f;
        REQUIRE(solveExtrinsicsFromCorrespondences(corr, K, E, nullptr, false, /*fixRotation=*/true));
        CHECK(E.om == doctest::Approx(truth.om));
        CHECK(E.fi == doctest::Approx(truth.fi));
        CHECK(E.ka == doctest::Approx(truth.ka));
        CHECK(E.tx == doctest::Approx(truth.tx).epsilon(1e-3));
        CHECK(E.ty == doctest::Approx(truth.ty).epsilon(1e-3));
        CHECK(E.tz == doctest::Approx(truth.tz).epsilon(1e-3));
    }
    SUBCASE("a wrong locked value is held, not pulled to the truth")
    {
        for (const bool lockRotation : { false, true })
        {
            CAPTURE(lockRotation);
            Extrinsics E = perturbed(truth);
            const Extrinsics locked = E;
            REQUIRE(solveExtrinsicsFromCorrespondences(corr, K, E, nullptr, !lockRotation, lockRotation));
            if (lockRotation)
            {
                CHECK(E.om == doctest::Approx(locked.om));
                CHECK(E.fi == doctest::Approx(locked.fi));
                CHECK(E.ka == doctest::Approx(locked.ka));
            }
            else
            {
                CHECK(E.tx == doctest::Approx(locked.tx));
                CHECK(E.ty == doctest::Approx(locked.ty));
                CHECK(E.tz == doctest::Approx(locked.tz));
            }
        }
    }
    SUBCASE("both locked fails and leaves the extrinsics alone")
    {
        Extrinsics E = perturbed(truth);
        const Extrinsics before = E;
        CHECK_FALSE(solveExtrinsicsFromCorrespondences(corr, K, E, nullptr, true, true));
        CHECK(E.tx == before.tx);
        CHECK(E.om == before.om);
    }
}

#ifndef CALIB_ENABLE_CERES

TEST_CASE("solveExtrinsicsCeres: stub explains the build flag when Ceres is disabled")
{
    std::vector<PointPixelCorrespondence> corr(3); // content doesn't matter -- fails before using it
    Extrinsics E;
    std::string err;
    CHECK_FALSE(solveExtrinsicsCeres(corr, meiIntrinsics(), E, err));
    CHECK(err.find("CALIB_ENABLE_CERES") != std::string::npos);
}

#else // CALIB_ENABLE_CERES

namespace
{
    // Coefficients in the range a real ~180 deg OpenCV fisheye calibration
    // produces, not a specific camera.
    Intrinsics fisheyeIntrinsics()
    {
        Intrinsics K;
        K.model = CameraModel::Fisheye;
        K.fx = 285.f; K.fy = 286.f;
        K.cx = 322.f; K.cy = 238.f;
        K.k1 = -0.0075f; K.k2 = 0.0435f; K.k3 = -0.0414f; K.k4 = 0.0077f;
        return K;
    }
} // namespace

TEST_CASE("solveExtrinsicsCeres: recovers known extrinsics from synthetic correspondences")
{
    for (const Intrinsics& K : { meiIntrinsics(), fisheyeIntrinsics() })
    {
        CAPTURE(modelToString(K.model));
        const Extrinsics truth = groundTruthExtrinsics();
        const auto corr = syntheticCorrespondences(K, truth);

        // Perturbed initial guess -- a solver that just echoed its input back
        // unchanged (e.g. Ceres silently failing to run and IsSolutionUsable()
        // being CHECK_FALSE'd elsewhere) would not pass this.
        Extrinsics guess = truth;
        guess.tx += 0.3f; guess.ty -= 0.2f; guess.tz += 0.15f;
        guess.om += 2.f; guess.fi -= 1.5f; guess.ka += 1.f;

        double rms = -1.0;
        std::string err;
        REQUIRE(solveExtrinsicsCeres(corr, K, guess, err, &rms, false));

        CHECK(rms < 0.5); // px -- points are noise-free, should fit almost exactly
        CHECK(guess.tx == doctest::Approx(truth.tx).epsilon(1e-3));
        CHECK(guess.ty == doctest::Approx(truth.ty).epsilon(1e-3));
        CHECK(guess.tz == doctest::Approx(truth.tz).epsilon(1e-3));
        CHECK(guess.om == doctest::Approx(truth.om).epsilon(1e-2));
        CHECK(guess.fi == doctest::Approx(truth.fi).epsilon(1e-2));
        CHECK(guess.ka == doctest::Approx(truth.ka).epsilon(1e-2));
    }
}

TEST_CASE("solveExtrinsicsCeres: fixTranslation leaves tx/ty/tz untouched")
{
    for (const Intrinsics& K : { meiIntrinsics(), fisheyeIntrinsics() })
    {
        CAPTURE(modelToString(K.model));
        const Extrinsics truth = groundTruthExtrinsics();
        const auto corr = syntheticCorrespondences(K, truth);

        Extrinsics guess = truth;
        guess.om += 3.f; guess.fi -= 2.f; guess.ka += 1.5f;
        const float lockedTx = guess.tx, lockedTy = guess.ty, lockedTz = guess.tz;

        double rms = -1.0;
        std::string err;
        REQUIRE(solveExtrinsicsCeres(corr, K, guess, err, &rms, /*fixTranslation=*/true));

        CHECK(guess.tx == doctest::Approx(lockedTx));
        CHECK(guess.ty == doctest::Approx(lockedTy));
        CHECK(guess.tz == doctest::Approx(lockedTz));
        CHECK(guess.om == doctest::Approx(truth.om).epsilon(1e-2));
        CHECK(guess.fi == doctest::Approx(truth.fi).epsilon(1e-2));
        CHECK(guess.ka == doctest::Approx(truth.ka).epsilon(1e-2));
    }
}

TEST_CASE("solveExtrinsicsCeres: fixRotation leaves om/fi/ka untouched")
{
    for (const Intrinsics& K : { meiIntrinsics(), fisheyeIntrinsics() })
    {
        CAPTURE(modelToString(K.model));
        const Extrinsics truth = groundTruthExtrinsics();
        const auto corr = syntheticCorrespondences(K, truth);

        Extrinsics guess = truth;
        guess.tx += 0.3f; guess.ty -= 0.2f; guess.tz += 0.15f;

        double rms = -1.0;
        std::string err;
        REQUIRE(solveExtrinsicsCeres(corr, K, guess, err, &rms, false, /*fixRotation=*/true));

        CHECK(guess.om == doctest::Approx(truth.om));
        CHECK(guess.fi == doctest::Approx(truth.fi));
        CHECK(guess.ka == doctest::Approx(truth.ka));
        CHECK(guess.tx == doctest::Approx(truth.tx).epsilon(1e-3));
        CHECK(guess.ty == doctest::Approx(truth.ty).epsilon(1e-3));
        CHECK(guess.tz == doctest::Approx(truth.tz).epsilon(1e-3));

        // A wrong locked rotation is held, not pulled to the truth.
        Extrinsics wrong = perturbed(truth);
        const Extrinsics locked = wrong;
        REQUIRE(solveExtrinsicsCeres(corr, K, wrong, err, &rms, false, /*fixRotation=*/true));
        CHECK(wrong.om == doctest::Approx(locked.om));
        CHECK(wrong.fi == doctest::Approx(locked.fi));
        CHECK(wrong.ka == doctest::Approx(locked.ka));
    }
}

TEST_CASE("solveExtrinsicsCeres: both locks set fails and says why")
{
    const Intrinsics K = meiIntrinsics();
    const auto corr = syntheticCorrespondences(K, groundTruthExtrinsics());
    Extrinsics E = perturbed(groundTruthExtrinsics());
    const Extrinsics before = E;
    std::string err;
    CHECK_FALSE(solveExtrinsicsCeres(corr, K, E, err, nullptr, true, true));
    CHECK(err.find("locked") != std::string::npos);
    CHECK(E.tx == before.tx);
    CHECK(E.om == before.om);
}

TEST_CASE("solveExtrinsicsCeres: a model it has no residual for fails and says why")
{
    // Routing a pinhole camera here by mistake would otherwise solve it as Mei.
    Intrinsics K; // Pinhole
    std::vector<PointPixelCorrespondence> corr(3);
    Extrinsics E = groundTruthExtrinsics();
    std::string err;
    CHECK_FALSE(solveExtrinsicsCeres(corr, K, E, err));
    CHECK(err.find("pinhole") != std::string::npos);
    CHECK(E.tx == groundTruthExtrinsics().tx);
}

#endif // CALIB_ENABLE_CERES