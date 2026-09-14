#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <CalibCore/Camera.h>
#include <CalibCore/MeiCamera.h>

#include <cmath>

using namespace calib;

namespace
{
    constexpr int kW = 3840; // the 360 rig's equirect frame size
    constexpr int kH = 1920;

    Intrinsics equirect()
    {
        Intrinsics K;
        K.model = CameraModel::Equirectangular;
        K.width = kW;
        K.height = kH;
        return K;
    }

    // A representative Mei/unified-sphere fisheye, values in the shape
    // insta360_mei_v2 calibrations take (see MeiCamera.h) rather than a
    // real calibrated camera.
    Intrinsics mei()
    {
        Intrinsics K;
        K.model = CameraModel::Mei;
        K.fx = 300.f; K.fy = 300.f;
        K.cx = 320.f; K.cy = 240.f;
        K.xi = 1.2f;
        K.k1 = -0.15f; K.k2 = 0.02f; K.k3 = -0.001f;
        K.p1 = 0.001f; K.p2 = -0.0005f;
        K.width = 640; K.height = 480;
        return K;
    }

    // The same camera as mei(), built directly as a MeiCamera -- used to
    // check projectPoint()'s Mei branch against the type it wraps, not
    // against a re-derivation of the formula.
    MeiCamera meiCamera()
    {
        const Intrinsics K = mei();
        MeiCamera cam;
        cam.fx = K.fx; cam.fy = K.fy; cam.cx = K.cx; cam.cy = K.cy;
        cam.xi = K.xi;
        cam.k1 = K.k1; cam.k2 = K.k2; cam.k3 = K.k3;
        cam.p1 = K.p1; cam.p2 = K.p2;
        return cam;
    }

    // Identity pose: p_cam == p_lidar, so test points can be written directly
    // in camera axes (X = right, Y = down, Z = forward).
    const Eigen::Matrix3f kIdentity = Eigen::Matrix3f::Identity();
    const Eigen::Vector3f kOrigin = Eigen::Vector3f::Zero();

    // Convenience wrapper: projects and returns the pixel, CHECKing success.
    struct Px
    {
        float u, v, depth;
    };

    Px project(const Intrinsics& K, const Eigen::Vector3f& p, const Eigen::Matrix3f& R_wc = kIdentity, const Eigen::Vector3f& t = kOrigin)
    {
        Px r{ 0, 0, 0 };
        REQUIRE(projectPoint(p.x(), p.y(), p.z(), K, R_wc, t, r.u, r.v, r.depth));
        return r;
    }
} // namespace

// ── Equirectangular ───────────────────────────────────────────────────────────

TEST_CASE("equirectangular: cardinal bearings land on the expected pixels")
{
    const Intrinsics K = equirect();

    SUBCASE("forward is the image centre")
    {
        Px r = project(K, { 0, 0, 10 });
        CHECK(r.u == doctest::Approx(kW * 0.5));
        CHECK(r.v == doctest::Approx(kH * 0.5));
        CHECK(r.depth == doctest::Approx(10.0));
    }
    SUBCASE("right is three quarters across")
    {
        Px r = project(K, { 5, 0, 0 });
        CHECK(r.u == doctest::Approx(kW * 0.75));
        CHECK(r.v == doctest::Approx(kH * 0.5));
    }
    SUBCASE("left is one quarter across")
    {
        Px r = project(K, { -5, 0, 0 });
        CHECK(r.u == doctest::Approx(kW * 0.25));
        CHECK(r.v == doctest::Approx(kH * 0.5));
    }
    SUBCASE("straight down is the bottom edge, inclusive")
    {
        Px r = project(K, { 0, 3, 0 });
        CHECK(r.v == doctest::Approx(kH)); // documented inclusive upper bound
    }
    SUBCASE("straight up is the top edge")
    {
        Px r = project(K, { 0, -3, 0 });
        CHECK(r.v == doctest::Approx(0.0));
    }
}

TEST_CASE("equirectangular: depth is range, not z")
{
    const Intrinsics K = equirect();
    Px r = project(K, { 3, 0, 4 });
    CHECK(r.depth == doctest::Approx(5.0)); // a pinhole camera would report 4
}

TEST_CASE("equirectangular: points behind the camera still project")
{
    const Intrinsics K = equirect();

    // Directly behind: atan2(0, -1) == +pi maps to u == width, which wraps to 0.
    Px back = project(K, { 0, 0, -10 });
    CHECK(back.u == doctest::Approx(0.0));
    CHECK(back.v == doctest::Approx(kH * 0.5));

    // The same point is rejected outright by the pinhole model.
    Intrinsics P; // defaults to Pinhole
    float u, v, depth;
    CHECK_FALSE(projectPoint(0, 0, -10, P, kIdentity, kOrigin, u, v, depth));
}

TEST_CASE("equirectangular: u stays inside [0, width) either side of the seam")
{
    const Intrinsics K = equirect();

    // Just past the seam on each side -- the wrap must not push u to width.
    for (float dx : { -1e-3f, 1e-3f })
    {
        Px r = project(K, { dx, 0, -10 });
        CHECK(r.u >= 0.f);
        CHECK(r.u < static_cast<float>(kW));
    }
}

TEST_CASE("equirectangular: poles produce no NaN")
{
    const Intrinsics K = equirect();

    // asin's argument is y/|p|, which rounds to slightly outside [-1, 1] for a
    // point exactly on the axis unless it is clamped.
    for (float sign : { -1.f, 1.f })
    {
        Px r = project(K, { 0, sign * 7.f, 0 });
        CHECK_FALSE(std::isnan(r.u));
        CHECK_FALSE(std::isnan(r.v));
    }
}

TEST_CASE("equirectangular: bearing -> pixel -> bearing round trip")
{
    const Intrinsics K = equirect();
    const float pi = static_cast<float>(M_PI);

    const Eigen::Vector3f bearings[] = {
        Eigen::Vector3f(0.3f, -0.2f, 0.9f).normalized(),
        Eigen::Vector3f(-0.7f, 0.5f, -0.4f).normalized(),
        Eigen::Vector3f(0.1f, 0.95f, 0.05f).normalized(),
        Eigen::Vector3f(-0.6f, -0.1f, -0.8f).normalized(),
    };

    for (const auto& b : bearings)
    {
        Px r = project(K, b * 12.f);

        const float az = (r.u / kW - 0.5f) * 2.f * pi;
        const float el = (r.v / kH - 0.5f) * pi;
        Eigen::Vector3f back(std::cos(el) * std::sin(az), std::sin(el), std::cos(el) * std::cos(az));

        CHECK(back.x() == doctest::Approx(b.x()).epsilon(1e-4));
        CHECK(back.y() == doctest::Approx(b.y()).epsilon(1e-4));
        CHECK(back.z() == doctest::Approx(b.z()).epsilon(1e-4));
    }
}

TEST_CASE("equirectangular: respects the extrinsics")
{
    const Intrinsics K = equirect();

    // om=fi=ka=0 is the nominal camera-vs-LiDAR alignment, so LiDAR forward
    // (+X) should come out as camera forward, i.e. the image centre.
    const Eigen::Matrix3f R_wc = kCameraLidarAxisOffset;

    SUBCASE("LiDAR forward is the image centre")
    {
        Px r = project(K, { 10, 0, 0 }, R_wc);
        CHECK(r.u == doctest::Approx(kW * 0.5));
        CHECK(r.v == doctest::Approx(kH * 0.5));
    }
    SUBCASE("LiDAR left is one quarter across")
    {
        Px r = project(K, { 0, 10, 0 }, R_wc);
        CHECK(r.u == doctest::Approx(kW * 0.25));
    }
    SUBCASE("LiDAR up is the top edge")
    {
        Px r = project(K, { 0, 0, 10 }, R_wc);
        CHECK(r.v == doctest::Approx(0.0));
    }
    SUBCASE("the camera position is subtracted")
    {
        // Point at the camera itself: too close to give a bearing.
        const Eigen::Vector3f C(1.f, 2.f, 3.f);
        float u, v, depth;
        CHECK_FALSE(projectPoint(C.x(), C.y(), C.z(), K, R_wc, C, u, v, depth));

        // One metre in front of the camera, not of the origin.
        Px r = project(K, C + Eigen::Vector3f(1.f, 0.f, 0.f), R_wc, C);
        CHECK(r.depth == doctest::Approx(1.0));
        CHECK(r.u == doctest::Approx(kW * 0.5));
    }
}

// ── Mei ─────────────────────────────────────────────────────────────────────

TEST_CASE("mei: forward is the image centre, depth is range")
{
    const Intrinsics K = mei();

    Px r = project(K, { 0, 0, 10 });
    CHECK(r.u == doctest::Approx(K.cx));
    CHECK(r.v == doctest::Approx(K.cy));
    CHECK(r.depth == doctest::Approx(10.0)); // range, not z -- see below

    Px oblique = project(K, { 3, 0, 4 });
    CHECK(oblique.depth == doctest::Approx(5.0)); // a pinhole camera would report 4
}

TEST_CASE("mei: projectPoint wraps MeiCamera::Project rather than re-deriving it")
{
    const Intrinsics K = mei();
    const MeiCamera cam = meiCamera();

    const Eigen::Vector3f points[] = {
        { 0.3f, -0.2f, 0.9f },
        { -1.5f, 0.8f, 2.0f },
        { 0.05f, 0.02f, 1.0f },
        { -0.6f, -1.1f, 0.8f },
    };

    for (const auto& p : points)
    {
        Px r = project(K, p);
        const cv::Point2d expected = cam.Project(cv::Point3d(p.x(), p.y(), p.z()));
        CHECK(r.u == doctest::Approx(expected.x));
        CHECK(r.v == doctest::Approx(expected.y));
    }
}

TEST_CASE("mei: a point on the camera itself is rejected")
{
    const Intrinsics K = mei();
    float u, v, depth;
    CHECK_FALSE(projectPoint(0, 0, 0, K, kIdentity, kOrigin, u, v, depth));
}

TEST_CASE("mei: a point behind the camera is rejected, not silently mis-projected")
{
    // MeiCamera::Project has no domain guard of its own, and past the valid
    // dome the projection is not injective -- it folds far-off-axis
    // directions back onto real pixels instead of pushing them out of frame.
    float u, v, depth;

    // Direction at `deg` from the optical axis, in the plane y = 0. The
    // explicit return type matters: `auto` would deduce an Eigen expression
    // template holding a reference to the temporary, and dangle.
    auto at = [](float deg) -> Eigen::Vector3f
    {
        const float r = deg * float(M_PI) / 180.f;
        return Eigen::Vector3f(std::sin(r), 0.f, std::cos(r)) * 10.f;
    };
    auto projects = [&](const Intrinsics& K, const Eigen::Vector3f& p)
    { return projectPoint(p.x(), p.y(), p.z(), K, kIdentity, kOrigin, u, v, depth); };

    SUBCASE("xi > 1: the limit is the fold-back angle, acos(-1/xi)")
    {
        const Intrinsics K = mei(); // xi = 1.2 -> 146.44 deg
        CHECK(projects(K, at(0.f)));
        CHECK(projects(K, at(145.f)));
        CHECK_FALSE(projects(K, at(148.f)));
        // Straight behind used to land on (cx, cy) -- the whole point of the guard.
        CHECK_FALSE(projects(K, at(180.f)));
    }

    SUBCASE("xi <= 1: the limit is where the denominator blows up, acos(-xi)")
    {
        Intrinsics K = mei();
        K.xi = 0.5f; // -> 120 deg
        CHECK(projects(K, at(0.f)));
        CHECK(projects(K, at(119.f)));
        CHECK_FALSE(projects(K, at(121.f)));
        CHECK_FALSE(projects(K, at(180.f)));
    }

    SUBCASE("xi = 0 reduces to the pinhole half-space")
    {
        Intrinsics K = mei();
        K.xi = 0.f;
        CHECK(projects(K, at(89.f)));
        CHECK_FALSE(projects(K, at(91.f)));
    }
}

TEST_CASE("mei: respects the extrinsics")
{
    const Intrinsics K = mei();
    const MeiCamera cam = meiCamera();

    // om=fi=ka=0 is the nominal camera-vs-LiDAR alignment, so LiDAR forward
    // (+X) should come out as camera forward, i.e. the image centre.
    const Eigen::Matrix3f R_wc = kCameraLidarAxisOffset;

    Px r = project(K, { 10, 0, 0 }, R_wc);
    CHECK(r.u == doctest::Approx(K.cx));
    CHECK(r.v == doctest::Approx(K.cy));

    // The camera position is subtracted: one metre in front of an offset
    // camera reprojects the same as one metre in front of the origin.
    const Eigen::Vector3f C(1.f, 2.f, 3.f);
    Px offset = project(K, C + Eigen::Vector3f(1.f, 0.f, 0.f), R_wc, C);
    // p_lidar - C = LiDAR +X, which R_wc's transpose turns into camera +Z
    // (camera-forward) -- same axis remap as the centre check above.
    const cv::Point2d expected = cam.Project(cv::Point3d(0.0, 0.0, 1.0));
    CHECK(offset.u == doctest::Approx(expected.x));
    CHECK(offset.v == doctest::Approx(expected.y));
    CHECK(offset.depth == doctest::Approx(1.0));
}

// ── Pinhole (regression: this path must not change) ───────────────────────────

TEST_CASE("pinhole is the default model")
{
    CHECK(Intrinsics{}.model == CameraModel::Pinhole);
}

TEST_CASE("pinhole: projection matches hand-computed values")
{
    Intrinsics K; // fx = fy = 800, cx = 640, cy = 360

    SUBCASE("undistorted")
    {
        Px r = project(K, { 1, 2, 4 });
        CHECK(r.u == doctest::Approx(840.0));
        CHECK(r.v == doctest::Approx(760.0));
        CHECK(r.depth == doctest::Approx(4.0));
    }
    SUBCASE("radial numerator")
    {
        K.k1 = 0.1f;
        Px r = project(K, { 1, 2, 4 });
        CHECK(r.u == doctest::Approx(846.25));
        CHECK(r.v == doctest::Approx(772.5));
    }
    SUBCASE("rational denominator")
    {
        K.k1 = 0.1f;
        K.k4 = 0.2f;
        Px r = project(K, { 1, 2, 4 });
        CHECK(r.u == doctest::Approx(834.117647));
        CHECK(r.v == doctest::Approx(748.235294));
    }
    SUBCASE("tangential")
    {
        K.p1 = 0.01f;
        K.p2 = 0.02f;
        Px r = project(K, { 1, 2, 4 });
        CHECK(r.u == doctest::Approx(849.0));
        CHECK(r.v == doctest::Approx(770.5));
    }
}

TEST_CASE("pinhole: rejects points at or behind the camera plane")
{
    Intrinsics K;
    float u, v, depth;
    CHECK_FALSE(projectPoint(1, 2, -4, K, kIdentity, kOrigin, u, v, depth));
    CHECK_FALSE(projectPoint(1, 2, 0, K, kIdentity, kOrigin, u, v, depth));
}

// ── scaleRoi ──────────────────────────────────────────────────────────────────

TEST_CASE("scaleRoi: a half-size image halves the rectangle")
{
    Roi r{ true, 100, 200, 40, 60 };
    Roi h = scaleRoi(r, 0.5f);
    CHECK(h.enabled);
    CHECK(h.x == 50);
    CHECK(h.y == 100);
    CHECK(h.w == 20);
    CHECK(h.h == 30);
}

TEST_CASE("scaleRoi: abutting rectangles stay abutting")
{
    // Scaling the width on its own would give both of these w == 2 and make
    // them overlap at x == 2; rounding the two edges and subtracting cannot.
    Roi a{ true, 1, 1, 3, 3 };
    Roi b{ true, 4, 4, 3, 3 };
    Roi as = scaleRoi(a, 0.5f);
    Roi bs = scaleRoi(b, 0.5f);
    CHECK(as.x + as.w == bs.x);
    CHECK(as.y + as.h == bs.y);
}

TEST_CASE("scaleRoi: a non-empty rectangle never scales down to empty")
{
    // w/h == 0 reads as "no ROI set", i.e. accept everything -- the exact
    // opposite of what a ROI this small is asking for.
    Roi tiny{ true, 10, 10, 2, 2 };
    Roi s = scaleRoi(tiny, 0.1f);
    CHECK(s.w >= 1);
    CHECK(s.h >= 1);
}

TEST_CASE("scaleRoi: an unset rectangle is left alone")
{
    Roi none;
    Roi s = scaleRoi(none, 0.5f);
    CHECK_FALSE(s.enabled);
    CHECK(s.w == 0);
    CHECK(s.h == 0);
}

// ── scaleIntrinsics ───────────────────────────────────────────────────────────

TEST_CASE("scaleIntrinsics: a half-size image projects to half the pixel")
{
    SUBCASE("pinhole")
    {
        Intrinsics K;
        Intrinsics H = scaleIntrinsics(K, 0.5f);
        CHECK(H.model == CameraModel::Pinhole);
        CHECK(H.fx == doctest::Approx(400.0));
        CHECK(H.cx == doctest::Approx(320.0));

        Px full = project(K, { 1, 2, 4 });
        Px half = project(H, { 1, 2, 4 });
        CHECK(half.u == doctest::Approx(full.u * 0.5));
        CHECK(half.v == doctest::Approx(full.v * 0.5));
    }
    SUBCASE("equirectangular")
    {
        Intrinsics K = equirect();
        Intrinsics H = scaleIntrinsics(K, 0.5f);
        CHECK(H.model == CameraModel::Equirectangular);
        CHECK(H.width == kW / 2);
        CHECK(H.height == kH / 2);

        Px full = project(K, { 3, -1, 4 });
        Px half = project(H, { 3, -1, 4 });
        CHECK(half.u == doctest::Approx(full.u * 0.5));
        CHECK(half.v == doctest::Approx(full.v * 0.5));
    }
    SUBCASE("distortion and model are carried over unchanged")
    {
        Intrinsics K;
        K.k1 = 0.1f;
        K.p2 = 0.02f;
        Intrinsics H = scaleIntrinsics(K, 0.25f);
        CHECK(H.k1 == doctest::Approx(0.1));
        CHECK(H.p2 == doctest::Approx(0.02));
    }
    SUBCASE("mei")
    {
        Intrinsics K = mei();
        Intrinsics H = scaleIntrinsics(K, 0.5f);
        CHECK(H.model == CameraModel::Mei);
        CHECK(H.fx == doctest::Approx(K.fx * 0.5));
        CHECK(H.cx == doctest::Approx(K.cx * 0.5));
        // xi and the k*/p* polynomial are dimensionless, carried over as-is.
        CHECK(H.xi == doctest::Approx(K.xi));
        CHECK(H.k1 == doctest::Approx(K.k1));
        CHECK(H.p2 == doctest::Approx(K.p2));

        Px full = project(K, { 0.3f, -0.2f, 0.9f });
        Px half = project(H, { 0.3f, -0.2f, 0.9f });
        CHECK(half.u == doctest::Approx(full.u * 0.5));
        CHECK(half.v == doctest::Approx(full.v * 0.5));
    }
}