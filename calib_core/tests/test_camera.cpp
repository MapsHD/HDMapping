#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <CalibCore/Camera.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <fstream>
#include <string>

using namespace calib;

namespace
{
    // A representative Mei/unified-sphere fisheye, values in the shape
    // insta360_mei_v2 calibrations take rather than a real calibrated camera.
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

    // Coefficients in the range a real ~180 deg OpenCV fisheye calibration
    // produces, not a specific camera. Their theta -> theta_d polynomial is
    // increasing all the way to pi.
    Intrinsics fisheye()
    {
        Intrinsics K;
        K.model = CameraModel::Fisheye;
        K.fx = 285.f; K.fy = 286.f;
        K.cx = 322.f; K.cy = 238.f;
        K.k1 = -0.0075f; K.k2 = 0.0435f; K.k3 = -0.0414f; K.k4 = 0.0077f;
        K.width = 640; K.height = 480;
        return K;
    }

    // Direction at `deg` from the optical axis, in the plane y = 0. The
    // explicit return type matters: `auto` would deduce an Eigen expression
    // template holding a reference to the temporary, and dangle.
    Eigen::Vector3f offAxis(float deg)
    {
        const float r = deg * float(M_PI) / 180.f;
        return Eigen::Vector3f(std::sin(r), 0.f, std::cos(r)) * 10.f;
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

TEST_CASE("mei: unified-sphere projection matches known-good reference values")
{
    // Pins the unified-sphere + polynomial math against values captured from
    // the implementation, so a change to the formula has to be deliberate.
    // A Mei camera has no closed-form check as simple as the pinhole one, and
    // these were cross-checked against the rig's own reprojection.
    const Intrinsics K = mei();
    struct Ref
    {
        Eigen::Vector3f p;
        double u, v, depth;
    };
    const Ref refs[] = {
        { { 0.3f, -0.2f, 0.9f }, 363.3981018, 211.0740356, 0.9695359 },
        { { -1.5f, 0.8f, 2.0f }, 233.9576416, 285.9132385, 2.6248810 },
        { { 0.05f, 0.02f, 1.0f }, 326.8120728, 242.7250366, 1.0014490 },
        { { -0.6f, -1.1f, 0.8f }, 252.7274475, 116.8022079, 1.4866068 },
    };

    for (const auto& r : refs)
    {
        Px got = project(K, r.p);
        CHECK(got.u == doctest::Approx(r.u).epsilon(1e-6));
        CHECK(got.v == doctest::Approx(r.v).epsilon(1e-6));
        CHECK(got.depth == doctest::Approx(r.depth).epsilon(1e-6));
    }
}

TEST_CASE("mei: a point on the optical axis lands on the principal point")
{
    const Intrinsics K = mei();
    Px r = project(K, { 0.f, 0.f, 1.f });
    CHECK(r.u == doctest::Approx(K.cx));
    CHECK(r.v == doctest::Approx(K.cy));
    CHECK(r.depth == doctest::Approx(1.0));
}

TEST_CASE("mei: a point on the camera itself is rejected")
{
    const Intrinsics K = mei();
    float u, v, depth;
    CHECK_FALSE(projectPoint(0, 0, 0, K, kIdentity, kOrigin, u, v, depth));
}

TEST_CASE("mei: a point behind the camera is rejected, not silently mis-projected")
{
    // The projection has no domain guard of its own, and past the valid
    // dome the projection is not injective -- it folds far-off-axis
    // directions back onto real pixels instead of pushing them out of frame.
    float u, v, depth;

    const auto at = offAxis;
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
    // On-axis, so it lands on the principal point, as the centre check above.
    CHECK(offset.u == doctest::Approx(K.cx));
    CHECK(offset.v == doctest::Approx(K.cy));
    CHECK(offset.depth == doctest::Approx(1.0));
}

// ── Fisheye ───────────────────────────────────────────────────────────────────

TEST_CASE("fisheye: matches cv::fisheye::projectPoints")
{
    // Captured from OpenCV 4.6's cv::fisheye::projectPoints with an identity
    // pose and the same K/D -- the model this one has to agree with, so that
    // an OpenCV fisheye calibration can be loaded verbatim.
    const Intrinsics K = fisheye();
    struct Ref
    {
        Eigen::Vector3f p;
        double u, v;
    };
    const Ref refs[] = {
        { { 0.3f, -0.2f, 0.9f }, 412.3305153, 177.5683570 },
        { { -1.5f, 0.8f, 2.0f }, 144.4155107, 333.0440495 },
        { { 0.05f, 0.02f, 1.0f }, 336.2359451, 243.7143583 },
        { { -0.6f, -1.1f, 0.8f }, 184.8716725, -14.2840458 },
        { { 2.0f, 1.0f, 0.3f }, 668.3977375, 411.8065841 },
    };

    for (const auto& r : refs)
    {
        Px got = project(K, r.p);
        CHECK(got.u == doctest::Approx(r.u).epsilon(1e-5));
        CHECK(got.v == doctest::Approx(r.v).epsilon(1e-5));
        CHECK(got.depth == doctest::Approx(r.p.norm()));
    }
}

TEST_CASE("fisheye: a point on the optical axis lands on the principal point")
{
    const Intrinsics K = fisheye();
    Px r = project(K, { 0.f, 0.f, 4.f });
    CHECK(r.u == doctest::Approx(K.cx));
    CHECK(r.v == doctest::Approx(K.cy));
    CHECK(r.depth == doctest::Approx(4.0));
}

TEST_CASE("fisheye: without distortion the image radius is f * theta, past 90 deg too")
{
    Intrinsics K = fisheye();
    K.k1 = K.k2 = K.k3 = K.k4 = 0.f;
    const double pi = M_PI;

    Px side = project(K, offAxis(90.f));
    CHECK(side.u == doctest::Approx(K.cx + K.fx * pi / 2));
    CHECK(side.v == doctest::Approx(K.cy));

    // Behind the image plane, which a pinhole camera cannot see at all.
    Px behind = project(K, offAxis(120.f));
    CHECK(behind.u == doctest::Approx(K.cx + K.fx * 2 * pi / 3));
    CHECK(behind.depth == doctest::Approx(10.0)); // range, not z (which is negative)

    Intrinsics P; // Pinhole
    float u, v, depth;
    const Eigen::Vector3f p = offAxis(120.f);
    CHECK_FALSE(projectPoint(p.x(), p.y(), p.z(), P, kIdentity, kOrigin, u, v, depth));
}

TEST_CASE("fisheye: directions past the fold-back angle are rejected")
{
    float u, v, depth;
    auto projects = [&](const Intrinsics& K, const Eigen::Vector3f& p)
    { return projectPoint(p.x(), p.y(), p.z(), K, kIdentity, kOrigin, u, v, depth); };

    SUBCASE("a turning point limits the field of view")
    {
        // theta_d = theta - 0.3 theta^3 peaks at theta = sqrt(1/0.9) (60.4
        // deg) and falls back to 0 -- the principal point -- at 104.6 deg.
        Intrinsics K = fisheye();
        K.k1 = -0.3f;
        K.k2 = K.k3 = K.k4 = 0.f;
        CHECK(fisheyeMaxTheta(K) == doctest::Approx(std::sqrt(1.0 / 0.9)).epsilon(2e-3));
        CHECK(projects(K, offAxis(55.f)));
        CHECK_FALSE(projects(K, offAxis(65.f)));
        CHECK_FALSE(projects(K, offAxis(104.6f)));
    }

    SUBCASE("a monotonic polynomial keeps everything short of straight behind")
    {
        const Intrinsics K = fisheye();
        CHECK(fisheyeMaxTheta(K) == doctest::Approx(M_PI));
        CHECK(projects(K, offAxis(170.f)));
        CHECK_FALSE(projects(K, { 0.f, 0.f, -10.f }));
    }

    SUBCASE("a point on the camera itself")
    {
        CHECK_FALSE(projects(fisheye(), { 0.f, 0.f, 0.f }));
    }
}

TEST_CASE("fisheye: respects the extrinsics")
{
    const Intrinsics K = fisheye();
    const Eigen::Matrix3f R_wc = kCameraLidarAxisOffset;

    // LiDAR forward is camera forward at om=fi=ka=0.
    Px r = project(K, { 10, 0, 0 }, R_wc);
    CHECK(r.u == doctest::Approx(K.cx));
    CHECK(r.v == doctest::Approx(K.cy));

    const Eigen::Vector3f C(1.f, 2.f, 3.f);
    Px offset = project(K, C + Eigen::Vector3f(1.f, 0.f, 0.f), R_wc, C);
    CHECK(offset.u == doctest::Approx(K.cx));
    CHECK(offset.v == doctest::Approx(K.cy));
    CHECK(offset.depth == doctest::Approx(1.0));
}

// ── modelToString / modelFromString ───────────────────────────────────────────

TEST_CASE("modelFromString reads back every name modelToString writes")
{
    for (CameraModel m : { CameraModel::Pinhole, CameraModel::Mei, CameraModel::Fisheye })
    {
        CAPTURE(modelToString(m));
        CHECK(modelFromString(modelToString(m)) == m);
    }
    CHECK(modelFromString("equidistant") == CameraModel::Fisheye); // the ROS/Kalibr name
    CHECK(modelFromString("") == CameraModel::Pinhole);
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
        CHECK(H.width == K.width / 2);
        CHECK(H.height == K.height / 2);
        // xi and the k*/p* polynomial are dimensionless, carried over as-is.
        CHECK(H.xi == doctest::Approx(K.xi));
        CHECK(H.k1 == doctest::Approx(K.k1));
        CHECK(H.p2 == doctest::Approx(K.p2));

        Px full = project(K, { 0.3f, -0.2f, 0.9f });
        Px half = project(H, { 0.3f, -0.2f, 0.9f });
        CHECK(half.u == doctest::Approx(full.u * 0.5));
        CHECK(half.v == doctest::Approx(full.v * 0.5));
    }
    SUBCASE("fisheye")
    {
        Intrinsics K = fisheye();
        Intrinsics H = scaleIntrinsics(K, 0.5f);
        CHECK(H.model == CameraModel::Fisheye);
        CHECK(H.k4 == doctest::Approx(K.k4)); // theta coefficients are dimensionless

        Px full = project(K, { 2.0f, 1.0f, 0.3f });
        Px half = project(H, { 2.0f, 1.0f, 0.3f });
        CHECK(half.u == doctest::Approx(full.u * 0.5));
        CHECK(half.v == doctest::Approx(full.v * 0.5));
    }
}
// ── CameraIdentity::empty ─────────────────────────────────────────────────────

TEST_CASE("CameraIdentity::empty: a default-constructed identity is empty")
{
    CHECK(CameraIdentity{}.empty());
}

TEST_CASE("CameraIdentity::empty: a serial alone makes it non-empty")
{
    CameraIdentity id;
    id.serial = "SN-1";
    CHECK_FALSE(id.empty());
}

TEST_CASE("CameraIdentity::empty: a frame_id alone makes it non-empty")
{
    CameraIdentity id;
    id.frameId = "camera_front";
    CHECK_FALSE(id.empty());
}

TEST_CASE("CameraIdentity::empty: model/firmware alone do not count")
{
    // Only serial/frameId identify a physical camera; model and firmware are
    // descriptive metadata that can be present without either.
    CameraIdentity id;
    id.model = "Insta360 X4";
    id.firmware = "1.2.3";
    CHECK(id.empty());
}

// ── loadCameraInfoYaml ────────────────────────────────────────────────────────

namespace
{
    // Writes `body` to a temp file and loads it into K, so the parser is
    // exercised through its real file-reading path.
    bool loadIntoFromString(const std::string& body, Intrinsics& K)
    {
        const std::string path = (std::filesystem::temp_directory_path() / "calib_core_test_camera_info.yaml").string();
        {
            std::ofstream f(path);
            f << body;
        }
        const bool ok = loadCameraInfoYaml(path, K);
        std::filesystem::remove(path);
        return ok;
    }

    // As above, into a fresh Intrinsics. Returns nullopt when the load failed.
    std::optional<Intrinsics> loadFromString(const std::string& body)
    {
        Intrinsics K;
        return loadIntoFromString(body, K) ? std::optional<Intrinsics>(K) : std::nullopt;
    }

    // As above, for loadCameraIdentity. `id` is only meaningful when this
    // returns true.
    bool loadIdentityFromString(const std::string& body, CameraIdentity& id)
    {
        const std::string path = (std::filesystem::temp_directory_path() / "calib_core_test_camera_info.yaml").string();
        {
            std::ofstream f(path);
            f << body;
        }
        const bool ok = loadCameraIdentity(path, id);
        std::filesystem::remove(path);
        return ok;
    }

    const char* kSample = R"(# this rig's camera_info.yaml
frame_id: camera_front
distortion_model: insta360_mei_v2
width: 3840
height: 1920
fx: 620.5
fy: 621.25
cx: 959.5
cy: 539.5
xi: 1.234
distortion: [-0.0123, 0.0045, -0.0007, 0.0011, -0.0002]
)";

    // Verbatim from insta360-test-calib's SaveCamera, comment line included.
    const char* kFisheyeSample =
        R"(# calib_app: intrinsics re-estimated from equidistant, 5 views, 422 corners: rms 0.556 px (was 6.787), 40 iterations; started from a generic guess
width: 2880
height: 2880
distortion_model: equidistant
fx: 634.3157288
fy: 633.5613727
cx: 1454.934903
cy: 1429.728826
distortion: [0.1417399868, -0.01614857261, 0.01912948205, -0.006671667431]
k: [634.3157288, 0, 1454.934903, 0, 633.5613727, 1429.728826, 0, 0, 1]
r: [1, 0, 0, 0, 1, 0, 0, 0, 1]
p: [634.3157288, 0, 1454.934903, 0, 0, 633.5613727, 1429.728826, 0, 0, 0, 1, 0]
)";

    // The same file with `distortion_model` and `distortion` replaced.
    std::string withModel(const std::string& model, const std::string& distortion)
    {
        std::string body = kFisheyeSample;
        auto replaceLine = [&](const std::string& key, const std::string& value)
        {
            const auto at = body.find("\n" + key + ": ");
            REQUIRE(at != std::string::npos);
            const auto end = body.find('\n', at + 1);
            body.replace(at + 1, end - at - 1, key + ": " + value);
        };
        replaceLine("distortion_model", model);
        replaceLine("distortion", distortion);
        return body;
    }
} // namespace

TEST_CASE("loadCameraInfoYaml: reads this rig's flat camera_info.yaml")
{
    const auto K = loadFromString(kSample);
    REQUIRE(K.has_value());
    CHECK(K->model == CameraModel::Mei);
    CHECK(K->width == 3840);
    CHECK(K->height == 1920);
    CHECK(K->fx == doctest::Approx(620.5));
    CHECK(K->cy == doctest::Approx(539.5));
    CHECK(K->xi == doctest::Approx(1.234));
    // distortion is (k1, k2, k3, p1, p2) -- NOT OpenCV's pinhole order.
    CHECK(K->k1 == doctest::Approx(-0.0123));
    CHECK(K->k2 == doctest::Approx(0.0045));
    CHECK(K->k3 == doctest::Approx(-0.0007));
    CHECK(K->p1 == doctest::Approx(0.0011));
    CHECK(K->p2 == doctest::Approx(-0.0002));
    // The Mei polynomial has no rational denominator.
    CHECK(K->k4 == 0.f);
    CHECK(K->k5 == 0.f);
    CHECK(K->k6 == 0.f);
}

TEST_CASE("loadCameraInfoYaml: quotes and trailing comments are not taken literally")
{
    std::string body = kSample;
    body += "\nxi: 0.75  # trailing comment\n";
    const auto K = loadFromString(body);
    REQUIRE(K.has_value());
    CHECK(K->xi == doctest::Approx(0.75));
}

TEST_CASE("loadCameraInfoYaml: a missing field fails instead of defaulting to 0")
{
    // A calibration that silently reads xi as 0 reprojects wrongly with no
    // visible failure, so the load has to reject it outright.
    std::string body = kSample;
    const auto at = body.find("xi: 1.234\n");
    REQUIRE(at != std::string::npos);
    body.erase(at, std::string("xi: 1.234\n").size());

    CHECK_FALSE(loadFromString(body).has_value());
}

TEST_CASE("loadCameraInfoYaml: a missing file fails cleanly, and leaves K alone")
{
    Intrinsics K = mei();
    const Intrinsics before = K;
    CHECK_FALSE(loadCameraInfoYaml("/nonexistent/camera_info.yaml", K));
    CHECK(K.fx == before.fx);
    CHECK(K.xi == before.xi);
}

TEST_CASE("loadCameraInfoYaml: reads insta360-test-calib's equidistant file")
{
    const auto K = loadFromString(kFisheyeSample);
    REQUIRE(K.has_value());
    CHECK(K->model == CameraModel::Fisheye);
    CHECK(K->width == 2880);
    CHECK(K->height == 2880);
    CHECK(K->fx == doctest::Approx(634.3157288));
    CHECK(K->fy == doctest::Approx(633.5613727));
    CHECK(K->cx == doctest::Approx(1454.934903));
    CHECK(K->cy == doctest::Approx(1429.728826));
    CHECK(K->k1 == doctest::Approx(0.1417399868));
    CHECK(K->k2 == doctest::Approx(-0.01614857261));
    CHECK(K->k3 == doctest::Approx(0.01912948205));
    CHECK(K->k4 == doctest::Approx(-0.006671667431));
    CHECK(K->p1 == 0.f);
    CHECK(K->p2 == 0.f);
    CHECK(K->xi == 0.f);

    Px r = project(*K, { 0.f, 0.f, 3.f });
    CHECK(r.u == doctest::Approx(K->cx));
    CHECK(r.v == doctest::Approx(K->cy));
}

TEST_CASE("loadCameraInfoYaml: distortion_model fisheye is read as equidistant")
{
    const auto K = loadFromString(withModel("fisheye", "[0.1, 0.01, 0, 0]"));
    REQUIRE(K.has_value());
    CHECK(K->model == CameraModel::Fisheye);
    CHECK(K->k1 == doctest::Approx(0.1));
}

TEST_CASE("loadCameraInfoYaml: equidistant needs exactly four coefficients, and a failure leaves K alone")
{
    Intrinsics K = mei();
    const Intrinsics before = K;
    CHECK_FALSE(loadIntoFromString(withModel("equidistant", "[0.1, 0.01, 0, 0, 0.5]"), K));
    CHECK(K.model == before.model);
    CHECK(K.fx == before.fx);
    CHECK(K.k1 == before.k1);
}

TEST_CASE("loadCameraInfoYaml: the pinhole models take OpenCV's coefficient order")
{
    // Unlike Mei's (k1, k2, k3, p1, p2), p1/p2 come before k3.
    SUBCASE("plumb_bob")
    {
        const auto K = loadFromString(withModel("plumb_bob", "[0.1, -0.2, 0.001, 0.002, 0.05]"));
        REQUIRE(K.has_value());
        CHECK(K->model == CameraModel::Pinhole);
        CHECK(K->k1 == doctest::Approx(0.1));
        CHECK(K->k2 == doctest::Approx(-0.2));
        CHECK(K->p1 == doctest::Approx(0.001));
        CHECK(K->p2 == doctest::Approx(0.002));
        CHECK(K->k3 == doctest::Approx(0.05));
        CHECK(K->k4 == 0.f);
    }
    SUBCASE("rational_polynomial")
    {
        const auto K = loadFromString(withModel("rational_polynomial", "[0.4, -0.05, 0.0006, -0.0004, 0.001, 0.7, -0.02, 0.005]"));
        REQUIRE(K.has_value());
        CHECK(K->model == CameraModel::Pinhole);
        CHECK(K->p1 == doctest::Approx(0.0006));
        CHECK(K->k3 == doctest::Approx(0.001));
        CHECK(K->k4 == doctest::Approx(0.7));
        CHECK(K->k5 == doctest::Approx(-0.02));
        CHECK(K->k6 == doctest::Approx(0.005));
    }
}

TEST_CASE("loadCameraInfoYaml: an unknown distortion_model fails rather than falling back to pinhole")
{
    CHECK_FALSE(loadFromString(withModel("scaramuzza", "[0, 0, 0, 0]")).has_value());
}

TEST_CASE("loadCameraInfoYaml: xi with no distortion_model is read as Mei")
{
    std::string body = kSample;
    const auto at = body.find("distortion_model: insta360_mei_v2\n");
    REQUIRE(at != std::string::npos);
    body.erase(at, std::string("distortion_model: insta360_mei_v2\n").size());

    const auto K = loadFromString(body);
    REQUIRE(K.has_value());
    CHECK(K->model == CameraModel::Mei);
    CHECK(K->xi == doctest::Approx(1.234));
}

// ── loadCameraIdentity ────────────────────────────────────────────────────────
// Independent of loadCameraInfoYaml -- opens the same kind of file again on
// its own and only ever looks at `serial`/`frame_id`/`model`, so these tests
// don't depend on the intrinsics fields being present or valid at all.

TEST_CASE("loadCameraIdentity: reads serial, frame_id and model when all are present")
{
    std::string body = kSample;
    body += "\nserial: SN-12345\nmodel: Insta360 X4\n";

    CameraIdentity id;
    CHECK(loadIdentityFromString(body, id));
    CHECK(id.serial == "SN-12345");
    CHECK(id.frameId == "camera_front");
    CHECK(id.model == "Insta360 X4");
}

TEST_CASE("loadCameraIdentity: a field the file does not name comes back empty")
{
    // kSample has frame_id but no serial.
    CameraIdentity id;
    CHECK(loadIdentityFromString(kSample, id));
    CHECK(id.serial.empty());
    CHECK(id.frameId == "camera_front");
}

TEST_CASE("loadCameraIdentity: a successful load clears a previously-populated id")
{
    // Loading a file that names no camera must drop the previous identity
    // rather than leave it attached to a different one.
    CameraIdentity id;
    id.serial = "stale-serial";
    id.model = "stale-model";
    id.firmware = "stale-firmware";

    CHECK(loadIdentityFromString(kSample, id));
    CHECK(id.serial.empty());
    CHECK(id.model.empty());
    CHECK(id.firmware.empty());
    CHECK(id.frameId == "camera_front");
}

TEST_CASE("loadCameraIdentity: quotes around a value are not taken literally")
{
    std::string body = kSample;
    body += "\nserial: \"SN-12345\"\n";

    CameraIdentity id;
    CHECK(loadIdentityFromString(body, id));
    CHECK(id.serial == "SN-12345");
}

TEST_CASE("loadCameraIdentity: neither field present comes back empty, not a failure")
{
    // Unlike loadCameraInfoYaml, no field here is required -- a file that
    // simply doesn't name a camera is a valid, successful "no identity".
    std::string body = "distortion_model: insta360_mei_v2\nwidth: 640\n";
    CameraIdentity id;
    CHECK(loadIdentityFromString(body, id));
    CHECK(id.empty());
}

TEST_CASE("loadCameraIdentity: a missing file fails cleanly, and leaves id alone")
{
    CameraIdentity id;
    id.serial = "untouched";

    CHECK_FALSE(loadCameraIdentity("/nonexistent/camera_info.yaml", id));
    CHECK(id.serial == "untouched");
}

// ── LoadTimestampFromSideCar ────────────────────────────────────────────────

namespace
{
    // Real-world sample, trimmed from a libcamera-style .meta.json sidecar
    // next to a captured frame -- FRAME_WALL_CLOCK is a quoted nanosecond
    // epoch string, not a bare JSON number.
    const char* kMetaSample = R"({
    "AE_STATE": "2",
    "ANALOGUE_GAIN": "1.000000",
    "EXPOSURE_TIME": 6.34,
    "FRAME_DURATION": 16.68,
    "FRAME_WALL_CLOCK": "1789125060554994432",
    "LUX": "580.969055"
})";

    // Writes `metaBody` to "<dir>/<stem>.meta.json" and calls
    // LoadTimestampFromSideCar on "<dir>/<stem>.<ext>" (a file that need not
    // itself exist -- only the sidecar is read).
    std::optional<double> loadTimestampForStem(const std::string& stem, const std::string& ext, const std::string& metaBody)
    {
        const auto dir = std::filesystem::temp_directory_path();
        const std::string sidecar = (dir / (stem + ".meta.json")).string();
        {
            std::ofstream f(sidecar);
            f << metaBody;
        }
        const auto result = LoadTimestampFromSideCar((dir / (stem + "." + ext)).string());
        std::filesystem::remove(sidecar);
        return result;
    }
} // namespace

TEST_CASE("LoadTimestampFromSideCar: reads FRAME_WALL_CLOCK from the image's .meta.json")
{
    const auto ts = loadTimestampForStem("calib_core_test_cam0_frame", "jpg", kMetaSample);
    REQUIRE(ts.has_value());
    CHECK(*ts == doctest::Approx(1789125060554994432.0));
}

TEST_CASE("LoadTimestampFromSideCar: a missing sidecar returns nullopt")
{
    const auto dir = std::filesystem::temp_directory_path();
    const auto missing = (dir / "calib_core_test_no_such_frame.jpg").string();
    CHECK_FALSE(LoadTimestampFromSideCar(missing).has_value());
}

TEST_CASE("LoadTimestampFromSideCar: a sidecar with no FRAME_WALL_CLOCK returns nullopt")
{
    const auto ts = loadTimestampForStem("calib_core_test_cam0_nofield", "jpg", R"({"LUX": "580.969055"})");
    CHECK_FALSE(ts.has_value());
}

TEST_CASE("LoadTimestampFromSideCar: an unquoted numeric value is read too")
{
    const auto ts = loadTimestampForStem("calib_core_test_cam0_unquoted", "jpg", R"({"FRAME_WALL_CLOCK": 1789125060554994432})");
    REQUIRE(ts.has_value());
    CHECK(*ts == doctest::Approx(1789125060554994432.0));
}
