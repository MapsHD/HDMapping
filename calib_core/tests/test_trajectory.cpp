// Trajectory loading and smoothing tests; registers into test_camera.cpp's doctest main().
#include <doctest.h>

#include <CalibCore/Trajectory.h>

#include <cmath>
#include <filesystem>
#include <fstream>

using namespace calib;

//! 4 s at 10 Hz walking +x at 1 m/s, with a 2 Hz sideways sway (0.1 m) and roll
//! (10 deg) that both peak at t = 2 s -- the kind of jitter smoothedPose() removes.
static Trajectory swayingWalk()
{
    Trajectory t;
    for (int k = 0; k <= 40; ++k)
    {
        const double sec = k * 0.1;
        const double phase = std::cos(4.0 * M_PI * (sec - 2.0));
        TrajPose p;
        p.ts_ns = static_cast<int64_t>(k) * 100'000'000;
        p.T.translation() = Eigen::Vector3f(float(sec), float(0.1 * phase), 0.f);
        p.T.linear() = Eigen::AngleAxisf(float(10.0 * M_PI / 180.0 * phase), Eigen::Vector3f::UnitX()).toRotationMatrix();
        t.poses.push_back(p);
    }
    return t;
}

TEST_CASE("Trajectory::loadCSV: fractional timestamp does not shift the pose columns")
{
    // lidar_odometry_step_1 writes seconds * 1e9 as a double, so a row can
    // carry a fraction like this one (seen in a real session).
    const auto path = std::filesystem::temp_directory_path() / "calib_core_test_trajectory_lio_0.csv";
    {
        std::ofstream f(path);
        f << "timestamp_nanoseconds pose00 pose01 pose02 pose03 pose10 pose11 pose12 pose13 pose20 pose21 pose22 pose23 "
             "timestampUnix_nanoseconds om_rad fi_rad ka_rad\n";
        f << "548348730180 1 0 0 1 0 1 0 2 0 0 1 3 0 0 0 0\n";
        f << "548348730189.99993896 1 0 0 4 0 1 0 5 0 0 1 6 0 0 0 0\n";
    }

    Trajectory t;
    REQUIRE(t.loadCSV(path.string()));
    std::filesystem::remove(path);

    REQUIRE(t.poses.size() == 2);
    CHECK(t.poses[1].ts_ns == 548348730190);
    CHECK(t.poses[1].T.linear().isIdentity(1e-6f));
    CHECK(t.poses[1].T.translation().isApprox(Eigen::Vector3f(4.f, 5.f, 6.f)));
}

TEST_CASE("Trajectory::smoothedPose: averages out sway and roll, keeps the path")
{
    const Trajectory t = swayingWalk();
    const int64_t mid = t.timeAt(0.5f);
    REQUIRE(mid == 2'000'000'000);

    // The raw pose at the sway peak is 0.1 m off the path and rolled 10 deg.
    const Eigen::Affine3f& raw = t.nearest(mid)->get().T;
    CHECK(raw.translation().y() == doctest::Approx(0.1f));

    const auto smooth = t.smoothedPose(mid, 1.0);
    REQUIRE(smooth);
    CHECK(smooth->translation().x() == doctest::Approx(2.0f).epsilon(1e-4));
    CHECK(std::abs(smooth->translation().y()) < 1e-3f);
    CHECK(Eigen::AngleAxisf(smooth->linear()).angle() < float(0.1 * M_PI / 180.0));
}

TEST_CASE("Trajectory::smoothedPose: nearest pose without a window, nullopt when empty")
{
    const Trajectory t = swayingWalk();
    const int64_t mid = t.timeAt(0.5f);
    const auto off = t.smoothedPose(mid, 0.0);
    REQUIRE(off);
    CHECK(off->isApprox(t.nearest(mid)->get().T));

    // A window narrower than the 0.1 s pose spacing, between two poses, holds none.
    const auto sparse = t.smoothedPose(mid + 30'000'000, 0.01);
    REQUIRE(sparse);
    CHECK(sparse->isApprox(t.nearest(mid + 30'000'000)->get().T));

    CHECK_FALSE(Trajectory{}.smoothedPose(0, 1.0));
}

TEST_CASE("levelHorizon: removes roll, keeps heading and pitch")
{
    const Eigen::Matrix3f R = (Eigen::AngleAxisf(0.5f, Eigen::Vector3f::UnitZ()) *
                               Eigen::AngleAxisf(-0.3f, Eigen::Vector3f::UnitY()) *
                               Eigen::AngleAxisf(0.25f, Eigen::Vector3f::UnitX()))
                                  .toRotationMatrix();
    const Eigen::Matrix3f L = levelHorizon(R);
    CHECK(L.col(0).isApprox(R.col(0)));
    CHECK(std::abs(L.col(1).z()) < 1e-6f); // left axis is level
    CHECK(L.col(2).z() > 0.f); // still upright
    CHECK((L.transpose() * L).isIdentity(1e-5f));
    CHECK(L.determinant() == doctest::Approx(1.0f));

    // Looking straight up, there is no horizon to level.
    const Eigen::Matrix3f up = Eigen::AngleAxisf(float(-M_PI / 2), Eigen::Vector3f::UnitY()).toRotationMatrix();
    CHECK(levelHorizon(up).isApprox(up));
}
