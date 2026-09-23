// Trajectory CSV loading tests; registers into test_camera.cpp's doctest main().
#include <doctest.h>

#include <CalibCore/Trajectory.h>

#include <filesystem>
#include <fstream>

using namespace calib;

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
