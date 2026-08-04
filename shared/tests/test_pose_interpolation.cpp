#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <HDMapping/PoseInterpolation.h>

#include <cmath>

namespace
{
Eigen::Matrix4d makePose(double tx, double ty, double tz, double yawRad = 0.0)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<3, 3>() = Eigen::AngleAxisd(yawRad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T.col(3).head<3>() = Eigen::Vector3d(tx, ty, tz);
    return T;
}

bool isZeroMatrix(const Eigen::Matrix4d& m)
{
    return m.isZero(0.0);
}
} // namespace

TEST_CASE("getInterpolatedPose: empty trajectory returns a zero matrix")
{
    std::map<double, Eigen::Matrix4d> traj;
    CHECK(isZeroMatrix(getInterpolatedPose(traj, 0.0)));
    CHECK(isZeroMatrix(getInterpolatedPose(traj, 123.456)));
}

TEST_CASE("getInterpolatedPose: query before the first timestamp returns a zero matrix")
{
    std::map<double, Eigen::Matrix4d> traj{
        { 1.0, makePose(0, 0, 0) },
        { 2.0, makePose(10, 0, 0) },
        { 3.0, makePose(20, 0, 0) },
    };
    CHECK(isZeroMatrix(getInterpolatedPose(traj, 0.0)));
    CHECK(isZeroMatrix(getInterpolatedPose(traj, -100.0)));
}

TEST_CASE("getInterpolatedPose: exact match at an interior timestamp returns that exact pose")
{
    // Interior = not the very first key -- see the "first segment" case below
    // for why the first key specifically behaves differently.
    std::map<double, Eigen::Matrix4d> traj{
        { 1.0, makePose(0, 0, 0) },
        { 2.0, makePose(10, 0, 0) },
        { 3.0, makePose(20, 0, 0) },
    };
    Eigen::Matrix4d result = getInterpolatedPose(traj, 2.0);
    CHECK(result.isApprox(makePose(10, 0, 0), 1e-9));

    result = getInterpolatedPose(traj, 3.0);
    CHECK(result.isApprox(makePose(20, 0, 0), 1e-9));
}

TEST_CASE("getInterpolatedPose: linearly interpolates translation between two interior poses")
{
    std::map<double, Eigen::Matrix4d> traj{
        { 0.0, makePose(0, 0, 0) },
        { 10.0, makePose(0, 0, 0) },
        { 20.0, makePose(100, 0, 0) },
    };
    // Query inside the *second* segment [10, 20] -- the first segment [0, 10]
    // is covered separately below, since (see that test) it behaves
    // differently from every later segment.
    Eigen::Matrix4d result = getInterpolatedPose(traj, 15.0);
    CHECK(result(0, 3) == doctest::Approx(50.0));
    CHECK(result(1, 3) == doctest::Approx(0.0));
    CHECK(result(2, 3) == doctest::Approx(0.0));

    result = getInterpolatedPose(traj, 12.0);
    CHECK(result(0, 3) == doctest::Approx(20.0));
}

TEST_CASE("getInterpolatedPose: SLERPs rotation between two interior poses")
{
    const double kHalfPi = M_PI / 2.0;
    std::map<double, Eigen::Matrix4d> traj{
        { 0.0, makePose(0, 0, 0, 0.0) },
        { 10.0, makePose(0, 0, 0, 0.0) },
        { 20.0, makePose(0, 0, 0, kHalfPi) },
    };
    Eigen::Matrix4d result = getInterpolatedPose(traj, 15.0);
    Eigen::Matrix3d expectedR = Eigen::AngleAxisd(kHalfPi / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    CHECK(result.topLeftCorner<3, 3>().isApprox(expectedR, 1e-9));
}

TEST_CASE("getInterpolatedPose: exact match at the very first timestamp currently returns a zero matrix")
{
    // Documented current behavior, not necessarily desired: there's no pose
    // "before" the first sample to interpolate from, so getInterpolatedPose
    // treats query_time <= the first timestamp as out-of-range and signals
    // that with a zero matrix, same as querying before the range entirely.
    // This test pins that behavior down so a future change to it is a
    // deliberate, visible decision rather than an accidental regression.
    std::map<double, Eigen::Matrix4d> traj{
        { 1.0, makePose(5, 0, 0) },
        { 2.0, makePose(10, 0, 0) },
    };
    CHECK(isZeroMatrix(getInterpolatedPose(traj, 1.0)));
}

TEST_CASE("getInterpolatedPose: query strictly inside the first segment currently returns a zero matrix")
{
    // Same current limitation as above, one step further: even a query
    // strictly *between* the first and second timestamps returns zero
    // rather than interpolating -- only segments from the second one onward
    // interpolate normally (see the "linearly interpolates" test above,
    // which deliberately queries the *second* segment for that reason).
    std::map<double, Eigen::Matrix4d> traj{
        { 0.0, makePose(0, 0, 0) },
        { 10.0, makePose(100, 0, 0) },
        { 20.0, makePose(200, 0, 0) },
    };
    CHECK(isZeroMatrix(getInterpolatedPose(traj, 5.0)));
}
