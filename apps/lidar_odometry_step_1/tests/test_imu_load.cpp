#include <doctest.h>

#include "../lidar_odometry_utils.h"

namespace
{
    using ImuSamples = std::vector<std::tuple<std::pair<double, double>, Eigen::Vector3f, Eigen::Vector3f>>;

    // One sample at `t` [s]; the Unix timestamp is t + 1000, gyro and accelerometer readings are `value` on every axis
    void appendSample(ImuSamples& imu, double t, float value)
    {
        imu.emplace_back(std::pair(t, t + 1000.0), Eigen::Vector3f::Constant(value), Eigen::Vector3f::Constant(-value));
    }
} // namespace

TEST_CASE("average_imu_to_rate: a rate <= 0 returns the samples unchanged")
{
    ImuSamples imu;
    appendSample(imu, 10.000, 1.0f);
    appendSample(imu, 10.001, 2.0f);

    for (const double rate : { -1.0, 0.0 })
    {
        const auto averaged = average_imu_to_rate(imu, rate);
        REQUIRE(averaged.size() == imu.size());
        for (size_t i = 0; i < imu.size(); i++)
        {
            CHECK(std::get<0>(averaged[i]) == std::get<0>(imu[i]));
            CHECK(std::get<1>(averaged[i]) == std::get<1>(imu[i]));
            CHECK(std::get<2>(averaged[i]) == std::get<2>(imu[i]));
        }
    }
}

TEST_CASE("average_imu_to_rate: each bin of 1 / rate seconds becomes the mean of its readings at its last sample's time")
{
    ImuSamples imu;
    appendSample(imu, 10.000, 1.0f); // 200 Hz: bin [10.000, 10.005)
    appendSample(imu, 10.001, 2.0f);
    appendSample(imu, 10.002, 3.0f);
    appendSample(imu, 10.003, 6.0f);
    appendSample(imu, 10.006, 10.0f); // bin [10.005, 10.010)
    appendSample(imu, 10.008, 20.0f);

    const auto averaged = average_imu_to_rate(imu, 200.0);
    REQUIRE(averaged.size() == 2);

    CHECK(std::get<0>(averaged[0]).first == doctest::Approx(10.003));
    CHECK(std::get<0>(averaged[0]).second == doctest::Approx(1010.003));
    CHECK(std::get<1>(averaged[0]).x() == doctest::Approx(3.0));
    CHECK(std::get<2>(averaged[0]).z() == doctest::Approx(-3.0));

    CHECK(std::get<0>(averaged[1]).first == doctest::Approx(10.008));
    CHECK(std::get<1>(averaged[1]).y() == doctest::Approx(15.0));
    CHECK(std::get<2>(averaged[1]).x() == doctest::Approx(-15.0));
}

TEST_CASE("average_imu_to_rate: samples on either side of a gap stay apart")
{
    ImuSamples imu;
    appendSample(imu, 10.0000, 1.0f);
    appendSample(imu, 10.0010, 3.0f);
    appendSample(imu, 10.1002, 5.0f); // 20 bins later, the bins in between are empty
    appendSample(imu, 10.1012, 7.0f);

    const auto averaged = average_imu_to_rate(imu, 200.0);
    REQUIRE(averaged.size() == 2);
    CHECK(std::get<0>(averaged[0]).first == doctest::Approx(10.0010));
    CHECK(std::get<1>(averaged[0]).x() == doctest::Approx(2.0));
    CHECK(std::get<0>(averaged[1]).first == doctest::Approx(10.1012));
    CHECK(std::get<1>(averaged[1]).x() == doctest::Approx(6.0));
}

TEST_CASE("rotate_imu: +90 deg about z maps the IMU x axis to lidar y and y to -x")
{
    ImuSamples imu;
    imu.emplace_back(std::pair(10.0, 1010.0), Eigen::Vector3f(1.0f, 0.0f, 0.5f), Eigen::Vector3f(0.0f, 1.0f, 0.0f));

    // not normalized on purpose: the rotation is the same
    rotate_imu(imu, Eigen::Quaterniond(2.0, 0.0, 0.0, 2.0));

    const auto& [timestamps, gyr, acc] = imu[0];
    CHECK(timestamps.first == 10.0);
    CHECK(timestamps.second == 1010.0);
    CHECK(gyr.x() == doctest::Approx(0.0));
    CHECK(gyr.y() == doctest::Approx(1.0));
    CHECK(gyr.z() == doctest::Approx(0.5));
    CHECK(acc.x() == doctest::Approx(-1.0));
    CHECK(acc.y() == doctest::Approx(0.0));
    CHECK(acc.z() == doctest::Approx(0.0));
}
