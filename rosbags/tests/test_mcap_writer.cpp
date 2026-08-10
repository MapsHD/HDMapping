#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include "McapWriter.h"
#include "cdr_serializer.hpp"

// No MCAP_IMPLEMENTATION define here: McapWriter.cpp (linked into this test
// binary) already provides the mcap library's implementation once.
#include <mcap/reader.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <random>

namespace fs = std::filesystem;

namespace
{

fs::path tempMcapPath(const char* name)
{
    return fs::temp_directory_path() / name;
}

std::vector<rosbags::McapPoint> makePoints(size_t n, double t0)
{
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> coord(-50.0f, 50.0f);

    std::vector<rosbags::McapPoint> points;
    points.reserve(n);
    for (size_t i = 0; i < n; ++i)
    {
        rosbags::McapPoint p{};
        p.x = coord(rng);
        p.y = coord(rng);
        p.z = coord(rng);
        p.intensity = static_cast<float>(i % 256);
        p.ring = static_cast<uint16_t>(i % 16);
        p.laser_id = static_cast<uint8_t>(i % 4);
        p.timestamp = t0 + static_cast<double>(i) * 1e-6;
        points.push_back(p);
    }
    return points;
}

// Reads every message on `topic` back out of an mcap file and decodes it with `decode`.
template<typename Decoded, typename Decode>
std::vector<Decoded> readTopic(const fs::path& path, const std::string& topic, Decode decode)
{
    std::vector<Decoded> out;
    mcap::McapReader reader;
    auto status = reader.open(path.string());
    REQUIRE(status.ok());

    auto messages = reader.readMessages();
    for (const auto& view : messages)
    {
        if (view.channel->topic != topic)
            continue;
        auto decoded = decode(reinterpret_cast<const uint8_t*>(view.message.data), view.message.dataSize);
        out.push_back(std::move(decoded));
    }
    reader.close();
    return out;
}

} // namespace

TEST_CASE("McapFileWriter: PointCloud2 round-trip (Generic layout)")
{
    const auto path = tempMcapPath("hdmapping_test_generic.mcap");
    const auto points = makePoints(500, 1000.0);

    {
        rosbags::McapWriterOptions options;
        rosbags::McapFileWriter writer(path, options);
        REQUIRE(writer.isOpen());
        writer.writePointCloud(static_cast<uint64_t>(points.front().timestamp * 1e9), points);
    }

    const auto clouds = readTopic<std::vector<rosbags::McapPoint>>(
        path, "/lidar_points", [](const uint8_t* d, size_t n)
        {
            return rosbags::decodePc2(d, n);
        });

    REQUIRE(clouds.size() == 1);
    const auto& decoded = clouds.front();
    REQUIRE(decoded.size() == points.size());

    for (size_t i = 0; i < points.size(); ++i)
    {
        CHECK(decoded[i].x == doctest::Approx(points[i].x));
        CHECK(decoded[i].y == doctest::Approx(points[i].y));
        CHECK(decoded[i].z == doctest::Approx(points[i].z));
        CHECK(decoded[i].intensity == doctest::Approx(points[i].intensity));
        CHECK(decoded[i].ring == points[i].ring);
        CHECK(decoded[i].laser_id == points[i].laser_id);
        CHECK(decoded[i].timestamp == doctest::Approx(points[i].timestamp).epsilon(1e-6));
    }

    fs::remove(path);
}

TEST_CASE("McapFileWriter: PointCloud2 round-trip (Velodyne layout drops laser_id)")
{
    const auto path = tempMcapPath("hdmapping_test_velodyne.mcap");
    const auto points = makePoints(200, 2000.0);

    {
        rosbags::McapWriterOptions options;
        options.lidar_layout = rosbags::PointCloudLayout::Velodyne;
        rosbags::McapFileWriter writer(path, options);
        REQUIRE(writer.isOpen());
        writer.writePointCloud(static_cast<uint64_t>(points.front().timestamp * 1e9), points);
    }

    const auto clouds = readTopic<std::vector<rosbags::McapPoint>>(
        path, "/lidar_points", [](const uint8_t* d, size_t n)
        {
            return rosbags::decodePc2(d, n);
        });

    REQUIRE(clouds.size() == 1);
    const auto& decoded = clouds.front();
    REQUIRE(decoded.size() == points.size());

    for (size_t i = 0; i < points.size(); ++i)
    {
        CHECK(decoded[i].x == doctest::Approx(points[i].x));
        CHECK(decoded[i].ring == points[i].ring);
        CHECK(decoded[i].laser_id == 0); // Velodyne layout has no laser_id field
        CHECK(decoded[i].timestamp == doctest::Approx(points[i].timestamp).epsilon(1e-6));
    }

    fs::remove(path);
}

TEST_CASE("McapFileWriter: PointCloud2 round-trip (Ouster layout, relative uint32 ns time)")
{
    const auto path = tempMcapPath("hdmapping_test_ouster.mcap");
    const auto points = makePoints(200, 3000.0);

    {
        rosbags::McapWriterOptions options;
        options.lidar_layout = rosbags::PointCloudLayout::Ouster;
        rosbags::McapFileWriter writer(path, options);
        REQUIRE(writer.isOpen());
        writer.writePointCloud(static_cast<uint64_t>(points.front().timestamp * 1e9), points);
    }

    const auto clouds = readTopic<std::vector<rosbags::McapPoint>>(
        path, "/lidar_points", [](const uint8_t* d, size_t n)
        {
            return rosbags::decodePc2(d, n);
        });

    REQUIRE(clouds.size() == 1);
    const auto& decoded = clouds.front();
    REQUIRE(decoded.size() == points.size());

    for (size_t i = 0; i < points.size(); ++i)
    {
        CHECK(decoded[i].x == doctest::Approx(points[i].x));
        CHECK(decoded[i].ring == points[i].ring);
        CHECK(decoded[i].laser_id == 0); // Ouster layout has no laser_id field
        // "t" is uint32 ns, so round-trip precision is ~1ns rather than exact double equality.
        CHECK(decoded[i].timestamp == doctest::Approx(points[i].timestamp).epsilon(1e-8));
    }

    fs::remove(path);
}

TEST_CASE("McapFileWriter: PointCloud2 round-trip (Hesai layout, absolute timestamp)")
{
    const auto path = tempMcapPath("hdmapping_test_hesai.mcap");
    const auto points = makePoints(200, 4000.0);

    {
        rosbags::McapWriterOptions options;
        options.lidar_layout = rosbags::PointCloudLayout::Hesai;
        rosbags::McapFileWriter writer(path, options);
        REQUIRE(writer.isOpen());
        writer.writePointCloud(static_cast<uint64_t>(points.front().timestamp * 1e9), points);
    }

    const auto clouds = readTopic<std::vector<rosbags::McapPoint>>(
        path, "/lidar_points", [](const uint8_t* d, size_t n)
        {
            return rosbags::decodePc2(d, n);
        });

    REQUIRE(clouds.size() == 1);
    const auto& decoded = clouds.front();
    REQUIRE(decoded.size() == points.size());

    for (size_t i = 0; i < points.size(); ++i)
    {
        CHECK(decoded[i].x == doctest::Approx(points[i].x));
        CHECK(decoded[i].ring == points[i].ring);
        CHECK(decoded[i].laser_id == 0); // Hesai layout has no laser_id field
        // timestamp is written as a raw absolute double -- exact round-trip.
        CHECK(decoded[i].timestamp == points[i].timestamp);
    }

    fs::remove(path);
}

TEST_CASE("McapFileWriter: IMU round-trip")
{
    const auto path = tempMcapPath("hdmapping_test_imu.mcap");

    std::vector<rosbags::McapImuSample> samples;
    for (int i = 0; i < 10; ++i)
    {
        rosbags::McapImuSample s{};
        s.timestamp = 5000.0 + i * 0.005;
        s.gyro_x = 0.01f * i;
        s.gyro_y = -0.02f * i;
        s.gyro_z = 0.03f * i;
        s.acc_x = 0.1f;
        s.acc_y = 0.2f;
        s.acc_z = 9.81f;
        samples.push_back(s);
    }

    {
        rosbags::McapFileWriter writer(path);
        REQUIRE(writer.isOpen());
        writer.writeImu(samples);
    }

    const auto decoded = readTopic<std::optional<rosbags::McapImuSample>>(
        path, "/imu", [](const uint8_t* d, size_t n)
        {
            return rosbags::decodeImu(d, n);
        });

    REQUIRE(decoded.size() == samples.size());
    for (size_t i = 0; i < samples.size(); ++i)
    {
        REQUIRE(decoded[i].has_value());
        CHECK(decoded[i]->gyro_x == doctest::Approx(samples[i].gyro_x));
        CHECK(decoded[i]->gyro_y == doctest::Approx(samples[i].gyro_y));
        CHECK(decoded[i]->gyro_z == doctest::Approx(samples[i].gyro_z));
        CHECK(decoded[i]->acc_x == doctest::Approx(samples[i].acc_x));
        CHECK(decoded[i]->acc_y == doctest::Approx(samples[i].acc_y));
        CHECK(decoded[i]->acc_z == doctest::Approx(samples[i].acc_z));
        CHECK(decoded[i]->timestamp == doctest::Approx(samples[i].timestamp).epsilon(1e-6));
    }

    fs::remove(path);
}

TEST_CASE("McapFileWriter: custom topic names are honored")
{
    const auto path = tempMcapPath("hdmapping_test_topics.mcap");

    rosbags::McapWriterOptions options;
    options.lidar_topic = "/custom/points";
    options.imu_topic = "/custom/imu";
    options.sn_topic = "/custom/sn";

    const auto points = makePoints(10, 42.0);
    {
        rosbags::McapFileWriter writer(path, options);
        REQUIRE(writer.isOpen());
        writer.writePointCloud(static_cast<uint64_t>(points.front().timestamp * 1e9), points);
        writer.writeSn(0, "SN123");
    }

    mcap::McapReader reader;
    auto status = reader.open(path.string());
    REQUIRE(status.ok());
    status = reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
    REQUIRE(status.ok());

    std::vector<std::string> topics;
    for (const auto& [id, channel] : reader.channels())
        topics.push_back(channel->topic);
    reader.close();

    CHECK(std::find(topics.begin(), topics.end(), "/custom/points") != topics.end());
    CHECK(std::find(topics.begin(), topics.end(), "/custom/imu") != topics.end());
    CHECK(std::find(topics.begin(), topics.end(), "/custom/sn") != topics.end());

    fs::remove(path);
}