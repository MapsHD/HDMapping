// See rosbags/McapWriter.h for why this needs to come before any standard
// header (including <doctest.h>) on MSVC.
#if defined(_MSC_VER)
#  undef _HAS_STD_BYTE
#  define _HAS_STD_BYTE 1
#endif

#include <doctest.h>

#include "Ros1Bag.h"

#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>

namespace fs = std::filesystem;

namespace
{

rosbags::Ros1PointCloud2 makeCloud(uint32_t seq, uint64_t stamp_ns, uint32_t n)
{
    rosbags::Ros1PointCloud2 c;
    c.seq = seq;
    c.stamp_ns = stamp_ns;
    c.frame_id = "lidar";
    c.width = n;
    c.fields = {{"x", 0, 7, 1}, {"y", 4, 7, 1}, {"z", 8, 7, 1}, {"ring", 12, 4, 1}};
    c.point_step = 16;
    c.row_step = n * c.point_step;
    c.data.resize(c.row_step);
    for (size_t i = 0; i < c.data.size(); ++i)
        c.data[i] = static_cast<uint8_t>((i * 7 + seq) & 0xff);
    return c;
}

rosbags::Ros1Imu makeImu(uint32_t seq, uint64_t stamp_ns)
{
    rosbags::Ros1Imu m;
    m.seq = seq;
    m.stamp_ns = stamp_ns;
    m.frame_id = "lidar";
    m.angular_velocity[0] = 0.1 * seq;
    m.angular_velocity[2] = -0.5;
    m.linear_acceleration[2] = 9.81;
    return m;
}

} // namespace

TEST_CASE("Ros1Bag: PointCloud2 encode/decode round-trip")
{
    const auto in = makeCloud(7, 1'710'255'656'973'876'000ULL, 50);
    const auto bytes = rosbags::encodeRos1PointCloud2(in);
    rosbags::Ros1PointCloud2 out;
    REQUIRE(rosbags::decodeRos1PointCloud2(bytes.data(), bytes.size(), out));
    CHECK(out.seq == in.seq);
    CHECK(out.stamp_ns == in.stamp_ns);
    CHECK(out.frame_id == in.frame_id);
    CHECK(out.width == in.width);
    REQUIRE(out.fields.size() == in.fields.size());
    CHECK(out.fields[3].name == "ring");
    CHECK(out.fields[3].datatype == 4);
    CHECK(out.point_step == in.point_step);
    CHECK(out.data == in.data);
    CHECK(out.is_dense);

    // A truncated message must be rejected, not read past its end.
    CHECK_FALSE(rosbags::decodeRos1PointCloud2(bytes.data(), bytes.size() - 10, out));
}

TEST_CASE("Ros1Bag: Imu encode/decode round-trip")
{
    const auto in = makeImu(3, 42'000'000'123ULL);
    const auto bytes = rosbags::encodeRos1Imu(in);
    // header (4 + 8 + 4 + 5) + 4 + 9 + 3 + 9 + 3 + 9 doubles
    CHECK(bytes.size() == 21 + 37 * 8);
    rosbags::Ros1Imu out;
    REQUIRE(rosbags::decodeRos1Imu(bytes.data(), bytes.size(), out));
    CHECK(out.stamp_ns == in.stamp_ns);
    CHECK(out.orientation[3] == 1.0);
    CHECK(out.orientation_covariance[0] == -1.0);
    CHECK(out.angular_velocity[0] == in.angular_velocity[0]);
    CHECK(out.angular_velocity[2] == in.angular_velocity[2]);
    CHECK(out.linear_acceleration[2] == in.linear_acceleration[2]);
}

TEST_CASE("Ros1Bag: writer output reads back message for message across chunks")
{
    const auto path = fs::temp_directory_path() / "hdmapping_test_ros1.bag";
    const uint64_t t0 = 1'710'255'656'000'000'000ULL;
    std::vector<std::vector<uint8_t>> clouds, imus;

    {
        // Small chunks, so the bag has many chunks and index records to get right.
        rosbags::Ros1BagWriter writer(path, 4096);
        REQUIRE(writer.isOpen());
        const auto lidar = writer.addConnection(
            "/lidar/pointcloud",
            rosbags::kRos1PointCloud2Type,
            rosbags::kRos1PointCloud2Md5,
            rosbags::kRos1PointCloud2Definition);
        const auto imu = writer.addConnection(
            "/lidar/imu", rosbags::kRos1ImuType, rosbags::kRos1ImuMd5, rosbags::kRos1ImuDefinition);
        for (uint32_t i = 0; i < 20; ++i)
        {
            const uint64_t t = t0 + i * 100'000'000ULL;
            clouds.push_back(rosbags::encodeRos1PointCloud2(makeCloud(i, t, 100)));
            writer.write(lidar, t, clouds.back());
            for (uint32_t k = 0; k < 40; ++k)
            {
                const uint64_t ti = t + k * 2'500'000ULL;
                imus.push_back(rosbags::encodeRos1Imu(makeImu(i * 40 + k, ti)));
                writer.write(imu, ti, imus.back());
            }
        }
        REQUIRE(writer.close());
    }

    rosbags::Ros1BagReader reader(path);
    REQUIRE(reader.isOpen());
    // Connections come from the index section, before any message is read.
    REQUIRE(reader.connections().size() == 2);
    CHECK(reader.connections().at(0).topic == "/lidar/pointcloud");
    CHECK(reader.connections().at(0).md5sum == rosbags::kRos1PointCloud2Md5);
    CHECK(reader.connections().at(1).type == rosbags::kRos1ImuType);
    CHECK(reader.connections().at(1).message_definition == rosbags::kRos1ImuDefinition);

    size_t n_cloud = 0, n_imu = 0;
    uint64_t last_cloud_time = 0;
    const bool ok = reader.forEachMessage(
        [&](const rosbags::Ros1Connection& c, uint64_t time_ns, const uint8_t* data, size_t size)
        {
            const std::vector<uint8_t> bytes(data, data + size);
            if (c.topic == "/lidar/pointcloud")
            {
                REQUIRE(n_cloud < clouds.size());
                CHECK(bytes == clouds[n_cloud]);
                CHECK(time_ns == t0 + n_cloud * 100'000'000ULL);
                last_cloud_time = time_ns;
                ++n_cloud;
            }
            else
            {
                REQUIRE(n_imu < imus.size());
                CHECK(bytes == imus[n_imu]);
                ++n_imu;
            }
            return true;
        });
    CHECK(ok);
    CHECK(n_cloud == clouds.size());
    CHECK(n_imu == imus.size());
    CHECK(last_cloud_time == t0 + 19 * 100'000'000ULL);

    // Stopping early is honoured.
    size_t seen = 0;
    CHECK(reader.forEachMessage(
        [&](const rosbags::Ros1Connection&, uint64_t, const uint8_t*, size_t)
        {
            return ++seen < 5;
        }));
    CHECK(seen == 5);

    fs::remove(path);
}

TEST_CASE("Ros1Bag: reader rejects a file that is not a ROS1 bag")
{
    const auto path = fs::temp_directory_path() / "hdmapping_test_not_a_bag.bag";
    {
        std::ofstream f(path, std::ios::binary);
        f << "#ROSBAG V1.2\nnot a bag";
    }
    rosbags::Ros1BagReader reader(path);
    CHECK_FALSE(reader.isOpen());
    CHECK_FALSE(reader.error().empty());
    fs::remove(path);
}

TEST_CASE("Ros1BagFileWriter -> Ros1BagFileReader: canonical layout and IMU round-trip")
{
    const auto path = fs::temp_directory_path() / "hdmapping_test_ros1_canonical.bag";
    const double t0 = 1'710'255'656.5;
    std::vector<std::vector<rosbags::McapPoint>> clouds(3);
    for (size_t c = 0; c < clouds.size(); ++c)
    {
        for (uint32_t i = 0; i < 640; ++i)
        {
            rosbags::McapPoint p{};
            p.x = 0.01f * static_cast<float>(i);
            p.y = -0.02f * static_cast<float>(i);
            p.z = 1.5f;
            p.intensity = static_cast<float>(i % 200);
            p.ring = static_cast<uint16_t>(i % 64);
            p.timestamp = t0 + 0.1 * static_cast<double>(c) + 1e-4 * static_cast<double>(i / 64);
            clouds[c].push_back(p);
        }
    }
    std::vector<rosbags::McapImuSample> imu;
    for (int k = 0; k < 120; ++k)
        imu.push_back({t0 + 0.0025 * k, 0.01f * k, -0.02f, 0.3f, 0.1f, -0.2f, 9.81f});

    {
        rosbags::McapWriterOptions options;
        options.lidar_topic = "/lidar/pointcloud";
        options.imu_topic = "/lidar/imu";
        options.lidar_layout = rosbags::PointCloudLayout::Canonical;
        rosbags::Ros1BagFileWriter writer(path, options);
        REQUIRE_MESSAGE(writer.isOpen(), writer.error());
        for (const auto& c : clouds)
            writer.writePointCloud(static_cast<uint64_t>(std::llround(c.front().timestamp * 1e9)), c);
        writer.writeImu(imu);
        REQUIRE(writer.close());
    }

    // Topics are auto-detected by type, as for MCAP.
    rosbags::Ros1BagFileReader reader(path);
    REQUIRE_MESSAGE(reader.isOpen(), reader.error());
    CHECK(reader.topics().lidar == "/lidar/pointcloud");
    CHECK(reader.topics().imu == "/lidar/imu");
    CHECK(reader.topics().lidar_schema == rosbags::LidarSchema::PointCloud2);
    CHECK(reader.channels().size() == 2); // no empty /lidar_sn topic

    std::vector<std::vector<rosbags::McapPoint>> got;
    std::vector<rosbags::McapImuSample> got_imu;
    rosbags::Ros1BagFileReader::Callbacks callbacks;
    callbacks.onPointCloud = [&](uint64_t, std::vector<rosbags::McapPoint>&& points)
    {
        got.push_back(std::move(points));
    };
    callbacks.onImu = [&](const rosbags::McapImuSample& s)
    {
        got_imu.push_back(s);
    };
    REQUIRE(reader.read(callbacks));
    CHECK(reader.lidarLayoutWarning().empty());

    REQUIRE(got.size() == clouds.size());
    for (size_t c = 0; c < clouds.size(); ++c)
    {
        REQUIRE(got[c].size() == clouds[c].size());
        for (size_t i = 0; i < clouds[c].size(); ++i)
        {
            CHECK(got[c][i].x == clouds[c][i].x);
            CHECK(got[c][i].intensity == clouds[c][i].intensity);
            CHECK(got[c][i].ring == clouds[c][i].ring);
            // decodePc2Points() takes the float32 `time` (seconds since the first point), which
            // resolves well below a microsecond over a 0.1 ms spread.
            CHECK(got[c][i].timestamp == doctest::Approx(clouds[c][i].timestamp).epsilon(1e-12));
        }
    }
    REQUIRE(got_imu.size() == imu.size());
    for (size_t k = 0; k < imu.size(); ++k)
    {
        CHECK(got_imu[k].gyro_x == imu[k].gyro_x);
        CHECK(got_imu[k].acc_z == imu[k].acc_z);
        CHECK(got_imu[k].timestamp == doctest::Approx(imu[k].timestamp).epsilon(1e-12));
    }

    fs::remove(path);
}

TEST_CASE("Ros1BagFileReader: livox_ros_driver CustomMsg becomes points with ring = line")
{
    const auto path = fs::temp_directory_path() / "hdmapping_test_ros1_livox.bag";
    const uint64_t timebase = 1'700'000'000'000'000'000ULL;
    {
        // ROS1 serialization of livox_ros_driver/CustomMsg with three points.
        std::vector<uint8_t> msg;
        const auto put = [&msg](const void* v, size_t n)
        {
            const auto* b = static_cast<const uint8_t*>(v);
            msg.insert(msg.end(), b, b + n);
        };
        const uint32_t seq = 0, sec = 1'700'000'000, nsec = 0, frame_len = 0;
        put(&seq, 4);
        put(&sec, 4);
        put(&nsec, 4);
        put(&frame_len, 4);
        const uint32_t point_num = 3, n_points = 3;
        const uint8_t lidar_id = 192, rsvd[3] = {0, 0, 0};
        put(&timebase, 8);
        put(&point_num, 4);
        put(&lidar_id, 1);
        put(rsvd, 3);
        put(&n_points, 4);
        for (uint32_t i = 0; i < 3; ++i)
        {
            const uint32_t offset_time = i * 1000;
            const float xyz[3] = {1.0f + i, 2.0f, 3.0f};
            const uint8_t reflectivity = static_cast<uint8_t>(10 * i), tag = 0x10, line = static_cast<uint8_t>(i);
            put(&offset_time, 4);
            put(xyz, 12);
            put(&reflectivity, 1);
            put(&tag, 1);
            put(&line, 1);
        }
        rosbags::Ros1BagWriter writer(path);
        const auto conn = writer.addConnection("/livox/lidar", "livox_ros_driver/CustomMsg", "e4d6829bdfe657cb6c21a746c86b21a6", "");
        writer.write(conn, timebase, msg);
        REQUIRE(writer.close());
    }

    rosbags::Ros1BagFileReader reader(path);
    REQUIRE_MESSAGE(reader.isOpen(), reader.error());
    CHECK(reader.topics().lidar == "/livox/lidar");
    CHECK(reader.topics().lidar_schema == rosbags::LidarSchema::LivoxCustomMsg);
    std::vector<rosbags::McapPoint> points;
    rosbags::Ros1BagFileReader::Callbacks callbacks;
    callbacks.onPointCloud = [&](uint64_t, std::vector<rosbags::McapPoint>&& p)
    {
        points = std::move(p);
    };
    REQUIRE(reader.read(callbacks));
    REQUIRE(points.size() == 3);
    for (uint32_t i = 0; i < 3; ++i)
    {
        CHECK(points[i].x == 1.0f + i);
        CHECK(points[i].ring == i);
        CHECK(points[i].intensity == 10.0f * i);
        CHECK(points[i].timestamp == doctest::Approx(1.7e9 + i * 1e-6).epsilon(1e-15));
    }
    fs::remove(path);
}
