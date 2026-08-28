// See rosbags/McapWriter.h for why this needs to come before any standard
// header (including <doctest.h>, which pulls in plenty of its own) gets a
// chance to lock in the project-wide _HAS_STD_BYTE=0 MSVC workaround.
#if defined(_MSC_VER)
#  undef _HAS_STD_BYTE
#  define _HAS_STD_BYTE 1
#endif

// No DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN here: test_mcap_writer.cpp (linked into
// the same binary) provides doctest's main once.
#include <doctest.h>

#include "MandeyeSessionWriter.h"
#include "McapReader.h"
#include "McapWriter.h"

#include <laszip/laszip_api.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{

fs::path tempPath(const char* name)
{
    return fs::temp_directory_path() / name;
}

std::vector<rosbags::McapPoint> makeReaderPoints(size_t n, double t0)
{
    std::mt19937 rng(7);
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
        p.laser_id = static_cast<uint8_t>(1 + i % 3); // non-zero, so the round trip can be checked
        p.timestamp = t0 + static_cast<double>(i) * 1e-6;
        points.push_back(p);
    }
    return points;
}

std::vector<rosbags::McapImuSample> makeImuSamples(size_t n, double t0)
{
    std::vector<rosbags::McapImuSample> samples;
    samples.reserve(n);
    for (size_t i = 0; i < n; ++i)
    {
        rosbags::McapImuSample s{};
        s.timestamp = t0 + static_cast<double>(i) * 0.005;
        s.gyro_x = 0.01f * static_cast<float>(i);
        s.gyro_y = -0.02f * static_cast<float>(i);
        s.gyro_z = 0.03f * static_cast<float>(i);
        s.acc_x = 0.1f;
        s.acc_y = 0.2f;
        s.acc_z = 9.81f;
        samples.push_back(s);
    }
    return samples;
}

// One point as stored in a LAZ file, decoded exactly the way
// load_point_cloud() (apps/lidar_odometry_step_1/lidar_odometry_utils.cpp) does:
// coordinates from the header scale/offset, timestamp straight out of gps_time,
// lidar id out of user_data.
struct LazPoint
{
    double x, y, z;
    double gps_time;
    uint16_t intensity;
    uint8_t user_data;
};

std::vector<LazPoint> readLaz(const fs::path& path)
{
    std::vector<LazPoint> out;

    laszip_POINTER reader = nullptr;
    REQUIRE(laszip_create(&reader) == 0);

    laszip_BOOL is_compressed = 0;
    REQUIRE(laszip_open_reader(reader, path.string().c_str(), &is_compressed) == 0);

    laszip_header* header = nullptr;
    REQUIRE(laszip_get_header_pointer(reader, &header) == 0);

    laszip_point* point = nullptr;
    REQUIRE(laszip_get_point_pointer(reader, &point) == 0);

    out.reserve(header->number_of_point_records);
    for (laszip_U32 i = 0; i < header->number_of_point_records; ++i)
    {
        REQUIRE(laszip_read_point(reader) == 0);
        LazPoint p{};
        p.x = header->x_offset + header->x_scale_factor * static_cast<double>(point->X);
        p.y = header->y_offset + header->y_scale_factor * static_cast<double>(point->Y);
        p.z = header->z_offset + header->z_scale_factor * static_cast<double>(point->Z);
        p.gps_time = point->gps_time;
        p.intensity = point->intensity;
        p.user_data = point->user_data;
        out.push_back(p);
    }

    laszip_close_reader(reader);
    laszip_destroy(reader);
    return out;
}

std::vector<std::string> readLines(const fs::path& path)
{
    std::vector<std::string> lines;
    std::ifstream in(path);
    std::string line;
    while (std::getline(in, line))
    {
        if (!line.empty())
            lines.push_back(line);
    }
    return lines;
}

} // namespace

TEST_CASE("McapFileReader: PointCloud2 round-trip through every layout")
{
    struct Case
    {
        const char* name;
        rosbags::PointCloudLayout layout;
        bool carries_laser_id;
    };
    const Case cases[] = {
        {"hdmapping_reader_generic.mcap", rosbags::PointCloudLayout::Generic, true},
        {"hdmapping_reader_velodyne.mcap", rosbags::PointCloudLayout::Velodyne, false},
        {"hdmapping_reader_ouster.mcap", rosbags::PointCloudLayout::Ouster, false},
        {"hdmapping_reader_hesai.mcap", rosbags::PointCloudLayout::Hesai, false},
    };

    for (const auto& c : cases)
    {
        CAPTURE(c.name);
        const auto path = tempPath(c.name);
        const auto points = makeReaderPoints(300, 1000.0);

        {
            rosbags::McapWriterOptions options;
            options.lidar_layout = c.layout;
            rosbags::McapFileWriter writer(path, options);
            REQUIRE(writer.isOpen());
            writer.writePointCloud(static_cast<uint64_t>(points.front().timestamp * 1e9), points);
        }

        rosbags::McapFileReader reader(path);
        REQUIRE_MESSAGE(reader.isOpen(), reader.error());

        std::vector<rosbags::McapPoint> decoded;
        size_t messages = 0;
        rosbags::McapFileReader::Callbacks callbacks;
        callbacks.onPointCloud = [&](uint64_t, std::vector<rosbags::McapPoint>&& pts)
        {
            ++messages;
            decoded.insert(decoded.end(), pts.begin(), pts.end());
        };
        REQUIRE(reader.read(callbacks));

        CHECK(messages == 1);
        REQUIRE(decoded.size() == points.size());
        for (size_t i = 0; i < points.size(); ++i)
        {
            CHECK(decoded[i].x == doctest::Approx(points[i].x));
            CHECK(decoded[i].y == doctest::Approx(points[i].y));
            CHECK(decoded[i].z == doctest::Approx(points[i].z));
            CHECK(decoded[i].intensity == doctest::Approx(points[i].intensity));
            CHECK(decoded[i].ring == points[i].ring);
            CHECK(decoded[i].laser_id == (c.carries_laser_id ? points[i].laser_id : 0));
            CHECK(decoded[i].timestamp == doctest::Approx(points[i].timestamp).epsilon(1e-8));
        }

        fs::remove(path);
    }
}

TEST_CASE("McapFileReader: IMU round-trip and serial number")
{
    const auto path = tempPath("hdmapping_reader_imu.mcap");
    const auto samples = makeImuSamples(20, 5000.0);

    {
        rosbags::McapFileWriter writer(path);
        REQUIRE(writer.isOpen());
        writer.writeSn(static_cast<uint64_t>(5000.0 * 1e9), "SN-ABC-123");
        writer.writeImu(samples);
    }

    rosbags::McapFileReader reader(path);
    REQUIRE_MESSAGE(reader.isOpen(), reader.error());

    std::vector<rosbags::McapImuSample> decoded;
    std::string serial;
    rosbags::McapFileReader::Callbacks callbacks;
    callbacks.onImu = [&](const rosbags::McapImuSample& s) { decoded.push_back(s); };
    callbacks.onSn = [&](uint64_t, const std::string& sn) { serial = sn; };
    REQUIRE(reader.read(callbacks));

    CHECK(serial == "SN-ABC-123");
    REQUIRE(decoded.size() == samples.size());
    for (size_t i = 0; i < samples.size(); ++i)
    {
        CHECK(decoded[i].gyro_x == doctest::Approx(samples[i].gyro_x));
        CHECK(decoded[i].gyro_z == doctest::Approx(samples[i].gyro_z));
        CHECK(decoded[i].acc_z == doctest::Approx(samples[i].acc_z));
        CHECK(decoded[i].timestamp == doctest::Approx(samples[i].timestamp).epsilon(1e-8));
    }

    fs::remove(path);
}

TEST_CASE("McapFileReader: topic resolution")
{
    const auto path = tempPath("hdmapping_reader_topics.mcap");
    const auto points = makeReaderPoints(10, 42.0);

    rosbags::McapWriterOptions write_options;
    write_options.lidar_topic = "/custom/points";
    write_options.imu_topic = "/custom/imu";
    write_options.sn_topic = "/custom/sn";
    {
        rosbags::McapFileWriter writer(path, write_options);
        REQUIRE(writer.isOpen());
        writer.writePointCloud(static_cast<uint64_t>(points.front().timestamp * 1e9), points);
    }

    SUBCASE("auto-detection finds the single channel of each type")
    {
        rosbags::McapFileReader reader(path);
        REQUIRE_MESSAGE(reader.isOpen(), reader.error());
        CHECK(reader.topics().lidar == "/custom/points");
        CHECK(reader.topics().imu == "/custom/imu");
        CHECK(reader.topics().sn == "/custom/sn");
        CHECK(reader.channels().size() == 3);
    }

    SUBCASE("an explicit topic is honored")
    {
        rosbags::McapReaderOptions options;
        options.lidar_topic = "/custom/points";
        rosbags::McapFileReader reader(path, options);
        REQUIRE_MESSAGE(reader.isOpen(), reader.error());
        CHECK(reader.topics().lidar == "/custom/points");
    }

    SUBCASE("a topic that is not in the file is an error, and channels stay listable")
    {
        rosbags::McapReaderOptions options;
        options.lidar_topic = "/does/not/exist";
        rosbags::McapFileReader reader(path, options);
        CHECK_FALSE(reader.isOpen());
        CHECK(reader.error().find("/does/not/exist") != std::string::npos);
        CHECK(reader.channels().size() == 3); // --list still works on an unresolvable file
    }

    fs::remove(path);
}

TEST_CASE("McapFileReader: the same reader can be read twice")
{
    // mcap_to_laz relies on this: one pass buffers the IMU stream, a second
    // streams the point clouds.
    const auto path = tempPath("hdmapping_reader_two_pass.mcap");
    const auto points = makeReaderPoints(50, 100.0);
    const auto samples = makeImuSamples(10, 100.0);

    {
        rosbags::McapFileWriter writer(path);
        REQUIRE(writer.isOpen());
        writer.writePointCloud(static_cast<uint64_t>(points.front().timestamp * 1e9), points);
        writer.writeImu(samples);
    }

    rosbags::McapFileReader reader(path);
    REQUIRE_MESSAGE(reader.isOpen(), reader.error());

    size_t imu_count = 0;
    rosbags::McapFileReader::Callbacks imu_pass;
    imu_pass.onImu = [&](const rosbags::McapImuSample&) { ++imu_count; };
    REQUIRE(reader.read(imu_pass));

    size_t point_count = 0;
    rosbags::McapFileReader::Callbacks lidar_pass;
    lidar_pass.onPointCloud = [&](uint64_t, std::vector<rosbags::McapPoint>&& pts) { point_count += pts.size(); };
    REQUIRE(reader.read(lidar_pass));

    CHECK(imu_count == samples.size());
    CHECK(point_count == points.size());

    fs::remove(path);
}

TEST_CASE("MandeyeSessionWriter: LAZ stores unscaled gps_time, zero lidar id and intensity")
{
    const auto dir = tempPath("hdmapping_session_laz");
    fs::remove_all(dir);

    const auto points = makeReaderPoints(200, 1234.5);
    {
        rosbags::MandeyeSessionWriter writer(dir);
        REQUIRE_MESSAGE(writer.isOpen(), writer.error());
        REQUIRE(writer.beginChunk(0));
        REQUIRE(writer.addPoints(points));
        REQUIRE(writer.endChunk());
        CHECK(writer.pointsWritten() == points.size());
    }

    const auto laz = dir / "lidar0000.laz";
    REQUIRE(fs::exists(laz));

    const auto decoded = readLaz(laz);
    REQUIRE(decoded.size() == points.size());
    for (size_t i = 0; i < points.size(); ++i)
    {
        // 0.1 mm coordinate scale, so agreement is absolute to within one step,
        // not relative -- points near the origin have no significant digits to
        // measure a relative tolerance against.
        CHECK(std::abs(decoded[i].x - points[i].x) <= 1e-4);
        CHECK(std::abs(decoded[i].y - points[i].y) <= 1e-4);
        CHECK(std::abs(decoded[i].z - points[i].z) <= 1e-4);

        // The unit contract: load_point_cloud() reads p.timestamp = gps_time
        // directly, so a 1e9 factor here would silently break every consumer.
        // LAS point format 1 stores gps_time as a full float64, so this is exact.
        CHECK(decoded[i].gps_time == points[i].timestamp);
        CHECK(decoded[i].gps_time < 1e6);

        // Single-lidar sessions: the lidar id load_point_cloud() reads out of
        // user_data is always 0, whatever laser_id the bag carried. A non-zero id
        // with no calibration entry would make it drop the points.
        CHECK(decoded[i].user_data == 0);
        CHECK(decoded[i].intensity == static_cast<uint16_t>(points[i].intensity));
    }

    fs::remove_all(dir);
}

TEST_CASE("MandeyeSessionWriter: IMU csv carries the columns load_imu() requires")
{
    const auto dir = tempPath("hdmapping_session_csv");
    fs::remove_all(dir);

    const auto samples = makeImuSamples(5, 10.0);
    {
        rosbags::MandeyeSessionWriter writer(dir);
        REQUIRE_MESSAGE(writer.isOpen(), writer.error());
        REQUIRE(writer.writeImuChunk(3, samples));
        CHECK(writer.imuSamplesWritten() == samples.size());
    }

    const auto csv = dir / "imu0003.csv";
    REQUIRE(fs::exists(csv));

    const auto lines = readLines(csv);
    REQUIRE(lines.size() == samples.size() + 1);
    CHECK(lines.front() == "timestamp timestampUnix accX accY accZ gyroX gyroY gyroZ");

    for (size_t i = 0; i < samples.size(); ++i)
    {
        std::istringstream iss(lines[i + 1]);
        uint64_t ts = 0, ts_unix = 1;
        float acc_x = 0, acc_y = 0, acc_z = 0, gyro_x = 0, gyro_y = 0, gyro_z = 0;
        REQUIRE(bool(iss >> ts >> ts_unix >> acc_x >> acc_y >> acc_z >> gyro_x >> gyro_y >> gyro_z));

        // load_imu() divides the timestamp columns by 1e9.
        CHECK(static_cast<double>(ts) / 1e9 == doctest::Approx(samples[i].timestamp).epsilon(1e-9));
        CHECK(ts_unix == 0); // not carried by the bag format
        CHECK(acc_z == doctest::Approx(samples[i].acc_z));
        CHECK(gyro_x == doctest::Approx(samples[i].gyro_x));
        CHECK(gyro_z == doctest::Approx(samples[i].gyro_z));
    }

    fs::remove_all(dir);
}

TEST_CASE("MandeyeSessionWriter: bag -> session round trip preserves points per chunk")
{
    const auto path = tempPath("hdmapping_session_roundtrip.mcap");
    const auto dir = tempPath("hdmapping_session_roundtrip");
    fs::remove_all(dir);

    // Two clouds 30 s apart, so a 20 s chunk grid puts them in separate chunks.
    const auto cloud_a = makeReaderPoints(120, 2000.0);
    const auto cloud_b = makeReaderPoints(80, 2030.0);
    {
        rosbags::McapFileWriter writer(path);
        REQUIRE(writer.isOpen());
        writer.writePointCloud(static_cast<uint64_t>(cloud_a.front().timestamp * 1e9), cloud_a);
        writer.writePointCloud(static_cast<uint64_t>(cloud_b.front().timestamp * 1e9), cloud_b);
    }

    rosbags::McapFileReader reader(path);
    REQUIRE_MESSAGE(reader.isOpen(), reader.error());

    rosbags::MandeyeSessionWriter writer(dir);
    REQUIRE_MESSAGE(writer.isOpen(), writer.error());

    int chunk = 0;
    rosbags::McapFileReader::Callbacks callbacks;
    callbacks.onPointCloud = [&](uint64_t, std::vector<rosbags::McapPoint>&& pts)
    {
        REQUIRE(writer.beginChunk(chunk++));
        REQUIRE(writer.addPoints(pts));
    };
    REQUIRE(reader.read(callbacks));
    REQUIRE(writer.endChunk());

    REQUIRE(chunk == 2);
    const auto chunk_a = readLaz(dir / "lidar0000.laz");
    const auto chunk_b = readLaz(dir / "lidar0001.laz");
    CHECK(chunk_a.size() == cloud_a.size());
    CHECK(chunk_b.size() == cloud_b.size());
    CHECK(chunk_a.front().gps_time == cloud_a.front().timestamp);
    CHECK(chunk_b.front().gps_time == cloud_b.front().timestamp);

    fs::remove(path);
    fs::remove_all(dir);
}

// ---------------------------------------------------------------------------
// decodePc2: datatype awareness and forced layout presets
// ---------------------------------------------------------------------------

namespace
{

struct Pc2TestField
{
    std::string name;
    uint32_t offset;
    uint8_t datatype;
};

// Builds a sensor_msgs/msg/PointCloud2 CDR payload with an arbitrary field
// description and raw record blob, so a test can present layouts McapWriter
// itself never emits (foreign datatypes, padding gaps, malformed offsets).
std::vector<uint8_t> buildPc2Message(
    double stamp_s, const std::vector<Pc2TestField>& fields, uint32_t point_step, const std::vector<uint8_t>& raw,
    bool is_bigendian = false)
{
    CdrWriter w;
    const auto ns = static_cast<uint64_t>(stamp_s * 1e9);
    w.write_i32(static_cast<int32_t>(ns / 1000000000ULL));
    w.write_u32(static_cast<uint32_t>(ns % 1000000000ULL));
    w.write_string("test_frame");

    w.write_u32(1); // height
    w.write_u32(point_step ? static_cast<uint32_t>(raw.size() / point_step) : 0); // width

    w.write_u32(static_cast<uint32_t>(fields.size()));
    for (const auto& f : fields)
    {
        w.write_string(f.name);
        w.write_u32(f.offset);
        w.write_u8(f.datatype);
        w.write_u32(1); // count
    }

    w.write_bool(is_bigendian);
    w.write_u32(point_step);
    w.write_u32(static_cast<uint32_t>(raw.size())); // row_step
    w.write_u32(static_cast<uint32_t>(raw.size())); // data length
    w.write_raw(raw.data(), raw.size());
    w.write_bool(true); // is_dense
    return w.data();
}

template<typename T>
void poke(std::vector<uint8_t>& buffer, size_t offset, T value)
{
    REQUIRE(offset + sizeof(T) <= buffer.size());
    std::memcpy(buffer.data() + offset, &value, sizeof(T));
}

constexpr uint8_t PF_UINT8 = 2;
constexpr uint8_t PF_UINT16 = 4;
constexpr uint8_t PF_UINT32 = 6;
constexpr uint8_t PF_FLOAT32 = 7;
constexpr uint8_t PF_FLOAT64 = 8;

// The record layout of a real HesaiLidar_ROS_2.0 /hesai/pandar message, as
// measured from a recording: a 4-byte pad after z, so intensity sits at 16 and
// point_step is 48 -- not the packing PointCloudLayout::Hesai writes.
std::vector<uint8_t> buildRealHesaiMessage(double stamp_s, size_t n)
{
    constexpr uint32_t kStep = 48;
    std::vector<uint8_t> raw(kStep * n, 0);
    for (size_t i = 0; i < n; ++i)
    {
        const size_t base = i * kStep;
        poke<float>(raw, base + 0, 1.0f + static_cast<float>(i));
        poke<float>(raw, base + 4, 2.0f + static_cast<float>(i));
        poke<float>(raw, base + 8, 3.0f + static_cast<float>(i));
        poke<float>(raw, base + 16, static_cast<float>(100 + i));
        poke<double>(raw, base + 24, stamp_s + static_cast<double>(i) * 1e-6);
        poke<uint16_t>(raw, base + 32, static_cast<uint16_t>(i % 32));
    }

    const std::vector<Pc2TestField> fields = {
        {"x", 0, PF_FLOAT32},
        {"y", 4, PF_FLOAT32},
        {"z", 8, PF_FLOAT32},
        {"intensity", 16, PF_FLOAT32},
        {"timestamp", 24, PF_FLOAT64},
        {"ring", 32, PF_UINT16},
    };
    return buildPc2Message(stamp_s, fields, kStep, raw);
}

} // namespace

TEST_CASE("decodePc2: real Hesai Pandar layout, auto and forced preset agree")
{
    const double t0 = 1720513004.923256397;
    const auto msg = buildRealHesaiMessage(t0, 5);

    std::string auto_warning;
    const auto by_auto = rosbags::decodePc2(msg.data(), msg.size(), rosbags::Pc2Preset::Auto, &auto_warning);

    std::string preset_warning;
    const auto by_preset = rosbags::decodePc2(msg.data(), msg.size(), rosbags::Pc2Preset::Hesai, &preset_warning);

    CHECK(auto_warning.empty());
    CHECK_MESSAGE(preset_warning.empty(), preset_warning); // preset matches the file exactly
    REQUIRE(by_auto.size() == 5);
    REQUIRE(by_preset.size() == 5);

    for (size_t i = 0; i < by_auto.size(); ++i)
    {
        CHECK(by_auto[i].x == doctest::Approx(1.0 + static_cast<double>(i)));
        CHECK(by_auto[i].y == doctest::Approx(2.0 + static_cast<double>(i)));
        CHECK(by_auto[i].z == doctest::Approx(3.0 + static_cast<double>(i)));
        CHECK(by_auto[i].intensity == doctest::Approx(100.0 + static_cast<double>(i)));
        CHECK(by_auto[i].ring == static_cast<uint16_t>(i % 32));
        // Absolute f64 timestamp, so exact -- and nowhere near the message stamp
        // offset an intensity@12 misread would produce.
        CHECK(by_auto[i].timestamp == t0 + static_cast<double>(i) * 1e-6);

        CHECK(by_preset[i].x == by_auto[i].x);
        CHECK(by_preset[i].intensity == by_auto[i].intensity);
        CHECK(by_preset[i].ring == by_auto[i].ring);
        CHECK(by_preset[i].timestamp == by_auto[i].timestamp);
    }
}

TEST_CASE("decodePc2: a preset that disagrees with the file is reported, not silent")
{
    const auto msg = buildRealHesaiMessage(1720513004.5, 4);

    std::string warning;
    const auto points = rosbags::decodePc2(msg.data(), msg.size(), rosbags::Pc2Preset::Ouster, &warning);

    // Decoding still proceeds under the forced layout -- the point is that the
    // caller is told the layout does not match.
    CHECK(points.size() == 4);
    REQUIRE_FALSE(warning.empty());
    CHECK(warning.find("ouster") != std::string::npos);
    CHECK(warning.find("ring") != std::string::npos);
}

TEST_CASE("decodePc2: honors each field's datatype rather than assuming float32")
{
    // float64 coordinates and uint8 intensity/ring: every field is a different
    // width from what McapWriter emits, so a fixed-width decoder reads garbage.
    constexpr uint32_t kStep = 27;
    std::vector<uint8_t> raw(kStep * 3, 0);
    for (size_t i = 0; i < 3; ++i)
    {
        const size_t base = i * kStep;
        poke<double>(raw, base + 0, -10.5 - static_cast<double>(i));
        poke<double>(raw, base + 8, 20.25 + static_cast<double>(i));
        poke<double>(raw, base + 16, 0.125 * static_cast<double>(i));
        poke<uint8_t>(raw, base + 24, static_cast<uint8_t>(200 + i));
        poke<uint16_t>(raw, base + 25, static_cast<uint16_t>(500 + i));
    }

    const std::vector<Pc2TestField> fields = {
        {"x", 0, PF_FLOAT64},
        {"y", 8, PF_FLOAT64},
        {"z", 16, PF_FLOAT64},
        {"intensity", 24, PF_UINT8},
        {"ring", 25, PF_UINT16},
    };
    const auto msg = buildPc2Message(1000.0, fields, kStep, raw);

    std::string warning;
    const auto points = rosbags::decodePc2(msg.data(), msg.size(), rosbags::Pc2Preset::Auto, &warning);

    CHECK(warning.empty());
    REQUIRE(points.size() == 3);
    for (size_t i = 0; i < points.size(); ++i)
    {
        CHECK(points[i].x == doctest::Approx(-10.5 - static_cast<double>(i)));
        CHECK(points[i].y == doctest::Approx(20.25 + static_cast<double>(i)));
        CHECK(points[i].z == doctest::Approx(0.125 * static_cast<double>(i)));
        CHECK(points[i].intensity == doctest::Approx(200.0 + static_cast<double>(i)));
        CHECK(points[i].ring == static_cast<uint16_t>(500 + i));
        CHECK(points[i].timestamp == doctest::Approx(1000.0)); // no time field -> message stamp
    }
}

TEST_CASE("decodePc2: integer 't' is nanoseconds, float 't' is seconds, both relative")
{
    constexpr uint32_t kStep = 16;

    SUBCASE("uint32 t (ouster convention)")
    {
        std::vector<uint8_t> raw(kStep, 0);
        poke<float>(raw, 0, 1.0f);
        poke<float>(raw, 4, 2.0f);
        poke<float>(raw, 8, 3.0f);
        poke<uint32_t>(raw, 12, 250000000u); // 0.25 s in ns
        const auto msg = buildPc2Message(
            500.0, {{"x", 0, PF_FLOAT32}, {"y", 4, PF_FLOAT32}, {"z", 8, PF_FLOAT32}, {"t", 12, PF_UINT32}}, kStep, raw);

        const auto points = rosbags::decodePc2(msg.data(), msg.size());
        REQUIRE(points.size() == 1);
        CHECK(points[0].timestamp == doctest::Approx(500.25));
    }

    SUBCASE("float32 t is already seconds")
    {
        std::vector<uint8_t> raw(kStep, 0);
        poke<float>(raw, 0, 1.0f);
        poke<float>(raw, 4, 2.0f);
        poke<float>(raw, 8, 3.0f);
        poke<float>(raw, 12, 0.25f);
        const auto msg = buildPc2Message(
            500.0, {{"x", 0, PF_FLOAT32}, {"y", 4, PF_FLOAT32}, {"z", 8, PF_FLOAT32}, {"t", 12, PF_FLOAT32}}, kStep, raw);

        const auto points = rosbags::decodePc2(msg.data(), msg.size());
        REQUIRE(points.size() == 1);
        CHECK(points[0].timestamp == doctest::Approx(500.25));
    }
}

TEST_CASE("decodePc2: intensity is optional")
{
    // Many public datasets publish geometry only. These used to decode to nothing.
    constexpr uint32_t kStep = 12;
    std::vector<uint8_t> raw(kStep * 2, 0);
    poke<float>(raw, 0, 7.0f);
    poke<float>(raw, 4, 8.0f);
    poke<float>(raw, 8, 9.0f);

    const auto msg = buildPc2Message(42.0, {{"x", 0, PF_FLOAT32}, {"y", 4, PF_FLOAT32}, {"z", 8, PF_FLOAT32}}, kStep, raw);

    std::string warning;
    const auto points = rosbags::decodePc2(msg.data(), msg.size(), rosbags::Pc2Preset::Auto, &warning);

    CHECK(warning.empty());
    REQUIRE(points.size() == 2);
    CHECK(points[0].x == doctest::Approx(7.0));
    CHECK(points[0].intensity == 0.0f);
    CHECK(points[0].timestamp == doctest::Approx(42.0));
}

TEST_CASE("decodePc2: malformed clouds are refused with a reason")
{
    const std::vector<Pc2TestField> xyz = {{"x", 0, PF_FLOAT32}, {"y", 4, PF_FLOAT32}, {"z", 8, PF_FLOAT32}};

    SUBCASE("big-endian is refused rather than misread")
    {
        const auto msg = buildPc2Message(1.0, xyz, 12, std::vector<uint8_t>(12, 0), /*is_bigendian=*/true);
        std::string warning;
        CHECK(rosbags::decodePc2(msg.data(), msg.size(), rosbags::Pc2Preset::Auto, &warning).empty());
        CHECK(warning.find("big-endian") != std::string::npos);
    }

    SUBCASE("zero point_step")
    {
        const auto msg = buildPc2Message(1.0, xyz, 0, std::vector<uint8_t>(12, 0));
        std::string warning;
        CHECK(rosbags::decodePc2(msg.data(), msg.size(), rosbags::Pc2Preset::Auto, &warning).empty());
        CHECK(warning.find("point_step") != std::string::npos);
    }

    SUBCASE("no x/y/z")
    {
        const auto msg = buildPc2Message(1.0, {{"intensity", 0, PF_FLOAT32}}, 4, std::vector<uint8_t>(4, 0));
        std::string warning;
        CHECK(rosbags::decodePc2(msg.data(), msg.size(), rosbags::Pc2Preset::Auto, &warning).empty());
        CHECK(warning.find("x/y/z") != std::string::npos);
    }

    SUBCASE("a field reaching past point_step would read into the next record")
    {
        // z occupies bytes 8..12 but point_step is only 8.
        const auto msg = buildPc2Message(1.0, xyz, 8, std::vector<uint8_t>(16, 0));
        std::string warning;
        CHECK(rosbags::decodePc2(msg.data(), msg.size(), rosbags::Pc2Preset::Auto, &warning).empty());
        CHECK(warning.find("past point_step") != std::string::npos);
    }
}

TEST_CASE("decodePc2: point timestamps come from the payload header stamp")
{
    // The contract mcap_to_laz's chunk binning depends on: a relative per-point
    // time is measured from the PointCloud2 header stamp inside the payload.
    // McapFileWriter derives that stamp from its own timestamp_ns argument, so the
    // message is hand-built here to set the two independently.
    constexpr double kSensorTime = 563.706196350;
    constexpr uint32_t kStep = 20;

    std::vector<uint8_t> raw(kStep * 3, 0);
    for (size_t i = 0; i < 3; ++i)
    {
        const size_t base = i * kStep;
        poke<float>(raw, base + 0, 1.0f + static_cast<float>(i));
        poke<float>(raw, base + 4, 2.0f);
        poke<float>(raw, base + 8, 3.0f);
        poke<double>(raw, base + 12, static_cast<double>(i) * 0.01); // "time", relative seconds
    }
    const auto msg = buildPc2Message(
        kSensorTime,
        {{"x", 0, PF_FLOAT32}, {"y", 4, PF_FLOAT32}, {"z", 8, PF_FLOAT32}, {"time", 12, PF_FLOAT64}},
        kStep,
        raw);

    const auto points = rosbags::decodePc2(msg.data(), msg.size());
    REQUIRE(points.size() == 3);
    for (size_t i = 0; i < points.size(); ++i)
        CHECK(points[i].timestamp == doctest::Approx(kSensorTime + static_cast<double>(i) * 0.01).epsilon(1e-12));
}

TEST_CASE("decodePc2: an absolute per-point timestamp ignores the header stamp entirely")
{
    // A Hesai `timestamp` field is already absolute, so it must be used as-is and
    // never added to the header stamp. The two are set far apart here so a stray
    // offset could not hide in the noise.
    constexpr double kHeaderStamp = 1720513004.0;
    constexpr double kPointTime = 563.706196350;
    constexpr uint32_t kStep = 20;

    std::vector<uint8_t> raw(kStep, 0);
    poke<float>(raw, 0, 1.0f);
    poke<float>(raw, 4, 2.0f);
    poke<float>(raw, 8, 3.0f);
    poke<double>(raw, 12, kPointTime);

    const auto msg = buildPc2Message(
        kHeaderStamp,
        {{"x", 0, PF_FLOAT32}, {"y", 4, PF_FLOAT32}, {"z", 8, PF_FLOAT32}, {"timestamp", 12, PF_FLOAT64}},
        kStep,
        raw);

    const auto points = rosbags::decodePc2(msg.data(), msg.size());
    REQUIRE(points.size() == 1);
    CHECK(points[0].timestamp == kPointTime);
}
