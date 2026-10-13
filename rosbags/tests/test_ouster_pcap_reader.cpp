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

#include "OusterPcapReader.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{

constexpr uint64_t kInitId = 12345;
constexpr uint64_t kSerial = 992612000030;

// A 4-beam sensor, 4 columns per frame, 2 columns per packet. Beams look
// straight out at 0, 0, +30 and -30 degrees with no beam offset, so the SDK's
// XYZ model reduces to plain spherical coordinates: measurement id 0 points
// along +X, id 1 along -Y. The lidar sits 10 mm above the sensor origin; the
// IMU is turned 90 degrees about Z.
std::string metadataJson(const std::string& profile, const std::string& pixel_data_format = "")
{
    std::string json = R"({
        "sensor_info": {"prod_line": "OS-TEST", "prod_sn": ")" +
        std::to_string(kSerial) + R"(", "image_rev": "test", "initialization_id": )" + std::to_string(kInitId) + R"(},
        "config_params": {"udp_port_lidar": 7502, "udp_port_imu": 7503, "udp_profile_lidar": ")" +
        profile + R"(", "udp_profile_imu": "LEGACY"},
        "lidar_data_format": {"pixels_per_column": 4, "columns_per_packet": 2, "columns_per_frame": 4,
                              "udp_profile_lidar": ")" +
        profile + R"(", "header_type": "STANDARD"},
        "beam_intrinsics": {"beam_altitude_angles": [0, 0, 30, -30], "beam_azimuth_angles": [0, 0, 0, 0],
                            "beam_to_lidar_transform": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1],
                            "lidar_origin_to_beam_origin_mm": 0},
        "lidar_intrinsics": {"lidar_to_sensor_transform": [1,0,0,0, 0,1,0,0, 0,0,1,10, 0,0,0,1]},
        "imu_intrinsics": {"imu_to_sensor_transform": [0,-1,0,0, 1,0,0,0, 0,0,1,0, 0,0,0,1]})";
    if (!pixel_data_format.empty())
        json += R"(, "pixel_data_format": )" + pixel_data_format;
    return json + "}";
}

rosbags::OusterMetadata parseOrFail(const std::string& json)
{
    rosbags::OusterMetadata m;
    std::string error;
    const bool ok = rosbags::parseOusterMetadata(json, m, error);
    INFO(error);
    REQUIRE(ok);
    return m;
}

void putLe(std::vector<uint8_t>& out, size_t offset, uint64_t value, int bytes)
{
    for (int i = 0; i < bytes; ++i)
        out[offset + i] = static_cast<uint8_t>(value >> (8 * i));
}

void putBe16(std::vector<uint8_t>& out, size_t offset, uint16_t value)
{
    out[offset] = static_cast<uint8_t>(value >> 8);
    out[offset + 1] = static_cast<uint8_t>(value);
}

struct Column
{
    uint64_t timestamp_ns;
    uint16_t measurement_id;
    bool valid;
    uint32_t range_mm[4];
    uint8_t reflectivity[4];
    uint16_t rgb[4][3]; // only written into 16-byte (RGB16) pixels
};

// One lidar packet for the sensor above: RNG19_RFL8_SIG16_NIR16 (12-byte pixels)
// or, with pixel_bytes = 16, RNG19_RFL8_SIG16_NIR16_RGB16.
std::vector<uint8_t> lidarPacket(const Column (&columns)[2], uint64_t init_id = kInitId, size_t pixel_bytes = 12)
{
    const size_t kPixel = pixel_bytes;
    const size_t kColumn = 12 + 4 * kPixel;
    std::vector<uint8_t> p(32 + 2 * kColumn + 32, 0);
    putLe(p, 0, 1, 2); // packet type
    putLe(p, 4, init_id, 3);
    putLe(p, 7, kSerial, 5);
    for (size_t c = 0; c < 2; ++c)
    {
        const size_t col = 32 + c * kColumn;
        putLe(p, col, columns[c].timestamp_ns, 8);
        putLe(p, col + 8, columns[c].measurement_id, 2);
        putLe(p, col + 10, columns[c].valid ? 1 : 0, 2);
        for (size_t u = 0; u < 4; ++u)
        {
            const size_t px = col + 12 + u * kPixel;
            // Bit 19 set on purpose: reserved/bloom bits must not leak into the range.
            putLe(p, px, columns[c].range_mm[u] | (1u << 19) | (uint32_t{columns[c].reflectivity[u]} << 24), 4);
            putLe(p, px + 4, 0xBEEF, 2); // signal
            if (pixel_bytes >= 14)
            {
                for (size_t k = 0; k < 3; ++k)
                    putLe(p, px + 8 + 2 * k, columns[c].rgb[u][k], 2);
            }
        }
    }
    return p;
}

std::vector<uint8_t> legacyImuPacket(uint64_t gyro_ts_ns, const float (&acc_g)[3], const float (&gyro_dps)[3])
{
    std::vector<uint8_t> p(48, 0);
    putLe(p, 0, 1, 8); // system timestamp: a different clock, must be ignored
    putLe(p, 8, gyro_ts_ns - 1000, 8);
    putLe(p, 16, gyro_ts_ns, 8);
    for (int i = 0; i < 3; ++i)
    {
        std::memcpy(&p[24 + 4 * i], &acc_g[i], 4);
        std::memcpy(&p[36 + 4 * i], &gyro_dps[i], 4);
    }
    return p;
}

struct ImuMeasurement
{
    uint64_t timestamp_ns;
    bool valid;
    float acc_ms2[3];
    float gyro_rads[3];
};

// ACCEL32_GYRO32_NMEA with two measurements per packet.
std::vector<uint8_t> accel32Gyro32NmeaImuPacket(const ImuMeasurement (&measurements)[2], uint64_t init_id = kInitId)
{
    std::vector<uint8_t> p(32 + 100 + 2 * 36 + 32, 0);
    putLe(p, 0, 2, 2); // packet type
    putLe(p, 4, init_id, 3);
    putLe(p, 7, kSerial, 5);
    for (size_t i = 0; i < 2; ++i)
    {
        const size_t m = 32 + 100 + i * 36;
        putLe(p, m, measurements[i].timestamp_ns, 8);
        putLe(p, m + 8, 31 + 32 * i, 2);
        putLe(p, m + 10, measurements[i].valid ? 1 : 0, 2);
        for (int k = 0; k < 3; ++k)
        {
            std::memcpy(&p[m + 12 + 4 * k], &measurements[i].acc_ms2[k], 4);
            std::memcpy(&p[m + 24 + 4 * k], &measurements[i].gyro_rads[k], 4);
        }
    }
    return p;
}

// Wraps a UDP payload in Ethernet/IPv4/UDP, split into IPv4 fragments carrying at
// most `fragment_bytes` (a multiple of 8) of IP payload each; 0 = unfragmented.
std::vector<std::vector<uint8_t>> udpFrames(const std::vector<uint8_t>& payload, uint16_t dport, uint16_t ip_id, size_t fragment_bytes = 0)
{
    std::vector<uint8_t> datagram(8 + payload.size());
    putBe16(datagram, 0, 7502);
    putBe16(datagram, 2, dport);
    putBe16(datagram, 4, static_cast<uint16_t>(datagram.size()));
    std::copy(payload.begin(), payload.end(), datagram.begin() + 8);

    if (fragment_bytes == 0)
        fragment_bytes = datagram.size();
    std::vector<std::vector<uint8_t>> frames;
    for (size_t offset = 0; offset < datagram.size(); offset += fragment_bytes)
    {
        const size_t n = std::min(fragment_bytes, datagram.size() - offset);
        const bool more = offset + n < datagram.size();
        std::vector<uint8_t> f(14 + 20 + n, 0);
        putBe16(f, 12, 0x0800);
        f[14] = 0x45;
        putBe16(f, 16, static_cast<uint16_t>(20 + n));
        putBe16(f, 18, ip_id);
        putBe16(f, 20, static_cast<uint16_t>((more ? 0x2000 : 0) | (offset / 8)));
        f[22] = 64;
        f[23] = 17;
        const uint8_t src[4] = {169, 254, 0, 2};
        const uint8_t dst[4] = {169, 254, 0, 1};
        std::memcpy(&f[26], src, 4);
        std::memcpy(&f[30], dst, 4);
        std::copy(datagram.begin() + offset, datagram.begin() + offset + n, f.begin() + 34);
        frames.push_back(std::move(f));
    }
    return frames;
}

void writePcap(const fs::path& path, const std::vector<std::vector<uint8_t>>& frames)
{
    std::vector<uint8_t> out(24, 0);
    putLe(out, 0, 0xA1B2C3D4, 4);
    putLe(out, 4, 2, 2);
    putLe(out, 6, 4, 2);
    putLe(out, 16, 65535, 4);
    putLe(out, 20, 1, 4); // Ethernet
    for (size_t i = 0; i < frames.size(); ++i)
    {
        std::vector<uint8_t> record(16, 0);
        putLe(record, 0, 1000 + i, 4);
        putLe(record, 8, frames[i].size(), 4);
        putLe(record, 12, frames[i].size(), 4);
        out.insert(out.end(), record.begin(), record.end());
        out.insert(out.end(), frames[i].begin(), frames[i].end());
    }
    std::ofstream(path, std::ios::binary).write(reinterpret_cast<const char*>(out.data()), static_cast<std::streamsize>(out.size()));
}

struct ReadResult
{
    std::vector<rosbags::McapPoint> points;
    std::vector<std::array<uint16_t, 3>> rgb;
    std::vector<rosbags::McapImuSample> imu;
    rosbags::OusterPcapStats stats;
};

ReadResult readAll(const fs::path& path, const rosbags::OusterMetadata& meta, const rosbags::OusterPcapReaderOptions& options = {})
{
    rosbags::OusterPcapReader reader(path, meta, options);
    INFO(reader.error());
    REQUIRE(reader.isOpen());
    ReadResult r;
    rosbags::OusterPcapReader::Callbacks callbacks;
    callbacks.onPoints = [&](std::vector<rosbags::McapPoint>&& points, std::vector<std::array<uint16_t, 3>>&& rgb)
    {
        if (!rgb.empty())
            REQUIRE(rgb.size() == points.size());
        r.points.insert(r.points.end(), points.begin(), points.end());
        r.rgb.insert(r.rgb.end(), rgb.begin(), rgb.end());
    };
    callbacks.onImu = [&](const rosbags::McapImuSample& s)
    {
        r.imu.push_back(s);
    };
    REQUIRE(reader.read(callbacks));
    r.stats = reader.stats();
    return r;
}

const Column kColumns[2] = {
    {5'000'000'000, 0, true, {1000, 0, 2000, 3000}, {10, 0, 30, 40}, {{100, 200, 300}, {1, 2, 3}, {400, 500, 600}, {0xFFFF, 0xFFFF, 0xFFFF}}},
    {5'000'100'000, 1, true, {0, 2000, 0, 0}, {0, 20, 0, 0}, {{0, 0, 0}, {7, 8, 9}, {0, 0, 0}, {0, 0, 0}}},
};

} // namespace

TEST_CASE("parseOusterMetadata: profile table, pixel_data_format, and refusals")
{
    SUBCASE("a known profile name gives the RNG19 word-0 layout")
    {
        const auto m = parseOrFail(metadataJson("RNG19_RFL8_SIG16_NIR16"));
        CHECK(m.pixel_bytes == 12);
        CHECK(m.range_start_bit == 0);
        CHECK(m.range_width == 19);
        CHECK(m.reflectivity_start_bit == 24);
        CHECK(m.reflectivity_width == 8);
        CHECK(m.lidarPacketSize() == 32 + 2 * (12 + 4 * 12) + 32);
        CHECK(m.initialization_id == kInitId);
        CHECK(m.rgb_width == 0);
        CHECK(!m.rgb_float16);
    }
    SUBCASE("the RGB16 profile name brings red, green, blue in words 2 and 3")
    {
        const auto m = parseOrFail(metadataJson("RNG19_RFL8_SIG16_NIR16_RGB16"));
        CHECK(m.pixel_bytes == 16);
        CHECK(m.rgb_width == 16);
        CHECK(m.rgb_float16);
        CHECK(m.rgb_start_bit == std::array<int, 3>{64, 80, 96});
    }
    SUBCASE("pixel_data_format describes a profile no table knows")
    {
        const auto m = parseOrFail(metadataJson(
            "SOME_FUTURE_PROFILE",
            R"({"pixel_bytes": 20, "fields": [
                {"field_name": "range_capped_mm_0", "start_bit": 0, "width": 19, "type": "uint"},
                {"field_name": "bloom_0", "start_bit": 19, "width": 1, "type": "bool"},
                {"field_name": "reflectivity_0", "start_bit": 24, "width": 8, "type": "uint"},
                {"field_name": "range_capped_mm_1", "start_bit": 32, "width": 19, "type": "uint"},
                {"field_name": "rgb_r1", "start_bit": 64, "width": 16, "type": "uint"},
                {"field_name": "rgb_r", "start_bit": 112, "width": 16, "type": "uint"},
                {"field_name": "rgb_g", "start_bit": 128, "width": 16, "type": "uint"},
                {"field_name": "rgb_b", "start_bit": 144, "width": 16, "type": "uint"}]})"));
        CHECK(m.pixel_bytes == 20);
        CHECK(m.range_start_bit == 0); // the first return, not range_capped_mm_1
        CHECK(m.reflectivity_start_bit == 24);
        CHECK(m.rgb_width == 16);
        CHECK(m.rgb_float16);
        CHECK(m.rgb_start_bit == std::array<int, 3>{112, 128, 144}); // rgb_r, not rgb_r1
    }
    SUBCASE("an unknown profile without pixel_data_format is refused by name")
    {
        rosbags::OusterMetadata m;
        std::string error;
        CHECK_FALSE(rosbags::parseOusterMetadata(metadataJson("SOME_FUTURE_PROFILE"), m, error));
        CHECK(error.find("SOME_FUTURE_PROFILE") != std::string::npos);
    }
    SUBCASE("LEGACY lidar packets are refused")
    {
        rosbags::OusterMetadata m;
        std::string error;
        CHECK_FALSE(rosbags::parseOusterMetadata(metadataJson("LEGACY"), m, error));
    }
}

TEST_CASE("OusterPcapReader: points follow the SDK XYZ model, from fragmented datagrams in any order")
{
    const auto meta = parseOrFail(metadataJson("RNG19_RFL8_SIG16_NIR16"));
    const fs::path path = fs::temp_directory_path() / "hdmapping_ouster_reader_points.pcap";

    auto frames = udpFrames(lidarPacket(kColumns), 7502, 1, 64);
    REQUIRE(frames.size() > 2);
    std::swap(frames.front(), frames.back()); // last fragment first
    writePcap(path, frames);

    const auto r = readAll(path, meta);
    CHECK(r.rgb.empty()); // no colour in this profile
    CHECK(r.stats.reassembled_datagrams == 1);
    CHECK(r.stats.dropped_fragments == 0);
    CHECK(r.stats.lidar_packets == 1);
    REQUIRE(r.points.size() == 4); // zero ranges dropped

    const double s30 = std::sin(std::numbers::pi / 6), c30 = std::cos(std::numbers::pi / 6);
    struct Expected
    {
        double x, y, z;
        float intensity;
        uint16_t ring;
        double t;
    };
    const Expected expected[4] = {
        {1.0, 0.0, 0.010, 10, 0, 5.0}, // id 0, level beam: +X, lifted by lidar_to_sensor
        {2.0 * c30, 0.0, 2.0 * s30 + 0.010, 30, 2, 5.0},
        {3.0 * c30, 0.0, -3.0 * s30 + 0.010, 40, 3, 5.0},
        {0.0, -2.0, 0.010, 20, 1, 5.0001}, // id 1 of 4: a quarter turn clockwise, -Y
    };
    for (size_t i = 0; i < 4; ++i)
    {
        CAPTURE(i);
        CHECK(r.points[i].x == doctest::Approx(expected[i].x).epsilon(1e-6));
        CHECK(r.points[i].y == doctest::Approx(expected[i].y).epsilon(1e-6));
        CHECK(r.points[i].z == doctest::Approx(expected[i].z).epsilon(1e-6));
        CHECK(r.points[i].intensity == expected[i].intensity);
        CHECK(r.points[i].ring == expected[i].ring);
        CHECK(r.points[i].timestamp == doctest::Approx(expected[i].t).epsilon(1e-12));
    }

    SUBCASE("column_step keeps only every Nth measurement id")
    {
        rosbags::OusterPcapReaderOptions options;
        options.column_step = 2;
        const auto thinned = readAll(path, meta, options);
        REQUIRE(thinned.points.size() == 3);
        for (const auto& p : thinned.points)
            CHECK(p.timestamp == doctest::Approx(5.0));
    }
    fs::remove(path);
}

TEST_CASE("OusterPcapReader: an RGB profile hands each point's colour words alongside it")
{
    const auto meta = parseOrFail(metadataJson("RNG19_RFL8_SIG16_NIR16_RGB16"));
    const fs::path path = fs::temp_directory_path() / "hdmapping_ouster_reader_rgb.pcap";
    writePcap(path, udpFrames(lidarPacket(kColumns, kInitId, 16), 7502, 1));

    const auto r = readAll(path, meta);
    REQUIRE(r.points.size() == 4);
    REQUIRE(r.rgb.size() == 4);
    // Same point order as the 12-byte profile; zero-range pixels drop their colour too.
    const std::array<uint16_t, 3> expected[4] = {{100, 200, 300}, {400, 500, 600}, {0xFFFF, 0xFFFF, 0xFFFF}, {7, 8, 9}};
    for (size_t i = 0; i < 4; ++i)
    {
        CAPTURE(i);
        CHECK(r.rgb[i] == expected[i]);
    }
    fs::remove(path);
}

TEST_CASE("OusterPcapReader: LEGACY IMU is rotated into the sensor frame, deg/s -> rad/s, g kept")
{
    const auto meta = parseOrFail(metadataJson("RNG19_RFL8_SIG16_NIR16"));
    const fs::path path = fs::temp_directory_path() / "hdmapping_ouster_reader_imu.pcap";
    writePcap(path, udpFrames(legacyImuPacket(7'000'000'000, {1.0f, 0.0f, 0.0f}, {90.0f, 0.0f, 0.0f}), 7503, 2));

    const auto r = readAll(path, meta);
    REQUIRE(r.imu.size() == 1);
    const auto& s = r.imu.front();
    CHECK(s.timestamp == doctest::Approx(7.0)); // gyro read time, not the system timestamp
    CHECK(s.acc_x == doctest::Approx(0.0));
    CHECK(s.acc_y == doctest::Approx(1.0)); // +X of the IMU is +Y of the sensor
    CHECK(s.acc_z == doctest::Approx(0.0));
    CHECK(s.gyro_x == doctest::Approx(0.0));
    CHECK(s.gyro_y == doctest::Approx(std::numbers::pi / 2));
    CHECK(s.gyro_z == doctest::Approx(0.0));
    fs::remove(path);
}

TEST_CASE("OusterPcapReader: ACCEL32_GYRO32_NMEA IMU is rotated into the sensor frame, m/s^2 -> g, invalid samples dropped")
{
    std::string json = metadataJson("RNG19_RFL8_SIG16_NIR16");
    json.replace(json.find(R"("udp_profile_imu": "LEGACY")"), 27, R"("udp_profile_imu": "ACCEL32_GYRO32_NMEA")");
    json.insert(json.rfind('}'), R"(, "imu_data_format": {"imu_measurements_per_packet": 2})");
    const auto meta = parseOrFail(json);
    CHECK(meta.imu_measurements_per_packet == 2);
    CHECK(meta.imuPacketSize() == 32 + 100 + 2 * 36 + 32);

    const fs::path path = fs::temp_directory_path() / "hdmapping_ouster_reader_imu_nmea.pcap";
    const ImuMeasurement measurements[2] = {
        {7'000'000'000, true, {9.80665f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f}},
        {7'001'562'500, false, {1.0f, 2.0f, 3.0f}, {1.0f, 2.0f, 3.0f}},
    };
    std::vector<std::vector<uint8_t>> frames = udpFrames(accel32Gyro32NmeaImuPacket(measurements), 7503, 1);
    const auto foreign = udpFrames(accel32Gyro32NmeaImuPacket(measurements, kInitId + 1), 7503, 2);
    frames.insert(frames.end(), foreign.begin(), foreign.end());
    writePcap(path, frames);

    const auto r = readAll(path, meta);
    CHECK(r.stats.imu_packets == 1);
    CHECK(r.stats.foreign_packets == 1);
    REQUIRE(r.imu.size() == 1);
    const auto& s = r.imu.front();
    CHECK(s.timestamp == doctest::Approx(7.0));
    CHECK(s.acc_x == doctest::Approx(0.0));
    CHECK(s.acc_y == doctest::Approx(1.0)); // +X of the IMU is +Y of the sensor
    CHECK(s.acc_z == doctest::Approx(0.0));
    CHECK(s.gyro_x == doctest::Approx(0.0));
    CHECK(s.gyro_y == doctest::Approx(0.5));
    CHECK(s.gyro_z == doctest::Approx(0.0));
    fs::remove(path);
}

TEST_CASE("OusterPcapReader: foreign, mis-sized and unrelated packets are counted, not decoded")
{
    const auto meta = parseOrFail(metadataJson("RNG19_RFL8_SIG16_NIR16"));
    const fs::path path = fs::temp_directory_path() / "hdmapping_ouster_reader_skips.pcap";

    std::vector<std::vector<uint8_t>> frames;
    auto add = [&](std::vector<std::vector<uint8_t>> more)
    {
        frames.insert(frames.end(), more.begin(), more.end());
    };
    add(udpFrames(lidarPacket(kColumns, kInitId + 1), 7502, 1)); // sensor re-initialized since the metadata
    auto truncated = lidarPacket(kColumns);
    truncated.resize(truncated.size() - 4);
    add(udpFrames(truncated, 7502, 2));
    add(udpFrames(std::vector<uint8_t>(16, 0), 9999, 3));
    add({udpFrames(lidarPacket(kColumns), 7502, 4, 64).front()}); // a fragment whose siblings never arrive
    writePcap(path, frames);

    const auto r = readAll(path, meta);
    CHECK(r.points.empty());
    CHECK(r.stats.foreign_packets == 1);
    CHECK(r.stats.bad_size_packets == 1);
    CHECK(r.stats.other_packets == 1);
    CHECK(r.stats.dropped_fragments == 1);
    CHECK(r.stats.lidar_packets == 0);
    fs::remove(path);
}

TEST_CASE("OusterPcapReader: pcapng is refused with a conversion hint")
{
    const auto meta = parseOrFail(metadataJson("RNG19_RFL8_SIG16_NIR16"));
    const fs::path path = fs::temp_directory_path() / "hdmapping_ouster_reader.pcapng";
    std::vector<uint8_t> block(28, 0);
    putLe(block, 0, 0x0A0D0D0A, 4);
    std::ofstream(path, std::ios::binary).write(reinterpret_cast<const char*>(block.data()), static_cast<std::streamsize>(block.size()));

    rosbags::OusterPcapReader reader(path, meta);
    CHECK_FALSE(reader.isOpen());
    CHECK(reader.error().find("editcap") != std::string::npos);
    fs::remove(path);
}
