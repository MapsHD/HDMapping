#include <doctest.h>

#include "export.h"
#include "forestry_preset.h"
#include "las14_writer.h"
#include "test_helpers.h"

#include <Core/export_laz.h>

#include <laszip/laszip_api.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>

using namespace easy_gui;
namespace fs = std::filesystem;

namespace
{
    struct ReadPoint
    {
        Eigen::Vector3d xyz;
        double gps_time;
        uint16_t intensity;
        uint16_t point_source_id;
        uint8_t user_data;
        float range_m;
        uint8_t near_added;
    };

    struct ReadFile
    {
        laszip_header header{};
        std::vector<std::string> extra_names;
        std::vector<int> extra_types;
        std::vector<int> extra_options;
        std::vector<ReadPoint> points;
    };

    ReadFile readLaz(const fs::path& file)
    {
        ReadFile r;
        laszip_POINTER reader = nullptr;
        REQUIRE(laszip_create(&reader) == 0);
        laszip_BOOL compressed = 0;
        REQUIRE(laszip_open_reader(reader, file.string().c_str(), &compressed) == 0);
        laszip_header* h = nullptr;
        laszip_get_header_pointer(reader, &h);
        r.header = *h;
        for (laszip_U32 v = 0; v < h->number_of_variable_length_records; v++)
        {
            const auto& vlr = h->vlrs[v];
            if (std::string(vlr.user_id) == "LASF_Spec" && vlr.record_id == 4)
                for (size_t k = 0; k + 192 <= vlr.record_length_after_header; k += 192)
                {
                    const laszip_U8* d = vlr.data + k;
                    r.extra_types.push_back(d[2]);
                    r.extra_options.push_back(d[3]);
                    r.extra_names.emplace_back(reinterpret_cast<const char*>(d + 4), strnlen(reinterpret_cast<const char*>(d + 4), 32));
                }
        }
        laszip_point* p = nullptr;
        laszip_get_point_pointer(reader, &p);
        const uint64_t n = h->number_of_point_records ? h->number_of_point_records : h->extended_number_of_point_records;
        for (uint64_t i = 0; i < n; i++)
        {
            REQUIRE(laszip_read_point(reader) == 0);
            ReadPoint q{};
            q.xyz = { h->x_offset + h->x_scale_factor * p->X,
                      h->y_offset + h->y_scale_factor * p->Y,
                      h->z_offset + h->z_scale_factor * p->Z };
            q.gps_time = p->gps_time;
            q.intensity = p->intensity;
            q.point_source_id = p->point_source_ID;
            q.user_data = p->user_data;
            if (p->num_extra_bytes >= 5)
            {
                std::memcpy(&q.range_m, p->extra_bytes, 4);
                q.near_added = p->extra_bytes[4];
            }
            r.points.push_back(q);
        }
        laszip_close_reader(reader);
        laszip_destroy(reader);
        return r;
    }

    struct RawPoint
    {
        Eigen::Vector3d xyz;
        double t;
        uint16_t intensity = 7;
    };

    //! Mandeye-style raw LAZ (LAS 1.2 PF1, lidar frame) with the given points.
    void writeRaw(const fs::path& file, const std::vector<RawPoint>& points, int lidar_id)
    {
        laszip_POINTER writer = nullptr;
        REQUIRE(laszip_create(&writer) == 0);
        laszip_header* header = nullptr;
        laszip_get_header_pointer(writer, &header);
        header->version_major = 1;
        header->version_minor = 2;
        header->point_data_format = 1;
        header->point_data_record_length = 28;
        header->x_scale_factor = header->y_scale_factor = header->z_scale_factor = 0.0001;
        REQUIRE(laszip_open_writer(writer, file.string().c_str(), 1) == 0);
        laszip_point* point = nullptr;
        laszip_get_point_pointer(writer, &point);
        for (const auto& p : points)
        {
            point->gps_time = p.t;
            point->user_data = static_cast<laszip_U8>(lidar_id);
            point->intensity = p.intensity;
            laszip_F64 xyz[3] = { p.xyz.x(), p.xyz.y(), p.xyz.z() };
            laszip_set_coordinates(writer, xyz);
            laszip_write_point(writer);
            laszip_update_inventory(writer);
        }
        laszip_close_writer(writer);
        laszip_destroy(writer);
    }

    void writePoseFile(const fs::path& file, const std::vector<Eigen::Affine3d>& poses, const std::vector<std::string>& names)
    {
        std::ofstream out(file);
        out << poses.size() << "\n" << std::setprecision(17);
        for (size_t i = 0; i < poses.size(); i++)
        {
            out << names[i] << "\n";
            const auto& m = poses[i].matrix();
            for (int r = 0; r < 4; r++)
                out << m(r, 0) << " " << m(r, 1) << " " << m(r, 2) << " " << m(r, 3) << "\n";
        }
    }

    //! A small synthetic step 1 result: kScans scans of kNodes trajectory nodes at 200 Hz along a curved path, written
    //! in exactly step 1's file formats (save_result in lidar_odometry.cpp), plus the matching raw recording.
    struct SyntheticSession
    {
        static constexpr int kScans = 2;
        static constexpr int kNodes = 40;
        static constexpr double kT0 = 1783328634.0;
        static constexpr int kLidarId = 3;

        fs::path session_file;
        RecordingFiles recording;
        std::vector<Eigen::Vector3d> sensor_points; // per scan_lio point, in the sensor frame (for range_m)
        size_t odometry_points = 0;

        static double nodeTime(int k)
        {
            return kT0 + 0.005 * k;
        }
        //! Global pose of node k (k counts across scans).
        static Eigen::Affine3d pose(int k)
        {
            Eigen::Affine3d m = Eigen::Affine3d::Identity();
            m.linear() = (Eigen::AngleAxisd(0.5 + 0.02 * k, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()))
                             .toRotationMatrix();
            m.translation() = Eigen::Vector3d(10.0 + 0.005 * k, 5.0 + 0.002 * k, 1.0);
            return m;
        }

        explicit SyntheticSession(const fs::path& root)
        {
            const fs::path session = root / "lio_session";
            const fs::path raw = root / "recording";
            fs::create_directories(session);
            fs::create_directories(raw);

            std::vector<Eigen::Affine3d> scan_poses;
            std::vector<std::string> names;
            nlohmann::json laz_names;
            for (int s = 0; s < kScans; s++)
            {
                const int first = s * kNodes;
                const Eigen::Affine3d m_pose = pose(first);
                scan_poses.push_back(m_pose);
                names.push_back("scan_lio_" + std::to_string(s) + ".laz");
                laz_names.push_back({ { "file_name", (session / names.back()).string() } });

                std::ofstream trj(session / ("trajectory_lio_" + std::to_string(s) + ".csv"));
                trj << "timestamp_nanoseconds pose00 pose01 pose02 pose03 pose10 pose11 pose12 pose13 pose20 pose21 pose22 pose23 "
                       "timestampUnix_nanoseconds om_rad fi_rad ka_rad\n";
                std::vector<Eigen::Vector3d> local_points;
                std::vector<unsigned short> intensity;
                std::vector<double> timestamps;
                for (int j = 0; j < kNodes; j++)
                {
                    const Eigen::Affine3d local = m_pose.inverse() * pose(first + j);
                    trj << std::setprecision(20) << nodeTime(first + j) * 1e9 << " " << std::setprecision(10);
                    for (int r = 0; r < 3; r++)
                        for (int c = 0; c < 4; c++)
                            trj << local(r, c) << " ";
                    trj << std::setprecision(20) << std::floor(nodeTime(first + j) * 1000) << " 0.01 0.02 0.03 \n";

                    // two odometry points per node, 1 ms after it: one near, one far
                    for (const Eigen::Vector3d p_sensor : { Eigen::Vector3d(1.2, 0.3, 0.4), Eigen::Vector3d(-8.0, 3.0, 2.0) })
                    {
                        local_points.push_back(local * p_sensor);
                        intensity.push_back(static_cast<unsigned short>(20 + j));
                        timestamps.push_back(nodeTime(first + j) + 0.001); // exportLaz writes seconds x 1e9, as step 1
                        sensor_points.push_back(p_sensor);
                    }
                }
                REQUIRE(exportLaz((session / names.back()).string(), local_points, intensity, timestamps));
                odometry_points += local_points.size();
            }
            writePoseFile(session / "session_poses.mrp", scan_poses, names);
            writePoseFile(session / "session_ini_poses.mri", scan_poses, names);
            std::string error;
            REQUIRE(writeStep1ParameterFile(session / "HDMapping_params_test.toml", error));

            nlohmann::json settings = { { "offset_x", 0.0 },
                                        { "offset_y", 0.0 },
                                        { "offset_z", 0.0 },
                                        { "folder_name", session.string() },
                                        { "out_folder_name", session.string() },
                                        { "poses_file_name", (session / "session_poses.mrp").string() },
                                        { "initial_poses_file_name", (session / "session_ini_poses.mri").string() },
                                        { "out_poses_file_name", (session / "session_poses.mrp").string() } };
            session_file = session / "session.mjs";
            std::ofstream(session_file) << nlohmann::json{ { "Session Settings", settings }, { "laz_file_names", laz_names } }.dump(2);

            recording.laz_files = { raw / "lidar0000.laz", raw / "lidar0001.laz" };
            recording.csv_files = { raw / "imu0000.csv", raw / "imu0001.csv" };
            recording.sn_file = raw / "lidar0000.sn";
            recording.pairs = 2;
            easy_gui_test::writeText(recording.sn_file, std::to_string(kLidarId) + " ARMCP1U0038132\n");
        }
    };

    bool near(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double tolerance = 0.0002)
    {
        return (a - b).norm() <= tolerance;
    }
} // namespace

TEST_CASE("Las14Writer writes LAS 1.4 point format 6 with named, unscaled extra bytes")
{
    easy_gui_test::TempDir tmp;
    const fs::path file = tmp.path / "out.laz";
    Las14Writer writer;
    std::string error;
    REQUIRE(writer.open(
        file.string(),
        { { ExtraBytesAttribute::Type::F32, "range_m", "range" }, { ExtraBytesAttribute::Type::U8, "near_added", "flag" } },
        Eigen::Vector3d(100.0, 200.0, 0.0),
        0.0001,
        "test",
        error));
#pragma pack(push, 1)
    struct
    {
        float r;
        uint8_t f;
    } extra{ 12.5f, 1 };
#pragma pack(pop)
    Las14Point p;
    p.position = { 101.23456, 198.5, -3.25 };
    p.gps_time = 1783328634.123456;
    p.intensity = 77;
    p.point_source_id = 4321;
    p.user_data = 2;
    p.extra_bytes = reinterpret_cast<const uint8_t*>(&extra);
    REQUIRE(writer.write(p, error));
    p.extra_bytes = nullptr; // all-zero extra bytes
    p.position = { 99.0, 201.0, 5.0 };
    REQUIRE(writer.write(p, error));
    REQUIRE(writer.close(error));

    const ReadFile r = readLaz(file);
    CHECK(r.header.version_major == 1);
    CHECK(r.header.version_minor == 4);
    CHECK(r.header.point_data_format == 6);
    CHECK(r.header.point_data_record_length == 35);
    CHECK((r.header.global_encoding & 1) == 0); // gps_time is not adjusted GPS time
    CHECK((r.header.global_encoding & 16) == 16); // WKT bit, required for PF6
    CHECK(r.header.extended_number_of_point_records == 2);
    CHECK(r.header.number_of_point_records == 0); // legacy counter stays 0 for PF6
    CHECK(r.header.min_x == doctest::Approx(99.0));
    CHECK(r.header.max_z == doctest::Approx(5.0));
    REQUIRE(r.extra_names.size() == 2);
    CHECK(r.extra_names[0] == "range_m");
    CHECK(r.extra_names[1] == "near_added");
    CHECK(r.extra_types[0] == 9); // float
    CHECK(r.extra_types[1] == 1); // unsigned char
    CHECK(r.extra_options[0] == 0); // no scale: readers return the raw float / uint8
    CHECK(r.extra_options[1] == 0);
    REQUIRE(r.points.size() == 2);
    CHECK(near(r.points[0].xyz, { 101.2346, 198.5, -3.25 }, 1e-6));
    CHECK(r.points[0].gps_time == 1783328634.123456);
    CHECK(r.points[0].intensity == 77);
    CHECK(r.points[0].point_source_id == 4321);
    CHECK(r.points[0].user_data == 2);
    CHECK(r.points[0].range_m == 12.5f);
    CHECK(r.points[0].near_added == 1);
    CHECK(r.points[1].range_m == 0.0f);
    CHECK(r.points[1].near_added == 0);
}

TEST_CASE("exportSession: step 1 points as step 2 places them, near points with step 1's transform chain")
{
    easy_gui_test::TempDir tmp;
    SyntheticSession s(tmp.path);
    using S = SyntheticSession;

    // Raw points. Node 5 lies in scan 0, node 47 in scan 1 (7th node of that scan).
    const double t5 = S::nodeTime(5) + 0.002;
    const double t47 = S::nodeTime(47) + 0.004;
    const Eigen::Vector3d near_a(0.5, 0.3, 0.2); // horizontal 0.58 m, 3D 0.62 m: near point
    const Eigen::Vector3d near_steep(0.6, 0.0, 1.5); // horizontal 0.6 m but 3D 1.62 m: still a near point
    const Eigen::Vector3d band(1.1, 0.0, 0.0); // step 1 exports this one itself
    std::vector<RawPoint> file0 = {
        { near_a, t5, 11 },
        { Eigen::Vector3d(0.2, 0.1, 0.0), t5 }, // 3D 0.22 m < near_min_range
        { Eigen::Vector3d::Zero(), t5 }, // no return
        { Eigen::Vector3d(0.5, 0.0, 0.0), 0.0 }, // time 0
        { Eigen::Vector3d(3.0, 0.0, 0.0), t5 }, // step 1's point
        { band, t5 },
        { near_a, S::kT0 - 1.0 }, // before the trajectory: no pose within 10 ms
    };
    std::vector<RawPoint> file1 = {
        { near_steep, t47, 12 },
        { near_a, S::nodeTime(2 * S::kNodes - 1) + 0.02 }, // 20 ms after the last node
    };
    writeRaw(s.recording.laz_files[0], file0, S::kLidarId);
    writeRaw(s.recording.laz_files[1], file1, S::kLidarId);

    ExportRequest request;
    request.session_file = s.session_file;
    request.recording = s.recording;
    request.lidar_id = S::kLidarId;
    request.near_min_range_m = 0.4;
    request.laz_file = tmp.path / "out" / "point_cloud.laz";
    request.trajectory_file = tmp.path / "out" / "trajectory.csv";
    request.readme_file = tmp.path / "out" / "README.txt";

    std::atomic<bool> cancel{ false };
    double last_fraction = -1.0;
    bool monotonic = true;
    ExportStats stats;
    std::string error;
    REQUIRE_MESSAGE(
        exportSession(
            request,
            cancel,
            [&](double f, const std::string&)
            {
                monotonic = monotonic && f >= last_fraction;
                last_fraction = f;
            },
            stats,
            error),
        error);
    CHECK(monotonic);
    CHECK(last_fraction == 1.0);

    CHECK(stats.scans == 2);
    CHECK(stats.trajectory_nodes == 2 * S::kNodes);
    CHECK(stats.odometry_points == s.odometry_points);
    CHECK(stats.near_points == 2);
    CHECK(stats.check_band_points == 0);
    CHECK(stats.raw_points_read == file0.size() + file1.size());
    CHECK(stats.raw_no_return == 2);
    CHECK(stats.raw_near_outside_trajectory == 2);
    CHECK(stats.filter_threshold_xy_inner == 1.0);
    CHECK(stats.threshould_output_filter == 1.0); // Easy GUI's output setting, read back from the session

    const ReadFile r = readLaz(request.laz_file);
    REQUIRE(r.points.size() == s.odometry_points + 2);

    // odometry points: scan pose x local point, range from the sensor frame, ramp time in seconds
    for (size_t i = 0; i < s.odometry_points; i++)
    {
        const auto& p = r.points[i];
        const int scan = static_cast<int>(i / (2 * S::kNodes));
        const int node = scan * S::kNodes + static_cast<int>(i % (2 * S::kNodes)) / 2;
        CHECK(p.near_added == kFromOdometry);
        CHECK(p.point_source_id == scan);
        CHECK(p.user_data == S::kLidarId);
        CHECK(near(p.xyz, S::pose(node) * s.sensor_points[i]));
        CHECK(p.range_m == doctest::Approx(s.sensor_points[i].norm()).epsilon(1e-4));
        CHECK(p.gps_time == doctest::Approx(S::nodeTime(node) + 0.001).epsilon(1e-15));
    }

    // near points: pose of the last node before the raw time, raw time and intensity kept
    const auto& a = r.points[s.odometry_points];
    CHECK(a.near_added == kNearAdded);
    CHECK(near(a.xyz, S::pose(5) * near_a));
    CHECK(a.gps_time == t5);
    CHECK(a.intensity == 11);
    CHECK(a.point_source_id == 0);
    CHECK(a.range_m == doctest::Approx(near_a.norm()));
    const auto& b = r.points[s.odometry_points + 1];
    CHECK(b.near_added == kNearAdded);
    CHECK(near(b.xyz, S::pose(47) * near_steep));
    CHECK(b.point_source_id == 1);
    CHECK(b.range_m == doctest::Approx(near_steep.norm()));
    CHECK(stats.range_min_m == doctest::Approx(near_a.norm()).epsilon(1e-6));

    // trajectory: one row per node, global pose, Unix seconds
    std::ifstream csv(request.trajectory_file);
    std::string header, row;
    std::getline(csv, header);
    CHECK(header.starts_with("timestamp_unix_s,x,y,z,qw,qx,qy,qz"));
    int rows = 0;
    std::vector<double> row47;
    while (std::getline(csv, row))
    {
        if (rows == 47)
        {
            std::stringstream ss(row);
            std::string v;
            while (std::getline(ss, v, ','))
                row47.push_back(std::stod(v));
        }
        rows++;
    }
    CHECK(rows == 2 * S::kNodes);
    REQUIRE(row47.size() == 16);
    CHECK(row47[0] == doctest::Approx(S::nodeTime(47)).epsilon(1e-15));
    CHECK(near({ row47[1], row47[2], row47[3] }, S::pose(47).translation()));
    const Eigen::Quaterniond q(S::pose(47).rotation());
    CHECK(std::abs(std::abs(q.dot(Eigen::Quaterniond(row47[4], row47[5], row47[6], row47[7]))) - 1.0) < 1e-8);
    CHECK(row47[11] == 1); // scan_index

    std::ifstream readme(request.readme_file);
    const std::string text((std::istreambuf_iterator<char>(readme)), std::istreambuf_iterator<char>());
    CHECK(text.find("Unix epoch seconds") != std::string::npos);
    CHECK(text.find("at least 0.40 m") != std::string::npos);
    CHECK(text.find("near_added") != std::string::npos);

    SUBCASE("check-band test mode adds the 1.0-1.2 m raw points with flag 2")
    {
        request.check_band_outer_m = 1.2;
        REQUIRE(exportSession(
            request,
            cancel,
            [](double, const std::string&)
            {
            },
            stats,
            error));
        CHECK(stats.near_points == 2);
        CHECK(stats.check_band_points == 1);
        const ReadFile rb = readLaz(request.laz_file);
        REQUIRE(rb.points.size() == s.odometry_points + 3);
        const auto& c = rb.points[s.odometry_points + 1]; // raw file order: near_a, band, near_steep
        CHECK(c.near_added == kCheckBand);
        CHECK(near(c.xyz, S::pose(5) * band));
        CHECK(c.range_m == doctest::Approx(band.norm()));
    }

    SUBCASE("cancel stops the export")
    {
        cancel = true;
        CHECK_FALSE(exportSession(
            request,
            cancel,
            [](double, const std::string&)
            {
            },
            stats,
            error));
        CHECK(error.find("cancelled") != std::string::npos);
    }
}

TEST_CASE("exportSession refuses a session without its parameter file, in plain language")
{
    easy_gui_test::TempDir tmp;
    SyntheticSession s(tmp.path);
    fs::remove(s.session_file.parent_path() / "HDMapping_params_test.toml");
    ExportRequest request;
    request.session_file = s.session_file;
    request.recording = s.recording;
    request.laz_file = tmp.path / "point_cloud.laz";
    request.trajectory_file = tmp.path / "trajectory.csv";
    request.readme_file = tmp.path / "README.txt";
    std::atomic<bool> cancel{ false };
    ExportStats stats;
    std::string error;
    CHECK_FALSE(exportSession(
        request,
        cancel,
        [](double, const std::string&)
        {
        },
        stats,
        error));
    CHECK(error.find("settings file") != std::string::npos);
}
