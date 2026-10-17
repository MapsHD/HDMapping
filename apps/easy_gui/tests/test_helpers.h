#pragma once

#include <laszip/laszip_api.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

namespace easy_gui_test
{
    //! Fresh empty directory under the system temp dir, removed again in the destructor.
    struct TempDir
    {
        std::filesystem::path path;
        TempDir()
        {
            static std::atomic<int> counter{ 0 };
            path = std::filesystem::temp_directory_path() /
                ("easy_gui_test_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + "_" +
                 std::to_string(counter++));
            std::filesystem::create_directories(path);
        }
        ~TempDir()
        {
            std::error_code ec;
            std::filesystem::remove_all(path, ec);
        }
    };

    //! Mandeye-style raw LAZ (LAS 1.2 PF1): `n` points starting at gps_time `t0` [s], 1 ms apart.
    inline bool writeRawLaz(const std::filesystem::path& file, int n, double t0, int lidar_id = 0)
    {
        laszip_POINTER writer = nullptr;
        if (laszip_create(&writer))
            return false;
        laszip_header* header = nullptr;
        laszip_get_header_pointer(writer, &header);
        header->version_major = 1;
        header->version_minor = 2;
        header->point_data_format = 1;
        header->point_data_record_length = 28;
        header->x_scale_factor = header->y_scale_factor = header->z_scale_factor = 0.0001;
        if (laszip_open_writer(writer, file.string().c_str(), 1))
            return false;
        laszip_point* point = nullptr;
        laszip_get_point_pointer(writer, &point);
        for (int i = 0; i < n; i++)
        {
            point->gps_time = t0 + i * 0.001;
            point->user_data = static_cast<laszip_U8>(lidar_id);
            point->intensity = 10;
            laszip_F64 xyz[3] = { 2.0 + i * 0.01, 1.0, 0.5 };
            laszip_set_coordinates(writer, xyz);
            laszip_write_point(writer);
            laszip_update_inventory(writer);
        }
        laszip_close_writer(writer);
        laszip_destroy(writer);
        return true;
    }

    //! Mandeye IMU csv with named columns, `n` samples at 200 Hz starting at `t0` [s] (written in ns).
    inline void writeImuCsv(const std::filesystem::path& file, int n, double t0)
    {
        std::ofstream out(file);
        out << "timestamp gyroX gyroY gyroZ accX accY accZ imuId timestampUnix\n";
        for (int i = 0; i < n; i++)
        {
            const auto ns = static_cast<long long>((t0 + i * 0.005) * 1e9);
            out << ns << " 0 0 0 0 0 1 0 " << ns / 1000000 << "\n";
        }
    }

    inline void writeText(const std::filesystem::path& file, const std::string& text)
    {
        std::ofstream out(file, std::ios::binary);
        out << text;
    }

    //! A plausible two-pair recording (plus an unmatched trailing lidar file, like real recordings often have),
    //! 60 s long so it does not trigger the short-recording warning.
    inline void writeRecording(const std::filesystem::path& dir, double t0 = 1783328629.0)
    {
        std::filesystem::create_directories(dir);
        writeRawLaz(dir / "lidar0000.laz", 2000, t0);
        writeImuCsv(dir / "imu0000.csv", 6000, t0);
        writeRawLaz(dir / "lidar0001.laz", 2000, t0 + 30.0);
        writeImuCsv(dir / "imu0001.csv", 6000, t0 + 30.0);
        writeRawLaz(dir / "lidar0002.laz", 100, t0 + 60.0);
        writeText(dir / "lidar0000.sn", "0 ARMCP1U0038132\n");
        writeText(dir / "lidar0001.sn", "0 ARMCP1U0038132\n");
        writeText(dir / "status0000.json", "{}");
    }
} // namespace easy_gui_test
