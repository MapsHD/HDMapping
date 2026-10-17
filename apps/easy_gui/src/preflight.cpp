#include "preflight.h"

#include <lidar_odometry_utils.h>

#include <laszip/laszip_api.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>

namespace fs = std::filesystem;

namespace easy_gui
{
    namespace
    {
        std::string lower(std::string s)
        {
            std::transform(
                s.begin(),
                s.end(),
                s.begin(),
                [](unsigned char c)
                {
                    return static_cast<char>(std::tolower(c));
                });
            return s;
        }

        std::string formatBytes(uint64_t bytes)
        {
            std::ostringstream out;
            out.precision(1);
            out << std::fixed << static_cast<double>(bytes) / 1e9 << " GB";
            return out.str();
        }

        std::string names(const std::vector<std::string>& list, size_t max_shown = 5)
        {
            std::string out;
            for (size_t i = 0; i < list.size() && i < max_shown; i++)
                out += (i ? ", " : "") + list[i];
            if (list.size() > max_shown)
                out += " and " + std::to_string(list.size() - max_shown) + " more";
            return out;
        }

        // Last 4 characters of the file stem, as load_data's get4index.
        std::string index4(const fs::path& p)
        {
            const auto s = p.stem().string();
            return s.size() > 4 ? s.substr(s.size() - 4) : s;
        }

        struct LazSummary
        {
            bool readable = false;
            uint64_t points = 0;
            double first_time = std::numeric_limits<double>::max();
            double last_time = std::numeric_limits<double>::lowest();
            std::set<int> lidar_ids;
        };

        // Header + a sample of points from the start and the end of the file: enough to check the lidar id and the
        // time range without decompressing whole files.
        LazSummary summarizeLaz(const fs::path& file)
        {
            constexpr uint64_t kSample = 20000;
            LazSummary s;
            laszip_POINTER reader = nullptr;
            if (laszip_create(&reader))
                return s;

            laszip_BOOL compressed = 0;
            laszip_header* header = nullptr;
            laszip_point* point = nullptr;
            if (laszip_open_reader(reader, file.string().c_str(), &compressed) || laszip_get_header_pointer(reader, &header) ||
                laszip_get_point_pointer(reader, &point))
            {
                laszip_destroy(reader);
                return s;
            }

            s.points = header->number_of_point_records ? header->number_of_point_records : header->extended_number_of_point_records;
            s.readable = true;

            auto sample = [&](uint64_t begin, uint64_t end)
            {
                for (uint64_t i = begin; i < end; i++)
                {
                    if (laszip_read_point(reader))
                    {
                        s.readable = false;
                        return;
                    }
                    s.lidar_ids.insert(point->user_data);
                    if (point->gps_time != 0.0)
                    {
                        s.first_time = std::min(s.first_time, point->gps_time);
                        s.last_time = std::max(s.last_time, point->gps_time);
                    }
                }
            };

            sample(0, std::min(kSample, s.points));
            if (s.readable && s.points > kSample)
            {
                const uint64_t tail_begin = std::max(kSample, s.points - std::min<uint64_t>(1000, s.points));
                if (laszip_seek_point(reader, static_cast<laszip_I64>(tail_begin)))
                    s.readable = false;
                else
                    sample(tail_begin, s.points);
            }

            laszip_close_reader(reader);
            laszip_destroy(reader);
            return s;
        }

        // IMU timestamps (ns in the file) of the first and last sample, in seconds. Handles the named-column format
        // and the legacy headerless one, like load_imu().
        bool imuTimeRange(const fs::path& first_file, const fs::path& last_file, double& t_first, double& t_last)
        {
            auto split = [](const std::string& line)
            {
                std::vector<std::string> out;
                std::string token;
                for (char c : line)
                {
                    if (c == ' ' || c == ',' || c == '\t' || c == '\r')
                    {
                        if (!token.empty())
                            out.push_back(token);
                        token.clear();
                    }
                    else
                        token += c;
                }
                if (!token.empty())
                    out.push_back(token);
                return out;
            };

            auto timestampColumn = [&](const fs::path& file, size_t& column, bool& has_header) -> bool
            {
                std::ifstream in(file);
                std::string line;
                if (!std::getline(in, line))
                    return false;
                const auto cols = split(line);
                has_header = false;
                column = 0;
                for (size_t i = 0; i < cols.size(); i++)
                    if (cols[i] == "timestamp")
                    {
                        has_header = true;
                        column = i;
                    }
                return true;
            };

            auto parseTime = [&](const std::string& line, size_t column, double& t) -> bool
            {
                const auto cols = split(line);
                if (column >= cols.size())
                    return false;
                try
                {
                    t = std::stod(cols[column]) / 1e9;
                } catch (...)
                {
                    return false;
                }
                return t > 0.0;
            };

            size_t column = 0;
            bool has_header = false;
            if (!timestampColumn(first_file, column, has_header))
                return false;

            {
                std::ifstream in(first_file);
                std::string line;
                if (has_header)
                    std::getline(in, line);
                bool found = false;
                while (!found && std::getline(in, line))
                    found = parseTime(line, column, t_first);
                if (!found)
                    return false;
            }
            {
                size_t last_column = 0;
                bool last_has_header = false;
                if (!timestampColumn(last_file, last_column, last_has_header))
                    return false;
                std::ifstream in(last_file);
                std::string line;
                std::string last_good;
                double t = 0.0;
                bool first = true;
                while (std::getline(in, line))
                {
                    if (first && last_has_header)
                    {
                        first = false;
                        continue;
                    }
                    first = false;
                    if (parseTime(line, last_column, t))
                        t_last = t;
                }
            }
            return t_last > t_first;
        }
    } // namespace

    bool isSameOrInside(const fs::path& inner, const fs::path& outer)
    {
        std::error_code ec;
        const fs::path a = fs::weakly_canonical(fs::absolute(inner, ec), ec).lexically_normal();
        const fs::path b = fs::weakly_canonical(fs::absolute(outer, ec), ec).lexically_normal();
        auto ai = a.begin();
        for (auto bi = b.begin(); bi != b.end(); ++bi, ++ai)
        {
            if (bi->empty()) // trailing separator
                continue;
            if (ai == a.end() || *ai != *bi)
                return false;
        }
        return true;
    }

    uint64_t estimateWorkBytes(uint64_t total_points)
    {
        return total_points * 120;
    }

    uint64_t estimateResultBytes(uint64_t raw_laz_bytes)
    {
        // Measured on a 117 M-point forest walk (734 MB raw LAZ): session 0.82x + point cloud 1.29x (with near and gap
        // points) + trajectory CSV 0.03x = 2.14x; 2.25x leaves some margin (the near-point share depends on the site).
        return raw_laz_bytes * 9 / 4;
    }

    RecordingFiles classifyRecordingFiles(const fs::path& recording)
    {
        RecordingFiles files;
        std::vector<fs::path> entries;
        std::error_code ec;
        for (const auto& entry : fs::directory_iterator(recording, ec))
            if (!entry.is_directory(ec))
                entries.push_back(entry.path());
        std::sort(entries.begin(), entries.end());

        for (const auto& p : entries)
        {
            const std::string ext = lower(p.extension().string());
            if (ext == ".csv")
                files.csv_files.push_back(p);
            else if (ext == ".laz" || ext == ".las")
                files.laz_files.push_back(p);
            else if (ext == ".json")
            {
                const std::string stem = p.stem().string();
                if (!stem.starts_with("status") && !stem.starts_with("cam0") && !stem.starts_with("cam1") && files.calibration_file.empty())
                    files.calibration_file = p;
            }
            else if (ext == ".mjc" && files.calibration_file.empty())
                files.calibration_file = p;
            else if (ext == ".sn" && files.sn_file.empty())
                files.sn_file = p;
        }
        files.pairs = std::min(files.laz_files.size(), files.csv_files.size());
        return files;
    }

    PreflightResult runPreflight(const fs::path& recording, const fs::path& output_folder)
    {
        PreflightResult r;
        auto add = [&r](Severity s, std::string m)
        {
            r.findings.push_back({ s, std::move(m) });
        };

        std::error_code ec;
        if (recording.empty() || !fs::is_directory(recording, ec))
        {
            add(Severity::Error, "The recording folder '" + recording.string() + "' does not exist or is not a folder.");
            return r;
        }

        r.files = classifyRecordingFiles(recording);
        const auto& f = r.files;

        if (f.laz_files.empty() || f.csv_files.empty())
        {
            add(Severity::Error,
                std::string("This folder does not look like a Mandeye recording: ") +
                    (f.laz_files.empty() ? "there are no point cloud files (lidar*.laz)" : "there are no IMU files (imu*.csv)") +
                    ". Choose the folder the scanner wrote, for example 'continousScanning_0012'.");
            return r;
        }

        // --- pairing (by position, as load_data does)
        if (f.laz_files.size() != f.csv_files.size())
        {
            const auto& longer = f.laz_files.size() > f.csv_files.size() ? f.laz_files : f.csv_files;
            std::vector<std::string> skipped;
            for (size_t i = f.pairs; i < longer.size(); i++)
                skipped.push_back(longer[i].filename().string());
            add(Severity::Warning,
                "There are " + std::to_string(f.laz_files.size()) + " point cloud files but " + std::to_string(f.csv_files.size()) +
                    " IMU files. The unmatched file(s) at the end will not be processed: " + names(skipped) +
                    ". This is normal when the scanner stopped between two files.");
        }
        std::vector<std::string> misaligned;
        for (size_t i = 0; i < f.pairs; i++)
            if (index4(f.laz_files[i]) != index4(f.csv_files[i]))
                misaligned.push_back(f.laz_files[i].filename().string() + " / " + f.csv_files[i].filename().string());
        if (!misaligned.empty())
            add(Severity::Warning,
                "Some file numbers do not match (" + names(misaligned, 3) +
                    "). Files are used in order, so a missing file in the middle of the recording can leave a gap in the result.");

        // --- point cloud files: readable, one lidar, time range
        std::vector<std::string> unreadable;
        std::set<int> lidar_ids;
        double lidar_first = std::numeric_limits<double>::max();
        double lidar_last = std::numeric_limits<double>::lowest();
        for (size_t i = 0; i < f.pairs; i++)
        {
            const auto s = summarizeLaz(f.laz_files[i]);
            if (!s.readable)
            {
                unreadable.push_back(f.laz_files[i].filename().string());
                continue;
            }
            r.total_points += s.points;
            r.raw_laz_bytes += fs::file_size(f.laz_files[i], ec);
            lidar_ids.insert(s.lidar_ids.begin(), s.lidar_ids.end());
            lidar_first = std::min(lidar_first, s.first_time);
            lidar_last = std::max(lidar_last, s.last_time);
        }
        if (!unreadable.empty())
            add(Severity::Error,
                "These point cloud files cannot be read (damaged, or not fully copied from the scanner): " + names(unreadable) +
                    ". Copy the recording again, or remove the damaged files at the end of the recording.");
        if (lidar_ids.size() > 1)
        {
            std::string ids;
            for (int id : lidar_ids)
                ids += (ids.empty() ? "" : ", ") + std::to_string(id);
            add(Severity::Error,
                "This recording contains data from more than one lidar (ids " + ids +
                    "). Easy GUI supports single-lidar recordings only; use the standard HDMapping apps for this one.");
        }
        else if (lidar_ids.size() == 1)
            r.lidar_id = *lidar_ids.begin();

        // --- calibration (mirrors load_data's checks, which otherwise end in std::exit)
        const auto id_to_sn = MLvxCalib::GetIdToSnMapping(f.sn_file.string());
        if (!f.calibration_file.empty())
        {
            const auto calibration = MLvxCalib::GetCalibrationFromFile(f.calibration_file.string());
            const std::string imu_sn = MLvxCalib::GetImuSnToUse(f.calibration_file.string());
            if (calibration.empty())
                add(Severity::Warning,
                    "The calibration file '" + f.calibration_file.filename().string() + "' could not be read and will be ignored.");
            else if (calibration.size() > 1)
                add(Severity::Error,
                    "The calibration file '" + f.calibration_file.filename().string() +
                        "' describes more than one lidar. Easy GUI supports single-lidar recordings only.");
            else
            {
                for (const auto& [id, sn] : id_to_sn)
                    if (!calibration.contains(sn))
                        add(Severity::Error,
                            "The calibration file '" + f.calibration_file.filename().string() +
                                "' belongs to a different scanner (this recording was made with lidar " + sn + ").");
                if (!calibration.contains(imu_sn))
                    add(Severity::Error,
                        "The calibration file '" + f.calibration_file.filename().string() + "' does not name a valid IMU ('" + imu_sn +
                            "' is not one of its sensors).");
            }
        }
        if (id_to_sn.size() > 1)
            add(Severity::Error,
                "The scanner serial-number file lists more than one lidar. Easy GUI supports single-lidar recordings only.");

        // --- IMU vs lidar clocks
        double imu_first = 0.0, imu_last = 0.0;
        if (!imuTimeRange(f.csv_files.front(), f.csv_files[f.pairs - 1], imu_first, imu_last))
            add(Severity::Error,
                "The IMU files could not be read (no valid timestamps in '" + f.csv_files.front().filename().string() + "').");
        else
        {
            r.duration_s = imu_last - imu_first;
            if (lidar_last > lidar_first)
            {
                const double overlap = std::min(imu_last, lidar_last) - std::max(imu_first, lidar_first);
                if (overlap <= 0.0)
                    add(Severity::Error,
                        "The IMU and point cloud timestamps do not overlap, so they cannot be combined. The recording is probably "
                        "damaged.");
            }
            if (r.duration_s < 20.0)
                add(Severity::Warning,
                    "The recording is very short (" + std::to_string(static_cast<int>(r.duration_s)) +
                        " s). Lidar odometry needs some movement; results may be poor.");
        }

        // --- output folder
        r.estimated_work_bytes = estimateWorkBytes(r.total_points);
        r.estimated_result_bytes = estimateResultBytes(r.raw_laz_bytes);
        r.estimated_disk_bytes = r.estimated_work_bytes + r.estimated_result_bytes;
        const std::string need = "about " + formatBytes(r.estimated_disk_bytes) + " (" + formatBytes(r.estimated_work_bytes) +
            " temporary during processing + " + formatBytes(r.estimated_result_bytes) + " of results)";
        std::string disk_summary;
        if (output_folder.empty())
            add(Severity::Error, "Choose an output folder.");
        else if (isSameOrInside(output_folder, recording))
            add(Severity::Error,
                "The output folder is inside the recording folder. Choose a folder elsewhere: Easy GUI never writes into the recording "
                "(it may be on the scanner's USB stick).");
        else
        {
            if (fs::exists(output_folder, ec) && !fs::is_empty(output_folder, ec))
                add(Severity::Error,
                    "The output folder '" + output_folder.string() + "' already contains files. Choose an empty or new folder.");

            fs::path existing = fs::absolute(output_folder, ec);
            while (!existing.empty() && !fs::exists(existing, ec) && existing != existing.parent_path())
                existing = existing.parent_path();
            const auto space = fs::space(existing, ec);
            if (ec)
                add(Severity::Warning, "Could not determine the free disk space at '" + existing.string() + "'.");
            else if (space.available < r.estimated_disk_bytes)
                add(Severity::Error,
                    "Not enough free disk space for the output folder: " + need + " needed, " + formatBytes(space.available) +
                        " free. Free some space or choose a folder on another disk.");
            else if (space.available < r.estimated_disk_bytes + r.estimated_disk_bytes / 2)
                add(Severity::Warning, "Free disk space is tight: " + need + " needed, " + formatBytes(space.available) + " free.");
            else
                disk_summary = "Disk space: " + need + " needed, " + formatBytes(space.available) + " free.";
        }
        if (!disk_summary.empty() && r.ok())
            r.findings.push_back({ Severity::Ok, disk_summary });

        if (r.ok())
        {
            std::ostringstream summary;
            summary.precision(1);
            summary << std::fixed << "Recording looks good: " << f.pairs << " file pairs, " << r.total_points / 1000000.0
                    << " million points, " << r.duration_s / 60.0 << " minutes.";
            r.findings.insert(r.findings.begin(), { Severity::Ok, summary.str() });
        }
        return r;
    }
} // namespace easy_gui
