// Processed lidar_odometry_step_1 session (session.json) -> MCAP exporter.
//
// Unlike laz_to_mcap (which exports a *raw* mandeye recording, points still
// in each scan's own moving sensor frame), a session's point clouds are
// already motion-compensated: PointCloud::points_local is undistorted and
// expressed relative to that chunk's own first pose, and PointCloud::m_pose
// places the chunk in the map frame -- see core/include/Core/export_laz.h's
// save_all_to_las() for the same "m_pose * points_local[i]" composition.
// This tool writes those already-registered points straight into the map
// frame (matching apps/camera_lidar_trajectory_viewer/RosExport.h's
// "exportLidarUndistorted" convention: frame_id = map, no further motion
// compensation needed), plus a /tf stream of map -> lidar samples taken from
// each chunk's local_trajectory (falling back to one static-ish sample per
// chunk for older sessions saved without a trajectory_lio_*.csv).
//
// A session keeps no raw IMU samples (WorkerData::raw_imu_data only exists
// during the live lidar_odometry_step_1 run and isn't serialized), so /imu
// is optional and, if wanted, is re-read from the *original* mandeye
// recording directory via load_imu() -- the same function laz_to_mcap uses.
#include "McapWriter.h"
#include "lidar_odometry_utils.h"

#include <Core/session.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>

#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

namespace
{

    bool check_path_ext(const std::string& path, const char* ext)
    {
        return fs::path(path).extension() == ext;
    }

    std::string to_lower(std::string s)
    {
        std::transform(
            s.begin(),
            s.end(),
            s.begin(),
            [](unsigned char c)
            {
                return std::tolower(c);
            });
        return s;
    }

    // PointCloud::timestamps / LocalTrajectoryNode::timestamps.first are stored
    // in NANOSECONDS: lidar_odometry.cpp writes both the scan_lio_*.laz gps_time
    // field and the trajectory_lio_*.csv "timestamp_nanoseconds" column as
    // `seconds * 1e9`, and both are read back verbatim (no /1e9). McapPoint /
    // McapImuSample / McapTransform all expect absolute seconds, so every
    // session-sourced timestamp is converted here, once.
    constexpr double kNanosecondsToSeconds = 1e-9;

    // Session point clouds carry no per-point ring/laser_id, so PointCloud2's
    // Generic layout (the only one that needs them) still round-trips fine --
    // both fields just come out zero.
    std::vector<rosbags::McapPoint> to_mcap_points(const PointCloud& pc)
    {
        std::vector<rosbags::McapPoint> out;
        out.reserve(pc.points_local.size());
        for (size_t i = 0; i < pc.points_local.size(); ++i)
        {
            const double ts_ns = (i < pc.timestamps.size()) ? pc.timestamps[i] : 0.0;
            if (ts_ns == 0.0) // sentinel for "no timestamp", same convention as save_all_to_las's skip_ts_0
                continue;

            const Eigen::Vector3d world = pc.m_pose * pc.points_local[i];

            rosbags::McapPoint mp{};
            mp.x = static_cast<float>(world.x());
            mp.y = static_cast<float>(world.y());
            mp.z = static_cast<float>(world.z());
            mp.intensity = (i < pc.intensities.size()) ? static_cast<float>(pc.intensities[i]) : 0.0f;
            mp.timestamp = ts_ns * kNanosecondsToSeconds;
            out.push_back(mp);
        }
        return out;
    }

    void sort_points_by_timestamp(std::vector<rosbags::McapPoint>& points)
    {
        std::sort(
            points.begin(),
            points.end(),
            [](const auto& a, const auto& b)
            {
                return a.timestamp < b.timestamp;
            });
    }

    rosbags::McapTransform to_mcap_transform(double timestamp_s, const Eigen::Affine3d& T)
    {
        rosbags::McapTransform t{};
        t.timestamp = timestamp_s;
        t.tx = T.translation().x();
        t.ty = T.translation().y();
        t.tz = T.translation().z();
        Eigen::Quaterniond q(T.linear());
        q.normalize();
        t.qx = q.x();
        t.qy = q.y();
        t.qz = q.z();
        t.qw = q.w();
        return t;
    }

    // T_map_lidar per node = pc.m_pose * node.m_pose: local_trajectory poses are
    // stored relative to the chunk's own first pose (lidar_odometry.cpp writes
    // `intermediate_trajectory[0].inverse() * intermediate_trajectory[j]`), the
    // same convention points_local uses -- see the file header comment.
    std::vector<rosbags::McapTransform> to_mcap_transforms(const PointCloud& pc)
    {
        std::vector<rosbags::McapTransform> out;
        if (!pc.local_trajectory.empty())
        {
            out.reserve(pc.local_trajectory.size());
            for (const auto& node : pc.local_trajectory)
                out.push_back(to_mcap_transform(node.timestamps.first * kNanosecondsToSeconds, pc.m_pose * node.m_pose));
            return out;
        }

        // Older session without a trajectory_lio_*.csv: one sample for the whole
        // chunk, stamped at its first valid (non-sentinel) point timestamp.
        double ts_ns = 0.0;
        for (double t : pc.timestamps)
        {
            if (t != 0.0)
            {
                ts_ns = t;
                break;
            }
        }
        out.push_back(to_mcap_transform(ts_ns * kNanosecondsToSeconds, pc.m_pose));
        return out;
    }

    // Cuts a session chunk's points into one PointCloud2 message per 1/msg_hz
    // seconds, so the bag replays at a lidar-like rate instead of one huge
    // message per chunk. Mirrors laz_to_mcap.cpp's MessageSplitter exactly
    // (absolute time-bin grid, pending points carry over a chunk boundary);
    // duplicated rather than shared since that one buffers Point3Di and this one
    // already-converted McapPoint.
    class MessageSplitter
    {
    public:
        MessageSplitter(rosbags::McapFileWriter& writer, double msg_hz)
            : writer_(writer)
            , msg_hz_(msg_hz)
        {
        }

        // `points` must be sorted by timestamp, and successive calls must be in
        // timestamp order too (session chunks are processed in container order).
        void add(const std::vector<rosbags::McapPoint>& points)
        {
            if (msg_hz_ <= 0.0)
            {
                write(points);
                return;
            }
            for (const auto& p : points)
            {
                const int64_t bin = static_cast<int64_t>(std::floor(p.timestamp * msg_hz_));
                if (!pending_.empty() && bin != current_bin_)
                    flush();
                current_bin_ = bin;
                pending_.push_back(p);
            }
        }

        void flush()
        {
            write(pending_);
            pending_.clear();
        }

        size_t messages_written() const
        {
            return messages_;
        }

    private:
        void write(const std::vector<rosbags::McapPoint>& points)
        {
            if (points.empty())
                return;
            const uint64_t stamp_ns = static_cast<uint64_t>(points.front().timestamp * 1e9);
            writer_.writePointCloud(stamp_ns, points);
            ++messages_;
        }

        rosbags::McapFileWriter& writer_;
        double msg_hz_;
        std::vector<rosbags::McapPoint> pending_;
        int64_t current_bin_ = 0;
        size_t messages_ = 0;
    };

    // load_imu() reads a single sensor's stream out of an imuNNNN.csv (see its doc
    // comment in lidar_odometry_utils.h): on a rig with more than one IMU, rows
    // carry an optional "imuId" column and only rows matching this id are kept.
    // session_to_mcap has no per-sensor calibration to resolve which id is "the"
    // IMU (a session keeps no raw IMU/calibration provenance at all), so it
    // always reads id 0 and instead warns when a file actually contains more
    // than one id -- see distinct_imu_ids() below.
    constexpr int kImuIdToUse = 0;

    // Splits a line the same way load_imu()'s CSVFormat does (space/comma/tab
    // delimited), just for peeking at the header/imuId column below.
    std::vector<std::string> split_csv_line(const std::string& line)
    {
        std::vector<std::string> out;
        std::string cur;
        for (char c : line)
        {
            if (c == ' ' || c == ',' || c == '\t')
            {
                if (!cur.empty())
                {
                    out.push_back(cur);
                    cur.clear();
                }
            }
            else
                cur.push_back(c);
        }
        if (!cur.empty())
            out.push_back(cur);
        return out;
    }

    // Returns every distinct "imuId" value in a modern-format (named-column)
    // IMU csv, purely to warn when a file mixes more than one IMU. Empty for a
    // file with no imuId column (a single-IMU recording -- id 0 covers it, no
    // warning needed) or for the legacy headerless format load_imu() also
    // accepts (not inspected here; load_imu() itself still reads it correctly).
    std::set<int> distinct_imu_ids(const std::string& csv_path)
    {
        std::set<int> ids;
        std::ifstream file(csv_path);
        std::string header_line;
        if (!file.is_open() || !std::getline(file, header_line))
            return ids;

        const auto header = split_csv_line(header_line);
        const auto it = std::find(header.begin(), header.end(), "imuId");
        if (it == header.end())
            return ids;
        const auto imu_id_index = static_cast<size_t>(std::distance(header.begin(), it));

        std::string line;
        while (std::getline(file, line))
        {
            const auto row = split_csv_line(line);
            if (imu_id_index >= row.size())
                continue;
            try
            {
                ids.insert(std::stoi(row[imu_id_index]));
            } catch (const std::exception&)
            {
            }
        }
        return ids;
    }

    // Reads every imu*.csv in raw_dir (mandeye's imuNNNN.csv chunk convention)
    // and merges them into one timestamp-sorted stream, always using IMU id 0
    // (warning first if any file actually carries more than one IMU id).
    std::vector<rosbags::McapImuSample> load_all_imu(const fs::path& raw_dir)
    {
        std::vector<std::string> csvs;
        for (const auto& entry : fs::directory_iterator(raw_dir))
        {
            if (!entry.is_regular_file())
                continue;
            if (to_lower(entry.path().extension().string()) != ".csv")
                continue;
            if (!to_lower(entry.path().stem().string()).starts_with("imu"))
                continue;
            csvs.push_back(entry.path().string());
        }
        std::sort(csvs.begin(), csvs.end());

        std::set<int> all_ids;
        for (const auto& csv : csvs)
        {
            const auto ids = distinct_imu_ids(csv);
            all_ids.insert(ids.begin(), ids.end());
        }
        if (all_ids.size() > 1)
        {
            std::string ids_str;
            for (int id : all_ids)
                ids_str += (ids_str.empty() ? "" : ", ") + std::to_string(id);
            spdlog::warn(
                "{} carries more than one IMU (ids: {}) - session_to_mcap always reads id {}", raw_dir.string(), ids_str, kImuIdToUse);
        }

        std::vector<rosbags::McapImuSample> out;
        for (const auto& csv : csvs)
        {
            const auto imu_data = load_imu(csv, kImuIdToUse);
            for (const auto& [ts, gyr, acc] : imu_data)
            {
                rosbags::McapImuSample s{};
                s.timestamp = ts.first;
                s.gyro_x = gyr.x();
                s.gyro_y = gyr.y();
                s.gyro_z = gyr.z();
                s.acc_x = acc.x();
                s.acc_y = acc.y();
                s.acc_z = acc.z();
                out.push_back(s);
            }
        }
        std::sort(
            out.begin(),
            out.end(),
            [](const auto& a, const auto& b)
            {
                return a.timestamp < b.timestamp;
            });
        return out;
    }

    void print_usage(const char* argv0)
    {
        spdlog::error("Usage: {} <session.json> <output.mcap> [options]", argv0);
        spdlog::error("  session.json  a lidar_odometry_step_1 session; its point clouds are already");
        spdlog::error("                undistorted and are written straight into the map frame, plus a");
        spdlog::error("                /tf stream of map->lidar samples taken from each chunk's trajectory");
        spdlog::error("Options:");
        spdlog::error("  --raw-dir <dir>         original mandeye recording directory (imuNNNN.csv files);");
        spdlog::error("                          a session keeps no raw IMU samples, so this is the only way");
        spdlog::error("                          to include /imu. Omitted: lidar + tf only, no /imu channel is written.");
        spdlog::error("                          Always reads IMU id 0; warns if a file carries more than one IMU id.");
        spdlog::error("  --lidar-topic <name>    lidar PointCloud2 topic (default: /lidar_points)");
        spdlog::error("  --imu-topic <name>      IMU topic (default: /imu)");
        spdlog::error("  --tf-topic <name>       tf topic (default: /tf)");
        spdlog::error("  --map-frame <name>      tf parent frame / PointCloud2 frame_id (default: map)");
        spdlog::error("  --lidar-frame <name>    tf child frame / Imu frame_id (default: lidar)");
        spdlog::error("  --lidar-type <type>     PointCloud2 field layout: generic|velodyne|ouster|hesai (default: generic)");
        spdlog::error("  --msg_hz <float>        message rate: points are split into one PointCloud2 per");
        spdlog::error("                          1/hz seconds (default: 10; 0 = one message per session chunk)");
    }

} // namespace

int main(const int argc, const char** argv)
{
    if (argc < 3)
    {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    const std::string session_path = argv[1];
    const std::string mcap_path = argv[2];
    std::string raw_dir;
    rosbags::McapWriterOptions options;
    options.frame_id = "lidar";
    options.pointcloud_frame_id = "map";
    options.map_frame = "map";
    double msg_hz = 10.0;

    for (int i = 3; i < argc; ++i)
    {
        const std::string arg = argv[i];
        const bool hasValue = i + 1 < argc;

        if (arg == "--raw-dir" && hasValue)
            raw_dir = argv[++i];
        else if (arg == "--lidar-topic" && hasValue)
            options.lidar_topic = argv[++i];
        else if (arg == "--imu-topic" && hasValue)
            options.imu_topic = argv[++i];
        else if (arg == "--tf-topic" && hasValue)
            options.tf_topic = argv[++i];
        else if (arg == "--map-frame" && hasValue)
        {
            const std::string value = argv[++i];
            options.pointcloud_frame_id = value;
            options.map_frame = value;
        }
        else if (arg == "--lidar-frame" && hasValue)
            options.frame_id = argv[++i];
        else if (arg == "--msg_hz" && hasValue)
        {
            const std::string value = argv[++i];
            try
            {
                msg_hz = std::stod(value);
            } catch (const std::exception&)
            {
                spdlog::error("Invalid --msg_hz '{}' (expected a number)", value);
                return EXIT_FAILURE;
            }
            if (!std::isfinite(msg_hz) || msg_hz < 0.0)
            {
                spdlog::error("Invalid --msg_hz '{}' (expected >= 0; 0 = one message per session chunk)", value);
                return EXIT_FAILURE;
            }
        }
        else if (arg == "--lidar-type" && hasValue)
        {
            const std::string type = argv[++i];
            if (type == "generic")
                options.lidar_layout = rosbags::PointCloudLayout::Generic;
            else if (type == "velodyne")
                options.lidar_layout = rosbags::PointCloudLayout::Velodyne;
            else if (type == "ouster")
                options.lidar_layout = rosbags::PointCloudLayout::Ouster;
            else if (type == "hesai")
                options.lidar_layout = rosbags::PointCloudLayout::Hesai;
            else
            {
                spdlog::error("Unknown --lidar-type '{}' (expected generic|velodyne|ouster|hesai)", type);
                return EXIT_FAILURE;
            }
        }
        else
        {
            spdlog::error("Unrecognized argument '{}'", arg);
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    if (!check_path_ext(mcap_path, ".mcap"))
    {
        spdlog::error("Invalid extension for output file {} - expected .mcap", mcap_path);
        return EXIT_FAILURE;
    }
    if (!fs::exists(session_path))
    {
        spdlog::error("Session file {} does not exist", session_path);
        return EXIT_FAILURE;
    }
    Session session;
    if (!session.load(session_path, /*is_decimate=*/false, 0, 0, 0, /*calculate_offset=*/false))
    {
        spdlog::error("Failed to load session '{}'", session_path);
        return EXIT_FAILURE;
    }

    rosbags::McapFileWriter writer(mcap_path, options);
    if (!writer.isOpen())
    {
        spdlog::error("Failed to open output mcap file {}", mcap_path);
        return EXIT_FAILURE;
    }

    const auto& clouds = session.point_clouds_container.point_clouds;
    spdlog::info("Loaded session with {} chunk(s) from {}", clouds.size(), session_path);

    size_t total_points = 0;
    std::vector<rosbags::McapTransform> all_tf;
    MessageSplitter splitter(writer, msg_hz);
    for (size_t idx = 0; idx < clouds.size(); ++idx)
    {
        const auto& pc = clouds[idx];
        if (!pc.visible)
        {
            spdlog::info("[{}/{}] {}: skipped (not visible)", idx + 1, clouds.size(), pc.file_name);
            continue;
        }

        auto points = to_mcap_points(pc);
        sort_points_by_timestamp(points);
        total_points += points.size();
        splitter.add(points);
        spdlog::info("[{}/{}] {}: {} points", idx + 1, clouds.size(), pc.file_name, points.size());

        const auto tf = to_mcap_transforms(pc);
        all_tf.insert(all_tf.end(), tf.begin(), tf.end());
    }
    splitter.flush();
    spdlog::info(
        "Loaded {} points across {} chunk(s), wrote {} point cloud message(s)", total_points, clouds.size(), splitter.messages_written());

    std::sort(
        all_tf.begin(),
        all_tf.end(),
        [](const auto& a, const auto& b)
        {
            return a.timestamp < b.timestamp;
        });
    writer.writeTf(all_tf);
    spdlog::info("Wrote {} tf sample(s)", all_tf.size());

    if (!raw_dir.empty())
    {
        if (!fs::exists(raw_dir) || !fs::is_directory(raw_dir))
        {
            spdlog::error("--raw-dir {} does not exist or is not a directory - no /imu written", raw_dir);
        }
        else
        {
            const auto imu = load_all_imu(raw_dir);
            if (!imu.empty())
            {
                writer.writeImu(imu);
                spdlog::info("Loaded {} IMU sample(s) from {}", imu.size(), raw_dir);
            }
            else
            {
                spdlog::warn("No imu*.csv samples found in {} - no /imu written", raw_dir);
            }
        }
    }

    spdlog::info("Wrote {}", mcap_path);
    return EXIT_SUCCESS;
}
