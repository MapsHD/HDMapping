// Standalone mandeye session directory -> MCAP/ROS2-bag exporter. Built a
// second time as laz_to_bag (HDMAPPING_EXPORT_ROS1_BAG), which writes a ROS1 bag
// through rosbags/Ros1Bag instead -- same layouts, topics and message split.
//
// Reuses load_point_cloud()/load_imu() (lidar_odometry_utils.cpp) -- the
// same LAZ/IMU-csv parsing lidar_odometry_step_1 itself uses -- rather than
// load_data(), which additionally applies odometry-pipeline-specific
// post-processing (e.g. it discards the first loaded LAZ file's points
// entirely) that would silently drop data for a lossless exporter.
#include "McapWriter.h"
#include "lidar_odometry_utils.h"
#if defined(HDMAPPING_EXPORT_ROS1_BAG)
#include "Ros1Bag.h"
#endif

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <string>

#include <spdlog/spdlog.h>

#include "console_progress.h"

namespace fs = std::filesystem;

// load_imu()'s return type spelled out (rather than the `Imu` alias, which
// lives in lidar_odometry.h -- not included here, see the comment above).
using ImuData = std::vector<std::tuple<std::pair<double, double>, Eigen::Vector3f, Eigen::Vector3f>>;

namespace
{

#if defined(HDMAPPING_EXPORT_ROS1_BAG)
    using BagWriter = rosbags::Ros1BagFileWriter;
    constexpr const char* kOutputExtension = ".bag";
#else
    using BagWriter = rosbags::McapFileWriter;
    constexpr const char* kOutputExtension = ".mcap";
#endif

    // Standard gravity: the session's IMU csv is in g, sensor_msgs/Imu wants m/s^2.
    constexpr double kStandardGravity = 9.80665;

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

    // `rings` is parallel to `points` (load_point_cloud()'s ring output). The result
    // is sorted by timestamp, ties by ring: the order in which a spinning lidar
    // fires, and the order every PointCloud2 layout's relative time is taken in.
    std::vector<rosbags::McapPoint> to_mcap_points(const std::vector<Point3Di>& points, const std::vector<uint16_t>& rings)
    {
        std::vector<rosbags::McapPoint> out;
        out.reserve(points.size());
        for (size_t i = 0; i < points.size(); ++i)
        {
            const auto& p = points[i];
            rosbags::McapPoint mp{};
            mp.x = static_cast<float>(p.point.x());
            mp.y = static_cast<float>(p.point.y());
            mp.z = static_cast<float>(p.point.z());
            mp.intensity = p.intensity;
            mp.ring = i < rings.size() ? rings[i] : 0;
            mp.laser_id = p.lidarid;
            mp.timestamp = p.timestamp;
            out.push_back(mp);
        }
        std::stable_sort(
            out.begin(),
            out.end(),
            [](const rosbags::McapPoint& a, const rosbags::McapPoint& b)
            {
                return a.timestamp < b.timestamp || (a.timestamp == b.timestamp && a.ring < b.ring);
            });
        return out;
    }

    // `acc_scale` brings the csv's acceleration to the unit written (1 keeps g).
    std::vector<rosbags::McapImuSample> to_mcap_imu(const ImuData& imu_data, double acc_scale)
    {
        std::vector<rosbags::McapImuSample> out;
        out.reserve(imu_data.size());
        for (const auto& [ts, gyr, acc] : imu_data)
        {
            rosbags::McapImuSample s{};
            s.timestamp = ts.first;
            s.gyro_x = gyr.x();
            s.gyro_y = gyr.y();
            s.gyro_z = gyr.z();
            s.acc_x = static_cast<float>(acc.x() * acc_scale);
            s.acc_y = static_cast<float>(acc.y() * acc_scale);
            s.acc_z = static_cast<float>(acc.z() * acc_scale);
            out.push_back(s);
        }
        return out;
    }

    void sort_imu_by_timestamp(ImuData& imu)
    {
        std::sort(
            imu.begin(),
            imu.end(),
            [](const auto& a, const auto& b)
            {
                return std::get<0>(a).first < std::get<0>(b).first;
            });
    }

    // Cuts the incoming points into one PointCloud2 message per 1/msg_hz seconds,
    // so the bag replays at a lidar-like rate instead of one huge message per LAZ
    // file. A point with timestamp t lands in bin floor(t * msg_hz): the bin grid
    // is absolute, so it is identical for every chunk and a bin straddling a
    // chunk boundary still yields a single message (pending points carry over to
    // the next add()). msg_hz == 0 disables splitting -- one message per add().
    class MessageSplitter
    {
    public:
        MessageSplitter(BagWriter& writer, double msg_hz)
            : writer_(writer)
            , msg_hz_(msg_hz)
        {
        }

        // `points` must be sorted by timestamp, and successive calls must be in
        // timestamp order too (session chunks are processed oldest-first).
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

        // Writes whatever is still buffered; call once after the last add().
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

        BagWriter& writer_;
        double msg_hz_;
        std::vector<rosbags::McapPoint> pending_;
        int64_t current_bin_ = 0;
        size_t messages_ = 0;
    };

    // Last 4 characters of the filename stem, matching the chunk-index convention
    // mandeye recordings use (lidar0001.laz <-> imu0001.csv), mirroring
    // lidar_odometry.cpp's load_data()/get4index.
    std::string chunk_index(const std::string& path)
    {
        const std::string stem = fs::path(path).stem().string();
        return stem.size() > 4 ? stem.substr(stem.size() - 4) : stem;
    }

    // Per-sensor extrinsics (sensor id -> transform into the shared frame) plus
    // which sensor id's IMU stream to use, resolved from an MLvxCalib
    // calibration.json/.mjc + .sn file pair. Empty `per_sensor` means "apply no
    // calibration", matching load_point_cloud()'s own convention.
    struct Calibration
    {
        std::unordered_map<int, Eigen::Affine3d> per_sensor;
        int imu_id_to_use = 0;
    };

    // Mirrors lidar_odometry.cpp's load_data(): a session directory carries one
    // calibration file (calibration.json/.mjc, skipping camera-calibration jsons
    // named status*/cam0*/cam1*) and one .sn file describing every chunk.
    void find_calibration_files(const fs::path& dir, std::string& calibration_file, std::string& sn_file)
    {
        std::vector<std::string> jsons, mjcs, sns;
        for (const auto& entry : fs::directory_iterator(dir))
        {
            if (!entry.is_regular_file())
                continue;
            const std::string ext = to_lower(entry.path().extension().string());
            if (ext == ".json")
            {
                const std::string stem = to_lower(entry.path().stem().string());
                if (stem.starts_with("status") || stem.starts_with("cam0") || stem.starts_with("cam1"))
                    continue;
                jsons.push_back(entry.path().string());
            }
            else if (ext == ".mjc")
                mjcs.push_back(entry.path().string());
            else if (ext == ".sn")
                sns.push_back(entry.path().string());
        }
        std::sort(jsons.begin(), jsons.end());
        std::sort(mjcs.begin(), mjcs.end());
        std::sort(sns.begin(), sns.end());

        if (!jsons.empty())
            calibration_file = jsons.front();
        else if (!mjcs.empty())
            calibration_file = mjcs.front();

        if (!sns.empty())
            sn_file = sns.front();
    }

    // Loads an MLvxCalib calibration.json/.mjc + .sn pair (see
    // lidar_odometry_utils.h's MLvxCalib namespace doc comment for the file
    // format) and combines them into a sensor-id-keyed extrinsics map, ready to
    // pass to load_point_cloud(). Returns a default (no-op) Calibration if either
    // path is empty.
    Calibration load_calibration(const std::string& calibration_file, const std::string& sn_file)
    {
        Calibration result;
        if (calibration_file.empty() || sn_file.empty())
            return result;

        const auto preloadedCalibration = MLvxCalib::GetCalibrationFromFile(calibration_file);
        if (preloadedCalibration.empty())
        {
            spdlog::warn("No calibration data found in {} - exporting without per-sensor calibration", calibration_file);
            return result;
        }
        const auto idToSn = MLvxCalib::GetIdToSnMapping(sn_file);
        const std::string imuSnToUse = MLvxCalib::GetImuSnToUse(calibration_file);

        spdlog::info("Loaded calibration for {} sensor(s) from {}", preloadedCalibration.size(), calibration_file);
        for (const auto& [sn, _] : preloadedCalibration)
            spdlog::info(" -> {}", sn);

        for (const auto& [id, sn] : idToSn)
        {
            const auto it = preloadedCalibration.find(sn);
            if (it == preloadedCalibration.end())
            {
                spdlog::warn(
                    "Sensor id {} (serial '{}') from {} has no entry in {} - its points will be dropped",
                    id,
                    sn,
                    sn_file,
                    calibration_file);
                continue;
            }
            result.per_sensor[id] = it->second;
        }

        result.imu_id_to_use = MLvxCalib::GetImuIdToUse(idToSn, imuSnToUse);
        spdlog::info("Using IMU id {} (serial '{}')", result.imu_id_to_use, imuSnToUse);
        return result;
    }

    struct Chunk
    {
        std::string laz;
        std::string imu; // empty if no matching csv was found
    };

    // Scans `dir` for lidarNNNN.laz/.las + imuNNNN.csv chunk pairs (a real
    // mandeye recording folder) and matches them by chunk_index(). LAZ files
    // without a matching csv are still exported (points only, no IMU for that
    // chunk).
    std::vector<Chunk> scan_session_directory(const fs::path& dir)
    {
        std::vector<std::string> laz_files, csv_files;
        for (const auto& entry : fs::directory_iterator(dir))
        {
            if (!entry.is_regular_file())
                continue;
            const std::string ext = to_lower(entry.path().extension().string());
            if (ext == ".laz" || ext == ".las")
                laz_files.push_back(entry.path().string());
            else if (ext == ".csv")
                csv_files.push_back(entry.path().string());
        }
        std::sort(laz_files.begin(), laz_files.end());
        std::sort(csv_files.begin(), csv_files.end());

        std::vector<Chunk> chunks;
        chunks.reserve(laz_files.size());
        for (const auto& laz : laz_files)
        {
            const std::string idx = chunk_index(laz);
            Chunk chunk{ laz, {} };
            for (const auto& csv : csv_files)
            {
                if (chunk_index(csv) == idx)
                {
                    chunk.imu = csv;
                    break;
                }
            }
            if (chunk.imu.empty())
                spdlog::warn("No matching IMU csv for {} (chunk index '{}')", laz, idx);
            chunks.push_back(std::move(chunk));
        }
        return chunks;
    }

    void print_usage(const char* argv0)
    {
        spdlog::error("Usage: {} <session_dir> <output{}> [options]", argv0, kOutputExtension);
        spdlog::error("  session_dir  a mandeye recording folder of lidarNNNN.laz + imuNNNN.csv chunk");
        spdlog::error("               pairs (matched by filename's last 4 digits); every chunk becomes");
        spdlog::error("               its own /lidar_points message, IMU samples are merged into one stream");
        spdlog::error("Options:");
        spdlog::error("  --lidar-topic <name>    lidar PointCloud2 topic (default: /lidar_points)");
        spdlog::error("  --imu-topic <name>      IMU topic (default: /imu)");
        spdlog::error("  --sn-topic <name>       serial-number string topic (default: /lidar_sn)");
        spdlog::error("  --frame-id <name>       frame_id written into message headers (default: lidar)");
        spdlog::error("  --lidar-type <type>     PointCloud2 field layout: generic|velodyne|ouster|hesai|canonical");
        spdlog::error("                          (default: generic); canonical = x y z intensity ring time(float32) t,");
        spdlog::error("                          the benchmark's canonical spinning-lidar bag");
        spdlog::error("  --imu-acc-unit <g|ms2>  unit IMU acceleration is written in (default: g, the session's own,");
        spdlog::error("                          copied as is; ms2 = m/s^2 as sensor_msgs/Imu defines it)");
        spdlog::error("  --msg_hz <float>        message rate: points are split into one PointCloud2 per");
        spdlog::error("                          1/hz seconds (default: 10; 0 = one message per input file/chunk)");
        spdlog::error("  --calibration <file>    MLvxCalib calibration.json/.mjc (multi-LiVoX extrinsics + IMU");
        spdlog::error("                          selection); session_dir auto-detects this if omitted");
        spdlog::error("  --sn <file>             MLvxCalib .sn file (sensor id -> serial number); session_dir");
        spdlog::error("                          auto-detects this if omitted. Must be given together with --calibration");
        spdlog::error("The lidar ring is read from the LAS extra-bytes attribute 'ring' when the session has it");
        spdlog::error("(mcap_to_laz and bag_to_laz write it), otherwise every point's ring is 0.");
    }

    // The ROS1 bag writer finishes its index explicitly so a write error is not lost
    // in a destructor; MCAP finishes on destruction.
    bool finish(rosbags::McapFileWriter&)
    {
        return true;
    }
#if defined(HDMAPPING_EXPORT_ROS1_BAG)
    bool finish(rosbags::Ros1BagFileWriter& writer)
    {
        if (writer.close())
            return true;
        spdlog::error("Failed to finish the bag: {}", writer.error());
        return false;
    }
#endif

    int run_session_directory(
        const std::string& dir_path,
        const std::string& mcap_path,
        const rosbags::McapWriterOptions& options,
        double msg_hz,
        double acc_scale,
        std::string calibration_path,
        std::string sn_path)
    {
        auto chunks = scan_session_directory(dir_path);
        if (chunks.empty())
        {
            spdlog::error("No .laz/.las files found in {}", dir_path);
            return EXIT_FAILURE;
        }
        spdlog::info("Found {} chunk(s) in {}", chunks.size(), dir_path);

        if (calibration_path.empty() || sn_path.empty())
        {
            std::string found_calibration, found_sn;
            find_calibration_files(dir_path, found_calibration, found_sn);
            if (calibration_path.empty())
                calibration_path = found_calibration;
            if (sn_path.empty())
                sn_path = found_sn;
        }
        const Calibration calib = load_calibration(calibration_path, sn_path);

        BagWriter writer(mcap_path, options);
        if (!writer.isOpen())
        {
            spdlog::error("Failed to open output file {}", mcap_path);
            return EXIT_FAILURE;
        }

        size_t total_points = 0;
        size_t points_with_ring = 0;
        ImuData all_imu;
        std::vector<uint16_t> rings;
        MessageSplitter splitter(writer, msg_hz);
        const ConsoleProgress progress("Chunks", chunks.size());
        for (size_t i = 0; i < chunks.size(); ++i)
        {
            auto points = load_point_cloud(
                chunks[i].laz,
                /*ommit_points_with_timestamp_equals_zero=*/false,
                /*filter_threshold_xy_inner=*/0.0,
                /*filter_threshold_xy_outer=*/std::numeric_limits<double>::max(),
                /*calibrations=*/calib.per_sensor,
                /*rings=*/&rings);
            total_points += points.size();
            points_with_ring += static_cast<size_t>(std::count_if(
                rings.begin(),
                rings.end(),
                [](uint16_t r)
                {
                    return r != 0;
                }));
            splitter.add(to_mcap_points(points, rings));
            spdlog::info("[{}] {}: {} points", progress.status(i + 1), chunks[i].laz, points.size());

            if (!chunks[i].imu.empty())
            {
                auto imu_data = load_imu(chunks[i].imu, calib.imu_id_to_use);
                all_imu.insert(all_imu.end(), std::make_move_iterator(imu_data.begin()), std::make_move_iterator(imu_data.end()));
            }
        }
        splitter.flush();
        spdlog::info(
            "Loaded {} points across {} chunk(s), wrote {} point cloud message(s)",
            total_points,
            chunks.size(),
            splitter.messages_written());
        if (points_with_ring == 0 && total_points > 0)
            spdlog::info("No point has a ring (the session has no 'ring' attribute, or every ring is 0)");

        if (!all_imu.empty())
        {
            sort_imu_by_timestamp(all_imu);
            writer.writeImu(to_mcap_imu(all_imu, acc_scale));
            spdlog::info("Loaded {} IMU samples across matched chunk(s)", all_imu.size());
        }

        if (!finish(writer))
            return EXIT_FAILURE;
        spdlog::info("Wrote {}", mcap_path);
        return EXIT_SUCCESS;
    }

} // namespace

int main(const int argc, const char** argv)
{
    if (argc < 3)
    {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    const std::string input_path = argv[1];
    const std::string mcap_path = argv[2];
    std::string calibration_path;
    std::string sn_path;
    rosbags::McapWriterOptions options;
    double msg_hz = 10.0;
    double acc_scale = 1.0;

    for (int i = 3; i < argc; ++i)
    {
        const std::string arg = argv[i];
        const bool hasValue = i + 1 < argc;

        if (arg == "--calibration" && hasValue)
            calibration_path = argv[++i];
        else if (arg == "--sn" && hasValue)
            sn_path = argv[++i];
        else if (arg == "--lidar-topic" && hasValue)
            options.lidar_topic = argv[++i];
        else if (arg == "--imu-topic" && hasValue)
            options.imu_topic = argv[++i];
        else if (arg == "--sn-topic" && hasValue)
            options.sn_topic = argv[++i];
        else if (arg == "--frame-id" && hasValue)
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
                spdlog::error("Invalid --msg_hz '{}' (expected >= 0; 0 = one message per input file/chunk)", value);
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
            else if (type == "canonical")
                options.lidar_layout = rosbags::PointCloudLayout::Canonical;
            else
            {
                spdlog::error("Unknown --lidar-type '{}' (expected generic|velodyne|ouster|hesai|canonical)", type);
                return EXIT_FAILURE;
            }
        }
        else if (arg == "--imu-acc-unit" && hasValue)
        {
            const std::string unit = argv[++i];
            if (unit == "g")
                acc_scale = 1.0;
            else if (unit == "ms2")
                acc_scale = kStandardGravity;
            else
            {
                spdlog::error("Unknown --imu-acc-unit '{}' (expected g|ms2)", unit);
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

    if (!check_path_ext(mcap_path, kOutputExtension))
    {
        spdlog::error("Invalid extension for output file {} - expected {}", mcap_path, kOutputExtension);
        return EXIT_FAILURE;
    }
    if (!fs::exists(input_path))
    {
        spdlog::error("Input path {} does not exist", input_path);
        return EXIT_FAILURE;
    }
    if (!fs::is_directory(input_path))
    {
        spdlog::error("Input path {} is not a directory - expected a mandeye session directory", input_path);
        return EXIT_FAILURE;
    }
    if (calibration_path.empty() != sn_path.empty())
    {
        spdlog::error("--calibration and --sn must be given together");
        return EXIT_FAILURE;
    }

    return run_session_directory(input_path, mcap_path, options, msg_hz, acc_scale, calibration_path, sn_path);
}