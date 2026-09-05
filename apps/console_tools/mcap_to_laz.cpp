// MCAP/ROS2-bag -> mandeye session directory importer: the inverse of
// laz_to_mcap.
//
// Deliberately does not link lidar_odometry_utils.cpp. Nothing here needs
// load_point_cloud()/load_imu(); reading is handled by rosbags/McapReader and
// writing by rosbags/MandeyeSessionWriter, which keeps this tool free of
// core/TBB/glm/toml++/vqf.
#include "MandeyeSessionWriter.h"
#include "McapReader.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

namespace
{

    // Absolute time grid, matching MessageSplitter in laz_to_mcap.cpp: a timestamp t
    // belongs to bin floor(t / chunk_seconds). Because the grid is absolute rather
    // than relative to the first message, lidar and IMU bin identically without the
    // two streams having to agree on a time origin -- which they cannot, since which
    // stream a bag's first message belongs to depends on its read order.
    int64_t timeBin(double timestamp_s, double chunk_seconds)
    {
        if (chunk_seconds <= 0.0)
            return 0; // splitting disabled: everything lands in one chunk
        return static_cast<int64_t>(std::floor(timestamp_s / chunk_seconds));
    }

    void print_usage(const char* argv0)
    {
        spdlog::error("Usage: {} <input.mcap> <output_session_dir> [options]", argv0);
        spdlog::error("  input.mcap          a ros2msg/CDR MCAP bag, e.g. one written by laz_to_mcap");
        spdlog::error("  output_session_dir  written as mandeye lidarNNNN.laz + imuNNNN.csv chunk pairs,");
        spdlog::error("                      ready to open in lidar_odometry_step_1");
        spdlog::error("Options:");
        spdlog::error("  --chunk-seconds <float>  chunk length in seconds (default: 20, mandeye's own");
        spdlog::error("                           cadence; 0 = one single chunk)");
        spdlog::error("  --lidar-topic <name>     PointCloud2 topic (default: the file's only one)");
        spdlog::error("  --imu-topic <name>       Imu topic (default: the file's only one)");
        spdlog::error("  --sn-topic <name>        serial-number String topic (default: the file's only one)");
        spdlog::error("  --lidar-type <preset>    PointCloud2 record layout: auto|hesai|ouster (default: auto)");
        spdlog::error("                           auto reads each message's own field offsets and datatypes and is");
        spdlog::error("                           correct for any bag that fills them in, including laz_to_mcap's.");
        spdlog::error("                           A preset forces that driver's layout for bags whose field array is");
        spdlog::error("                           missing or wrong, and warns if it disagrees with what the file says.");
        spdlog::error("  --list                   print the file's channels and exit");
        spdlog::error("");
        spdlog::error("Single-lidar bags only: one PointCloud2 topic in, one point cloud stream out. No");
        spdlog::error("calibration.json or .sn file is written, so lidar_odometry_step_1 applies no");
        spdlog::error("extrinsics to the result.");
    }

    int list_channels(const rosbags::McapFileReader& reader)
    {
        const auto& channels = reader.channels();
        if (channels.empty())
        {
            spdlog::error("No channels found: {}", reader.error());
            return EXIT_FAILURE;
        }

        spdlog::info("{} channel(s):", channels.size());
        for (const auto& c : channels)
            spdlog::info("  {:<28} {:<32} encoding={:<10} messages={}", c.topic, c.schema_name, c.message_encoding, c.message_count);

        if (reader.isOpen())
        {
            const auto& topics = reader.topics();
            spdlog::info("Resolved: lidar='{}' imu='{}' sn='{}'", topics.lidar, topics.imu, topics.sn);
        }
        else
        {
            spdlog::warn("Topics could not be resolved: {}", reader.error());
        }
        return EXIT_SUCCESS;
    }

    struct ImportStats
    {
        size_t lidar_messages = 0;
        bool out_of_order = false;
    };

    // Reads every IMU sample (and the serial number, if present) into memory. The
    // full stream is small -- 200 Hz for an hour is ~700k samples -- and buffering it
    // lets each chunk's csv be written in one shot after the lidar pass has decided
    // where the chunk boundaries fall.
    bool collect_imu(rosbags::McapFileReader& reader, std::vector<rosbags::McapImuSample>& out, std::string& serial_number)
    {
        rosbags::McapFileReader::Callbacks callbacks;
        callbacks.onImu = [&](const rosbags::McapImuSample& sample)
        {
            out.push_back(sample);
        };
        callbacks.onSn = [&](uint64_t, const std::string& sn)
        {
            if (serial_number.empty())
                serial_number = sn;
        };

        if (!reader.read(callbacks))
            return false;

        std::sort(
            out.begin(),
            out.end(),
            [](const auto& a, const auto& b)
            {
                return a.timestamp < b.timestamp;
            });
        return true;
    }

    // Streams point clouds into lidarNNNN.laz, starting a new chunk whenever the
    // message crosses a time-bin boundary. Returns the bin of each chunk written, in
    // chunk-index order, so the IMU pass can route samples to the same chunks.
    bool write_lidar_chunks(
        rosbags::McapFileReader& reader,
        rosbags::MandeyeSessionWriter& writer,
        double chunk_seconds,
        std::vector<int64_t>& chunk_bins,
        ImportStats& stats)
    {
        bool ok = true;
        int64_t current_bin = 0;
        double last_stamp = -std::numeric_limits<double>::infinity();

        rosbags::McapFileReader::Callbacks callbacks;
        callbacks.onPointCloud = [&](uint64_t /*log_time_ns*/, std::vector<rosbags::McapPoint>&& points)
        {
            if (!ok || points.empty())
                return;

            // Bin on the cloud's first point timestamp, not on the MCAP log time:
            // point timestamps are what lands in the laz files and what IMU samples
            // are binned by, so this keeps both streams on one clock. The cloud is
            // the atomic unit -- splitting one across two laz files would be
            // pointless churn given load_data() concatenates the chunks anyway.
            const double stamp_s = points.front().timestamp;
            if (stamp_s < last_stamp)
                stats.out_of_order = true;
            last_stamp = stamp_s;

            const int64_t bin = timeBin(stamp_s, chunk_seconds);
            if (chunk_bins.empty() || bin != current_bin)
            {
                if (!writer.beginChunk(static_cast<int>(chunk_bins.size())))
                {
                    ok = false;
                    return;
                }
                chunk_bins.push_back(bin);
                current_bin = bin;
            }

            if (!writer.addPoints(points))
            {
                ok = false;
                return;
            }
            ++stats.lidar_messages;
        };

        if (!reader.read(callbacks))
            return false;
        if (!writer.endChunk())
            return false;
        return ok;
    }

    // Routes every buffered IMU sample into the chunk that was open at its
    // timestamp: the last chunk whose bin is <= the sample's bin, clamped to the
    // first chunk for samples that predate all lidar data. No sample is dropped --
    // load_data() concatenates every imuNNNN.csv into one stream, so which chunk a
    // sample lands in only affects file layout, not the resulting IMU trajectory.
    bool write_imu_chunks(
        rosbags::MandeyeSessionWriter& writer,
        const std::vector<rosbags::McapImuSample>& samples,
        const std::vector<int64_t>& chunk_bins,
        double chunk_seconds)
    {
        std::vector<std::vector<rosbags::McapImuSample>> per_chunk(chunk_bins.size());

        // chunk_bins is ascending for any bag whose clouds arrive in time order,
        // but a file-order read of an out-of-order bag can break that -- and
        // upper_bound on an unsorted range would route samples arbitrarily. Sort a
        // (bin, chunk index) view instead of assuming.
        std::vector<std::pair<int64_t, size_t>> bin_to_chunk;
        bin_to_chunk.reserve(chunk_bins.size());
        for (size_t i = 0; i < chunk_bins.size(); ++i)
            bin_to_chunk.emplace_back(chunk_bins[i], i);
        std::sort(bin_to_chunk.begin(), bin_to_chunk.end());

        for (const auto& sample : samples)
        {
            const int64_t bin = timeBin(sample.timestamp, chunk_seconds);
            const auto it = std::upper_bound(
                bin_to_chunk.begin(),
                bin_to_chunk.end(),
                bin,
                [](int64_t value, const std::pair<int64_t, size_t>& entry)
                {
                    return value < entry.first;
                });
            const size_t index = (it == bin_to_chunk.begin()) ? bin_to_chunk.front().second : std::prev(it)->second;
            per_chunk[index].push_back(sample);
        }

        for (size_t i = 0; i < per_chunk.size(); ++i)
        {
            if (!writer.writeImuChunk(static_cast<int>(i), per_chunk[i]))
                return false;
        }
        return true;
    }

} // namespace

int main(const int argc, const char** argv)
{
    if (argc < 2)
    {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    const std::string mcap_path = argv[1];
    std::string session_dir;
    rosbags::McapReaderOptions options;
    double chunk_seconds = 20.0;
    bool list_only = false;

    for (int i = 2; i < argc; ++i)
    {
        const std::string arg = argv[i];
        const bool hasValue = i + 1 < argc;

        if (arg == "--list")
            list_only = true;
        else if (arg == "--lidar-topic" && hasValue)
            options.lidar_topic = argv[++i];
        else if (arg == "--imu-topic" && hasValue)
            options.imu_topic = argv[++i];
        else if (arg == "--sn-topic" && hasValue)
            options.sn_topic = argv[++i];
        else if (arg == "--lidar-type" && hasValue)
        {
            const std::string type = argv[++i];
            if (type == "auto")
                options.lidar_preset = rosbags::Pc2Preset::Auto;
            else if (type == "hesai")
                options.lidar_preset = rosbags::Pc2Preset::Hesai;
            else if (type == "ouster")
                options.lidar_preset = rosbags::Pc2Preset::Ouster;
            else
            {
                spdlog::error("Unknown --lidar-type '{}' (expected auto|hesai|ouster)", type);
                return EXIT_FAILURE;
            }
        }
        else if (arg == "--chunk-seconds" && hasValue)
        {
            const std::string value = argv[++i];
            try
            {
                chunk_seconds = std::stod(value);
            } catch (const std::exception&)
            {
                spdlog::error("Invalid --chunk-seconds '{}' (expected a number)", value);
                return EXIT_FAILURE;
            }
            if (!std::isfinite(chunk_seconds) || chunk_seconds < 0.0)
            {
                spdlog::error("Invalid --chunk-seconds '{}' (expected >= 0; 0 = one single chunk)", value);
                return EXIT_FAILURE;
            }
        }
        else if (!arg.starts_with("--") && session_dir.empty())
            session_dir = arg;
        else
        {
            spdlog::error("Unrecognized argument '{}'", arg);
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    if (fs::path(mcap_path).extension() != ".mcap")
    {
        spdlog::error("Invalid extension for input file {} - expected .mcap", mcap_path);
        return EXIT_FAILURE;
    }
    if (!fs::exists(mcap_path))
    {
        spdlog::error("Input file {} does not exist", mcap_path);
        return EXIT_FAILURE;
    }
    if (!list_only && session_dir.empty())
    {
        spdlog::error("Missing <output_session_dir>");
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    rosbags::McapFileReader reader(mcap_path, options);
    if (list_only)
        return list_channels(reader);

    if (!reader.isOpen())
    {
        spdlog::error("Cannot read {}: {}", mcap_path, reader.error());
        spdlog::error("Run with --list to see the file's channels.");
        return EXIT_FAILURE;
    }

    const auto& topics = reader.topics();
    if (topics.lidar.empty())
    {
        spdlog::error("No sensor_msgs/msg/PointCloud2 topic found in {}", mcap_path);
        return EXIT_FAILURE;
    }
    spdlog::info(
        "Reading lidar='{}' (layout: {}) imu='{}' sn='{}'",
        topics.lidar,
        rosbags::pc2PresetName(options.lidar_preset),
        topics.imu.empty() ? "<none>" : topics.imu,
        topics.sn.empty() ? "<none>" : topics.sn);
    if (!reader.timeOrdered())
        spdlog::warn("{} has no message indexes - reading in file order instead of log-time order", mcap_path);

    std::vector<rosbags::McapImuSample> imu;
    std::string serial_number;
    if (!topics.imu.empty() && !collect_imu(reader, imu, serial_number))
    {
        spdlog::error("Failed reading IMU stream: {}", reader.error());
        return EXIT_FAILURE;
    }
    if (!serial_number.empty())
        spdlog::info("Bag serial number: {}", serial_number);
    spdlog::info("Read {} IMU sample(s)", imu.size());

    rosbags::MandeyeSessionWriter writer(session_dir);
    if (!writer.isOpen())
    {
        spdlog::error("Cannot write session directory: {}", writer.error());
        return EXIT_FAILURE;
    }

    std::vector<int64_t> chunk_bins;
    ImportStats stats;
    if (!write_lidar_chunks(reader, writer, chunk_seconds, chunk_bins, stats))
    {
        spdlog::error("Failed writing lidar chunks: {}", writer.error().empty() ? reader.error() : writer.error());
        return EXIT_FAILURE;
    }
    // Emitted before the empty check on purpose: a forced --lidar-type that the
    // file disagrees with is the likeliest reason for decoding nothing at all, so
    // the explanation has to come out even on the failure path.
    if (!reader.lidarLayoutWarning().empty())
        spdlog::warn("PointCloud2 decoding: {}", reader.lidarLayoutWarning());

    if (chunk_bins.empty())
    {
        spdlog::error("No point cloud messages decoded from '{}' - nothing to write", topics.lidar);
        if (options.lidar_preset != rosbags::Pc2Preset::Auto)
            spdlog::error("Try --lidar-type auto, which takes the record layout from the messages themselves.");
        return EXIT_FAILURE;
    }
    if (!write_imu_chunks(writer, imu, chunk_bins, chunk_seconds))
    {
        spdlog::error("Failed writing IMU chunks: {}", writer.error());
        return EXIT_FAILURE;
    }

    spdlog::info(
        "Wrote {} chunk(s) to {}: {} point(s) from {} message(s), {} IMU sample(s)",
        chunk_bins.size(),
        session_dir,
        writer.pointsWritten(),
        stats.lidar_messages,
        writer.imuSamplesWritten());

    if (stats.out_of_order)
        spdlog::warn("Point cloud messages were not in ascending time order - chunk boundaries may not be monotonic");

    // lidar_odometry_step_1's load_data() needs more than two input files and then
    // discards the first chunk's points outright, so a session of one or two
    // chunks contributes little or nothing there.
    if (chunk_bins.size() < 3)
        spdlog::warn(
            "Only {} chunk(s): lidar_odometry_step_1 requires more than two files and discards the first chunk's "
            "points - lower --chunk-seconds to split this bag further",
            chunk_bins.size());

    if (!imu.empty())
        spdlog::info(
            "IMU timestampUnix is written as 0: the bag format carries only one IMU timestamp. It feeds the step-1 "
            "trajectory csv only.");

    return EXIT_SUCCESS;
}
