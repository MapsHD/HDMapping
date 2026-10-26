// MCAP/ROS2-bag -> mandeye session directory importer: the inverse of
// laz_to_mcap. Built a second time as bag_to_laz (HDMAPPING_IMPORT_ROS1_BAG),
// which reads a ROS1 bag through rosbags/Ros1Bag instead -- the inverse of
// laz_to_bag. Everything after the reader is shared.
//
// Deliberately does not link lidar_odometry_utils.cpp. Nothing here needs
// load_point_cloud()/load_imu(); reading is handled by rosbags/McapReader and
// writing by rosbags/MandeyeSessionWriter, which keeps this tool free of
// core/TBB/glm/toml++/vqf.
#include "MandeyeSessionWriter.h"
#include "McapReader.h"
#if defined(HDMAPPING_IMPORT_ROS1_BAG)
#include "Ros1Bag.h"
#endif

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

namespace
{

#if defined(HDMAPPING_IMPORT_ROS1_BAG)
    using BagReader = rosbags::Ros1BagFileReader;
    constexpr const char* kInputExtension = ".bag";
    constexpr const char* kInputDescription = "a ROS1 bag (format 2.0; uncompressed, bz2 or lz4 chunks)";
#else
    using BagReader = rosbags::McapFileReader;
    constexpr const char* kInputExtension = ".mcap";
    constexpr const char* kInputDescription = "a ros2msg/CDR MCAP bag, e.g. one written by laz_to_mcap";
#endif

    // Standard gravity, the g HDMapping's IMU csv is scaled in.
    constexpr double kStandardGravity = 9.80665;

    // What the importer does to every IMU sample before it reaches the csv:
    // rotate it into the lidar frame (the frame HDMapping assumes the IMU is in)
    // and bring acceleration to g.
    struct ImuTransform
    {
        double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0; // IMU frame -> lidar frame
        double acc_scale = 1.0; // 1 for a bag already in g, 1/9.80665 for m/s^2
    };

    void rotate(const ImuTransform& q, float& x, float& y, float& z)
    {
        // v' = v + 2w (u x v) + 2 u x (u x v), u = (qx, qy, qz)
        const double vx = x, vy = y, vz = z;
        const double cx = q.qy * vz - q.qz * vy, cy = q.qz * vx - q.qx * vz, cz = q.qx * vy - q.qy * vx;
        x = static_cast<float>(vx + 2.0 * (q.qw * cx + q.qy * cz - q.qz * cy));
        y = static_cast<float>(vy + 2.0 * (q.qw * cy + q.qz * cx - q.qx * cz));
        z = static_cast<float>(vz + 2.0 * (q.qw * cz + q.qx * cy - q.qy * cx));
    }

    void apply_imu_transform(const ImuTransform& t, std::vector<rosbags::McapImuSample>& samples)
    {
        for (auto& s : samples)
        {
            rotate(t, s.gyro_x, s.gyro_y, s.gyro_z);
            rotate(t, s.acc_x, s.acc_y, s.acc_z);
            s.acc_x = static_cast<float>(s.acc_x * t.acc_scale);
            s.acc_y = static_cast<float>(s.acc_y * t.acc_scale);
            s.acc_z = static_cast<float>(s.acc_z * t.acc_scale);
        }
    }

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
        spdlog::error("Usage: {} <input{}> [<input{}> ...] <output_session_dir> [options]", argv0, kInputExtension, kInputExtension);
        spdlog::error("  input{}  {}; several inputs (e.g. the parts of a split", kInputExtension, kInputDescription);
        spdlog::error("               recording) are read in the order given into one session");
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
        spdlog::error("  --imu-rotation qx qy qz qw  rotates IMU samples into the lidar frame (R_lidar_imu as a");
        spdlog::error("                           quaternion; default: identity, i.e. the IMU is in the lidar frame)");
        spdlog::error("  --imu-acc-unit <g|ms2>   unit of the bag's IMU acceleration (default: g, copied as is;");
        spdlog::error("                           ms2 = m/s^2 as sensor_msgs/Imu defines it, divided by 9.80665)");
        spdlog::error("  --list                   print the file's channels and exit");
        spdlog::error("");
        spdlog::error("The lidar ring, when the bag has one, is kept as the LAS extra-bytes attribute 'ring'.");
        spdlog::error("Single-lidar bags only: one PointCloud2 topic in, one point cloud stream out. No");
        spdlog::error("calibration.json or .sn file is written, so lidar_odometry_step_1 applies no");
        spdlog::error("extrinsics to the result.");
    }

    int list_channels(const BagReader& reader)
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
    bool collect_imu(BagReader& reader, std::vector<rosbags::McapImuSample>& out, std::string& serial_number)
    {
        BagReader::Callbacks callbacks;
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
    // Several readers are one continuous stream: the bin grid is absolute, so a bin
    // that straddles two input files stays a single chunk.
    bool write_lidar_chunks(
        std::vector<std::unique_ptr<BagReader>>& readers,
        rosbags::MandeyeSessionWriter& writer,
        double chunk_seconds,
        std::vector<int64_t>& chunk_bins,
        ImportStats& stats)
    {
        bool ok = true;
        int64_t current_bin = 0;
        double last_stamp = -std::numeric_limits<double>::infinity();

        BagReader::Callbacks callbacks;
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

        for (auto& reader : readers)
        {
            if (!reader->read(callbacks))
                return false;
            if (!ok)
                return false;
        }
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

    std::vector<std::string> positionals;
    rosbags::McapReaderOptions options;
    double chunk_seconds = 20.0;
    bool list_only = false;
    ImuTransform imu_transform;

    for (int i = 1; i < argc; ++i)
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
        else if (arg == "--imu-rotation" && i + 4 < argc)
        {
            try
            {
                imu_transform.qx = std::stod(argv[++i]);
                imu_transform.qy = std::stod(argv[++i]);
                imu_transform.qz = std::stod(argv[++i]);
                imu_transform.qw = std::stod(argv[++i]);
            } catch (const std::exception&)
            {
                spdlog::error("Invalid --imu-rotation (expected four numbers qx qy qz qw)");
                return EXIT_FAILURE;
            }
            const double n = std::sqrt(
                imu_transform.qx * imu_transform.qx + imu_transform.qy * imu_transform.qy + imu_transform.qz * imu_transform.qz +
                imu_transform.qw * imu_transform.qw);
            if (!std::isfinite(n) || std::abs(n - 1.0) > 1e-3)
            {
                spdlog::error("Invalid --imu-rotation (not a unit quaternion, norm {:.6f})", n);
                return EXIT_FAILURE;
            }
            imu_transform.qx /= n;
            imu_transform.qy /= n;
            imu_transform.qz /= n;
            imu_transform.qw /= n;
        }
        else if (arg == "--imu-acc-unit" && hasValue)
        {
            const std::string unit = argv[++i];
            if (unit == "g")
                imu_transform.acc_scale = 1.0;
            else if (unit == "ms2")
                imu_transform.acc_scale = 1.0 / kStandardGravity;
            else
            {
                spdlog::error("Unknown --imu-acc-unit '{}' (expected g|ms2)", unit);
                return EXIT_FAILURE;
            }
        }
        else if (!arg.starts_with("--"))
            positionals.push_back(arg);
        else
        {
            spdlog::error("Unrecognized argument '{}'", arg);
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    // Every positional is an input except the last, the session dir (with --list,
    // all of them are inputs).
    std::string session_dir;
    if (!list_only)
    {
        if (positionals.size() < 2)
        {
            spdlog::error("Missing <output_session_dir>");
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
        session_dir = positionals.back();
        positionals.pop_back();
    }
    if (positionals.empty())
    {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }
    for (const auto& input : positionals)
    {
        if (fs::path(input).extension() != kInputExtension)
        {
            spdlog::error("Invalid extension for input file {} - expected {}", input, kInputExtension);
            return EXIT_FAILURE;
        }
        if (!fs::exists(input))
        {
            spdlog::error("Input file {} does not exist", input);
            return EXIT_FAILURE;
        }
    }

    std::vector<std::unique_ptr<BagReader>> readers;
    for (const auto& input : positionals)
        readers.push_back(std::make_unique<BagReader>(input, options));
    if (list_only)
    {
        int rc = EXIT_SUCCESS;
        for (size_t k = 0; k < readers.size(); ++k)
        {
            spdlog::info("{}:", positionals[k]);
            if (list_channels(*readers[k]) != EXIT_SUCCESS)
                rc = EXIT_FAILURE;
        }
        return rc;
    }

    for (size_t k = 0; k < readers.size(); ++k)
    {
        if (!readers[k]->isOpen())
        {
            spdlog::error("Cannot read {}: {}", positionals[k], readers[k]->error());
            spdlog::error("Run with --list to see the file's channels.");
            return EXIT_FAILURE;
        }
        if (readers[k]->topics().lidar.empty())
        {
            spdlog::error("No sensor_msgs/msg/PointCloud2 topic found in {}", positionals[k]);
            return EXIT_FAILURE;
        }
        // One session is one sensor: auto-detection must land on the same topics in every input.
        const auto& t = readers[k]->topics();
        const auto& first = readers.front()->topics();
        if (t.lidar != first.lidar || t.imu != first.imu)
        {
            spdlog::error(
                "{} resolves to lidar='{}' imu='{}' but {} to lidar='{}' imu='{}' - pass --lidar-topic/--imu-topic",
                positionals[k],
                t.lidar,
                t.imu,
                positionals.front(),
                first.lidar,
                first.imu);
            return EXIT_FAILURE;
        }
    }
    if (readers.size() > 1)
        spdlog::info("Reading {} inputs in the order given", readers.size());

    const auto& topics = readers.front()->topics();
    spdlog::info(
        "Reading lidar='{}' (layout: {}) imu='{}' sn='{}'",
        topics.lidar,
        rosbags::pc2PresetName(options.lidar_preset),
        topics.imu.empty() ? "<none>" : topics.imu,
        topics.sn.empty() ? "<none>" : topics.sn);
    for (size_t k = 0; k < readers.size(); ++k)
        if (!readers[k]->timeOrdered())
            spdlog::warn("{} has no message indexes - reading in file order instead of log-time order", positionals[k]);

    std::vector<rosbags::McapImuSample> imu;
    std::string serial_number;
    for (size_t k = 0; k < readers.size() && !topics.imu.empty(); ++k)
    {
        if (!collect_imu(*readers[k], imu, serial_number))
        {
            spdlog::error("Failed reading IMU stream of {}: {}", positionals[k], readers[k]->error());
            return EXIT_FAILURE;
        }
    }
    if (!serial_number.empty())
        spdlog::info("Bag serial number: {}", serial_number);
    apply_imu_transform(imu_transform, imu);
    spdlog::info("Read {} IMU sample(s)", imu.size());

    rosbags::MandeyeSessionWriter writer(session_dir, /*with_rgb=*/false, /*with_ring=*/true);
    if (!writer.isOpen())
    {
        spdlog::error("Cannot write session directory: {}", writer.error());
        return EXIT_FAILURE;
    }

    std::vector<int64_t> chunk_bins;
    ImportStats stats;
    if (!write_lidar_chunks(readers, writer, chunk_seconds, chunk_bins, stats))
    {
        std::string why = writer.error();
        for (const auto& reader : readers)
            if (why.empty())
                why = reader->error();
        spdlog::error("Failed writing lidar chunks: {}", why);
        return EXIT_FAILURE;
    }
    // Emitted before the empty check on purpose: a forced --lidar-type that the
    // file disagrees with is the likeliest reason for decoding nothing at all, so
    // the explanation has to come out even on the failure path.
    for (size_t k = 0; k < readers.size(); ++k)
        if (!readers[k]->lidarLayoutWarning().empty())
            spdlog::warn("PointCloud2 decoding ({}): {}", positionals[k], readers[k]->lidarLayoutWarning());

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
        spdlog::warn(
            "Point cloud messages were not in ascending time order - chunk boundaries may not be monotonic{}",
            readers.size() > 1 ? " (are the inputs given in recording order?)" : "");

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
