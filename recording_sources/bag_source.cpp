// The rosbags headers come first: they re-enable std::byte on MSVC before any standard header (see rosbags/McapWriter.h).
#include "ImuTransform.h"
#include "McapReader.h"
#include "Ros1Bag.h"

#include "bag_source.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string_view>

namespace fs = std::filesystem;

namespace
{
    std::string lower_extension(const fs::path& file)
    {
        std::string ext = file.extension().string();
        std::transform(
            ext.begin(),
            ext.end(),
            ext.begin(),
            [](unsigned char c)
            {
                return static_cast<char>(std::tolower(c));
            });
        return ext;
    }

    bool is_ros1_bag(const fs::path& file)
    {
        return lower_extension(file) == ".bag";
    }

    //! Message type without its package, as both readers compare it: "sensor_msgs/msg/Imu" -> "Imu".
    std::string_view type_name(std::string_view type)
    {
        const auto slash = type.rfind('/');
        return slash == std::string_view::npos ? type : type.substr(slash + 1);
    }

    rosbags::McapReaderOptions reader_options(const BagSourceConfig& config)
    {
        rosbags::McapReaderOptions options;
        options.lidar_topic = config.lidar_topic;
        options.imu_topic = config.imu_topic;
        switch (config.lidar_layout)
        {
        case BagSourceConfig::LidarLayout::Auto:
            options.lidar_preset = rosbags::Pc2Preset::Auto;
            break;
        case BagSourceConfig::LidarLayout::Hesai:
            options.lidar_preset = rosbags::Pc2Preset::Hesai;
            break;
        case BagSourceConfig::LidarLayout::Ouster:
            options.lidar_preset = rosbags::Pc2Preset::Ouster;
            break;
        }
        return options;
    }

    rosbags::ImuTransform imu_transform(const BagSourceConfig& config)
    {
        rosbags::ImuTransform t;
        t.qx = config.imu_to_lidar[0];
        t.qy = config.imu_to_lidar[1];
        t.qz = config.imu_to_lidar[2];
        t.qw = config.imu_to_lidar[3];
        t.acc_scale = config.imu_acc_in_g ? 1.0 : 1.0 / rosbags::kStandardGravity;
        return t;
    }

    ChunkStamp file_stamp(const fs::path& file)
    {
        std::error_code ec;
        ChunkStamp s;
        s.size = fs::file_size(file, ec);
        if (!ec)
            s.time = fs::last_write_time(file, ec);
        s.valid = !ec;
        return s;
    }

    //! A ROS1 bag or MCAP file split into windows of message log time. Every read opens its own reader, so chunks can be
    //! loaded from several threads at once.
    template<typename Reader>
    class BagSource final : public RecordingSource
    {
    public:
        BagSource(const BagSourceConfig& config, const rosbags::LogTimeRange& span)
            : config_(config)
            , options_(reader_options(config))
            , imu_transform_(imu_transform(config))
            , span_(span)
        {
            const uint64_t length = span_.end_ns - span_.start_ns;
            chunk_ns_ = config_.chunk_seconds > 0.0 ? static_cast<uint64_t>(config_.chunk_seconds * 1e9) : length;
            chunk_ns_ = std::max<uint64_t>(chunk_ns_, 1);
            chunk_count_ = std::max<size_t>(1, static_cast<size_t>((length + chunk_ns_ - 1) / chunk_ns_));
        }

        fs::path workingDirectory() const override
        {
            return config_.file.parent_path();
        }

        size_t chunkCount() const override
        {
            return chunk_count_;
        }

        std::string chunkName(size_t i) const override
        {
            char index[16];
            std::snprintf(index, sizeof(index), "#%04zu", i);
            return config_.file.string() + index;
        }

        std::vector<Point3Di> loadPoints(size_t i, const PointFilter& filter) const override
        {
            std::vector<Point3Di> points;
            Reader reader(config_.file, options_);
            if (!reader.isOpen())
            {
                std::cerr << "cannot open '" << config_.file.string() << "': " << reader.error() << "\n";
                return points;
            }

            size_t total = 0;
            typename Reader::Callbacks callbacks;
            callbacks.onPointCloud = [&](uint64_t, std::vector<rosbags::McapPoint>&& cloud)
            {
                total += cloud.size();
                for (const auto& p : cloud)
                {
                    if (filter.omit_timestamp_zero && p.timestamp == 0.0)
                        continue;
                    const double r = std::sqrt(static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y);
                    if (r <= filter.xy_inner || r >= filter.xy_outer)
                        continue;
                    Point3Di q{};
                    q.point = Eigen::Vector3d(p.x, p.y, p.z);
                    q.timestamp = p.timestamp;
                    q.intensity = p.intensity;
                    q.lidarid = 0;
                    points.push_back(q);
                }
            };
            if (!reader.read(callbacks, window(i)))
                std::cerr << "reading '" << chunkName(i) << "' failed: " << reader.error() << "\n";
            if (!reader.lidarLayoutWarning().empty())
                std::cerr << "'" << config_.file.string() << "': " << reader.lidarLayoutWarning() << "\n";

            std::sort(
                points.begin(),
                points.end(),
                [](const Point3Di& a, const Point3Di& b)
                {
                    return a.timestamp < b.timestamp;
                });
            std::cout << chunkName(i) << ": " << total << " - " << total - points.size() << " = " << points.size() << std::endl;
            return points;
        }

        ChunkStamp stamp(size_t) const override
        {
            return file_stamp(config_.file);
        }

        void loadImu(Imu& imu_data, std::vector<InputFileSpan>& imuSpans) const override
        {
            std::cout << "loading IMU data from '" << config_.file.string() << "' ..\n";
            Reader reader(config_.file, options_);
            if (!reader.isOpen())
            {
                std::cerr << "cannot open '" << config_.file.string() << "': " << reader.error() << "\n";
                return;
            }
            const size_t first = imu_data.size();
            typename Reader::Callbacks callbacks;
            callbacks.onImu = [&](rosbags::McapImuSample s)
            {
                rosbags::applyImuTransform(imu_transform_, s);
                imu_data.emplace_back(
                    std::pair(s.timestamp, 0.0), Eigen::Vector3f(s.gyro_x, s.gyro_y, s.gyro_z), Eigen::Vector3f(s.acc_x, s.acc_y, s.acc_z));
            };
            if (!reader.read(callbacks))
                std::cerr << "reading IMU from '" << config_.file.string() << "' failed: " << reader.error() << "\n";

            InputFileSpan span{ config_.file.string() };
            for (size_t k = first; k < imu_data.size(); ++k)
            {
                const double t = std::get<0>(imu_data[k]).first;
                span.first = span.count == 0 ? t : std::min(span.first, t);
                span.last = span.count == 0 ? t : std::max(span.last, t);
                span.count++;
            }
            imuSpans.push_back(span);
        }

    private:
        rosbags::LogTimeRange window(size_t i) const
        {
            rosbags::LogTimeRange w;
            w.start_ns = span_.start_ns + i * chunk_ns_;
            w.end_ns = i + 1 == chunk_count_ ? span_.end_ns : w.start_ns + chunk_ns_;
            return w;
        }

        BagSourceConfig config_;
        rosbags::McapReaderOptions options_;
        rosbags::ImuTransform imu_transform_;
        rosbags::LogTimeRange span_;
        uint64_t chunk_ns_ = 1;
        size_t chunk_count_ = 1;
    };

    template<typename Reader>
    std::shared_ptr<RecordingSource> open_with(const BagSourceConfig& config, std::string& error)
    {
        Reader reader(config.file, reader_options(config));
        if (!reader.isOpen())
        {
            error = reader.error();
            return nullptr;
        }
        if (reader.topics().lidar.empty())
        {
            error = "no lidar topic (sensor_msgs/PointCloud2 or livox_ros_driver/CustomMsg) in the file";
            return nullptr;
        }
        if (reader.topics().imu.empty())
        {
            error = "no IMU topic (sensor_msgs/Imu) in the file";
            return nullptr;
        }
        const auto span = reader.logTimeSpan();
        if (!span)
        {
            error = "the file has no index, so its time windows cannot be read on their own (reindex it: `rosbag reindex` for a "
                    "ROS1 bag, `mcap recover` for an MCAP file)";
            return nullptr;
        }
        return std::make_shared<BagSource<Reader>>(config, *span);
    }

    template<typename Reader>
    std::vector<BagChannel> channels_of(const fs::path& file, std::string& error)
    {
        Reader reader(file);
        std::vector<BagChannel> channels;
        for (const auto& c : reader.channels())
        {
            BagChannel channel{ c.topic, c.schema_name, c.message_count, BagChannel::Kind::Other };
            const auto type = type_name(c.schema_name);
            if (type == "PointCloud2" || type == "CustomMsg")
                channel.kind = BagChannel::Kind::Lidar;
            else if (type == "Imu")
                channel.kind = BagChannel::Kind::Imu;
            channels.push_back(std::move(channel));
        }
        // a reader that found channels but no unique lidar/IMU topic still lists them; only an unreadable file is an error
        if (channels.empty())
            error = reader.error().empty() ? "the file has no topics" : reader.error();
        return channels;
    }
} // namespace

bool is_bag_file(const fs::path& file)
{
    const std::string ext = lower_extension(file);
    return ext == ".bag" || ext == ".mcap";
}

std::vector<BagChannel> list_bag_channels(const fs::path& file, std::string& error)
{
    error.clear();
    return is_ros1_bag(file) ? channels_of<rosbags::Ros1BagFileReader>(file, error) : channels_of<rosbags::McapFileReader>(file, error);
}

std::shared_ptr<RecordingSource> open_bag_source(const BagSourceConfig& config, std::string& error)
{
    error.clear();
    if (!is_bag_file(config.file))
    {
        error = "not a .bag or .mcap file: " + config.file.string();
        return nullptr;
    }
    if (config.chunk_seconds < 0.0)
    {
        error = "chunk length must not be negative";
        return nullptr;
    }
    return is_ros1_bag(config.file) ? open_with<rosbags::Ros1BagFileReader>(config, error)
                                    : open_with<rosbags::McapFileReader>(config, error);
}
