#pragma once

#include "recording_source.h"

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

// No rosbags header here: they re-enable std::byte on MSVC and must come before any standard header (see
// rosbags/McapWriter.h), which a header included by GUI code cannot guarantee.

//! How a ROS1 bag or an MCAP file becomes a step 1 recording. Mirrors the options of bag_to_laz / mcap_to_laz.
struct BagSourceConfig
{
    //! PointCloud2 record layout; Auto reads it from each message's fields.
    enum class LidarLayout
    {
        Auto,
        Hesai,
        Ouster
    };

    std::filesystem::path file;
    std::string lidar_topic; //!< sensor_msgs/PointCloud2 or livox_ros_driver/CustomMsg; empty: the file's only one
    std::string imu_topic; //!< sensor_msgs/Imu; empty: the file's only one
    LidarLayout lidar_layout = LidarLayout::Auto;
    bool imu_acc_in_g = true; //!< false: m/s^2 (the sensor_msgs/Imu convention), divided by standard gravity
    std::array<double, 4> imu_to_lidar{ 0.0, 0.0, 0.0, 1.0 }; //!< rotation IMU frame -> lidar frame, qx qy qz qw
    double chunk_seconds = 10.0; //!< length of one chunk of message log time
};

//! One topic of a bag file, for topic pickers.
struct BagChannel
{
    enum class Kind
    {
        Lidar,
        Imu,
        Other
    };

    std::string topic;
    std::string type; //!< message type as the file names it
    uint64_t message_count = 0; //!< 0 when the file carries no counts
    Kind kind = Kind::Other;
};

//! True for the files open_bag_source reads: .bag (ROS1) and .mcap.
bool is_bag_file(const std::filesystem::path& file);

//! The file's topics. Returns an empty list and sets error when the file cannot be read.
std::vector<BagChannel> list_bag_channels(const std::filesystem::path& file, std::string& error);

//! Opens a ROS1 bag or an MCAP file as a RecordingSource. Chunks are chunk_seconds windows of message log time, read
//! through the file's chunk index, so lazy loading reads each window on its own. Points keep the lidar frame (lidar id 0,
//! no calibration); IMU samples are rotated into the lidar frame and scaled to g. The working directory is the file's
//! folder. Requires an indexed file.
//! Returns null and sets error when the file cannot be used.
std::shared_ptr<RecordingSource> open_bag_source(const BagSourceConfig& config, std::string& error);
