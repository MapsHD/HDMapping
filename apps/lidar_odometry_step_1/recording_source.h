#pragma once

#include "lidar_odometry_utils.h"

#include <cstdint>
#include <filesystem>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

using Imu = std::vector<std::tuple<std::pair<double, double>, Eigen::Vector3f, Eigen::Vector3f>>;

//! Time span of one input file as load_data read it.
struct InputFileSpan
{
    std::string file;
    double first = 0.0; //!< first sample's LiDAR-clock time [s]; 0 when count is 0
    double last = 0.0; //!< last sample's LiDAR-clock time [s]
    size_t count = 0; //!< points or IMU samples read
};

//! Filter every RecordingSource applies to the points it returns (the arguments of load_point_cloud).
struct PointFilter
{
    bool omit_timestamp_zero = true;
    double xy_inner = 0.0;
    double xy_outer = 0.0;
};

//! State of a chunk's backing data, compared to detect a chunk that changed between two reads.
struct ChunkStamp
{
    bool valid = false;
    std::uintmax_t size = 0;
    std::filesystem::file_time_type time{};
};

//! A recording as step 1 reads it: an IMU stream plus raw LiDAR points split into time-ordered chunks.
//! load_data() and the lazy raw cloud loader read input data only through this interface.
class RecordingSource
{
public:
    virtual ~RecordingSource() = default;

    //! Folder that receives preview/, cache/ and the default lio_result_*.
    virtual std::filesystem::path workingDirectory() const = 0;
    //! Number of point chunks.
    virtual size_t chunkCount() const = 0;
    //! Name of chunk i in logs and InputFileSpans (the full path for file-backed chunks).
    virtual std::string chunkName(size_t i) const = 0;
    //! Calibrated points of chunk i that pass the filter, sorted by timestamp. Safe to call from several threads.
    virtual std::vector<Point3Di> loadPoints(size_t i, const PointFilter& filter) const = 0;
    //! Current state of chunk i's backing data.
    virtual ChunkStamp stamp(size_t i) const = 0;
    //! Appends every IMU sample, unsorted, to imu_data; imuSpans receives one span per IMU input file.
    virtual void loadImu(Imu& imu_data, std::vector<InputFileSpan>& imuSpans) const = 0;
};
