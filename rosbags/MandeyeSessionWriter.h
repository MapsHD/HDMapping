#pragma once
#include "McapWriter.h" // McapPoint, McapImuSample
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace rosbags
{

// Writes a single-lidar mandeye-style recording directory: lidarNNNN.laz /
// imuNNNN.csv chunk pairs, indexed by the same trailing-4-digit convention
// load_data() in apps/lidar_odometry_step_1 matches them by.
//
// The LAZ side deliberately does NOT go through export_laz.h's LazWriter, which
// stores gps_time as `timestamp * 1e9` -- whereas load_point_cloud()
// (lidar_odometry_utils.cpp) reads timestamps straight out of gps_time. Reusing
// it would produce a session whose timestamps are off by a factor of 1e9.
//
// Point record layout written here, matching what load_point_cloud() reads back:
//   gps_time   = McapPoint::timestamp    (absolute seconds, unscaled)
//   user_data  = 0                       (lidar id; single sensor per session)
//   intensity  = McapPoint::intensity    (clamped to uint16)
//
// The IMU csv uses the modern named-column format load_imu() prefers, in the
// column order concatenate_multi_livox already writes, with both timestamps in
// integer nanoseconds (load_imu divides by 1e9).
class MandeyeSessionWriter
{
public:
	// Creates `dir` if it does not exist.
	explicit MandeyeSessionWriter(const std::filesystem::path& dir);
	~MandeyeSessionWriter();

	// Non-copyable
	MandeyeSessionWriter(const MandeyeSessionWriter&) = delete;
	MandeyeSessionWriter& operator=(const MandeyeSessionWriter&) = delete;

	bool isOpen() const;
	const std::string& error() const;

	// Opens lidar<index:%04d>.laz. Any chunk still open is closed first.
	bool beginChunk(int index);
	bool addPoints(const std::vector<McapPoint>& points);
	bool endChunk();

	// Writes imu<index:%04d>.csv in one shot. Samples are written in the order
	// given; an empty vector still produces a header-only file, because
	// load_data() reports a missing csv for every laz it cannot pair.
	bool writeImuChunk(int index, const std::vector<McapImuSample>& samples);

	uint64_t pointsWritten() const;
	uint64_t imuSamplesWritten() const;

private:
	struct Impl;
	std::unique_ptr<Impl> impl_;
};

} // namespace rosbags
