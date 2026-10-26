#pragma once
#include "McapWriter.h" // McapPoint, McapImuSample
#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace rosbags
{

// Name of the LAS extra-bytes attribute that carries each point's laser ring.
inline constexpr const char* kSessionRingAttribute = "ring";

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
//   red/green/blue (with_rgb only)       LAS 16-bit colour, point format 3
//                                        instead of 1; load_point_cloud() reads
//                                        neither the colour nor the format
//   ring (with_ring only)                McapPoint::ring as a uint16 LAS "extra
//                                        bytes" attribute named "ring" (see
//                                        kSessionRingAttribute). user_data stays the
//                                        lidar id and point_source_ID stays free;
//                                        load_point_cloud() returns it only when
//                                        asked for rings
//
// The IMU csv uses the modern named-column format load_imu() prefers, in the
// column order concatenate_multi_livox already writes, with both timestamps in
// integer nanoseconds (load_imu divides by 1e9).
class MandeyeSessionWriter
{
public:
	// One point's LAS colour (red, green, blue, full 16-bit scale).
	using Rgb = std::array<uint16_t, 3>;

	// Creates `dir` if it does not exist. `with_rgb` writes every chunk as LAS
	// point format 3 (format 1 + RGB) instead of format 1; `with_ring` adds the
	// "ring" extra-bytes attribute.
	explicit MandeyeSessionWriter(const std::filesystem::path& dir, bool with_rgb = false, bool with_ring = false);
	~MandeyeSessionWriter();

	// Non-copyable
	MandeyeSessionWriter(const MandeyeSessionWriter&) = delete;
	MandeyeSessionWriter& operator=(const MandeyeSessionWriter&) = delete;

	bool isOpen() const;
	const std::string& error() const;

	// Opens lidar<index:%04d>.laz. Any chunk still open is closed first.
	bool beginChunk(int index);
	// Without colours, a with_rgb writer stores black.
	bool addPoints(const std::vector<McapPoint>& points);
	// `rgb` is parallel to `points`; only valid on a with_rgb writer.
	bool addPoints(const std::vector<McapPoint>& points, const std::vector<Rgb>& rgb);
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
