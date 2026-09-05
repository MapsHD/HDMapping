#pragma once
#include "McapWriter.h" // McapPoint, McapImuSample -- the same structs the writer consumes
#include "cdr_serializer.hpp" // Pc2Preset
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace rosbags
{

// Topic selection. An empty name means "auto-detect": the reader looks for
// exactly one channel whose schema is the matching ros2 message type. More than
// one candidate is an error for the lidar/IMU streams (picking one silently
// would quietly export the wrong sensor) but not for the serial-number stream,
// which is cosmetic and simply stays unresolved.
struct McapReaderOptions
{
	std::string lidar_topic; // sensor_msgs/msg/PointCloud2
	std::string imu_topic; // sensor_msgs/msg/Imu
	std::string sn_topic; // std_msgs/msg/String

	// How PointCloud2 point records are unpacked. Auto reads the layout from each
	// message's own `fields` array and is right for any bag that fills it in
	// honestly; a preset forces a known driver layout for bags that do not.
	Pc2Preset lidar_preset = Pc2Preset::Auto;
};

// The topics actually in use after auto-detection; empty means "not present".
struct ResolvedTopics
{
	std::string lidar;
	std::string imu;
	std::string sn;
};

// One channel as found in the file, for --list output and diagnostics.
struct ChannelInfo
{
	std::string topic;
	std::string schema_name;
	std::string message_encoding;
	uint64_t message_count = 0; // 0 if the file carries no statistics record
};

// Reads ros2msg/CDR-encoded MCAP files back into the McapPoint/McapImuSample
// structs McapFileWriter writes, i.e. the inverse of McapFileWriter.
//
// Decoding is delegated to decodePc2()/decodeImu()/decodeSn() in
// cdr_serializer.hpp, which parse the PointCloud2 `fields` array at runtime and
// therefore handle every PointCloudLayout the writer can emit.
//
// Messages are delivered through callbacks rather than returned in bulk: a real
// recording is far too large to hold in memory, so callers see one message at a
// time. Delivery is in ascending log time when the file has message indexes
// (see timeOrdered()), otherwise in file order.
class McapFileReader
{
public:
	explicit McapFileReader(const std::filesystem::path& path, const McapReaderOptions& options = {});
	~McapFileReader();

	// Non-copyable
	McapFileReader(const McapFileReader&) = delete;
	McapFileReader& operator=(const McapFileReader&) = delete;

	// False if the file could not be opened or a requested/auto-detected topic
	// could not be resolved; error() then says why.
	bool isOpen() const;
	const std::string& error() const;

	const ResolvedTopics& topics() const;
	const std::vector<ChannelInfo>& channels() const;

	// True when read() delivers messages in ascending log time. False means the
	// file has no message indexes and messages arrive in file order instead.
	bool timeOrdered() const;

	// The first point-cloud decoding complaint seen during read(), or empty.
	// Most importantly, this is where a forced lidar_preset that disagrees with
	// the file's own field descriptions gets reported: decoding still proceeds
	// with the preset, so the caller must surface this rather than ignore it.
	const std::string& lidarLayoutWarning() const;

	// Any callback may be left empty; its topic is then skipped without being
	// decoded.
	//
	// `log_time_ns` is the message's MCAP log time -- the recorder's clock, not the
	// sensor clock the payload timestamps are on. Prefer
	// McapPoint/McapImuSample::timestamp for anything that has to agree with the
	// data.
	//
	// onPointCloud is only called with a non-empty vector.
	struct Callbacks
	{
		std::function<void(uint64_t log_time_ns, std::vector<McapPoint>&&)> onPointCloud;
		std::function<void(const McapImuSample&)> onImu;
		std::function<void(uint64_t log_time_ns, const std::string&)> onSn;
	};

	// Streams the whole file through `callbacks`. Returns false on a read error
	// (see error()); a file with no matching messages is not an error.
	bool read(const Callbacks& callbacks);

private:
	struct Impl;
	std::unique_ptr<Impl> impl_;
};

} // namespace rosbags
