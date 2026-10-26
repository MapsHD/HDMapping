#pragma once

// ROS1 bag (format 2.0) reading and writing without a ROS installation, plus
// ROS1 serialization of sensor_msgs/PointCloud2 and sensor_msgs/Imu.
//
// Reader: chunk compression none, bz2 and lz4 (roslz4 writes LZ4 frames).
// Writer: uncompressed chunks with per-chunk index records and a full index
// section, so `rosbag info`, `rosbag play` and the Python rosbags package all
// read the result.
//
// Ros1BagFileReader / Ros1BagFileWriter at the end of this header are the ROS1
// counterparts of McapFileReader / McapFileWriter: same options, same
// McapPoint/McapImuSample structs, same PointCloud2 layouts.

#include "McapReader.h" // McapReaderOptions, ResolvedTopics, ChannelInfo, Callbacks
#include "McapWriter.h" // McapPoint, McapImuSample, McapWriterOptions, PointCloudLayout

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace rosbags
{

struct Ros1Connection
{
	uint32_t id{};
	std::string topic;
	std::string type; // e.g. sensor_msgs/PointCloud2
	std::string md5sum;
	std::string message_definition;
};

class Ros1BagReader
{
public:
	explicit Ros1BagReader(const std::filesystem::path& path);

	bool isOpen() const;
	const std::string& error() const;

	// Connections listed in the bag's index section (empty for an unindexed bag
	// until forEachMessage() has seen them inside the chunks).
	const std::map<uint32_t, Ros1Connection>& connections() const;

	// Calls fn for every message in file order. fn returns false to stop early.
	// Returns false on a read or decompression error (see error()).
	using MessageFn = std::function<bool(const Ros1Connection&, uint64_t time_ns, const uint8_t* data, size_t size)>;
	bool forEachMessage(const MessageFn& fn);

private:
	bool readIndex();
	bool parseRecords(const uint8_t* data, size_t size, const MessageFn& fn, bool& stop);

	std::ifstream file_;
	std::string error_;
	uint64_t index_pos_{};
	std::map<uint32_t, Ros1Connection> connections_;
};

class Ros1BagWriter
{
public:
	explicit Ros1BagWriter(const std::filesystem::path& path, size_t chunk_threshold = 768 * 1024);
	~Ros1BagWriter();

	Ros1BagWriter(const Ros1BagWriter&) = delete;
	Ros1BagWriter& operator=(const Ros1BagWriter&) = delete;

	bool isOpen() const;
	const std::string& error() const;

	// Returns the connection id to pass to write().
	uint32_t addConnection(
		const std::string& topic, const std::string& type, const std::string& md5sum, const std::string& message_definition);

	// Writes one ROS1-serialized message; time_ns is the bag record time.
	void write(uint32_t connection, uint64_t time_ns, const std::vector<uint8_t>& data);

	// Writes the last chunk and the index section. Called by the destructor too.
	bool close();

private:
	struct IndexEntry
	{
		uint64_t time_ns;
		uint32_t offset;
	};
	struct ChunkInfo
	{
		uint64_t pos;
		uint64_t start_ns;
		uint64_t end_ns;
		std::map<uint32_t, uint32_t> counts;
	};

	void flushChunk();

	std::ofstream file_;
	std::string error_;
	size_t chunk_threshold_;
	bool closed_{false};
	std::vector<Ros1Connection> connections_;
	std::vector<uint8_t> chunk_;
	std::set<uint32_t> chunk_connections_;
	std::map<uint32_t, std::vector<IndexEntry>> chunk_index_;
	uint64_t chunk_start_ns_{};
	uint64_t chunk_end_ns_{};
	std::vector<ChunkInfo> chunk_infos_;
};

// ---------------------------------------------------------------------------
// sensor_msgs in ROS1 serialization
// ---------------------------------------------------------------------------

struct Ros1PointField
{
	std::string name;
	uint32_t offset{};
	uint8_t datatype{}; // 1 INT8, 2 UINT8, 3 INT16, 4 UINT16, 5 INT32, 6 UINT32, 7 FLOAT32, 8 FLOAT64
	uint32_t count{1};
};

struct Ros1PointCloud2
{
	uint32_t seq{};
	uint64_t stamp_ns{};
	std::string frame_id;
	uint32_t height{1};
	uint32_t width{};
	std::vector<Ros1PointField> fields;
	bool is_bigendian{false};
	uint32_t point_step{};
	uint32_t row_step{};
	std::vector<uint8_t> data;
	bool is_dense{true};
};

struct Ros1Imu
{
	uint32_t seq{};
	uint64_t stamp_ns{};
	std::string frame_id;
	double orientation[4]{0.0, 0.0, 0.0, 1.0}; // x, y, z, w
	double orientation_covariance[9]{-1.0}; // [0] = -1: orientation unknown
	double angular_velocity[3]{};
	double angular_velocity_covariance[9]{};
	double linear_acceleration[3]{};
	double linear_acceleration_covariance[9]{};
};

extern const char* const kRos1PointCloud2Type;
extern const char* const kRos1PointCloud2Md5;
extern const char* const kRos1PointCloud2Definition;
extern const char* const kRos1ImuType;
extern const char* const kRos1ImuMd5;
extern const char* const kRos1ImuDefinition;

bool decodeRos1PointCloud2(const uint8_t* data, size_t size, Ros1PointCloud2& msg);
std::vector<uint8_t> encodeRos1PointCloud2(const Ros1PointCloud2& msg);
bool decodeRos1Imu(const uint8_t* data, size_t size, Ros1Imu& msg);
std::vector<uint8_t> encodeRos1Imu(const Ros1Imu& msg);

// ---------------------------------------------------------------------------
// McapFileReader / McapFileWriter counterparts
// ---------------------------------------------------------------------------

// Reads a ROS1 bag into McapPoint/McapImuSample, like McapFileReader does for
// MCAP: topics are auto-detected or forced the same way (sensor_msgs/PointCloud2
// or livox_ros_driver CustomMsg for the lidar, sensor_msgs/Imu, std_msgs/String),
// and PointCloud2 records go through the same decodePc2Points() as MCAP ones.
class Ros1BagFileReader
{
public:
	using Callbacks = McapFileReader::Callbacks;

	explicit Ros1BagFileReader(const std::filesystem::path& path, const McapReaderOptions& options = {});

	bool isOpen() const;
	const std::string& error() const;
	const ResolvedTopics& topics() const;
	const std::vector<ChannelInfo>& channels() const;
	// `rosbag record` writes messages in the order it received them, so a bag is
	// read in ascending record time.
	bool timeOrdered() const;
	const std::string& lidarLayoutWarning() const;

	// Streams the whole bag through `callbacks`; `log_time_ns` is the bag record
	// time. Returns false on a read error (see error()).
	bool read(const Callbacks& callbacks);

private:
	Ros1BagReader reader_;
	McapReaderOptions options_;
	ResolvedTopics topics_;
	std::vector<ChannelInfo> channels_;
	std::string error_;
	std::string warning_;
	bool open_{false};
};

// Writes McapPoint/McapImuSample to a ROS1 bag, like McapFileWriter does for
// MCAP: same topics, frame ids and PointCloud2 layouts (pointCloudFields()).
class Ros1BagFileWriter
{
public:
	explicit Ros1BagFileWriter(const std::filesystem::path& path, const McapWriterOptions& options = {});

	bool isOpen() const;
	const std::string& error() const;

	void writePointCloud(uint64_t timestamp_ns, const std::vector<McapPoint>& points);
	void writeImuSample(const McapImuSample& imu);
	void writeImu(const std::vector<McapImuSample>& imu);
	void writeSn(uint64_t timestamp_ns, const std::string& data);

	// Finishes the bag's index; the destructor calls it too.
	bool close();

private:
	Ros1BagWriter bag_;
	McapWriterOptions options_;
	uint32_t lidar_{}, imu_{};
	std::optional<uint32_t> sn_;
	uint32_t lidar_seq_{}, imu_seq_{};
};

} // namespace rosbags
