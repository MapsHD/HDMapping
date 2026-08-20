#define MCAP_IMPLEMENTATION
#include "McapWriter.h"
#include "cdr_serializer.hpp"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <iterator>
#include <mcap/mcap.hpp> // writer + reader implementations compiled here once

namespace rosbags
{

// ---------------------------------------------------------------------------
// ros2msg schema strings (full definitions including nested types)
// ---------------------------------------------------------------------------

static constexpr const char* kPointCloud2Schema = R"(std_msgs/Header header
uint32 height
uint32 width
sensor_msgs/PointField[] fields
bool    is_bigendian
uint32  point_step
uint32  row_step
uint8[] data
bool    is_dense
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: sensor_msgs/PointField
string name
uint32 offset
uint8  datatype
uint32 count
uint8 INT8=1
uint8 UINT8=2
uint8 INT16=3
uint8 UINT16=4
uint8 INT32=5
uint8 UINT32=6
uint8 FLOAT32=7
uint8 FLOAT64=8
)";

static constexpr const char* kStringSchema = R"(string data
)";

static constexpr const char* kImuSchema = R"(std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
)";

// ---------------------------------------------------------------------------
// PointCloud2 binary layouts, little-endian (see PointCloudLayout in McapWriter.h
// for the full per-layout offset table).
// ---------------------------------------------------------------------------

static constexpr uint32_t kPointStepGeneric = 28;
static constexpr uint32_t kPointStepVelodyne = 26;
static constexpr uint32_t kPointStepOuster = 26;
static constexpr uint32_t kPointStepHesai = 26;

static uint32_t pointStepFor(PointCloudLayout layout)
{
	switch(layout)
	{
	case PointCloudLayout::Velodyne: return kPointStepVelodyne;
	case PointCloudLayout::Ouster: return kPointStepOuster;
	case PointCloudLayout::Hesai: return kPointStepHesai;
	case PointCloudLayout::Generic:
	default: return kPointStepGeneric;
	}
}

// Write one point as raw bytes per `layout`. Done field-by-field to avoid
// C++ struct padding surprises.
static void writePointBytes(CdrWriter& w, const McapPoint& p, double t0, PointCloudLayout layout)
{
	w.write_raw(&p.x, 4);
	w.write_raw(&p.y, 4);
	w.write_raw(&p.z, 4);
	w.write_raw(&p.intensity, 4);

	switch(layout)
	{
	case PointCloudLayout::Velodyne:
	{
		w.write_raw(&p.ring, 2);
		const double t = p.timestamp - t0;
		w.write_raw(&t, 8);
		return;
	}
	case PointCloudLayout::Ouster:
	{
		// t: ns relative to the message stamp, matching ouster-ros' per-point offset.
		const uint32_t t = static_cast<uint32_t>((p.timestamp - t0) * 1e9);
		// LAZ carries no native reflectivity/ambient channel: reflectivity is
		// approximated from intensity, ambient is always 0.
		const uint16_t reflectivity = static_cast<uint16_t>(std::clamp(p.intensity, 0.0f, 65535.0f));
		const uint16_t ambient = 0;
		w.write_raw(&t, 4);
		w.write_raw(&reflectivity, 2);
		w.write_raw(&p.ring, 2);
		w.write_raw(&ambient, 2);
		return;
	}
	case PointCloudLayout::Hesai:
	{
		// timestamp: ABSOLUTE seconds (not relative to t0), per Hesai driver convention.
		w.write_raw(&p.timestamp, 8);
		w.write_raw(&p.ring, 2);
		return;
	}
	case PointCloudLayout::Generic:
	default:
	{
		w.write_raw(&p.ring, 2);
		w.write_raw(&p.laser_id, 1);
		const uint8_t pad = 0;
		w.write_raw(&pad, 1);
		const double t = p.timestamp - t0;
		w.write_raw(&t, 8);
		return;
	}
	}
}

// ---------------------------------------------------------------------------
// CDR helpers
// ---------------------------------------------------------------------------

static void writeHeader(CdrWriter& w, uint64_t timestamp_ns, const std::string& frame_id)
{
	w.write_i32(static_cast<int32_t>(timestamp_ns / 1'000'000'000ULL));
	w.write_u32(static_cast<uint32_t>(timestamp_ns % 1'000'000'000ULL));
	w.write_string(frame_id);
}

static std::vector<uint8_t> serializePointCloud2(
	uint64_t timestamp_ns, const std::vector<McapPoint>& pts, const std::string& frame_id, PointCloudLayout layout)
{
	CdrWriter w;

	// Header
	writeHeader(w, timestamp_ns, frame_id);

	// height / width (unordered: height=1, width=N)
	w.write_u32(1);
	w.write_u32(static_cast<uint32_t>(pts.size()));

	// PointField[]
	struct FieldDef
	{
		const char* name;
		uint32_t offset;
		uint8_t datatype; // 6=UINT32, 7=FLOAT32, 4=UINT16, 2=UINT8, 8=FLOAT64
	};
	static constexpr FieldDef kFieldsGeneric[] = {
		{"x",         0,  7}, // FLOAT32
		{"y",         4,  7}, // FLOAT32
		{"z",         8,  7}, // FLOAT32
		{"intensity", 12, 7}, // FLOAT32
		{"ring",      16, 4}, // UINT16
		{"laser_id",  18, 2}, // UINT8
		{"time",      20, 8}, // FLOAT64
	};
	static constexpr FieldDef kFieldsVelodyne[] = {
		{"x",         0,  7}, // FLOAT32
		{"y",         4,  7}, // FLOAT32
		{"z",         8,  7}, // FLOAT32
		{"intensity", 12, 7}, // FLOAT32
		{"ring",      16, 4}, // UINT16
		{"time",      18, 8}, // FLOAT64
	};
	static constexpr FieldDef kFieldsOuster[] = {
		{"x",            0,  7}, // FLOAT32
		{"y",            4,  7}, // FLOAT32
		{"z",            8,  7}, // FLOAT32
		{"intensity",    12, 7}, // FLOAT32
		{"t",            16, 6}, // UINT32
		{"reflectivity", 20, 4}, // UINT16
		{"ring",         22, 4}, // UINT16
		{"ambient",      24, 4}, // UINT16
	};
	static constexpr FieldDef kFieldsHesai[] = {
		{"x",         0,  7}, // FLOAT32
		{"y",         4,  7}, // FLOAT32
		{"z",         8,  7}, // FLOAT32
		{"intensity", 12, 7}, // FLOAT32
		{"timestamp", 16, 8}, // FLOAT64
		{"ring",      24, 4}, // UINT16
	};

	const FieldDef* fields = kFieldsGeneric;
	uint32_t nFields = std::size(kFieldsGeneric);
	switch(layout)
	{
	case PointCloudLayout::Velodyne:
		fields = kFieldsVelodyne;
		nFields = std::size(kFieldsVelodyne);
		break;
	case PointCloudLayout::Ouster:
		fields = kFieldsOuster;
		nFields = std::size(kFieldsOuster);
		break;
	case PointCloudLayout::Hesai:
		fields = kFieldsHesai;
		nFields = std::size(kFieldsHesai);
		break;
	case PointCloudLayout::Generic:
	default: break;
	}
	const uint32_t pointStep = pointStepFor(layout);

	w.write_u32(nFields); // sequence length
	for(uint32_t i = 0; i < nFields; ++i)
	{
		w.write_string(fields[i].name);
		w.write_u32(fields[i].offset);
		w.write_u8(fields[i].datatype);
		w.write_u32(1); // count
	}

	w.write_bool(false); // is_bigendian
	w.write_u32(pointStep); // point_step
	w.write_u32(static_cast<uint32_t>(pts.size()) * pointStep); // row_step

	// data[] — uint32 length + raw bytes (no internal CDR padding)
	const uint32_t dataBytes = static_cast<uint32_t>(pts.size()) * pointStep;
	w.write_u32(dataBytes);

	const double t0 = pts.empty() ? 0.0 : pts.front().timestamp;
	for(const auto& p : pts)
		writePointBytes(w, p, t0, layout);

	w.write_bool(true); // is_dense
	return w.data();
}

static std::vector<uint8_t> serializeImu(uint64_t timestamp_ns, const McapImuSample& imu, const std::string& frame_id)
{
	CdrWriter w;
	writeHeader(w, timestamp_ns, frame_id);

	// orientation quaternion — identity (no orientation data from lidar IMU)
	w.write_f64(0.0); // x
	w.write_f64(0.0); // y
	w.write_f64(0.0); // z
	w.write_f64(1.0); // w

	// orientation_covariance[9] — -1 in [0] signals "unknown"
	w.write_f64(-1.0);
	for(int i = 1; i < 9; ++i)
		w.write_f64(0.0);

	// angular_velocity (rad/s)
	w.write_f64(static_cast<double>(imu.gyro_x));
	w.write_f64(static_cast<double>(imu.gyro_y));
	w.write_f64(static_cast<double>(imu.gyro_z));

	// angular_velocity_covariance[9]
	for(int i = 0; i < 9; ++i)
		w.write_f64(0.0);

	// linear_acceleration (m/s²)
	w.write_f64(static_cast<double>(imu.acc_x));
	w.write_f64(static_cast<double>(imu.acc_y));
	w.write_f64(static_cast<double>(imu.acc_z));

	// linear_acceleration_covariance[9]
	for(int i = 0; i < 9; ++i)
		w.write_f64(0.0);

	return w.data();
}

// ---------------------------------------------------------------------------
// Impl
// ---------------------------------------------------------------------------

struct McapFileWriter::Impl
{
	mcap::McapWriter writer;
	mcap::ChannelId lidarChannelId{0};
	mcap::ChannelId imuChannelId{0};
	mcap::ChannelId snChannelId{0};
	uint32_t lidarSequence{0};
	uint32_t imuSequence{0};
	uint32_t snSequence{0};
	McapWriterOptions options;
	bool open{false};
};

McapFileWriter::McapFileWriter(const std::filesystem::path& path, const McapWriterOptions& options)
	: impl_(std::make_unique<Impl>())
{
	impl_->options = options;

	mcap::McapWriterOptions opts("ros2");
	opts.compression = mcap::Compression::None;
	opts.chunkSize = 128ULL * 1024 * 1024;

	auto status = impl_->writer.open(path.string(), opts);
	if(!status.ok())
	{
		std::cerr << "McapWriter: failed to open " << path << ": " << status.message << "\n";
		return;
	}

	// Register PointCloud2 schema (shared by both PointCloudLayout variants --
	// only the runtime `fields`/point_step differ, not the message schema itself)
	mcap::Schema pc2Schema("sensor_msgs/msg/PointCloud2",
						   "ros2msg",
						   {reinterpret_cast<const std::byte*>(kPointCloud2Schema),
							reinterpret_cast<const std::byte*>(kPointCloud2Schema) + std::strlen(kPointCloud2Schema)});
	impl_->writer.addSchema(pc2Schema);

	// Register Imu schema
	mcap::Schema imuSchema(
		"sensor_msgs/msg/Imu",
		"ros2msg",
		{reinterpret_cast<const std::byte*>(kImuSchema), reinterpret_cast<const std::byte*>(kImuSchema) + std::strlen(kImuSchema)});
	impl_->writer.addSchema(imuSchema);

	// Register channels
	mcap::Channel lidarChannel(impl_->options.lidar_topic, "cdr", pc2Schema.id);
	impl_->writer.addChannel(lidarChannel);
	impl_->lidarChannelId = lidarChannel.id;

	mcap::Channel imuChannel(impl_->options.imu_topic, "cdr", imuSchema.id);
	impl_->writer.addChannel(imuChannel);
	impl_->imuChannelId = imuChannel.id;

	// Register std_msgs/msg/String schema + /lidar_sn channel
	mcap::Schema strSchema("std_msgs/msg/String", "ros2msg",
		{reinterpret_cast<const std::byte*>(kStringSchema),
		 reinterpret_cast<const std::byte*>(kStringSchema) + std::strlen(kStringSchema)});
	impl_->writer.addSchema(strSchema);

	mcap::Channel snChannel(impl_->options.sn_topic, "cdr", strSchema.id);
	impl_->writer.addChannel(snChannel);
	impl_->snChannelId = snChannel.id;

	impl_->open = true;
}

McapFileWriter::~McapFileWriter()
{
	if(impl_ && impl_->open)
		impl_->writer.close();
}

bool McapFileWriter::isOpen() const
{
	return impl_ && impl_->open;
}

void McapFileWriter::writeSn(uint64_t timestamp_ns, const std::string& data)
{
	if(!isOpen())
		return;

	CdrWriter w;
	w.write_string(data);

	const auto& payload = w.data();
	mcap::Message msg;
	msg.channelId   = impl_->snChannelId;
	msg.sequence    = impl_->snSequence++;
	msg.publishTime = timestamp_ns;
	msg.logTime     = timestamp_ns;
	msg.data        = reinterpret_cast<const std::byte*>(payload.data());
	msg.dataSize    = payload.size();

	auto status = impl_->writer.write(msg);
	if(!status.ok())
		std::cerr << "McapWriter: writeSn error: " << status.message << "\n";
}

void McapFileWriter::writePointCloud(uint64_t timestamp_ns, const std::vector<McapPoint>& points)
{
	if(!isOpen() || points.empty())
		return;
        std::cerr << "McapWriter:  " << points.size() << " points\n";
	auto payload = serializePointCloud2(timestamp_ns, points, impl_->options.frame_id, impl_->options.lidar_layout);

	mcap::Message msg;
	msg.channelId = impl_->lidarChannelId;
	msg.sequence = impl_->lidarSequence++;
	msg.publishTime = timestamp_ns;
	msg.logTime = timestamp_ns;
	msg.data = reinterpret_cast<const std::byte*>(payload.data());
	msg.dataSize = payload.size();

	auto status = impl_->writer.write(msg);
	if(!status.ok())
		std::cerr << "McapWriter: write error: " << status.message << "\n";
}

void McapFileWriter::writeImuSample(const McapImuSample& sample)
{
	if(!isOpen())
		return;

	const uint64_t ts = static_cast<uint64_t>(sample.timestamp * 1e9);
	auto payload = serializeImu(ts, sample, impl_->options.frame_id);

	mcap::Message msg;
	msg.channelId = impl_->imuChannelId;
	msg.sequence = impl_->imuSequence++;
	msg.publishTime = ts;
	msg.logTime = ts;
	msg.data = reinterpret_cast<const std::byte*>(payload.data());
	msg.dataSize = payload.size();

	auto s = impl_->writer.write(msg);
	if(!s.ok())
		std::cerr << "McapWriter: IMU write error: " << s.message << "\n";
}

void McapFileWriter::writeImu(const std::vector<McapImuSample>& imu)
{
	for(const auto& sample : imu)
		writeImuSample(sample);
}

} // namespace rosbags