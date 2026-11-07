#define MCAP_IMPLEMENTATION
#include "McapWriter.h"
#include "cdr_serializer.hpp"
#include <algorithm>
#include <cmath>
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

static constexpr const char* kTfMessageSchema = R"(geometry_msgs/TransformStamped[] transforms
================================================================================
MSG: geometry_msgs/TransformStamped
std_msgs/Header header
string child_frame_id
geometry_msgs/Transform transform
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/Transform
geometry_msgs/Vector3 translation
geometry_msgs/Quaternion rotation
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
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
// for the full per-layout offset table). Public so the ROS1 bag writer packs
// exactly the same records.
// ---------------------------------------------------------------------------

std::vector<PointCloudField> pointCloudFields(PointCloudLayout layout)
{
	switch(layout)
	{
	case PointCloudLayout::Velodyne:
		return {
			{"x",         0,  7}, // FLOAT32
			{"y",         4,  7}, // FLOAT32
			{"z",         8,  7}, // FLOAT32
			{"intensity", 12, 7}, // FLOAT32
			{"ring",      16, 4}, // UINT16
			{"time",      18, 8}, // FLOAT64
		};
	case PointCloudLayout::Ouster:
		return {
			{"x",            0,  7}, // FLOAT32
			{"y",            4,  7}, // FLOAT32
			{"z",            8,  7}, // FLOAT32
			{"intensity",    12, 7}, // FLOAT32
			{"t",            16, 6}, // UINT32
			{"reflectivity", 20, 4}, // UINT16
			{"ring",         22, 4}, // UINT16
			{"ambient",      24, 4}, // UINT16
		};
	case PointCloudLayout::Hesai:
		return {
			{"x",         0,  7}, // FLOAT32
			{"y",         4,  7}, // FLOAT32
			{"z",         8,  7}, // FLOAT32
			{"intensity", 12, 7}, // FLOAT32
			{"timestamp", 16, 8}, // FLOAT64
			{"ring",      24, 4}, // UINT16
		};
	case PointCloudLayout::Canonical:
		return {
			{"x",         0,  7}, // FLOAT32
			{"y",         4,  7}, // FLOAT32
			{"z",         8,  7}, // FLOAT32
			{"intensity", 12, 7}, // FLOAT32
			{"ring",      16, 4}, // UINT16
			{"time",      20, 7}, // FLOAT32
			{"t",         24, 6}, // UINT32
		};
	case PointCloudLayout::Generic:
	default:
		return {
			{"x",         0,  7}, // FLOAT32
			{"y",         4,  7}, // FLOAT32
			{"z",         8,  7}, // FLOAT32
			{"intensity", 12, 7}, // FLOAT32
			{"ring",      16, 4}, // UINT16
			{"laser_id",  18, 2}, // UINT8
			{"time",      20, 8}, // FLOAT64
		};
	}
}

uint32_t pointCloudPointStep(PointCloudLayout layout)
{
	switch(layout)
	{
	case PointCloudLayout::Velodyne:
	case PointCloudLayout::Ouster:
	case PointCloudLayout::Hesai: return 26;
	case PointCloudLayout::Canonical:
	case PointCloudLayout::Generic:
	default: return 28;
	}
}

// Packs one point at `d` per `layout`. Done field-by-field to avoid C++ struct
// padding surprises; padding bytes are left as the caller zeroed them.
static void packPoint(uint8_t* d, const McapPoint& p, double t0, PointCloudLayout layout)
{
	const auto put = [&d](const void* v, size_t n)
	{
		std::memcpy(d, v, n);
		d += n;
	};
	put(&p.x, 4);
	put(&p.y, 4);
	put(&p.z, 4);
	put(&p.intensity, 4);

	switch(layout)
	{
	case PointCloudLayout::Velodyne:
	{
		put(&p.ring, 2);
		const double t = p.timestamp - t0;
		put(&t, 8);
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
		put(&t, 4);
		put(&reflectivity, 2);
		put(&p.ring, 2);
		put(&ambient, 2);
		return;
	}
	case PointCloudLayout::Hesai:
	{
		// timestamp: ABSOLUTE seconds (not relative to t0), per Hesai driver convention.
		put(&p.timestamp, 8);
		put(&p.ring, 2);
		return;
	}
	case PointCloudLayout::Canonical:
	{
		put(&p.ring, 2);
		d += 2; // padding
		const double dt = p.timestamp - t0;
		const float time = static_cast<float>(dt);
		const uint32_t t = static_cast<uint32_t>(std::llround(dt * 1e9));
		put(&time, 4);
		put(&t, 4);
		return;
	}
	case PointCloudLayout::Generic:
	default:
	{
		put(&p.ring, 2);
		put(&p.laser_id, 1);
		d += 1; // padding
		const double t = p.timestamp - t0;
		put(&t, 8);
		return;
	}
	}
}

std::vector<uint8_t> packPointCloud(const std::vector<McapPoint>& points, PointCloudLayout layout)
{
	const uint32_t step = pointCloudPointStep(layout);
	std::vector<uint8_t> data(points.size() * step, 0);
	const double t0 = points.empty() ? 0.0 : points.front().timestamp;
	for(size_t i = 0; i < points.size(); ++i)
		packPoint(data.data() + i * step, points[i], t0, layout);
	return data;
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
	const auto fields = pointCloudFields(layout);
	const uint32_t pointStep = pointCloudPointStep(layout);

	w.write_u32(static_cast<uint32_t>(fields.size())); // sequence length
	for(const auto& f : fields)
	{
		w.write_string(f.name);
		w.write_u32(f.offset);
		w.write_u8(f.datatype);
		w.write_u32(1); // count
	}

	w.write_bool(false); // is_bigendian
	w.write_u32(pointStep); // point_step
	w.write_u32(static_cast<uint32_t>(pts.size()) * pointStep); // row_step

	// data[] — uint32 length + raw bytes (no internal CDR padding)
	const auto data = packPointCloud(pts, layout);
	w.write_u32(static_cast<uint32_t>(data.size()));
	w.write_raw(data.data(), data.size());

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

// tf2_msgs/msg/TFMessage carrying a single TransformStamped, matching how a
// real /tf topic publishes one changed transform per message.
static std::vector<uint8_t> serializeTf(
	uint64_t timestamp_ns, const McapTransform& t, const std::string& parent_frame, const std::string& child_frame)
{
	CdrWriter w;

	w.write_u32(1); // transforms[] sequence length

	writeHeader(w, timestamp_ns, parent_frame); // TransformStamped.header
	w.write_string(child_frame);

	// transform.translation
	w.write_f64(t.tx);
	w.write_f64(t.ty);
	w.write_f64(t.tz);

	// transform.rotation
	w.write_f64(t.qx);
	w.write_f64(t.qy);
	w.write_f64(t.qz);
	w.write_f64(t.qw);

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
	mcap::ChannelId tfChannelId{0};
	uint32_t lidarSequence{0};
	uint32_t imuSequence{0};
	uint32_t snSequence{0};
	uint32_t tfSequence{0};
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

	// Register tf2_msgs/msg/TFMessage schema + /tf channel
	mcap::Schema tfSchema("tf2_msgs/msg/TFMessage", "ros2msg",
		{reinterpret_cast<const std::byte*>(kTfMessageSchema),
		 reinterpret_cast<const std::byte*>(kTfMessageSchema) + std::strlen(kTfMessageSchema)});
	impl_->writer.addSchema(tfSchema);

	mcap::Channel tfChannel(impl_->options.tf_topic, "cdr", tfSchema.id);
	impl_->writer.addChannel(tfChannel);
	impl_->tfChannelId = tfChannel.id;

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
	const std::string& frame_id = impl_->options.pointcloud_frame_id.empty() ? impl_->options.frame_id : impl_->options.pointcloud_frame_id;
	auto payload = serializePointCloud2(timestamp_ns, points, frame_id, impl_->options.lidar_layout);

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

void McapFileWriter::writeTfSample(const McapTransform& transform)
{
	if(!isOpen())
		return;

	const uint64_t ts = static_cast<uint64_t>(transform.timestamp * 1e9);
	auto payload = serializeTf(ts, transform, impl_->options.map_frame, impl_->options.frame_id);

	mcap::Message msg;
	msg.channelId = impl_->tfChannelId;
	msg.sequence = impl_->tfSequence++;
	msg.publishTime = ts;
	msg.logTime = ts;
	msg.data = reinterpret_cast<const std::byte*>(payload.data());
	msg.dataSize = payload.size();

	auto s = impl_->writer.write(msg);
	if(!s.ok())
		std::cerr << "McapWriter: tf write error: " << s.message << "\n";
}

void McapFileWriter::writeTf(const std::vector<McapTransform>& transforms)
{
	for(const auto& t : transforms)
		writeTfSample(t);
}

} // namespace rosbags