#pragma once
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

// Forward-declare mcap types so this header stays light.
namespace mcap
{
class McapWriter;
}

namespace rosbags
{

// One lidar point, laid out for direct copy into the ros2msg PointCloud2
// wire format (see McapFileWriter's class comment below). `timestamp` is
// an absolute timestamp in seconds (e.g. LAS/LAZ gps_time), matching the
// unit HDMapping already stores on Core::Point3Di::timestamp.
struct McapPoint
{
	float x{};
	float y{};
	float z{};
	float intensity{};
	uint16_t ring{};
	uint8_t laser_id{};
	double timestamp{};
};

// One IMU sample. `timestamp` is an absolute timestamp in seconds.
// gyro is rad/s, acc is m/s^2 (sensor_msgs/Imu convention) -- values are
// written through as given, no unit conversion is performed.
struct McapImuSample
{
	double timestamp{};
	float gyro_x{};
	float gyro_y{};
	float gyro_z{};
	float acc_x{};
	float acc_y{};
	float acc_z{};
};

// Selects the sensor_msgs/msg/PointCloud2 field layout the lidar channel is
// written with. The message type is always PointCloud2 -- only the `fields`
// array/point_step (and thus which McapPoint members get written) changes,
// to match what a given downstream consumer expects.
enum class PointCloudLayout
{
	Generic, // x,y,z,intensity,ring,laser_id,time -- point_step = 28 (default)
	Velodyne, // x,y,z,intensity,ring,time -- point_step = 26 (no laser_id)
	Ouster, // x,y,z,intensity,t,reflectivity,ring,ambient -- point_step = 26
	Hesai, // x,y,z,intensity,timestamp,ring -- point_step = 26
};

struct McapWriterOptions
{
	std::string frame_id = "lidar";
	std::string lidar_topic = "/lidar_points";
	std::string imu_topic = "/imu";
	std::string sn_topic = "/lidar_sn";
	PointCloudLayout lidar_layout = PointCloudLayout::Generic;
};

// Writes lidar points and IMU samples to an MCAP file using ros2msg schema
// encoding + CDR serialization.  The resulting file is playable with
// `ros2 bag play` and renderable in Foxglove Studio without any ROS2 runtime.
//
// Default topics:
//   /lidar_points  — sensor_msgs/msg/PointCloud2  (field layout per options().lidar_layout)
//   /imu           — sensor_msgs/msg/Imu
//   /lidar_sn      — std_msgs/msg/String
//
// PointCloud2 field layouts (see PointCloudLayout):
//   Generic (point_step = 28):
//     x          float32  offset  0
//     y          float32  offset  4
//     z          float32  offset  8
//     intensity  float32  offset 12
//     ring       uint16   offset 16
//     laser_id   uint8    offset 18
//     time       float64  offset 20  (seconds, relative to the message stamp)
//   Velodyne (point_step = 26, no laser_id):
//     x          float32  offset  0
//     y          float32  offset  4
//     z          float32  offset  8
//     intensity  float32  offset 12
//     ring       uint16   offset 16
//     time       float64  offset 18  (seconds, relative to the message stamp)
//   Ouster (point_step = 26, matches ouster-ros' field names/types):
//     x             float32  offset  0
//     y             float32  offset  4
//     z             float32  offset  8
//     intensity     float32  offset 12
//     t             uint32   offset 16  (nanoseconds, relative to the message stamp)
//     reflectivity  uint16   offset 20  (McapPoint::intensity clamped to [0,65535]; LAZ has no native reflectivity)
//     ring          uint16   offset 22
//     ambient       uint16   offset 24  (always 0; LAZ has no native ambient-light channel)
//   Hesai (point_step = 26, matches common Hesai driver field names/types):
//     x          float32  offset  0
//     y          float32  offset  4
//     z          float32  offset  8
//     intensity  float32  offset 12
//     timestamp  float64  offset 16  (seconds, ABSOLUTE -- not relative to the message stamp, per Hesai convention)
//     ring       uint16   offset 24
class McapFileWriter
{
public:
	explicit McapFileWriter(const std::filesystem::path& path, const McapWriterOptions& options = {});
	~McapFileWriter();

	// Non-copyable, movable
	McapFileWriter(const McapFileWriter&) = delete;
	McapFileWriter& operator=(const McapFileWriter&) = delete;

	// Write a batch of points as a single PointCloud2 message (layout per options().lidar_layout).
	// timestamp_ns is the message publish time (nanoseconds since epoch).
	void writePointCloud(uint64_t timestamp_ns, const std::vector<McapPoint>& points);

	// Write a single IMU sample as its own /imu message.
	void writeImuSample(const McapImuSample& imu);

	// Write a batch of IMU samples, one /imu message per sample.
	void writeImu(const std::vector<McapImuSample>& imu);

	// Write a string to /lidar_sn (std_msgs/msg/String).
	void writeSn(uint64_t timestamp_ns, const std::string& data);

	bool isOpen() const;

private:
	struct Impl;
	std::unique_ptr<Impl> impl_;
};

} // namespace rosbags