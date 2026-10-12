#pragma once
#include "McapWriter.h" // McapPoint, McapImuSample -- the structs MandeyeSessionWriter consumes
#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace rosbags
{

// Ouster sensor description, parsed from the metadata JSON the Ouster SDK /
// Ouster Studio / the sensor's HTTP API (`/api/v1/sensor/metadata`) write next
// to a recording. Only the non-legacy layout (FW 2.4+: config_params,
// lidar_data_format, beam_intrinsics, ...) is understood.
struct OusterMetadata
{
	std::string prod_sn;
	std::string prod_line;
	std::string firmware;
	uint64_t initialization_id = 0;
	bool has_initialization_id = false;

	uint16_t udp_port_lidar = 7502;
	uint16_t udp_port_imu = 7503;
	std::string udp_profile_lidar;
	std::string udp_profile_imu;
	std::string header_type;

	int pixels_per_column = 0;
	int columns_per_packet = 0;
	int columns_per_frame = 0;

	// Channel data block layout: bytes per pixel and where the first-return range
	// (mm) and reflectivity live inside it. From `pixel_data_format` when the
	// metadata carries one (FW 4.0 does, including pre-release profiles no SDK
	// knows), otherwise from a table of known profile names.
	int pixel_bytes = 0;
	int range_start_bit = 0;
	int range_width = 0;
	int reflectivity_start_bit = 0;
	int reflectivity_width = 0; // 0 = profile has no reflectivity
	std::array<int, 3> rgb_start_bit{}; // red, green, blue
	int rgb_width = 0; // 0 = profile has no colour

	std::vector<double> beam_altitude_angles_deg;
	std::vector<double> beam_azimuth_angles_deg;
	std::array<double, 16> beam_to_lidar_transform{}; // row-major 4x4, mm
	double lidar_origin_to_beam_origin_mm = 0.0;
	std::array<double, 16> lidar_to_sensor_transform{}; // row-major 4x4, mm
	std::array<double, 16> imu_to_sensor_transform{}; // row-major 4x4, mm

	// Bytes in one lidar UDP payload: packet header + columns + packet footer.
	size_t lidarPacketSize() const;
};

// Parses `json_text`. On failure returns false and fills `error`.
bool parseOusterMetadata(const std::string& json_text, OusterMetadata& out, std::string& error);

struct OusterPcapReaderOptions
{
	// 0 = take the port from the metadata.
	uint16_t lidar_port = 0;
	uint16_t imu_port = 0;

	// Keep every Nth measurement column (1 = all). Thins the cloud horizontally
	// without touching the vertical beam pattern.
	int column_step = 1;
};

struct OusterPcapStats
{
	uint64_t records = 0;
	uint64_t lidar_packets = 0;
	uint64_t imu_packets = 0;
	uint64_t other_packets = 0; // non-UDP, non-IPv4, or a port that is neither lidar nor IMU
	uint64_t bad_size_packets = 0; // on the lidar/IMU port but the wrong length for the metadata's profile
	uint64_t foreign_packets = 0; // header init_id / serial number does not match the metadata
	uint64_t invalid_columns = 0; // column status bit 0 clear (no data, e.g. outside the azimuth window)
	uint64_t reassembled_datagrams = 0;
	uint64_t dropped_fragments = 0; // IPv4 fragments of a datagram that never completed
	double first_lidar_timestamp = 0.0;
	double last_lidar_timestamp = 0.0;
};

// Streams lidar points and IMU samples out of a classic libpcap capture of an
// Ouster sensor (tcpdump, Wireshark saved as "pcap", ouster-cli / Ouster SDK
// recordings). Ethernet / Linux cooked (SLL, SLL2) / raw-IP link types; IPv4
// only; fragmented UDP datagrams (a 128-beam lidar packet is ~20-25 kB, far
// above a 1500-byte MTU) are reassembled.
//
// Points come out in the SENSOR frame (lidar_to_sensor_transform applied) in
// metres, each stamped with its column's timestamp in seconds on the sensor
// clock. Zero-range (no return) pixels are dropped. Intensity carries the 8-bit
// calibrated reflectivity, the scale mandeye's own LAZ files use. On an RGB
// profile each point's colour comes alongside, as the sensor's raw 16-bit words.
//
// IMU samples (LEGACY IMU profile only) are rotated into the sensor frame,
// gyro in rad/s, acceleration in g -- the units mandeye's imuNNNN.csv uses,
// NOT the m/s^2 McapImuSample documents for bags; MandeyeSessionWriter writes
// the values through unchanged.
class OusterPcapReader
{
public:
	struct Callbacks
	{
		// One lidar packet's worth of points, in column (= time) order. `rgb` is
		// parallel to `points` (raw red, green, blue words) on an RGB profile and
		// empty otherwise.
		std::function<void(std::vector<McapPoint>&& points, std::vector<std::array<uint16_t, 3>>&& rgb)> onPoints;
		std::function<void(const McapImuSample&)> onImu;
	};

	OusterPcapReader(const std::filesystem::path& pcap_path, const OusterMetadata& metadata, const OusterPcapReaderOptions& options = {});
	~OusterPcapReader();

	OusterPcapReader(const OusterPcapReader&) = delete;
	OusterPcapReader& operator=(const OusterPcapReader&) = delete;

	bool isOpen() const;
	const std::string& error() const;

	// Reads the whole capture front to back. Returns false on an I/O or format
	// error; packets it cannot use are counted in stats(), not treated as errors.
	bool read(const Callbacks& callbacks);

	// Makes read() return after the packet being processed -- callable from a
	// callback to end the read early.
	void requestStop();

	const OusterPcapStats& stats() const;

private:
	struct Impl;
	std::unique_ptr<Impl> impl_;
};

} // namespace rosbags
