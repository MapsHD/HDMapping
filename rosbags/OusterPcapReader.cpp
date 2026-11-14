#include "OusterPcapReader.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <map>
#include <numbers>
#include <tuple>

namespace rosbags
{

namespace
{

using nlohmann::json;

// Non-legacy lidar packet framing ("header_type": "STANDARD").
constexpr size_t kPacketHeaderBytes = 32;
constexpr size_t kPacketFooterBytes = 32;
constexpr size_t kColumnHeaderBytes = 12; // u64 timestamp ns, u16 measurement_id, u16 status
constexpr size_t kLegacyImuPacketBytes = 48;
// ACCEL32_GYRO32_NMEA IMU packet: the lidar packet header, a 100-byte NMEA
// block (ignored), N measurements, the packet footer.
constexpr size_t kNmeaBlockBytes = 100;
constexpr size_t kImuMeasurementBytes = 36; // u64 timestamp ns, u16 measurement_id, u16 status, 6 x f32
constexpr double kStandardGravity = 9.80665;

// Profiles whose channel data block is known without a pixel_data_format. All of
// them carry the first return in word 0 -- range [18:0] in mm, reflectivity
// [31:24] -- then signal and NIR in word 1, and differ only in what follows:
// the RGB ones put red, green, blue in bits [79:64], [95:80], [111:96].
struct KnownProfile
{
	const char* name;
	int pixel_bytes;
	bool rgb;
};
constexpr KnownProfile kKnownProfiles[] = {
	{"RNG19_RFL8_SIG16_NIR16", 12, false},
	{"RNG19_RFL8_SIG16_NIR16_RGB16", 16, true},
	// FW 4.0.0 pre-release name of the RGB profile: two RGB sets, 20 bytes.
	{"RNG19_RFL8_SIG16_NIR16_R16_G16_B16", 20, true},
};

// pcap link-layer types handled by linkPayload().
constexpr uint32_t kLinkNull = 0;
constexpr uint32_t kLinkEthernet = 1;
constexpr uint32_t kLinkRaw = 101;
constexpr uint32_t kLinkLoop = 108;
constexpr uint32_t kLinkLinuxSll = 113;
constexpr uint32_t kLinkIpv4 = 228;
constexpr uint32_t kLinkLinuxSll2 = 276;

// Anything bigger is a corrupt record header, not a packet: the largest Ouster
// lidar packet is ~33 kB and a reassembled IPv4 datagram is at most 64 kB.
constexpr uint32_t kMaxRecordBytes = 256 * 1024;
// Datagrams being reassembled at once; Ouster streams interleave at most a
// lidar and an IMU datagram, so anything beyond this is fragments whose
// siblings were never captured.
constexpr size_t kMaxPendingDatagrams = 64;

uint16_t le16(const uint8_t* p)
{
	return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

uint64_t le64(const uint8_t* p)
{
	uint64_t v = 0;
	for(int i = 7; i >= 0; --i)
		v = (v << 8) | p[i];
	return v;
}

float leFloat(const uint8_t* p)
{
	const uint32_t bits = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
						  (static_cast<uint32_t>(p[3]) << 24);
	float v;
	std::memcpy(&v, &bits, sizeof(v));
	return v;
}

uint16_t be16(const uint8_t* p)
{
	return static_cast<uint16_t>((p[0] << 8) | p[1]);
}

uint32_t be32(const uint8_t* p)
{
	return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) | (static_cast<uint32_t>(p[2]) << 8) | p[3];
}

uint32_t swap32(uint32_t v)
{
	return ((v & 0xFF) << 24) | ((v & 0xFF00) << 8) | ((v >> 8) & 0xFF00) | (v >> 24);
}

// Little-endian bit field of a channel data block, as pixel_data_format
// describes it (bit 0 = LSB of byte 0).
uint32_t bitField(const uint8_t* pixel, int start_bit, int width)
{
	const uint8_t* p = pixel + start_bit / 8;
	const int shift = start_bit % 8;
	const int bytes = (shift + width + 7) / 8;
	uint64_t v = 0;
	for(int i = bytes - 1; i >= 0; --i)
		v = (v << 8) | p[i];
	return static_cast<uint32_t>((v >> shift) & ((uint64_t{1} << width) - 1));
}

bool readMatrix(const json& parent, const char* key, std::array<double, 16>& out, std::string& error)
{
	if(!parent.contains(key) || !parent[key].is_array() || parent[key].size() != 16)
	{
		error = std::string("metadata: '") + key + "' must be an array of 16 numbers";
		return false;
	}
	for(size_t i = 0; i < 16; ++i)
		out[i] = parent[key][i].get<double>();
	return true;
}

bool readAngles(const json& parent, const char* key, size_t expected, std::vector<double>& out, std::string& error)
{
	if(!parent.contains(key) || !parent[key].is_array() || parent[key].size() != expected)
	{
		error = std::string("metadata: beam_intrinsics.") + key + " must hold one angle per beam (" + std::to_string(expected) + ")";
		return false;
	}
	out = parent[key].get<std::vector<double>>();
	return true;
}

} // namespace

size_t OusterMetadata::lidarPacketSize() const
{
	return kPacketHeaderBytes + static_cast<size_t>(columns_per_packet) * (kColumnHeaderBytes + static_cast<size_t>(pixels_per_column) * pixel_bytes) +
		   kPacketFooterBytes;
}

size_t OusterMetadata::imuPacketSize() const
{
	if(udp_profile_imu == "LEGACY")
		return kLegacyImuPacketBytes;
	if(udp_profile_imu == "ACCEL32_GYRO32_NMEA")
		return kPacketHeaderBytes + kNmeaBlockBytes + static_cast<size_t>(imu_measurements_per_packet) * kImuMeasurementBytes + kPacketFooterBytes;
	return 0;
}

bool parseOusterMetadata(const std::string& json_text, OusterMetadata& out, std::string& error)
{
	OusterMetadata m;
	try
	{
		const json j = json::parse(json_text);
		if(!j.contains("lidar_data_format") || !j.contains("beam_intrinsics") || !j.contains("config_params"))
		{
			error = "metadata: missing lidar_data_format / beam_intrinsics / config_params -- only the non-legacy "
					"(FW 2.4+) metadata layout is supported";
			return false;
		}

		if(j.contains("sensor_info"))
		{
			const json& info = j["sensor_info"];
			m.prod_sn = info.value("prod_sn", "");
			m.prod_line = info.value("prod_line", "");
			m.firmware = info.value("image_rev", "");
			if(info.contains("initialization_id") && info["initialization_id"].is_number_unsigned())
			{
				m.initialization_id = info["initialization_id"].get<uint64_t>();
				m.has_initialization_id = true;
			}
		}

		const json& config = j["config_params"];
		m.udp_port_lidar = config.value("udp_port_lidar", uint16_t{7502});
		m.udp_port_imu = config.value("udp_port_imu", uint16_t{7503});
		m.udp_profile_imu = config.value("udp_profile_imu", std::string("LEGACY"));
		if(j.contains("imu_data_format"))
			m.imu_measurements_per_packet = j["imu_data_format"].value("imu_measurements_per_packet", m.imu_measurements_per_packet);
		if(m.imu_measurements_per_packet <= 0)
		{
			error = "metadata: imu_data_format.imu_measurements_per_packet must be positive";
			return false;
		}

		const json& format = j["lidar_data_format"];
		m.pixels_per_column = format.at("pixels_per_column").get<int>();
		m.columns_per_packet = format.at("columns_per_packet").get<int>();
		m.columns_per_frame = format.at("columns_per_frame").get<int>();
		m.udp_profile_lidar = format.value("udp_profile_lidar", config.value("udp_profile_lidar", std::string("LEGACY")));
		m.header_type = format.value("header_type", std::string("STANDARD"));
		if(m.pixels_per_column <= 0 || m.columns_per_packet <= 0 || m.columns_per_frame <= 0)
		{
			error = "metadata: lidar_data_format has a non-positive pixels_per_column / columns_per_packet / columns_per_frame";
			return false;
		}
		if(m.header_type != "STANDARD")
		{
			error = "metadata: header_type '" + m.header_type + "' is not supported (STANDARD only)";
			return false;
		}
		if(m.udp_profile_lidar == "LEGACY")
		{
			error = "metadata: the LEGACY lidar profile is not supported -- reconfigure the sensor to RNG19_RFL8_SIG16_NIR16";
			return false;
		}

		// The sensor's own description of the channel data block wins over the
		// name table: it is what lets a pre-release profile be read at all.
		if(j.contains("pixel_data_format"))
		{
			const json& pdf = j["pixel_data_format"];
			m.pixel_bytes = pdf.at("pixel_bytes").get<int>();
			bool has_range = false;
			std::array<int, 3> rgb_width{};
			for(const auto& field : pdf.at("fields"))
			{
				const std::string name = field.at("field_name").get<std::string>();
				const int start = field.at("start_bit").get<int>();
				const int width = field.at("width").get<int>();
				// First match only: in a dual-return profile return 1 is listed first.
				// Range fields are in mm (e.g. range_capped_mm_0).
				if(!has_range && name.starts_with("range"))
				{
					m.range_start_bit = start;
					m.range_width = width;
					has_range = true;
				}
				else if(m.reflectivity_width == 0 && name.starts_with("reflectivity"))
				{
					m.reflectivity_start_bit = start;
					m.reflectivity_width = width;
				}
				else
				{
					// Exact names: the pre-release RGB profile also lists a second
					// set, rgb_r1/rgb_g1/rgb_b1, which carries no image.
					static constexpr const char* kRgbNames[3] = {"rgb_r", "rgb_g", "rgb_b"};
					for(int c = 0; c < 3; ++c)
					{
						if(name == kRgbNames[c])
						{
							m.rgb_start_bit[c] = start;
							rgb_width[c] = width;
						}
					}
				}
			}
			if(!has_range)
			{
				error = "metadata: pixel_data_format has no range field";
				return false;
			}
			// Colour only when all three channels are there and fit a LAS 16-bit channel.
			if(rgb_width[0] > 0 && rgb_width[0] <= 16 && rgb_width[1] == rgb_width[0] && rgb_width[2] == rgb_width[0])
				m.rgb_width = rgb_width[0];
		}
		else
		{
			const auto it = std::find_if(
				std::begin(kKnownProfiles),
				std::end(kKnownProfiles),
				[&](const KnownProfile& p)
				{
					return m.udp_profile_lidar == p.name;
				});
			if(it == std::end(kKnownProfiles))
			{
				std::string known;
				for(const auto& p : kKnownProfiles)
					known += std::string(known.empty() ? "" : ", ") + p.name;
				error = "metadata: lidar profile '" + m.udp_profile_lidar + "' is not supported and the metadata has no pixel_data_format "
						"to describe it (known: " + known + ")";
				return false;
			}
			m.pixel_bytes = it->pixel_bytes;
			m.range_start_bit = 0;
			m.range_width = 19;
			m.reflectivity_start_bit = 24;
			m.reflectivity_width = 8;
			if(it->rgb)
			{
				m.rgb_start_bit = {64, 80, 96};
				m.rgb_width = 16;
			}
		}
		for(const auto& [start, width] :
			{std::pair{m.range_start_bit, m.range_width},
			 std::pair{m.reflectivity_start_bit, m.reflectivity_width},
			 std::pair{m.rgb_start_bit[0], m.rgb_width},
			 std::pair{m.rgb_start_bit[1], m.rgb_width},
			 std::pair{m.rgb_start_bit[2], m.rgb_width}})
		{
			if(width > 32 || start < 0 || start + width > m.pixel_bytes * 8)
			{
				error = "metadata: a pixel field lies outside the " + std::to_string(m.pixel_bytes) + "-byte channel data block or is wider than 32 bits";
				return false;
			}
		}

		m.rgb_float16 = m.rgb_width == 16;

		const json& beams = j["beam_intrinsics"];
		const size_t beam_count = static_cast<size_t>(m.pixels_per_column);
		if(!readAngles(beams, "beam_altitude_angles", beam_count, m.beam_altitude_angles_deg, error) ||
		   !readAngles(beams, "beam_azimuth_angles", beam_count, m.beam_azimuth_angles_deg, error))
			return false;
		m.lidar_origin_to_beam_origin_mm = beams.value("lidar_origin_to_beam_origin_mm", 0.0);
		if(beams.contains("beam_to_lidar_transform"))
		{
			if(!readMatrix(beams, "beam_to_lidar_transform", m.beam_to_lidar_transform, error))
				return false;
		}
		else
		{
			// Pre-FW 2.5 metadata: only the scalar offset, along the beam's x axis.
			m.beam_to_lidar_transform = {1, 0, 0, m.lidar_origin_to_beam_origin_mm, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
		}

		if(!j.contains("lidar_intrinsics") || !readMatrix(j["lidar_intrinsics"], "lidar_to_sensor_transform", m.lidar_to_sensor_transform, error))
		{
			error = "metadata: lidar_intrinsics.lidar_to_sensor_transform is missing or malformed";
			return false;
		}
		if(!j.contains("imu_intrinsics") || !readMatrix(j["imu_intrinsics"], "imu_to_sensor_transform", m.imu_to_sensor_transform, error))
		{
			error = "metadata: imu_intrinsics.imu_to_sensor_transform is missing or malformed";
			return false;
		}
	} catch(const std::exception& e)
	{
		error = std::string("metadata: ") + e.what();
		return false;
	}

	out = std::move(m);
	return true;
}

struct OusterPcapReader::Impl
{
	OusterMetadata meta;
	OusterPcapReaderOptions options;
	OusterPcapStats stats;
	std::string error;
	bool open = false;
	bool stop = false;
	bool have_lidar_timestamp = false;

	std::ifstream in;
	std::vector<char> buffer;
	bool swapped = false;
	bool nanoseconds = false;
	uint32_t linktype = 0;

	uint64_t prod_sn = 0;
	bool check_sn = false;

	// Per pixel (row-major, beam u, measurement id v): point = direction * range_mm
	// + offset, already in the sensor frame and in metres -- the Ouster SDK's
	// make_xyz_lut() with range_unit = 1 mm.
	std::vector<double> lut_direction;
	std::vector<double> lut_offset;
	std::array<double, 9> imu_rotation{};

	struct PendingDatagram
	{
		std::map<uint32_t, std::vector<uint8_t>> parts; // byte offset -> payload
		uint32_t total = 0; // known once the last fragment arrives
		uint64_t age = 0;
	};
	std::map<std::tuple<uint32_t, uint32_t, uint16_t>, PendingDatagram> pending;
	uint64_t pending_clock = 0;

	std::vector<uint8_t> record;
	std::vector<uint8_t> datagram;
	std::vector<McapPoint> points;
	std::vector<std::array<uint16_t, 3>> rgb;

	bool fail(std::string message)
	{
		if(error.empty())
			error = std::move(message);
		// Release the file: a rejected capture must not stay locked (Windows) for the reader's lifetime.
		in.close();
		open = false;
		return false;
	}

	void buildLut()
	{
		const int w = meta.columns_per_frame;
		const int h = meta.pixels_per_column;
		const auto& b2l = meta.beam_to_lidar_transform;
		const auto& l2s = meta.lidar_to_sensor_transform;
		const double beam_offset_x = b2l[3];
		const double beam_offset_z = b2l[11];
		const double n = std::hypot(beam_offset_x, beam_offset_z);
		constexpr double kDeg = std::numbers::pi / 180.0;
		constexpr double kMmToM = 0.001;

		lut_direction.resize(static_cast<size_t>(w) * h * 3);
		lut_offset.resize(static_cast<size_t>(w) * h * 3);
		for(int u = 0; u < h; ++u)
		{
			const double azimuth = -meta.beam_azimuth_angles_deg[u] * kDeg;
			const double altitude = meta.beam_altitude_angles_deg[u] * kDeg;
			for(int v = 0; v < w; ++v)
			{
				const double encoder = 2.0 * std::numbers::pi - v * (2.0 * std::numbers::pi / w);
				const double dir[3] = {
					std::cos(encoder + azimuth) * std::cos(altitude), std::sin(encoder + azimuth) * std::cos(altitude), std::sin(altitude)};
				const double off[3] = {
					std::cos(encoder) * beam_offset_x - dir[0] * n, std::sin(encoder) * beam_offset_x - dir[1] * n, -dir[2] * n + beam_offset_z};

				const size_t i = (static_cast<size_t>(u) * w + v) * 3;
				for(int r = 0; r < 3; ++r)
				{
					const double* row = &l2s[r * 4];
					lut_direction[i + r] = (row[0] * dir[0] + row[1] * dir[1] + row[2] * dir[2]) * kMmToM;
					lut_offset[i + r] = (row[0] * off[0] + row[1] * off[1] + row[2] * off[2] + row[3]) * kMmToM;
				}
			}
		}
	}

	uint32_t u32(const uint8_t* p) const
	{
		uint32_t v;
		std::memcpy(&v, p, sizeof(v));
		return swapped ? swap32(v) : v;
	}

	// Locates the IPv4 header inside a captured frame. Returns false for anything
	// that is not IPv4 (ARP, IPv6, ...).
	bool linkPayload(const uint8_t* frame, size_t size, size_t& ip_offset) const
	{
		switch(linktype)
		{
		case kLinkEthernet:
		{
			size_t off = 12;
			if(size < off + 2)
				return false;
			uint16_t ethertype = be16(frame + off);
			while(ethertype == 0x8100 || ethertype == 0x88A8 || ethertype == 0x9100) // VLAN / QinQ tags
			{
				off += 4;
				if(size < off + 2)
					return false;
				ethertype = be16(frame + off);
			}
			ip_offset = off + 2;
			return ethertype == 0x0800;
		}
		case kLinkLinuxSll:
			ip_offset = 16;
			return size >= 16 && be16(frame + 14) == 0x0800;
		case kLinkLinuxSll2:
			ip_offset = 20;
			return size >= 20 && be16(frame) == 0x0800;
		case kLinkNull:
		case kLinkLoop:
			ip_offset = 4;
			return size >= 4 && (u32(frame) == 2 || swap32(u32(frame)) == 2); // AF_INET, host order of the capturing machine
		default: // kLinkRaw, kLinkIpv4
			ip_offset = 0;
			return size >= 1 && (frame[0] >> 4) == 4;
		}
	}

	void handleUdp(const uint8_t* udp, size_t size)
	{
		if(size < 8)
		{
			++stats.other_packets;
			return;
		}
		const uint16_t dport = be16(udp + 2);
		const size_t length = std::min<size_t>(std::max<size_t>(be16(udp + 4), 8), size) - 8;
		const uint8_t* payload = udp + 8;

		if(dport == options.lidar_port)
			handleLidar(payload, length);
		else if(dport == options.imu_port && meta.udp_profile_imu == "LEGACY")
			handleLegacyImu(payload, length);
		else if(dport == options.imu_port && meta.udp_profile_imu == "ACCEL32_GYRO32_NMEA")
			handleAccel32Gyro32NmeaImu(payload, length);
		else
			++stats.other_packets;
	}

	// Packet header init_id / serial number differ from the metadata's.
	bool isForeign(const uint8_t* p) const
	{
		const uint64_t init_id = p[4] | (p[5] << 8) | (static_cast<uint64_t>(p[6]) << 16);
		uint64_t sn = 0;
		for(int i = 11; i >= 7; --i)
			sn = (sn << 8) | p[i];
		return (meta.has_initialization_id && init_id != meta.initialization_id) || (check_sn && sn != prod_sn);
	}

	void handleLidar(const uint8_t* p, size_t size)
	{
		if(size != meta.lidarPacketSize())
		{
			++stats.bad_size_packets;
			return;
		}
		if(isForeign(p))
		{
			++stats.foreign_packets;
			return;
		}

		const int w = meta.columns_per_frame;
		const int h = meta.pixels_per_column;
		const size_t column_bytes = kColumnHeaderBytes + static_cast<size_t>(h) * meta.pixel_bytes;
		points.clear();
		points.reserve(static_cast<size_t>(meta.columns_per_packet) * h);
		rgb.clear();
		if(meta.rgb_width)
			rgb.reserve(points.capacity());
		bool any_valid = false;
		for(int c = 0; c < meta.columns_per_packet; ++c)
		{
			const uint8_t* col = p + kPacketHeaderBytes + c * column_bytes;
			const uint64_t timestamp_ns = le64(col);
			const uint16_t measurement_id = le16(col + 8);
			const uint16_t status = le16(col + 10);
			if(!(status & 1) || measurement_id >= w)
			{
				++stats.invalid_columns;
				continue;
			}
			const double t = static_cast<double>(timestamp_ns) * 1e-9;
			if(!have_lidar_timestamp)
			{
				stats.first_lidar_timestamp = t;
				have_lidar_timestamp = true;
			}
			stats.last_lidar_timestamp = std::max(stats.last_lidar_timestamp, t);
			any_valid = true;
			if(measurement_id % options.column_step != 0)
				continue;

			const uint8_t* pixel = col + kColumnHeaderBytes;
			for(int u = 0; u < h; ++u, pixel += meta.pixel_bytes)
			{
				const uint32_t range_mm = bitField(pixel, meta.range_start_bit, meta.range_width);
				if(range_mm == 0)
					continue; // no return
				const size_t i = (static_cast<size_t>(u) * w + measurement_id) * 3;
				McapPoint pt;
				pt.x = static_cast<float>(lut_direction[i] * range_mm + lut_offset[i]);
				pt.y = static_cast<float>(lut_direction[i + 1] * range_mm + lut_offset[i + 1]);
				pt.z = static_cast<float>(lut_direction[i + 2] * range_mm + lut_offset[i + 2]);
				pt.intensity =
					meta.reflectivity_width ? static_cast<float>(bitField(pixel, meta.reflectivity_start_bit, meta.reflectivity_width)) : 0.0f;
				pt.ring = static_cast<uint16_t>(u);
				pt.timestamp = t;
				points.push_back(pt);
				if(meta.rgb_width)
				{
					rgb.push_back(
						{static_cast<uint16_t>(bitField(pixel, meta.rgb_start_bit[0], meta.rgb_width)),
						 static_cast<uint16_t>(bitField(pixel, meta.rgb_start_bit[1], meta.rgb_width)),
						 static_cast<uint16_t>(bitField(pixel, meta.rgb_start_bit[2], meta.rgb_width))});
				}
			}
		}
		if(any_valid)
			++stats.lidar_packets;
		if(!points.empty() && onPoints)
			onPoints(std::move(points), std::move(rgb));
		points = {};
		rgb = {};
	}

	// Rotates one sample (acceleration in g, gyro in rad/s, IMU frame) into the
	// sensor frame and hands it on.
	void emitImu(double timestamp, const double (&acc)[3], const double (&gyro)[3])
	{
		const auto& r = imu_rotation;
		McapImuSample s;
		s.timestamp = timestamp;
		s.acc_x = static_cast<float>(r[0] * acc[0] + r[1] * acc[1] + r[2] * acc[2]);
		s.acc_y = static_cast<float>(r[3] * acc[0] + r[4] * acc[1] + r[5] * acc[2]);
		s.acc_z = static_cast<float>(r[6] * acc[0] + r[7] * acc[1] + r[8] * acc[2]);
		s.gyro_x = static_cast<float>(r[0] * gyro[0] + r[1] * gyro[1] + r[2] * gyro[2]);
		s.gyro_y = static_cast<float>(r[3] * gyro[0] + r[4] * gyro[1] + r[5] * gyro[2]);
		s.gyro_z = static_cast<float>(r[6] * gyro[0] + r[7] * gyro[1] + r[8] * gyro[2]);
		if(onImu)
			onImu(s);
	}

	void handleLegacyImu(const uint8_t* p, size_t size)
	{
		// LEGACY IMU packet: u64 system / accel / gyro read times (ns), then
		// accel x y z (g) and gyro x y z (deg/s) as float32.
		if(size != kLegacyImuPacketBytes)
		{
			++stats.bad_size_packets;
			return;
		}
		constexpr double kDegToRad = std::numbers::pi / 180.0;
		const double acc[3] = {leFloat(p + 24), leFloat(p + 28), leFloat(p + 32)};
		const double gyro[3] = {leFloat(p + 36) * kDegToRad, leFloat(p + 40) * kDegToRad, leFloat(p + 44) * kDegToRad};
		++stats.imu_packets;
		// The gyro read time follows the configured timestamp_mode, like the lidar
		// column timestamps; the system timestamp is a diagnostic counter.
		emitImu(static_cast<double>(le64(p + 16)) * 1e-9, acc, gyro);
	}

	void handleAccel32Gyro32NmeaImu(const uint8_t* p, size_t size)
	{
		// ACCEL32_GYRO32_NMEA (FW 3.1+): standard packet header, NMEA block, then
		// per measurement u64 timestamp (ns, same clock as the lidar columns),
		// u16 measurement_id, u16 status (bit 0 = valid), accel x y z (m/s^2)
		// and gyro x y z (rad/s) as float32.
		if(size != meta.imuPacketSize())
		{
			++stats.bad_size_packets;
			return;
		}
		if(isForeign(p))
		{
			++stats.foreign_packets;
			return;
		}
		++stats.imu_packets;
		const uint8_t* m = p + kPacketHeaderBytes + kNmeaBlockBytes;
		for(int i = 0; i < meta.imu_measurements_per_packet; ++i, m += kImuMeasurementBytes)
		{
			if(!(le16(m + 10) & 1))
				continue;
			const double acc[3] = {
				leFloat(m + 12) / kStandardGravity, leFloat(m + 16) / kStandardGravity, leFloat(m + 20) / kStandardGravity};
			const double gyro[3] = {leFloat(m + 24), leFloat(m + 28), leFloat(m + 32)};
			emitImu(static_cast<double>(le64(m)) * 1e-9, acc, gyro);
		}
	}

	// Feeds one IPv4 fragment in; once every byte of its datagram has arrived,
	// hands the reassembled UDP datagram on and forgets it.
	void handleFragment(const uint8_t* ip, size_t ihl, size_t ip_length)
	{
		const auto key = std::make_tuple(be32(ip + 12), be32(ip + 16), be16(ip + 4));
		const uint16_t flags_offset = be16(ip + 6);
		const bool more_fragments = (flags_offset & 0x2000) != 0;
		const uint32_t offset = static_cast<uint32_t>(flags_offset & 0x1FFF) * 8;
		const size_t length = ip_length - ihl;

		auto [it, inserted] = pending.try_emplace(key);
		PendingDatagram& d = it->second;
		if(inserted)
			d.age = pending_clock++;
		d.parts[offset].assign(ip + ihl, ip + ihl + length);
		if(!more_fragments)
			d.total = offset + static_cast<uint32_t>(length);

		if(d.total != 0)
		{
			uint32_t covered = 0;
			for(const auto& [part_offset, bytes] : d.parts)
			{
				if(part_offset > covered)
					break; // hole
				covered = std::max<uint32_t>(covered, part_offset + static_cast<uint32_t>(bytes.size()));
			}
			if(covered >= d.total)
			{
				datagram.assign(d.total, 0);
				for(const auto& [part_offset, bytes] : d.parts)
				{
					if(part_offset >= d.total)
						break;
					std::memcpy(datagram.data() + part_offset, bytes.data(), std::min<size_t>(bytes.size(), d.total - part_offset));
				}
				pending.erase(it);
				++stats.reassembled_datagrams;
				handleUdp(datagram.data(), datagram.size());
				return;
			}
		}

		if(pending.size() > kMaxPendingDatagrams)
		{
			auto oldest = std::min_element(
				pending.begin(),
				pending.end(),
				[](const auto& a, const auto& b)
				{
					return a.second.age < b.second.age;
				});
			stats.dropped_fragments += oldest->second.parts.size();
			pending.erase(oldest);
		}
	}

	void handleFrame(const uint8_t* frame, size_t size)
	{
		size_t off = 0;
		if(!linkPayload(frame, size, off) || size < off + 20)
		{
			++stats.other_packets;
			return;
		}
		const uint8_t* ip = frame + off;
		const size_t ihl = static_cast<size_t>(ip[0] & 0x0F) * 4;
		const size_t ip_length = std::min<size_t>(be16(ip + 2), size - off);
		if((ip[0] >> 4) != 4 || ihl < 20 || ip_length < ihl || ip[9] != 17 /* UDP */)
		{
			++stats.other_packets;
			return;
		}
		if((be16(ip + 6) & 0x3FFF) != 0) // more-fragments flag or a non-zero offset
			handleFragment(ip, ihl, ip_length);
		else
			handleUdp(ip + ihl, ip_length - ihl);
	}

	decltype(Callbacks::onPoints) onPoints;
	std::function<void(const McapImuSample&)> onImu;
};

OusterPcapReader::OusterPcapReader(const std::filesystem::path& pcap_path, const OusterMetadata& metadata, const OusterPcapReaderOptions& options)
	: impl_(std::make_unique<Impl>())
{
	Impl& d = *impl_;
	d.meta = metadata;
	d.options = options;
	if(d.options.lidar_port == 0)
		d.options.lidar_port = metadata.udp_port_lidar;
	if(d.options.imu_port == 0)
		d.options.imu_port = metadata.udp_port_imu;
	d.options.column_step = std::max(1, d.options.column_step);

	if(!metadata.prod_sn.empty())
	{
		try
		{
			d.prod_sn = std::stoull(metadata.prod_sn);
			d.check_sn = true;
		} catch(const std::exception&)
		{
			// Non-numeric serial: nothing to compare the 40-bit header field with.
		}
	}

	const auto& m = metadata.imu_to_sensor_transform;
	d.imu_rotation = {m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]};
	d.buildLut();

	d.buffer.resize(1 << 20);
	d.in.rdbuf()->pubsetbuf(d.buffer.data(), static_cast<std::streamsize>(d.buffer.size()));
	d.in.open(pcap_path, std::ios::binary);
	if(!d.in.is_open())
	{
		d.fail("cannot open " + pcap_path.string());
		return;
	}

	uint8_t header[24];
	if(!d.in.read(reinterpret_cast<char*>(header), sizeof(header)))
	{
		d.fail(pcap_path.string() + " is too short to be a pcap file");
		return;
	}
	uint32_t magic;
	std::memcpy(&magic, header, sizeof(magic));
	switch(magic)
	{
	case 0xA1B2C3D4:
		break;
	case 0xA1B23C4D:
		d.nanoseconds = true;
		break;
	case 0xD4C3B2A1:
		d.swapped = true;
		break;
	case 0x4D3CB2A1:
		d.swapped = d.nanoseconds = true;
		break;
	case 0x0A0D0D0A:
		d.fail(pcap_path.string() + " is pcapng, which is not supported -- convert it with: editcap -F pcap in.pcapng out.pcap");
		return;
	default:
		d.fail(pcap_path.string() + " is not a pcap file");
		return;
	}
	d.linktype = d.u32(header + 20) & 0x0FFFFFFF; // upper bits: FCS length flags
	switch(d.linktype)
	{
	case kLinkNull:
	case kLinkEthernet:
	case kLinkRaw:
	case kLinkLoop:
	case kLinkLinuxSll:
	case kLinkIpv4:
	case kLinkLinuxSll2:
		break;
	default:
		d.fail("unsupported pcap link type " + std::to_string(d.linktype));
		return;
	}

	d.open = true;
}

OusterPcapReader::~OusterPcapReader() = default;

bool OusterPcapReader::isOpen() const
{
	return impl_->open;
}

const std::string& OusterPcapReader::error() const
{
	return impl_->error;
}

void OusterPcapReader::requestStop()
{
	impl_->stop = true;
}

const OusterPcapStats& OusterPcapReader::stats() const
{
	return impl_->stats;
}

bool OusterPcapReader::read(const Callbacks& callbacks)
{
	Impl& d = *impl_;
	if(!d.open)
		return false;
	d.onPoints = callbacks.onPoints;
	d.onImu = callbacks.onImu;

	uint8_t record_header[16];
	while(!d.stop)
	{
		if(!d.in.read(reinterpret_cast<char*>(record_header), sizeof(record_header)))
			break; // end of file; a torn final header (capture killed mid-write) ends it too
		const uint32_t captured = d.u32(record_header + 8);
		if(captured > kMaxRecordBytes)
			return d.fail("corrupt pcap record #" + std::to_string(d.stats.records + 1) + ": " + std::to_string(captured) + " bytes");
		d.record.resize(captured);
		if(!d.in.read(reinterpret_cast<char*>(d.record.data()), captured))
			break; // torn final record
		++d.stats.records;
		d.stats.bytes_read += sizeof(record_header) + captured;
		d.handleFrame(d.record.data(), d.record.size());
	}
	for(const auto& [key, datagram] : d.pending)
		d.stats.dropped_fragments += datagram.parts.size();
	d.pending.clear();
	return true;
}

} // namespace rosbags
