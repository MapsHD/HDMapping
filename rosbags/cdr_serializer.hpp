#pragma once
#include "McapWriter.h"
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

// Minimal CDR (OMG CDR little-endian) serializer for ros2msg-encoded MCAP messages.
// The 4-byte RTPS representation header {0x00,0x01,0x00,0x00} is written in the
// constructor so callers just append fields directly.
class CdrWriter
{
public:
	CdrWriter()
	{
		buf_ = {0x00, 0x01, 0x00, 0x00};
	}

	void write_bool(bool v)
	{
		buf_.push_back(v ? 1u : 0u);
	}
	void write_u8(uint8_t v)
	{
		buf_.push_back(v);
	}

	void write_u16(uint16_t v)
	{
		pad_to(2);
		append(&v, 2);
	}
	void write_i32(int32_t v)
	{
		pad_to(4);
		append(&v, 4);
	}
	void write_u32(uint32_t v)
	{
		pad_to(4);
		append(&v, 4);
	}
	void write_f32(float v)
	{
		pad_to(4);
		append(&v, 4);
	}
	void write_f64(double v)
	{
		pad_to(8);
		append(&v, 8);
	}

	void write_string(const std::string& s)
	{
		write_u32(static_cast<uint32_t>(s.size() + 1));
		buf_.insert(buf_.end(), s.begin(), s.end());
		buf_.push_back(0);
	}
	void write_string(const char* s)
	{
		const size_t len = std::strlen(s);
		write_u32(static_cast<uint32_t>(len + 1));
		buf_.insert(buf_.end(), s, s + len);
		buf_.push_back(0);
	}

	// Raw bytes — no alignment, for uint8[] sequence payloads.
	void write_raw(const void* src, size_t n)
	{
		const auto* p = static_cast<const uint8_t*>(src);
		buf_.insert(buf_.end(), p, p + n);
	}
	void write_raw(const std::string& s)
	{
		buf_.insert(buf_.end(), s.begin(), s.end());
	}

	const std::vector<uint8_t>& data() const
	{
		return buf_;
	}

private:
	std::vector<uint8_t> buf_;

	// Alignment is counted from byte 4 (the CDR data section start).
	void pad_to(size_t n)
	{
		size_t mod = (buf_.size() - 4) % n;
		if(mod)
			buf_.resize(buf_.size() + (n - mod), 0);
	}

	void append(const void* src, size_t n)
	{
		const auto* p = static_cast<const uint8_t*>(src);
		buf_.insert(buf_.end(), p, p + n);
	}
};

// ---------------------------------------------------------------------------
// CDR reader — mirrors CdrWriter alignment rules exactly.
// Constructed from a raw message buffer; skips the 4-byte RTPS header.
// ---------------------------------------------------------------------------
class CdrReader
{
public:
	CdrReader(const void* data, size_t size)
		: data_(static_cast<const uint8_t*>(data))
		, size_(size)
		, pos_(4)
	{ }

	bool ok() const
	{
		return pos_ <= size_;
	}

	bool read_bool()
	{
		return pos_ < size_ ? data_[pos_++] != 0 : false;
	}
	uint8_t read_u8()
	{
		return pos_ < size_ ? data_[pos_++] : 0;
	}

	uint16_t read_u16()
	{
		align(2);
		uint16_t v = 0;
		safe_copy(&v, 2);
		return v;
	}
	int32_t read_i32()
	{
		align(4);
		int32_t v = 0;
		safe_copy(&v, 4);
		return v;
	}
	uint32_t read_u32()
	{
		align(4);
		uint32_t v = 0;
		safe_copy(&v, 4);
		return v;
	}
	float read_f32()
	{
		align(4);
		float v = 0;
		safe_copy(&v, 4);
		return v;
	}
	double read_f64()
	{
		align(8);
		double v = 0;
		safe_copy(&v, 8);
		return v;
	}

	std::string read_string()
	{
		uint32_t len = read_u32();
		if(len == 0 || pos_ + len > size_)
		{
			pos_ = std::min(pos_ + len, size_);
			return {};
		}
		std::string s(reinterpret_cast<const char*>(data_ + pos_), len - 1);
		pos_ += len;
		return s;
	}

	// Returns raw pointer at current pos and advances by n; nullptr if out of bounds.
	const uint8_t* read_raw(size_t n)
	{
		if(pos_ + n > size_)
			return nullptr;
		const uint8_t* p = data_ + pos_;
		pos_ += n;
		return p;
	}

	void skip(size_t n)
	{
		pos_ = std::min(pos_ + n, size_);
	}

private:
	const uint8_t* data_;
	size_t size_;
	size_t pos_;

	// Alignment is counted from byte 4 (the CDR data section start).
	void align(size_t n)
	{
		size_t mod = (pos_ - 4) % n;
		if(mod)
			pos_ += n - mod;
	}

	void safe_copy(void* dst, size_t n)
	{
		if(pos_ + n <= size_)
		{
			std::memcpy(dst, data_ + pos_, n);
			pos_ += n;
		}
	}
};

// ---------------------------------------------------------------------------
// ROS2 CDR message decoders
// ---------------------------------------------------------------------------

namespace rosbags
{

// sensor_msgs/msg/PointCloud2 → std::vector<McapPoint>
// Decodes by field name/offset (as parsed from the message's own `fields`
// array) rather than a fixed per-layout struct, since several
// PointCloudLayout variants share the same point_step (26 bytes) but place
// different fields at different offsets. Recognizes the field names
// McapWriter emits for any PointCloudLayout: x,y,z,intensity (always),
// ring, laser_id, time (relative f64), t (relative u32 ns, Ouster),
// timestamp (absolute f64, Hesai). Unrecognized fields are ignored.
inline std::vector<McapPoint> decodePc2(const uint8_t* data, size_t size)
{
	std::vector<McapPoint> points;
	CdrReader r(data, size);

	const int32_t stamp_sec = r.read_i32();
	const uint32_t stamp_nsec = r.read_u32();
	r.read_string(); // frame_id

	const double stamp_s = static_cast<double>(stamp_sec) + static_cast<double>(stamp_nsec) * 1e-9;

	r.read_u32(); // height
	const uint32_t width = r.read_u32();

	struct FieldInfo
	{
		uint32_t offset;
		bool present = false;
	};
	FieldInfo xF, yF, zF, intensityF, ringF, laserIdF, timeF, tF, timestampF;

	const uint32_t nFields = r.read_u32();
	for(uint32_t i = 0; i < nFields; ++i)
	{
		const std::string name = r.read_string();
		const uint32_t offset = r.read_u32();
		r.read_u8(); // datatype (implied by field name for our own writer's output)
		r.read_u32(); // count

		FieldInfo info{offset, true};
		if(name == "x")
			xF = info;
		else if(name == "y")
			yF = info;
		else if(name == "z")
			zF = info;
		else if(name == "intensity")
			intensityF = info;
		else if(name == "ring")
			ringF = info;
		else if(name == "laser_id")
			laserIdF = info;
		else if(name == "time")
			timeF = info;
		else if(name == "t")
			tF = info;
		else if(name == "timestamp")
			timestampF = info;
	}

	r.read_bool();
	const uint32_t point_step = r.read_u32();
	r.read_u32(); // row_step

	const uint32_t dataLen = r.read_u32();
	const uint8_t* rawData = r.read_raw(dataLen);

	if(!rawData || point_step == 0 || !xF.present || !yF.present || !zF.present || !intensityF.present)
		return points;

	const uint32_t nPts = dataLen / point_step;
	(void)width;
	points.reserve(nPts);

	for(uint32_t i = 0; i < nPts; ++i)
	{
		const uint8_t* p = rawData + i * point_step;

		McapPoint pt{};
		std::memcpy(&pt.x, p + xF.offset, 4);
		std::memcpy(&pt.y, p + yF.offset, 4);
		std::memcpy(&pt.z, p + zF.offset, 4);
		std::memcpy(&pt.intensity, p + intensityF.offset, 4);

		if(ringF.present)
			std::memcpy(&pt.ring, p + ringF.offset, 2);
		if(laserIdF.present)
			std::memcpy(&pt.laser_id, p + laserIdF.offset, 1);

		if(timestampF.present)
		{
			// Absolute timestamp (Hesai) -- no offset from the message stamp.
			std::memcpy(&pt.timestamp, p + timestampF.offset, 8);
		}
		else if(timeF.present)
		{
			double rel_time = 0.0;
			std::memcpy(&rel_time, p + timeF.offset, 8);
			pt.timestamp = stamp_s + rel_time;
		}
		else if(tF.present)
		{
			uint32_t rel_ns = 0;
			std::memcpy(&rel_ns, p + tF.offset, 4);
			pt.timestamp = stamp_s + static_cast<double>(rel_ns) * 1e-9;
		}
		else
		{
			pt.timestamp = stamp_s;
		}

		points.push_back(pt);
	}

	return points;
}

// sensor_msgs/msg/Imu → McapImuSample (one sample per CDR message)
inline std::optional<McapImuSample> decodeImu(const uint8_t* data, size_t size)
{
	CdrReader r(data, size);

	const int32_t stamp_sec = r.read_i32();
	const uint32_t stamp_nsec = r.read_u32();
	r.read_string(); // frame_id

	const double stamp_s = static_cast<double>(stamp_sec) + static_cast<double>(stamp_nsec) * 1e-9;

	// orientation quaternion (skip)
	r.read_f64();
	r.read_f64();
	r.read_f64();
	r.read_f64();
	// orientation_covariance[9] (skip)
	for(int i = 0; i < 9; ++i)
		r.read_f64();

	McapImuSample imu{};
	imu.gyro_x = static_cast<float>(r.read_f64());
	imu.gyro_y = static_cast<float>(r.read_f64());
	imu.gyro_z = static_cast<float>(r.read_f64());
	// angular_velocity_covariance[9] (skip)
	for(int i = 0; i < 9; ++i)
		r.read_f64();

	imu.acc_x = static_cast<float>(r.read_f64());
	imu.acc_y = static_cast<float>(r.read_f64());
	imu.acc_z = static_cast<float>(r.read_f64());

	imu.timestamp = stamp_s;
	if(!r.ok())
		return std::nullopt;
	return imu;
}

// std_msgs/msg/String → std::string  (empty on parse error)
inline std::string decodeSn(const uint8_t* data, size_t size)
{
	CdrReader r(data, size);
	std::string s = r.read_string();
	return r.ok() ? s : std::string{};
}

} // namespace rosbags