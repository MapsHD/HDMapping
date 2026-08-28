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

// sensor_msgs/msg/PointField datatype codes. None is our own "field absent"
// marker, not part of the ROS message.
enum class Pc2Type : uint8_t
{
	None = 0,
	Int8 = 1,
	UInt8 = 2,
	Int16 = 3,
	UInt16 = 4,
	Int32 = 5,
	UInt32 = 6,
	Float32 = 7,
	Float64 = 8,
};

inline uint32_t pc2TypeSize(Pc2Type type)
{
	switch(type)
	{
	case Pc2Type::Int8:
	case Pc2Type::UInt8: return 1;
	case Pc2Type::Int16:
	case Pc2Type::UInt16: return 2;
	case Pc2Type::Int32:
	case Pc2Type::UInt32:
	case Pc2Type::Float32: return 4;
	case Pc2Type::Float64: return 8;
	case Pc2Type::None:
	default: return 0;
	}
}

struct Pc2Field
{
	uint32_t offset = 0;
	Pc2Type type = Pc2Type::None;

	bool present() const
	{
		return type != Pc2Type::None;
	}
};

// Reads one field out of a point record and widens it to double, so callers do
// not care which of the eight PointField datatypes a driver happened to use.
inline double readPc2Field(const uint8_t* point, const Pc2Field& field)
{
	const uint8_t* p = point + field.offset;
	switch(field.type)
	{
	case Pc2Type::Int8:
	{
		int8_t v = 0;
		std::memcpy(&v, p, 1);
		return static_cast<double>(v);
	}
	case Pc2Type::UInt8: return static_cast<double>(*p);
	case Pc2Type::Int16:
	{
		int16_t v = 0;
		std::memcpy(&v, p, 2);
		return static_cast<double>(v);
	}
	case Pc2Type::UInt16:
	{
		uint16_t v = 0;
		std::memcpy(&v, p, 2);
		return static_cast<double>(v);
	}
	case Pc2Type::Int32:
	{
		int32_t v = 0;
		std::memcpy(&v, p, 4);
		return static_cast<double>(v);
	}
	case Pc2Type::UInt32:
	{
		uint32_t v = 0;
		std::memcpy(&v, p, 4);
		return static_cast<double>(v);
	}
	case Pc2Type::Float32:
	{
		float v = 0;
		std::memcpy(&v, p, 4);
		return static_cast<double>(v);
	}
	case Pc2Type::Float64:
	{
		double v = 0;
		std::memcpy(&v, p, 8);
		return v;
	}
	case Pc2Type::None:
	default: return 0.0;
	}
}

// The PointCloud2 fields McapPoint can represent. Three mutually exclusive
// per-point time fields are recognized, distinguished by name because the
// message carries no unit metadata:
//   timestamp - absolute seconds (Hesai convention)
//   time      - seconds relative to the message stamp
//   t         - relative; nanoseconds if an integer type, seconds if floating
// Everything else in a record (reflectivity, ambient, range, ...) is ignored.
struct Pc2Layout
{
	uint32_t point_step = 0;
	Pc2Field x, y, z, intensity, ring, laser_id, time, t, timestamp;
};

// Selects how a PointCloud2's point records are interpreted.
//   Auto   - trust the message's own `fields` array (correct for every bag
//            laz_to_mcap writes, and for any driver that fills it in honestly)
//   Hesai  - force the HesaiLidar_ROS_2.0 Pandar record layout
//   Ouster - force the ouster_ros Point record layout
// The presets exist for bags whose `fields` array is missing, truncated, or
// mislabelled; Auto is otherwise strictly better, since it adapts per message.
enum class Pc2Preset
{
	Auto,
	Hesai,
	Ouster,
};

// Verified against a real HesaiLidar_ROS_2.0 recording (/hesai/pandar): the
// PCL-style 4-byte pad after z puts intensity at 16, not 12, and point_step is
// 48 -- notably NOT the same packing PointCloudLayout::Hesai writes.
inline Pc2Layout hesaiPandarLayout()
{
	Pc2Layout l;
	l.point_step = 48;
	l.x = {0, Pc2Type::Float32};
	l.y = {4, Pc2Type::Float32};
	l.z = {8, Pc2Type::Float32};
	l.intensity = {16, Pc2Type::Float32};
	l.timestamp = {24, Pc2Type::Float64};
	l.ring = {32, Pc2Type::UInt16};
	return l;
}

// ouster_ros' Point struct: PCL_ADD_POINT4D (16 bytes) + intensity, t,
// reflectivity, ring, ambient, range, padded to 48. Transcribed from the
// driver's published layout rather than measured, and ring's width has moved
// between driver releases -- decodePc2 cross-checks any preset against the
// message's own fields array and reports a mismatch, so a wrong guess here
// surfaces as a warning instead of as silently shifted data.
inline Pc2Layout ousterRosLayout()
{
	Pc2Layout l;
	l.point_step = 48;
	l.x = {0, Pc2Type::Float32};
	l.y = {4, Pc2Type::Float32};
	l.z = {8, Pc2Type::Float32};
	l.intensity = {16, Pc2Type::Float32};
	l.t = {20, Pc2Type::UInt32};
	l.ring = {26, Pc2Type::UInt8};
	return l;
}

// False for Auto, which has no fixed layout.
inline bool pc2PresetLayout(Pc2Preset preset, Pc2Layout& out)
{
	switch(preset)
	{
	case Pc2Preset::Hesai: out = hesaiPandarLayout(); return true;
	case Pc2Preset::Ouster: out = ousterRosLayout(); return true;
	case Pc2Preset::Auto:
	default: return false;
	}
}

inline const char* pc2PresetName(Pc2Preset preset)
{
	switch(preset)
	{
	case Pc2Preset::Hesai: return "hesai";
	case Pc2Preset::Ouster: return "ouster";
	case Pc2Preset::Auto:
	default: return "auto";
	}
}

// Every present field must lie wholly inside one point record, or decoding
// would read into the following point (or off the end of the buffer for the
// last one). Only reachable with a forced preset whose point_step disagrees
// with the file.
inline bool pc2FieldFits(const Pc2Field& field, uint32_t point_step)
{
	if(!field.present())
		return true;
	return static_cast<uint64_t>(field.offset) + pc2TypeSize(field.type) <= point_step;
}

inline bool pc2LayoutFits(const Pc2Layout& l)
{
	return pc2FieldFits(l.x, l.point_step) && pc2FieldFits(l.y, l.point_step) && pc2FieldFits(l.z, l.point_step) &&
		   pc2FieldFits(l.intensity, l.point_step) && pc2FieldFits(l.ring, l.point_step) && pc2FieldFits(l.laser_id, l.point_step) &&
		   pc2FieldFits(l.time, l.point_step) && pc2FieldFits(l.t, l.point_step) && pc2FieldFits(l.timestamp, l.point_step);
}

namespace detail
{

// Appends a human-readable "<name> <preset spec> vs <message spec>" note for one
// field whose preset and message descriptions disagree.
inline void describePc2Mismatch(const char* name, const Pc2Field& preset, const Pc2Field& message, std::string& out)
{
	if(preset.offset == message.offset && preset.type == message.type)
		return;
	if(!preset.present() && !message.present())
		return;
	if(!out.empty())
		out += ", ";
	out += name;
	out += " preset@" + std::to_string(preset.offset) + "/type" + std::to_string(static_cast<int>(preset.type));
	out += " vs file@" + std::to_string(message.offset) + "/type" + std::to_string(static_cast<int>(message.type));
}

inline std::string comparePc2Layouts(const Pc2Layout& preset, const Pc2Layout& message)
{
	std::string diff;
	describePc2Mismatch("x", preset.x, message.x, diff);
	describePc2Mismatch("y", preset.y, message.y, diff);
	describePc2Mismatch("z", preset.z, message.z, diff);
	describePc2Mismatch("intensity", preset.intensity, message.intensity, diff);
	describePc2Mismatch("ring", preset.ring, message.ring, diff);
	describePc2Mismatch("laser_id", preset.laser_id, message.laser_id, diff);
	describePc2Mismatch("time", preset.time, message.time, diff);
	describePc2Mismatch("t", preset.t, message.t, diff);
	describePc2Mismatch("timestamp", preset.timestamp, message.timestamp, diff);
	if(preset.point_step != message.point_step)
	{
		if(!diff.empty())
			diff += ", ";
		diff += "point_step preset=" + std::to_string(preset.point_step) + " vs file=" + std::to_string(message.point_step);
	}
	return diff;
}

} // namespace detail

// sensor_msgs/msg/PointCloud2 → std::vector<McapPoint>
//
// By default (Pc2Preset::Auto) the record layout is taken from the message's own
// `fields` array -- offsets *and* datatypes -- so one decoder handles every
// PointCloudLayout McapWriter emits as well as foreign driver bags, which pack
// the same field names at different offsets and widths. Recognized names are
// x, y, z (required), intensity, ring, laser_id, and the three time fields
// described on Pc2Layout; anything else is ignored.
//
// Passing a preset instead forces that driver's layout and ignores the file's
// fields array, for bags whose array is missing or wrong. When the file does
// describe its fields, the preset is still compared against it and any
// disagreement is reported through `warning` -- forcing a layout that does not
// match the data is the one way this decoder can silently produce plausible
// nonsense, so it is never done quietly.
inline std::vector<McapPoint> decodePc2(
	const uint8_t* data, size_t size, Pc2Preset preset = Pc2Preset::Auto, std::string* warning = nullptr)
{
	const auto warn = [&](std::string message) {
		if(warning && warning->empty())
			*warning = std::move(message);
	};

	std::vector<McapPoint> points;
	CdrReader r(data, size);

	const int32_t stamp_sec = r.read_i32();
	const uint32_t stamp_nsec = r.read_u32();
	r.read_string(); // frame_id

	const double stamp_s = static_cast<double>(stamp_sec) + static_cast<double>(stamp_nsec) * 1e-9;

	r.read_u32(); // height
	r.read_u32(); // width -- the record count is derived from data/point_step instead

	Pc2Layout msg;
	const uint32_t nFields = r.read_u32();
	for(uint32_t i = 0; i < nFields; ++i)
	{
		const std::string name = r.read_string();
		const uint32_t offset = r.read_u32();
		const auto type = static_cast<Pc2Type>(r.read_u8());
		const uint32_t count = r.read_u32();

		// count > 1 is an array field (never used for the scalars below).
		if(count != 1 || pc2TypeSize(type) == 0)
			continue;

		const Pc2Field field{offset, type};
		if(name == "x")
			msg.x = field;
		else if(name == "y")
			msg.y = field;
		else if(name == "z")
			msg.z = field;
		else if(name == "intensity")
			msg.intensity = field;
		else if(name == "ring")
			msg.ring = field;
		else if(name == "laser_id")
			msg.laser_id = field;
		else if(name == "time")
			msg.time = field;
		else if(name == "t")
			msg.t = field;
		else if(name == "timestamp")
			msg.timestamp = field;
	}

	const bool is_bigendian = r.read_bool();
	msg.point_step = r.read_u32();
	r.read_u32(); // row_step

	const uint32_t dataLen = r.read_u32();
	const uint8_t* rawData = r.read_raw(dataLen);

	if(!rawData)
	{
		warn("PointCloud2 data array is truncated");
		return points;
	}
	if(is_bigendian)
	{
		// No real ROS2 bag sets this; byte-swapping every field to support a
		// hypothetical one would be untestable, so refuse rather than misread.
		warn("PointCloud2 is big-endian, which is not supported");
		return points;
	}

	Pc2Layout layout;
	if(pc2PresetLayout(preset, layout))
	{
		if(nFields > 0)
		{
			const std::string diff = detail::comparePc2Layouts(layout, msg);
			if(!diff.empty())
				warn(std::string("forced '") + pc2PresetName(preset) + "' layout disagrees with the file's own fields (" + diff + ")");
		}
	}
	else
	{
		layout = msg;
	}

	if(layout.point_step == 0)
	{
		warn("PointCloud2 has a zero point_step");
		return points;
	}
	if(!layout.x.present() || !layout.y.present() || !layout.z.present())
	{
		warn("PointCloud2 has no x/y/z fields");
		return points;
	}
	if(!pc2LayoutFits(layout))
	{
		warn(std::string("'") + pc2PresetName(preset) + "' layout has fields reaching past point_step " +
			 std::to_string(layout.point_step));
		return points;
	}

	const uint32_t nPts = dataLen / layout.point_step;
	points.reserve(nPts);

	for(uint32_t i = 0; i < nPts; ++i)
	{
		const uint8_t* p = rawData + static_cast<size_t>(i) * layout.point_step;

		McapPoint pt{};
		pt.x = static_cast<float>(readPc2Field(p, layout.x));
		pt.y = static_cast<float>(readPc2Field(p, layout.y));
		pt.z = static_cast<float>(readPc2Field(p, layout.z));
		if(layout.intensity.present())
			pt.intensity = static_cast<float>(readPc2Field(p, layout.intensity));
		if(layout.ring.present())
			pt.ring = static_cast<uint16_t>(readPc2Field(p, layout.ring));
		if(layout.laser_id.present())
			pt.laser_id = static_cast<uint8_t>(readPc2Field(p, layout.laser_id));

		if(layout.timestamp.present())
		{
			pt.timestamp = readPc2Field(p, layout.timestamp);
		}
		else if(layout.time.present())
		{
			pt.timestamp = stamp_s + readPc2Field(p, layout.time);
		}
		else if(layout.t.present())
		{
			// Integer t is nanoseconds (ouster_ros); floating t is seconds.
			const double raw = readPc2Field(p, layout.t);
			const bool integral = layout.t.type != Pc2Type::Float32 && layout.t.type != Pc2Type::Float64;
			pt.timestamp = stamp_s + (integral ? raw * 1e-9 : raw);
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