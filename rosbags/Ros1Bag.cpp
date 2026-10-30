#include "Ros1Bag.h"

#if !defined(MCAP_COMPRESSION_NO_LZ4) // same switch as the mcap target: no compression libraries on Windows
#include <bzlib.h>
#include <lz4frame.h>
#endif

#include <algorithm>
#include <cstring>
#include <string_view>
#include <utility>

namespace rosbags
{

// ---------------------------------------------------------------------------
// Message definitions as `rosbag record` stores them (sensor_msgs 1.13).
// ---------------------------------------------------------------------------

const char* const kRos1PointCloud2Type = "sensor_msgs/PointCloud2";
const char* const kRos1PointCloud2Md5 = "1158d486dd51d683ce2f1be655c3c181";
const char* const kRos1PointCloud2Definition = R"(std_msgs/Header header
uint32 height
uint32 width
sensor_msgs/PointField[] fields
bool is_bigendian
uint32 point_step
uint32 row_step
uint8[] data
bool is_dense
================================================================================
MSG: std_msgs/Header
uint32 seq
time stamp
string frame_id
================================================================================
MSG: sensor_msgs/PointField
uint8 INT8=1
uint8 UINT8=2
uint8 INT16=3
uint8 UINT16=4
uint8 INT32=5
uint8 UINT32=6
uint8 FLOAT32=7
uint8 FLOAT64=8
string name
uint32 offset
uint8 datatype
uint32 count
)";

const char* const kRos1ImuType = "sensor_msgs/Imu";
const char* const kRos1ImuMd5 = "6a62c6daae103f4ff57a132d6f95cec2";
const char* const kRos1ImuDefinition = R"(std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
================================================================================
MSG: std_msgs/Header
uint32 seq
time stamp
string frame_id
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

namespace
{

constexpr char kMagic[] = "#ROSBAG V2.0\n";
constexpr size_t kMagicSize = sizeof(kMagic) - 1;
constexpr size_t kBagHeaderRecordSize = 4096;

constexpr uint8_t kOpMessageData = 0x02;
constexpr uint8_t kOpBagHeader = 0x03;
constexpr uint8_t kOpIndexData = 0x04;
constexpr uint8_t kOpChunk = 0x05;
constexpr uint8_t kOpChunkInfo = 0x06;
constexpr uint8_t kOpConnection = 0x07;

// Little-endian cursor over a byte buffer; any read past the end clears `ok`.
struct ByteReader
{
	const uint8_t* p;
	size_t n;
	size_t i = 0;
	bool ok = true;

	template <class T>
	T get()
	{
		T v{};
		if(i + sizeof(T) > n)
		{
			ok = false;
			return v;
		}
		std::memcpy(&v, p + i, sizeof(T));
		i += sizeof(T);
		return v;
	}

	const uint8_t* bytes(size_t k)
	{
		if(i + k > n)
		{
			ok = false;
			return nullptr;
		}
		const uint8_t* out = p + i;
		i += k;
		return out;
	}

	std::string str()
	{
		const uint32_t len = get<uint32_t>();
		const uint8_t* b = ok ? bytes(len) : nullptr;
		return b ? std::string(reinterpret_cast<const char*>(b), len) : std::string();
	}
};

struct ByteWriter
{
	std::vector<uint8_t>& out;

	template <class T>
	void put(T v)
	{
		const auto* b = reinterpret_cast<const uint8_t*>(&v);
		out.insert(out.end(), b, b + sizeof(T));
	}

	void bytes(const void* data, size_t k)
	{
		const auto* b = static_cast<const uint8_t*>(data);
		out.insert(out.end(), b, b + k);
	}

	void str(const std::string& s)
	{
		put<uint32_t>(static_cast<uint32_t>(s.size()));
		bytes(s.data(), s.size());
	}
};

using HeaderFields = std::map<std::string, std::string>;

bool parseHeaderFields(const uint8_t* p, size_t n, HeaderFields& fields)
{
	ByteReader r{p, n};
	while(r.ok && r.i < n)
	{
		const std::string field = r.str();
		if(!r.ok)
			return false;
		const auto eq = field.find('=');
		if(eq == std::string::npos)
			return false;
		fields[field.substr(0, eq)] = field.substr(eq + 1);
	}
	return r.ok;
}

template <class T>
bool fieldAs(const HeaderFields& fields, const char* name, T& value)
{
	const auto it = fields.find(name);
	if(it == fields.end() || it->second.size() != sizeof(T))
		return false;
	std::memcpy(&value, it->second.data(), sizeof(T));
	return true;
}

// ROS time: uint32 sec, uint32 nsec.
uint64_t timeFromField(const std::string& v)
{
	uint32_t sec = 0, nsec = 0;
	if(v.size() == 8)
	{
		std::memcpy(&sec, v.data(), 4);
		std::memcpy(&nsec, v.data() + 4, 4);
	}
	return uint64_t(sec) * 1'000'000'000ULL + nsec;
}

template <class T>
std::string asField(T v)
{
	return std::string(reinterpret_cast<const char*>(&v), sizeof(T));
}

std::string timeField(uint64_t ns)
{
	const uint32_t sec = static_cast<uint32_t>(ns / 1'000'000'000ULL);
	const uint32_t nsec = static_cast<uint32_t>(ns % 1'000'000'000ULL);
	return asField(sec) + asField(nsec);
}

using FieldList = std::vector<std::pair<std::string, std::string>>;

std::vector<uint8_t> headerBytes(const FieldList& fields)
{
	std::vector<uint8_t> out;
	ByteWriter w{out};
	for(const auto& [name, value] : fields)
	{
		w.put<uint32_t>(static_cast<uint32_t>(name.size() + 1 + value.size()));
		w.bytes(name.data(), name.size());
		w.put<char>('=');
		w.bytes(value.data(), value.size());
	}
	return out;
}

void appendRecord(std::vector<uint8_t>& out, const FieldList& header, const uint8_t* data, size_t size)
{
	const auto h = headerBytes(header);
	ByteWriter w{out};
	w.put<uint32_t>(static_cast<uint32_t>(h.size()));
	w.bytes(h.data(), h.size());
	w.put<uint32_t>(static_cast<uint32_t>(size));
	w.bytes(data, size);
}

std::vector<uint8_t> connectionData(const Ros1Connection& c)
{
	return headerBytes(
		{{"topic", c.topic}, {"type", c.type}, {"md5sum", c.md5sum}, {"message_definition", c.message_definition}});
}

void appendConnectionRecord(std::vector<uint8_t>& out, const Ros1Connection& c)
{
	const auto data = connectionData(c);
	appendRecord(
		out, {{"op", asField(kOpConnection)}, {"conn", asField(c.id)}, {"topic", c.topic}}, data.data(), data.size());
}

bool decompress(const std::string& compression, const std::vector<uint8_t>& in, size_t out_size, std::vector<uint8_t>& out, std::string& error)
{
	out.resize(out_size);
	if(compression == "none")
	{
		if(in.size() != out_size)
		{
			error = "uncompressed chunk size mismatch";
			return false;
		}
		out = in;
		return true;
	}
#if !defined(MCAP_COMPRESSION_NO_LZ4)
	if(compression == "bz2")
	{
		unsigned int dst = static_cast<unsigned int>(out_size);
		const int rc = BZ2_bzBuffToBuffDecompress(
			reinterpret_cast<char*>(out.data()),
			&dst,
			const_cast<char*>(reinterpret_cast<const char*>(in.data())),
			static_cast<unsigned int>(in.size()),
			0,
			0);
		if(rc != BZ_OK || dst != out_size)
		{
			error = "bz2 chunk decompression failed (code " + std::to_string(rc) + ")";
			return false;
		}
		return true;
	}
	if(compression == "lz4")
	{
		LZ4F_dctx* ctx = nullptr;
		if(LZ4F_isError(LZ4F_createDecompressionContext(&ctx, LZ4F_VERSION)))
		{
			error = "cannot create an LZ4 decompression context";
			return false;
		}
		size_t src_pos = 0, dst_pos = 0;
		size_t rc = 1;
		while(rc != 0 && src_pos < in.size() && dst_pos < out_size)
		{
			size_t src_len = in.size() - src_pos, dst_len = out_size - dst_pos;
			rc = LZ4F_decompress(ctx, out.data() + dst_pos, &dst_len, in.data() + src_pos, &src_len, nullptr);
			if(LZ4F_isError(rc))
				break;
			src_pos += src_len;
			dst_pos += dst_len;
		}
		LZ4F_freeDecompressionContext(ctx);
		if(LZ4F_isError(rc) || dst_pos != out_size)
		{
			error = "lz4 chunk decompression failed";
			return false;
		}
		return true;
	}
#endif
	error = "unsupported chunk compression '" + compression + "'";
	return false;
}

// One record read from the file: header fields plus its data.
bool readRecord(std::ifstream& f, HeaderFields& fields, std::vector<uint8_t>& data, std::string& error)
{
	uint32_t header_len = 0;
	if(!f.read(reinterpret_cast<char*>(&header_len), 4))
		return false; // clean end of file
	std::vector<uint8_t> header(header_len);
	uint32_t data_len = 0;
	if(!f.read(reinterpret_cast<char*>(header.data()), header_len) || !f.read(reinterpret_cast<char*>(&data_len), 4))
	{
		error = "truncated record header";
		return false;
	}
	data.resize(data_len);
	if(!f.read(reinterpret_cast<char*>(data.data()), data_len))
	{
		error = "truncated record data";
		return false;
	}
	fields.clear();
	if(!parseHeaderFields(header.data(), header.size(), fields))
	{
		error = "malformed record header";
		return false;
	}
	return true;
}

Ros1Connection connectionFromRecord(const HeaderFields& header, const std::vector<uint8_t>& data, bool& ok)
{
	Ros1Connection c;
	ok = fieldAs(header, "conn", c.id);
	HeaderFields info;
	ok = ok && parseHeaderFields(data.data(), data.size(), info);
	const auto topic = header.find("topic");
	c.topic = topic != header.end() ? topic->second : info["topic"];
	c.type = info["type"];
	c.md5sum = info["md5sum"];
	c.message_definition = info["message_definition"];
	return c;
}

} // namespace

// ---------------------------------------------------------------------------
// Ros1BagReader
// ---------------------------------------------------------------------------

Ros1BagReader::Ros1BagReader(const std::filesystem::path& path)
	: file_(path, std::ios::binary)
{
	if(!file_)
	{
		error_ = "cannot open " + path.string();
		return;
	}
	char magic[kMagicSize];
	if(!file_.read(magic, kMagicSize) || std::memcmp(magic, kMagic, kMagicSize) != 0)
	{
		error_ = path.string() + " is not a ROS1 bag (format 2.0)";
		return;
	}
	HeaderFields header;
	std::vector<uint8_t> data;
	uint8_t op = 0;
	if(!readRecord(file_, header, data, error_) || !fieldAs(header, "op", op) || op != kOpBagHeader ||
	   !fieldAs(header, "index_pos", index_pos_))
	{
		if(error_.empty())
			error_ = "missing bag header record";
		return;
	}
	readIndex();
}

bool Ros1BagReader::isOpen() const
{
	return error_.empty();
}

const std::string& Ros1BagReader::error() const
{
	return error_;
}

const std::map<uint32_t, Ros1Connection>& Ros1BagReader::connections() const
{
	return connections_;
}

bool Ros1BagReader::readIndex()
{
	if(index_pos_ == 0)
		return true; // unindexed (still recording or not reindexed): connections come from the chunks
	const auto resume = file_.tellg();
	file_.seekg(static_cast<std::streamoff>(index_pos_));
	HeaderFields header;
	std::vector<uint8_t> data;
	std::string err;
	while(readRecord(file_, header, data, err))
	{
		uint8_t op = 0;
		if(fieldAs(header, "op", op) && op == kOpConnection)
		{
			bool ok = false;
			const auto c = connectionFromRecord(header, data, ok);
			if(ok)
				connections_[c.id] = c;
		}
	}
	file_.clear();
	file_.seekg(resume);
	return err.empty();
}

bool Ros1BagReader::parseRecords(const uint8_t* data, size_t size, const MessageFn& fn, bool& stop)
{
	ByteReader r{data, size};
	while(r.i < size)
	{
		const uint32_t header_len = r.get<uint32_t>();
		const uint8_t* header = r.bytes(header_len);
		const uint32_t data_len = r.get<uint32_t>();
		const uint8_t* body = r.bytes(data_len);
		HeaderFields fields;
		uint8_t op = 0;
		if(!r.ok || !parseHeaderFields(header, header_len, fields) || !fieldAs(fields, "op", op))
		{
			error_ = "malformed record inside a chunk";
			return false;
		}
		if(op == kOpConnection)
		{
			bool ok = false;
			const auto c = connectionFromRecord(fields, std::vector<uint8_t>(body, body + data_len), ok);
			if(!ok)
			{
				error_ = "malformed connection record";
				return false;
			}
			connections_.try_emplace(c.id, c);
		}
		else if(op == kOpMessageData)
		{
			uint32_t conn = 0;
			const auto time = fields.find("time");
			if(!fieldAs(fields, "conn", conn) || time == fields.end())
			{
				error_ = "malformed message record";
				return false;
			}
			const auto it = connections_.find(conn);
			if(it == connections_.end())
			{
				error_ = "message on unknown connection " + std::to_string(conn);
				return false;
			}
			if(!fn(it->second, timeFromField(time->second), body, data_len))
			{
				stop = true;
				return true;
			}
		}
	}
	return true;
}

bool Ros1BagReader::forEachMessage(const MessageFn& fn)
{
	if(!isOpen())
		return false;
	file_.clear();
	file_.seekg(static_cast<std::streamoff>(kMagicSize));
	HeaderFields header;
	std::vector<uint8_t> data, chunk;
	if(!readRecord(file_, header, data, error_))
		return false; // bag header record

	bool stop = false;
	while(!stop)
	{
		const auto pos = static_cast<uint64_t>(file_.tellg());
		if(index_pos_ != 0 && pos >= index_pos_)
			break;
		if(!readRecord(file_, header, data, error_))
			return error_.empty();
		uint8_t op = 0;
		fieldAs(header, "op", op);
		if(op == kOpChunk)
		{
			uint32_t size = 0;
			const auto compression = header.find("compression");
			if(compression == header.end() || !fieldAs(header, "size", size))
			{
				error_ = "malformed chunk record";
				return false;
			}
			if(!decompress(compression->second, data, size, chunk, error_))
				return false;
			if(!parseRecords(chunk.data(), chunk.size(), fn, stop))
				return false;
		}
		else if(op == kOpConnection || op == kOpMessageData)
		{
			// Records outside chunks: rebuild the record and reuse the chunk parser.
			std::vector<uint8_t> record;
			FieldList list(header.begin(), header.end());
			appendRecord(record, list, data.data(), data.size());
			if(!parseRecords(record.data(), record.size(), fn, stop))
				return false;
		}
		// Index data and chunk info records carry nothing a sequential reader needs.
	}
	return true;
}

// ---------------------------------------------------------------------------
// Ros1BagWriter
// ---------------------------------------------------------------------------

namespace
{

std::vector<uint8_t> bagHeaderRecord(uint64_t index_pos, uint32_t conn_count, uint32_t chunk_count)
{
	const FieldList header = {
		{"op", asField(kOpBagHeader)},
		{"index_pos", asField(index_pos)},
		{"conn_count", asField(conn_count)},
		{"chunk_count", asField(chunk_count)}};
	const size_t header_len = headerBytes(header).size();
	// The bag header record is padded to 4096 bytes so it can be rewritten in place.
	const std::vector<uint8_t> padding(kBagHeaderRecordSize - 8 - header_len, ' ');
	std::vector<uint8_t> out;
	appendRecord(out, header, padding.data(), padding.size());
	return out;
}

} // namespace

Ros1BagWriter::Ros1BagWriter(const std::filesystem::path& path, size_t chunk_threshold)
	: file_(path, std::ios::binary | std::ios::trunc)
	, chunk_threshold_(chunk_threshold)
{
	if(!file_)
	{
		error_ = "cannot create " + path.string();
		return;
	}
	file_.write(kMagic, kMagicSize);
	const auto header = bagHeaderRecord(0, 0, 0);
	file_.write(reinterpret_cast<const char*>(header.data()), header.size());
}

Ros1BagWriter::~Ros1BagWriter()
{
	close();
}

bool Ros1BagWriter::isOpen() const
{
	return error_.empty() && file_.good();
}

const std::string& Ros1BagWriter::error() const
{
	return error_;
}

uint32_t Ros1BagWriter::addConnection(
	const std::string& topic, const std::string& type, const std::string& md5sum, const std::string& message_definition)
{
	const auto id = static_cast<uint32_t>(connections_.size());
	connections_.push_back({id, topic, type, md5sum, message_definition});
	return id;
}

void Ros1BagWriter::write(uint32_t connection, uint64_t time_ns, const std::vector<uint8_t>& data)
{
	if(closed_ || connection >= connections_.size())
		return;
	if(chunk_.empty())
	{
		chunk_start_ns_ = time_ns;
		chunk_end_ns_ = time_ns;
	}
	chunk_start_ns_ = std::min(chunk_start_ns_, time_ns);
	chunk_end_ns_ = std::max(chunk_end_ns_, time_ns);
	// A chunk must carry the connection record of every connection it uses.
	if(chunk_connections_.insert(connection).second)
		appendConnectionRecord(chunk_, connections_[connection]);
	const auto offset = static_cast<uint32_t>(chunk_.size());
	appendRecord(
		chunk_,
		{{"op", asField(kOpMessageData)}, {"conn", asField(connection)}, {"time", timeField(time_ns)}},
		data.data(),
		data.size());
	chunk_index_[connection].push_back({time_ns, offset});
	if(chunk_.size() >= chunk_threshold_)
		flushChunk();
}

void Ros1BagWriter::flushChunk()
{
	if(chunk_.empty())
		return;
	ChunkInfo info{static_cast<uint64_t>(file_.tellp()), chunk_start_ns_, chunk_end_ns_, {}};
	std::vector<uint8_t> out;
	appendRecord(
		out,
		{{"op", asField(kOpChunk)}, {"compression", "none"}, {"size", asField(static_cast<uint32_t>(chunk_.size()))}},
		chunk_.data(),
		chunk_.size());
	for(const auto& [conn, entries] : chunk_index_)
	{
		std::vector<uint8_t> index;
		ByteWriter w{index};
		for(const auto& e : entries)
		{
			const auto t = timeField(e.time_ns);
			w.bytes(t.data(), t.size());
			w.put<uint32_t>(e.offset);
		}
		appendRecord(
			out,
			{{"op", asField(kOpIndexData)},
			 {"ver", asField(uint32_t{1})},
			 {"conn", asField(conn)},
			 {"count", asField(static_cast<uint32_t>(entries.size()))}},
			index.data(),
			index.size());
		info.counts[conn] = static_cast<uint32_t>(entries.size());
	}
	file_.write(reinterpret_cast<const char*>(out.data()), out.size());
	chunk_infos_.push_back(std::move(info));
	chunk_.clear();
	chunk_connections_.clear();
	chunk_index_.clear();
}

bool Ros1BagWriter::close()
{
	if(closed_)
		return isOpen();
	closed_ = true;
	if(!file_.is_open())
		return false;
	flushChunk();
	const auto index_pos = static_cast<uint64_t>(file_.tellp());
	std::vector<uint8_t> out;
	for(const auto& c : connections_)
		appendConnectionRecord(out, c);
	for(const auto& info : chunk_infos_)
	{
		std::vector<uint8_t> data;
		ByteWriter w{data};
		for(const auto& [conn, count] : info.counts)
		{
			w.put<uint32_t>(conn);
			w.put<uint32_t>(count);
		}
		appendRecord(
			out,
			{{"op", asField(kOpChunkInfo)},
			 {"ver", asField(uint32_t{1})},
			 {"chunk_pos", asField(info.pos)},
			 {"start_time", timeField(info.start_ns)},
			 {"end_time", timeField(info.end_ns)},
			 {"count", asField(static_cast<uint32_t>(info.counts.size()))}},
			data.data(),
			data.size());
	}
	file_.write(reinterpret_cast<const char*>(out.data()), out.size());
	const auto header = bagHeaderRecord(
		index_pos, static_cast<uint32_t>(connections_.size()), static_cast<uint32_t>(chunk_infos_.size()));
	file_.seekp(static_cast<std::streamoff>(kMagicSize));
	file_.write(reinterpret_cast<const char*>(header.data()), header.size());
	file_.close();
	if(file_.fail() && error_.empty())
		error_ = "write failed";
	return error_.empty();
}

// ---------------------------------------------------------------------------
// sensor_msgs
// ---------------------------------------------------------------------------

namespace
{

void readHeader(ByteReader& r, uint32_t& seq, uint64_t& stamp_ns, std::string& frame_id)
{
	seq = r.get<uint32_t>();
	const uint32_t sec = r.get<uint32_t>();
	const uint32_t nsec = r.get<uint32_t>();
	stamp_ns = uint64_t(sec) * 1'000'000'000ULL + nsec;
	frame_id = r.str();
}

void writeHeader(ByteWriter& w, uint32_t seq, uint64_t stamp_ns, const std::string& frame_id)
{
	w.put<uint32_t>(seq);
	w.put<uint32_t>(static_cast<uint32_t>(stamp_ns / 1'000'000'000ULL));
	w.put<uint32_t>(static_cast<uint32_t>(stamp_ns % 1'000'000'000ULL));
	w.str(frame_id);
}

} // namespace

bool decodeRos1PointCloud2(const uint8_t* data, size_t size, Ros1PointCloud2& msg)
{
	ByteReader r{data, size};
	readHeader(r, msg.seq, msg.stamp_ns, msg.frame_id);
	msg.height = r.get<uint32_t>();
	msg.width = r.get<uint32_t>();
	const uint32_t n_fields = r.get<uint32_t>();
	msg.fields.clear();
	for(uint32_t i = 0; i < n_fields && r.ok; ++i)
	{
		Ros1PointField f;
		f.name = r.str();
		f.offset = r.get<uint32_t>();
		f.datatype = r.get<uint8_t>();
		f.count = r.get<uint32_t>();
		msg.fields.push_back(f);
	}
	msg.is_bigendian = r.get<uint8_t>() != 0;
	msg.point_step = r.get<uint32_t>();
	msg.row_step = r.get<uint32_t>();
	const uint32_t n_data = r.get<uint32_t>();
	const uint8_t* bytes = r.ok ? r.bytes(n_data) : nullptr;
	if(bytes)
		msg.data.assign(bytes, bytes + n_data);
	msg.is_dense = r.get<uint8_t>() != 0;
	return r.ok;
}

std::vector<uint8_t> encodeRos1PointCloud2(const Ros1PointCloud2& msg)
{
	std::vector<uint8_t> out;
	out.reserve(msg.data.size() + 256);
	ByteWriter w{out};
	writeHeader(w, msg.seq, msg.stamp_ns, msg.frame_id);
	w.put<uint32_t>(msg.height);
	w.put<uint32_t>(msg.width);
	w.put<uint32_t>(static_cast<uint32_t>(msg.fields.size()));
	for(const auto& f : msg.fields)
	{
		w.str(f.name);
		w.put<uint32_t>(f.offset);
		w.put<uint8_t>(f.datatype);
		w.put<uint32_t>(f.count);
	}
	w.put<uint8_t>(msg.is_bigendian ? 1 : 0);
	w.put<uint32_t>(msg.point_step);
	w.put<uint32_t>(msg.row_step);
	w.put<uint32_t>(static_cast<uint32_t>(msg.data.size()));
	w.bytes(msg.data.data(), msg.data.size());
	w.put<uint8_t>(msg.is_dense ? 1 : 0);
	return out;
}

bool decodeRos1Imu(const uint8_t* data, size_t size, Ros1Imu& msg)
{
	ByteReader r{data, size};
	readHeader(r, msg.seq, msg.stamp_ns, msg.frame_id);
	for(double& v : msg.orientation)
		v = r.get<double>();
	for(double& v : msg.orientation_covariance)
		v = r.get<double>();
	for(double& v : msg.angular_velocity)
		v = r.get<double>();
	for(double& v : msg.angular_velocity_covariance)
		v = r.get<double>();
	for(double& v : msg.linear_acceleration)
		v = r.get<double>();
	for(double& v : msg.linear_acceleration_covariance)
		v = r.get<double>();
	return r.ok;
}

std::vector<uint8_t> encodeRos1Imu(const Ros1Imu& msg)
{
	std::vector<uint8_t> out;
	ByteWriter w{out};
	writeHeader(w, msg.seq, msg.stamp_ns, msg.frame_id);
	for(double v : msg.orientation)
		w.put<double>(v);
	for(double v : msg.orientation_covariance)
		w.put<double>(v);
	for(double v : msg.angular_velocity)
		w.put<double>(v);
	for(double v : msg.angular_velocity_covariance)
		w.put<double>(v);
	for(double v : msg.linear_acceleration)
		w.put<double>(v);
	for(double v : msg.linear_acceleration_covariance)
		w.put<double>(v);
	return out;
}

// ---------------------------------------------------------------------------
// Ros1BagFileReader
// ---------------------------------------------------------------------------

namespace
{

const char* const kRos1StringType = "std_msgs/String";
const char* const kRos1StringMd5 = "992ce8a1687cec8c8bd883ec73ca41d1";
const char* const kRos1StringDefinition = "string data\n";

// "sensor_msgs/PointCloud2" -> "PointCloud2", as McapReader compares schema names.
std::string_view typeName(std::string_view type)
{
	const auto slash = type.rfind('/');
	return slash == std::string_view::npos ? type : type.substr(slash + 1);
}

// livox_ros_driver/CustomMsg, the same mapping as decodeLivoxCustomMsg(): ring =
// line, laser_id = tag, intensity = reflectivity, time = timebase + offset_time.
std::vector<McapPoint> decodeRos1LivoxCustomMsg(const uint8_t* data, size_t size, std::string* warning)
{
	ByteReader r{data, size};
	uint32_t seq = 0;
	uint64_t stamp_ns = 0;
	std::string frame_id;
	readHeader(r, seq, stamp_ns, frame_id);
	const uint64_t timebase = r.get<uint64_t>();
	r.get<uint32_t>(); // point_num
	r.get<uint8_t>(); // lidar_id
	r.bytes(3); // rsvd
	const uint32_t n = r.get<uint32_t>();
	std::vector<McapPoint> points;
	if(!r.ok)
	{
		if(warning && warning->empty())
			*warning = "truncated CustomMsg header";
		return points;
	}
	points.reserve(n);
	const double timebase_s = static_cast<double>(timebase) * 1e-9;
	for(uint32_t i = 0; i < n; ++i)
	{
		const uint32_t offset_time = r.get<uint32_t>();
		McapPoint p{};
		p.x = r.get<float>();
		p.y = r.get<float>();
		p.z = r.get<float>();
		p.intensity = static_cast<float>(r.get<uint8_t>());
		p.laser_id = r.get<uint8_t>();
		p.ring = r.get<uint8_t>();
		if(!r.ok)
		{
			if(warning && warning->empty())
				*warning = "truncated CustomMsg point data at index " + std::to_string(i);
			break;
		}
		p.timestamp = timebase_s + static_cast<double>(offset_time) * 1e-9;
		points.push_back(p);
	}
	return points;
}

} // namespace

Ros1BagFileReader::Ros1BagFileReader(const std::filesystem::path& path, const McapReaderOptions& options)
	: reader_(path)
	, options_(options)
{
	if(!reader_.isOpen())
	{
		error_ = reader_.error();
		return;
	}
	for(const auto& [id, c] : reader_.connections())
	{
		// One topic can be recorded through several connections (publishers).
		const bool seen = std::any_of(channels_.begin(), channels_.end(), [&](const ChannelInfo& ch) { return ch.topic == c.topic; });
		if(!seen)
			channels_.push_back({c.topic, c.type, "ros1", 0});
	}
	if(channels_.empty())
	{
		error_ = "the bag has no index section (still recording or not reindexed: run `rosbag reindex`)";
		return;
	}
	std::sort(channels_.begin(), channels_.end(), [](const ChannelInfo& a, const ChannelInfo& b) { return a.topic < b.topic; });

	// Same rules as McapFileReader: a requested topic must exist; otherwise the
	// single channel of a matching type is used, and ambiguity is an error for the
	// streams that must be unique.
	const auto resolve = [&](const std::string& requested, std::initializer_list<std::string_view> types, bool required_unique, std::string& out)
	{
		if(!requested.empty())
		{
			if(std::none_of(channels_.begin(), channels_.end(), [&](const ChannelInfo& c) { return c.topic == requested; }))
			{
				error_ = "topic '" + requested + "' is not present in the bag";
				return false;
			}
			out = requested;
			return true;
		}
		std::vector<const ChannelInfo*> candidates;
		for(const auto& c : channels_)
			if(std::find(types.begin(), types.end(), typeName(c.schema_name)) != types.end())
				candidates.push_back(&c);
		if(candidates.empty())
			return true;
		if(candidates.size() > 1)
		{
			if(!required_unique)
				return true;
			error_ = "found " + std::to_string(candidates.size()) + " candidate topics (";
			for(size_t i = 0; i < candidates.size(); ++i)
				error_ += (i ? ", " : "") + candidates[i]->topic;
			error_ += ") - pick one explicitly";
			return false;
		}
		out = candidates.front()->topic;
		return true;
	};
	if(!resolve(options.lidar_topic, {"PointCloud2", "CustomMsg"}, true, topics_.lidar) ||
	   !resolve(options.imu_topic, {"Imu"}, true, topics_.imu) || !resolve(options.sn_topic, {"String"}, false, topics_.sn))
		return;

	if(!topics_.lidar.empty())
	{
		const auto it = std::find_if(channels_.begin(), channels_.end(), [&](const ChannelInfo& c) { return c.topic == topics_.lidar; });
		const auto type = typeName(it->schema_name);
		if(type == "PointCloud2")
			topics_.lidar_schema = LidarSchema::PointCloud2;
		else if(type == "CustomMsg")
			topics_.lidar_schema = LidarSchema::LivoxCustomMsg;
		else
		{
			error_ = "topic '" + topics_.lidar + "' carries " + it->schema_name + ", not a point cloud";
			return;
		}
	}
	open_ = true;
}

bool Ros1BagFileReader::isOpen() const
{
	return open_;
}

const std::string& Ros1BagFileReader::error() const
{
	return error_;
}

const ResolvedTopics& Ros1BagFileReader::topics() const
{
	return topics_;
}

const std::vector<ChannelInfo>& Ros1BagFileReader::channels() const
{
	return channels_;
}

bool Ros1BagFileReader::timeOrdered() const
{
	return true;
}

const std::string& Ros1BagFileReader::lidarLayoutWarning() const
{
	return warning_;
}

bool Ros1BagFileReader::read(const Callbacks& callbacks)
{
	if(!open_)
		return false;
	Ros1PointCloud2 cloud;
	Ros1Imu imu;
	const bool ok = reader_.forEachMessage(
		[&](const Ros1Connection& c, uint64_t time_ns, const uint8_t* data, size_t size)
		{
			if(c.topic == topics_.lidar && callbacks.onPointCloud)
			{
				std::vector<McapPoint> points;
				if(topics_.lidar_schema == LidarSchema::LivoxCustomMsg)
				{
					points = decodeRos1LivoxCustomMsg(data, size, &warning_);
				}
				else if(!decodeRos1PointCloud2(data, size, cloud))
				{
					if(warning_.empty())
						warning_ = "truncated PointCloud2 message";
				}
				else if(cloud.is_bigendian)
				{
					if(warning_.empty())
						warning_ = "PointCloud2 is big-endian, which is not supported";
				}
				else
				{
					Pc2Layout msg;
					for(const auto& f : cloud.fields)
						addPc2Field(msg, f.name, f.offset, static_cast<Pc2Type>(f.datatype), f.count);
					msg.point_step = cloud.point_step;
					points = decodePc2Points(
						msg,
						static_cast<uint32_t>(cloud.fields.size()),
						static_cast<double>(cloud.stamp_ns) * 1e-9,
						cloud.data.data(),
						cloud.data.size(),
						options_.lidar_preset,
						&warning_);
				}
				if(!points.empty())
					callbacks.onPointCloud(time_ns, std::move(points));
			}
			else if(c.topic == topics_.imu && callbacks.onImu)
			{
				if(decodeRos1Imu(data, size, imu))
				{
					McapImuSample s;
					s.timestamp = static_cast<double>(imu.stamp_ns) * 1e-9;
					s.gyro_x = static_cast<float>(imu.angular_velocity[0]);
					s.gyro_y = static_cast<float>(imu.angular_velocity[1]);
					s.gyro_z = static_cast<float>(imu.angular_velocity[2]);
					s.acc_x = static_cast<float>(imu.linear_acceleration[0]);
					s.acc_y = static_cast<float>(imu.linear_acceleration[1]);
					s.acc_z = static_cast<float>(imu.linear_acceleration[2]);
					callbacks.onImu(s);
				}
			}
			else if(c.topic == topics_.sn && callbacks.onSn)
			{
				ByteReader r{data, size};
				const std::string sn = r.str();
				if(r.ok)
					callbacks.onSn(time_ns, sn);
			}
			return true;
		});
	if(!ok)
		error_ = reader_.error();
	return ok;
}

// ---------------------------------------------------------------------------
// Ros1BagFileWriter
// ---------------------------------------------------------------------------

Ros1BagFileWriter::Ros1BagFileWriter(const std::filesystem::path& path, const McapWriterOptions& options)
	: bag_(path)
	, options_(options)
{
	lidar_ = bag_.addConnection(options_.lidar_topic, kRos1PointCloud2Type, kRos1PointCloud2Md5, kRos1PointCloud2Definition);
	imu_ = bag_.addConnection(options_.imu_topic, kRos1ImuType, kRos1ImuMd5, kRos1ImuDefinition);
}

bool Ros1BagFileWriter::isOpen() const
{
	return bag_.isOpen();
}

const std::string& Ros1BagFileWriter::error() const
{
	return bag_.error();
}

void Ros1BagFileWriter::writePointCloud(uint64_t timestamp_ns, const std::vector<McapPoint>& points)
{
	Ros1PointCloud2 msg;
	msg.seq = lidar_seq_++;
	msg.stamp_ns = timestamp_ns;
	msg.frame_id = options_.pointcloud_frame_id.empty() ? options_.frame_id : options_.pointcloud_frame_id;
	msg.width = static_cast<uint32_t>(points.size());
	for(const auto& f : pointCloudFields(options_.lidar_layout))
		msg.fields.push_back({f.name, f.offset, f.datatype, 1});
	msg.point_step = pointCloudPointStep(options_.lidar_layout);
	msg.row_step = msg.width * msg.point_step;
	msg.data = packPointCloud(points, options_.lidar_layout);
	bag_.write(lidar_, timestamp_ns, encodeRos1PointCloud2(msg));
}

void Ros1BagFileWriter::writeImuSample(const McapImuSample& sample)
{
	Ros1Imu msg;
	msg.seq = imu_seq_++;
	msg.stamp_ns = static_cast<uint64_t>(sample.timestamp * 1e9);
	msg.frame_id = options_.frame_id;
	msg.angular_velocity[0] = sample.gyro_x;
	msg.angular_velocity[1] = sample.gyro_y;
	msg.angular_velocity[2] = sample.gyro_z;
	msg.linear_acceleration[0] = sample.acc_x;
	msg.linear_acceleration[1] = sample.acc_y;
	msg.linear_acceleration[2] = sample.acc_z;
	bag_.write(imu_, msg.stamp_ns, encodeRos1Imu(msg));
}

void Ros1BagFileWriter::writeImu(const std::vector<McapImuSample>& imu)
{
	for(const auto& s : imu)
		writeImuSample(s);
}

void Ros1BagFileWriter::writeSn(uint64_t timestamp_ns, const std::string& data)
{
	// Added on first use, so a bag without a serial number has no empty topic.
	if(!sn_)
		sn_ = bag_.addConnection(options_.sn_topic, kRos1StringType, kRos1StringMd5, kRos1StringDefinition);
	std::vector<uint8_t> out;
	ByteWriter w{out};
	w.str(data);
	bag_.write(*sn_, timestamp_ns, out);
}

bool Ros1BagFileWriter::close()
{
	return bag_.close();
}

} // namespace rosbags
