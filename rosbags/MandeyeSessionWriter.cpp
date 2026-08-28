#include "MandeyeSessionWriter.h"

#include <laszip/laszip_api.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>

namespace rosbags
{

namespace
{

// Same scale mandeye/HDMapping LAZ files use (see LazWriter::open in
// core/include/Core/export_laz.h): 0.1 mm, i.e. an int32 X/Y/Z spans +-214 km,
// far beyond any sensor-local coordinate.
constexpr double kCoordScale = 0.0001;

std::string chunkFileName(const char* prefix, int index, const char* ext)
{
	std::array<char, 64> buffer{};
	std::snprintf(buffer.data(), buffer.size(), "%s%04d%s", prefix, index, ext);
	return std::string(buffer.data());
}

} // namespace

struct MandeyeSessionWriter::Impl
{
	std::filesystem::path dir;
	std::string error;
	bool open = false;

	laszip_POINTER writer = nullptr;
	laszip_point* point = nullptr;
	int currentChunk = -1;

	uint64_t points = 0;
	uint64_t imuSamples = 0;

	bool fail(std::string message)
	{
		if(error.empty())
			error = std::move(message);
		return false;
	}
};

MandeyeSessionWriter::MandeyeSessionWriter(const std::filesystem::path& dir)
	: impl_(std::make_unique<Impl>())
{
	impl_->dir = dir;

	std::error_code ec;
	std::filesystem::create_directories(dir, ec);
	if(ec)
	{
		impl_->fail("failed to create output directory " + dir.string() + ": " + ec.message());
		return;
	}
	if(!std::filesystem::is_directory(dir))
	{
		impl_->fail(dir.string() + " exists but is not a directory");
		return;
	}

	impl_->open = true;
}

MandeyeSessionWriter::~MandeyeSessionWriter()
{
	if(impl_ && impl_->writer)
		endChunk();
}

bool MandeyeSessionWriter::isOpen() const
{
	return impl_ && impl_->open;
}

const std::string& MandeyeSessionWriter::error() const
{
	return impl_->error;
}

bool MandeyeSessionWriter::beginChunk(int index)
{
	if(!isOpen())
		return false;
	if(impl_->writer && !endChunk())
		return false;

	const auto path = impl_->dir / chunkFileName("lidar", index, ".laz");

	if(laszip_create(&impl_->writer))
	{
		impl_->writer = nullptr;
		return impl_->fail("failed to create laszip writer");
	}

	laszip_header* header = nullptr;
	if(laszip_get_header_pointer(impl_->writer, &header))
		return impl_->fail("failed to get laszip header pointer");

	header->file_source_ID = 4711;
	header->global_encoding = (1 << 0);
	header->version_major = 1;
	header->version_minor = 2;
	header->point_data_format = 1; // XYZ + intensity + user_data + gps_time
	header->point_data_record_length = 28;
	header->number_of_point_records = 0;
	header->number_of_points_by_return[0] = 0;
	header->number_of_points_by_return[1] = 0;
	header->x_scale_factor = kCoordScale;
	header->y_scale_factor = kCoordScale;
	header->z_scale_factor = kCoordScale;
	// Bag points are sensor-local, so no georeferencing offset is applied.
	header->x_offset = 0.0;
	header->y_offset = 0.0;
	header->z_offset = 0.0;

	if(laszip_open_writer(impl_->writer, path.string().c_str(), /*compress=*/1))
		return impl_->fail("failed to open laszip writer for " + path.string());

	if(laszip_get_point_pointer(impl_->writer, &impl_->point))
		return impl_->fail("failed to get laszip point pointer");

	impl_->currentChunk = index;
	return true;
}

bool MandeyeSessionWriter::addPoints(const std::vector<McapPoint>& points)
{
	if(!isOpen())
		return false;
	if(!impl_->writer)
		return impl_->fail("addPoints() called with no chunk open");

	for(const auto& p : points)
	{
		impl_->point->intensity = static_cast<laszip_U16>(std::clamp(p.intensity, 0.0f, 65535.0f));
		impl_->point->return_number = 1;
		impl_->point->number_of_returns = 1;
		// Unscaled: load_point_cloud() assigns p.timestamp = point->gps_time directly.
		impl_->point->gps_time = p.timestamp;
		// Single-lidar sessions only, so the lidar id load_point_cloud() reads out of
		// user_data is always 0 -- and it must be written, because a session with a
		// non-zero id but no matching calibration entry has its points dropped.
		impl_->point->user_data = 0;

		laszip_F64 coordinates[3] = {p.x, p.y, p.z};
		if(laszip_set_coordinates(impl_->writer, coordinates))
			return impl_->fail("failed to set laszip coordinates");
		if(laszip_write_point(impl_->writer))
			return impl_->fail("failed to write laszip point");
		if(laszip_update_inventory(impl_->writer))
			return impl_->fail("failed to update laszip inventory");

		++impl_->points;
	}
	return true;
}

bool MandeyeSessionWriter::endChunk()
{
	if(!impl_->writer)
		return true;

	bool ok = true;
	if(laszip_close_writer(impl_->writer))
		ok = impl_->fail("failed to close laszip writer for chunk " + std::to_string(impl_->currentChunk));
	if(laszip_destroy(impl_->writer))
		ok = impl_->fail("failed to destroy laszip writer for chunk " + std::to_string(impl_->currentChunk));

	impl_->writer = nullptr;
	impl_->point = nullptr;
	impl_->currentChunk = -1;
	return ok;
}

bool MandeyeSessionWriter::writeImuChunk(int index, const std::vector<McapImuSample>& samples)
{
	if(!isOpen())
		return false;

	const auto path = impl_->dir / chunkFileName("imu", index, ".csv");
	std::ofstream out(path);
	if(!out.is_open())
		return impl_->fail("failed to open " + path.string());

	// Column set and order per concatenate_multi_livox.cpp; load_imu() looks the
	// columns up by name, so the order is only a convention.
	out << "timestamp timestampUnix accX accY accZ gyroX gyroY gyroZ\n";
	out << std::setprecision(std::numeric_limits<float>::max_digits10);

	for(const auto& s : samples)
	{
		// timestampUnix is 0: McapImuSample has no slot for load_imu()'s second
		// (wall-clock) timestamp, so the exporter never carried it into the bag.
		out << static_cast<uint64_t>(s.timestamp * 1e9) << " " << 0 << " " << s.acc_x << " " << s.acc_y << " " << s.acc_z << " " << s.gyro_x
			<< " " << s.gyro_y << " " << s.gyro_z << "\n";
		++impl_->imuSamples;
	}

	out.flush();
	if(!out)
		return impl_->fail("failed to write " + path.string());
	return true;
}

uint64_t MandeyeSessionWriter::pointsWritten() const
{
	return impl_->points;
}

uint64_t MandeyeSessionWriter::imuSamplesWritten() const
{
	return impl_->imuSamples;
}

} // namespace rosbags
