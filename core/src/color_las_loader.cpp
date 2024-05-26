#include "color_las_loader.h"
#include <iostream>
#include <vector>

std::vector<mandeye::Point> mandeye::load(const std::string &lazFile)
{
	std::vector<Point> points;
	laszip_POINTER laszip_reader;
	if (laszip_create(&laszip_reader))
	{
		fprintf(stderr, "DLL ERROR: creating laszip reader\n");
		std::abort();
	}

	laszip_BOOL is_compressed = 0;
	if (laszip_open_reader(laszip_reader, lazFile.c_str(), &is_compressed))
	{
		fprintf(stderr, "DLL ERROR: opening laszip reader for '%s'\n", lazFile.c_str());
		std::abort();
	}
	std::cout << "compressed : " << is_compressed << std::endl;
	laszip_header *header;

	if (laszip_get_header_pointer(laszip_reader, &header))
	{
		fprintf(stderr, "DLL ERROR: getting header pointer from laszip reader\n");
		std::abort();
	}
	// fprintf(stderr, "file '%s' contains %u points\n", lazFile.c_str(), header->number_of_point_records);

	// laszip_point *point;
	// if (laszip_get_point_pointer(laszip_reader, &point))
	//{
	//	fprintf(stderr, "DLL ERROR: getting point pointer from laszip reader\n");
	//	std::abort();
	// }

	/*std::cout << "header->number_of_point_records: " << header->number_of_point_records << std::endl;
	std::cout << "header->extended_number_of_point_records: " << header->extended_number_of_point_records << std::endl;

	//for (int j = 0; j < header->number_of_point_records; j++)
	for (int j = 0; j < header->extended_number_of_point_records; j++)
	{
		if (laszip_read_point(laszip_reader))
		{
			fprintf(stderr, "DLL ERROR: reading point %u\n", j);
			std::abort();
		}

		Point p;
		p.point.x() = header->x_offset + header->x_scale_factor * static_cast<double>(point->X);
		p.point.y() = header->y_offset + header->y_scale_factor * static_cast<double>(point->Y);
		p.point.z() = header->z_offset + header->z_scale_factor * static_cast<double>(point->Z);
		p.timestamp = point->gps_time;
		p.intensity = point->intensity;

		points.emplace_back(p);
	}*/

	laszip_I64 npoints = (header->number_of_point_records ? header->number_of_point_records : header->extended_number_of_point_records);

	// report how many points the file has

	fprintf(stderr, "file '%s' contains %I64d points\n", lazFile.c_str(), npoints);

	laszip_point *point;

	if (laszip_get_point_pointer(laszip_reader, &point))
	{
		fprintf(stderr, "DLL ERROR: getting point pointer from laszip reader\n");
	}

	laszip_I64 p_count = 0;

	while (p_count < npoints)
	{
		if (laszip_read_point(laszip_reader))
		{
			fprintf(stderr, "DLL ERROR: reading point %I64d\n", p_count);
	
		}
		Point p;
		p.point.x() = header->x_offset + header->x_scale_factor * static_cast<double>(point->X);
		p.point.y() = header->y_offset + header->y_scale_factor * static_cast<double>(point->Y);
		p.point.z() = header->z_offset + header->z_scale_factor * static_cast<double>(point->Z);
		p.timestamp = point->gps_time;
		p.intensity = point->intensity;

		points.emplace_back(p);

		p_count++;
	}

	return points;
}

bool mandeye::saveLaz(const std::string &filename, const std::vector<mandeye::PointRGB> &buffer)
{

	constexpr float scale = 0.0001f; // one tenth of milimeter
	// find max
	double max_x{std::numeric_limits<double>::lowest()};
	double max_y{std::numeric_limits<double>::lowest()};
	double max_z{std::numeric_limits<double>::lowest()};

	double min_x{std::numeric_limits<double>::max()};
	double min_y{std::numeric_limits<double>::max()};
	double min_z{std::numeric_limits<double>::max()};

	for (auto p : buffer)
	{
		double x = p.point.x();
		double y = p.point.y();
		double z = p.point.z();

		max_x = std::max(max_x, x);
		max_y = std::max(max_y, y);
		max_z = std::max(max_z, z);

		min_x = std::min(min_x, x);
		min_y = std::min(min_y, y);
		min_z = std::min(min_z, z);
	}

	std::cout << "processing: " << filename << "points " << buffer.size() << std::endl;

	laszip_POINTER laszip_writer;
	if (laszip_create(&laszip_writer))
	{
		fprintf(stderr, "DLL ERROR: creating laszip writer\n");
		return false;
	}

	// get a pointer to the header of the writer so we can populate it

	laszip_header *header;

	if (laszip_get_header_pointer(laszip_writer, &header))
	{
		fprintf(stderr, "DLL ERROR: getting header pointer from laszip writer\n");
		return false;
	}

	// populate the header

	header->file_source_ID = 4711;
	header->global_encoding = (1 << 0); // see LAS specification for details
	header->version_major = 1;
	header->version_minor = 2;
	//    header->file_creation_day = 120;
	//    header->file_creation_year = 2013;
	header->point_data_format = 2;
	header->point_data_record_length = 0;
	header->number_of_point_records = buffer.size();
	header->number_of_points_by_return[0] = buffer.size();
	header->number_of_points_by_return[1] = 0;
	header->point_data_record_length = 26;
	header->x_scale_factor = scale;
	header->y_scale_factor = scale;
	header->z_scale_factor = scale;

	header->max_x = max_x;
	header->min_x = min_x;
	header->max_y = max_y;
	header->min_y = min_y;
	header->max_z = max_z;
	header->min_z = min_z;

	// optional: use the bounding box and the scale factor to create a "good" offset
	// open the writer
	laszip_BOOL compress = (strstr(filename.c_str(), ".laz") != 0);

	if (laszip_open_writer(laszip_writer, filename.c_str(), compress))
	{
		fprintf(stderr, "DLL ERROR: opening laszip writer for '%s'\n", filename.c_str());
		return false;
	}

	fprintf(stderr, "writing file '%s' %scompressed\n", filename.c_str(), (compress ? "" : "un"));

	// get a pointer to the point of the writer that we will populate and write

	laszip_point *point;
	if (laszip_get_point_pointer(laszip_writer, &point))
	{
		fprintf(stderr, "DLL ERROR: getting point pointer from laszip writer\n");
		return false;
	}

	laszip_I64 p_count = 0;
	laszip_F64 coordinates[3];

	for (int i = 0; i < buffer.size(); i++)
	{

		const auto &p = buffer.at(i);
		point->intensity = p.intensity;
		point->rgb[0] = 255 * p.rgb[0];
		point->rgb[1] = 255 * p.rgb[1];
		point->rgb[2] = 255 * p.rgb[2];
		point->rgb[3] = 255 * p.rgb[3];

		point->gps_time = p.timestamp * 1e-9;
		//		point->user_data = p.line_id;
		//		point->classification = p.tag;
		p_count++;
		coordinates[0] = p.point[0];
		coordinates[1] = p.point[1];
		coordinates[2] = p.point[2];
		if (laszip_set_coordinates(laszip_writer, coordinates))
		{
			fprintf(stderr, "DLL ERROR: setting coordinates for point %I64d\n", p_count);
			return false;
		}

		if (laszip_write_point(laszip_writer))
		{
			fprintf(stderr, "DLL ERROR: writing point %I64d\n", p_count);
			return false;
		}
	}

	if (laszip_get_point_count(laszip_writer, &p_count))
	{
		fprintf(stderr, "DLL ERROR: getting point count\n");
		return false;
	}

	fprintf(stderr, "successfully written %I64d points\n", p_count);

	// close the writer

	if (laszip_close_writer(laszip_writer))
	{
		fprintf(stderr, "DLL ERROR: closing laszip writer\n");
		return false;
	}

	// destroy the writer

	if (laszip_destroy(laszip_writer))
	{
		fprintf(stderr, "DLL ERROR: destroying laszip writer\n");
		return false;
	}

	std::cout << "exportLaz DONE" << std::endl;
	return true;
}