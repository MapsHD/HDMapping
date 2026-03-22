#include <pch/pch.h>

#include "Core/color_las_loader.h"

#include <spdlog/spdlog.h>

std::vector<mandeye::Point> mandeye::load(const std::string& lazFile)
{
    std::vector<Point> points;
    laszip_POINTER laszip_reader;
    if (laszip_create(&laszip_reader))
    {
        spdlog::error("DLL ERROR: creating laszip reader");
        std::abort();
    }

    laszip_BOOL is_compressed = 0;
    if (laszip_open_reader(laszip_reader, lazFile.c_str(), &is_compressed))
    {
        spdlog::error("DLL ERROR: opening laszip reader for '{}'", lazFile);
        std::abort();
    }
    std::cout << "compressed : " << is_compressed << std::endl;
    laszip_header* header;

    if (laszip_get_header_pointer(laszip_reader, &header))
    {
        spdlog::error("DLL ERROR: getting header pointer from laszip reader");
        std::abort();
    }

    laszip_I64 npoints = (header->number_of_point_records ? header->number_of_point_records : header->extended_number_of_point_records);

    spdlog::info("file '{}' contains {} points", lazFile, npoints);

    laszip_point* point;

    if (laszip_get_point_pointer(laszip_reader, &point))
    {
        spdlog::error("DLL ERROR: getting point pointer from laszip reader");
    }

    laszip_I64 p_count = 0;

    while (p_count < npoints)
    {
        if (laszip_read_point(laszip_reader))
        {
            spdlog::error("DLL ERROR: reading point {}", p_count);
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

bool mandeye::saveLaz(const std::string& filename, const std::vector<mandeye::PointRGB>& buffer)
{
    constexpr float scale = 0.0001f; // one tenth of milimeter
    // find max
    double max_x{ std::numeric_limits<double>::lowest() };
    double max_y{ std::numeric_limits<double>::lowest() };
    double max_z{ std::numeric_limits<double>::lowest() };

    double min_x{ std::numeric_limits<double>::max() };
    double min_y{ std::numeric_limits<double>::max() };
    double min_z{ std::numeric_limits<double>::max() };

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
        spdlog::error("DLL ERROR: creating laszip writer");
        return false;
    }

    // get a pointer to the header of the writer so we can populate it

    laszip_header* header;

    if (laszip_get_header_pointer(laszip_writer, &header))
    {
        spdlog::error("DLL ERROR: getting header pointer from laszip writer");
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
        spdlog::error("DLL ERROR: opening laszip writer for '{}'", filename);
        return false;
    }

    spdlog::info("writing file '{}' {}compressed", filename, (compress ? "" : "un"));

    // get a pointer to the point of the writer that we will populate and write

    laszip_point* point;
    if (laszip_get_point_pointer(laszip_writer, &point))
    {
        spdlog::error("DLL ERROR: getting point pointer from laszip writer");
        return false;
    }

    laszip_I64 p_count = 0;
    laszip_F64 coordinates[3];

    for (int i = 0; i < buffer.size(); i++)
    {
        const auto& p = buffer.at(i);
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
            spdlog::error("DLL ERROR: setting coordinates for point {}", p_count);
            return false;
        }

        if (laszip_write_point(laszip_writer))
        {
            spdlog::error("DLL ERROR: writing point {}", p_count);
            return false;
        }
    }

    if (laszip_get_point_count(laszip_writer, &p_count))
    {
        spdlog::error("DLL ERROR: getting point count");
        return false;
    }

    spdlog::info("successfully written {} points\n", p_count);

    // close the writer

    if (laszip_close_writer(laszip_writer))
    {
        spdlog::error("DLL ERROR: closing laszip writer");
        return false;
    }

    // destroy the writer

    if (laszip_destroy(laszip_writer))
    {
        spdlog::error("DLL ERROR: destroying laszip writer");
        return false;
    }

    std::cout << "exportLaz DONE" << std::endl;
    return true;
}
