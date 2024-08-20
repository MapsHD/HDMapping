#include <string>
#include <vector>
#include <Eigen/Eigen>

namespace fs = std::filesystem;

inline bool exportLaz(const std::string &filename,
               const std::vector<Eigen::Vector3d> &pointcloud,
               const std::vector<unsigned short> &intensity,
               const std::vector<double> &timestamps,
               double offset_x, double offset_y, double offset_alt)
{

    constexpr float scale = 0.0001f; // one tenth of milimeter
    // find max
    Eigen::Vector3d _max(-1000000000.0, -1000000000.0, -1000000000.0);
    Eigen::Vector3d _min(1000000000.0, 1000000000.0, 1000000000.0);

    for (auto &p : pointcloud)
    {
        if (p.x() < _min.x())
        {
            _min.x() = p.x();
        }
        if (p.y() < _min.y())
        {
            _min.y() = p.y();
        }
        if (p.z() < _min.z())
        {
            _min.z() = p.z();
        }

        if (p.x() > _max.x())
        {
            _max.x() = p.x();
        }
        if (p.y() > _max.y())
        {
            _max.y() = p.y();
        }
        if (p.z() > _max.z())
        {
            _max.z() = p.z();
        }
    }

    // create the writer
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
    header->point_data_format = 1;
    header->point_data_record_length = 0;
    header->number_of_point_records = pointcloud.size();
    header->number_of_points_by_return[0] = pointcloud.size();
    header->number_of_points_by_return[1] = 0;
    header->point_data_record_length = 28;
    header->x_scale_factor = scale;
    header->y_scale_factor = scale;
    header->z_scale_factor = scale;

    header->max_x = _max.x() + offset_x;
    header->min_x = _min.x() + offset_x;
    header->max_y = _max.y() + offset_y;
    header->min_y = _min.y() + offset_y;
    header->max_z = _max.z() + offset_alt;
    header->min_z = _min.z() + offset_alt;

    header->x_offset = offset_x;
    header->y_offset = offset_y;
    header->z_offset = offset_alt;

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

    for (int i = 0; i < pointcloud.size(); i++)
    {
        point->intensity = intensity[i];

        const auto &p = pointcloud[i];
        point->gps_time = timestamps[i] * 1e9;

        p_count++;
        coordinates[0] = p.x() + offset_x;
        coordinates[1] = p.y() + offset_y;
        coordinates[2] = p.z() + offset_alt;
        if (laszip_set_coordinates(laszip_writer, coordinates))
        {
            fprintf(stderr, "DLL ERROR: setting coordinates for point %I64d\n", p_count);
            return false;
        }

        // p.SetIntensity(pp.intensity);

        // if (i < intensity.size()) {
        //     point->intensity = intensity[i];
        // }
        // laszip_set_point

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

inline void adjustHeader(laszip_header *header, const Eigen::Affine3d &m_pose, const Eigen::Vector3d &offset_in)
{
    Eigen::Vector3d max{header->max_x, header->max_y, header->max_z};
    Eigen::Vector3d min{header->min_x, header->min_y, header->min_z};

    //max -= offset_in;
    //min -= offset_in;

    Eigen::Vector3d adj_max = m_pose * max + offset_in;
    Eigen::Vector3d adj_min = m_pose * min + offset_in;

    header->max_x = adj_max.x();
    header->max_y = adj_max.y();
    header->max_z = adj_max.z();

    header->min_x = adj_min.x();
    header->min_y = adj_min.y();
    header->min_z = adj_min.z();
}

inline void adjustPoint(laszip_F64 output_coordinates[3], laszip_F64 input_coordinates[3], const Eigen::Affine3d &m_pose, const Eigen::Vector3d &offset)
{
    Eigen::Vector3d i{input_coordinates[0], input_coordinates[1], input_coordinates[2]};
    //i -= offset;
    Eigen::Vector3d o = m_pose * i;
    //o += offset;
    output_coordinates[0] = o.x();
    output_coordinates[1] = o.y();
    output_coordinates[2] = o.z();
}

inline void save_processed_pc(const fs::path &file_path_in, const fs::path &file_path_put, const Eigen::Affine3d &m_pose, const Eigen::Vector3d &offset, bool override_compressed = false)
{
    std::cout << "processing: " << file_path_in << std::endl;

    laszip_POINTER laszip_reader;
    if (laszip_create(&laszip_reader))
    {
        fprintf(stderr, "DLL ERROR: creating laszip reader\n");
        std::abort();
    }

    laszip_POINTER laszip_writer;
    if (laszip_create(&laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: creating laszip reader\n");
        std::abort();
    }

    const std::string file_name_in = file_path_in.string();
    const std::string file_name_out = file_path_put.string();

    laszip_BOOL is_compressed = 0;
    if (laszip_open_reader(laszip_reader, file_name_in.c_str(), &is_compressed))
    {
        fprintf(stderr, "DLL ERROR: opening laszip reader for '%s'\n", file_name_in.c_str());
        std::abort();
    }
    std::cout << "compressed : " << is_compressed << std::endl;

    laszip_header *header;

    if (laszip_get_header_pointer(laszip_reader, &header))
    {
        fprintf(stderr, "DLL ERROR: getting header pointer from laszip reader\n");
        std::abort();
    }

    std::cout << "header min before:" << std::endl;
    std::cout << header->min_x << " " << header->min_y << " " << header->min_z << std::endl;

    std::cout << "header max before:" << std::endl;
    std::cout << header->max_x << " " << header->max_y << " " << header->max_z << std::endl;

    adjustHeader(header, m_pose, offset);

    std::cout << "header min after:" << std::endl;
    std::cout << header->min_x << " " << header->min_y << " " << header->min_z << std::endl;

    std::cout << "header max after:" << std::endl;
    std::cout << header->max_x << " " << header->max_y << " " << header->max_z << std::endl;

    if (laszip_set_header(laszip_writer, header))
    {
        fprintf(stderr, "DLL ERROR: setting header pointer from laszip reader\n");
        std::abort();
    }

    fprintf(stderr, "file '%s' contains %u points\n", file_name_in.c_str(), header->number_of_point_records);

    if (override_compressed)
    {
        is_compressed = 0;
        std::cout << "compressed : " << is_compressed << std::endl;
    }

    if (laszip_open_writer(laszip_writer, file_name_out.c_str(), is_compressed))
    {
        fprintf(stderr, "DLL ERROR: opening laszip writer for '%s'\n", file_name_out.c_str());
        return;
    }

    laszip_point *input_point;
    if (laszip_get_point_pointer(laszip_reader, &input_point))
    {
        fprintf(stderr, "DLL ERROR: getting point pointer from laszip reader\n");
        std::abort();
    }

    laszip_point *output_point;
    if (laszip_get_point_pointer(laszip_writer, &output_point))
    {
        fprintf(stderr, "DLL ERROR: getting point pointer from laszip reader\n");
        std::abort();
    }

    for (int i = 0; i < header->number_of_point_records; i++)
    {
        if (laszip_read_point(laszip_reader))
        {
            fprintf(stderr, "DLL ERROR: reading point %u\n", i);
            std::abort();
        }

        laszip_F64 input_coordinates[3];
        if (laszip_get_coordinates(laszip_reader, input_coordinates))
        {
            fprintf(stderr, "DLL ERROR: laszip_set_coordinates %u\n", i);
            std::abort();
        }

        laszip_F64 output_coordinates[3];
        adjustPoint(output_coordinates, input_coordinates, m_pose, offset);

        if (laszip_set_coordinates(laszip_writer, output_coordinates))
        {
            fprintf(stderr, "DLL ERROR: laszip_set_coordinates %u\n", i);
            std::abort();
        }
        output_point->intensity = input_point->intensity;
        output_point->classification = input_point->classification;
        output_point->num_extra_bytes = input_point->num_extra_bytes;
        output_point->gps_time = input_point->gps_time;
        memcpy(output_point->extra_bytes, input_point->extra_bytes, output_point->num_extra_bytes);

        if (laszip_write_point(laszip_writer))
        {
            fprintf(stderr, "DLL ERROR: writing point %I64d\n", i);
            return;
        }
    }

    // close the reader
    if (laszip_close_reader(laszip_reader))
    {
        fprintf(stderr, "DLL ERROR: closing laszip reader\n");
        return;
    }

    // destroy the reader

    if (laszip_destroy(laszip_reader))
    {
        fprintf(stderr, "DLL ERROR: destroying laszip reader\n");
        return;
    }

    laszip_I64 p_count{0};
    if (laszip_get_point_count(laszip_writer, &p_count))
    {
        fprintf(stderr, "DLL ERROR: getting point count\n");
        return;
    }

    fprintf(stderr, "successfully written %ld points\n", p_count);

    // close the writer

    if (laszip_close_writer(laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: closing laszip writer\n");
        return;
    }

    // destroy the writer

    // ToDo --> solve it
    if (laszip_destroy(laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: destroying laszip writer\n");
        return;
    }

    std::cout << "saving to " << file_path_put << std::endl;
}