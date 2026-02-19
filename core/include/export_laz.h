#pragma once

#include <filesystem>
#include <iostream>
#include <session.h>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <laszip/laszip_api.h>

namespace fs = std::filesystem;

class LazWriter
{
public:
    LazWriter() = default;
    ~LazWriter()
    {
        if (writer_)
            close();
    }

    LazWriter(const LazWriter&) = delete;
    LazWriter& operator=(const LazWriter&) = delete;

    bool open(const std::string& filename,
              double offset_x, double offset_y, double offset_z)
    {
        constexpr double scale = 0.0001;

        if (laszip_create(&writer_))
        {
            fprintf(stderr, "DLL ERROR: creating laszip writer\n");
            return false;
        }

        laszip_header* header;
        if (laszip_get_header_pointer(writer_, &header))
        {
            fprintf(stderr, "DLL ERROR: getting header pointer from laszip writer\n");
            return false;
        }

        header->file_source_ID = 4711;
        header->global_encoding = (1 << 0);
        header->version_major = 1;
        header->version_minor = 2;
        header->point_data_format = 1;
        header->point_data_record_length = 28;
        header->number_of_point_records = 0;
        header->number_of_points_by_return[0] = 0;
        header->number_of_points_by_return[1] = 0;
        header->x_scale_factor = scale;
        header->y_scale_factor = scale;
        header->z_scale_factor = scale;
        header->x_offset = offset_x;
        header->y_offset = offset_y;
        header->z_offset = offset_z;

        laszip_BOOL compress = (strstr(filename.c_str(), ".laz") != 0);
        if (laszip_open_writer(writer_, filename.c_str(), compress))
        {
            fprintf(stderr, "DLL ERROR: opening laszip writer for '%s'\n", filename.c_str());
            return false;
        }

        fprintf(stderr, "writing %scompressed file '%s'\n", (compress ? "" : "un"), filename.c_str());

        if (laszip_get_point_pointer(writer_, &point_))
        {
            fprintf(stderr, "DLL ERROR: getting point pointer from laszip writer\n");
            return false;
        }

        return true;
    }

    bool writePoint(const Eigen::Vector3d& position,
                    unsigned short intensity,
                    double timestamp,
                    double offset_x, double offset_y, double offset_z)
    {
        point_->intensity = intensity;
        point_->return_number = 1;
        point_->number_of_returns = 1;
        point_->gps_time = timestamp * 1e9;

        laszip_F64 coordinates[3];
        coordinates[0] = position.x() + offset_x;
        coordinates[1] = position.y() + offset_y;
        coordinates[2] = position.z() + offset_z;

        if (laszip_set_coordinates(writer_, coordinates))
        {
            fprintf(stderr, "DLL ERROR: setting coordinates\n");
            return false;
        }

        if (laszip_write_point(writer_))
        {
            fprintf(stderr, "DLL ERROR: writing point\n");
            return false;
        }

        if (laszip_update_inventory(writer_))
        {
            fprintf(stderr, "DLL ERROR: updating inventory\n");
            return false;
        }

        return true;
    }

    bool close()
    {
        if (!writer_)
            return true;

        laszip_I64 p_count = 0;
        if (laszip_get_point_count(writer_, &p_count))
        {
            fprintf(stderr, "DLL ERROR: getting point count\n");
            return false;
        }

        fprintf(stderr, "successfully written %lld points\n", p_count);

        if (laszip_close_writer(writer_))
        {
            fprintf(stderr, "DLL ERROR: closing laszip writer\n");
            return false;
        }

        if (laszip_destroy(writer_))
        {
            fprintf(stderr, "DLL ERROR: destroying laszip writer\n");
            return false;
        }

        writer_ = nullptr;
        point_ = nullptr;
        return true;
    }

private:
    laszip_POINTER writer_ = nullptr;
    laszip_point* point_ = nullptr;
};

inline bool exportLaz(
    const std::string& filename,
    const std::vector<Eigen::Vector3d>& pointcloud,
    const std::vector<unsigned short>& intensity,
    const std::vector<double>& timestamps,
    double offset_x = 0.0,
    double offset_y = 0.0,
    double offset_alt = 0.0)
{
    LazWriter writer;
    if (!writer.open(filename, offset_x, offset_y, offset_alt))
        return false;

    for (size_t i = 0; i < pointcloud.size(); i++)
    {
        if (!writer.writePoint(pointcloud[i], intensity[i], timestamps[i],
                               offset_x, offset_y, offset_alt))
            return false;
    }

    return writer.close();
}


// inline Eigen::Vector3d adjustPoint(laszip_F64 input_coordinates[3], const Eigen::Affine3d &m_pose)
//{
//     Eigen::Vector3d i(input_coordinates[0], input_coordinates[1], input_coordinates[2]);
// i -= offset;
//    Eigen::Vector3d o = m_pose * i;

// std::cout << i.x() << " " << i.y() << " " << i.z() << " " << o.x() << " " << o.y() << " " << o.z() << std::endl;
// o += offset;
// output_coordinates[0] = o.x();
// output_coordinates[1] = o.y();
// output_coordinates[2] = o.z();
//    return o;
//}

inline void save_processed_pc(
    const fs::path& file_path_in,
    const fs::path& file_path_put,
    const Eigen::Affine3d& m_pose,
    const Eigen::Vector3d& offset,
    bool override_compressed = false)
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

    laszip_header* header;

    if (laszip_get_header_pointer(laszip_reader, &header))
    {
        fprintf(stderr, "DLL ERROR: getting header pointer from laszip reader\n");
        std::abort();
    }

    std::cout << "header min before:" << std::endl;
    std::cout << header->min_x << " " << header->min_y << " " << header->min_z << std::endl;

    std::cout << "header max before:" << std::endl;
    std::cout << header->max_x << " " << header->max_y << " " << header->max_z << std::endl;

    std::cout << "header before:" << std::endl;
    std::cout << header->x_offset << " " << header->y_offset << " " << header->z_offset << std::endl;

    header->x_offset = offset.x();
    header->y_offset = offset.y();
    header->z_offset = offset.z();

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

    laszip_point* input_point;
    if (laszip_get_point_pointer(laszip_reader, &input_point))
    {
        fprintf(stderr, "DLL ERROR: getting point pointer from laszip reader\n");
        std::abort();
    }

    laszip_point* output_point;
    if (laszip_get_point_pointer(laszip_writer, &output_point))
    {
        fprintf(stderr, "DLL ERROR: getting point pointer from laszip reader\n");
        std::abort();
    }

    for (uint32_t i = 0; i < header->number_of_point_records; i++)
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

        Eigen::Vector3d pg = Eigen::Vector3d(input_coordinates[0], input_coordinates[1], input_coordinates[2]);
        pg = m_pose * pg;
        // adjustPoint(input_coordinates, m_pose);
        laszip_F64 output_coordinates[3];
        output_coordinates[0] = pg.x();
        output_coordinates[1] = pg.y();
        output_coordinates[2] = pg.z();

        if (laszip_set_coordinates(laszip_writer, output_coordinates))
        // if (laszip_set_coordinates(laszip_writer, input_coordinates))
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
            fprintf(stderr, "DLL ERROR: writing point %u\n", i);
            return;
        }

        if (laszip_update_inventory(laszip_writer))
        {
            fprintf(stderr, "DLL ERROR: updating inventory for point %u\n", i);
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

    laszip_I64 p_count{ 0 };
    if (laszip_get_point_count(laszip_writer, &p_count))
    {
        fprintf(stderr, "DLL ERROR: getting point count\n");
        return;
    }

    fprintf(stderr, "successfully written %lld points\n", p_count);

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

inline void points_to_vector(
    const std::vector<Point3Di> points,
    std::vector<Eigen::Affine3d>& trajectory,
    double threshold_output_filter,
    std::vector<int>* index_poses,
    std::vector<Eigen::Vector3d>& pointcloud,
    std::vector<unsigned short>& intensity,
    std::vector<double>& timestamps,
    bool use_first_pose,
    bool save_index_pose)
{
    Eigen::Affine3d m_pose = trajectory[0].inverse();
    for (const auto& org_p : points)
    {
        Point3Di p = org_p;
        if (p.point.norm() > threshold_output_filter)
        {
            if (use_first_pose)
            {
                p.point = m_pose * (trajectory[org_p.index_pose] * org_p.point);
            }
            else
            {
                p.point = trajectory[org_p.index_pose] * org_p.point;
            }
            pointcloud.push_back(p.point);
            intensity.push_back(p.intensity);
            timestamps.push_back(p.timestamp);
            if (save_index_pose)
            {
                if (index_poses)
                {
                    index_poses->push_back(org_p.index_pose);
                }
            }
        }
    }
}

inline void save_all_to_las(const Session& session, std::string output_las_name, bool as_local, bool skip_ts_0)
{
    Eigen::Affine3d first_pose = Eigen::Affine3d::Identity();
    bool found_first_pose = false;

    const auto& offset = session.point_clouds_container.offset;
    LazWriter writer;
    if (!writer.open(output_las_name, offset.x(), offset.y(), offset.z()))
    {
        std::cout << "problem with saving file: " << output_las_name << std::endl;
        return;
    }

    for (const auto& p : session.point_clouds_container.point_clouds)
    {
        if (p.visible)
        {
            if (!found_first_pose)
            {
                found_first_pose = true;
                first_pose = p.m_pose;
            }

            for (size_t i = 0; i < p.points_local.size(); ++i)
            {
                if (skip_ts_0)
                {
                    if (i >= p.timestamps.size() || p.timestamps[i] == 0.0)
                        continue;
                }

                Eigen::Vector3d vp = p.m_pose * p.points_local[i];
                if (as_local)
                    vp = first_pose * vp;

                unsigned short inten = (i < p.intensities.size()) ? p.intensities[i] : 0;
                double ts = (i < p.timestamps.size()) ? p.timestamps[i] : 0.0;

                if (!writer.writePoint(vp, inten, ts, offset.x(), offset.y(), offset.z()))
                    return;
            }
        }
    }

    writer.close();
}
