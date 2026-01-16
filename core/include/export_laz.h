#pragma once

#include <filesystem>
#include <iostream>
#include <session.h>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <laszip/laszip_api.h>

namespace fs = std::filesystem;

inline bool exportLaz(
    const std::string& filename,
    const std::vector<Eigen::Vector3d>& pointcloud,
    const std::vector<unsigned short>& intensity,
    const std::vector<double>& timestamps,
    double offset_x = 0.0,
    double offset_y = 0.0,
    double offset_alt = 0.0)
{
    double min_ts = std::numeric_limits<double>::max();
    double max_ts = std::numeric_limits<double>::lowest();
    int number_of_points_with_timestamp_eq_0 = 0;

    constexpr float scale = 0.0001f; // one tenth of milimeter
    // find max
    Eigen::Vector3d _max(
        std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());
    Eigen::Vector3d _min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());

    for (auto& p : pointcloud)
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

    laszip_header* header;

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

    fprintf(stderr, "writing %scompressed file '%s'\n", (compress ? "" : "un"), filename.c_str());

    // get a pointer to the point of the writer that we will populate and write

    laszip_point* point;
    if (laszip_get_point_pointer(laszip_writer, &point))
    {
        fprintf(stderr, "DLL ERROR: getting point pointer from laszip writer\n");
        return false;
    }

    laszip_I64 p_count = 0;
    laszip_F64 coordinates[3];

    for (int i = 0; i < pointcloud.size(); i++)
    {
        const auto& p = pointcloud[i];
        point->intensity = intensity[i];
        point->gps_time = timestamps[i] * 1e9;

        if (point->gps_time < min_ts)
        {
            min_ts = point->gps_time;
        }

        if (point->gps_time > max_ts)
        {
            max_ts = point->gps_time;
        }

        if (point->gps_time == 0.0)
        {
            number_of_points_with_timestamp_eq_0++;
        }
        // std::setprecision(20);
        // std::cout << "point->gps_time " << point->gps_time << " " << timestamps[i] << " " << timestamps[i] * 1e9 << std::endl;

        p_count++;
        coordinates[0] = p.x() + offset_x;
        coordinates[1] = p.y() + offset_y;
        coordinates[2] = p.z() + offset_alt;
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

    std::cout << "min_ts " << min_ts << std::endl;
    std::cout << "max_ts " << max_ts << std::endl;
    std::cout << "number_of_points_with_timestamp_eq_0: " << number_of_points_with_timestamp_eq_0 << std::endl;

    return true;
}

inline void adjustHeader(laszip_header* header, const Eigen::Affine3d& m_pose, const Eigen::Vector3d& offset_in)
{
    Eigen::Vector3d max{ header->max_x, header->max_y, header->max_z };
    Eigen::Vector3d min{ header->min_x, header->min_y, header->min_z };

    // max -= offset_in;
    // min -= offset_in;

    Eigen::Vector3d adj_max = m_pose * max + offset_in;
    Eigen::Vector3d adj_min = m_pose * min + offset_in;

    header->max_x = adj_max.x();
    header->max_y = adj_max.y();
    header->max_z = adj_max.z();

    header->min_x = adj_min.x();
    header->min_y = adj_min.y();
    header->min_z = adj_min.z();

    header->x_offset = offset_in.x();
    header->y_offset = offset_in.y();
    header->z_offset = offset_in.z();
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

    adjustHeader(header, m_pose, offset);

    std::cout << "header min after:" << std::endl;
    std::cout << header->min_x << " " << header->min_y << " " << header->min_z << std::endl;

    std::cout << "header max after:" << std::endl;
    std::cout << header->max_x << " " << header->max_y << " " << header->max_z << std::endl;

    std::cout << "header after:" << std::endl;
    std::cout << header->x_offset << " " << header->y_offset << " " << header->z_offset << std::endl;

    std::cout << "m_pose: " << std::endl << m_pose.matrix() << std::endl;

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
    std::vector<Eigen::Vector3d> pointcloud;
    std::vector<unsigned short> intensity;
    std::vector<double> timestamps;

    Eigen::Affine3d first_pose = Eigen::Affine3d::Identity();
    bool found_first_pose = false;

    for (const auto& p : session.point_clouds_container.point_clouds)
    {
        if (p.visible)
        {
            if (!found_first_pose)
            {
                found_first_pose = true;
                first_pose = p.m_pose; //.inverse();  // valid
            }

            for (size_t i = 0; i < p.points_local.size(); ++i)
            {
                const auto& pp = p.points_local[i];
                Eigen::Vector3d vp = p.m_pose * pp;

                if (skip_ts_0)
                {
                    if (i < p.timestamps.size())
                    {
                        if (p.timestamps[i] != 0.0)
                        {
                            pointcloud.push_back(vp);

                            if (i < p.intensities.size())
                            {
                                intensity.push_back(p.intensities[i]);
                            }
                            else
                            {
                                intensity.push_back(0);
                            }
                            if (i < p.timestamps.size())
                            {
                                timestamps.push_back(p.timestamps[i]);
                            }
                        }
                    }
                }
                else
                {
                    pointcloud.push_back(vp);

                    if (i < p.intensities.size())
                    {
                        intensity.push_back(p.intensities[i]);
                    }
                    else
                    {
                        intensity.push_back(0);
                    }

                    if (i < p.timestamps.size())
                    {
                        timestamps.push_back(p.timestamps[i]);
                    }
                    else
                    {
                        timestamps.push_back(0.0); // ToDo this is caused by BUG in data
                    }
                }
            }
        }
    }

    if (as_local)
    {
        for (auto& pt : pointcloud)
        {
            pt = first_pose * pt;
        }
    }

    if (!exportLaz(
            output_las_name,
            pointcloud,
            intensity,
            timestamps,
            session.point_clouds_container.offset.x(),
            session.point_clouds_container.offset.y(),
            session.point_clouds_container.offset.z()))
    {
        std::cout << "problem with saving file: " << output_las_name << std::endl;
    }
}
