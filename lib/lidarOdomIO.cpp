#include "lidarOdomIO.h"
#include "lidarOdometry.h"
#include <structures.h>
#include <ndt.h>
#include <Fusion.h>

namespace mandeye::utilsIO
{

    std::vector<Point3Di> load_point_cloud(const std::string& lazFile, bool ommit_points_with_timestamp_equals_zero)
    {
        double filter_threshold_xy = 0.5;
        std::vector<Point3Di> points;
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
        laszip_header* header;

        if (laszip_get_header_pointer(laszip_reader, &header))
        {
            fprintf(stderr, "DLL ERROR: getting header pointer from laszip reader\n");
            std::abort();
        }
        fprintf(stderr, "file '%s' contains %u points\n", lazFile.c_str(), header->number_of_point_records);
        laszip_point* point;
        if (laszip_get_point_pointer(laszip_reader, &point))
        {
            fprintf(stderr, "DLL ERROR: getting point pointer from laszip reader\n");
            std::abort();
        }

        int counter_ts0 = 0;
        int counter_filtered_points = 0;
        for (laszip_U32 j = 0; j < header->number_of_point_records; j++)
        {
            if (laszip_read_point(laszip_reader))
            {
                fprintf(stderr, "DLL ERROR: reading point %u\n", j);
                laszip_close_reader(laszip_reader);
                return points;
                //std::abort();
            }
            Point3Di p;

            double cal_x = 11.0 / 1000.0; // ToDo change if lidar differen than livox mid 360
            double cal_y = 23.29 / 1000.0;
            double cal_z = -44.12 / 1000.0;

            Eigen::Vector3d pf(header->x_offset + header->x_scale_factor * static_cast<double>(point->X), header->y_offset + header->y_scale_factor * static_cast<double>(point->Y), header->z_offset + header->z_scale_factor * static_cast<double>(point->Z));
            p.point.x() = pf.x() - cal_x;
            p.point.y() = pf.y() - cal_y;
            p.point.z() = pf.z() - cal_z;
            p.timestamp = point->gps_time;
            p.intensity = point->intensity;

            if (p.timestamp == 0 && ommit_points_with_timestamp_equals_zero)
            {
                counter_ts0++;
            }
            else
            {
                if (sqrt(pf.x() * pf.x() + pf.y() * pf.y()) > filter_threshold_xy)
                {
                    points.emplace_back(p);
                }
                else
                {
                    counter_filtered_points++;
                }
            }
        }

        std::cout << "number points with ts == 0: " << counter_ts0 << std::endl;
        std::cout << "counter_filtered_points: " << counter_filtered_points << std::endl;
        std::cout << "total number points: " << points.size() << std::endl;
        laszip_close_reader(laszip_reader);
        return points;
    }

    std::vector<std::tuple<double, FusionVector, FusionVector>> load_imu(const std::string& imu_file)
    {
        std::vector<std::tuple<double, FusionVector, FusionVector>> all_data;
        std::ifstream myfile(imu_file);
        if (myfile.is_open())
        {
            while (myfile)
            {
                double data[7];
                myfile >> data[0] >> data[1] >> data[2] >> data[3] >> data[4] >> data[5] >> data[6];
                // std::cout << data[0] << " " << data[1] << " " << data[2] << " " << data[3] << " " << data[4] << " " << data[5] << " " << data[6] << std::endl;
                if (data[0] > 0)
                {
                    FusionVector gyr;
                    gyr.axis.x = data[1];
                    gyr.axis.y = data[2];
                    gyr.axis.z = data[3];

                    FusionVector acc;
                    acc.axis.x = data[4];
                    acc.axis.y = data[5];
                    acc.axis.z = data[6];

                    all_data.emplace_back(data[0] / 1e9, gyr, acc);
                }
            }
            myfile.close();
        }
        return all_data;
    }

    bool save_poses(const std::string file_name, std::vector<Eigen::Affine3d> m_poses, std::vector<std::string> filenames)
    {
        std::ofstream outfile;
        outfile.open(file_name);
        if (!outfile.good())
        {
            std::cout << "can not save file: " << file_name << std::endl;
            return false;
        }

        outfile << m_poses.size() << std::endl;
        for (size_t i = 0; i < m_poses.size(); i++)
        {
            outfile << filenames[i] << std::endl;
            outfile << m_poses[i](0, 0) << " " << m_poses[i](0, 1) << " " << m_poses[i](0, 2) << " " << m_poses[i](0, 3) << std::endl;
            outfile << m_poses[i](1, 0) << " " << m_poses[i](1, 1) << " " << m_poses[i](1, 2) << " " << m_poses[i](1, 3) << std::endl;
            outfile << m_poses[i](2, 0) << " " << m_poses[i](2, 1) << " " << m_poses[i](2, 2) << " " << m_poses[i](2, 3) << std::endl;
            outfile << "0 0 0 1" << std::endl;
        }
        outfile.close();

        return true;
    }

    bool saveLaz(const std::string& filename, const std::vector<Point3Di>& points_global)
    {

        constexpr float scale = 0.0001f; // one tenth of milimeter
        // find max
        double max_x{ std::numeric_limits<double>::lowest() };
        double max_y{ std::numeric_limits<double>::lowest() };
        double max_z{ std::numeric_limits<double>::lowest() };
        double min_x = 1000000000000.0;
        double min_y = 1000000000000.0;
        double min_z = 1000000000000.0;

        for (auto& p : points_global)
        {
            if (p.point.x() < min_x)
            {
                min_x = p.point.x();
            }
            if (p.point.x() > max_x)
            {
                max_x = p.point.x();
            }

            if (p.point.y() < min_y)
            {
                min_y = p.point.y();
            }
            if (p.point.y() > max_y)
            {
                max_y = p.point.y();
            }

            if (p.point.z() < min_z)
            {
                min_z = p.point.z();
            }
            if (p.point.z() > max_z)
            {
                max_z = p.point.z();
            }
        }

        std::cout << "processing: " << filename << "points " << points_global.size() << std::endl;

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
        header->number_of_point_records = points_global.size();
        header->number_of_points_by_return[0] = points_global.size();
        header->number_of_points_by_return[1] = 0;
        header->point_data_record_length = 28;
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

        laszip_point* point;
        if (laszip_get_point_pointer(laszip_writer, &point))
        {
            fprintf(stderr, "DLL ERROR: getting point pointer from laszip writer\n");
            return false;
        }

        laszip_I64 p_count = 0;
        laszip_F64 coordinates[3];

        for (int i = 0; i < points_global.size(); i++)
        {
            const auto& p = points_global[i];
            point->intensity = p.intensity;
            point->gps_time = p.timestamp * 1e9;
            // std::cout << p.timestamp << std::endl;
            //  point->user_data = 0;//p.line_id;
            //  point->classification = p.point.tag;
            p_count++;
            coordinates[0] = p.point.x();
            coordinates[1] = p.point.y();
            coordinates[2] = p.point.z();
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

}

