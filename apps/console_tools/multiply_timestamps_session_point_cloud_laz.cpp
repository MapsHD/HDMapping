#include <Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <structures.h>
#include <transformations.h>

#include <point_clouds.h>

#include <laszip/laszip_api.h>

#include <export_laz.h>

bool load_pc(PointCloud& pc, std::string input_file_name)
{
    laszip_POINTER laszip_reader;
    if (laszip_create(&laszip_reader))
    {
        fprintf(stderr, ":DLL ERROR: creating laszip reader\n");
        /*PointCloud pc;
        pc.m_pose = Eigen::Affine3d::Identity();
        pc.m_initial_pose = pc.m_pose;
        pc.pose = pose_tait_bryan_from_affine_matrix(pc.m_pose);
        pc.gui_translation[0] = pc.pose.px;
        pc.gui_translation[1] = pc.pose.py;
        pc.gui_translation[2] = pc.pose.pz;
        pc.gui_rotation[0] = rad2deg(pc.pose.om);
        pc.gui_rotation[1] = rad2deg(pc.pose.fi);
        pc.gui_rotation[2] = rad2deg(pc.pose.ka);
        pc.file_name = input_file_names[i];
        point_clouds.push_back(pc);*/
        return false;
    }

    laszip_BOOL is_compressed = 0;
    if (laszip_open_reader(laszip_reader, input_file_name.c_str(), &is_compressed))
    {
        fprintf(stderr, ":DLL ERROR: opening laszip reader for '%s'\n", input_file_name.c_str());
        /*PointCloud pc;
        pc.m_pose = Eigen::Affine3d::Identity();
        pc.m_initial_pose = pc.m_pose;
        pc.pose = pose_tait_bryan_from_affine_matrix(pc.m_pose);
        pc.gui_translation[0] = pc.pose.px;
        pc.gui_translation[1] = pc.pose.py;
        pc.gui_translation[2] = pc.pose.pz;
        pc.gui_rotation[0] = rad2deg(pc.pose.om);
        pc.gui_rotation[1] = rad2deg(pc.pose.fi);
        pc.gui_rotation[2] = rad2deg(pc.pose.ka);
        pc.file_name = input_file_names[i];
        point_clouds.push_back(pc);*/
        return false;
    }
    laszip_header* header;

    if (laszip_get_header_pointer(laszip_reader, &header))
    {
        fprintf(stderr, ":DLL ERROR: getting header pointer from laszip reader\n");
        /*PointCloud pc;
        pc.m_pose = Eigen::Affine3d::Identity();
        pc.m_initial_pose = pc.m_pose;
        pc.pose = pose_tait_bryan_from_affine_matrix(pc.m_pose);
        pc.gui_translation[0] = pc.pose.px;
        pc.gui_translation[1] = pc.pose.py;
        pc.gui_translation[2] = pc.pose.pz;
        pc.gui_rotation[0] = rad2deg(pc.pose.om);
        pc.gui_rotation[1] = rad2deg(pc.pose.fi);
        pc.gui_rotation[2] = rad2deg(pc.pose.ka);
        pc.file_name = input_file_names[i];
        point_clouds.push_back(pc);*/
        return false;
    }

    // fprintf(stderr, "file '%s' contains %u points\n", input_file_name.c_str(), header->number_of_point_records);

    laszip_point* point;
    if (laszip_get_point_pointer(laszip_reader, &point))
    {
        fprintf(stderr, ":DLL ERROR: getting point pointer from laszip reader\n");
        return false;
    }

    pc.m_pose = Eigen::Affine3d::Identity();
    pc.m_initial_pose = pc.m_pose;
    pc.pose = pose_tait_bryan_from_affine_matrix(pc.m_pose);
    pc.gui_translation[0] = static_cast<float>(pc.pose.px);
    pc.gui_translation[1] = static_cast<float>(pc.pose.py);
    pc.gui_translation[2] = static_cast<float>(pc.pose.pz);
    pc.gui_rotation[0] = rad2deg(pc.pose.om);
    pc.gui_rotation[1] = rad2deg(pc.pose.fi);
    pc.gui_rotation[2] = rad2deg(pc.pose.ka);

    /*for (int j = 0; j < header->number_of_point_records; j++)
    {
        if (laszip_read_point(laszip_reader))
        {
            fprintf(stderr, ":DLL ERROR: reading point %u\n", j);
            laszip_close_reader(laszip_reader);
            return true;
            // continue;
        }

        LAZPoint p;
        p.x = header->x_offset + header->x_scale_factor * static_cast<double>(point->X);
        p.y = header->y_offset + header->y_scale_factor * static_cast<double>(point->Y);
        p.z = header->z_offset + header->z_scale_factor * static_cast<double>(point->Z);
        p.timestamp = point->gps_time;

        Eigen::Vector3d pp(p.x, p.y, p.z);
        pc.points_local.push_back(pp);
        pc.intensities.push_back(point->intensity);
        pc.timestamps.push_back(p.timestamp);
    }*/

    laszip_I64 npoints = (header->number_of_point_records ? header->number_of_point_records : header->extended_number_of_point_records);

    std::cout << (is_compressed ? "" : "un") << "compressed file '" << (std::filesystem::path(input_file_name).filename().string())
              << "' contains " << npoints << " points" << std::endl;

    laszip_I64 p_count = 0;

    while (p_count < npoints)
    {
        if (laszip_read_point(laszip_reader))
        {
            fprintf(stderr, "DLL ERROR: reading point %I64d\n", p_count);
            laszip_close_reader(laszip_reader);
            return false;
        }

        LAZPoint p;
        p.x = header->x_offset + header->x_scale_factor * static_cast<double>(point->X);
        p.y = header->y_offset + header->y_scale_factor * static_cast<double>(point->Y);
        p.z = header->z_offset + header->z_scale_factor * static_cast<double>(point->Z);
        p.timestamp = point->gps_time;

        Eigen::Vector3d pp(p.x, p.y, p.z);
        pc.points_local.push_back(pp);
        pc.intensities.push_back(point->intensity);
        pc.timestamps.push_back(p.timestamp);

        // Eigen::Vector3d color(
        //	static_cast<uint8_t>(0xFFU * ((point->rgb[0] > 0) ? static_cast<float>(point->rgb[0]) / static_cast<float>(0xFFFFU) : 1.0f))
        /// 256.0, 	static_cast<uint8_t>(0xFFU * ((point->rgb[1] > 0) ? static_cast<float>(point->rgb[1]) / static_cast<float>(0xFFFFU)
        //: 1.0f)) / 256.0, 	static_cast<uint8_t>(0xFFU * ((point->rgb[2] > 0) ? static_cast<float>(point->rgb[2]) /
        // static_cast<float>(0xFFFFU) : 1.0f)) / 256.0);

        Eigen::Vector3d color(
            static_cast<float>(point->rgb[0]) / 256.0,
            static_cast<float>(point->rgb[1]) / 256.0,
            static_cast<float>(point->rgb[2]) / 256.0);

        // std::cout << point->rgb[0] << " " << point->rgb[1] << " " << point->rgb[2] << std::endl;

        pc.colors.push_back(color);

        p_count++;
    }

    laszip_close_reader(laszip_reader);
    // laszip_clean(laszip_reader);
    // laszip_destroy(laszip_reader);

    return true;
}

int main(int argc, char* argv[])
{
    if (argc != 4)
    {
        std::cout << "USAGE: " << argv[0] << " input_point_cloud output_point_cloud multiplier(e.g. 1000)" << std::endl;
        return 1;
    }

    PointCloud pc;

    if (!load_pc(pc, argv[1]))
    {
        std::cout << "Problem with loading '" << argv[1] << "'" << std::endl;
        return 2;
    }

    for (auto& t : pc.timestamps)
    {
        t *= atof(argv[2]);
    }

    if (!exportLaz(argv[2], pc.points_local, pc.intensities, pc.timestamps))
    {
        std::cout << "Problem with saving: '" << argv[3] << std::endl;
    }

    return 0;
}