#include <Eigen/Eigen>
#include <fstream>
#include <iostream>

#include <Core/export_laz.h>
#include <Core/point_clouds.h>
#include <Core/structures.h>
#include <Core/transformations.h>

#include <laszip/laszip_api.h>

#include <spdlog/spdlog.h>

bool load_pc(PointCloud& pc, const std::string& input_file_name)
{
    laszip_POINTER laszip_reader;
    if (laszip_create(&laszip_reader))
    {
        spdlog::error(":DLL ERROR: creating laszip reader");
        return false;
    }

    laszip_BOOL is_compressed = 0;
    if (laszip_open_reader(laszip_reader, input_file_name.c_str(), &is_compressed))
    {
        spdlog::error(":DLL ERROR: opening laszip reader for '{}'", input_file_name);
        return false;
    }
    laszip_header* header;

    if (laszip_get_header_pointer(laszip_reader, &header))
    {
        spdlog::error(":DLL ERROR: getting header pointer from laszip reader");
        return false;
    }

    laszip_point* point;
    if (laszip_get_point_pointer(laszip_reader, &point))
    {
        spdlog::error(":DLL ERROR: getting point pointer from laszip reader");
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

    laszip_I64 npoints = (header->number_of_point_records ? header->number_of_point_records : header->extended_number_of_point_records);

    std::cout << (is_compressed ? "" : "un") << "compressed file '" << (std::filesystem::path(input_file_name).filename().string())
              << "' contains " << npoints << " points" << std::endl;

    laszip_I64 p_count = 0;

    while (p_count < npoints)
    {
        if (laszip_read_point(laszip_reader))
        {
            spdlog::error("DLL ERROR: reading point {}", p_count);
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

        Eigen::Vector3d color(
            static_cast<float>(point->rgb[0]) / 256.0,
            static_cast<float>(point->rgb[1]) / 256.0,
            static_cast<float>(point->rgb[2]) / 256.0);

        pc.colors.push_back(color);

        p_count++;
    }

    laszip_close_reader(laszip_reader);

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