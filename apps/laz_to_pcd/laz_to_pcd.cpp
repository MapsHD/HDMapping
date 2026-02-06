#include <laszip/laszip_api.h>

#include <cassert>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <span>
#include <string>
#include <vector>
namespace fs = std::filesystem;


bool check_path_ext(const char* path, const char* ext)
{
    return std::filesystem::path(path).extension() == ext;
}

bool convert_and_save(const char* from, const char* to)
{
    laszip_POINTER laszip_reader = nullptr;
    if (laszip_create(&laszip_reader))
    {
        return false;
    }

    laszip_BOOL is_compressed = 0;
    if (laszip_open_reader(laszip_reader, from, &is_compressed))
    {
        return false;
    }

    laszip_header* header = nullptr;
    if (laszip_get_header_pointer(laszip_reader, &header))
    {
        return false;
    }

    laszip_point* point = nullptr;
    if (laszip_get_point_pointer(header, &point))
    {
        return false;
    }

    laszip_I64 point_count = header->number_of_point_records ? header->number_of_point_records : header->extended_number_of_point_records;


    const size_t PointSizeBytes = sizeof(float) * 4 + sizeof(double); // x,y,z,intensity,timestamp

    std::vector<uint8_t> pointsBinary(point_count * PointSizeBytes);

    laszip_I64 point_read_count = 0;



    while (point_read_count < point_count)
    {
        if (laszip_read_point(header))
        {
            return false;
        }
        uint8_t* pointDataPtr = pointsBinary.data() + point_read_count * PointSizeBytes;
        uint8_t* pointerTimstamp = pointDataPtr + sizeof(float)*4;
        uint8_t* pointDataEndPtr = pointDataPtr + PointSizeBytes;

        std::span<float> xyzi(reinterpret_cast<float*>(pointDataPtr), 3);
        xyzi[0] = static_cast<float>(header->x_offset + header->x_scale_factor * static_cast<double>(point->X));
        xyzi[1] = static_cast<float>(header->y_offset + header->x_scale_factor * static_cast<double>(point->Y));
        xyzi[2] = static_cast<float>(header->z_offset + header->x_scale_factor * static_cast<double>(point->Z));
        xyzi[3] = static_cast<float>(point->intensity);
        const double ts = point->gps_time;
        memcpy(pointerTimstamp, &ts , sizeof(double));
        point_read_count++;
    }



    if (std::ofstream to_file = std::ofstream(to, std::ios::out | std::ios::binary))
    {
        to_file << "# .PCD v0.7\n";
        to_file << "VERSION 0.7\n";
        to_file << "FIELDS x y z intensity timestamp\n";
        to_file << "SIZE 4 4 4 4 8\n";
        to_file << "TYPE F F F F F\n";
        to_file << "COUNT 1 1 1 1 1\n";
        to_file << "WIDTH " << point_count << "\n";
        to_file << "HEIGHT 1\n";
        to_file << "VIEWPOINT 0 0 0 1 0 0 0\n";
        to_file << "POINTS " << point_count << "\n";
        to_file << "DATA binary\n";

        to_file.write((const char*)pointsBinary.data(), pointsBinary.size() );
    }
    else
    {
        return false;
    }

    return true;
}

int main(const int argc, const char** argv)
{
    const int expected_argc = 3;

    const char* expected_laz_extension = ".laz";
    const char* expected_ply_extension = ".pcd";

    if (argc != expected_argc)
    {
        std::fprintf(stderr, "Invalid argument count. Got %d expected %d.\n", argc, expected_argc);
        std::fprintf(stderr, "Usage : %s </PATH/FROM/POINT/CLOUD.laz> </PATH/TO/POINT/CLOUD.pcd>\n", argv[0]);

        return EXIT_FAILURE;
    }

    const char* from = argv[1];
    const char* to = argv[2];

    if (!check_path_ext(from, expected_laz_extension))
    {
        std::fprintf(stderr, "Invalid extension for input file %s - expected %s\n", from, expected_laz_extension);

        return EXIT_FAILURE;
    }

    if (!check_path_ext(to, expected_ply_extension))
    {
        std::fprintf(stderr, "Invalid extension for output file %s - expected %s\n", from, expected_ply_extension);

        return EXIT_FAILURE;
    }

    if (!std::filesystem::exists(from))
    {
        std::fprintf(stderr, "Input file %s - does not exist\n", from);

        return EXIT_FAILURE;
    }

    if (!convert_and_save(from, to))
    {
        std::fprintf(stderr, "Conversion from %s to  %s failed\n", from, to);

        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}