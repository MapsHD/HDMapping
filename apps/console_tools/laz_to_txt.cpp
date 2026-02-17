#include <laszip/laszip_api.h>

#include <cassert>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <vector>

struct Point
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float intensity = 0.0f;
    double timestamp = 0.0;
};

static_assert(sizeof(Point) == 24, "Invalid Point struct size!");

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

    std::vector<Point> points(point_count);

    laszip_I64 point_read_count = 0;
    while (point_read_count < point_count)
    {
        if (laszip_read_point(header))
        {
            return false;
        }

        points[point_read_count].x = header->x_offset + header->x_scale_factor * static_cast<double>(point->X);
        points[point_read_count].y = header->y_offset + header->y_scale_factor * static_cast<double>(point->Y);
        points[point_read_count].z = header->z_offset + header->z_scale_factor * static_cast<double>(point->Z);
        points[point_read_count].intensity = static_cast<double>(point->intensity);
        points[point_read_count].timestamp = static_cast<double>(point->gps_time);

        point_read_count++;
    }

    if (std::ofstream to_file = std::ofstream(to, std::ios::out))
    {
        for (const auto& point : points)
        {
            to_file << point.x << ' ' << point.y << ' ' << point.z << ' ' << point.intensity << ' ' << point.timestamp << '\n';
        }

        to_file.write((const char*)points.data(), points.size() * sizeof(Point));
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
    const char* expected_ply_extension = ".txt";

    if (argc != expected_argc)
    {
        std::fprintf(stderr, "Invalid argument count. Got %d expected %d.\n", argc, expected_argc);
        std::fprintf(stderr, "Usage : %s </PATH/FROM/POINT/CLOUD.laz> </PATH/TO/POINT/CLOUD.txt>\n", argv[0]);

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