#include <laszip/laszip_api.h>

#include <cassert>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

bool check_path_ext(const char* path, const char* ext)
{
    return std::filesystem::path(path).extension() == ext;
}

struct PCDField
{
    std::string name;
    std::string type; // F, U, I, etc.
    int size = 0;
    int offset = 0;
};

struct PCDHeader
{
    int width = 0;
    int height = 0;
    int points = 0;
    bool is_binary = false;
    std::vector<PCDField> fields;
    int point_record_length = 0;
};

bool read_pcd_header(std::ifstream& file, PCDHeader& header, size_t& data_offset)
{
    std::string line;
    int current_offset = 0;

    while (std::getline(file, line))
    {
        if (line.find("WIDTH") == 0)
        {
            std::istringstream iss(line.substr(5));
            iss >> header.width;
        }
        else if (line.find("HEIGHT") == 0)
        {
            std::istringstream iss(line.substr(6));
            iss >> header.height;
        }
        else if (line.find("POINTS") == 0)
        {
            std::istringstream iss(line.substr(6));
            iss >> header.points;
        }
        else if (line.find("FIELDS") == 0)
        {
            std::istringstream iss(line.substr(6));
            std::string field_name;
            while (iss >> field_name)
            {
                PCDField field;
                field.name = field_name;
                header.fields.push_back(field);
            }
        }
        else if (line.find("SIZE") == 0)
        {
            std::istringstream iss(line.substr(4));
            for (auto& field : header.fields)
            {
                iss >> field.size;
                field.offset = current_offset;
                current_offset += field.size;
            }
            header.point_record_length = current_offset;
        }
        else if (line.find("TYPE") == 0)
        {
            std::istringstream iss(line.substr(4));
            for (auto& field : header.fields)
            {
                iss >> field.type;
            }
        }
        else if (line.find("DATA") == 0)
        {
            header.is_binary = line.find("binary") != std::string::npos;
            data_offset = file.tellg();
            break;
        }
    }
    return header.points > 0 && header.point_record_length > 0;
}

bool convert_and_save(const char* from, const char* to)
{
    std::ifstream pcd_file(from, std::ios::binary);
    if (!pcd_file.is_open())
    {
        std::fprintf(stderr, "Failed to open input file %s\n", from);
        return false;
    }

    PCDHeader pcd_header;
    size_t data_offset = 0;
    if (!read_pcd_header(pcd_file, pcd_header, data_offset))
    {
        std::fprintf(stderr, "Failed to read PCD header from file %s\n", from);
        return false;
    }

    // Find field offsets
    int x_offset = -1, y_offset = -1, z_offset = -1, intensity_offset = -1;

    for (const auto& field : pcd_header.fields)
    {
        if (field.name == "x")
            x_offset = field.offset;
        else if (field.name == "y")
            y_offset = field.offset;
        else if (field.name == "z")
            z_offset = field.offset;
        else if (field.name == "intensity")
            intensity_offset = field.offset;
    }

    if (x_offset < 0 || y_offset < 0 || z_offset < 0)
    {
        std::fprintf(stderr, "PCD file must contain x, y, z fields\n");
        return false;
    }

    std::fprintf(stderr, "PCD Header: %d points, point record length: %d bytes\n", pcd_header.points, pcd_header.point_record_length);
    std::fprintf(
        stderr, "Fields: x_offset=%d, y_offset=%d, z_offset=%d, intensity_offset=%d\n", x_offset, y_offset, z_offset, intensity_offset);

    // Read all point data
    std::vector<uint8_t> pointsBinary(pcd_header.points * pcd_header.point_record_length);
    pcd_file.read((char*)pointsBinary.data(), pointsBinary.size());
    if (pcd_file.gcount() != static_cast<std::streamsize>(pointsBinary.size()))
    {
        std::fprintf(stderr, "Failed to read point data from file %s\n", from);
        std::fprintf(stderr, "Expected %zu bytes, got %zd bytes\n", pointsBinary.size(), pcd_file.gcount());
        return false;
    }

    // Create LAZ writer
    laszip_POINTER laszip_writer = nullptr;
    if (laszip_create(&laszip_writer))
    {
        std::fprintf(stderr, "Failed to create LAZ writer\n");
        return false;
    }

    // Setup header for LAZ output
    laszip_header laszip_hdr;
    memset(&laszip_hdr, 0, sizeof(laszip_header));

    laszip_hdr.version_major = 1;
    laszip_hdr.version_minor = 0;
    laszip_hdr.header_size = 227;
    laszip_hdr.offset_to_point_data = 227;

    laszip_hdr.number_of_point_records = pcd_header.points;
    laszip_hdr.number_of_points_by_return[0] = pcd_header.points;

    laszip_hdr.x_scale_factor = 0.001;
    laszip_hdr.y_scale_factor = 0.001;
    laszip_hdr.z_scale_factor = 0.001;

    // Set reasonable offsets based on data
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double min_z = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    double max_z = std::numeric_limits<double>::lowest();

    // Calculate bounds
    for (int i = 0; i < pcd_header.points; i++)
    {
        uint8_t* pointDataPtr = pointsBinary.data() + i * pcd_header.point_record_length;

        float x = *reinterpret_cast<float*>(pointDataPtr + x_offset);
        float y = *reinterpret_cast<float*>(pointDataPtr + y_offset);
        float z = *reinterpret_cast<float*>(pointDataPtr + z_offset);

        min_x = std::min(min_x, static_cast<double>(x));
        min_y = std::min(min_y, static_cast<double>(y));
        min_z = std::min(min_z, static_cast<double>(z));
        max_x = std::max(max_x, static_cast<double>(x));
        max_y = std::max(max_y, static_cast<double>(y));
        max_z = std::max(max_z, static_cast<double>(z));
    }

    laszip_hdr.x_offset = min_x;
    laszip_hdr.y_offset = min_y;
    laszip_hdr.z_offset = min_z;

    if (laszip_set_header(laszip_writer, &laszip_hdr))
    {
        laszip_destroy(laszip_writer);
        std::fprintf(stderr, "Failed to set LAZ header\n");
        return false;
    }

    if (laszip_open_writer(laszip_writer, to, 1))
    {
        laszip_destroy(laszip_writer);
        std::fprintf(stderr, "Failed to open LAZ writer for file %s\n", to);
        return false;
    }

    // Get point pointer
    laszip_point* laszip_pt = nullptr;
    if (laszip_get_point_pointer(laszip_writer, &laszip_pt))
    {
        laszip_close_writer(laszip_writer);
        laszip_destroy(laszip_writer);
        std::fprintf(stderr, "Failed to get LAZ point pointer\n");
        return false;
    }

    // Write points
    for (int i = 0; i < pcd_header.points; i++)
    {
        uint8_t* pointDataPtr = pointsBinary.data() + i * pcd_header.point_record_length;

        float x = *reinterpret_cast<float*>(pointDataPtr + x_offset);
        float y = *reinterpret_cast<float*>(pointDataPtr + y_offset);
        float z = *reinterpret_cast<float*>(pointDataPtr + z_offset);

        uint16_t intensity = 0;
        if (intensity_offset >= 0)
        {
            // Read intensity if available
            for (const auto& field : pcd_header.fields)
            {
                if (field.name == "intensity")
                {
                    if (field.type == "U" && field.size == 2)
                    {
                        intensity = *reinterpret_cast<uint16_t*>(pointDataPtr + intensity_offset);
                    }
                    else if (field.type == "U" && field.size == 1)
                    {
                        intensity = static_cast<uint16_t>(*reinterpret_cast<uint8_t*>(pointDataPtr + intensity_offset)) * 256;
                    }
                    else if (field.type == "F" && field.size == 4)
                    {
                        float int_val = *reinterpret_cast<float*>(pointDataPtr + intensity_offset);
                        intensity = static_cast<uint16_t>(std::clamp(int_val * 65535.0f, 0.0f, 65535.0f));
                    }
                    break;
                }
            }
        }

        laszip_pt->X = static_cast<int32_t>((x - laszip_hdr.x_offset) / laszip_hdr.x_scale_factor);
        laszip_pt->Y = static_cast<int32_t>((y - laszip_hdr.y_offset) / laszip_hdr.y_scale_factor);
        laszip_pt->Z = static_cast<int32_t>((z - laszip_hdr.z_offset) / laszip_hdr.z_scale_factor);
        laszip_pt->intensity = intensity;

        if (laszip_write_point(laszip_writer))
        {
            laszip_close_writer(laszip_writer);
            laszip_destroy(laszip_writer);
            std::fprintf(stderr, "Failed to write point %d to LAZ file %s\n", i, to);
            return false;
        }
    }

    if (laszip_close_writer(laszip_writer))
    {
        laszip_destroy(laszip_writer);
        std::fprintf(stderr, "Failed to close LAZ writer for file %s\n", to);
        return false;
    }

    laszip_destroy(laszip_writer);
    std::fprintf(stderr, "Successfully converted %d points\n", pcd_header.points);
    return true;
}

int main(const int argc, const char** argv)
{
    const int expected_argc = 3;

    const char* expected_pcd_extension = ".pcd";
    const char* expected_laz_extension = ".laz";

    if (argc != expected_argc)
    {
        std::fprintf(stderr, "Invalid argument count. Got %d expected %d.\n", argc, expected_argc);
        std::fprintf(stderr, "Usage : %s </PATH/FROM/POINT/CLOUD.pcd> </PATH/TO/POINT/CLOUD.laz>\n", argv[0]);

        return EXIT_FAILURE;
    }

    const char* from = argv[1];
    const char* to = argv[2];

    if (!check_path_ext(from, expected_pcd_extension))
    {
        std::fprintf(stderr, "Invalid extension for input file %s - expected %s\n", from, expected_pcd_extension);

        return EXIT_FAILURE;
    }

    if (!check_path_ext(to, expected_laz_extension))
    {
        std::fprintf(stderr, "Invalid extension for output file %s - expected %s\n", to, expected_laz_extension);

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
