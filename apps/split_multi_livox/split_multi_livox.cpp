#include <Eigen/Dense>
#include <HDMapping/Version.hpp>
#include <export_laz.h>
#include <filesystem>
#include <iostream>
#include <laszip/laszip_api.h>
#include <string>
#include <vector>

struct Point3Dis
{
    Eigen::Vector3d point;
    double timestamp;
    float intensity;
};

struct Point3Dil
{
    Eigen::Vector3d point;
    double timestamp;
    float intensity;
    int livoxId;
};

std::vector<Point3Dil> load_point_cloud(const std::string& lazFile, bool ommit_points_with_timestamp_equals_zero)
{
    std::vector<Point3Dil> points;
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
            // std::abort();
        }
        Point3Dil p;

        Eigen::Vector3d pf(
            header->x_offset + header->x_scale_factor * static_cast<double>(point->X),
            header->y_offset + header->y_scale_factor * static_cast<double>(point->Y),
            header->z_offset + header->z_scale_factor * static_cast<double>(point->Z));
        p.point.x() = pf.x();
        p.point.y() = pf.y();
        p.point.z() = pf.z();
        p.timestamp = point->gps_time;
        p.intensity = point->intensity;
        p.livoxId = point->user_data;

        if (p.timestamp == 0 && ommit_points_with_timestamp_equals_zero)
        {
            counter_ts0++;
        }
        else
        {
            points.emplace_back(p);
        }
    }

    std::cout << "number points with ts == 0: " << counter_ts0 << std::endl;
    std::cout << "counter_filtered_points: " << counter_filtered_points << std::endl;
    std::cout << "total number points: " << points.size() << std::endl;
    laszip_close_reader(laszip_reader);
    return points;
}

std::unordered_map<int, std::string> GetIdToStringMapping(const std::string& filename)
{
    std::unordered_map<int, std::string> dataMap;
    std::ifstream fst(filename);
    std::string line;
    while (std::getline(fst, line))
    {
        std::istringstream iss(line);
        int key;
        std::string value;

        if (iss >> key >> value)
        {
            dataMap[key] = value;
        }
        else
        {
            std::cerr << "Failed to parse line: " << line << std::endl;
        }
    }
    return dataMap;
}

int main(int argc, char* argv[])
{
    std::cout << "Version " HDMAPPING_VERSION_STRING << std::endl;
    std::vector<std::string> arguments;
    for (int i = 1; i < argc; i++)
    {
        arguments.push_back(argv[i]);
    }
    if (arguments.size() != 2)
    {
        std::cout << "Usage \n";
        std::cout << " " << argv[0] << " <input_file_from_mandeye> <output>\n";
        return 0;
    }
    const std::filesystem::path inputLazFile{ arguments[0] };
    const std::filesystem::path inputSnFile = inputLazFile.parent_path() / inputLazFile.stem().concat(".sn");
    const std::filesystem::path outputFile{ arguments[1] };
    std::cout << "Input Laz file " << inputLazFile << std::endl;
    std::cout << "Input Sn file " << inputSnFile << std::endl;

    if (!std::filesystem::exists(inputLazFile))
    {
        std::cout << "No input file " << inputLazFile << std::endl;
    }
    if (!std::filesystem::exists(inputSnFile))
    {
        std::cout << "No input file " << inputSnFile << std::endl;
    }

    const auto mapping = GetIdToStringMapping(inputSnFile.string());

    const auto data = load_point_cloud(inputLazFile.string(), true);

    std::unordered_map<int, std::vector<Point3Dis>> data_separated;

    for (const auto& pInput : data)
    {
        const int id = pInput.livoxId;

        Point3Dis pOutput;
        pOutput.point = pInput.point;
        pOutput.intensity = pInput.intensity;
        pOutput.timestamp = pInput.timestamp;
        data_separated[id].push_back(pOutput);
    }

    for (const auto& [id, sn] : mapping)
    {
        const std::filesystem::path outputFileSn = outputFile.parent_path() / outputFile.stem().concat("_" + sn + ".laz");
        const auto& lidarData = data_separated[id];
        std::vector<Eigen::Vector3d> global_pointcloud;
        std::vector<unsigned short> intensity;
        std::vector<double> timestamps;
        for (const auto& p : lidarData)
        {
            global_pointcloud.push_back(p.point);
            intensity.push_back(p.intensity);
            timestamps.push_back(p.timestamp);
        }
        exportLaz(outputFileSn.string(), global_pointcloud, intensity, timestamps);
    }

    return 0;
}
