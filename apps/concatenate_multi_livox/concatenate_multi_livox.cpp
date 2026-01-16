#include "../lidar_odometry_step_1/lidar_odometry_utils.h"
#include "export_laz.h"
#include <filesystem>
#include <iostream>

int main(int argc, char* argv[])
{
    if (argc < 3)
    {
        std::cout << "Usage: " << argv[0] << " <input_dir> <output_dir>" << std::endl;
        return 1;
    }
    const std::string input_dir = argv[1];
    const std::string output_dir = argv[2];
    if (!std::filesystem::exists(input_dir))
    {
        std::cout << "Input directory does not exist: " << input_dir << std::endl;
        return 1;
    }

    std::vector<std::string> laz_files;
    std::vector<std::string> csv_files;
    std::vector<std::string> sn_files;
    std::vector<std::string> all_file_names;
    for (const auto& entry : std::filesystem::directory_iterator(input_dir))
    {
        if (entry.is_regular_file())
        {
            const std::string filename = entry.path().filename().string();
            if (filename.ends_with(".laz") || filename.ends_with(".las"))
            {
                laz_files.push_back(entry.path().string());
                all_file_names.push_back(entry.path().string());
            }
            else if (filename.ends_with(".csv"))
            {
                csv_files.push_back(entry.path().string());
            }
            else if (filename.ends_with(".sn"))
            {
                sn_files.push_back(entry.path().string());
            }
        }
    }
    // check if number of laz files is equal to number of csv files and sn files
    const auto scan_number = laz_files.size();
    if (scan_number != csv_files.size() || scan_number != sn_files.size())
    {
        std::cerr << "Number of laz files, csv files and sn files is not equal!" << std::endl;
        std::cerr << "laz files: " << laz_files.size() << std::endl;
        std::cerr << "csv files: " << csv_files.size() << std::endl;
        std::cerr << "sn files: " << sn_files.size() << std::endl;
        return 1;
    }
    std::cout << "Found " << scan_number << " scans" << std::endl;
    // load calibration file
    const auto calibrationFile = (std::filesystem::path(input_dir) / "calibration.json").string();
    std::cout << "Loading calibration file: " << calibrationFile << std::endl;
    auto preloadedCalibration = MLvxCalib::GetCalibrationFromFile(calibrationFile);
    if (preloadedCalibration.empty())
    {
        std::cerr << "No calibration data found in file: " << calibrationFile << std::endl;
        std::cerr << "Please check the file format and content." << std::endl;
        return 1;
    }
    std::cout << "Loaded calibration for " << preloadedCalibration.size() << " sensors." << std::endl;
    for (const auto& [sn, _] : preloadedCalibration)
    {
        std::cout << " -> " << sn << std::endl;
    }
    // Get Id of Imu to use
    const std::string imuSnToUse = MLvxCalib::GetImuSnToUse(calibrationFile);
    if (imuSnToUse.empty())
    {
        std::cerr << "No IMU serial number found in calibration file: " << calibrationFile << std::endl;
        std::cerr << "Please check the file format and content.";
        return 1;
    }
    std::cout << "IMU to use: " << imuSnToUse << std::endl;
    // get id to serial number mapping

    if (!std::filesystem::exists(output_dir))
    {
        // create output directory if it does not exist
        std::filesystem::create_directory(output_dir);
    }

    // sort
    std::sort(std::begin(laz_files), std::end(laz_files));
    std::sort(std::begin(csv_files), std::end(csv_files));
    std::sort(std::begin(sn_files), std::end(sn_files));

    // process laz
    for (int i = 0; i < scan_number; i++)
    {
        const std::string& laz_file = laz_files[i];
        const std::string& csv_file = csv_files[i];
        const std::string& fnSn = sn_files[i];

        const auto imuBaseName = std::filesystem::path(csv_file).filename().string();

        const std::filesystem::path output_path = std::filesystem::path(output_dir) / std::filesystem::path(laz_file).filename();
        const std::filesystem::path output_path_csv = std::filesystem::path(output_dir) / imuBaseName;
        std::cout << "Processing LAZ file: " << laz_file << std::endl;
        // Load mapping from id to sn

        const auto idToSn = MLvxCalib::GetIdToSnMapping(fnSn);
        const int imuNumberToUse = MLvxCalib::GetImuIdToUse(idToSn, imuSnToUse);
        const auto calibration = MLvxCalib::CombineIntoCalibration(idToSn, preloadedCalibration);
        auto data = load_point_cloud(laz_file.c_str(), false, 0, std::numeric_limits<double>::max(), calibration);
        std::cout << "Loaded " << data.size() << " points from " << laz_file << std::endl;

        std::vector<Eigen::Vector3d> pointcloud;
        std::vector<unsigned short> intensity;
        std::vector<double> timestamps;
        pointcloud.reserve(data.size());
        intensity.reserve(data.size());
        timestamps.reserve(data.size());
        int counter = 0;
        for (const auto& point : data)
        {
            pointcloud.push_back(point.point);
            intensity.push_back(point.intensity);
            timestamps.push_back(double(point.timestamp) / 1e9);
        }
        exportLaz(output_path.string(), pointcloud, intensity, timestamps, 0, 0, 0);
        std::cout << "Saved processed points to: " << output_path.string() << std::endl;

        // imu
        auto imu = load_imu(csv_file.c_str(), imuNumberToUse);

        std::ofstream out(output_path_csv.string());
        if (!out.is_open())
        {
            std::cerr << "Failed to open output file: " << output_path_csv.string() << std::endl;
            continue;
        }
        out << "timestamp timestampUnix accX accY accZ gyroX gyroY gyroZ\n";
        for (const auto& [timestamp, gyro, accel] : imu)
        {
            out << static_cast<uint64_t>(1e9 * timestamp.first) << " " << static_cast<uint64_t>(1e9 * timestamp.second) << " "
                << accel.axis.x << " " << accel.axis.y << " " << accel.axis.z << " " << gyro.axis.x << " " << gyro.axis.y << " "
                << gyro.axis.z << "\n";
        }
        std::cout << "Saved IMU data to: " << output_path_csv.string() << std::endl;
    }
}