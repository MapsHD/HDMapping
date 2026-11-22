#include "laz_csv_source.h"

#include "tbb/tbb.h"

#include <algorithm>
#include <iostream>

namespace fs = std::filesystem;

//! Time span and sample count of one IMU file's samples.
static InputFileSpan imu_file_span(const std::string& file, const Imu& samples)
{
    InputFileSpan span{ file };
    for (const auto& sample : samples)
    {
        const double t = std::get<0>(sample).first;
        span.first = span.count == 0 ? t : std::min(span.first, t);
        span.last = span.count == 0 ? t : std::max(span.last, t);
        span.count++;
    }
    return span;
}

std::unique_ptr<LazCsvSource> LazCsvSource::open(std::vector<std::string>& input_file_names, bool debugMsg)
{
    std::sort(input_file_names.begin(), input_file_names.end());
    std::vector<std::string> csv_files, laz_files;
    std::string sn_file;
    std::string calibrationFile;

    std::cout << "Loading data..\n--------------\n";

    // --- Separate files by type
    for (const auto& fileName : input_file_names)
    {
        std::string ext = fs::path(fileName).extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

        if (ext == ".csv")
            csv_files.push_back(fileName);
        else if (ext == ".json")
        {
            std::string stem = fs::path(fileName).filename().stem().string(); // get filename without extension

            // skip files not related to calibration
            if (!stem.starts_with("status") && !stem.starts_with("cam0") && !stem.starts_with("cam1"))
                if (calibrationFile.empty())
                    calibrationFile = fileName;
                else
                    std::cout << "Unexpected JSON file found (" << fileName << "). Ignored..\n";
        }
        else if ((ext == ".laz") || (ext == ".las"))
            laz_files.push_back(fileName);
        else if (ext == ".mjc")
        {
            if (calibrationFile.empty())
                calibrationFile = fileName;
            else
                std::cout << "Unexpected calibration file found (" << fileName << "). Ignored.." << std::endl;
        }
        else if ((ext == ".sn") && sn_file.empty())
        {
            sn_file = fileName;
            std::cout << "Only using first SN file found\n";
        }
    }

    auto get4index = [](const std::string& path) -> std::string
    {
        auto s = fs::path(path).stem().string();
        return s.size() > 4 ? s.substr(s.size() - 4) : s;
    };

    for (size_t i = 0; i < laz_files.size(); i++)
    {
        std::string lfs = get4index(laz_files[i]);

        bool found = false;
        for (size_t j = 0; j < csv_files.size(); j++)
        {
            std::string cfs = get4index(csv_files[j]);
            if (lfs.compare(cfs) == 0)
            {
                found = true;
                break;
            }
        }

        if (!found)
        {
            std::cout << "\n!!! there is no CSV file for: " << laz_files[i] << " !!! \n\n";
        }
    }

    for (size_t i = 0; i < csv_files.size(); i++)
    {
        std::string lfs = get4index(csv_files[i]);

        bool found = false;
        for (size_t j = 0; j < laz_files.size(); j++)
        {
            std::string cfs = get4index(laz_files[j]);
            if (lfs.compare(cfs) == 0)
            {
                found = true;
                break;
            }
        }

        if (!found)
        {
            std::cout << "\n!!! there is no LAZ file for: " << csv_files[i] << " !!! \n\n";
        }
    }

    if (input_file_names.size() <= 2)
    {
        std::cout << "Not enough files to continue: " << input_file_names.size() << std::endl;
        return nullptr;
    }

    std::string working_directory = fs::path(input_file_names.front()).parent_path().string();

    if (laz_files.size() != csv_files.size())
        std::cout << "!!! file number issue. continuing can lead to incomplete data!!!\n" << std::endl;

    // check if folder exists!
    if (!fs::exists(working_directory))
    {
        std::cerr << "Folder '" << working_directory << "' does not exist.\n";
        return nullptr;
    }

    // --- Calibration and IMU setup
    const auto preloadedCalibration = MLvxCalib::GetCalibrationFromFile(calibrationFile);
    const std::string imuSnToUse = MLvxCalib::GetImuSnToUse(calibrationFile);
    const auto idToSn = MLvxCalib::GetIdToSnMapping(sn_file);

    if (!preloadedCalibration.empty())
    {
        std::cout << "Loaded calibration for:\n";
        for (const auto& [sn, _] : preloadedCalibration)
            std::cout << " -> " << sn << "\n";

        bool hasError = false;

        for (const auto& [id, sn] : idToSn)
        {
            if (preloadedCalibration.find(sn) == preloadedCalibration.end())
            {
                std::cerr << "WRONG CALIBRATION FILE! THE SERIAL NUMBER SHOULD BE " << sn << "!!!\n";
                hasError = true;
            }
        }

        if (!hasError && preloadedCalibration.find(imuSnToUse) == preloadedCalibration.end())
        {
            std::cerr << "MISSING CALIBRATION FOR imuSnToUse: " << imuSnToUse << "!!!\n";
            std::cerr << "Available serial numbers in calibration file are:\n";
            for (const auto& [snKey, _] : preloadedCalibration)
                std::cerr << "  - " << snKey << "\n";

            hasError = true;
        }

        if (hasError)
        {
            std::cerr << "Press ENTER to exit...\n";
            std::cin.get();
            std::exit(EXIT_FAILURE);
        }
    }
    else
    {
        std::cout << "There is no calibration file in folder!" << std::endl;
        std::cout << "IGNORE THIS MESSAGE IF YOU ONLY HAVE 1 LiDAR" << std::endl;

        // example file for 2x LiDAR":
        /*
        {
            "calibration" : {
                "47MDL9T0020193" : {
                    "identity" : "true"
                },
                "47MDL9S0020300" :
                    {
                        "order" : "ROW",
                        "inverted" : "TRUE",
                        "data" : [
                            0.999824, 0.00466397, -0.0181595, -0.00425984,
                            -0.0181478, -0.00254457, -0.999832, -0.151599,
                            -0.0047094, 0.999986, -0.00245948, -0.146408,
                            0, 0, 0, 1
                        ]
                    }
            },
                            "imuToUse" : "47MDL9T0020193"
        }*/
    }

    std::unique_ptr<LazCsvSource> source(new LazCsvSource());
    source->working_directory_ = working_directory;
    source->sn_file_ = sn_file;
    source->imu_number_to_use_ = MLvxCalib::GetImuIdToUse(idToSn, imuSnToUse);
    // --- Precompute calibration once (major optimization)
    source->calibration_ = MLvxCalib::CombineIntoCalibration(idToSn, preloadedCalibration);

    const std::size_t minSize = std::min(laz_files.size(), csv_files.size());
    source->laz_files_.assign(laz_files.begin(), laz_files.begin() + minSize);
    source->csv_files_.assign(csv_files.begin(), csv_files.begin() + minSize);

    if (debugMsg) // if true this will break parallelism
    {
        for (const auto& [id, calib] : source->calibration_)
            std::cout << " id : " << id << "\n" << calib.matrix() << "\n";
    }
    return source;
}

fs::path LazCsvSource::workingDirectory() const
{
    return working_directory_;
}

size_t LazCsvSource::chunkCount() const
{
    return laz_files_.size();
}

std::string LazCsvSource::chunkName(size_t i) const
{
    return laz_files_[i];
}

std::vector<Point3Di> LazCsvSource::loadPoints(size_t i, const PointFilter& filter) const
{
    auto data = load_point_cloud(laz_files_[i].c_str(), filter.omit_timestamp_zero, filter.xy_inner, filter.xy_outer, calibration_);
    std::sort(
        data.begin(),
        data.end(),
        [](const Point3Di& a, const Point3Di& b)
        {
            return a.timestamp < b.timestamp;
        });
    return data;
}

ChunkStamp LazCsvSource::stamp(size_t i) const
{
    std::error_code ec;
    ChunkStamp s;
    s.size = fs::file_size(laz_files_[i], ec);
    if (!ec)
        s.time = fs::last_write_time(laz_files_[i], ec);
    s.valid = !ec;
    return s;
}

void LazCsvSource::loadImu(Imu& imu_data, std::vector<InputFileSpan>& imuSpans) const
{
    // --- Load IMU data (still sequential, could parallelize later)
    std::cout << "loading IMU data from 'imu****.csv' using ID " << imu_number_to_use_ << " from reference '" << sn_file_ << "' ..\n";

    const std::size_t minSize = csv_files_.size();
    // Each thread loads into its own local vector
    std::vector<Imu> imuChunks(minSize);

    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, minSize),
        [&](const tbb::blocked_range<size_t>& range)
        {
            for (size_t i = range.begin(); i < range.end(); ++i)
            {
                imuChunks[i] = load_imu(csv_files_[i].c_str(), imu_number_to_use_);
            }
        });

    for (size_t i = 0; i < minSize; ++i)
        imuSpans.push_back(imu_file_span(csv_files_[i], imuChunks[i]));

    // Merge all chunks sequentially at the end
    for (auto& chunk : imuChunks)
        imu_data.insert(imu_data.end(), std::make_move_iterator(chunk.begin()), std::make_move_iterator(chunk.end()));

    // Units with an external IMU (e.g. Murata) leave 'imu****.csv' header-only and log the IMU
    // into a subfolder, in files not aligned with the LiDAR chunks
    const fs::path murataImuDir = working_directory_ / "MURATA_IMU";
    if (imu_data.empty() && fs::is_directory(murataImuDir))
    {
        std::vector<std::string> murataFiles;
        for (const auto& entry : fs::directory_iterator(murataImuDir))
        {
            std::string ext = entry.path().extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
            if (entry.is_regular_file() && ext == ".csv" && entry.file_size() > 0)
                murataFiles.push_back(entry.path().string());
        }
        std::sort(murataFiles.begin(), murataFiles.end());
        std::cout << "no IMU data in 'imu****.csv', loading " << murataFiles.size() << " file(s) from '" << murataImuDir.string()
                  << "' ..\n";

        std::vector<Imu> murataChunks(murataFiles.size());
        tbb::parallel_for(
            size_t(0),
            murataFiles.size(),
            [&](size_t i)
            {
                murataChunks[i] = load_imu(murataFiles[i], imu_number_to_use_);
            });
        imuSpans.clear();
        for (size_t i = 0; i < murataFiles.size(); ++i)
            imuSpans.push_back(imu_file_span(murataFiles[i], murataChunks[i]));
        for (auto& chunk : murataChunks)
            imu_data.insert(imu_data.end(), std::make_move_iterator(chunk.begin()), std::make_move_iterator(chunk.end()));
    }
}
