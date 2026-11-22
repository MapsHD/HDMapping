#pragma once

#include "recording_source.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

//! A Mandeye recording folder: lidarNNNN.laz chunks paired with imuNNNN.csv files by their last four stem characters,
//! plus the optional calibration (.mjc or calibration .json), .sn file and MURATA_IMU folder.
class LazCsvSource : public RecordingSource
{
public:
    //! Sorts input_file_names, pairs the files and loads and checks the calibration; exits the process on a wrong
    //! calibration file, as load_data always did. Returns null when there are not enough files or the folder is missing.
    static std::unique_ptr<LazCsvSource> open(std::vector<std::string>& input_file_names, bool debugMsg);

    std::filesystem::path workingDirectory() const override;
    size_t chunkCount() const override;
    std::string chunkName(size_t i) const override;
    std::vector<Point3Di> loadPoints(size_t i, const PointFilter& filter) const override;
    ChunkStamp stamp(size_t i) const override;
    void loadImu(Imu& imu_data, std::vector<InputFileSpan>& imuSpans) const override;

private:
    LazCsvSource() = default;

    std::filesystem::path working_directory_;
    std::vector<std::string> laz_files_; //!< the first chunkCount() LAZ files, each paired with csv_files_[i]
    std::vector<std::string> csv_files_;
    std::string sn_file_;
    int imu_number_to_use_ = 0;
    std::unordered_map<int, Eigen::Affine3d> calibration_;
};
