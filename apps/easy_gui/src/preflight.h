#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace easy_gui
{
    enum class Severity
    {
        Ok,
        Warning,
        Error
    };

    struct Finding
    {
        Severity severity;
        std::string message; // plain language, shown as-is in the GUI
    };

    //! The recording's files, classified exactly as load_data() in lidar_odometry.cpp does it (sorted names; every
    //! .laz/.las is a point cloud, every .csv an IMU file; status*/cam0*/cam1* JSON ignored; the first other .json or
    //! .mjc is the calibration; the first .sn is used). load_data pairs point clouds and IMU files *by position* and
    //! only loads min(#laz, #csv) pairs, so `pairs` is what step 1 will actually read.
    struct RecordingFiles
    {
        std::vector<std::filesystem::path> laz_files;
        std::vector<std::filesystem::path> csv_files;
        std::filesystem::path sn_file;
        std::filesystem::path calibration_file;
        size_t pairs = 0;
    };

    RecordingFiles classifyRecordingFiles(const std::filesystem::path& recording);

    struct PreflightResult
    {
        RecordingFiles files;
        std::vector<Finding> findings;
        uint64_t total_points = 0; // point records in the paired LAZ files (incl. no-return points)
        double duration_s = 0.0; // IMU time span
        int lidar_id = -1; // the single lidar id (LAZ user_data) seen in the samples
        uint64_t raw_laz_bytes = 0; // size of the paired LAZ files
        uint64_t estimated_work_bytes = 0; // temporary, freed after the run
        uint64_t estimated_result_bytes = 0; // kept: step 1 session + exported point cloud and trajectory
        uint64_t estimated_disk_bytes = 0; // work + results: what must be free when the run starts

        bool ok() const
        {
            for (const auto& f : findings)
                if (f.severity == Severity::Error)
                    return false;
            return true;
        }
    };

    //! Checks the recording and the output folder before anything is started. Catches, with a plain message, the
    //! cases where step 1 would otherwise abort() (unreadable LAZ), exit() (calibration mismatch) or silently produce
    //! something incomplete (unpaired files, non-overlapping clocks).
    PreflightResult runPreflight(const std::filesystem::path& recording, const std::filesystem::path& output_folder);

    //! Temporary disk space of a run: step 1's caches (original + decimated + concatenated chunk points, ~45 B/point
    //! each with decimation 0.01). Measured peak on a 117 M-point forest walk: ~108 B per raw point; 120 B leaves
    //! some margin.
    uint64_t estimateWorkBytes(uint64_t total_points);

    //! Disk space of what is kept: the step 1 session (scan_lio LAZ, 0.82x the raw LAZ size measured), the exported
    //! LAZ (1.4 PF6 with extra bytes and the near points, 1.24x measured) and the trajectory CSV (0.03x): 2.25x.
    uint64_t estimateResultBytes(uint64_t raw_laz_bytes);

    //! True if `inner` is `outer` or lies inside it (both made absolute and normalised first).
    bool isSameOrInside(const std::filesystem::path& inner, const std::filesystem::path& outer);
} // namespace easy_gui
