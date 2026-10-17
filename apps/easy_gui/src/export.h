#pragma once

#include "preflight.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>

namespace easy_gui
{
    //! Values of point_origin, the extra-bytes field of every exported point.
    enum PointOrigin : uint8_t
    {
        kStep1 = 0, // lidar odometry's (step 1's) own point, placed by step 1
        kNearPoint = 1, // raw point step 1's distance filters removed, added by Easy GUI at its real time
        kGapPoint = 2, // raw point inside step 1's limits that step 1 skipped between its 0.1 s pieces, added at its real time
        kCheckBand = 3, // test mode only: raw point step 1 also exported, added again to check the alignment
    };

    struct ExportRequest
    {
        std::filesystem::path session_file; // step 1 session.mjs
        RecordingFiles recording; // the raw files step 1 read (near points come from them)
        int lidar_id = 0; // user_data of the raw points (single-lidar recordings only)
        double near_min_range_m = 0.4; // near points closer than this (3D) are left out
        //! Test mode: > 0 also adds the raw points with filter_threshold_xy_inner < horizontal range < this, which step 1
        //! exported too, as kCheckBand, so their placement can be compared with step 1's own points.
        double check_band_outer_m = 0.0;
        //! Experimental: place every point from the raw files with the trajectory pose at its real gps_time, instead of
        //! taking step 1's points (scan_lio_N.laz, placed at step 1's ramp time). Same distance filters, near-point and
        //! gap-point rules; raw gps_time is written. point_origin = 0 then means "inside step 1's distance limits, and
        //! not in a gap". check_band_outer_m is ignored (those points are all included). Measured slightly less sharp
        //! than the default (see CLAUDE.md), hence off.
        bool place_by_raw_time = false;
        std::filesystem::path laz_file;
        std::filesystem::path trajectory_file;
        std::filesystem::path readme_file;
    };

    struct ExportStats
    {
        size_t scans = 0;
        size_t trajectory_nodes = 0;
        uint64_t odometry_points = 0; // point_origin = 0
        uint64_t near_points = 0; // point_origin = 1
        uint64_t gap_points = 0; // point_origin = 2
        uint64_t check_band_points = 0; // point_origin = 3
        uint64_t raw_points_read = 0;
        uint64_t raw_no_return = 0; // (0,0,0) or gps_time 0: never exported
        uint64_t raw_outside_trajectory = 0; // exported candidates with no trajectory node within 10 ms
        double range_min_m = 0.0;
        double range_max_m = 0.0;
        //! range_m histogram, 0.1 m bins from 0 m; the last bin collects everything beyond.
        std::array<uint64_t, 1001> range_histogram{};
        uint64_t laz_bytes = 0;
        uint64_t trajectory_bytes = 0;
        double filter_threshold_xy_inner = 0.0;
        double filter_threshold_xy_outer = 0.0;
        double threshould_output_filter = 0.0;
        int threshold_nr_poses = 0; // trajectory poses per step 1 piece (worker chunk)
    };

    //! Called with the fraction done (0..1) and a short description of the current step.
    using ExportProgress = std::function<void(double fraction, const std::string& step)>;

    //! Writes the Easy GUI results of one step 1 session:
    //!  - laz_file: LAS 1.4 point format 6 LAZ with all step 1 points plus the near points (see README.txt), in
    //!    step 1's coordinate frame, poses as step 2 applies them (session_poses.mrp x the scan's points);
    //!  - trajectory_file: CSV, one row per trajectory node;
    //!  - readme_file: plain-text description of both files.
    //! Streams one scan (step 1 chunk) or one raw file at a time, so memory does not grow with the recording.
    //! Returns false with a plain-language `error` on failure, or when `cancel` became true.
    bool exportSession(
        const ExportRequest& request,
        const std::atomic<bool>& cancel,
        const ExportProgress& progress,
        ExportStats& stats,
        std::string& error);

    //! The step 1 parameter file in a session folder (HDMapping_params_*.toml, written by save_result), or empty.
    std::filesystem::path findSessionParameterFile(const std::filesystem::path& session_folder);
} // namespace easy_gui
