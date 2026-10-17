#pragma once

#include <string>

namespace easy_gui
{
    //! Phases of the lidar_odometry_step_1 child process, in the order they occur.
    enum class Step1Phase
    {
        Starting,
        LoadingFiles, // reading the LAZ files (load_data)
        ImuTrajectory, // AHRS / initial trajectory from the IMU (calculate_trajectory)
        Preparing, // cutting the trajectory into odometry chunks (compute_step_1)
        Odometry, // the lidar odometry itself (compute_step_2)
        Saving, // save_result
        Finished
    };

    const char* phaseLabel(Step1Phase phase);

    struct Step1Progress
    {
        Step1Phase phase = Step1Phase::Starting;
        int files_total = 0;
        int files_done = 0;
        int prepare_done = 0;
        int prepare_total = 0;
        int chunks_done = 0;
        int chunks_total = 0;
        int scans_saved = 0;
        long long points_kept = 0; // "TOTAL: n" after loading: points left after the load filter

        bool failed = false;
        std::string failure; // plain-language reason, if one was recognised
        std::string last_error_line; // last raw line that looked like an error, for the details view

        //! Fraction 0..1 within the current phase, or -1 if the phase gives no measurable progress.
        double phaseFraction() const;
        //! Rough fraction 0..1 of the whole step 1 run (odometry dominates the run time).
        double overallFraction() const;
    };

    //! Turns lidar_odometry_step_1's console output into progress.
    //!
    //! NOTE: this parses free-form log text that step 1 does not treat as an interface. It only uses a few stable
    //! markers (see the unit test, which feeds an excerpt of a real log), never fails on unknown lines, and the GUI
    //! falls back to an indeterminate bar if a phase never reports numbers. Parallel file loading can interleave
    //! lines, so per-file progress counts markers instead of relying on line order.
    class ProgressParser
    {
    public:
        void feed(const std::string& line);
        const Step1Progress& progress() const
        {
            return progress_;
        }

    private:
        void advance(Step1Phase phase);
        void fail(const std::string& reason, const std::string& line);
        Step1Progress progress_;
    };
} // namespace easy_gui
