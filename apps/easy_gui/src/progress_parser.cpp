#include "progress_parser.h"

#include <algorithm>
#include <regex>

namespace easy_gui
{
    namespace
    {
        struct KnownFailure
        {
            const char* marker;
            const char* message;
        };

        // Lines that mean step 1 is going to fail (or already has), with what to tell the user.
        const KnownFailure kKnownFailures[] = {
            { "MISSING CALIBRATION",
              "The calibration file does not match this scanner, so the recording cannot be processed with it. Remove the calibration "
              "file from the recording, or use the one that belongs to this scanner." },
            { "WRONG CALIBRATION FILE",
              "The calibration file belongs to a different scanner. Remove it from the recording, or use the one that belongs to this "
              "scanner." },
            { "System is out of memory",
              "The computer ran out of memory. Close other programs and try again, or process the recording on a computer with more "
              "memory." },
            { "Loading poses went wrong",
              "No trajectory could be computed from the IMU data. The IMU files are probably empty or damaged." },
            { "DLL ERROR",
              "A point cloud file could not be read (damaged, or not fully copied from the scanner). Copy the recording again." },
            { "points_global spill", "Writing temporary data failed. The disk is probably full: free some space and try again." },
            { "changed or vanished since it was first read",
              "A recording file changed or disappeared during processing (was the USB stick removed?). Keep the recording available "
              "until processing has finished." },
            { "Lidar odometry failed",
              "Lidar odometry did not succeed on this recording. Check that the scanner was moving steadily and that the recording "
              "is complete; the details below show where it stopped." },
        };

        const std::regex kFilesTotal(R"(Loading point clouds \((\d+) files\))");
        const std::regex kPointsKept(R"(^TOTAL: (\d+))");
        const std::regex kPrepare(R"(running iterations: (\d+)/(\d+))");
        const std::regex kOdometry(R"(finished optimizing .* - (\d+) / (\d+) at iteration)");
        const std::regex kSavedScan(R"(saving to: .*scan_lio_(\d+)\.laz)");

        int toInt(const std::ssub_match& m)
        {
            try
            {
                return std::stoi(m.str());
            } catch (...)
            {
                return 0;
            }
        }

        // save_result writes one scan_lio file per 51 odometry chunks (its "counter > 50" grouping).
        int expectedScans(int chunks)
        {
            return std::max(1, (chunks + 50) / 51);
        }
    } // namespace

    const char* phaseLabel(Step1Phase phase)
    {
        switch (phase)
        {
        case Step1Phase::Starting:
            return "Starting";
        case Step1Phase::LoadingFiles:
            return "Reading the recording";
        case Step1Phase::ImuTrajectory:
            return "Computing the motion from the IMU";
        case Step1Phase::Preparing:
            return "Preparing lidar odometry";
        case Step1Phase::Odometry:
            return "Lidar odometry";
        case Step1Phase::Saving:
            return "Saving the odometry result";
        case Step1Phase::Finished:
            return "Lidar odometry finished";
        }
        return "";
    }

    double Step1Progress::phaseFraction() const
    {
        auto ratio = [](int done, int total)
        {
            return total > 0 ? std::clamp(static_cast<double>(done) / total, 0.0, 1.0) : -1.0;
        };
        switch (phase)
        {
        case Step1Phase::LoadingFiles:
            return ratio(files_done, files_total);
        case Step1Phase::Preparing:
            return ratio(prepare_done, prepare_total);
        case Step1Phase::Odometry:
            return ratio(chunks_done, chunks_total);
        case Step1Phase::Saving:
            return ratio(scans_saved, expectedScans(chunks_total));
        case Step1Phase::Finished:
            return 1.0;
        default:
            return -1.0;
        }
    }

    double Step1Progress::overallFraction() const
    {
        // Share of the run time per phase, measured on a 10-minute forest walk (115 file pairs, 93 M points kept).
        struct Span
        {
            Step1Phase phase;
            double weight;
        };
        static const Span kSpans[] = {
            { Step1Phase::Starting, 0.0 },   { Step1Phase::LoadingFiles, 0.04 }, { Step1Phase::ImuTrajectory, 0.01 },
            { Step1Phase::Preparing, 0.05 }, { Step1Phase::Odometry, 0.85 },     { Step1Phase::Saving, 0.05 },
        };
        if (phase == Step1Phase::Finished)
            return 1.0;
        double done = 0.0;
        for (const auto& span : kSpans)
        {
            if (span.phase == phase)
            {
                const double f = phaseFraction();
                return done + span.weight * (f < 0.0 ? 0.0 : f);
            }
            done += span.weight;
        }
        return done;
    }

    void ProgressParser::advance(Step1Phase phase)
    {
        // Phases only move forward: a stray late line from an earlier phase must not move the bar backwards.
        if (static_cast<int>(phase) > static_cast<int>(progress_.phase))
            progress_.phase = phase;
    }

    void ProgressParser::fail(const std::string& reason, const std::string& line)
    {
        progress_.last_error_line = line;
        if (!progress_.failed) // the first recognised cause is the real one; later lines are consequences
        {
            progress_.failed = true;
            progress_.failure = reason;
        }
    }

    void ProgressParser::feed(const std::string& line)
    {
        for (const auto& known : kKnownFailures)
            if (line.find(known.marker) != std::string::npos)
            {
                fail(known.message, line);
                return;
            }
        if (line.find("[error]") != std::string::npos || line.find("ERROR") != std::string::npos)
            progress_.last_error_line = line;

        std::smatch m;
        if (line.find("Loading data..") != std::string::npos)
            advance(Step1Phase::LoadingFiles);
        else if (std::regex_search(line, m, kFilesTotal))
        {
            advance(Step1Phase::LoadingFiles);
            progress_.files_total = toInt(m[1]);
        }
        else if (line.find("counter_filtered_points:") != std::string::npos)
            progress_.files_done++; // phaseFraction() clamps
        else if (std::regex_search(line, m, kPointsKept))
        {
            try
            {
                progress_.points_kept = std::stoll(m[1].str());
            } catch (...)
            {
            }
        }
        else if (line.find("start calculating trajectory") != std::string::npos)
            advance(Step1Phase::ImuTrajectory);
        else if (std::regex_search(line, m, kPrepare))
        {
            advance(Step1Phase::Preparing);
            progress_.prepare_done = toInt(m[1]);
            progress_.prepare_total = toInt(m[2]);
        }
        else if (line.find("step_1_done") != std::string::npos)
            progress_.prepare_done = progress_.prepare_total;
        else if (std::regex_search(line, m, kOdometry))
        {
            advance(Step1Phase::Odometry);
            progress_.chunks_done = std::max(progress_.chunks_done, toInt(m[1]));
            progress_.chunks_total = toInt(m[2]);
        }
        else if (std::regex_search(line, m, kSavedScan))
        {
            advance(Step1Phase::Saving);
            progress_.scans_saved = std::max(progress_.scans_saved, toInt(m[1]) + 1);
        }
        else if (line.find("original_points_to_save_cache_file_name") != std::string::npos)
            advance(Step1Phase::Saving);
        else if (line.find("remove cache:") != std::string::npos && line.find("FINISHED") != std::string::npos)
            advance(Step1Phase::Finished);
    }
} // namespace easy_gui
