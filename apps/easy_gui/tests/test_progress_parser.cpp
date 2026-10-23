#include <doctest.h>

#include "progress_parser.h"

#include <string>
#include <vector>

using namespace easy_gui;

namespace
{
    // Excerpt of a real lidar_odometry_step_1 CLI log (forest walk, 115 file pairs), paths shortened. The order and
    // the interleaving of the per-file lines (parallel loading) are as captured.
    const std::vector<std::string> kRealLogStart = {
        "SUCCESS: Version compatible: 0.105.0",
        "Parameters loaded OK from: /out/work/easy_gui_step1.toml",
        "\"/out/work/staging/lidar0080.laz\"",
        "Loading data..",
        "--------------",
        "Only using first SN file found",
        "There is no calibration file in folder!",
        "IGNORE THIS MESSAGE IF YOU ONLY HAVE 1 LiDAR",
        "loading IMU data from 'imu****.csv' using ID 0 from reference '/out/work/staging/lidar0000.sn' ..",
        "Loading point clouds (115 files)..",
        "total   - filter = kept",
        "-----------------------",
        "header->number_of_point_records: 1008288",
        "counter_filtered_points: 141284",
        "1008288 - 141284 = 867004",
        "header->number_of_point_records: 1002336",
        "counter_filtered_points: 184105",
        "1002336 - 184105 = 818231",
        "counter_filtered_points: 189379", // its header line was interleaved away
        "1000704 - 189379 = 811325",
    };

    const std::vector<std::string> kRealLogMiddle = {
        "TOTAL: 93371048 (lazy: ranges recorded, clouds loaded on demand)",
        "..loading finished.",
        "start calculating trajectory..",
        "AHRS: Fusion (gain=0.5, samples=115959)",
        "number of poses: 114944",
        "Creating folder: \"/out/work/staging/cache\"",
        "running iterations: 1/5747",
        "running iterations: 2851/5747",
        "[2026-10-17 18:40:42.920] [info] finished optimizing (since start : 0.100 [s]) - 1 / 5747 at iteration 172 / 500 in (load "
        "vector 0.012 [s], step1 0.000 [s], step2 0.000 [s], odometry 0.066 [s]) with acc_distance 0.000[m], delta<1.0e-12 "
        "(converged)",
        "[2026-10-17 18:47:02.538] [info] finished optimizing (since start : 379.716 [s]) - 1810 / 5747 at iteration 182 / 500 in "
        "(load vector 0.003 [s], step1 0.000 [s], step2 0.000 [s], odometry 0.195 [s]) with acc_distance 157.907[m], delta<1.0e-12 "
        "(converged)",
    };
} // namespace

TEST_CASE("phases and numbers from a real log")
{
    ProgressParser parser;
    CHECK(parser.progress().phase == Step1Phase::Starting);
    CHECK(parser.progress().phaseFraction() < 0.0);

    for (const auto& line : kRealLogStart)
        parser.feed(line);
    CHECK(parser.progress().phase == Step1Phase::LoadingFiles);
    CHECK(parser.progress().files_total == 115);
    CHECK(parser.progress().files_done == 3);
    CHECK(parser.progress().phaseFraction() == doctest::Approx(3.0 / 115.0));

    for (size_t i = 0; i < 4; i++)
        parser.feed(kRealLogMiddle[i]);
    CHECK(parser.progress().points_kept == 93371048);
    CHECK(parser.progress().phase == Step1Phase::ImuTrajectory);
    CHECK(parser.progress().phaseFraction() < 0.0); // no numbers: the GUI shows an indeterminate bar

    for (size_t i = 4; i < 8; i++)
        parser.feed(kRealLogMiddle[i]);
    CHECK(parser.progress().phase == Step1Phase::Preparing);
    CHECK(parser.progress().prepare_done == 2851);
    CHECK(parser.progress().prepare_total == 5747);

    parser.feed(kRealLogMiddle[8]);
    parser.feed(kRealLogMiddle[9]);
    CHECK(parser.progress().phase == Step1Phase::Odometry);
    CHECK(parser.progress().chunks_done == 1810);
    CHECK(parser.progress().chunks_total == 5747);
    const double overall = parser.progress().overallFraction();
    CHECK(overall > 0.10);
    CHECK(overall < 0.50);
    CHECK_FALSE(parser.progress().failed);
}

TEST_CASE("saving and finishing")
{
    ProgressParser parser;
    parser.feed("[2026-10-17 18:40:42.920] [info] finished optimizing (since start : 1.0 [s]) - 102 / 102 at iteration 1 / 500 in x");
    parser.feed("original_points_to_save_cache_file_name \"/out/work/staging/cache/session_chunk_0.cache\"");
    CHECK(parser.progress().phase == Step1Phase::Saving);
    parser.feed("saving to: \"/out/work/lio_session/scan_lio_0.laz\"");
    CHECK(parser.progress().scans_saved == 1);
    CHECK(parser.progress().phaseFraction() == doctest::Approx(0.5)); // 102 chunks -> 2 scan files
    parser.feed("remove cache: '/out/work/staging/cache' FINISHED");
    CHECK(parser.progress().phase == Step1Phase::Finished);
    CHECK(parser.progress().overallFraction() == doctest::Approx(1.0));
}

TEST_CASE("phases never go backwards")
{
    ProgressParser parser;
    parser.feed("[x] [info] finished optimizing (since start : 1.0 [s]) - 5 / 10 at iteration 1 / 500 in x");
    parser.feed("running iterations: 1/10"); // a late line from an earlier phase
    CHECK(parser.progress().phase == Step1Phase::Odometry);
}

TEST_CASE("known failures become plain-language messages")
{
    struct Case
    {
        const char* line;
        const char* expected_fragment;
    };
    const Case cases[] = {
        { "MISSING CALIBRATION FOR imuSnToUse: ABC!!!", "calibration file does not match" },
        { "WRONG CALIBRATION FILE! THE SERIAL NUMBER SHOULD BE X!!!", "different scanner" },
        { "System is out of memory : std::bad_alloc", "ran out of memory" },
        { "Loading poses went wrong! Could not load poses!", "IMU" },
        { "[2026-10-17] [error] points_global spill: write failed", "disk is probably full" },
        { "lazy_load_raw_clouds: '/x/lidar0001.laz' changed or vanished since it was first read", "USB stick" },
        { "Lidar odometry failed: no results saved", "did not succeed" },
    };
    for (const auto& c : cases)
    {
        CAPTURE(c.line);
        ProgressParser parser;
        parser.feed(c.line);
        CHECK(parser.progress().failed);
        CHECK(parser.progress().failure.find(c.expected_fragment) != std::string::npos);
        CHECK(parser.progress().last_error_line == c.line);
    }

    SUBCASE("the first cause wins")
    {
        ProgressParser parser;
        parser.feed("MISSING CALIBRATION FOR imuSnToUse: ABC!!!");
        parser.feed("Lidar odometry failed: no results saved");
        CHECK(parser.progress().failure.find("calibration") != std::string::npos);
    }
}

TEST_CASE("unknown and malformed lines are ignored")
{
    ProgressParser parser;
    parser.feed("");
    parser.feed("running iterations: x/y");
    parser.feed("finished optimizing - a / b at iteration");
    parser.feed("Loading point clouds (lots files)..");
    CHECK(parser.progress().phase == Step1Phase::Starting);
    CHECK_FALSE(parser.progress().failed);
}
