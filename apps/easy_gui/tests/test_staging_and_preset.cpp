#include <doctest.h>

#include "forestry_preset.h"
#include "staging.h"
#include "test_helpers.h"

#include <toml_io.h>

#include <memory>

using namespace easy_gui;
using easy_gui_test::TempDir;
namespace fs = std::filesystem;

TEST_CASE("staging links exactly the files step 1 reads and never touches the recording")
{
    TempDir tmp;
    const fs::path rec = tmp.path / "rec";
    easy_gui_test::writeRecording(rec);
    const auto files = classifyRecordingFiles(rec);

    std::string method, error;
    REQUIRE(createStagingFolder(files, tmp.path / "staging", method, error));
    CHECK(fs::exists(tmp.path / "staging" / "lidar0000.laz"));
    CHECK(fs::exists(tmp.path / "staging" / "imu0001.csv"));
    CHECK(fs::exists(tmp.path / "staging" / "lidar0000.sn"));
    CHECK_FALSE(fs::exists(tmp.path / "staging" / "lidar0002.laz")); // unpaired: load_data would skip it anyway
    CHECK_FALSE(fs::exists(tmp.path / "staging" / "status0000.json"));

    // The staged view reads the same data.
    CHECK(fs::file_size(tmp.path / "staging" / "lidar0001.laz") == fs::file_size(rec / "lidar0001.laz"));

    // Removing the staging folder (as the pipeline does) leaves the recording intact.
    fs::remove_all(tmp.path / "staging");
    CHECK(fs::exists(rec / "lidar0000.laz"));
    CHECK(fs::exists(rec / "imu0000.csv"));

    SUBCASE("an existing staging folder is refused")
    {
        fs::create_directories(tmp.path / "staging2");
        CHECK_FALSE(createStagingFolder(files, tmp.path / "staging2", method, error));
    }
}

TEST_CASE("the step 1 parameter file is preset 2 plus the output-only filter change")
{
    TempDir tmp;
    std::string error;
    REQUIRE(writeStep1ParameterFile(tmp.path / "p.toml", error));

    auto params = std::make_unique<LidarOdometryParams>();
    TomlIO toml_io;
    REQUIRE(toml_io.LoadParametersFromTomlFile((tmp.path / "p.toml").string(), *params));

    CHECK(params->decimation == doctest::Approx(0.01));
    CHECK(params->in_out_params_indoor.resolution_X == doctest::Approx(0.1));
    CHECK(params->in_out_params_outdoor.resolution_Z == doctest::Approx(0.3));
    CHECK(params->filter_threshold_xy_inner == doctest::Approx(1.0));
    CHECK(params->filter_threshold_xy_outer == doctest::Approx(70.0));
    CHECK(params->nr_iter == 500);
    CHECK(params->sliding_window_trajectory_length_threshold == doctest::Approx(10000.0));
    CHECK(params->max_distance_lidar == doctest::Approx(70.0));
    CHECK_FALSE(params->use_robust_and_accurate_lidar_odometry);
    // The preset says 1.5; Easy GUI lowers it to the load filter (output-only, see forestry_preset.h).
    CHECK(params->threshould_output_filter == doctest::Approx(params->filter_threshold_xy_inner));
}
