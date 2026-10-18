#include <doctest.h>

#include "pipeline.h"
#include "preflight.h"
#include "test_helpers.h"

#include <algorithm>

using namespace easy_gui;
using easy_gui_test::TempDir;
namespace fs = std::filesystem;

namespace
{
    bool hasFinding(const PreflightResult& r, Severity s, const std::string& text)
    {
        return std::any_of(
            r.findings.begin(),
            r.findings.end(),
            [&](const Finding& f)
            {
                return f.severity == s && f.message.find(text) != std::string::npos;
            });
    }
} // namespace

TEST_CASE("classifyRecordingFiles pairs like load_data")
{
    TempDir tmp;
    easy_gui_test::writeRecording(tmp.path / "rec");
    const auto f = classifyRecordingFiles(tmp.path / "rec");
    CHECK(f.laz_files.size() == 3);
    CHECK(f.csv_files.size() == 2);
    CHECK(f.pairs == 2);
    CHECK(f.sn_file.filename() == "lidar0000.sn");
    CHECK(f.calibration_file.empty()); // status*.json is not a calibration
}

TEST_CASE("a plausible recording passes with a warning about the unmatched file")
{
    TempDir tmp;
    easy_gui_test::writeRecording(tmp.path / "rec");
    const auto r = runPreflight(tmp.path / "rec", tmp.path / "out");
    CHECK(r.ok());
    CHECK(r.lidar_id == 0);
    CHECK(r.total_points == 4000);
    CHECK(r.duration_s == doctest::Approx(60.0).epsilon(0.01));
    CHECK(hasFinding(r, Severity::Ok, "Recording looks good"));
    CHECK(hasFinding(r, Severity::Warning, "lidar0002.laz"));

    // Disk estimate: 120 B per raw point of work space + 2.25x the paired raw LAZ size of results, always reported.
    const uint64_t laz_bytes = fs::file_size(tmp.path / "rec" / "lidar0000.laz") + fs::file_size(tmp.path / "rec" / "lidar0001.laz");
    CHECK(r.raw_laz_bytes == laz_bytes);
    CHECK(r.estimated_work_bytes == 4000 * 120);
    CHECK(r.estimated_result_bytes == laz_bytes * 9 / 4);
    CHECK(r.estimated_disk_bytes == r.estimated_work_bytes + r.estimated_result_bytes);
    CHECK(hasFinding(r, Severity::Ok, "Disk space: about"));
}

TEST_CASE("errors are reported in plain language")
{
    TempDir tmp;
    const fs::path rec = tmp.path / "rec";

    SUBCASE("missing folder")
    {
        const auto r = runPreflight(tmp.path / "nope", tmp.path / "out");
        CHECK_FALSE(r.ok());
        CHECK(hasFinding(r, Severity::Error, "does not exist"));
    }
    SUBCASE("not a recording")
    {
        fs::create_directories(rec);
        easy_gui_test::writeText(rec / "notes.txt", "hello");
        const auto r = runPreflight(rec, tmp.path / "out");
        CHECK_FALSE(r.ok());
        CHECK(hasFinding(r, Severity::Error, "does not look like a Mandeye recording"));
    }
    SUBCASE("damaged point cloud file")
    {
        easy_gui_test::writeRecording(rec);
        easy_gui_test::writeText(rec / "lidar0001.laz", "this is not a laz file");
        const auto r = runPreflight(rec, tmp.path / "out");
        CHECK_FALSE(r.ok());
        CHECK(hasFinding(r, Severity::Error, "lidar0001.laz"));
    }
    SUBCASE("two lidars")
    {
        easy_gui_test::writeRecording(rec);
        easy_gui_test::writeRawLaz(rec / "lidar0001.laz", 2000, 1783328659.0, 1);
        const auto r = runPreflight(rec, tmp.path / "out");
        CHECK_FALSE(r.ok());
        CHECK(hasFinding(r, Severity::Error, "more than one lidar"));
    }
    SUBCASE("clocks do not overlap")
    {
        easy_gui_test::writeRecording(rec);
        easy_gui_test::writeImuCsv(rec / "imu0000.csv", 6000, 1000.0);
        easy_gui_test::writeImuCsv(rec / "imu0001.csv", 6000, 1030.0);
        const auto r = runPreflight(rec, tmp.path / "out");
        CHECK_FALSE(r.ok());
        CHECK(hasFinding(r, Severity::Error, "do not overlap"));
    }
    SUBCASE("output inside the recording")
    {
        easy_gui_test::writeRecording(rec);
        const auto r = runPreflight(rec, rec / "results");
        CHECK_FALSE(r.ok());
        CHECK(hasFinding(r, Severity::Error, "inside the recording folder"));
    }
    SUBCASE("output folder not empty")
    {
        easy_gui_test::writeRecording(rec);
        fs::create_directories(tmp.path / "out");
        easy_gui_test::writeText(tmp.path / "out" / "old.txt", "x");
        const auto r = runPreflight(rec, tmp.path / "out");
        CHECK_FALSE(r.ok());
        CHECK(hasFinding(r, Severity::Error, "already contains files"));
    }
    SUBCASE("calibration for another scanner")
    {
        easy_gui_test::writeRecording(rec);
        easy_gui_test::writeText(
            rec / "calibration.json", R"({"calibration": {"OTHERSERIAL": {"identity": "true"}}, "imuToUse": "OTHERSERIAL"})");
        const auto r = runPreflight(rec, tmp.path / "out");
        CHECK_FALSE(r.ok());
        CHECK(hasFinding(r, Severity::Error, "different scanner"));
    }
}

TEST_CASE("isSameOrInside")
{
    CHECK(isSameOrInside("/a/b/c", "/a/b"));
    CHECK(isSameOrInside("/a/b", "/a/b"));
    CHECK(isSameOrInside("/a/b/", "/a/b"));
    CHECK_FALSE(isSameOrInside("/a/bc", "/a/b"));
    CHECK_FALSE(isSameOrInside("/a", "/a/b"));
}

TEST_CASE("resume_unfinished accepts an output folder that holds only an earlier attempt's own files")
{
    TempDir tmp;
    easy_gui_test::writeRecording(tmp.path / "rec");
    const fs::path out = tmp.path / "out";
    fs::create_directories(out / "work" / "lio_session");
    easy_gui_test::writeText(out / "easy_gui.log", "earlier attempt\n");
    easy_gui_test::writeText(out / "point_cloud.laz", "half written");
    CHECK(holdsUnfinishedOutput(out));

    CHECK_FALSE(runPreflight(tmp.path / "rec", out).ok()); // single mode: still refused
    const auto r = runPreflight(tmp.path / "rec", out, true);
    CHECK(r.ok());
    CHECK(hasFinding(r, Severity::Ok, "unfinished earlier attempt"));

    SUBCASE("a finished batch result (done marker) is not unfinished")
    {
        easy_gui_test::writeText(out / "easy_gui_done.txt", "done");
        CHECK_FALSE(holdsUnfinishedOutput(out));
        CHECK_FALSE(runPreflight(tmp.path / "rec", out, true).ok());
    }
    SUBCASE("anything else in the folder is not ours")
    {
        easy_gui_test::writeText(out / "notes.txt", "mine");
        CHECK_FALSE(holdsUnfinishedOutput(out));
        CHECK_FALSE(runPreflight(tmp.path / "rec", out, true).ok());
    }
    SUBCASE("missing and empty folders are not unfinished attempts")
    {
        CHECK_FALSE(holdsUnfinishedOutput(tmp.path / "missing"));
        fs::create_directories(tmp.path / "empty");
        CHECK_FALSE(holdsUnfinishedOutput(tmp.path / "empty"));
    }
}

TEST_CASE("holdsUnfinishedOutput knows every file of OutputLayout except the done marker")
{
    TempDir tmp;
    const OutputLayout out(tmp.path / "out");
    fs::create_directories(out.session);
    for (const auto& file : { out.log_file, out.point_cloud, out.trajectory, out.readme, out.step1_parameters })
        easy_gui_test::writeText(file, "x");
    CHECK(holdsUnfinishedOutput(out.root));
    easy_gui_test::writeText(out.done_marker, "x");
    CHECK_FALSE(holdsUnfinishedOutput(out.root));
}
