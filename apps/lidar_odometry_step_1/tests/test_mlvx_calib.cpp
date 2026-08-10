#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include "../lidar_odometry_utils.h"

#include <cstdio>
#include <fstream>

namespace
{
    // Each test gets its own file under the OS temp dir, named after the
    // running test case, so parallel/leftover runs don't collide.
    class TempFile
    {
    public:
        explicit TempFile(const std::string& content)
            : m_path(
                  (std::filesystem::temp_directory_path()
                   / ("mlvx_calib_test_" + std::to_string(reinterpret_cast<uintptr_t>(this)) + ".tmp"))
                      .string())
        {
            std::ofstream f(m_path);
            f << content;
        }

        ~TempFile()
        {
            std::filesystem::remove(m_path);
        }

        const std::string& path() const
        {
            return m_path;
        }

    private:
        std::string m_path;
    };

    bool isIdentity(const Eigen::Affine3d& a)
    {
        return a.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-12);
    }
} // namespace

// ---------------------------------------------------------------------------
// GetIdToSnMapping: id<->serial-number mapping from the .sn file
// ---------------------------------------------------------------------------

TEST_CASE("GetIdToSnMapping: nonexistent file returns an empty map")
{
    auto result = MLvxCalib::GetIdToSnMapping("/nonexistent/path/does_not_exist.sn");
    CHECK(result.empty());
}

TEST_CASE("GetIdToSnMapping: parses 'id serial_number' lines")
{
    TempFile file("0 47MDL9T0020193\n1 47MDL9S0020300\n");
    auto result = MLvxCalib::GetIdToSnMapping(file.path());

    REQUIRE(result.size() == 2);
    CHECK(result.at(0) == "47MDL9T0020193");
    CHECK(result.at(1) == "47MDL9S0020300");
}

TEST_CASE("GetIdToSnMapping: skips malformed lines but keeps parsing the valid ones")
{
    // A line missing the serial-number column fails the `iss >> key >> value`
    // parse and is skipped (with a diagnostic to stderr); every other line is
    // still parsed independently.
    TempFile file("0 47MDL9T0020193\nnotanumber\n1 47MDL9S0020300\n");
    auto result = MLvxCalib::GetIdToSnMapping(file.path());

    REQUIRE(result.size() == 2);
    CHECK(result.at(0) == "47MDL9T0020193");
    CHECK(result.at(1) == "47MDL9S0020300");
}

TEST_CASE("GetIdToSnMapping: empty file returns an empty map")
{
    TempFile file("");
    auto result = MLvxCalib::GetIdToSnMapping(file.path());
    CHECK(result.empty());
}

// ---------------------------------------------------------------------------
// GetCalibrationFromFile: serial-number -> extrinsic calibration
// ---------------------------------------------------------------------------

TEST_CASE("GetCalibrationFromFile: nonexistent file returns an empty map")
{
    auto result = MLvxCalib::GetCalibrationFromFile("/nonexistent/path/does_not_exist.json");
    CHECK(result.empty());
}

TEST_CASE("GetCalibrationFromFile: invalid JSON returns an empty map")
{
    TempFile file("{ this is not valid json");
    auto result = MLvxCalib::GetCalibrationFromFile(file.path());
    CHECK(result.empty());
}

TEST_CASE("GetCalibrationFromFile: missing 'calibration' key returns an empty map")
{
    TempFile file(R"({"imuToUse": "SN1"})");
    auto result = MLvxCalib::GetCalibrationFromFile(file.path());
    CHECK(result.empty());
}

TEST_CASE("GetCalibrationFromFile: 'identity' entry yields the identity matrix")
{
    TempFile file(R"({"calibration": {"SN_IDENTITY": {"identity": "true"}}})");
    auto result = MLvxCalib::GetCalibrationFromFile(file.path());

    REQUIRE(result.count("SN_IDENTITY") == 1);
    CHECK(isIdentity(result.at("SN_IDENTITY")));
}

TEST_CASE("GetCalibrationFromFile: default (ROW) order reads translation from the last column")
{
    TempFile file(R"({
        "calibration": {
            "SN_ROW": {
                "data": [1,0,0,1, 0,1,0,2, 0,0,1,3, 0,0,0,1]
            }
        }
    })");
    auto result = MLvxCalib::GetCalibrationFromFile(file.path());

    REQUIRE(result.count("SN_ROW") == 1);
    const Eigen::Vector3d t = result.at("SN_ROW").translation();
    CHECK(t.isApprox(Eigen::Vector3d(1, 2, 3), 1e-12));
}

TEST_CASE("GetCalibrationFromFile: 'COLUMN' order transposes the raw data before use")
{
    // Filled row-major first (value(i,j) = data[i*4+j]), then transposed
    // because order == COLUMN -- so the *last row* of the raw data ends up
    // as the translation column after the transpose.
    TempFile file(R"({
        "calibration": {
            "SN_COLUMN": {
                "order": "COLUMN",
                "data": [1,0,0,0, 0,1,0,0, 0,0,1,0, 5,6,7,1]
            }
        }
    })");
    auto result = MLvxCalib::GetCalibrationFromFile(file.path());

    REQUIRE(result.count("SN_COLUMN") == 1);
    const Eigen::Vector3d t = result.at("SN_COLUMN").translation();
    CHECK(t.isApprox(Eigen::Vector3d(5, 6, 7), 1e-12));
}

TEST_CASE("GetCalibrationFromFile: 'inverted' flag inverts the parsed matrix")
{
    TempFile file(R"({
        "calibration": {
            "SN_INV": {
                "inverted": "true",
                "data": [1,0,0,2, 0,1,0,0, 0,0,1,0, 0,0,0,1]
            }
        }
    })");
    auto result = MLvxCalib::GetCalibrationFromFile(file.path());

    REQUIRE(result.count("SN_INV") == 1);
    const Eigen::Vector3d t = result.at("SN_INV").translation();
    CHECK(t.isApprox(Eigen::Vector3d(-2, 0, 0), 1e-9));
}

TEST_CASE("GetCalibrationFromFile: blacklisted serial numbers are removed from the result")
{
    TempFile file(R"({
        "calibration": {
            "SN_KEEP": {"identity": "true"},
            "SN_DROP": {"identity": "true"}
        },
        "blacklist": ["SN_DROP"]
    })");
    auto result = MLvxCalib::GetCalibrationFromFile(file.path());

    CHECK(result.count("SN_KEEP") == 1);
    CHECK(result.count("SN_DROP") == 0);
}

// ---------------------------------------------------------------------------
// GetImuSnToUse: serial number of the Livox to use for IMU data
// ---------------------------------------------------------------------------

TEST_CASE("GetImuSnToUse: nonexistent file returns an empty string")
{
    CHECK(MLvxCalib::GetImuSnToUse("/nonexistent/path/does_not_exist.json").empty());
}

TEST_CASE("GetImuSnToUse: invalid JSON returns an empty string")
{
    TempFile file("{ not json");
    CHECK(MLvxCalib::GetImuSnToUse(file.path()).empty());
}

TEST_CASE("GetImuSnToUse: missing 'imuToUse' key returns an empty string")
{
    TempFile file(R"({"calibration": {}})");
    CHECK(MLvxCalib::GetImuSnToUse(file.path()).empty());
}

TEST_CASE("GetImuSnToUse: non-string 'imuToUse' value returns an empty string")
{
    TempFile file(R"({"imuToUse": 123})");
    CHECK(MLvxCalib::GetImuSnToUse(file.path()).empty());
}

TEST_CASE("GetImuSnToUse: returns the serial number when present")
{
    TempFile file(R"({"imuToUse": "47MDL9T0020193"})");
    CHECK(MLvxCalib::GetImuSnToUse(file.path()) == "47MDL9T0020193");
}

// ---------------------------------------------------------------------------
// CombineIntoCalibration: (id->sn) + (sn->calibration) -> (id->calibration)
// ---------------------------------------------------------------------------

TEST_CASE("CombineIntoCalibration: empty calibration map returns an empty result regardless of idToSn")
{
    std::unordered_map<int, std::string> idToSn{ { 0, "SN1" } };
    std::unordered_map<std::string, Eigen::Affine3d> calibration;
    auto result = MLvxCalib::CombineIntoCalibration(idToSn, calibration);
    CHECK(result.empty());
}

TEST_CASE("CombineIntoCalibration: joins id->sn and sn->calibration by serial number")
{
    std::unordered_map<int, std::string> idToSn{ { 0, "SN1" }, { 1, "SN2" } };
    std::unordered_map<std::string, Eigen::Affine3d> calibration{
        { "SN1", Eigen::Affine3d(Eigen::Translation3d(1, 0, 0)) },
        { "SN2", Eigen::Affine3d(Eigen::Translation3d(2, 0, 0)) },
    };

    auto result = MLvxCalib::CombineIntoCalibration(idToSn, calibration);

    REQUIRE(result.size() == 2);
    CHECK(result.at(0).translation().isApprox(Eigen::Vector3d(1, 0, 0)));
    CHECK(result.at(1).translation().isApprox(Eigen::Vector3d(2, 0, 0)));
}

TEST_CASE("CombineIntoCalibration: an id whose serial number is absent from calibration throws")
{
    // Documented current behavior: lookup uses std::unordered_map::at(), so a
    // sensor id present in the .sn file but missing from the calibration
    // JSON is a hard error rather than being silently skipped.
    std::unordered_map<int, std::string> idToSn{ { 0, "SN_UNKNOWN" } };
    std::unordered_map<std::string, Eigen::Affine3d> calibration{
        { "SN_OTHER", Eigen::Affine3d::Identity() },
    };

    CHECK_THROWS_AS(MLvxCalib::CombineIntoCalibration(idToSn, calibration), std::out_of_range);
}

// ---------------------------------------------------------------------------
// GetImuIdToUse: sensor id of the Livox to use for IMU data
// ---------------------------------------------------------------------------

TEST_CASE("GetImuIdToUse: empty idToSn returns 0")
{
    std::unordered_map<int, std::string> idToSn;
    CHECK(MLvxCalib::GetImuIdToUse(idToSn, "SN1") == 0);
}

TEST_CASE("GetImuIdToUse: empty snToUse returns 0")
{
    std::unordered_map<int, std::string> idToSn{ { 5, "SN1" } };
    CHECK(MLvxCalib::GetImuIdToUse(idToSn, "") == 0);
}

TEST_CASE("GetImuIdToUse: returns the id matching the requested serial number")
{
    std::unordered_map<int, std::string> idToSn{ { 0, "SN1" }, { 5, "SN2" } };
    CHECK(MLvxCalib::GetImuIdToUse(idToSn, "SN2") == 5);
}

TEST_CASE("GetImuIdToUse: serial number not present in idToSn returns 0")
{
    std::unordered_map<int, std::string> idToSn{ { 0, "SN1" } };
    CHECK(MLvxCalib::GetImuIdToUse(idToSn, "SN_UNKNOWN") == 0);
}
