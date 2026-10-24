#include <doctest.h>

#include <HDMapping/SessionPaths.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;

namespace
{
    // Fresh folder under the system temp folder, removed again at the end of the test
    struct TempDir
    {
        fs::path path;

        TempDir()
        {
            const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
            path = fs::absolute(fs::temp_directory_path() / ("hdmapping_session_paths_" + std::to_string(stamp)));
            fs::create_directories(path);
        }

        ~TempDir()
        {
            std::error_code ec;
            fs::remove_all(path, ec);
        }
    };

    // Switches the current folder and restores it at the end of the test
    struct CurrentDir
    {
        fs::path previous = fs::current_path();

        explicit CurrentDir(const fs::path& dir)
        {
            fs::current_path(dir);
        }

        ~CurrentDir()
        {
            std::error_code ec;
            fs::current_path(previous, ec);
        }
    };

    void touch(const fs::path& file)
    {
        fs::create_directories(file.parent_path());
        std::ofstream(file) << "x";
    }

    bool samePath(const std::string& a, const fs::path& b)
    {
        return fs::path(a).lexically_normal() == b.lexically_normal();
    }
} // namespace

TEST_CASE("toSessionPath: paths become relative to the session file's folder")
{
    TempDir tmp;
    const std::string session = (tmp.path / "lio_result_0" / "session.mjs").string();

    CHECK(toSessionPath((tmp.path / "lio_result_0" / "scan_lio_0.laz").string(), session) == "scan_lio_0.laz");
    CHECK(toSessionPath((tmp.path / "lio_result_0" / "cache" / "0.cache").string(), session) == "cache/0.cache");
    CHECK(toSessionPath((tmp.path / "lio_result_1" / "scan_lio_0.laz").string(), session) == "../lio_result_1/scan_lio_0.laz");
    CHECK(toSessionPath((tmp.path / "lio_result_0").string(), session) == ".");
}

TEST_CASE("toSessionPath: an empty path stays empty")
{
    TempDir tmp;
    CHECK(toSessionPath("", (tmp.path / "session.mjs").string()).empty());
}

#ifdef _WIN32
TEST_CASE("toSessionPath: a path on another drive stays absolute")
{
    CHECK(toSessionPath("D:/data/scan_lio_0.laz", "C:/sessions/session.mjs") == "D:/data/scan_lio_0.laz");
}
#endif

TEST_CASE("resolveSessionPath: relative paths resolve against the session file's folder")
{
    TempDir tmp;
    const std::string session = (tmp.path / "lio_result_0" / "session.mjs").string();
    touch(tmp.path / "lio_result_0" / "scan_lio_0.laz");
    touch(tmp.path / "lio_result_1" / "scan_lio_0.laz");

    CHECK(samePath(resolveSessionPath("scan_lio_0.laz", session), tmp.path / "lio_result_0" / "scan_lio_0.laz"));
    CHECK(samePath(resolveSessionPath("../lio_result_1/scan_lio_0.laz", session), tmp.path / "lio_result_1" / "scan_lio_0.laz"));
    CHECK(samePath(resolveSessionPath("..\\lio_result_1\\scan_lio_0.laz", session), tmp.path / "lio_result_1" / "scan_lio_0.laz"));
    CHECK(fs::equivalent(resolveSessionPath(".", session), tmp.path / "lio_result_0"));
}

TEST_CASE("resolveSessionPath: the current folder doesn't matter")
{
    TempDir tmp;
    const std::string session = (tmp.path / "lio_result_0" / "session.mjs").string();
    touch(tmp.path / "lio_result_0" / "scan_lio_0.laz");
    touch(tmp.path / "elsewhere" / "scan_lio_0.laz");
    CurrentDir cwd(tmp.path / "elsewhere");

    CHECK(samePath(resolveSessionPath("scan_lio_0.laz", session), tmp.path / "lio_result_0" / "scan_lio_0.laz"));
}

TEST_CASE("resolveSessionPath: an absolute path from an older session is used while it exists")
{
    TempDir tmp;
    const fs::path data = tmp.path / "data" / "scan_lio_0.laz";
    touch(data);

    CHECK(samePath(resolveSessionPath(data.string(), (tmp.path / "session.mjs").string()), data));
}

TEST_CASE("resolveSessionPath: a missing path falls back to the same name next to the session file")
{
    TempDir tmp;
    const std::string session = (tmp.path / "moved" / "session.mjs").string();
    const fs::path next_to_session = tmp.path / "moved" / "scan_lio_0.laz";
    touch(next_to_session);

    // absolute paths from the computer the session was made on
    CHECK(samePath(resolveSessionPath("C:\\data\\lio_result_0\\scan_lio_0.laz", session), next_to_session));
    CHECK(samePath(resolveSessionPath((tmp.path / "gone" / "scan_lio_0.laz").string(), session), next_to_session));
}

TEST_CASE("Session paths survive moving the session folder")
{
    // A session file one level above its data: the old "same name next to the
    // session file" fallback can't find these files once the folder is moved.
    TempDir tmp;
    const fs::path before = tmp.path / "before";
    const fs::path after = tmp.path / "after";
    const std::string stored = toSessionPath((before / "lio_result_0" / "scan_lio_0.laz").string(), (before / "session.mjs").string());
    CHECK(stored == "lio_result_0/scan_lio_0.laz");

    touch(after / "lio_result_0" / "scan_lio_0.laz");
    CHECK(samePath(resolveSessionPath(stored, (after / "session.mjs").string()), after / "lio_result_0" / "scan_lio_0.laz"));
}
