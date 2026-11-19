#pragma once
#include <algorithm>
#include <filesystem>
#include <string>
#include <system_error>

// Session files (Session::save() in core, session.mjs from lidar_odometry_step_1)
// store the paths of the files they reference relative to the session file's
// folder, so a session keeps working after its folder is moved or copied to
// another computer. In memory the paths stay absolute: writers convert with
// toSessionPath(), readers with resolveSessionPath(). Step 3 projects (.mjp)
// store their session files the same way and read them with resolveProjectPath().

// Path to write into session_file for `path`: relative to the session file's
// folder with '/' separators, or absolute when there is no relative path (e.g.
// another drive on Windows). An empty path stays empty.
inline std::string toSessionPath(const std::string& path, const std::string& session_file)
{
    namespace fs = std::filesystem;
    if (path.empty())
    {
        return path;
    }
    std::error_code ec_path;
    std::error_code ec_session;
    const fs::path absolute_path = fs::absolute(path, ec_path).lexically_normal();
    const fs::path session_dir = fs::absolute(session_file, ec_session).parent_path().lexically_normal();
    if (ec_path || ec_session)
    {
        return fs::path(path).generic_string();
    }
    const fs::path relative_path = absolute_path.lexically_relative(session_dir);
    return (relative_path.empty() ? absolute_path : relative_path).generic_string();
}

//! Absolute path for a path read from `file`: relative paths resolve against the
//! folder of `file`, absolute ones (older files) are used as they are.
inline std::string resolveStoredPath(const std::string& stored_path, const std::string& file)
{
    namespace fs = std::filesystem;
    // Files written on Windows use '\' separators
    std::string normalized = stored_path;
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    const fs::path path(normalized);

    std::error_code ec;
    const fs::path dir = fs::absolute(file, ec).parent_path();
    return (path.is_absolute() ? path : (dir / path).lexically_normal()).string();
}

// Absolute path for a path read from session_file, see resolveStoredPath(). If
// the result doesn't exist, the file of the same name next to the session file
// is used instead, so sessions moved together with their data keep loading.
inline std::string resolveSessionPath(const std::string& stored_path, const std::string& session_file)
{
    namespace fs = std::filesystem;
    const fs::path resolved = resolveStoredPath(stored_path, session_file);

    std::error_code ec;
    if (fs::exists(resolved, ec))
    {
        return resolved.string();
    }
    return (fs::absolute(session_file, ec).parent_path() / resolved.filename()).string();
}

//! Absolute path of a session file read from a step 3 project, see
//! resolveStoredPath(). Older projects kept paths given on the command line as
//! they were, relative to the current folder; such a path is used when the
//! project-relative one doesn't exist.
inline std::string resolveProjectPath(const std::string& stored_path, const std::string& project_file)
{
    namespace fs = std::filesystem;
    const std::string resolved = resolveStoredPath(stored_path, project_file);

    std::error_code ec;
    if (!fs::exists(resolved, ec) && fs::exists(stored_path, ec))
    {
        const fs::path absolute_path = fs::absolute(stored_path, ec);
        return ec ? stored_path : absolute_path.string();
    }
    return resolved;
}
