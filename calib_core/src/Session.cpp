#include <CalibCore/Session.h>

#include <HDMapping/SessionPaths.h>

#include <fstream>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;

namespace calib {

std::map<std::string, Eigen::Affine3f> parseMRP(const fs::path& mrpPath)
{
    std::map<std::string, Eigen::Affine3f> result;
    std::ifstream f(mrpPath);
    if (!f)
        return result;
    int n;
    f >> n;
    for (int i = 0; i < n; i++)
    {
        std::string name;
        f >> name;
        auto dot = name.rfind('.');
        std::string key = (dot != std::string::npos) ? name.substr(0, dot) : name;
        float raw[16];
        for (int r = 0; r < 16; r++)
            f >> raw[r];
        if (!f)
            continue;
        Eigen::Matrix4f M4;
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 4; c++)
                M4(r, c) = raw[r * 4 + c];
        result[key] = Eigen::Affine3f(M4);
    }
    return result;
}

fs::path resolveMjs(const fs::path& p, std::string& err)
{
    if (!fs::is_directory(p))
    {
        if (fs::is_regular_file(p))
            return p;
        err = "Session file not found: " + p.string();
        return {};
    }
    if (fs::is_regular_file(p / "session.mjs"))
        return p / "session.mjs";
    std::vector<fs::path> found;
    for (const auto& e : fs::directory_iterator(p))
        if (e.path().extension() == ".mjs")
            found.push_back(e.path());
    if (found.size() == 1)
        return found.front();
    err = found.empty() ? "No .mjs session file in " + p.string()
                        : std::to_string(found.size()) + " .mjs files in " + p.string() + " -- pick one";
    return {};
}

bool readSessionFiles(const fs::path& mjs, SessionFiles& out, std::string& err)
{
    out = SessionFiles{};
    out.mjs = mjs;
    auto relocate = [&](const std::string& path) -> fs::path
    {
        return path.empty() ? fs::path() : fs::path(resolveSessionPath(path, mjs.string()));
    };
    try
    {
        std::ifstream f(mjs);
        if (!f)
        {
            err = "Cannot open " + mjs.string();
            return false;
        }
        nlohmann::json data = nlohmann::json::parse(f);
        const auto& settings = data["Session Settings"];
        out.poses = parseMRP(relocate(settings.value("poses_file_name", "")));
        if (out.poses.empty())
            out.poses = parseMRP(relocate(settings.value("initial_poses_file_name", "")));
        out.hasPoses = !out.poses.empty();
        for (const auto& fn : data["laz_file_names"])
            out.lazPaths.push_back(relocate(fn.value("file_name", "")));
    } catch (const std::exception& e)
    {
        err = "Bad session file " + mjs.string() + ": " + e.what();
        return false;
    }
    return true;
}

fs::path chunkTrajectoryCsv(const fs::path& lazPath)
{
    std::string stem = lazPath.stem().string();
    return lazPath.parent_path() / ("trajectory_lio_" + stem.substr(stem.rfind('_') + 1) + ".csv");
}

void loadSessionTrajectory(const SessionFiles& sf, Trajectory& traj)
{
    for (const auto& lp : sf.lazPaths)
    {
        fs::path cp = chunkTrajectoryCsv(lp);
        if (!fs::exists(cp))
            continue;
        auto it = sf.poses.find(lp.stem().string());
        traj.loadCSV(cp.string(), it != sf.poses.end() ? &it->second : nullptr);
    }
    traj.sort();
}

}  // namespace calib
