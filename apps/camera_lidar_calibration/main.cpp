#include "App.h"
#include <CalibCore/CliArgs.h>
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <string>

using namespace calib;
namespace fs = std::filesystem;

static std::string ext(const std::string& path)
{
    auto pos = path.rfind('.');
    if (pos == std::string::npos)
        return "";
    std::string e = path.substr(pos + 1);
    std::transform(e.begin(), e.end(), e.begin(), ::tolower);
    return e;
}

static bool isImage(const std::string& e)
{
    return e == "jpg" || e == "jpeg" || e == "png" || e == "bmp";
}

// Load each path by file type (clouds, images, intrinsics, calibration).
static void preloadByExt(App& app, const std::string& p)
{
    std::string e = ext(p);
    if (isImage(e))
        app.preloadImage(p.c_str());
    else if (e == "laz" || e == "las")
        app.preloadCloud(p.c_str());
    else if (e == "yml" || e == "yaml")
        app.preloadIntrinsics(p.c_str());
    else if (e == "json")
        app.preloadCalibration(p.c_str());
}

int main(int argc, char* argv[])
{
    CliArgs args = parseArgs(argc, argv);
    const std::vector<std::string> usage = { cliopt::CAMERA_DIR, cliopt::LAZ, cliopt::CALIB };
    if (args.help)
    {
        printUsage("CalibrationApp", "Camera/LiDAR calibration tool", usage);
        return 0;
    }
    if (!args.valid)
    {
        std::fprintf(stderr, "%s\n\n", args.error.c_str());
        printUsage("CalibrationApp", "Camera/LiDAR calibration tool", usage, /*toStderr=*/true);
        return 1;
    }

    App app;

    // --laz: one or more point clouds.
    for (const auto& laz : args.getAll("laz"))
        app.preloadCloud(laz.c_str());

    // --calib: calibration json (intrinsic + extrinsic).
    if (args.has("calib"))
        app.preloadCalibration(args.get("calib").c_str());

    // --camera_dir: load the first image found in the directory.
    if (args.has("camera_dir"))
    {
        fs::path dir(args.get("camera_dir"));
        if (fs::is_directory(dir))
        {
            std::vector<fs::path> imgs;
            for (auto& e : fs::directory_iterator(dir))
                if (e.is_regular_file() && isImage(ext(e.path().filename().string())))
                    imgs.push_back(e.path());
            std::sort(imgs.begin(), imgs.end());
            if (!imgs.empty())
                app.preloadImage(imgs.front().string().c_str());
        }
        else if (fs::is_regular_file(dir))
        {
            app.preloadImage(dir.string().c_str());
        }
    }

    // Positional files keep working by extension (drag-and-drop / shell glob).
    for (const auto& p : args.positional)
        preloadByExt(app, p);

    app.run();
    return 0;
}