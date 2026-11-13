//! camera_lidar_depth_images: renders a session's lidar points into the camera
//! at regular intervals and writes, per selected frame, a depth image and an
//! intensity image the size of the camera image (or scaled by --scale).
//!
//! Same inputs as camera_lidar_trajectory_viewer: the session (.mjs), the
//! camera-lidar calibration JSON and the camera frames, whose timestamps must be
//! on the lidar clock. Every point within --window seconds of the frame is moved
//! into the lidar frame at the frame time (trajectory interpolated) and
//! projected with calib::projectPoint; the nearest point wins each pixel.

#include <CalibCore/Camera.h>
#include <CalibCore/CliArgs.h>
#include <CalibCore/PointCloud.h>
#include <CalibCore/Session.h>
#include <CalibCore/Trajectory.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace calib;

namespace
{
    constexpr const char* kAppName = "camera_lidar_depth_images";

    //! Depth images store millimetres in 16 bits, so this is the farthest depth they can hold.
    constexpr float kMaxEncodableDepthM = 65.534f;

    //! Intensity shown white in the intensity preview. Livox reflectivity is diffuse up to 150 of 255 and
    //! retroreflective above, so the grey scale spans the diffuse range.
    constexpr float kPreviewMaxIntensity = 150.f / 255.f;

    struct Options
    {
        fs::path mjs, calib, cameraDir, outDir;
        double everySec = 5.0;
        double windowSec = 10.0;
        float scale = 1.f;
        float minDepth = 0.3f;
        float maxDepth = kMaxEncodableDepthM;
        int pointRadius = 0;
        bool previewDepth = false;
        bool previewIntensity = false;
    };

    //! A lidar point in the session's world frame.
    struct WorldPoint
    {
        Eigen::Vector3f p;
        float intensity; //!< [0,1], as calib::PointCloud normalizes it
        int64_t ts_ns;
    };

    //! One scan_lio_N chunk and the time span of its trajectory.
    struct Chunk
    {
        fs::path laz;
        Eigen::Affine3f pose = Eigen::Affine3f::Identity();
        int64_t first = 0, last = 0;
    };

    struct Frame
    {
        int64_t ts;
        fs::path image;
        Eigen::Affine3f T_wl; //!< lidar pose at the frame time
    };

    bool parseNumber(const CliArgs& a, const char* key, double& out)
    {
        if (!a.has(key))
            return true;
        try
        {
            out = std::stod(a.get(key));
            return true;
        } catch (const std::exception&)
        {
            std::fprintf(stderr, "--%s: not a number: '%s'\n", key, a.get(key).c_str());
            return false;
        }
    }

    void usage(bool toStderr)
    {
        printUsage(
            kAppName,
            "renders lidar depth and intensity images at camera frames, one frame every --every seconds",
            { cliopt::MJS,
              cliopt::CALIB,
              cliopt::CAMERA_DIR,
              "  --out <dir>                  output directory (default: <camera_dir>_lidar_images)",
              "  --every <s>                  seconds between rendered frames (default 5)",
              "  --window <s>                 use points within +/- this many seconds of the frame (default 10)",
              "  --scale <f>                  output size relative to the camera image (default 1)",
              "  --min_depth <m>              drop points closer than this (default 0.3)",
              "  --max_depth <m>              drop points farther than this (default and maximum 65.534)",
              "  --point_radius <px>          draw each point as a disc of this radius (default 0 = one pixel)",
              "  --preview [depth] [intensity]  also write images for viewing: depth over the camera image\n"
              "                               (preview/<frame>.jpg, the default) and/or the intensity image\n"
              "                               with enlarged points (preview_intensity/<frame>.jpg)" },
            toStderr);
    }

    bool parseOptions(int argc, char* argv[], Options& o)
    {
        const CliArgs a = parseArgs(argc, argv);
        if (a.help)
        {
            usage(false);
            std::exit(0);
        }
        if (!a.valid || !a.has("mjs") || !a.has("calib") || !a.has("camera_dir"))
        {
            if (!a.valid)
                std::fprintf(stderr, "%s\n", a.error.c_str());
            usage(true);
            return false;
        }
        o.mjs = a.get("mjs");
        o.calib = a.get("calib");
        o.cameraDir = fs::path(a.get("camera_dir"));
        if (!o.cameraDir.has_filename())
            o.cameraDir = o.cameraDir.parent_path(); // trailing separator
        o.outDir = a.has("out") ? fs::path(a.get("out")) : o.cameraDir.parent_path() / (o.cameraDir.filename().string() + "_lidar_images");
        double scale = o.scale, minDepth = o.minDepth, maxDepth = o.maxDepth, radius = o.pointRadius;
        if (!parseNumber(a, "every", o.everySec) || !parseNumber(a, "window", o.windowSec) || !parseNumber(a, "scale", scale) ||
            !parseNumber(a, "min_depth", minDepth) || !parseNumber(a, "max_depth", maxDepth) || !parseNumber(a, "point_radius", radius))
            return false;
        o.scale = float(scale);
        o.minDepth = float(minDepth);
        o.maxDepth = std::min(float(maxDepth), kMaxEncodableDepthM);
        o.pointRadius = std::max(0, int(std::lround(radius)));
        for (const std::string& v : a.getAll("preview"))
        {
            if (v.empty() || v == "depth")
                o.previewDepth = true;
            else if (v == "intensity")
                o.previewIntensity = true;
            else
            {
                std::fprintf(stderr, "--preview: expected depth or intensity, got '%s'\n", v.c_str());
                return false;
            }
        }
        if (o.everySec < 0.0 || o.windowSec < 0.0 || o.scale <= 0.f || o.minDepth >= o.maxDepth)
        {
            std::fprintf(stderr, "Invalid --every, --window, --scale or depth range\n");
            return false;
        }
        return true;
    }

    //! Every chunk of the session with its pose and the time span of its trajectory CSV.
    std::vector<Chunk> listChunks(const SessionFiles& sf)
    {
        std::vector<Chunk> chunks;
        for (const auto& lp : sf.lazPaths)
        {
            Trajectory t;
            t.loadCSV(chunkTrajectoryCsv(lp).string());
            t.sort();
            if (t.empty())
            {
                std::fprintf(stderr, "No trajectory for %s, skipped\n", lp.string().c_str());
                continue;
            }
            Chunk c;
            c.laz = lp;
            if (auto it = sf.poses.find(lp.stem().string()); it != sf.poses.end())
                c.pose = it->second;
            c.first = t.poses.front().ts_ns;
            c.last = t.poses.back().ts_ns;
            chunks.push_back(std::move(c));
        }
        return chunks;
    }

    std::vector<WorldPoint> loadChunk(const Chunk& c)
    {
        std::vector<WorldPoint> out;
        PointCloud pc;
        if (!pc.load(c.laz.string()))
        {
            std::fprintf(stderr, "Cannot read %s\n", c.laz.string().c_str());
            return out;
        }
        out.reserve(pc.points.size());
        for (const auto& pt : pc.points)
            out.push_back({ c.pose * Eigen::Vector3f(pt.x, pt.y, pt.z), pt.intensity, pt.ts_ns });
        return out;
    }

    //! Depth (metres, 0 = no point) and intensity ([0,1]) of the nearest point per pixel.
    struct Rendering
    {
        cv::Mat depth; //!< CV_32F
        cv::Mat intensity; //!< CV_32F
        size_t projected = 0;
    };

    Rendering render(
        const std::vector<const std::vector<WorldPoint>*>& clouds,
        const Frame& f,
        const Options& o,
        const Intrinsics& Ks,
        const Eigen::Matrix3f& R_wc,
        const Eigen::Vector3f& C)
    {
        Rendering r;
        r.depth = cv::Mat(Ks.height, Ks.width, CV_32F, cv::Scalar(std::numeric_limits<float>::infinity()));
        r.intensity = cv::Mat(Ks.height, Ks.width, CV_32F, cv::Scalar(0.f));
        const Eigen::Affine3f T_lw = f.T_wl.inverse();
        const int64_t windowNs = std::llround(o.windowSec * 1e9);
        const int rad = o.pointRadius;

        for (const auto* cloud : clouds)
        {
            for (const auto& wp : *cloud)
            {
                if (std::llabs(wp.ts_ns - f.ts) > windowNs)
                    continue;
                const Eigen::Vector3f pl = T_lw * wp.p;
                float u, v, depth;
                if (!projectPoint(pl.x(), pl.y(), pl.z(), Ks, R_wc, C, u, v, depth))
                    continue;
                if (depth < o.minDepth || depth > o.maxDepth)
                    continue;
                // Bounded before the int conversion: near-grazing points can land far outside.
                if (!(u > -0.5f - rad && v > -0.5f - rad && u < Ks.width - 0.5f + rad && v < Ks.height - 0.5f + rad))
                    continue;
                const int iu = int(std::lround(u));
                const int iv = int(std::lround(v));
                ++r.projected;
                for (int dy = -rad; dy <= rad; ++dy)
                {
                    const int y = iv + dy;
                    if (y < 0 || y >= Ks.height)
                        continue;
                    for (int dx = -rad; dx <= rad; ++dx)
                    {
                        const int x = iu + dx;
                        if (x < 0 || x >= Ks.width || dx * dx + dy * dy > rad * rad)
                            continue;
                        float& zb = r.depth.at<float>(y, x);
                        if (depth < zb)
                        {
                            zb = depth;
                            r.intensity.at<float>(y, x) = wp.intensity;
                        }
                    }
                }
            }
        }
        r.depth.setTo(0.f, r.depth == std::numeric_limits<float>::infinity());
        return r;
    }

    //! The camera image resized to the rendering, or an empty Mat when it can't be read.
    cv::Mat readPhoto(const fs::path& image, cv::Size size)
    {
        cv::Mat img = cv::imread(image.string(), cv::IMREAD_COLOR);
        if (!img.empty() && img.size() != size)
            cv::resize(img, img, size, 0, 0, cv::INTER_AREA);
        return img;
    }

    //! `values` scaled so that maxValue maps to 255, clamped.
    cv::Mat toByte(const cv::Mat& values, float maxValue)
    {
        cv::Mat value8;
        values.convertTo(value8, CV_8U, 255.0 / maxValue);
        return value8;
    }

    //! `canvas` with `colors` copied into every pixel that has a point. Single pixels vanish in a downscaled
    //! view of a large image, so the points are grown for display.
    cv::Mat paintPoints(cv::Mat canvas, const cv::Mat& colors, const cv::Mat& hasPoint)
    {
        const int grow = std::max(1, colors.cols / 1000);
        const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * grow + 1, 2 * grow + 1));
        cv::Mat grown, valid;
        cv::dilate(colors, grown, kernel);
        cv::dilate(hasPoint, valid, kernel);
        grown.copyTo(canvas, valid);
        return canvas;
    }
} // namespace

int main(int argc, char* argv[])
{
    Options o;
    if (!parseOptions(argc, argv, o))
        return 1;

    std::string err;
    const fs::path mjs = resolveMjs(o.mjs, err);
    SessionFiles sf;
    if (mjs.empty() || !readSessionFiles(mjs, sf, err))
    {
        std::fprintf(stderr, "%s\n", err.c_str());
        return 1;
    }
    if (!sf.hasPoses)
        std::fprintf(stderr, "Warning: no poses file in %s, chunks stay in their local frames\n", mjs.string().c_str());
    Trajectory traj;
    loadSessionTrajectory(sf, traj);
    if (traj.empty())
    {
        std::fprintf(stderr, "No trajectory in %s\n", mjs.string().c_str());
        return 1;
    }
    const auto trajMap = buildTrajMap(traj);
    const std::vector<Chunk> chunks = listChunks(sf);

    Intrinsics K;
    Eigen::Matrix3f R_wc = Eigen::Matrix3f::Identity();
    Eigen::Vector3f C = Eigen::Vector3f::Zero();
    Roi roi;
    if (!loadCalibrationJson(o.calib.string(), K, R_wc, C, roi, err))
    {
        std::fprintf(stderr, "%s\n", err.c_str());
        return 1;
    }

    // Frames on the trajectory, one every o.everySec.
    std::map<int64_t, fs::path> images;
    if (!fs::is_directory(o.cameraDir))
    {
        std::fprintf(stderr, "No camera directory %s\n", o.cameraDir.string().c_str());
        return 1;
    }
    for (const auto& e : fs::directory_iterator(o.cameraDir))
        if (const int64_t ts = parseImageTsNs(e.path()); ts >= 0)
            images[ts] = e.path();
    std::vector<Frame> frames;
    int64_t next = std::numeric_limits<int64_t>::min();
    const int64_t everyNs = std::llround(o.everySec * 1e9);
    for (const auto& [ts, path] : images)
    {
        Eigen::Affine3f T;
        if (ts < next || !interpPose(trajMap, ts, T))
            continue;
        frames.push_back({ ts, path, T });
        next = ts + everyNs;
    }
    if (frames.empty())
    {
        std::fprintf(stderr, "None of the %zu images in %s falls on the trajectory\n", images.size(), o.cameraDir.string().c_str());
        return 1;
    }

    const cv::Mat probe = cv::imread(frames.front().image.string(), cv::IMREAD_COLOR);
    if (probe.empty())
    {
        std::fprintf(stderr, "Cannot read %s\n", frames.front().image.string().c_str());
        return 1;
    }
    K.width = probe.cols;
    K.height = probe.rows;
    const Intrinsics Ks = scaleIntrinsics(K, o.scale);

    std::error_code ec;
    fs::create_directories(o.outDir / "depth", ec);
    fs::create_directories(o.outDir / "intensity", ec);
    if (o.previewDepth)
        fs::create_directories(o.outDir / "preview", ec);
    if (o.previewIntensity)
        fs::create_directories(o.outDir / "preview_intensity", ec);
    std::ofstream index(o.outDir / "frames.csv");
    if (!index)
    {
        std::fprintf(stderr, "Cannot write to %s\n", o.outDir.string().c_str());
        return 1;
    }
    index << "image,timestamp_ns,points_projected,pixels_with_depth\n";

    std::printf(
        "%zu images, %zu frames to render (every %.1f s), %zu chunks, %dx%d px -> %s\n",
        images.size(),
        frames.size(),
        o.everySec,
        chunks.size(),
        Ks.width,
        Ks.height,
        o.outDir.string().c_str());

    // Frames are in time order, so the chunks a frame needs are kept until a later frame no longer does.
    std::map<size_t, std::vector<WorldPoint>> loaded;
    const int64_t windowNs = std::llround(o.windowSec * 1e9);
    for (size_t fi = 0; fi < frames.size(); ++fi)
    {
        const Frame& f = frames[fi];
        std::vector<const std::vector<WorldPoint>*> clouds;
        for (size_t ci = 0; ci < chunks.size(); ++ci)
        {
            // A chunk's points can sit a little past its last trajectory pose; one second covers that.
            const int64_t margin = 1'000'000'000;
            const bool needed = chunks[ci].first - margin <= f.ts + windowNs && chunks[ci].last + margin >= f.ts - windowNs;
            if (!needed)
            {
                loaded.erase(ci);
                continue;
            }
            auto it = loaded.find(ci);
            if (it == loaded.end())
                it = loaded.emplace(ci, loadChunk(chunks[ci])).first;
            clouds.push_back(&it->second);
        }

        const Rendering r = render(clouds, f, o, Ks, R_wc, C);
        cv::Mat depthMm, intensity8;
        r.depth.convertTo(depthMm, CV_16U, 1000.0);
        r.intensity.convertTo(intensity8, CV_8U, 255.0);
        const int withDepth = cv::countNonZero(depthMm);
        const std::string stem = f.image.stem().string();
        bool ok = cv::imwrite((o.outDir / "depth" / (stem + ".png")).string(), depthMm);
        ok &= cv::imwrite((o.outDir / "intensity" / (stem + ".png")).string(), intensity8);
        const cv::Mat hasPoint = r.depth > 0.f;
        if (o.previewDepth)
        {
            // Turbo, near = blue, over the camera image.
            cv::Mat photo = readPhoto(f.image, r.depth.size());
            cv::Mat colors;
            cv::applyColorMap(toByte(r.depth, o.maxDepth), colors, cv::COLORMAP_TURBO);
            ok &= !photo.empty() &&
                cv::imwrite((o.outDir / "preview" / (stem + ".jpg")).string(), paintPoints(std::move(photo), colors, hasPoint));
        }
        if (o.previewIntensity)
        {
            const cv::Mat black = cv::Mat::zeros(r.intensity.size(), CV_8U);
            ok &= cv::imwrite(
                (o.outDir / "preview_intensity" / (stem + ".jpg")).string(),
                paintPoints(black, toByte(r.intensity, kPreviewMaxIntensity), hasPoint));
        }
        if (!ok)
            std::fprintf(stderr, "Failed to write the images of %s\n", stem.c_str());
        index << f.image.filename().string() << ',' << f.ts << ',' << r.projected << ',' << withDepth << '\n';
        std::printf(
            "[%zu/%zu] %s  %zu points, %.2f%% pixels with depth\n",
            fi + 1,
            frames.size(),
            stem.c_str(),
            r.projected,
            100.0 * withDepth / (double(Ks.width) * Ks.height));
        std::fflush(stdout);
    }
    return 0;
}
