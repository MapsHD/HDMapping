// session_intensity_panoramas: renders a session's lidar intensity as 360-degree equirectangular panoramas
// seen from the lidar, one every --every seconds along the trajectory.
//
// Each panorama uses the points recorded within --window seconds of its time, moved into the lidar frame at
// that time (pose interpolated on the session trajectory); the nearest point wins each pixel and small gaps
// between points are filled. Files are named intensity_<lidar timestamp ns>.jpg, so the output folder can be
// given to the BoW loop closure (step 2, bow_loop_closure_detect) as a camera folder.
#include <Core/session.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{
    struct Options
    {
        std::string session_file;
        fs::path out_dir;
        double every_s = 1.0;
        double window_s = 1.0;
        double width_px = 1920;
        double elevation_min_deg = -45.0;
        double elevation_max_deg = 60.0;
        double min_range_m = 0.5;
        double max_range_m = 100.0;
        double max_intensity = 0.0; //!< intensity shown white; 0 = the 99th percentile of the first chunk
        double fill_px = 1.5;
        bool level = false;
    };

    void print_usage(const char* app)
    {
        std::cout << "USAGE: " << app << " session_file(.mjs/.json) [output_dir] [options]\n"
                  << "  Renders the session's lidar intensity as 360-degree panoramas seen from the lidar, one every --every\n"
                  << "  seconds, written as <output_dir>/intensity_<lidar timestamp ns>.jpg plus frames.csv. The folder\n"
                  << "  works as a camera folder for the BoW loop closure (step 2, bow_loop_closure_detect).\n"
                  << "  output_dir                default: <session folder>/intensity_panoramas\n"
                  << "  --every <s>               seconds between panoramas (default 1)\n"
                  << "  --window <s>              use the points recorded within +/- this many seconds (default 1)\n"
                  << "  --width <px>              panorama width; the height follows from the elevation range (default 1920)\n"
                  << "  --elevation_min <deg>     lowest elevation shown (default -45)\n"
                  << "  --elevation_max <deg>     highest elevation shown (default 60)\n"
                  << "  --min_range <m>           drop points closer to the lidar than this (default 0.5)\n"
                  << "  --max_range <m>           drop points farther than this (default 100)\n"
                  << "  --max_intensity <value>   intensity shown white (default: 99th percentile of the first chunk)\n"
                  << "  --fill <px>               fill gaps between points, Gaussian sigma in pixels (default 1.5, 0 = off)\n"
                  << "  --level                   keep the lidar heading but level the panorama (z up), removing roll and pitch\n";
    }

    bool parse_options(int argc, char* argv[], Options& o)
    {
        const std::map<std::string, double*> numbers{
            { "every", &o.every_s },
            { "window", &o.window_s },
            { "width", &o.width_px },
            { "elevation_min", &o.elevation_min_deg },
            { "elevation_max", &o.elevation_max_deg },
            { "min_range", &o.min_range_m },
            { "max_range", &o.max_range_m },
            { "max_intensity", &o.max_intensity },
            { "fill", &o.fill_px },
        };
        std::vector<std::string> positional;
        for (int i = 1; i < argc; ++i)
        {
            const std::string arg = argv[i];
            if (arg == "-h" || arg == "--help")
            {
                print_usage(argv[0]);
                std::exit(0);
            }
            if (arg == "--level")
            {
                o.level = true;
                continue;
            }
            if (arg.rfind("--", 0) != 0)
            {
                positional.push_back(arg);
                continue;
            }
            const auto it = numbers.find(arg.substr(2));
            if (it == numbers.end())
            {
                spdlog::error("Unknown option '{}'", arg);
                return false;
            }
            if (i + 1 >= argc)
            {
                spdlog::error("{} needs a value", arg);
                return false;
            }
            const std::string value = argv[++i];
            try
            {
                size_t used = 0;
                *it->second = std::stod(value, &used);
                if (used != value.size())
                    throw std::invalid_argument(value);
            } catch (const std::exception&)
            {
                spdlog::error("{}: not a number: '{}'", arg, value);
                return false;
            }
        }
        if (positional.empty() || positional.size() > 2)
        {
            print_usage(argv[0]);
            return false;
        }
        o.session_file = positional[0];
        o.out_dir = positional.size() == 2 ? fs::path(positional[1]) : fs::path(o.session_file).parent_path() / "intensity_panoramas";
        if (!(o.every_s > 0.0 && o.window_s >= 0.0 && o.width_px >= 16.0 && o.elevation_min_deg >= -90.0 &&
              o.elevation_min_deg < o.elevation_max_deg && o.elevation_max_deg <= 90.0 && o.min_range_m >= 0.0 &&
              o.min_range_m < o.max_range_m && o.max_intensity >= 0.0 && o.fill_px >= 0.0))
        {
            spdlog::error("Invalid option value, see --help");
            return false;
        }
        return true;
    }

    struct TrajectoryPose
    {
        int64_t ts_ns;
        Eigen::Vector3d t;
        Eigen::Quaterniond q;
    };

    //! Maps a chunk's points, as stored in its LAZ file, to the world. PointClouds::load subtracts the session
    //! offset from the points before m_pose applies; the offset goes into the pose here instead.
    Eigen::Affine3d chunk_pose(const PointCloud& pc, const Eigen::Vector3d& offset)
    {
        return pc.m_pose * Eigen::Translation3d(-offset);
    }

    //! World lidar poses of every chunk's trajectory node, sorted by time.
    std::vector<TrajectoryPose> session_trajectory(const PointClouds& container)
    {
        std::vector<TrajectoryPose> out;
        for (const auto& pc : container.point_clouds)
        {
            const Eigen::Affine3d T_wc = chunk_pose(pc, container.offset);
            for (const auto& node : pc.local_trajectory)
            {
                const Eigen::Affine3d T_wl = T_wc * node.m_pose;
                out.push_back({ std::llround(node.timestamps.first), T_wl.translation(), Eigen::Quaterniond(T_wl.linear()).normalized() });
            }
        }
        std::stable_sort(
            out.begin(),
            out.end(),
            [](const TrajectoryPose& a, const TrajectoryPose& b)
            {
                return a.ts_ns < b.ts_ns;
            });
        // Neighbouring chunks can share a node time; keep the first.
        out.erase(
            std::unique(
                out.begin(),
                out.end(),
                [](const TrajectoryPose& a, const TrajectoryPose& b)
                {
                    return a.ts_ns == b.ts_ns;
                }),
            out.end());
        return out;
    }

    bool interpolate_pose(const std::vector<TrajectoryPose>& trajectory, int64_t ts_ns, Eigen::Affine3d& T_wl)
    {
        const auto it = std::lower_bound(
            trajectory.begin(),
            trajectory.end(),
            ts_ns,
            [](const TrajectoryPose& p, int64_t t)
            {
                return p.ts_ns < t;
            });
        if (it == trajectory.end() || (it == trajectory.begin() && it->ts_ns != ts_ns))
            return false;
        if (it->ts_ns == ts_ns)
        {
            T_wl = Eigen::Translation3d(it->t) * it->q;
            return true;
        }
        const TrajectoryPose& a = *(it - 1);
        const TrajectoryPose& b = *it;
        const double s = double(ts_ns - a.ts_ns) / double(b.ts_ns - a.ts_ns);
        T_wl = Eigen::Translation3d(a.t + s * (b.t - a.t)) * a.q.slerp(s, b.q);
        return true;
    }

    //! The lidar pose with roll and pitch removed: same position and heading, z along the world z.
    Eigen::Affine3d leveled(const Eigen::Affine3d& T_wl)
    {
        const Eigen::Vector3d x = T_wl.linear().col(0);
        const double heading = std::atan2(x.y(), x.x());
        return Eigen::Translation3d(T_wl.translation()) * Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ());
    }

    //! A chunk's points as stored in its LAZ file (chunk frame), without the points that have no timestamp.
    struct ChunkPoints
    {
        std::vector<Eigen::Vector3f> p;
        std::vector<float> intensity;
        std::vector<int64_t> ts_ns;
    };

    ChunkPoints load_chunk(const PointCloud& pc)
    {
        ChunkPoints out;
        PointCloud loaded; // a fresh cloud, so the session's chunk keeps its pose and trajectory
        if (!loaded.load_pc(pc.file_name, true))
        {
            spdlog::error("Cannot read '{}'", pc.file_name);
            return out;
        }
        out.p.reserve(loaded.points_local.size());
        out.intensity.reserve(loaded.points_local.size());
        out.ts_ns.reserve(loaded.points_local.size());
        for (size_t i = 0; i < loaded.points_local.size(); ++i)
        {
            if (i >= loaded.timestamps.size() || loaded.timestamps[i] == 0.0)
                continue;
            out.p.push_back(loaded.points_local[i].cast<float>());
            out.intensity.push_back(i < loaded.intensities.size() ? float(loaded.intensities[i]) : 0.f);
            out.ts_ns.push_back(std::llround(loaded.timestamps[i]));
        }
        return out;
    }

    double percentile(std::vector<float> values, double q)
    {
        if (values.empty())
            return 0.0;
        const size_t k = std::min(values.size() - 1, size_t(q * double(values.size())));
        std::nth_element(values.begin(), values.begin() + k, values.end());
        return values[k];
    }

    struct Panorama
    {
        cv::Mat intensity; //!< CV_32F, intensity of the nearest point per pixel
        cv::Mat has_point; //!< CV_8U, 255 where a point landed
        size_t points = 0;
    };

    Panorama render(
        const std::vector<std::pair<const ChunkPoints*, Eigen::Affine3d>>& chunks, // points and their chunk -> world pose
        const Eigen::Affine3d& T_wl,
        int64_t ts_ns,
        const Options& o,
        cv::Size size)
    {
        cv::Mat range(size, CV_32F, cv::Scalar(std::numeric_limits<float>::infinity()));
        Panorama out;
        out.intensity = cv::Mat(size, CV_32F, cv::Scalar(0.f));
        const int64_t window_ns = std::llround(o.window_s * 1e9);
        const float elevation_max = float(o.elevation_max_deg * M_PI / 180.0);
        const float px_per_rad_u = float(size.width / (2.0 * M_PI));
        const float px_per_rad_v = float(size.height / ((o.elevation_max_deg - o.elevation_min_deg) * M_PI / 180.0));
        const Eigen::Affine3d T_lw = T_wl.inverse();
        for (const auto& [points, T_wc] : chunks)
        {
            const Eigen::Affine3f T_lc = (T_lw * T_wc).cast<float>();
            for (size_t i = 0; i < points->p.size(); ++i)
            {
                if (std::llabs(points->ts_ns[i] - ts_ns) > window_ns)
                    continue;
                const Eigen::Vector3f q = T_lc * points->p[i];
                const float r = q.norm();
                if (r < o.min_range_m || r > o.max_range_m)
                    continue;
                // Azimuth 0 (lidar x) in the middle, +y to its left, as seen from the lidar.
                const float u = (float(M_PI) - std::atan2(q.y(), q.x())) * px_per_rad_u;
                const float v = (elevation_max - std::asin(q.z() / r)) * px_per_rad_v;
                if (!(v >= 0.f && v < size.height))
                    continue;
                const int col = std::min(int(u), size.width - 1);
                const int row = int(v);
                ++out.points;
                float& nearest = range.at<float>(row, col);
                if (r < nearest)
                {
                    nearest = r;
                    out.intensity.at<float>(row, col) = points->intensity[i];
                }
            }
        }
        out.has_point = range < std::numeric_limits<float>::infinity();
        return out;
    }

    //! Gaussian blur that wraps around horizontally, where the panorama's left and right edges meet, and sees
    //! zeros above and below it. OpenCV's filters reject BORDER_WRAP, so the wrapped columns are padded in.
    cv::Mat blur_wrapped(const cv::Mat& src, double sigma)
    {
        const int pad = std::min(src.cols, int(std::ceil(4.0 * sigma)) + 1); // the kernel radius OpenCV picks for float
        cv::Mat padded, blurred;
        cv::copyMakeBorder(src, padded, 0, 0, pad, pad, cv::BORDER_WRAP);
        cv::GaussianBlur(padded, blurred, cv::Size(), sigma, 0, cv::BORDER_CONSTANT);
        return blurred(cv::Rect(pad, 0, src.cols, src.rows)).clone();
    }

    //! 8-bit image: the intensity scaled so that max_intensity is white, gaps up to about two sigma from the
    //! nearest point filled with the Gaussian-weighted mean of the points around them, black elsewhere.
    cv::Mat to_image(const Panorama& p, double max_intensity, double fill_px)
    {
        cv::Mat value = p.intensity.clone();
        if (fill_px > 0.0)
        {
            cv::Mat weight, mean;
            p.has_point.convertTo(weight, CV_32F, 1.0 / 255.0);
            const cv::Mat weighted_sum = blur_wrapped(p.intensity.mul(weight), fill_px);
            weight = blur_wrapped(weight, fill_px);
            cv::divide(weighted_sum, weight, mean);
            const cv::Mat gap = (p.has_point == 0) & (weight > 0.1f);
            mean.copyTo(value, gap);
        }
        cv::Mat image;
        value.convertTo(image, CV_8U, 255.0 / max_intensity);
        return image;
    }
} // namespace

int main(int argc, char* argv[])
{
    Options o;
    if (!parse_options(argc, argv, o))
        return 1;

    Session session;
    session.load_cache_mode = true; // poses and trajectories only; chunks are read when a panorama needs them
    if (!session.load(o.session_file, false, 0.0, 0.0, 0.0, false))
    {
        spdlog::error("Failed to load session '{}'", o.session_file);
        return 2;
    }
    const PointClouds& container = session.point_clouds_container;
    const std::vector<TrajectoryPose> trajectory = session_trajectory(container);
    if (trajectory.size() < 2)
    {
        spdlog::error("No trajectory in '{}' (trajectory_lio_*.csv next to the chunks)", o.session_file);
        return 2;
    }

    // Time span of each chunk's trajectory; its points can sit a little past the last node, one second covers that.
    constexpr int64_t chunk_margin_ns = 1'000'000'000;
    std::vector<std::pair<int64_t, int64_t>> chunk_spans;
    for (const auto& pc : container.point_clouds)
    {
        if (pc.local_trajectory.empty())
        {
            spdlog::warn("No trajectory for '{}', its points are not used", pc.file_name);
            chunk_spans.emplace_back(std::numeric_limits<int64_t>::max(), std::numeric_limits<int64_t>::min());
            continue;
        }
        chunk_spans.emplace_back(
            std::llround(pc.local_trajectory.front().timestamps.first) - chunk_margin_ns,
            std::llround(pc.local_trajectory.back().timestamps.first) + chunk_margin_ns);
    }

    const int width = int(std::lround(o.width_px));
    const int height = std::max(1, int(std::lround(width * (o.elevation_max_deg - o.elevation_min_deg) / 360.0)));
    std::error_code ec;
    fs::create_directories(o.out_dir, ec);
    std::ofstream index(o.out_dir / "frames.csv");
    if (!index)
    {
        spdlog::error("Cannot write to '{}'", o.out_dir.string());
        return 3;
    }
    index << "image,timestamp_ns,points,coverage_percent,x,y,z,qx,qy,qz,qw\n";

    const int64_t every_ns = std::llround(o.every_s * 1e9);
    const int64_t window_ns = std::llround(o.window_s * 1e9);
    const int64_t first_ns = trajectory.front().ts_ns;
    const int64_t last_ns = trajectory.back().ts_ns;
    const size_t frame_count = size_t((last_ns - first_ns) / every_ns) + 1;
    spdlog::info(
        "{} chunks, {:.1f} s of trajectory: {} panoramas of {}x{} px -> '{}'",
        container.point_clouds.size(),
        (last_ns - first_ns) * 1e-9,
        frame_count,
        width,
        height,
        o.out_dir.string());

    // Panoramas are in time order, so a chunk is kept until a later panorama no longer needs it.
    std::map<size_t, ChunkPoints> loaded;
    double max_intensity = o.max_intensity;
    size_t written = 0;
    for (size_t fi = 0; fi < frame_count; ++fi)
    {
        const int64_t ts = first_ns + int64_t(fi) * every_ns;
        Eigen::Affine3d T_wl;
        if (!interpolate_pose(trajectory, ts, T_wl))
            continue;
        if (o.level)
            T_wl = leveled(T_wl);

        std::vector<std::pair<const ChunkPoints*, Eigen::Affine3d>> chunks;
        for (size_t ci = 0; ci < container.point_clouds.size(); ++ci)
        {
            if (chunk_spans[ci].first > ts + window_ns || chunk_spans[ci].second < ts - window_ns)
            {
                loaded.erase(ci);
                continue;
            }
            auto it = loaded.find(ci);
            if (it == loaded.end())
                it = loaded.emplace(ci, load_chunk(container.point_clouds[ci])).first;
            if (max_intensity <= 0.0 && !it->second.intensity.empty())
            {
                max_intensity = std::max(1.0, percentile(it->second.intensity, 0.99));
                spdlog::info("Intensity {:.0f} (99th percentile of '{}') shown white", max_intensity, container.point_clouds[ci].file_name);
            }
            chunks.emplace_back(&it->second, chunk_pose(container.point_clouds[ci], container.offset));
        }
        if (max_intensity <= 0.0)
            continue; // no points so far

        const Panorama p = render(chunks, T_wl, ts, o, cv::Size(width, height));
        const std::string name = "intensity_" + std::to_string(ts) + ".jpg";
        if (!cv::imwrite((o.out_dir / name).string(), to_image(p, max_intensity, o.fill_px), { cv::IMWRITE_JPEG_QUALITY, 95 }))
        {
            spdlog::error("Cannot write '{}'", (o.out_dir / name).string());
            return 3;
        }
        ++written;
        const double coverage = 100.0 * cv::countNonZero(p.has_point) / (double(width) * height);
        const Eigen::Quaterniond q(T_wl.linear());
        const Eigen::Vector3d& t = T_wl.translation();
        index << name << ',' << ts << ',' << p.points << ',' << coverage << ',' << t.x() << ',' << t.y() << ',' << t.z() << ',' << q.x()
              << ',' << q.y() << ',' << q.z() << ',' << q.w() << '\n';
        spdlog::info("[{}/{}] {}  {} points, {:.1f}% pixels with a point", fi + 1, frame_count, name, p.points, coverage);
    }
    spdlog::info("{} panoramas written to '{}'", written, o.out_dir.string());
    return written > 0 ? 0 : 4;
}
