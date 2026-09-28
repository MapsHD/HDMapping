#include <CalibCore/Trajectory.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>

namespace calib {

bool Trajectory::loadCSV(const std::string& path, const Eigen::Affine3f* mrp) {
    std::ifstream f(path);
    if (!f) return false;

    std::string line;
    std::getline(f, line); // skip header

    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        TrajPose p;
        float raw[12];
        // lidar_odometry_step_1 writes the timestamp as a double (seconds * 1e9),
        // so it can carry a fraction ("548348730189.99993896"); reading it
        // straight into an int64 would stop at the '.' and shift every column.
        double ts_ns = 0.0;
        ss >> ts_ns;
        p.ts_ns = std::llround(ts_ns);
        for (int i = 0; i < 12; i++) ss >> raw[i];
        if (!ss) continue;

        // row-major 3×4 → Affine3f
        p.T.linear() << raw[0], raw[1],  raw[2],
                        raw[4], raw[5],  raw[6],
                        raw[8], raw[9],  raw[10];
        p.T.translation() << raw[3], raw[7], raw[11];

        if (mrp) p.T = *mrp * p.T;
        poses.push_back(p);
    }
    return true;
}

void Trajectory::sort() {
    std::sort(poses.begin(), poses.end(),
              [](const TrajPose& a, const TrajPose& b){ return a.ts_ns < b.ts_ns; });
}

std::optional<std::reference_wrapper<const TrajPose>> Trajectory::nearest(int64_t ts_ns) const {
    if (poses.empty()) return std::nullopt;
    auto it = std::lower_bound(poses.begin(), poses.end(), ts_ns,
        [](const TrajPose& p, int64_t t){ return p.ts_ns < t; });
    if (it == poses.end())   return std::cref(poses.back());
    if (it == poses.begin()) return std::cref(poses.front());
    auto prev = std::prev(it);
    return (std::abs(it->ts_ns - ts_ns) < std::abs(prev->ts_ns - ts_ns)) ? std::cref(*it) : std::cref(*prev);
}

std::optional<std::reference_wrapper<const TrajPose>> Trajectory::nearest(float f) const
{
    if (poses.empty()) return std::nullopt;
    const int64_t t0 = poses.front().ts_ns;
    const int64_t t1 = poses.back().ts_ns;
    const int64_t t = t0 + static_cast<int64_t>(static_cast<double>(t1 - t0) * f);
    return nearest(t);
}

}  // namespace calib