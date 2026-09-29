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
    return nearest(timeAt(f));
}

int64_t Trajectory::timeAt(float f) const
{
    if (poses.empty()) return 0;
    const int64_t t0 = poses.front().ts_ns;
    const int64_t t1 = poses.back().ts_ns;
    return t0 + static_cast<int64_t>(static_cast<double>(t1 - t0) * f);
}

std::optional<Eigen::Affine3f> Trajectory::smoothedPose(int64_t ts_ns, double halfWindowSec) const
{
    const auto nearestPose = nearest(ts_ns);
    if (!nearestPose) return std::nullopt;
    const Eigen::Affine3f& nearestT = nearestPose->get().T;
    if (halfWindowSec <= 0.0) return nearestT;

    const int64_t halfNs = static_cast<int64_t>(halfWindowSec * 1e9);
    const auto first = std::lower_bound(poses.begin(), poses.end(), ts_ns - halfNs,
        [](const TrajPose& p, int64_t t){ return p.ts_ns < t; });
    const auto last = std::upper_bound(first, poses.end(), ts_ns + halfNs,
        [](int64_t t, const TrajPose& p){ return t < p.ts_ns; });

    const Eigen::Quaternionf qRef(nearestT.linear());
    Eigen::Vector3f posSum = Eigen::Vector3f::Zero();
    Eigen::Vector4f quatSum = Eigen::Vector4f::Zero();
    float weightSum = 0.f;
    for (auto it = first; it != last; ++it) {
        const double u = static_cast<double>(it->ts_ns - ts_ns) / static_cast<double>(halfNs); // [-1, 1]
        const float w = static_cast<float>(0.5 * (1.0 + std::cos(M_PI * u)));
        Eigen::Quaternionf q(it->T.linear());
        if (q.dot(qRef) < 0.f) q.coeffs() = -q.coeffs(); // q and -q are the same rotation
        posSum += w * it->T.translation();
        quatSum += w * q.coeffs();
        weightSum += w;
    }
    if (weightSum < 1e-6f) return nearestT; // poses too sparse to land inside the window

    Eigen::Affine3f out = Eigen::Affine3f::Identity();
    out.translation() = posSum / weightSum;
    out.linear() = Eigen::Quaternionf(quatSum.normalized()).toRotationMatrix();
    return out;
}

Eigen::Matrix3f levelHorizon(const Eigen::Matrix3f& R)
{
    const Eigen::Vector3f forward = R.col(0).normalized();
    const Eigen::Vector3f left = Eigen::Vector3f::UnitZ().cross(forward);
    if (left.norm() < 1e-3f) return R;
    Eigen::Matrix3f out;
    out.col(0) = forward;
    out.col(1) = left.normalized();
    out.col(2) = forward.cross(out.col(1));
    return out;
}

}  // namespace calib