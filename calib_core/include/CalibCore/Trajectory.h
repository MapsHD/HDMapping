#pragma once
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <cstdint>

namespace calib {

// One LiDAR pose from the trajectory CSV.
// T = T_world_lidar:  p_world = T * p_lidar
struct TrajPose {
    int64_t         ts_ns = 0;
    Eigen::Affine3f T     = Eigen::Affine3f::Identity();
};

struct Trajectory {
    std::vector<TrajPose> poses;

    // Load one trajectory_lio_N.csv. Appends to poses.
    // If mrp != nullptr it is applied to every pose: T_corrected = *mrp * T_pose.
    bool loadCSV(const std::string& path, const Eigen::Affine3f* mrp = nullptr);

    void sort();

    const TrajPose* nearest(int64_t ts_ns) const;

    bool empty() const { return poses.empty(); }
};

}  // namespace calib