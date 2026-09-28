#pragma once
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <cstdint>
#include <optional>
#include <functional>

namespace calib {

//! One LiDAR pose from the trajectory CSV.
struct TrajPose {
    //! Timestamp, nanoseconds.
    int64_t         ts_ns = 0;
    //! T_world_lidar, i.e. p_world = T * p_lidar.
    Eigen::Affine3f T     = Eigen::Affine3f::Identity();
};

//! A LiDAR trajectory: poses over time, loaded from Mandeye's CSV.
struct Trajectory {
    //! The poses. Kept in whatever order they were loaded until @ref sort.
    std::vector<TrajPose> poses;

    //! Load one trajectory_lio_N.csv, appending to @ref poses.
    //! @param path CSV to read
    //! @param mrp optional correction applied to every pose loaded,
    //!        T_corrected = *mrp * T_pose; ignored when null
    //! @return false if the file could not be opened
    bool loadCSV(const std::string& path, const Eigen::Affine3f* mrp = nullptr);

    //! Sort @ref poses by ascending timestamp. Call after loading, and before
    //! @ref nearest, which relies on the ordering.
    void sort();

    //! Pose closest in time to `ts_ns`.
    //! @param ts_ns timestamp to look up, nanoseconds
    //! @return the nearest pose, clamped to the first or last one when `ts_ns`
    //!         falls outside the trajectory, or nullopt when it is empty
    //! @warning Assumes @ref poses is sorted by timestamp -- it binary-searches.
    //!          Call @ref sort first, or the result is arbitrary.
    [[nodiscard]] std::optional<std::reference_wrapper<const TrajPose>> nearest(int64_t ts_ns) const;

    //! Pose at fractional position `f` along the trajectory's time span.
    //! @param f 0 = @ref poses front (earliest), 1 = back (latest); not
    //!        clamped -- values outside [0,1] extrapolate past the ends and
    //!        get clamped by @ref nearest(int64_t) to the first/last pose.
    //! @return nullopt when it is empty; see @ref nearest(int64_t) otherwise.
    //! @warning Same ordering requirement as @ref nearest(int64_t).
    [[nodiscard]] std::optional<std::reference_wrapper<const TrajPose>> nearest(float f) const;


    //! True when no poses have been loaded.
    bool empty() const { return poses.empty(); }
};

}  // namespace calib