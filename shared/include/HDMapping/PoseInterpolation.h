#pragma once
#include <Eigen/Geometry>
#include <cmath>
#include <map>

// Was byte-for-byte duplicated between apps/lidar_odometry_step_1's
// lidar_odometry_utils.h/.cpp (also compiled directly into
// multi_view_tls_registration_step_2 and mandeye_compare_trajectories) and
// apps/camera_lidar_trajectory_viewer's TrajectoryViewer.cpp. Lives under
// shared/ (like HDMapping/Version.hpp) rather than calib_core or Core: it's
// plain Eigen/std with no family-specific dependencies, and shared/include
// is already on every target's include path via the top-level
// CMakeLists.txt's include_directories(shared/include), so no target needs
// a new include dir or library link to use it.

// Interpolates (SLERP for rotation, linear for translation) the pose at
// query_time from a time(seconds) -> T_world_lidar trajectory map. Returns a
// zero matrix if query_time falls outside the trajectory's covered range.
inline Eigen::Matrix4d getInterpolatedPose(const std::map<double, Eigen::Matrix4d>& trajectory, double query_time)
{
    Eigen::Matrix4d ret(Eigen::Matrix4d::Zero());
    auto it_lower = trajectory.lower_bound(query_time);
    auto it_next = it_lower;

    if (it_lower == trajectory.begin())
    {
        return ret;
    }
    if (it_lower->first > query_time)
    {
        it_lower = std::prev(it_lower);
    }
    if (it_lower == trajectory.begin())
    {
        return ret;
    }
    if (it_lower == trajectory.end())
    {
        return ret;
    }

    constexpr double MaxInterpolationS = 0.1;
    double t1 = it_lower->first;
    double t2 = it_next->first;
    double difft1 = t1 - query_time;

    if (t1 == t2 && std::fabs(difft1) < MaxInterpolationS)
    {
        ret = Eigen::Matrix4d::Identity();
        ret.col(3).head<3>() = it_next->second.col(3).head<3>();
        ret.topLeftCorner(3, 3) = it_lower->second.topLeftCorner(3, 3);
        return ret;
    }

    {
        assert(t2 > t1);
        assert(query_time > t1);
        assert(query_time < t2);
        ret = Eigen::Matrix4d::Identity();
        const double res = (query_time - t1) / (t2 - t1); //residual
        const Eigen::Vector3d diff = it_next->second.col(3).head<3>() - it_lower->second.col(3).head<3>();
        ret.col(3).head<3>() = it_lower->second.col(3).head<3>() + diff * res;
        Eigen::Matrix3d r1 = it_lower->second.topLeftCorner(3, 3).matrix();
        Eigen::Matrix3d r2 = it_next->second.topLeftCorner(3, 3).matrix();
        Eigen::Quaterniond q1(r1);
        Eigen::Quaterniond q2(r2);
        Eigen::Quaterniond qt = q1.slerp(res, q2);
        ret.topLeftCorner(3, 3) = qt.toRotationMatrix();
        return ret;
    }
}
