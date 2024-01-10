#ifndef _LIDAR_ODOMETRY_UTILS_H_
#define _LIDAR_ODOMETRY_UTILS_H_

#include <structures.h>
#include <Eigen/Dense>
#include <vector>

struct WorkerData
{
    std::vector<Point3Di> intermediate_points;
    std::vector<Point3Di> original_points;
    std::vector<Eigen::Affine3d> intermediate_trajectory;
    std::vector<Eigen::Affine3d> intermediate_trajectory_motion_model;
    std::vector<double> intermediate_trajectory_timestamps;
    std::vector<std::pair<double, double>> imu_roll_pitch;
    bool show = false;
};

unsigned long long int get_index(const int16_t x, const int16_t y, const int16_t z);
unsigned long long int get_rgd_index(const Eigen::Vector3d p, const Eigen::Vector3d b);

#endif