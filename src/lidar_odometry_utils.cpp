#include "lidar_odometry_utils.h"

// this function provides unique index
unsigned long long int get_index(const int16_t x, const int16_t y, const int16_t z)
{
    return ((static_cast<unsigned long long int>(x) << 32) & (0x0000FFFF00000000ull)) |
           ((static_cast<unsigned long long int>(y) << 16) & (0x00000000FFFF0000ull)) |
           ((static_cast<unsigned long long int>(z) << 0) & (0x000000000000FFFFull));
}

// this function provides unique index for input point p and 3D space decomposition into buckets b
unsigned long long int get_rgd_index(const Eigen::Vector3d p, const Eigen::Vector3d b)
{
    int16_t x = static_cast<int16_t>(p.x() / b.x());
    int16_t y = static_cast<int16_t>(p.y() / b.y());
    int16_t z = static_cast<int16_t>(p.z() / b.z());
    return get_index(x, y, z);
}