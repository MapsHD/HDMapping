#include <pch/pch.h>

#include <hash_utils.h>

uint64_t get_index(const int16_t x, const int16_t y, const int16_t z)
{
    return ((static_cast<uint64_t>(x) << 32) & (0x0000FFFF00000000ull)) | ((static_cast<uint64_t>(y) << 16) & (0x00000000FFFF0000ull)) |
        ((static_cast<uint64_t>(z) << 0) & (0x000000000000FFFFull));
}

uint64_t get_rgd_index(const Eigen::Vector3d p, const Eigen::Vector3d b)
{
    int16_t x = static_cast<int16_t>(p.x() / b.x());
    int16_t y = static_cast<int16_t>(p.y() / b.y());
    int16_t z = static_cast<int16_t>(p.z() / b.z());
    return get_index(x, y, z);
}
