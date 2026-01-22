#include <pch/pch.h>

#include <hash_utils.h>

uint64_t get_index_2d(const int16_t x, const int16_t y)
{
    static constexpr auto shift_x = 16 * 1;
    static constexpr auto shift_y = 16 * 0;

    static constexpr auto mask_x = 0x00000000FFFF0000ull;
    static constexpr auto mask_y = 0x000000000000FFFFull;

    const auto x_part = ((static_cast<uint64_t>(x) << shift_x) & (mask_x));
    const auto y_part = ((static_cast<uint64_t>(y) << shift_y) & (mask_y));

    return x_part | y_part;
}

uint64_t get_rgd_index_2d(const Eigen::Vector3d& p, const Eigen::Vector2d& b)
{
    const int16_t x = static_cast<int16_t>(p.x() / b.x());
    const int16_t y = static_cast<int16_t>(p.y() / b.y());

    return get_index_2d(x, y);
}

uint64_t get_index_3d(const int16_t x, const int16_t y, const int16_t z)
{
    static constexpr auto shift_x = 16 * 2;
    static constexpr auto shift_y = 16 * 1;
    static constexpr auto shift_z = 16 * 0;

    static constexpr auto mask_x = 0x0000FFFF00000000ull;
    static constexpr auto mask_y = 0x00000000FFFF0000ull;
    static constexpr auto mask_z = 0x000000000000FFFFull;

    const auto x_part = ((static_cast<uint64_t>(x) << shift_x) & (mask_x));
    const auto y_part = ((static_cast<uint64_t>(y) << shift_y) & (mask_y));
    const auto z_part = ((static_cast<uint64_t>(z) << shift_z) & (mask_z));

    return x_part | y_part | z_part;
}

uint64_t get_rgd_index_3d(const Eigen::Vector3d& p, const Eigen::Vector3d& b)
{
    const int16_t x = static_cast<int16_t>(p.x() / b.x());
    const int16_t y = static_cast<int16_t>(p.y() / b.y());
    const int16_t z = static_cast<int16_t>(p.z() / b.z());

    return get_index_3d(x, y, z);
}
