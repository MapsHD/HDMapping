#pragma once

#include "lidar_odometry_utils.h"

#include <spdlog/spdlog.h>

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <string>

//! Between sliding-window resets the map buffer `points_global` only grows (every frame's points) and is read only AT a
//! reset: its second half becomes the new buffer and is folded into the buckets point by point, in order. Kept in a
//! file instead of RAM, and streamed back in the same order with the ray-cast decision of its whole size, the buckets
//! and so the trajectory are the ones the in-memory buffer gives; the RAM is one chunk. Empty directory = in memory.
//!
//! One spill file per compute_step_2 call, owned by a PointsGlobalSpill on that call's stack and reached through
//! LidarOdometryParams::points_global_spill: created on entry, its final close checked on success, removed on every exit.
//! A spill I/O failure ends the computation (compute_step_2 returns false); the PointsGlobalSpill that owns the files
//! removes them while the exception unwinds.
class PointsGlobalSpill
{
public:
    explicit PointsGlobalSpill(LidarOdometryParams& params);
    ~PointsGlobalSpill();
    PointsGlobalSpill(const PointsGlobalSpill&) = delete;
    PointsGlobalSpill& operator=(const PointsGlobalSpill&) = delete;

    void append(const Point3Di& p);

    // The reset, spilled: records [n/2, n) become the new buffer and are folded into the buckets in order.
    template<typename Update>
    void reset(Update update);

    // a successful run: the final close is checked, so a failed flush is not hidden behind a reported success
    void finish();

private:
    [[noreturn]] static void fail(const std::string& what);
    static void seek64(std::FILE* f, std::uint64_t offset);
    std::filesystem::path path() const;

    LidarOdometryParams& params_;
    std::filesystem::path dir_;
    std::filesystem::path pending_remove_;
    std::string tag_;
    std::FILE* f_ = nullptr;
    std::FILE* in_ = nullptr;
    std::uint64_t n_ = 0;
    int gen_ = 0;
};

template<typename Update>
void PointsGlobalSpill::reset(Update update)
{
    std::FILE* g = f_;
    f_ = nullptr; // closed below whatever fclose returns: the destructor must not close it again
    if (std::fclose(g) != 0)
        fail("close failed on '" + path().string() + "'");
    const std::filesystem::path old_path = path();
    pending_remove_ = old_path;
    in_ = std::fopen(old_path.string().c_str(), "rb");
    const std::uint64_t first = n_ / 2, keep = n_ - n_ / 2;
    gen_++;
    f_ = std::fopen(path().string().c_str(), "wb");
    if (!in_ || !f_)
        fail("reset could not open '" + old_path.string() + "' / '" + path().string() + "'");
    seek64(in_, first * sizeof(Point3Di));
    const bool ray_cast = keep < 100000;
    std::vector<Point3Di> chunk;
    const std::uint64_t CH = std::uint64_t(1) << 21;
    for (std::uint64_t done = 0; done < keep;)
    {
        const size_t m = static_cast<size_t>(std::min(CH, keep - done));
        chunk.resize(m);
        if (std::fread(chunk.data(), sizeof(Point3Di), m, in_) != m || std::fwrite(chunk.data(), sizeof(Point3Di), m, f_) != m)
            fail("reset read/write failed on '" + old_path.string() + "'");
        update(chunk, ray_cast);
        done += m;
    }
    std::fclose(in_);
    in_ = nullptr;
    std::error_code ec;
    std::filesystem::remove(old_path, ec);
    if (ec)
        spdlog::warn("points_global spill: could not remove '{}': {}", old_path.string(), ec.message());
    pending_remove_.clear();
    n_ = keep;
}