#include "points_global_spill.h"

#include <chrono>
#include <limits>
#include <random>
#include <stdexcept>

PointsGlobalSpill::PointsGlobalSpill(LidarOdometryParams& params)
    : params_(params)
{
    if (params.points_global_spill_directory.empty())
        return;
    dir_ = params.points_global_spill_directory;
    std::random_device rd;
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    tag_ = std::to_string((static_cast<std::uint64_t>(rd()) << 32 ^ rd()) ^ static_cast<std::uint64_t>(now));
    f_ = std::fopen(path().string().c_str(), "wb");
    if (!f_)
        fail("cannot create '" + path().string() + "' (points_global_spill_directory)");
    spdlog::info("points_global spilled to '{}'", path().string());
    params_.points_global_spill = this;
}

PointsGlobalSpill::~PointsGlobalSpill()
{
    if (params_.points_global_spill == this)
        params_.points_global_spill = nullptr;
    if (in_)
        std::fclose(in_);
    if (f_)
        std::fclose(f_);
    std::error_code ec;
    if (!pending_remove_.empty()) // a reset that did not finish: the previous generation's file too
        std::filesystem::remove(pending_remove_, ec);
    if (!dir_.empty())
        std::filesystem::remove(path(), ec);
}

void PointsGlobalSpill::append(const Point3Di& p)
{
    if (std::fwrite(&p, sizeof(Point3Di), 1, f_) != 1)
        fail("write failed on '" + path().string() + "'");
    n_++;
}

void PointsGlobalSpill::finish()
{
    if (f_ && std::fclose(f_) != 0)
    {
        f_ = nullptr;
        fail("close failed on '" + path().string() + "'");
    }
    f_ = nullptr;
}

void PointsGlobalSpill::fail(const std::string& what)
{
    throw std::runtime_error("points_global spill: " + what);
}

void PointsGlobalSpill::seek64(std::FILE* f, std::uint64_t offset)
{
#if defined(_WIN32)
    const bool ok = _fseeki64(f, static_cast<__int64>(offset), SEEK_SET) == 0;
#else
    // off_t is 32-bit on a 32-bit POSIX build without large-file support: refuse an offset it cannot hold
    if (offset > static_cast<std::uint64_t>(std::numeric_limits<off_t>::max()))
        fail("offset beyond this platform's off_t (build with _FILE_OFFSET_BITS=64)");
    const bool ok = fseeko(f, static_cast<off_t>(offset), SEEK_SET) == 0;
#endif
    if (!ok)
        fail("seek failed");
}

std::filesystem::path PointsGlobalSpill::path() const
{
    return dir_ / ("points_global_" + tag_ + "_" + std::to_string(gen_) + ".bin");
}