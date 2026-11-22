#include "raw_cloud_loader.h"

#include <iostream>

bool RawCloudLoader::ensure(std::vector<std::vector<Point3Di>>& ppf, size_t i)
{
    if (loaded[i] || done[i] || n[i] == 0)
        return true;
    const auto unchanged = [&]()
    {
        const ChunkStamp now = source->stamp(i);
        return now.valid && now.size == stamp[i].size && now.time == stamp[i].time;
    };
    if (!unchanged())
    {
        std::cerr << "lazy_load_raw_clouds: '" << source->chunkName(i) << "' changed or vanished since it was first read\n";
        return false;
    }
    ppf[i] = source->loadPoints(i, filter);
    // checked again after the read, so a file replaced while it was being opened is refused too
    if (!unchanged() || ppf[i].size() != n[i] || ppf[i].front().timestamp != t_first[i] || ppf[i].back().timestamp != t_last[i])
    {
        std::cerr << "lazy_load_raw_clouds: '" << source->chunkName(i) << "' reads differently than when it was first read\n";
        std::vector<Point3Di>().swap(ppf[i]);
        return false;
    }
    loaded[i] = 1;
    return true;
}

void RawCloudLoader::release(std::vector<std::vector<Point3Di>>& ppf, size_t i)
{
    if (!loaded[i])
        return;
    std::vector<Point3Di>().swap(ppf[i]);
    loaded[i] = 0;
    done[i] = 1;
}