#include "raw_cloud_loader.h"

#include <iostream>

bool lazy_ensure(RawCloudLoader* L, std::vector<std::vector<Point3Di>>& ppf, size_t i)
{
    if (!L || L->loaded[i] || L->done[i] || L->n[i] == 0)
        return true;
    const auto unchanged = [&]()
    {
        std::error_code ec;
        const auto size = fs::file_size(L->files[i], ec);
        const auto time = fs::last_write_time(L->files[i], ec);
        return !ec && size == L->file_size[i] && time == L->file_time[i];
    };
    if (!unchanged())
    {
        std::cerr << "lazy_load_raw_clouds: '" << L->files[i] << "' changed or vanished since it was first read\n";
        return false;
    }
    ppf[i] = L->load(i);
    // checked again after the read, so a file replaced while it was being opened is refused too
    if (!unchanged() || ppf[i].size() != L->n[i] || ppf[i].front().timestamp != L->t_first[i] || ppf[i].back().timestamp != L->t_last[i])
    {
        std::cerr << "lazy_load_raw_clouds: '" << L->files[i] << "' reads differently than when it was first read\n";
        std::vector<Point3Di>().swap(ppf[i]);
        return false;
    }
    L->loaded[i] = 1;
    return true;
}

void lazy_release(RawCloudLoader* L, std::vector<std::vector<Point3Di>>& ppf, size_t i)
{
    if (!L || !L->loaded[i])
        return;
    std::vector<Point3Di>().swap(ppf[i]);
    L->loaded[i] = 0;
    L->done[i] = 1;
}