#pragma once

#include "lidar_odometry_utils.h"

#include <functional>
#include <string>
#include <vector>

//! Raw point clouds loaded per file during step 1 (LidarOdometryParams::lazy_load_raw_clouds).
//!
//! Eagerly, load_data holds every file's raw cloud before step 1 (~56 B per point: ~30 GB for 500 M points). Step 1
//! reads them chunk by chunk in time order, each chunk taking [t0, t1) from every file by binary search. Lazily, the
//! first pass loads each file once (one per thread at a time) only to record its time range; step 1 then loads a file
//! when its range overlaps the chunk and frees it once the chunk start has passed its last point. A file is loaded by
//! the same call and sorted by the same comparator as the eager load, so every chunk receives the same points in the
//! same order. The loader lives in LidarOdometryParams::raw_cloud_loader, so each pipeline owns its own.
struct RawCloudLoader
{
    std::function<std::vector<Point3Di>(size_t)> load;
    std::vector<std::string> files;
    std::vector<std::uintmax_t> file_size;
    std::vector<fs::file_time_type> file_time;
    std::vector<double> t_first, t_last;
    std::vector<size_t> n;
    std::vector<char> loaded, done;
};

//! Load file i if step 1 needs it and it is not in memory. Lazy loading reads every file twice (the range pass in
//! load_data, then here), so a file that changed in between is refused rather than silently processed.
bool lazy_ensure(RawCloudLoader* L, std::vector<std::vector<Point3Di>>& ppf, size_t i);

void lazy_release(RawCloudLoader* L, std::vector<std::vector<Point3Di>>& ppf, size_t i);