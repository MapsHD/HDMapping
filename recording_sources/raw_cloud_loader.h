#pragma once

#include "lidar_odometry_utils.h"
#include "recording_source.h"

#include <memory>
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
    std::shared_ptr<const RecordingSource> source;
    PointFilter filter;
    std::vector<ChunkStamp> stamp; //!< each chunk's state when first read
    std::vector<double> t_first, t_last;
    std::vector<size_t> n;
    std::vector<char> loaded, done;

    //! Load chunk i into ppf[i] if step 1 needs it and it is not in memory. Lazy loading reads every chunk twice (the
    //! range pass in load_data, then here), so a chunk that changed in between is refused rather than silently processed.
    bool ensure(std::vector<std::vector<Point3Di>>& ppf, size_t i);
    //! Free ppf[i] for good: step 1 has passed its last point.
    void release(std::vector<std::vector<Point3Di>>& ppf, size_t i);
};
