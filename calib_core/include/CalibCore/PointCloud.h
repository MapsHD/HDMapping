#pragma once
#include <vector>
#include <string>

#include <cstdint>

namespace calib {

struct Point3D {
    float   x, y, z;
    float   intensity; // normalized to [0,1]
    int64_t ts_ns = 0; // GPS time cast from gps_time field (0 if unavailable)
    uint8_t r = 0, g = 0, b = 0; // LAS rgb (16-bit >> 8); 0 when the file has none
};

struct PointCloud {
    std::vector<Point3D> points;
    float minX = 0.f, maxX = 0.f;
    float minY = 0.f, maxY = 0.f;
    float minZ = 0.f, maxZ = 0.f;
    bool hasRgb = false; // point format carries rgb AND at least one point is non-black

    bool load(const std::string& path);
    void clear();
    bool empty() const { return points.empty(); }
};

std::string GetLidarSerial(const char* path);

}  // namespace calib
