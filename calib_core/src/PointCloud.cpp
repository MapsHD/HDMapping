#include <CalibCore/PointCloud.h>
#include <laszip/laszip_api.h>
#include <limits>
#include <cstdio>

namespace calib {

void PointCloud::clear() {
    points.clear();
    minX = maxX = minY = maxY = minZ = maxZ = 0.f;
}

bool PointCloud::load(const std::string& path) {
    clear();

    laszip_POINTER reader = nullptr;
    if (laszip_create(&reader) != 0) {
        fprintf(stderr, "laszip_create failed\n");
        return false;
    }

    laszip_BOOL is_compressed = 0;
    if (laszip_open_reader(reader, path.c_str(), &is_compressed) != 0) {
        laszip_CHAR* err = nullptr;
        laszip_get_error(reader, &err);
        fprintf(stderr, "Cannot open %s: %s\n", path.c_str(), err ? err : "?");
        laszip_destroy(reader);
        return false;
    }

    laszip_header_struct* header = nullptr;
    laszip_get_header_pointer(reader, &header);

    laszip_I64 npoints = (header->number_of_point_records > 0)
        ? static_cast<laszip_I64>(header->number_of_point_records)
        : static_cast<laszip_I64>(header->extended_number_of_point_records);

    laszip_point_struct* point = nullptr;
    laszip_get_point_pointer(reader, &point);

    points.reserve(static_cast<size_t>(npoints));

    float minInt =  std::numeric_limits<float>::max();
    float maxInt = -std::numeric_limits<float>::max();
    float xmin =  1e38f, xmax = -1e38f;
    float ymin =  1e38f, ymax = -1e38f;
    float zmin =  1e38f, zmax = -1e38f;

    for (laszip_I64 i = 0; i < npoints; ++i) {
        if (laszip_read_point(reader) != 0) break;

        laszip_F64 coords[3];
        laszip_get_coordinates(reader, coords);

        Point3D p;
        p.x = static_cast<float>(coords[0]);
        p.y = static_cast<float>(coords[1]);
        p.z = static_cast<float>(coords[2]);
        p.intensity = static_cast<float>(point->intensity);
        // gps_time is GPS time in seconds (LAS spec); ts_ns is nanoseconds
        // everywhere else it's used (Trajectory, image matching), so convert here.
        p.ts_ns     = static_cast<int64_t>(point->gps_time * 1e9);

        if (p.x < xmin) xmin = p.x; if (p.x > xmax) xmax = p.x;
        if (p.y < ymin) ymin = p.y; if (p.y > ymax) ymax = p.y;
        if (p.z < zmin) zmin = p.z; if (p.z > zmax) zmax = p.z;
        if (p.intensity < minInt) minInt = p.intensity;
        if (p.intensity > maxInt) maxInt = p.intensity;

        points.push_back(p);
    }

    // Normalize intensity to [0,1]
    float intRange = (maxInt > minInt) ? (maxInt - minInt) : 1.f;
    for (auto& p : points)
        p.intensity = (p.intensity - minInt) / intRange;

    minX = xmin; maxX = xmax;
    minY = ymin; maxY = ymax;
    minZ = zmin; maxZ = zmax;

    laszip_close_reader(reader);
    laszip_destroy(reader);

    printf("Loaded %zu points from %s\n", points.size(), path.c_str());
    return true;

}

}  // namespace calib
