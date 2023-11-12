#pragma once
#include <vector>
#include <Eigen/Dense>
#include <laszip/laszip_api.h>
namespace mandeye
{
    struct Point {
        double timestamp;
        float intensity;
        Eigen::Vector3d point;
    };


    struct PointRGB {
        double timestamp;
        float intensity;
        Eigen::Vector3d point;
        Eigen::Vector4f rgb;
        PointRGB() = default;
        PointRGB(const mandeye::Point& p) :
            timestamp(p.timestamp),
            intensity(p.intensity),
            point(p.point) {};
    };

    std::vector<Point> load(const std::string& lazFile);
    bool saveLaz(const std::string& filename, const std::vector<mandeye::PointRGB>& buffer);
}