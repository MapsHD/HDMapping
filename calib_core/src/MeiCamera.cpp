#include <CalibCore/MeiCamera.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

cv::Point2d MeiCamera::Project(cv::Point3d P) const {
    const double n = cv::norm(P);
    const cv::Point3d Xs(P.x / n, P.y / n, P.z / n); // onto the unit sphere

    const double denom = Xs.z + xi;
    const double x = Xs.x / denom, y = Xs.y / denom;
    const double r2 = x * x + y * y;
    const double radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    const double xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
    const double yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
    return {fx * xd + cx, fy * yd + cy};
}

cv::Point3d MeiCamera::Unproject(cv::Point2d uv, double* thetaDeg) const {
    const double xd = (uv.x - cx) / fx, yd = (uv.y - cy) / fy;

    // Invert the radial/tangential distortion by fixed-point (Newton-style)
    // iteration, same scheme cv::undistortPoints uses for the pinhole model.
    double x = xd, y = yd;
    for (int it = 0; it < 30; ++it) {
        const double r2 = x * x + y * y;
        const double radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
        const double dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
        const double dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
        x = (xd - dx) / radial;
        y = (yd - dy) / radial;
    }

    // Closed-form inverse of the unit-sphere/xi projection: solve
    // (rho2+1)*s^2 - 2*xi*s + (xi^2-1) = 0 for s = Xs.z + xi, where
    // Xs.x = x*s, Xs.y = y*s, Xs.z = s - xi, subject to |Xs| = 1.
    const double rho2 = x * x + y * y;
    const double s = (xi + std::sqrt(std::max(0.0, 1.0 + rho2 * (1.0 - xi * xi)))) / (rho2 + 1.0);
    const cv::Point3d Xs(x * s, y * s, s - xi);

    if (thetaDeg) *thetaDeg = std::acos(std::clamp(Xs.z, -1.0, 1.0)) * 180.0 / CV_PI;
    return Xs;
}

MeiCamera LoadMeiCamera(const std::string& path) {
    MeiCamera cam;
    YAML::Node node;
    try {
        node = YAML::LoadFile(path);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "calib_app: failed to load '%s': %s\n", path.c_str(), e.what());
        return cam;
    }

    try {
        cam.frameId         = node["frame_id"] ? node["frame_id"].as<std::string>() : "";
        cam.distortionModel = node["distortion_model"] ? node["distortion_model"].as<std::string>() : "";
        cam.width  = node["width"].as<int>();
        cam.height = node["height"].as<int>();
        cam.fx = node["fx"].as<double>();
        cam.fy = node["fy"].as<double>();
        cam.cx = node["cx"].as<double>();
        cam.cy = node["cy"].as<double>();
        cam.xi = node["xi"].as<double>();

        // distortion is (k1, k2, k3, p1, p2) for insta360_mei_v2 — see
        // MeiCamera.h. Read defensively: warn (don't silently drop data) if
        // the array isn't exactly the 5 elements this order assumes.
        YAML::Node d = node["distortion"];
        const size_t n = d.size();
        if (n != 5) {
            std::fprintf(stderr,
                "calib_app: WARNING '%s' distortion has %zu elements, expected 5 "
                "(k1,k2,k3,p1,p2 for %s) — missing ones default to 0, extras are ignored\n",
                path.c_str(), n, cam.distortionModel.c_str());
        }
        auto at = [&](size_t i) { return i < n ? d[i].as<double>() : 0.0; };
        cam.k1 = at(0); cam.k2 = at(1); cam.k3 = at(2); cam.p1 = at(3); cam.p2 = at(4);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "calib_app: '%s' is missing an expected field: %s\n", path.c_str(), e.what());
        return cam;
    }

    if (cam.distortionModel != "insta360_mei_v2") {
        std::fprintf(stderr,
            "calib_app: WARNING '%s' has distortion_model='%s', this app only knows how to "
            "reproject insta360_mei_v2 (results will be wrong if the model differs)\n",
            path.c_str(), cam.distortionModel.c_str());
    }

    cam.loaded = true;
    std::printf("calib_app: loaded intrinsics from %s (frame '%s', %dx%d, fx=%.3f fy=%.3f cx=%.3f cy=%.3f "
                "xi=%.4f k1=%.6g k2=%.6g k3=%.6g p1=%.6g p2=%.6g)\n",
                path.c_str(), cam.frameId.c_str(), cam.width, cam.height, cam.fx, cam.fy, cam.cx, cam.cy,
                cam.xi, cam.k1, cam.k2, cam.k3, cam.p1, cam.p2);
    return cam;
}
