#include <CalibCore/MeiCamera.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

Eigen::Vector2d MeiCamera::Project(const Eigen::Vector3d& P) const
{
    const Eigen::Vector3d Xs = P.normalized(); // onto the unit sphere

    const double denom = Xs.z() + xi;
    const double x = Xs.x() / denom, y = Xs.y() / denom;
    const double r2 = x * x + y * y;
    const double radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    const double xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
    const double yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
    return { fx * xd + cx, fy * yd + cy };
}

namespace
{
    std::string trim(std::string s)
    {
        const char* ws = " \t\r\n";
        const auto b = s.find_first_not_of(ws);
        if (b == std::string::npos)
            return {};
        return s.substr(b, s.find_last_not_of(ws) - b + 1);
    }

    // This rig's camera_info.yaml is a flat mapping of `key: value` scalars
    // plus a `distortion: [a, b, c, d, e]` flow sequence -- no nesting, no
    // anchors, no block sequences. Parsed here rather than with a YAML
    // library so calib_core keeps depending on nothing but Eigen/LASzip/std.
    std::map<std::string, std::string> readFlatYaml(std::istream& in)
    {
        std::map<std::string, std::string> kv;
        std::string line;
        while (std::getline(in, line))
        {
            const auto hash = line.find('#');
            if (hash != std::string::npos)
                line = line.substr(0, hash);
            const auto colon = line.find(':');
            if (colon == std::string::npos)
                continue;
            std::string key = trim(line.substr(0, colon));
            if (!key.empty())
                kv[key] = trim(line.substr(colon + 1));
        }
        return kv;
    }

    std::vector<double> parseArray(const std::string& v)
    {
        std::vector<double> out;
        std::string inner = trim(v);
        if (inner.size() >= 2 && inner.front() == '[' && inner.back() == ']')
            inner = inner.substr(1, inner.size() - 2);
        std::stringstream ss(inner);
        std::string tok;
        while (std::getline(ss, tok, ','))
        {
            tok = trim(tok);
            if (!tok.empty())
                out.push_back(std::strtod(tok.c_str(), nullptr));
        }
        return out;
    }
} // namespace

MeiCamera LoadMeiCamera(const std::string& path)
{
    MeiCamera cam;
    std::ifstream f(path);
    if (!f)
    {
        std::fprintf(stderr, "calib_app: failed to open '%s'\n", path.c_str());
        return cam;
    }
    const std::map<std::string, std::string> kv = readFlatYaml(f);

    // Every numeric field is required: a calibration silently defaulting one
    // of these to 0 reprojects wrongly with no visible failure.
    for (const char* key : { "width", "height", "fx", "fy", "cx", "cy", "xi", "distortion" })
    {
        if (kv.find(key) == kv.end())
        {
            std::fprintf(stderr, "calib_app: '%s' is missing required field '%s'\n", path.c_str(), key);
            return cam;
        }
    }

    auto num = [&](const char* key) { return std::strtod(kv.at(key).c_str(), nullptr); };
    const auto unquote = [](std::string s)
    {
        if (s.size() >= 2 && (s.front() == '"' || s.front() == '\'') && s.back() == s.front())
            return s.substr(1, s.size() - 2);
        return s;
    };

    const auto frameIt = kv.find("frame_id");
    const auto modelIt = kv.find("distortion_model");
    cam.frameId = frameIt != kv.end() ? unquote(frameIt->second) : "";
    cam.distortionModel = modelIt != kv.end() ? unquote(modelIt->second) : "";
    cam.width = static_cast<int>(num("width"));
    cam.height = static_cast<int>(num("height"));
    cam.fx = num("fx");
    cam.fy = num("fy");
    cam.cx = num("cx");
    cam.cy = num("cy");
    cam.xi = num("xi");

    // distortion is (k1, k2, k3, p1, p2) for insta360_mei_v2 -- see
    // MeiCamera.h. Warn rather than silently drop data if it isn't the 5
    // elements that order assumes.
    const std::vector<double> d = parseArray(kv.at("distortion"));
    if (d.size() != 5)
    {
        std::fprintf(
            stderr,
            "calib_app: WARNING '%s' distortion has %zu elements, expected 5 "
            "(k1,k2,k3,p1,p2 for %s) -- missing ones default to 0, extras are ignored\n",
            path.c_str(),
            d.size(),
            cam.distortionModel.c_str());
    }
    auto at = [&](size_t i) { return i < d.size() ? d[i] : 0.0; };
    cam.k1 = at(0);
    cam.k2 = at(1);
    cam.k3 = at(2);
    cam.p1 = at(3);
    cam.p2 = at(4);

    if (cam.distortionModel != "insta360_mei_v2")
    {
        std::fprintf(
            stderr,
            "calib_app: WARNING '%s' has distortion_model='%s', only insta360_mei_v2 is supported "
            "(results will be wrong if the model differs)\n",
            path.c_str(),
            cam.distortionModel.c_str());
    }

    cam.loaded = true;
    return cam;
}
