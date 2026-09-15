#include <CalibCore/Camera.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace calib
{

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

bool loadMeiIntrinsics(const std::string& path, Intrinsics& K)
{
    std::ifstream f(path);
    if (!f)
    {
        std::fprintf(stderr, "calib_core: failed to open '%s'\n", path.c_str());
        return false;
    }
    const std::map<std::string, std::string> kv = readFlatYaml(f);

    // Every numeric field is required: a calibration silently defaulting one
    // of these to 0 reprojects wrongly with no visible failure.
    for (const char* key : { "width", "height", "fx", "fy", "cx", "cy", "xi", "distortion" })
    {
        if (kv.find(key) == kv.end())
        {
            std::fprintf(stderr, "calib_core: '%s' is missing required field '%s'\n", path.c_str(), key);
            return false;
        }
    }

    auto num = [&](const char* key) { return std::strtod(kv.at(key).c_str(), nullptr); };
    const auto unquote = [](std::string s)
    {
        if (s.size() >= 2 && (s.front() == '"' || s.front() == '\'') && s.back() == s.front())
            return s.substr(1, s.size() - 2);
        return s;
    };

    const auto modelIt = kv.find("distortion_model");
    const std::string distortionModel = modelIt != kv.end() ? unquote(modelIt->second) : "";

    K = Intrinsics{};
    K.model = CameraModel::Mei;
    K.width = static_cast<int>(num("width"));
    K.height = static_cast<int>(num("height"));
    K.fx = static_cast<float>(num("fx"));
    K.fy = static_cast<float>(num("fy"));
    K.cx = static_cast<float>(num("cx"));
    K.cy = static_cast<float>(num("cy"));
    K.xi = static_cast<float>(num("xi"));

    // distortion is (k1, k2, k3, p1, p2) for insta360_mei_v2 -- see
    // Camera.h. Warn rather than silently drop data if it is not the 5
    // elements that order assumes.
    const std::vector<double> d = parseArray(kv.at("distortion"));
    if (d.size() != 5)
    {
        std::fprintf(
            stderr,
            "calib_core: WARNING '%s' distortion has %zu elements, expected 5 "
            "(k1,k2,k3,p1,p2 for %s) -- missing ones default to 0, extras are ignored\n",
            path.c_str(),
            d.size(),
            distortionModel.c_str());
    }
    auto at = [&](size_t i) { return i < d.size() ? static_cast<float>(d[i]) : 0.f; };
    K.k1 = at(0);
    K.k2 = at(1);
    K.k3 = at(2);
    K.p1 = at(3);
    K.p2 = at(4);
    // k4/k5/k6 are the rational denominator, which the Mei polynomial has no
    // equivalent of; Intrinsics{} above already left them at 0.

    if (distortionModel != "insta360_mei_v2")
    {
        std::fprintf(
            stderr,
            "calib_core: WARNING '%s' has distortion_model='%s', only insta360_mei_v2 is supported "
            "(results will be wrong if the model differs)\n",
            path.c_str(),
            distortionModel.c_str());
    }

    return true;
}

} // namespace calib
