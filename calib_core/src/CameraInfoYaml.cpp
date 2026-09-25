#include <CalibCore/Camera.h>

#include <algorithm>
#include <cctype>
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

    // A flat camera_info.yaml is a mapping of `key: value` scalars plus a
    // `distortion: [a, b, c, d, e]` flow sequence -- no nesting, no anchors,
    // no block sequences. Parsed here rather than with a YAML library so
    // calib_core keeps depending on nothing but Eigen/LASzip/std.
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

    //! How one distortion_model maps onto Intrinsics.
    struct FlatModel
    {
        const char* name; //!< distortion_model, lower-case
        CameraModel model;
        std::vector<float Intrinsics::*> order; //!< the field each `distortion` entry goes to
    };

    const std::vector<FlatModel>& flatModels()
    {
        using I = Intrinsics;
        static const std::vector<FlatModel> models = {
            { "insta360_mei_v2", CameraModel::Mei, { &I::k1, &I::k2, &I::k3, &I::p1, &I::p2 } },
            { "equidistant", CameraModel::Fisheye, { &I::k1, &I::k2, &I::k3, &I::k4 } },
            { "plumb_bob", CameraModel::Pinhole, { &I::k1, &I::k2, &I::p1, &I::p2, &I::k3 } },
            { "rational_polynomial", CameraModel::Pinhole, { &I::k1, &I::k2, &I::p1, &I::p2, &I::k3, &I::k4, &I::k5, &I::k6 } },
        };
        return models;
    }
} // namespace

bool loadCameraInfoYaml(const std::string& path, Intrinsics& K)
{
    std::ifstream f(path);
    if (!f)
    {
        std::fprintf(stderr, "calib_core: failed to open '%s'\n", path.c_str());
        return false;
    }
    const std::map<std::string, std::string> kv = readFlatYaml(f);

    const auto unquote = [](std::string s)
    {
        if (s.size() >= 2 && (s.front() == '"' || s.front() == '\'') && s.back() == s.front())
            return s.substr(1, s.size() - 2);
        return s;
    };
    const auto modelIt = kv.find("distortion_model");
    std::string distortionModel = modelIt != kv.end() ? unquote(modelIt->second) : "";
    std::transform(
        distortionModel.begin(),
        distortionModel.end(),
        distortionModel.begin(),
        [](unsigned char c)
        {
            return static_cast<char>(std::tolower(c));
        });

    const FlatModel* layout = nullptr;
    const std::string name = distortionModel == "fisheye" ? "equidistant" : distortionModel;
    for (const FlatModel& m : flatModels())
        if (name == m.name)
            layout = &m;
    // Files the Mei-only loader this replaced used to accept.
    if (!layout && (distortionModel.find("mei") != std::string::npos || (distortionModel.empty() && kv.count("xi"))))
    {
        std::fprintf(
            stderr,
            "calib_core: WARNING '%s' has distortion_model='%s', reading it as insta360_mei_v2 "
            "(results will be wrong if the model differs)\n",
            path.c_str(),
            distortionModel.c_str());
        layout = &flatModels().front();
    }
    if (!layout)
    {
        std::string known;
        for (const FlatModel& m : flatModels())
            known += std::string(known.empty() ? "" : ", ") + m.name;
        std::fprintf(
            stderr, "calib_core: '%s' has distortion_model='%s', expected one of %s\n", path.c_str(), distortionModel.c_str(), known.c_str());
        return false;
    }

    // Every numeric field is required: a calibration silently defaulting one
    // of these to 0 reprojects wrongly with no visible failure.
    std::vector<const char*> required = { "width", "height", "fx", "fy", "cx", "cy", "distortion" };
    if (layout->model == CameraModel::Mei)
        required.push_back("xi");
    for (const char* key : required)
    {
        if (kv.find(key) == kv.end())
        {
            std::fprintf(stderr, "calib_core: '%s' is missing required field '%s'\n", path.c_str(), key);
            return false;
        }
    }

    const std::vector<double> d = parseArray(kv.at("distortion"));
    const size_t n = layout->order.size();
    if (d.size() != n)
    {
        // Four coefficients are all a fisheye has, so any other count means
        // the file is not the model it names.
        if (layout->model == CameraModel::Fisheye)
        {
            std::fprintf(
                stderr, "calib_core: '%s' distortion has %zu elements, equidistant needs exactly 4 (k1,k2,k3,k4)\n", path.c_str(), d.size());
            return false;
        }
        std::fprintf(
            stderr,
            "calib_core: WARNING '%s' distortion has %zu elements, expected %zu for %s "
            "-- missing ones default to 0, extras are ignored\n",
            path.c_str(),
            d.size(),
            n,
            layout->name);
    }

    auto num = [&](const char* key)
    {
        return std::strtod(kv.at(key).c_str(), nullptr);
    };
    K = Intrinsics{};
    K.model = layout->model;
    K.width = static_cast<int>(num("width"));
    K.height = static_cast<int>(num("height"));
    K.fx = static_cast<float>(num("fx"));
    K.fy = static_cast<float>(num("fy"));
    K.cx = static_cast<float>(num("cx"));
    K.cy = static_cast<float>(num("cy"));
    if (layout->model == CameraModel::Mei)
        K.xi = static_cast<float>(num("xi"));
    for (size_t i = 0; i < n; ++i)
        K.*(layout->order[i]) = i < d.size() ? static_cast<float>(d[i]) : 0.f;
    return true;
}

} // namespace calib