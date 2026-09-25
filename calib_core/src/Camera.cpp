#include <CalibCore/Camera.h>

#include <algorithm>
#include <cctype>
#include <fstream>

#include <nlohmann/json.hpp>

// Reuses (does not duplicate) core's own om/fi/ka<->matrix conversion --
// header-only, pulls in nothing but Eigen/std (see structures.h), so this
// doesn't violate calib_core's no-raylib/imgui/OpenCV design (see
// calib_core/CMakeLists.txt); nothing here links core/core_math.
#include <Core/transformations.h>

namespace calib {

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

    std::string unquote(std::string s)
    {
        if (s.size() >= 2 && (s.front() == '"' || s.front() == '\'') && s.back() == s.front())
            return s.substr(1, s.size() - 2);
        return s;
    }
} // namespace

// Unrelated to loadMeiIntrinsics (MeiIntrinsics.cpp) -- opens and reads the
// file on its own rather than sharing a file handle or result with it,
// since parsing intrinsics and reading identity fields are two different
// jobs. Works on any flat `key: value` yaml, not just a Mei camera_info.yaml.
bool loadCameraIdentity(const std::string& path, CameraIdentity& id)
{
    std::ifstream f(path);
    if (!f)
        return false;

    // Cleared rather than merged, so a file naming no camera comes back
    // empty instead of keeping whatever was loaded before it.
    CameraIdentity next;
    std::string line;
    while (std::getline(f, line))
    {
        const auto hash = line.find('#');
        if (hash != std::string::npos)
            line = line.substr(0, hash);
        const auto colon = line.find(':');
        if (colon == std::string::npos)
            continue;
        const std::string key = trim(line.substr(0, colon));
        const std::string value = unquote(trim(line.substr(colon + 1)));
        if (key == "serial")
            next.serial = value;
        else if (key == "frame_id")
            next.frameId = value;
        else if (key == "model")
            next.model = value;
    }
    id = next;

    return true;
}

std::optional<double> LoadTimestampFromSideCar(const std::string& path)
{
    const auto dot = path.rfind('.');
    const std::string sidecar = (dot != std::string::npos ? path.substr(0, dot) : path) + ".meta.json";

    std::ifstream f(sidecar);
    if (!f)
        return std::nullopt;

    nlohmann::json j;
    try
    {
        f >> j;
    } catch (const nlohmann::json::exception&)
    {
        return std::nullopt;
    }

    const auto it = j.find("FRAME_WALL_CLOCK");
    if (it == j.end())
        return std::nullopt;

    // FRAME_WALL_CLOCK is nanoseconds since epoch, as a number or a numeric
    // string -- returned as-is, matching the filename timestamps.
    if (it->is_string())
    {
        try
        {
            return std::stod(it->get<std::string>());
        } catch (const std::exception&)
        {
            return std::nullopt;
        }
    }
    if (it->is_number())
        return it->get<double>();
    return std::nullopt;
}

Eigen::Matrix3f omFiKaToMat3(float om_deg, float fi_deg, float ka_deg) {
    TaitBryanPose pose;
    pose.om = deg2rad(om_deg);
    pose.fi = deg2rad(fi_deg);
    pose.ka = deg2rad(ka_deg);
    Eigen::Matrix3f Rdelta = affine_matrix_from_pose_tait_bryan(pose).linear().cast<float>();
    return kCameraLidarAxisOffset * Rdelta;
}

void omFiKaFromMat3(const Eigen::Matrix3f& R, float& om_deg, float& fi_deg, float& ka_deg) {
    Eigen::Affine3d m = Eigen::Affine3d::Identity();
    m.linear() = (kCameraLidarAxisOffset.transpose() * R).cast<double>();
    TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(m);
    om_deg = static_cast<float>(rad2deg(pose.om));
    fi_deg = static_cast<float>(rad2deg(pose.fi));
    ka_deg = static_cast<float>(rad2deg(pose.ka));
}

// No `default:` case on purpose: -Wswitch then flags a future CameraModel
// enumerator added without a matching string here, instead of it silently
// falling through to "pinhole".
const char* modelToString(CameraModel m)
{
    switch (m)
    {
    case CameraModel::Pinhole:
        return "pinhole";
    case CameraModel::Mei:
        return "mei";
    case CameraModel::Fisheye:
        return "fisheye";
    }
    return "pinhole";
}

CameraModel modelFromString(const std::string& s)
{
    if (s == "mei")
        return CameraModel::Mei;
    if (s == "fisheye" || s == "equidistant")
        return CameraModel::Fisheye;
    return CameraModel::Pinhole;
}

float fisheyeMaxTheta(const Intrinsics& K)
{
    const double k1 = K.k1, k2 = K.k2, k3 = K.k3, k4 = K.k4;
    auto thetaD = [&](double t)
    {
        const double t2 = t * t;
        return t * (1.0 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4))));
    };
    // Scanned numerically, like maxValidRadiusSq below: a 9th-order
    // polynomial's first turning point has no useful closed form.
    const double kStep = 1e-3;
    double prev = 0.0;
    for (double t = kStep; t <= M_PI; t += kStep)
    {
        const double cur = thetaD(t);
        if (cur <= prev)
            return static_cast<float>(t - kStep);
        prev = cur;
    }
    return static_cast<float>(M_PI);
}

// Memoized for the same reason as cachedMaxValidRadiusSq below.
static float cachedFisheyeMaxTheta(const Intrinsics& K)
{
    thread_local float lastK[4] = { 0.f, 0.f, 0.f, 0.f };
    thread_local float lastResult = -1.f;
    if (lastResult >= 0.f && lastK[0] == K.k1 && lastK[1] == K.k2 && lastK[2] == K.k3 && lastK[3] == K.k4)
        return lastResult;
    lastResult = fisheyeMaxTheta(K);
    lastK[0] = K.k1;
    lastK[1] = K.k2;
    lastK[2] = K.k3;
    lastK[3] = K.k4;
    return lastResult;
}

// Radius (in normalized camera coords, squared) past which the rational distortion model
// stops being usable. r -> r*radial(r) is only injective up to its turning point; beyond it
// the model folds, so directions far outside the lens' actual field of view map back onto
// valid pixel coordinates -- painting whatever is at the centre of the frame onto geometry
// the camera never saw. The projection alone cannot tell such a fold-back from a genuine
// hit, so find the turning point once and reject everything past it. Scanned numerically --
// the turning point of a 6th-order rational function has no useful closed form. It always
// lies outside the image itself (otherwise the calibration could not reach its own corners),
// so no legitimate pixel is lost. Ported from the equivalent fix applied directly in
// TrajectoryViewer.cpp's (now-removed) inline distortion code -- see upstream commit
// "Fix colorization for calibration for invalid points" (#527) -- but placed here so every
// caller of projectPoint() gets it, not just that one call site.
static float maxValidRadiusSq(float k1, float k2, float k3, float k4, float k5, float k6) {
    auto g = [&](float r) {
        float r2 = r * r;
        float den = 1.f + (k4 + (k5 + k6 * r2) * r2) * r2;
        if (std::fabs(den) < 1e-9f)
            return -1.f; // pole -- certainly past the turning point
        return r * (1.f + (k1 + (k2 + k3 * r2) * r2) * r2) / den;
    };
    // 8.0 == tan(83 deg), wider than any lens this app sees. A distortion-free model is
    // monotonic everywhere and so keeps the whole range, i.e. no behaviour change.
    const float kLimit = 8.f, kStep = 0.005f;
    float prev = 0.f;
    for (float r = kStep; r <= kLimit; r += kStep) {
        float cur = g(r);
        if (cur <= prev)
            return (r - kStep) * (r - kStep);
        prev = cur;
    }
    return kLimit * kLimit;
}

// projectPoint() is called per-point -- potentially millions of times per colorize pass --
// with the SAME Intrinsics each time, so re-running the numeric scan above on every call
// would be a severe perf regression. Memoize on the six coefficients actually scanned; exact
// float equality is fine here since it's detecting "same Intrinsics as last call", not
// comparing independently-derived values.
static float cachedMaxValidRadiusSq(float k1, float k2, float k3, float k4, float k5, float k6) {
    thread_local float lastK[6] = { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f };
    thread_local float lastResult = -1.f;
    if (lastResult >= 0.f && lastK[0] == k1 && lastK[1] == k2 && lastK[2] == k3 &&
        lastK[3] == k4 && lastK[4] == k5 && lastK[5] == k6) {
        return lastResult;
    }
    lastResult = maxValidRadiusSq(k1, k2, k3, k4, k5, k6);
    lastK[0] = k1; lastK[1] = k2; lastK[2] = k3; lastK[3] = k4; lastK[4] = k5; lastK[5] = k6;
    return lastResult;
}

Intrinsics scaleIntrinsics(const Intrinsics& K, float s) {
    Intrinsics out = K;
    out.fx *= s;
    out.fy *= s;
    out.cx *= s;
    out.cy *= s;
    out.width  = static_cast<int>(std::lround(K.width  * s));
    out.height = static_cast<int>(std::lround(K.height * s));
    return out;
}

Roi scaleRoi(const Roi& r, float s) {
    Roi out = r;
    if (r.w <= 0 || r.h <= 0) {
        return out;  // w/h == 0 is the "no ROI set" sentinel; leave it alone
    }
    const int x0 = static_cast<int>(std::lround(r.x * s));
    const int y0 = static_cast<int>(std::lround(r.y * s));
    const int x1 = static_cast<int>(std::lround((r.x + r.w) * s));
    const int y1 = static_cast<int>(std::lround((r.y + r.h) * s));
    out.x = x0;
    out.y = y0;
    // Both edges are rounded and then subtracted, rather than the width being
    // scaled on its own, so two abutting rectangles cannot come back
    // overlapping. The clamp keeps a rectangle too small to survive the scale
    // at one pixel: collapsing it to w/h == 0 would read as "no ROI" and
    // silently pass everything the ROI was there to reject.
    out.w = std::max(1, x1 - x0);
    out.h = std::max(1, y1 - y0);
    return out;
}

bool projectPoint(float px, float py, float pz,
                  const Intrinsics& K,
                  const Eigen::Matrix3f& R_wc,
                  const Eigen::Vector3f& t,
                  float& u, float& v, float& depth) {
    // p_cam = R_wc^T * (p_lidar - C)
    Eigen::Vector3f pc = R_wc.transpose() * (Eigen::Vector3f(px, py, pz) - t);

    if (K.model == CameraModel::Mei) {
        depth = pc.norm();
        if (depth < 1e-4f) return false;  // point sits on the camera itself

        // Validity domain. r(theta) = sin/(cos+xi) is only injective up to
        // its turning point at cos(theta) = -1/xi; past it the radius shrinks
        // again and far-off-axis directions FOLD BACK onto valid pixels --
        // at theta = 180 deg exactly onto (cx, cy). For xi <= 1 the
        // denominator blows up first, so "Xs.z + xi > 0" is the limit there.
        //   xi <= 1: Xs.z > -xi      (reduces to Pinhole's pc.z > 0 at xi = 0)
        //   xi  > 1: Xs.z > -1/xi
        const float zMin = (K.xi > 1.f) ? -1.f / K.xi : -K.xi;
        if (pc.z() / depth <= zMin) return false;

        // Unified sphere, then a plain (non-rational) radial/tangential
        // polynomial. Computed in double: the xi denominator gets small near
        // the edge of the valid dome, where float loses too much.
        const Eigen::Vector3d Xs = pc.cast<double>().normalized();
        const double den = Xs.z() + K.xi;
        const double x = Xs.x() / den, y = Xs.y() / den;
        const double r2 = x*x + y*y;
        const double radial = 1.0 + K.k1*r2 + K.k2*r2*r2 + K.k3*r2*r2*r2;
        const double xd = x*radial + 2*K.p1*x*y + K.p2*(r2 + 2*x*x);
        const double yd = y*radial + K.p1*(r2 + 2*y*y) + 2*K.p2*x*y;

        u = static_cast<float>(K.fx * xd + K.cx);
        v = static_cast<float>(K.fy * yd + K.cy);
        return true;
    }

    if (K.model == CameraModel::Fisheye) {
        depth = pc.norm();
        if (depth < 1e-4f) return false;  // point sits on the camera itself

        // Equidistant: image radius grows with the incidence angle theta, not
        // tan(theta), so there is no z > 0 requirement -- only the fold-back
        // limit.
        const double x = pc.x(), y = pc.y(), z = pc.z();
        const double r = std::hypot(x, y);
        const double theta = std::atan2(r, z);
        if (theta >= cachedFisheyeMaxTheta(K)) return false;
        // Directly behind has no direction to push the point out along, so
        // s below would drop it on the principal point. Checked on its own:
        // a limit of pi, rounded to float, lies just above the double pi.
        if (r == 0.0 && z < 0.0) return false;

        const double t2 = theta * theta;
        const double thetaD = theta * (1.0 + t2*(K.k1 + t2*(K.k2 + t2*(K.k3 + t2*K.k4))));
        // r == 0 is the optical axis, which lands on the principal point.
        const double s = r > 0.0 ? thetaD / r : 0.0;

        u = static_cast<float>(K.fx * x * s + K.cx);
        v = static_cast<float>(K.fy * y * s + K.cy);
        return true;
    }

    depth = pc.z();
    if (depth <= 1e-4f) return false;

    float xn = pc.x() / depth;
    float yn = pc.y() / depth;

    // Off-axis cutoff: beyond the rational distortion model's turning point, the projection
    // folds back and would paint frame-centre content onto geometry the camera never saw.
    // See maxValidRadiusSq() above.
    float r2 = xn*xn + yn*yn;
    if (r2 > cachedMaxValidRadiusSq(K.k1, K.k2, K.k3, K.k4, K.k5, K.k6))
        return false;

    float r4 = r2 * r2;
    float r6 = r4 * r2;
    float radial = (1.f + K.k1*r2 + K.k2*r4 + K.k3*r6)
                 / (1.f + K.k4*r2 + K.k5*r4 + K.k6*r6);
    float xd = xn*radial + 2.f*K.p1*xn*yn + K.p2*(r2 + 2.f*xn*xn);
    float yd = yn*radial + K.p1*(r2 + 2.f*yn*yn) + 2.f*K.p2*xn*yn;

    u = K.fx * xd + K.cx;
    v = K.fy * yd + K.cy;
    return true;
}

}  // namespace calib
