#include <CalibCore/Camera.h>

#include <CalibCore/MeiCamera.h>

#include <algorithm>

// Reuses (does not duplicate) core's own om/fi/ka<->matrix conversion --
// header-only, pulls in nothing but Eigen/std (see structures.h), so this
// doesn't violate calib_core's no-raylib/imgui/OpenCV design (see
// calib_core/CMakeLists.txt); nothing here links core/core_math.
#include <Core/transformations.h>

namespace calib {


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

    if (K.model == CameraModel::Equirectangular) {
        // Longitude from atan2(x, z) across the full width, latitude from
        // asin(y/|p|) across the height -- camera X = right, Y = down,
        // Z = forward, i.e. kCameraLidarAxisOffset's convention, so v grows
        // downward like image rows. Same model apps/manual_color colors with;
        // that app reaches it through the vendored equirectangular_camera_
        // colinearity_tait_bryan_wc_jacobian.h, not used here because it
        // re-derives the rotation from a Tait-Bryan pose per point while
        // R_wc/t are already in hand.
        depth = pc.norm();
        if (depth < 1e-4f) return false;  // point sits on the camera itself

        const float pi = static_cast<float>(M_PI);
        const float w = static_cast<float>(K.width);
        const float h = static_cast<float>(K.height);

        u = w * (0.5f + std::atan2(pc.x(), pc.z()) / (2.f*pi));
        // atan2 returns exactly +pi on the seam, which maps to u == w
        u = std::fmod(u + w, w);
        v = h * (0.5f + std::asin(std::clamp(pc.y() / depth, -1.f, 1.f)) / pi);
        return true;
    }

    if (K.model == CameraModel::Mei) {
        // Delegates to the tested/certified MeiCamera::Project (MeiCamera.h)
        // instead of re-deriving the unified-sphere + radial/tangential
        // formula here -- only the R_wc/t transform into camera frame, the
        // "point sits on the camera itself" guard (same idiom as
        // Equirectangular above), and the "in front of the camera" guard
        // just below belong to this wrapper.
        depth = pc.norm();
        if (depth < 1e-4f) return false;

        // Validity domain. r(theta) = sin/(cos+xi) is only injective up to
        // its turning point at cos(theta) = -1/xi; past it the radius shrinks
        // again and far-off-axis directions FOLD BACK onto valid pixels --
        // at theta = 180 deg exactly onto (cx, cy). For xi <= 1 the
        // denominator blows up first, so "Xs.z + xi > 0" is the limit there.
        //   xi <= 1: Xs.z > -xi      (reduces to Pinhole's pc.z > 0 at xi = 0)
        //   xi  > 1: Xs.z > -1/xi
        // MeiCamera::Project has no guard of its own, so it belongs here.
        const float zMin = (K.xi > 1.f) ? -1.f / K.xi : -K.xi;
        if (pc.z() / depth <= zMin) return false;

        MeiCamera cam;
        cam.fx = K.fx; cam.fy = K.fy; cam.cx = K.cx; cam.cy = K.cy;
        cam.xi = K.xi;
        cam.k1 = K.k1; cam.k2 = K.k2; cam.k3 = K.k3;
        cam.p1 = K.p1; cam.p2 = K.p2;

        const cv::Point2d px = cam.Project(cv::Point3d(pc.x(), pc.y(), pc.z()));
        u = static_cast<float>(px.x);
        v = static_cast<float>(px.y);
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
