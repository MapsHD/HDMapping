#include <CalibCore/Camera.h>

// Reuses (does not duplicate) core's own om/fi/ka<->matrix conversion --
// header-only, pulls in nothing but Eigen/std (see structures.h), so this
// doesn't violate calib_core's no-raylib/imgui/OpenCV design (see
// calib_core/CMakeLists.txt); nothing here links core/core_math.
#include <Core/transformations.h>

namespace calib {

void avoidGimbalLock(float& fi_deg) {
    if (std::fabs(std::fabs(fi_deg) - 90.f) < kGimbalLockEpsilonDeg)
        fi_deg = std::copysign(90.f - kGimbalLockEpsilonDeg, fi_deg);
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
    avoidGimbalLock(fi_deg);
}

bool projectPoint(float px, float py, float pz,
                  const Intrinsics& K,
                  const Eigen::Matrix3f& R_wc,
                  const Eigen::Vector3f& t,
                  float& u, float& v, float& depth) {
    // p_cam = R_wc^T * (p_lidar - C)
    Eigen::Vector3f pc = R_wc.transpose() * (Eigen::Vector3f(px, py, pz) - t);

    depth = pc.z();
    if (depth <= 1e-4f) return false;

    float xn = pc.x() / depth;
    float yn = pc.y() / depth;

    float r2 = xn*xn + yn*yn;
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
