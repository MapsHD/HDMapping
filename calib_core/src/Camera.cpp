#include <CalibCore/Camera.h>

namespace calib {

Eigen::Matrix3f eulerZYXtoMat3(float rx_deg, float ry_deg, float rz_deg) {
    const float d2r = static_cast<float>(M_PI) / 180.f;
    return (Eigen::AngleAxisf(rz_deg * d2r, Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(ry_deg * d2r, Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(rx_deg * d2r, Eigen::Vector3f::UnitX()))
           .toRotationMatrix();
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