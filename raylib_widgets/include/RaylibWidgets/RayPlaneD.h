#pragma once
#include <Eigen/Geometry>

// Double-precision ray/plane/line math -- for callers whose world
// coordinates need more than raylib's native float precision (e.g. large
// projected/UTM-like coordinates), shared between
// multi_view_tls_registration_step_2's ground-plane rotation-center picking
// and any other consumer with the same need. Deliberately kept out of
// PointPicking.h, which stays Eigen-free for camera_lidar_calibration/
// camera_lidar_intrinsics_calib -- only link/include this header if you
// actually need double precision.
namespace raylib_widgets {

// Intersects the ray (rayPos + t*rayDir, t >= 0 not enforced -- callers
// that only want points ahead of the ray should check the sign of the
// result relative to rayPos themselves) with the plane
// a*x + b*y + c*z + d = 0. Returns false (outPoint untouched) if the ray is
// ~parallel to the plane (rayDir's component along the plane normal is
// within 1e-4 of zero).
bool intersectPlane(
    const Eigen::Vector3d& rayPos, const Eigen::Vector3d& rayDir, double a, double b, double c, double d, Eigen::Vector3d& outPoint);

// Perpendicular distance from `point` to the infinite line through rayPos
// with direction rayDir. Does NOT normalize by |rayDir| -- the result scales
// with rayDir's own magnitude, matching the original call sites' only use
// of this (finding the nearest of several candidate points against one
// fixed ray, where a constant scale factor across all candidates doesn't
// change which one wins).
double distancePointToLine(const Eigen::Vector3d& point, const Eigen::Vector3d& rayPos, const Eigen::Vector3d& rayDir);

} // namespace raylib_widgets
