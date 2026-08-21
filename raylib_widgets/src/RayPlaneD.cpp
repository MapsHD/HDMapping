#include "RaylibWidgets/RayPlaneD.h"

namespace raylib_widgets {

bool intersectPlane(
    const Eigen::Vector3d& rayPos, const Eigen::Vector3d& rayDir, double a, double b, double c, double d, Eigen::Vector3d& outPoint)
{
    const double kTolerance = 0.0001;

    double denom = a * rayDir.x() + b * rayDir.y() + c * rayDir.z();
    if (denom > -kTolerance && denom < kTolerance)
        return false;

    double distFromPlane = a * rayPos.x() + b * rayPos.y() + c * rayPos.z() + d;
    outPoint = rayPos - rayDir * (distFromPlane / denom);
    return true;
}

double distancePointToLine(const Eigen::Vector3d& point, const Eigen::Vector3d& rayPos, const Eigen::Vector3d& rayDir)
{
    Eigen::Vector3d AP = point - rayPos;
    return (AP.cross(rayDir)).norm();
}

} // namespace raylib_widgets
