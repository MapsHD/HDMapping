#include "RaylibWidgets/PointPicking.h"
#include "raymath.h"
#include <cmath>
#include <limits>

namespace raylib_widgets
{
    bool pickNearestPoint(
        const Vector3* points, size_t count, const Ray& ray, float cameraFovYDeg, float viewportHeightPx, float pixelThreshold,
        size_t& outIndex)
    {
        if (points == nullptr || count == 0 || viewportHeightPx <= 0.f)
            return false;

        const float halfFovyRad = cameraFovYDeg * DEG2RAD * 0.5f;
        float bestPerpDist = std::numeric_limits<float>::max();
        bool found = false;

        for (size_t i = 0; i < count; ++i)
        {
            Vector3 toP = Vector3Subtract(points[i], ray.position);
            float depthAlong = Vector3DotProduct(toP, ray.direction);
            if (depthAlong <= 0.f)
                continue; // behind the camera

            Vector3 closest = Vector3Add(ray.position, Vector3Scale(ray.direction, depthAlong));
            float perpDist = Vector3Distance(points[i], closest);

            // World distance covered by one screen pixel at this depth, so
            // the pixel tolerance stays constant in screen space.
            float worldPerPixel = (2.f * depthAlong * std::tan(halfFovyRad)) / viewportHeightPx;
            float maxWorldDist = pixelThreshold * worldPerPixel;

            if (perpDist <= maxWorldDist && perpDist < bestPerpDist)
            {
                bestPerpDist = perpDist;
                outIndex = i;
                found = true;
            }
        }

        return found;
    }
} // namespace raylib_widgets
