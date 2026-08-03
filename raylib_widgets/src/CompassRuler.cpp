#include <RaylibWidgets/CompassRuler.h>
#include "raymath.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace raylib_widgets {

void drawCompassRuler(Vector3 right, Vector3 up, float zoomDistance, Color rulerColor, CompassAxisLabels labels)
{
    const float compassSize = 200.0f;
    const float originX = compassSize * 0.5f;
    const float originY = static_cast<float>(GetScreenHeight()) - compassSize * 0.5f;
    const float axisPixelLength = compassSize * 0.35f;

    struct Axis {
        Vector3 dir;
        const char* label;
        Color color;
    };
    const Axis axes[3] = {
        { Vector3{ 1.f, 0.f, 0.f }, labels.x, RED },
        { Vector3{ 0.f, 1.f, 0.f }, labels.y, GREEN },
        { Vector3{ 0.f, 0.f, 1.f }, labels.z, BLUE },
    };
    for (const auto& axis : axes)
    {
        float ex = Vector3DotProduct(axis.dir, right);
        float ey = Vector3DotProduct(axis.dir, up);
        Vector2 tip = { originX + ex * axisPixelLength, originY - ey * axisPixelLength };
        DrawLineEx(Vector2{ originX, originY }, tip, 2.f, axis.color);
        DrawText(axis.label, (int)tip.x + 4, (int)tip.y - 6, 12, axis.color);
    }

    // "Nice" (1/2/5 x 10^n) ruler length tied to the current camera zoom.
    float rawUnit = std::max(0.001f, 0.1f * std::fabs(zoomDistance));
    float base = std::pow(10.0f, std::floor(std::log10(rawUnit)));
    float normalized = rawUnit / base;
    float niceUnit = normalized < 2.0f ? 1.0f : (normalized < 5.0f ? 2.0f : 5.0f);
    float worldLength = niceUnit * base;

    char label[32];
    if (worldLength >= 1000.0f)
        snprintf(label, sizeof(label), "%.0f [km]", worldLength / 1000.0f);
    else if (worldLength >= 1.0f)
        snprintf(label, sizeof(label), "%.0f [m]", worldLength);
    else if (worldLength >= 0.01f)
        snprintf(label, sizeof(label), "%.0f [cm]", worldLength * 100.0f);
    else
        snprintf(label, sizeof(label), "<1 [cm]");

    float rulerY = originY + compassSize * 0.45f;
    DrawLineEx(Vector2{ originX - 40.f, rulerY }, Vector2{ originX + 40.f, rulerY }, 2.f, rulerColor);
    DrawLineEx(Vector2{ originX - 40.f, rulerY - 5.f }, Vector2{ originX - 40.f, rulerY + 5.f }, 2.f, rulerColor);
    DrawLineEx(Vector2{ originX + 40.f, rulerY - 5.f }, Vector2{ originX + 40.f, rulerY + 5.f }, 2.f, rulerColor);
    DrawText(label, (int)originX - 20, (int)rulerY + 6, 14, rulerColor);
}

} // namespace raylib_widgets
