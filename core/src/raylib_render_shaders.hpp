#pragma once

// GLSL source for ScanRenderer's point shader. Split out of raylib_render.cpp
// to keep the .cpp free of embedded shader text; included only there.
#include <RaylibWidgets/Shaders.h>

#include <string>

namespace raylib_render_shaders
{
    // vertexIntensity is per-point LAS/LAZ intensity, normalized to [0,1] per
    // scan at upload time (see ScanRenderer::rebuild). vertexPosition is
    // already world-space (rebuild() applies pc.m_pose before uploading), so
    // it doubles as the Elevation/Distance color modes' input with no extra
    // per-vertex data needed. colorMode selects between the flat per-scan
    // pointColor (0, the original app's only mode) and the ScanColorMode
    // gradients (1/2/3), matching the enum's Intensity/Elevation/Distance
    // ordering exactly (see ScanRenderer::draw()'s static_cast below).
    inline constexpr const char* kPointVS = R"(
#version 330
in vec3 vertexPosition;
in float vertexIntensity;
uniform mat4 mvp;
uniform float pointSize;
out float fragIntensity;
out vec3 fragWorldPos;
void main()
{
    gl_Position = mvp * vec4(vertexPosition, 1.0);
    gl_PointSize = pointSize;
    fragIntensity = vertexIntensity;
    fragWorldPos = vertexPosition;
}
)";

    // jet() is shared (raylib_widgets/Shaders.h) with camera_lidar_calibration's
    // and camera_lidar_trajectory_viewer's point shaders -- was byte-for-byte
    // duplicated here.
    inline const std::string kPointFS = std::string(R"(
#version 330
uniform vec4 pointColor;
uniform int colorMode;
uniform float elevMin;
uniform float elevMax;
uniform vec3 distCenter;
uniform float distMax;
// Intersection-slab gating: was PointCloud::render()'s (core/src/point_cloud.cpp,
// legacy immediate-mode GL) per-point xz/yz/xy_intersection check, ported here
// as a fragment discard instead of a CPU-side render_point bool. xzOn/yzOn/xyOn
// are 0/1 (bool uniforms aren't portable pre-4.x); intersectionWidth is the
// same half-width in world units on both sides of each cutting plane through
// the world origin. With all three off, every point draws (matches the
// original's "no intersection mode active -> always render" fallback).
uniform int xzOn;
uniform int yzOn;
uniform int xyOn;
uniform float intersectionWidth;
in float fragIntensity;
in vec3 fragWorldPos;
out vec4 finalColor;
)") + raylib_widgets::kJetColormapGLSL +
        R"(
void main()
{
    if (xzOn != 0 || yzOn != 0 || xyOn != 0)
    {
        bool inSlab = false;
        if (xzOn != 0 && abs(fragWorldPos.y) < intersectionWidth) inSlab = true;
        if (yzOn != 0 && abs(fragWorldPos.x) < intersectionWidth) inSlab = true;
        if (xyOn != 0 && abs(fragWorldPos.z) < intersectionWidth) inSlab = true;
        if (!inSlab) discard;
    }

    if (colorMode == 1)
    {
        finalColor = vec4(jet(fragIntensity), pointColor.a);
    }
    else if (colorMode == 2)
    {
        float range = max(elevMax - elevMin, 1e-6);
        finalColor = vec4(jet((fragWorldPos.z - elevMin) / range), pointColor.a);
    }
    else if (colorMode == 3)
    {
        float d = length(fragWorldPos - distCenter);
        finalColor = vec4(jet(d / max(distMax, 1e-6)), pointColor.a);
    }
    else
    {
        finalColor = pointColor;
    }
}
)";
} // namespace raylib_render_shaders
